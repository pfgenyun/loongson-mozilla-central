/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_mips_MacroAssembler_mips_h
#define jit_mips_MacroAssembler_mips_h

#include "mozilla/DebugOnly.h"

#include "jit/mips/Assembler-mips.h"
#include "jit/IonCaches.h"

#include "jit/IonFrames.h"
#include "jsopcode.h"
#include "jit/MoveResolver.h"

#include "jscompartment.h"

namespace js {
namespace jit {

class MacroAssemblerMIPS : public Assembler
{
    // Number of bytes the stack is adjusted inside a call to C. Calls to C may
    // not be nested.
    bool inCall_;
    uint32_t args_;
    uint32_t passedArgs_;
    uint32_t passedArgsfake_;   //by weizhenwei
    uint32_t passedArgsBits_[4];//bitmap, 1 is double, 2 is int, by weizhenwei, 2013.11.08
    uint32_t stackForCall_;
    bool dynamicAlignment_;
    bool enoughMemory_;

    struct Double {
        double value;
        AbsoluteLabel uses;
        Double(double value) : value(value) {}
    };
    Vector<Double, 0, SystemAllocPolicy> doubles_;

    typedef HashMap<double, size_t, DefaultHasher<double>, SystemAllocPolicy> DoubleMap;
    DoubleMap doubleMap_;

  protected:
    MoveResolver moveResolver_;

  private:
    Operand payloadOf(const Address &address) {
        return Operand(address.base, address.offset);
    }
    Operand tagOf(const Address &address) {
        return Operand(address.base, address.offset + 4);
    }
    Operand tagOf(const BaseIndex &address) {
        return Operand(address.base, address.index, address.scale, address.offset + 4);
    }

    void setupABICall(uint32_t args);

  public:

    enum Result {
        GENERAL,
        DOUBLE
    };

    typedef MoveResolver::MoveOperand MoveOperand;
    typedef MoveResolver::Move Move;

    // The buffer is about to be linked, make sure any constant pools or excess
    // bookkeeping has been flushed to the instruction stream.
    void finish();

    bool oom() const {
        return Assembler::oom() || !enoughMemory_;
    }

    Operand ToPayload(Operand base) {
        return base;
    }
    Operand ToType(Operand base) {
        switch (base.kind()) {
          case Operand::REG_DISP:
            return Operand(Register::FromCode(base.base()), base.disp() + sizeof(void *));

          case Operand::SCALE:
            return Operand(Register::FromCode(base.base()), Register::FromCode(base.index()), base.scale(), base.disp() + sizeof(void *));

          default:
            JS_NOT_REACHED("unexpected operand kind");
            return base; // Silence GCC warning.
        }
    }
    void moveValue(const Value &val, Register type, Register data) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        movl(Imm32(jv.s.tag), type);
        if (val.isMarkable())
            movl(ImmGCPtr(reinterpret_cast<gc::Cell *>(val.toGCThing())), data);
        else
            movl(Imm32(jv.s.payload.i32), data);
    }
    void moveValue(const Value &val, const ValueOperand &dest) {
        moveValue(val, dest.typeReg(), dest.payloadReg());
    }
    void moveValue(const ValueOperand &src, const ValueOperand &dest) {
        JS_ASSERT(src.typeReg() != dest.payloadReg());
        JS_ASSERT(src.payloadReg() != dest.typeReg());
        if (src.typeReg() != dest.typeReg())
            movl(src.typeReg(), dest.typeReg());
        if (src.payloadReg() != dest.payloadReg())
            movl(src.payloadReg(), dest.payloadReg());
    }

    void storeValue(ValueOperand val, Operand dest) {
        movl(val.payloadReg(), ToPayload(dest));
        movl(val.typeReg(), ToType(dest));
    }
    void storeValue(ValueOperand val, const Address &dest) {
        storeValue(val, Operand(dest));
    }
    template <typename T>
    void storeValue(JSValueType type, Register reg, const T &dest) {
        storeTypeTag(ImmTag(JSVAL_TYPE_TO_TAG(type)), Operand(dest));
        storePayload(reg, Operand(dest));
    }
    template <typename T>
    void storeValue(const Value &val, const T &dest) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        storeTypeTag(ImmTag(jv.s.tag), Operand(dest));
        storePayload(val, Operand(dest));
    }
    void storeValue(ValueOperand val, BaseIndex dest) {
        storeValue(val, Operand(dest));
    }
    void loadValue(Operand src, ValueOperand val) {
        Operand payload = ToPayload(src);
        Operand type = ToType(src);

        // Ensure that loading the payload does not erase the pointer to the
        // Value in memory or the index.
        Register baseReg = Register::FromCode(src.base());
        Register indexReg = (src.kind() == Operand::SCALE) ? Register::FromCode(src.index()) : InvalidReg;

        if (baseReg == val.payloadReg() || indexReg == val.payloadReg()) {
            JS_ASSERT(baseReg != val.typeReg());
            JS_ASSERT(indexReg != val.typeReg());

            movl(type, val.typeReg());
            movl(payload, val.payloadReg());
        } else {
            JS_ASSERT(baseReg != val.payloadReg());
            JS_ASSERT(indexReg != val.payloadReg());

            movl(payload, val.payloadReg());
            movl(type, val.typeReg());
        }
    }
    void loadValue(Address src, ValueOperand val) {
        loadValue(Operand(src), val);
    }
    void loadValue(const BaseIndex &src, ValueOperand val) {
        loadValue(Operand(src), val);
    }
    void tagValue(JSValueType type, Register payload, ValueOperand dest) {
        JS_ASSERT(payload != dest.typeReg());
        movl(ImmType(type), dest.typeReg());
        if (payload != dest.payloadReg())
            movl(payload, dest.payloadReg());
    }
    void pushValue(ValueOperand val) {
        push(val.typeReg());
        push(val.payloadReg());
    }
    void popValue(ValueOperand val) {
        pop(val.payloadReg());
        pop(val.typeReg());
    }
    void pushValue(const Value &val) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        push(Imm32(jv.s.tag));
        if (val.isMarkable())
            push(ImmGCPtr(reinterpret_cast<gc::Cell *>(val.toGCThing())));
        else
            push(Imm32(jv.s.payload.i32));
    }
    void pushValue(JSValueType type, Register reg) {
        push(ImmTag(JSVAL_TYPE_TO_TAG(type)));
        push(reg);
    }
    void pushValue(const Address &addr) {
        push(tagOf(addr));
        push(payloadOf(addr));
    }
    void storePayload(const Value &val, Operand dest) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        if (val.isMarkable())
            movl(ImmGCPtr((gc::Cell *)jv.s.payload.ptr), ToPayload(dest));
        else
            movl(Imm32(jv.s.payload.i32), ToPayload(dest));
    }
    void storePayload(Register src, Operand dest) {
        movl(src, ToPayload(dest));
    }
    void storeTypeTag(ImmTag tag, Operand dest) {
        movl(tag, ToType(dest));
    }

    void movePtr(const Register &src, const Register &dest) {
        movl(src, dest);
    }
    void movePtr(const Register &src, const Operand &dest) {
        movl(src, dest);
    }

    // Returns the register containing the type tag.
    Register splitTagForTest(const ValueOperand &value) {
        return value.typeReg();
    }

    Condition testUndefined(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_UNDEFINED));
        return cond;
    }
    Condition testBoolean(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_BOOLEAN));
        return cond;
    }
    Condition testInt32(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testDouble(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
        Condition actual = (cond == Equal) ? Below : AboveOrEqual;
        cmpl(tag, ImmTag(JSVAL_TAG_CLEAR));
        return actual;
    }
    Condition testNull(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_NULL));
        return cond;
    }
    Condition testString(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_STRING));
        return cond;
    }
    Condition testObject(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_OBJECT));
        return cond;
    }
    Condition testNumber(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_UPPER_INCL_TAG_OF_NUMBER_SET));
        return cond == Equal ? BelowOrEqual : Above;
    }
    Condition testGCThing(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }
    Condition testGCThing(Condition cond, const Address &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }
    Condition testMagic(Condition cond, const Address &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testMagic(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testMagic(Condition cond, const Operand &operand) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(ToType(operand), ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testPrimitive(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_UPPER_EXCL_TAG_OF_PRIMITIVE_SET));
        return cond == Equal ? Below : AboveOrEqual;
    }
    Condition testError(Condition cond, const Register &tag) {
        return testMagic(cond, tag);
    }
    Condition testInt32(Condition cond, const Operand &operand) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(ToType(operand), ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testInt32(Condition cond, const Address &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        return testInt32(cond, Operand(address));
    }
    Condition testDouble(Condition cond, const Operand &operand) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        Condition actual = (cond == Equal) ? Below : AboveOrEqual;
        cmpl(ToType(operand), ImmTag(JSVAL_TAG_CLEAR));
        return actual;
    }
    Condition testDouble(Condition cond, const Address &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        return testDouble(cond, Operand(address));
    }
    Condition testUndefined(Condition cond, const ValueOperand &value) {
        return testUndefined(cond, value.typeReg());
    }
    Condition testBoolean(Condition cond, const ValueOperand &value) {
        return testBoolean(cond, value.typeReg());
    }
    Condition testInt32(Condition cond, const ValueOperand &value) {
        return testInt32(cond, value.typeReg());
    }
    Condition testDouble(Condition cond, const ValueOperand &value) {
        return testDouble(cond, value.typeReg());
    }
    Condition testNull(Condition cond, const ValueOperand &value) {
        return testNull(cond, value.typeReg());
    }
    Condition testString(Condition cond, const ValueOperand &value) {
        return testString(cond, value.typeReg());
    }
    Condition testObject(Condition cond, const ValueOperand &value) {
        return testObject(cond, value.typeReg());
    }
    Condition testMagic(Condition cond, const ValueOperand &value) {
        return testMagic(cond, value.typeReg());
    }
    Condition testError(Condition cond, const ValueOperand &value) {
        return testMagic(cond, value);
    }
    Condition testNumber(Condition cond, const ValueOperand &value) {
        return testNumber(cond, value.typeReg());
    }
    Condition testGCThing(Condition cond, const ValueOperand &value) {
        return testGCThing(cond, value.typeReg());
    }
    Condition testPrimitive(Condition cond, const ValueOperand &value) {
        return testPrimitive(cond, value.typeReg());
    }


    Condition testUndefined(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_UNDEFINED));
        return cond;
    }
    Condition testNull(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_NULL));
        return cond;
    }
    Condition testBoolean(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_BOOLEAN));
        return cond;
    }
    Condition testString(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_STRING));
        return cond;
    }
    Condition testInt32(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testObject(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_OBJECT));
        return cond;
    }
    Condition testDouble(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        Condition actual = (cond == Equal) ? Below : AboveOrEqual;
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_CLEAR));
        return actual;
    }
    Condition testMagic(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    Condition testGCThing(Condition cond, const BaseIndex &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }



    void branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label);
    void branchTestValue(Condition cond, const Address &valaddr, const ValueOperand &value,
                         Label *label)
    {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        branchPtr(cond, tagOf(valaddr), value.typeReg(), label);
        branchPtr(cond, payloadOf(valaddr), value.payloadReg(), label);
    }

    void cmpPtr(Register lhs, const ImmWord rhs) {
        cmpl(lhs, Imm32(rhs.value));
    }
    void cmpPtr(Register lhs, const ImmGCPtr rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const ImmWord rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const ImmGCPtr rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const Imm32 rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Address &lhs, Register rhs) {
        cmpl(Operand(lhs), rhs);
    }
    void cmpPtr(const Address &lhs, const ImmWord rhs) {
        cmpl(Operand(lhs), rhs);
    }
    void cmpPtr(Register lhs, Register rhs) {
        cmpl(lhs, rhs);
    }
    void testPtr(Register lhs, Register rhs) {
	    // overwrite testl, by wangqing, 2013-12-02 
		andInsn(cmpTempRegister, lhs, rhs);
		movl(zero, cmpTemp2Register);
    }

    Condition testNegativeZero(const FloatRegister &reg, const Register &scratch);

    void reserveStack(uint32_t amount) {
        if (amount)
            subl(Imm32(amount), StackPointer);
        framePushed_ += amount;
    }
    void freeStack(uint32_t amount) {
        JS_ASSERT(amount <= framePushed_);
        if (amount)
            addl(Imm32(amount), StackPointer);
        framePushed_ -= amount;
    }
    void freeStack(Register amount) {
        addl(amount, StackPointer);
    }

    void addPtr(const Register &src, const Register &dest) {
        addl(src, dest);
    }
    void addPtr(Imm32 imm, const Register &dest) {
        addl(imm, dest);
    }
    void addPtr(ImmWord imm, const Register &dest) {
        addl(Imm32(imm.value), dest);
    }
    void addPtr(Imm32 imm, const Address &dest) {
        addl(imm, Operand(dest));
    }
    void addPtr(const Address &src, const Register &dest) {
        addl(Operand(src), dest);
    }
    void subPtr(Imm32 imm, const Register &dest) {
        subl(imm, dest);
    }
    void subPtr(const Register &src, const Register &dest) {
        subl(src, dest);
    }
    void subPtr(const Address &addr, const Register &dest) {
        subl(Operand(addr), dest);
    }

    void branch32(Condition cond, const AbsoluteAddress &lhs, Imm32 rhs, Label *label) {
        cmpl(Operand(lhs), rhs);
        j(cond, label);
    }
    void branch32(Condition cond, const AbsoluteAddress &lhs, Register rhs, Label *label) {
        cmpl(Operand(lhs), rhs);
        j(cond, label);
    }

    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, Label *label) {
        cmpl(Operand(lhs), ptr);
        j(cond, label);
    }

    template <typename T>
    void branchPrivatePtr(Condition cond, T lhs, ImmWord ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }

    void branchPrivatePtr(Condition cond, const Address &lhs, Register ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }

    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, RepatchLabel *label) {
        cmpl(Operand(lhs), ptr);
        j(cond, label);
    }

    CodeOffsetJump jumpWithPatch(RepatchLabel *label) {
        jump(label);
        return CodeOffsetJump(size());
    }

    template <typename S, typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, S lhs, T ptr, RepatchLabel *label) {
        branchPtr(cond, lhs, ptr, label);
        return CodeOffsetJump(size());
    }
    void branchPtr(Condition cond, Register lhs, Register rhs, RepatchLabel *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
    void branchPtr(Condition cond, Register lhs, Register rhs, Label *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
    void branchTestPtr(Condition cond, Register lhs, Register rhs, Label *label) {
	    // overwrite testl, by wangqing, 2013-12-02 
		andInsn(cmpTempRegister, lhs, rhs);
		movl(zero, cmpTemp2Register);
        j(cond, label);
    }
    void branchTestPtr(Condition cond, Register lhs, Imm32 imm, Label *label) {
	    // overwrite testl, by wangqing, 2013-12-02 
		movl(imm, cmpTempRegister);
		andInsn(cmpTempRegister, lhs, cmpTempRegister);
		movl(zero, cmpTemp2Register);
        j(cond, label);
    }
    void branchTestPtr(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        testl(Operand(lhs), imm);
        j(cond, label);
    }
    void decBranchPtr(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        subPtr(imm, lhs);
        //add by QuQiuwen
        cmpl(lhs,zero);
        j(cond, label);
    }

    void movePtr(ImmWord imm, Register dest) {
        movl(Imm32(imm.value), dest);
    }
    void movePtr(ImmGCPtr imm, Register dest) {
        movl(imm, dest);
    }
    void loadPtr(const Address &address, Register dest) {
        movl(Operand(address), dest);
    }
    void loadPtr(const Operand &src, Register dest) {
        movl(src, dest);
    }
    void loadPtr(const BaseIndex &src, Register dest) {
        movl(Operand(src), dest);
    }
    void loadPtr(const AbsoluteAddress &address, Register dest) {
        movl(Operand(address), dest);
    }
    void loadPrivate(const Address &src, Register dest) {
        movl(payloadOf(src), dest);
    }
    void storePtr(ImmWord imm, const Address &address) {
        movl(Imm32(imm.value), Operand(address));
    }
    void storePtr(ImmGCPtr imm, const Address &address) {
        movl(imm, Operand(address));
    }
    void storePtr(Register src, const Address &address) {
        movl(src, Operand(address));
    }
    void storePtr(Register src, const Operand &dest) {
        movl(src, dest);
    }
    void storePtr(Register src, const AbsoluteAddress &address) {
        movl(src, Operand(address));
    }

    void setStackArg(const Register &reg, uint32_t arg) {
        movl(reg, Operand(sp, arg * STACK_SLOT_SIZE));
    }

    // Type testing instructions can take a tag in a register or a
    // ValueOperand.
    template <typename T>
    void branchTestUndefined(Condition cond, const T &t, Label *label) {
        cond = testUndefined(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestInt32(Condition cond, const T &t, Label *label) {
        cond = testInt32(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestBoolean(Condition cond, const T &t, Label *label) {
        cond = testBoolean(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestDouble(Condition cond, const T &t, Label *label) {
        cond = testDouble(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestNull(Condition cond, const T &t, Label *label) {
        cond = testNull(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestString(Condition cond, const T &t, Label *label) {
        cond = testString(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestObject(Condition cond, const T &t, Label *label) {
        cond = testObject(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestNumber(Condition cond, const T &t, Label *label) {
        cond = testNumber(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestGCThing(Condition cond, const T &t, Label *label) {
        cond = testGCThing(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestPrimitive(Condition cond, const T &t, Label *label) {
        cond = testPrimitive(cond, t);
        j(cond, label);
    }
    template <typename T>
    void branchTestMagic(Condition cond, const T &t, Label *label) {
        cond = testMagic(cond, t);
        j(cond, label);
    }
    void branchTestMagicValue(Condition cond, const ValueOperand &val, JSWhyMagic why,
                              Label *label)
    {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        if (cond == Equal) {
            // Test for magic
            Label notmagic;
            Condition testCond = testMagic(Equal, val);
            j(InvertCondition(testCond), &notmagic);
            // Test magic value
            branch32(Equal, val.payloadReg(), Imm32(static_cast<int32_t>(why)), label);
            bind(&notmagic);
        } else {
            Condition testCond = testMagic(NotEqual, val);
            j(testCond, label);
            branch32(NotEqual, val.payloadReg(), Imm32(static_cast<int32_t>(why)), label);
        }
    }
    // Note: this function clobbers the source register.
    void boxDouble(const FloatRegister &src, const ValueOperand &dest) {
        fastStoreDouble(src, dest.payloadReg(), dest.typeReg());
    }
    void boxNonDouble(JSValueType type, const Register &src, const ValueOperand &dest) {
        if (src != dest.payloadReg())
            movl(src, dest.payloadReg());
        movl(ImmType(type), dest.typeReg());
    }
    void unboxInt32(const ValueOperand &src, const Register &dest) {
        movl(src.payloadReg(), dest);
    }
    void unboxInt32(const Address &src, const Register &dest) {
        movl(payloadOf(src), dest);
    }
    void unboxDouble(const Address &src, const FloatRegister &dest) {
        movsd(Operand(src), dest);
    }
    void unboxBoolean(const ValueOperand &src, const Register &dest) {
        movl(src.payloadReg(), dest);
    }
    void unboxBoolean(const Address &src, const Register &dest) {
        movl(payloadOf(src), dest);
    }
    void unboxDouble(const ValueOperand &src, const FloatRegister &dest) {
        JS_ASSERT(dest != ScratchFloatReg);
        fastLoadDouble(src.payloadReg(), src.typeReg(), dest);
    }
    //xsb:fixme
    void unboxDouble(const Operand &payload, const Operand &type,
                     const Register &scratch, const FloatRegister &dest) {
        JS_ASSERT(dest != ScratchFloatReg);
        JS_ASSERT(0);
    }
    
    void unboxValue(const ValueOperand &src, AnyRegister dest) {
        if (dest.isFloat()) {
			// by wangqing, 2013-12-02
            Label notInt32, end;
			movl(ImmTag(JSVAL_TAG_INT32), cmpTempRegister);
			bne(src.typeReg(), cmpTempRegister, &notInt32);
			nop();

            cvtsi2sd(src.payloadReg(), dest.fpu());
            b(&end);
			nop();
            bindBranch(&notInt32);
            unboxDouble(src, dest.fpu());
            bindBranch(&end);
        } else {
            if (src.payloadReg() != dest.gpr())
                movl(src.payloadReg(), dest.gpr());
        }
    }
    void unboxPrivate(const ValueOperand &src, Register dest) {
        if (src.payloadReg() != dest)
            movl(src.payloadReg(), dest);
    }

    void notBoolean(const ValueOperand &val) {
        xorl(Imm32(1), val.payloadReg());
    }

    // Extended unboxing API. If the payload is already in a register, returns
    // that register. Otherwise, provides a move to the given scratch register,
    // and returns that.
    Register extractObject(const Address &address, Register scratch) {
        movl(payloadOf(address), scratch);
        return scratch;
    }
    Register extractObject(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    Register extractInt32(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    Register extractBoolean(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    Register extractTag(const Address &address, Register scratch) {
        movl(tagOf(address), scratch);
        return scratch;
    }
    Register extractTag(const ValueOperand &value, Register scratch) {
        return value.typeReg();
    }

    void boolValueToDouble(const ValueOperand &operand, const FloatRegister &dest) {
        cvtsi2sd(operand.payloadReg(), dest);
    }
    void int32ValueToDouble(const ValueOperand &operand, const FloatRegister &dest) {
        cvtsi2sd(operand.payloadReg(), dest);
    }

    void loadConstantDouble(double d, const FloatRegister &dest);
    void loadStaticDouble(const double *dp, const FloatRegister &dest) {
        movsd(dp, dest);
    }

	//xsb:fixme
    void branchTruncateDouble(const FloatRegister &src, const Register &dest, Label *fail) {
        cvttsd2si(src, dest);
        cmpl(dest, Imm32(0x7fffffff));
        j(Assembler::Equal, fail);
    }

    Condition testInt32Truthy(bool truthy, const ValueOperand &operand) {
		cmpl(operand.payloadReg(), zero);
        return truthy ? NonZero : Zero;
    }
    void branchTestBooleanTruthy(bool truthy, const ValueOperand &operand, Label *label) {
		cmpl(operand.payloadReg(), zero);
        j(truthy ? NonZero : Zero, label);
    }
    Condition testStringTruthy(bool truthy, const ValueOperand &value) {
        Register string = value.payloadReg();
        Operand lengthAndFlags(string, JSString::offsetOfLengthAndFlags());

        size_t mask = (0xFFFFFFFF << JSString::LENGTH_SHIFT);
        testl(lengthAndFlags, Imm32(mask));
        return truthy ? Assembler::NonZero : Assembler::Zero;
    }


    void loadInt32OrDouble(const Operand &operand, const FloatRegister &dest) {
        Label notInt32, end;
		// by wangqing, 2013-12-02
        cmpl(ToType(operand), ImmTag(JSVAL_TAG_INT32));
		bne(cmpTempRegister, cmpTemp2Register, &notInt32);
		nop();

        cvtsi2sd(ToPayload(operand), dest);
        b(&end);
		nop();
        bindBranch(&notInt32);
        movsd(operand, dest);
        bindBranch(&end);
    }

    template <typename T>
    void loadUnboxedValue(const T &src, MIRType type, AnyRegister dest) {
        if (dest.isFloat())
            loadInt32OrDouble(Operand(src), dest.fpu());
        else
            movl(Operand(src), dest.gpr());
    }

    void rshiftPtr(Imm32 imm, Register dest) {
        shrl(imm, dest);
    }
    void lshiftPtr(Imm32 imm, Register dest) {
        shll(imm, dest);
    }
    void xorPtr(Imm32 imm, Register dest) {
        xorl(imm, dest);
    }
    void xorPtr(Register src, Register dest) {
        xorl(src, dest);
    }
    void orPtr(Imm32 imm, Register dest) {
        orl(imm, dest);
    }
    void orPtr(Register src, Register dest) {
        orl(src, dest);
    }
    void andPtr(Imm32 imm, Register dest) {
        andl(imm, dest);
    }
    void andPtr(Register src, Register dest) {
        andl(src, dest);
    }

    void loadInstructionPointerAfterCall(const Register &dest) {
        movl(Operand(StackPointer, 0x0), dest);
    }

    // Note: this function clobbers the source register.
    void convertUInt32ToDouble(const Register &src, const FloatRegister &dest) {
        // src is [0, 2^32-1]
        subl(Imm32(0x80000000), src);

        // Now src is [-2^31, 2^31-1] - int range, but not the same value.
        cvtsi2sd(src, dest);

        // dest is now a double with the int range.
        // correct the double value by adding 0x80000000.
        static const double NegativeOne = 2147483648.0;
        addsd(Operand(&NegativeOne), dest);
    }

    void inc64(AbsoluteAddress dest) {
        addl(Imm32(1), Operand(dest));
		// by wangqing, 2013-12-02
        Label noOverflow;
		movl(Operand(dest), cmpTempRegister);
		bne(cmpTempRegister, zero, &noOverflow);
		nop();

        addl(Imm32(1), Operand(dest.offset(4)));
        bindBranch(&noOverflow);
    }

    // If source is a double, load it into dest. If source is int32,
    // convert it to double. Else, branch to failure.
    void ensureDouble(const ValueOperand &source, FloatRegister dest, Label *failure) {
    	// by wangqing, 2013-12-02
        Label isDouble, done;
		movl(ImmTag(JSVAL_TAG_CLEAR), cmpTempRegister);
		sltu(cmpTempRegister, source.typeReg(), cmpTempRegister);
		bgtz(cmpTempRegister, &isDouble);
		nop();

        branchTestInt32(Assembler::NotEqual, source.typeReg(), failure);

        convertInt32ToDouble(source.payloadReg(), dest);
        b(&done);
		nop();

        bindBranch(&isDouble);
        unboxDouble(source, dest);

        bindBranch(&done);
    }

    // Setup a call to C/C++ code, given the number of general arguments it
    // takes. Note that this only supports cdecl.
    //
    // In order for alignment to work correctly, the MacroAssembler must have a
    // consistent view of the stack displacement. It is okay to call "push"
    // manually, however, if the stack alignment were to change, the macro
    // assembler should be notified before starting a call.
    void setupAlignedABICall(uint32_t args);

    // Sets up an ABI call for when the alignment is not known. This may need a
    // scratch register.
    void setupUnalignedABICall(uint32_t args, const Register &scratch);

    // Arguments must be assigned to a C/C++ call in order. They are moved
    // in parallel immediately before performing the call. This process may
    // temporarily use more stack, in which case esp-relative addresses will be
    // automatically adjusted. It is extremely important that esp-relative
    // addresses are computed *after* setupABICall(). Furthermore, no
    // operations should be emitted while setting arguments.
    void passABIArg(const MoveOperand &from);
    void passABIArg(const Register &reg);
    void passABIArg(const FloatRegister &reg);

  private:
    void callWithABIPre(uint32_t *stackAdjust);
    void callWithABIPost(uint32_t stackAdjust, Result result);

  public:
    // Emits a call to a C/C++ function, resolving all argument moves.
    void callWithABI(void *fun, Result result = GENERAL);
    void callWithABI(const Address &fun, Result result = GENERAL);

    // Used from within an Exit frame to handle a pending exception.
    void handleFailureWithHandler(void *handler);

    void makeFrameDescriptor(Register frameSizeReg, FrameType type) {
        shll(Imm32(FRAMESIZE_SHIFT), frameSizeReg);
        orl(Imm32(type), frameSizeReg);
    }

    // Save an exit frame (which must be aligned to the stack pointer) to
    // ThreadData::ionTop of the main thread.
    void linkExitFrame() {
        JSCompartment *compartment = GetIonContext()->compartment;
        movl(StackPointer, Operand(&compartment->rt->mainThread.ionTop));
    }

    void callWithExitFrame(IonCode *target, Register dynStack) {
        addPtr(Imm32(framePushed()), dynStack);
        makeFrameDescriptor(dynStack, IonFrame_OptimizedJS);
        Push(dynStack);
        call(target);
    }

    // Save an exit frame to the thread data of the current thread, given a
    // register that holds a PerThreadData *.
    void linkParallelExitFrame(const Register &pt) {
        movl(StackPointer, Operand(pt, offsetof(PerThreadData, ionTop)));
    }

    void enterOsr(Register calleeToken, Register code) {
        push(Imm32(0)); // num actual args.
        push(calleeToken);
        push(Imm32(MakeFrameDescriptor(0, IonFrame_Osr)));
        ma_callIonHalfPush(code);
        #if ! defined (JS_CPU_MIPS)
        addl(Imm32(sizeof(uintptr_t) * 2), sp);
        #endif
    }

    void patchAsmJSGlobalAccess(unsigned offset, uint8_t *code, unsigned codeBytes,
                                unsigned globalDataOffset)
    {
        uint8_t *nextInsn = code + offset;
        JS_ASSERT(nextInsn <= code + codeBytes);
        uint8_t *target = code + codeBytes + globalDataOffset;
        ((int32_t *)nextInsn)[-1] = uintptr_t(target);
    }

  protected:
    // Bytes pushed onto the frame by the callee; includes frameDepth_. This is
    // needed to compute offsets to stack slots while temporary space has been
    // reserved for unexpected spills or C++ function calls. It is maintained
    // by functions which track stack alignment, which for clear distinction
    // use StudlyCaps (for example, Push, Pop).
    uint32_t framePushed_;

  public:
    MacroAssemblerMIPS()
      : inCall_(false),
        framePushed_(0),
        enoughMemory_(true)
    {
    }

    //by weizhenwei, 2013.11.27
    mJump branchTrue(bool branchTrue)
    {
        masm.appendJump();
        if (branchTrue) {
            masm.bc1t();
        } else {
            masm.bc1f();

        }
        masm.nop();

        //insertRelaxationWords(); replaced by following, weizhenwei, 2013.11.22
        /* We need four words for relaxation. */
        masm.beq(mRegisterID(zero.code()), mRegisterID(zero.code()), 3); // Jump over nops;
        masm.nop();
        masm.nop();
        masm.nop();

        return mJump(masm.newJmpSrc());
    }

    //by weizhenwei, 2013.11.27
    mJump branchDouble(DoubleCondition cond, mFPRegisterID left, mFPRegisterID right)
    {
        if (cond == DoubleUnordered) {
            masm.cud(left, right);
            return branchTrue(true);
        }
        if (cond == DoubleOrdered) {
            masm.cud(left, right);
            return branchTrue(false); // false
        }
        if (cond == DoubleEqual) {
            masm.ceqd(left, right);
            return branchTrue(true);
        }
        if (cond == DoubleNotEqual) {
            masm.ceqd(left, right);
            return branchTrue(false); // false
        }
        if (cond == DoubleGreaterThan) {
            masm.cngtd(left, right);
            return branchTrue(false); // false
        }
        if (cond == DoubleGreaterThanOrEqual) {
            masm.cnged(left, right);
            return branchTrue(false); // false
        }
        if (cond == DoubleLessThan) {
            masm.cltd(left, right);
            return branchTrue(true);
        }
        if (cond == DoubleLessThanOrEqual) {
            masm.cled(left, right);
            return branchTrue(true);
        }
        if (cond == DoubleEqualOrUnordered) {
            masm.cueqd(left, right);
            return branchTrue(true);
        }
        if (cond == DoubleNotEqualOrUnordered) {
            masm.ceqd(left, right);
            return branchTrue(false); // false
        }
        if (cond == DoubleGreaterThanOrUnordered) {
            masm.coled(left, right);
            return branchTrue(false); // false
        }
        if (cond == DoubleGreaterThanOrEqualOrUnordered) {
            masm.coltd(left, right);
            return branchTrue(false); // false
        }
        if (cond == DoubleLessThanOrUnordered) {
            masm.cultd(left, right);
            return branchTrue(true);
        }
        if (cond == DoubleLessThanOrEqualOrUnordered) {
            masm.culed(left, right);
            return branchTrue(true);
        }

        ASSERT(0);
        return branchTrue(true);
    }

    void branchDouble(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, Label *label)
    {
        JmpSrc j;

        //by weizhenwei, 2013.10.29
        if (cond & DoubleConditionBitInvert) {
            JS_ASSERT(0);
        }

        j = branchDouble(cond, mFPRegisterID(lhs.code()), mFPRegisterID(rhs.code())).getJmpSrc();

        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            masm.setNextJump(j, prev);
        }
    }

    //by weizhenwei, 2013.11.27
    void branchDoubleLocal(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, Label *label)
    {
        mFPRegisterID left = mFPRegisterID(lhs.code());
        mFPRegisterID right = mFPRegisterID(rhs.code());

        if (cond == DoubleUnordered) {
            masm.cud(left, right);
            bc1t(label);
        }
        if (cond == DoubleOrdered) {
            masm.cud(left, right);
            bc1f(label); //false
        }
        if (cond == DoubleEqual) {
            masm.ceqd(left, right);
            bc1t(label);
        }
        if (cond == DoubleNotEqual) {
            masm.ceqd(left, right);
            bc1f(label); //false
        }
        if (cond == DoubleGreaterThan) {
            masm.cngtd(left, right);
            bc1f(label); //false
        }
        if (cond == DoubleGreaterThanOrEqual) {
            masm.cnged(left, right);
            bc1f(label); //false
        }
        if (cond == DoubleLessThan) {
            masm.cltd(left, right);
            bc1t(label);
        }
        if (cond == DoubleLessThanOrEqual) {
            masm.cled(left, right);
            bc1t(label);
        }
        if (cond == DoubleEqualOrUnordered) {
            masm.cueqd(left, right);
            bc1t(label);
        }
        if (cond == DoubleNotEqualOrUnordered) {
            masm.ceqd(left, right);
            bc1f(label); //false
        }
        if (cond == DoubleGreaterThanOrUnordered) {
            masm.coled(left, right);
            bc1f(label); //false
        }
        if (cond == DoubleGreaterThanOrEqualOrUnordered) {
            masm.coltd(left, right);
            bc1f(label); //false
        }
        if (cond == DoubleLessThanOrUnordered) {
            masm.cultd(left, right);
            bc1t(label);
        }
        if (cond == DoubleLessThanOrEqualOrUnordered) {
            masm.culed(left, right);
            bc1t(label);
        }

        masm.nop();
    }

    void move32(const Imm32 &imm, const Register &dest) {
        if (imm.value == 0)
            xorl(dest, dest);
        else
            movl(imm, dest);
    }

    void move32(const Imm32 &imm, const Operand &dest) {
        movl(imm, dest);
    }

    void and32(const Imm32 &imm, const Register &dest) {
        andl(imm, dest);
    }

    void and32(const Imm32 &imm, const Address &dest) {
        andl(imm, Operand(dest));
    }
    void or32(const Imm32 &imm, const Register &dest) {
        orl(imm, dest);
    }
    void or32(const Imm32 &imm, const Address &dest) {
        orl(imm, Operand(dest));
    }
    void neg32(const Register &reg) {
        negl(reg);
    }
    void cmp32(const Register &lhs, const Imm32 &rhs) {
        cmpl(lhs, rhs);
    }
    void test32(const Register &lhs, const Register &rhs) {
	    // overwrite testl, by wangqing, 2013-12-02 
		andInsn(cmpTempRegister, lhs, rhs);
		movl(zero, cmpTemp2Register);
    }
    void test32(const Address &addr, Imm32 imm) {
        testl(Operand(addr), imm);
    }
    void cmp32(Register a, Register b) {
        cmpl(a, b);
    }
    void cmp32(const Operand &lhs, const Imm32 &rhs) {
        cmpl(lhs, rhs);
    }
    void cmp32(const Operand &lhs, const Register &rhs) {
        cmpl(lhs, rhs);
    }
    void add32(Register src, Register dest) {
        addl(src, dest);
    }
    void add32(Imm32 imm, Register dest) {
        addl(imm, dest);
    }
    void add32(Imm32 imm, const Address &dest) {
        addl(imm, Operand(dest));
    }
    void sub32(Imm32 imm, Register dest) {
        subl(imm, dest);
    }
    void sub32(Register src, Register dest) {
        subl(src, dest);
    }
    void xor32(Imm32 imm, Register dest) {
        xorl(imm, dest);
    }

    void branch32(Condition cond, const Operand &lhs, const Register &rhs, Label *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
    void branch32(Condition cond, const Operand &lhs, Imm32 rhs, Label *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
    void branch32(Condition cond, const Address &lhs, const Register &rhs, Label *label) {
        cmpl(Operand(lhs), rhs);
        j(cond, label);
    }
    void branch32(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        cmpl(Operand(lhs), imm);
        j(cond, label);
    }
    void branch32(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        cmpl(lhs, imm);
        j(cond, label);
    }
    void branch32(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
    void branchTest32(Condition cond, const Register &lhs, const Register &rhs, Label *label) {
        testl(lhs, rhs);
        j(cond, label);
    }
    void branchTest32(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        testl(lhs, imm);
        j(cond, label);
    }
    void branchTest32(Condition cond, const Address &address, Imm32 imm, Label *label) {
        testl(Operand(address), imm);
        j(cond, label);
    }

    // The following functions are exposed for use in platform-shared code.
    template <typename T>
    void Push(const T &t) {
        push(t);
        framePushed_ += STACK_SLOT_SIZE;
    }
    void Push(const FloatRegister &t) {
        push(t);
        framePushed_ += sizeof(double);
    }
    CodeOffsetLabel PushWithPatch(const ImmWord &word) {
        framePushed_ += sizeof(word.value);
        return pushWithPatch(word);
    }

    void Pop(const Register &reg) {
        pop(reg);
        framePushed_ -= STACK_SLOT_SIZE;
    }
    void implicitPop(uint32_t args) {
        JS_ASSERT(args % STACK_SLOT_SIZE == 0);
        framePushed_ -= args;
    }
    uint32_t framePushed() const {
        return framePushed_;
    }
    void setFramePushed(uint32_t framePushed) {
        framePushed_ = framePushed;
    }

    void jump(Label *label) {
        jmp(label);
    }
    void jump(RepatchLabel *label) {
        jmp(label);
    }
    void jump(Register reg) {
        jmp(Operand(reg));
    }

    void convertInt32ToDouble(const Register &src, const FloatRegister &dest) {
        cvtsi2sd(src, dest);
    }
    void convertInt32ToDouble(const Address &src, FloatRegister dest) {
        cvtsi2sd(Operand(src), dest);
    }
    Condition testDoubleTruthy(bool truthy, const FloatRegister &reg) {
		zerod(ScratchFloatReg);
        return truthy ? NonZero : Zero;
    }
    void load8ZeroExtend(const Address &src, const Register &dest) {
        movzbl(Operand(src), dest);
    }
    void load8ZeroExtend(const BaseIndex &src, const Register &dest) {
        movzbl(Operand(src), dest);
    }
    void load8SignExtend(const Address &src, const Register &dest) {
        movxbl(Operand(src), dest);
    }
    void load8SignExtend(const BaseIndex &src, const Register &dest) {
        movxbl(Operand(src), dest);
    }
    template <typename S, typename T>
    void store8(const S &src, const T &dest) {
        movb(src, Operand(dest));
    }
    void load16ZeroExtend(const Address &src, const Register &dest) {
        movzwl(Operand(src), dest);
    }
    void load16ZeroExtend(const BaseIndex &src, const Register &dest) {
        movzwl(Operand(src), dest);
    }
    template <typename S, typename T>
    void store16(const S &src, const T &dest) {
        movw(src, Operand(dest));
    }
    void load16SignExtend(const Address &src, const Register &dest) {
        movxwl(Operand(src), dest);
    }
    void load16SignExtend(const BaseIndex &src, const Register &dest) {
        movxwl(Operand(src), dest);
    }
    void load32(const Address &address, Register dest) {
        movl(Operand(address), dest);
    }
    void load32(const BaseIndex &src, Register dest) {
        movl(Operand(src), dest);
    }
    void load32(const Operand &src, Register dest) {
        movl(src, dest);
    }
    template <typename S, typename T>
    void store32(const S &src, const T &dest) {
        movl(src, Operand(dest));
    }
    void loadDouble(const Address &src, FloatRegister dest) {
        movsd(Operand(src), dest);
    }
    void loadDouble(const BaseIndex &src, FloatRegister dest) {
        movsd(Operand(src), dest);
    }
    void loadDouble(const Operand &src, FloatRegister dest) {
        movsd(src, dest);
    }
    void storeDouble(FloatRegister src, const Address &dest) {
        movsd(src, Operand(dest));
    }
    void storeDouble(FloatRegister src, const BaseIndex &dest) {
        movsd(src, Operand(dest));
    }
    void storeDouble(FloatRegister src, const Operand &dest) {
        movsd(src, dest);
    }
    void zeroDouble(FloatRegister reg) {
        zerod(reg);
    }
	// by wangqing ,2013-11-06 make the FLoatRegister negated.
    void negateDouble(FloatRegister reg) {
	    negd(reg, reg); 
    }
    void addDouble(FloatRegister src, FloatRegister dest) {
        addsd(src, dest);
    }
    void subDouble(FloatRegister src, FloatRegister dest) {
        subsd(src, dest);
    }
    void mulDouble(FloatRegister src, FloatRegister dest) {
        mulsd(src, dest);
    }
    void divDouble(FloatRegister src, FloatRegister dest) {
        divsd(src, dest);
    }
    void convertDoubleToFloat(const FloatRegister &src, const FloatRegister &dest) {
        cvtsd2ss(src, dest);
    }
    void loadFloatAsDouble(const Register &src, FloatRegister dest) {
		movss(src, dest);  // by wangqing, 2013-11-08
    }
    void loadFloatAsDouble(const Address &src, FloatRegister dest) {
        movss(Operand(src), dest);
    }
    void loadFloatAsDouble(const BaseIndex &src, FloatRegister dest) {
        movss(Operand(src), dest);
    }
    void loadFloatAsDouble(const Operand &src, FloatRegister dest) {
        movss(src, dest);
    }
    void storeFloat(FloatRegister src, const Address &dest) {
        movss(src, Operand(dest));
    }
    void storeFloat(FloatRegister src, const BaseIndex &dest) {
        movss(src, Operand(dest));
    }

    // Checks whether a double is representable as a 32-bit integer. If so, the
    // integer is written to the output register. Otherwise, a bailout is taken to
    // the given snapshot. This function overwrites the scratch float register.
    void convertDoubleToInt32(FloatRegister src, Register dest, Label *fail,
                              bool negativeZeroCheck = true)
    {
        cvttsd2si(src, dest);
        cvtsi2sd(dest, ScratchFloatReg);
		//by weizhenwei, 2013.11.05
		branchDouble(Assembler::DoubleUnordered, src, ScratchFloatReg, fail);
		branchDouble(Assembler::DoubleNotEqual, src, ScratchFloatReg, fail);

        // Check for -0
        if (negativeZeroCheck) {
		    // by wangqing, 2013-12-02
            Label notZero;
			bne(dest, zero, &notZero);
			nop();

            // bit 0 = sign of low double
            // bit 1 = sign of high double
            // move double's high 32 to dest and get its sign bit
            mfc1(dest, js::jit::FloatRegister::FromCode(src.code() + 1));
            shrl(Imm32(0x1f), dest);

            //add by QuQiuwen
            cmpl(dest,zero);
            j(Assembler::NonZero, fail);

            bindBranch(&notZero);
        }
    }

    void clampIntToUint8(Register src, Register dest) {
		// by wangqing, 2013-11-02
        Label inRange, done;
		movl(Imm32(0xffffff00), cmpTempRegister);
		andl(src, cmpTempRegister);
		beq(cmpTempRegister, zero, &inRange);
		nop();
        {
            Label negative;
			bltz(src, &negative);
			nop();
            {
                movl(Imm32(255), dest);
                b(&done);
				nop();
            }
            bindBranch(&negative);
            {
                xorl(dest, dest);
                b(&done);
				nop();
            }
        }
        bindBranch(&inRange);
        if (src != dest)
            movl(src, dest);
        bindBranch(&done);
    }

    bool maybeInlineDouble(uint64_t u, const FloatRegister &dest) {
        // This implements parts of "13.4 Generating constants" of
        // "2. Optimizing subroutines in assembly language" by Agner Fog,
        // generalized to handle any case that can use a pcmpeqw and
        // up to two shifts.

        if (u == 0) {
			zerod(dest);
            return true;
        }

        return false;
    }

    void emitSet(Assembler::Condition cond, const Register &dest,
                 Assembler::NaNCond ifNaN = Assembler::NaN_HandledByCond) {
        if (GeneralRegisterSet(Registers::SingleByteRegs).has(dest)) {
            // If the register we're defining is a single byte register,
            // take advantage of the setCC instruction
            setCC(cond, dest);
            movzxbl(dest, dest);

            if (ifNaN != Assembler::NaN_HandledByCond) {
                Label noNaN;
            	ASSERT(0);// test Parity!
                j(Assembler::NoParity, &noNaN);
                if (ifNaN == Assembler::NaN_IsTrue)
                    movl(Imm32(1), dest);
                else
                    xorl(dest, dest);
                bind(&noNaN);
            }
        } else {
            Label end;
            Label ifFalse;

            if (ifNaN == Assembler::NaN_IsFalse)
            	ASSERT(0);// test Parity!
			// by wangqing, 2013-12-11
			// Parity-->DoubleUnorder
            j(Assembler::Parity, &ifFalse);
            movl(Imm32(1), dest);
            j(cond, &end);
            if (ifNaN == Assembler::NaN_IsTrue)
            	ASSERT(0);// test Parity!
            j(Assembler::Parity, &end);
              
            bind(&ifFalse);
            xorl(dest, dest);

            bind(&end);
        }
    }

    // Emit a JMP that can be toggled to a CMP. See ToggleToJmp(), ToggleToCmp().
    CodeOffsetLabel toggledJump(Label *label) {
        CodeOffsetLabel offset(size());
        jump(label);
        return offset;
    }

    template <typename T>
    void computeEffectiveAddress(const T &address, Register dest) {
        lea(Operand(address), dest);
    }

    // Builds an exit frame on the stack, with a return address to an internal
    // non-function. Returns offset to be passed to markSafepointAt().
    bool buildFakeExitFrame(const Register &scratch, uint32_t *offset) {
        mozilla::DebugOnly<uint32_t> initialDepth = framePushed();

        CodeLabel cl;
        mov(cl.dest(), scratch);

        uint32_t descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
        Push(Imm32(descriptor));
        Push(scratch);

        bind(cl.src());
        *offset = currentOffset();

        JS_ASSERT(framePushed() == initialDepth + IonExitFrameLayout::Size());
        return addCodeLabel(cl);
    }

    bool buildOOLFakeExitFrame(void *fakeReturnAddr) {
        uint32_t descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
        Push(Imm32(descriptor));
        Push(ImmWord(fakeReturnAddr));
        return true;
    }

    void callWithExitFrame(IonCode *target) {
        uint32_t descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
        Push(Imm32(descriptor));
        call(target);
    }

    void callIon(const Register &callee){
         ma_callIonHalfPush(callee);
    }

    void checkStackAlignment() {
        // Exists for ARM compatibility.
    }

    CodeOffsetLabel labelForPatch() {
        return CodeOffsetLabel(size());
    }
    
    void abiret() {
        ret();
    }
};

typedef MacroAssemblerMIPS MacroAssemblerSpecific;

} // namespace jit
} // namespace js

#endif // js_jit_macro_assembler_mips_h__


