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
#include "jsopcode.h"

#include "jit/IonFrames.h"
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
    uint32_t passedArgsfake_;
    uint32_t passedArgsBits_[4];//bitmap, 1 is double, 2 is int
    uint32_t stackForCall_;
    bool dynamicAlignment_;
    bool enoughMemory_;

    struct Double {
        double value;
        AbsoluteLabel uses;
        Double(double value) : value(value) {}
    };
    Vector<Double, 0, SystemAllocPolicy> doubles_;
    struct Float {
        float value;
        AbsoluteLabel uses;
        Float(float value) : value(value) {}
    };
    Vector<Float, 0, SystemAllocPolicy> floats_;

    typedef HashMap<double, size_t, DefaultHasher<double>, SystemAllocPolicy> DoubleMap;
    DoubleMap doubleMap_;
    typedef HashMap<float, size_t, DefaultHasher<float>, SystemAllocPolicy> FloatMap;
    FloatMap floatMap_;

    Double *getDouble(double d);
    Float *getFloat(float f);

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
        DOUBLE,
        FLOAT
    };

    // The buffer is about to be linked, make sure any constant pools or excess
    // bookkeeping has been flushed to the instruction stream.
    void finish();

    bool oom() const {
        return Assembler::oom() || !enoughMemory_;
    }

    /////////////////////////////////////////////////////////////////
    // X86-specific interface.
    /////////////////////////////////////////////////////////////////

    Operand ToPayload(Operand base) {
        return base;
    }
    Operand ToType(Operand base) {
        switch (base.kind()) {
          case Operand::MEM_REG_DISP:
            return Operand(Register::FromCode(base.base()), base.disp() + sizeof(void *));

          case Operand::MEM_SCALE:
            return Operand(Register::FromCode(base.base()), Register::FromCode(base.index()),
                           base.scale(), base.disp() + sizeof(void *));

          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
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
        Register s0 = src.typeReg(), d0 = dest.typeReg(),
                 s1 = src.payloadReg(), d1 = dest.payloadReg();

        // Either one or both of the source registers could be the same as a
        // destination register.
        if (s1 == d0) {
            if (s0 == d1) {
                // If both are, this is just a swap of two registers.
                xchgl(d0, d1);
                return;
            }
            // If only one is, copy that source first.
            mozilla::Swap(s0, s1);
            mozilla::Swap(d0, d1);
        }

        if (s0 != d0)
            movl(s0, d0);
        if (s1 != d1)
            movl(s1, d1);
    }

    /////////////////////////////////////////////////////////////////
    // X86/X64-common interface.
    /////////////////////////////////////////////////////////////////
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
        Register indexReg = (src.kind() == Operand::MEM_SCALE) ? Register::FromCode(src.index()) : InvalidReg;

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
        JS_ASSERT(dest.typeReg() != dest.payloadReg());
        if (payload != dest.payloadReg())
            movl(payload, dest.payloadReg());
        movl(ImmType(type), dest.typeReg());
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
    void Push(const ValueOperand &val) {
        pushValue(val);
        framePushed_ += sizeof(Value);
    }
    void Pop(const ValueOperand &val) {
        popValue(val);
        framePushed_ -= sizeof(Value);
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


    Condition testUndefined(Condition cond, const Operand &operand) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(ToType(operand), ImmTag(JSVAL_TAG_UNDEFINED));
        return cond;
    }
    Condition testUndefined(Condition cond, const Address &addr) {
        return testUndefined(cond, Operand(addr));
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
        // Check payload before tag, since payload is more likely to differ.
        if (cond == NotEqual) {
            branchPtr(NotEqual, payloadOf(valaddr), value.payloadReg(), label);
            branchPtr(NotEqual, tagOf(valaddr), value.typeReg(), label);

        } else {
            Label fallthrough;
            branchPtr(NotEqual, payloadOf(valaddr), value.payloadReg(), &fallthrough);
            branchPtr(Equal, tagOf(valaddr), value.typeReg(), label);
            bind(&fallthrough);
        }
    }

    void cmpPtr(Register lhs, const ImmWord rhs) {
        cmpl(lhs, Imm32(rhs.value));
    }
    void cmpPtr(Register lhs, const ImmPtr imm) {
        cmpPtr(lhs, ImmWord(uintptr_t(imm.value)));
    }
    void cmpPtr(Register lhs, const ImmGCPtr rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const ImmWord rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const ImmPtr imm) {
        cmpPtr(lhs, ImmWord(uintptr_t(imm.value)));
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
    void cmpPtr(const Address &lhs, const ImmPtr rhs) {
        cmpPtr(lhs, ImmWord(uintptr_t(rhs.value)));
    }
    void cmpPtr(Register lhs, Register rhs) {
        cmpl(lhs, rhs);
    }
    void testPtr(Register lhs, Register rhs) {
        testl(lhs, rhs);
    }

    Condition testNegativeZero(const FloatRegister &reg, const Register &scratch);
    Condition testNegativeZeroFloat32(const FloatRegister &reg, const Register &scratch);

    /////////////////////////////////////////////////////////////////
    // Common interface.
    /////////////////////////////////////////////////////////////////
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
    void addPtr(ImmPtr imm, const Register &dest) {
        addPtr(ImmWord(uintptr_t(imm.value)), dest);
    }
    void addPtr(Imm32 imm, const Address &dest) {
        addl(imm, Operand(dest));
    }
    void addPtr(Imm32 imm, const Operand &dest) {
        addl(imm, dest);
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

    // Specialization for AsmJSAbsoluteAddress.
    void branchPtr(Condition cond, AsmJSAbsoluteAddress lhs, Register ptr, Label *label) {
        cmpl(lhs, ptr);
        j(cond, label);
    }

    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, Label *label) {
        cmpl(Operand(lhs), ptr);
        j(cond, label);
    }

    void branchPrivatePtr(Condition cond, const Address &lhs, ImmPtr ptr, Label *label) {
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

    CodeOffsetJump jumpWithPatch(RepatchLabel *label, Assembler::Condition cond) {
        j(cond, label);
        return CodeOffsetJump(size());
    }

    // Add by weizhenwei, 2013.12.24
    CodeOffsetJump jumpWithPatch(RepatchLabel *label, const FloatRegister &lhs,
            const FloatRegister &rhs, Assembler::DoubleCondition cond) {
        branchDoubleImpl(cond, lhs, rhs, label);
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
        testl(lhs, rhs);
        j(cond, label);
    }
    void branchTestPtr(Condition cond, Register lhs, Imm32 imm, Label *label) {
        testl(lhs, imm);
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
    void movePtr(ImmPtr imm, Register dest) {
        movl(imm, dest);
    }
    void movePtr(AsmJSImmPtr imm, Register dest) {
        mov(imm, dest);
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
    void storePtr(ImmPtr imm, const Address &address) {
        storePtr(ImmWord(uintptr_t(imm.value)), address);
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
//        movd(src, dest.payloadReg());
//        psrldq(Imm32(4), src);
//        movd(src, dest.typeReg());
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
    void unboxObject(const ValueOperand &src, const Register &dest) {
        if (src.payloadReg() != dest)
            movl(src.payloadReg(), dest);
    }
    void unboxDouble(const ValueOperand &src, const FloatRegister &dest) {
        JS_ASSERT(dest != ScratchFloatReg);

        fastLoadDouble(src.payloadReg(), src.typeReg(), dest);
    }
    //It is different with x86!
    //xsb:fixme
    void unboxDouble(const Operand &payload, const Operand &type,
                     const Register &scratch, const FloatRegister &dest) {
        JS_ASSERT(dest != ScratchFloatReg);
        JS_ASSERT(0);
    }
    void unboxString(const ValueOperand &src, const Register &dest) {
        movl(src.payloadReg(), dest);
    }
    void unboxString(const Address &src, const Register &dest) {
        movl(payloadOf(src), dest);
    }
    void unboxValue(const ValueOperand &src, AnyRegister dest) {
        if (dest.isFloat()) {
            Label notInt32, end;
            branchTestInt32(Assembler::NotEqual, src, &notInt32);
            convertInt32ToDouble(src.payloadReg(), dest.fpu());
            jump(&end);
            bind(&notInt32);
            unboxDouble(src, dest.fpu());
            bind(&end);
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
        convertInt32ToDouble(operand.payloadReg(), dest);
    }
    void boolValueToFloat32(const ValueOperand &operand, const FloatRegister &dest) {
        convertInt32ToFloat32(operand.payloadReg(), dest);
    }
    void int32ValueToDouble(const ValueOperand &operand, const FloatRegister &dest) {
        convertInt32ToDouble(operand.payloadReg(), dest);
    }
    void int32ValueToFloat32(const ValueOperand &operand, const FloatRegister &dest) {
        convertInt32ToFloat32(operand.payloadReg(), dest);
    }

    void loadConstantDouble(double d, const FloatRegister &dest);
    void addConstantDouble(double d, const FloatRegister &dest);
    void loadConstantFloat32(float f, const FloatRegister &dest);
    void addConstantFloat32(float f, const FloatRegister &dest);

    // by wangqing
    void branchTruncateDouble(const FloatRegister &src, const Register &dest, Label *fail) {
        cvttsd2si(src, dest);

        // cvttsd2si returns 0x80000000 on failure. Test for it by
        // subtracting 1 and testing overflow (this permits the use of a
        // smaller immediate field).
        cmpl(dest, Imm32(0x7fffffff));
        j(Assembler::Equal, fail);
    }
    void branchTruncateFloat32(const FloatRegister &src, const Register &dest, Label *fail) {
        cvttss2si(src, dest);

        // cvttss2si returns 0x80000000 on failure. Test for it by
        // subtracting 1 and testing overflow (this permits the use of a
        // smaller immediate field).
        cmpl(dest, Imm32(0x7fffffff));
        j(Assembler::Equal, fail);
    }

    Condition testInt32Truthy(bool truthy, const ValueOperand &operand) {
        testl(operand.payloadReg(), operand.payloadReg());
        return truthy ? NonZero : Zero;
    }
    void branchTestBooleanTruthy(bool truthy, const ValueOperand &operand, Label *label) {
        testl(operand.payloadReg(), operand.payloadReg());
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
        branchTestInt32(Assembler::NotEqual, operand, &notInt32);
        convertInt32ToDouble(ToPayload(operand), dest);
        jump(&end);
        bind(&notInt32);
        loadDouble(operand, dest);
        bind(&end);
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
        convertInt32ToDouble(src, dest);

        // dest is now a double with the int range.
        // correct the double value by adding 0x80000000.
        addConstantDouble(2147483648.0, dest);
    }

    // Note: this function clobbers the source register.
    void convertUInt32ToFloat32(const Register &src, const FloatRegister &dest) {
        // src is [0, 2^32-1]
        subl(Imm32(0x80000000), src);

        // Do it the GCC way
        convertInt32ToFloat32(src, dest);

        // dest is now a double with the int range.
        // correct the double value by adding 0x80000000.
        addConstantFloat32(2147483648.f, dest);
    }

    void inc64(AbsoluteAddress dest) {
        addl(Imm32(1), Operand(dest));
        Label noOverflow;
        // add by QuQiuwen
        cmpl(Operand(dest), zero);
        j(NonZero, &noOverflow);
        addl(Imm32(1), Operand(dest.offset(4)));
        bind(&noOverflow);
    }


    // If source is a double, load it into dest. If source is int32,
    // convert it to double. Else, branch to failure.
    void ensureDouble(const ValueOperand &source, FloatRegister dest, Label *failure) {
        Label isDouble, done;
        branchTestDouble(Assembler::Equal, source.typeReg(), &isDouble);
        branchTestInt32(Assembler::NotEqual, source.typeReg(), failure);

        convertInt32ToDouble(source.payloadReg(), dest);
        jump(&done);

        bind(&isDouble);
        unboxDouble(source, dest);

        bind(&done);
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
    void callWithABI(AsmJSImmPtr fun, Result result = GENERAL);
    void callWithABI(const Address &fun, Result result = GENERAL);

    // Used from within an Exit frame to handle a pending exception.
    void handleFailureWithHandler(void *handler);
    void handleFailureWithHandlerTail();

    void makeFrameDescriptor(Register frameSizeReg, FrameType type) {
        shll(Imm32(FRAMESIZE_SHIFT), frameSizeReg);
        orl(Imm32(type), frameSizeReg);
    }

    // Save an exit frame (which must be aligned to the stack pointer) to
    // ThreadData::ionTop of the main thread.
    void linkExitFrame() {
        movl(StackPointer, Operand(AbsoluteAddress(GetIonContext()->runtime->addressOfIonTop())));
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

    void enterOsr(Register calleeToken, Register code);


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

    void compareDouble(DoubleCondition cond, const FloatRegister &lhs, const FloatRegister &rhs) {
        ASSERT(0);
    }

    //by weizhenwei, 2013.12.24
    void branchDoubleImpl(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, Label *label)
    {
        JmpSrc j;
        if (cond & DoubleConditionBitInvert) {
            j = mcss.branchDouble(static_cast<JSC::MacroAssemblerMIPS::DoubleCondition>(cond),
                    rhs.code(), lhs.code()).m_jmp;
        } else {
            j = mcss.branchDouble(static_cast<JSC::MacroAssemblerMIPS::DoubleCondition>(cond),
                    lhs.code(), rhs.code()).m_jmp;
        }

        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            masm.setNextJump(j, prev);
        }
    //    return j;
    }
    void branchDoubleImpl(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, RepatchLabel *label)
    {
        JmpSrc j;
        if (cond & DoubleConditionBitInvert) {
            j = mcss.branchDouble(static_cast<JSC::MacroAssemblerMIPS::DoubleCondition>(cond),
                    rhs.code(), lhs.code()).m_jmp;
        } else {
            j = mcss.branchDouble(static_cast<JSC::MacroAssemblerMIPS::DoubleCondition>(cond),
                    lhs.code(), rhs.code()).m_jmp;
        }

        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            label->use(j.offset());
        }
    //    return j;
    }
    // TODO: consider cond & DoubleConditionBitInvert, weizhenwei, 3013.12.27
    void branchDouble(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, Label *label)
    {
        if (cond == DoubleEqual) {
            Label unordered;
            branchDoubleImpl(Assembler::DoubleUnordered, lhs, rhs,  &unordered);
            branchDoubleImpl(Assembler::DoubleEqual, lhs, rhs, label);
            bind(&unordered);
            return;
        }
        if (cond == DoubleNotEqualOrUnordered) {
            branchDoubleImpl(Assembler::DoubleNotEqual, lhs, rhs, label);
            branchDoubleImpl(Assembler::DoubleUnordered, lhs, rhs, label);
            return;
        }

        JS_ASSERT(!(cond & DoubleConditionBitSpecial));
        branchDoubleImpl(cond, lhs, rhs, label);
    }

    void compareFloat(DoubleCondition cond, const FloatRegister &lhs, const FloatRegister &rhs) {
        if (cond & DoubleConditionBitInvert)
            ucomiss(rhs, lhs);
        else
            ucomiss(lhs, rhs);
    }
    void branchFloat(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, Label *label)
    {
        if (cond == DoubleEqual) {
            Label unordered;
            branchDoubleImpl(Assembler::DoubleUnordered, lhs, rhs,  &unordered);
            branchDoubleImpl(Assembler::DoubleEqual, lhs, rhs, label);
            bind(&unordered);
            return;
        }
        if (cond == DoubleNotEqualOrUnordered) {
            branchDoubleImpl(Assembler::DoubleNotEqual, lhs, rhs, label);
            branchDoubleImpl(Assembler::DoubleUnordered, lhs, rhs, label);
            return;
        }

        JS_ASSERT(!(cond & DoubleConditionBitSpecial));
        branchDoubleImpl(cond, lhs, rhs, label);
    }

    void move32(const Imm32 &imm, const Register &dest) {
        // Use the ImmWord version of mov to register, which has special
        // optimizations. Casting to uint32_t here ensures that the value
        // is zero-extended.
        mov(ImmWord(uint32_t(imm.value)), dest);
    }
    void move32(const Imm32 &imm, const Operand &dest) {
        movl(imm, dest);
    }
    void move32(const Register &src, const Register &dest) {
        movl(src, dest);
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
        testl(lhs, rhs);
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
    void not32(Register reg) {
        notl(reg);
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
    CodeOffsetLabel PushWithPatch(const ImmPtr &imm) {
        return PushWithPatch(ImmWord(uintptr_t(imm.value)));
    }

    template <typename T>
    void Pop(const T &t) {
        pop(t);
        framePushed_ -= STACK_SLOT_SIZE;
    }
    void Pop(const FloatRegister &t) {
        pop(t);
        framePushed_ -= sizeof(double);
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
    void jump(const Address &addr) {
        jmp(Operand(addr));
    }

    void convertInt32ToDouble(const Register &src, const FloatRegister &dest) {
        // cvtsi2sd and friends write only part of their output register, which
        // causes slowdowns on out-of-order processors. Explicitly break
        // dependencies with xorpd (and xorps elsewhere), which are handled
        // specially in modern CPUs, for this purpose. See sections 8.14, 9.8,
        // 10.8, 12.9, 13.16, 14.14, and 15.8 of Agner's Microarchitecture
        // document.
        zeroDouble(dest);
        cvtsi2sd(src, dest);
    }
    void convertInt32ToDouble(const Address &src, FloatRegister dest) {
        convertInt32ToDouble(Operand(src), dest);
    }
    void convertInt32ToDouble(const Operand &src, FloatRegister dest) {
        // Clear the output register first to break dependencies; see above;
        zeroDouble(dest);
        cvtsi2sd(Operand(src), dest);
    }
    void convertInt32ToFloat32(const Register &src, const FloatRegister &dest) {
        // Clear the output register first to break dependencies; see above;
        zeroFloat32(dest);
        cvtsi2ss(src, dest);
    }
    void convertInt32ToFloat32(const Address &src, FloatRegister dest) {
        convertInt32ToFloat32(Operand(src), dest);
    }
    void convertInt32ToFloat32(const Operand &src, FloatRegister dest) {
        // Clear the output register first to break dependencies; see above;
        zeroFloat32(dest);
        cvtsi2ss(src, dest);
    }
    Condition testDoubleTruthy(bool truthy, const FloatRegister &reg) {
        //zeroDouble(ScratchFloatReg);
        //ucomisd(ScratchFloatReg, reg);
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
        movsbl(Operand(src), dest);
    }
    void load8SignExtend(const BaseIndex &src, const Register &dest) {
        movsbl(Operand(src), dest);
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
        movswl(Operand(src), dest);
    }
    void load16SignExtend(const BaseIndex &src, const Register &dest) {
        movswl(Operand(src), dest);
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
        movsd(src, dest);
    }
    void loadDouble(const BaseIndex &src, FloatRegister dest) {
        movsd(src, dest);
    }
    void loadDouble(const Operand &src, FloatRegister dest) {
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
            loadDouble(src.toAddress(), dest);
            break;
          case Operand::MEM_SCALE:
            loadDouble(src.toBaseIndex(), dest);
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void storeDouble(FloatRegister src, const Address &dest) {
        movsd(src, dest);
    }
    void storeDouble(FloatRegister src, const BaseIndex &dest) {
        movsd(src, dest);
    }
    void storeDouble(FloatRegister src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
            storeDouble(src, dest.toAddress());
            break;
          case Operand::MEM_SCALE:
            storeDouble(src, dest.toBaseIndex());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void moveDouble(FloatRegister src, FloatRegister dest) {
        // Use movapd instead of movsd to avoid dependencies.
        movapd(src, dest);
    }
    void zeroDouble(FloatRegister reg) {
        xorpd(reg, reg);
    }
    void zeroFloat32(FloatRegister reg) {
        xorps(reg, reg);
    }
    // by wangqing, 2013-11-06 make the DoubleRegister negated.
    void negateDouble(FloatRegister reg) {
        negd(reg, reg);
    }
    // by wangqing, 2013-12-25 make the FloatRegister negated.
    void negateFloat(FloatRegister reg) {
        negs(reg, reg);
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
    void convertFloatToDouble(const FloatRegister &src, const FloatRegister &dest) {
        cvtss2sd(src, dest);
    }
    void convertDoubleToFloat(const FloatRegister &src, const FloatRegister &dest) {
        cvtsd2ss(src, dest);
    }
    void moveFloatAsDouble(const Register &src, FloatRegister dest) {
        movd(src, dest);
        cvtss2sd(dest, dest);
    }
    // add by wangqing. 2013-12-25
    void loadFloatAsDouble(const Register &src, FloatRegister dest) {
        movss(src, dest);
        cvtss2sd(dest, dest);
    }
    void loadFloatAsDouble(const Address &src, FloatRegister dest) {
        movss(src, dest);
        cvtss2sd(dest, dest);
    }
    void loadFloatAsDouble(const BaseIndex &src, FloatRegister dest) {
        movss(src, dest);
        cvtss2sd(dest, dest);
    }
    void loadFloatAsDouble(const Operand &src, FloatRegister dest) {
        loadFloat(src, dest);
        cvtss2sd(dest, dest);
    }
    void loadFloat(const Address &src, FloatRegister dest) {
        movss(src, dest);
    }
    void loadFloat(const BaseIndex &src, FloatRegister dest) {
        movss(src, dest);
    }
    void loadFloat(const Operand &src, FloatRegister dest) {
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
            loadFloat(src.toAddress(), dest);
            break;
          case Operand::MEM_SCALE:
            loadFloat(src.toBaseIndex(), dest);
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void storeFloat(FloatRegister src, const Address &dest) {
        movss(src, dest);
    }
    void storeFloat(FloatRegister src, const BaseIndex &dest) {
        movss(src, dest);
    }
    void storeFloat(FloatRegister src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
            storeFloat(src, dest.toAddress());
            break;
          case Operand::MEM_SCALE:
            storeFloat(src, dest.toBaseIndex());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void moveFloat(FloatRegister src, FloatRegister dest) {
        // Use movaps instead of movss to avoid dependencies.
        movaps(src, dest);
    }

    // Checks whether a double is representable as a 32-bit integer. If so, the
    // integer is written to the output register. Otherwise, a bailout is taken to
    // the given snapshot. This function overwrites the scratch float register.
    void convertDoubleToInt32(FloatRegister src, Register dest, Label *fail,
                              bool negativeZeroCheck = true)
    {
        cvttsd2si(src, dest);
        cvtsi2sd(dest, ScratchFloatReg);
//        ucomisd(src, ScratchFloatReg);
//        j(Assembler::Parity, fail);
//        j(Assembler::NotEqual, fail);
        branchDouble(Assembler::DoubleUnordered, src, ScratchFloatReg, fail);
        branchDouble(Assembler::DoubleNotEqual, src, ScratchFloatReg, fail);

        // Check for -0
        if (negativeZeroCheck) {
            Label notZero;
            testl(dest, dest);
            j(Assembler::NonZero, &notZero);

            // bit 0 = sign of low double
            // bit 1 = sign of high double
            // move double's high 32 to dest and get its sign bit
            mfc1(dest, js::jit::FloatRegister::FromCode(src.code() + 1));
            shrl(Imm32(0x1f), dest);

            //cmpl(zero, dest);
            cmpl(dest,zero);
            j(Assembler::NonZero, fail);

            bind(&notZero);
        }
    }

    // Checks whether a float32 is representable as a 32-bit integer. If so, the
    // integer is written to the output register. Otherwise, a bailout is taken to
    // the given snapshot. This function overwrites the scratch float register.
    void convertFloat32ToInt32(FloatRegister src, Register dest, Label *fail,
                               bool negativeZeroCheck = true)
    {
        cvttss2si(src, dest);
        convertInt32ToFloat32(dest, ScratchFloatReg);
        //ucomiss(src, ScratchFloatReg);
        //j(Assembler::Parity, fail);
        //j(Assembler::NotEqual, fail);
        branchDouble(Assembler::DoubleUnordered, src, ScratchFloatReg, fail);
        branchDouble(Assembler::DoubleNotEqual, src, ScratchFloatReg, fail);

        // Check for -0
        if (negativeZeroCheck) {
            Label notZero;
            branchTest32(Assembler::NonZero, dest, dest, &notZero);

//            if (Assembler::HasSSE41()) {
//                ptest(src, src);
//                j(Assembler::NonZero, fail);
//            } else {
                // bit 0 = sign of low float
                // bits 1 to 3 = signs of higher floats
                //movmskps(src, dest);
                //andl(Imm32(1), dest);
                // move double's high 32 to dest and get its sign bit
                // TODO: check whether float and double are same, weizhenwei, 2013.12.26
                mfc1(dest, js::jit::FloatRegister::FromCode(src.code() + 1));
                shrl(Imm32(0x1f), dest);

                cmpl(zero, dest);
                j(Assembler::NonZero, fail);
//            }

            bind(&notZero);
        }
    }

    void clampIntToUint8(Register reg) {
        Label inRange;
        branchTest32(Assembler::Zero, reg, Imm32(0xffffff00), &inRange);
        {
            sarl(Imm32(31), reg);
            notl(reg);
            andl(Imm32(255), reg);
        }
        bind(&inRange);
    }

    bool maybeInlineDouble(double d, const FloatRegister &dest) {
        uint64_t u = mozilla::BitwiseCast<uint64_t>(d);

        // Loading zero with xor is specially optimized in hardware.
        if (u == 0) {
            xorpd(dest, dest);
            return true;
        }

        // It is also possible to load several common constants using pcmpeqw
        // to get all ones and then psllq and psrlq to get zeros at the ends,
        // as described in "13.4 Generating constants" of
        // "2. Optimizing subroutines in assembly language" by Agner Fog, and as
        // previously implemented here. However, with x86 and x64 both using
        // constant pool loads for double constants, this is probably only
        // worthwhile in cases where a load is likely to be delayed.

        return false;
    }

    bool maybeInlineFloat(float f, const FloatRegister &dest) {
        uint32_t u = mozilla::BitwiseCast<uint32_t>(f);

        // See comment above
        if (u == 0) {
            xorps(dest, dest);
            return true;
        }
        return false;
    }

    void convertBoolToInt32(Register source, Register dest) {
        // Note that C++ bool is only 1 byte, so zero extend it to clear the
        // higher-order bits.
        movzbl(source, dest);
    }

    void emitSet(Assembler::Condition cond, const Register &dest,
                 Assembler::NaNCond ifNaN = Assembler::NaN_HandledByCond) {
        if (GeneralRegisterSet(Registers::SingleByteRegs).has(dest)) {
            // If the register we're defining is a single byte register,
            // take advantage of the setCC instruction
            setCC(cond, dest);

            if (ifNaN != Assembler::NaN_HandledByCond) {
                Label noNaN;
                mov(ImmWord(ifNaN == Assembler::NaN_IsTrue), dest);
                bind(&noNaN);
            }
        } else {
            Label end;
            Label ifFalse;

            if (ifNaN == Assembler::NaN_IsFalse)
                ASSERT(0);
            // Note a subtlety here: FLAGS is live at this point, and the
            // mov interface doesn't guarantee to preserve FLAGS. Use
            // movl instead of mov, because the movl instruction
            // preserves FLAGS.
            movl(Imm32(1), dest);
            j(cond, &end);
            if (ifNaN == Assembler::NaN_IsTrue)
                ASSERT(0);
            bind(&ifFalse);
            mov(ImmWord(0), dest);

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

    void callWithExitFrame(IonCode *target) {
        uint32_t descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
        Push(Imm32(descriptor));
        call(target);
    }
    void callIon(const Register &callee);

    void checkStackAlignment() {
        // Exists for ARM compatibility.
    }

    CodeOffsetLabel labelForPatch() {
        return CodeOffsetLabel(size());
    }

    void abiret() {
        ret();
    }

  protected:
    bool buildOOLFakeExitFrame(void *fakeReturnAddr) {
        uint32_t descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
        Push(Imm32(descriptor));
        Push(ImmPtr(fakeReturnAddr));
        return true;
    }
};

typedef MacroAssemblerMIPS MacroAssemblerSpecific;

} // namespace jit
} // namespace js

#endif /* jit_shared_MacroAssembler_mips_shared_h */
