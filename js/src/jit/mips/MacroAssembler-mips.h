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
    uint32_t stackForCall_;
    bool dynamicAlignment_;
    bool enoughMemory_;

//NOTE*:this is new in ff24
    struct Double {
        double value;
        AbsoluteLabel uses;
        Double(double value) : value(value) {}
    };
    Vector<Double, 0, SystemAllocPolicy> doubles_; //this is only used in loadConstantDouble() and finish();

    typedef HashMap<double, size_t, DefaultHasher<double>, SystemAllocPolicy> DoubleMap;
    DoubleMap doubleMap_;
    
    
  protected:
    MoveResolver moveResolver_;

  private:
    Operand payloadOf(const Address &address) {  //获取到address所指向的操作数的payload域
        return Operand(address.base, address.offset);
    }
    Operand tagOf(const Address &address) {//为地址生成一个tag，使用的时候都用在compare类的指令时，应该是用于标记跳转的地址
        return Operand(address.base, address.offset + 4);//一般和payloadof一起使用，表明紧接着payload域存放
    }
    Operand tagOf(const BaseIndex &address) {//TBD BaseIndex special treat 获取到BaseIndex的tag域
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

//NOTE*:this is new in ff24
    // The buffer is about to be linked, make sure any constant pools or excess
    // bookkeeping has been flushed to the instruction stream.
    void finish();
    
    
    bool oom() const {
        return Assembler::oom() || !enoughMemory_;
    }

    /////////////////////////////////////////////////////////////////
    // X86-specific interface.
    /////////////////////////////////////////////////////////////////

    Operand ToPayload(Operand base) { //获取地址操作数
        return base;
    }
    Operand ToType(Operand base) {
        switch (base.kind()) {
          case Operand::REG_DISP:
            return Operand(Register::FromCode(base.base()), base.disp() + sizeof(void *));

          case Operand::SCALE:
            return Operand(Register::FromCode(base.base()), Register::FromCode(base.index()),
                           base.scale(), base.disp() + sizeof(void *));

          default:
            JS_NOT_REACHED("unexpected operand kind");
            return base; // Silence GCC warning.
        }
    }
    void moveValue(const Value &val, Register type, Register data) {//将一个boxed的值移动到寄存器，boxed值的高32位存放的是存储类型编码type，低32位存放实际的数据data
        jsval_layout jv = JSVAL_TO_IMPL(val);
        movl(Imm32(jv.s.tag), type);//生成一个立即数到寄存器的move指令
        if (val.isMarkable())
            movl(ImmGCPtr(reinterpret_cast<gc::Cell *>(val.toGCThing())), data);//根据对象的不同生成move指令或者store指令
        else
            movl(Imm32(jv.s.payload.i32), data);
    }
    void moveValue(const Value &val, const ValueOperand &dest) {//ValueOperand 其成员变量为两个Register的对象，用于存放一个boxed的值，一个boxed的值用一个64位的寄存器或者两个32位的寄存器来表示，

																				//tag和payload。使用typeReg来获取tag，用payloadReg来获取payload
        moveValue(val, dest.typeReg(), dest.payloadReg());
    }
//NOTE*:this is new in ff24
    void moveValue(const ValueOperand &src, const ValueOperand &dest) {
        JS_ASSERT(src.typeReg() != dest.payloadReg());
        JS_ASSERT(src.payloadReg() != dest.typeReg());
        if (src.typeReg() != dest.typeReg())
            movl(src.typeReg(), dest.typeReg());
        if (src.payloadReg() != dest.payloadReg())
            movl(src.payloadReg(), dest.payloadReg());
    }

    /////////////////////////////////////////////////////////////////
    // X86/X64-common interface.
    /////////////////////////////////////////////////////////////////
    void storeValue(ValueOperand val, Operand dest) {//将一个boxed的值存放至内存
        movl(val.payloadReg(), ToPayload(dest));//先存payload
        movl(val.typeReg(), ToType(dest));//再存放tag域
    }
    void storeValue(ValueOperand val, const Address &dest) {
        storeValue(val, Operand(dest));
    }
    template <typename T>
    void storeValue(JSValueType type, Register reg, const T &dest) {//JSValueType为一个1个字节大小，用于存放JS执行时数据的类型
        storeTypeTag(ImmTag(JSVAL_TYPE_TO_TAG(type)), Operand(dest));
        storePayload(reg, Operand(dest));
    }
    template <typename T>
    void storeValue(const Value &val, const T &dest) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        storeTypeTag(ImmTag(jv.s.tag), Operand(dest));
        storePayload(val, Operand(dest));
    }
    void storeValue(ValueOperand val, BaseIndex dest) {//TBD BaseIndex special treat
        storeValue(val, Operand(dest));
    }
    void loadValue(Operand src, ValueOperand val) {//从内存中取出一个值，正确的区分payload域和type域，存放至ValueOperand中
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
    void loadValue(const BaseIndex &src, ValueOperand val) {//TBD BaseIndex special treat
        loadValue(Operand(src), val);
    }
    void tagValue(JSValueType type, Register payload, ValueOperand dest) {//将payload的值打上tag，包装成ValueOperand
        JS_ASSERT(payload != dest.typeReg());
        movl(ImmType(type), dest.typeReg());
        if (payload != dest.payloadReg())
            movl(payload, dest.payloadReg());
    }
    void pushValue(ValueOperand val) {//将一个boxed的值压栈
        push(val.typeReg());
        push(val.payloadReg());
    }
    void popValue(ValueOperand val) {
        pop(val.payloadReg());
        pop(val.typeReg());
    }
    void pushValue(const Value &val) {//将一个Value类型压栈，Value为JS执行时使用的对象，包括多个类型，可通过 JSVAL_TO_IMPL(val)来获取类型信息
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
    //NOTE*:this is new in ff24
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
    //NOTE*:this is new in ff24
    void movePtr(const Register &src, const Operand &dest) {
        movl(src, dest);
    }
    
        // Returns the register containing the type tag.仅返回类型标签的寄存器
    Register splitTagForTest(const ValueOperand &value) {
        return value.typeReg();
    }

//这些test函数都用于检测数据的标签
    Condition testUndefined(Condition cond, const Register &tag) {//用于测试标签是否被定义
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_UNDEFINED));
        return cond;
    }
    Condition testBoolean(Condition cond, const Register &tag) {//用于测试标签是否为bool型
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_BOOLEAN));
        return cond;
    }
    Condition testInt32(Condition cond, const Register &tag) {//测试标签是否为int32型
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_INT32));
        return cond;
    }
    Condition testDouble(Condition cond, const Register &tag) { //?
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
    Condition testNumber(Condition cond, const Register &tag) { //?
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
 /*   Condition testGCThing(Condition cond, const BaseIndex &address) {//TBD BaseIndex special treat
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET));
        return cond == Equal ? AboveOrEqual : Below;
    }*/
    Condition testMagic(Condition cond, const Address &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }
    
/*    Condition testMagic(Condition cond, const BaseIndex &address) {//TBD BaseIndex special treat
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tagOf(address), ImmTag(JSVAL_TAG_MAGIC));
        return cond;
    }*/
        Condition testMagic(Condition cond, const Register &tag) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        cmpl(tag, ImmTag(JSVAL_TAG_MAGIC));
        return cond;
            }
           //NOTE*:this is new in ff24     
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
//NOTE*:Following is new in ff24
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

       //NOTE*:this is new in ff24   
        Condition testInt32(Condition cond, const Address &address) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        return testInt32(cond, Operand(address));
    }
       //NOTE*:this is new in ff24   
    Condition testDouble(Condition cond, const Operand &operand) {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        Condition actual = (cond == Equal) ? Below : AboveOrEqual;
        cmpl(ToType(operand), ImmTag(JSVAL_TAG_CLEAR));
        return actual;
    }
       //NOTE*:this is new in ff24   
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
//cmpPtr指令，仅完成了compare的指令的一部分，将需要比较的数移动至指定的两个寄存器中
    void cmpPtr(Register lhs, const ImmGCPtr rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const ImmWord rhs) {
        cmpl(lhs, rhs);
    }
    void cmpPtr(const Operand &lhs, const ImmGCPtr rhs) {
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
           //NOTE*:this is new in ff24   
        void cmpPtr(const Operand &lhs, const Imm32 rhs) {
        cmpl(lhs, rhs);
    }
        void cmpPtr(Register lhs, const ImmWord rhs) {
        cmpl(lhs, Imm32(rhs.value));
    }
    void testPtr(Register lhs, Register rhs) {
        testl(lhs, rhs);
    }

    Condition testNegativeZero(const FloatRegister &reg, const Register &scratch);

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
//生成两个操作数的add指令
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
       //NOTE*:this is new in ff24 
      void addPtr(const Address &src, const Register &dest) {
        addl(Operand(src), dest);
    }
    void subPtr(Imm32 imm, const Register &dest) {
        subl(imm, dest);
    }
           //NOTE*:this is new in ff24 
      void subPtr(const Register &src, const Register &dest) {
        subl(src, dest);
    }
           //NOTE*:this is new in ff24 
    void subPtr(const Address &addr, const Register &dest) {
        subl(Operand(addr), dest);
    }

//生成生成比较+跳转的指令
    template <typename T, typename S>
    void branchPtr(Condition cond, T lhs, S ptr, Label *label) {
        cmpl(Operand(lhs), ptr);
        j(cond, label);
    }

    template <typename T>
    void branchPrivatePtr(Condition cond, T lhs, ImmWord ptr, Label *label) {
        branchPtr(cond, lhs, ptr, label);
    }
 //NOTE*:this is new in ff24 
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
        testl(lhs, rhs);
        j(cond, label);
    }
     //NOTE*:this is new in ff24 
    void branchTestPtr(Condition cond, Register lhs, Imm32 imm, Label *label) {
        testl(lhs, imm);
        j(cond, label);
    }
     //NOTE*:this is new in ff24 
    void branchTestPtr(Condition cond, const Address &lhs, Imm32 imm, Label *label) {
        testl(Operand(lhs), imm);
        j(cond, label);
    }
    void decBranchPtr(Condition cond, const Register &lhs, Imm32 imm, Label *label) {
        subPtr(imm, lhs);
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
    void loadPtr(const BaseIndex &src, Register dest) {//TBD BaseIndex special treat
        movl(Operand(src), dest);
    }
    void loadPtr(const AbsoluteAddress &address, Register dest) {
        movl(Operand(address), dest);
    }
    ////NOTE*:this is new in ff24 
        void loadPtr(const Operand &src, Register dest) {
        movl(src, dest);
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
    void storePtr(Register src, const AbsoluteAddress &address) {
        movl(src, Operand(address));
    }
//NOTE*:this is new in ff24
    void storePtr(Register src, const Operand &dest) {
        movl(src, dest);
    }
    void setStackArg(const Register &reg, uint32_t arg) {
        movl(reg, Operand(sp, arg * STACK_SLOT_SIZE));
    }

    // Type testing instructions can take a tag in a register or a
    // ValueOperand.类型检测+跳转
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
    void branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label);

//NOTE*:this is new in ff24
 void branchTestValue(Condition cond, const Address &valaddr, const ValueOperand &value,
                         Label *label)
    {
        JS_ASSERT(cond == Equal || cond == NotEqual);
        branchPtr(cond, tagOf(valaddr), value.typeReg(), label);
        branchPtr(cond, payloadOf(valaddr), value.payloadReg(), label);
    }
//NOTE*:this is new in ff24
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

//不同数据类型的Box操作
    // Note: this function clobbers the source register.
    void boxDouble(const FloatRegister &src, const ValueOperand &dest) {//开箱Double类型，将其double类型的高32和低32位分别放入ValueOperand类型的tape域和payload域
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
    void unboxBoolean(const ValueOperand &src, const Register &dest) {
        movl(src.payloadReg(), dest);
    }
    void unboxBoolean(const Address &src, const Register &dest) {
        movl(payloadOf(src), dest);
    }
    void unboxDouble(const ValueOperand &src, const FloatRegister &dest) {
        JS_ASSERT(dest != ScratchFloatReg);
#if 0
//        if (0/*Assembler::HasSSE41()*/) {
//            movd(src.payloadReg(), dest);
//            pinsrd(src.typeReg(), dest);
//        } else {
//            movd(src.payloadReg(), dest);
//            movd(src.typeReg(), ScratchFloatReg);
//            unpcklps(ScratchFloatReg, dest);
//        }
#endif
        fastLoadDouble(src.payloadReg(), src.typeReg(), dest);
    }
    //该函数还未实现，没有使用到的地方
    void unboxDouble(const Operand &payload, const Operand &type,
                     const Register &scratch, const FloatRegister &dest) {
        JS_ASSERT(dest != ScratchFloatReg);
        JS_ASSERT(0);
//        if (0/*Assembler::HasSSE41()*/) {
//            movl(payload, scratch);
//            movd(scratch, dest);
//            movl(type, scratch);
//            pinsrd(scratch, dest);//合并数到寄存器dest中，
//        } else {
//            movl(payload, scratch);
//            movd(scratch, dest);
//            movl(type, scratch);
//            movd(scratch, ScratchFloatReg);
//            unpcklps(ScratchFloatReg, dest);//合并浮点数
//        }
//可用mips中的mtc1指令和mthc1指令来完成
    }
    
    //NOTE*:this is new in ff24
        void unboxDouble(const Address &src, const FloatRegister &dest) {
       movsd(Operand(src), dest);
    }
    void unboxValue(const ValueOperand &src, AnyRegister dest) {
        if (dest.isFloat()) {
            Label notInt32, end;
            branchTestInt32(Assembler::NotEqual, src, &notInt32);
        //NOTE*:update in ff24;      
        //    cvtsi2sd(Operand(src.payloadReg()), dest.fpu()); 
          cvtsi2sd(src.payloadReg(), dest.fpu());
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
    //NOTE*:this is new in ff24
    void notBoolean(const ValueOperand &val) {
        xorl(Imm32(1), val.payloadReg());
    }
    // Extended unboxing API. If the payload is already in a register, returns
    // that register. Otherwise, provides a move to the given scratch register,
    // and returns that.
  //提取payload域的值
    Register extractObject(const Address &address, Register scratch) {
        movl(payloadOf(address), scratch);
        return scratch;
    }
    Register extractObject(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
      //NOTE*:this is new in ff24
    Register extractInt32(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
      //NOTE*:this is new in ff24
    Register extractBoolean(const ValueOperand &value, Register scratch) {
        return value.payloadReg();
    }
    
    //提取tag域的值
    Register extractTag(const Address &address, Register scratch) {
        movl(tagOf(address), scratch);
        return scratch;
    }
    Register extractTag(const ValueOperand &value, Register scratch) {
        return value.typeReg();
    }
//不同类型间的转换
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
        Operand lengthAndFlags(string, JSString::offsetOfLengthAndFlags());//在vm/String.h中定义lengthAndFlags为size_t

        size_t mask = (0xFFFFFFFF << JSString::LENGTH_SHIFT);
        testl(lengthAndFlags, Imm32(mask));
        return truthy ? Assembler::NonZero : Assembler::Zero;
    }


    void loadInt32OrDouble(const Operand &operand, const FloatRegister &dest) {
        Label notInt32, end;
        branchTestInt32(Assembler::NotEqual, operand, &notInt32);
        cvtsi2sd(ToPayload(operand), dest);
        jump(&end);
        bind(&notInt32);
        movsd(operand, dest);
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
    //NOTE*:this is new in ff24
        void xorPtr(Imm32 imm, Register dest) {
        xorl(imm, dest);
    }
    //NOTE*:this is new in ff24
    void xorPtr(Register src, Register dest) {
        xorl(src, dest);
    }
    
    void orPtr(Imm32 imm, Register dest) {
        orl(imm, dest);
    }

    //NOTE*:this is new in ff24
    void orPtr(Register src, Register dest) {
        orl(src, dest);
    }
    //NOTE*:this is new in ff24
        void andPtr(Imm32 imm, Register dest) {
        andl(imm, dest);
    }
    //NOTE*:this is new in ff24
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
        Label noOverflow;
      //add by QuQiuwen
      //J指令会对两个比较寄存器中的数作比较，需要往比较寄存器中传递数据，条件为NonZero时，代码定义两个比较的数不相等则跳转
     cmpl(zero,Operand(dest));
        j(NonZero, &noOverflow);
        addl(Imm32(1), Operand(dest.offset(4)));
        bind(&noOverflow);
    }

    //NOTE*:this is new in ff24
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
    // takes. Note that this only supports cdecl.调用C++代码时，需要传递参数
    //
    // In order for alignment to work correctly, the MacroAssembler must have a
    // consistent view of the stack displacement. It is okay to call "push"
    // manually, however, if the stack alignment were to change, the macro
    // assembler should be notified before starting a call.为保持堆栈分布的一致性
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
  	  //NOTE*:this is new in ff24
    void callWithABIPre(uint32_t *stackAdjust);
      //NOTE*:this is new in ff24
    void callWithABIPost(uint32_t stackAdjust, Result result);
    
     public:
    // Emits a call to a C/C++ function, resolving all argument moves.
    void callWithABI(void *fun, Result result = GENERAL);
    
    //NOTE*:this is new in ff24
    void callWithABI(const Address &fun, Result result = GENERAL);
    // Used from within an Exit frame to handle a pending exception.

//this funtion is deleted in ff24;
/*  
    void handleException();
    */
    
    //NOTE*:this is new in ff24
        // Used from within an Exit frame to handle a pending exception.
    void handleFailureWithHandler(void *handler);

    void makeFrameDescriptor(Register frameSizeReg, FrameType type) {
        shll(Imm32(FRAMESIZE_SHIFT), frameSizeReg);//FRAMESIZE_SHIFT为4，对应MIPS::sll指令，将frameSizeReg的值左移4位
        orl(Imm32(type), frameSizeReg);//将对应的位置1
    }

    // Save an exit frame (which must be aligned to the stack pointer) to
    // ThreadData::ionTop.
    void linkExitFrame() {
        JSCompartment *compartment = GetIonContext()->compartment;
          //NOTE*:this is new in ff24
     //   movl(StackPointer, Operand(&compartment->rt->ionTop));
       movl(StackPointer, Operand(&compartment->rt->mainThread.ionTop));
    }

     //NOTE*:this is new in ff24
    // Save an exit frame to the thread data of the current thread, given a
    // register that holds a PerThreadData *.
    void linkParallelExitFrame(const Register &pt) {
        movl(StackPointer, Operand(pt, offsetof(PerThreadData, ionTop)));
    }
    
    void callWithExitFrame(IonCode *target, Register dynStack);

    void enterOsr(Register calleeToken, Register code);
       //NOTE*:this is new in ff24
        // See CodeGeneratorX86 calls to noteAsmJSGlobalAccess.
    void patchAsmJSGlobalAccess(unsigned offset, uint8_t *code, unsigned codeBytes,
                                unsigned globalDataOffset)
    {
        uint8_t *nextInsn = code + offset;
        JS_ASSERT(nextInsn <= code + codeBytes);
        uint8_t *target = code + codeBytes + globalDataOffset;
        ((int32_t *)nextInsn)[-1] = uintptr_t(target);
    }

  protected://x86
    // Bytes pushed onto the frame by the callee; includes frameDepth_. This is
    // needed to compute offsets to stack slots while temporary space has been
    // reserved for unexpected spills or C++ function calls. It is maintained
    // by functions which track stack alignment, which for clear distinction
    // use StudlyCaps (for example, Push, Pop).
    uint32_t framePushed_;

  public://x86
    MacroAssemblerMIPS()
      : inCall_(false),
        framePushed_(0),
        enoughMemory_(true)
    {
    }

    void compareDouble(DoubleCondition cond, const FloatRegister &lhs, const FloatRegister &rhs) {
        if (cond & DoubleConditionBitInvert)
            ucomisd(rhs, lhs);
        else
            ucomisd(lhs, rhs);
    }
    void branchDouble(DoubleCondition cond, const FloatRegister &lhs,
                      const FloatRegister &rhs, Label *label)
    {
        compareDouble(cond, lhs, rhs);

        if (cond == DoubleEqual) {
            Label unordered;
            j(Parity, &unordered);
            j(Equal, label);
            bind(&unordered);
            return;
        }
        if (cond == DoubleNotEqualOrUnordered) {
            j(NotEqual, label);
            j(Parity, label);
            return;
        }

        JS_ASSERT(!(cond & DoubleConditionBitSpecial));
        j(ConditionFromDoubleCondition(cond), label);
    }

    void move32(const Imm32 &imm, const Register &dest) {
        if (imm.value == 0)
            xorl(dest, dest);
        else
            movl(imm, dest);
    }
           //NOTE*:this is new in ff24
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
        testl(lhs, rhs);
    }
      //NOTE*:this is new in ff24
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
    void add32(Imm32 imm, Register dest) {
        addl(imm, dest);
    }
    void add32(Imm32 imm, const Address &dest) {
        addl(imm, Operand(dest));
    }
          //NOTE*:this is new in ff24
        void add32(Register src, Register dest) {
        addl(src, dest);
    }
    void sub32(Imm32 imm, Register dest) {
        subl(imm, dest);
    }
        //NOTE*:this is new in ff24
    void sub32(Register src, Register dest) {
        subl(src, dest);
    }
       //NOTE*:this is new in ff24
    void xor32(Imm32 imm, Register dest) {
        xorl(imm, dest);
    }

 //NOTE*:this is new in ff24
    void branch32(Condition cond, const Operand &lhs, const Register &rhs, Label *label) {
        cmpl(lhs, rhs);
        j(cond, label);
    }
 //NOTE*:this is new in ff24
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
    
       //NOTE*:this is new in ff24
        void branch32(Condition cond, const AbsoluteAddress &lhs, Imm32 rhs, Label *label) {
        cmpl(Operand(lhs), rhs);
        j(cond, label);
    }
   //NOTE*:this is new in ff24
    void branch32(Condition cond, const AbsoluteAddress &lhs, Register rhs, Label *label) {
        cmpl(Operand(lhs), rhs);
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
//update in ff24
 /*   void convertInt32ToDouble(const Register &src, const FloatRegister &dest) {
        cvtsi2sd(Operand(src), dest);
    }
    */
          //NOTE*:this is new in ff24
       void convertInt32ToDouble(const Register &src, const FloatRegister &dest) {
        cvtsi2sd(src, dest);
    }
          //NOTE*:this is new in ff24
    void convertInt32ToDouble(const Address &src, FloatRegister dest) {
        cvtsi2sd(Operand(src), dest);
    } 
    
    
    
   Condition testDoubleTruthy(bool truthy, const FloatRegister &reg) {
        xorpd(ScratchFloatReg, ScratchFloatReg);//将ScratchFloatReg置零；ScratchFloatReg在MIPS中定义为f2寄存器；
        ucomisd(ScratchFloatReg, reg);//将两个操作数移入两个特定的比较寄存器中；
        //return truthy ? Assembler::DoubleNotEqual : Assembler::DoubleEqual;
        return truthy ? NonZero : Zero;
    }
    void branchTruncateDouble(const FloatRegister &src, const Register &dest, Label *fail) {
        JS_STATIC_ASSERT(INT_MIN == int(0x80000000));
        cvttsd2si(src, dest);//将double转换为32位整型；
        cmpl(dest, Imm32(INT_MIN));//将这个32位数与0x80000000作比较；
        j(Assembler::Equal, fail);
    }
    void load8ZeroExtend(const Address &src, const Register &dest) {
        movzbl(Operand(src), dest);
    }
    void load8ZeroExtend(const BaseIndex &src, const Register &dest) {//TBD BaseIndex special treat
        movzbl(Operand(src), dest);
    }
    void load8SignExtend(const Address &src, const Register &dest) {
        movxbl(Operand(src), dest);
    }
    void load8SignExtend(const BaseIndex &src, const Register &dest) {//TBD BaseIndex special treat
        movxbl(Operand(src), dest);
    }
    template <typename S, typename T>
    void store8(const S &src, const T &dest) {
        movb(src, Operand(dest));
    }
    void load16ZeroExtend(const Address &src, const Register &dest) {
        movzwl(Operand(src), dest);
    }
    void load16ZeroExtend(const BaseIndex &src, const Register &dest) {//TBD BaseIndex special treat
        movzwl(Operand(src), dest);
    }
    template <typename S, typename T>
    void store16(const S &src, const T &dest) {
        movw(src, Operand(dest));
    }
    void load16SignExtend(const Address &src, const Register &dest) {
        movxwl(Operand(src), dest);
    }
    void load16SignExtend(const BaseIndex &src, const Register &dest) {//TBD BaseIndex special treat
        movxwl(Operand(src), dest);
    }
    void load32(const Address &address, Register dest) {
        movl(Operand(address), dest);
    }
    void load32(const BaseIndex &src, Register dest) {//TBD BaseIndex special treat
        movl(Operand(src), dest);
    }
    //NOTE*:this is new in ff24
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
    void loadDouble(const BaseIndex &src, FloatRegister dest) {//TBD BaseIndex special treat
        movsd(Operand(src), dest);
    }
    //NOTE*:this is new in ff24 
    void loadDouble(const Operand &src, FloatRegister dest) {
      movsd(src, dest);
    }
    void storeDouble(FloatRegister src, const Address &dest) {
        movsd(src, Operand(dest));
    }
    void storeDouble(FloatRegister src, const BaseIndex &dest) {//TBD BaseIndex special treat
        movsd(src, Operand(dest));
    }
    //NOTE*:this is new in ff24 
        void storeDouble(FloatRegister src, const Operand &dest) {
        movsd(src, dest);
    }
    void zeroDouble(FloatRegister reg) {
        zerod(reg);
    }
  //rename in ff24;
    /*  void negDouble(FloatRegister src, FloatRegister dest) {
        negd(src, dest);
    }*/
        //NOTE*:this is new in ff24 
     void negateDouble(FloatRegister reg) {
  /*      // From MacroAssemblerX86Shared::maybeInlineDouble
        pcmpeqw(ScratchFloatReg, ScratchFloatReg);
        psllq(Imm32(63), ScratchFloatReg);

        // XOR the float in a float register with -0.0.
        xorpd(ScratchFloatReg, reg); // s ^ 0x80000000000000*/
       ASSERT(0); 
       //  negd(reg, reg);
    }
    void addDouble(FloatRegister src, FloatRegister dest) {
        addsd(src, dest);
    }
    //NOTE*:this is new in ff24 
    void subDouble(FloatRegister src, FloatRegister dest) {
        subsd(src, dest);
    }
        //NOTE*:this is new in ff24 
    void mulDouble(FloatRegister src, FloatRegister dest) {
        mulsd(src, dest);
    }
        //NOTE*:this is new in ff24 
    void divDouble(FloatRegister src, FloatRegister dest) {
        divsd(src, dest);
    }
    void convertDoubleToFloat(const FloatRegister &src, const FloatRegister &dest) {
        cvtsd2ss(src, dest);
    }
    void loadFloatAsDouble(const Register &src, FloatRegister dest) {
        movd(src, dest);//将int32转换为double；
        cvtss2sd(dest, dest);//将单精度数转换为双精度数；
    }
    void loadFloatAsDouble(const Address &src, FloatRegister dest) {
        movss(Operand(src), dest);
        cvtss2sd(dest, dest);
    }
    void loadFloatAsDouble(const BaseIndex &src, FloatRegister dest) {//TBD BaseIndex special treat
        movss(Operand(src), dest);
        cvtss2sd(dest, dest);
    }
       //NOTE*:this is new in ff24 
     void loadFloatAsDouble(const Operand &src, FloatRegister dest) {
        movss(src, dest);
        cvtss2sd(dest, dest);
    }
    void storeFloat(FloatRegister src, const Address &dest) {
        movss(src, Operand(dest));
    }
    void storeFloat(FloatRegister src, const BaseIndex &dest) {//TBD BaseIndex special treat
        movss(src, Operand(dest));
    }
     //NOTE*:this is new in ff24 ; it's a  copy of x86;
     //此函数中关于比较的指令，可能需要重写；
    // Checks whether a double is representable as a 32-bit integer. If so, the
    // integer is written to the output register. Otherwise, a bailout is taken to
    // the given snapshot. This function overwrites the scratch float register.
    void convertDoubleToInt32(FloatRegister src, Register dest, Label *fail,
                              bool negativeZeroCheck = true)
    {
        cvttsd2si(src, dest);
        cvtsi2sd(dest, ScratchFloatReg);
        ucomisd(src, ScratchFloatReg);
        j(Assembler::Parity, fail);//目前mips还未定义关于PF的检测方案
        j(Assembler::NotEqual, fail);

        // Check for -0
        if (negativeZeroCheck) {
            Label notZero;
            testl(dest, dest);
            j(Assembler::NonZero, &notZero);

         //   if (Assembler::HasSSE41()) {
      //          ptest(src, src);
     //           j(Assembler::NonZero, fail);
      //      } else {
                // bit 0 = sign of low double
                // bit 1 = sign of high double
                movmskpd(src, dest);
                andl(Imm32(1), dest);
                //add by QuQiuwen
                cmpl(zero,dest);
                j(Assembler::NonZero, fail);
   //         }

            bind(&notZero);
        }
    }


//条件检测，注意！
    void clampIntToUint8(Register src, Register dest) {
        Label inRange, done;
        branchTest32(Assembler::Zero, src, Imm32(0xffffff00), &inRange);//高24位如果为零，则无需转换；
        {
            Label negative;
            branchTest32(Assembler::Signed, src, src, &negative);
            {
                movl(Imm32(255), dest);//dest置为0111 1111
                jump(&done);
            }
            bind(&negative);
            {
                xorl(dest, dest);//dest置零；
                jump(&done);
            }
        }
        bind(&inRange);
        if (src != dest)
            movl(src, dest);
        bind(&done);
    }
//this function is updete in ff24; it keeps the same as old one;
    bool maybeInlineDouble(uint64_t u, const FloatRegister &dest) {
        // This implements parts of "13.4 Generating constants" of 
        // "2. Optimizing subroutines in assembly language" by Agner Fog.
        switch (u) {
          case 0x0000000000000000ULL: // 0.0
            xorpd(dest, dest);//将dest置零；
            break;
          case 0x8000000000000000ULL: // -0.0
            pcmpeqw(dest, dest);//MIP中未实现；在x86中，pcmpeqw是将两个XMM寄存器作比较
            psllq(Imm32(63), dest);//MIP中未实现；
            break;
          case 0x3fe0000000000000ULL: // 0.5
            pcmpeqw(dest, dest);
            psllq(Imm32(55), dest);
            psrlq(Imm32(2), dest);
            break;
          case 0x3ff0000000000000ULL: // 1.0
            pcmpeqw(dest, dest);
            psllq(Imm32(54), dest);
            psrlq(Imm32(2), dest);
            break;
          case 0x3ff8000000000000ULL: // 1.5
            pcmpeqw(dest, dest);
            psllq(Imm32(53), dest);
            psrlq(Imm32(2), dest);
            break;
          case 0x4000000000000000ULL: // 2.0
            pcmpeqw(dest, dest);
            psllq(Imm32(63), dest);
            psrlq(Imm32(1), dest);
            break;
          case 0xc000000000000000ULL: // -2.0
            pcmpeqw(dest, dest);
            psllq(Imm32(62), dest);
            break;
          default:
            return false;
        }
        return true;
    }
    
//NOTE*:this is new in ff24 , it's a  copy of x86;
//此函数需要重定义；
    void emitSet(Assembler::Condition cond, const Register &dest,
              Assembler::NaNCond ifNaN = Assembler::NaN_HandledByCond) {
             if (GeneralRegisterSet(Registers::SingleByteRegs).has(dest)) {
            // If the register we're defining is a single byte register,
            // take advantage of the setCC instruction
            setCC(cond, dest);
         //   movzxbl(dest, dest);//mips中尚未定义；

            if (ifNaN != Assembler::NaN_HandledByCond) {
                Label noNaN;
          //      j(Assembler::NoParity, &noNaN);//NoParity的条件检测 MIPS未定义
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
            	ASSERT(0);
        //        j(Assembler::Parity, &ifFalse);//Parity的条件检测 MIPS未定义
            movl(Imm32(1), dest);
            j(cond, &end);
            if (ifNaN == Assembler::NaN_IsTrue)
            	ASSERT(0);
           //    j(Assembler::Parity, &end);
              
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

//NOTE*:this is update in ff24; it's a  copy of x86;
    // Builds an exit frame on the stack, with a return address to an internal
    // non-function. Returns offset to be passed to markSafepointAt().
    bool buildFakeExitFrame(const Register &scratch, uint32_t *offset) {
     /*   mozilla::DebugOnly<uint32_t> initialDepth = framePushed();//获取到当前栈已经使用的空间的大小；

        CodeLabel *cl = new CodeLabel();
        if (!addCodeLabel(cl))//将cl加入到codeLabels_链表中；codeLabels_链表用于存放链接后才能实施patch的label；
            return false;
        mov(cl->dest(), scratch);

        uint32 descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);//将frameSize左移4位后，与IonFrame_OptimizedJS做或操作；
        Push(Imm32(descriptor));
        Push(scratch);

        bind(cl->src());
        *offset = currentOffset();

        JS_ASSERT(framePushed() == initialDepth + IonExitFrameLayout::Size());
        return true;*/
        
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

    void callWithExitFrame(IonCode *target);

    void callIon(const Register &callee);

    void checkStackAlignment() {
        // Exists for ARM compatibility.
    }

    CodeOffsetLabel labelForPatch() {
        return CodeOffsetLabel(size());
    }
    //NOTE*:this is update in ff24;
        void abiret() {
        ret();
    }
    
};

typedef MacroAssemblerMIPS MacroAssemblerSpecific;

} // namespace ion
} // namespace js

#endif // jsion_macro_assembler_x86_h__


