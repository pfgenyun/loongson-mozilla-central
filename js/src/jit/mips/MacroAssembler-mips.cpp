/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/mips/MacroAssembler-mips.h"

#include "mozilla/Casting.h"

#include "jit/Bailouts.h"
#include "jit/BaselineFrame.h"
#include "jit/IonFrames.h"
#include "jit/MoveEmitter.h"

#include "jsscriptinlines.h"

using namespace js;
using namespace js::jit;

MacroAssemblerMIPS::Double *
MacroAssemblerMIPS::getDouble(double d)
{
    if (!doubleMap_.initialized()) {
        enoughMemory_ &= doubleMap_.init();
        if (!enoughMemory_)
            return nullptr;
    }
    size_t doubleIndex;
    DoubleMap::AddPtr p = doubleMap_.lookupForAdd(d);
    if (p) {
        doubleIndex = p->value();
    } else {
        doubleIndex = doubles_.length();
        enoughMemory_ &= doubles_.append(Double(d));
        enoughMemory_ &= doubleMap_.add(p, d, doubleIndex);
        if (!enoughMemory_)
            return nullptr;
    }
    Double &dbl = doubles_[doubleIndex];
    JS_ASSERT(!dbl.uses.bound());
    return &dbl;
}

void
MacroAssemblerMIPS::loadConstantDouble(double d, const FloatRegister &dest)
{
    if (maybeInlineDouble(d, dest))
        return;
    Double *dbl = getDouble(d);
    if (!dbl)
        return;
//    masm.movsd_mr(reinterpret_cast<const void *>(dbl->uses.prev()), dest.code());
    mcss.loadDouble(reinterpret_cast<const void *>(dbl->uses.prev()), dest.code());
    dbl->uses.setPrev(masm.size());
}

void
MacroAssemblerMIPS::addConstantDouble(double d, const FloatRegister &dest)
{
    Double *dbl = getDouble(d);
    if (!dbl)
        return;
//    masm.addsd_mr(reinterpret_cast<const void *>(dbl->uses.prev()), dest.code());  // need to modify . by wangqing
    mcss.loadDouble(reinterpret_cast<const void *>(dbl->uses.prev()), dest.code()); 
    dbl->uses.setPrev(masm.size());
}

MacroAssemblerMIPS::Float *
MacroAssemblerMIPS::getFloat(float f)
{
    if (!floatMap_.initialized()) {
        enoughMemory_ &= floatMap_.init();
        if (!enoughMemory_)
            return nullptr;
    }
    size_t floatIndex;
    FloatMap::AddPtr p = floatMap_.lookupForAdd(f);
    if (p) {
        floatIndex = p->value();
    } else {
        floatIndex = floats_.length();
        enoughMemory_ &= floats_.append(Float(f));
        enoughMemory_ &= floatMap_.add(p, f, floatIndex);
        if (!enoughMemory_)
            return nullptr;
    }
    Float &flt = floats_[floatIndex];
    JS_ASSERT(!flt.uses.bound());
    return &flt;
}

void
MacroAssemblerMIPS::loadConstantFloat32(float f, const FloatRegister &dest)
{
    if (maybeInlineFloat(f, dest))
        return;
    Float *flt = getFloat(f);
    if (!flt)
        return;
//    masm.movss_mr(reinterpret_cast<const void *>(flt->uses.prev()), dest.code());
   mcss.loadFloat(reinterpret_cast<void *>(flt->uses.prev()), dest.code());
    flt->uses.setPrev(masm.size());
}

void
MacroAssemblerMIPS::addConstantFloat32(float f, const FloatRegister &dest)
{
    Float *flt = getFloat(f);
    if (!flt)
        return;
//    masm.addss_mr(reinterpret_cast<const void *>(flt->uses.prev()), dest.code()); // need to modify. by wangqing
    mcss.loadFloat(reinterpret_cast<const void *>(flt->uses.prev()), dest.code()); 
    flt->uses.setPrev(masm.size());
}

void
MacroAssemblerMIPS::finish()
{
    if (doubles_.empty() && floats_.empty())
        return;

    masm.align(sizeof(double));
    for (size_t i = 0; i < doubles_.length(); i++) {
        CodeLabel cl(doubles_[i].uses);
        writeDoubleConstant(doubles_[i].value, cl.src());
        enoughMemory_ &= addCodeLabel(cl);
        if (!enoughMemory_)
            return;
    }
    for (size_t i = 0; i < floats_.length(); i++) {
        CodeLabel cl(floats_[i].uses);
        writeFloatConstant(floats_[i].value, cl.src());
        enoughMemory_ &= addCodeLabel(cl);
        if (!enoughMemory_)
            return;
    }
}

void
MacroAssemblerMIPS::setupABICall(uint32_t args)
{
    JS_ASSERT(!inCall_);
    inCall_ = true;

    args_ = args;
    passedArgs_ = 0;
    stackForCall_ = 16; // fix me: by Quqiuwen
}

void
MacroAssemblerMIPS::setupAlignedABICall(uint32_t args)
{
    setupABICall(args);
    dynamicAlignment_ = false;
}

void
MacroAssemblerMIPS::setupUnalignedABICall(uint32_t args, const Register &scratch)
{
    setupABICall(args);
    dynamicAlignment_ = true;

    movl(sp, scratch);
    andl(Imm32(~(StackAlignment - 1)), sp);
    push(scratch);
}

void
MacroAssemblerMIPS::passABIArg(const MoveOperand &from)
{
    MoveOperand to;

    ++passedArgs_;

    if (passedArgs_ <= 4) {
        Register destReg;
        FloatRegister destFloatReg;
    
        if (from.isDouble() && GetArgFloatReg(passedArgs_, &destFloatReg)) {
            to = MoveOperand(destFloatReg);
            enoughMemory_ &= moveResolver_.addMove(from, to, MoveOp::DOUBLE);
        }else {
            GetArgReg(passedArgs_, &destReg); 
            to = MoveOperand(destReg);
            enoughMemory_ &= moveResolver_.addMove(from, to, MoveOp::GENERAL);
        }
    } else {
#if 1
        to = MoveOperand(StackPointer, stackForCall_);
        if (from.isDouble()) {
            stackForCall_ += sizeof(double);
            enoughMemory_ &= moveResolver_.addMove(from, to, MoveOp::DOUBLE);
        } else {
            stackForCall_ += sizeof(int32_t);
            enoughMemory_ &= moveResolver_.addMove(from, to, MoveOp::GENERAL);
        }
#endif
    }
}

void
MacroAssemblerMIPS::passABIArg(const Register &reg)
{
    passABIArg(MoveOperand(reg));
}

void
MacroAssemblerMIPS::passABIArg(const FloatRegister &reg)
{
    passABIArg(MoveOperand(reg));
}

void
MacroAssemblerMIPS::callWithABIPre(uint32_t *stackAdjust)
{
    JS_ASSERT(inCall_);
    JS_ASSERT(args_ == passedArgs_);

    if (dynamicAlignment_) {
        *stackAdjust = stackForCall_
                     + ComputeByteAlignment(stackForCall_ + STACK_SLOT_SIZE,
                                            StackAlignment);
    } else {
        *stackAdjust = stackForCall_
                     + ComputeByteAlignment(stackForCall_ + framePushed_,
                                            StackAlignment);
    }

    reserveStack(*stackAdjust);

    // Position all arguments.
    {
        enoughMemory_ &= moveResolver_.resolve();
        if (!enoughMemory_)
            return;

        MoveEmitter emitter(*this);
        emitter.emit(moveResolver_);
        emitter.finish();
    }

#ifdef DEBUG
    {
        // Check call alignment.
        Label good;
        //TODO
        testl(sp, Imm32(StackAlignment - 1));
        j(Equal, &good);
        breakpoint();
        bind(&good);
    }
#endif
}

void
MacroAssemblerMIPS::callWithABIPost(uint32_t stackAdjust, Result result)
{
 /*   freeStack(stackAdjust);
    if (result == DOUBLE) {
        reserveStack(sizeof(double));
        fstp(Operand(esp, 0));
        movsd(Operand(esp, 0), ReturnFloatReg);
        freeStack(sizeof(double));
    }
    if (dynamicAlignment_)
        pop(esp);

    JS_ASSERT(inCall_);
    inCall_ = false;*/
}
void
MacroAssemblerMIPS::callWithABI(void *fun, Result result)
{
    JS_ASSERT(inCall_);
    JS_ASSERT(args_ == passedArgs_);

    uint32_t stackAdjust = ((passedArgs_ > 4) ? passedArgs_ : 4) * STACK_SLOT_SIZE;
    if (dynamicAlignment_) {
#if 0
        stackAdjust = stackForCall_
                    + ComputeByteAlignment(stackForCall_,
                                           StackAlignment);
#endif
        stackAdjust += ComputeByteAlignment(stackAdjust + STACK_SLOT_SIZE, StackAlignment);
    } else {
        stackAdjust +=
            ComputeByteAlignment(framePushed_ + stackAdjust, StackAlignment);
    }

    reserveStack(stackAdjust);
//    subl(Imm32(16), StackPointer);

    // Position all arguments.
    {
        enoughMemory_ &= moveResolver_.resolve();
        if (!enoughMemory_)
            return;

        MoveEmitter emitter(*this);
        emitter.emit(moveResolver_);
        emitter.finish();
    }

#ifdef DEBUG
    {
        // Check call alignment.
        Label good;
        movl(sp, t0);
        testl(t0, Imm32(StackAlignment - 1));
        j(Equal, &good);
        breakpoint();
        bind(&good);
    }
#endif

//ok    //ma_call
    call(ImmPtr(fun));

//    addl(Imm32(16), StackPointer);
    freeStack(stackAdjust);
    if (result == DOUBLE) {
        reserveStack(sizeof(double));//申请空间
        fstp(Operand(sp, 0));//，
        movsd(Operand(sp, 0), ReturnFloatReg);//sp指向的值加载至ReturnFloatReg；
        freeStack(sizeof(double));
    }
    if (dynamicAlignment_)
        //pop(sp);
        movl(Operand(sp, 0), sp);

    JS_ASSERT(inCall_);
    inCall_ = false;
}

// New function
void
MacroAssemblerMIPS::callWithABI(AsmJSImmPtr fun, Result result)
{
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(fun);
    callWithABIPost(stackAdjust, result);
}

// New function
void
MacroAssemblerMIPS::callWithABI(const Address &fun, Result result)
{
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(Operand(fun));
    callWithABIPost(stackAdjust, result);
}

void
MacroAssemblerMIPS::handleFailureWithHandler(void *handler)
{
    // Reserve space for exception information.
    subl(Imm32(sizeof(ResumeFromException)), sp);
    movl(sp, a0);

    // Ask for an exception handler.
    setupUnalignedABICall(1, v0);
    passABIArg(a0);
    callWithABI(handler);

    IonCode *excTail = GetIonContext()->runtime->jitRuntime()->getExceptionTail();
    jmp(excTail);
}

void
MacroAssemblerMIPS::handleFailureWithHandlerTail()
{
/*
    Label entryFrame;
    Label catch_;
    Label finally;
    Label return_;
    Label bailout;

    loadPtr(Address(esp, offsetof(ResumeFromException, kind)), eax);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_ENTRY_FRAME), &entryFrame);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_CATCH), &catch_);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_FINALLY), &finally);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_FORCED_RETURN), &return_);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_BAILOUT), &bailout);

    breakpoint(); // Invalid kind.

    // No exception handler. Load the error value, load the new stack pointer
    // and return from the entry frame.
    bind(&entryFrame);
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    loadPtr(Address(esp, offsetof(ResumeFromException, stackPointer)), esp);
    ret();

    // If we found a catch handler, this must be a baseline frame. Restore state
    // and jump to the catch block.
    bind(&catch_);
    loadPtr(Address(esp, offsetof(ResumeFromException, target)), eax);
    loadPtr(Address(esp, offsetof(ResumeFromException, framePointer)), ebp);
    loadPtr(Address(esp, offsetof(ResumeFromException, stackPointer)), esp);
    jmp(Operand(eax));

    // If we found a finally block, this must be a baseline frame. Push
    // two values expected by JSOP_RETSUB: BooleanValue(true) and the
    // exception.
    bind(&finally);
    ValueOperand exception = ValueOperand(ecx, edx);
    loadValue(Address(esp, offsetof(ResumeFromException, exception)), exception);

    loadPtr(Address(esp, offsetof(ResumeFromException, target)), eax);
    loadPtr(Address(esp, offsetof(ResumeFromException, framePointer)), ebp);
    loadPtr(Address(esp, offsetof(ResumeFromException, stackPointer)), esp);

    pushValue(BooleanValue(true));
    pushValue(exception);
    jmp(Operand(eax));

    // Only used in debug mode. Return BaselineFrame->returnValue() to the caller.
    bind(&return_);
    loadPtr(Address(esp, offsetof(ResumeFromException, framePointer)), ebp);
    loadPtr(Address(esp, offsetof(ResumeFromException, stackPointer)), esp);
    loadValue(Address(ebp, BaselineFrame::reverseOffsetOfReturnValue()), JSReturnOperand);
    movl(ebp, esp);
    pop(ebp);
    ret();

    // If we are bailing out to baseline to handle an exception, jump to
    // the bailout tail stub.
    bind(&bailout);
    loadPtr(Address(esp, offsetof(ResumeFromException, bailoutInfo)), ecx);
    movl(Imm32(BAILOUT_RETURN_OK), eax);
    jmp(Operand(esp, offsetof(ResumeFromException, target)));
*/
}

void
MacroAssemblerMIPS::branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label)
{
    jsval_layout jv = JSVAL_TO_IMPL(v);
    if (v.isMarkable())
        cmpl(value.payloadReg(), ImmGCPtr(reinterpret_cast<gc::Cell *>(v.toGCThing())));
    else
        cmpl(value.payloadReg(), Imm32(jv.s.payload.i32));

    if (cond == Equal) {
        Label done;
        j(NotEqual, &done);
        {
            cmpl(value.typeReg(), Imm32(jv.s.tag));
            j(Equal, label);
        }
        bind(&done);
    } else {
        JS_ASSERT(cond == NotEqual);
        j(NotEqual, label);

        cmpl(value.typeReg(), Imm32(jv.s.tag));
        j(NotEqual, label);
    }
}

Assembler::Condition
MacroAssemblerMIPS::testNegativeZero(const FloatRegister &reg, const Register &scratch)
{
    // Determines whether the single double contained in the XMM register reg
    // is equal to double-precision -0.

    Label nonZero;

    // Compare to zero. Lets through {0, -0}.
    xorpd(ScratchFloatReg, ScratchFloatReg);
    // If reg is non-zero, then a test of Zero is false.
    branchDouble(DoubleNotEqual, reg, ScratchFloatReg, &nonZero);

    // Input register is either zero or negative zero. Test sign bit.
    // by wangqing
    mfc1(scratch, js::jit::FloatRegister::FromCode(reg.code() + 1));
    shrl(Imm32(0x1f), scratch);
    // If reg is -0, then a test of Zero is true.
    cmpl(scratch, Imm32(1));

    bind(&nonZero);
    return Zero;
}

Assembler::Condition
MacroAssemblerMIPS::testNegativeZeroFloat32(const FloatRegister &reg, const Register &scratch)
{
    movd(reg, scratch);
    cmpl(scratch, Imm32(1));
    return Overflow;
}

void 
MacroAssemblerMIPS::callIon(const Register &callee) {
/*arm :
    JS_ASSERT((framePushed() & 3) == 0);
    if ((framePushed() & 7) == 4) {
        ma_callIonHalfPush(callee);
    } else {
        adjustFrame(sizeof(void*));
        ma_callIon(callee);
    }
*/
//ok    call(callee);
    ma_callIonHalfPush(callee);//ok   //获取到当前的pc+7的值，存放至v0后压栈，然后成跳转至callee的跳转指令；
#if 0 //try above line
    JS_ASSERT((framePushed() & 3) == 0);
    if ((framePushed() & 7) == 4) {
        ma_callIonHalfPush(callee);//ok
    } else {
        //adjustFrame(sizeof(void*));
        setFramePushed(framePushed_ + sizeof(void*));
        ma_callIon(callee);//ok
    }
#endif
}


void 
MacroAssemblerMIPS::enterOsr(Register calleeToken, Register code) {
    push(Imm32(0)); // num actual args.
    push(calleeToken);
    push(Imm32(MakeFrameDescriptor(0, IonFrame_Osr)));
//ok    //arm : ma_callIonHalfPush
//ok    call(code);
    ma_callIonHalfPush(code);//获取到当前的pc+7，然后压栈，跳转至code处；
#if ! defined (JS_CPU_MIPS)
    addl(Imm32(sizeof(uintptr_t) * 2), sp);
#endif
}
