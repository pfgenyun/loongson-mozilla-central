/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "MacroAssembler-mips.h"
#include "jit/MoveEmitter.h"
#include "jit/IonFrames.h"

#include "jsscriptinlines.h"

using namespace js;
using namespace js::jit;

//NOTE*:this function is a copy of x86 !
void
MacroAssemblerMIPS::loadConstantDouble(double d, const FloatRegister &dest)
{
    union DoublePun {
        uint64_t u;
        double d;
    } dpun;
    dpun.d = d;
    if (maybeInlineDouble(dpun.u, dest))
        return;

    if (!doubleMap_.initialized()) {
        enoughMemory_ &= doubleMap_.init();
        if (!enoughMemory_)
            return;
    }
    size_t doubleIndex;
    DoubleMap::AddPtr p = doubleMap_.lookupForAdd(d);
    if (p) {
        doubleIndex = p->value;
    } else {
        doubleIndex = doubles_.length();
        enoughMemory_ &= doubles_.append(Double(d));
        enoughMemory_ &= doubleMap_.add(p, d, doubleIndex);
        if (!enoughMemory_)
            return;
    }
    Double &dbl = doubles_[doubleIndex]; 
 //  masm.movsd_mr(reinterpret_cast<void *>(dbl.uses.prev()), dest.code());
   mcss.loadDouble(reinterpret_cast<void *>(dbl.uses.prev()), dest.code());
 
    dbl.uses.setPrev(masm.size());
}



//NOTE*:this function is a copy of x86 !
void
MacroAssemblerMIPS::finish()
{
    if (doubles_.empty())
        return;

    masm.align(sizeof(double));
    for (size_t i = 0; i < doubles_.length(); i++) {
        CodeLabel cl(doubles_[i].uses);
        writeDoubleConstant(doubles_[i].value, cl.src());
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
    stackForCall_ = 16;
//    subl(Imm32(16), sp);
}

void
MacroAssemblerMIPS::setupAlignedABICall(uint32_t args)
{
    setupABICall(args);
    dynamicAlignment_ = false;
}

void
MacroAssemblerMIPS::setupUnalignedABICall(uint32_t args, const Register &scratch)//与X86不同，MIPS中使用的堆栈指针为sp
{
    setupABICall(args);
    dynamicAlignment_ = true;

    movl(sp, scratch);
    andl(Imm32(~(StackAlignment - 1)), sp);//在MIPS中StackAlignment=16，将sp的后四位全部设为0
    push(scratch);
}

void
MacroAssemblerMIPS::passABIArg(const MoveOperand &from) //传递一个参数
{
    MoveOperand to;

    ++passedArgs_; //参数个数加1

    if(passedArgs_ <= 4){ //参数小于4个时，仅用寄存器便可以完成参数传递
        Register destReg;
        FloatRegister destFloatReg;
    
        if (from.isDouble() && GetArgFloatReg(passedArgs_, &destFloatReg)) {
            to = MoveOperand(destFloatReg);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
        }else {
            GetArgReg(passedArgs_, &destReg); 
            to = MoveOperand(destReg);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
        }
    }else{//参数大于四个时，需要通过堆栈来存放传递参数
#if 1
        to = MoveOperand(StackPointer, stackForCall_);
        if (from.isDouble()) {
            stackForCall_ += sizeof(double);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
        } else {
            stackForCall_ += sizeof(int32_t);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
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
MacroAssemblerMIPS::callWithABI(void *fun, Result result)
{
    JS_ASSERT(inCall_);
    JS_ASSERT(args_ == passedArgs_);//检测所有的参数是否都传递进来

    uint32_t stackAdjust = ((passedArgs_ > 4) ? passedArgs_ : 4) * STACK_SLOT_SIZE;//为参数预留栈空间
    if (dynamicAlignment_) {//非对齐ABI调用
#if 0
        stackAdjust = stackForCall_
                    + ComputeByteAlignment(stackForCall_,
                                           StackAlignment);
#else
        stackAdjust += ComputeByteAlignment(stackAdjust + STACK_SLOT_SIZE, StackAlignment);
#endif
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

        MoveEmitter emitter(*this);// ？
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
    call(ImmWord(fun));

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
  //NOTE*:this is new in ff24
void
MacroAssemblerMIPS::callWithABI(const Address &fun, Result result)
{
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(Operand(fun));
    callWithABIPost(stackAdjust, result);
}

  //NOTE*:this is new in ff24
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
        testl(esp, Imm32(StackAlignment - 1));
        j(Equal, &good);
        breakpoint();
        bind(&good);
    }
#endif
}
  //NOTE*:this is new in ff24
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
/*
void
MacroAssemblerMIPS::handleException()
{
    // Reserve space for exception information.
    subl(Imm32(sizeof(ResumeFromException)), sp);
    movl(sp, a0);

    // Ask for an exception handler.
    setupUnalignedABICall(1, v0);//传递一个参数，将sp的值放至v0中；
    passABIArg(a0);
    callWithABI(JS_FUNC_TO_DATA_PTR(void *, ion::HandleException));
    
    // Load the error value, load the new stack pointer, and return.
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    movl(Operand(sp, offsetof(ResumeFromException, stackPointer)), sp);
    ret();
}
*/
  //NOTE*:this is new in ff24, this  is copy of function handleException(), need to review;
void
MacroAssemblerMIPS::handleFailureWithHandler(void *handler)
{
	/*
    // Reserve space for exception information.
    subl(Imm32(sizeof(ResumeFromException)), esp);
    movl(esp, eax);

    // Ask for an exception handler.
    setupUnalignedABICall(1, ecx);
    passABIArg(eax);
    callWithABI(handler);

    Label entryFrame;
    Label catch_;
    Label finally;
    Label return_;

    loadPtr(Address(esp, offsetof(ResumeFromException, kind)), eax);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_ENTRY_FRAME), &entryFrame);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_CATCH), &catch_);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_FINALLY), &finally);
    branch32(Assembler::Equal, eax, Imm32(ResumeFromException::RESUME_FORCED_RETURN), &return_);

    breakpoint(); // Invalid kind.

    // No exception handler. Load the error value, load the new stack pointer
    // and return from the entry frame.
    bind(&entryFrame);
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    movl(Operand(esp, offsetof(ResumeFromException, stackPointer)), esp);
    ret();

    // If we found a catch handler, this must be a baseline frame. Restore state
    // and jump to the catch block.
    bind(&catch_);
    movl(Operand(esp, offsetof(ResumeFromException, target)), eax);
    movl(Operand(esp, offsetof(ResumeFromException, framePointer)), ebp);
    movl(Operand(esp, offsetof(ResumeFromException, stackPointer)), esp);
    jmp(Operand(eax));

    // If we found a finally block, this must be a baseline frame. Push
    // two values expected by JSOP_RETSUB: BooleanValue(true) and the
    // exception.
    bind(&finally);
    ValueOperand exception = ValueOperand(ecx, edx);
    loadValue(Operand(esp, offsetof(ResumeFromException, exception)), exception);

    movl(Operand(esp, offsetof(ResumeFromException, target)), eax);
    movl(Operand(esp, offsetof(ResumeFromException, framePointer)), ebp);
    movl(Operand(esp, offsetof(ResumeFromException, stackPointer)), esp);

    pushValue(BooleanValue(true));
    pushValue(exception);
    jmp(Operand(eax));

    // Only used in debug mode. Return BaselineFrame->returnValue() to the caller.
    bind(&return_);
    movl(Operand(esp, offsetof(ResumeFromException, framePointer)), ebp);
    movl(Operand(esp, offsetof(ResumeFromException, stackPointer)), esp);
    loadValue(Address(ebp, BaselineFrame::reverseOffsetOfReturnValue()), JSReturnOperand);
    movl(ebp, esp);
    pop(ebp);
    ret();
    */
    // Reserve space for exception information.
    subl(Imm32(sizeof(ResumeFromException)), sp);
    movl(sp, a0);

    // Ask for an exception handler.
    setupUnalignedABICall(1, v0);//传递一个参数，将sp的值放至v0中；
    passABIArg(a0);
    callWithABI(JS_FUNC_TO_DATA_PTR(void *, jit::HandleException));
    
    // Load the error value, load the new stack pointer, and return.
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    movl(Operand(sp, offsetof(ResumeFromException, stackPointer)), sp);
    ret();
    
    
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
    movmskpd(reg, scratch);
    // If reg is -0, then a test of Zero is true.
    cmpl(scratch, Imm32(1));

    bind(&nonZero);
    return Zero;
}

void 
MacroAssemblerMIPS::callWithExitFrame(IonCode *target, Register dynStack) {
    addPtr(Imm32(framePushed()), dynStack);//dynStack+当前已经使用堆栈的大小
    makeFrameDescriptor(dynStack, IonFrame_OptimizedJS);//对dynStack左移4位，并根据IonFrame_OptimizedJS的值将dynStack的某些位置1；
    Push(dynStack);//
//ok    //arm : ma_callIonHalfPush
    call(target);
}

void 
MacroAssemblerMIPS::callWithExitFrame(IonCode *target) {
    uint32_t descriptor = MakeFrameDescriptor(framePushed(), IonFrame_OptimizedJS);
//cause failure when descriptor==0x4e0
    Push(Imm32(descriptor));
//ok    //arm : ma_callIonHalfPush
    call(target);
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
