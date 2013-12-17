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

void
MacroAssemblerMIPS::loadConstantDouble(double d, const FloatRegister &dest)
{
    union DoublePun {
        uint64_t u;
        double d;
    } dpun;
    dpun.d = d;

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
    //dbl.uses.setPrev(masm.size());
	//hwj date:1106
	mov(&(dbl.uses), addrTempRegister);
	ldc1(dest,addrTempRegister,ImmWord((void*)0));
}

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
    int i = 0;
    JS_ASSERT(!inCall_);
    inCall_ = true;

    args_ = args;
    passedArgs_ = 0;

    //by weizhenwei, 2013.11.08, ugly
    passedArgsfake_ = 0;
    for (i = 0; i < 4; i++) {
        passedArgsBits_[i] = 0;
    }

    stackForCall_ = 16;
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

//by weizhenwei, 2013.11.08
void MacroAssemblerMIPS::passABIArg(const MoveOperand &from)
{
    MoveOperand to;

    ++passedArgs_; 
    ++passedArgsfake_;

    if(passedArgsfake_ <= 4) {
        Register destReg;
        FloatRegister destFloatReg;

        switch (passedArgs_) {
            case 1:
                if (from.isDouble() ) {//first is Double, store in f12
                    passedArgsBits_[0] = 1;
                    if (GetArgFloatReg(passedArgsfake_, &destFloatReg)) {
                        to = MoveOperand(destFloatReg);
                        enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
                    }
                    passedArgsfake_++; //for align problem.
                } else { //first is int, store in $4.
                    passedArgsBits_[0] = 2;
                    GetArgReg(passedArgsfake_, &destReg); 
                    to = MoveOperand(destReg);
                    enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
                }
                break;
            case 2:
                if (from.isDouble()) { //second is double;
                    passedArgsfake_++; //for align problem.
                    passedArgsBits_[1] = 1;
                    if (passedArgsBits_[0] == 1) { //first is double, second double store f14;
                        if (GetArgFloatReg(passedArgsfake_, &destFloatReg)) {
                            to = MoveOperand(destFloatReg);
                            enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
                        }
                    } else if (passedArgsBits_[0] == 2) { //first is int, second double store ($6, $7)
                        //TODO
                        JS_ASSERT(from.isFloatReg());
                        FloatRegister temp1 = from.floatReg();
                        FloatRegister temp2 = js::jit::FloatRegister::FromCode(temp1.code() + 1);
                        MoveOperand from1 = MoveOperand(temp1);
                        MoveOperand from2 = MoveOperand(temp2);

                        GetArgReg(passedArgsfake_, &destReg); 
                        to = MoveOperand(destReg);
                        enoughMemory_ &= moveResolver_.addMove(from1, to, Move::GENERAL);

                        passedArgsfake_++; //for align reason.
                        GetArgReg(passedArgsfake_, &destReg); 
                        to = MoveOperand(destReg);
                        enoughMemory_ &= moveResolver_.addMove(from2, to, Move::GENERAL);
                    } else { //impossible here
                        JS_ASSERT(0);
                    }
                } else { // second is int, store in $5 or $6, passedArgsfake_ indicates.
                    passedArgsBits_[1] = 2;
                    GetArgReg(passedArgsfake_, &destReg); 
                    to = MoveOperand(destReg);
                    enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
                }
                break;
            case 3:
                if (from.isDouble()) { //third is double;
                    passedArgsBits_[2] = 1;
                    passedArgsfake_++; //for align reason
                    if ( passedArgsBits_[0] == 1) { //first is double, third double must in stack for align reason.
                        //third is stack
                        to = MoveOperand(StackPointer, stackForCall_);
                        stackForCall_ += sizeof(double);
                        enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
                    } else if ( passedArgsBits_[0] == 2) { //first is int, 
                        if ( passedArgsBits_[1] == 1) { //second is double, 
                            //third double is on stack.
                            to = MoveOperand(StackPointer, stackForCall_);
                            stackForCall_ += sizeof(double);
                            enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
                        } else if ( passedArgsBits_[1] == 2) { //second is int, 
                            //third is on ($6, $7)
                            JS_ASSERT(from.isFloatReg());
                            FloatRegister temp1 = from.floatReg();
                            FloatRegister temp2 = js::jit::FloatRegister::FromCode(temp1.code() + 1);
                            MoveOperand from1 = MoveOperand(temp1);
                            MoveOperand from2 = MoveOperand(temp2);

                            GetArgReg(passedArgsfake_, &destReg); 
                            to = MoveOperand(destReg);
                            enoughMemory_ &= moveResolver_.addMove(from1, to, Move::GENERAL);

                            passedArgsfake_++; //for align reason.
                            GetArgReg(passedArgsfake_, &destReg); 
                            to = MoveOperand(destReg);
                            enoughMemory_ &= moveResolver_.addMove(from2, to, Move::GENERAL);
                        }
                    } else {
                        JS_ASSERT(0);
                    }
                } else { // third is int
                    passedArgsBits_[2] = 2;
                    if (passedArgsBits_[0] == 1) { //first is double, 
                        if (passedArgsBits_[1] == 1) { //second is double, passedArgsfake_==5 then, impossible here
                            JS_ASSERT(0);
                        } else if ( passedArgsBits_[1] == 2) { //second is int, 
                            //thired store on $7
                            GetArgReg(passedArgsfake_, &destReg); 
                            to = MoveOperand(destReg);
                            enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
                        } else {
                            JS_ASSERT(0);
                        }
                    } else if (passedArgsBits_[0] == 2) { //first is int
                        if (passedArgsBits_[1] == 1) { //second is double, passedArgsfake_ == 5 then, impossible here.
                            JS_ASSERT(0);
                        } else if ( passedArgsBits_[1] == 2) { //second is int, 
                            //thired store on $6
                            GetArgReg(passedArgsfake_, &destReg);
                            to = MoveOperand(destReg);
                            enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
                        } else {
                            JS_ASSERT(0);
                        }
                    } else { //impossible here
                        JS_ASSERT(0);
                    }
                }
                break;
            case 4:
                if (from.isDouble()) { //forth is double, store on stack
                    passedArgsBits_[3] = 1;
                    to = MoveOperand(StackPointer, stackForCall_);
                    stackForCall_ += sizeof(double);
                    enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
                    passedArgsfake_++; //for align reason.
                } else { // forth is int, store on $7
                    passedArgsBits_[3] = 2;
                    GetArgReg(passedArgsfake_, &destReg); 
                    to = MoveOperand(destReg);
                    enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
                }
                break;
            default:
                JS_ASSERT(0);
        }
    } else { //args > 4, store on stack.
        to = MoveOperand(StackPointer, stackForCall_);
        if (from.isDouble()) {
            stackForCall_ += sizeof(double);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::DOUBLE);
        } else {
            stackForCall_ += sizeof(int32_t);
            enoughMemory_ &= moveResolver_.addMove(from, to, Move::GENERAL);
        }
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
    freeStack(stackAdjust);
    if (result == DOUBLE) {
    //xsb:the result has been in f0,due to O32 ABI
	//and we set ReturnFloatReg=f0
	//so we don't need to move result from f0 to ReturnFloatReg
	}
    if (dynamicAlignment_)
        pop(sp);

    JS_ASSERT(inCall_);
    inCall_ = false;
}

void
MacroAssemblerMIPS::callWithABI(void *fun, Result result)
{
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    ma_call(ImmWord(fun));//1031
	callWithABIPost(stackAdjust, result);
}
void
MacroAssemblerMIPS::callWithABI(const Address &fun, Result result)
{
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    ma_call(Operand(fun));//1031
    callWithABIPost(stackAdjust, result);
}

void
MacroAssemblerMIPS::handleFailureWithHandler(void *handler)
{
    // Reserve space for exception information.
    subl(Imm32(sizeof(ResumeFromException)), sp);
    movl(sp, t6);

    // Ask for an exception handler.
    setupUnalignedABICall(1, t8);
    passABIArg(t6);
    callWithABI(handler);

    Label entryFrame;
    Label catch_;
    Label finally;
    Label return_;
    loadPtr(Address(sp, offsetof(ResumeFromException, kind)), t6);
	// Modify branch32, by wangqing, 2013-11-29
	movl(Imm32(ResumeFromException::RESUME_ENTRY_FRAME), cmpTempRegister);
	beq(t6, cmpTempRegister, &entryFrame);
	nop();

	// Modify branch32, by wangqing, 2013-11-29
	movl(Imm32(ResumeFromException::RESUME_CATCH), cmpTempRegister);
	beq(t6, cmpTempRegister, &catch_);
	nop();

	// Modify branch32, by wangqing, 2013-11-29
	movl(Imm32(ResumeFromException::RESUME_FINALLY), cmpTempRegister);
	beq(t6, cmpTempRegister, &finally);
	nop();

	// Modify branch32, by wangqing, 2013-11-29
	movl(Imm32(ResumeFromException::RESUME_FORCED_RETURN), cmpTempRegister);
	beq(t6, cmpTempRegister, &return_);
	nop();

    breakpoint(); // Invalid kind.

    // No exception handler. Load the error value, load the new stack pointer
    // and return from the entry frame.
    bindBranch(&entryFrame);
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    movl(Operand(sp, offsetof(ResumeFromException, stackPointer)), sp);
    ret();

    // If we found a catch handler, this must be a baseline frame. Restore state
    // and jump to the catch block.
	bindBranch(&catch_);
    movl(Operand(sp, offsetof(ResumeFromException, target)), t6);
    movl(Operand(sp, offsetof(ResumeFromException, framePointer)), fp);
    movl(Operand(sp, offsetof(ResumeFromException, stackPointer)), sp);
    jmp(Operand(t6));

    // If we found a finally block, this must be a baseline frame. Push
    // two values expected by JSOP_RETSUB: BooleanValue(true) and the
    // exception.
    bindBranch(&finally);
    ValueOperand exception = ValueOperand(t7, t8);
    loadValue(Operand(sp, offsetof(ResumeFromException, exception)), exception);

    movl(Operand(sp, offsetof(ResumeFromException, target)), t6);
    movl(Operand(sp, offsetof(ResumeFromException, framePointer)), fp);
    movl(Operand(sp, offsetof(ResumeFromException, stackPointer)), sp);

    pushValue(BooleanValue(true));
    pushValue(exception);
    jmp(Operand(t6));

    // Only used in debug mode. Return BaselineFrame->returnValue() to the caller.
    bindBranch(&return_);
    movl(Operand(sp, offsetof(ResumeFromException, framePointer)), fp);
    movl(Operand(sp, offsetof(ResumeFromException, stackPointer)), sp);
    loadValue(Address(fp, BaselineFrame::reverseOffsetOfReturnValue()), JSReturnOperand);
    movl(fp, sp);
    pop(fp);
    ret();
}

void
MacroAssemblerMIPS::branchTestValue(Condition cond, const ValueOperand &value, const Value &v, Label *label)
{
    jsval_layout jv = JSVAL_TO_IMPL(v);
	// by wangqing, 2013-11-29
    if (v.isMarkable())
		movl(ImmGCPtr(reinterpret_cast<gc::Cell *>(v.toGCThing())), cmpTempRegister);
    else
		movl(Imm32(jv.s.payload.i32), cmpTempRegister);

    if (cond == Equal) {
        Label done;
		bne(value.payloadReg(), cmpTempRegister, &done);
		nop();
        {
            cmpl(value.typeReg(), Imm32(jv.s.tag));
            j(Equal, label);
        }
        bindBranch(&done);
    } else {
        JS_ASSERT(cond == NotEqual);
	    movl(value.payloadReg(), cmpTemp2Register);	

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
    // check DoubleNotEqual, by wangqing, 2013-11-29
    ceqd(reg, ScratchFloatReg);
    bc1f(&nonZero);
    nop();

    // Input register is either zero or negative zero. Test sign bit.
    // by wangqing, 2013-11-19 
    // get sign bit of Double
    mfc1(scratch, js::jit::FloatRegister::FromCode(reg.code() + 1));
    shrl(Imm32(31), scratch);
    // If reg is -0, then a test of Zero is true.
    cmpl(scratch, Imm32(1));

    bindBranch(&nonZero);
    return Zero;
}
