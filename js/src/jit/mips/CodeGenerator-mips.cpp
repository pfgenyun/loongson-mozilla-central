/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "mozilla/DebugOnly.h"

#include "jscntxt.h"
#include "jscompartment.h"
#include "jsnum.h"

#include "CodeGenerator-mips.h"
#include "jit/PerfSpewer.h"
#include "jit/CodeGenerator.h"
#include "jit/IonCompartment.h"
#include "jit/IonFrames.h"
#include "jit/MIR.h"
#include "jit/MIRGraph.h"
#include "jit/MoveEmitter.h"
#include "jit/shared/CodeGenerator-shared-inl.h"
#include "vm/Shape.h"
#include "jsscriptinlines.h"
#include "jsmath.h"
#include "jit/ParallelFunctions.h"
#include "jit/ExecutionModeInlines.h"
using namespace js;
using namespace js::jit;
using mozilla::DebugOnly;
using mozilla::DoubleExponentBias;
using mozilla::DoubleExponentShift;

namespace js {
namespace jit {
CodeGeneratorMIPS::CodeGeneratorMIPS(MIRGenerator *gen, LIRGraph *graph, MacroAssembler *masm)
  : CodeGeneratorShared(gen, graph, masm),
    deoptLabel_(NULL)
{
}

double
test(double x, double y)
{
    return x + y;
}

bool
CodeGeneratorMIPS::generatePrologue()
{
    // Note that this automatically sets MacroAssembler::framePushed().
    masm.reserveStack(frameSize());

    // Allocate returnLabel_ on the heap, so we don't run its destructor and
    // assert-not-bound in debug mode on compilation failure.
    returnLabel_ = new HeapLabel();

    return true;
}

bool
CodeGeneratorMIPS::generateEpilogue()
{
    masm.bind(returnLabel_);

    // Pop the stack we allocated at the start of the function.
    masm.freeStack(frameSize());
    JS_ASSERT(masm.framePushed() == 0);

    masm.ret();
    return true;
}

bool
OutOfLineBailout::accept(CodeGeneratorMIPS *codegen)
{
    return codegen->visitOutOfLineBailout(this);
}

void
CodeGeneratorMIPS::emitBranch(Assembler::Condition cond, MBasicBlock *mirTrue,
                                   MBasicBlock *mirFalse, Assembler::NaNCond ifNaN)
{
    LBlock *ifTrue = mirTrue->lir();
    LBlock *ifFalse = mirFalse->lir();

    if (ifNaN == Assembler::NaN_IsFalse)
        masm.j(Assembler::Parity, ifFalse->label());
    else if (ifNaN == Assembler::NaN_IsTrue)
        masm.j(Assembler::Parity, ifTrue->label());

    if (isNextBlock(ifFalse)) {
        masm.j(cond, ifTrue->label());
    } else {
        masm.j(Assembler::InvertCondition(cond), ifFalse->label());
        if (!isNextBlock(ifTrue))
            masm.jmp(ifTrue->label());
    }
}

//by weizhenwei, 2013.11.07
void
CodeGeneratorMIPS::emitBranch(Assembler::DoubleCondition cond,
        const FloatRegister &lhs, const FloatRegister &rhs,
        MBasicBlock *mirTrue, MBasicBlock *mirFalse, Assembler::NaNCond ifNaN)
{
    LBlock *ifTrue = mirTrue->lir();
    LBlock *ifFalse = mirFalse->lir();

    if (ifNaN == Assembler::NaN_IsFalse) {
        masm.branchDouble(Assembler::DoubleUnordered, lhs, rhs, ifFalse->label());
    } else if (ifNaN == Assembler::NaN_IsTrue) {
        masm.branchDouble(Assembler::DoubleUnordered, lhs, rhs, ifTrue->label());
    } else {
        masm.branchDouble(Assembler::DoubleUnordered, lhs, rhs, ifFalse->label());
    }


    if (isNextBlock(ifFalse)) {
        masm.branchDouble(cond, lhs, rhs, ifTrue->label());
    } else {
        masm.branchDouble(masm.InvertDoubleCondition(cond), lhs, rhs, ifFalse->label());
        if (!isNextBlock(ifTrue))
            masm.jmp(ifTrue->label());
    }
}

//by weizhenwei, 2013.11.07
void
CodeGeneratorMIPS::emitSet(Assembler::DoubleCondition cond, const FloatRegister &lhs,
        const FloatRegister &rhs, const Register &dest, Assembler::NaNCond ifNaN) {

    if (GeneralRegisterSet(Registers::SingleByteRegs).has(dest)) {
        // If the register we're defining is a single byte register,
        // take advantage of the setCC instruction
        Label setDest;
        masm.movl(Imm32(1), dest);
        masm.branchDoubleLocal(cond, lhs, rhs, &setDest); //by weizhenwei, 2013.11.27
        masm.xorl(dest, dest);
        masm.bindBranch(&setDest);

        if (ifNaN != Assembler::NaN_HandledByCond) {
            //DoubleOrdered check, by wangqing, 2013.11.27
            Label noNaN;
            masm.cud(lhs, rhs);
            masm.bc1f(&noNaN);
            masm.nop();

            if (ifNaN == Assembler::NaN_IsTrue)
                masm.movl(Imm32(1), dest);
            else
                masm.xorl(dest, dest);
            masm.bindBranch(&noNaN);
        }
    } else {
        Label end, ifFalse;

        if (ifNaN == Assembler::NaN_IsFalse) {
            //DoubleUnordered check, by wangqing, 2013.11.27
            masm.cud(lhs, rhs);
            masm.bc1t(&ifFalse);
            masm.nop();
        }
        masm.movl(Imm32(1), dest);
        masm.branchDoubleLocal(cond, lhs, rhs, &end);
        if (ifNaN == Assembler::NaN_IsTrue) {
            //DoubleUnordered check, by weizhenwei, 2013.11.27
            masm.cud(lhs, rhs);
            masm.bc1t(&end);
            masm.nop();
        }
          
        masm.bindBranch(&ifFalse);
        masm.xorl(dest, dest);

        masm.bindBranch(&end);
    }
}

bool
CodeGeneratorMIPS::visitDouble(LDouble *ins)
{
    const LDefinition *out = ins->getDef(0);
    masm.loadConstantDouble(ins->getDouble(), ToFloatRegister(out));
    return true;
}

bool
CodeGeneratorMIPS::visitTestIAndBranch(LTestIAndBranch *test)
{
    const LAllocation *opd = test->input();

    // Test the operand
	masm.cmpl(ToRegister(opd), zero);
    emitBranch(Assembler::NonZero, test->ifTrue(), test->ifFalse());
    return true;
}

bool
CodeGeneratorMIPS::visitTestDAndBranch(LTestDAndBranch *test)
{
    const LAllocation *opd = test->input();

    // ucomisd flags:
    //             Z  P  C
    //            ---------
    //      NaN    1  1  1
    //        >    0  0  0
    //        <    0  0  1
    //        =    1  0  0
    //
    // NaN is falsey, so comparing against 0 and then using the Z flag is
    // enough to determine which branch to take.
	//by weizhenwei, 2013.11.05
    masm.zerod(ScratchFloatReg);
    emitBranch(Assembler::DoubleNotEqual, ToFloatRegister(opd),
            ScratchFloatReg, test->ifTrue(), test->ifFalse());

    return true;
}

void
CodeGeneratorMIPS::emitCompare(MCompare::CompareType type,
        const LAllocation *left, const LAllocation *right)
{
    if (right->isConstant())
        masm.cmpl(ToRegister(left), Imm32(ToInt32(right)));
    else
        masm.cmpl(ToRegister(left), ToOperand(right));
}

bool
CodeGeneratorMIPS::visitCompare(LCompare *comp)
{
    MCompare *mir = comp->mir();
    emitCompare(mir->compareType(), comp->left(), comp->right());
    masm.emitSet(JSOpToCondition(mir->compareType(), comp->jsop()), ToRegister(comp->output()));
    return true;
}

bool
CodeGeneratorMIPS::visitCompareAndBranch(LCompareAndBranch *comp)
{
    MCompare *mir = comp->mir();
    emitCompare(mir->compareType(), comp->left(), comp->right());
    Assembler::Condition cond = JSOpToCondition(mir->compareType(), comp->jsop());
    emitBranch(cond, comp->ifTrue(), comp->ifFalse());
    return true;
}

bool
CodeGeneratorMIPS::visitCompareD(LCompareD *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());
    //by weizhenwei, 2013.11.07
    emitSet(cond, lhs, rhs, ToRegister(comp->output()),
            Assembler::NaNCondFromDoubleCondition(cond));
    return true;
}

bool
CodeGeneratorMIPS::visitNotI(LNotI *ins)
{
    masm.cmpl(ToRegister(ins->input()), Imm32(0));
    masm.emitSet(Assembler::Equal, ToRegister(ins->output()));
    return true;
}

bool
CodeGeneratorMIPS::visitNotD(LNotD *ins)
{
    FloatRegister opd = ToFloatRegister(ins->input());

    //by weizhenwei, 2013.11.07
    masm.zerod(ScratchFloatReg);
    emitSet(Assembler::DoubleEqualOrUnordered, opd, ScratchFloatReg,
            ToRegister(ins->output()), Assembler::NaN_IsTrue);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareDAndBranch(LCompareDAndBranch *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());
//  by weizhenwei, 2013.11.07
    emitBranch(cond, lhs, rhs, comp->ifTrue(), comp->ifFalse(),
            Assembler::NaNCondFromDoubleCondition(cond));
        
      
    return true;
}

bool
CodeGeneratorMIPS::visitAsmJSPassStackArg(LAsmJSPassStackArg *ins)
{
    const MAsmJSPassStackArg *mir = ins->mir();
    Operand dst(StackPointer, mir->spOffset());
    if (ins->arg()->isConstant()) {
        masm.mov(Imm32(ToInt32(ins->arg())), dst);
    } else {
        if (ins->arg()->isGeneralReg())
            masm.mov(ToRegister(ins->arg()), dst);
        else
            masm.movsd(ToFloatRegister(ins->arg()), dst);
    }
    return true;
}

bool
CodeGeneratorMIPS::generateOutOfLineCode()
{
    if (!CodeGeneratorShared::generateOutOfLineCode())
        return false;

    if (deoptLabel_) {
        // All non-table-based bailouts will go here.
        masm.bind(deoptLabel_);

        // Push the frame size, so the handler can recover the IonScript.
        masm.push(Imm32(frameSize()));

        IonCompartment *ion = GetIonContext()->compartment->ionCompartment();
        IonCode *handler = ion->getGenericBailoutHandler();

        masm.jmp(handler->raw(), Relocation::IONCODE);
    }

    return true;
}

class BailoutJump {
    Assembler::Condition cond_;

  public:
    BailoutJump(Assembler::Condition cond) : cond_(cond)
    { }
#if 0
    void operator()(MacroAssembler &masm, uint8_t *code) const {
        masm.j(cond_, code, Relocation::HARDCODED);
    }
#endif
    void operator()(MacroAssembler &masm, Label *label) const {
        masm.j(cond_, label);
    }
};

class BailoutLabel {
    Label *label_;

  public:
    BailoutLabel(Label *label) : label_(label)
    { }
#if 0
    void operator()(MacroAssembler &masm, uint8_t *code) const {
        masm.retarget(label_, code, Relocation::HARDCODED);
    }
#endif
    void operator()(MacroAssembler &masm, Label *label) const {
        masm.retarget(label_, label);
    }
};

template <typename T> bool
CodeGeneratorMIPS::bailout(const T &binder, LSnapshot *snapshot)
{
    CompileInfo &info = snapshot->mir()->block()->info();
    switch (info.executionMode()) {
      case ParallelExecution: {
        // in parallel mode, make no attempt to recover, just signal an error.
        OutOfLineParallelAbort *ool = oolParallelAbort(ParallelBailoutUnsupported,
                                                       snapshot->mir()->block(),
                                                       snapshot->mir()->pc());
        binder(masm, ool->entry());
        return true;
      }
      case SequentialExecution:
        break;
      default:
        JS_NOT_REACHED("No such execution mode");
    }

    if (!encode(snapshot))
        return false;

    // Though the assembler doesn't track all frame pushes, at least make sure
    // the known value makes sense. We can't use bailout tables if the stack
    // isn't properly aligned to the static frame size.
    JS_ASSERT_IF(frameClass_ != FrameSizeClass::None() && deoptTable_,
                 frameClass_.frameSize() == masm.framePushed());

#if 0
    // On mips, bailout tables are pointless, because 16 extra bytes are
    // reserved per external jump, whereas it takes only 10 bytes to encode a
    // a non-table based bailout.
    if (assignBailoutId(snapshot)) {
        binder(masm, deoptTable_->raw() + snapshot->bailoutId() * BAILOUT_TABLE_ENTRY_SIZE);
        return true;
    }
#endif

    // We could not use a jump table, either because all bailout IDs were
    // reserved, or a jump table is not optimal for this frame size or
    // platform. Whatever, we will generate a lazy bailout.
    OutOfLineBailout *ool = new OutOfLineBailout(snapshot);
    if (!addOutOfLineCode(ool))
        return false;

    binder(masm, ool->entry());
    return true;
}

bool
CodeGeneratorMIPS::bailoutIf(Assembler::Condition condition, LSnapshot *snapshot)
{
    return bailout(BailoutJump(condition), snapshot);
}

bool
CodeGeneratorMIPS::bailoutFrom(Label *label, LSnapshot *snapshot)
{
    JS_ASSERT(label->used() && !label->bound());
    return bailout(BailoutLabel(label), snapshot);
}

bool
CodeGeneratorMIPS::bailout(LSnapshot *snapshot)
{
    Label label;
    masm.jump(&label);
    return bailoutFrom(&label, snapshot);
}

bool
CodeGeneratorMIPS::visitOutOfLineBailout(OutOfLineBailout *ool)
{
    if (!deoptLabel_)
        deoptLabel_ = new HeapLabel();

    masm.push(Imm32(ool->snapshot()->snapshotOffset()));
    masm.jmp(deoptLabel_);
    return true;
}


// by wangqing, 2013-11-27
bool
CodeGeneratorMIPS::visitMinMaxD(LMinMaxD *ins)
{
    FloatRegister first = ToFloatRegister(ins->first());
    FloatRegister second = ToFloatRegister(ins->second());
    FloatRegister output = ToFloatRegister(ins->output());

    JS_ASSERT(first == output);

    Assembler::Condition cond = ins->mir()->isMax()
                               ? Assembler::Above
                               : Assembler::Below;
    Label nan, equal, returnSecond, done;

    // DoubleUnordered check, by wangqing, 2013-11-27
    // if first or shecond is NaN, then return NaN
    masm.cud(second, first);
    masm.bc1t(&nan);
    masm.nop();

    // DoubleEqual check, by wangqing, 2013-11-27
    // make sure we handle -0 and 0 right.
    masm.cseqd(second, first);
    masm.bc1t(&equal);
    masm.nop();

    if (cond == Assembler::Above) {
        //DoubleGreaterThan check, by wangqing, 2013.11.07
        masm.cngtd(second, first);
        masm.bc1f(&returnSecond);
        masm.nop();
    } else if (cond == Assembler::Below) {
        //DoubleLessThan check, by wangqing, 2013.11.27
        masm.cltd(second, first);
        masm.bc1t(&returnSecond);
        masm.nop();
    }

    masm.b(&done);
    masm.nop();

    // Check for zero.
    masm.bindBranch(&equal);

    // DoubleNotEqual check, by wangqing, 2013.11.27
    // first wasn't 0 or -0, so just return it.
    masm.zerod(ScratchFloatReg);
    masm.ceqd(first, ScratchFloatReg);
    masm.bc1f(&done);
    masm.nop();

    // So now both operands are either -0 or 0.
    if (ins->mir()->isMax())
        masm.addsd(second, first); // -0 + -0 = -0 and -0 + 0 = 0.
    else
        masm.orpd(second, first); // This just ors the sign bit.
    masm.b(&done);
    masm.nop();

    masm.bindBranch(&nan);
    masm.loadStaticDouble(&js_NaN, output);
    masm.b(&done);
    masm.nop();

    masm.bindBranch(&returnSecond);
    masm.movsd(second, output);

    masm.bindBranch(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitAbsD(LAbsD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));

    //by weizhenwei, 2013.11.08
    masm.absd(input, input);
    return true;
}

bool
CodeGeneratorMIPS::visitSqrtD(LSqrtD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));
    masm.sqrtsd(input, input);
    return true;
}

// by wangqing, 2013-11-25
bool
CodeGeneratorMIPS::visitPowHalfD(LPowHalfD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    Register scratch = ToRegister(ins->temp());
    JS_ASSERT(input == ToFloatRegister(ins->output()));

    const uint32_t NegInfinityFloatBits = 0xFF800000;
    Label done, sqrt; // by wangqing, 2013-11-27

    // Branch if not -Infinity.
    masm.move32(Imm32(NegInfinityFloatBits), scratch);
    masm.loadFloatAsDouble(scratch, ScratchFloatReg);
    //DoubleNotEqualOrUnordered check, by wangqing, 2013.11.27
    masm.cseqd(input, ScratchFloatReg);
    masm.bc1f(&sqrt);
    masm.nop();

    //by weizhenwei, 2013.11.08
    masm.zerod(input);
    masm.subsd(ScratchFloatReg, input);
    masm.b(&done);
    masm.nop();

    // Math.pow(-0, 0.5) == 0 == Math.pow(0, 0.5). Adding 0 converts any -0 to 0.
    masm.bindBranch(&sqrt);
    //by weizhenwei, 2013.11.08
    masm.zerod(ScratchFloatReg);
    masm.addsd(ScratchFloatReg, input);
    masm.sqrtsd(input, input);

    masm.bindBranch(&done);
    return true;
}

class OutOfLineUndoALUOperation : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    LInstruction *ins_;

  public:
    OutOfLineUndoALUOperation(LInstruction *ins)
        : ins_(ins)
    { }
    virtual bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitOutOfLineUndoALUOperation(this);
    }
    LInstruction *ins() const {
        return ins_;
    }
};

bool
CodeGeneratorMIPS::visitAddI(LAddI *ins)
{
    //edit by QuQiuwen
    if (ins->rhs()->isConstant()){
        masm.cmpl(ToOperand(ins->lhs()), Imm32(ToInt32(ins->rhs())));
        masm.negl(cmpTemp2Register);
        masm.addl(Imm32(ToInt32(ins->rhs())), ToOperand(ins->lhs()));
    }
    else{
        masm.cmpl(ToOperand(ins->rhs()), ToRegister(ins->lhs()));
        masm.negl(cmpTemp2Register);
        masm.addl(ToOperand(ins->rhs()), ToRegister(ins->lhs()));
    }

    if (ins->snapshot()) {
        if (ins->recoversInput()) {
            OutOfLineUndoALUOperation *ool = new OutOfLineUndoALUOperation(ins);
            if (!addOutOfLineCode(ool))
                return false;
            masm.j(Assembler::Overflow, ool->entry());
        } else {
            if (!bailoutIf(Assembler::Overflow, ins->snapshot()))
                return false;
        }
    }
    return true;
}

bool
CodeGeneratorMIPS::visitSubI(LSubI *ins)
{
    //add cmpl ,edit by QuQiuwen,
   if (ins->rhs()->isConstant()){
        masm.cmpl(ToOperand(ins->lhs()),Imm32(ToInt32(ins->rhs())));
        masm.subl(Imm32(ToInt32(ins->rhs())), ToOperand(ins->lhs()));
     }
    else{
        masm.cmpl(ToRegister(ins->lhs()),ToOperand(ins->rhs()));
        masm.subl(ToOperand(ins->rhs()), ToRegister(ins->lhs()));
    }

    if (ins->snapshot()) {
        if (ins->recoversInput()) {
            OutOfLineUndoALUOperation *ool = new OutOfLineUndoALUOperation(ins);
            if (!addOutOfLineCode(ool))
                return false;
            masm.j(Assembler::Overflow, ool->entry());
        } else {
            if (!bailoutIf(Assembler::Overflow, ins->snapshot()))
                return false;
        }
    }
    return true;
}

bool
CodeGeneratorMIPS::visitOutOfLineUndoALUOperation(OutOfLineUndoALUOperation *ool )
{
    LInstruction *ins = ool->ins();
    Register reg = ToRegister(ins->getDef(0));

    mozilla::DebugOnly<LAllocation *> lhs = ins->getOperand(0);
    LAllocation *rhs = ins->getOperand(1);

    JS_ASSERT(reg == ToRegister(lhs));
    JS_ASSERT_IF(rhs->isGeneralReg(), reg != ToRegister(rhs));

    // Undo the effect of the ALU operation, which was performed on the output
    // register and overflowed. Writing to the output register clobbered an
    // input reg, and the original value of the input needs to be recovered
    // to satisfy the constraint imposed by any RECOVERED_INPUT operands to
    // the bailout snapshot.

    if (rhs->isConstant()) {
        Imm32 constant(ToInt32(rhs));
        if (ins->isAddI())
            masm.subl(constant, reg);
        else
            masm.addl(constant, reg);
    } else {
        if (ins->isAddI())
            masm.subl(ToOperand(rhs), reg);
        else
            masm.addl(ToOperand(rhs), reg);
    }

    return bailout(ool->ins()->snapshot());
}

class MulNegativeZeroCheck : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    LMulI *ins_;

  public:
    MulNegativeZeroCheck(LMulI *ins)
      : ins_(ins)
    { }
    virtual bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitMulNegativeZeroCheck(this);
    }
    LMulI *ins() const {
        return ins_;
    }
};

bool
CodeGeneratorMIPS::visitMulI(LMulI *ins)
{
    const LAllocation *lhs = ins->lhs();
    const LAllocation *rhs = ins->rhs();
    MMul *mul = ins->mir();
    JS_ASSERT_IF(mul->mode() == MMul::Integer, !mul->canBeNegativeZero() && !mul->canOverflow());

    if (rhs->isConstant()) {
        // Bailout on -0.0
        int32_t constant = ToInt32(rhs);
        if (mul->canBeNegativeZero() && constant <= 0) {
            Assembler::Condition bailoutCond = (constant == 0) ? Assembler::Signed : Assembler::Equal;
			// by wangqing, 2013-11-27 testl--->cmpl
			masm.cmpl(ToRegister(lhs), zero);
            if (!bailoutIf(bailoutCond, ins->snapshot()))
                    return false;
        }

        switch (constant) {
          case 0:
            masm.xorl(ToOperand(lhs), ToRegister(lhs));
            return true; // escape overflow check;
          case 1:
            return true; // escape overflow check;
          default:
            if (!mul->canOverflow() && constant > 0) {
                // Use shift if cannot overflow and constant is power of 2
                int32_t shift;
                JS_FLOOR_LOG2(shift, constant);
                if ((1 << shift) == constant) {
                    masm.shll(Imm32(shift), ToRegister(lhs));
                    return true;
                }
            }
	    // by wangqing, 2013-11-29
	    // Optimization the mul instruction.
            if(mul->canOverflow()){
            	masm.imull(Imm32(ToInt32(rhs)), ToRegister(lhs));

            	//overflow check, by weizhenwei, 2013.11.14
            	masm.mfhi(cmpTempRegister);
            	masm.mflo(cmpTemp2Register);
            	masm.sarl(Imm32(0x1f), cmpTemp2Register);

            	// Bailout on overflow
            	//see See MIPS Run Linux Chinese 2rd, Page 139. overflow check logic
            	if (mul->canOverflow() && !bailoutIf(Assembler::NotEqual, ins->snapshot()))
                	return false;
	    	}else{
			        masm.movl(Imm32(ToInt32(rhs)), cmpTempRegister);
                	masm.mul_opt(ToRegister(lhs), cmpTempRegister, ToRegister(lhs));
	    	}
	   }
    } else {
	// by wangqing, 2013-11-29
	// Optimization the mul instruction.
	if(mul->canOverflow()){
           	masm.imull(ToOperand(rhs), ToRegister(lhs));

        	//overflow check, by weizhenwei, 2013.11.14
      	  	masm.mfhi(cmpTempRegister);
        	masm.mflo(cmpTemp2Register);
       		masm.sarl(Imm32(0x1f), cmpTemp2Register);

        	// Bailout on overflow
      		//see See MIPS Run Linux Chinese 2rd, Page 139. overflow check logic
            if (mul->canOverflow() && !bailoutIf(Assembler::NotEqual, ins->snapshot()))
                return false;
	}else{
            masm.imull_opt(ToOperand(rhs), ToRegister(lhs));
	}

        if (mul->canBeNegativeZero()) {
            // Jump to an OOL path if the result is 0.
            MulNegativeZeroCheck *ool = new MulNegativeZeroCheck(ins);
            if (!addOutOfLineCode(ool))
                return false;

            // by wangqing, 2013-11-27
            // Modify the testl ----> cmpl
            masm.cmpl(ToRegister(lhs), zero); 
            masm.j(Assembler::Zero, ool->entry());
            masm.bind(ool->rejoin());
        }
    }

    return true;
}

//by weizhenwei, 2013.11.28
//divide AsmJSDivOrMod into AsmJSDiv and AsmJSMod
bool
CodeGeneratorMIPS::visitAsmJSDiv(LAsmJSDiv *ins)
{
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());
    Register output = ToRegister(ins->output());

    JS_ASSERT( lhs == output);

    Label afterDiv;
    Label notzero;
    masm.bne(rhs, zero, &notzero);
    masm.nop();
    masm.xorl(output, output);
    masm.b(&afterDiv);
    masm.nop();
    masm.bindBranch(&notzero);

    masm.udiv(lhs, rhs);

    masm.bindBranch(&afterDiv);

    return true;
}
bool
CodeGeneratorMIPS::visitAsmJSMod(LAsmJSMod *ins)
{
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());
    Register output = ToRegister(ins->output());

    JS_ASSERT( lhs == output);

    Label afterMod;
    Label notzero;
    masm.bne(rhs, zero, &notzero);
    masm.nop();
    masm.xorl(output, output);
    masm.b(&afterMod);
    masm.nop();
    masm.bindBranch(&notzero);

    masm.udivmod(lhs, rhs);

    masm.bindBranch(&afterMod);

    return true;
}

bool
CodeGeneratorMIPS::visitMulNegativeZeroCheck(MulNegativeZeroCheck *ool)
{
    LMulI *ins = ool->ins();
    Register result = ToRegister(ins->output());
    Operand lhsCopy = ToOperand(ins->lhsCopy());
    Operand rhs = ToOperand(ins->rhs());
    JS_ASSERT_IF(lhsCopy.kind() == Operand::REG, lhsCopy.reg() != result.code());

    // Result is -0 if lhs or rhs is negative.
    masm.movl(lhsCopy, result);
    masm.orl(rhs, result);
    // by wangqing, 2013-11-19
    masm.cmpl(result, zero);
    if (!bailoutIf(Assembler::Signed, ins->snapshot()))
        return false;

    masm.xorl(result, result);
    masm.jmp(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitDivPowTwoI(LDivPowTwoI *ins)
{
    Register lhs = ToRegister(ins->numerator());
    Register lhsCopy = ToRegister(ins->numeratorCopy());
    mozilla::DebugOnly<Register> output = ToRegister(ins->output());
    int32_t shift = ins->shift();

    // We use defineReuseInput so these should always be the same, which is
    // convenient since all of our instructions here are two-address.
    JS_ASSERT(lhs == output);

    if (shift != 0) {
        if (!ins->mir()->isTruncated()) {
            // If the remainder is != 0, bailout since this must be a double.
            // Overwrite testl, by wangqing, 2013-12-11
			masm.movl(Imm32(UINT32_MAX >> (32 - shift)), cmpTempRegister);
			masm.andInsn(cmpTempRegister, lhs, cmpTempRegister);
			masm.movl(zero, cmpTemp2Register);
            if (!bailoutIf(Assembler::NonZero, ins->snapshot()))
                return false;
        }

        // Adjust the value so that shifting produces a correctly rounded result
        // when the numerator is negative. See 10-1 "Signed Division by a Known
        // Power of 2" in Henry S. Warren, Jr.'s Hacker's Delight.
        // Note that we wouldn't need to do this adjustment if we could use
        // Range Analysis to find cases when the value is never negative. We
        // wouldn't even need the lhsCopy either in that case.
        if (shift > 1)
            masm.sarl(Imm32(31), lhs);
        masm.shrl(Imm32(32 - shift), lhs);
        masm.addl(lhsCopy, lhs);

        // Do the shift.
        masm.sarl(Imm32(shift), lhs);
    }

    return true;
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitDivI(LDivI *ins)
{
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());
    Register output = ToRegister(ins->output());
    MDiv *mir = ins->mir();

    JS_ASSERT(lhs == output); //by weizhenwei, 2013.11.28
    Label done;

    // Handle divide by zero.
    if (mir->canBeDivideByZero()) {
        if (mir->isTruncated()) {
            // Truncated division by zero is zero (Infinity|0 == 0)
            Label notzero;
            masm.bne(rhs, zero, &notzero);
            masm.nop();
            masm.xorl(output, output);
            masm.b(&done);
            masm.nop();
            masm.bindBranch(&notzero);
        } else {
            // by wangqing, 2013-11-27 testl--->cmpl
            masm.cmpl(rhs, zero);
            JS_ASSERT(mir->fallible());
            if (!bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
        }
    }

    // Handle an integer overflow exception from -2147483648 / -1.
    if (mir->canBeNegativeOverflow()) {
        Label notmin;
	// by wangqing, 2013-11-25
	masm.movl(Imm32(INT32_MIN), cmpTempRegister);
	masm.bne(lhs, cmpTempRegister, &notmin);
	masm.nop();
	// by wangqing, 2013-11-27
        if (mir->isTruncated()) {
            // (-INT32_MIN)|0 == INT32_MIN and INT32_MIN is already in the
            // output register.
            masm.movl(Imm32(-1), cmpTempRegister);
            masm.beq(rhs, cmpTempRegister, &done);
            masm.nop();
        } else {
            masm.cmpl(rhs, Imm32(-1));
            JS_ASSERT(mir->fallible());
            if (!bailoutIf(Assembler::Equal, ins->snapshot()))
                return false;
        }
        masm.bindBranch(&notmin);
    }

    // Handle negative 0.
    if (!mir->isTruncated() && mir->canBeNegativeZero()) {
        Label nonzero;
		// by wangqing, 2013-11-25
        masm.bne(lhs, zero, &nonzero);
        masm.nop();
        masm.cmpl(rhs, Imm32(0));
        if (!bailoutIf(Assembler::LessThan, ins->snapshot()))
            return false;
        masm.bindBranch(&nonzero);
    }

    //by weizhenwei, 2013.11.28
    masm.idiv(lhs, rhs);

    if (!mir->isTruncated()) {
        // If the remainder is > 0, bailout since this must be a double.
	    //masm.cmpl(remainder, zero); // by wangqing, 2013-11-27 testl--->cmpl
        //by weizhenwei, 2013.11.28
        masm.mfhi(cmpTempRegister);
        masm.movl(zero, cmpTemp2Register);
        if (!bailoutIf(Assembler::NonZero, ins->snapshot()))
            return false;
    }

    masm.bindBranch(&done);

    return true;
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitModPowTwoI(LModPowTwoI *ins)
{
    Register lhs = ToRegister(ins->getOperand(0));
    int32_t shift = ins->shift();

    Label negative, done;
    // Switch based on sign of the lhs.
    // Positive numbers are just a bitmask
    masm.bltz(lhs, &negative);
    masm.nop();
    {
        masm.andl(Imm32((1 << shift) - 1), lhs);
        masm.b(&done);
	    masm.nop();
    }
    // Negative numbers need a negate, bitmask, negate
    {
        masm.bindBranch(&negative);
        // visitModI has an overflow check here to catch INT_MIN % -1, but
        // here the rhs is a power of 2, and cannot be -1, so the check is not generated.
        masm.negl(lhs);
        masm.andl(Imm32((1 << shift) - 1), lhs);
        masm.negl(lhs);
        if (!ins->mir()->isTruncated() && !bailoutIf(Assembler::Zero, ins->snapshot()))
            return false;
    }
    masm.bindBranch(&done);
    return true;

}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitModI(LModI *ins)
{
    Register remainder = ToRegister(ins->remainder());
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());

    //by weizhenwei, 2013.11.28
    JS_ASSERT(lhs == remainder);

    Label done;

    // Prevent divide by zero
    if (ins->mir()->isTruncated()) {
        Label notzero;
        masm.bne(rhs, zero, &notzero);
        masm.nop();
        masm.xorl(remainder, remainder);
        masm.b(&done);
        masm.nop();
        masm.bindBranch(&notzero);
    } else {
        // by wangqing, 2013-11-27 testl--->cmpl
        masm.cmpl(rhs, zero);
        if (!bailoutIf(Assembler::Zero, ins->snapshot()))
            return false;
    }

    Label negative;

    // Switch based on sign of the lhs.
    masm.bltz(lhs, &negative);
    masm.nop();
    // If lhs >= 0 then remainder = lhs % rhs. The remainder must be positive.
    {
        // Since lhs >= 0, the sign-extension will be 0
        // by weizhenwei, 2013.11.28
        masm.idivmod(lhs, rhs);
        masm.b(&done);
	    masm.nop();
    }

    // Otherwise, we have to beware of two special cases:
    {
        masm.bindBranch(&negative);

        // Prevent an integer overflow exception from -2147483648 % -1
        Label notmin;
    	masm.movl(Imm32(INT32_MIN), cmpTempRegister);
        masm.bne(lhs, cmpTempRegister, &notmin);
        masm.nop();

        // by wangqing, 2013-11-27
        if (ins->mir()->isTruncated()) {
	    	masm.movl(Imm32(-1), cmpTempRegister);
            masm.bne(rhs, cmpTempRegister, &notmin);
            masm.nop();
            //by weizhenwei, 2013.11.28
            masm.xorl(remainder, remainder);
            masm.b(&done);
            masm.nop();
        } else {
        	masm.cmpl(rhs, Imm32(-1));
            if (!bailoutIf(Assembler::Equal, ins->snapshot()))
                return false;
        }
        masm.bindBranch(&notmin);

        //by weizhenwei, 2013.11.28
        masm.idivmod(lhs, rhs);

        if (!ins->mir()->isTruncated()) {
            // A remainder of 0 means that the rval must be -0, which is a double.
	        masm.cmpl(remainder, zero); // by wangqing, 2013-11-27 testl--->cmpl
            if (!bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
        }
    }

    masm.bindBranch(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitBitNotI(LBitNotI *ins)
{
    const LAllocation *input = ins->getOperand(0);
    JS_ASSERT(!input->isConstant());

    masm.notl(ToOperand(input));
    return true;
}

bool
CodeGeneratorMIPS::visitBitOpI(LBitOpI *ins)
{
    const LAllocation *lhs = ins->getOperand(0);
    const LAllocation *rhs = ins->getOperand(1);

    switch (ins->bitop()) {
        case JSOP_BITOR:
            if (rhs->isConstant())
                masm.orl(Imm32(ToInt32(rhs)), ToOperand(lhs));
            else
                masm.orl(ToOperand(rhs), ToRegister(lhs));
            break;
        case JSOP_BITXOR:
            if (rhs->isConstant())
                masm.xorl(Imm32(ToInt32(rhs)), ToOperand(lhs));
            else
                masm.xorl(ToOperand(rhs), ToRegister(lhs));
            break;
        case JSOP_BITAND:
            if (rhs->isConstant())
                masm.andl(Imm32(ToInt32(rhs)), ToOperand(lhs));
            else
                masm.andl(ToOperand(rhs), ToRegister(lhs));
            break;
        default:
            JS_NOT_REACHED("unexpected binary opcode");
    }

    return true;
}

bool
CodeGeneratorMIPS::visitShiftI(LShiftI *ins)
{
    Register lhs = ToRegister(ins->lhs());
    const LAllocation *rhs = ins->rhs();

    if (rhs->isConstant()) {
        int32_t shift = ToInt32(rhs) & 0x1F;
        switch (ins->bitop()) {
          case JSOP_LSH:
            if (shift)
                masm.shll(Imm32(shift), lhs);
            break;
          case JSOP_RSH:
            if (shift)
                masm.sarl(Imm32(shift), lhs);
            break;
          case JSOP_URSH:
            if (shift) {
                masm.shrl(Imm32(shift), lhs);
            } else if (ins->mir()->toUrsh()->canOverflow()) {
                // x >>> 0 can overflow.
				masm.cmpl(lhs, zero); // by wangqing , 2013-11-27, testl--->cmpl
                if (!bailoutIf(Assembler::Signed, ins->snapshot()))
                    return false;
            }
            break;
          default:
            JS_NOT_REACHED("Unexpected shift op");
        }
    } else {
        //by weizhenwei, 2013.11.28
        JS_ASSERT(rhs->isGeneralReg());
        Register cl = ToRegister(rhs);
        switch (ins->bitop()) {
          case JSOP_LSH:
            masm.shll_cl(lhs, cl);
            break;
          case JSOP_RSH:
            masm.sarl_cl(lhs, cl);
            break;
          case JSOP_URSH:
            masm.shrl_cl(lhs, cl);
            if (ins->mir()->toUrsh()->canOverflow()) {
                // x >>> 0 can overflow.
				masm.cmpl(lhs, zero); // by wangqing, 2013-11-27, testl--->cmpl
                if (!bailoutIf(Assembler::Signed, ins->snapshot()))
                    return false;
            }
            break;
          default:
            JS_NOT_REACHED("Unexpected shift op");
        }
    }

    return true;
}

bool
CodeGeneratorMIPS::visitUrshD(LUrshD *ins)
{
    Register lhs = ToRegister(ins->lhs());
    JS_ASSERT(ToRegister(ins->temp()) == lhs);

    const LAllocation *rhs = ins->rhs();
    FloatRegister out = ToFloatRegister(ins->output());

    if (rhs->isConstant()) {
        int32_t shift = ToInt32(rhs) & 0x1F;
        if (shift)
            masm.shrl(Imm32(shift), lhs);
    } else {
        //by weizhenwei, 2013.11.28
        JS_ASSERT(rhs->isGeneralReg());
        Register cl = ToRegister(rhs);
        masm.shrl_cl(lhs, cl);
    }

    masm.convertUInt32ToDouble(lhs, out);
    return true;
}

typedef MoveResolver::MoveOperand MoveOperand;

MoveOperand
CodeGeneratorMIPS::toMoveOperand(const LAllocation *a) const
{
    if (a->isGeneralReg())
        return MoveOperand(ToRegister(a));
    if (a->isFloatReg())
        return MoveOperand(ToFloatRegister(a));
    return MoveOperand(StackPointer, ToStackOffset(a));
}

bool
CodeGeneratorMIPS::visitMoveGroup(LMoveGroup *group)
{
    if (!group->numMoves())
        return true;

    MoveResolver &resolver = masm.moveResolver();

    for (size_t i = 0; i < group->numMoves(); i++) {
        const LMove &move = group->getMove(i);

        const LAllocation *from = move.from();
        const LAllocation *to = move.to();

        // No bogus moves.
        JS_ASSERT(*from != *to);
        JS_ASSERT(!from->isConstant());
        JS_ASSERT(from->isDouble() == to->isDouble());

        MoveResolver::Move::Kind kind = from->isDouble()
                                        ? MoveResolver::Move::DOUBLE
                                        : MoveResolver::Move::GENERAL;

        if (!resolver.addMove(toMoveOperand(from), toMoveOperand(to), kind))
            return false;
    }

    if (!resolver.resolve())
        return false;

    MoveEmitter emitter(masm);
    emitter.emit(resolver);
    emitter.finish();

    return true;
}

class OutOfLineTableSwitch : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    MTableSwitch *mir_;
    CodeLabel jumpLabel_;

    bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitOutOfLineTableSwitch(this);
    }

  public:
    OutOfLineTableSwitch(MTableSwitch *mir)
      : mir_(mir)
    {}

    MTableSwitch *mir() const {
        return mir_;
    }

    CodeLabel *jumpLabel() {
        return &jumpLabel_;
    }
};


bool
CodeGeneratorMIPS::visitOutOfLineTableSwitch(OutOfLineTableSwitch *ool)
{
    MTableSwitch *mir = ool->mir();

    masm.align(sizeof(void*));
    masm.bind(ool->jumpLabel()->src());
    if (!masm.addCodeLabel(*ool->jumpLabel()))
        return false;

    for (size_t i = 0; i < mir->numCases(); i++) {
        LBlock *caseblock = mir->getCase(i)->lir();
        Label *caseheader = caseblock->label();
        uint32_t caseoffset = caseheader->offset();

        // The entries of the jump table need to be absolute addresses and thus
        // must be patched after codegen is finished.
        CodeLabel cl;
        masm.writeCodePointer(cl.dest());
        cl.src()->bind(caseoffset);
        if (!masm.addCodeLabel(cl))
            return false;
    }

    return true;
}

bool
CodeGeneratorMIPS::emitTableSwitchDispatch(MTableSwitch *mir, const Register &index,
                                                const Register &base)
{
    Label *defaultcase = mir->getDefault()->lir()->label();

    // Lower value with low value
    if (mir->low() != 0)
        masm.subl(Imm32(mir->low()), index);

    // Jump to default case if input is out of range
    int32_t cases = mir->numCases();
    masm.cmpl(index, Imm32(cases));
    masm.j(Assembler::AboveOrEqual, defaultcase);

    // To fill in the CodeLabels for the case entries, we need to first
    // generate the case entries (we don't yet know their offsets in the
    // instruction stream).
    OutOfLineTableSwitch *ool = new OutOfLineTableSwitch(mir);
    if (!addOutOfLineCode(ool))
        return false;

    // Compute the position where a pointer to the right case stands.
    masm.mov(ool->jumpLabel()->dest(), base);
    Operand pointer = Operand(base, index, ScalePointer);

    // Jump to the right case
    masm.jmp(pointer);

    return true;
}

bool
CodeGeneratorMIPS::visitMathD(LMathD *math)
{
    FloatRegister lhs = ToFloatRegister(math->lhs());
    Operand rhs = ToOperand(math->rhs());

    JS_ASSERT(ToFloatRegister(math->output()) == lhs);

    switch (math->jsop()) {
      case JSOP_ADD:
        masm.addsd(rhs, lhs);
        break;
      case JSOP_SUB:
        masm.subsd(rhs, lhs);
        break;
      case JSOP_MUL:
        masm.mulsd(rhs, lhs);
        break;
      case JSOP_DIV:
        masm.divsd(rhs, lhs);
        break;
      default:
        JS_NOT_REACHED("unexpected opcode");
        return false;
    }
    return true;
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitFloor(LFloor *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    FloatRegister scratch = ScratchFloatReg;
    Register output = ToRegister(lir->output());

    Label negative, end; // by wangqing, 2013-11-27

    // Branch to a slow path for negative inputs. Doesn't catch NaN or -0.
    // by weizhenwei, 2013.11.08
    masm.zerod(scratch);

    //DoubleLessThan check, by wangqing, 2013.11.27
    masm.cltd(input, scratch);
    masm.bc1t(&negative);
    masm.nop();

    // Bail on negative-zero.
    Assembler::Condition bailCond = masm.testNegativeZero(input, output);
    if (!bailoutIf(bailCond, lir->snapshot()))
        return false;

    // Input is non-negative, so truncation correctly rounds.
    masm.cvttsd2si(input, output);
    masm.cmpl(output, Imm32(0x7fffffff)); //by wangqing, 2013-11-19
    if (!bailoutIf(Assembler::Equal, lir->snapshot()))
        return false;

    masm.b(&end);
    masm.nop();

    // Input is negative, but isn't -0.
    // Negative values go on a comparatively expensive path, since no
    // native rounding mode matches JS semantics. Still better than callVM.
    masm.bindBranch(&negative);
    {
        // Truncate and round toward zero.
        // This is off-by-one for everything but integer-valued inputs.
        masm.cvttsd2si(input, output);
        masm.cmpl(output, Imm32(0x7fffffff)); // by wangqing, 2013-11-19
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;

        // Test whether the input double was integer-valued.
        masm.cvtsi2sd(output, scratch);
        //DoubleEqualOrUnordered check, by wangqing, 2013.11.27
        masm.cueqd(input, scratch);
        masm.bc1t(&end);
        masm.nop();

        // Input is not integer-valued, so we rounded off-by-one in the
        // wrong direction. Correct by subtraction.
        masm.subl(Imm32(1), output);
        // Cannot overflow: output was already checked against INT_MIN.
    }

    masm.bindBranch(&end);
    return true;
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitRound(LRound *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    FloatRegister temp = ToFloatRegister(lir->temp());
    FloatRegister scratch = ScratchFloatReg;
    Register output = ToRegister(lir->output());

    Label negative, end; // by wangqing, 2013-11-27

    // Load 0.5 in the temp register.
    static const double PointFive = 0.5;
    masm.loadStaticDouble(&PointFive, temp);

    // Branch to a slow path for negative inputs. Doesn't catch NaN or -0.
    // by weizhenwei, 2013.11.08
    masm.zerod(scratch);
    //DoubleLessThan check, by wangqing, 2013.11.27
    masm.cltd(input, scratch);
    masm.bc1t(&negative);
    masm.nop();

    // Bail on negative-zero.
    Assembler::Condition bailCond = masm.testNegativeZero(input, output);
    if (!bailoutIf(bailCond, lir->snapshot()))
        return false;

    // Input is non-negative. Add 0.5 and truncate, rounding down. Note that we
    // have to add the input to the temp register (which contains 0.5) because
    // we're not allowed to modify the input register.
    masm.addsd(input, temp);

    masm.cvttsd2si(temp, output);
    masm.cmpl(output, Imm32(0x7fffffff)); // by wangqing, 2013-11-19
    if (!bailoutIf(Assembler::Equal, lir->snapshot()))
        return false;

    masm.b(&end);
    masm.nop();


    // Input is negative, but isn't -0.
    masm.bindBranch(&negative);

    masm.addsd(input, temp);

    // Round toward -Infinity without the benefit of ROUNDSD.
    Label testZero; // by wangqing, 2013-11-27
    {
        // Truncate and round toward zero.
        // This is off-by-one for everything but integer-valued inputs.
        masm.cvttsd2si(temp, output);
        masm.cmpl(output, Imm32(0x7fffffff)); // by wangqing, 2013-11-19
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;

        // Test whether the truncated double was integer-valued.
        masm.cvtsi2sd(output, scratch);
        //DoubleEqualOrUnordered check, by wangqing, 2013.11.27
        masm.cueqd(temp, scratch);
        masm.bc1t(&testZero);
        masm.nop();

        // Input is not integer-valued, so we rounded off-by-one in the
        // wrong direction. Correct by subtraction.
        masm.subl(Imm32(1), output);
        // Cannot overflow: output was already checked against INT_MIN.

        // Fall through to testZero.
    }

    masm.bindBranch(&testZero);
    if (!bailoutIf(Assembler::Zero, lir->snapshot()))
        return false;

    //add NaN check and Bailout, by weizhenwei, 2013.11.19
    if (!bailoutIf(Assembler::Parity, lir->snapshot()))
        return false;

    masm.bindBranch(&end);
    return true;
}

bool
CodeGeneratorMIPS::visitGuardShape(LGuardShape *guard)
{
    Register obj = ToRegister(guard->input());
    masm.cmpPtr(Operand(obj, JSObject::offsetOfShape()), ImmGCPtr(guard->mir()->shape()));

    return bailoutIf(Assembler::NotEqual, guard->snapshot());
}

bool
CodeGeneratorMIPS::visitGuardObjectType(LGuardObjectType *guard)
{
    Register obj = ToRegister(guard->input());
    masm.cmpPtr(Operand(obj, JSObject::offsetOfType()), ImmGCPtr(guard->mir()->typeObject()));

    Assembler::Condition cond =
        guard->mir()->bailOnEquality() ? Assembler::Equal : Assembler::NotEqual;
    return bailoutIf(cond, guard->snapshot());
}

bool
CodeGeneratorMIPS::visitGuardClass(LGuardClass *guard)
{
    Register obj = ToRegister(guard->input());
    Register tmp = ToRegister(guard->tempInt());

    masm.loadPtr(Address(obj, JSObject::offsetOfType()), tmp);
    masm.cmpPtr(Operand(tmp, offsetof(types::TypeObject, clasp)), ImmWord(guard->mir()->getClass()));
    if (!bailoutIf(Assembler::NotEqual, guard->snapshot()))
        return false;
    return true;
}

bool
CodeGeneratorMIPS::visitEffectiveAddress(LEffectiveAddress *ins)
{
    const MEffectiveAddress *mir = ins->mir();
    Register base = ToRegister(ins->base());
    Register index = ToRegister(ins->index());
    Register output = ToRegister(ins->output());
    masm.leal(Operand(base, index, mir->scale(), mir->displacement()), output);
    return true;
}

Operand
CodeGeneratorMIPS::createArrayElementOperand(Register elements, const LAllocation *index)
{
    if (index->isConstant())
        return Operand(elements, ToInt32(index) * sizeof(js::Value));

    return Operand(elements, ToRegister(index), TimesEight);
}
bool
CodeGeneratorMIPS::generateInvalidateEpilogue()
{
    // Ensure that there is enough space in the buffer for the OsiPoint
    // patching to occur. Otherwise, we could overwrite the invalidation
    // epilogue.
    for (size_t i = 0; i < sizeof(void *); i+= Assembler::nopSize())
        masm.nop();
    
    masm.nop();
    masm.nop();
    masm.nop();
    masm.nop();
    masm.nop();
    masm.nop();

    masm.bind(&invalidate_);

    // Push the Ion script onto the stack (when we determine what that pointer is).
    invalidateEpilogueData_ = masm.pushWithPatch(ImmWord(uintptr_t(-1)));
    IonCode *thunk = GetIonContext()->compartment->ionCompartment()->getInvalidationThunk();

    masm.call(thunk);

    // We should never reach this point in JIT code -- the invalidation thunk should
    // pop the invalidated JS frame and return directly to its caller.
    masm.breakpoint();
    return true;
}

bool
CodeGeneratorMIPS::visitNegI(LNegI *ins)
{
    Register input = ToRegister(ins->input());
    JS_ASSERT(input == ToRegister(ins->output()));

    masm.neg32(input);
    return true;
}

bool
CodeGeneratorMIPS::visitNegD(LNegD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));

    masm.negateDouble(input);
    return true;
}


} // namespace jit
} // namespace js

static const uint32_t FrameSizes[] = { 128, 256, 512, 1024 };

FrameSizeClass
FrameSizeClass::FromDepth(uint32_t frameDepth)
{
    for (uint32_t i = 0; i < JS_ARRAY_LENGTH(FrameSizes); i++) {
        if (frameDepth < FrameSizes[i])
            return FrameSizeClass(i);
    }

    return FrameSizeClass::None();
}

FrameSizeClass
FrameSizeClass::ClassLimit()
{
    return FrameSizeClass(JS_ARRAY_LENGTH(FrameSizes));
}

uint32_t
FrameSizeClass::frameSize() const
{
    JS_ASSERT(class_ != NO_FRAME_SIZE_CLASS_ID);
    JS_ASSERT(class_ < JS_ARRAY_LENGTH(FrameSizes));

    return FrameSizes[class_];
}

ValueOperand
CodeGeneratorMIPS::ToValue(LInstruction *ins, size_t pos)
{
    Register typeReg = ToRegister(ins->getOperand(pos + TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getOperand(pos + PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

ValueOperand
CodeGeneratorMIPS::ToOutValue(LInstruction *ins)
{
    Register typeReg = ToRegister(ins->getDef(TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getDef(PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

ValueOperand
CodeGeneratorMIPS::ToTempValue(LInstruction *ins, size_t pos)
{
    Register typeReg = ToRegister(ins->getTemp(pos + TYPE_INDEX));
    Register payloadReg = ToRegister(ins->getTemp(pos + PAYLOAD_INDEX));
    return ValueOperand(typeReg, payloadReg);
}

bool
CodeGeneratorMIPS::visitValue(LValue *value)
{
    const ValueOperand out = ToOutValue(value);
    masm.moveValue(value->value(), out);
    return true;
}

bool
CodeGeneratorMIPS::visitOsrValue(LOsrValue *value)
{
    const LAllocation *frame   = value->getOperand(0);
    const ValueOperand out     = ToOutValue(value);
    const ptrdiff_t frameOffset = value->mir()->frameOffset();

    masm.loadValue(Operand(ToRegister(frame), frameOffset), out);
    return true;
}

bool
CodeGeneratorMIPS::visitBox(LBox *box)
{
    const LDefinition *type = box->getDef(TYPE_INDEX);

    DebugOnly<const LAllocation *> a = box->getOperand(0);
    JS_ASSERT(!a->isConstant());

    // On mips, the input operand and the output payload have the same
    // virtual register. All that needs to be written is the type tag for
    // the type definition.
    masm.movl(Imm32(MIRTypeToTag(box->type())), ToRegister(type));
    return true;
}

bool
CodeGeneratorMIPS::visitBoxDouble(LBoxDouble *box)
{
    const LAllocation *in = box->getOperand(0);
    const ValueOperand out = ToOutValue(box);

    masm.boxDouble(ToFloatRegister(in), out);
    return true;
}

bool
CodeGeneratorMIPS::visitUnbox(LUnbox *unbox)
{
    // Note that for unbox, the type and payload indexes are switched on the
    // inputs.
    MUnbox *mir = unbox->mir();

    if (mir->fallible()) {
        masm.cmpl(ToOperand(unbox->type()), Imm32(MIRTypeToTag(mir->type())));
        if (!bailoutIf(Assembler::NotEqual, unbox->snapshot()))
            return false;
    }
    return true;
}

bool
CodeGeneratorMIPS::visitLoadSlotV(LLoadSlotV *load)
{
    const ValueOperand out = ToOutValue(load);
    Register base = ToRegister(load->input());
    int32_t offset = load->mir()->slot() * sizeof(js::Value);

    masm.loadValue(Operand(base, offset), out);
    return true;
}

bool
CodeGeneratorMIPS::visitLoadSlotT(LLoadSlotT *load)
{
    Register base = ToRegister(load->input());
    int32_t offset = load->mir()->slot() * sizeof(js::Value);

    if (load->mir()->type() == MIRType_Double)
        masm.loadInt32OrDouble(Operand(base, offset), ToFloatRegister(load->output()));
    else
        masm.movl(Operand(base, offset + NUNBOX32_PAYLOAD_OFFSET), ToRegister(load->output()));
    return true;
}

bool
CodeGeneratorMIPS::visitStoreSlotT(LStoreSlotT *store)
{
    Register base = ToRegister(store->slots());
    int32_t offset = store->mir()->slot() * sizeof(js::Value);

    const LAllocation *value = store->value();
    MIRType valueType = store->mir()->value()->type();

    if (store->mir()->needsBarrier())
        emitPreBarrier(Address(base, offset), store->mir()->slotType());

    if (valueType == MIRType_Double) {
        masm.movsd(ToFloatRegister(value), Operand(base, offset));
        return true;
    }

    // Store the type tag if needed.
    if (valueType != store->mir()->slotType())
        masm.storeTypeTag(ImmType(ValueTypeFromMIRType(valueType)), Operand(base, offset));

    // Store the payload.
    if (value->isConstant())
        masm.storePayload(*value->toConstant(), Operand(base, offset));
    else
        masm.storePayload(ToRegister(value), Operand(base, offset));

    return true;
}

bool
CodeGeneratorMIPS::visitLoadElementT(LLoadElementT *load)
{
    Operand source = createArrayElementOperand(ToRegister(load->elements()), load->index());

    if (load->mir()->needsHoleCheck()) {
        Assembler::Condition cond = masm.testMagic(Assembler::Equal, source);
        if (!bailoutIf(cond, load->snapshot()))
            return false;
    }

    if (load->mir()->type() == MIRType_Double) {
        FloatRegister fpreg = ToFloatRegister(load->output());
        if (load->mir()->loadDoubles()) {
            if (source.kind() == Operand::REG_DISP)
                masm.loadDouble(source.toAddress(), fpreg);
            else
                masm.loadDouble(source.toBaseIndex(), fpreg);
        } else {
            masm.loadInt32OrDouble(source, fpreg);
        }
    } else {
        masm.movl(masm.ToPayload(source), ToRegister(load->output()));
    }

    return true;
}

void
CodeGeneratorMIPS::storeElementTyped(const LAllocation *value, MIRType valueType, MIRType elementType,
                                    const Register &elements, const LAllocation *index)
{
    Operand dest = createArrayElementOperand(elements, index);

    if (valueType == MIRType_Double) {
        masm.movsd(ToFloatRegister(value), dest);
        return;
    }

    // Store the type tag if needed.
    if (valueType != elementType)
        masm.storeTypeTag(ImmType(ValueTypeFromMIRType(valueType)), dest);

    // Store the payload.
    if (value->isConstant())
        masm.storePayload(*value->toConstant(), dest);
    else
        masm.storePayload(ToRegister(value), dest);
}

bool
CodeGeneratorMIPS::visitImplicitThis(LImplicitThis *lir)
{
    Register callee = ToRegister(lir->callee());
    const ValueOperand out = ToOutValue(lir);

    // The implicit |this| is always |undefined| if the function's environment
    // is the current global.
    GlobalObject *global = &gen->info().script()->global();
    masm.cmpPtr(Operand(callee, JSFunction::offsetOfEnvironment()), ImmGCPtr(global));

    if (!bailoutIf(Assembler::NotEqual, lir->snapshot()))
        return false;

    masm.moveValue(UndefinedValue(), out);
    return true;
}

typedef bool (*InterruptCheckFn)(JSContext *);
static const VMFunction InterruptCheckInfo = FunctionInfo<InterruptCheckFn>(InterruptCheck);

bool
CodeGeneratorMIPS::visitInterruptCheck(LInterruptCheck *lir)
{
    OutOfLineCode *ool = oolCallVM(InterruptCheckInfo, lir, (ArgList()), StoreNothing());
    if (!ool)
        return false;

    void *interrupt = (void*)&gen->compartment->rt->interrupt;
    masm.cmpl(Operand(interrupt), Imm32(0));
    masm.j(Assembler::NonZero, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitCompareB(LCompareB *lir)
{
    MCompare *mir = lir->mir();

    const ValueOperand lhs = ToValue(lir, LCompareB::Lhs);
    const LAllocation *rhs = lir->rhs();
    const Register output = ToRegister(lir->output());

    JS_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    Label notBoolean, done;
    // by wangqing, 2013-11-25
    masm.movl(ImmTag(JSVAL_TAG_BOOLEAN), cmpTempRegister);
    masm.bne(lhs.typeReg(), cmpTempRegister, &notBoolean);
    masm.nop();
    {
        if (rhs->isConstant())
            masm.cmp32(lhs.payloadReg(), Imm32(rhs->toConstant()->toBoolean()));
        else
            masm.cmp32(lhs.payloadReg(), ToRegister(rhs));
        masm.emitSet(JSOpToCondition(mir->compareType(), mir->jsop()), output);
	    masm.b(&done);
	    masm.nop();
    }
    masm.bindBranch(&notBoolean);
    {
        masm.move32(Imm32(mir->jsop() == JSOP_STRICTNE), output);
    }

    masm.bindBranch(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareBAndBranch(LCompareBAndBranch *lir)
{
    MCompare *mir = lir->mir();
    const ValueOperand lhs = ToValue(lir, LCompareBAndBranch::Lhs);
    const LAllocation *rhs = lir->rhs();

    JS_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    if (mir->jsop() == JSOP_STRICTEQ)
        masm.branchTestBoolean(Assembler::NotEqual, lhs, lir->ifFalse()->lir()->label());
    else
        masm.branchTestBoolean(Assembler::NotEqual, lhs, lir->ifTrue()->lir()->label());

    if (rhs->isConstant())
        masm.cmp32(lhs.payloadReg(), Imm32(rhs->toConstant()->toBoolean()));
    else
        masm.cmp32(lhs.payloadReg(), ToRegister(rhs));
    emitBranch(JSOpToCondition(mir->compareType(), mir->jsop()), lir->ifTrue(), lir->ifFalse());
    return true;
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitCompareV(LCompareV *lir)
{
    MCompare *mir = lir->mir();
    Assembler::Condition cond = JSOpToCondition(mir->compareType(), mir->jsop());
    const ValueOperand lhs = ToValue(lir, LCompareV::LhsInput);
    const ValueOperand rhs = ToValue(lir, LCompareV::RhsInput);
    const Register output = ToRegister(lir->output());

    JS_ASSERT(IsEqualityOp(mir->jsop()));

    Label notEqual, done;
    masm.bne(lhs.typeReg(), rhs.typeReg(), &notEqual);
    masm.nop();
    {
        masm.cmp32(lhs.payloadReg(), rhs.payloadReg());
        masm.emitSet(cond, output);
        masm.b(&done);
		masm.nop();
    }
    masm.bindBranch(&notEqual);
    {
        masm.move32(Imm32(cond == Assembler::NotEqual), output);
    }

    masm.bindBranch(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareVAndBranch(LCompareVAndBranch *lir)
{
    MCompare *mir = lir->mir();
    Assembler::Condition cond = JSOpToCondition(mir->compareType(), mir->jsop());
    const ValueOperand lhs = ToValue(lir, LCompareVAndBranch::LhsInput);
    const ValueOperand rhs = ToValue(lir, LCompareVAndBranch::RhsInput);

    JS_ASSERT(mir->jsop() == JSOP_EQ || mir->jsop() == JSOP_STRICTEQ ||
              mir->jsop() == JSOP_NE || mir->jsop() == JSOP_STRICTNE);

    Label *notEqual;
    if (cond == Assembler::Equal)
        notEqual = lir->ifFalse()->lir()->label();
    else
        notEqual = lir->ifTrue()->lir()->label();

    masm.cmp32(lhs.typeReg(), rhs.typeReg());
    masm.j(Assembler::NotEqual, notEqual);
    masm.cmp32(lhs.payloadReg(), rhs.payloadReg());
    emitBranch(cond, lir->ifTrue(), lir->ifFalse());

    return true;
}

bool
CodeGeneratorMIPS::visitUInt32ToDouble(LUInt32ToDouble *lir)
{
    Register input = ToRegister(lir->input());
    Register temp = ToRegister(lir->temp());

    if (input != temp)
        masm.mov(input, temp);

    // Beware: convertUInt32ToDouble clobbers input.
    masm.convertUInt32ToDouble(temp, ToFloatRegister(lir->output()));
    return true;
}

// Load a NaN or zero into a register for an out of bounds AsmJS or static
// typed array load.
class jit::OutOfLineLoadTypedArrayOutOfBounds : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    AnyRegister dest_;
  public:
    OutOfLineLoadTypedArrayOutOfBounds(AnyRegister dest) : dest_(dest) {}
    const AnyRegister &dest() const { return dest_; }
    bool accept(CodeGeneratorMIPS *codegen) { return codegen->visitOutOfLineLoadTypedArrayOutOfBounds(this); }
};

void
CodeGeneratorMIPS::loadViewTypeElement(ArrayBufferView::ViewType vt, const Address &srcAddr,
                                      const LDefinition *out)
{
    switch (vt) {
      case ArrayBufferView::TYPE_INT8:    masm.movxblWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_UINT8_CLAMPED:
      case ArrayBufferView::TYPE_UINT8:   masm.movzblWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_INT16:   masm.movxwlWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_UINT16:  masm.movzwlWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_INT32:   masm.movlWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_UINT32:  masm.movlWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_FLOAT64: masm.movsdWithPatch(srcAddr, ToFloatRegister(out)); break;
      default: JS_NOT_REACHED("unexpected array type");
    }
}

bool
CodeGeneratorMIPS::visitLoadTypedArrayElementStatic(LLoadTypedArrayElementStatic *ins)
{
    const MLoadTypedArrayElementStatic *mir = ins->mir();
    ArrayBufferView::ViewType vt = mir->viewType();

    Register ptr = ToRegister(ins->ptr());
    const LDefinition *out = ins->output();

    OutOfLineLoadTypedArrayOutOfBounds *ool = NULL;
    if (!mir->fallible()) {
        ool = new OutOfLineLoadTypedArrayOutOfBounds(ToAnyRegister(out));
        if (!addOutOfLineCode(ool))
            return false;
    }

    masm.cmpl(ptr, Imm32(mir->length()));
    if (ool)
        masm.j(Assembler::AboveOrEqual, ool->entry());
    else if (!bailoutIf(Assembler::AboveOrEqual, ins->snapshot()))
        return false;

    Address srcAddr(ptr, (int32_t) mir->base());
    if (vt == ArrayBufferView::TYPE_FLOAT32) {
        FloatRegister dest = ToFloatRegister(out);
        masm.movssWithPatch(srcAddr, dest);
        masm.cvtss2sd(dest, dest);
        masm.canonicalizeDouble(dest);
        if (ool)
            masm.bind(ool->rejoin());
        return true;
    }
    loadViewTypeElement(vt, srcAddr, out);
    if (vt == ArrayBufferView::TYPE_FLOAT64)
        masm.canonicalizeDouble(ToFloatRegister(out));
    if (ool)
        masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitAsmJSLoadHeap(LAsmJSLoadHeap *ins)
{
    // This is identical to LoadTypedArrayElementStatic, except that the
    // array's base and length are not known ahead of time and can be patched
    // later on, and the instruction is always infallible.
    const MAsmJSLoadHeap *mir = ins->mir();
    ArrayBufferView::ViewType vt = mir->viewType();

    Register ptr = ToRegister(ins->ptr());
    const LDefinition *out = ins->output();

    OutOfLineLoadTypedArrayOutOfBounds *ool = new OutOfLineLoadTypedArrayOutOfBounds(ToAnyRegister(out));
    if (!addOutOfLineCode(ool))
        return false;

    CodeOffsetLabel cmp = masm.cmplWithPatch(ptr, Imm32(0));
    masm.j(Assembler::AboveOrEqual, ool->entry());

    Address srcAddr(ptr, 0);
    if (vt == ArrayBufferView::TYPE_FLOAT32) {
        FloatRegister dest = ToFloatRegister(out);
        uint32_t before = masm.size();
        masm.movssWithPatch(srcAddr, dest);
        uint32_t after = masm.size();
        masm.cvtss2sd(dest, dest);
        masm.bind(ool->rejoin());
        return gen->noteHeapAccess(AsmJSHeapAccess(cmp.offset(), before, after, vt, AnyRegister(dest)));
    }
    uint32_t before = masm.size();
    loadViewTypeElement(vt, srcAddr, out);
    uint32_t after = masm.size();
    masm.bind(ool->rejoin());
    bool temp= gen->noteHeapAccess(AsmJSHeapAccess(cmp.offset(), before, after, vt, ToAnyRegister(out)));
    return temp;
}

bool
CodeGeneratorMIPS::visitOutOfLineLoadTypedArrayOutOfBounds(OutOfLineLoadTypedArrayOutOfBounds *ool)
{
    if (ool->dest().isFloat()) {
        masm.movsd(&js_NaN, ool->dest().fpu());
    } else {
        Register destReg = ool->dest().gpr();
        masm.xorl(destReg, destReg);
    }
    masm.jmp(ool->rejoin());
    return true;
}

void
CodeGeneratorMIPS::storeViewTypeElement(ArrayBufferView::ViewType vt, const LAllocation *value,
                                       const Address &dstAddr)
{
    switch (vt) {
      case ArrayBufferView::TYPE_INT8:    masm.movbWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_UINT8_CLAMPED:
      case ArrayBufferView::TYPE_UINT8:   masm.movbWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_INT16:   masm.movwWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_UINT16:  masm.movwWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_INT32:   masm.movlWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_UINT32:  masm.movlWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_FLOAT64: masm.movsdWithPatch(ToFloatRegister(value), dstAddr); break;
      default: JS_NOT_REACHED("unexpected array type");
    }
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitStoreTypedArrayElementStatic(LStoreTypedArrayElementStatic *ins)
{
    MStoreTypedArrayElementStatic *mir = ins->mir();
    ArrayBufferView::ViewType vt = mir->viewType();

    Register ptr = ToRegister(ins->ptr());
    const LAllocation *value = ins->value();

    masm.cmpl(ptr, Imm32(mir->length()));
    Label rejoin;
    masm.sltu(cmpTempRegister, cmpTempRegister, cmpTemp2Register);
    masm.blez(cmpTempRegister, &rejoin);
    masm.nop();

    Address dstAddr(ptr, (int32_t) mir->base());
    if (vt == ArrayBufferView::TYPE_FLOAT32) {
        masm.convertDoubleToFloat(ToFloatRegister(value), ScratchFloatReg);
        masm.movssWithPatch(ScratchFloatReg, dstAddr);
        masm.bindBranch(&rejoin);
        return true;
    }
    storeViewTypeElement(vt, value, dstAddr);
    masm.bindBranch(&rejoin);
    return true;
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitAsmJSStoreHeap(LAsmJSStoreHeap *ins)
{
    // This is identical to StoreTypedArrayElementStatic, except that the
    // array's base and length are not known ahead of time and can be patched
    // later on.
    MAsmJSStoreHeap *mir = ins->mir();
    ArrayBufferView::ViewType vt = mir->viewType();

    Register ptr = ToRegister(ins->ptr());
    const LAllocation *value = ins->value();

    CodeOffsetLabel cmp = masm.cmplWithPatch(ptr, Imm32(0));
    Label rejoin;
    masm.sltu(cmpTempRegister, cmpTempRegister, cmpTemp2Register);
    masm.blez(cmpTempRegister, &rejoin);
    masm.nop();

    Address dstAddr(ptr, 0);
    if (vt == ArrayBufferView::TYPE_FLOAT32) {
        masm.convertDoubleToFloat(ToFloatRegister(value), ScratchFloatReg);
        uint32_t before = masm.size();
        masm.movssWithPatch(ScratchFloatReg, dstAddr);
        uint32_t after = masm.size();
        masm.bindBranch(&rejoin);
        return gen->noteHeapAccess(AsmJSHeapAccess(cmp.offset(), before, after));
    }
    uint32_t before = masm.size();
    storeViewTypeElement(vt, value, dstAddr);
    uint32_t after = masm.size();
    masm.bindBranch(&rejoin);
    bool temp= gen->noteHeapAccess(AsmJSHeapAccess(cmp.offset(), before, after));
	return temp;
}

bool
CodeGeneratorMIPS::visitAsmJSLoadGlobalVar(LAsmJSLoadGlobalVar *ins)
{
    MAsmJSLoadGlobalVar *mir = ins->mir();

    CodeOffsetLabel label;
    if (mir->type() == MIRType_Int32)
        label = masm.movlWithPatch(NULL, ToRegister(ins->output()));
    else
        label = masm.movsdWithPatch(NULL, ToFloatRegister(ins->output()));

    return gen->noteGlobalAccess(label.offset(), mir->globalDataOffset());
}

bool
CodeGeneratorMIPS::visitAsmJSStoreGlobalVar(LAsmJSStoreGlobalVar *ins)
{
    MAsmJSStoreGlobalVar *mir = ins->mir();

    MIRType type = mir->value()->type();
    JS_ASSERT(type == MIRType_Int32 || type == MIRType_Double);

    CodeOffsetLabel label;
    if (type == MIRType_Int32)
        label = masm.movlWithPatch(ToRegister(ins->value()), NULL);
    else
        label = masm.movsdWithPatch(ToFloatRegister(ins->value()), NULL);

    return gen->noteGlobalAccess(label.offset(), mir->globalDataOffset());
}

bool
CodeGeneratorMIPS::visitAsmJSLoadFuncPtr(LAsmJSLoadFuncPtr *ins)
{
    MAsmJSLoadFuncPtr *mir = ins->mir();

    Register index = ToRegister(ins->index());
    Register out = ToRegister(ins->output());
    CodeOffsetLabel label = masm.movlWithPatch(NULL, index, TimesFour, out);

    return gen->noteGlobalAccess(label.offset(), mir->globalDataOffset());
}

bool
CodeGeneratorMIPS::visitAsmJSLoadFFIFunc(LAsmJSLoadFFIFunc *ins)
{
    MAsmJSLoadFFIFunc *mir = ins->mir();

    Register out = ToRegister(ins->output());
    CodeOffsetLabel label = masm.movlWithPatch(NULL, out);

    return gen->noteGlobalAccess(label.offset(), mir->globalDataOffset());
}

void
CodeGeneratorMIPS::postAsmJSCall(LAsmJSCall *lir)
{
    ASSERT(0);
    return;
	//no need to move double return value
	//to ReturnFloatReg=f0,
	//since it has been in f0,due to ABI
}


void
ParallelGetPropertyIC::initializeAddCacheState(LInstruction *ins, AddCacheState *addState)
{
    // We don't have a scratch register, but only use the temp if we needed
    // one, it's BogusTemp otherwise.
    JS_ASSERT(ins->isGetPropertyCacheV() || ins->isGetPropertyCacheT());
    if (ins->isGetPropertyCacheV() || ins->toGetPropertyCacheT()->temp()->isBogusTemp())
        addState->dispatchScratch = output_.scratchReg().gpr();
    else
        addState->dispatchScratch = ToRegister(ins->toGetPropertyCacheT()->temp());
}


class jit::OutOfLineTruncate : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    LTruncateDToInt32 *ins_;

  public:
    OutOfLineTruncate(LTruncateDToInt32 *ins)
      : ins_(ins)
    { }

    bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitOutOfLineTruncate(this);
    }
    LTruncateDToInt32 *ins() const {
        return ins_;
    }
};


bool
CodeGeneratorMIPS::visitTruncateDToInt32(LTruncateDToInt32 *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    Register output = ToRegister(ins->output());

    OutOfLineTruncate *ool = new OutOfLineTruncate(ins);
    if (!addOutOfLineCode(ool))
        return false;

    masm.branchTruncateDouble(input, output, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

// by wangqing, 2013-11-21
bool
CodeGeneratorMIPS::visitOutOfLineTruncate(OutOfLineTruncate *ool)
{
    LTruncateDToInt32 *ins = ool->ins();
    FloatRegister input = ToFloatRegister(ins->input());
    Register output = ToRegister(ins->output());

    Label fail;  // by wangqing, 2013-11-27

    FloatRegister temp = ToFloatRegister(ins->tempFloat());

    // Try to convert doubles representing integers within 2^32 of a signed
    // integer, by adding/subtracting 2^32 and then trying to convert to int32.
    // This has to be an exact conversion, as otherwise the truncation works
    // incorrectly on the modified value.

    //by weizhenwei, 2013.11.05
    masm.zerod(ScratchFloatReg);
    //DoubleUnordered check, by wangqing, 2013.11.27
    masm.cud(input, ScratchFloatReg);
    masm.bc1t(&fail);
    masm.nop();

    {
        Label positive; // by wangqing, 2013-11-27
        //DoubleGreaterThan check, by wangqing, 2013.11.27
        masm.cngtd(input, ScratchFloatReg);
        masm.bc1f(&positive);
        masm.nop();

        static const double shiftNeg = 4294967296.0;
        masm.loadStaticDouble(&shiftNeg, temp);
        Label skip;
        masm.b(&skip);
                    masm.nop();

        masm.bindBranch(&positive);
        static const double shiftPos = -4294967296.0;
        masm.loadStaticDouble(&shiftPos, temp);
        masm.bindBranch(&skip);
    }
    masm.addsd(input, temp);
    masm.cvttsd2si(temp, output);
    masm.cvtsi2sd(output, ScratchFloatReg);

    //DoubleUnordered check, by wangqing, 2013.11.27
    masm.cud(temp, ScratchFloatReg);
    masm.bc1t(&fail);
    masm.nop();

    masm.branchDouble(Assembler::DoubleEqual, temp, ScratchFloatReg, ool->rejoin());

    masm.bindBranch(&fail);
    {
        saveVolatile(output);
        
        masm.setupUnalignedABICall(1, output);
        masm.passABIArg(input);
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, js::ToInt32));
        masm.storeCallResult(output);

        restoreVolatile(output);
    }

    masm.jump(ool->rejoin());
    return true;
}

