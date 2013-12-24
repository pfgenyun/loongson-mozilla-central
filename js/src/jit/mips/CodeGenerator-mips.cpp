/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/mips/CodeGenerator-mips.h"

// From jit/shared/CodeGenerator-x86-shared.cpp
#include "mozilla/DebugOnly.h"
#include "mozilla/MathAlgorithms.h"

#include "jit/IonFrames.h"
#include "jit/JitCompartment.h"
#include "jit/RangeAnalysis.h"
#include "jit/shared/CodeGenerator-shared-inl.h"

// From jit/x86/CodeGenerator-x86.cpp
#include "jsnum.h"
#include "jit/ExecutionModeInlines.h"
#include "jit/IonCaches.h"
#include "jit/MIR.h"
#include "jit/MIRGraph.h"
#include "vm/Shape.h"
#include "jsscriptinlines.h"

using namespace js;
using namespace js::jit;

// From jit/x86/CodeGenerator-x86.cpp
using mozilla::DebugOnly;
using mozilla::DoubleExponentBias;
using mozilla::DoubleExponentShift;
using mozilla::FloatExponentBias;
using mozilla::FloatExponentShift;
using mozilla::FloatExponentBits;
using JS::GenericNaN;

using mozilla::DoubleSignificandBits;
using mozilla::FloatSignificandBits;
using mozilla::FloorLog2;
using mozilla::NegativeInfinity;
using mozilla::SpecificNaN;
using mozilla::SpecificFloatNaN;

namespace js {
namespace jit {
// Following is copy from jit/shared/CodeGenerator-x86-shared.cpp
CodeGeneratorMIPS::CodeGeneratorMIPS(MIRGenerator *gen, LIRGraph *graph, MacroAssembler *masm)
  : CodeGeneratorShared(gen, graph, masm)
{
}

bool
CodeGeneratorMIPS::generatePrologue()
{
    // Note that this automatically sets MacroAssembler::framePushed().
    masm.reserveStack(frameSize());

    return true;
}

bool
CodeGeneratorMIPS::generateEpilogue()
{
    masm.bind(&returnLabel_);

#if JS_TRACE_LOGGING
    masm.tracelogStop();
#endif

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
    if (ifNaN == Assembler::NaN_IsFalse)
        jumpToBlock(mirFalse, Assembler::Parity);
    else if (ifNaN == Assembler::NaN_IsTrue)
        jumpToBlock(mirTrue, Assembler::Parity);

    if (isNextBlock(mirFalse->lir())) {
        jumpToBlock(mirTrue, cond);
    } else {
        jumpToBlock(mirFalse, Assembler::InvertCondition(cond));
        jumpToBlock(mirTrue);
    }
}

// By weizhenwei, 2013.12.23
// From jit/shared/CodeGenerator-shared.cpp
void
CodeGeneratorMIPS::jumpToBlockDouble(MBasicBlock *mir, const FloatRegister &lhs,
        const FloatRegister &rhs, Assembler::DoubleCondition cond)
{
    //TODO: check the if branch, by weizhenwei, 2013.12.24
    if (Label *oolEntry = labelForBackedgeWithImplicitCheck(mir)) {
        // Note: the backedge is initially a jump to the next instruction.
        // It will be patched to the target block's label during link().
        RepatchLabel rejoin;
        CodeOffsetJump backedge = masm.jumpWithPatch(&rejoin, lhs, rhs, cond);
        masm.bind(&rejoin);

        masm.propagateOOM(patchableBackedges_.append(PatchableBackedgeInfo(backedge, mir->lir()->label(), oolEntry)));
    } else {
        masm.branchDouble(cond, lhs, rhs, mir->lir()->label());
    }
}
void
CodeGeneratorMIPS::emitBranch(Assembler::DoubleCondition cond,
        const FloatRegister &lhs, const FloatRegister &rhs,
        MBasicBlock *mirTrue, MBasicBlock *mirFalse, Assembler::NaNCond ifNaN)
{
    if (ifNaN == Assembler::NaN_IsFalse) {
        //jumpToBlock(mirFalse, Assembler::Parity);
        //masm.branchDouble(Assembler::DoubleUnordered, lhs, rhs, ifFalse->label());
        jumpToBlockDouble(mirFalse, lhs, rhs, Assembler::DoubleUnordered);
    } else if (ifNaN == Assembler::NaN_IsTrue) {
        //jumpToBlock(mirTrue, Assembler::Parity);
        //masm.branchDouble(Assembler::DoubleUnordered, lhs, rhs, ifTrue->label());
        jumpToBlockDouble(mirTrue, lhs, rhs, Assembler::DoubleUnordered);
    }

    if (isNextBlock(mirFalse->lir())) {
        //jumpToBlock(mirTrue, cond);
        //masm.branchDouble(cond, lhs, rhs, ifTrue->label());
        jumpToBlockDouble(mirTrue, lhs, rhs, cond);
    } else {
        //jumpToBlock(mirFalse, Assembler::InvertCondition(cond));
        //jumpToBlock(mirTrue);
        //TODO:check this jump is right, by weizhenwei, 2013.12.24
        masm.j(Assembler::InvertCondition(masm.ConditionFromDoubleCondition(cond)),
                mirFalse->lir()->label());
        if (!isNextBlock(mirTrue->lir()))
            masm.jmp(mirTrue->lir()->label());
    }
}

//by weizhenwei, 2013.11.07
void
CodeGeneratorMIPS::emitSet(Assembler::DoubleCondition cond, const FloatRegister &lhs,
        const FloatRegister &rhs, const Register &dest, Assembler::NaNCond ifNaN) {


    if (GeneralRegisterSet(Registers::SingleByteRegs).has(dest)) {
        // If the register we're defining is a single byte register,
        // take advantage of the setCC instruction
        //setCC(cond, dest);
        //movzxbl(dest, dest);
        Label setDest;
        masm.branchDouble(cond, lhs, rhs, &setDest);
        masm.xorl(dest, dest);
        masm.bind(&setDest);
        masm.movl(Imm32(1), dest);

        if (ifNaN != Assembler::NaN_HandledByCond) {
            Label noNaN;
           //j(Assembler::NoParity, &noNaN);
           masm.branchDouble(Assembler::DoubleOrdered, lhs, rhs, &noNaN);
            if (ifNaN == Assembler::NaN_IsTrue)
                masm.movl(Imm32(1), dest);
            else
                masm.xorl(dest, dest);
            masm.bind(&noNaN);
        }
    } else {
        Label end;
        Label ifFalse;

        if (ifNaN == Assembler::NaN_IsFalse) {
            //j(Assembler::Parity, &ifFalse);
            masm.branchDouble(Assembler::DoubleUnordered, lhs, rhs, &ifFalse);
        }
        masm.movl(Imm32(1), dest);
        //j(cond, &end);
        masm.branchDouble(cond, lhs, rhs, &end);
        if (ifNaN == Assembler::NaN_IsTrue) {
           //j(Assembler::Parity, &end);
            masm.branchDouble(Assembler::DoubleUnordered, lhs, rhs, &end);
        }
          
        masm.bind(&ifFalse);
        masm.xorl(dest, dest);

        masm.bind(&end);
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
CodeGeneratorMIPS::visitFloat32(LFloat32 *ins)
{
    const LDefinition *out = ins->getDef(0);
    masm.loadConstantFloat32(ins->getFloat(), ToFloatRegister(out));
    return true;
}

bool
CodeGeneratorMIPS::visitTestIAndBranch(LTestIAndBranch *test)
{
    const LAllocation *opd = test->input();

    // Test the operand
    masm.testl(ToRegister(opd), ToRegister(opd));
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
    masm.zerod(ScratchFloatReg);
    //masm.ucomisd(ToFloatRegister(opd), ScratchFloatReg);
    //emitBranch(Assembler::NotEqual, test->ifTrue(), test->ifFalse());
    emitBranch(Assembler::DoubleNotEqual, ToFloatRegister(opd),
            ScratchFloatReg, test->ifTrue(), test->ifFalse());
    return true;
}

bool
CodeGeneratorMIPS::visitTestFAndBranch(LTestFAndBranch *test)
{
    const LAllocation *opd = test->input();
    // ucomiss flags are the same as doubles; see comment above
    masm.xorps(ScratchFloatReg, ScratchFloatReg);
    masm.ucomiss(ToFloatRegister(opd), ScratchFloatReg);
    emitBranch(Assembler::NotEqual, test->ifTrue(), test->ifFalse());
    return true;
}

bool
CodeGeneratorMIPS::visitBitAndAndBranch(LBitAndAndBranch *baab)
{
    if (baab->right()->isConstant())
        masm.testl(ToRegister(baab->left()), Imm32(ToInt32(baab->right())));
    else
        masm.testl(ToRegister(baab->left()), ToRegister(baab->right()));
    emitBranch(Assembler::NonZero, baab->ifTrue(), baab->ifFalse());
    return true;
}

void
CodeGeneratorMIPS::emitCompare(MCompare::CompareType type,
        const LAllocation *left, const LAllocation *right)
{
#ifdef JS_CPU_X64
    if (type == MCompare::Compare_Object) {
        masm.cmpq(ToRegister(left), ToOperand(right));
        return;
    }
#endif

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
    MCompare *mir = comp->cmpMir();
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

    Assembler::NaNCond nanCond = Assembler::NaNCondFromDoubleCondition(cond);
    if (comp->mir()->operandsAreNeverNaN())
        nanCond = Assembler::NaN_HandledByCond;

    masm.compareDouble(cond, lhs, rhs);
    masm.emitSet(Assembler::ConditionFromDoubleCondition(cond), ToRegister(comp->output()), nanCond);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareF(LCompareF *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->mir()->jsop());

    Assembler::NaNCond nanCond = Assembler::NaNCondFromDoubleCondition(cond);
    if (comp->mir()->operandsAreNeverNaN())
        nanCond = Assembler::NaN_HandledByCond;

    masm.compareFloat(cond, lhs, rhs);
    masm.emitSet(Assembler::ConditionFromDoubleCondition(cond), ToRegister(comp->output()), nanCond);
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

    // Not returns true if the input is a NaN. We don't have to worry about
    // it if we know the input is never NaN though.
    Assembler::NaNCond nanCond = Assembler::NaN_IsTrue;
    if (ins->mir()->operandIsNeverNaN())
        nanCond = Assembler::NaN_HandledByCond;

    masm.xorpd(ScratchFloatReg, ScratchFloatReg);
    masm.compareDouble(Assembler::DoubleEqualOrUnordered, opd, ScratchFloatReg);
    masm.emitSet(Assembler::Equal, ToRegister(ins->output()), nanCond);
    return true;
}

bool
CodeGeneratorMIPS::visitNotF(LNotF *ins)
{
    FloatRegister opd = ToFloatRegister(ins->input());

    // Not returns true if the input is a NaN. We don't have to worry about
    // it if we know the input is never NaN though.
    Assembler::NaNCond nanCond = Assembler::NaN_IsTrue;
    if (ins->mir()->operandIsNeverNaN())
        nanCond = Assembler::NaN_HandledByCond;

    masm.xorps(ScratchFloatReg, ScratchFloatReg);
    masm.compareFloat(Assembler::DoubleEqualOrUnordered, opd, ScratchFloatReg);
    masm.emitSet(Assembler::Equal, ToRegister(ins->output()), nanCond);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareDAndBranch(LCompareDAndBranch *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->cmpMir()->jsop());

    Assembler::NaNCond nanCond = Assembler::NaNCondFromDoubleCondition(cond);
    if (comp->cmpMir()->operandsAreNeverNaN())
        nanCond = Assembler::NaN_HandledByCond;

    masm.compareDouble(cond, lhs, rhs);
    emitBranch(Assembler::ConditionFromDoubleCondition(cond), comp->ifTrue(), comp->ifFalse(), nanCond);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareFAndBranch(LCompareFAndBranch *comp)
{
    FloatRegister lhs = ToFloatRegister(comp->left());
    FloatRegister rhs = ToFloatRegister(comp->right());

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(comp->cmpMir()->jsop());

    Assembler::NaNCond nanCond = Assembler::NaNCondFromDoubleCondition(cond);
    if (comp->cmpMir()->operandsAreNeverNaN())
        nanCond = Assembler::NaN_HandledByCond;

    masm.compareFloat(cond, lhs, rhs);
    emitBranch(Assembler::ConditionFromDoubleCondition(cond), comp->ifTrue(), comp->ifFalse(), nanCond);
    return true;
}

bool
CodeGeneratorMIPS::visitAsmJSPassStackArg(LAsmJSPassStackArg *ins)
{
    const MAsmJSPassStackArg *mir = ins->mir();
    Address dst(StackPointer, mir->spOffset());
    if (ins->arg()->isConstant()) {
        masm.storePtr(ImmWord(ToInt32(ins->arg())), dst);
    } else {
        if (ins->arg()->isGeneralReg())
            masm.storePtr(ToRegister(ins->arg()), dst);
        else
            masm.storeDouble(ToFloatRegister(ins->arg()), dst);
    }
    return true;
}

bool
CodeGeneratorMIPS::generateOutOfLineCode()
{
    if (!CodeGeneratorShared::generateOutOfLineCode())
        return false;

    if (deoptLabel_.used()) {
        // All non-table-based bailouts will go here.
        masm.bind(&deoptLabel_);

        // Push the frame size, so the handler can recover the IonScript.
        masm.push(Imm32(frameSize()));

        IonCode *handler = gen->jitRuntime()->getGenericBailoutHandler();
        masm.jmp(ImmPtr(handler->raw()), Relocation::IONCODE);
    }

    return true;
}

class BailoutJump {
    Assembler::Condition cond_;

  public:
    BailoutJump(Assembler::Condition cond) : cond_(cond)
    { }
#ifdef JS_CPU_X86
    void operator()(MacroAssembler &masm, uint8_t *code) const {
        masm.j(cond_, ImmPtr(code), Relocation::HARDCODED);
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
#ifdef JS_CPU_X86
    void operator()(MacroAssembler &masm, uint8_t *code) const {
        masm.retarget(label_, ImmPtr(code), Relocation::HARDCODED);
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
        OutOfLineAbortPar *ool = oolAbortPar(ParallelBailoutUnsupported,
                                             snapshot->mir()->block(),
                                             snapshot->mir()->pc());
        binder(masm, ool->entry());
        return true;
      }
      case SequentialExecution:
        break;
      default:
        MOZ_ASSUME_UNREACHABLE("No such execution mode");
    }

    if (!encode(snapshot))
        return false;

    // Though the assembler doesn't track all frame pushes, at least make sure
    // the known value makes sense. We can't use bailout tables if the stack
    // isn't properly aligned to the static frame size.
    JS_ASSERT_IF(frameClass_ != FrameSizeClass::None() && deoptTable_,
                 frameClass_.frameSize() == masm.framePushed());

#ifdef JS_CPU_X86
    // On x64, bailout tables are pointless, because 16 extra bytes are
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
    OutOfLineBailout *ool = new(alloc()) OutOfLineBailout(snapshot);
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
CodeGeneratorMIPS::bailoutIf(Assembler::DoubleCondition condition, LSnapshot *snapshot)
{
    JS_ASSERT(Assembler::NaNCondFromDoubleCondition(condition) == Assembler::NaN_HandledByCond);
    return bailoutIf(Assembler::ConditionFromDoubleCondition(condition), snapshot);
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
    masm.push(Imm32(ool->snapshot()->snapshotOffset()));
    masm.jmp(&deoptLabel_);
    return true;
}


bool
CodeGeneratorMIPS::visitMinMaxD(LMinMaxD *ins)
{
    FloatRegister first = ToFloatRegister(ins->first());
    FloatRegister second = ToFloatRegister(ins->second());
#ifdef DEBUG
    FloatRegister output = ToFloatRegister(ins->output());
    JS_ASSERT(first == output);
#endif

    Label done, nan, minMaxInst;

    // Do a ucomisd to catch equality and NaNs, which both require special
    // handling. If the operands are ordered and inequal, we branch straight to
    // the min/max instruction. If we wanted, we could also branch for less-than
    // or greater-than here instead of using min/max, however these conditions
    // will sometimes be hard on the branch predictor.
    masm.ucomisd(first, second);
    masm.j(Assembler::NotEqual, &minMaxInst);
    if (!ins->mir()->range() || ins->mir()->range()->canBeNaN())
        masm.j(Assembler::Parity, &nan);

    // Ordered and equal. The operands are bit-identical unless they are zero
    // and negative zero. These instructions merge the sign bits in that
    // case, and are no-ops otherwise.
    if (ins->mir()->isMax())
        masm.andpd(second, first);
    else
        masm.orpd(second, first);
    masm.jump(&done);

    // x86's min/max are not symmetric; if either operand is a NaN, they return
    // the read-only operand. We need to return a NaN if either operand is a
    // NaN, so we explicitly check for a NaN in the read-write operand.
    if (!ins->mir()->range() || ins->mir()->range()->canBeNaN()) {
        masm.bind(&nan);
        masm.ucomisd(first, first);
        masm.j(Assembler::Parity, &done);
    }

    // When the values are inequal, or second is NaN, x86's min and max will
    // return the value we need.
    masm.bind(&minMaxInst);
    if (ins->mir()->isMax())
        masm.maxsd(second, first);
    else
        masm.minsd(second, first);

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitAbsD(LAbsD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));
    // Load a value which is all ones except for the sign bit.
    masm.loadConstantDouble(SpecificNaN(0, DoubleSignificandBits), ScratchFloatReg);
    masm.andpd(ScratchFloatReg, input);
    return true;
}

bool
CodeGeneratorMIPS::visitAbsF(LAbsF *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));
    // Same trick as visitAbsD above.
    masm.loadConstantFloat32(SpecificFloatNaN(0, FloatSignificandBits), ScratchFloatReg);
    masm.andps(ScratchFloatReg, input);
    return true;
}

bool
CodeGeneratorMIPS::visitSqrtD(LSqrtD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    FloatRegister output = ToFloatRegister(ins->output());
    masm.sqrtsd(input, output);
    return true;
}

bool
CodeGeneratorMIPS::visitSqrtF(LSqrtF *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    FloatRegister output = ToFloatRegister(ins->output());
    masm.sqrtss(input, output);
    return true;
}

bool
CodeGeneratorMIPS::visitPowHalfD(LPowHalfD *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));

    Label done, sqrt;

    if (!ins->mir()->operandIsNeverNegativeInfinity()) {
        // Branch if not -Infinity.
        masm.loadConstantDouble(NegativeInfinity(), ScratchFloatReg);

        Assembler::DoubleCondition cond = Assembler::DoubleNotEqualOrUnordered;
        if (ins->mir()->operandIsNeverNaN())
            cond = Assembler::DoubleNotEqual;
        masm.branchDouble(cond, input, ScratchFloatReg, &sqrt);

        // Math.pow(-Infinity, 0.5) == Infinity.
        masm.xorpd(input, input);
        masm.subsd(ScratchFloatReg, input);
        masm.jump(&done);

        masm.bind(&sqrt);
    }

    if (!ins->mir()->operandIsNeverNegativeZero()) {
        // Math.pow(-0, 0.5) == 0 == Math.pow(0, 0.5). Adding 0 converts any -0 to 0.
        masm.xorpd(ScratchFloatReg, ScratchFloatReg);
        masm.addsd(ScratchFloatReg, input);
    }

    masm.sqrtsd(input, input);

    masm.bind(&done);
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
    if (ins->rhs()->isConstant())
        masm.addl(Imm32(ToInt32(ins->rhs())), ToOperand(ins->lhs()));
    else
        masm.addl(ToOperand(ins->rhs()), ToRegister(ins->lhs()));

    if (ins->snapshot()) {
        if (ins->recoversInput()) {
            OutOfLineUndoALUOperation *ool = new(alloc()) OutOfLineUndoALUOperation(ins);
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
    if (ins->rhs()->isConstant())
        masm.subl(Imm32(ToInt32(ins->rhs())), ToOperand(ins->lhs()));
    else
        masm.subl(ToOperand(ins->rhs()), ToRegister(ins->lhs()));

    if (ins->snapshot()) {
        if (ins->recoversInput()) {
            OutOfLineUndoALUOperation *ool = new(alloc()) OutOfLineUndoALUOperation(ins);
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
CodeGeneratorMIPS::visitOutOfLineUndoALUOperation(OutOfLineUndoALUOperation *ool)
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
            masm.testl(ToRegister(lhs), ToRegister(lhs));
            if (!bailoutIf(bailoutCond, ins->snapshot()))
                    return false;
        }

        switch (constant) {
          case -1:
            masm.negl(ToOperand(lhs));
            break;
          case 0:
            masm.xorl(ToOperand(lhs), ToRegister(lhs));
            return true; // escape overflow check;
          case 1:
            // nop
            return true; // escape overflow check;
          case 2:
            masm.addl(ToOperand(lhs), ToRegister(lhs));
            break;
          default:
            if (!mul->canOverflow() && constant > 0) {
                // Use shift if cannot overflow and constant is power of 2
                int32_t shift = FloorLog2(constant);
                if ((1 << shift) == constant) {
                    masm.shll(Imm32(shift), ToRegister(lhs));
                    return true;
                }
            }
            masm.imull(Imm32(ToInt32(rhs)), ToRegister(lhs));
        }

        // Bailout on overflow
        if (mul->canOverflow() && !bailoutIf(Assembler::Overflow, ins->snapshot()))
            return false;
    } else {
        masm.imull(ToOperand(rhs), ToRegister(lhs));

        // Bailout on overflow
        if (mul->canOverflow() && !bailoutIf(Assembler::Overflow, ins->snapshot()))
            return false;

        if (mul->canBeNegativeZero()) {
            // Jump to an OOL path if the result is 0.
            MulNegativeZeroCheck *ool = new(alloc()) MulNegativeZeroCheck(ins);
            if (!addOutOfLineCode(ool))
                return false;

            masm.testl(ToRegister(lhs), ToRegister(lhs));
            masm.j(Assembler::Zero, ool->entry());
            masm.bind(ool->rejoin());
        }
    }

    return true;
}

class ReturnZero : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    Register reg_;

  public:
    explicit ReturnZero(Register reg)
      : reg_(reg)
    { }

    virtual bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitReturnZero(this);
    }
    Register reg() const {
        return reg_;
    }
};

bool
CodeGeneratorMIPS::visitReturnZero(ReturnZero *ool)
{
    masm.mov(ImmWord(0), ool->reg());
    masm.jmp(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitUDivOrMod(LUDivOrMod *ins)
{
    JS_ASSERT(ToRegister(ins->lhs()) == t6/*eax*/);
    Register rhs = ToRegister(ins->rhs());
    Register output = ToRegister(ins->output());

    JS_ASSERT_IF(output == t6/*eax*/, ToRegister(ins->remainder()) == t7/*edx*/);

    ReturnZero *ool = nullptr;

    // Prevent divide by zero.
    if (ins->canBeDivideByZero()) {
        masm.testl(rhs, rhs);
        if (ins->mir()->isTruncated()) {
            if (!ool)
                ool = new(alloc()) ReturnZero(output);
            masm.j(Assembler::Zero, ool->entry());
        } else {
            if (!bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
        }
    }

    masm.mov(ImmWord(0), t7/*edx*/);
    masm.udiv(rhs);

    // Unsigned div or mod can return a value that's not a signed int32.
    // If our users aren't expecting that, bail.
    if (!ins->mir()->isTruncated()) {
        masm.testl(output, output);
        if (!bailoutIf(Assembler::Signed, ins->snapshot()))
            return false;
    }

    if (ool) {
        if (!addOutOfLineCode(ool))
            return false;
        masm.bind(ool->rejoin());
    }

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
    if (!bailoutIf(Assembler::Signed, ins->snapshot()))
        return false;

    masm.mov(ImmWord(0), result);
    masm.jmp(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitDivPowTwoI(LDivPowTwoI *ins)
{
    Register lhs = ToRegister(ins->numerator());
    mozilla::DebugOnly<Register> output = ToRegister(ins->output());
    int32_t shift = ins->shift();

    // We use defineReuseInput so these should always be the same, which is
    // convenient since all of our instructions here are two-address.
    JS_ASSERT(lhs == output);

    if (shift != 0) {
        MDiv *mir = ins->mir();
        if (!mir->isTruncated()) {
            // If the remainder is != 0, bailout since this must be a double.
            masm.testl(lhs, Imm32(UINT32_MAX >> (32 - shift)));
            if (!bailoutIf(Assembler::NonZero, ins->snapshot()))
                return false;
        }

        if (!mir->canBeNegativeDividend()) {
            // Numerator is unsigned, so needs no adjusting. Do the shift.
            masm.sarl(Imm32(shift), lhs);
            return true;
        }

        // Adjust the value so that shifting produces a correctly rounded result
        // when the numerator is negative. See 10-1 "Signed Division by a Known
        // Power of 2" in Henry S. Warren, Jr.'s Hacker's Delight.
        Register lhsCopy = ToRegister(ins->numeratorCopy());
        JS_ASSERT(lhsCopy != lhs);
        if (shift > 1)
            masm.sarl(Imm32(31), lhs);
        masm.shrl(Imm32(32 - shift), lhs);
        masm.addl(lhsCopy, lhs);

        // Do the shift.
        masm.sarl(Imm32(shift), lhs);
    }

    return true;
}

bool
CodeGeneratorMIPS::visitDivSelfI(LDivSelfI *ins)
{
    Register op = ToRegister(ins->op());
    Register output = ToRegister(ins->output());
    MDiv *mir = ins->mir();

    // If we can't divide by zero, lowering should have just used a constant one.
    JS_ASSERT(mir->canBeDivideByZero());

    masm.testl(op, op);
    if (mir->isTruncated()) {
        masm.emitSet(Assembler::NonZero, output);
    } else {
       if (!bailoutIf(Assembler::Zero, ins->snapshot()))
           return false;
        masm.mov(ImmWord(1), output);
    }

    return true;
}

bool
CodeGeneratorMIPS::visitDivI(LDivI *ins)
{
    Register remainder = ToRegister(ins->remainder());
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());
    Register output = ToRegister(ins->output());

    MDiv *mir = ins->mir();

    JS_ASSERT(remainder == t7/*edx*/);
    JS_ASSERT(lhs == t6/*eax*/);
    JS_ASSERT(output == t6/*eax*/);

    Label done;
    ReturnZero *ool = nullptr;

    // Handle divide by zero.
    if (mir->canBeDivideByZero()) {
        masm.testl(rhs, rhs);
        if (mir->isTruncated()) {
            // Truncated division by zero is zero (Infinity|0 == 0)
            if (!ool)
                ool = new(alloc()) ReturnZero(output);
            masm.j(Assembler::Zero, ool->entry());
        } else {
            JS_ASSERT(mir->fallible());
            if (!bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
        }
    }

    // Handle an integer overflow exception from -2147483648 / -1.
    if (mir->canBeNegativeOverflow()) {
        Label notmin;
        masm.cmpl(lhs, Imm32(INT32_MIN));
        masm.j(Assembler::NotEqual, &notmin);
        masm.cmpl(rhs, Imm32(-1));
        if (mir->isTruncated()) {
            // (-INT32_MIN)|0 == INT32_MIN and INT32_MIN is already in the
            // output register (lhs == eax).
            masm.j(Assembler::Equal, &done);
        } else {
            JS_ASSERT(mir->fallible());
            if (!bailoutIf(Assembler::Equal, ins->snapshot()))
                return false;
        }
        masm.bind(&notmin);
    }

    // Handle negative 0.
    if (!mir->isTruncated() && mir->canBeNegativeZero()) {
        Label nonzero;
        masm.testl(lhs, lhs);
        masm.j(Assembler::NonZero, &nonzero);
        masm.cmpl(rhs, Imm32(0));
        if (!bailoutIf(Assembler::LessThan, ins->snapshot()))
            return false;
        masm.bind(&nonzero);
    }

    // Sign extend eax into edx to make (edx:eax), since idiv is 64-bit.
    masm.cdq();
    masm.idiv(rhs);

    if (!mir->isTruncated()) {
        // If the remainder is > 0, bailout since this must be a double.
        masm.testl(remainder, remainder);
        if (!bailoutIf(Assembler::NonZero, ins->snapshot()))
            return false;
    }

    masm.bind(&done);

    if (ool) {
        if (!addOutOfLineCode(ool))
            return false;
        masm.bind(ool->rejoin());
    }

    return true;
}

bool
CodeGeneratorMIPS::visitModSelfI(LModSelfI *ins)
{
    Register op = ToRegister(ins->op());
    Register output = ToRegister(ins->output());
    MMod *mir = ins->mir();

    // If we're not fallible, lowering should have just used a constant zero.
    JS_ASSERT(mir->fallible());
    JS_ASSERT(mir->canBeDivideByZero() || (!mir->isUnsigned() && mir->canBeNegativeDividend()));

    masm.testl(op, op);

    // For a negative operand, we need to return negative zero. We can't
    // represent that as an int32, so bail if that happens.
    if (!mir->isUnsigned() && mir->canBeNegativeDividend()) {
        if (!bailoutIf(Assembler::Signed, ins->snapshot()))
             return false;
    }

    // For a zero operand, we need to return NaN. We can't
    // represent that as an int32, so bail if that happens.
    if (mir->canBeDivideByZero()) {
        if (!bailoutIf(Assembler::Zero, ins->snapshot()))
            return false;
    }

    // For any other value, return 0.
    masm.mov(ImmWord(0), output);

    return true;
}

bool
CodeGeneratorMIPS::visitModPowTwoI(LModPowTwoI *ins)
{
    Register lhs = ToRegister(ins->getOperand(0));
    int32_t shift = ins->shift();

    Label negative;

    if (ins->mir()->canBeNegativeDividend()) {
        // Switch based on sign of the lhs.
        // Positive numbers are just a bitmask
        masm.branchTest32(Assembler::Signed, lhs, lhs, &negative);
    }

    masm.andl(Imm32((1 << shift) - 1), lhs);

    if (ins->mir()->canBeNegativeDividend()) {
        Label done;
        masm.jump(&done);

        // Negative numbers need a negate, bitmask, negate
        masm.bind(&negative);
        // visitModI has an overflow check here to catch INT_MIN % -1, but
        // here the rhs is a power of 2, and cannot be -1, so the check is not generated.
        masm.negl(lhs);
        masm.andl(Imm32((1 << shift) - 1), lhs);
        masm.negl(lhs);
        if (!ins->mir()->isTruncated() && !bailoutIf(Assembler::Zero, ins->snapshot()))
            return false;
        masm.bind(&done);
    }
    return true;

}

class ModOverflowCheck : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    Label done_;
    LModI *ins_;
    Register rhs_;

  public:
    explicit ModOverflowCheck(LModI *ins, Register rhs)
      : ins_(ins), rhs_(rhs)
    { }

    virtual bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitModOverflowCheck(this);
    }
    Label *done() {
        return &done_;
    }
    LModI *ins() const {
        return ins_;
    }
    Register rhs() const {
        return rhs_;
    }
};

bool
CodeGeneratorMIPS::visitModOverflowCheck(ModOverflowCheck *ool)
{
    masm.cmpl(ool->rhs(), Imm32(-1));
    if (ool->ins()->mir()->isTruncated()) {
        masm.j(Assembler::NotEqual, ool->rejoin());
        masm.mov(ImmWord(0), t7/*edx*/);
        masm.jmp(ool->done());
    } else {
        if (!bailoutIf(Assembler::Equal, ool->ins()->snapshot()))
            return false;
       masm.jmp(ool->rejoin());
    }
    return true;
}

bool
CodeGeneratorMIPS::visitModI(LModI *ins)
{
    Register remainder = ToRegister(ins->remainder());
    Register lhs = ToRegister(ins->lhs());
    Register rhs = ToRegister(ins->rhs());

    // Required to use idiv.
    JS_ASSERT(lhs == t6/*eax*/);
    JS_ASSERT(remainder == t7/*edx*/);
    JS_ASSERT(ToRegister(ins->getTemp(0)) == t6/*eax*/);

    Label done;
    ReturnZero *ool = nullptr;
    ModOverflowCheck *overflow = nullptr;

    // Prevent divide by zero.
    if (ins->mir()->canBeDivideByZero()) {
        masm.testl(rhs, rhs);
        if (ins->mir()->isTruncated()) {
            if (!ool)
                ool = new(alloc()) ReturnZero(t7/*edx*/);
            masm.j(Assembler::Zero, ool->entry());
        } else {
            if (!bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
        }
    }

    Label negative;

    // Switch based on sign of the lhs.
    if (ins->mir()->canBeNegativeDividend())
        masm.branchTest32(Assembler::Signed, lhs, lhs, &negative);

    // If lhs >= 0 then remainder = lhs % rhs. The remainder must be positive.
    {
        // Check if rhs is a power-of-two.
        if (ins->mir()->canBePowerOfTwoDivisor()) {
            JS_ASSERT(rhs != remainder);

            // Rhs y is a power-of-two if (y & (y-1)) == 0. Note that if
            // y is any negative number other than INT32_MIN, both y and
            // y-1 will have the sign bit set so these are never optimized
            // as powers-of-two. If y is INT32_MIN, y-1 will be INT32_MAX
            // and because lhs >= 0 at this point, lhs & INT32_MAX returns
            // the correct value.
            Label notPowerOfTwo;
            masm.mov(rhs, remainder);
            masm.subl(Imm32(1), remainder);
            masm.branchTest32(Assembler::NonZero, remainder, rhs, &notPowerOfTwo);
            {
                masm.andl(lhs, remainder);
                masm.jmp(&done);
            }
            masm.bind(&notPowerOfTwo);
        }

        // Since lhs >= 0, the sign-extension will be 0
        masm.mov(ImmWord(0), t7/*edx*/);
        masm.idiv(rhs);
    }

    // Otherwise, we have to beware of two special cases:
    if (ins->mir()->canBeNegativeDividend()) {
        masm.jump(&done);

        masm.bind(&negative);

        // Prevent an integer overflow exception from -2147483648 % -1
        Label notmin;
        masm.cmpl(lhs, Imm32(INT32_MIN));
        overflow = new(alloc()) ModOverflowCheck(ins, rhs);
        masm.j(Assembler::Equal, overflow->entry());
        masm.bind(overflow->rejoin());
        masm.cdq();
        masm.idiv(rhs);

        if (!ins->mir()->isTruncated()) {
            // A remainder of 0 means that the rval must be -0, which is a double.
            masm.testl(remainder, remainder);
            if (!bailoutIf(Assembler::Zero, ins->snapshot()))
                return false;
        }
    }

    masm.bind(&done);

    if (overflow) {
        if (!addOutOfLineCode(overflow))
            return false;
        masm.bind(overflow->done());
    }

    if (ool) {
        if (!addOutOfLineCode(ool))
            return false;
        masm.bind(ool->rejoin());
    }

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
            MOZ_ASSUME_UNREACHABLE("unexpected binary opcode");
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
            } else if (ins->mir()->toUrsh()->fallible()) {
                // x >>> 0 can overflow.
                masm.testl(lhs, lhs);
                if (!bailoutIf(Assembler::Signed, ins->snapshot()))
                    return false;
            }
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("Unexpected shift op");
        }
    } else {
        JS_ASSERT(ToRegister(rhs) == t8/*ecx*/);
        switch (ins->bitop()) {
          case JSOP_LSH:
            masm.shll_cl(lhs);
            break;
          case JSOP_RSH:
            masm.sarl_cl(lhs);
            break;
          case JSOP_URSH:
            masm.shrl_cl(lhs);
            if (ins->mir()->toUrsh()->fallible()) {
                // x >>> 0 can overflow.
                masm.testl(lhs, lhs);
                if (!bailoutIf(Assembler::Signed, ins->snapshot()))
                    return false;
            }
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("Unexpected shift op");
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
        JS_ASSERT(ToRegister(rhs) == t8/*ecx*/);
        masm.shrl_cl(lhs);
    }

    masm.convertUInt32ToDouble(lhs, out);
    return true;
}

MoveOperand
CodeGeneratorMIPS::toMoveOperand(const LAllocation *a) const
{
    if (a->isGeneralReg())
        return MoveOperand(ToRegister(a));
    if (a->isFloatReg())
        return MoveOperand(ToFloatRegister(a));
    return MoveOperand(StackPointer, ToStackOffset(a));
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
    OutOfLineTableSwitch *ool = new(alloc()) OutOfLineTableSwitch(mir);
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
        MOZ_ASSUME_UNREACHABLE("unexpected opcode");
    }
    return true;
}

bool
CodeGeneratorMIPS::visitMathF(LMathF *math)
{
    FloatRegister lhs = ToFloatRegister(math->lhs());
    Operand rhs = ToOperand(math->rhs());

    JS_ASSERT(ToFloatRegister(math->output()) == lhs);

    switch (math->jsop()) {
      case JSOP_ADD:
        masm.addss(rhs, lhs);
        break;
      case JSOP_SUB:
        masm.subss(rhs, lhs);
        break;
      case JSOP_MUL:
        masm.mulss(rhs, lhs);
        break;
      case JSOP_DIV:
        masm.divss(rhs, lhs);
        break;
      default:
        MOZ_ASSUME_UNREACHABLE("unexpected opcode");
        return false;
    }
    return true;
}

bool
CodeGeneratorMIPS::visitFloor(LFloor *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    FloatRegister scratch = ScratchFloatReg;
    Register output = ToRegister(lir->output());

    /*if (AssemblerX86Shared::HasSSE41()) {
        // Bail on negative-zero.
        Assembler::Condition bailCond = masm.testNegativeZero(input, output);
        if (!bailoutIf(bailCond, lir->snapshot()))
            return false;

        // Round toward -Infinity.
        masm.roundsd(input, scratch, JSC::X86Assembler::RoundDown);

        masm.cvttsd2si(scratch, output);
        masm.cmp32(output, Imm32(INT_MIN));
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;
    } else */{
        Label negative, end;

        // Branch to a slow path for negative inputs. Doesn't catch NaN or -0.
        masm.xorpd(scratch, scratch);
        masm.branchDouble(Assembler::DoubleLessThan, input, scratch, &negative);

        // Bail on negative-zero.
        Assembler::Condition bailCond = masm.testNegativeZero(input, output);
        if (!bailoutIf(bailCond, lir->snapshot()))
            return false;

        // Input is non-negative, so truncation correctly rounds.
        masm.cvttsd2si(input, output);
        masm.cmp32(output, Imm32(INT_MIN));
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;

        masm.jump(&end);

        // Input is negative, but isn't -0.
        // Negative values go on a comparatively expensive path, since no
        // native rounding mode matches JS semantics. Still better than callVM.
        masm.bind(&negative);
        {
            // Truncate and round toward zero.
            // This is off-by-one for everything but integer-valued inputs.
            masm.cvttsd2si(input, output);
            masm.cmp32(output, Imm32(INT_MIN));
            if (!bailoutIf(Assembler::Equal, lir->snapshot()))
                return false;

            // Test whether the input double was integer-valued.
            masm.convertInt32ToDouble(output, scratch);
            masm.branchDouble(Assembler::DoubleEqualOrUnordered, input, scratch, &end);

            // Input is not integer-valued, so we rounded off-by-one in the
            // wrong direction. Correct by subtraction.
            masm.subl(Imm32(1), output);
            // Cannot overflow: output was already checked against INT_MIN.
        }

        masm.bind(&end);
    }
    return true;
}

bool
CodeGeneratorMIPS::visitFloorF(LFloorF *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    FloatRegister scratch = ScratchFloatReg;
    Register output = ToRegister(lir->output());

    /*if (AssemblerX86Shared::HasSSE41()) {
        // Bail on negative-zero.
        Assembler::Condition bailCond = masm.testNegativeZeroFloat32(input, output);
        if (!bailoutIf(bailCond, lir->snapshot()))
            return false;

        // Round toward -Infinity.
        masm.roundss(input, scratch, JSC::X86Assembler::RoundDown);

        masm.cvttss2si(scratch, output);
        masm.cmp32(output, Imm32(INT_MIN));
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;
    } else */{
        Label negative, end;

        // Branch to a slow path for negative inputs. Doesn't catch NaN or -0.
        masm.xorps(scratch, scratch);
        masm.branchFloat(Assembler::DoubleLessThan, input, scratch, &negative);

        // Bail on negative-zero.
        Assembler::Condition bailCond = masm.testNegativeZeroFloat32(input, output);
        if (!bailoutIf(bailCond, lir->snapshot()))
            return false;

        // Input is non-negative, so truncation correctly rounds.
        masm.cvttss2si(input, output);
        masm.cmp32(output, Imm32(INT_MIN));
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;

        masm.jump(&end);

        // Input is negative, but isn't -0.
        // Negative values go on a comparatively expensive path, since no
        // native rounding mode matches JS semantics. Still better than callVM.
        masm.bind(&negative);
        {
            // Truncate and round toward zero.
            // This is off-by-one for everything but integer-valued inputs.
            masm.cvttss2si(input, output);
            masm.cmp32(output, Imm32(INT_MIN));
            if (!bailoutIf(Assembler::Equal, lir->snapshot()))
                return false;

            // Test whether the input double was integer-valued.
            masm.convertInt32ToFloat32(output, scratch);
            masm.branchFloat(Assembler::DoubleEqualOrUnordered, input, scratch, &end);

            // Input is not integer-valued, so we rounded off-by-one in the
            // wrong direction. Correct by subtraction.
            masm.subl(Imm32(1), output);
            // Cannot overflow: output was already checked against INT_MIN.
        }

        masm.bind(&end);
    }
    return true;
}

bool
CodeGeneratorMIPS::visitRound(LRound *lir)
{
    FloatRegister input = ToFloatRegister(lir->input());
    FloatRegister temp = ToFloatRegister(lir->temp());
    FloatRegister scratch = ScratchFloatReg;
    Register output = ToRegister(lir->output());

    Label negative, end;

    // Load 0.5 in the temp register.
    masm.loadConstantDouble(0.5, temp);

    // Branch to a slow path for negative inputs. Doesn't catch NaN or -0.
    masm.xorpd(scratch, scratch);
    masm.branchDouble(Assembler::DoubleLessThan, input, scratch, &negative);

    // Bail on negative-zero.
    Assembler::Condition bailCond = masm.testNegativeZero(input, output);
    if (!bailoutIf(bailCond, lir->snapshot()))
        return false;

    // Input is non-negative. Add 0.5 and truncate, rounding down. Note that we
    // have to add the input to the temp register (which contains 0.5) because
    // we're not allowed to modify the input register.
    masm.addsd(input, temp);

    masm.cvttsd2si(temp, output);
    masm.cmp32(output, Imm32(INT_MIN));
    if (!bailoutIf(Assembler::Equal, lir->snapshot()))
        return false;

    masm.jump(&end);


    // Input is negative, but isn't -0.
    masm.bind(&negative);

    /*if (AssemblerX86Shared::HasSSE41()) {
        // Add 0.5 and round toward -Infinity. The result is stored in the temp
        // register (currently contains 0.5).
        masm.addsd(input, temp);
        masm.roundsd(temp, scratch, JSC::X86Assembler::RoundDown);

        // Truncate.
        masm.cvttsd2si(scratch, output);
        masm.cmp32(output, Imm32(INT_MIN));
        if (!bailoutIf(Assembler::Equal, lir->snapshot()))
            return false;

        // If the result is positive zero, then the actual result is -0. Bail.
        // Otherwise, the truncation will have produced the correct negative integer.
        masm.testl(output, output);
        if (!bailoutIf(Assembler::Zero, lir->snapshot()))
            return false;

    } else */{
        masm.addsd(input, temp);

        // Round toward -Infinity without the benefit of ROUNDSD.
        {
            // If input + 0.5 >= 0, input is a negative number >= -0.5 and the result is -0.
            masm.compareDouble(Assembler::DoubleGreaterThanOrEqual, temp, scratch);
            if (!bailoutIf(Assembler::DoubleGreaterThanOrEqual, lir->snapshot()))
                return false;

            // Truncate and round toward zero.
            // This is off-by-one for everything but integer-valued inputs.
            masm.cvttsd2si(temp, output);
            masm.cmp32(output, Imm32(INT_MIN));
            if (!bailoutIf(Assembler::Equal, lir->snapshot()))
                return false;

            // Test whether the truncated double was integer-valued.
            masm.convertInt32ToDouble(output, scratch);
            masm.branchDouble(Assembler::DoubleEqualOrUnordered, temp, scratch, &end);

            // Input is not integer-valued, so we rounded off-by-one in the
            // wrong direction. Correct by subtraction.
            masm.subl(Imm32(1), output);
            // Cannot overflow: output was already checked against INT_MIN.
        }
    }

    masm.bind(&end);
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
    masm.cmpPtr(Operand(tmp, offsetof(types::TypeObject, clasp)), ImmPtr(guard->mir()->getClass()));
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

    masm.bind(&invalidate_);

    // Push the Ion script onto the stack (when we determine what that pointer is).
    invalidateEpilogueData_ = masm.pushWithPatch(ImmWord(uintptr_t(-1)));
    IonCode *thunk = gen->jitRuntime()->getInvalidationThunk();

    masm.call(thunk);

    // We should never reach this point in JIT code -- the invalidation thunk should
    // pop the invalidated JS frame and return directly to its caller.
    masm.assumeUnreachable("Should have returned directly to its caller instead of here.");
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

bool
CodeGeneratorMIPS::visitNegF(LNegF *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    JS_ASSERT(input == ToFloatRegister(ins->output()));

    masm.negateFloat(input);
    return true;
}

} // namespace jit
} // namespace js



//**************************************************
//**************************************************
// Following is copy form jit/x86/CodeGenerator-x86.cpp
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
CodeGeneratorMIPS::visitBox(LBox *box)
{
    const LDefinition *type = box->getDef(TYPE_INDEX);

    DebugOnly<const LAllocation *> a = box->getOperand(0);
    JS_ASSERT(!a->isConstant());

    // On x86, the input operand and the output payload have the same
    // virtual register. All that needs to be written is the type tag for
    // the type definition.
    masm.mov(ImmWord(MIRTypeToTag(box->type())), ToRegister(type));
    return true;
}

bool
CodeGeneratorMIPS::visitBoxFloatingPoint(LBoxFloatingPoint *box)
{
    const LAllocation *in = box->getOperand(0);
    const ValueOperand out = ToOutValue(box);

    FloatRegister reg = ToFloatRegister(in);
    if (box->type() == MIRType_Float32) {
        masm.convertFloatToDouble(reg, ScratchFloatReg);
        reg = ScratchFloatReg;
    }
    masm.boxDouble(reg, out);
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

    masm.loadValue(Address(base, offset), out);
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
        masm.load32(Address(base, offset + NUNBOX32_PAYLOAD_OFFSET), ToRegister(load->output()));
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
        masm.storeDouble(ToFloatRegister(value), Operand(base, offset));
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
            if (source.kind() == Operand::MEM_REG_DISP)
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
        masm.storeDouble(ToFloatRegister(value), dest);
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

    // TODO: OOL stub path.
    if (!bailoutIf(Assembler::NotEqual, lir->snapshot()))
        return false;

    masm.moveValue(UndefinedValue(), out);
    return true;
}

bool
CodeGeneratorMIPS::visitInterruptCheck(LInterruptCheck *lir)
{
    OutOfLineCode *ool = oolCallVM(InterruptCheckInfo, lir, (ArgList()), StoreNothing());
    if (!ool)
        return false;

    masm.cmpl(Operand(AbsoluteAddress(GetIonContext()->runtime->addressOfInterrupt())), Imm32(0));
    masm.j(Assembler::NonZero, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitCompareB(LCompareB *lir)
{
    MCompare *mir = lir->mir();

    const ValueOperand lhs = ToValue(lir, LCompareB::Lhs);
    const LAllocation *rhs = lir->rhs();
    const Register output = ToRegister(lir->output());

    JS_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    Label notBoolean, done;
    masm.branchTestBoolean(Assembler::NotEqual, lhs, &notBoolean);
    {
        if (rhs->isConstant())
            masm.cmp32(lhs.payloadReg(), Imm32(rhs->toConstant()->toBoolean()));
        else
            masm.cmp32(lhs.payloadReg(), ToRegister(rhs));
        masm.emitSet(JSOpToCondition(mir->compareType(), mir->jsop()), output);
        masm.jump(&done);
    }
    masm.bind(&notBoolean);
    {
        masm.move32(Imm32(mir->jsop() == JSOP_STRICTNE), output);
    }

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareBAndBranch(LCompareBAndBranch *lir)
{
    MCompare *mir = lir->cmpMir();
    const ValueOperand lhs = ToValue(lir, LCompareBAndBranch::Lhs);
    const LAllocation *rhs = lir->rhs();

    JS_ASSERT(mir->jsop() == JSOP_STRICTEQ || mir->jsop() == JSOP_STRICTNE);

    Assembler::Condition cond = masm.testBoolean(Assembler::NotEqual, lhs);
    jumpToBlock((mir->jsop() == JSOP_STRICTEQ) ? lir->ifFalse() : lir->ifTrue(), cond);

    if (rhs->isConstant())
        masm.cmp32(lhs.payloadReg(), Imm32(rhs->toConstant()->toBoolean()));
    else
        masm.cmp32(lhs.payloadReg(), ToRegister(rhs));
    emitBranch(JSOpToCondition(mir->compareType(), mir->jsop()), lir->ifTrue(), lir->ifFalse());
    return true;
}

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
    masm.cmp32(lhs.typeReg(), rhs.typeReg());
    masm.j(Assembler::NotEqual, &notEqual);
    {
        masm.cmp32(lhs.payloadReg(), rhs.payloadReg());
        masm.emitSet(cond, output);
        masm.jump(&done);
    }
    masm.bind(&notEqual);
    {
        masm.move32(Imm32(cond == Assembler::NotEqual), output);
    }

    masm.bind(&done);
    return true;
}

bool
CodeGeneratorMIPS::visitCompareVAndBranch(LCompareVAndBranch *lir)
{
    MCompare *mir = lir->cmpMir();
    Assembler::Condition cond = JSOpToCondition(mir->compareType(), mir->jsop());
    const ValueOperand lhs = ToValue(lir, LCompareVAndBranch::LhsInput);
    const ValueOperand rhs = ToValue(lir, LCompareVAndBranch::RhsInput);

    JS_ASSERT(mir->jsop() == JSOP_EQ || mir->jsop() == JSOP_STRICTEQ ||
              mir->jsop() == JSOP_NE || mir->jsop() == JSOP_STRICTNE);

    MBasicBlock *notEqual = (cond == Assembler::Equal) ? lir->ifFalse() : lir->ifTrue();

    masm.cmp32(lhs.typeReg(), rhs.typeReg());
    jumpToBlock(notEqual, Assembler::NotEqual);
    masm.cmp32(lhs.payloadReg(), rhs.payloadReg());
    emitBranch(cond, lir->ifTrue(), lir->ifFalse());

    return true;
}

bool
CodeGeneratorMIPS::visitAsmJSUInt32ToDouble(LAsmJSUInt32ToDouble *lir)
{
    Register input = ToRegister(lir->input());
    Register temp = ToRegister(lir->temp());

    if (input != temp)
        masm.mov(input, temp);

    // Beware: convertUInt32ToDouble clobbers input.
    masm.convertUInt32ToDouble(temp, ToFloatRegister(lir->output()));
    return true;
}

bool
CodeGeneratorMIPS::visitAsmJSUInt32ToFloat32(LAsmJSUInt32ToFloat32 *lir)
{
    Register input = ToRegister(lir->input());
    Register temp = ToRegister(lir->temp());
    FloatRegister output = ToFloatRegister(lir->output());

    if (input != temp)
        masm.mov(input, temp);

    // Beware: convertUInt32ToFloat32 clobbers input.
    masm.convertUInt32ToFloat32(temp, output);
    return true;
}

// Load a NaN or zero into a register for an out of bounds AsmJS or static
// typed array load.
class jit::OutOfLineLoadTypedArrayOutOfBounds : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    AnyRegister dest_;
    bool isFloat32Load_;
  public:
    OutOfLineLoadTypedArrayOutOfBounds(AnyRegister dest, bool isFloat32Load)
      : dest_(dest), isFloat32Load_(isFloat32Load)
    {}

    const AnyRegister &dest() const { return dest_; }
    bool isFloat32Load() const { return isFloat32Load_; }
    bool accept(CodeGeneratorMIPS *codegen) { return codegen->visitOutOfLineLoadTypedArrayOutOfBounds(this); }
};

template<typename T>
void
CodeGeneratorMIPS::loadViewTypeElement(ArrayBufferView::ViewType vt, const T &srcAddr,
                                      const LDefinition *out)
{
    switch (vt) {
      case ArrayBufferView::TYPE_INT8:    masm.movsblWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_UINT8_CLAMPED:
      case ArrayBufferView::TYPE_UINT8:   masm.movzblWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_INT16:   masm.movswlWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_UINT16:  masm.movzwlWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_INT32:
      case ArrayBufferView::TYPE_UINT32:  masm.movlWithPatch(srcAddr, ToRegister(out)); break;
      case ArrayBufferView::TYPE_FLOAT32: masm.movssWithPatch(srcAddr, ToFloatRegister(out)); break;
      case ArrayBufferView::TYPE_FLOAT64: masm.movsdWithPatch(srcAddr, ToFloatRegister(out)); break;
      default: MOZ_ASSUME_UNREACHABLE("unexpected array type");
    }
}

template<typename T>
bool
CodeGeneratorMIPS::loadAndNoteViewTypeElement(ArrayBufferView::ViewType vt, const T &srcAddr,
                                             const LDefinition *out)
{
    uint32_t before = masm.size();
    loadViewTypeElement(vt, srcAddr, out);
    uint32_t after = masm.size();
    return gen->noteHeapAccess(AsmJSHeapAccess(before, after, vt, ToAnyRegister(out)));
}

bool
CodeGeneratorMIPS::visitLoadTypedArrayElementStatic(LLoadTypedArrayElementStatic *ins)
{
    const MLoadTypedArrayElementStatic *mir = ins->mir();
    ArrayBufferView::ViewType vt = mir->viewType();
    JS_ASSERT_IF(vt == ArrayBufferView::TYPE_FLOAT32, mir->type() == MIRType_Float32);

    Register ptr = ToRegister(ins->ptr());
    const LDefinition *out = ins->output();

    OutOfLineLoadTypedArrayOutOfBounds *ool = nullptr;
    bool isFloat32Load = (vt == ArrayBufferView::TYPE_FLOAT32);
    if (!mir->fallible()) {
        ool = new(alloc()) OutOfLineLoadTypedArrayOutOfBounds(ToAnyRegister(out), isFloat32Load);
        if (!addOutOfLineCode(ool))
            return false;
    }

    masm.cmpl(ptr, Imm32(mir->length()));
    if (ool)
        masm.j(Assembler::AboveOrEqual, ool->entry());
    else if (!bailoutIf(Assembler::AboveOrEqual, ins->snapshot()))
        return false;

    Address srcAddr(ptr, (int32_t) mir->base());
    loadViewTypeElement(vt, srcAddr, out);
    if (vt == ArrayBufferView::TYPE_FLOAT64)
        masm.canonicalizeDouble(ToFloatRegister(out));
    if (vt == ArrayBufferView::TYPE_FLOAT32)
        masm.canonicalizeFloat(ToFloatRegister(out));
    if (ool)
        masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitAsmJSLoadHeap(LAsmJSLoadHeap *ins)
{
    const MAsmJSLoadHeap *mir = ins->mir();
    ArrayBufferView::ViewType vt = mir->viewType();
    const LAllocation *ptr = ins->ptr();
    const LDefinition *out = ins->output();

    if (ptr->isConstant()) {
        // The constant displacement still needs to be added to the as-yet-unknown
        // base address of the heap. For now, embed the displacement as an
        // immediate in the instruction. This displacement will fixed up when the
        // base address is known during dynamic linking (AsmJSModule::initHeap).
        PatchedAbsoluteAddress srcAddr((void *) ptr->toConstant()->toInt32());
        return loadAndNoteViewTypeElement(vt, srcAddr, out);
    }

    Register ptrReg = ToRegister(ptr);
    Address srcAddr(ptrReg, 0);

    if (mir->skipBoundsCheck())
        return loadAndNoteViewTypeElement(vt, srcAddr, out);

    bool isFloat32Load = vt == ArrayBufferView::TYPE_FLOAT32;
    OutOfLineLoadTypedArrayOutOfBounds *ool = new(alloc()) OutOfLineLoadTypedArrayOutOfBounds(ToAnyRegister(out), isFloat32Load);
    if (!addOutOfLineCode(ool))
        return false;

    CodeOffsetLabel cmp = masm.cmplWithPatch(ptrReg, Imm32(0));
    masm.j(Assembler::AboveOrEqual, ool->entry());

    uint32_t before = masm.size();
    loadViewTypeElement(vt, srcAddr, out);
    uint32_t after = masm.size();
    masm.bind(ool->rejoin());
    bool temp = gen->noteHeapAccess(AsmJSHeapAccess(before, after, vt, ToAnyRegister(out), cmp.offset()));
    return temp;
}

bool
CodeGeneratorMIPS::visitOutOfLineLoadTypedArrayOutOfBounds(OutOfLineLoadTypedArrayOutOfBounds *ool)
{
    if (ool->dest().isFloat()) {
        if (ool->isFloat32Load())
            masm.loadConstantFloat32(float(GenericNaN()), ool->dest().fpu());
        else
            masm.loadConstantDouble(GenericNaN(), ool->dest().fpu());
    } else {
        Register destReg = ool->dest().gpr();
        masm.mov(ImmWord(0), destReg);
    }
    masm.jmp(ool->rejoin());
    return true;
}

template<typename T>
void
CodeGeneratorMIPS::storeViewTypeElement(ArrayBufferView::ViewType vt, const LAllocation *value,
                                       const T &dstAddr)
{
    switch (vt) {
      case ArrayBufferView::TYPE_INT8:
      case ArrayBufferView::TYPE_UINT8_CLAMPED:
      case ArrayBufferView::TYPE_UINT8:   masm.movbWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_INT16:
      case ArrayBufferView::TYPE_UINT16:  masm.movwWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_INT32:
      case ArrayBufferView::TYPE_UINT32:  masm.movlWithPatch(ToRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_FLOAT32: masm.movssWithPatch(ToFloatRegister(value), dstAddr); break;
      case ArrayBufferView::TYPE_FLOAT64: masm.movsdWithPatch(ToFloatRegister(value), dstAddr); break;
      default: MOZ_ASSUME_UNREACHABLE("unexpected array type");
    }
}

template<typename T>
bool
CodeGeneratorMIPS::storeAndNoteViewTypeElement(ArrayBufferView::ViewType vt, const LAllocation *value,
                                              const T &dstAddr)
{
    uint32_t before = masm.size();
    storeViewTypeElement(vt, value, dstAddr);
    uint32_t after = masm.size();
    return gen->noteHeapAccess(AsmJSHeapAccess(before, after));
}

bool
CodeGeneratorMIPS::visitStoreTypedArrayElementStatic(LStoreTypedArrayElementStatic *ins)
{
    MStoreTypedArrayElementStatic *mir = ins->mir();
    ArrayBufferView::ViewType vt = mir->viewType();

    Register ptr = ToRegister(ins->ptr());
    const LAllocation *value = ins->value();

    masm.cmpl(ptr, Imm32(mir->length()));
    Label rejoin;
    masm.j(Assembler::AboveOrEqual, &rejoin);

    Address dstAddr(ptr, (int32_t) mir->base());
    storeViewTypeElement(vt, value, dstAddr);
    masm.bind(&rejoin);
    return true;
}

bool
CodeGeneratorMIPS::visitAsmJSStoreHeap(LAsmJSStoreHeap *ins)
{
    MAsmJSStoreHeap *mir = ins->mir();
    ArrayBufferView::ViewType vt = mir->viewType();
    const LAllocation *value = ins->value();
    const LAllocation *ptr = ins->ptr();

    if (ptr->isConstant()) {
        // The constant displacement still needs to be added to the as-yet-unknown
        // base address of the heap. For now, embed the displacement as an
        // immediate in the instruction. This displacement will fixed up when the
        // base address is known during dynamic linking (AsmJSModule::initHeap).
        PatchedAbsoluteAddress dstAddr((void *) ptr->toConstant()->toInt32());
        return storeAndNoteViewTypeElement(vt, value, dstAddr);
    }

    Register ptrReg = ToRegister(ptr);
    Address dstAddr(ptrReg, 0);

    if (mir->skipBoundsCheck())
        return storeAndNoteViewTypeElement(vt, value, dstAddr);

    CodeOffsetLabel cmp = masm.cmplWithPatch(ptrReg, Imm32(0));
    Label rejoin;
    masm.j(Assembler::AboveOrEqual, &rejoin);

    uint32_t before = masm.size();
    storeViewTypeElement(vt, value, dstAddr);
    uint32_t after = masm.size();
    masm.bind(&rejoin);
    bool temp = gen->noteHeapAccess(AsmJSHeapAccess(before, after, cmp.offset()));
    return temp;
}

bool
CodeGeneratorMIPS::visitAsmJSLoadGlobalVar(LAsmJSLoadGlobalVar *ins)
{
    MAsmJSLoadGlobalVar *mir = ins->mir();
    MIRType type = mir->type();
    JS_ASSERT(IsNumberType(type));

    CodeOffsetLabel label;
    if (type == MIRType_Int32)
        label = masm.movlWithPatch(PatchedAbsoluteAddress(), ToRegister(ins->output()));
    else if (type == MIRType_Float32)
        label = masm.movssWithPatch(PatchedAbsoluteAddress(), ToFloatRegister(ins->output()));
    else
        label = masm.movsdWithPatch(PatchedAbsoluteAddress(), ToFloatRegister(ins->output()));

    return gen->noteGlobalAccess(label.offset(), mir->globalDataOffset());
}

bool
CodeGeneratorMIPS::visitAsmJSStoreGlobalVar(LAsmJSStoreGlobalVar *ins)
{
    MAsmJSStoreGlobalVar *mir = ins->mir();

    MIRType type = mir->value()->type();
    JS_ASSERT(IsNumberType(type));

    CodeOffsetLabel label;
    if (type == MIRType_Int32)
        label = masm.movlWithPatch(ToRegister(ins->value()), PatchedAbsoluteAddress());
    else if (type == MIRType_Float32)
        label = masm.movssWithPatch(ToFloatRegister(ins->value()), PatchedAbsoluteAddress());
    else
        label = masm.movsdWithPatch(ToFloatRegister(ins->value()), PatchedAbsoluteAddress());

    return gen->noteGlobalAccess(label.offset(), mir->globalDataOffset());
}

bool
CodeGeneratorMIPS::visitAsmJSLoadFuncPtr(LAsmJSLoadFuncPtr *ins)
{
    MAsmJSLoadFuncPtr *mir = ins->mir();

    Register index = ToRegister(ins->index());
    Register out = ToRegister(ins->output());
    CodeOffsetLabel label = masm.movlWithPatch(PatchedAbsoluteAddress(), index, TimesFour, out);

    return gen->noteGlobalAccess(label.offset(), mir->globalDataOffset());
}

bool
CodeGeneratorMIPS::visitAsmJSLoadFFIFunc(LAsmJSLoadFFIFunc *ins)
{
    MAsmJSLoadFFIFunc *mir = ins->mir();

    Register out = ToRegister(ins->output());
    CodeOffsetLabel label = masm.movlWithPatch(PatchedAbsoluteAddress(), out);

    return gen->noteGlobalAccess(label.offset(), mir->globalDataOffset());
}

void
CodeGeneratorMIPS::postAsmJSCall(LAsmJSCall *lir)
{
    // NOTE:this part is about ASM.JS in firefox,so deleted for temp
    /*
    MAsmJSCall *mir = lir->mir();
    if (!IsFloatingPointType(mir->type()) || mir->callee().which() != MAsmJSCall::Callee::Builtin)
        return;

    if (mir->type() == MIRType_Float32) {
        Operand op(esp, -sizeof(float));
        masm.fstp32(op);
        masm.loadFloat(op, ReturnFloatReg);
    } else {
        Operand op(esp, -sizeof(double));
        masm.fstp(op);
        masm.loadDouble(op, ReturnFloatReg);
    }
    */
}

void
DispatchIonCache::initializeAddCacheState(LInstruction *ins, AddCacheState *addState)
{
    // On x86, where there is no general purpose scratch register available,
    // child cache classes must manually specify a dispatch scratch register.
    MOZ_ASSUME_UNREACHABLE("x86 needs manual assignment of dispatchScratch");
}

#ifdef JS_CPU_X86
void
GetPropertyParIC::initializeAddCacheState(LInstruction *ins, AddCacheState *addState)
{
    // We don't have a scratch register, but only use the temp if we needed
    // one, it's BogusTemp otherwise.
    JS_ASSERT(ins->isGetPropertyCacheV() || ins->isGetPropertyCacheT());
    if (ins->isGetPropertyCacheV() || ins->toGetPropertyCacheT()->temp()->isBogusTemp())
        addState->dispatchScratch = output_.scratchReg().gpr();
    else
        addState->dispatchScratch = ToRegister(ins->toGetPropertyCacheT()->temp());
}
#endif

#ifdef JS_CPU_X86
void
GetElementParIC::initializeAddCacheState(LInstruction *ins, AddCacheState *addState)
{
    // We don't have a scratch register, but only use the temp if we needed
    // one, it's BogusTemp otherwise.
    JS_ASSERT(ins->isGetElementCacheV() || ins->isGetElementCacheT());
    if (ins->isGetElementCacheV() || ins->toGetElementCacheT()->temp()->isBogusTemp())
        addState->dispatchScratch = output_.scratchReg().gpr();
    else
        addState->dispatchScratch = ToRegister(ins->toGetElementCacheT()->temp());
}
#endif

#ifdef JS_CPU_X86
void
SetPropertyParIC::initializeAddCacheState(LInstruction *ins, AddCacheState *addState)
{
    // We don't have an output register to reuse, so we always need a temp.
    JS_ASSERT(ins->isSetPropertyCacheV() || ins->isSetPropertyCacheT());
    if (ins->isSetPropertyCacheV())
        addState->dispatchScratch = ToRegister(ins->toSetPropertyCacheV()->tempForDispatchCache());
    else
        addState->dispatchScratch = ToRegister(ins->toSetPropertyCacheT()->tempForDispatchCache());
}
#endif

#ifdef JS_CPU_X86
void
SetElementParIC::initializeAddCacheState(LInstruction *ins, AddCacheState *addState)
{
    // We don't have an output register to reuse, but luckily SetElementCache
    // already needs a temp.
    JS_ASSERT(ins->isSetElementCacheV() || ins->isSetElementCacheT());
    if (ins->isSetElementCacheV())
        addState->dispatchScratch = ToRegister(ins->toSetElementCacheV()->temp());
    else
        addState->dispatchScratch = ToRegister(ins->toSetElementCacheT()->temp());
}
#endif

namespace js {
namespace jit {

class OutOfLineTruncate : public OutOfLineCodeBase<CodeGeneratorMIPS>
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

class OutOfLineTruncateFloat32 : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    LTruncateFToInt32 *ins_;

  public:
    OutOfLineTruncateFloat32(LTruncateFToInt32 *ins)
      : ins_(ins)
    { }

    bool accept(CodeGeneratorMIPS *codegen) {
        return codegen->visitOutOfLineTruncateFloat32(this);
    }
    LTruncateFToInt32 *ins() const {
        return ins_;
    }
};

} // namespace jit
} // namespace js

bool
CodeGeneratorMIPS::visitTruncateDToInt32(LTruncateDToInt32 *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    Register output = ToRegister(ins->output());

    OutOfLineTruncate *ool = new(alloc()) OutOfLineTruncate(ins);
    if (!addOutOfLineCode(ool))
        return false;

    masm.branchTruncateDouble(input, output, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitTruncateFToInt32(LTruncateFToInt32 *ins)
{
    FloatRegister input = ToFloatRegister(ins->input());
    Register output = ToRegister(ins->output());

    OutOfLineTruncateFloat32 *ool = new(alloc()) OutOfLineTruncateFloat32(ins);
    if (!addOutOfLineCode(ool))
        return false;

    masm.branchTruncateFloat32(input, output, ool->entry());
    masm.bind(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitOutOfLineTruncate(OutOfLineTruncate *ool)
{
    LTruncateDToInt32 *ins = ool->ins();
    FloatRegister input = ToFloatRegister(ins->input());
    Register output = ToRegister(ins->output());

    Label fail;

    /*if (Assembler::HasSSE3()) {
        // Push double.
        masm.subl(Imm32(sizeof(double)), esp);
        masm.storeDouble(input, Operand(esp, 0));

        static const uint32_t EXPONENT_MASK = 0x7ff00000;
        static const uint32_t EXPONENT_SHIFT = DoubleExponentShift - 32;
        static const uint32_t TOO_BIG_EXPONENT = (DoubleExponentBias + 63) << EXPONENT_SHIFT;

        // Check exponent to avoid fp exceptions.
        Label failPopDouble;
        masm.load32(Address(esp, 4), output);
        masm.and32(Imm32(EXPONENT_MASK), output);
        masm.branch32(Assembler::GreaterThanOrEqual, output, Imm32(TOO_BIG_EXPONENT), &failPopDouble);

        // Load double, perform 64-bit truncation.
        masm.fld(Operand(esp, 0));
        masm.fisttp(Operand(esp, 0));

        // Load low word, pop double and jump back.
        masm.load32(Address(esp, 0), output);
        masm.addl(Imm32(sizeof(double)), esp);
        masm.jump(ool->rejoin());

        masm.bind(&failPopDouble);
        masm.addl(Imm32(sizeof(double)), esp);
        masm.jump(&fail);
    } else*/ {
        FloatRegister temp = ToFloatRegister(ins->tempFloat());

        // Try to convert doubles representing integers within 2^32 of a signed
        // integer, by adding/subtracting 2^32 and then trying to convert to int32.
        // This has to be an exact conversion, as otherwise the truncation works
        // incorrectly on the modified value.
        masm.xorpd(ScratchFloatReg, ScratchFloatReg);
        masm.ucomisd(input, ScratchFloatReg);
        masm.j(Assembler::Parity, &fail);

        {
            Label positive;
            masm.j(Assembler::Above, &positive);

            masm.loadConstantDouble(4294967296.0, temp);
            Label skip;
            masm.jmp(&skip);

            masm.bind(&positive);
            masm.loadConstantDouble(-4294967296.0, temp);
            masm.bind(&skip);
        }

        masm.addsd(input, temp);
        masm.cvttsd2si(temp, output);
        masm.cvtsi2sd(output, ScratchFloatReg);

        masm.ucomisd(temp, ScratchFloatReg);
        masm.j(Assembler::Parity, &fail);
        masm.j(Assembler::Equal, ool->rejoin());
    }

    masm.bind(&fail);
    {
        saveVolatile(output);

        masm.setupUnalignedABICall(1, output);
        masm.passABIArg(input);
        if (gen->compilingAsmJS())
            masm.callWithABI(AsmJSImm_ToInt32);
        else
            masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, js::ToInt32));
        masm.storeCallResult(output);

        restoreVolatile(output);
    }

    masm.jump(ool->rejoin());
    return true;
}

bool
CodeGeneratorMIPS::visitOutOfLineTruncateFloat32(OutOfLineTruncateFloat32 *ool)
{
    LTruncateFToInt32 *ins = ool->ins();
    FloatRegister input = ToFloatRegister(ins->input());
    Register output = ToRegister(ins->output());

    Label fail;

    /*if (Assembler::HasSSE3()) {
        // Push float32, but subtracts 64 bits so that the value popped by fisttp fits
        masm.subl(Imm32(sizeof(uint64_t)), esp);
        masm.storeFloat(input, Operand(esp, 0));

        static const uint32_t EXPONENT_MASK = FloatExponentBits;
        static const uint32_t EXPONENT_SHIFT = FloatExponentShift;
        // Integers are still 64 bits long, so we can still test for an exponent > 63.
        static const uint32_t TOO_BIG_EXPONENT = (FloatExponentBias + 63) << EXPONENT_SHIFT;

        // Check exponent to avoid fp exceptions.
        Label failPopFloat;
        masm.movl(Operand(esp, 0), output);
        masm.and32(Imm32(EXPONENT_MASK), output);
        masm.branch32(Assembler::GreaterThanOrEqual, output, Imm32(TOO_BIG_EXPONENT), &failPopFloat);

        // Load float, perform 32-bit truncation.
        masm.fld32(Operand(esp, 0));
        masm.fisttp(Operand(esp, 0));

        // Load low word, pop 64bits and jump back.
        masm.movl(Operand(esp, 0), output);
        masm.addl(Imm32(sizeof(uint64_t)), esp);
        masm.jump(ool->rejoin());

        masm.bind(&failPopFloat);
        masm.addl(Imm32(sizeof(uint64_t)), esp);
        masm.jump(&fail);
    } else */{
        FloatRegister temp = ToFloatRegister(ins->tempFloat());

        // Try to convert float32 representing integers within 2^32 of a signed
        // integer, by adding/subtracting 2^32 and then trying to convert to int32.
        // This has to be an exact conversion, as otherwise the truncation works
        // incorrectly on the modified value.
        masm.xorps(ScratchFloatReg, ScratchFloatReg);
        masm.ucomiss(input, ScratchFloatReg);
        masm.j(Assembler::Parity, &fail);

        {
            Label positive;
            masm.j(Assembler::Above, &positive);

            masm.loadConstantFloat32(4294967296.f, temp);
            Label skip;
            masm.jmp(&skip);

            masm.bind(&positive);
            masm.loadConstantFloat32(-4294967296.f, temp);
            masm.bind(&skip);
        }

        masm.addss(input, temp);
        masm.cvttss2si(temp, output);
        masm.cvtsi2ss(output, ScratchFloatReg);

        masm.ucomiss(temp, ScratchFloatReg);
        masm.j(Assembler::Parity, &fail);
        masm.j(Assembler::Equal, ool->rejoin());
    }

    masm.bind(&fail);
    {
        saveVolatile(output);

        masm.push(input);
        masm.setupUnalignedABICall(1, output);
        masm.cvtss2sd(input, input);
        masm.passABIArg(input);

        if (gen->compilingAsmJS())
            masm.callWithABI(AsmJSImm_ToInt32);
        else
            masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, js::ToInt32));

        masm.storeCallResult(output);
        masm.pop(input);

        restoreVolatile(output);
    }

    masm.jump(ool->rejoin());
    return true;
}
