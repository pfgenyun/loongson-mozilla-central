/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_mips_CodeGenerator_mips_h
#define jit_mips_CodeGenerator_mips_h

#include "Assembler-mips.h"
#include "jit/shared/CodeGenerator-shared.h"
namespace js {
namespace jit {

class OutOfLineBailout;
class OutOfLineUndoALUOperation;
class MulNegativeZeroCheck;
class OutOfLineTableSwitch;
class OutOfLineLoadTypedArrayOutOfBounds;
class OutOfLineTruncate;

class CodeGeneratorMIPS : public CodeGeneratorShared
{

     friend class MoveResolverMIPS;

    CodeGeneratorMIPS *thisFromCtor() {
        return this;
    }

    template <typename T>
    bool bailout(const T &t, LSnapshot *snapshot);

  protected:
    // Label for the common return path.
    HeapLabel *returnLabel_;
    HeapLabel *deoptLabel_;

    inline Operand ToOperand(const LAllocation &a) {
        if (a.isGeneralReg())
            return Operand(a.toGeneralReg()->reg());
        if (a.isFloatReg())
            return Operand(a.toFloatReg()->reg());
        return Operand(StackPointer, ToStackOffset(&a));
    }
    inline Operand ToOperand(const LAllocation *a) {
        return ToOperand(*a);
    }
    inline Operand ToOperand(const LDefinition *def) {
        return ToOperand(def->output());
    }

    MoveResolver::MoveOperand toMoveOperand(const LAllocation *a) const;

    bool bailoutIf(Assembler::Condition condition, LSnapshot *snapshot);
    bool bailoutFrom(Label *label, LSnapshot *snapshot);
    bool bailout(LSnapshot *snapshot);

  protected:
    bool generatePrologue();
    bool generateEpilogue();
    bool generateOutOfLineCode();

    Operand createArrayElementOperand(Register elements, const LAllocation *index);

    void emitCompare(MCompare::CompareType type, const LAllocation *left, const LAllocation *right);

    // Emits a branch that directs control flow to the true block if |cond| is
    // true, and the false block if |cond| is false.
    void emitBranch(Assembler::Condition cond, MBasicBlock *ifTrue, MBasicBlock *ifFalse,
                    Assembler::NaNCond ifNaN = Assembler::NaN_HandledByCond);
    void emitBranch(Assembler::DoubleCondition cond, MBasicBlock *ifTrue, MBasicBlock *ifFalse);

	//by weizhenwei, 2013.11.05
    void emitBranch(Assembler::DoubleCondition cond, const FloatRegister &lhs,
		const FloatRegister &rhs, MBasicBlock *ifTrue, MBasicBlock *ifFalse,
                Assembler::NaNCond ifNaN = Assembler::NaN_HandledByCond);

    //by weizhenwei, 2013.11.07
    void emitSet(Assembler::DoubleCondition cond, const FloatRegister &lhs,
        const FloatRegister &rhs, const Register &dest,
        Assembler::NaNCond ifNaN = Assembler::NaN_HandledByCond);

    bool emitTableSwitchDispatch(MTableSwitch *mir, const Register &index, const Register &base);

  public:
    CodeGeneratorMIPS(MIRGenerator *gen, LIRGraph *graph, MacroAssembler *masm);

  public:
    // Instruction visitors.
    virtual bool visitDouble(LDouble *ins);
    virtual bool visitMinMaxD(LMinMaxD *ins);
    virtual bool visitAbsD(LAbsD *ins);
    virtual bool visitSqrtD(LSqrtD *ins);
    virtual bool visitPowHalfD(LPowHalfD *ins);
    virtual bool visitAddI(LAddI *ins);
    virtual bool visitSubI(LSubI *ins);
    virtual bool visitMulI(LMulI *ins);
    virtual bool visitDivI(LDivI *ins);
    virtual bool visitDivPowTwoI(LDivPowTwoI *ins);
    virtual bool visitModI(LModI *ins);
    virtual bool visitModPowTwoI(LModPowTwoI *ins);
    virtual bool visitBitNotI(LBitNotI *ins);
    virtual bool visitBitOpI(LBitOpI *ins);
    virtual bool visitShiftI(LShiftI *ins);
    virtual bool visitUrshD(LUrshD *ins);
    virtual bool visitMoveGroup(LMoveGroup *group);
    virtual bool visitTestIAndBranch(LTestIAndBranch *test);
    virtual bool visitTestDAndBranch(LTestDAndBranch *test);
    virtual bool visitCompare(LCompare *comp);
    virtual bool visitCompareAndBranch(LCompareAndBranch *comp);
    virtual bool visitCompareD(LCompareD *comp);
    virtual bool visitCompareDAndBranch(LCompareDAndBranch *comp);
    virtual bool visitNotI(LNotI *comp);
    virtual bool visitNotD(LNotD *comp);
    virtual bool visitMathD(LMathD *math);
    virtual bool visitFloor(LFloor *lir);
    virtual bool visitRound(LRound *lir);
    virtual bool visitGuardShape(LGuardShape *guard);
    virtual bool visitGuardObjectType(LGuardObjectType *guard);
    virtual bool visitGuardClass(LGuardClass *guard);
    virtual bool visitEffectiveAddress(LEffectiveAddress *ins);
    //divide AsmJSDivOrMod into AsmJSDiv and AsmJSMod
    //by weizhenwei, 2013.11.28
    virtual bool visitAsmJSDiv(LAsmJSDiv *ins);
    virtual bool visitAsmJSMod(LAsmJSMod *ins);
    virtual bool visitAsmJSPassStackArg(LAsmJSPassStackArg *ins);

    bool visitNegI(LNegI *lir);
    bool visitNegD(LNegD *lir);

    // Out of line visitors.
    bool visitOutOfLineBailout(OutOfLineBailout *ool);
    bool visitOutOfLineUndoALUOperation(OutOfLineUndoALUOperation *ool );
    bool visitMulNegativeZeroCheck(MulNegativeZeroCheck *ool);
    bool visitOutOfLineTableSwitch(OutOfLineTableSwitch *ool);
    bool generateInvalidateEpilogue();

protected:
    ValueOperand ToValue(LInstruction *ins, size_t pos);
    ValueOperand ToOutValue(LInstruction *ins);
    ValueOperand ToTempValue(LInstruction *ins, size_t pos);

    void loadViewTypeElement(ArrayBufferView::ViewType vt, const Address &srcAddr,
                             const LDefinition *out);
    void storeViewTypeElement(ArrayBufferView::ViewType vt, const LAllocation *value,
                              const Address &dstAddr);
    void storeElementTyped(const LAllocation *value, MIRType valueType, MIRType elementType,
                           const Register &elements, const LAllocation *index);


  public:
    bool visitBox(LBox *box);
    bool visitBoxDouble(LBoxDouble *box);
    bool visitUnbox(LUnbox *unbox);
    bool visitValue(LValue *value);
    bool visitOsrValue(LOsrValue *value);
    bool visitLoadSlotV(LLoadSlotV *load);
    bool visitLoadSlotT(LLoadSlotT *load);
    bool visitStoreSlotT(LStoreSlotT *store);
    bool visitLoadElementT(LLoadElementT *load);
    bool visitImplicitThis(LImplicitThis *lir);
    bool visitInterruptCheck(LInterruptCheck *lir);
    bool visitCompareB(LCompareB *lir);
    bool visitCompareBAndBranch(LCompareBAndBranch *lir);
    bool visitCompareV(LCompareV *lir);
    bool visitCompareVAndBranch(LCompareVAndBranch *lir);
    bool visitUInt32ToDouble(LUInt32ToDouble *lir);
    bool visitTruncateDToInt32(LTruncateDToInt32 *ins);
    bool visitLoadTypedArrayElementStatic(LLoadTypedArrayElementStatic *ins);
    bool visitStoreTypedArrayElementStatic(LStoreTypedArrayElementStatic *ins);
    bool visitAsmJSLoadHeap(LAsmJSLoadHeap *ins);
    bool visitAsmJSStoreHeap(LAsmJSStoreHeap *ins);
    bool visitAsmJSLoadGlobalVar(LAsmJSLoadGlobalVar *ins);
    bool visitAsmJSStoreGlobalVar(LAsmJSStoreGlobalVar *ins);
    bool visitAsmJSLoadFuncPtr(LAsmJSLoadFuncPtr *ins);
    bool visitAsmJSLoadFFIFunc(LAsmJSLoadFFIFunc *ins);

    bool visitOutOfLineLoadTypedArrayOutOfBounds(OutOfLineLoadTypedArrayOutOfBounds *ool);
    bool visitOutOfLineTruncate(OutOfLineTruncate *ool);

    void postAsmJSCall(LAsmJSCall *lir);
};

typedef CodeGeneratorMIPS CodeGeneratorSpecific;


// An out-of-line bailout thunk.
class OutOfLineBailout : public OutOfLineCodeBase<CodeGeneratorMIPS>
{
    LSnapshot *snapshot_;

  public:
    OutOfLineBailout(LSnapshot *snapshot)
      : snapshot_(snapshot)
    { }

    bool accept(CodeGeneratorMIPS *codegen);

    LSnapshot *snapshot() const {
        return snapshot_;
    }
};
} // namespace jit
} // namespace js

#endif /* jit_mips_CodeGenerator_mips_h */
