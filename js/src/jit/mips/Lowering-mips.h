/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_mips_Lowering_mips_h
#define jit_mips_Lowering_mips_h

#include "jit/shared/Lowering-shared.h"

namespace js {
namespace jit {

class LIRGeneratorMIPS : public LIRGeneratorMIPSShared
{
  public:
    LIRGeneratorMIPS(MIRGenerator *gen, MIRGraph &graph, LIRGraph &lirGraph)
      : LIRGeneratorMIPSShared(gen, graph, lirGraph)
    { }

  protected:
    // Adds a box input to an instruction, setting operand |n| to the type and
    // |n+1| to the payload.
    bool useBox(LInstruction *lir, size_t n, MDefinition *mir,
                LUse::Policy policy = LUse::REGISTER, bool useAtStart = false);
    bool useBoxFixed(LInstruction *lir, size_t n, MDefinition *mir, Register reg1, Register reg2);

    // It's a trap! On x86, the 1-byte store can only use one of
    // {al,bl,cl,dl,ah,bh,ch,dh}. That means if the register allocator
    // gives us one of {edi,esi,ebp,esp}, we're out of luck. (The formatter
    // will assert on us.) Ideally, we'd just ask the register allocator to
    // give us one of {al,bl,cl,dl}. For now, just useFixed(al).
    LAllocation useByteOpRegister(MDefinition *mir);
    LAllocation useByteOpRegisterOrNonDoubleConstant(MDefinition *mir);

    inline LDefinition tempToUnbox() {
        return LDefinition::BogusTemp();
    }

    LDefinition tempForDispatchCache(MIRType outputType = MIRType_None);

    void lowerUntypedPhiInput(MPhi *phi, uint32_t inputPosition, LBlock *block, size_t lirIndex);
    bool defineUntypedPhi(MPhi *phi, size_t lirIndex);

  public:
    bool visitBox(MBox *box);
    bool visitUnbox(MUnbox *unbox);
    bool visitReturn(MReturn *ret);
    bool visitAsmJSUnsignedToDouble(MAsmJSUnsignedToDouble *ins);
    bool visitAsmJSUnsignedToFloat32(MAsmJSUnsignedToFloat32 *ins);
    bool visitAsmJSLoadHeap(MAsmJSLoadHeap *ins);
    bool visitAsmJSStoreHeap(MAsmJSStoreHeap *ins);
    bool visitAsmJSLoadFuncPtr(MAsmJSLoadFuncPtr *ins);
    bool visitStoreTypedArrayElementStatic(MStoreTypedArrayElementStatic *ins);
    bool lowerPhi(MPhi *phi);

    static bool allowTypedElementHoleCheck() {
        return true;
    }

    static bool allowStaticTypedArrayAccesses() {
        return true;
    }

    static bool allowFloat32Optimizations() {
        return true;
    }

    LTableSwitch *newLTableSwitch(const LAllocation &in, const LDefinition &inputCopy,
                                  MTableSwitch *ins);
    LTableSwitchV *newLTableSwitchV(MTableSwitch *ins);

    bool visitGuardShape(MGuardShape *ins);
    bool visitGuardObjectType(MGuardObjectType *ins);
    bool visitPowHalf(MPowHalf *ins);
    bool lowerForShift(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs,
                       MDefinition *rhs);
    bool lowerForALU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir, MDefinition *input);
    bool lowerForALU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs,
                     MDefinition *rhs);
    bool lowerForFPU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs,
                     MDefinition *rhs);
    bool lowerForBitAndAndBranch(LBitAndAndBranch *baab, MInstruction *mir,
                                 MDefinition *lhs, MDefinition *rhs);
    bool visitConstant(MConstant *ins);
    bool visitAsmJSNeg(MAsmJSNeg *ins);
    bool lowerMulI(MMul *mul, MDefinition *lhs, MDefinition *rhs);
    bool lowerDivI(MDiv *div);
    bool lowerModI(MMod *mod);
    bool lowerUDiv(MDiv *div);
    bool lowerUMod(MMod *mod);
    bool lowerUrshD(MUrsh *mir);
    bool lowerConstantDouble(double d, MInstruction *ins);
    bool lowerConstantFloat32(float d, MInstruction *ins);
    bool lowerTruncateDToInt32(MTruncateToInt32 *ins);
    bool lowerTruncateFToInt32(MTruncateToInt32 *ins);
};

typedef LIRGeneratorMIPS LIRGeneratorSpecific;

} // namespace js
} // namespace jit

#endif /* jit_mips_Lowering_mips_h */
