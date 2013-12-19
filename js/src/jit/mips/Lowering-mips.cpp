/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/shared/Lowering-shared.h"
#include "mozilla/MathAlgorithms.h"
#include "jit/MIR.h"
#include "jit/mips/Assembler-mips.h"
#include "jit/shared/Lowering-shared-inl.h"

using namespace js;
using namespace js::jit;

using mozilla::FloorLog2;

LTableSwitch *
LIRGeneratorMIPSShared::newLTableSwitch(const LAllocation &in, const LDefinition &inputCopy,
                                       MTableSwitch *tableswitch)
{
    return new(alloc()) LTableSwitch(in, inputCopy, temp(), tableswitch);
}

LTableSwitchV *
LIRGeneratorMIPSShared::newLTableSwitchV(MTableSwitch *tableswitch)
{
    return new(alloc()) LTableSwitchV(temp(), tempDouble(), temp(), tableswitch);
}

bool
LIRGeneratorMIPSShared::visitGuardShape(MGuardShape *ins)
{
    JS_ASSERT(ins->obj()->type() == MIRType_Object);

    LGuardShape *guard = new(alloc()) LGuardShape(useRegister(ins->obj()));
    if (!assignSnapshot(guard, ins->bailoutKind()))
        return false;
    if (!add(guard, ins))
        return false;
    return redefine(ins, ins->obj());
}

bool
LIRGeneratorMIPSShared::visitGuardObjectType(MGuardObjectType *ins)
{
    JS_ASSERT(ins->obj()->type() == MIRType_Object);

    LGuardObjectType *guard = new(alloc()) LGuardObjectType(useRegister(ins->obj()));
    if (!assignSnapshot(guard))
        return false;
    if (!add(guard, ins))
        return false;
    return redefine(ins, ins->obj());
}

bool
LIRGeneratorMIPSShared::visitPowHalf(MPowHalf *ins)
{
    MDefinition *input = ins->input();
    JS_ASSERT(input->type() == MIRType_Double);
    LPowHalfD *lir = new(alloc()) LPowHalfD(useRegisterAtStart(input));
    return defineReuseInput(lir, ins, 0);
}

bool
LIRGeneratorMIPSShared::lowerForShift(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir,
                                     MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegisterAtStart(lhs));

    // shift operator should be constant or in register ecx
    // x86 can't shift a non-ecx register
    if (rhs->isConstant())
        ins->setOperand(1, useOrConstant(rhs));
    else
        ins->setOperand(1, useFixed(rhs, ecx));

    return defineReuseInput(ins, mir, 0);
}

bool
LIRGeneratorMIPSShared::lowerForALU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir,
                                   MDefinition *input)
{
    ins->setOperand(0, useRegisterAtStart(input));
    return defineReuseInput(ins, mir, 0);
}

bool
LIRGeneratorMIPSShared::lowerForALU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir,
                                   MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegisterAtStart(lhs));
    ins->setOperand(1, useOrConstant(rhs));
    return defineReuseInput(ins, mir, 0);
}

bool
LIRGeneratorMIPSShared::lowerForFPU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegisterAtStart(lhs));
    ins->setOperand(1, use(rhs));
    return defineReuseInput(ins, mir, 0);
}

bool
LIRGeneratorMIPSShared::lowerForBitAndAndBranch(LBitAndAndBranch *baab, MInstruction *mir,
                                               MDefinition *lhs, MDefinition *rhs)
{
    baab->setOperand(0, useRegisterAtStart(lhs));
    baab->setOperand(1, useRegisterOrConstantAtStart(rhs));
    return add(baab, mir);
}

bool
LIRGeneratorMIPSShared::lowerMulI(MMul *mul, MDefinition *lhs, MDefinition *rhs)
{
    // Note: lhs is used twice, so that we can restore the original value for the
    // negative zero check.
    LMulI *lir = new(alloc()) LMulI(useRegisterAtStart(lhs), useOrConstant(rhs), use(lhs));
    if (mul->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
        return false;
    return defineReuseInput(lir, mul, 0);
}

bool
LIRGeneratorMIPSShared::lowerDivI(MDiv *div)
{
    if (div->isUnsigned())
        return lowerUDiv(div);

    // Division instructions are slow. Division by constant denominators can be
    // rewritten to use other instructions.
    if (div->rhs()->isConstant()) {
        int32_t rhs = div->rhs()->toConstant()->value().toInt32();

        // Check for division by a positive power of two, which is an easy and
        // important case to optimize. Note that other optimizations are also
        // possible; division by negative powers of two can be optimized in a
        // similar manner as positive powers of two, and division by other
        // constants can be optimized by a reciprocal multiplication technique.
        int32_t shift = FloorLog2(rhs);
        if (rhs > 0 && 1 << shift == rhs) {
            LAllocation lhs = useRegisterAtStart(div->lhs());
            LDivPowTwoI *lir;
            if (!div->canBeNegativeDividend()) {
                // Numerator is unsigned, so does not need adjusting.
                lir = new(alloc()) LDivPowTwoI(lhs, lhs, shift);
            } else {
                // Numerator is signed, and needs adjusting, and an extra
                // lhs copy register is needed.
                lir = new(alloc()) LDivPowTwoI(lhs, useRegister(div->lhs()), shift);
            }
            if (div->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
                return false;
            return defineReuseInput(lir, div, 0);
        }
    }

    // Optimize x/x. This is quaint, but it also protects the LDivI code below.
    // Since LDivI requires lhs to be in t6, and since the register allocator
    // can't put a virtual register in two physical registers at the same time,
    // this puts rhs in t6 too, and since rhs isn't marked usedAtStart, it
    // would conflict with the t6 output register. (rhs could be marked
    // usedAtStart but for the fact that LDivI clobbers t7 early and rhs could
    // happen to be in t7).
    if (div->lhs() == div->rhs()) {
        if (!div->canBeDivideByZero())
            return define(new(alloc()) LInteger(1), div);

        LDivSelfI *lir = new(alloc()) LDivSelfI(useRegisterAtStart(div->lhs()));
        if (div->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
            return false;
        return define(lir, div);
    }

    LDivI *lir = new(alloc()) LDivI(useFixed(div->lhs(), t6), useRegister(div->rhs()), tempFixed(t7));
    if (div->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
        return false;
    return defineFixed(lir, div, LAllocation(AnyRegister(t6)));
}

bool
LIRGeneratorMIPSShared::lowerModI(MMod *mod)
{
    if (mod->isUnsigned())
        return lowerUMod(mod);

    if (mod->rhs()->isConstant()) {
        int32_t rhs = mod->rhs()->toConstant()->value().toInt32();
        int32_t shift = FloorLog2(rhs);
        if (rhs > 0 && 1 << shift == rhs) {
            LModPowTwoI *lir = new(alloc()) LModPowTwoI(useRegisterAtStart(mod->lhs()), shift);
            if (mod->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
                return false;
            return defineReuseInput(lir, mod, 0);
        }
    }

    // Optimize x%x. The comments in lowerDivI apply here as well, except
    // that we return 0 for all cases except when x is 0 and we're not
    // truncated.
    if (mod->rhs() == mod->lhs()) {
        if (mod->isTruncated())
            return define(new(alloc()) LInteger(0), mod);

        LModSelfI *lir = new(alloc()) LModSelfI(useRegisterAtStart(mod->lhs()));
        if (mod->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
            return false;
        return define(lir, mod);
    }

    LModI *lir = new(alloc()) LModI(useFixedAtStart(mod->lhs(), t6),
                                    useRegister(mod->rhs()),
                                    tempFixed(t6));
    if (mod->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
        return false;
    return defineFixed(lir, mod, LAllocation(AnyRegister(t7)));
}

bool
LIRGeneratorMIPSShared::visitAsmJSNeg(MAsmJSNeg *ins)
{
    if (ins->type() == MIRType_Int32)
        return defineReuseInput(new(alloc()) LNegI(useRegisterAtStart(ins->input())), ins, 0);

    if (ins->type() == MIRType_Float32)
        return defineReuseInput(new(alloc()) LNegF(useRegisterAtStart(ins->input())), ins, 0);

    JS_ASSERT(ins->type() == MIRType_Double);
    return defineReuseInput(new(alloc()) LNegD(useRegisterAtStart(ins->input())), ins, 0);
}

bool
LIRGeneratorMIPSShared::lowerUDiv(MDiv *div)
{
    // Optimize x/x. The comments in lowerDivI apply here as well.
    if (div->lhs() == div->rhs()) {
        if (!div->canBeDivideByZero())
            return define(new(alloc()) LInteger(1), div);

        LDivSelfI *lir = new(alloc()) LDivSelfI(useRegisterAtStart(div->lhs()));
        if (div->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
            return false;
        return define(lir, div);
    }

    LUDivOrMod *lir = new(alloc()) LUDivOrMod(useFixedAtStart(div->lhs(), t6),
                                              useRegister(div->rhs()),
                                              tempFixed(t7));
    if (div->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
        return false;
    return defineFixed(lir, div, LAllocation(AnyRegister(t6)));
}

bool
LIRGeneratorMIPSShared::lowerUMod(MMod *mod)
{
    // Optimize x%x. The comments in lowerModI apply here as well.
    if (mod->lhs() == mod->rhs()) {
        if (mod->isTruncated() || (mod->isUnsigned() && !mod->canBeDivideByZero()))
            return define(new(alloc()) LInteger(0), mod);

        LModSelfI *lir = new(alloc()) LModSelfI(useRegisterAtStart(mod->lhs()));
        if (mod->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
            return false;
        return define(lir, mod);
    }

    LUDivOrMod *lir = new(alloc()) LUDivOrMod(useFixedAtStart(mod->lhs(), t6),
                                              useRegister(mod->rhs()),
                                              tempFixed(t6));
    if (mod->fallible() && !assignSnapshot(lir, Bailout_BaselineInfo))
        return false;
    return defineFixed(lir, mod, LAllocation(AnyRegister(t7)));
}

bool
LIRGeneratorMIPSShared::lowerUrshD(MUrsh *mir)
{
    MDefinition *lhs = mir->lhs();
    MDefinition *rhs = mir->rhs();

    JS_ASSERT(lhs->type() == MIRType_Int32);
    JS_ASSERT(rhs->type() == MIRType_Int32);
    JS_ASSERT(mir->type() == MIRType_Double);

#ifdef JS_CPU_X64
    JS_ASSERT(ecx == rcx);
#endif

    LUse lhsUse = useRegisterAtStart(lhs);
    LAllocation rhsAlloc = rhs->isConstant() ? useOrConstant(rhs) : useFixed(rhs, ecx);

    LUrshD *lir = new(alloc()) LUrshD(lhsUse, rhsAlloc, tempCopy(lhs, 0));
    return define(lir, mir);
}

bool
LIRGeneratorMIPSShared::lowerConstantDouble(double d, MInstruction *mir)
{
    return define(new(alloc()) LDouble(d), mir);
}

bool
LIRGeneratorMIPSShared::lowerConstantFloat32(float f, MInstruction *mir)
{
    return define(new(alloc()) LFloat32(f), mir);
}

bool
LIRGeneratorMIPSShared::visitConstant(MConstant *ins)
{
    if (ins->type() == MIRType_Double)
        return lowerConstantDouble(ins->value().toDouble(), ins);

    if (ins->type() == MIRType_Float32)
        return lowerConstantFloat32(ins->value().toDouble(), ins);

    // Emit non-double constants at their uses.
    if (ins->canEmitAtUses())
        return emitAtUses(ins);

    return LIRGeneratorShared::visitConstant(ins);
}

bool
LIRGeneratorMIPSShared::lowerTruncateDToInt32(MTruncateToInt32 *ins)
{
    MDefinition *opd = ins->input();
    JS_ASSERT(opd->type() == MIRType_Double);

    LDefinition maybeTemp = Assembler::HasSSE3() ? LDefinition::BogusTemp() : tempDouble();
    return define(new(alloc()) LTruncateDToInt32(useRegister(opd), maybeTemp), ins);
}

bool
LIRGeneratorMIPSShared::lowerTruncateFToInt32(MTruncateToInt32 *ins)
{
    MDefinition *opd = ins->input();
    JS_ASSERT(opd->type() == MIRType_Float32);

    LDefinition maybeTemp = Assembler::HasSSE3() ? LDefinition::BogusTemp() : tempFloat32();
    return define(new(alloc()) LTruncateFToInt32(useRegister(opd), maybeTemp), ins);
}

LDefinition
LIRGeneratorMIPS::tempForDispatchCache(MIRType outputType)
{
    // x86 doesn't have a scratch register and we need one for the
    // indirect jump for dispatch-style ICs.
    //
    // Note that currently we only install dispatch-style ICs for parallel
    // execution. If this assumption changes, please change it here.
    if (gen->info().executionMode() != ParallelExecution)
        return LDefinition::BogusTemp();

    // If we don't have an output register, we need a temp.
    if (outputType == MIRType_None)
        return temp();

    // If we have a double output register, we need a temp.
    if (outputType == MIRType_Double)
        return temp();

    // Otherwise we have a non-double output register and we can reuse it.
    return LDefinition::BogusTemp();
}

bool
LIRGeneratorMIPS::useBox(LInstruction *lir, size_t n, MDefinition *mir,
                        LUse::Policy policy, bool useAtStart)
{
    JS_ASSERT(mir->type() == MIRType_Value);

    if (!ensureDefined(mir))
        return false;
    lir->setOperand(n, LUse(mir->virtualRegister(), policy, useAtStart));
    lir->setOperand(n + 1, LUse(VirtualRegisterOfPayload(mir), policy, useAtStart));
    return true;
}

bool
LIRGeneratorMIPS::useBoxFixed(LInstruction *lir, size_t n, MDefinition *mir, Register reg1,
                             Register reg2)
{
    JS_ASSERT(mir->type() == MIRType_Value);
    JS_ASSERT(reg1 != reg2);

    if (!ensureDefined(mir))
        return false;
    lir->setOperand(n, LUse(reg1, mir->virtualRegister()));
    lir->setOperand(n + 1, LUse(reg2, VirtualRegisterOfPayload(mir)));
    return true;
}

LAllocation
LIRGeneratorMIPS::useByteOpRegister(MDefinition *mir)
{
    return useFixed(mir, t6);
}

LAllocation
LIRGeneratorMIPS::useByteOpRegisterOrNonDoubleConstant(MDefinition *mir)
{
    return useFixed(mir, t6);
}

bool
LIRGeneratorMIPS::visitBox(MBox *box)
{
    MDefinition *inner = box->getOperand(0);

    // If the box wrapped a double, it needs a new register.
    if (IsFloatingPointType(inner->type()))
        return defineBox(new(alloc()) LBoxFloatingPoint(useRegisterAtStart(inner), tempCopy(inner, 0),
                                                        inner->type()), box);

    if (box->canEmitAtUses())
        return emitAtUses(box);

    if (inner->isConstant())
        return defineBox(new(alloc()) LValue(inner->toConstant()->value()), box);

    LBox *lir = new(alloc()) LBox(use(inner), inner->type());

    // Otherwise, we should not define a new register for the payload portion
    // of the output, so bypass defineBox().
    uint32_t vreg = getVirtualRegister();
    if (vreg >= MAX_VIRTUAL_REGISTERS)
        return false;

    // Note that because we're using PASSTHROUGH, we do not change the type of
    // the definition. We also do not define the first output as "TYPE",
    // because it has no corresponding payload at (vreg + 1). Also note that
    // although we copy the input's original type for the payload half of the
    // definition, this is only for clarity. PASSTHROUGH definitions are
    // ignored.
    lir->setDef(0, LDefinition(vreg, LDefinition::GENERAL));
    lir->setDef(1, LDefinition(inner->virtualRegister(), LDefinition::TypeFrom(inner->type()),
                               LDefinition::PASSTHROUGH));
    box->setVirtualRegister(vreg);
    return add(lir);
}

bool
LIRGeneratorMIPS::visitUnbox(MUnbox *unbox)
{
    // An unbox on x86 reads in a type tag (either in memory or a register) and
    // a payload. Unlike most instructions conusming a box, we ask for the type
    // second, so that the result can re-use the first input.
    MDefinition *inner = unbox->getOperand(0);

    if (!ensureDefined(inner))
        return false;

    if (IsFloatingPointType(unbox->type())) {
        LUnboxFloatingPoint *lir = new(alloc()) LUnboxFloatingPoint(unbox->type());
        if (unbox->fallible() && !assignSnapshot(lir, unbox->bailoutKind()))
            return false;
        if (!useBox(lir, LUnboxFloatingPoint::Input, inner))
            return false;
        return define(lir, unbox);
    }

    // Swap the order we use the box pieces so we can re-use the payload register.
    LUnbox *lir = new(alloc()) LUnbox;
    lir->setOperand(0, usePayloadInRegisterAtStart(inner));
    lir->setOperand(1, useType(inner, LUse::ANY));

    if (unbox->fallible() && !assignSnapshot(lir, unbox->bailoutKind()))
        return false;

    // Note that PASSTHROUGH here is illegal, since types and payloads form two
    // separate intervals. If the type becomes dead before the payload, it
    // could be used as a Value without the type being recoverable. Unbox's
    // purpose is to eagerly kill the definition of a type tag, so keeping both
    // alive (for the purpose of gcmaps) is unappealing. Instead, we create a
    // new virtual register.
    return defineReuseInput(lir, unbox, 0);
}

bool
LIRGeneratorMIPS::visitReturn(MReturn *ret)
{
    MDefinition *opd = ret->getOperand(0);
    JS_ASSERT(opd->type() == MIRType_Value);

    LReturn *ins = new(alloc()) LReturn;
    ins->setOperand(0, LUse(JSReturnReg_Type));
    ins->setOperand(1, LUse(JSReturnReg_Data));
    return fillBoxUses(ins, 0, opd) && add(ins);
}

bool
LIRGeneratorMIPS::defineUntypedPhi(MPhi *phi, size_t lirIndex)
{
    LPhi *type = current->getPhi(lirIndex + VREG_TYPE_OFFSET);
    LPhi *payload = current->getPhi(lirIndex + VREG_DATA_OFFSET);

    uint32_t typeVreg = getVirtualRegister();
    if (typeVreg >= MAX_VIRTUAL_REGISTERS)
        return false;

    phi->setVirtualRegister(typeVreg);

    uint32_t payloadVreg = getVirtualRegister();
    if (payloadVreg >= MAX_VIRTUAL_REGISTERS)
        return false;
    JS_ASSERT(typeVreg + 1 == payloadVreg);

    type->setDef(0, LDefinition(typeVreg, LDefinition::TYPE));
    payload->setDef(0, LDefinition(payloadVreg, LDefinition::PAYLOAD));
    annotate(type);
    annotate(payload);
    return true;
}

void
LIRGeneratorMIPS::lowerUntypedPhiInput(MPhi *phi, uint32_t inputPosition, LBlock *block, size_t lirIndex)
{
    MDefinition *operand = phi->getOperand(inputPosition);
    LPhi *type = block->getPhi(lirIndex + VREG_TYPE_OFFSET);
    LPhi *payload = block->getPhi(lirIndex + VREG_DATA_OFFSET);
    type->setOperand(inputPosition, LUse(operand->virtualRegister() + VREG_TYPE_OFFSET, LUse::ANY));
    payload->setOperand(inputPosition, LUse(VirtualRegisterOfPayload(operand), LUse::ANY));
}

bool
LIRGeneratorMIPS::visitAsmJSUnsignedToDouble(MAsmJSUnsignedToDouble *ins)
{
    JS_ASSERT(ins->input()->type() == MIRType_Int32);
    LAsmJSUInt32ToDouble *lir = new(alloc()) LAsmJSUInt32ToDouble(useRegisterAtStart(ins->input()), temp());
    return define(lir, ins);
}

bool
LIRGeneratorMIPS::visitAsmJSUnsignedToFloat32(MAsmJSUnsignedToFloat32 *ins)
{
    JS_ASSERT(ins->input()->type() == MIRType_Int32);
    LAsmJSUInt32ToFloat32 *lir = new(alloc()) LAsmJSUInt32ToFloat32(useRegisterAtStart(ins->input()), temp());
    return define(lir, ins);
}

bool
LIRGeneratorMIPS::visitAsmJSLoadHeap(MAsmJSLoadHeap *ins)
{
    MDefinition *ptr = ins->ptr();
    LAllocation ptrAlloc;
    JS_ASSERT(ptr->type() == MIRType_Int32);

    // For the x86 it is best to keep the 'ptr' in a register if a bounds check is needed.
    if (ptr->isConstant() && ins->skipBoundsCheck()) {
        int32_t ptrValue = ptr->toConstant()->value().toInt32();
        // A bounds check is only skipped for a positive index.
        JS_ASSERT(ptrValue >= 0);
        ptrAlloc = LAllocation(ptr->toConstant()->vp());
    } else {
        ptrAlloc = useRegisterAtStart(ptr);
    }
    LAsmJSLoadHeap *lir = new(alloc()) LAsmJSLoadHeap(ptrAlloc);
    return define(lir, ins);
}

bool
LIRGeneratorMIPS::visitAsmJSStoreHeap(MAsmJSStoreHeap *ins)
{
    MDefinition *ptr = ins->ptr();
    LAsmJSStoreHeap *lir;
    JS_ASSERT(ptr->type() == MIRType_Int32);

    if (ptr->isConstant() && ins->skipBoundsCheck()) {
        int32_t ptrValue = ptr->toConstant()->value().toInt32();
        JS_ASSERT(ptrValue >= 0);
        LAllocation ptrAlloc = LAllocation(ptr->toConstant()->vp());
        switch (ins->viewType()) {
          case ArrayBufferView::TYPE_INT8: case ArrayBufferView::TYPE_UINT8:
            // See comment below.
            lir = new(alloc()) LAsmJSStoreHeap(ptrAlloc, useFixed(ins->value(), t6));
            break;
          case ArrayBufferView::TYPE_INT16: case ArrayBufferView::TYPE_UINT16:
          case ArrayBufferView::TYPE_INT32: case ArrayBufferView::TYPE_UINT32:
          case ArrayBufferView::TYPE_FLOAT32: case ArrayBufferView::TYPE_FLOAT64:
            // See comment below.
            lir = new(alloc()) LAsmJSStoreHeap(ptrAlloc, useRegisterAtStart(ins->value()));
            break;
          default: MOZ_ASSUME_UNREACHABLE("unexpected array type");
        }
        return add(lir, ins);
    }

    switch (ins->viewType()) {
      case ArrayBufferView::TYPE_INT8: case ArrayBufferView::TYPE_UINT8:
        // See comment for LIRGeneratorMIPS::useByteOpRegister.
        lir = new(alloc()) LAsmJSStoreHeap(useRegister(ins->ptr()), useFixed(ins->value(), t6));
        break;
      case ArrayBufferView::TYPE_INT16: case ArrayBufferView::TYPE_UINT16:
      case ArrayBufferView::TYPE_INT32: case ArrayBufferView::TYPE_UINT32:
      case ArrayBufferView::TYPE_FLOAT32: case ArrayBufferView::TYPE_FLOAT64:
        // For now, don't allow constant values. The immediate operand
        // affects instruction layout which affects patching.
        lir = new(alloc()) LAsmJSStoreHeap(useRegisterAtStart(ptr), useRegisterAtStart(ins->value()));
        break;
      default: MOZ_ASSUME_UNREACHABLE("unexpected array type");
    }

    return add(lir, ins);
}

bool
LIRGeneratorMIPS::visitStoreTypedArrayElementStatic(MStoreTypedArrayElementStatic *ins)
{
    // The code generated for StoreTypedArrayElementStatic is identical to that
    // for AsmJSStoreHeap, and the same concerns apply.
    LStoreTypedArrayElementStatic *lir;
    switch (ins->viewType()) {
      case ArrayBufferView::TYPE_INT8: case ArrayBufferView::TYPE_UINT8:
      case ArrayBufferView::TYPE_UINT8_CLAMPED:
        lir = new(alloc()) LStoreTypedArrayElementStatic(useRegister(ins->ptr()),
                                                         useFixed(ins->value(), t6));
        break;
      case ArrayBufferView::TYPE_INT16: case ArrayBufferView::TYPE_UINT16:
      case ArrayBufferView::TYPE_INT32: case ArrayBufferView::TYPE_UINT32:
      case ArrayBufferView::TYPE_FLOAT32: case ArrayBufferView::TYPE_FLOAT64:
        lir = new(alloc()) LStoreTypedArrayElementStatic(useRegisterAtStart(ins->ptr()),
                                                         useRegisterAtStart(ins->value()));
        break;
      default: MOZ_ASSUME_UNREACHABLE("unexpected array type");
    }

    return add(lir, ins);
}

bool
LIRGeneratorMIPS::visitAsmJSLoadFuncPtr(MAsmJSLoadFuncPtr *ins)
{
    return define(new(alloc()) LAsmJSLoadFuncPtr(useRegisterAtStart(ins->index())), ins);
}
