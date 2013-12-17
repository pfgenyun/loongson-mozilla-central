/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/MIR.h"
#include "jit/Lowering.h"
#include "Assembler-mips.h"
#include "jit/shared/Lowering-shared-inl.h"

using namespace js;
using namespace js::jit;

LTableSwitch *
LIRGeneratorMIPS::newLTableSwitch(const LAllocation &in, const LDefinition &inputCopy,
                                       MTableSwitch *tableswitch)
{
    return new LTableSwitch(in, inputCopy, temp(), tableswitch);
}

LTableSwitchV *
LIRGeneratorMIPS::newLTableSwitchV(MTableSwitch *tableswitch)
{
    return new LTableSwitchV(temp(), tempFloat(), temp(), tableswitch);
}

bool
LIRGeneratorMIPS::visitInterruptCheck(MInterruptCheck *ins)
{
    LInterruptCheck *lir = new LInterruptCheck();
    if (!add(lir, ins))
        return false;
    if (!assignSafepoint(lir, ins))
        return false;
    return true;
}

bool
LIRGeneratorMIPS::visitGuardShape(MGuardShape *ins)
{
    JS_ASSERT(ins->obj()->type() == MIRType_Object);

    LGuardShape *guard = new LGuardShape(useRegister(ins->obj()));
    if (!assignSnapshot(guard, ins->bailoutKind()))
        return false;
    if (!add(guard, ins))
        return false;
    return redefine(ins, ins->obj());
}

bool
LIRGeneratorMIPS::visitGuardObjectType(MGuardObjectType *ins)
{
    JS_ASSERT(ins->obj()->type() == MIRType_Object);

    LGuardObjectType *guard = new LGuardObjectType(useRegister(ins->obj()));
    if (!assignSnapshot(guard))
        return false;
    if (!add(guard, ins))
        return false;
    return redefine(ins, ins->obj());
}

bool
LIRGeneratorMIPS::visitPowHalf(MPowHalf *ins)
{
    MDefinition *input = ins->input();
    JS_ASSERT(input->type() == MIRType_Double);
    LPowHalfD *lir = new LPowHalfD(useRegisterAtStart(input), temp());
    return defineReuseInput(lir, ins, 0);
}

bool
LIRGeneratorMIPS::lowerForShift(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir,
                                     MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegisterAtStart(lhs));

    if (rhs->isConstant())
        ins->setOperand(1, useOrConstant(rhs));
    else
        //by weizhenwei, 2013.11.28
        ins->setOperand(1, useRegister(rhs));

    return defineReuseInput(ins, mir, 0);
}

bool
LIRGeneratorMIPS::lowerForALU(LInstructionHelper<1, 1, 0> *ins, MDefinition *mir,
                                   MDefinition *input)
{
    ins->setOperand(0, useRegisterAtStart(input));
    return defineReuseInput(ins, mir, 0);
}

bool
LIRGeneratorMIPS::lowerForALU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir,
                                   MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegisterAtStart(lhs));
    ins->setOperand(1, useOrConstant(rhs));
    return defineReuseInput(ins, mir, 0);
}

bool
LIRGeneratorMIPS::lowerForFPU(LInstructionHelper<1, 2, 0> *ins, MDefinition *mir, MDefinition *lhs, MDefinition *rhs)
{
    ins->setOperand(0, useRegisterAtStart(lhs));
    ins->setOperand(1, use(rhs));
    return defineReuseInput(ins, mir, 0);
}

bool
LIRGeneratorMIPS::lowerMulI(MMul *mul, MDefinition *lhs, MDefinition *rhs)
{
    // Note: lhs is used twice, so that we can restore the original value for the
    // negative zero check.
    LMulI *lir = new LMulI(useRegisterAtStart(lhs), useOrConstant(rhs), use(lhs));
    if (mul->fallible() && !assignSnapshot(lir))
        return false;
    return defineReuseInput(lir, mul, 0);
}

bool
LIRGeneratorMIPS::lowerDivI(MDiv *div)
{
    // Division instructions are slow. Division by constant denominators can be
    // rewritten to use other instructions.
    if (div->rhs()->isConstant()) {
        int32_t rhs = div->rhs()->toConstant()->value().toInt32();

        // Check for division by a positive power of two, which is an easy and
        // important case to optimize. Note that other optimizations are also
        // possible; division by negative powers of two can be optimized in a
        // similar manner as positive powers of two, and division by other
        // constants can be optimized by a reciprocal multiplication technique.
        int32_t shift;
        JS_FLOOR_LOG2(shift, rhs);
        if (rhs > 0 && 1 << shift == rhs) {
            LDivPowTwoI *lir = new LDivPowTwoI(useRegisterAtStart(div->lhs()), useRegister(div->lhs()), shift);
            if (div->fallible() && !assignSnapshot(lir))
                return false;
            return defineReuseInput(lir, div, 0);
        }
    }

    //by weizhenwei, 2013.11.28
    LDivI *lir = new LDivI(useRegisterAtStart(div->lhs()), useRegister(div->rhs()));
    if (div->fallible() && !assignSnapshot(lir))
        return false;
    return defineReuseInput(lir, div, 0);
}

bool
LIRGeneratorMIPS::lowerModI(MMod *mod)
{
    if (mod->rhs()->isConstant()) {
        int32_t rhs = mod->rhs()->toConstant()->value().toInt32();
        int32_t shift;
        JS_FLOOR_LOG2(shift, rhs);
        if (rhs > 0 && 1 << shift == rhs) {
            LModPowTwoI *lir = new LModPowTwoI(useRegisterAtStart(mod->lhs()), shift);
            if (mod->fallible() && !assignSnapshot(lir))
                return false;
            return defineReuseInput(lir, mod, 0);
        }
    }

    //by weizhenwei, 2013.11.28
    LModI *lir = new LModI(useRegisterAtStart(mod->lhs()), useRegister(mod->rhs()));
    if (mod->fallible() && !assignSnapshot(lir))
        return false;

    return defineReuseInput(lir, mod, 0);
}

bool
LIRGeneratorMIPS::visitAsmJSNeg(MAsmJSNeg *ins)
{
    if (ins->type() == MIRType_Int32)
        return defineReuseInput(new LNegI(useRegisterAtStart(ins->input())), ins, 0);

    JS_ASSERT(ins->type() == MIRType_Double);
    return defineReuseInput(new LNegD(useRegisterAtStart(ins->input())), ins, 0);
}

bool
LIRGeneratorMIPS::visitAsmJSUDiv(MAsmJSUDiv *div)
{
//by weizhenwei, 2013.11.28
    LAsmJSDiv *lir = new LAsmJSDiv(useRegisterAtStart(div->lhs()), useRegister(div->rhs()));
    return defineReuseInput(lir, div, 0);
}

bool
LIRGeneratorMIPS::visitAsmJSUMod(MAsmJSUMod *mod)
{
//by weizhenwei, 2013.11.28
    LAsmJSMod *lir = new LAsmJSMod(useRegisterAtStart(mod->lhs()), useRegister(mod->rhs()));
    return defineReuseInput(lir, mod, 0);
}

bool
LIRGeneratorMIPS::lowerUrshD(MUrsh *mir)
{
    MDefinition *lhs = mir->lhs();
    MDefinition *rhs = mir->rhs();

    JS_ASSERT(lhs->type() == MIRType_Int32);
    JS_ASSERT(rhs->type() == MIRType_Int32);
    JS_ASSERT(mir->type() == MIRType_Double);

    LUse lhsUse = useRegisterAtStart(lhs);

    //by weizhenwei, 2013.11.28
    LAllocation rhsAlloc = rhs->isConstant() ? useOrConstant(rhs) : useRegister(rhs);

    LUrshD *lir = new LUrshD(lhsUse, rhsAlloc, tempCopy(lhs, 0));
    return define(lir, mir);
}

bool
LIRGeneratorMIPS::lowerConstantDouble(double d, MInstruction *mir)
{
    return define(new LDouble(d), mir);
}

bool
LIRGeneratorMIPS::visitConstant(MConstant *ins)
{
    if (ins->type() == MIRType_Double)
        return lowerConstantDouble(ins->value().toDouble(), ins);

    // Emit non-double constants at their uses.
    if (ins->canEmitAtUses())
        return emitAtUses(ins);

    return LIRGeneratorShared::visitConstant(ins);
}

bool
LIRGeneratorMIPS::lowerTruncateDToInt32(MTruncateToInt32 *ins)
{
    MDefinition *opd = ins->input();
    JS_ASSERT(opd->type() == MIRType_Double);

  //  LDefinition maybeTemp = Assembler::HasSSE3() ? LDefinition::BogusTemp() : tempFloat();
    LDefinition maybeTemp = tempFloat();
    return define(new LTruncateDToInt32(useRegister(opd), maybeTemp), ins);
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

bool
LIRGeneratorMIPS::visitBox(MBox *box)
{
    MDefinition *inner = box->getOperand(0);

    // If the box wrapped a double, it needs a new register.
    if (inner->type() == MIRType_Double)
        return defineBox(new LBoxDouble(useRegisterAtStart(inner), tempCopy(inner, 0)), box);

    if (box->canEmitAtUses())
        return emitAtUses(box);

    if (inner->isConstant())
        return defineBox(new LValue(inner->toConstant()->value()), box);

    LBox *lir = new LBox(use(inner), inner->type());

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

    if (unbox->type() == MIRType_Double) {
        LUnboxDouble *lir = new LUnboxDouble;
        if (unbox->fallible() && !assignSnapshot(lir, unbox->bailoutKind()))
            return false;
        if (!useBox(lir, LUnboxDouble::Input, inner))
            return false;
        return define(lir, unbox);
    }

    // Swap the order we use the box pieces so we can re-use the payload register.
    LUnbox *lir = new LUnbox;
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

    LReturn *ins = new LReturn;
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
LIRGeneratorMIPS::visitStoreTypedArrayElement(MStoreTypedArrayElement *ins)
{
    JS_ASSERT(ins->elements()->type() == MIRType_Elements);
    JS_ASSERT(ins->index()->type() == MIRType_Int32);

    if (ins->isFloatArray())
        JS_ASSERT(ins->value()->type() == MIRType_Double);
    else
        JS_ASSERT(ins->value()->type() == MIRType_Int32);

    LUse elements = useRegister(ins->elements());
    LAllocation index = useRegisterOrConstant(ins->index());
    LAllocation value;

    // For byte arrays, the value has to be in a byte register on mips.
    if (ins->isByteArray())
        //value = useFixed(ins->value(), t6);
        //by weizhenwei, 2013.11.28
        value = useRegister(ins->value());
    else
        value = useRegisterOrNonDoubleConstant(ins->value());
    return add(new LStoreTypedArrayElement(elements, index, value), ins);
}

bool
LIRGeneratorMIPS::visitStoreTypedArrayElementHole(MStoreTypedArrayElementHole *ins)
{
    JS_ASSERT(ins->elements()->type() == MIRType_Elements);
    JS_ASSERT(ins->index()->type() == MIRType_Int32);
    JS_ASSERT(ins->length()->type() == MIRType_Int32);

    if (ins->isFloatArray())
        JS_ASSERT(ins->value()->type() == MIRType_Double);
    else
        JS_ASSERT(ins->value()->type() == MIRType_Int32);

    LUse elements = useRegister(ins->elements());
    LAllocation length = useAnyOrConstant(ins->length());
    LAllocation index = useRegisterOrConstant(ins->index());
    LAllocation value;

    // For byte arrays, the value has to be in a byte register on mips.
    if (ins->isByteArray())
        //value = useFixed(ins->value(), t6);
        //by weizhenwei, 2013.11.28
        value = useRegister(ins->value());
    else
        value = useRegisterOrNonDoubleConstant(ins->value());
    return add(new LStoreTypedArrayElementHole(elements, length, index, value), ins);
}

bool
LIRGeneratorMIPS::visitAsmJSUnsignedToDouble(MAsmJSUnsignedToDouble *ins)
{
    JS_ASSERT(ins->input()->type() == MIRType_Int32);
    LUInt32ToDouble *lir = new LUInt32ToDouble(useRegisterAtStart(ins->input()), temp());
    return define(lir, ins);
}

bool
LIRGeneratorMIPS::visitAsmJSStoreHeap(MAsmJSStoreHeap *ins)
{
    LAsmJSStoreHeap *lir;
    switch (ins->viewType()) {
      case ArrayBufferView::TYPE_INT8: case ArrayBufferView::TYPE_UINT8:
//        lir = new LAsmJSStoreHeap(useRegister(ins->ptr()),
//                                  useFixed(ins->value(), t6));
        //byw weizhenwei, 2013.11.28
        lir = new LAsmJSStoreHeap(useRegister(ins->ptr()), useRegister(ins->value()));
        break;
      case ArrayBufferView::TYPE_INT16: case ArrayBufferView::TYPE_UINT16:
      case ArrayBufferView::TYPE_INT32: case ArrayBufferView::TYPE_UINT32:
      case ArrayBufferView::TYPE_FLOAT32: case ArrayBufferView::TYPE_FLOAT64:
        // For now, don't allow constants. The immediate operand affects
        // instruction layout which affects patching.
        lir = new LAsmJSStoreHeap(useRegisterAtStart(ins->ptr()),
                                  useRegisterAtStart(ins->value()));
        break;
      default: JS_NOT_REACHED("unexpected array type");
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
        //lir = new LStoreTypedArrayElementStatic(useRegister(ins->ptr()),
        //                                        useFixed(ins->value(), t6));
        //by weizhenwei, 2013.11.28
        lir = new LStoreTypedArrayElementStatic(useRegister(ins->ptr()),
                                                useRegister(ins->value()));
        break;
      case ArrayBufferView::TYPE_INT16: case ArrayBufferView::TYPE_UINT16:
      case ArrayBufferView::TYPE_INT32: case ArrayBufferView::TYPE_UINT32:
      case ArrayBufferView::TYPE_FLOAT32: case ArrayBufferView::TYPE_FLOAT64:
        lir = new LStoreTypedArrayElementStatic(useRegisterAtStart(ins->ptr()),
                                                useRegisterAtStart(ins->value()));
        break;
      default: JS_NOT_REACHED("unexpected array type");
    }

    return add(lir, ins);
}

bool
LIRGeneratorMIPS::visitAsmJSLoadFuncPtr(MAsmJSLoadFuncPtr *ins)
{
    return define(new LAsmJSLoadFuncPtr(useRegisterAtStart(ins->index())), ins);
}

LGetPropertyCacheT *
LIRGeneratorMIPS::newLGetPropertyCacheT(MGetPropertyCache *ins)
{
    LDefinition scratch;
    if (ins->type() == MIRType_Double)
        scratch = temp();
    else
        scratch = LDefinition::BogusTemp();
    return new LGetPropertyCacheT(useRegister(ins->object()), scratch);
}
