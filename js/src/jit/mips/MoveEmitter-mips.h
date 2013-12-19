/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_MoveEmitter_mips_shared_h
#define jit_MoveEmitter_mips_shared_h

#if defined(JS_CPU_X86)
# include "jit/x86/MacroAssembler-x86.h"
#elif defined(JS_CPU_X64)
# include "jit/x64/MacroAssembler-x64.h"
#elif defined(JS_CPU_ARM)
# include "jit/arm/MacroAssembler-arm.h"
#elif defined(JS_CPU_MIPS)
# include "jit/mips/MacroAssembler-mips.h"
#endif
#include "jit/MoveResolver.h"

namespace js {
namespace jit {

class CodeGenerator;

class MoveEmitterMIPS
{
    bool inCycle_;
    MacroAssemblerSpecific &masm;

    // Original stack push value.
    uint32_t pushedAtStart_;

    // This is a store stack offset for the cycle-break spill slot, snapshotting
    // codegen->framePushed_ at the time it is allocated. -1 if not allocated.
    int32_t pushedAtCycle_;

    void assertDone();
    Address cycleSlot();
    Address toAddress(const MoveOperand &operand) const;
    Operand toOperand(const MoveOperand &operand) const;
    Operand toPopOperand(const MoveOperand &operand) const;

    size_t characterizeCycle(const MoveResolver &moves, size_t i,
                             bool *allGeneralRegs, bool *allFloatRegs);
    bool maybeEmitOptimizedCycle(const MoveResolver &moves, size_t i,
                                 bool allGeneralRegs, bool allFloatRegs, size_t swapCount);
    void emitGeneralMove(const MoveOperand &from, const MoveOperand &to);
    void emitDoubleMove(const MoveOperand &from, const MoveOperand &to);
    void breakCycle(const MoveOperand &to, MoveOp::Kind kind);
    void completeCycle(const MoveOperand &to, MoveOp::Kind kind);

  public:
    MoveEmitterMIPS(MacroAssemblerSpecific &masm);
    ~MoveEmitterMIPS();
    void emit(const MoveResolver &moves);
    void finish();
};

typedef MoveEmitterMIPS MoveEmitter;

} // ion
} // js

#endif /* jit_MoveEmitter_mips_shared_h */
