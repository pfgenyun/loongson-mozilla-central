/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_mips_BaselineRegisters_mips_h
#define jit_mips_BaselineRegisters_mips_h

#ifdef JS_ION

#include "jit/IonMacroAssembler.h"

namespace js {
namespace jit {

static const Register BaselineFrameReg = fp;
static const Register BaselineStackReg = sp;

// ValueOperands R0, R1, and R2
static const ValueOperand R0(v1, v0);
static const ValueOperand R1(s1, s0);
static const ValueOperand R2(t7, t6);

// BaselineTailCallReg and BaselineStubReg reuse
// registers from R2.
static const Register BaselineTailCallReg = t6;
static const Register BaselineStubReg     = t7;

static const Register ExtractTemp0        = InvalidReg;
static const Register ExtractTemp1        = InvalidReg;

// FloatReg0 must be equal to ReturnFloatReg.
static const FloatRegister FloatReg0      = f0;
static const FloatRegister FloatReg1      = f2;

} // namespace jit
} // namespace js

#endif // JS_ION

#endif /* jit_x86_BaselineRegisters_x86_h */
