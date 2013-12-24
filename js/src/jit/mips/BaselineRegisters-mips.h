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

// by wangqing, fp-->s2
//static const Register BaselineFrameReg = fp;
static const Register BaselineFrameReg = s2;
static MOZ_CONSTEXPR_VAR Register BaselineStackReg = sp;

// ValueOperands R0, R1, and R2
static MOZ_CONSTEXPR_VAR ValueOperand R0(t7, t8);
static MOZ_CONSTEXPR_VAR ValueOperand R1(s1, s0);
static MOZ_CONSTEXPR_VAR ValueOperand R2(s3, s4);

// BaselineTailCallReg and BaselineStubReg reuse
// registers from R2.
static MOZ_CONSTEXPR_VAR Register BaselineTailCallReg = s3;
static MOZ_CONSTEXPR_VAR Register BaselineStubReg     = s4;

static MOZ_CONSTEXPR_VAR Register ExtractTemp0        = InvalidReg;
static MOZ_CONSTEXPR_VAR Register ExtractTemp1        = InvalidReg;

// FloatReg0 must be equal to ReturnFloatReg.
static MOZ_CONSTEXPR_VAR FloatRegister FloatReg0      = f0;
static MOZ_CONSTEXPR_VAR FloatRegister FloatReg1      = f4;

} // namespace jit
} // namespace js

#endif // JS_ION

#endif /* jit_mips_BaselineRegisters_mips_h */
