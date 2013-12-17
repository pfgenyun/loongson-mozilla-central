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

// by wangqing
static const Register BaselineFrameReg = fp;//1031
static const Register BaselineStackReg = sp;

// ValueOperands R0, R1, and R2
// xsb :need fix
static const ValueOperand R0(t7, t8);
static const ValueOperand R1(s1, s0);
static const ValueOperand R2(s3, s4);

// BaselineTailCallReg and BaselineStubReg reuse
// registers from R2.
static const Register BaselineTailCallReg = s3;
static const Register BaselineStubReg     = s4;

static const Register ExtractTemp0        = InvalidReg;
static const Register ExtractTemp1        = InvalidReg;

// FloatReg0 must be equal to ReturnFloatReg.
static const FloatRegister FloatReg0      = f0;
static const FloatRegister FloatReg1      = f4;//hwj 1031

} // namespace jit
} // namespace js

#endif // JS_ION

#endif /* jit_mips_BaselineRegisters_mips_h */
