/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/BaselineJIT.h"
#include "jit/BaselineIC.h"
#include "jit/BaselineCompiler.h"
#include "jit/BaselineHelpers.h"
#include "jit/IonLinker.h"

using namespace js;
using namespace js::jit;

namespace js {
namespace jit {

bool
ICCompare_Double::Compiler::generateStubCode(MacroAssembler &masm)
{
	// by wangqing, 2013-11-28
    Label failure, notNaN, isTrue;
    /* if R0 is a double, load it into dest. If R0 is int32,  
	   convert it to double. Else, branch to failure.
	   by wangqing, 2013-11-28
	*/	
	    Label isDouble, done;

		masm.movl(ImmTag(JSVAL_TAG_CLEAR), cmpTempRegister);
		masm.sltu(cmpTempRegister, R0.typeReg(), cmpTempRegister);
		masm.bgtz(cmpTempRegister, &isDouble);
		masm.nop();

		masm.movl(ImmTag(JSVAL_TAG_INT32), cmpTempRegister);
		masm.bne(R0.typeReg(), cmpTempRegister, &failure);
		masm.nop();

        masm.convertInt32ToDouble(R0.payloadReg(), FloatReg0);
        masm.b(&done);
		masm.nop();

        masm.bindBranch(&isDouble);
        masm.unboxDouble(R0, FloatReg0);

        masm.bindBranch(&done);

	/* if R1 is a double, load it into dest. If R1 is int32,  
	   convert it to double. Else, branch to failure.
	   by wangqing, 2013-11-28
	*/	
	    Label isDouble1, done1;

		masm.movl(ImmTag(JSVAL_TAG_CLEAR), cmpTempRegister);
		masm.sltu(cmpTempRegister, R1.typeReg(), cmpTempRegister);
		masm.bgtz(cmpTempRegister, &isDouble1);
		masm.nop();

		masm.movl(ImmTag(JSVAL_TAG_INT32), cmpTempRegister);
		masm.bne(R1.typeReg(), cmpTempRegister, &failure);
		masm.nop();

        masm.convertInt32ToDouble(R1.payloadReg(), FloatReg1);
        masm.b(&done1);
		masm.nop();

        masm.bindBranch(&isDouble1);
        masm.unboxDouble(R1, FloatReg1);

        masm.bindBranch(&done1);

    Register dest = R0.scratchReg();

    Assembler::DoubleCondition cond = JSOpToDoubleCondition(op);
    masm.addiu(dest, zero, 1);
    masm.branchDoubleLocal(cond, FloatReg0, FloatReg1, &isTrue);
    masm.xorl(dest, dest);
    masm.bindBranch(&isTrue);


    // Check for NaN, if needed.
    Assembler::NaNCond nanCond = Assembler::NaNCondFromDoubleCondition(cond);
    if (nanCond != Assembler::NaN_HandledByCond) {
	  // check DoubleOrdered, by wangqing, 2013-11-28
	  masm.cud(FloatReg0, FloatReg1);
	  masm.bc1f(&notNaN);
	  masm.nop();
      masm.mov(Imm32(nanCond == Assembler::NaN_IsTrue), dest);
      masm.bindBranch(&notNaN);
    }

    masm.tagValue(JSVAL_TYPE_BOOLEAN, dest, R0);
    EmitReturnFromIC(masm);

    // Failure case - jump to next stub
    masm.bindBranch(&failure);
    EmitStubGuardFailure(masm);
    return true;
}

// ICCompare_Int32
bool
ICCompare_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    // Guard that R0 is an integer and R1 is an integer.
    Label failure;
    // by wangqing, 2013-11-22
    masm.movl(ImmTag(JSVAL_TAG_INT32), cmpTempRegister);
    masm.bne(R0.typeReg(), cmpTempRegister, &failure);
    masm.nop();

    masm.movl(ImmTag(JSVAL_TAG_INT32), cmpTempRegister);
    masm.bne(R1.typeReg(), cmpTempRegister, &failure);
    masm.nop();

    // Compare payload regs of R0 and R1.
    Assembler::Condition cond = JSOpToCondition(op, /* signed = */true);
    masm.cmpl(R0.payloadReg(), R1.payloadReg());
    masm.setCC(cond, R0.payloadReg());
    masm.movzxbl(R0.payloadReg(), R0.payloadReg());

    // Box the result and return
    masm.tagValue(JSVAL_TYPE_BOOLEAN, R0.payloadReg(), R0);
    EmitReturnFromIC(masm);

    // Failure case - jump to next stub
    masm.bindBranch(&failure);
    EmitStubGuardFailure(masm);
    return true;
}

// ICBinaryArith_Int32
// by wangqing, 2013-11-22
bool
ICBinaryArith_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    // Guard that R0 is an integer and R1 is an integer.
    Label failure;
	
    masm.movl(ImmTag(JSVAL_TAG_INT32), cmpTempRegister);
    masm.bne(R0.typeReg(), cmpTempRegister, &failure);
    masm.nop();

    masm.movl(ImmTag(JSVAL_TAG_INT32), cmpTempRegister);
    masm.bne(R1.typeReg(), cmpTempRegister, &failure);
    masm.nop();

    // Add R0 and R1.  Don't need to explicitly unbox, just use the TailCallReg which
    // should be available.
    Register scratchReg = BaselineTailCallReg;

    Label revertRegister, maybeNegZero;
    // xsb:fix me
    // fixed by weizhenwei, 2013.11.05
    switch(op_) {
      case JSOP_ADD:
        // Add R0 and R1.  Don't need to explicitly unbox.
      
        // mov tow oprand to cmp registers to prepared for Overflow check.
        masm.cmpl(R0.payloadReg(), R1.payloadReg());
        masm.negl(cmpTemp2Register);

        // do the add
        masm.movl(R0.payloadReg(), scratchReg);
        masm.addl(R1.payloadReg(), scratchReg);

        // Just jump to failure on overflow.  R0 and R1 are preserved, so we can just jump to
        // the next stub.
    	masm.xorInsn(dataTempRegister, cmpTempRegister, cmpTemp2Register);
		masm.bgez(dataTempRegister, 7);
		masm.nop();
		masm.subu(dataTempRegister, cmpTempRegister, cmpTemp2Register);
		masm.xorInsn(dataTempRegister, dataTempRegister, cmpTempRegister);
		masm.bgez(dataTempRegister, 3);
		masm.nop();
		masm.b(&failure);
		masm.nop();

        // Just overwrite the payload, the tag is still fine.
        masm.movl(scratchReg, R0.payloadReg());
        break;
      case JSOP_SUB:
        masm.cmpl(R0.payloadReg(), R1.payloadReg());

        masm.movl(R0.payloadReg(), scratchReg);
        masm.subl(R1.payloadReg(), scratchReg);

		// jump to failure on overflow, by wangqing, 2013-11-22
		masm.xorInsn(dataTempRegister, cmpTempRegister, cmpTemp2Register);
		masm.bgez(dataTempRegister, 7);
		masm.nop();
		masm.subu(dataTempRegister, cmpTempRegister, cmpTemp2Register);
		masm.xorInsn(dataTempRegister, dataTempRegister, cmpTempRegister);
		masm.bgez(dataTempRegister, 3);
		masm.nop();
		masm.b(&failure);
		masm.nop();

        masm.movl(scratchReg, R0.payloadReg());
        break;
      case JSOP_MUL:
        masm.movl(R0.payloadReg(), scratchReg);
        masm.imull(R1.payloadReg(), scratchReg);

        // test whether signed multiply overflow. by weizhenwei, 2013.11.01
        masm.mfhi(cmpTempRegister);
        masm.mflo(cmpTemp2Register);
        masm.sarl(Imm32(0x1f), cmpTemp2Register);
        masm.bne(cmpTempRegister, cmpTemp2Register, &failure);
		masm.nop();

		masm.beq(scratchReg, zero, &maybeNegZero);
		masm.nop();

        masm.movl(scratchReg, R0.payloadReg());
        break;
      case JSOP_DIV:
        // Prevent division by 0.
		masm.beq(R1.payloadReg(), zero, &failure);
		masm.nop();

        // Prevent negative 0 and -2147483648 / -1.
		/* rewrite testl(reg, imm), avoid use cmpTemp2Register by wangqing, 2013-11-22*/
		masm.movl(R0.payloadReg(), cmpTempRegister);
		masm.andl(Imm32(0x7fffffff), cmpTempRegister);
		masm.beq(cmpTempRegister, zero, &failure);
		masm.nop();

        // Preserve R0.payloadReg()
		masm.div(R0.payloadReg(), R1.payloadReg());

        // A remainder implies a double result.
        // by weizhenwei, 2013.11.02
        masm.mfhi(cmpTempRegister);
		masm.bne(cmpTempRegister, zero, &failure);
		masm.nop();

        // by weizhenwei, 2013.11.05
        masm.mflo(R0.payloadReg());
        break;
      case JSOP_MOD:
      {
        // x % 0 always results in NaN.
		masm.beq(R1.payloadReg(), zero, &failure);
		masm.nop();

        // Prevent negative 0 and -2147483648 % -1.
		/* rewrite testl(reg, imm), avoid use cmpTemp2Register by wangqing, 2013-11-22*/
		masm.movl(R0.payloadReg(), cmpTempRegister);
		masm.andl(Imm32(0x7fffffff), cmpTempRegister);
		masm.beq(cmpTempRegister, zero, &failure);
		masm.nop();

    	masm.div(R0.payloadReg(), R1.payloadReg());

        // Fail when we would need a negative remainder.
        Label done;
        masm.mfhi(cmpTempRegister);
	    masm.bne(cmpTempRegister, zero, &done);
	    masm.nop();

	    masm.bltz(R0.payloadReg(), &failure);
	    masm.nop();

	    masm.bltz(R1.payloadReg(), &failure);
	    masm.nop();

        masm.bindBranch(&done);

        //move reminder to R0.payloadReg, by weizhenwei, 2013.11.05
        masm.mfhi(R0.payloadReg());
        break;
      }
      case JSOP_BITOR:
        // We can overide R0, because the instruction is unfailable.
        // The R0.typeReg() is also still intact.
        masm.orl(R1.payloadReg(), R0.payloadReg());
        break;
      case JSOP_BITXOR:
        masm.xorl(R1.payloadReg(), R0.payloadReg());
        break;
      case JSOP_BITAND:
        masm.andl(R1.payloadReg(), R0.payloadReg());
        break;
      case JSOP_LSH:
        // R0.payloadReg() is result, R1.payloadReg90 is shiftAmount.
        // rewrite by weizhenwei, 2013.11.06
        masm.sllv(R0.payloadReg(), R0.payloadReg(), R1.payloadReg());

        // We need to tag again, because we overwrote it.
        masm.tagValue(JSVAL_TYPE_INT32, R0.payloadReg(), R0);
        break;
      case JSOP_RSH:
        // R0.payloadReg() is result, R1.payloadReg90 is shiftAmount.
        // rewrite by weizhenwei, 2013.11.06
        masm.srav(R0.payloadReg(), R0.payloadReg(), R1.payloadReg());

        // We need to tag again, because we overwrote it.
        masm.tagValue(JSVAL_TYPE_INT32, R0.payloadReg(), R0);
        break;
      case JSOP_URSH:
        if (!allowDouble_)
            masm.movl(R0.payloadReg(), scratchReg);

        // R0.payloadReg() is result, R1.payloadReg() is shiftAmount.
        // rewrite by weizhenwei, 2013.11.06
        masm.srlv(R0.payloadReg(), R0.payloadReg(), R1.payloadReg());

        // by wangqing. 2013-11-22
        if (allowDouble_) {
            Label toUint;
	        masm.bltz(R0.payloadReg(),&toUint);
	        masm.nop();

            // Box and return.
            masm.tagValue(JSVAL_TYPE_INT32, R0.payloadReg(), R0);
            EmitReturnFromIC(masm);

            masm.bindBranch(&toUint);
            masm.convertUInt32ToDouble(R0.payloadReg(), ScratchFloatReg);
            masm.boxDouble(ScratchFloatReg, R0);
        } else {
	        masm.bltz(R0.payloadReg(),&revertRegister);
	        masm.nop();
            masm.tagValue(JSVAL_TYPE_INT32, R0.payloadReg(), R0);
        }
        break;
      default:
       JS_NOT_REACHED("Unhandled op for BinaryArith_Int32.  ");
       return false;
    }

    // Return.
    EmitReturnFromIC(masm);

    switch(op_) {
      case JSOP_MUL:
        masm.bindBranch(&maybeNegZero);

        // Result is -0 if exactly one of lhs or rhs is negative.
        masm.movl(R0.payloadReg(), scratchReg);
        masm.orl(R1.payloadReg(), scratchReg);
        //add by QuQiuwen;
	    masm.bltz(scratchReg, &failure);
	    masm.nop();

        // Result is +0.
        masm.xorl(R0.payloadReg(), R0.payloadReg());
        EmitReturnFromIC(masm);
        break;
      case JSOP_URSH:
        // Revert the content of R0 in the fallible >>> case.
        if (!allowDouble_) {
            masm.bindBranch(&revertRegister);
            masm.tagValue(JSVAL_TYPE_INT32, scratchReg, R0);
        }
        break;
      default:
        // No special failure handling required.
        // Fall through to failure.
        break;
    }

    // Failure case - jump to next stub
    masm.bindBranch(&failure);
    EmitStubGuardFailure(masm);

    return true;
}

bool
ICUnaryArith_Int32::Compiler::generateStubCode(MacroAssembler &masm)
{
    Label failure;
    // by wangqing, 2013-11-26
    masm.movl(ImmTag(JSVAL_TAG_INT32), cmpTempRegister);
    masm.bne(R0.typeReg(), cmpTempRegister, &failure);
    masm.nop();

    switch (op) {
      case JSOP_BITNOT:
        masm.notl(R0.payloadReg());
        break;
      case JSOP_NEG:
        // Guard against 0 and MIN_INT, both result in a double.
		/* rewrite testl(reg, imm), avoid use cmpTemp2Register by wangqing, 2013-11-22*/
	    masm.movl(R0.payloadReg(), cmpTempRegister);
		masm.andl(Imm32(0x7fffffff), cmpTempRegister);
		masm.beq(cmpTempRegister, zero, &failure);
		masm.nop();
        masm.negl(R0.payloadReg());
        break;
      default:
        JS_NOT_REACHED("Unexpected op");
        return false;
    }

    EmitReturnFromIC(masm);

    masm.bindBranch(&failure);
    EmitStubGuardFailure(masm);
    return true;
}

} // namespace jit
} // namespace js
