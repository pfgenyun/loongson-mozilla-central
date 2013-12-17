/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=4 sw=4 et tw=99:
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jscompartment.h"
#include "assembler/assembler/MacroAssembler.h"
#include "jit/BaselineJIT.h"
#include "jit/IonCompartment.h"
#include "jit/IonLinker.h"
#include "jit/IonFrames.h"
#include "jit/IonSpewer.h"
#include "jit/Bailouts.h"
#include "jit/VMFunctions.h"
#include "jit/mips/BaselineHelpers-mips.h"
#include "jit/ExecutionModeInlines.h"

#include "jsscriptinlines.h"


using namespace js;
using namespace js::jit;

enum EnterJitEbpArgumentOffset {
    ARG_JITCODE         = 2 * sizeof(void *),
    ARG_ARGC            = 3 * sizeof(void *),
    ARG_ARGV            = 4 * sizeof(void *),
    ARG_STACKFRAME      = 5 * sizeof(void *),
    ARG_CALLEETOKEN     = 6 * sizeof(void *),
    ARG_SCOPECHAIN      = 7 * sizeof(void *),
    ARG_STACKVALUES     = 8 * sizeof(void *),
    ARG_RESULT          = 9 * sizeof(void *)
};

/*
 * Generates a trampoline for a C++ function with the EnterIonCode signature,
 * using the standard cdecl calling convention.
 */
IonCode *
IonRuntime::generateEnterJIT(JSContext *cx, EnterJitType type)
{
    MacroAssembler masm(cx);

    // Save return address, on mips ra saved mannually.
    masm.push(ra);
    // Save old stack frame pointer, set new stack frame pointer.
    masm.push(fp);
    masm.movl(sp, fp);

    // Save non-volatile registers. These must be saved by the trampoline,
    // rather than the JIT'd code, because they are scanned by the conservative
    // scanner.
    masm.push(t0);
    masm.push(t1);
    masm.push(t2);
    masm.push(t3);
    masm.push(t4);
    masm.push(t5);

    masm.push(t6);
    masm.push(t7);

    masm.push(s0);
    masm.push(s1);
    masm.push(s2);
    masm.push(s3);
    masm.push(s4);
    masm.push(s5);
    masm.push(s6);
    masm.push(s7);

    masm.push(t8);

    //save args to reserved slots
    masm.movl(a0, Operand(fp, ARG_JITCODE));
    masm.movl(a1, Operand(fp, ARG_ARGC));
    masm.movl(a2, Operand(fp, ARG_ARGV));
    masm.movl(a3, Operand(fp, ARG_STACKFRAME));

    masm.movl(sp, s0);

    masm.movl(Operand(fp, ARG_ARGC), t6);
    masm.shll(Imm32(3), t6);

    // We need to ensure that the stack is aligned on a 12-byte boundary, so inside the JIT function the stack is 16-byte aligned.
    // Our stack right now might not be aligned on some platforms (win32, gcc) so we factor this possibility in, and simulate what the new stack address would be.
    //   +argc * 8 for arguments
    //   +4 for pushing alignment
    //   +4 for pushing the callee token
    //   +4 for pushing the return address
    masm.movl(sp, t8);
    masm.subl(t6, t8);
    masm.subl(Imm32(12), t8);

    masm.andl(Imm32(15), t8);
    masm.subl(t8, sp);

    /***************************************************************
    Loop over argv vector, push arguments onto stack in reverse order
    ***************************************************************/

    masm.movl(Operand(fp, ARG_ARGV), s1);
    masm.addl(s1, t6);
    {
	// by wangqing, 2013-11-21
        Label header, footer;
        masm.bindBranch(&header);

		masm.sltu(cmpTempRegister, s1, t6);
		masm.blez(cmpTempRegister, &footer);
		masm.nop();

        masm.subl(Imm32(8), t6);

        // Push what eax points to on stack, a Value is 2 words
        masm.push(Operand(t6, 4));
        masm.push(Operand(t6, 0));

        masm.b(&header);
		masm.nop();
        masm.bindBranch(&footer);
    }


    // Push the number of actual arguments.  
    // |result| is used to store the actual number of arguments without adding an extra argument to the enter
    // JIT.
    masm.mov(Operand(fp, ARG_RESULT), t6);
    masm.unboxInt32(Address(t6, 0x0), t6);
    masm.push(t6);

    // Push the callee token.
    masm.push(Operand(fp, ARG_CALLEETOKEN));

    // Load the StackFrame address into the OsrFrameReg.
    // This address is also used for setting the constructing bit on all paths.
    masm.movl(Operand(fp, ARG_STACKFRAME), OsrFrameReg);

    /*****************************************************************
    Push the number of bytes we've pushed so far on the stack and call
    *****************************************************************/
    // Create a frame descriptor.
    masm.subl(sp, s0);
    masm.makeFrameDescriptor(s0, IonFrame_Entry);
    masm.push(s0);
    
    //hwj
    CodeLabel returnLabel;
    if (type == EnterJitBaseline) {
        // Handle OSR.
        GeneralRegisterSet regs(GeneralRegisterSet::All());
        regs.take(JSReturnOperand);
        regs.takeUnchecked(OsrFrameReg);

        Register scratch = regs.takeAny();

        Label notOsr;
		masm.beq(OsrFrameReg, zero, &notOsr);
		masm.nop();

        Register numStackValues = regs.takeAny();
        masm.movl(Operand(fp, ARG_STACKVALUES), numStackValues);

        Register jitcode = regs.takeAny();
        masm.movl(Operand(fp, ARG_JITCODE), jitcode);

        // Push return address, previous frame pointer.
        masm.mov(returnLabel.dest(), scratch);
        masm.push(scratch);
        masm.push(fp);

        // Reserve frame.
        Register framePtr = fp;
        masm.subPtr(Imm32(BaselineFrame::Size()), sp);
        masm.mov(sp, framePtr);

        // Reserve space for locals and stack values.
        masm.mov(numStackValues, scratch);
        masm.shll(Imm32(3), scratch);
        masm.subPtr(scratch, sp);

        // Enter exit frame.
        masm.addPtr(Imm32(BaselineFrame::Size() + BaselineFrame::FramePointerOffset), scratch);
        masm.makeFrameDescriptor(scratch, IonFrame_BaselineJS);
        masm.push(scratch);
        masm.push(Imm32(0)); // Fake return address.
        masm.enterFakeExitFrame();

        masm.push(framePtr);
        masm.push(jitcode);

        masm.setupUnalignedABICall(3, scratch);
        masm.passABIArg(framePtr); // BaselineFrame
        masm.passABIArg(OsrFrameReg); // StackFrame
        masm.passABIArg(numStackValues);
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, jit::InitBaselineFrameForOsr));

        masm.pop(jitcode);
        masm.pop(framePtr);

        JS_ASSERT(jitcode != ReturnReg);

        Label error;
        masm.addPtr(Imm32(IonExitFrameLayout::SizeWithFooter()), sp);
        masm.addPtr(Imm32(BaselineFrame::Size()), framePtr);
		masm.beq(ReturnReg, zero, &error);
		masm.nop();

        masm.jump(jitcode);

        // OOM: load error value, discard return address and previous frame
        // pointer and return.
        masm.bindBranch(&error);
        masm.mov(framePtr, sp);
        masm.addPtr(Imm32(2 * sizeof(uintptr_t)), sp);
        masm.moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
        masm.mov(returnLabel.dest(), scratch);
        masm.jump(scratch);

        masm.bindBranch(&notOsr);
        masm.movl(Operand(fp, ARG_SCOPECHAIN), R1.scratchReg());
    }
    /***************************************************************
        Call passed-in code, 
        get return value 
        fill in the passed in return value pointer
    ***************************************************************/
    //hwj
    masm.call(Operand(fp, ARG_JITCODE));
    //hwj
    if (type == EnterJitBaseline) {
        // Baseline OSR will return here.
        masm.bind(returnLabel.src());
        if (!masm.addCodeLabel(returnLabel))
            return NULL;
    }
    
    // Pop arguments off the stack.
    masm.pop(t6);
    masm.shrl(Imm32(FRAMESIZE_SHIFT), t6); // Unmark EntryFrame.
    masm.addl(t6, sp);

    // 19 regs saved on mips
    masm.movl(Operand(sp, ARG_RESULT + 17 * sizeof(void *)), t6);
    masm.storeValue(JSReturnOperand, Operand(t6, 0));

    /**************************************************************
        Return stack and registers to correct state
    **************************************************************/
    // Restore non-volatile registers
    masm.pop(t8);
    masm.pop(s7);
    masm.pop(s6);
    masm.pop(s5);
    masm.pop(s4);
    masm.pop(s3);
    masm.pop(s2);
    masm.pop(s1);
    masm.pop(s0);
    masm.pop(t7);
    masm.pop(t6);
    masm.pop(t5);
    masm.pop(t4);
    masm.pop(t3);
    masm.pop(t2);
    masm.pop(t1);
    masm.pop(t0);

    // Restore old stack frame pointer
    masm.pop(fp);
    masm.ret();

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

IonCode *
IonRuntime::generateInvalidator(JSContext *cx)
{
    AutoIonContextAlloc aica(cx);
    MacroAssembler masm(cx);

    // We do the minimum amount of work in assembly and shunt the rest off to InvalidationBailout. 
    // Assembly does:
    // - Pop the return address from the invalidation epilogue call.
    // - Push the machine state onto the stack.
    // - Call the InvalidationBailout routine with the stack pointer.
    // - Now that the frame has been bailed out, convert the invalidated frame into an exit frame.
    // - Do the normal check-return-code-and-thunk-to-the-interpreter dance.

    masm.addl(Imm32(sizeof(uintptr_t)), sp);

    masm.reserveStack(Registers::Total * sizeof(void *));
    for (uint32_t i = 0; i < Registers::Total; i++)
        masm.movl(Register::FromCode(i), Operand(sp, i * sizeof(void *)));

    masm.reserveStack(FloatRegisters::Total * sizeof(double));
    for (uint32_t i = 0; i < FloatRegisters::Total; i += 2)
        masm.movsd(FloatRegister::FromCode(i), Operand(sp, i * sizeof(double)));

    masm.movl(sp, t6); // Argument to ion::InvalidationBailout.

    // Make space for InvalidationBailout's frameSize outparam.
    masm.reserveStack(sizeof(size_t));
    masm.movl(sp, s1);
    
    //hwj
    // Make space for InvalidationBailout's bailoutInfo outparam.
    masm.reserveStack(sizeof(void *));
    masm.movl(sp, t8);
    
    masm.setupUnalignedABICall(3, t7);
    masm.passABIArg(t6);
    masm.passABIArg(s1);
    masm.passABIArg(t8);
    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, InvalidationBailout));

    //hwj
    masm.pop(t8); // Get bailoutInfo outparam.
    masm.pop(s1); // Get the frameSize outparam.

    // Pop the machine state and the dead frame.
    masm.lea(Operand(sp, s1, TimesOne, sizeof(InvalidationBailoutStack)), sp);//caution FloatRegisters::Total

    //hwj: position difference
    masm.generateBailoutTail(t7,t8);

    Linker linker(masm);
    IonCode *code = linker.newCode(cx, JSC::OTHER_CODE);
    IonSpew(IonSpew_Invalidate, "   invalidation thunk created at %p", (void *) code->raw());
    return code;
}

//IonCode *
//IonRuntime::generateArgumentsRectifier(JSContext *cx)
IonCode *
IonRuntime::generateArgumentsRectifier(JSContext *cx, ExecutionMode mode, void **returnAddrOut)
{
    MacroAssembler masm(cx);

    // ArgumentsRectifierReg contains the |nargs| pushed onto the current frame.
    // Including |this|, there are (|nargs| + 1) arguments to copy.
    JS_ASSERT(ArgumentsRectifierReg == s5);//s0

    // Load the number of |undefined|s to push into %ecx.
    masm.movl(Operand(sp, IonRectifierFrameLayout::offsetOfCalleeToken()), t6);
    masm.clearCalleeTag(t6, mode);
    masm.movzwl(Operand(t6, offsetof(JSFunction, nargs)), t8);
    masm.subl(s5, t8);

    // Copy the number of actual arguments.
    masm.movl(Operand(sp, IonRectifierFrameLayout::offsetOfNumActualArgs()), t7);

    masm.moveValue(UndefinedValue(), s1, s2);

    // NOTE: The fact that x86 ArgumentsRectifier saves the FramePointer is relied upon
    // by the baseline bailout code.  If this changes, fix that code!  See
    // BaselineJIT.cpp/BaselineStackBuilder::calculatePrevFramePtr, and
    // BaselineJIT.cpp/InitFromBailout. 
    masm.push(FramePointer);
    masm.movl(sp, FramePointer); 

    // Push undefined.
    {
	// by wangqing, 2013-11-21
        Label undefLoopTop;
        masm.bindBranch(&undefLoopTop);

        masm.push(s1); // type(undefined);
        masm.push(s2); // payload(undefined);
        masm.subl(Imm32(1), t8);

		masm.bne(t8, zero, &undefLoopTop);
		masm.nop();
    }

    // Get the topmost argument. We did a push of %ebp earlier, so be sure to
    // account for this in the offset
    BaseIndex b = BaseIndex(FramePointer, s5, TimesEight,
                            sizeof(IonRectifierFrameLayout) + sizeof(void*));
    masm.lea(Operand(b), t8);

    // Push arguments, |nargs| + 1 times (to include |this|).
    {
	// by wangqing, 2013-11-21
        Label copyLoopTop, initialSkip;

        masm.b(&initialSkip);
		masm.nop();

        masm.bindBranch(&copyLoopTop);
        masm.subl(Imm32(sizeof(Value)), t8);
        masm.subl(Imm32(1), s5);
        masm.bindBranch(&initialSkip);

        masm.push(Operand(t8, sizeof(Value)/2));
        masm.push(Operand(t8, 0x0));

		masm.bne(s5, zero, &copyLoopTop);
		masm.nop();
    }

    // Construct descriptor, accounting for pushed frame pointer above
    masm.lea(Operand(FramePointer, sizeof(void*)), s1);
    masm.subl(sp, s1);
    masm.makeFrameDescriptor(s1, IonFrame_Rectifier);

    // Construct IonJSFrameLayout.
    masm.push(t7); // number of actual arguments
    masm.push(t6); // calleeToken
    masm.push(s1); // descriptor

    // Call the target function.
    // Note that this assumes the function is JITted.
    masm.movl(Operand(t6, JSFunction::offsetOfNativeOrScript()), t6);
    masm.loadBaselineOrIonRaw(t6, t6, mode, NULL);
    masm.call(t6);
    uint32_t returnOffset = masm.currentOffset();

    // Remove the rectifier frame.
    masm.pop(s1);            // ebx <- descriptor with FrameType.
    masm.shrl(Imm32(FRAMESIZE_SHIFT), s1); // ebx <- descriptor.
    masm.pop(s2);            // Discard calleeToken.
    masm.pop(s2);            // Discard number of actual arguments.

    // Discard pushed arguments, but not the pushed frame pointer.
    BaseIndex unwind = BaseIndex(sp, s1, TimesOne, -int32_t(sizeof(void*)));//TBD BaseIndex special treat
    masm.lea(Operand(unwind), sp);

    masm.pop(FramePointer);
    masm.ret();

    Linker linker(masm);
    IonCode *code = linker.newCode(cx, JSC::OTHER_CODE);

    CodeOffsetLabel returnLabel(returnOffset);
    returnLabel.fixup(&masm);
    if (returnAddrOut)
        *returnAddrOut = (void *) (code->raw() + returnLabel.offset());
    return code;
}

static void
GenerateBailoutThunk(JSContext *cx, MacroAssembler &masm, uint32_t frameClass)
{
    // Push registers such that we can access them from [base + code].
    masm.reserveStack(Registers::Total * sizeof(void *));
    for (uint32_t i = 0; i < Registers::Total; i++)
        masm.movl(Register::FromCode(i), Operand(sp, i * sizeof(void *)));

    // Push xmm registers, such that we can access them from [base + code].
    masm.reserveStack(FloatRegisters::Total * sizeof(double));
    for (uint32_t i = 0; i < FloatRegisters::Total; i += 2)
        masm.movsd(FloatRegister::FromCode(i), Operand(sp, i * sizeof(double)));

    // padding  //hwj
    masm.push(Imm32(0));
    // Push the bailout table number.
    masm.push(Imm32(frameClass));

    // The current stack pointer is the first argument to ion::Bailout.
    masm.movl(sp, t6);
    // Make space for Bailout's baioutInfo outparam.
    masm.reserveStack(sizeof(void *));
    masm.movl(sp, s1);

    // Call the bailout function. This will correct the size of the bailout.
    masm.setupUnalignedABICall(2,t8);
    masm.passABIArg(t6);
    masm.passABIArg(s1);

    masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, Bailout));
    
    masm.pop(s1);
    // Common size of stuff we've pushed.
    const uint32_t BailoutDataSize = sizeof(void *) * 2 + // frameClass and padding
                                   sizeof(double) * FloatRegisters::Total +
                                   sizeof(void *) * Registers::Total;

    // Remove both the bailout frame and the topmost Ion frame's stack.
    if (frameClass == NO_FRAME_SIZE_CLASS_ID) {
        // We want the frameSize. Stack is:
        //    ... frame ...
        //    snapshotOffset
        //    frameSize
        //    ... bailoutFrame ...
        masm.addl(Imm32(BailoutDataSize), sp);
        masm.pop(t8);
        masm.addl(Imm32(sizeof(uint32_t)), sp);
        masm.addl(t8, sp);
    } else {
        // Stack is:
        //    ... frame ...
        //    bailoutId
        //    ... bailoutFrame ...
        uint32_t frameSize = FrameSizeClass::FromClass(frameClass).frameSize();
        masm.addl(Imm32(BailoutDataSize + sizeof(void *) + frameSize), sp);
    }

    masm.generateBailoutTail(t7,s1);
}

IonCode *
IonRuntime::generateBailoutTable(JSContext *cx, uint32_t frameClass)
{
    MacroAssembler masm;
	
    // by wangqing, 2013-11-21
    Label bailout;
    for (size_t i = 0; i < BAILOUT_TABLE_SIZE; i++)
        masm.call(&bailout);
    masm.bind(&bailout);

    GenerateBailoutThunk(cx, masm, frameClass);

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

IonCode *
IonRuntime::generateBailoutHandler(JSContext *cx)
{
    MacroAssembler masm;

    GenerateBailoutThunk(cx, masm, NO_FRAME_SIZE_CLASS_ID);

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

IonCode *
IonRuntime::generateVMWrapper(JSContext *cx, const VMFunction &f)
{
    typedef MoveResolver::MoveOperand MoveOperand;

    JS_ASSERT(!StackKeptAligned);
    JS_ASSERT(functionWrappers_);
    JS_ASSERT(functionWrappers_->initialized());
    VMWrapperMap::AddPtr p = functionWrappers_->lookupForAdd(&f);
    if (p)
        return p->value;

    // Generate a separated code for the wrapper.
    MacroAssembler masm;

    // Avoid conflicts with argument registers while discarding the result after
    // the function call.
    GeneralRegisterSet regs = GeneralRegisterSet(Register::Codes::WrapperMask);

    // Wrapper register set is a superset of Volatile register set.
    JS_STATIC_ASSERT((Register::Codes::VolatileMask & ~Register::Codes::WrapperMask) == 0);

    // The context is the first argument.
    Register cxreg = regs.takeAny();
    // Stack is:
    //    ... frame ...
    //  +8  [args]
    //  +4  descriptor
    //  +0  returnAddress
    //
    // We're aligned to an exit frame, so link it up.
    masm.enterExitFrameAndLoadContext(&f, cxreg, regs.getAny(), f.executionMode);

    // Save the current stack pointer as the base for copying arguments.
    Register argsBase = InvalidReg;
    if (f.explicitArgs) {
        argsBase = regs.takeAny();
        masm.lea(Operand(sp, IonExitFrameLayout::SizeWithFooter()), argsBase);
    }

    // Reserve space for the outparameter.
    Register outReg = InvalidReg;
    switch (f.outParam) {
      case Type_Value:
        outReg = regs.takeAny();
        masm.reserveStack(sizeof(Value));
        masm.movl(sp, outReg);
        break;

      case Type_Handle:
        outReg = regs.takeAny();
        masm.PushEmptyRooted(f.outParamRootType);
        masm.movl(sp, outReg);
        break;

      case Type_Int32:
      case Type_Pointer:
        outReg = regs.takeAny();
        masm.reserveStack(sizeof(int32_t));
        masm.movl(sp, outReg);
        break;

      default:
        JS_ASSERT(f.outParam == Type_Void);
        break;
    }

    masm.setupUnalignedABICall(f.argc(), regs.getAny());
    masm.passABIArg(cxreg);

    size_t argDisp = 0;

    // Copy arguments.
    if (f.explicitArgs) {
        for (uint32_t explicitArg = 0; explicitArg < f.explicitArgs; explicitArg++) {
            MoveOperand from;
            switch (f.argProperties(explicitArg)) {
              case VMFunction::WordByValue:
                masm.passABIArg(MoveOperand(argsBase, argDisp));
                argDisp += sizeof(void *);
                break;
              case VMFunction::DoubleByValue:
                masm.passABIArg(MoveOperand(argsBase, argDisp));
                argDisp += sizeof(void *);
                masm.passABIArg(MoveOperand(argsBase, argDisp));
                argDisp += sizeof(void *);
                break;
              case VMFunction::WordByRef:
                masm.passABIArg(MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE));
                argDisp += sizeof(void *);
                break;
              case VMFunction::DoubleByRef:
                masm.passABIArg(MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE));
#if 0
    masm.breakpoint();
#endif
                argDisp += 2 * sizeof(void *);
                break;
            }
        }
    }

    // Copy the implicit outparam, if any.
    if (outReg != InvalidReg)
        masm.passABIArg(outReg);

    masm.callWithABI(f.wrapped);

    // Test for failure.
	// by wangqing, 2013-11-21
    Label failure;
    switch (f.failType()) {
      case Type_Object:
		masm.beq(v0, zero, &failure);
		masm.nop();
        break;
      case Type_Bool:
		masm.beq(v0, zero, &failure);
		masm.nop();
        break;
      case Type_ParallelResult:
		masm.movl(v0, cmpTempRegister);
		masm.andl(Imm32(TP_SUCCESS), cmpTempRegister);
		masm.bne(cmpTempRegister, zero, &failure);
		masm.nop();
        break;
      default:
        JS_NOT_REACHED("unknown failure kind");
        break;
    }

    // Load the outparam and free any allocated stack.
    switch (f.outParam) {
      case Type_Handle:
        masm.popRooted(f.outParamRootType, ReturnReg, JSReturnOperand);
        break;

      case Type_Value:
        masm.loadValue(Address(sp, 0), JSReturnOperand);
        masm.freeStack(sizeof(Value));
        break;

      case Type_Int32:
      case Type_Pointer:
        masm.load32(Address(sp, 0), ReturnReg);
        masm.freeStack(sizeof(int32_t));
        break;

      default:
        JS_ASSERT(f.outParam == Type_Void);
        break;
    }
    masm.leaveExitFrame();
    masm.retn(Imm32(sizeof(IonExitFrameLayout) +
                    f.explicitStackSlots() * sizeof(void *) +
                    f.extraValuesToPop * sizeof(Value)));

    masm.bindBranch(&failure);
    masm.handleFailure(f.executionMode);

    Linker linker(masm);
    IonCode *wrapper = linker.newCode(cx, JSC::OTHER_CODE);
    if (!wrapper)
        return NULL;

    // linker.newCode may trigger a GC and sweep functionWrappers_ so we have to
    // use relookupOrAdd instead of add.
    if (!functionWrappers_->relookupOrAdd(p, &f, wrapper))
        return NULL;

    return wrapper;
}

IonCode *
IonRuntime::generatePreBarrier(JSContext *cx, MIRType type)
{
    MacroAssembler masm;

    RegisterSet save;
    if (cx->runtime()->jitSupportsFloatingPoint) {
        save = RegisterSet(GeneralRegisterSet(Registers::VolatileMask),
                           FloatRegisterSet(FloatRegisters::VolatileMask));
    } else {
        save = RegisterSet(GeneralRegisterSet(Registers::VolatileMask),
                           FloatRegisterSet());
    }
    masm.PushRegsInMask(save);

    JS_ASSERT(PreBarrierReg == t7);
    masm.movl(ImmWord(cx->runtime()), t8);

    masm.setupUnalignedABICall(2, t6);
    masm.passABIArg(t8);
    masm.passABIArg(t7);

    if (type == MIRType_Value) {
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, MarkValueFromIon));
    } else {
        JS_ASSERT(type == MIRType_Shape);
        masm.callWithABI(JS_FUNC_TO_DATA_PTR(void *, MarkShapeFromIon));
    }

    masm.PopRegsInMask(save);
    masm.ret();

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}

typedef bool (*HandleDebugTrapFn)(JSContext *, BaselineFrame *, uint8_t *, JSBool *);
static const VMFunction HandleDebugTrapInfo = FunctionInfo<HandleDebugTrapFn>(HandleDebugTrap);

IonCode *
IonRuntime::generateDebugTrapHandler(JSContext *cx)
{
    MacroAssembler masm;

    Register scratch1 = t6;
    Register scratch2 = t8;
    Register scratch3 = t7;

    // Load the return address in scratch1.
    masm.loadPtr(Address(sp, 0), scratch1);

    // Load BaselineFrame pointer in scratch2.
    masm.mov(fp, scratch2);
    masm.subPtr(Imm32(BaselineFrame::Size()), scratch2);

    // Enter a stub frame and call the HandleDebugTrap VM function. Ensure
    // the stub frame has a NULL ICStub pointer, since this pointer is marked
    // during GC.
    masm.movePtr(ImmWord((void *)NULL), BaselineStubReg);
    EmitEnterStubFrame(masm, scratch3);

    IonCompartment *ion = cx->compartment()->ionCompartment();
    IonCode *code = ion->getVMWrapper(HandleDebugTrapInfo);
    if (!code)
        return NULL;

    masm.push(scratch1);
    masm.push(scratch2);
    EmitCallVM(code, masm);

    EmitLeaveStubFrame(masm);

    // If the stub returns |true|, we have to perform a forced return
    // (return from the JS frame). If the stub returns |false|, just return
    // from the trap stub so that execution continues at the current pc.
    Label forcedReturn;
	masm.bne(ReturnReg, zero, &forcedReturn);
	masm.nop();
    masm.ret();

    masm.bindBranch(&forcedReturn);
    masm.loadValue(Address(fp, BaselineFrame::reverseOffsetOfReturnValue()),
                   JSReturnOperand);
    masm.mov(fp, sp);
    masm.pop(fp);
    masm.ret();

    Linker linker(masm);
    return linker.newCode(cx, JSC::OTHER_CODE);
}
