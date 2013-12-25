/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jsutil.h"
#include "assembler/jit/ExecutableAllocator.h"
#include "jscompartment.h"
#include "jit/JitCompartment.h"
#include "jit/mips/Assembler-mips.h"
#include "gc/Marking.h"
using namespace js;
using namespace js::jit;

// from jit/x86/Assembler-x86.cpp

ABIArgGenerator::ABIArgGenerator()
  : stackOffset_(0),
    current_()
{}

ABIArg
ABIArgGenerator::next(MIRType type)
{
    current_ = ABIArg(stackOffset_);
    switch (type) {
      case MIRType_Int32:
      case MIRType_Pointer:
        stackOffset_ += sizeof(uint32_t);
        break;
      case MIRType_Float32: // Float32 moves are actually double moves
      case MIRType_Double:
        stackOffset_ += sizeof(uint64_t);
        break;
      default:
        MOZ_ASSUME_UNREACHABLE("Unexpected argument type");
    }
    return current_;
}

const Register ABIArgGenerator::NonArgReturnVolatileReg0 = s4;//ecx;
const Register ABIArgGenerator::NonArgReturnVolatileReg1 = s6;//edx;
const Register ABIArgGenerator::NonVolatileReg =s3;// ebx;

void
Assembler::executableCopy(uint8_t *buffer)
{
    masm.executableCopy(buffer);

    for (size_t i = 0; i < jumps_.length(); i++) {
        RelativePatch &rp = jumps_[i];
//ok        JSC::X86Assembler::setRel32(buffer + rp.offset, rp.target);
        mcss.repatchJump(JSC::CodeLocationJump(buffer + rp.offset), JSC::CodeLocationLabel(rp.target));
    }
}

class RelocationIterator
{
    CompactBufferReader reader_;
    uint32_t offset_;

  public:
    RelocationIterator(CompactBufferReader &reader)
      : reader_(reader)
    { }

    bool read() {
        if (!reader_.more())
            return false;
        offset_ = reader_.readUnsigned();
        return true;
    }

    uint32_t offset() const {
        return offset_;
    }
};

static inline IonCode *
CodeFromJump(uint8_t *jump)
{
    uint8_t *target = (uint8_t *)JSC::MIPSAssembler::getRel32Target(jump);
    return IonCode::FromExecutable(target);
}

void
Assembler::TraceJumpRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader)
{
    RelocationIterator iter(reader);
    while (iter.read()) {
        IonCode *child = CodeFromJump(code->raw() + iter.offset());
        MarkIonCodeUnbarriered(trc, &child, "rel32");
        JS_ASSERT(child == CodeFromJump(code->raw() + iter.offset()));
    }
}


//from jit/shared/Assembler-x86-shared.cpp

void
Assembler::copyJumpRelocationTable(uint8_t *dest)
{
    if (jumpRelocations_.length())
        memcpy(dest, jumpRelocations_.buffer(), jumpRelocations_.length());
}

void
Assembler::copyDataRelocationTable(uint8_t *dest)
{
    if (dataRelocations_.length())
        memcpy(dest, dataRelocations_.buffer(), dataRelocations_.length());
}

void
Assembler::copyPreBarrierTable(uint8_t *dest)
{
    if (preBarriers_.length())
        memcpy(dest, preBarriers_.buffer(), preBarriers_.length());
}

static void
TraceDataRelocations(JSTracer *trc, uint8_t *buffer, CompactBufferReader &reader)
{
    while (reader.more()) {
        size_t offset = reader.readUnsigned();
        void *ptr = JSC::MIPSAssembler::getPointer(buffer + offset);

        // No barrier needed since these are constants.
        gc::MarkGCThingUnbarriered(trc, reinterpret_cast<void **>(&ptr), "ion-masm-ptr");
    }
}	

void
Assembler::TraceDataRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader)
{
    ::TraceDataRelocations(trc, code->raw(), reader);
}

void
Assembler::trace(JSTracer *trc)
{
    for (size_t i = 0; i < jumps_.length(); i++) {
        RelativePatch &rp = jumps_[i];
        if (rp.kind == Relocation::IONCODE) {
            IonCode *code = IonCode::FromExecutable((uint8_t *)rp.target);
            MarkIonCodeUnbarriered(trc, &code, "masmrel32");
            JS_ASSERT(code == IonCode::FromExecutable((uint8_t *)rp.target));
        }
    }
    if (dataRelocations_.length()) {
        CompactBufferReader reader(dataRelocations_);
       // ::(trc, masm.buffer(), reader);
        ::TraceDataRelocations(trc, masm.buffer(), reader);
    }
}

// executeableCopy is new added from jit/shared/Assembler-x86-shared.cpp
void
Assembler::executableCopy(void *buffer)
{
    masm.executableCopy(buffer);
}

void
Assembler::processCodeLabels(uint8_t *rawCode)
{
    for (size_t i = 0; i < codeLabels_.length(); i++) {
     //   CodeLabel *label = codeLabels_[i];
     //   Bind(code, label->dest(), code->raw() + label->src()->offset());
        CodeLabel label = codeLabels_[i];
        Bind(rawCode, label.dest(), rawCode + label.src()->offset());
    }
}

Assembler::Condition
Assembler::InvertCondition(Condition cond)
{
    switch (cond) {
      case Zero:
        return NonZero;
      case NonZero:
        return Zero;
      case LessThan:
        return GreaterThanOrEqual;
      case LessThanOrEqual:
        return GreaterThan;
      case GreaterThan:
        return LessThanOrEqual;
      case GreaterThanOrEqual:
        return LessThan;
      case Above:
        return BelowOrEqual;
      case AboveOrEqual:
        return Below;
      case Below:
        return AboveOrEqual;
      case BelowOrEqual:
        return Above;
      default:
        MOZ_ASSUME_UNREACHABLE("unexpected condition");
    }
}

void
AutoFlushCache::update(uintptr_t newStart, size_t len)
{
}

void
AutoFlushCache::flushAnyway()
{
}

AutoFlushCache::~AutoFlushCache()
{
    if (!runtime_)
        return;

    if (runtime_->flusher() == this)
        runtime_->setFlusher(nullptr);
}


// The following is from jit/mips/Assembler-mips.h

void
Assembler::absd(const FloatRegister &src) {
    mcss.absDouble(mFPRegisterID(src.code()), mFPRegisterID(src.code()));
}
void
Assembler::zerod(const FloatRegister &src) {
    mcss.zeroDouble(mFPRegisterID(src.code()));
}

void
Assembler::retn(Imm32 n) {
    // Remove the size of the return address which is included in the frame.
//okm   masm.ret(n.value - sizeof(void *));
    pop(ra);
    mcss.ret((n.value - sizeof(void *)));//参数代表恢复栈的空间大小
    //mcss.ret((n.value));
}
Assembler::JmpSrc
Assembler::callWithPush() 
{
    mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call().m_jmp;//4insns
    return src;
}
Assembler::JmpSrc
Assembler::callRelWithPush() 
{
    mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns//
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.callRel().m_jmp;//4insns
    return src;
}

void 
Assembler::call(Label *label) {
    if (label->bound()) {
//ok            masm.linkJump(mcss.call(), JmpDst(label->offset()));
//ok    masm.linkJump(mcss.call().m_jmp, JmpDst(label->offset()));
    masm.linkJump(callRelWithPush(), JmpDst(label->offset()));
} else {
//ok            JmpSrc j = mcss.call();
//ok        JmpSrc j = mcss.call().m_jmp;
        JmpSrc j = callRelWithPush();
        JmpSrc prev = JmpSrc(label->use(j.offset()));
        masm.setNextJump(j, prev);
    }
}
void 
Assembler::call(const Register &reg) {
//ok    mcss.call(reg.code());
    ma_callIonHalfPush(reg);
}
void 
Assembler::call(const Operand &op) {
    switch (op.kind()) {
      case Operand::REG:
//ok        mcss.call(op.reg());
        ma_callIonHalfPush(Register::FromCode((int)(op.reg()))); //force cast
        break;
      case Operand::MEM_REG_DISP:            	
//ok            masm.call_m(op.disp(), op.base());
//ok        mcss.call(mAddress(op.base(), op.disp()));
        mcss.load32(mAddress(op.base(), op.disp()), v1.code());
        ma_callIonHalfPush(v1);
        break;
      default:
        MOZ_ASSUME_UNREACHABLE("Unexpected argument type");
    }
}
Assembler::JmpSrc
Assembler::ma_callIon(const Register r)
{
    // When the stack is 8 byte aligned,
    // we want to decrement sp by 8, and write pc+8 into the new sp.
    // when we return from this call, sp will be its present value minus 4.
    //as_dtr(IsStore, 32, PreIndex, pc, DTRAddr(sp, DtrOffImm(-8)));
    //as_blx(r);
//ok
    mcss.offsetFromPCToV0(sizeof(int*)*8);//1insns
    mcss.sub32(mTrustedImm32(4), sp.code());//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call(r.code()).m_jmp;//2insns
    return src;
}
Assembler::JmpSrc
Assembler::ma_callIonNoPush(const Register r)
{
    // Since we just write the return address into the stack, which is
    // popped on return, the net effect is removing 4 bytes from the stack
    //as_dtr(IsStore, 32, Offset, pc, DTRAddr(sp, DtrOffImm(0)));
//ok    //as_blx(r);
    mcss.offsetFromPCToV0(sizeof(int*)*8);//1insns
    mcss.add32(mTrustedImm32(4), sp.code());//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call(r.code()).m_jmp;//2insns
    return src;
}

Assembler::JmpSrc
Assembler::ma_callIonHalfPush(const Register r)
{
    // The stack is unaligned by 4 bytes.
    // We push the pc to the stack to align the stack before the call, when we
    // return the pc is poped and the stack is restored to its unaligned state.
    //ma_push(pc);
    //as_blx(r);
    mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call(r.code()).m_jmp;//2insns
    return src;
}

Assembler::JmpSrc
Assembler::ma_call(void *dest) // KEEP EMPTY
{
    JmpSrc src;
    MOZ_ASSUME_UNREACHABLE("Unexpected argument type");
    return  src;
    //ma_mov(Imm32((uint32)dest), CallReg);
    //as_blx(CallReg);
}

static unsigned int* __getpc(void)
{
    unsigned int *rtaddr;
    
    __asm__ volatile ("move %0, $31" : "=r"(rtaddr));
    
    return rtaddr;
}

void
Assembler::patchWrite_NearCall(CodeLocationLabel startLabel, CodeLocationLabel target){
 /*     uint8_t *start = startLabel.raw();
              *start = 0xE8;
                      ptrdiff_t offset = target - startLabel - patchWrite_NearCallSize();
                              JS_ASSERT(int32_t(offset) == offset);
                                      *((int32_t *) (start + 1)) = offset;*/
        //TBD ok
 unsigned lw, hg;
    hg = ((unsigned int)__getpc)>>16;
    lw = ((unsigned int)__getpc)&0xffff;

    uint32_t *start = (uint32_t*)startLabel.raw();
    uint32_t *to = (uint32_t*)target.raw();
    *start = 0x3c190000 | hg;
    *(start + 1) = 0x37390000 | lw;
    *(start + 2) = 0x0320f809;
    *(start + 4) = 0x24420000 | 0x14;
    *(start + 5) = 0x27bdfffc;
    *(start + 6) = 0xafa20000;
    *(start + 7) = 0x0c000000 | (((reinterpret_cast<intptr_t>(to)) >> 2) & 0x3ffffff);
}
