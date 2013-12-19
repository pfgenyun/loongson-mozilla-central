/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_mips_Assembler_mips_h
#define jit_mips_Assembler_mips_h

#include "mozilla/ArrayUtils.h"


#include "jit/shared/Assembler-shared.h"
#include "assembler/assembler/MIPSAssembler.h"
#include "assembler/assembler/MacroAssemblerMIPS.h"
#include "jit/CompactBuffer.h"
#include "jit/IonCode.h"

#include "jsscriptinlines.h"

namespace js {
namespace jit {

static const MOZ_CONSTEXPR Register zero= { JSC::MIPSRegisters::zero };
static const MOZ_CONSTEXPR Register at = { JSC::MIPSRegisters::at };
static const MOZ_CONSTEXPR Register v0 = { JSC::MIPSRegisters::v0 };
static const MOZ_CONSTEXPR Register v1 = { JSC::MIPSRegisters::v1 };
static const MOZ_CONSTEXPR Register a0 = { JSC::MIPSRegisters::a0 };
static const MOZ_CONSTEXPR Register a1 = { JSC::MIPSRegisters::a1 };
static const MOZ_CONSTEXPR Register a2 = { JSC::MIPSRegisters::a2 };
static const MOZ_CONSTEXPR Register a3 = { JSC::MIPSRegisters::a3 };
static const MOZ_CONSTEXPR Register t0 = { JSC::MIPSRegisters::t0 };
static const MOZ_CONSTEXPR Register t1 = { JSC::MIPSRegisters::t1 };
static const MOZ_CONSTEXPR Register t2 = { JSC::MIPSRegisters::t2 };
static const MOZ_CONSTEXPR Register t3 = { JSC::MIPSRegisters::t3 };
static const MOZ_CONSTEXPR Register t4 = { JSC::MIPSRegisters::t4 };
static const MOZ_CONSTEXPR Register t5 = { JSC::MIPSRegisters::t5 };
static const MOZ_CONSTEXPR Register t6 = { JSC::MIPSRegisters::t6 };
static const MOZ_CONSTEXPR Register t7 = { JSC::MIPSRegisters::t7 };
static const MOZ_CONSTEXPR Register t8 = { JSC::MIPSRegisters::t8 };
static const MOZ_CONSTEXPR Register t9 = { JSC::MIPSRegisters::t9 };
static const MOZ_CONSTEXPR Register s0 = { JSC::MIPSRegisters::s0 };
static const MOZ_CONSTEXPR Register s1 = { JSC::MIPSRegisters::s1 };
static const MOZ_CONSTEXPR Register s2 = { JSC::MIPSRegisters::s2 };
static const MOZ_CONSTEXPR Register s3 = { JSC::MIPSRegisters::s3 };
static const MOZ_CONSTEXPR Register s4 = { JSC::MIPSRegisters::s4 };
static const MOZ_CONSTEXPR Register s5 = { JSC::MIPSRegisters::s5 };
static const MOZ_CONSTEXPR Register s6 = { JSC::MIPSRegisters::s6 };
static const MOZ_CONSTEXPR Register s7 = { JSC::MIPSRegisters::s7 };
static const MOZ_CONSTEXPR Register k0 = { JSC::MIPSRegisters::k0 };
static const MOZ_CONSTEXPR Register k1 = { JSC::MIPSRegisters::k1 };
static const MOZ_CONSTEXPR Register gp = { JSC::MIPSRegisters::gp };
static const MOZ_CONSTEXPR Register sp = { JSC::MIPSRegisters::sp };
static const MOZ_CONSTEXPR Register fp = { JSC::MIPSRegisters::fp };
static const MOZ_CONSTEXPR Register ra = { JSC::MIPSRegisters::ra };


static const MOZ_CONSTEXPR FloatRegister f0 = { JSC::MIPSRegisters::f0 };
static const MOZ_CONSTEXPR FloatRegister f1 = { JSC::MIPSRegisters::f1 };
static const MOZ_CONSTEXPR FloatRegister f2 = { JSC::MIPSRegisters::f2 };
static const MOZ_CONSTEXPR FloatRegister f3 = { JSC::MIPSRegisters::f3 };
static const MOZ_CONSTEXPR FloatRegister f4 = { JSC::MIPSRegisters::f4 };
static const MOZ_CONSTEXPR FloatRegister f5 = { JSC::MIPSRegisters::f5 };
static const MOZ_CONSTEXPR FloatRegister f6 = { JSC::MIPSRegisters::f6 };
static const MOZ_CONSTEXPR FloatRegister f7 = { JSC::MIPSRegisters::f7 };
static const MOZ_CONSTEXPR FloatRegister f8 = { JSC::MIPSRegisters::f8 };
static const MOZ_CONSTEXPR FloatRegister f9 = { JSC::MIPSRegisters::f9 };
static const MOZ_CONSTEXPR FloatRegister f10 = { JSC::MIPSRegisters::f10 };
static const MOZ_CONSTEXPR FloatRegister f11 = { JSC::MIPSRegisters::f11 };
static const MOZ_CONSTEXPR FloatRegister f12 = { JSC::MIPSRegisters::f12 };
static const MOZ_CONSTEXPR FloatRegister f13 = { JSC::MIPSRegisters::f13 };
static const MOZ_CONSTEXPR FloatRegister f14 = { JSC::MIPSRegisters::f14 };
static const MOZ_CONSTEXPR FloatRegister f15 = { JSC::MIPSRegisters::f15 };
static const MOZ_CONSTEXPR FloatRegister f16 = { JSC::MIPSRegisters::f16 };
static const MOZ_CONSTEXPR FloatRegister f17 = { JSC::MIPSRegisters::f17 };
static const MOZ_CONSTEXPR FloatRegister f18 = { JSC::MIPSRegisters::f18 };
static const MOZ_CONSTEXPR FloatRegister f19 = { JSC::MIPSRegisters::f19 };
static const MOZ_CONSTEXPR FloatRegister f20 = { JSC::MIPSRegisters::f20 };
static const MOZ_CONSTEXPR FloatRegister f21 = { JSC::MIPSRegisters::f21 };
static const MOZ_CONSTEXPR FloatRegister f22 = { JSC::MIPSRegisters::f22 };
static const MOZ_CONSTEXPR FloatRegister f23 = { JSC::MIPSRegisters::f23 };
static const MOZ_CONSTEXPR FloatRegister f24 = { JSC::MIPSRegisters::f24 };
static const MOZ_CONSTEXPR FloatRegister f25 = { JSC::MIPSRegisters::f25 };
static const MOZ_CONSTEXPR FloatRegister f26 = { JSC::MIPSRegisters::f26 };
static const MOZ_CONSTEXPR FloatRegister f27 = { JSC::MIPSRegisters::f27 };
static const MOZ_CONSTEXPR FloatRegister f28 = { JSC::MIPSRegisters::f28 };
static const MOZ_CONSTEXPR FloatRegister f29 = { JSC::MIPSRegisters::f29 };
static const MOZ_CONSTEXPR FloatRegister f30 = { JSC::MIPSRegisters::f30 };
static const MOZ_CONSTEXPR FloatRegister f31 = { JSC::MIPSRegisters::f31 };
/*
static const MOZ_CONSTEXPR Register eax = { JSC::MIPSRegisters::eax };
static const MOZ_CONSTEXPR Register ecx = { JSC::MIPSRegisters::ecx };
static const MOZ_CONSTEXPR Register edx = { JSC::MIPSRegisters::edx };
static const MOZ_CONSTEXPR Register ebx = { JSC::MIPSRegisters::ebx };
static const MOZ_CONSTEXPR Register esp = { JSC::MIPSRegisters::esp };
static const MOZ_CONSTEXPR Register ebp = { JSC::MIPSRegisters::ebp };
static const MOZ_CONSTEXPR Register esi = { JSC::MIPSRegisters::esi };
static const MOZ_CONSTEXPR Register edi = { JSC::MIPSRegisters::edi };

static const MOZ_CONSTEXPR FloatRegister xmm0 = { JSC::MIPSRegisters::xmm0 };
static const MOZ_CONSTEXPR FloatRegister xmm1 = { JSC::MIPSRegisters::xmm1 };
static const MOZ_CONSTEXPR FloatRegister xmm2 = { JSC::MIPSRegisters::xmm2 };
static const MOZ_CONSTEXPR FloatRegister xmm3 = { JSC::MIPSRegisters::xmm3 };
static const MOZ_CONSTEXPR FloatRegister xmm4 = { JSC::MIPSRegisters::xmm4 };
static const MOZ_CONSTEXPR FloatRegister xmm5 = { JSC::MIPSRegisters::xmm5 };
static const MOZ_CONSTEXPR FloatRegister xmm6 = { JSC::MIPSRegisters::xmm6 };
static const MOZ_CONSTEXPR FloatRegister xmm7 = { JSC::MIPSRegisters::xmm7 };

*/
static const MOZ_CONSTEXPR Register InvalidReg = { JSC::MIPSRegisters::invalid_reg };
static const MOZ_CONSTEXPR FloatRegister InvalidFloatReg = { JSC::MIPSRegisters::invalid_freg };

static const MOZ_CONSTEXPR Register JSReturnReg_Type = t7;
static const MOZ_CONSTEXPR Register JSReturnReg_Data = t8;
//static const MOZ_CONSTEXPR Register StackPointer = sp;
static const MOZ_CONSTEXPR Register StackPointer = sp;
static const MOZ_CONSTEXPR Register FramePointer=InvalidReg;
//static const MOZ_CONSTEXPR Register FramePointer=InvalidReg;
static const MOZ_CONSTEXPR Register ReturnReg = v0;
static const MOZ_CONSTEXPR FloatRegister ReturnFloatReg = {JSC::MIPSRegisters::f0};
static const MOZ_CONSTEXPR FloatRegister ScratchFloatReg = {JSC::MIPSRegisters::f2};
static const MOZ_CONSTEXPR Register ArgumentsRectifierReg = s0;//esi;
static const MOZ_CONSTEXPR Register CallTempReg0 = s1;//edi;
static const MOZ_CONSTEXPR Register CallTempReg1 = s2;//eax;
static const MOZ_CONSTEXPR Register CallTempReg2 = s3;//ebx;
static const MOZ_CONSTEXPR Register CallTempReg3 = s4;//ecx;
static const MOZ_CONSTEXPR Register CallTempReg4 = s5;//esi;
static const MOZ_CONSTEXPR Register CallTempReg5 = s6;//edx;
static const MOZ_CONSTEXPR Register CallTempReg6 = s7;//ebp;
//Õâ¼¸¸ö¼Ä´æÆ÷ÎªMIPS×Ô¶¨Òå
//static const MOZ_CONSTEXPR Register immTempRegister  = t0;//ÔÚIonÄ¿Â¼ÏÂÃ»ÓÐÓÃµ½£»ÔÚassembler/assemblerÄ¿Â¼ÏÂÒÑ¾­¶¨Òå
static const MOZ_CONSTEXPR Register dataTempRegister = t1;//x86ÖÐµÄÖ¸Áî¿ÉÖ±½Ó¶ÔÄÚ´æÊý½øÐÐALUÔËËã£¬Òò´Ë¶ÔÓÚÕâÀàÔËËã£¬MIPSÏÈ½«ÄÚ´æÊýÈ¡³ö£¬·ÅÈëÒ»¸öÖ¸¶¨¼Ä´æÆ÷£¬ÔËËãÍê³ÉºóÔÙ·Å»ØÄÚ´æ£»
static const MOZ_CONSTEXPR Register addrTempRegister = t2;//ÔÚIonÄ¿Â¼ÏÂÃ»ÓÐÓÃµ½£»ÔÚassembler/assemblerÄ¿Â¼ÏÂÒÑ¾­¶¨Òå
static const MOZ_CONSTEXPR Register cmpTempRegister  = t3;
static const MOZ_CONSTEXPR Register cmpTemp2Register  = t5;
//static const MOZ_CONSTEXPR Register dataTemp2Register = t4;//ÔÚIonÄ¿Â¼ÏÂÃ»ÓÐÓÃµ½£»ÔÚassembler/assemblerÄ¿Â¼ÏÂÒÑ¾­¶¨Òå


static const MOZ_CONSTEXPR FloatRegister fpTempRegister = f28;
static const MOZ_CONSTEXPR FloatRegister fpTemp2Register = f30;

// We have no arg regs, so our NonArgRegs are just our CallTempReg*
static const MOZ_CONSTEXPR Register CallTempNonArgRegs[] = { s1, s2, s3, s4, s5, s6 };
static const uint32_t NumCallTempNonArgRegs =
    mozilla::ArrayLength(CallTempNonArgRegs);

class ABIArgGenerator
{
    uint32_t stackOffset_;
    ABIArg current_;

  public:
    ABIArgGenerator();
    ABIArg next(MIRType argType);
    ABIArg &current() { return current_; }
    uint32_t stackBytesConsumedSoFar() const { return stackOffset_; }

    // Note: these registers are all guaranteed to be different
    static const Register NonArgReturnVolatileReg0;//ÔÚAsmJS.cppÖÐÊ¹ÓÃ£»
    static const Register NonArgReturnVolatileReg1;//ÔÚAsmJS.cppÖÐÊ¹ÓÃ£»
    static const Register NonVolatileReg;//ÔÚAsmJS.cppÖÐÊ¹ÓÃ£»
};

static const MOZ_CONSTEXPR Register OsrFrameReg = s6;//edx;
static const MOZ_CONSTEXPR Register PreBarrierReg =s6;// edx;

// GCC stack is aligned on 16 bytes, but we don't maintain the invariant in
// jitted code.
#if defined(__GNUC__)
static const uint32_t StackAlignment = 16;
#else
static const uint32_t StackAlignment = 4;
#endif
static const bool StackKeptAligned = false;
static const uint32_t CodeAlignment = 8;
static const uint32_t NativeFrameSize = sizeof(void*);
static const uint32_t AlignmentAtPrologue = sizeof(void*);
static const uint32_t AlignmentMidPrologue = AlignmentAtPrologue;
struct ImmTag : public Imm32
{
    ImmTag(JSValueTag mask)
      : Imm32(int32_t(mask))
    { }
};

struct ImmType : public ImmTag
{
    ImmType(JSValueType type)
      : ImmTag(JSVAL_TYPE_TO_TAG(type))
    { }
};

static const Scale ScalePointer = TimesFour;

class Operand
{
  public:
    enum Kind {
        REG,
        REG_DISP,
        FPREG,
        SCALE,
        ADDRESS
    };

    Kind kind_ : 4;
    int32_t index_ : 5;// TangZL set it 6
    Scale scale_ : 3;
    int32_t base_;
    int32_t disp_;

  public:
    explicit Operand(Register reg)
      : kind_(REG),
        base_(reg.code())
    { }
    explicit Operand(FloatRegister reg)
      : kind_(FPREG),
        base_(reg.code())
    { }
    explicit Operand(const Address &address)
      : kind_(REG_DISP),
        base_(address.base.code()),
        disp_(address.offset)
    { }
    explicit Operand(const BaseIndex &address)
      : kind_(SCALE),
        index_(address.index.code()),
        scale_(address.scale),
        base_(address.base.code()),
        disp_(address.offset)
    { }
    Operand(Register base, Register index, Scale scale, int32_t disp = 0)
      : kind_(SCALE),
        index_(index.code()),
        scale_(scale),
        base_(base.code()),
        disp_(disp)
    { }
    Operand(Register reg, int32_t disp)
      : kind_(REG_DISP),
        base_(reg.code()),
        disp_(disp)
    { }
    explicit Operand(const AbsoluteAddress &address)
      : kind_(ADDRESS),
        base_(reinterpret_cast<int32_t>(address.addr))
    { }
    explicit Operand(const void *address)
      : kind_(ADDRESS),
        base_(reinterpret_cast<int32_t>(address))
    { }

    Address toAddress() {
        JS_ASSERT(kind() == REG_DISP);
        return Address(Register::FromCode(base()), disp());
    }

    BaseIndex toBaseIndex() {
        JS_ASSERT(kind() == SCALE);
        return BaseIndex(Register::FromCode(base()), Register::FromCode(index()), scale(), disp());
    }

    Kind kind() const {
        return kind_;
    }
    Registers::Code reg() const {
        JS_ASSERT(kind() == REG);
        return (Registers::Code)base_;
    }
    Registers::Code base() const {
        JS_ASSERT(kind() == REG_DISP || kind() == SCALE);
        return (Registers::Code)base_;
    }
    Registers::Code index() const {
        JS_ASSERT(kind() == SCALE);
        return (Registers::Code)index_;
    }
    Scale scale() const {
        JS_ASSERT(kind() == SCALE);
        return scale_;
    }
    FloatRegisters::Code fpu() const {
        JS_ASSERT(kind() == FPREG);
        return (FloatRegisters::Code)base_;
    }
    int32_t disp() const {
        JS_ASSERT(kind() == REG_DISP || kind() == SCALE);
        return disp_;
    }
    void *address() const {
        JS_ASSERT(kind() == ADDRESS);
        return reinterpret_cast<void *>(base_);
    }
};

} // namespace jit
} // namespace js

//#include "jit/shared/Assembler-x86-shared.h"

namespace js {
namespace jit {

static inline void
PatchJump(CodeLocationJump jump, CodeLocationLabel label)//ÔÚIonCaches.cppÖÐ±»Ê¹ÓÃ£»
{
#ifdef DEBUG
    // Assert that we're overwriting a jump instruction, either:
    //   0F 80+cc <imm32>, or
    //   E9 <imm32>
    unsigned char *x = (unsigned char *)jump.raw() - 5;
 //   JS_ASSERT(((*x >= 0x80 && *x <= 0x8F) && *(x - 1) == 0x0F) || (*x == 0xE9));
#endif
 //   JSC::MIPSAssembler::setRel32(jump.raw(), label.raw());
 JSC::MacroAssemblerMIPS::repatchJump(JSC::CodeLocationJump(jump.raw()), JSC::CodeLocationLabel(label.raw()));
}

// Return operand from a JS -> JS call.
static const ValueOperand JSReturnOperand = ValueOperand(JSReturnReg_Type, JSReturnReg_Data);

class Assembler
{
	//this is from Assembler-x86-shared.h
  protected:
    struct RelativePatch {
        int32_t offset;
        void *target;
        Relocation::Kind kind;

        RelativePatch(int32_t offset, void *target, Relocation::Kind kind)
          : offset(offset),
            target(target),
            kind(kind)
        { }
    };

    Vector<CodeLabel, 0, SystemAllocPolicy> codeLabels_;
    Vector<RelativePatch, 8, SystemAllocPolicy> jumps_;
    CompactBufferWriter jumpRelocations_;
    CompactBufferWriter dataRelocations_;
    CompactBufferWriter preBarriers_;
    bool enoughMemory_;
    
       void writeDataRelocation(const Value &val) {
        if (val.isMarkable()) {
            JS_ASSERT(static_cast<gc::Cell*>(val.toGCThing())->isTenured());
            dataRelocations_.writeUnsigned(masm.currentOffset());
        }
    }
    void writeDataRelocation(const ImmGCPtr &ptr) {
        if (ptr.value)
            dataRelocations_.writeUnsigned(masm.currentOffset());
    }
    //this is new in ff24
    void writePrebarrierOffset(CodeOffsetLabel label) {//ÔÚIonMacroAssembler.hÖÐ±»Ê¹ÓÃ£»
        preBarriers_.writeUnsigned(label.offset());
    }
    //end
      
  protected:
    JSC::MacroAssemblerMIPS mcss;
    JSC::MIPSAssembler& masm;
    JSC::MIPSAssembler& m_assembler;
    typedef JSC::MacroAssemblerMIPS::Address mAddress ;
    typedef JSC::MacroAssemblerMIPS::ExtendedAddress mExtendedAddress;
    typedef JSC::MacroAssemblerMIPS::ImplicitAddress mImplicitAddress;
    typedef JSC::MacroAssemblerMIPS::BaseIndex mBaseIndex;
    typedef JSC::MacroAssemblerMIPS::AbsoluteAddress mAbsoluteAddress;
    typedef JSC::MacroAssemblerMIPS::TrustedImmPtr mTrustedImmPtr;
    typedef JSC::MacroAssemblerMIPS::TrustedImm32 mTrustedImm32;
    typedef JSC::MacroAssemblerMIPS::Scale mScale;
    typedef JSC::MacroAssemblerMIPS::ImmPtr mImmPtr;
    typedef JSC::MacroAssemblerMIPS::Imm32 mImm32;
    typedef JSC::MacroAssemblerMIPS::ImmDouble mImmDouble;
    typedef JSC::MacroAssemblerMIPS::RegisterID mRegisterID;
    typedef JSC::MacroAssemblerMIPS::FPRegisterID mFPRegisterID;

    typedef JSC::MIPSAssembler::JmpSrc JmpSrc;
    typedef JSC::MIPSAssembler::JmpDst JmpDst;
      
      
    void writeRelocation(JmpSrc src) {
        jumpRelocations_.writeUnsigned(src.offset());
    }
    void addPendingJump(JmpSrc src, void *target, Relocation::Kind kind) {
        enoughMemory_ &= jumps_.append(RelativePatch(src.offset(), target, kind));
        if (kind == Relocation::IONCODE)
            writeRelocation(src);
    }

  public:
  	
  	
      enum Condition {
        Equal = JSC::MacroAssemblerMIPS::Equal,
        NotEqual = JSC::MacroAssemblerMIPS::NotEqual,
        Above = JSC::MacroAssemblerMIPS::Above,
        AboveOrEqual = JSC::MacroAssemblerMIPS::AboveOrEqual,
        Below = JSC::MacroAssemblerMIPS::Below,
        BelowOrEqual = JSC::MacroAssemblerMIPS::BelowOrEqual,
        GreaterThan = JSC::MacroAssemblerMIPS::GreaterThan,
        GreaterThanOrEqual = JSC::MacroAssemblerMIPS::GreaterThanOrEqual,
        LessThan = JSC::MacroAssemblerMIPS::LessThan,
        LessThanOrEqual = JSC::MacroAssemblerMIPS::LessThanOrEqual,
        Overflow = JSC::MacroAssemblerMIPS::Overflow,
        Signed = JSC::MacroAssemblerMIPS::Signed,
        Zero = JSC::MacroAssemblerMIPS::Equal,
        NonZero = JSC::MacroAssemblerMIPS::NotEqual,
        Parity = JSC::MacroAssemblerMIPS::DoubleUnordered,
        NoParity = JSC::MacroAssemblerMIPS::DoubleOrdered
    };

    // If this bit is set, the ucomisd operands have to be inverted.
    static const int DoubleConditionBitInvert = 0x10;

    // Bit set when a DoubleCondition does not map to a single x86 condition.
    // The macro assembler has to special-case these conditions.
    static const int DoubleConditionBitSpecial = 0x20;
    static const int DoubleConditionBits = DoubleConditionBitInvert | DoubleConditionBitSpecial;

    enum DoubleCondition {
        // These conditions will only evaluate to true if the comparison is ordered - i.e. neither operand is NaN.
        DoubleOrdered = NoParity,
        DoubleEqual = JSC::MacroAssemblerMIPS::DoubleEqual,
        DoubleNotEqual = JSC::MacroAssemblerMIPS::DoubleNotEqual,
        DoubleGreaterThan = JSC::MacroAssemblerMIPS::DoubleGreaterThan,
        DoubleGreaterThanOrEqual = JSC::MacroAssemblerMIPS::DoubleGreaterThanOrEqual,
        DoubleLessThan = JSC::MacroAssemblerMIPS::DoubleLessThan,
        DoubleLessThanOrEqual = JSC::MacroAssemblerMIPS::DoubleLessThanOrEqual,
        // If either operand is NaN, these conditions always evaluate to true.
        DoubleUnordered = Parity,
        DoubleEqualOrUnordered = JSC::MacroAssemblerMIPS::DoubleEqualOrUnordered,
        DoubleNotEqualOrUnordered = JSC::MacroAssemblerMIPS::DoubleNotEqualOrUnordered,
        DoubleGreaterThanOrUnordered = JSC::MacroAssemblerMIPS::DoubleGreaterThanOrUnordered,
        DoubleGreaterThanOrEqualOrUnordered = JSC::MacroAssemblerMIPS::DoubleGreaterThanOrEqualOrUnordered,
        DoubleLessThanOrUnordered = JSC::MacroAssemblerMIPS::DoubleLessThanOrUnordered,
        DoubleLessThanOrEqualOrUnordered = JSC::MacroAssemblerMIPS::DoubleLessThanOrEqualOrUnordered 
    };
//this is new in ff24
  enum NaNCond {
        NaN_HandledByCond,
        NaN_IsTrue,
        NaN_IsFalse
    };


//this is new in ff24
    // If the primary condition returned by ConditionFromDoubleCondition doesn't
    // handle NaNs properly, return NaN_IsFalse if the comparison should be
    // overridden to return false on NaN, NaN_IsTrue if it should be overridden
    // to return true on NaN, or NaN_HandledByCond if no secondary check is
    // needed.
  static inline NaNCond NaNCondFromDoubleCondition(DoubleCondition cond) {
        switch (cond) {
          case DoubleOrdered:
          case DoubleNotEqual:
          case DoubleGreaterThan:
          case DoubleGreaterThanOrEqual:
          case DoubleLessThan:
          case DoubleLessThanOrEqual:
          case DoubleUnordered:
          case DoubleEqualOrUnordered:
          case DoubleGreaterThanOrUnordered:
          case DoubleGreaterThanOrEqualOrUnordered:
          case DoubleLessThanOrUnordered:
          case DoubleLessThanOrEqualOrUnordered:
            return NaN_HandledByCond;
          case DoubleEqual:
            return NaN_IsFalse;
          case DoubleNotEqualOrUnordered:
            return NaN_IsTrue;
        }

        MOZ_ASSUME_UNREACHABLE("Unknown double condition");
        return NaN_HandledByCond;
    }

    static void staticAsserts() {
        // DoubleConditionBits should not interfere with x86 condition codes.
        JS_STATIC_ASSERT(!((Equal | NotEqual | Above | AboveOrEqual | Below |
                            BelowOrEqual | Parity | NoParity) & DoubleConditionBits));
    }

    Assembler()
      : masm(mcss.assembler()),
        m_assembler(mcss.assembler()),
      //  dataBytesNeeded_(0),
        enoughMemory_(true)
    {
    }

      static Condition InvertCondition(Condition cond);//½«Ìõ¼þ×ª»»Îª¶ÔÁ¢µÄÒ»¸ö£¬ÀýÈçµÈÓÚÁã×ª»»Îª·ÇÁã£»

    // Return the primary condition to test. Some primary conditions may not
    // handle NaNs properly and may therefore require a secondary condition.
    // Use NaNCondFromDoubleCondition to determine what else is needed.
    static inline Condition ConditionFromDoubleCondition(DoubleCondition cond) {
        return static_cast<Condition>(cond & ~DoubleConditionBits);
    }

    static void TraceDataRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader);

    // MacroAssemblers hold onto gcthings, so they are traced by the GC.
    void trace(JSTracer *trc);

    bool oom() const {
        return masm.oom() ||
               !enoughMemory_ ||
               jumpRelocations_.oom() ||
               dataRelocations_.oom() ||
               preBarriers_.oom();
    }

    void setPrinter(Sprinter *sp) {
        masm.setPrinter(sp);
    }

    void processCodeLabels(uint8_t *rawCode);
    void copyJumpRelocationTable(uint8_t *dest);
    void copyDataRelocationTable(uint8_t *dest);
    void copyPreBarrierTable(uint8_t *dest);

    bool addCodeLabel(CodeLabel label) {
        return codeLabels_.append(label);
    }
    size_t numCodeLabels() const {
        return codeLabels_.length();
    }

    // Size of the instruction stream, in bytes.
    size_t size() const {
        return masm.size();
    }
    // Size of the jump relocation table, in bytes.
    size_t jumpRelocationTableBytes() const {
        return jumpRelocations_.length();
    }
    size_t dataRelocationTableBytes() const {
        return dataRelocations_.length();
    }
    size_t preBarrierTableBytes() const {
        return preBarriers_.length();
    }
    // Size of the data table, in bytes.
    size_t bytesNeeded() const {
        return size() +
               jumpRelocationTableBytes() +
               dataRelocationTableBytes() +
               preBarrierTableBytes();
    }
  	
//   using AssemblerMIPSShared::movl;
//   using AssemblerMIPSShared::j;
//   using AssemblerMIPSShared::jmp;
//   using AssemblerMIPSShared::movsd;
//   using AssemblerMIPSShared::retarget;
//   using AssemblerMIPSShared::cmpl;
//   using AssemblerMIPSShared::call;
//   using AssemblerMIPSShared::push;


    // The buffer is about to be linked, make sure any constant pools or excess
    // bookkeeping has been flushed to the instruction stream.
    void flush() { }
    
    static void TraceJumpRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader);

    // Copy the assembly code to the given buffer, and perform any pending
    // relocations relying on the target address.
    void executableCopy(uint8_t *buffer);

    // Actual assembly emitting functions.

    void push(const ImmGCPtr &ptr) {
        push(Imm32(ptr.value));
        writeDataRelocation(ptr);
    }
    void push(const ImmWord imm) {
        push(Imm32(imm.value));
    }
    void push(const FloatRegister &src) {
        subl(Imm32(sizeof(double)), StackPointer);
        movsd(src, Operand(StackPointer, 0));
    }

    CodeOffsetLabel pushWithPatch(const ImmWord &word) {
        push(Imm32(word.value));
        return masm.currentOffset();
    }

    CodeOffsetLabel movWithPatch(const ImmWord &word, const Register &dest) {
        movl(Imm32(word.value), dest);
        return masm.currentOffset();
    }

  void fastStoreDouble(const FloatRegister &src, Register lo, Register hi){
        mcss.fastStoreDouble(mFPRegisterID(src.code()), mRegisterID(lo.code()), mRegisterID(hi.code()));
    }
    
    void fastLoadDouble(Register lo, Register hi, const FloatRegister &dest){
        mcss.fastLoadDouble(mRegisterID(lo.code()), mRegisterID(hi.code()), mFPRegisterID(dest.code()));
    }

    void movl(const ImmGCPtr &ptr, const Register &dest) {
         mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(ptr.value)), dest.code());
        writeDataRelocation(ptr);
        }
        
    void movl(const ImmGCPtr &ptr, const Operand &dest) {
           switch (dest.kind()) {
          case Operand::REG:
//ok            masm.movl_i32r(ptr.value, dest.reg());
            mcss.move(mTrustedImm32(ptr.value), dest.reg());
            writeDataRelocation(ptr);
            break;
          case Operand::REG_DISP:
//ok            masm.movl_i32m(ptr.value, dest.disp(), dest.base());
            mcss.store32(mTrustedImm32(ptr.value), mImplicitAddress(mAddress(dest.base(), dest.disp())));
            writeDataRelocation(ptr);
            break;
          case Operand::SCALE:
//ok            masm.movl_i32m(ptr.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store32(mTrustedImm32(ptr.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            writeDataRelocation(ptr);
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
  
    }
    void movl(ImmWord imm, Register dest) {
    	//ok        masm.movl_i32r(imm.value, dest.code());
        mcss.move(mTrustedImm32(imm.value), dest.code());
    }
    void mov(ImmWord imm, Register dest) {
        movl(imm, dest);
    }
    void mov(Imm32 imm, Register dest) {
        movl(imm, dest);
    }
    void mov(const Operand &src, const Register &dest) {
        movl(src, dest);
    }
    void mov(const Register &src, const Operand &dest) {
        movl(src, dest);
    }
    //NOTE*:This is new in ff24
    void mov(Imm32 imm, const Operand &dest) {
        movl(imm, dest);
    }
    void mov(AbsoluteLabel *label, const Register &dest) {
        JS_ASSERT(!label->bound());
        // Thread the patch list through the unpatched address word in the
        // instruction stream.
     //   masm.movl_i32r(label->prev(), dest.code());
       mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(label->prev())), dest.code());
        label->setPrev(masm.size());
    }
    void mov(const Register &src, const Register &dest) {
        movl(src, dest);
    }
    //NOTE*:this is differrent in ff24 ,need to update!!
    void lea(const Operand &src, const Register &dest) {
            return leal(src, dest);
    }
     //edit by QuQiuwen
    void cmpl(const Register src, ImmWord ptr) {
        movl(src,cmpTempRegister);
        movl(ptr,cmpTemp2Register);
    }
     //edit by QuQiuwen
    void cmpl(const Register src, ImmGCPtr ptr) {
        movl(src,cmpTempRegister);
        movl(ptr,cmpTemp2Register);
        writeDataRelocation(ptr);
    }
     //edit by QuQiuwen
    void cmpl(const Register &lhs, const Register &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
     //edit by QuQiuwen
    void cmpl(const Operand &op, ImmGCPtr imm) {
        movl(op,cmpTempRegister);
        movl(imm,cmpTemp2Register);
        writeDataRelocation(imm);
    }
       //NOTE*:This is new in ff24. 
       //½«Á¢¼´ÊýÓë¼Ä´æÆ÷µÄÊý×÷±È½Ï£»¸Ãº¯ÊýARMºÍx64ÖÐ¾ùÃ»ÓÐ¶¨Òå£¬
    CodeOffsetLabel cmplWithPatch(const Register &lhs, Imm32 rhs) {

     //   masm.cmpl_ir_force32(rhs.value, lhs.code());
     mcss.move(lhs.code(),cmpTempRegister.code());
     mcss.move(mTrustedImm32(rhs.value),cmpTemp2Register.code());
       return masm.currentOffset();
    }

    void jmp(void *target, Relocation::Kind reloc = Relocation::HARDCODED) {
     //  JmpSrc src = masm.jmp();
         JmpSrc src = mcss.jump().m_jmp;
        addPendingJump(src, target, reloc);
    }
    void j(Condition cond, void *target,
           Relocation::Kind reloc = Relocation::HARDCODED) {
    //    JmpSrc src = masm.jCC(static_cast<JSC::MIPSAssembler::Condition>(cond));
       JmpSrc src = mcss.branch32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), cmpTempRegister.code(), cmpTemp2Register.code()).m_jmp;
        addPendingJump(src, target, reloc);
    }

    void jmp(IonCode *target) {
        jmp(target->raw(), Relocation::IONCODE);
    }
    void j(Condition cond, IonCode *target) {
        j(cond, target->raw(), Relocation::IONCODE);
    }
    void call(IonCode *target) {
  //      JmpSrc src = masm.call();
     mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns»ñÈ¡µ½µ±Ç°pcµÄÖµÈ»ºóÑ¹Õ»£»
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call().m_jmp;//4insns
    addPendingJump(src, target->raw(), Relocation::IONCODE);
    }
    
    void call(ImmWord target) {
//ok        JmpSrc src = masm.call();
    //arm : ma_call((void *) word.value);
//    mcss.offsetFromPCToV0(sizeof(int*)*7);//2insns
//    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call().m_jmp;
    addPendingJump(src, target.asPointer(), Relocation::HARDCODED);
    }

   //NOTE*:This is new in ff24.
    //ÈôÒªÓëx86±£³ÖÒ»ÖÂ£¬ÐèÒª¶¨ÒåÒ»ÌõÖ¸ÁîÓÃÀ´Ä£Äâ¿ÕÖ¸Áî£»
       // Emit a CALL or CMP (nop) instruction. ToggleCall can be used to patch
    // this instruction.
    CodeOffsetLabel toggledCall(IonCode *target, bool enabled) {
 	ASSERT(0);
        CodeOffsetLabel offset(size());
     /*  JmpSrc src = enabled ? masm.call() : masm.cmp_eax();
     addPendingJump(src, target->raw(), Relocation::IONCODE);
          JS_ASSERT(size() - offset.offset() == ToggledCallSize());
     */
   return offset;
    }
   //NOTE*:This is new in ff24! This is need to update!
    static size_t ToggledCallSize() {
        // Size of a call instruction.
    //    return 5;
    		return 32; //ÔÚmipsÖÐcall()»áÉú³É4ÌõÖ¸Áî£»µ«ÊÇx86ÎªÊ²Ã´ÎªÉè¶¨Îª5£¿
    }

    // Re-routes pending jumps to an external target, flushing the label in the
    // process.
    void retarget(Label *label, void *target, Relocation::Kind reloc) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            bool more;
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::MIPSAssembler::JmpSrc next;
                more = masm.nextJump(jmp, &next);
                addPendingJump(jmp, target, reloc);
                jmp = next;
            } while (more);
        }
        label->reset();
    }

    void movsd(const double *dp, const FloatRegister &dest) {
    //    JS_ASSERT(HasSSE2());
   //     masm.movsd_mr((const void *)dp, dest.code());
   mcss.loadDouble(reinterpret_cast<const void *>(dp), dest.code());
    }


   //NOTE*:following  is new in ff24  £» mov**WithPatchÎª×Ô¶¨Òå´úÂë
    // Move a 32-bit immediate into a register where the immediate can be
    // patched.
    CodeOffsetLabel movlWithPatch(Imm32 imm, Register dest) {
  //      masm.movl_i32r(imm.value, dest.code());
    mcss.move(mTrustedImm32(imm.value), dest.code());
        return masm.currentOffset();
    }

    // Load from *addr where addr can be patched.
    CodeOffsetLabel movlWithPatch(void *addr, Register dest) {
      //  masm.movl_mr(addr, dest.code());
      mcss.load32(addr,dest.code());
        return masm.currentOffset();
    }
    CodeOffsetLabel movsdWithPatch(void *addr, FloatRegister dest) {
     //   JS_ASSERT(HasSSE2());
  //      masm.movsd_mr(addr, dest.code());
   mcss.loadDouble(addr,dest.code());
        return masm.currentOffset();
    }

    // Store to *addr where addr can be patched
    CodeOffsetLabel movlWithPatch(Register src, void *addr) {
     //   masm.movl_rm(src.code(), addr);
      mcss.store32(src.code(), addr);
        return masm.currentOffset();
    }
    CodeOffsetLabel movsdWithPatch(FloatRegister dest, void *addr) {
  //      JS_ASSERT(HasSSE2());
  //      masm.movsd_rm(dest.code(), addr);
 ASSERT(0);  
// mcss.storeDouble(dest.code(), addr);
        return masm.currentOffset();
    }

    // Load from *(base + disp32) where disp32 can be patched.
    CodeOffsetLabel movxblWithPatch(Address src, Register dest) {
      //   masm.movxbl_mr_disp32(src.offset, src.base.code(), dest.code());//movxbl in x86
         movxbl(Operand(src),dest);
        return masm.currentOffset();
    }
    CodeOffsetLabel movzblWithPatch(Address src, Register dest) {
  //      masm.movzbl_mr_disp32(src.offset, src.base.code(), dest.code());
  	movzbl(Operand(src),dest);
       return masm.currentOffset();
    }
    CodeOffsetLabel movxwlWithPatch(Address src, Register dest) {
        //  masm.movxwl_mr_disp32(src.offset, src.base.code(), dest.code());
      movxwl(Operand(src),dest);
        return masm.currentOffset();
    }
    CodeOffsetLabel movzwlWithPatch(Address src, Register dest) {
      //  masm.movzwl_mr_disp32(src.offset, src.base.code(), dest.code());
      movzwl(Operand(src),dest);
        return masm.currentOffset();
    }
    CodeOffsetLabel movlWithPatch(Address src, Register dest) {
        //   masm.movl_mr_disp32(src.offset, src.base.code(), dest.code());
       movl(Operand(src),dest);
        return masm.currentOffset();
    }
    CodeOffsetLabel movssWithPatch(Address src, FloatRegister dest) {
    //    JS_ASSERT(HasSSE2());
    //    masm.movss_mr_disp32(src.offset, src.base.code(), dest.code());
       movss(Operand(src),dest);
   return masm.currentOffset();
    }
    CodeOffsetLabel movsdWithPatch(Address src, FloatRegister dest) {
     //   JS_ASSERT(HasSSE2());
    //    masm.movsd_mr_disp32(src.offset, src.base.code(), dest.code());
   		movsd(Operand(src),dest);
   		return masm.currentOffset();
    }

    // Store to *(base + disp32) where disp32 can be patched.
    CodeOffsetLabel movbWithPatch(Register src, Address dest) {
      //  masm.movb_rm_disp32(src.code(), dest.offset, dest.base.code());
     ASSERT(0);
      //	movv(src,Operand(dest));
        return masm.currentOffset();
    }
    CodeOffsetLabel movwWithPatch(Register src, Address dest) {
          // masm.movw_rm_disp32(src.code(), dest.offset, dest.base.code());
          movw(src,Operand(dest));
        return masm.currentOffset();
    }
    CodeOffsetLabel movlWithPatch(Register src, Address dest) {
   //     masm.movl_rm_disp32(src.code(), dest.offset, dest.base.code());
     		movl(src,Operand(dest));
   	     return masm.currentOffset();
    }
    CodeOffsetLabel movssWithPatch(FloatRegister src, Address dest) {
      //  JS_ASSERT(HasSSE2());
      //  masm.movss_rm_disp32(src.code(), dest.offset, dest.base.code());
   	   movss(src,Operand(dest));
         return masm.currentOffset();
    }
    CodeOffsetLabel movsdWithPatch(FloatRegister src, Address dest) {
      //  JS_ASSERT(HasSSE2());
    //    masm.movsd_rm_disp32(src.code(), dest.offset, dest.base.code());
    		movsd(src,Operand(dest));
          return masm.currentOffset();
    }

    // Load from *(addr + index*scale) where addr can be patched.
    CodeOffsetLabel movlWithPatch(void *addr, Register index, Scale scale, Register dest) {
   //      masm.movl_mr(addr, index.code(), scale, dest.code());
   ASSERT(0);
  //		 mov(mImmPtr(addr),addrTempRegister);
    //mBaseIndex need to review!          
       //  mcss.load32(mBaseIndex(addrTempRegister, index, mScale(scale)), dest.code());
           return masm.currentOffset();
    }
    // folloing is from Assembler-x86-shared.h
   public:
    void align(int alignment) {
        masm.align(alignment);
    }
    
    //NOTE*:this function is new in ff24
    void writeCodePointer(AbsoluteLabel *label) {
      	ASSERT(0);
   /*     JS_ASSERT(!label->bound());
        // Thread the patch list through the unpatched address word in the
        // instruction stream.
        masm.jumpTablePointer(label->prev());
        label->setPrev(masm.size());
        */
    }
    //NOTE*:this function is new in ff24
    void writeDoubleConstant(double d, Label *label) {
  		 ASSERT(0);
    /*    label->bind(masm.size());
        masm.doubleConstant(d);
    */
    }
    void movl(const Imm32 &imm32, const Register &dest) {
   //     masm.movl_i32r(imm32.value, dest.code());
    mcss.move(mTrustedImm32(imm32.value), dest.code());
    }
    void movl(const Register &src, const Register &dest) {
   //     masm.movl_rr(src.code(), dest.code());
       mcss.move(src.code(), dest.code());
    }
    void movl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
     //       masm.movl_rr(src.reg(), dest.code());
          mcss.move(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
   //         masm.movl_mr(src.disp(), src.base(), dest.code());
     mcss.load32(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
    //        masm.movl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
       mcss.load32(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
      //      masm.movl_mr(src.address(), dest.code());
         mcss.load32(src.address(), dest.code());
            break;
//#endif
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movl(const Register &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG:
       //     masm.movl_rr(src.code(), dest.reg());
         mcss.move(src.code(),dest.reg());
            break;
          case Operand::REG_DISP:
     //       masm.movl_rm(src.code(), dest.disp(), dest.base());
        mcss.store32(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
      //      masm.movl_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
        mcss.store32(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
     //      masm.movl_rm(src.code(), dest.address());
        mcss.store32(src.code(), dest.address());
            break;
//#endif
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
        void movl(const Imm32 &imm32, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG:
//ok            masm.movl_i32r(imm32.value, dest.reg());
            mcss.move(mTrustedImm32(imm32.value), dest.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.movl_i32m(imm32.value, dest.disp(), dest.base());
            mcss.store32(mTrustedImm32(imm32.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movl_i32m(imm32.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store32(mTrustedImm32(imm32.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }


    void movsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.movsd_rr(src.code(), dest.code());
            mcss.moveDouble(src.code(), dest.code());
    }
    void movsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.movsd_rr(src.fpu(), dest.code());
            mcss.moveDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.movsd_mr(src.disp(), src.base(), dest.code());
            mcss.loadDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movsd_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.loadDouble(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
  void movsd(const FloatRegister &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::FPREG:
//ok            masm.movsd_rr(src.code(), dest.fpu());
            mcss.moveDouble(src.code(), dest.fpu());
            break;
          case Operand::REG_DISP:
//ok            masm.movsd_rm(src.code(), dest.disp(), dest.base());
            mcss.storeDouble(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movsd_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.storeDouble(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movss(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movss_mr(src.disp(), src.base(), dest.code());
            mcss.loadFloat(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movss_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.loadFloat(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movss(const FloatRegister &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movss_rm(src.code(), dest.disp(), dest.base());
            mcss.storeFloat(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movss_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.storeFloat(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    /* //128Î»ÊýµÄÒÆ¶¯²Ù×÷£»
    void movdqa(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::REG_DISP:
            masm.movdqa_mr(src.disp(), src.base(), dest.code());
            break;
          case Operand::SCALE:
            masm.movdqa_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movdqa(const FloatRegister &src, const Operand &dest) {
        JS_ASSERT(HasSSE2());
        switch (dest.kind()) {
          case Operand::REG_DISP:
            masm.movdqa_rm(src.code(), dest.disp(), dest.base());
            break;
          case Operand::SCALE:
            masm.movdqa_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    */
    
     void cvtss2sd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.cvtss2sd_rr(src.code(), dest.code());
        mcss.convertFloatToDouble(src.code(), dest.code());
    }
    void cvtsd2ss(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.cvtsd2ss_rr(src.code(), dest.code());
        mcss.convertDoubleToFloat(src.code(), dest.code());
    }

    void movzbl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movzbl_mr(src.disp(), src.base(), dest.code());
            mcss.load8ZeroExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movzbl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load8ZeroExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movxbl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movxbl_mr(src.disp(), src.base(), dest.code());
            mcss.load8SignExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movxbl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load8SignExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movb(const Register &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movb_rm(src.code(), dest.disp(), dest.base());
            mcss.store8(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movb_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store8(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movb(const Imm32 &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movb_i8m(src.value, dest.disp(), dest.base());
            mcss.store8(mTrustedImm32(src.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movb_i8m(src.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store8(mTrustedImm32(src.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movzwl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movzwl_mr(src.disp(), src.base(), dest.code());
            mcss.load16ZeroExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movzwl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load16ZeroExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
       void movw(const Register &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movw_rm(src.code(), dest.disp(), dest.base());
            mcss.store16(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movw_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store16(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movw(const Imm32 &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG_DISP:
//ok            masm.movw_i16m(src.value, dest.disp(), dest.base());
            mcss.store16(mTrustedImm32(src.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::SCALE:
//ok            masm.movw_i16m(src.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store16(mTrustedImm32(src.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movxwl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
//ok            masm.movxwl_mr(src.disp(), src.base(), dest.code());
            mcss.load16SignExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.movxwl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load16SignExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    
    //NOTE*:This is new in ff24, it has the same define with lea() in old edit;
    void leal(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG_DISP:
      //     masm.leal_mr(src.disp(), src.base(), dest.code());
         mcss.lea(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
        //    masm.leal_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
         mcss.lea(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

  protected:
    JmpSrc jSrc(Condition cond, Label *label) {
//okm        JmpSrc j = masm.jCC(static_cast<JSC::X86Assembler::Condition>(cond));
        JmpSrc j = mcss.branch32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), cmpTempRegister.code(), cmpTemp2Register.code()).m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            masm.setNextJump(j, prev);
        }
        return j;
    }
    JmpSrc jmpSrc(Label *label) {
//ok        JmpSrc j = masm.jmp();
        JmpSrc j = mcss.jump().m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            JmpSrc prev = JmpSrc(label->use(j.offset()));
            masm.setNextJump(j, prev);
        }
        return j;
    }
/*//no used
    // Comparison of EAX against the address given by a Label.
    JmpSrc cmpSrc(Label *label) ;
*/
    JmpSrc jSrc(Condition cond, RepatchLabel *label) {
     //   JmpSrc j = masm.jCC(static_cast<JSC::X86Assembler::Condition>(cond));
       JmpSrc j = mcss.branch32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), cmpTempRegister.code(), cmpTemp2Register.code()).m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            label->use(j.offset());
        }
        return j;
    }
    JmpSrc jmpSrc(RepatchLabel *label) {
      /*  JmpSrc j = masm.jmp();
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            label->use(j.offset());
        }
        return j;*/
	ASSERT(0);
	//in mips jmp() need to define in assembler/assembler/
	return 0;
    }

  public:
    void nop() { masm.nop(); }
    void j(Condition cond, Label *label) { jSrc(cond, label); }
    void jmp(Label *label) { jmpSrc(label); }
    void j(Condition cond, RepatchLabel *label) { jSrc(cond, label); }
    void jmp(RepatchLabel *label) { jmpSrc(label); }

   void jmp(const Operand &op){
        switch (op.kind()) {
          case Operand::SCALE:
//ok            masm.jmp_m(op.disp(), op.base(), op.index(), op.scale());
            mcss.jump(mBaseIndex(op.base(), op.index(), mScale(op.scale()), op.disp()));
            break;
          case Operand::REG:
//ok            masm.jmp_r(op.reg());
            mcss.jump(op.reg());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // no used
 //   void cmpEAX(Label *label) { cmpSrc(label); }
    void bind(Label *label) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            bool more;
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::MIPSAssembler::JmpSrc next;
                more = masm.nextJump(jmp, &next);
                masm.linkJump(jmp, masm.label());
                jmp = next;
            } while (more);
        }
        label->bind(masm.label().offset());
    }
    void bind(RepatchLabel *label) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            masm.linkJump(jmp, masm.label());
        }
        label->bind(masm.label().offset());
    }
    uint32_t currentOffset() {
        return masm.label().offset();
    }

    // Re-routes pending jumps to a new label.
    void retarget(Label *label, Label *target) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            bool more;
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::MIPSAssembler::JmpSrc next;
                more = masm.nextJump(jmp, &next);

                if (target->bound()) {
                    // The jump can be immediately patched to the correct destination.
                    masm.linkJump(jmp, JmpDst(target->offset()));
                } else {
                    // Thread the jump list through the unpatched jump targets.
                    JmpSrc prev = JmpSrc(target->use(jmp.offset()));
                    masm.setNextJump(jmp, prev);
                }

                jmp = next;
            } while (more);
        }
        label->reset();
    }
  static void Bind(uint8_t *raw, AbsoluteLabel *label, const void *address) {
        if (label->used()) {
            intptr_t src = label->offset();
            do {
                intptr_t next = reinterpret_cast<intptr_t>(JSC::MIPSAssembler::getPointer(raw + src));
                JSC::MIPSAssembler::setPointer(raw + src, address);
                src = next;
            } while (src != AbsoluteLabel::INVALID_OFFSET);
        }
        label->bind();
    }
    void ret() {
//ok        masm.ret();
        pop(ra);
        mcss.ret();
    }
   void retn(Imm32 n);
    JmpSrc callWithPush();
    JmpSrc callRelWithPush();
    void call(Label *label);
    void call(const Register &reg);
    void call(const Operand &op);

   // void call(IonCode *target);
//    void call(ImmWord target);

   // calls an Ion function, assumes that the stack is untouched (8 byte alinged)
    JmpSrc ma_callIon(const Register reg);
    // callso an Ion function, assuming that sp has already been decremented
    JmpSrc ma_callIonNoPush(const Register reg);
    // calls an ion function, assuming that the stack is currently not 8 byte aligned
    JmpSrc ma_callIonHalfPush(const Register reg);

    JmpSrc ma_call(void *dest);

    void breakpoint() {
    //    masm.int3();
      mcss.breakpoint();
    }
    // The below cmpl methods switch the lhs and rhs when it invokes the
    // macroassembler to conform with intel standard.  When calling this
    // function put the left operand on the left as you would expect.
     //edit by QuQiuwen
    void cmpl(const Register &lhs, const Operand &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
     //edit by QuQiuwen
    void cmpl(const Register &src, Imm32 imm) {
        movl(src,cmpTempRegister);
        movl(imm,cmpTemp2Register);
    }
     //edit by QuQiuwen
    void cmpl(const Operand &op, Imm32 imm) {
        movl(op,cmpTempRegister);
        movl(imm,cmpTemp2Register);
    }
     //edit by QuQiuwen
    void cmpl(const Operand &lhs, const Register &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
     //edit by QuQiuwen
    void cmpl(const Operand &op, ImmWord imm) {
        movl(op,cmpTempRegister);
        movl(imm,cmpTemp2Register);
    }
    void setCC(Condition cond, const Register &r){
    //    masm.setCC_r(static_cast<JSC::X86Assembler::Condition>(cond), r.code());
    }
     //edit by QuQiuwen
    void testb(const Register &lhs, const Register &rhs) {
        JS_ASSERT(GeneralRegisterSet(Registers::SingleByteRegs).has(lhs));//SingleBytesRegs:t6,t7,t8,s0...s7,v0
        JS_ASSERT(GeneralRegisterSet(Registers::SingleByteRegs).has(rhs));//?
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
     //edit by QuQiuwen
    void testl(const Register &lhs, const Register &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
        andl(cmpTempRegister,cmpTemp2Register);
        movl(zero,cmpTempRegister);
    }
     //edit by QuQiuwen
    void testl(const Register &lhs, Imm32 rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
        andl(cmpTempRegister,cmpTemp2Register);
        movl(zero,cmpTempRegister);
    }
     //edit by QuQiuwen
   void testl(const Operand &lhs, Imm32 rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
        andl(cmpTempRegister,cmpTemp2Register);
        movl(zero,cmpTempRegister);
    }

    void addl(Imm32 imm, const Register &dest) {
//ok        masm.addl_ir(imm.value, dest.code());
       mcss.add32(mTrustedImm32(imm.value), dest.code());
    }
    void addl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.addl_ir(imm.value, op.reg());
            mcss.add32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.addl_im(imm.value, op.disp(), op.base());
            mcss.add32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//ok            masm.addl_im(imm.value, op.address());
            mcss.load32(op.address(), dataTempRegister.code());
            mcss.add32(mTrustedImm32(imm.value), dataTempRegister.code());
            mcss.store32(dataTempRegister.code(), op.address());
            break;
//#endif
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void subl(Imm32 imm, const Register &dest) {
//ok        masm.subl_ir(imm.value, dest.code());
        mcss.sub32(mTrustedImm32(imm.value), dest.code());
    }
    void subl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.subl_ir(imm.value, op.reg());
            mcss.sub32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.subl_im(imm.value, op.disp(), op.base());
            mcss.sub32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void addl(const Register &src, const Register &dest) {
//ok        masm.addl_rr(src.code(), dest.code());
        mcss.add32(src.code(), dest.code());
    }
    void subl(const Register &src, const Register &dest) {
//ok        masm.subl_rr(src.code(), dest.code());
        mcss.sub32(src.code(), dest.code());
    }
    void subl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.subl_rr(src.reg(), dest.code());
            mcss.sub32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.subl_mr(src.disp(), src.base(), dest.code());
            mcss.sub32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void orl(const Register &reg, const Register &dest) {
//ok        masm.orl_rr(reg.code(), dest.code());
        mcss.or32(reg.code(), dest.code());
    }
    void orl(Imm32 imm, const Register &reg) {
//ok        masm.orl_ir(imm.value, reg.code());
        mcss.or32(mTrustedImm32(imm.value), reg.code());
    }
    void orl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.orl_ir(imm.value, op.reg());
            mcss.or32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.orl_im(imm.value, op.disp(), op.base());
            mcss.or32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void xorl(const Register &src, const Register &dest) {
//ok        masm.xorl_rr(src.code(), dest.code());
         mcss.xor32(src.code(), dest.code());
    }
    void xorl(Imm32 imm, const Register &reg) {
//ok        masm.xorl_ir(imm.value, reg.code());
        mcss.xor32(mTrustedImm32(imm.value), reg.code());
    }
    void xorl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.xorl_ir(imm.value, op.reg());
            mcss.xor32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.xorl_im(imm.value, op.disp(), op.base());
            mcss.xor32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    //NOTE*:this is new in ff24;
    void andl(const Register &src, const Register &dest) {
      // masm.andl_rr(src.code(), dest.code());
        mcss.and32(src.code(), dest.code());
    }
    void andl(Imm32 imm, const Register &dest) {
//ok        masm.andl_ir(imm.value, dest.code());
        mcss.and32(mTrustedImm32(imm.value), dest.code());
    }
    
 void andl(Imm32 imm, const Operand &op) {
        switch (op.kind()) {
          case Operand::REG:
//ok            masm.andl_ir(imm.value, op.reg());
            mcss.and32(mTrustedImm32(imm.value), op.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.andl_im(imm.value, op.disp(), op.base());
            mcss.and32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void addl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.addl_rr(src.reg(), dest.code());
            mcss.add32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.addl_mr(src.disp(), src.base(), dest.code());
            mcss.add32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void orl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.orl_rr(src.reg(), dest.code());
            mcss.or32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.orl_mr(src.disp(), src.base(), dest.code());
            mcss.or32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void xorl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.xorl_rr(src.reg(), dest.code());
            mcss.xor32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.xorl_mr(src.disp(), src.base(), dest.code());
            mcss.xor32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void andl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.andl_rr(src.reg(), dest.code());
            mcss.and32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.andl_mr(src.disp(), src.base(), dest.code());
            mcss.and32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
 void imull(Imm32 imm, const Register &dest) {
//ok        masm.imull_i32r(dest.code(), imm.value, dest.code());
        mcss.mul32(mTrustedImm32(imm.value), dest.code());
    }
    void imull(const Register &src, const Register &dest) {
//ok        masm.imull_rr(src.code(), dest.code());
        mcss.mul32(src.code(), dest.code());
    }
    void imull(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.imull_rr(src.reg(), dest.code());
            mcss.mul32(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.imull_mr(src.disp(), src.base(), dest.code());
            mcss.mul32(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void negl(const Operand &src) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.negl_r(src.reg());
            mcss.neg32(src.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.negl_m(src.disp(), src.base());
            mcss.neg32(mAddress(src.base(), src.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void negl(const Register &reg) {
//ok        masm.negl_r(reg.code());
        mcss.neg32(reg.code());
    }
    
    void notl(const Operand &src) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.notl_r(src.reg());
            mcss.not32(src.reg());
            break;
          case Operand::REG_DISP:
//ok            masm.notl_m(src.disp(), src.base());
            mcss.not32(mAddress(src.base(), src.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    //NOTE* :this is new in ff24;
    void notl(const Register &reg) {
      //  masm.notl_r(reg.code());
      mcss.not32(reg.code());
    }
    void shrl(const Imm32 imm, const Register &dest) {
//ok        masm.shrl_i8r(imm.value, dest.code());
        mcss.urshift32(mTrustedImm32(imm.value), dest.code());
    }
    void shll(const Imm32 imm, const Register &dest) {
//ok        masm.shll_i8r(imm.value, dest.code());
        mcss.lshift32(mTrustedImm32(imm.value), dest.code());
    }
    void sarl(const Imm32 imm, const Register &dest) {
//ok        masm.sarl_i8r(imm.value, dest.code());
        mcss.rshift32(mTrustedImm32(imm.value), dest.code());
    }
    void shrl_cl(const Register &dest) {
     //  masm.shrl_CLr(dest.code());
         mcss.urshift32(mRegisterID(v0.code()), dest.code());
    }
    void shll_cl(const Register &dest) {
     //  masm.shll_CLr(dest.code());
        mcss.lshift32(mRegisterID(v0.code()), dest.code());
    }
    void sarl_cl(const Register &dest) {
  //      masm.sarl_CLr(dest.code());
      mcss.rshift32(mRegisterID(v0.code()), dest.code());
    }

    void push(const Imm32 imm) {
//ok??        masm.push_i32(imm.value);
        mcss.push(mTrustedImm32(imm.value));
    }

    void push(const Operand &src) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.push_r(src.reg());
            if (src.reg() == sp.code()){
                mcss.store32(mRegisterID(src.reg()), mAddress(src.base(), -4));
                mcss.sub32(mTrustedImm32(4), mRegisterID(src.reg()));
            }else 
                mcss.push(mRegisterID(src.reg()));
            break;
          case Operand::REG_DISP:
//ok            masm.push_m(src.disp(), src.base());
            mcss.sub32(mTrustedImm32(4), mRegisterID(sp.code()));
            mcss.load32(mAddress(src.base(), src.disp()), dataTempRegister.code());
            mcss.store32(dataTempRegister.code(), mAddress(sp.code(), 0));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void push(const Register &src) {
//ok        masm.push_r(src.code());
        if (src.code() == sp.code()){
            mcss.store32(mRegisterID(src.code()), mAddress(src.code(), -4));
            mcss.sub32(mTrustedImm32(4), mRegisterID(src.code()));
        }else 
            mcss.push(mRegisterID(src.code()));
    }
    void pop(const Operand &src) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.pop_r(src.reg());
            if (src.reg() == sp.code()){
                mcss.load32(mAddress(src.base(), 0), mRegisterID(src.reg()));
            }else 
                mcss.pop(mRegisterID(src.reg()));
            break;
          case Operand::REG_DISP:
//ok            masm.pop_m(src.disp(), src.base());
            mcss.load32(mAddress(sp.code(), 0), dataTempRegister.code());
            mcss.store32(dataTempRegister.code(), mAddress(src.base(), src.disp()));
            mcss.add32(mTrustedImm32(4), mRegisterID(sp.code()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void pop(const Register &src) {
//ok        masm.pop_r(src.code());
        if (src.code() == sp.code()){
            mcss.load32(mAddress(src.code(), 0), mRegisterID(src.code()));
        }else 
            mcss.pop(mRegisterID(src.code()));
    }
     //edit by QuQiuwen
    void pushFlags() {
        push(cmpTempRegister);
        push(cmpTemp2Register);
    }
     //edit by QuQiuwen
    void popFlags() {
        pop(cmpTemp2Register);
        pop(cmpTempRegister);
    }

/*#ifdef JS_CPU_X86
    void pushAllRegs() {
        masm.pusha();
    }
    void popAllRegs() {
        masm.popa();
    }
#endif
*/
    // Zero-extend byte to 32-bit integer.
    void movzxbl(const Register &src, const Register &dest)
    { 
    	//masm.movzbl_rr(src.code(), dest.code());
    	mcss.zeroExtend32ToPtr(src.code(),dest.code());//mipsÖÐ²¢Ã»ÓÐÊµÏÖÀ©Õ¹£¡
    }
    //Converts signed DWORD in EAX to a signed quad word in EDX:EAX by
  //      extending the high order bit of EAX throughout EDX
    void cdq() {
      //  masm.cdq();
        ASSERT(0);
    }
    void idiv(Register divisor) {
      //  masm.idivl_r(divisor.code());//in x86:idivl  signed
      //   mcss.div(t6.code(), divisor.code());
      //  mcss.mflo(divisor.code());
         masm.div(t6.code(), divisor.code());
        masm.mflo(divisor.code());
    }
    //NOTE*:this is new in ff24; Need to update!
    void udiv(Register divisor) {
       ASSERT(0);
      // masm.divl_r(divisor.code());// in x86:div unsigned
    }

    void unpcklps(const FloatRegister &src, const FloatRegister &dest) {
    	ASSERT(0);
   //     JS_ASSERT(HasSSE2());
    //    masm.unpcklps_rr(src.code(), dest.code());
    }
    void pinsrd(const Register &src, const FloatRegister &dest) {
    	ASSERT(0);
  //      JS_ASSERT(HasSSE2());
   //     masm.pinsrd_rr(src.code(), dest.code());
    }
    void pinsrd(const Operand &src, const FloatRegister &dest) {
    	ASSERT(0);
  /*      JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::REG:
            masm.pinsrd_rr(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
            masm.pinsrd_mr(src.disp(), src.base(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }*/
    }
    void psrldq(Imm32 shift, const FloatRegister &dest) {
      ASSERT(0);
      /*  JS_ASSERT(HasSSE2());
        masm.psrldq_ir(shift.value, dest.code());*/
    }
    void psllq(Imm32 shift, const FloatRegister &dest) {
    ASSERT(0);
    /*    JS_ASSERT(HasSSE2());
        masm.psllq_ir(shift.value, dest.code());*/
    }
    void psrlq(Imm32 shift, const FloatRegister &dest) {
    ASSERT(0);
   /*     JS_ASSERT(HasSSE2());
        masm.psrlq_ir(shift.value, dest.code());*/
    }

   void ptest(const FloatRegister &lhs, const FloatRegister &rhs) {
  ASSERT(0);
   /*     JS_ASSERT(HasSSE41());
        masm.ptest_rr(rhs.code(), lhs.code());*/
    }
    void pcmpeqw(const FloatRegister &lhs, const FloatRegister &rhs) {
   ASSERT(0);
    /*  JS_ASSERT(HasSSE2());
        masm.pcmpeqw_rr(rhs.code(), lhs.code());*/
    } 
    
    void cvtsi2sd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.cvtsi2sd_rr(src.reg(), dest.code());
            mcss.convertInt32ToDouble(src.reg(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.cvtsi2sd_mr(src.disp(), src.base(), dest.code());
            mcss.convertInt32ToDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::SCALE:
//ok            masm.cvtsi2sd_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.convertInt32ToDouble(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void cvttsd2si(const FloatRegister &src, const Register &dest) {
//ok        masm.cvttsd2si_rr(src.code(), dest.code());
        mcss.truncateDoubleToInt32(src.code(), dest.code());
    }
    void cvtsi2sd(const Register &src, const FloatRegister &dest) {
//ok        masm.cvtsi2sd_rr(src.code(), dest.code());
        mcss.convertInt32ToDouble(src.code(), dest.code());
    }
    void movmskpd(const FloatRegister &src, const Register &dest) {
     //   JS_ASSERT(HasSSE2());
    //    masm.movmskpd_rr(src.code(), dest.code());
        masm.dmfc1(mRegisterID(src.code()), mFPRegisterID(dest.code()));
    masm.dsrl32(mRegisterID(dest.code()), mRegisterID(dest.code()), 31);
    }
// NOT OK! This is about double compare. --QuQiuwen 
    void ucomisd(const FloatRegister &lhs, const FloatRegister &rhs) {
     //   JS_ASSERT(HasSSE2());
   //     masm.ucomisd_rr(rhs.code(), lhs.code());
     mcss.moveDouble(lhs.code(), fpTempRegister.code());
     mcss.moveDouble(rhs.code(), fpTemp2Register.code());
    }
   
    void movd(const Register &src, const FloatRegister &dest) {
//ok        masm.movd_rr(src.code(), dest.code());
        mcss.convertInt32ToDouble(src.code(),dest.code());
    }
    void movd(const FloatRegister &src, const Register &dest) {
//ok        masm.movd_rr(src.code(), dest.code());
        mcss.truncateDoubleToInt32(src.code(), dest.code());
    }
    void addsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.addsd_rr(src.code(), dest.code());
        mcss.addDouble(src.code(), dest.code());
    }
    void addsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.addsd_rr(src.fpu(), dest.code());
            mcss.addDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.addsd_mr(src.disp(), src.base(), dest.code());
            mcss.addDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
//#ifdef JS_CPU_X86
          case Operand::ADDRESS:
//ok            masm.addsd_mr(src.address(), dest.code());
            mcss.addDouble(src.address(), dest.code());
            break;
//#endif
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void subsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.subsd_rr(src.code(), dest.code());
        mcss.subDouble(src.code(), dest.code());
    }
    void subsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.subsd_rr(src.fpu(), dest.code());
            mcss.subDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.subsd_mr(src.disp(), src.base(), dest.code());
            mcss.subDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void mulsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.mulsd_rr(src.code(), dest.code());
        mcss.mulDouble(src.code(), dest.code());
    }
    void mulsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.mulsd_rr(src.fpu(), dest.code());
            mcss.mulDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.mulsd_mr(src.disp(), src.base(), dest.code());
            mcss.mulDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void divsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.divsd_rr(src.code(), dest.code());
        mcss.divDouble(src.code(), dest.code());
    }
    void divsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.divsd_rr(src.fpu(), dest.code());
            mcss.divDouble(src.fpu(), dest.code());
            break;
          case Operand::REG_DISP:
//ok            masm.divsd_mr(src.disp(), src.base(), dest.code());
            mcss.divDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void zerod(const FloatRegister &src);
    void absd(const FloatRegister &src);
    void negd(const FloatRegister &src, const FloatRegister &dest);
    void xorpd(const FloatRegister &src, const FloatRegister &dest) {
          ASSERT(0);
   //     JS_ASSERT(HasSSE2());
  //      masm.xorpd_rr(src.code(), dest.code());
     ASSERT(src.code() == dest.code());
    zerod(src);
    }
    void orpd(const FloatRegister &src, const FloatRegister &dest) {
        ASSERT(0);
    /*    JS_ASSERT(HasSSE2());
        masm.orpd_rr(src.code(), dest.code());*/
         
    }
    void andpd(const FloatRegister &src, const FloatRegister &dest) {
         ASSERT(0);
  /*      JS_ASSERT(HasSSE2());
        masm.andpd_rr(src.code(), dest.code());*/
    
    }
    void sqrtsd(const FloatRegister &src, const FloatRegister &dest) {
       // JS_ASSERT(HasSSE2());
      //  masm.sqrtsd_rr(src.code(), dest.code());
        mcss.sqrtDouble(src.code(), dest.code());
    }
    void roundsd(const FloatRegister &src, const FloatRegister &dest)
        //         JSC::X86Assembler::RoundingMode mode)
    {
    //    JS_ASSERT(HasSSE41());
    //    masm.roundsd_rr(src.code(), dest.code(), mode);
     mcss.floorDouble(src.code(), dest.code());
    }
      //NOTE* :this is new in ff24;
    void fisttp(const Operand &dest) {
    	    ASSERT(0);
    /*    JS_ASSERT(HasSSE3());
        switch (dest.kind()) {
          case Operand::REG_DISP:
            masm.fisttp_m(dest.disp(), dest.base());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }*/
    }
       //NOTE* :this is new in ff24;
    void fld(const Operand &dest) {
    	    ASSERT(0);
  /*      switch (dest.kind()) {
          case Operand::REG_DISP:
            masm.fld_m(dest.disp(), dest.base());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }*/
    }
    void fstp(const Operand &src) {
        ASSERT(0);
     /*   switch (src.kind()) {
          case Operand::REG_DISP:
            masm.fstp_m(src.disp(), src.base());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }*/
    }

    // Defined for compatibility with ARM's assembler
    uint32_t actualOffset(uint32_t x) {
        return x;
    }

    uint32_t actualIndex(uint32_t x) {
        return x;
    }

    void flushBuffer() {
    }

    // Patching.

    static size_t patchWrite_NearCallSize() {
     //   return 5;
         return 36;
    }
    //NOTE*: the type of return is changed;
    static uintptr_t getPointer(uint8_t *instPtr) {
  //      uintptr_t *ptr = ((uintptr_t *) instPtr) - 1;
  //      return *ptr;
     uintptr_t ptr = reinterpret_cast<uintptr_t>(JSC::MIPSAssembler::getPointer(instPtr));
    return ptr;
    }
    // Write a relative call at the start location |dataLabel|.
    // Note that this DOES NOT patch data that comes before |label|.
    static void patchWrite_NearCall(CodeLocationLabel startLabel, CodeLocationLabel target);

    static void patchWrite_Imm32(CodeLocationLabel dataLabel, Imm32 toWrite) {
     //   *((int32_t *) dataLabel.raw() - 1) = toWrite.value;
         JSC::MIPSAssembler::setInt32((int32 *) dataLabel.raw(), toWrite.value);
    }

    static void patchDataWithValueCheck(CodeLocationLabel data, ImmWord newData,
                                        ImmWord expectedData)  {
//TBDok
        // The pointer given is a pointer to *after* the data.
//        uintptr_t *ptr = ((uintptr_t *) data.raw()) - 1;
//        JS_ASSERT(*ptr == expectedData.value);
//        *ptr = newData.value;
        uint32_t old = JSC::MIPSAssembler::getInt32(data.raw());
        JS_ASSERT(old == expectedData.value);
        JSC::MIPSAssembler::setInt32(((uint8_t *)data.raw()), (newData.value));
    }
                                        
    static uint32_t nopSize() {
    //    return 1;
       return 4;
    }
    static uint8_t *nextInstruction(uint8_t *cur, uint32_t *count) {
        MOZ_ASSUME_UNREACHABLE("nextInstruction NYI on MIPS");
     
    }

//CMP->JMP
    // Toggle a jmp or cmp emitted by toggledJump().
    static void ToggleToJmp(CodeLocationLabel inst) {
  /*        uint8_t *ptr = (uint8_t *)inst.raw();
    //CMP AX,imm16
    JS_ASSERT(*ptr == 0x3D);
    //JMP rel32
    *ptr = 0xE9;*/
    ASSERT(0);
    }
    //JMP->CMP
    static void ToggleToCmp(CodeLocationLabel inst) {
     /*    uint8_t *ptr = (uint8_t *)inst.raw();
    JS_ASSERT(*ptr == 0xE9);
    *ptr = 0x3D;
    */
        ASSERT(0);
    }
 //set CMP|CALL     
         //NOTE* :this is new in ff24;
    static void ToggleCall(CodeLocationLabel inst, bool enabled) {
   /*     uint8_t *ptr = (uint8_t *)inst.raw();
        JS_ASSERT(*ptr == 0x3D || // CMP
                  *ptr == 0xE8);  // CALL
        *ptr = enabled ? 0xE8 : 0x3D;*/
            ASSERT(0);
    }
    
};

// Get a register in which we plan to put a quantity that will be used as an
// integer argument.  This differs from GetIntArgReg in that if we have no more
// actual argument registers to use we will fall back on using whatever
// CallTempReg* don't overlap the argument registers, and only fail once those
// run out too.
static inline bool
GetTempRegForIntArg(uint32_t usedIntArgs, uint32_t usedFloatArgs, Register *out)
{
    if (usedIntArgs >= NumCallTempNonArgRegs)
        return false;
    *out = CallTempNonArgRegs[usedIntArgs];
    return true;
}

static const uint32_t NumArgRegs = 4;
static inline bool
GetArgReg(uint32_t arg, Register *out)
{
    if (arg <= 4) {
        *out = Register::FromCode(arg + 3);
        return true;
    }
    return false;
}

static inline bool
GetArgFloatReg(uint32_t arg, FloatRegister *out)
{
    if (arg <= 4) {
        *out = FloatRegister::FromCode(arg + 11);
        return true;
    }
    return false;
}

static inline uint32_t
GetArgStackDisp(uint32_t arg)
{
    JS_ASSERT(arg >= NumArgRegs);
    return arg * STACK_SLOT_SIZE;
}


} // namespace jit
} // namespace js

#endif /* jit_x86_Assembler_x86_h */
