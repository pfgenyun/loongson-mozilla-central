/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_mips_Assembler_mips_h
#define jit_mips_Assembler_mips_h

#include "mozilla/ArrayUtils.h"
#include <cstddef>

#include "assembler/assembler/MIPSAssembler.h"
#include "jit/CompactBuffer.h"
#include "jit/IonCode.h"
#include "jit/shared/Assembler-shared.h"

#include "assembler/assembler/MacroAssemblerMIPS.h"


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

static const MOZ_CONSTEXPR Register InvalidReg = { JSC::MIPSRegisters::invalid_reg };
static const MOZ_CONSTEXPR FloatRegister InvalidFloatReg = { JSC::MIPSRegisters::invalid_freg };

static const MOZ_CONSTEXPR Register JSReturnReg_Type = t7;
static const MOZ_CONSTEXPR Register JSReturnReg_Data = t8;
static const MOZ_CONSTEXPR Register StackPointer = sp;
static const MOZ_CONSTEXPR Register FramePointer = fp;//by xsb
static const MOZ_CONSTEXPR Register ReturnReg = v0;
static const MOZ_CONSTEXPR FloatRegister ReturnFloatReg = {JSC::MIPSRegisters::f0};
static const MOZ_CONSTEXPR FloatRegister ScratchFloatReg = {JSC::MIPSRegisters::f2};
static const MOZ_CONSTEXPR Register ArgumentsRectifierReg = s5;
static const MOZ_CONSTEXPR Register CallTempReg0 = s1;
static const MOZ_CONSTEXPR Register CallTempReg1 = s2;
static const MOZ_CONSTEXPR Register CallTempReg2 = s3;
static const MOZ_CONSTEXPR Register CallTempReg3 = s4;
static const MOZ_CONSTEXPR Register CallTempReg4 = s5;
static const MOZ_CONSTEXPR Register CallTempReg5 = s6;
static const MOZ_CONSTEXPR Register CallTempReg6 = s7;

static const MOZ_CONSTEXPR Register immTempRegister  = t0;
static const MOZ_CONSTEXPR Register dataTempRegister = t1;
static const MOZ_CONSTEXPR Register addrTempRegister = t2;
static const MOZ_CONSTEXPR Register cmpTempRegister  = t3;
static const MOZ_CONSTEXPR Register dataTemp2Register = t4;
static const MOZ_CONSTEXPR Register cmpTemp2Register  = t5;


static const MOZ_CONSTEXPR FloatRegister fpTempRegister = f28;
static const MOZ_CONSTEXPR FloatRegister fpTemp2Register = f30;

// We have no arg regs, so our NonArgRegs are just our CallTempReg*
static MOZ_CONSTEXPR_VAR Register CallTempNonArgRegs[] = { s1, s2, s3, s4, s5, s6 };
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
    static const Register NonArgReturnVolatileReg0;
    static const Register NonArgReturnVolatileReg1;
    static const Register NonVolatileReg;
};

static MOZ_CONSTEXPR_VAR Register OsrFrameReg = t7;  //edx;
static MOZ_CONSTEXPR_VAR Register PreBarrierReg =t7;  // edx;

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

static inline void
PatchJump(CodeLocationJump jump, CodeLocationLabel label)
{
#ifdef DEBUG
    // Assert that we're overwriting a jump instruction, either:
    //   0F 80+cc <imm32>, or
    //   E9 <imm32>
    unsigned char *x = (unsigned char *)jump.raw() - 5;
#endif
 JSC::MacroAssemblerMIPS::repatchJump(JSC::CodeLocationJump(jump.raw()), JSC::CodeLocationLabel(label.raw()));
}

// Return operand from a JS -> JS call.
static const ValueOperand JSReturnOperand = ValueOperand(JSReturnReg_Type, JSReturnReg_Data);

class Operand
{
  public:
    enum Kind {
        REG,
        MEM_REG_DISP,
        FPREG,
        MEM_SCALE,
        MEM_ADDRESS32
    };

  private:
    Kind kind_ : 4;
    int32_t base_ : 6;   // fixme: by wangqing
    Scale scale_ : 3;
    int32_t index_ : 6;  // fixme: by wangqing
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
      : kind_(MEM_REG_DISP),
        base_(address.base.code()),
        disp_(address.offset)
    { }
    explicit Operand(const BaseIndex &address)
      : kind_(MEM_SCALE),
        base_(address.base.code()),
        scale_(address.scale),
        index_(address.index.code()),
        disp_(address.offset)
    { }
    Operand(Register base, Register index, Scale scale, int32_t disp = 0)
      : kind_(MEM_SCALE),
        base_(base.code()),
        scale_(scale),
        index_(index.code()),
        disp_(disp)
    { }
    Operand(Register reg, int32_t disp)
      : kind_(MEM_REG_DISP),
        base_(reg.code()),
        disp_(disp)
    { }
    explicit Operand(const AbsoluteAddress &address)
      : kind_(MEM_ADDRESS32),
        base_(reinterpret_cast<int32_t>(address.addr))
    { }
    explicit Operand(const void *address)
      : kind_(MEM_ADDRESS32),
        base_(reinterpret_cast<int32_t>(address))
    { }

    Address toAddress() const {
        JS_ASSERT(kind() == MEM_REG_DISP);
        return Address(Register::FromCode(base()), disp());
    }

    BaseIndex toBaseIndex() const {
        JS_ASSERT(kind() == MEM_SCALE);
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
        JS_ASSERT(kind() == MEM_REG_DISP || kind() == MEM_SCALE);
        return (Registers::Code)base_;
    }
    Registers::Code index() const {
        JS_ASSERT(kind() == MEM_SCALE);
        return (Registers::Code)index_;
    }
    Scale scale() const {
        JS_ASSERT(kind() == MEM_SCALE);
        return scale_;
    }
    FloatRegisters::Code fpu() const {
        JS_ASSERT(kind() == FPREG);
        return (FloatRegisters::Code)base_;
    }
    int32_t disp() const {
        JS_ASSERT(kind() == MEM_REG_DISP || kind() == MEM_SCALE);
        return disp_;
    }
    void *address() const {
        JS_ASSERT(kind() == MEM_ADDRESS32);
        return reinterpret_cast<void *>(disp_);
    }
};

class Assembler
{
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
    AsmJSAbsoluteLinkVector asmJSAbsoluteLinks_;
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
    void writePrebarrierOffset(CodeOffsetLabel label) {
        preBarriers_.writeUnsigned(label.offset());
    }

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
    void addPendingJump(JmpSrc src, ImmPtr target, Relocation::Kind kind) {
        enoughMemory_ &= jumps_.append(RelativePatch(src.offset(), target.value, kind));
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
        NotSigned = JSC::MacroAssemblerMIPS::NotSigned,
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

    enum NaNCond {
        NaN_HandledByCond,
        NaN_IsTrue,
        NaN_IsFalse
    };

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

    static Condition InvertCondition(Condition cond);

    // Return the primary condition to test. Some primary conditions may not
    // handle NaNs properly and may therefore require a secondary condition.
    // Use NaNCondFromDoubleCondition to determine what else is needed.
    static inline Condition ConditionFromDoubleCondition(DoubleCondition cond) {
        return static_cast<Condition>(cond & ~DoubleConditionBits);
    }
    
    //by weizhenwei, 2013.12.24
    static inline DoubleCondition DoubleConditionFromCondition(Condition cond) {
        if ((cond == Equal) || (cond == Zero)) {
            return DoubleEqual;
        } else if ((cond == NotEqual) || (cond == NonZero)) {
            return DoubleNotEqual;
        } else if ((cond == Above) || (cond == GreaterThan)) {
            return DoubleGreaterThan;
        } else if ((cond == AboveOrEqual) || (cond == GreaterThanOrEqual)) {
            return DoubleGreaterThanOrEqual;
        } else if ((cond == Below) || (cond == LessThan)) {
            return DoubleLessThan;
        } else if ((cond == BelowOrEqual) || (cond == LessThanOrEqual)) {
            return DoubleLessThanOrEqual;
        } else if (cond == Parity) {
            return DoubleUnordered;
        } else if (cond == NoParity) {
            return DoubleOrdered;
        } else {
            MOZ_ASSUME_UNREACHABLE("unexpected Double Condition");
        }
    }
    static inline DoubleCondition InvertDoubleCondition(DoubleCondition cond) {
        if (cond == DoubleOrdered) {
            return DoubleUnordered;
        } else if (cond == DoubleUnordered) {
            return DoubleOrdered;
        } else if (cond == DoubleEqual) {
            return DoubleNotEqualOrUnordered;
        } else if (cond == DoubleNotEqual) {
            return DoubleEqualOrUnordered;
        } else if (cond == DoubleGreaterThan) {
            return DoubleLessThanOrEqualOrUnordered;
        } else if (cond == DoubleGreaterThanOrEqual) {
            return DoubleLessThanOrUnordered;
        } else if (cond == DoubleLessThan) {
            return DoubleGreaterThanOrEqualOrUnordered;
        } else if (cond == DoubleLessThanOrEqual) {
            return DoubleGreaterThanOrUnordered;
        } else if (cond == DoubleEqualOrUnordered) {
            return DoubleNotEqual;
        } else if (cond == DoubleNotEqualOrUnordered) {
            return DoubleEqual;
        } else if (cond == DoubleGreaterThanOrUnordered) {
            return DoubleLessThanOrEqual;
        } else if (cond == DoubleGreaterThanOrEqualOrUnordered) {
            return DoubleLessThan;
        } else if (cond == DoubleLessThanOrUnordered) {
            return DoubleGreaterThanOrEqual;
        } else if (cond == DoubleLessThanOrEqualOrUnordered) {
            return DoubleGreaterThan;
        } else {
            MOZ_ASSUME_UNREACHABLE("unexpected Double Condition");
        }
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

    void executableCopy(void *buffer);
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
    CodeLabel codeLabel(size_t i) {
        return codeLabels_[i];
    }

    size_t numAsmJSAbsoluteLinks() const {
        return asmJSAbsoluteLinks_.length();
    }
    const AsmJSAbsoluteLink &asmJSAbsoluteLink(size_t i) const {
        return asmJSAbsoluteLinks_[i];
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

    // The buffer is about to be linked, make sure any constant pools or excess
    // bookkeeping has been flushed to the instruction stream.
    void flush() { }
    
    static void TraceJumpRelocations(JSTracer *trc, IonCode *code, CompactBufferReader &reader);

    // Copy the assembly code to the given buffer, and perform any pending
    // relocations relying on the target address.
    void executableCopy(uint8_t *buffer);

    // Actual assembly emitting functions.

    void push(const ImmGCPtr &ptr) {
       // push(Imm32(ptr.value));
       // writeDataRelocation(ptr);
        /* 
         * author: wangqing
         * date: 2013-10-26
         *
         * lui immTemp, ptr.value >> 16
         * ori immTemp, immTemp, ptr.value & 0x0000ffff
         * writDateRelocation(ptr)
         * addiu sp, sp, -4
         * sw immTemp, sp, 0
         */
        lui(immTempRegister, (uint32_t)ptr.value >> 16);
        ori(immTempRegister, immTempRegister, (uint32_t)ptr.value & 0x0000ffff);
        writeDataRelocation(ptr);
        addiu(sp, sp, -4);
        sw(immTempRegister, sp, 0);
    }
    void push(const ImmWord imm) {
        push(Imm32(imm.value));
    }
    void push(const ImmPtr imm) {
        push(ImmWord(uintptr_t(imm.value)));
    }
    void push(const FloatRegister &src) {
        subl(Imm32(sizeof(double)), StackPointer);
        movsd(src, Address(StackPointer, 0));
    }

    CodeOffsetLabel pushWithPatch(const ImmWord &word) {
        //push(Imm32(word.value));
        //return masm.currentOffset();
       /* 
        * author: wangqing
        * date: 2010-10-18
        * 
        * lui immTemp.code(), word.value>>16
        * ori immTemp.code(), immTemp.code(), word.value&0x0000ffff
        * addiu sp, sp, -4
        * sw immTemp.code(), sp, 0
        *
        */
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(immTempRegister, word.value >> 16);
        ori(immTempRegister, immTempRegister, word.value & 0x0000ffff);
        addiu(sp, sp, -4);
        sw(immTempRegister, sp, 0);
        return label;
    }

    void pop(const FloatRegister &src) {
        movsd(Address(StackPointer, 0), src);
        addl(Imm32(sizeof(double)), StackPointer);
    }

    CodeOffsetLabel movWithPatch(const ImmWord &word, const Register &dest) {
        //movl(Imm32(word.value), dest);
        //return masm.currentOffset();
        /*  
         * author: wangqing
         * date: 2013-10-21
         *
         * lui dest.code(), word.value >> 16
         * ori dest.code(), dest.code(), word.value & 0x0000ffff
         */
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(dest, word.value >> 16);
        ori(dest, dest, word.value & 0x0000ffff);
        return label;

    }
    CodeOffsetLabel movWithPatch(const ImmPtr &imm, const Register &dest) {
        return movWithPatch(ImmWord(uintptr_t(imm.value)), dest);
    }

    void fastStoreDouble(const FloatRegister &src, Register lo, Register hi){
        mcss.fastStoreDouble(mFPRegisterID(src.code()), mRegisterID(lo.code()), mRegisterID(hi.code()));
    }
    
    void fastLoadDouble(Register lo, Register hi, const FloatRegister &dest){
        mcss.fastLoadDouble(mRegisterID(lo.code()), mRegisterID(hi.code()), mFPRegisterID(dest.code()));
    }

    void movl(const ImmGCPtr &ptr, const Register &dest) {
       // mcss.move(mTrustedImmPtr(reinterpret_cast<const void*>(ptr.value)), dest.code());
       // writeDataRelocation(ptr);
        /* 
        * author: wangqing
        * date: 2013-10-26
        *
        * lui dest, ptr.value >> 16
        * ori dest, dest, ptr.value & 0x0000ffff
        * writDateRelocation(ptr)
        */
        lui(dest, (uint32_t)ptr.value >> 16);
        ori(dest, dest, (uint32_t)ptr.value & 0x0000ffff);
        writeDataRelocation(ptr);
    }
    void movl(const ImmGCPtr &ptr, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::REG:
//ok         masm.movl_i32r(ptr.value, dest.reg());
//           mcss.move(mTrustedImm32(ptr.value), dest.reg());
//           writeDataRelocation(ptr);

           /* 
            * author: wangqing
            * date: 2013-10-26
            *
            * lui dest, ptr.value >> 16
            * ori dest, dest, ptr.value & 0x0000ffff
            * writeDataRelocation(ptr);
            */
           lui(dest.reg(), (uint32_t)ptr.value >> 16);
           ori(dest.reg(), dest.reg(), (uint32_t)ptr.value & 0x0000ffff);
           writeDataRelocation(ptr);
           break;
          case Operand::MEM_REG_DISP:
//ok         masm.movl_i32m(ptr.value, dest.disp(), dest.base());
//           mcss.store32(mTrustedImm32(ptr.value), mImplicitAddress(mAddress(dest.base(), dest.disp())));
//           writeDataRelocation(ptr);

           /* 
            * author: wangqing
            * date: 2013-10-26
            *
            * lui addrTemp, dest.disp() >> 16
            * ori addrTemp, addrTemp, dest.disp() & 0x0000ffff
            * addu addrTemp, addrTemp, dest.base()
            * lui immTemp, ptr.value >> 16
            * ori immTemp, immTemp, ptr.value & 0x0000ffff
            * writeDataRelocation(ptr)
            * sw immTemp, addrTemp, 0
            */
           lui(addrTempRegister, dest.disp() >> 16);
           ori(addrTempRegister, addrTempRegister, dest.disp() & 0x0000ffff);
           addu(addrTempRegister, addrTempRegister, dest.base());
           lui(immTempRegister, (uint32_t)ptr.value >> 16);
           ori(immTempRegister, immTempRegister, (uint32_t)ptr.value & 0x0000ffff);
           writeDataRelocation(ptr);
           sw(immTempRegister, addrTempRegister, 0);
           break;
          case Operand::MEM_SCALE:
//ok         masm.movl_i32m(ptr.value, dest.disp(), dest.base(), dest.index(), dest.scale());
//           mcss.store32(mTrustedImm32(ptr.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
//           writeDataRelocation(ptr);
           /* 
            * author: wangqing
            * date:2013-10-26
            *
               sll     addrTemp, dest.index, dest.scale
               addu    addrTemp, addrTemp, dest.base
               lui     immTemp, dest.offset >> 16
               ori     immTemp, immTemp, dest.offset & 0x0000ffff
               addu    addrTemp, addrTemp, immTemp
               lui     immTemp, ptr.value >> 16
               ori     immTemp, immTemp, ptr.value & 0x0000ffff
               writeDataRelocation(ptr)
               sw      immTemp, (0)(addrTemp)
           */
           sll(addrTempRegister, dest.index(), dest.scale());
           addu(addrTempRegister, addrTempRegister, dest.base());
           lui(immTempRegister, dest.disp() >> 16);
           ori(immTempRegister, immTempRegister, dest.disp() & 0x0000ffff);
           addu(addrTempRegister, addrTempRegister, immTempRegister.code());
           lui(immTempRegister, (uint32_t)ptr.value >> 16);
           ori(immTempRegister, immTempRegister, (uint32_t)ptr.value & 0x0000ffff);
           writeDataRelocation(ptr);
           sw(immTempRegister, addrTempRegister, 0);
            break;
           default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movl(ImmWord imm, Register dest) {
    	//ok        masm.movl_i32r(imm.value, dest.code());
        mcss.move(mTrustedImm32(imm.value), dest.code());
    }
    void movl(ImmPtr imm, Register dest) {
        movl(ImmWord(uintptr_t(imm.value)), dest);
    }
    void mov(ImmWord imm, Register dest) {
        // Use xor for setting registers to zero, as it is specially optimized
        // for this purpose on modern hardware. Note that it does clobber FLAGS
        // though.
        if (imm.value == 0)
            xorl(dest, dest);
        else
            movl(imm, dest);
    }
    void mov(ImmPtr imm, Register dest) {
        mov(ImmWord(uintptr_t(imm.value)), dest);
    }
    // New function
    void mov(AsmJSImmPtr imm, Register dest) {
        JS_ASSERT(0);
        //masm.movl_i32r(-1, dest.code());
        //AsmJSAbsoluteLink link(masm.currentOffset(), imm.kind());
        //enoughMemory_ &= asmJSAbsoluteLinks_.append(link);
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
    void mov(Imm32 imm, const Operand &dest) {
        movl(imm, dest);
    }
    
    //author:huangwenjun date:2013-12-23
    void mov(AbsoluteLabel *label, const Register &dest) {
        JS_ASSERT(!label->bound());
        // Thread the patch list through the unpatched address word in the
        // instruction stream.
        int offset = label->prev();
        lui(dest, offset >> 16);
        ori(dest.code(), dest.code(), offset&0x0000ffff);
        label->setPrev(masm.size());
    }

    void mov(const Register &src, const Register &dest) {
        movl(src, dest);
    }
    void xchg(const Register &src, const Register &dest) {
        xchgl(src, dest);
    }
    void lea(const Operand &src, const Register &dest) {
        return leal(src, dest);
    }

    // New function. 
    void fld32(const Operand &dest) {
        JS_ASSERT(0);
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
            //masm.fld32_m(dest.disp(), dest.base());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    // New function
    void fstp32(const Operand &src) {
        JS_ASSERT(0);
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
            //masm.fstp32_m(src.disp(), src.base());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void cmpl(const Register src, ImmWord ptr) {
        movl(src,cmpTempRegister);
        movl(ptr,cmpTemp2Register);
    }
    void cmpl(const Register src, ImmPtr imm) {
        movl(src, cmpTempRegister);
        movl(ImmWord(uintptr_t(imm.value)), cmpTemp2Register);
    }

    void cmpl(const Register src, ImmGCPtr ptr) {
        movl(src,cmpTempRegister);
        //movl(ptr,cmpTemp2Register);
        /* 
         * author: wangqing
         * date: 2013-10-26
         *
         * lui cmpTemp2, ptr.value >> 16
         * ori cmpTemp2, cmpTemp2, ptr.value & 0x0000ffff
         * writDateRelocation(ptr)
         */
        lui(cmpTemp2Register, (uint32_t)ptr.value >> 16);
        ori(cmpTemp2Register, cmpTemp2Register, (uint32_t)ptr.value & 0x0000ffff);
        writeDataRelocation(ptr);
    }
    void cmpl(const Register &lhs, const Register &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
    void cmpl(const Operand &op, ImmGCPtr imm) {
        movl(op,cmpTempRegister);
       // movl(imm,cmpTemp2Register);
       // writeDataRelocation(imm);
        /* 
         * author: wangqing
         * date: 2013-10-26
         *
         * lui cmpTemp2, imm.value >> 16
         * ori cmpTemp2, cmpTemp2, imm.value & 0x0000ffff
         * writDateRelocation(imm)
         */
        lui(cmpTemp2Register, (uint32_t)imm.value >> 16);
        ori(cmpTemp2Register, cmpTemp2Register, (uint32_t)imm.value & 0x0000ffff);
        writeDataRelocation(imm);
    }
    // New function
    void cmpl(const AsmJSAbsoluteAddress &lhs, const Register &rhs) {
        JS_ASSERT(0);
//        masm.cmpl_rm_force32(rhs.code(), (void*)-1);
//        AsmJSAbsoluteLink link(masm.currentOffset(), lhs.kind());
//        enoughMemory_ &= asmJSAbsoluteLinks_.append(link);
    }
    CodeOffsetLabel cmplWithPatch(const Register &lhs, Imm32 rhs) {
     //   masm.cmpl_ir_force32(rhs.value, lhs.code());
        //mcss.move(lhs.code(),cmpTempRegister.code());
        //mcss.move(mTrustedImm32(rhs.value),cmpTemp2Register.code());
        //return masm.currentOffset();
       /* 
        * author: wangqing
        * date: 2013-10-23
        *
        * move lhs.code(), cmpTemp.code()
        * lui cmpTemp2.code(), rhs.value >> 16
        * ori cmpTemp2.code(), cmpTemp2.code(), rhs.value & 0x0000ffff
        */
       move(lhs,cmpTempRegister);
       CodeOffsetLabel label = CodeOffsetLabel(size());
       lui(cmpTemp2Register, rhs.value >> 16);
       ori(cmpTemp2Register, cmpTemp2Register, rhs.value & 0x0000ffff);
       return label; 
    }

    //author:huangwenjun date:2013-12-23
    void jmp(ImmPtr target, Relocation::Kind reloc = Relocation::HARDCODED) {
        JmpSrc src = mcss.jump().m_jmp;
        int to = (int)(target.value);
        lui(t9, to>>16);
        ori(t9, t9, to&0x0000ffff);

        if(reloc == Relocation::IONCODE) {
            JmpSrc src(size());
            addPendingJump(src, target, reloc);
        }

        jr(t9);
        nop();
    }
    
    //author:huangwenjun date:2013-12-23
    void j(Condition cond, ImmPtr target,
        Relocation::Kind reloc = Relocation::HARDCODED) {
        Register left = cmpTempRegister;
        Register right = cmpTemp2Register;
        Register regZero = zero;
        Register tmp = dataTempRegister;
        int to = (int)(target.value);
        //int to = (int)(target);

        if (cond == Equal || cond == Zero)
        {
           //masm.bne(left.code(), right.code(), 4);
           bne(left, right, 4);
        }
        else if (cond == NotEqual || cond == NonZero)
        {
            //masm.beq(left.code(), right.code(), 4);
            beq(left, right, 4);
        }
        else if (cond == Above) {
            sltu(tmp, right, left);
            beq(tmp, regZero, 4);
        }
        else if (cond == AboveOrEqual) {
            sltu(tmp, left, right);
            bne(tmp, regZero, 4);
        }
        else if (cond == Below) {
            sltu(tmp, left, right);
            beq(tmp, regZero, 4);
        }
        else if (cond == BelowOrEqual) {
            sltu(tmp, right, left);
            bne(tmp, regZero, 4);
        }
        else if (cond == GreaterThan) {
            slt(tmp, right, left);
            beq(tmp, regZero, 4);
        }
        else if (cond == GreaterThanOrEqual) {
            slt(tmp, left, right);
            bne(tmp, regZero, 4);
        }
        else if (cond == LessThan) {
            slt(tmp, left, right);
         beq(tmp, regZero, 4);
        }
        else if (cond == LessThanOrEqual) {
            slt(tmp, right, left);
            bne(tmp, regZero, 4);
         }
        else if (cond == Overflow) {
        /*
            xor     cmpTemp, left, right
            bgez    No_overflow, cmpTemp    # same sign bit -> no overflow
            nop
            subu    cmpTemp, left, right
            xor     cmpTemp, cmpTemp, left
            bgez    No_overflow, cmpTemp    # same sign bit -> no overflow
           
            lui
            ori     
            jr
            nop

          No_overflow:
        */
        xorInsn(tmp, left, right);
            bgez(tmp, 8);
            nop();
            subu(tmp, left, right);
            xorInsn(tmp, tmp, left);
            bgez(tmp, 4);
        }
        else if (cond == Signed) {
            subu(tmp, left, right);
            // Check if the result is negative.
            slt(tmp, tmp, regZero);
            beq(tmp, regZero, 4);
        }
        lui(t9,to>>16);
        ori(t9,t9,to&0x0000ffff);
        if(reloc == Relocation::IONCODE) {
            JmpSrc src(size());
            addPendingJump(src, target, reloc);
        }
        jr(t9);
        nop();
    }

    //author:huangwenjun date:2013-12-23
    void call(IonCode *target);
    
    //author:huangwenjun date:2013-12-23
    //need check
    void call(ImmPtr target);
    void call(Label *label);
    void call(const Register &reg);
    void call(const Operand &op);
    void call(ImmWord target);

    //author:huangwenjun need check
    void call(AsmJSImmPtr target);

    void jmp(IonCode *target) {
        jmp(ImmPtr(target->raw()), Relocation::IONCODE);
    }
    void j(Condition cond, IonCode *target) {
        j(cond, ImmPtr(target->raw()), Relocation::IONCODE);
    }

    // Emit a CALL or CMP (nop) instruction. ToggleCall can be used to patch
    // this instruction.
    // 8 ins
    // author: huangwenjun date:2013-12-23
    CodeOffsetLabel toggledCall(IonCode *target, bool enabled) {
        CodeOffsetLabel offset(size());

        int to = (int)(target->raw());
        Register regZero = zero;
        CodeLabel cl;

        mov(cl.dest(),t9);
        if(enabled){
            push(t9);
        } else{
            beq(regZero, regZero, 5);
            nop();
        }
        lui(t9,to>>16);
        ori(t9,t9,to&0x0000ffff);

        JmpSrc src(size());
        addPendingJump(src, ImmPtr(target->raw()), Relocation::IONCODE);

        jalr(t9);
        nop();
        bind(cl.src());
        addCodeLabel(cl);

        JS_ASSERT((size() - offset.offset()) == ToggledCallSize());
        return offset;
    }

    // author: huangwenjun date:2013-12-23
    static size_t ToggledCallSize() {
        // Size of a call instruction.
    	return 32;//4*8
    }

    // Re-routes pending jumps to an external target, flushing the label in the
    // process.
    // author: huangwenjun date:2013-12-23
    void retarget(Label *label, ImmPtr target, Relocation::Kind reloc) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            bool more;
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::MIPSAssembler::JmpSrc next;
                more = masm.nextJump(jmp, &next);
                //hwj
                masm.clearOffsetForLabel(jmp);
                masm.preLink(jmp, target.value);

                //save the pointer after lui,ori 
                if(reloc == Relocation::IONCODE)
                    addPendingJump(JSC::MIPSAssembler::JmpSrc(jmp.offset()-8), target, reloc);
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


    // Move a 32-bit immediate into a register where the immediate can be
    // patched.
    CodeOffsetLabel movlWithPatch(Imm32 imm, Register dest) {
  //      masm.movl_i32r(imm.value, dest.code());
        //mcss.move(mTrustedImm32(imm.value), dest.code());
        //return masm.currentOffset();
         /*
         * author: wangqing
         * date: 2013-10-23
         *
         * lui dest.code(), imm.value >> 16
         * ori dest.code(), dest.code(), imm.value&0x0000ffff
         */
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(dest, imm.value >> 16);
        ori(dest, dest, imm.value & 0x0000ffff);
        return label;
    }

    // Load from *addr where addr can be patched.
    CodeOffsetLabel movlWithPatch(void *addr, Register dest) {
      //  masm.movl_mr(addr, dest.code());
        //mcss.load32(addr,dest.code());
        //return masm.currentOffset();
        
        /* 
         * author: wangqing
         * date: 2013-10-23
         *
         * lui addrTemp.code(), addr >> 16
         * ori addrTemp.code(), addr & 0x0000ffff
         * lw dest.code(), addrTemp.code(), 0
         */
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(addrTempRegister, (int)addr >> 16);
        ori(addrTempRegister, addrTempRegister, (int)addr & 0x0000ffff);
        lw(dest, addrTempRegister, 0);
        return label;
    }
    CodeOffsetLabel movsdWithPatch(void *addr, FloatRegister dest) {
     //   JS_ASSERT(HasSSE2());
  //      masm.movsd_mr(addr, dest.code());
        //mcss.loadDouble(addr,dest.code());
        //return masm.currentOffset();
         /* 
         * author: wangqing
         * date: 2013-10-23
         *
         * lui addrTemp, addr >> 16
         * ori addrTemp, addrTemp, addr & 0x0000ffff
         * lwc1 dest, addrTemp, 0
         * lwc1 dest+1, addrTemp, 4
         */
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(addrTempRegister, (int)addr >> 16);
        ori(addrTempRegister, addrTempRegister, (int)addr & 0x0000ffff);  
        lwc1(dest, addrTempRegister, 0);
        lwc1(mFPRegisterID(dest.code()+1), addrTempRegister, 4);
        return label;
    }

    // Store to *addr where addr can be patched
    CodeOffsetLabel movlWithPatch(Register src, void *addr) {
     //   masm.movl_rm(src.code(), addr);
        //mcss.store32(src.code(), addr);
        //return masm.currentOffset();
         /* 
         * author: wangqing
         * date: 2013-10-23
         *
         * lui addrTemp, addr >> 16
         * ori addrTemp, addrTemp, addr & 0x0000ffff
         * sw src.code(), addrTemp, 0
         */
        
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(addrTempRegister, (int)addr >> 16);
        ori(addrTempRegister, addrTempRegister, (int)addr & 0x0000ffff);
        sw(src, addrTempRegister, 0);
        return label;
    }

    CodeOffsetLabel movsdWithPatch(FloatRegister dest, void *addr) {
  //      JS_ASSERT(HasSSE2());
  //      masm.movsd_rm(dest.code(), addr);
        //ASSERT(0);  
        //mcss.storeDouble(dest.code(), addr);
        //return masm.currentOffset();
         /* 
         * author: wangqing
         * date: 2013-10-23
         *
         * lui addrTemp, addr >> 16
         * ori addrTemp, addrTemp, addr & 0x0000ffff
         * swc1 dest, addrTemp, 0
         * swc1 dest+1, addrTemp, 4
         */
        
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(addrTempRegister, (int)addr >> 16);
        ori(addrTempRegister, addrTempRegister, (int)addr & 0x0000ffff);  
        swc1(dest, addrTempRegister, 0);
        swc1(mFPRegisterID(dest.code()+1), addrTempRegister, 4);
        return label;
    }

    // Load from *(base + disp32) where disp32 can be patched.
    CodeOffsetLabel movsblWithPatch(Address src, Register dest) {
      //   masm.movxbl_mr_disp32(src.offset, src.base.code(), dest.code());//movxbl in x86
        //movsbl(Operand(src),dest);
        //return masm.currentOffset();
          /*
          * 
          * author: wangqing
          * date: 2013-10-23
          *
             lui     addrTemp, offset >> 16
             ori     addrTemp, addrTemp, offset&0x0000ffff 
             addu    addrTemp, addrTemp, base
             lb      dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, src.offset >> 16);
         ori(addrTempRegister, addrTempRegister, src.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, src.base.code());
         lb(dest, addrTempRegister, 0);
         return label;
    }
    CodeOffsetLabel movzblWithPatch(Address src, Register dest) {
  //      masm.movzbl_mr_disp32(src.offset, src.base.code(), dest.code());
        //movzbl(Operand(src),dest);
        //return masm.currentOffset();
               
         /* 
          *
          * author: wangqing
          * date: 2013-10-23
          *
             lui     addrTemp, offset >> 16
             ori     addrTemp, addrTemp, offset & 0x0000ffff 
             addu    addrTemp, addrTemp, base
             lbu     dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, src.offset >> 16);
         ori(addrTempRegister, addrTempRegister, src.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, src.base);
         lbu(dest, addrTempRegister, 0);
         return label;
    }

    CodeOffsetLabel movswlWithPatch(Address src, Register dest) {
        //  masm.movxwl_mr_disp32(src.offset, src.base.code(), dest.code());
        //movswl(Operand(src),dest);
        //return masm.currentOffset();
        
         /* 
          *
          * author: wangqing
          * date: 2013-10-23
          *
             lui     addrTemp, offset >> 16
             ori     addrTemp, addrTemp, offset & 0x0000ffff
             addu    addrTemp, addrTemp, base
             lh      dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, src.offset >> 16);
         ori(addrTempRegister, addrTempRegister, src.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, src.base);
         lh(dest, addrTempRegister, 0);
         return label;
    }

    CodeOffsetLabel movzwlWithPatch(Address src, Register dest) {
        //  masm.movzwl_mr_disp32(src.offset, src.base.code(), dest.code());
        //movzwl(Operand(src),dest);
        //return masm.currentOffset();
        
         /* 
          *
          * author: wangqing
          * date: 2013-10-23
          *
             lui     addrTemp, offset >> 16
             ori     addrTemp, addrTemp, offset&0x0000ffff
             addu    addrTemp, addrTemp, base
             lhu     dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, src.offset >> 16);
         ori(addrTempRegister, addrTempRegister, src.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, src.base);
         lhu(dest, addrTempRegister, 0);
         return label;
    }

    CodeOffsetLabel movlWithPatch(Address src, Register dest) {
        //   masm.movl_mr_disp32(src.offset, src.base.code(), dest.code());
        //movl(Operand(src),dest);
        //return masm.currentOffset();
         /* 
          * 
          * author: wangqing
          * date: 2013-10-23
          *
             lui     addrTemp, offset >> 16
             ori     addrTemp, addrTemp, offset&0x0000ffff
             addu    addrTemp, addrTemp, base
             lw      dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, src.offset >> 16);
         ori(addrTempRegister, addrTempRegister, src.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, src.base);
         lw(dest, addrTempRegister, 0);
         return label;
    }

    CodeOffsetLabel movssWithPatch(Address src, FloatRegister dest) {
        //    JS_ASSERT(HasSSE2());
        //    masm.movss_mr_disp32(src.offset, src.base.code(), dest.code());
        //movss(Operand(src),dest);
        //return masm.currentOffset();
        /*
         * 
         * author: wangqing
         * date: 2013-10-23
         *
            lui     addrTemp, offset >> 16
            ori     addrTemp, addrTemp, offset & 0x0000ffff
            addu    addrTemp, addrTemp, base
            lwc1    dest, (0)(addrTemp)
            cvt.d.s dest, dest
        */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, src.offset >> 16);
         ori(addrTempRegister, addrTempRegister, src.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, src.base);
         lwc1(dest, addrTempRegister, 0);
         cvtds(dest, dest);
         return label;
    }

    CodeOffsetLabel movsdWithPatch(Address src, FloatRegister dest) {
        //   JS_ASSERT(HasSSE2());
        //    masm.movsd_mr_disp32(src.offset, src.base.code(), dest.code());
        //movsd(Operand(src),dest);
        //return masm.currentOffset();
         /*
          * 
          * author: wangqing
          * date: 2013-10-23
          *
               lui         addrTemp, offset >> 16
               ori         addrTemp, addrTemp, offset & 0x0000ffff
               addu        addrTemp, addrTemp, base
               lwc1        dest, 0(addrTemp)
               lwc1        dest+1, 4(addrTemp)
         */
          
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, src.offset >> 16);
         ori(addrTempRegister, addrTempRegister, src.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, src.base);
         lwc1(dest, addrTempRegister, 0);
         lwc1(mFPRegisterID(dest.code() + 1), addrTempRegister, 4);
         return label;
    }

    // Store to *(base + disp32) where disp32 can be patched.
    CodeOffsetLabel movbWithPatch(Register src, Address dest) {
        //  masm.movb_rm_disp32(src.code(), dest.offset, dest.base.code());
        //movb(src,Operand(dest));
        //return masm.currentOffset();
         /*
          * 
          * author: wangqing
          * date: 2013-10-23
          *
             lui     addrTemp, offset >> 16
             ori     addrTemp, addrTemp, offset & 0x0000ffff
             addu    addrTemp, addrTemp, base
             sb      src, (0)(addrTemp)
         */   
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, dest.offset >> 16);
         ori(addrTempRegister, addrTempRegister, dest.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, dest.base);
         sb(src, addrTempRegister, 0);
         return label;
    }

    CodeOffsetLabel movwWithPatch(Register src, Address dest) {
        // masm.movw_rm_disp32(src.code(), dest.offset, dest.base.code());
        //movw(src,Operand(dest));
        //return masm.currentOffset();
         /* 
          * 
          * author: wangqing
          * date: 2013-10-23
          *
              lui     addrTemp, offset >> 16
              ori     addrTemp, addrTemp, offset & 0x0000ffff
              addu    addrTemp, addrTemp, base
              sh      src, (0)(addrTemp)
         */  
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, dest.offset >> 16);
         ori(addrTempRegister, addrTempRegister, dest.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, dest.base);
         sh(src, addrTempRegister, 0);
         return label;
    }

    CodeOffsetLabel movlWithPatch(Register src, Address dest) {
        //     masm.movl_rm_disp32(src.code(), dest.offset, dest.base.code());
        //movl(src,Operand(dest));
        //return masm.currentOffset();
         /*
          * 
          * author : wangqing
          * date: 2013-10-23
          *
             lui     addrTemp, offset >> 16
             ori     addrTemp, addrTemp, offset & 0x0000ffff
             addu    addrTemp, addrTemp, base
             sw      src, addrTemp, 0
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, dest.offset >> 16);
         ori(addrTempRegister, addrTempRegister, dest.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, dest.base);
         sw(src, addrTempRegister, 0);
         return label;
    }

    CodeOffsetLabel movssWithPatch(FloatRegister src, Address dest) {
        //  JS_ASSERT(HasSSE2());
        //  masm.movss_rm_disp32(src.code(), dest.offset, dest.base.code());
        //movss(src,Operand(dest));
        //return masm.currentOffset();
                   
         /* 
          * author: wangqing
          * date: 2013-10-23
          *
              lui     addrTemp, offset >> 16
              ori     addrTemp, addrTemp, offset & 0xffff
              addu    addrTemp, addrTemp, base
              swc1    src, addrTemp, 0
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, dest.offset >> 16);
         ori(addrTempRegister, addrTempRegister, dest.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, dest.base);
         swc1(src, addrTempRegister, 0);
         return label;
    }

    CodeOffsetLabel movsdWithPatch(FloatRegister src, Address dest) {
        //  JS_ASSERT(HasSSE2());
        //    masm.movsd_rm_disp32(src.code(), dest.offset, dest.base.code());
        //movsd(src,Operand(dest));
        //return masm.currentOffset();
         /* 
          * author: wangqing
          * date: 2013-10-23
          *
            li          addrTemp, address.offset
            addu        addrTemp, addrTemp, base
            swc1        dest, 0(addrTemp)
            swc1        dest+1, 4(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, dest.offset >> 16);
         ori(addrTempRegister, addrTempRegister, dest.offset & 0x0000ffff);
         addu(addrTempRegister, addrTempRegister, dest.base);
         swc1(src, addrTempRegister, 0);
         swc1(mFPRegisterID(src.code() + 1), addrTempRegister, 4);
         return label;
    }

    // Load from *(addr + index*scale) where addr can be patched.
    CodeOffsetLabel movlWithPatch(PatchedAbsoluteAddress addr, Register index, Scale scale,
                                  Register dest)
    {
        //      masm.movl_mr(addr, index.code(), scale, dest.code());
        //      ASSERT(0);
        //		 mov(mImmPtr(addr),addrTempRegister);
        //mBaseIndex need to review!          
       // mcss.load32(mBaseIndex(addrTempRegister, index, mScale(scale)), dest.code());
       // return masm.currentOffset();
        /*
         * 
         * author: wangqing * date: 2013-10-23
         *     
             sll     addrTemp, address.index, address.scale
             addu    addrTemp, addrTemp, address.base
             lui     immTemp, address.offset >> 16
             ori     immTemp, immTemp, address.offset & 0xffff
             addu    addrTemp, addrTemp, immTemp
             lw      dest, 0(addrTemp)
        */
        sll(addrTempRegister, index, (int)scale);
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(immTempRegister, (int)addr.addr >> 16);
        ori(immTempRegister, immTempRegister, (int)addr.addr & 0x0000ffff);
        addu(addrTempRegister, addrTempRegister, immTempRegister);
        lw(dest, addrTempRegister, 0);
       return label;
    }

    // Load from *src where src can be patched.
    CodeOffsetLabel movsblWithPatch(const PatchedAbsoluteAddress &src, Register dest) {
        //masm.movsbl_mr(src.addr, dest.code());
        //JS_ASSERT(0);
        //movsbl(Operand(src.addr), dest);
        //return masm.currentOffset();
         /*
          * 
          * author: wangqing
          * date: 2013-12-24
          *
             lui     addrTemp, src.addr >> 16
             ori     addrTemp, addrTemp, src.addr&0x0000ffff 
             lb      dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, (int)src.addr >> 16);
         ori(addrTempRegister, addrTempRegister, (int)src.addr & 0x0000ffff);
         lb(dest, addrTempRegister, 0);
         return label;

    }

    CodeOffsetLabel movzblWithPatch(const PatchedAbsoluteAddress &src, Register dest) {
        //JS_ASSERT(0);
        //masm.movzbl_mr(src.addr, dest.code());
        //movzbl(Operand(src.addr), dest);
        //return masm.currentOffset();
        /* 
          *
          * author: wangqing
          * date: 2013-12-24
          *
             lui     addrTemp, src.addr >> 16
             ori     addrTemp, addrTemp, src.addr & 0x0000ffff 
             lbu     dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, (int)src.addr >> 16);
         ori(addrTempRegister, addrTempRegister, (int)src.addr & 0x0000ffff);
         lbu(dest, addrTempRegister, 0);
         return label;

    }
    // folloing is from Assembler-x86-shared.h
   public:
    void align(int alignment) {
        masm.align(alignment);
    }

    // author: huangwenjun date:2013-12-23
    void writeCodePointer(AbsoluteLabel *label) {
         JS_ASSERT(!label->bound());
        // Thread the patch list through the unpatched address word in the
        // instruction stream.
        masm.emitInst(label->prev());
        label->setPrev(masm.size());
        // for JumpTable
        label->setType(1);
    }
    
    // author: huangwenjun date:2013-12-23
    void writeDoubleConstant(double d, Label *label) {
        label->bind(masm.size());
        masm.doubleConstant(d);
    }

    // author: huangwenjun date:2013-12-23 need check
    //New function
    void writeFloatConstant(float f, Label *label) {
        JS_ASSERT(0);
//        label->bind(masm.size());
//        masm.floatConstant(f);
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
          case Operand::MEM_REG_DISP:
   //         masm.movl_mr(src.disp(), src.base(), dest.code());
            mcss.load32(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
    //        masm.movl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load32(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
      //      masm.movl_mr(src.address(), dest.code());
            mcss.load32(src.address(), dest.code());
            break;
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
          case Operand::MEM_REG_DISP:
     //       masm.movl_rm(src.code(), dest.disp(), dest.base());
            mcss.store32(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::MEM_SCALE:
      //      masm.movl_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store32(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          case Operand::MEM_ADDRESS32:
     //      masm.movl_rm(src.code(), dest.address());
            mcss.store32(src.code(), dest.address());
            break;
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
          case Operand::MEM_REG_DISP:
//ok            masm.movl_i32m(imm32.value, dest.disp(), dest.base());
            mcss.store32(mTrustedImm32(imm32.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::MEM_SCALE:
//ok            masm.movl_i32m(imm32.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store32(mTrustedImm32(imm32.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    
    // Exchange the value of src and dest.
    void xchgl(const Register &src, const Register &dest) {
        //JS_ASSERT(0);
        //masm.xchgl_rr(src.code(), dest.code());
        movl(src, addrTempRegister);
        movl(dest, src);
        movl(addrTempRegister, dest);
    }

    // Eventually movapd and movaps should be overloaded to support loads and
    // stores too.
    // New function
    void movapd(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.movapd_rr(src.code(), dest.code());
    }
    void movsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.movsd_rr(src.code(), dest.code());
            mcss.moveDouble(src.code(), dest.code());
    }
    // New function
    void movaps(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        masm.movaps_rr(src.code(), dest.code());
    }

    // movsd and movss are only provided in load/store form since the
    // register-to-register form has different semantics (it doesn't clobber
    // the whole output register) and isn't needed currently.
    // New function
    void movsd(const Address &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
        //masm.movsd_mr(src.offset, src.base.code(), dest.code());
        movsd(Operand(src), dest);
    }
    void movsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.movsd_rr(src.fpu(), dest.code());
            mcss.moveDouble(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
//ok            masm.movsd_mr(src.disp(), src.base(), dest.code());
            mcss.loadDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
//ok            masm.movsd_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.loadDouble(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
            mcss.loadDouble(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void movsd(const BaseIndex &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
        //masm.movsd_mr(src.offset, src.base.code(), src.index.code(), src.scale, dest.code());
        movsd(Operand(src), dest);
    }
    void movsd(const FloatRegister &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::FPREG:
//ok            masm.movsd_rr(src.code(), dest.fpu());
            mcss.moveDouble(src.code(), dest.fpu());
            break;
          case Operand::MEM_REG_DISP:
//ok            masm.movsd_rm(src.code(), dest.disp(), dest.base());
            mcss.storeDouble(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::MEM_SCALE:
//ok            masm.movsd_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.storeDouble(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          case Operand::MEM_ADDRESS32:
            mcss.storeDouble(src.code(), dest.address());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    CodeOffsetLabel movswlWithPatch(const PatchedAbsoluteAddress &src, Register dest) {
        //JS_ASSERT(0);
        //masm.movswl_mr(src.addr, dest.code());
        //movswl(Operand(src.addr), dest);
        //return masm.currentOffset();
         /* 
          *
          * author: wangqing
          * date: 2013-12-24
          *
             lui     addrTemp, src.addr >> 16
             ori     addrTemp, addrTemp, src.addr & 0x0000ffff
             lh      dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, (int)src.addr >> 16);
         ori(addrTempRegister, addrTempRegister, (int)src.addr & 0x0000ffff);
         lh(dest, addrTempRegister, 0);
         return label;

    }
    // New function
    void movsd(const FloatRegister &src, const Address &dest) {
        //JS_ASSERT(0);
        //masm.movsd_rm(src.code(), dest.offset, dest.base.code());
        movsd(src, Operand(dest));
    }

    CodeOffsetLabel movzwlWithPatch(const PatchedAbsoluteAddress &src, Register dest) {
        //JS_ASSERT(0);
        //masm.movzwl_mr(src.addr, dest.code());
        //movzwl(Operand(src.addr), dest);
        //return masm.currentOffset();
         /* 
          *
          * author: wangqing
          * date: 2013-12-24
          *
             lui     addrTemp, src.addr >> 16
             ori     addrTemp, addrTemp, src.addr&0x0000ffff
             lhu     dest, (0)(addrTemp)
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, (int)src.addr >> 16);
         ori(addrTempRegister, addrTempRegister, (int)src.addr & 0x0000ffff);
         lhu(dest, addrTempRegister, 0);
         return label;
    }

    void movss(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movss_mr(src.disp(), src.base(), dest.code());
            mcss.loadFloat(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
//ok            masm.movss_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.loadFloat(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
            mcss.loadFloat(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    CodeOffsetLabel movlWithPatch(const PatchedAbsoluteAddress &src, Register dest) {
        //JS_ASSERT(0);
        //masm.movl_mr(src.addr, dest.code());
        //movl(Operand(src.addr), dest);
        //return masm.currentOffset();
        /* 
         * author: wangqing
         * date: 2013-12-24
         *
         * lui addrTemp.code(), src.addr >> 16
         * ori addrTemp.code(), src.addr & 0x0000ffff
         * lw dest.code(), addrTemp.code(), 0
         */
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(addrTempRegister, (int)src.addr >> 16);
        ori(addrTempRegister, addrTempRegister, (int)src.addr & 0x0000ffff);
        lw(dest, addrTempRegister, 0);
        return label;

    }

    CodeOffsetLabel movssWithPatch(const PatchedAbsoluteAddress &src, FloatRegister dest) {
        //JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.movss_mr(src.addr, dest.code());
        //movss(Operand(src.addr), dest);
        //return masm.currentOffset();
        /*
         * 
         * author: wangqing
         * date: 2013-12-24
         *
            lui     addrTemp, src.addr >> 16
            ori     addrTemp, addrTemp, src.addr & 0x0000ffff
            lwc1    dest, (0)(addrTemp)
            cvt.d.s dest, dest
        */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, (int)src.addr >> 16);
         ori(addrTempRegister, addrTempRegister, (int)src.addr & 0x0000ffff);
         lwc1(dest, addrTempRegister, 0);
         cvtds(dest, dest);
         return label;

    }
    // New function
    void movsd(const FloatRegister &src, const BaseIndex &dest) {
        //JS_ASSERT(0);
        //masm.movsd_rm(src.code(), dest.offset, dest.base.code(), dest.index.code(), dest.scale);
        movsd(src, Operand(dest));
    }

    // by wangqing, 2013-11-08
    void movss(const Register &src, const FloatRegister &dest) {
        mtc1(src, dest);
        cvtds(dest, dest);
    }

    void movss(const FloatRegister &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movss_rm(src.code(), dest.disp(), dest.base());
            mcss.storeFloat(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::MEM_SCALE:
//ok            masm.movss_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.storeFloat(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
          case Operand::MEM_ADDRESS32:
            mcss.storeFloat(src.code(), dest.address());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    CodeOffsetLabel movsdWithPatch(const PatchedAbsoluteAddress &src, FloatRegister dest) {
        //JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.movsd_mr(src.addr, dest.code());
        //movsd(Operand(src.addr), dest);
        //return masm.currentOffset();
        /* 
         * author: wangqing
         * date: 2013-12-24
         *
         * lui addrTemp, src.addr >> 16
         * ori addrTemp, addrTemp, src.addr & 0x0000ffff
         * lwc1 dest, addrTemp, 0
         * lwc1 dest+1, addrTemp, 4
         */
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(addrTempRegister, (int)src.addr >> 16);
        ori(addrTempRegister, addrTempRegister, (int)src.addr & 0x0000ffff);  
        lwc1(dest, addrTempRegister, 0);
        lwc1(mFPRegisterID(dest.code()+1), addrTempRegister, 4);
        return label;

    }

    void movss(const Address &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
        //masm.movss_mr(src.offset, src.base.code(), dest.code());
        movss(Operand(src), dest);
    }

    void movss(const BaseIndex &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
        //masm.movss_mr(src.offset, src.base.code(), src.index.code(), src.scale, dest.code());
        movss(Operand(src), dest);
    }

    void movss(const FloatRegister &src, const Address &dest) {
        //JS_ASSERT(0);
        //masm.movss_rm(src.code(), dest.offset, dest.base.code());
        movss(src, Operand(dest));
    }

    void movss(const FloatRegister &src, const BaseIndex &dest) {
        //JS_ASSERT(0);
        //masm.movss_rm(src.code(), dest.offset, dest.base.code(), dest.index.code(), dest.scale);
        movss(src, Operand(dest));
    }

    // New function
    void movdqa(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        switch (src.kind()) {
//          case Operand::MEM_REG_DISP:
//            masm.movdqa_mr(src.disp(), src.base(), dest.code());
//            break;
//          case Operand::MEM_SCALE:
//            masm.movdqa_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
//            break;
//          default:
//            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
//        }
    }
    // New function
    void movdqa(const FloatRegister &src, const Operand &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        switch (dest.kind()) {
//          case Operand::MEM_REG_DISP:
//            masm.movdqa_rm(src.code(), dest.disp(), dest.base());
//            break;
//          case Operand::MEM_SCALE:
//            masm.movdqa_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
//            break;
//          default:
//            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
//        }
    }
     void cvtss2sd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.cvtss2sd_rr(src.code(), dest.code());
        mcss.convertFloatToDouble(src.code(), dest.code());
    }
    void cvtsd2ss(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.cvtsd2ss_rr(src.code(), dest.code());
        mcss.convertDoubleToFloat(src.code(), dest.code());
    }

    // Store to *dest where dest can be patched.
    CodeOffsetLabel movbWithPatch(Register src, const PatchedAbsoluteAddress &dest) {
        //JS_ASSERT(0);
        //masm.movb_rm(src.code(), dest.addr);
        //movb(src, Operand(dest.addr));
        //return masm.currentOffset();
         /*
          * 
          * author: wangqing
          * date: 2013-12-24
          *
             lui     addrTemp, dest.addr >> 16
             ori     addrTemp, addrTemp, dest.addr & 0x0000ffff
             sb      src, (0)(addrTemp)
         */   
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, (int)dest.addr >> 16);
         ori(addrTempRegister, addrTempRegister, (int)dest.addr & 0x0000ffff);
         sb(src, addrTempRegister, 0);
         return label;
    }
    void movzbl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movzbl_mr(src.disp(), src.base(), dest.code());
            mcss.load8ZeroExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
//ok            masm.movzbl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load8ZeroExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
            mcss.load8ZeroExtend(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    CodeOffsetLabel movwWithPatch(Register src, const PatchedAbsoluteAddress &dest) {
        //JS_ASSERT(0);
        //masm.movw_rm(src.code(), dest.addr);
        //movw(src, Operand(dest.addr));
        //return masm.currentOffset();
        /* 
          * 
          * author: wangqing
          * date: 2013-12-24
          *
              lui     addrTemp, dest.addr >> 16
              ori     addrTemp, addrTemp, dest.addr & 0x0000ffff
              sh      src, (0)(addrTemp)
         */  
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, (int)dest.addr >> 16);
         ori(addrTempRegister, addrTempRegister, (int)dest.addr & 0x0000ffff);
         sh(src, addrTempRegister, 0);
         return label;
    }

    void movsbl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movxbl_mr(src.disp(), src.base(), dest.code());
            mcss.load8SignExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
//ok            masm.movxbl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load8SignExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
            mcss.load8SignExtend(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    CodeOffsetLabel movlWithPatch(Register src, const PatchedAbsoluteAddress &dest) {
        //JS_ASSERT(0);
        //masm.movl_rm(src.code(), dest.addr);
        //movl(src, Operand(dest.addr));
        //return masm.currentOffset();
        /* 
         * author: wangqing
         * date: 2013-12-24
         *
         * lui addrTemp, dest.addr >> 16
         * ori addrTemp, addrTemp, dest.addr & 0x0000ffff
         * sw src.code(), addrTemp, 0
         */
        
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(addrTempRegister, (int)dest.addr >> 16);
        ori(addrTempRegister, addrTempRegister, (int)dest.addr & 0x0000ffff);
        sw(src, addrTempRegister, 0);
        return label;

    }
    void movb(const Register &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movb_rm(src.code(), dest.disp(), dest.base());
            mcss.store8(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::MEM_SCALE:
//ok            masm.movb_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store8(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          case Operand::MEM_ADDRESS32:
            mcss.store8(src.code(), dest.address());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    CodeOffsetLabel movssWithPatch(FloatRegister src, const PatchedAbsoluteAddress &dest) {
        //JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.movss_rm(src.code(), dest.addr);
        //movss(src, Operand(dest.addr));
        //return masm.currentOffset();
        /* 
          * author: wangqing
          * date: 2013-12-24
          *
              lui     addrTemp, dest.addr >> 16
              ori     addrTemp, addrTemp, dest.addr & 0xffff
              swc1    src, addrTemp, 0
         */
         CodeOffsetLabel label = CodeOffsetLabel(size());
         lui(addrTempRegister, (int)dest.addr >> 16);
         ori(addrTempRegister, addrTempRegister, (int)dest.addr & 0x0000ffff);
         swc1(src, addrTempRegister, 0);
         return label;

    }

    CodeOffsetLabel movsdWithPatch(FloatRegister src, const PatchedAbsoluteAddress &dest) {
        //JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.movsd_rm(src.code(), dest.addr);
        //movsd(src, Operand(dest.addr));
        //return masm.currentOffset();
        /* 
         * author: wangqing
         * date: 2013-12-24
         *
         * lui addrTemp, dest.addr >> 16
         * ori addrTemp, addrTemp, dest.addr & 0x0000ffff
         * swc1 src, addrTemp, 0
         * swc1 src+1, addrTemp, 4
         */
        
        CodeOffsetLabel label = CodeOffsetLabel(size());
        lui(addrTempRegister, (int)dest.addr >> 16);
        ori(addrTempRegister, addrTempRegister, (int)dest.addr & 0x0000ffff);  
        swc1(src, addrTempRegister, 0);
        swc1(mFPRegisterID(src.code()+1), addrTempRegister, 4);
        return label;
    }

    void movb(const Imm32 &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movb_i8m(src.value, dest.disp(), dest.base());
            mcss.store8(mTrustedImm32(src.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::MEM_SCALE:
//ok            masm.movb_i8m(src.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store8(mTrustedImm32(src.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movzwl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movzwl_mr(src.disp(), src.base(), dest.code());
            mcss.load16ZeroExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
//ok            masm.movzwl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load16ZeroExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
            mcss.load16ZeroExtend(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void movw(const Register &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movw_rm(src.code(), dest.disp(), dest.base());
            mcss.store16(src.code(), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::MEM_SCALE:
//ok            masm.movw_rm(src.code(), dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store16(src.code(), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          case Operand::MEM_ADDRESS32:
            mcss.store16(src.code(), dest.address());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movw(const Imm32 &src, const Operand &dest) {
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movw_i16m(src.value, dest.disp(), dest.base());
            mcss.store16(mTrustedImm32(src.value), mAddress(dest.base(), dest.disp()));
            break;
          case Operand::MEM_SCALE:
//ok            masm.movw_i16m(src.value, dest.disp(), dest.base(), dest.index(), dest.scale());
            mcss.store16(mTrustedImm32(src.value), mBaseIndex(dest.base(), dest.index(), mScale(dest.scale()), dest.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void movswl(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
//ok            masm.movxwl_mr(src.disp(), src.base(), dest.code());
            mcss.load16SignExtend(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
//ok            masm.movxwl_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.load16SignExtend(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
            mcss.load16SignExtend(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void leal(const Operand &src, const Register &dest) {
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
      //     masm.leal_mr(src.disp(), src.base(), dest.code());
         mcss.lea(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
        //    masm.leal_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
         mcss.lea(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

  protected:
    //author:huangwenjun date:2013-12-23
    JmpSrc jSrc(Condition cond, Label *label) {
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

    //author:huangwenjun date:2013-12-23
    JmpSrc jmpSrc(Label *label) {
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

    // Comparison of EAX against the address given by a Label.
    // no use
    // JmpSrc cmpSrc(Label *label) {
//        JmpSrc j = masm.cmp_eax();
//        if (label->bound()) {
//            // The jump can be immediately patched to the correct destination.
//            masm.linkJump(j, JmpDst(label->offset()));
//        } else {
//            // Thread the jump list through the unpatched jump targets.
//            JmpSrc prev = JmpSrc(label->use(j.offset()));
//            masm.setNextJump(j, prev);
//        }
//        return j;
//    }

    //author:huangwenjun date:2013-12-23
    JmpSrc jSrc(Condition cond, RepatchLabel *label) {
       JmpSrc j = mcss.branch32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), cmpTempRegister.code(), cmpTemp2Register.code()).m_jmp;
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            label->use(j.offset());
        }
        return j;
    }

    //author:huangwenjun date:2013-12-23
    JmpSrc jmpSrc(RepatchLabel *label) {
        JmpSrc j = mcss.jump().getJmpSrc();
        if (label->bound()) {
            // The jump can be immediately patched to the correct destination.
            masm.linkJump(j, JmpDst(label->offset()));
        } else {
            // Thread the jump list through the unpatched jump targets.
            label->use(j.offset());
        }
        return j;
    }

  public:
    void nop() { masm.nop(); }
    void j(Condition cond, Label *label) { jSrc(cond, label); }
    void jmp(Label *label) { jmpSrc(label); }
    void j(Condition cond, RepatchLabel *label) { jSrc(cond, label); }
    void jmp(RepatchLabel *label) { jmpSrc(label); }
    
    //author:huangwenjun date:2013-12-23
    void jmp(const Operand &op){
        switch (op.kind()) {
          case Operand::MEM_REG_DISP:
            mcss.jump(mAddress(op.base(), op.disp()));
            break;
          case Operand::MEM_SCALE:
            mcss.jump(mBaseIndex(op.base(), op.index(), mScale(op.scale()), op.disp()));
            break;
          case Operand::REG:
            mcss.jump(op.reg());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // no used
 //   void cmpEAX(Label *label) { cmpSrc(label); }
    
    //author:huangwenjun date:2013-12-23
    void bind(Label *label) {
        JSC::MacroAssembler::Label jsclabel;
        JSC::MIPSAssembler::JmpDst dst(masm.label());
        if (label->used()) {
            bool more;
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            do {
                JSC::MIPSAssembler::JmpSrc next;
                more = masm.nextJump(jmp, &next);
                
                //author:huangwenjun date:2013-12-23
                masm.clearOffsetForLabel(jmp);
                masm.linkJump(jmp, dst);

                jmp = next;
            } while (more);
        }
        label->bind(dst.offset());
    }

    //author:huangwenjun date:2013-12-23
    void bind(RepatchLabel *label) {
        JSC::MacroAssembler::Label jsclabel;
        if (label->used()) {
            JSC::MIPSAssembler::JmpSrc jmp(label->offset());
            //hwj
            masm.clearOffsetForLabel(jmp);
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
    
    //author:huangwenjun date:2013-12-23
    static void Bind(uint8_t *raw, AbsoluteLabel *label, const void *address) {
        if (label->used()&&(label->getType())) {//1 jump table
            intptr_t src = label->offset();
            do {
                intptr_t next =*((intptr_t*)(raw + src-5));//date:1108
                *((int*)(raw + src-4)) = (int)address;
                src = next;
            } while (src != AbsoluteLabel::INVALID_OFFSET);
        } else if (label->used()&&(!label->getType())) {
            //0 for mov function
            intptr_t src = label->offset();
            do {
                //hwj   //wangqing
                int* ptrLuiIns = (int*)(raw+src-8);//hwj date:1030
                int* ptrOriIns = (int*)(raw+src-4);//hwj date:1030

                int luiIns = *ptrLuiIns;
                int oriIns = *ptrOriIns;

                JS_ASSERT((luiIns&0xfc000000)==0x3c000000);
                JS_ASSERT((oriIns&0xfc000000)==0x34000000);
                intptr_t next = ((luiIns & 0x0000ffff)<<16) |(oriIns &0x0000ffff);
                *(ptrLuiIns) = (luiIns&0xffff0000)|((((int)address)&0xffff0000)>>16);
                *(ptrOriIns) = (oriIns&0xffff0000)|(((int)address)&0x0000ffff);
                src = next;
             } while (src != AbsoluteLabel::INVALID_OFFSET);
       }
       label->bind();
    }

    // See Bind and JSC::X86Assembler::setPointer.
    size_t labelOffsetToPatchOffset(size_t offset) {
        return offset - sizeof(void*);
    }
    void ret() {
//ok        masm.ret();
        pop(ra);
        mcss.ret();
    }
    void retn(Imm32 n);
    JmpSrc callWithPush();
    JmpSrc callRelWithPush();
    
    //author:huangwenjun date:2013-12-25
    void ma_call(const Register &reg);//for js->c++
    void ma_call(const Operand &op);//for js->c++
    void ma_call(ImmWord target);//for js->c++
    JmpSrc ma_call(void *dest);

   // calls an Ion function, assumes that the stack is untouched (8 byte alinged)
    JmpSrc ma_callIon(const Register reg);
    // callso an Ion function, assuming that sp has already been decremented
    JmpSrc ma_callIonNoPush(const Register reg);
    // calls an ion function, assuming that the stack is currently not 8 byte aligned
    JmpSrc ma_callIonHalfPush(const Register reg);

    void breakpoint() {
    //    masm.int3();
      mcss.breakpoint();
    }
/*
    // New function, maybe useless
    static bool HasSSE2() {
        JS_ASSERT(0);
        //return JSC::MacroAssembler::getSSEState() >= JSC::MacroAssembler::HasSSE2;
    }
    static bool HasSSE3() {
        JS_ASSERT(0);
        //return JSC::MacroAssembler::getSSEState() >= JSC::MacroAssembler::HasSSE3;
    }
    static bool HasSSE41() {
        JS_ASSERT(0);
        //return JSC::MacroAssembler::getSSEState() >= JSC::MacroAssembler::HasSSE4_1;
    }
*/
    // The below cmpl methods switch the lhs and rhs when it invokes the
    // macroassembler to conform with intel standard.  When calling this
    // function put the left operand on the left as you would expect.
    void cmpl(const Register &lhs, const Operand &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
    void cmpl(const Register &src, Imm32 imm) {
        movl(src,cmpTempRegister);
        movl(imm,cmpTemp2Register);
    }
    void cmpl(const Operand &op, Imm32 imm) {
        movl(op,cmpTempRegister);
        movl(imm,cmpTemp2Register);
    }
    void cmpl(const Operand &lhs, const Register &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
    void cmpl(const Operand &op, ImmWord imm) {
        movl(op,cmpTempRegister);
        movl(imm,cmpTemp2Register);
    }
    void cmpl(const Operand &op, ImmPtr imm) {
        cmpl(op, ImmWord(uintptr_t(imm.value)));
    }
    //author:huangwenjun date:2013-12-25
    void setCC(Condition cond, const Register &r);
    void testb(const Register &lhs, const Register &rhs) {
        JS_ASSERT(GeneralRegisterSet(Registers::SingleByteRegs).has(lhs));//SingleBytesRegs:t6,t7,t8,s0...s7,v0
        JS_ASSERT(GeneralRegisterSet(Registers::SingleByteRegs).has(rhs));//?
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
    void testl(const Register &lhs, const Register &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
        andl(cmpTempRegister,cmpTemp2Register);
        movl(zero,cmpTempRegister);
    }
    void testl(const Register &lhs, Imm32 rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
        andl(cmpTempRegister,cmpTemp2Register);
        movl(zero,cmpTempRegister);
    }
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
          case Operand::MEM_REG_DISP:
//ok            masm.addl_im(imm.value, op.disp(), op.base());
            mcss.add32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          case Operand::MEM_ADDRESS32:
//ok            masm.addl_im(imm.value, op.address());
            mcss.load32(op.address(), dataTempRegister.code());
            mcss.add32(mTrustedImm32(imm.value), dataTempRegister.code());
            mcss.store32(dataTempRegister.code(), op.address());
            break;
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
//ok            masm.xorl_im(imm.value, op.disp(), op.base());
            mcss.xor32(mTrustedImm32(imm.value), mAddress(op.base(), op.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
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
          case Operand::MEM_REG_DISP:
//ok            masm.notl_m(src.disp(), src.base());
            mcss.not32(mAddress(src.base(), src.disp()));
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
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
    // New function
    void xaddl(const Register &srcdest, const Operand &mem) {
        JS_ASSERT(0);
        switch (mem.kind()) {
          case Operand::MEM_REG_DISP:
            //masm.xaddl_rm(srcdest.code(), mem.disp(), mem.base());
            break;
          case Operand::MEM_SCALE:
            //masm.xaddl_rm(srcdest.code(), mem.disp(), mem.base(), mem.index(), mem.scale());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
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
          case Operand::MEM_REG_DISP:
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
    void push(const Address &src) {
        //masm.push_m(src.offset, src.base.code());
        mcss.sub32(mTrustedImm32(4), mRegisterID(sp.code()));
        mcss.load32(mAddress(src.base.code(), src.offset), dataTempRegister.code());
        mcss.store32(dataTempRegister.code(), mAddress(sp.code(), 0));
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
          case Operand::MEM_REG_DISP:
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

    void pushFlags() {
        push(cmpTempRegister);
        push(cmpTemp2Register);
    }
    void popFlags() {
        pop(cmpTemp2Register);
        pop(cmpTempRegister);
    }

    // New function
#ifdef JS_CPU_X86
    void pushAllRegs() {
        //masm.pusha();
    }
    void popAllRegs() {
        //masm.popa();
    }
#endif

    // Zero-extend byte to 32-bit integer.
    // New function
    void movzbl(const Register &src, const Register &dest) {
        //masm.movzbl_rr(src.code(), dest.code());
    	mcss.zeroExtend32ToPtr(src.code(),dest.code());//mipsÖÐ²¢Ã»ÓÐÊµÏÖÀ©Õ¹£¡
    }

    void movzxbl(const Register &src, const Register &dest)
    { 
    	//masm.movzbl_rr(src.code(), dest.code());
    	mcss.zeroExtend32ToPtr(src.code(),dest.code());//mipsÖÐ²¢Ã»ÓÐÊµÏÖÀ©Õ¹£¡
    }

    // Zero-extend byte to 32-bit integer.
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
        div(t6, divisor);
        mfhi(t7);
        mflo(t6);
    }
    void udiv(Register divisor) {
      // masm.divl_r(divisor.code());// in x86:div unsigned
        divu(t6, divisor);
        mfhi(t7);
        mflo(t6);
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
    // New function
    void pinsrd(const Operand &src, const FloatRegister &dest) {
    	JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::REG:
            //masm.pinsrd_rr(src.reg(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.pinsrd_mr(src.disp(), src.base(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void psrldq(Imm32 shift, const FloatRegister &dest) {
      ASSERT(0);
      /*  JS_ASSERT(HasSSE2());
        masm.psrldq_ir(shift.value, dest.code());*/
    }
    // New function
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

    void cvtsi2sd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::REG:
//ok            masm.cvtsi2sd_rr(src.reg(), dest.code());
            mcss.convertInt32ToDouble(src.reg(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
//ok            masm.cvtsi2sd_mr(src.disp(), src.base(), dest.code());
            mcss.convertInt32ToDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
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

    void cvttss2si(const FloatRegister &src, const Register &dest) {
        //JS_ASSERT(0);
        mcss.truncateFloatToInt32(src.code(), dest.code());
//        JS_ASSERT(HasSSE2());
//        masm.cvttss2si_rr(src.code(), dest.code());
    }

    void cvtsi2ss(const Operand &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::REG:
            //masm.cvtsi2ss_rr(src.reg(), dest.code());
            mcss.convertInt32ToFloat(src.reg(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.cvtsi2ss_mr(src.disp(), src.base(), dest.code());
            mcss.convertInt32ToFloat(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_SCALE:
            //masm.cvtsi2ss_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            mcss.convertInt32ToFloat(mBaseIndex(src.base(), src.index(), mScale(src.scale()), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void cvtsi2ss(const Register &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.cvtsi2ss_rr(src.code(), dest.code());
        mcss.convertInt32ToFloat(src.code(), dest.code());
    }

    void cvtsi2sd(const Register &src, const FloatRegister &dest) {
//ok        masm.cvtsi2sd_rr(src.code(), dest.code());
        mcss.convertInt32ToDouble(src.code(), dest.code());
    }

    void movmskpd(const FloatRegister &src, const Register &dest) {
        JS_ASSERT(0);
     //   JS_ASSERT(HasSSE2());
    //    masm.movmskpd_rr(src.code(), dest.code());
        // fix me: by wangqing
//        dmfc1(dest, src);
//        dsrl32(dest, dest, 31);
    }

    void movmskps(const FloatRegister &src, const Register &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        masm.movmskps_rr(src.code(), dest.code());
        // fix me; by wangqing
//        dmfc1(dest, src);
    }

   void ptest(const FloatRegister &lhs, const FloatRegister &rhs) {
  ASSERT(0);
   /*     JS_ASSERT(HasSSE41());
        masm.ptest_rr(rhs.code(), lhs.code());*/
    }

    void ucomisd(const FloatRegister &lhs, const FloatRegister &rhs) {
     //   JS_ASSERT(HasSSE2());
   //     masm.ucomisd_rr(rhs.code(), lhs.code());
        mcss.moveDouble(lhs.code(), fpTempRegister.code());
        mcss.moveDouble(rhs.code(), fpTemp2Register.code());
    }

    // New function
    void ucomiss(const FloatRegister &lhs, const FloatRegister &rhs) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        masm.ucomiss_rr(rhs.code(), lhs.code());
        mcss.moveFloat(lhs.code(), fpTempRegister.code());
        mcss.moveFloat(lhs.code(), fpTempRegister.code());
    }

    void pcmpeqw(const FloatRegister &lhs, const FloatRegister &rhs) {
   ASSERT(0);
    /*  JS_ASSERT(HasSSE2());
        masm.pcmpeqw_rr(rhs.code(), lhs.code());*/
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

    void addss(const FloatRegister &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        masm.addss_rr(src.code(), dest.code());
        mcss.addFloat(src.code(), dest.code());
    }

    void addsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.addsd_rr(src.fpu(), dest.code());
            mcss.addDouble(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
//ok            masm.addsd_mr(src.disp(), src.base(), dest.code());
            mcss.addDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
//ok            masm.addsd_mr(src.address(), dest.code());
            mcss.addDouble(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void addss(const Operand &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
            //masm.addss_rr(src.fpu(), dest.code());
            mcss.addFloat(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.addss_mr(src.disp(), src.base(), dest.code());
            mcss.addFloat(mAddress(src.base(), src.disp()), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
            //masm.addss_mr(src.address(), dest.code());
            mcss.addFloat(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void subsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.subsd_rr(src.code(), dest.code());
        mcss.subDouble(src.code(), dest.code());
    }

    void subss(const FloatRegister &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.subss_rr(src.code(), dest.code());
        mcss.subFloat(src.code(), dest.code());
    }
    void subsd(const Operand &src, const FloatRegister &dest) {
        //JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.subsd_rr(src.fpu(), dest.code());
            mcss.subDouble(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
//ok            masm.subsd_mr(src.disp(), src.base(), dest.code());
            mcss.subDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void subss(const Operand &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
            //masm.subss_rr(src.fpu(), dest.code());
            mcss.subFloat(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.subss_mr(src.disp(), src.base(), dest.code());
            mcss.subFloat(mAddress(src.base(), src.disp()), dest.code());
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
          case Operand::MEM_REG_DISP:
//ok            masm.mulsd_mr(src.disp(), src.base(), dest.code());
            mcss.mulDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void mulss(const Operand &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
            //masm.mulss_rr(src.fpu(), dest.code());
            mcss.mulFloat(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.mulss_mr(src.disp(), src.base(), dest.code());
            mcss.mulFloat(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void mulss(const FloatRegister &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        //masm.mulss_rr(src.code(), dest.code());
        mcss.mulFloat(src.code(), dest.code());
    }

    void divsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.divsd_rr(src.code(), dest.code());
        mcss.divDouble(src.code(), dest.code());
    }

    void divss(const FloatRegister &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        //masm.divss_rr(src.code(), dest.code());
        mcss.divFloat(src.code(), dest.code());
    }

    void divsd(const Operand &src, const FloatRegister &dest) {
        switch (src.kind()) {
          case Operand::FPREG:
//ok            masm.divsd_rr(src.fpu(), dest.code());
            mcss.divDouble(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
//ok            masm.divsd_mr(src.disp(), src.base(), dest.code());
            mcss.divDouble(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void divss(const Operand &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
            //masm.divss_rr(src.fpu(), dest.code());
            mcss.divFloat(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.divss_mr(src.disp(), src.base(), dest.code());
            mcss.divFloat(mAddress(src.base(), src.disp()), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }

    void zerod(const FloatRegister &src);
    void zeros(const FloatRegister &src);
    void xorpd(const FloatRegister &src, const FloatRegister &dest) {
          //ASSERT(0);
   //     JS_ASSERT(HasSSE2());
  //      masm.xorpd_rr(src.code(), dest.code());
        ASSERT(src.code() == dest.code());
        zerod(src);
    }

    void xorps(const FloatRegister &src, const FloatRegister &dest) {
        //JS_ASSERT(HasSSE2());
        //masm.xorps_rr(src.code(), dest.code());
        // fix me: by wangqing
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
    // New function
    void andps(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.andps_rr(src.code(), dest.code());
    }
    void sqrtsd(const FloatRegister &src, const FloatRegister &dest) {
       // JS_ASSERT(HasSSE2());
      //  masm.sqrtsd_rr(src.code(), dest.code());
        mcss.sqrtDouble(src.code(), dest.code());
    }
    // New function
    void sqrtss(const FloatRegister &src, const FloatRegister &dest) {
        //JS_ASSERT(0);
    //    JS_ASSERT(HasSSE2());
    //    masm.sqrtss_rr(src.code(), dest.code());
        mcss.sqrtFloat(src.code(), dest.code());
    }
    // New function
    void roundsd(const FloatRegister &src, const FloatRegister &dest)
                 //JSC::X86Assembler::RoundingMode mode)
    {
        //JS_ASSERT(0);
    //    JS_ASSERT(HasSSE41());
    //    masm.roundsd_rr(src.code(), dest.code(), mode);
        mcss.floorDouble(src.code(), dest.code());
    }

    void roundss(const FloatRegister &src, const FloatRegister &dest)
                 //JSC::X86Assembler::RoundingMode mode)
    {
        //JS_ASSERT(0);
//        JS_ASSERT(HasSSE41());
//        masm.roundss_rr(src.code(), dest.code(), mode);
        mcss.floorFloat(src.code(), dest.code());
    }
    
    //author:huangwenjun date:2013-12-25
    //modify:xueshuangbai
    // New function
    //void fisttp(const Operand &dest) {
    //    ASSERT(0);
        //JS_ASSERT(HasSSE3());
        //switch (dest.kind()) {
        //  case Operand::MEM_REG_DISP:
        //    //masm.fisttp_m(dest.disp(), dest.base());
        //    break;
        //  default:
        //    MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        //}
    //}
    
    //author:huangwenjun date:2013-12-25
    //modify:xueshuangbai
    // New function
    // void fld(const Operand &dest) {
    //     ASSERT(0);
    //    switch (dest.kind()) {
    //      case Operand::MEM_REG_DISP:
            //masm.fld_m(dest.disp(), dest.base());
    //        push(dest); // by wangqing
    //        break;
    //      default:
    //        MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
    //    }
   // }
    
    //author:huangwenjun date:2013-12-25
    //modify by xueshuanbai
    // New function
//    void fstp(const Operand &src) {
//        ASSERT(0);
//        switch (src.kind()) {
//          case Operand::MEM_REG_DISP:
//            //masm.fstp_m(src.disp(), src.base());
//            break;
//          default:
//            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
//        }
//    }

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
    //author:huangwenjun date:2013-12-23
    static size_t patchWrite_NearCallSize() {
        return 32;//8*4
    }

    //author:huangwenjun date:2013-12-23
    static uintptr_t getPointer(uint8_t *instPtr) {
        uintptr_t ptr = reinterpret_cast<uintptr_t>(JSC::MIPSAssembler::getPointer(instPtr));
        return ptr;
    }
    // Write a relative call at the start location |dataLabel|.
    // Note that this DOES NOT patch data that comes before |label|.
    static void patchWrite_NearCall(CodeLocationLabel startLabel, CodeLocationLabel target);

    static void patchWrite_Imm32(CodeLocationLabel dataLabel, Imm32 toWrite) {
     //   *((int32_t *) dataLabel.raw() - 1) = toWrite.value;
         JSC::MIPSAssembler::setInt32((int32_t *) dataLabel.raw() - 1, toWrite.value);
    }

    static void patchDataWithValueCheck(CodeLocationLabel data, PatchedImmPtr newData,
                                        PatchedImmPtr expectedData) {
        // The pointer given is a pointer to *after* the data.
        // uintptr_t *ptr = ((uintptr_t *) data.raw()) - 1;
        // JS_ASSERT(*ptr == expectedData.value);
        // *ptr = newData.value;
        uint32_t old = JSC::MIPSAssembler::getInt32(data.raw());
        JS_ASSERT(old == (uint32_t)expectedData.value);
        JSC::MIPSAssembler::setInt32(((uint8_t *)data.raw()), *(int32_t *)(newData.value));
        /* 
         * author: wangqing
         * date: 2013-10-18
         *
         * The pointer given is a pointer to *before* the data.
         *
         * lui reg, oldData_hi
         * ori reg, reg, oldData_low
         *      |    |     
         *      |    |
         * lui reg, newData_hi
         * ori reg, reg, newData_low
         */
        uint32_t *ptr = ((uintptr_t*) data.raw());
        uint32_t luiIns = *ptr;
        uint32_t oriIns = *(ptr+1);
        JS_ASSERT(luiIns & 0xfc000000 == 0x3c000000); // whether is lui 
        JS_ASSERT(oriIns & 0xfc000000 == 0x34000000); // whether is ori 
        uint32_t oldData = ((luiIns & 0x0000ffff) << 16) | (oriIns & 0x0000ffff);
        JS_ASSERT(oldData == (uint32_t)expectedData.value);
        *ptr = (luiIns & 0xffff0000) | ((*(int32_t *)(newData.value) & 0xffff0000) >> 16);
        *(ptr+1) = (oriIns & 0xffff0000) | (*(int32_t *)(newData.value) & 0x0000ffff);
    }
    static void patchDataWithValueCheck(CodeLocationLabel data, ImmPtr newData, ImmPtr expectedData) {
        patchDataWithValueCheck(data, PatchedImmPtr(newData.value), PatchedImmPtr(expectedData.value));
    }
    static uint32_t nopSize() {
    //    return 1;
       return 4;
    }
    static uint8_t *nextInstruction(uint8_t *cur, uint32_t *count) {
        MOZ_ASSUME_UNREACHABLE("nextInstruction NYI on MIPS");
    }

    //author:huangwenjun date:2013-12-23
    //CMP->JMP
    // Toggle a jmp or cmp emitted by toggledJump().
    static void ToggleToJmp(CodeLocationLabel inst) {
        int *ptr = (int *)inst.raw();

        ASSERT(*(ptr) == 0x10000005); //cmp eax
        *(ptr)=*(ptr+5);    //jmp recover
        *(ptr+5) = 0x00000000;
    }

    //author:huangwenjun date:2013-12-23
    //JMP->CMP
    static void ToggleToCmp(CodeLocationLabel inst) {
        int *ptr = (int *)inst.raw();
        ASSERT(*(ptr+5) == 0x00000000);
        *(ptr+5) = *ptr;    //backup fisrt instruction to nop
        *(ptr)=0x10000005;    //cmp eax
    }

    //author:huangwenjun date:2013-12-23
    //set CMP|CALL     
    static void ToggleCall(CodeLocationLabel inst, bool enabled) {
        int *ptr = (int *)inst.raw();

        ASSERT(((*(ptr+2)==0x10000005)&&(*(ptr+3) == 0x00000000))       //beq 0,0,5; nop ;        
             ||((*(ptr+2)==0x27bdfffc)&&(*(ptr+3) == 0xafb90000)));    //addiu, sp, sp,-4; sw t9,0(sp);
        if(enabled) {
            *(ptr+2) = 0x27bdfffc;      //addiu, sp, sp,-4
            *(ptr+3) = 0xafb90000;      //sw t9,0(sp)
        }
        else {
            *(ptr+2) = 0x10000005;      //beq r0, r0,5
            *(ptr+3) = 0x00000000;      //nop 
        }
    }

    void movz(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.movz(rd.code(), rs.code(), rt.code());
    }

    void move(const Register &rd, const Register &rs)
    {
        masm.move(rd.code(), rs.code());
    }

    /* Set an immediate value to a register.  This may generate 1 or 2
       instructions.  */
    void li(const Register &dest, ImmWord imm)
    {
        masm.li(dest.code(), imm.value);
    }

    void lui(const Register &rt, ImmWord imm)
    {
        masm.lui(rt.code(), imm.value);
    }
       
    // by wangqing, overloaded lui 
    void lui(const Register &rt, int32_t imm)
    {
        masm.lui(rt.code(), imm);
    }

    // by wangqing, overloaded lui 
    void lui(const mRegisterID rt, int32_t imm)
    {
        masm.lui(rt, imm);
    }

    void addiu(const Register &rt, const Register &rs, ImmWord imm)
    {
        masm.addiu(rt.code(), rs.code(), imm.value);
    }

    // by wangqing, overload addiu
    void addiu(const Register &rt, const Register &rs, int32_t imm)
    {
        masm.addiu(rt.code(), rs.code(), imm);
    }

    void addu(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.addu(rd.code(), rs.code(), rt.code());
    }

    //by wangqing, overloaded addu
    void addu(const Register &rd, const Register &rs, const mRegisterID rt)
    {
        masm.addu(rd.code(), rs.code(), rt);
    }


    void subu(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.subu(rd.code(), rs.code(), rt.code());
    }

    void mult(const Register &rs, const Register &rt)
    {
        masm.mult(rs.code(), rt.code());
    }

    void div(const Register &rs, const Register &rt)
    {
        masm.div(rs.code(), rt.code());
    }

    void divu(const Register &rs, const Register &rt)
    {
        masm.divu(rs.code(), rt.code());
    }

    void mfhi(const Register &rd)
    {
        masm.mfhi(rd.code());
    }

    void mflo(const Register &rd)
    {
        masm.mflo(rd.code());
    }

    void mul(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.mul(rd.code(), rs.code(), rt.code());
    }

    void andInsn(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.andInsn(rd.code(), rs.code(), rt.code());
    }

    void andi(const Register &rt, const Register &rs, ImmWord imm)
    {
        masm.andi(rt.code(), rs.code(), imm.value);
    }

    void nor(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.nor(rd.code(), rs.code(), rt.code());
    }

    void orInsn(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.orInsn(rd.code(), rs.code(), rt.code());
    }

    void ori(const Register &rt, const Register &rs, ImmWord imm)
    {
        masm.ori(rt.code(), rs.code(), imm.value);
    }

    // by wangqing ,overloaded ori
    void ori(const Register &rt, const Register &rs, int32_t imm)
    {
        masm.ori(rt.code(), rs.code(), imm);
    }

    // by wangqing ,overloaded ori
    void ori(const mRegisterID rt, const mRegisterID rs, int32_t imm)
    {
        masm.ori(rt, rs, imm);
    }

    void xorInsn(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.xorInsn(rd.code(), rs.code(), rt.code());
    }

    void xori(const Register &rt, const Register &rs, ImmWord imm)
    {
        masm.xori(rt.code(), rs.code(), imm.value);
    }

    void slt(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.slt(rd.code(), rs.code(), rt.code());
    }

    void sltu(const Register &rd, const Register &rs, const Register &rt)
    {
        masm.sltu(rd.code(), rs.code(), rt.code());
    }

    void sltiu(const Register &rt, const Register &rs, ImmWord imm)
    {
        masm.sltiu(rt.code(), rs.code(), imm.value);
    }

    void sll(const Register &rd, const Register &rt, ImmWord shamt)
    {
        masm.sll(rd.code(), rt.code(), shamt.value);
    }

    // by wangqing, overloaded sll
    void sll(const Register &rd, const mRegisterID rt, int32_t shamt)
    {
        masm.sll(rd.code(), rt, shamt);
    }
    
    // by wangqing, overloaded sll
    void sll(const Register &rd, const Register &rt, int32_t shamt)
    {
        masm.sll(rd.code(), rt.code(), shamt);
    }

    void sllv(const Register &rd, const Register &rt, const Register &rs)
    {
        masm.sllv(rd.code(), rt.code(), rs.code());
    }

    void sra(const Register &rd, const Register &rt, ImmWord shamt)
    {
        masm.sra(rd.code(), rt.code(), shamt.value);
    }

    void srav(const Register &rd, const Register &rt, const Register &rs)
    {
        masm.srav(rd.code(), rt.code(), rs.code());
    }

    void srl(const Register &rd, const Register &rt, ImmWord shamt)
    {
        masm.srl(rd.code(), rt.code(), shamt.value);
    }

    void srlv(const Register &rd, const Register &rt, const Register &rs)
    {
        masm.srlv(rd.code(), rt.code(), rs.code());
    }

    void lb(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.lb(rt.code(), rs.code(), offset.value);
    }

    // by wangqing, overloaded lb
    void lb(const Register &rt, const Register &rs, int32_t offset)
    {
        masm.lb(rt.code(), rs.code(), offset);
    }

    void lbu(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.lbu(rt.code(), rs.code(), offset.value);
    }
    
    // by wangqing, overloaded lbu
    void lbu(const Register &rt, const Register &rs, int32_t offset)
    {
        masm.lbu(rt.code(), rs.code(), offset);
    }

    void lw(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.lw(rt.code(), rs.code(), offset.value);
    }
    
    // by wangqing, overloaded lw
    void lw(const Register &rt, const Register &rs, int32_t offset)
    {
        masm.lw(rt.code(), rs.code(), offset);
    }

    void lwl(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.lwl(rt.code(), rs.code(), offset.value);
    }

    void lwr(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.lwr(rt.code(), rs.code(), offset.value);
    }

    void lh(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.lh(rt.code(), rs.code(), offset.value);
    }

    // by wangqing, overloaded lh
    void lh(const Register &rt, const Register &rs, int32_t offset)
    {
        masm.lh(rt.code(), rs.code(), offset);
    }

    void lhu(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.lhu(rt.code(), rs.code(), offset.value);
    }

    // by wangqing, overloaded lhu
    void lhu(const Register &rt, const Register &rs, int32_t offset)
    {
        masm.lhu(rt.code(), rs.code(), offset);
    }

    void sb(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.sb(rt.code(), rs.code(), offset.value);
    }

    // by wangqing, overloaded sb
    void sb(const Register &rt, const Register &rs, int32_t offset)
    {
        masm.sb(rt.code(), rs.code(), offset);
    }

    void sh(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.sh(rt.code(), rs.code(), offset.value);
    }

    // by wangqing, overloaded sh
    void sh(const Register &rt, const Register &rs, int32_t offset)
    {
        masm.sh(rt.code(), rs.code(), offset);
    }

    void sw(const Register &rt, const Register &rs, ImmWord offset)
    {
        masm.sw(rt.code(), rs.code(), offset.value);
    }

    // by wangqing, overloaded sw
    void sw(const Register &rt, const Register &rs, int32_t offset)
    {
        masm.sw(rt.code(), rs.code(), offset);
    }

    void jr(const Register &rs)
    {
        masm.jr(rs.code());
    }

    void jalr(const Register &rs)
    {
        masm.jalr(rs.code());
    }

    void jal()
    {
        masm.jal();
    }

    void bkpt()
    {
        masm.bkpt();
    }

    void bal(ImmWord imm)
    {
        masm.bal(imm.value);
    }

    void bgez(const Register &rs, ImmWord imm)
    {
        masm.bgez(rs.code(), imm.value);
    }

    // by wangqing, overloaded bgez
    void bgez(const Register &rs, int32_t imm)
    {
        masm.bgez(rs.code(), imm);
    }

    void bltz(const Register &rs, ImmWord imm)
    {
        masm.bltz(rs.code(), imm.value);
    }

    void beq(const Register &rs, const Register &rt, ImmWord imm)
    {
        masm.beq(rs.code(), rt.code(), imm.value);
    }

    // by wangqing, overloaded beq
    void beq(const Register &rs, const Register &rt, int32_t imm)
    {
        masm.beq(rs.code(), rt.code(), imm);
    }

    void bne(const Register &rs, const Register &rt, ImmWord imm)
    {
        masm.bne(rs.code(), rt.code(), imm.value);
    }

    // by wangqing, overloaded bne
    void bne(const Register &rs, const Register &rt, int32_t imm)
    {
        masm.bne(rs.code(), rt.code(), imm);
    }

    void bc1t()
    {
        masm.bc1t();
    }

    void bc1f()
    {
        masm.bc1f();
    }

    // by wangqing 2010-10-30
    JmpSrc newJmpSrc()
    {
        JSC::AssemblerBuffer m_buffer;
        return JSC::MIPSAssembler::JmpSrc(m_buffer.size());
    }

    void appendJump()
    {
        masm.appendJump();
    }

    void movd(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.movd(fd.code(), fs.code());
    }

    void addd(const FloatRegister &fd, const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.addd(fd.code(), fs.code(), ft.code());
    }

    // add by wangqing
    void adds(const FloatRegister &fd, const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.adds(fd.code(), fs.code(), ft.code());
    }

    void subd(const FloatRegister &fd, const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.subd(fd.code(), fs.code(), ft.code());
    }

    // add by wangqing
    void subs(const FloatRegister &fd, const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.subs(fd.code(), fs.code(), ft.code());
    }

    void muld(const FloatRegister &fd, const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.muld(fd.code(), fs.code(), ft.code());
    }

    // add by wangqing
    void muls(const FloatRegister &fd, const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.muls(fd.code(), fs.code(), ft.code());
    }

    void divd(const FloatRegister &fd, const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.divd(fd.code(), fs.code(), ft.code());
    }

    // add by wangqing
    void divs(const FloatRegister &fd, const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.divs(fd.code(), fs.code(), ft.code());
    }

    // add by wangqing
    void negd(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.negd(fd.code(), fs.code());
    }
    
    // add by wangqing
    void negs(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.negs(fd.code(), fs.code());
    }

    // add by wangqing
    void absd(const FloatRegister &fd, const FloatRegister &fs)               
    {   
        masm.absd(fd.code(), fs.code());                                      
    }

    void lwc1(const FloatRegister &ft, const Register &rs, ImmWord offset)
    {
        masm.lwc1(ft.code(), rs.code(), offset.value);
    }

    // by wangqing, overloaded lwc1
    void lwc1(const FloatRegister &ft, const Register &rs, int32_t offset)
    {
        masm.lwc1(ft.code(), rs.code(), offset);
    }

    // by wangqing, overloaded lwc1
    void lwc1(const mFPRegisterID ft, const Register &rs, int32_t offset)
    {
        masm.lwc1(ft, rs.code(), offset);
    }

    void ldc1(const FloatRegister &ft, const Register &rs, ImmWord offset)
    {
        masm.ldc1(ft.code(), rs.code(), offset.value);
    }

    void swc1(const FloatRegister &ft, const Register &rs, ImmWord offset)
    {
        masm.swc1(ft.code(), rs.code(), offset.value);
    }

    // by wangqing, overloaded swcl
    void swc1(const FloatRegister &ft, const Register &rs, int32_t offset)
    {
        masm.swc1(ft.code(), rs.code(), offset);
    }

    // by wangqing, overloaded swcl
    void swc1(const mFPRegisterID ft, const Register &rs, int32_t offset)
    {
        masm.swc1(ft, rs.code(), offset);
    }

    void sdc1(const FloatRegister &ft, const Register &rs, ImmWord offset)
    {
        masm.sdc1(ft.code(), rs.code(), offset.value);
    }

    void mtc1(const Register &rt, const FloatRegister &fs)
    {
        masm.mtc1(rt.code(), fs.code());
    }

    void mthc1(const Register &rt, const FloatRegister &fs)
    {
        masm.mthc1(rt.code(), fs.code());
    }

    void dsrl32(const Register &rt, const Register &rd, ImmWord saminus32)
    {
        masm.dsrl32(rt.code(), rd.code(), saminus32.value);
    }

    // by wangqing, overload dsrl32
    void dsrl32(const Register &rt, const Register &rd, int32_t saminus32)
    {
        masm.dsrl32(rt.code(), rd.code(), saminus32);
    }

    void dmfc1(const Register &rt, const FloatRegister &fs)
    {
        masm.dmfc1(rt.code(), fs.code());
    }

    // by wangqing, overloaded dmfc1
    void dmfc1(const mRegisterID rt, const mFPRegisterID fs)
    {
        masm.dmfc1(rt, fs);
    }

    void mfc1(const Register &rt, const FloatRegister &fs)
    {
        masm.mfc1(rt.code(), fs.code());
    }

    void sqrtd(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.sqrtd(fd.code(), fs.code());
    }

    // add by wangqing
    void sqrts(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.sqrts(fd.code(), fs.code());
    }

    void truncwd(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.truncwd(fd.code(), fs.code());
    }

    // add by wangqing
    void truncws(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.truncws(fd.code(), fs.code());
    }

    void floorwd(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.floorwd(fd.code(), fs.code());
    }

    // add by wangqing
    void floorws(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.floorws(fd.code(), fs.code());
    }

    void cvtdw(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.cvtdw(fd.code(), fs.code());
    }

    // add by wangqing
    void cvtsw(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.cvtsw(fd.code(), fs.code());
    }

    void cvtds(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.cvtds(fd.code(), fs.code());
    }

    void cvtsd(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.cvtsd(fd.code(), fs.code());
    }

    void cvtwd(const FloatRegister &fd, const FloatRegister &fs)
    {
        masm.cvtwd(fd.code(), fs.code());
    }

    void cud(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.cud(fs.code(), ft.code());
    }

    void ceqd(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.ceqd(fs.code(), ft.code());
    }

    void cseqd(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.cseqd(fs.code(), ft.code());
    }

    void cngtd(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.cngtd(fs.code(), ft.code());
    }

    void cnged(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.cnged(fs.code(), ft.code());
    }

    void cltd(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.cltd(fs.code(), ft.code());
    }

    void cled(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.cled(fs.code(), ft.code());
    }

    void cueqd(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.cueqd(fs.code(), ft.code());
    }

    void coled(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.coled(fs.code(), ft.code());
    }

    void coltd(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.coltd(fs.code(), ft.code());
    }

    void culed(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.culed(fs.code(), ft.code());
    }

    void cultd(const FloatRegister &fs, const FloatRegister &ft)
    {
        masm.cultd(fs.code(), ft.code());
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

#endif /* jit_mips_Assembler_mips_h */
