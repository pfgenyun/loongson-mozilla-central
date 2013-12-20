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

//#include "jsscriptinlines.h"

// From jit/x86/Assembler-x86.h
namespace js {
namespace jit {

static MOZ_CONSTEXPR_VAR Register zero= { JSC::MIPSRegisters::zero };
static MOZ_CONSTEXPR_VAR Register at = { JSC::MIPSRegisters::at };
static MOZ_CONSTEXPR_VAR Register v0 = { JSC::MIPSRegisters::v0 };
static MOZ_CONSTEXPR_VAR Register v1 = { JSC::MIPSRegisters::v1 };
static MOZ_CONSTEXPR_VAR Register a0 = { JSC::MIPSRegisters::a0 };
static MOZ_CONSTEXPR_VAR Register a1 = { JSC::MIPSRegisters::a1 };
static MOZ_CONSTEXPR_VAR Register a2 = { JSC::MIPSRegisters::a2 };
static MOZ_CONSTEXPR_VAR Register a3 = { JSC::MIPSRegisters::a3 };
static MOZ_CONSTEXPR_VAR Register t0 = { JSC::MIPSRegisters::t0 };
static MOZ_CONSTEXPR_VAR Register t1 = { JSC::MIPSRegisters::t1 };
static MOZ_CONSTEXPR_VAR Register t2 = { JSC::MIPSRegisters::t2 };
static MOZ_CONSTEXPR_VAR Register t3 = { JSC::MIPSRegisters::t3 };
static MOZ_CONSTEXPR_VAR Register t4 = { JSC::MIPSRegisters::t4 };
static MOZ_CONSTEXPR_VAR Register t5 = { JSC::MIPSRegisters::t5 };
static MOZ_CONSTEXPR_VAR Register t6 = { JSC::MIPSRegisters::t6 };
static MOZ_CONSTEXPR_VAR Register t7 = { JSC::MIPSRegisters::t7 };
static MOZ_CONSTEXPR_VAR Register t8 = { JSC::MIPSRegisters::t8 };
static MOZ_CONSTEXPR_VAR Register t9 = { JSC::MIPSRegisters::t9 };
static MOZ_CONSTEXPR_VAR Register s0 = { JSC::MIPSRegisters::s0 };
static MOZ_CONSTEXPR_VAR Register s1 = { JSC::MIPSRegisters::s1 };
static MOZ_CONSTEXPR_VAR Register s2 = { JSC::MIPSRegisters::s2 };
static MOZ_CONSTEXPR_VAR Register s3 = { JSC::MIPSRegisters::s3 };
static MOZ_CONSTEXPR_VAR Register s4 = { JSC::MIPSRegisters::s4 };
static MOZ_CONSTEXPR_VAR Register s5 = { JSC::MIPSRegisters::s5 };
static MOZ_CONSTEXPR_VAR Register s6 = { JSC::MIPSRegisters::s6 };
static MOZ_CONSTEXPR_VAR Register s7 = { JSC::MIPSRegisters::s7 };
static MOZ_CONSTEXPR_VAR Register k0 = { JSC::MIPSRegisters::k0 };
static MOZ_CONSTEXPR_VAR Register k1 = { JSC::MIPSRegisters::k1 };
static MOZ_CONSTEXPR_VAR Register gp = { JSC::MIPSRegisters::gp };
static MOZ_CONSTEXPR_VAR Register sp = { JSC::MIPSRegisters::sp };
static MOZ_CONSTEXPR_VAR Register fp = { JSC::MIPSRegisters::fp };
static MOZ_CONSTEXPR_VAR Register ra = { JSC::MIPSRegisters::ra };


static MOZ_CONSTEXPR_VAR FloatRegister f0 = { JSC::MIPSRegisters::f0 };
static MOZ_CONSTEXPR_VAR FloatRegister f1 = { JSC::MIPSRegisters::f1 };
static MOZ_CONSTEXPR_VAR FloatRegister f2 = { JSC::MIPSRegisters::f2 };
static MOZ_CONSTEXPR_VAR FloatRegister f3 = { JSC::MIPSRegisters::f3 };
static MOZ_CONSTEXPR_VAR FloatRegister f4 = { JSC::MIPSRegisters::f4 };
static MOZ_CONSTEXPR_VAR FloatRegister f5 = { JSC::MIPSRegisters::f5 };
static MOZ_CONSTEXPR_VAR FloatRegister f6 = { JSC::MIPSRegisters::f6 };
static MOZ_CONSTEXPR_VAR FloatRegister f7 = { JSC::MIPSRegisters::f7 };
static MOZ_CONSTEXPR_VAR FloatRegister f8 = { JSC::MIPSRegisters::f8 };
static MOZ_CONSTEXPR_VAR FloatRegister f9 = { JSC::MIPSRegisters::f9 };
static MOZ_CONSTEXPR_VAR FloatRegister f10 = { JSC::MIPSRegisters::f10 };
static MOZ_CONSTEXPR_VAR FloatRegister f11 = { JSC::MIPSRegisters::f11 };
static MOZ_CONSTEXPR_VAR FloatRegister f12 = { JSC::MIPSRegisters::f12 };
static MOZ_CONSTEXPR_VAR FloatRegister f13 = { JSC::MIPSRegisters::f13 };
static MOZ_CONSTEXPR_VAR FloatRegister f14 = { JSC::MIPSRegisters::f14 };
static MOZ_CONSTEXPR_VAR FloatRegister f15 = { JSC::MIPSRegisters::f15 };
static MOZ_CONSTEXPR_VAR FloatRegister f16 = { JSC::MIPSRegisters::f16 };
static MOZ_CONSTEXPR_VAR FloatRegister f17 = { JSC::MIPSRegisters::f17 };
static MOZ_CONSTEXPR_VAR FloatRegister f18 = { JSC::MIPSRegisters::f18 };
static MOZ_CONSTEXPR_VAR FloatRegister f19 = { JSC::MIPSRegisters::f19 };
static MOZ_CONSTEXPR_VAR FloatRegister f20 = { JSC::MIPSRegisters::f20 };
static MOZ_CONSTEXPR_VAR FloatRegister f21 = { JSC::MIPSRegisters::f21 };
static MOZ_CONSTEXPR_VAR FloatRegister f22 = { JSC::MIPSRegisters::f22 };
static MOZ_CONSTEXPR_VAR FloatRegister f23 = { JSC::MIPSRegisters::f23 };
static MOZ_CONSTEXPR_VAR FloatRegister f24 = { JSC::MIPSRegisters::f24 };
static MOZ_CONSTEXPR_VAR FloatRegister f25 = { JSC::MIPSRegisters::f25 };
static MOZ_CONSTEXPR_VAR FloatRegister f26 = { JSC::MIPSRegisters::f26 };
static MOZ_CONSTEXPR_VAR FloatRegister f27 = { JSC::MIPSRegisters::f27 };
static MOZ_CONSTEXPR_VAR FloatRegister f28 = { JSC::MIPSRegisters::f28 };
static MOZ_CONSTEXPR_VAR FloatRegister f29 = { JSC::MIPSRegisters::f29 };
static MOZ_CONSTEXPR_VAR FloatRegister f30 = { JSC::MIPSRegisters::f30 };
static MOZ_CONSTEXPR_VAR FloatRegister f31 = { JSC::MIPSRegisters::f31 };


static MOZ_CONSTEXPR_VAR Register InvalidReg = { JSC::MIPSRegisters::invalid_reg };
static MOZ_CONSTEXPR_VAR FloatRegister InvalidFloatReg = { JSC::MIPSRegisters::invalid_freg };

static MOZ_CONSTEXPR_VAR Register JSReturnReg_Type = t7;
static MOZ_CONSTEXPR_VAR Register JSReturnReg_Data = t8;
static MOZ_CONSTEXPR_VAR Register StackPointer = sp;
static MOZ_CONSTEXPR_VAR Register FramePointer=InvalidReg;
static MOZ_CONSTEXPR_VAR Register ReturnReg = v0;
static MOZ_CONSTEXPR_VAR FloatRegister ReturnFloatReg = {JSC::MIPSRegisters::f0};
static MOZ_CONSTEXPR_VAR FloatRegister ScratchFloatReg = {JSC::MIPSRegisters::f2};
static MOZ_CONSTEXPR_VAR Register ArgumentsRectifierReg = s0;//esi;
static MOZ_CONSTEXPR_VAR Register CallTempReg0 = s1;//edi;
static MOZ_CONSTEXPR_VAR Register CallTempReg1 = s2;//eax;
static MOZ_CONSTEXPR_VAR Register CallTempReg2 = s3;//ebx;
static MOZ_CONSTEXPR_VAR Register CallTempReg3 = s4;//ecx;
static MOZ_CONSTEXPR_VAR Register CallTempReg4 = s5;//esi;
static MOZ_CONSTEXPR_VAR Register CallTempReg5 = s6;//edx;
static MOZ_CONSTEXPR_VAR Register CallTempReg6 = s7;//ebp;

//static MOZ_CONSTEXPR_VAR Register immTempRegister  = t0;
static MOZ_CONSTEXPR_VAR Register dataTempRegister = t1;
static MOZ_CONSTEXPR_VAR Register addrTempRegister = t2;
static MOZ_CONSTEXPR_VAR Register cmpTempRegister  = t3;
static MOZ_CONSTEXPR_VAR Register cmpTemp2Register  = t5;
//static MOZ_CONSTEXPR_VAR Register dataTemp2Register = t4;


static MOZ_CONSTEXPR_VAR FloatRegister fpTempRegister = f28;
static MOZ_CONSTEXPR_VAR FloatRegister fpTemp2Register = f30;

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

static MOZ_CONSTEXPR_VAR Register OsrFrameReg = s6;  //edx;
static MOZ_CONSTEXPR_VAR Register PreBarrierReg =s6;  // edx;

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

//#include "jit/shared/Assembler-x86-shared.h"
static inline void
PatchJump(CodeLocationJump jump, CodeLocationLabel label)
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

// From jit/shared/Assembler-x86-shared.h
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
    int32_t base_ : 5;
    Scale scale_ : 3;
    int32_t index_ : 5;
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
    // Following is from jit/shared/Assembler-x86-shared.h
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
  	
//   using AssemblerMIPSShared::movl;
//   using AssemblerMIPSShared::j;
//   using AssemblerMIPSShared::jmp;
//   using AssemblerMIPSShared::movsd;
//   using AssemblerMIPSShared::retarget;
//   using AssemblerMIPSShared::cmpl;
//   using AssemblerMIPSShared::call;
//   using AssemblerMIPSShared::push;

    // Following is from jit/x86/Assembler-X86.h

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
    void push(const ImmPtr imm) {
        push(ImmWord(uintptr_t(imm.value)));
    }
    void push(const FloatRegister &src) {
        subl(Imm32(sizeof(double)), StackPointer);
        movsd(src, Address(StackPointer, 0));
    }

    CodeOffsetLabel pushWithPatch(const ImmWord &word) {
        push(Imm32(word.value));
        return masm.currentOffset();
    }

    void pop(const FloatRegister &src) {
        movsd(Address(StackPointer, 0), src);
        addl(Imm32(sizeof(double)), StackPointer);
    }

    CodeOffsetLabel movWithPatch(const ImmWord &word, const Register &dest) {
        movl(Imm32(word.value), dest);
        return masm.currentOffset();
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
          case Operand::MEM_REG_DISP:
//ok            masm.movl_i32m(ptr.value, dest.disp(), dest.base());
            mcss.store32(mTrustedImm32(ptr.value), mImplicitAddress(mAddress(dest.base(), dest.disp())));
            writeDataRelocation(ptr);
            break;
          case Operand::MEM_SCALE:
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
    void xchg(const Register &src, const Register &dest) {
        xchgl(src, dest);
    }
    void lea(const Operand &src, const Register &dest) {
        return leal(src, dest);
    }

    // New function
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
        movl(ptr,cmpTemp2Register);
        writeDataRelocation(ptr);
    }
    void cmpl(const Register &lhs, const Register &rhs) {
        movl(lhs,cmpTempRegister);
        movl(rhs,cmpTemp2Register);
    }
    void cmpl(const Operand &op, ImmGCPtr imm) {
        movl(op,cmpTempRegister);
        movl(imm,cmpTemp2Register);
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
        mcss.move(lhs.code(),cmpTempRegister.code());
        mcss.move(mTrustedImm32(rhs.value),cmpTemp2Register.code());
        return masm.currentOffset();
    }

    void jmp(ImmPtr target, Relocation::Kind reloc = Relocation::HARDCODED) {
     //  JmpSrc src = masm.jmp();
        JmpSrc src = mcss.jump().m_jmp;
        addPendingJump(src, target, reloc);
    }
    void j(Condition cond, ImmPtr target,
           Relocation::Kind reloc = Relocation::HARDCODED) {
    //    JmpSrc src = masm.jCC(static_cast<JSC::MIPSAssembler::Condition>(cond));
        JmpSrc src = mcss.branch32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond), cmpTempRegister.code(), cmpTemp2Register.code()).m_jmp;
        addPendingJump(src, target, reloc);
    }

    void jmp(IonCode *target) {
        jmp(ImmPtr(target->raw()), Relocation::IONCODE);
    }
    void j(Condition cond, IonCode *target) {
        j(cond, ImmPtr(target->raw()), Relocation::IONCODE);
    }
    void call(IonCode *target) {
  //      JmpSrc src = masm.call();
        mcss.offsetFromPCToV0(sizeof(int*)*7);
        mcss.push(mRegisterID(v0.code()));//2insns
        JmpSrc src = mcss.call().m_jmp;//4insns
        addPendingJump(src, ImmPtr(target->raw()), Relocation::IONCODE);
    }
    
    void call(ImmWord target) {
//ok        JmpSrc src = masm.call();
    //arm : ma_call((void *) word.value);
//    mcss.offsetFromPCToV0(sizeof(int*)*7);//2insns
//    mcss.push(mRegisterID(v0.code()));//2insns
////        JmpSrc src = mcss.call().m_jmp;
////        addPendingJump(src, target.asPointer(), Relocation::HARDCODED);
        call(ImmPtr((void*)target.value));
    }

    void call(ImmPtr target) {
//        JmpSrc src = masm.call();
//        addPendingJump(src, target, Relocation::HARDCODED);
        JmpSrc src = mcss.call().m_jmp;
        addPendingJump(src, target, Relocation::HARDCODED);
    }

    // New function
    void call(AsmJSImmPtr target) {
        JS_ASSERT(0);
        // Moving to a register is suboptimal. To fix (use a single
        // call-immediate instruction) we'll need to distinguish a new type of
        // relative patch to an absolute address in AsmJSAbsoluteLink.
//        mov(target, eax);
//        call(eax);
    }

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

    static size_t ToggledCallSize() {
        // Size of a call instruction.
        // return 5;
    	return 32;
    }

    // Re-routes pending jumps to an external target, flushing the label in the
    // process.
    void retarget(Label *label, ImmPtr target, Relocation::Kind reloc) {
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
    CodeOffsetLabel movsblWithPatch(Address src, Register dest) {
      //   masm.movxbl_mr_disp32(src.offset, src.base.code(), dest.code());//movxbl in x86
        movsbl(Operand(src),dest);
        return masm.currentOffset();
    }
    CodeOffsetLabel movzblWithPatch(Address src, Register dest) {
  //      masm.movzbl_mr_disp32(src.offset, src.base.code(), dest.code());
        movzbl(Operand(src),dest);
        return masm.currentOffset();
    }
    CodeOffsetLabel movswlWithPatch(Address src, Register dest) {
        //  masm.movxwl_mr_disp32(src.offset, src.base.code(), dest.code());
        movswl(Operand(src),dest);
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
    CodeOffsetLabel movlWithPatch(PatchedAbsoluteAddress addr, Register index, Scale scale,
                                  Register dest)
    {
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
    void writeCodePointer(AbsoluteLabel *label) {
      	ASSERT(0);
   /*     JS_ASSERT(!label->bound());
        // Thread the patch list through the unpatched address word in the
        // instruction stream.
        masm.jumpTablePointer(label->prev());
        label->setPrev(masm.size());
        */
    }
    void writeDoubleConstant(double d, Label *label) {
        ASSERT(0);
    /*    label->bind(masm.size());
        masm.doubleConstant(d);
    */
    }
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
    
    // New function
    void xchgl(const Register &src, const Register &dest) {
        JS_ASSERT(0);
        //masm.xchgl_rr(src.code(), dest.code());
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
        JS_ASSERT(0);
        //masm.movsd_mr(src.offset, src.base.code(), dest.code());
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
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void movsd(const BaseIndex &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //masm.movsd_mr(src.offset, src.base.code(), src.index.code(), src.scale, dest.code());
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
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void movsd(const FloatRegister &src, const Address &dest) {
        JS_ASSERT(0);
        //masm.movsd_rm(src.code(), dest.offset, dest.base.code());
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
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void movsd(const FloatRegister &src, const BaseIndex &dest) {
        JS_ASSERT(0);
        //masm.movsd_rm(src.code(), dest.offset, dest.base.code(), dest.index.code(), dest.scale);
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
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void movss(const Address &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //masm.movss_mr(src.offset, src.base.code(), dest.code());
    }
    // New function
    void movss(const BaseIndex &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //masm.movss_mr(src.offset, src.base.code(), src.index.code(), src.scale, dest.code());
    }
    // New function
    void movss(const FloatRegister &src, const Address &dest) {
        JS_ASSERT(0);
        //masm.movss_rm(src.code(), dest.offset, dest.base.code());
    }
    // New function
    void movss(const FloatRegister &src, const BaseIndex &dest) {
        JS_ASSERT(0);
        //masm.movss_rm(src.code(), dest.offset, dest.base.code(), dest.index.code(), dest.scale);
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
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
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
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
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
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
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
          case Operand::MEM_REG_DISP:
            // New branch,
            JS_ASSERT(0);
            //masm.jmp_m(op.disp(), op.base());
            break;
          case Operand::MEM_SCALE:
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
    void setCC(Condition cond, const Register &r) {
    //    masm.setCC_r(static_cast<JSC::X86Assembler::Condition>(cond), r.code());
    }
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
        masm.div(t6.code(), divisor.code());
        masm.mflo(divisor.code());
    }
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
    // New function
    void cvttss2si(const FloatRegister &src, const Register &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        masm.cvttss2si_rr(src.code(), dest.code());
    }
    // New function
    void cvtsi2ss(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::REG:
            //masm.cvtsi2ss_rr(src.reg(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.cvtsi2ss_mr(src.disp(), src.base(), dest.code());
            break;
          case Operand::MEM_SCALE:
            //masm.cvtsi2ss_mr(src.disp(), src.base(), src.index(), src.scale(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void cvtsi2ss(const Register &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.cvtsi2ss_rr(src.code(), dest.code());
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
    // New function
    void movmskps(const FloatRegister &src, const Register &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        masm.movmskps_rr(src.code(), dest.code());
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
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        masm.ucomiss_rr(rhs.code(), lhs.code());
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
    // New function
    void addss(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
//        masm.addss_rr(src.code(), dest.code());
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
    // New function
    void addss(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
            //masm.addss_rr(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.addss_mr(src.disp(), src.base(), dest.code());
            break;
          case Operand::MEM_ADDRESS32:
            //masm.addss_mr(src.address(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    void subsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.subsd_rr(src.code(), dest.code());
        mcss.subDouble(src.code(), dest.code());
    }
    // New function
    void subss(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.subss_rr(src.code(), dest.code());
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
    // New function
    void subss(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
            //masm.subss_rr(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.subss_mr(src.disp(), src.base(), dest.code());
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
    // New function
    void mulss(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
            //masm.mulss_rr(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.mulss_mr(src.disp(), src.base(), dest.code());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void mulss(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        //masm.mulss_rr(src.code(), dest.code());
    }
    void divsd(const FloatRegister &src, const FloatRegister &dest) {
//ok        masm.divsd_rr(src.code(), dest.code());
        mcss.divDouble(src.code(), dest.code());
    }
    // New function
    void divss(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        //masm.divss_rr(src.code(), dest.code());
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
    // New function
    void divss(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE2());
        switch (src.kind()) {
          case Operand::FPREG:
            //masm.divss_rr(src.fpu(), dest.code());
            break;
          case Operand::MEM_REG_DISP:
            //masm.divss_mr(src.disp(), src.base(), dest.code());
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
    // New function
    void xorps(const FloatRegister &src, const FloatRegister &dest) {
        //JS_ASSERT(HasSSE2());
        //masm.xorps_rr(src.code(), dest.code());
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
        JS_ASSERT(0);
    //    JS_ASSERT(HasSSE2());
    //    masm.sqrtss_rr(src.code(), dest.code());
    }
    // New function
    void roundsd(const FloatRegister &src, const FloatRegister &dest)
                 //JSC::X86Assembler::RoundingMode mode)
    {
        JS_ASSERT(0);
    //    JS_ASSERT(HasSSE41());
    //    masm.roundsd_rr(src.code(), dest.code(), mode);
     mcss.floorDouble(src.code(), dest.code());
    }
    // New function
    void roundss(const FloatRegister &src, const FloatRegister &dest)
                 //JSC::X86Assembler::RoundingMode mode)
    {
        JS_ASSERT(0);
//        JS_ASSERT(HasSSE41());
//        masm.roundss_rr(src.code(), dest.code(), mode);
    }
    // New function
    void minsd(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.minsd_rr(src.code(), dest.code());
    }
    // New function
    void minsd(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //switch (src.kind()) {
        //  case Operand::FPREG:
        //    masm.minsd_rr(src.fpu(), dest.code());
        //    break;
        //  case Operand::MEM_REG_DISP:
        //    masm.minsd_mr(src.disp(), src.base(), dest.code());
        //    break;
        //  default:
        //    MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        //}
    }
    // New function
    void maxsd(const FloatRegister &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //masm.maxsd_rr(src.code(), dest.code());
    }
    // New function
    void maxsd(const Operand &src, const FloatRegister &dest) {
        JS_ASSERT(0);
        //JS_ASSERT(HasSSE2());
        //switch (src.kind()) {
        //  case Operand::FPREG:
        //    masm.maxsd_rr(src.fpu(), dest.code());
        //    break;
        //  case Operand::MEM_REG_DISP:
        //    masm.maxsd_mr(src.disp(), src.base(), dest.code());
        //    break;
        //  default:
        //    MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        //}
    }
    // New function
    void fisttp(const Operand &dest) {
        ASSERT(0);
        //JS_ASSERT(HasSSE3());
        //switch (dest.kind()) {
        //  case Operand::MEM_REG_DISP:
        //    //masm.fisttp_m(dest.disp(), dest.base());
        //    break;
        //  default:
        //    MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        //}
    }
    // New function
    void fld(const Operand &dest) {
        ASSERT(0);
        switch (dest.kind()) {
          case Operand::MEM_REG_DISP:
            //masm.fld_m(dest.disp(), dest.base());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
    }
    // New function
    void fstp(const Operand &src) {
        ASSERT(0);
        switch (src.kind()) {
          case Operand::MEM_REG_DISP:
            //masm.fstp_m(src.disp(), src.base());
            break;
          default:
            MOZ_ASSUME_UNREACHABLE("unexpected operand kind");
        }
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
         JSC::MIPSAssembler::setInt32((int32_t *) dataLabel.raw(), toWrite.value);
    }

    static void patchDataWithValueCheck(CodeLocationLabel data, PatchedImmPtr newData,
                                        PatchedImmPtr expectedData) {
//TBDok
        // The pointer given is a pointer to *after* the data.
//        uintptr_t *ptr = ((uintptr_t *) data.raw()) - 1;
//        JS_ASSERT(*ptr == expectedData.value);
//        *ptr = newData.value;
        uint32_t old = JSC::MIPSAssembler::getInt32(data.raw());
        JS_ASSERT(old == (uint32_t)expectedData.value);
        JSC::MIPSAssembler::setInt32(((uint8_t *)data.raw()), *(int32_t *)(newData.value));
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

#endif /* jit_mips_Assembler_mips_h */
