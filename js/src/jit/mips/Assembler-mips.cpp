#include "jsutil.h"
#include "assembler/jit/ExecutableAllocator.h"
#include "jscompartment.h"
#include "jit/IonCompartment.h"

#include "Assembler-mips.h"
#include "gc/Marking.h"
using namespace js;
using namespace js::jit;

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
      case MIRType_Double:
        stackOffset_ += sizeof(uint64_t);
        break;
      default:
        JS_NOT_REACHED("Unexpected argument type");
    }
    return current_;
}

const Register ABIArgGenerator::NonArgReturnVolatileReg0 = s4;
const Register ABIArgGenerator::NonArgReturnVolatileReg1 = s6;
const Register ABIArgGenerator::NonVolatileReg =s3;
	
	
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
Assembler::absd(const FloatRegister &src) {
    //ok, by weizhenwei, 2013.10.20
    mcss.absDouble(mFPRegisterID(src.code()), mFPRegisterID(src.code()));
}
void
Assembler::zerod(const FloatRegister &src) {
    //ok, by weizhenwei, 2013.10.20
    mcss.zeroDouble(mFPRegisterID(src.code()));
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
        JS_NOT_REACHED("unexpected condition");
        return Equal;
    }
}
//edit by Quqiuwen,do not support PF
void Assembler::setCC(Condition cond,const Register &r)
{
    mcss.set32(static_cast<JSC::MacroAssemblerMIPS::Condition>(cond),cmpTempRegister.code(),cmpTemp2Register.code(),r.code());
}

//hwj 1028
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
        ::TraceDataRelocations(trc, masm.buffer(), reader);
    }
}
void
Assembler::processCodeLabels(uint8_t *rawCode)
{
    for (size_t i = 0; i < codeLabels_.length(); i++) {
        CodeLabel label = codeLabels_[i];
        Bind(rawCode, label.dest(), rawCode + label.src()->offset());
    }
}

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

//hwj 1028
static inline IonCode *
CodeFromJump(uint8_t *jump)
{
    int luiIns = *((int*)(jump-8));
    int oriIns = *((int*)(jump-4));
    int temp = (luiIns & 0x0000ffff) << 16;
    temp = temp | (oriIns&0x0000ffff);
    uint8_t *target = (uint8_t*)temp;

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

//hwj 1028
void
Assembler::executableCopy(uint8_t *buffer)
{
    masm.executableCopy(buffer);
}

void
Assembler::retn(Imm32 n) {
    // Remove the size of the return address which is included in the frame.
    pop(ra);
    mcss.ret((n.value - sizeof(void *)));
}

//hwj
void 
Assembler::call(Label *label) {
    mcss.offsetFromPCToV0(sizeof(int*)*9);//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    jmp(label);//6insns
}
//hwj
void 
Assembler::call(const Register &reg) {
	if(reg != t9)
	{
	    move(t9,reg);
	}
	CodeLabel cl;

    mov(cl.dest(),ra);
    push(ra);


    jalr(t9);
    nop();
    bind(cl.src());
    addCodeLabel(cl);//1031

}
//hwj
void 
Assembler::call(const Operand &op) {
    switch (op.kind()) {
      case Operand::REG:
        call(Register::FromCode((int)(op.reg())));//op.reg()<->Registers::Code
        break;
      case Operand::REG_DISP:            	
        mcss.load32(mAddress(op.base(), op.disp()), t9.code());
        call(t9);
        break;
      default:
        JS_NOT_REACHED("unexpected operand kind");
    }
}

//hwj
void 
Assembler::call(ImmWord target) {
    int to = (int)(target.value);
    CodeLabel cl;

    mov(cl.dest(),v0);
    push(v0);

    lui(t9,to>>16);
    ori(t9,t9,to&0x0000ffff);

    jalr(t9);
    nop();
    bind(cl.src());
    addCodeLabel(cl);//1031
}

//hwj:1031
void 
Assembler::ma_call(const Register &reg) {
	if(reg != t9)
	{
	    move(t9,reg);
	}
	jalr(t9);
    nop();
}
//hwj:1031
void 
Assembler::ma_call(const Operand &op) {
    switch (op.kind()) {
      case Operand::REG:
        ma_call(Register::FromCode((int)(op.reg())));//op.reg()<->Registers::Code
        break;
      case Operand::REG_DISP:            	
        mcss.load32(mAddress(op.base(), op.disp()), t9.code());
        ma_call(t9);
        break;
      default:
        JS_NOT_REACHED("unexpected operand kind");
    }
}
//hwj:1031
void
Assembler::ma_call(ImmWord target) {
    int to = (int)(target.value);
    lui(t9,to>>16);
    ori(t9,t9,to&0x0000ffff);
    ma_call(t9);
}

Assembler::JmpSrc
Assembler::ma_callIon(const Register r)
{
    // When the stack is 8 byte aligned,
    // we want to decrement sp by 8, and write pc+8 into the new sp.
    // when we return from this call, sp will be its present value minus 4.
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
    mcss.offsetFromPCToV0(sizeof(int*)*8);//1insns
    mcss.add32(mTrustedImm32(4), sp.code());//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call(r.code()).m_jmp;//4insns
    return src;
}

Assembler::JmpSrc
Assembler::ma_callIonHalfPush(const Register r)
{
    // The stack is unaligned by 4 bytes.
    // We push the pc to the stack to align the stack before the call, when we
    // return the pc is poped and the stack is restored to its unaligned state.
    mcss.offsetFromPCToV0(sizeof(int*)*7);//1insns
    mcss.push(mRegisterID(v0.code()));//2insns
    JmpSrc src = mcss.call(r.code()).m_jmp;//4insns
    return src;
}

Assembler::JmpSrc
Assembler::ma_call(void *dest) // KEEP EMPTY
{
    JS_NOT_REACHED("no use!");
}

static unsigned int* __getpc(void)
{
    unsigned int *rtaddr;
    
    __asm__ volatile ("move %0, $31" : "=r"(rtaddr));
    
    return rtaddr;
}

//hwj
void
Assembler::patchWrite_NearCall(CodeLocationLabel startLabel, CodeLocationLabel target){
    uint32_t *start = (uint32_t*)startLabel.raw();
    uint32_t returnAddress=((uint32_t) start)+8*4;
    uint32_t *to = (uint32_t*)target.raw();
    *(start + 0) = 0x3c190000 | (returnAddress>>16);   //lui t9, hg
    *(start + 1) = 0x37390000 | (returnAddress&0x0000ffff); //ori t9 t9,hw
    *(start + 2) = 0x27bdfffc;  //addiiu sp sp -4
    *(start + 3) = 0xafb90000;  //sw sp t9 0

    unsigned tolw, tohg;
    tohg = (unsigned int)to>>16;
    tolw = (unsigned int)to&0xffff;

    *(start + 4) = 0x3c190000 | tohg;   //lui t9, hg
    *(start + 5) = 0x37390000 | tolw; //ori t9 t9,hw
    *(start + 6) = 0x0320f809;  //jalr t9
    *(start + 7) = 0x00000000;  //nop
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
        runtime_->setFlusher(NULL);
}
