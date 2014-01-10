/*
 * Copyright (C) 2009 Apple Inc. All rights reserved.
 * Copyright (C) 2009 University of Szeged
 * All rights reserved.
 * Copyright (C) 2010 MIPS Technologies, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY MIPS TECHNOLOGIES, INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL MIPS TECHNOLOGIES, INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef assembler_assembler_MIPSAssembler_h
#define assembler_assembler_MIPSAssembler_h

#if ENABLE(ASSEMBLER) && CPU(MIPS)

#include "assembler/assembler/AssemblerBuffer.h"
#include "assembler/wtf/Assertions.h"
#include "assembler/wtf/SegmentedVector.h"
#define IPFX  "        %s"
#define ISPFX "        "
#ifdef JS_METHODJIT_SPEW
# define MAYBE_PAD (isOOLPath ? ">  " : "")
#else
# define MAYBE_PAD ""
#endif

namespace JSC {

typedef uint32_t MIPSWord;

namespace MIPSRegisters {
typedef enum {
    r0 = 0,
    r1,
    r2,
    r3,
    r4,
    r5,
    r6,
    r7,
    r8,
    r9,
    r10,
    r11,
    r12,
    r13,
    r14,
    r15,
    r16,
    r17,
    r18,
    r19,
    r20,
    r21,
    r22,
    r23,
    r24,
    r25,
    r26,
    r27,
    r28,
    r29,
    r30,
    r31,
    zero = r0,
    at = r1,
    v0 = r2,
    v1 = r3,
    a0 = r4,
    a1 = r5,
    a2 = r6,
    a3 = r7,
    t0 = r8,
    t1 = r9,
    t2 = r10,
    t3 = r11,
    t4 = r12,
    t5 = r13,
    t6 = r14,
    t7 = r15,
    s0 = r16,
    s1 = r17,
    s2 = r18,
    s3 = r19,
    s4 = r20,
    s5 = r21,
    s6 = r22,
    s7 = r23,
    t8 = r24,
    t9 = r25,
    k0 = r26,
    k1 = r27,
    gp = r28,
    sp = r29,
    fp = r30,
    ra = r31,
    invalid_reg
} RegisterID;

typedef enum {
    f0 = 0,
    f1,
    f2,
    f3,
    f4,
    f5,
    f6,
    f7,
    f8,
    f9,
    f10,
    f11,
    f12,
    f13,
    f14,
    f15,
    f16,
    f17,
    f18,
    f19,
    f20,
    f21,
    f22,
    f23,
    f24,
    f25,
    f26,
    f27,
    f28,
    f29,
    f30,
    f31,
    invalid_freg
} FPRegisterID;

} // namespace MIPSRegisters

class MIPSAssembler : public GenericAssembler {
public:
    typedef MIPSRegisters::RegisterID RegisterID;
    typedef MIPSRegisters::FPRegisterID FPRegisterID;
    typedef SegmentedVector<int, 64> Jumps;
    unsigned char *buffer() const { return m_buffer.buffer(); }
    bool oom() const { return m_buffer.oom(); }

    // MIPS instruction opcode field position
    enum {
        OP_SH_RD = 11,
        OP_SH_RT = 16,
        OP_SH_RS = 21,
        OP_SH_SHAMT = 6,
        OP_SH_CODE = 16,
        OP_SH_FD = 6,
        OP_SH_FS = 11,
        OP_SH_FT = 16
    };

    class JmpSrc {
        friend class MIPSAssembler;
    public:
        JmpSrc()
            : m_offset(-1)
        {
        }

        JmpSrc(int offset)
            : m_offset(offset)
        {
        }

	int offset() { return m_offset; }
	
	bool isSet() const { return m_offset != -1; }

    private:
        int m_offset;
    };

    class JmpDst {
        friend class MIPSAssembler;
    public:
        JmpDst()
            : m_offset(-1)
            , m_used(false)
        {
        }

        bool isUsed() const { return m_used; }
        void used() { m_used = true; }
        bool isValid() const { return m_offset != -1; }

        JmpDst(int offset)
            : m_offset(offset)
            , m_used(false)
        {
            ASSERT(m_offset == offset);
        }

	int offset() const { return m_offset; }

    private:
        int m_offset : 31;
        int m_used : 1;
    };

    void emitInst(MIPSWord op)
    {
        void* oldBase = m_buffer.data();

        m_buffer.putInt(op);

        void* newBase = m_buffer.data();
        if (oldBase != newBase)
            relocateJumps(oldBase, newBase);
    }

    void nop()
    {
        emitInst(0x00000000);
    }

    /* Need to insert one load data delay nop for mips1.  */
    void loadDelayNop()
    {
#if WTF_MIPS_ISA(1)
        nop();
#endif
    }

    /* Need to insert one coprocessor access delay nop for mips1.  */
    void copDelayNop()
    {
#if WTF_MIPS_ISA(1)
        nop();
#endif
    }

    void movz(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x0000000a | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void move(RegisterID rd, RegisterID rs)
    {
        /* addu */
        emitInst(0x00000021 | (rd << OP_SH_RD) | (rs << OP_SH_RS));
    }

    /* Set an immediate value to a register.  This may generate 1 or 2
       instructions.  */
    void li(RegisterID dest, int imm)
    {
        if (imm >= -32768 && imm <= 32767)
            addiu(dest, MIPSRegisters::zero, imm);
        else if (imm >= 0 && imm < 65536)
            ori(dest, MIPSRegisters::zero, imm);
        else {
            lui(dest, imm >> 16);
            if (imm & 0xffff)
                ori(dest, dest, imm);
        }
    }

    void lui(RegisterID rt, int imm)
    {
        emitInst(0x3c000000 | (rt << OP_SH_RT) | (imm & 0xffff));
    }

    void addiu(RegisterID rt, RegisterID rs, int imm)
    {
        emitInst(0x24000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (imm & 0xffff));
    }

    void addu(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x00000021 | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void subu(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x00000023 | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void mult(RegisterID rs, RegisterID rt)
    {
        emitInst(0x00000018 | (rs << OP_SH_RS) | (rt << OP_SH_RT));
    }

    void div(RegisterID rs, RegisterID rt)
    {
        emitInst(0x0000001a | (rs << OP_SH_RS) | (rt << OP_SH_RT));
    }

    // add by wangqing, 2013-12-24
    void divu(RegisterID rs, RegisterID rt)
    {
        emitInst(0x0000001b | (rs << OP_SH_RS) | (rt << OP_SH_RT));
    }

    void mfhi(RegisterID rd)
    {
        emitInst(0x00000010 | (rd << OP_SH_RD));
    }

    void mflo(RegisterID rd)
    {
        emitInst(0x00000012 | (rd << OP_SH_RD));
    }

    void mul(RegisterID rd, RegisterID rs, RegisterID rt)
    {
#if WTF_MIPS_ISA_AT_LEAST(32) 
        emitInst(0x70000002 | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
#else
        mult(rs, rt);
        mflo(rd);
#endif
    }

    void andInsn(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x00000024 | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void andi(RegisterID rt, RegisterID rs, int imm)
    {
        emitInst(0x30000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (imm & 0xffff));
    }

    void nor(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x00000027 | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void orInsn(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x00000025 | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void ori(RegisterID rt, RegisterID rs, int imm)
    {
        emitInst(0x34000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (imm & 0xffff));
    }

    void xorInsn(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x00000026 | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void xori(RegisterID rt, RegisterID rs, int imm)
    {
        emitInst(0x38000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (imm & 0xffff));
    }

    void slt(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x0000002a | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void sltu(RegisterID rd, RegisterID rs, RegisterID rt)
    {
        emitInst(0x0000002b | (rd << OP_SH_RD) | (rs << OP_SH_RS)
                 | (rt << OP_SH_RT));
    }

    void sltiu(RegisterID rt, RegisterID rs, int imm)
    {
        emitInst(0x2c000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (imm & 0xffff));
    }

    void sll(RegisterID rd, RegisterID rt, int shamt)
    {
        emitInst(0x00000000 | (rd << OP_SH_RD) | (rt << OP_SH_RT)
                 | ((shamt & 0x1f) << OP_SH_SHAMT));
    }

    // by wangqing, 2013-11-06, modify rs from int to RegisterID
    void sllv(RegisterID rd, RegisterID rt, RegisterID rs)
    {
        emitInst(0x00000004 | (rd << OP_SH_RD) | (rt << OP_SH_RT)
                 | (rs << OP_SH_RS));
    }

    void sra(RegisterID rd, RegisterID rt, int shamt)
    {
        emitInst(0x00000003 | (rd << OP_SH_RD) | (rt << OP_SH_RT)
                 | ((shamt & 0x1f) << OP_SH_SHAMT));
    }

    void srav(RegisterID rd, RegisterID rt, RegisterID rs)
    {
        emitInst(0x00000007 | (rd << OP_SH_RD) | (rt << OP_SH_RT)
                 | (rs << OP_SH_RS));
    }

    void srl(RegisterID rd, RegisterID rt, int shamt)
    {
        emitInst(0x00000002 | (rd << OP_SH_RD) | (rt << OP_SH_RT)
                 | ((shamt & 0x1f) << OP_SH_SHAMT));
    }

    void srlv(RegisterID rd, RegisterID rt, RegisterID rs)
    {
        emitInst(0x00000006 | (rd << OP_SH_RD) | (rt << OP_SH_RT)
                 | (rs << OP_SH_RS));
    }

    void lb(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0x80000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
        loadDelayNop();
    }

    void lbu(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0x90000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
        loadDelayNop();
    }

    void lw(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0x8c000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
        loadDelayNop();
    }

    void lwl(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0x88000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
        loadDelayNop();
    }

    void lwr(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0x98000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
        loadDelayNop();
    }

    void lh(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0x84000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
        loadDelayNop();
    }

    void lhu(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0x94000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
        loadDelayNop();
    }

    void sb(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0xa0000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
    }

    void sh(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0xa4000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
    }

    void sw(RegisterID rt, RegisterID rs, int offset)
    {
        emitInst(0xac000000 | (rt << OP_SH_RT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
    }

    void jr(RegisterID rs)
    {
        emitInst(0x00000008 | (rs << OP_SH_RS));
    }

    void jalr(RegisterID rs)
    {
        emitInst(0x0000f809 | (rs << OP_SH_RS));
    }

    void jal()
    {
        emitInst(0x0c000000);
    }

    void bkpt()
    {
        int value = 512; /* BRK_BUG */
        emitInst(0x0000000d | ((value & 0x3ff) << OP_SH_CODE));
    }

    // xsb: fix me.
    void bal(int imm)
    {
	emitInst(0x04110000 | (imm & 0xfffff));
    }

    void bgez(RegisterID rs, int imm)
    {
        emitInst(0x04010000 | (rs << OP_SH_RS) | (imm & 0xffff));
    }

    void bltz(RegisterID rs, int imm)
    {
        emitInst(0x04000000 | (rs << OP_SH_RS) | (imm & 0xffff));
    }

    void beq(RegisterID rs, RegisterID rt, int imm)
    {
        emitInst(0x10000000 | (rs << OP_SH_RS) | (rt << OP_SH_RT) | (imm & 0xffff));
    }

    void bne(RegisterID rs, RegisterID rt, int imm)
    {
        emitInst(0x14000000 | (rs << OP_SH_RS) | (rt << OP_SH_RT) | (imm & 0xffff));
    }

    void bc1t()
    {
        emitInst(0x45010000);
    }

    void bc1f()
    {
        emitInst(0x45000000);
    }

    JmpSrc newJmpSrc()
    {
        return JmpSrc(m_buffer.size());
    }

    void appendJump()
    {
        m_jumps.append(m_buffer.size());
    }

    void movd(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46200006 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // add by wangqing, 2013-12-25
    void movs(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46000006 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    void addd(FPRegisterID fd, FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200000 | (fd << OP_SH_FD) | (fs << OP_SH_FS)
                 | (ft << OP_SH_FT));
    }

    // add by wangqing, 2013-12-23
    void adds(FPRegisterID fd, FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46000000 | (fd << OP_SH_FD) | (fs << OP_SH_FS)
                 | (ft << OP_SH_FT));
    }

    void subd(FPRegisterID fd, FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200001 | (fd << OP_SH_FD) | (fs << OP_SH_FS)
                 | (ft << OP_SH_FT));
    }

    // add by wangqing, 2013-12-23
    void subs(FPRegisterID fd, FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46000001 | (fd << OP_SH_FD) | (fs << OP_SH_FS)
                 | (ft << OP_SH_FT));
    }

    void muld(FPRegisterID fd, FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200002 | (fd << OP_SH_FD) | (fs << OP_SH_FS)
                 | (ft << OP_SH_FT));
    }

    // add by wangqing, 2013-12-23
    void muls(FPRegisterID fd, FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46000002 | (fd << OP_SH_FD) | (fs << OP_SH_FS)
                 | (ft << OP_SH_FT));
    }

    void divd(FPRegisterID fd, FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200003 | (fd << OP_SH_FD) | (fs << OP_SH_FS)
                 | (ft << OP_SH_FT));
    }

    // add by wangqing, 2013-12-23
    void divs(FPRegisterID fd, FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46000003 | (fd << OP_SH_FD) | (fs << OP_SH_FS)
                 | (ft << OP_SH_FT));
    }

    void negd(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46200007 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // add by wangqing, 2013-12-25
    void negs(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46000007 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    void absd(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46200005 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // add by wangqing, 2013-12-26
    void abss(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46000005 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    void lwc1(FPRegisterID ft, RegisterID rs, int offset)
    {
        emitInst(0xc4000000 | (ft << OP_SH_FT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
        copDelayNop();
    }

    void ldc1(FPRegisterID ft, RegisterID rs, int offset)
    {
        emitInst(0xd4000000 | (ft << OP_SH_FT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
    }

    void swc1(FPRegisterID ft, RegisterID rs, int offset)
    {
        emitInst(0xe4000000 | (ft << OP_SH_FT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
    }

    void sdc1(FPRegisterID ft, RegisterID rs, int offset)
    {
        emitInst(0xf4000000 | (ft << OP_SH_FT) | (rs << OP_SH_RS)
                 | (offset & 0xffff));
    }

    void mtc1(RegisterID rt, FPRegisterID fs)
    {
        emitInst(0x44800000 | (fs << OP_SH_FS) | (rt << OP_SH_RT));
        copDelayNop();
    }

    void mthc1(RegisterID rt, FPRegisterID fs)
    {
        emitInst(0x44e00000 | (fs << OP_SH_FS) | (rt << OP_SH_RT));
        copDelayNop();
    }

    // by xsb: fix me
    void dsrl32(RegisterID rt, RegisterID rd, int saminus32)
    {
        emitInst(0x0000003d | (rd << OP_SH_RD) | (rt << OP_SH_RT) | (saminus32 << OP_SH_SHAMT));
        copDelayNop();
    }

    // by xsb: fix me
    void dmfc1(RegisterID rt, FPRegisterID fs)
    {
        emitInst(0x44200000 | (fs << OP_SH_FS) | (rt << OP_SH_RT));
        copDelayNop();
    }

    void mfc1(RegisterID rt, FPRegisterID fs)
    {
        emitInst(0x44000000 | (fs << OP_SH_FS) | (rt << OP_SH_RT));
        copDelayNop();
    }

    void sqrtd(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46200004 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // add by wangqing, 2013-12-23
    void sqrts(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46000004 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    void truncwd(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x4620000d | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // add by wangqing, 2013-12-23
    void truncws(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x4600000d | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // by xsb: fix me
    void floorwd(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x4620000f | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // add by wangqing, 2013-12-23
    void floorws(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x4600000f | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }
    void cvtdw(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46800021 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // add by wangqing, 2013-12-23
    void cvtsw(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46800020 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    void cvtds(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46000021 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    void cvtsd(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46200020 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    void cvtwd(FPRegisterID fd, FPRegisterID fs)
    {
        emitInst(0x46200024 | (fd << OP_SH_FD) | (fs << OP_SH_FS));
    }

    // by xsb: fix me
    void cud(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200031 | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void ceqd(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200032 | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    //by weizhenwei, 2013.10.29
    void cseqd(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x4620003a | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void cngtd(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x4620003f | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void cnged(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x4620003d | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void cltd(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x4620003c | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void cled(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x4620003e | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void cueqd(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200033 | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void coled(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200036 | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void coltd(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200034 | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void culed(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200037 | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    void cultd(FPRegisterID fs, FPRegisterID ft)
    {
        emitInst(0x46200035 | (fs << OP_SH_FS) | (ft << OP_SH_FT));
        copDelayNop();
    }

    // General helpers

    JmpDst label()
    {
        return JmpDst(m_buffer.size());
    }

    // by xsb; fix me
    size_t currentOffset() const { return m_buffer.size(); }

    JmpDst align(int alignment)
    {
        while (!m_buffer.isAligned(alignment))
            bkpt();

        return label();
    }

    static void* getRelocatedAddress(void* code, JmpSrc jump)
    {
        ASSERT(jump.m_offset != -1);
        void* b = reinterpret_cast<void*>((reinterpret_cast<intptr_t>(code)) + jump.m_offset);
        return b;
    }

    static void* getRelocatedAddress(void* code, JmpDst label)
    {
        void* b = reinterpret_cast<void*>((reinterpret_cast<intptr_t>(code)) + label.m_offset);
        return b;
    }

    static int getDifferenceBetweenLabels(JmpDst from, JmpDst to)
    {
        return to.m_offset - from.m_offset;
    }

    static int getDifferenceBetweenLabels(JmpDst from, JmpSrc to)
    {
        return to.m_offset - from.m_offset;
    }

    static int getDifferenceBetweenLabels(JmpSrc from, JmpDst to)
    {
        return to.m_offset - from.m_offset;
    }

    // Assembler admin methods:

    size_t size() const
    {
        return m_buffer.size();
    }

    void executableCopy(void* buffer)
    {
        memcpy(buffer, m_buffer.data(), m_buffer.size());
        relocateJumps(m_buffer.data(), buffer);
    }

    void* executableAllocAndCopy(ExecutableAllocator* allocator, ExecutablePool** poolp, CodeKind kind)
    {
        void *result = m_buffer.executableAllocAndCopy(allocator, poolp, kind);
        if (!result)
          return 0;

        relocateJumps(m_buffer.data(), result);
        return result;
    }

    // by xsb; fix me
    static void setRe132(void* from, void* to);

    static unsigned getCallReturnOffset(JmpSrc call);

    // Linking & patching:
    //
    // 'link' and 'patch' methods are for use on unprotected code - such as the code
    // within the AssemblerBuffer, and code being patched by the patch buffer.  Once
    // code has been finalized it is (platform support permitting) within a non-
    // writable region of memory; to modify the code in an execute-only execuable
    // pool the 'repatch' and 'relink' methods should be used.
    void linkJump(JmpSrc from, JmpDst to);
    static void linkJump(void* code, JmpSrc from, void* to);
    static bool canRelinkJump(void* from, void* to);

    static void linkCall(void* code, JmpSrc from, void* to);

    static void linkPointer(void* code, JmpDst from, void* to);

    static void relinkJump(void* from, void* to);
    static void relinkCall(void* from, void* to);

    static void repatchInt32(void* from, int32_t to);

    static void repatchPointer(void* from, void* to);

    static void repatchLoadPtrToLEA(void* from);

    static void repatchLEAToLoadPtr(void* from);

    // Like Lua's emitter, we thread jump lists through the unpatched target
    // field, which will get fixed up when the label (which has a pointer to
    // the head of the jump list) is bound.
    // by xsb; fix me.
    bool nextJump(const JmpSrc& from, JmpSrc* next);

    bool nextBranch(const JmpSrc& from, JmpSrc* next);

    void setNextJump(const JmpSrc& from, const JmpSrc &to);

    // author:huangwenjun date:2013-12-23
    void clearOffsetForLabel(const JmpSrc& from);

    //author:huangwenjun date:2013-12-23
    void doubleConstant(double d);
    
    // by wangqing, 2014-01-10
    void FloatConstant(float f);

    // by wangqing
    void linkBranch(JmpSrc from, JmpDst to);

    static void *getRel32Target(void* where);

    static void *getPointer(void* where);

    static void **getPointerRef(void* where);

    static void setRel32(void* from, void* to);

    static void setPointer(void* where, const void* value);

    static int32_t getInt32(void* where);

    static void setInt32(void* where, int32_t value);

    //author:huangwenjun date:2013-12-24
    static int linkWithOffset(MIPSWord* insn, void* to);

    /* Update each jump in the buffer of newBase.  */
    void relocateJumps(void* oldBase, void* newBase);

    static int linkCallInternal(void* from, void* to);

    void preLink(JmpSrc jump, void* target);

    //void preLink(JmpSrc jump, ImmPtr target);

//private:
    AssemblerBuffer m_buffer;
    Jumps m_jumps;
};

} // namespace JSC

#endif // ENABLE(ASSEMBLER) && CPU(MIPS)

#endif /* assembler_assembler_MIPSAssembler_h */
