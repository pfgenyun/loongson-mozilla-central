/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-

 * vim: set ts=8 sw=4 et tw=79:
 *
 * ***** BEGIN LICENSE BLOCK *****
 * Copyright (C) 2009 University of Szeged
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY UNIVERSITY OF SZEGED ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL UNIVERSITY OF SZEGED OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ***** END LICENSE BLOCK ***** */

#include "assembler/wtf/Platform.h"

#if ENABLE(ASSEMBLER) && CPU(MIPS)

#include "assembler/assembler/MIPSAssembler.h"

namespace JSC {

    void MIPSAssembler::setRel32(void* from, void* to)
    {
        intptr_t offset = reinterpret_cast<intptr_t>(to) - reinterpret_cast<intptr_t>(from);
        ASSERT(offset == static_cast<int32_t>(offset));
#define JS_CRASH(x) *(int *)x = 0
        if (offset != static_cast<int32_t>(offset))
            JS_CRASH(0xC0DE); 
#undef JS_CRASH
    
        staticSpew("##setRel32 ((from=%p)) ((to=%p))", from, to);
        setInt32(from, offset);
    }
    //author:huangwenjun date:2013-12-24
    unsigned MIPSAssembler::getCallReturnOffset(JmpSrc call)
    {
        // The return address is after a call and a delay slot instruction
        return call.m_offset;
    }

    //author:huangwenjun date:2013-12-24
    bool MIPSAssembler::nextJump(const JmpSrc& from, JmpSrc* next)
    {
        if (oom())
               return false;
        char* code = reinterpret_cast<char*>(m_buffer.data());
        int32_t offset = getInt32(code + from.m_offset-4);

        if (offset == -1)
            return false;
        *next = JmpSrc(offset);
        return true;
    }

    // by wangqing, 2013-11-20
    bool MIPSAssembler::nextBranch(const JmpSrc& from, JmpSrc* next)
    {
        if (oom())
               return false;
        char* code = reinterpret_cast<char*>(m_buffer.data());
        int32_t branchInsn = getInt32(code + from.m_offset);
		int32_t offset = (branchInsn << 16) >> 16; 

        if (offset == -1)
            return false;
        *next = JmpSrc(from.m_offset + 4 + (offset << 2));
        return true;
    }

    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::setNextJump(const JmpSrc& from, const JmpSrc &to)
    {
        // Sanity check - if the assembler has OOM'd, it will start overwriting
        // its internal buffer and thus our links could be garbage.
        if (oom())
            return;
        char* code = reinterpret_cast<char*>(m_buffer.data());
        setInt32(code + from.m_offset-4, to.m_offset);
    }

    //author:huangwenjun date:2013-12-23        set nop
    void MIPSAssembler::clearOffsetForLabel(const JmpSrc& from)
    {
        char* code = reinterpret_cast<char*>(m_buffer.data());
        setInt32(code + from.m_offset-4, 0);   
    }

    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::linkJump(JmpSrc from, JmpDst to)
    {
        ASSERT(to.m_offset != -1);
        ASSERT(from.m_offset != -1);
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(m_buffer.data()) + from.m_offset);
        MIPSWord* toPos = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(m_buffer.data()) + to.m_offset);

        ASSERT((!(*(insn - 1)) && !(*(insn - 2)) && !(*(insn - 3)) && !(*(insn - 5))));
        insn = insn - 6;
        linkWithOffset(insn, toPos);
    }

    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::linkJump(void* code, JmpSrc from, void* to)
    {
        ASSERT(from.m_offset != -1);
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(code) + from.m_offset);

        ASSERT((!(*(insn - 1)) && !(*(insn - 2)) && !(*(insn - 3)) && !(*(insn - 5))));
        insn = insn - 6;
        linkWithOffset(insn, to);
    }

    //author:huangwenjun date:2013-12-23
    bool MIPSAssembler::canRelinkJump(void* from, void* to)
    {
        return true;
    }

    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::doubleConstant(double d)
    {
        m_buffer.ensureSpace(sizeof(double));
        union {
           uint64_t u64;
           double d;
        } u;
        u.d = d;
        m_buffer.putInt64Unchecked(u.u64);
    }
    
    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::linkCall(void* code, JmpSrc from, void* to)
    {
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(code) + from.m_offset);
        linkCallInternal(insn, to);
    }

    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::linkPointer(void* code, JmpDst from, void* to)
    {
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(code) + from.m_offset);
        ASSERT((*insn & 0xffe00000) == 0x3c000000); // lui
        *insn = (*insn & 0xffff0000) | ((reinterpret_cast<intptr_t>(to) >> 16) & 0xffff);
        insn++;
        ASSERT((*insn & 0xfc000000) == 0x34000000); // ori
        *insn = (*insn & 0xffff0000) | (reinterpret_cast<intptr_t>(to) & 0xffff);
    }

    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::relinkJump(void* from, void* to)
    {
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(from);

        ASSERT(!(*(insn-1)) && !(*(insn - 5)));
        insn = insn - 6;
        int flushSize = linkWithOffset(insn, to);

        ExecutableAllocator::cacheFlush(insn, flushSize);
    }
   

    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::relinkCall(void* from, void* to)
    {
        void* start;
        int size = linkCallInternal(from, to);
        if (size == sizeof(MIPSWord))
            start = reinterpret_cast<void*>(reinterpret_cast<intptr_t>(from) - 2 * sizeof(MIPSWord));
        else
            start = reinterpret_cast<void*>(reinterpret_cast<intptr_t>(from) - 4 * sizeof(MIPSWord));

        ExecutableAllocator::cacheFlush(start, size);
    }
    
    //author:huangwenjun date:2013-12-26
    void MIPSAssembler::repatchInt32(void* from, int32_t to)
    {
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(from);
        ASSERT((*insn & 0xffe00000) == 0x3c000000); // lui
        *insn = (*insn & 0xffff0000) | ((to >> 16) & 0xffff);
        insn++;
        ASSERT((*insn & 0xfc000000) == 0x34000000); // ori
        *insn = (*insn & 0xffff0000) | (to & 0xffff);
        insn--;
        ExecutableAllocator::cacheFlush(insn, 2 * sizeof(MIPSWord));
    }

    void MIPSAssembler::repatchPointer(void* from, void* to)
    {
        repatchInt32(from, reinterpret_cast<int32_t>(to));
    }
    
    void MIPSAssembler::repatchLoadPtrToLEA(void* from)
    {
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(from);
        insn = insn + 3;
        ASSERT((*insn & 0xfc000000) == 0x8c000000); // lw
        /* lw -> addiu */
        *insn = 0x24000000 | (*insn & 0x03ffffff);

        ExecutableAllocator::cacheFlush(insn, sizeof(MIPSWord));
    }    
    
    void MIPSAssembler::repatchLEAToLoadPtr(void* from)
    {
          MIPSWord* insn = reinterpret_cast<MIPSWord*>(from);
          insn = insn + 3;
          if ((*insn & 0xfc000000) == 0x8c000000)
            return; // Valid lw instruction
  
          ASSERT((*insn & 0xfc000000) == 0x24000000); // addiu
          /* addiu -> lw */
          *insn = 0x8c000000 | (*insn & 0x03ffffff);
  
          ExecutableAllocator::cacheFlush(insn, sizeof(MIPSWord));
    }

    //author:huangwenjun date:2013-12-23
    int MIPSAssembler::linkWithOffset(MIPSWord* insn, void* to)
    {
        ASSERT((*insn & 0xfc000000) == 0x10000000 // beq
               || (*insn & 0xfc000000) == 0x14000000 // bne
               || (*insn & 0xffff0000) == 0x45010000 // bc1t
               || (*insn & 0xffff0000) == 0x45000000); // bc1f
        intptr_t diff = (reinterpret_cast<intptr_t>(to)
                         - reinterpret_cast<intptr_t>(insn) - 4) >> 2;

        if (diff < -32768 || diff > 32767 || *(insn + 2) != 0x10000003) {
            /*
                Convert the sequence:
                  beq $2, $3, target
                  nop
                  b 1f
                  nop
                  nop
                  nop
                1:

                to the new sequence if possible:
                  bne $2, $3, 1f
                  nop
                  j    target
                  nop
                  nop
                  nop
                1:

                OR to the new sequence:
                  bne $2, $3, 1f
                  nop
                  lui $25, target >> 16
                  ori $25, $25, target & 0xffff
                  jr $25
                  nop
                1:

                Note: beq/bne/bc1t/bc1f are converted to bne/beq/bc1f/bc1t.
            */

            if (*(insn + 2) == 0x10000003) {
                if ((*insn & 0xfc000000) == 0x10000000) // beq
                    *insn = (*insn & 0x03ff0000) | 0x14000005; // bne
                else if ((*insn & 0xfc000000) == 0x14000000) // bne
                    *insn = (*insn & 0x03ff0000) | 0x10000005; // beq
                else if ((*insn & 0xffff0000) == 0x45010000) // bc1t
                    *insn = 0x45000005; // bc1f
                else if ((*insn & 0xffff0000) == 0x45000000) // bc1f
                    *insn = 0x45010005; // bc1t
                else
                    ASSERT(0);
            }

            insn = insn + 2;
            if (0 && ((reinterpret_cast<intptr_t>(insn) + 4) >> 28
                == reinterpret_cast<intptr_t>(to) >> 28)) {
                *insn = 0x08000000 | ((reinterpret_cast<intptr_t>(to) >> 2) & 0x3ffffff);
                *(insn + 1) = 0;
                return 4 * sizeof(MIPSWord);
            }

            intptr_t newTargetAddress = reinterpret_cast<intptr_t>(to);
            /* lui */
            *insn = 0x3c000000 | (MIPSRegisters::t9 << OP_SH_RT) | ((newTargetAddress >> 16) & 0xffff);
            /* ori */
            *(insn + 1) = 0x34000000 | (MIPSRegisters::t9 << OP_SH_RT) | (MIPSRegisters::t9 << OP_SH_RS) | (newTargetAddress & 0xffff);
            /* jr */
            *(insn + 2) = 0x00000008 | (MIPSRegisters::t9 << OP_SH_RS);
            return 5 * sizeof(MIPSWord);
        }

        *insn = (*insn & 0xffff0000) | (diff & 0xffff);
        return sizeof(MIPSWord);
    }
    
    //author:huangwenjun date:2013-12-24
    void MIPSAssembler::relocateJumps(void* oldBase, void* newBase)
    {
        // Check each jump
        for (Jumps::Iterator iter = m_jumps.begin(); iter != m_jumps.end(); ++iter) {
            int pos = *iter;
            MIPSWord* insn = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(newBase) + pos);
            insn = insn + 2;
            // Need to make sure we have 5 valid instructions after pos
            if ((unsigned int)pos >= m_buffer.size() - 5 * sizeof(MIPSWord))
                continue;

            if ((*insn & 0xfc000000) == 0x08000000) { // j
                int offset = *insn & 0x03ffffff;
                int oldInsnAddress = (int)insn - (int)newBase + (int)oldBase;
                int topFourBits = (oldInsnAddress + 4) >> 28;
                int oldTargetAddress = (topFourBits << 28) | (offset << 2);
                int newTargetAddress = oldTargetAddress - (int)oldBase + (int)newBase;
                int newInsnAddress = (int)insn;
                //if (((newInsnAddress + 4) >> 28) == (newTargetAddress >> 28))
                if (0 && (((newInsnAddress + 4) >> 28) == (newTargetAddress >> 28)))
                    *insn = 0x08000000 | ((newTargetAddress >> 2) & 0x3ffffff);
                else {
                    /* lui */
                    *insn = 0x3c000000 | (MIPSRegisters::t9 << OP_SH_RT) | ((newTargetAddress >> 16) & 0xffff);
                    /* ori */
                    *(insn + 1) = 0x34000000 | (MIPSRegisters::t9 << OP_SH_RT) | (MIPSRegisters::t9 << OP_SH_RS) | (newTargetAddress & 0xffff);
                    /* jr */
                    *(insn + 2) = 0x00000008 | (MIPSRegisters::t9 << OP_SH_RS);
                }
            } else if ((*insn & 0xffe00000) == 0x3c000000) { // lui
                int high = (*insn & 0xffff) << 16;
                int low = *(insn + 1) & 0xffff;
                int oldTargetAddress = high | low;
                if((oldTargetAddress >= -(m_buffer.size()) || oldTargetAddress >= -128) && oldTargetAddress <= m_buffer.size()) continue;
                int newTargetAddress = oldTargetAddress - (int)oldBase + (int)newBase;
                /* lui */
                *insn = 0x3c000000 | (MIPSRegisters::t9 << OP_SH_RT) | ((newTargetAddress >> 16) & 0xffff);
                /* ori */
                *(insn + 1) = 0x34000000 | (MIPSRegisters::t9 << OP_SH_RT) | (MIPSRegisters::t9 << OP_SH_RS) | (newTargetAddress & 0xffff);
            }
        }
    }
    
    //author:huangwenjun date:2013-12-24
    int MIPSAssembler::linkCallInternal(void* from, void* to)
    {
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(from);
        insn = insn - 4;

        if ((*(insn + 2) & 0xfc000000) == 0x0c000000) { // jal
            if ((reinterpret_cast<intptr_t>(from) - 4) >> 28
                == reinterpret_cast<intptr_t>(to) >> 28) {
                *(insn + 2) = 0x0c000000 | ((reinterpret_cast<intptr_t>(to) >> 2) & 0x3ffffff);
                return sizeof(MIPSWord);
            }

            /* lui $25, (to >> 16) & 0xffff */
            *insn = 0x3c000000 | (MIPSRegisters::t9 << OP_SH_RT) | ((reinterpret_cast<intptr_t>(to) >> 16) & 0xffff);
            /* ori $25, $25, to & 0xffff */
            *(insn + 1) = 0x34000000 | (MIPSRegisters::t9 << OP_SH_RT) | (MIPSRegisters::t9 << OP_SH_RS) | (reinterpret_cast<intptr_t>(to) & 0xffff);
            /* jalr $25 */
            *(insn + 2) = 0x0000f809 | (MIPSRegisters::t9 << OP_SH_RS);
            return 3 * sizeof(MIPSWord);
        } else if((*(insn + 2) & 0xffff0000) == 0x04110000) {//bal
            intptr_t offset = reinterpret_cast<intptr_t>(to) >> 2;
            if(reinterpret_cast<intptr_t>(to) > 0x100000)
                offset = reinterpret_cast<intptr_t>(to) - reinterpret_cast<intptr_t>(from);
            *(insn + 2) = 0x04110000 | ((offset >> 2) & 0xffff);
            return sizeof(MIPSWord);
        }

        ASSERT((*insn & 0xffe00000) == 0x3c000000); // lui
        ASSERT((*(insn + 1) & 0xfc000000) == 0x34000000); // ori
        /* lui */
        *insn = (*insn & 0xffff0000) | ((reinterpret_cast<intptr_t>(to) >> 16) & 0xffff);
        /* ori */
        *(insn + 1) = (*(insn + 1) & 0xffff0000) | (reinterpret_cast<intptr_t>(to) & 0xffff);
        return 2 * sizeof(MIPSWord);
    }

    // by wangqing, 2013-11-20
    void MIPSAssembler::linkBranch(JmpSrc from, JmpDst to)
    {
        ASSERT(to.m_offset != -1);
        ASSERT(from.m_offset != -1);
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(m_buffer.data()) + from.m_offset);
		
	int32_t offset = (to.m_offset - (from.m_offset + 4)) >> 2;
	*insn = (*insn & 0xffff0000) | (offset & 0x0000ffff);
    }
   
    void * MIPSAssembler::getRel32Target(void* where)
    {
        int32_t rel = getInt32(where);
        return (char *)where + rel;
    }
    
    //author:huangwenjun date:2013-12-23
    void * MIPSAssembler::getPointer(void* where)
    {
        MIPSWord* insn = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(where));
        int32_t offset = -2;
        insn -= 2;
		
        ASSERT(((*(insn) & 0xfc000000) == 0x3c000000) && (((*(insn + 1)) & 0xfc000000) == 0x34000000));
        offset = (*(insn) & 0x0000ffff) << 16; // lui
        offset |= (*(insn + 1) & 0x0000ffff); // ori
        return reinterpret_cast<void *>(offset);
    }
    
    //author:huangwenjun date:2013-12-23
    void MIPSAssembler::setPointer(void* where, const void* value)
    {
        staticSpew("##setPtr     ((where=%p)) ((value=%p))", where, value);
        reinterpret_cast<const void**>(where)[-1] = value;
    }

    int32_t MIPSAssembler::getInt32(void* where)
    {

        return *((int32_t*)where);
    }

    void MIPSAssembler::setInt32(void* where, int32_t value)
    {
        *((int32_t *)where)= value;
    }

    //author:huangwenjun date:2013-12-24
    void MIPSAssembler::preLink(JmpSrc jump, void* target)
    {
        ASSERT(m_jumps.size() > 0);
        MIPSWord* insn = (MIPSWord*)(m_buffer.data()+jump.offset()-6*4);//head
        Jumps tmp;

        for (Jumps::Iterator iter = m_jumps.begin(); iter != m_jumps.end(); ++iter){
            if(*iter != jump.offset()) {
                tmp.append(*iter);
            }
        }

        m_jumps.clear();

        for (Jumps::Iterator iter = tmp.begin(); iter != tmp.end(); ++iter){
            m_jumps.append(*iter);
        }
        tmp.clear();
        /*
        for(i = 0; i < m_jumps.size(); i++)
        {
            if(m_jumps[i] == jump.offset()) {
                m_jumps[i] = m_jumps.last();
                break;
            }
        }
        ASSERT(i < m_jumps.size());
        */
        /*
        if(i < m_jumps.size())        
            m_jumps.removeLast();
        */

        ASSERT((*insn & 0xfc000000) == 0x10000000 // beq
               || (*insn & 0xfc000000) == 0x14000000 // bne
               || (*insn & 0xffff0000) == 0x45010000 // bc1t
               || (*insn & 0xffff0000) == 0x45000000); // bc1f

        if ((*insn & 0xfc000000) == 0x10000000) // beq
            *insn = (*insn & 0x03ff0000) | 0x14000005; // bne
        else if ((*insn & 0xfc000000) == 0x14000000) // bne
            *insn = (*insn & 0x03ff0000) | 0x10000005; // beq
        else if ((*insn & 0xffff0000) == 0x45010000) // bc1t
            *insn = 0x45000005; // bc1f
        else if ((*insn & 0xffff0000) == 0x45000000) // bc1f
            *insn = 0x45010005; // bc1t
        else
            ASSERT(0);
        ASSERT(*(insn + 1) == 0x00000000);

        int to = (int)target;
        //lui
        *(insn + 2) = 0x3c000000 | (MIPSRegisters::t9 << OP_SH_RT) | (to >> 16);
        //ori
        *(insn + 3) = 0x34000000 | (MIPSRegisters::t9 << OP_SH_RT) | (MIPSRegisters::t9 << OP_SH_RS) | (to & 0x0000ffff);
        /* jr */
        *(insn + 4) = 0x00000008 | (MIPSRegisters::t9 << OP_SH_RS);
        *(insn + 5) = 0x00000000;
    }

    //author:huangwenjun date:2013-12-24 need check
    /*
    void MIPSAssembler::preLink(JmpSrc jump, ImmPtr target)
    {
        ASSERT(0);
    }
    */

} // namespace JSC

#endif // ENABLE(ASSEMBLER) && CPU(MIPS)
