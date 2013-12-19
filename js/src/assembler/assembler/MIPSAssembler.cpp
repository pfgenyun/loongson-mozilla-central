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

    //hwj
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

    //hwj
    void MIPSAssembler::setNextJump(const JmpSrc& from, const JmpSrc &to)
    {
        // Sanity check - if the assembler has OOM'd, it will start overwriting
        // its internal buffer and thus our links could be garbage.
        if (oom())
            return;
        char* code = reinterpret_cast<char*>(m_buffer.data());
        setInt32(code + from.m_offset-4, to.m_offset);
    }

    //hwj:set nop
    void MIPSAssembler::clearOffsetForLabel(const JmpSrc& from)
    {
        char* code = reinterpret_cast<char*>(m_buffer.data());
        setInt32(code + from.m_offset-4, 0);   
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

    void ** MIPSAssembler::getPointerRef(void* where)
     {
         //return &reinterpret_cast<void **>(where)[-1];
         MIPSWord* insn = reinterpret_cast<MIPSWord*>(reinterpret_cast<intptr_t>(where));
         int32_t offset = -2;

         insn -= 2;
                 ASSERT(((*(insn) & 0xfc000000) == 0x3c000000) && (((*(insn + 1)) & 0xfc000000) == 0x34000000));
         offset = (*insn & 0x0000ffff) << 16; // lui
         offset |= (*(insn + 1) & 0x0000ffff); // ori
         return reinterpret_cast<void **>(offset);
     }

    //hwj
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

} // namespace JSC

#endif // ENABLE(ASSEMBLER) && CPU(MIPS)
