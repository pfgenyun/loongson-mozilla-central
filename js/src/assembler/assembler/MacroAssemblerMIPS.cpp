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

#if ENABLE_ASSEMBLER && WTF_CPU_MIPS

#include "assembler/assembler/MacroAssemblerMIPS.h"

using namespace JSC;

MacroAssemblerMIPS::Call 
MacroAssemblerMIPS::callRel()
{
    m_assembler.nop();
    m_assembler.nop();
    m_assembler.bal(0);
    m_assembler.nop();
    return Call(m_assembler.newJmpSrc(), Call::Linkable);
}

static unsigned int* __getpc(void)
{
    unsigned int *rtaddr;
    
    __asm__ volatile ("move %0, $31" : "=r"(rtaddr));
    
    return rtaddr;
}

void
MacroAssemblerMIPS::offsetFromPCToV0(int offset){
    unsigned lw, hg;
    hg = ((unsigned int)__getpc)>>16;
    lw = ((unsigned int)__getpc)&0xffff;
    
    
    m_assembler.lui(MIPSRegisters::t9, hg);
    m_assembler.ori(MIPSRegisters::t9, MIPSRegisters::t9, lw);
    m_assembler.jalr(MIPSRegisters::t9);
    m_assembler.nop();
    m_assembler.addiu(MIPSRegisters::v0, MIPSRegisters::v0, offset);
}

//author:huangwenjun date:2013-12-13
void
MacroAssemblerMIPS::skipOffsetFromPCToV0(int offset){
    unsigned lw, hg;
    hg = ((unsigned int)__getpc)>>16;
    lw = ((unsigned int)__getpc)&0xffff;
    
    
    m_assembler.lui(MIPSRegisters::t9, hg);
    m_assembler.ori(MIPSRegisters::t9, MIPSRegisters::t9, lw);
    m_assembler.beq(MIPSRegisters::r0, MIPSRegisters::r0, offset);//below this instruction would not execute
    m_assembler.nop();
    m_assembler.addiu(MIPSRegisters::v0, MIPSRegisters::v0, offset);
}

#endif /* ENABLE_ASSEMBLER && WTF_CPU_MIPS */
