/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/scripts/pg-isram.ld
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* The SAMA5D3 has 128 KB of ISRAM beginning at virtual address 0x0030:0000.
 * The .text vitual address space begins at the same location as the physical
 * address space:  0x0030:0000.  The virtual space is broken up into three
 * regions:
 *
 *                                           PHYSICAL         VIRTUAL
 *   NAME    DESCRIPTION                 SIZE    START    SIZE    START
 *  -------- -------------------------- ----- ----------- ------- -----------
 *   locked  Pages locked in memory.     40KB 0x0030:0000  40KB   0x0030:0000
 *   paged   Pages in nonvolatile store. 48KB 0x0030:a000  48KB   0x0030:a000
 *   data    .data/.bss/heap.            24KB 0x0031:6000  24KB   0x0031:6000
 *           Reserved for page table     16KB 0x0031:c000  16KB   0x0031:c000
 *  -------- -------------------------- ----- ----------- ------- -----------
 *                                      128KB 0x0032 0000 128KB   0x0032 0000
 *
 * These region sizes must match the size in pages specified for each region
 * in the NuttX configuration file:  CONFIG_PAGING_NLOCKED, CONFIG_PAGING_NVPAGED,
 * and CONFIG_PAGING_NDATA.
 *
 * NOTE 1: The locked region is should be big here so that you can enable lots of
 * debug output without overflowing the locked region.  32KB would probably be
 * plenty if this were a real, optimized application.
 *
 * NOTE 2: Different compilers will compile the code to different sizes.  If you
 * get a link time error saying that the locked region is full, you may have to
 * re-organize this memory layout (here and in defconfig) to make the locked
 * region even bigger.
 *
 * NOTE 3: Vectors in low memory are assumed and 16KB of ISRAM is reserved at
 * the high end of ISRAM for the page table (?).
 */

MEMORY
{
    locked (rx) : ORIGIN = 0x00300000, LENGTH = 40K
    paged  (rx) : ORIGIN = 0x0030a000, LENGTH = 48K
    data   (rw) : ORIGIN = 0x00316000, LENGTH = 24K
}

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(entry)
ENTRY(_stext)

SECTIONS
{
	.locked :
	{
		_slocked = ABSOLUTE(.);
		*(.vectors)
		up_head.o locked.r (.text .text.*)
		up_head.o locked.r (.fixup)
		up_head.o locked.r (.gnu.warning)
		up_head.o locked.r (.rodata .rodata.*)
		up_head.o locked.r (.gnu.linkonce.t.*)
		up_head.o locked.r (.glue_7)
		up_head.o locked.r (.glue_7t)
		up_head.o locked.r (.got)
		up_head.o locked.r (.gcc_except_table)
		up_head.o locked.r (.gnu.linkonce.r.*)
		_elocked = ABSOLUTE(.);
	} >locked

	.init_section :
	{
		_sinit = ABSOLUTE(.);
		*(.init_array .init_array.*)
		_einit = ABSOLUTE(.);
	} > locked

	/* .ARM.exidx is sorted, so has to go in its own output section.  */

	PROVIDE_HIDDEN (__exidx_start = .);
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > isram
	PROVIDE_HIDDEN (__exidx_end = .);

	.paged :
	{
		_spaged = ABSOLUTE(.);
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		*(.ARM.extab*)
		*(.gnu.linkonce.armextab.*)
		_epaged = ABSOLUTE(.);
	} > paged

	.data :
	{
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);
	} > data AT > locked

	.bss :
	{
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		_ebss = ABSOLUTE(.);
	} > data

	/* Uninitialized data */

	.noinit :
	{
		_snoinit = ABSOLUTE(.);
		*(.noinit*)
		_enoinit = ABSOLUTE(.);
	} > data

	/* Stabs debugging sections. */

	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}
