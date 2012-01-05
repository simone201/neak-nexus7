/*
 *	vmlinux.lds.S -- master linker script for m68knommu arch
 *
 *	(C) Copyright 2002-2012, Greg Ungerer <gerg@snapgear.com>
 *
 *	This linker script is equipped to build either ROM loaded or RAM
 *	run kernels.
 */

#if defined(CONFIG_RAMKERNEL)
#define	KTEXT_ADDR	CONFIG_KERNELBASE
#endif
#if defined(CONFIG_ROMKERNEL)
#define	KTEXT_ADDR	CONFIG_ROMSTART
#define	KDATA_ADDR	CONFIG_KERNELBASE
#define	LOAD_OFFSET	KDATA_ADDR + (ADDR(.text) + SIZEOF(.text))
#endif

#include <asm/page.h>
#include <asm/thread_info.h>
#include <asm-generic/vmlinux.lds.h>

OUTPUT_ARCH(m68k)
ENTRY(_start)

jiffies = jiffies_64 + 4;

SECTIONS {

#ifdef CONFIG_ROMVEC
	. = CONFIG_ROMVEC;
	.romvec : {
		__rom_start = .;
		_romvec = .;
		*(.romvec)
		*(.data..initvect)
	}
#endif

	. = KTEXT_ADDR;

	_text = .;
	_stext = .;
	.text : {
		HEAD_TEXT
		TEXT_TEXT
		SCHED_TEXT
		LOCK_TEXT
		*(.fixup)
		. = ALIGN(16);
	}
	_etext = .;

#ifdef KDATA_ADDR
	. = KDATA_ADDR;
#endif

	_sdata = .;
	RO_DATA_SECTION(PAGE_SIZE)
	RW_DATA_SECTION(16, PAGE_SIZE, THREAD_SIZE)
	_edata = .;

<<<<<<< HEAD
		. = ALIGN(16);          /* Exception table              */
		__start___ex_table = .;
		*(__ex_table)
		__stop___ex_table = .;

		*(.rodata) *(.rodata.*)
		*(__vermagic)		/* Kernel version magic */
		*(__markers_strings)
		*(.rodata1)
		*(.rodata.str1.1)

		/* Kernel symbol table: Normal symbols */
		. = ALIGN(4);
		__start___ksymtab = .;
		*(SORT(___ksymtab+*))
		__stop___ksymtab = .;

		/* Kernel symbol table: GPL-only symbols */
		__start___ksymtab_gpl = .;
		*(SORT(___ksymtab_gpl+*))
		__stop___ksymtab_gpl = .;

		/* Kernel symbol table: Normal unused symbols */
		__start___ksymtab_unused = .;
		*(SORT(___ksymtab_unused+*))
		__stop___ksymtab_unused = .;

		/* Kernel symbol table: GPL-only unused symbols */
		__start___ksymtab_unused_gpl = .;
		*(SORT(___ksymtab_unused_gpl+*))
		__stop___ksymtab_unused_gpl = .;

		/* Kernel symbol table: GPL-future symbols */
		__start___ksymtab_gpl_future = .;
		*(SORT(___ksymtab_gpl_future+*))
		__stop___ksymtab_gpl_future = .;

		/* Kernel symbol table: Normal symbols */
		__start___kcrctab = .;
		*(SORT(___kcrctab+*))
		__stop___kcrctab = .;

		/* Kernel symbol table: GPL-only symbols */
		__start___kcrctab_gpl = .;
		*(SORT(___kcrctab_gpl+*))
		__stop___kcrctab_gpl = .;

		/* Kernel symbol table: Normal unused symbols */
		__start___kcrctab_unused = .;
		*(SORT(___kcrctab_unused+*))
		__stop___kcrctab_unused = .;

		/* Kernel symbol table: GPL-only unused symbols */
		__start___kcrctab_unused_gpl = .;
		*(SORT(___kcrctab_unused_gpl+*))
		__stop___kcrctab_unused_gpl = .;

		/* Kernel symbol table: GPL-future symbols */
		__start___kcrctab_gpl_future = .;
		*(SORT(___kcrctab_gpl_future+*))
		__stop___kcrctab_gpl_future = .;

		/* Kernel symbol table: strings */
		*(__ksymtab_strings)

		/* Built-in module parameters */
		. = ALIGN(4) ;
		__start___param = .;
		*(__param)
		__stop___param = .;

		/* Built-in module versions */
		. = ALIGN(4) ;
		__start___modver = .;
		*(__modver)
		__stop___modver = .;

		. = ALIGN(4) ;
		_etext = . ;
	} > TEXT

	.data DATA_ADDR : {
		. = ALIGN(4);
		_sdata = . ;
		DATA_DATA
		CACHELINE_ALIGNED_DATA(32)
		PAGE_ALIGNED_DATA(PAGE_SIZE)
		*(.data..shared_aligned)
		INIT_TASK_DATA(THREAD_SIZE)
		_edata = . ;
	} > DATA
=======
	EXCEPTION_TABLE(16)
	NOTES
>>>>>>> f362f08... m68knommu: clean up linker script

	. = ALIGN(PAGE_SIZE);
	__init_begin = .;
	INIT_TEXT_SECTION(PAGE_SIZE)
	INIT_DATA_SECTION(16)
	PERCPU_SECTION(16)
	.m68k_fixup : {
		__start_fixup = .;
		*(.m68k_fixup)
		__stop_fixup = .;
	}
	.init.data : {
		. = ALIGN(PAGE_SIZE);
		__init_end = .;
	}

	_sbss = .;
	BSS_SECTION(0, 0, 0)
	_ebss = .;

	_end = .;

	STABS_DEBUG
	.comment 0 : { *(.comment) }

	/* Sections to be discarded */
	DISCARDS
}

