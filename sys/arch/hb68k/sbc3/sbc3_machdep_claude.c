/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 The NetBSD Foundation, Inc.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Platform support for SBC3, a homebrew 68030 single-board computer.
 *
 * Physical memory map:
 *
 *	$00000000 - $007FFFFF	RAM (8 MB); the first 8 bytes are
 *				overlaid by the ROM's reset vectors
 *	$00AD0000		XR68C681 DUART, 7.3728 MHz (odd bytes)
 *	$00AE0000		CompactFlash, 8-bit True IDE, polled
 *	$00AF0000		MC68230 PI/T (odd bytes)
 *	$00C00000 - $00FFFFFF	ROM (4 MB)
 *
 * Unlike Wrap030, the devices live in the same 16 MB as RAM, so they
 * can't be mapped with a 68030 Transparent Translation register (TT
 * registers match on A31-A24 only).  Instead we ask pmap_bootstrap1()
 * for a static, cache-inhibited page mapping of the I/O area.  This is
 * what lets the console be used before UVM is initialized:
 * bus_space_map() finds it via pmap_pa_has_static_mapping().
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/fdt_platform.h>

#include <uvm/uvm_extern.h>

/*
 * The I/O window: covers the DUART at $AD0000, the CompactFlash
 * interface at $AE0000 and the PI/T at $AF0000.  Grow this if you add devices elsewhere in $A00000-$BFFFFF.
 */
#define	SBC3_IO_BASE	0x00ad0000
#define	SBC3_IO_SIZE	0x00030000

/* Filled in by pmap_bootstrap1() with the VA of the I/O window. */
vaddr_t	sbc3_io_va;

/*
 * plat_bootstrap --
 *	Early bootstrap routine, mainly for the purpose of providing
 *	data necessary to configure the MMU.  Runs with the MMU off.
 */
static paddr_t
sbc3_plat_bootstrap(paddr_t nextpa, vaddr_t reloff)
{
	struct pmap_bootmap *pmbm =
	    (struct pmap_bootmap *)PMAP_BOOTSTRAP_RELOC_GLOB(machine_bootmap);

	/*
	 * Store the link-time address of sbc3_io_va; pmap_bootstrap1()
	 * applies the relocation itself when it fills it in.
	 */
	pmbm[0].pmbm_vaddr_ptr = &sbc3_io_va;
	pmbm[0].pmbm_paddr = SBC3_IO_BASE;
	pmbm[0].pmbm_size  = SBC3_IO_SIZE;
	pmbm[0].pmbm_flags = PMBM_F_CI;

	return nextpa;
}

/*
 * plat_uart_freq --
 *	Return the console UART clock frequency in Hz.  This is
 *	only used if the device tree doesn't say.
 */
static u_int
sbc3_plat_uart_freq(void)
{
	return 7372800;
}

static const struct fdt_platform sbc3_platform = {
	.fp_bootstrap = sbc3_plat_bootstrap,
	.fp_uart_freq = sbc3_plat_uart_freq,
};
FDT_PLATFORM(sbc3, "homebrew,sbc3", &sbc3_platform);
