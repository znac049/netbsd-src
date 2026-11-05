/*	$NetBSD: pit_timer.c,v 1.2 2026/08/07 23:37:46 thorpej Exp $	*/

/*-
 * Copyright (c) 2025 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe.
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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: pit_timer.c,v 1.2 2026/08/07 23:37:46 thorpej Exp $");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>

#include <machine/clockvar.h>

#include <dev/fdt/fdtvar.h>

struct pit_timer_softc {
	device_t	sc_dev;
	bus_space_tag_t	sc_st;
	bus_space_handle_t sc_sh;
	uint32_t	sc_reg_shift;
	void		*sc_ih;
	uint32_t	sc_freq;
	uint8_t		sc_vector;
	struct clock_if sc_clkif;
};

// Registers in the mc68230 PI/T
#define PIT_TCR   		0x10
#define   TCR_ENABLE  		0x01
#define   TCR_TOUT_IS_IRQ 	0xa0
#define   TCR_TOUT_IRQ_OFF	0x80
#define PIT_TIVR		0x11
#define PIT_CPRH  		0x13
#define PIT_CPRM  		0x14
#define PIT_CPRL  		0x15
#define PIT_TSR   		0x1a
#define   TSR_ZDFLAG		0x01



#define	REG_OFF(sc, r)		((r) << (sc)->sc_reg_shift)

#define	REG_READ(sc, r)		\
	bus_space_read_1((sc)->sc_st, (sc)->sc_sh, REG_OFF((sc), (r)))
#define	REG_WRITE(sc, r, v)	\
	bus_space_write_1((sc)->sc_st, (sc)->sc_sh, REG_OFF((sc), (r)), (v))

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "znac049,mc68230-timer" },
	DEVICE_COMPAT_EOL
};

static uint32_t
pit_timer_us_to_ticks(struct pit_timer_softc *sc, unsigned int interval_us)
{
	const uint64_t ticks =
	    ((uint64_t)sc->sc_freq * interval_us + 500000) / 1000000;

	if (ticks == 0 || ticks > 0xffffff) {
		panic("%s: impossible interval %u us (ticks=%llu)", __func__,
		    interval_us, ticks);
	}

	return (uint32_t)ticks - 1;
}

static int
pit_timer_init(struct clock_if *clk, int (*intrfunc)(void *))
{
	struct pit_timer_softc *sc =
	    container_of(clk, struct pit_timer_softc, sc_clkif);
	int phandle = devhandle_to_of(device_handle(sc->sc_dev));
	char intrstr[128];

	KASSERT(sc->sc_dev == clk->clk_dev);

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error_dev(sc->sc_dev, "failed to decode interrupt\n");
		return ENXIO;
	}

	sc->sc_ih = fdtbus_intr_establish_xname(phandle, 0, IPL_SCHED,
	    FDT_INTR_MPSAFE, intrfunc, NULL, device_xname(sc->sc_dev));
	if (sc->sc_ih == NULL) {
		aprint_error_dev(sc->sc_dev,
		    "failed to establish interrupt at %s\n", intrstr);
		return ENXIO;
	}
	aprint_normal_dev(sc->sc_dev, "interrupting at %s\n", intrstr);

	return 0;
}

static int
pit_timer_arm(struct clock_if *clk, unsigned int interval_us)
{
	struct pit_timer_softc *sc =
	    container_of(clk, struct pit_timer_softc, sc_clkif);
	const uint32_t ticks = pit_timer_us_to_ticks(sc, interval_us);

	REG_WRITE(sc, PIT_TCR, TCR_TOUT_IRQ_OFF);	// Stop - but leave as IRQ line
	REG_WRITE(sc, PIT_TSR, TSR_ZDFLAG);			// Clear Zero Detect - just in case

	REG_WRITE(sc, PIT_CPRH, (uint8_t)(ticks >> 16));
	REG_WRITE(sc, PIT_CPRM, (uint8_t)(ticks >> 8));
    REG_WRITE(sc, PIT_CPRL, (uint8_t)ticks);

	/*
	 * Configure the timer to:
	 * - generate vectored interrupts
	 * - reload from the preload register when we hit zero
	 * - use the external clock on the CLK pin, with prescaler (32)
	 */
	REG_WRITE(sc, PIT_TCR, TCR_TOUT_IS_IRQ | TCR_ENABLE);

	return 0;
}

static void
pit_timer_intr(struct clock_if *clk)
{
	struct pit_timer_softc *sc =
	    container_of(clk, struct pit_timer_softc, sc_clkif);

	REG_WRITE(sc, PIT_TSR, TSR_ZDFLAG);		// Clear the zero detect flag
}

static void
pit_timer_disarm(struct clock_if *clk)
{
	struct pit_timer_softc *sc =
	    container_of(clk, struct pit_timer_softc, sc_clkif);

	REG_WRITE(sc, PIT_TCR, (REG_READ(sc, PIT_TCR) & ~TCR_ENABLE));
	REG_WRITE(sc, PIT_TSR, TSR_ZDFLAG);
}

static int
pit_timer_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
pit_timer_attach(device_t parent, device_t self, void *aux)
{
	struct pit_timer_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	struct clk *clk;
	int error;
	int len;
	u_int int_inf[2];

	sc->sc_dev = self;
	sc->sc_st = faa->faa_bst;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	if (of_getprop_uint32(phandle, "reg-shift", &sc->sc_reg_shift)) {
		/* missing or bad reg-shift property, assume 0 */
		sc->sc_reg_shift = 0;
	}

	len = OF_getprop(phandle, "interrupts", int_inf, sizeof(int_inf));
	if (len != sizeof(int_inf)) {
		aprint_error(": no interrupt specifier\n");
		return;
	}
	sc->sc_vector = (uint8_t)be32toh(int_inf[1]);
	
	aprint_normal_dev(sc->sc_dev, "will set TIVR to %02x\n", sc->sc_vector);
	
	error = bus_space_map(sc->sc_st, addr, size, 0, &sc->sc_sh);
	if (error) {
		aprint_error(": couldn't map registers (error=%d)\n", error);
		return;
	}

	/* Make sure the timer is disabled. */
	REG_WRITE(sc, PIT_TCR, TCR_TOUT_IRQ_OFF);	// Stop - but leave as IRQ line
	REG_WRITE(sc, PIT_TSR, TSR_ZDFLAG);			// Clear Zero Detect - just in case

	if (sc->sc_vector) {
		REG_WRITE(sc, PIT_TIVR, sc->sc_vector);
	}

	clk = fdtbus_clock_get_index(phandle, 0);
	if (clk == NULL) {
		aprint_error(": couldn't get clock handle\n");
		return;
	}

	sc->sc_freq = clk_get_rate(clk) / 32;		// 32 is for the prescaler

	aprint_naive("\n");
	aprint_normal(": frequency %u Hz\n", sc->sc_freq);

	sc->sc_clkif.clk_dev = self;
	sc->sc_clkif.clk_init = pit_timer_init;
	sc->sc_clkif.clk_arm = pit_timer_arm;
	sc->sc_clkif.clk_intr = pit_timer_intr;
	sc->sc_clkif.clk_disarm = pit_timer_disarm;
	clock_attach(&sc->sc_clkif);
}

CFATTACH_DECL_NEW(pittmr, sizeof(struct pit_timer_softc),
    pit_timer_match, pit_timer_attach, NULL, NULL);
