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
 * System clock driver using the timer section of a Motorola MC68230
 * Parallel Interface/Timer (PI/T).
 *
 * The PI/T timer is a 24-bit down-counter that, in the mode we use,
 * reloads itself from the Counter Preload Registers (CPR) on reaching
 * zero and asserts TOUT as an interrupt request.  We therefore program
 * it once in clk_arm() and only have to acknowledge it in clk_intr().
 *
 * Device tree binding (see sbc3.dts):
 *
 *	compatible	"motorola,mc68230-timer"
 *	reg		PI/T base address (the address of PGCR)
 *	reg-shift	log2 of the register spacing (default 1, i.e. the
 *			PI/T sits on every other byte, as on an 8-bit
 *			odd-byte bus connection)
 *	clocks		the clock feeding the counter source: the CLK pin
 *			for clock modes 0/1, the TIN pin for modes 2/3
 *	motorola,clock-mode
 *			TCR bits 2-1 (default 0):
 *			  0 = CLK/32, TIN unused
 *			  1 = CLK/32, gated by TIN
 *			  2 = TIN/32
 *			  3 = TIN, no prescaler
 *	interrupts	<ipl vector>; vector 0 selects auto-vectored
 *			interrupts (TCR 111), non-zero selects vectored
 *			interrupts via PC7/TIACK and TIVR (TCR 101).
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>

#include <machine/clockvar.h>

#include <dev/fdt/fdtvar.h>

/* PI/T timer register numbers (multiplied by the register spacing). */
#define	PIT_TCR		0x10	/* timer control */
#define	PIT_TIVR	0x11	/* timer interrupt vector */
#define	PIT_CPRH	0x13	/* counter preload, high */
#define	PIT_CPRM	0x14	/* counter preload, middle */
#define	PIT_CPRL	0x15	/* counter preload, low */
#define	PIT_CNTRH	0x17	/* count, high */
#define	PIT_CNTRM	0x18	/* count, middle */
#define	PIT_CNTRL	0x19	/* count, low */
#define	PIT_TSR		0x1a	/* timer status */

/* TCR bits */
#define	TCR_TOUT_VEC_IRQ	(5U << 5) /* TOUT=irq, PC7=TIACK, enabled */
#define	TCR_TOUT_AUTO_IRQ	(7U << 5) /* TOUT=irq, PC7=port, enabled */
#define	TCR_ZD_ROLLOVER		__BIT(4)  /* 0 = reload from CPR on zero */
#define	TCR_CLK_MODE		__BITS(2,1)
#define	TCR_ENABLE		__BIT(0)

/* TSR bits */
#define	TSR_ZDS			__BIT(0)  /* zero detect; write 1 to clear */

#define	PIT_COUNT_MAX		0x00ffffffU

struct pittmr_softc {
	device_t	sc_dev;
	bus_space_tag_t	sc_st;
	bus_space_handle_t sc_sh;
	u_int		sc_shift;
	void		*sc_ih;
	uint32_t	sc_freq;	/* counter decrement rate in Hz */
	uint8_t		sc_tcr;		/* TCR value, minus the enable bit */
	uint8_t		sc_vector;	/* 0 = auto-vectored */
	struct clock_if sc_clkif;
};

#define	PIT_READ(sc, r)							\
	bus_space_read_1((sc)->sc_st, (sc)->sc_sh, (r) << (sc)->sc_shift)
#define	PIT_WRITE(sc, r, v)						\
	bus_space_write_1((sc)->sc_st, (sc)->sc_sh, (r) << (sc)->sc_shift, (v))

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "motorola,mc68230-timer" },
	DEVICE_COMPAT_EOL
};

static uint32_t
pittmr_us_to_ticks(struct pittmr_softc *sc, unsigned int interval_us)
{
	const uint64_t ticks =
	    ((uint64_t)sc->sc_freq * interval_us + 500000) / 1000000;

	if (ticks == 0 || ticks > PIT_COUNT_MAX) {
		panic("%s: impossible interval %u us (ticks=%llu)", __func__,
		    interval_us, (unsigned long long)ticks);
	}

	return (uint32_t)ticks;
}

static void
pittmr_stop(struct pittmr_softc *sc)
{
	PIT_WRITE(sc, PIT_TCR, sc->sc_tcr & ~TCR_ENABLE);
	PIT_WRITE(sc, PIT_TSR, TSR_ZDS);
}

static int
pittmr_init(struct clock_if *clk, int (*intrfunc)(void *))
{
	struct pittmr_softc *sc =
	    container_of(clk, struct pittmr_softc, sc_clkif);
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
pittmr_arm(struct clock_if *clk, unsigned int interval_us)
{
	struct pittmr_softc *sc =
	    container_of(clk, struct pittmr_softc, sc_clkif);
	const uint32_t ticks = pittmr_us_to_ticks(sc, interval_us);

	pittmr_stop(sc);

	PIT_WRITE(sc, PIT_CPRH, (ticks >> 16) & 0xff);
	PIT_WRITE(sc, PIT_CPRM, (ticks >> 8) & 0xff);
	PIT_WRITE(sc, PIT_CPRL, ticks & 0xff);

	/* Enabling the timer loads the counter from CPR. */
	PIT_WRITE(sc, PIT_TCR, sc->sc_tcr | TCR_ENABLE);

	return 0;
}

static void
pittmr_intr(struct clock_if *clk)
{
	struct pittmr_softc *sc =
	    container_of(clk, struct pittmr_softc, sc_clkif);

	/*
	 * The counter has already reloaded itself from CPR; we only
	 * need to clear the zero-detect status to negate TOUT.
	 */
	PIT_WRITE(sc, PIT_TSR, TSR_ZDS);
}

static void
pittmr_disarm(struct clock_if *clk)
{
	struct pittmr_softc *sc =
	    container_of(clk, struct pittmr_softc, sc_clkif);

	pittmr_stop(sc);
}

static int
pittmr_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
pittmr_attach(device_t parent, device_t self, void *aux)
{
	struct pittmr_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	u_int specifier[2];
	bus_addr_t addr;
	bus_size_t size;
	uint32_t mode;
	struct clk *clk;
	int error, len;

	sc->sc_dev = self;
	sc->sc_st = faa->faa_bst;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	if (of_getprop_uint32(phandle, "reg-shift", &sc->sc_shift) != 0) {
		sc->sc_shift = 1;
	}

	error = bus_space_map(sc->sc_st, addr, size, 0, &sc->sc_sh);
	if (error) {
		aprint_error(": couldn't map registers (error=%d)\n", error);
		return;
	}

	if (of_getprop_uint32(phandle, "motorola,clock-mode", &mode) != 0) {
		mode = 0;
	}
	if (mode > 3) {
		aprint_error(": invalid clock mode %u\n", mode);
		return;
	}

	clk = fdtbus_clock_get_index(phandle, 0);
	if (clk == NULL) {
		aprint_error(": couldn't get clock handle\n");
		return;
	}
	sc->sc_freq = clk_get_rate(clk);
	if (mode != 3) {
		sc->sc_freq /= 32;	/* prescaler in use */
	}
	if (sc->sc_freq == 0) {
		aprint_error(": invalid clock rate\n");
		return;
	}

	/* Is the timer interrupt vectored or auto-vectored? */
	len = OF_getprop(phandle, "interrupts", specifier, sizeof(specifier));
	if (len != sizeof(specifier)) {
		aprint_error(": couldn't get interrupt specifier\n");
		return;
	}
	sc->sc_vector = (uint8_t)be32toh(specifier[1]);

	sc->sc_tcr = __SHIFTIN(mode, TCR_CLK_MODE);
	sc->sc_tcr |= sc->sc_vector ? TCR_TOUT_VEC_IRQ : TCR_TOUT_AUTO_IRQ;

	/* Make sure the timer is stopped and quiet. */
	pittmr_stop(sc);
	if (sc->sc_vector) {
		PIT_WRITE(sc, PIT_TIVR, sc->sc_vector);
	}

	aprint_naive("\n");
	aprint_normal(": MC68230 timer, %u Hz count rate\n", sc->sc_freq);

	sc->sc_clkif.clk_dev = self;
	sc->sc_clkif.clk_init = pittmr_init;
	sc->sc_clkif.clk_arm = pittmr_arm;
	sc->sc_clkif.clk_intr = pittmr_intr;
	sc->sc_clkif.clk_disarm = pittmr_disarm;
	clock_attach(&sc->sc_clkif);
}

CFATTACH_DECL_NEW(pittmr, sizeof(struct pittmr_softc),
    pittmr_match, pittmr_attach, NULL, NULL);
