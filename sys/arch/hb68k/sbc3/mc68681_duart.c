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
 * Driver for the Motorola MC68681 DUART, the register-compatible
 * Philips/NXP SCN2681 / SCC2681, and the Exar XR68C681, attached via
 * Flattened Device Tree.
 *
 * The XR68C681 adds a per-channel, per-direction "extend" bit (X),
 * set and cleared with CR commands $80-$B0, which selects a second
 * baud rate table with rates up to 115200 at 3.6864 MHz (230400 at
 * 7.3728 MHz).  Those commands are only issued to chips declared as
 * "exar,xr68c681"; a plain MC68681 never sees them.
 *
 * One device instance drives both channels of a DUART.  Minor numbers
 * are (unit << 1) | channel, so duart0 channel A is minor 0 and
 * channel B is minor 1.
 *
 * This is deliberately simple: no modem control (carrier is always
 * assumed present), no hardware flow control, and the receive FIFO
 * is drained into a ring buffer at interrupt time and handed to the
 * tty layer from a soft interrupt.
 *
 * Device tree binding (see sbc3.dts):
 *
 *	compatible	"motorola,mc68681", "nxp,scn2681" or
 *			"exar,xr68c681"
 *	reg		base address of the DUART (the address of MR1A)
 *	reg-shift	log2 of the register spacing (default 1, i.e. the
 *			DUART sits on every other byte)
 *	clock-frequency	X1/CLK input frequency (default 3686400); the
 *			baud rate tables scale with it, so e.g. 7.3728 MHz
 *			doubles every rate
 *	interrupts	<ipl vector>; vector 0 selects auto-vectored
 *			interrupts, non-zero programs IVR with the vector
 *	motorola,console-channel
 *			0 (channel A, default) or 1 (channel B); which
 *			channel stdout-path refers to
 *
 * Baud rates: the driver always selects baud rate set 1 (ACR[7] = 0),
 * because ACR is shared by both channels.  For each requested rate it
 * tries, in order:
 *
 *	1. the standard set 1 table (X = 0);
 *	2. on an XR68C681, the extended set 1 table (X = 1);
 *	3. the counter/timer as a 16x clock (preset >= 2).
 *
 * The counter/timer is shared, so only one rate can use it at a time
 * across both channels.
 *
 * Rates available with ACR[7] = 0:
 *
 *	MC68681 @ 3.6864 MHz	50-9600, 38400; C/T: 19200, 57600
 *	XR68C681 @ 3.6864 MHz	adds 14400, 19200, 28800, 57600, 115200
 *	XR68C681 @ 7.3728 MHz	300-230400, including 9600, 19200,
 *				38400, 57600, 115200, 230400
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/fcntl.h>
#include <sys/intr.h>
#include <sys/kauth.h>
#include <sys/poll.h>
#include <sys/proc.h>
#include <sys/syslog.h>
#include <sys/systm.h>
#include <sys/tty.h>

#include <dev/cons.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/fdt_console.h>

/*
 * Register numbers.  The actual offset is (reg << shift).
 * Channel registers are given for channel A; add DU_CHAN_STRIDE
 * for channel B.
 */
#define	DU_MR		0	/* rw: MR1x / MR2x (via MR pointer) */
#define	DU_SR		1	/* ro: status */
#define	DU_CSR		1	/* wo: clock select */
#define	DU_CR		2	/* wo: command */
#define	DU_RHR		3	/* ro: receive holding */
#define	DU_THR		3	/* wo: transmit holding */
#define	DU_IPCR		4	/* ro: input port change */
#define	DU_ACR		4	/* wo: auxiliary control */
#define	DU_ISR		5	/* ro: interrupt status */
#define	DU_IMR		5	/* wo: interrupt mask */
#define	DU_CTUR		6	/* wo: counter/timer upper */
#define	DU_CTLR		7	/* wo: counter/timer lower */
#define	DU_IVR		12	/* rw: interrupt vector */
#define	DU_OPCR		13	/* wo: output port configuration */
#define	DU_START_CT	14	/* ro: start counter command */
#define	DU_STOP_CT	15	/* ro: stop counter command */

#define	DU_CHAN_STRIDE	8

/* SR bits */
#define	SR_RXRDY	0x01
#define	SR_FFULL	0x02
#define	SR_TXRDY	0x04
#define	SR_TXEMT	0x08
#define	SR_OE		0x10
#define	SR_PE		0x20
#define	SR_FE		0x40
#define	SR_RB		0x80	/* received break */

/* CR commands */
#define	CR_RX_ENABLE	0x01
#define	CR_RX_DISABLE	0x02
#define	CR_TX_ENABLE	0x04
#define	CR_TX_DISABLE	0x08
#define	CR_RESET_MRPTR	0x10
#define	CR_RESET_RX	0x20
#define	CR_RESET_TX	0x30
#define	CR_RESET_ERR	0x40
#define	CR_RESET_BRKINT	0x50
#define	CR_START_BRK	0x60
#define	CR_STOP_BRK	0x70

/* XR68C681-only CR commands */
#define	CR_XR_SET_RX_X	0x80	/* set Rx BRG select extend bit */
#define	CR_XR_CLR_RX_X	0x90	/* clear Rx BRG select extend bit */
#define	CR_XR_SET_TX_X	0xa0	/* set Tx BRG select extend bit */
#define	CR_XR_CLR_TX_X	0xb0	/* clear Tx BRG select extend bit */

/* MR1 bits */
#define	MR1_PAR_WITH	0x00
#define	MR1_PAR_NONE	0x10
#define	MR1_PAR_ODD	0x04
#define	MR1_BITS_5	0x00
#define	MR1_BITS_6	0x01
#define	MR1_BITS_7	0x02
#define	MR1_BITS_8	0x03

/* MR2 bits */
#define	MR2_STOP_1	0x07
#define	MR2_STOP_2	0x0f

/* ACR: baud rate set 1, counter/timer in timer mode clocked by X1/CLK */
#define	ACR_DEFAULT	0x60

/* CSR clock source code meaning "use the counter/timer" */
#define	CSR_CODE_CT	0x0d

/* ISR / IMR bits (channel B bits are channel A bits << 4) */
#define	ISR_TXRDY	0x01
#define	ISR_RXRDY	0x02
#define	ISR_DBRK	0x04
#define	ISR_CHAN_SHIFT	4

#define	DUART_DEFAULT_FREQ	3686400
#define	DUART_DEFAULT_SPEED	9600
#define	DUART_RBUF_SIZE		256	/* power of 2 */

#define	DUART_UNIT(dev)		(TTUNIT(dev) >> 1)
#define	DUART_CHAN(dev)		(TTUNIT(dev) & 1)
#define	DUART_DIALOUT(dev)	TTDIALOUT(dev)
#define	DUART_MINOR(u, c)	(((u) << 1) | (c))

struct duart_softc;

struct duart_chan {
	struct duart_softc *ch_sc;
	int		ch_num;
	struct tty	*ch_tty;
	bool		ch_console;
	dev_t		ch_cndev;

	/* Receive ring: (SR << 8) | data. */
	uint16_t	ch_rbuf[DUART_RBUF_SIZE];
	u_int		ch_rbput;
	u_int		ch_rbget;
	u_int		ch_rbovf;

	/* Transmit state. */
	u_char		*ch_tba;
	int		ch_tbc;

	/* Flags for the soft interrupt. */
	volatile bool	ch_rx_ready;
	volatile bool	ch_tx_done;

	bool		ch_uses_ct;
};

struct duart_softc {
	device_t	sc_dev;
	bus_space_tag_t	sc_bst;
	bus_space_handle_t sc_bsh;
	u_int		sc_shift;
	const struct duart_variant *sc_variant;
	u_int		sc_freq;
	kmutex_t	sc_lock;
	void		*sc_ih;
	void		*sc_si;
	uint8_t		sc_imr;		/* IMR is write-only */
	u_int		sc_ct_rate;	/* baud rate the C/T is generating */
	u_int		sc_ct_users;
	struct duart_chan sc_chan[2];
};

static int	duart_match(device_t, cfdata_t, void *);
static void	duart_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(duart, sizeof(struct duart_softc),
    duart_match, duart_attach, NULL, NULL);

extern struct cfdriver duart_cd;

static dev_type_open(duartopen);
static dev_type_close(duartclose);
static dev_type_read(duartread);
static dev_type_write(duartwrite);
static dev_type_ioctl(duartioctl);
static dev_type_stop(duartstop);
static dev_type_tty(duarttty);
static dev_type_poll(duartpoll);

const struct cdevsw duart_cdevsw = {
	.d_open = duartopen,
	.d_close = duartclose,
	.d_read = duartread,
	.d_write = duartwrite,
	.d_ioctl = duartioctl,
	.d_stop = duartstop,
	.d_tty = duarttty,
	.d_poll = duartpoll,
	.d_mmap = nommap,
	.d_kqfilter = ttykqfilter,
	.d_discard = nodiscard,
	.d_flag = D_TTY
};

static void	duartstart(struct tty *);
static int	duartparam(struct tty *, struct termios *);
static int	duart_intr(void *);
static void	duart_soft(void *);

struct duart_variant;

/*
 * Console state, set up by duart_console_consinit() long before
 * autoconfiguration.
 */
static struct {
	const struct duart_variant *variant;
	bus_space_tag_t	bst;
	bus_space_handle_t bsh;
	bus_addr_t	addr;
	u_int		shift;
	int		chan;
	u_int		freq;
	int		speed;
	tcflag_t	cflag;
	u_int		ct_rate;
	bool		active;
} duart_cons;

static cnm_state_t duart_cnm_state;

/*
 * Register access helpers usable with or without a softc.
 */
static inline uint8_t
du_read(bus_space_tag_t bst, bus_space_handle_t bsh, u_int shift, u_int reg)
{
	return bus_space_read_1(bst, bsh, reg << shift);
}

static inline void
du_write(bus_space_tag_t bst, bus_space_handle_t bsh, u_int shift, u_int reg,
    uint8_t val)
{
	bus_space_write_1(bst, bsh, reg << shift, val);
}

#define	CH_REG(chan, reg)	((reg) + (chan) * DU_CHAN_STRIDE)

#define	SC_READ(sc, reg)						\
	du_read((sc)->sc_bst, (sc)->sc_bsh, (sc)->sc_shift, (reg))
#define	SC_WRITE(sc, reg, val)						\
	du_write((sc)->sc_bst, (sc)->sc_bsh, (sc)->sc_shift, (reg), (val))

/*
 * Baud rate generator tables for ACR[7] = 0, as given in the data
 * sheets for a 3.6864 MHz X1 clock.  The internal BRG divides X1, so
 * the real rate is table * freq / 3686400.  Index is the CSR nibble;
 * 0 means "not a BRG rate" (codes $D-$F are C/T and external clocks).
 */
#define	DUART_BRG_REF_FREQ	3686400

static const u_int duart_brg_set1[16] = {	/* X = 0, any chip */
	50, 110, 134, 200, 300, 600, 1200, 1050,
	2400, 4800, 7200, 9600, 38400, 0, 0, 0
};

static const u_int duart_brg_set1_x[16] = {	/* X = 1, XR68C681 only */
	75, 110, 134, 150, 3600, 14400, 28800, 57600,
	115200, 4800, 1800, 9600, 19200, 0, 0, 0
};

/*
 * Chip variants, selected by the "compatible" property.
 */
struct duart_variant {
	const char	*dv_name;
	bool		dv_has_xbit;	/* XR68C681 extended BRG select */
};

static const struct duart_variant duart_variant_mc68681 = {
	.dv_name = "MC68681",
	.dv_has_xbit = false,
};

static const struct duart_variant duart_variant_xr68c681 = {
	.dv_name = "XR68C681",
	.dv_has_xbit = true,
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "motorola,mc68681",	.data = &duart_variant_mc68681 },
	{ .compat = "nxp,scn2681",	.data = &duart_variant_mc68681 },
	{ .compat = "exar,xr68c681",	.data = &duart_variant_xr68c681 },
	DEVICE_COMPAT_EOL
};

static const struct duart_variant *
duart_lookup_variant(int phandle)
{
	const struct device_compatible_entry *dce =
	    of_compatible_lookup(phandle, compat_data);

	return dce != NULL ? dce->data : &duart_variant_mc68681;
}

/*
 * How to generate a particular baud rate.
 */
struct duart_brg {
	uint8_t		db_code;	/* CSR nibble */
	bool		db_xbit;	/* XR68C681 extend bit for this rate */
	u_int		db_ct_div;	/* C/T preset, or 0 if C/T not used */
};

static bool
duart_brg_lookup(const u_int *table, u_int freq, u_int speed,
    uint8_t *codep)
{
	for (u_int code = 0; code < 16; code++) {
		if (table[code] == 0) {
			continue;
		}
		const uint64_t actual =
		    ((uint64_t)table[code] * freq + DUART_BRG_REF_FREQ / 2) /
		    DUART_BRG_REF_FREQ;
		const uint64_t err =
		    actual > speed ? actual - speed : speed - actual;
		/* Within 1%; the tables' "134" is really 134.5. */
		if (err * 100 <= speed) {
			*codep = (uint8_t)code;
			return true;
		}
	}
	return false;
}

/*
 * Work out how to generate a baud rate.  Returns false if the rate
 * cannot be generated.
 */
static bool
duart_speed_code(const struct duart_variant *dv, u_int freq, u_int speed,
    struct duart_brg *brg)
{
	memset(brg, 0, sizeof(*brg));

	if (speed == 0 || freq == 0) {
		return false;
	}

	/* 1. Standard baud rate set 1. */
	if (duart_brg_lookup(duart_brg_set1, freq, speed, &brg->db_code)) {
		return true;
	}

	/* 2. XR68C681 extended baud rate set 1. */
	if (dv->dv_has_xbit &&
	    duart_brg_lookup(duart_brg_set1_x, freq, speed, &brg->db_code)) {
		brg->db_xbit = true;
		return true;
	}

	/*
	 * 3. Counter/timer in timer mode, clocked by X1: the square wave
	 * output is X1 / (2 * N), used as a 16x clock, so the baud rate
	 * is X1 / (32 * N).  The C/T preset must be at least 2 in timer
	 * mode.  Accept up to 3% error.
	 */
	const u_int div = (freq + 16 * speed) / (32 * speed);
	if (div < 2 || div > 0xffff) {
		return false;
	}
	const u_int actual = freq / (32 * div);
	const u_int err = actual > speed ? actual - speed : speed - actual;
	if (err * 100 > speed * 3) {
		return false;
	}
	brg->db_code = CSR_CODE_CT;
	brg->db_ct_div = div;
	return true;
}

static void
duart_program_ct(bus_space_tag_t bst, bus_space_handle_t bsh, u_int shift,
    u_int div)
{
	du_write(bst, bsh, shift, DU_CTUR, (div >> 8) & 0xff);
	du_write(bst, bsh, shift, DU_CTLR, div & 0xff);
	/* Reading the start register (re)starts the C/T with the preset. */
	(void)du_read(bst, bsh, shift, DU_START_CT);
}

static void
duart_cflag_to_mr(tcflag_t cflag, uint8_t *mr1p, uint8_t *mr2p)
{
	uint8_t mr1, mr2;

	switch (ISSET(cflag, CSIZE)) {
	case CS5:
		mr1 = MR1_BITS_5;
		break;
	case CS6:
		mr1 = MR1_BITS_6;
		break;
	case CS7:
		mr1 = MR1_BITS_7;
		break;
	default:
		mr1 = MR1_BITS_8;
		break;
	}

	if (ISSET(cflag, PARENB)) {
		mr1 |= MR1_PAR_WITH;
		if (ISSET(cflag, PARODD)) {
			mr1 |= MR1_PAR_ODD;
		}
	} else {
		mr1 |= MR1_PAR_NONE;
	}

	mr2 = ISSET(cflag, CSTOPB) ? MR2_STOP_2 : MR2_STOP_1;

	*mr1p = mr1;
	*mr2p = mr2;
}

/*
 * The XR68C681 needs at least three X1 edges between consecutive
 * writes to the same register.  A dummy read of the status register
 * costs at least one DUART bus cycle, which is comfortably longer.
 */
static inline void
du_write_settle(bus_space_tag_t bst, bus_space_handle_t bsh, u_int shift,
    int chan, u_int reg, uint8_t val)
{
	du_write(bst, bsh, shift, CH_REG(chan, reg), val);
	(void)du_read(bst, bsh, shift, CH_REG(chan, DU_SR));
}

static void
duart_program_chan(const struct duart_variant *dv, bus_space_tag_t bst,
    bus_space_handle_t bsh, u_int shift, int chan, uint8_t mr1, uint8_t mr2,
    const struct duart_brg *brg)
{
	du_write_settle(bst, bsh, shift, chan, DU_CR, CR_RESET_MRPTR);
	du_write_settle(bst, bsh, shift, chan, DU_MR, mr1);
	du_write_settle(bst, bsh, shift, chan, DU_MR, mr2);

	if (dv->dv_has_xbit) {
		/* Same rate both ways, so Rx and Tx X bits match. */
		du_write_settle(bst, bsh, shift, chan, DU_CR,
		    brg->db_xbit ? CR_XR_SET_RX_X : CR_XR_CLR_RX_X);
		du_write_settle(bst, bsh, shift, chan, DU_CR,
		    brg->db_xbit ? CR_XR_SET_TX_X : CR_XR_CLR_TX_X);
	}

	du_write_settle(bst, bsh, shift, chan, DU_CSR,
	    (uint8_t)((brg->db_code << 4) | brg->db_code));
	du_write_settle(bst, bsh, shift, chan, DU_CR,
	    CR_RX_ENABLE | CR_TX_ENABLE);
}

/*
 * Console routines.
 */

static int
duart_cngetc(dev_t dev)
{
	const u_int sr = CH_REG(duart_cons.chan, DU_SR);
	const u_int rhr = CH_REG(duart_cons.chan, DU_RHR);
	int s, c;

	s = splhigh();
	while ((du_read(duart_cons.bst, duart_cons.bsh, duart_cons.shift,
			sr) & SR_RXRDY) == 0) {
		continue;
	}
	c = du_read(duart_cons.bst, duart_cons.bsh, duart_cons.shift, rhr);
	splx(s);

	return c;
}

static void
duart_cnputc(dev_t dev, int c)
{
	const u_int sr = CH_REG(duart_cons.chan, DU_SR);
	const u_int thr = CH_REG(duart_cons.chan, DU_THR);
	int s;

	s = splhigh();
	while ((du_read(duart_cons.bst, duart_cons.bsh, duart_cons.shift,
			sr) & SR_TXRDY) == 0) {
		continue;
	}
	du_write(duart_cons.bst, duart_cons.bsh, duart_cons.shift, thr, c);
	splx(s);
}

static void
duart_cnpollc(dev_t dev, int on)
{
}

static struct consdev duart_consdev = {
	.cn_getc = duart_cngetc,
	.cn_putc = duart_cnputc,
	.cn_pollc = duart_cnpollc,
	.cn_dev = NODEV,
	.cn_pri = CN_NORMAL,
};

static int
duart_console_match(int phandle)
{
	return of_compatible_match(phandle, compat_data);
}

static void
duart_console_consinit(struct fdt_attach_args *faa, u_int uart_freq)
{
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	uint32_t val;
	struct duart_brg brg;
	uint8_t mr1, mr2;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		return;
	}

	duart_cons.variant = duart_lookup_variant(phandle);
	duart_cons.bst = faa->faa_bst;
	duart_cons.addr = addr;
	duart_cons.shift =
	    of_getprop_uint32(phandle, "reg-shift", &val) == 0 ? val : 1;
	duart_cons.chan =
	    of_getprop_uint32(phandle, "motorola,console-channel", &val) == 0
	    ? (val & 1) : 0;
	duart_cons.freq = uart_freq ? uart_freq : DUART_DEFAULT_FREQ;

	duart_cons.speed = fdtbus_get_stdout_speed();
	if (duart_cons.speed < 0) {
		duart_cons.speed = DUART_DEFAULT_SPEED;
	}
	duart_cons.cflag = fdtbus_get_stdout_flags();

	/*
	 * This must hit a static mapping; UVM isn't up yet.  The
	 * platform code provides one via machine_bootmap[].
	 */
	if (bus_space_map(duart_cons.bst, addr, size, 0,
			  &duart_cons.bsh) != 0) {
		return;
	}

	if (!duart_speed_code(duart_cons.variant, duart_cons.freq,
			      duart_cons.speed, &brg)) {
		duart_cons.speed = DUART_DEFAULT_SPEED;
		if (!duart_speed_code(duart_cons.variant, duart_cons.freq,
				      duart_cons.speed, &brg)) {
			return;
		}
	}

	du_write(duart_cons.bst, duart_cons.bsh, duart_cons.shift, DU_ACR,
	    ACR_DEFAULT);
	if (brg.db_ct_div != 0) {
		duart_program_ct(duart_cons.bst, duart_cons.bsh,
		    duart_cons.shift, brg.db_ct_div);
		duart_cons.ct_rate = duart_cons.speed;
	}

	duart_cflag_to_mr(duart_cons.cflag, &mr1, &mr2);
	duart_program_chan(duart_cons.variant, duart_cons.bst,
	    duart_cons.bsh, duart_cons.shift, duart_cons.chan, mr1, mr2, &brg);

	duart_cons.active = true;
	cn_tab = &duart_consdev;
}

static const struct fdt_console duart_fdt_console = {
	.match = duart_console_match,
	.consinit = duart_console_consinit,
};

FDT_CONSOLE(duart, &duart_fdt_console);

/*
 * Autoconfiguration.
 */

static int
duart_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
duart_attach(device_t parent, device_t self, void *aux)
{
	struct duart_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	char intrstr[128];
	u_int specifier[2];
	bus_addr_t addr;
	bus_size_t size;
	uint32_t val;
	int hwipl, ipl, chan;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	sc->sc_variant = duart_lookup_variant(phandle);

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	sc->sc_shift =
	    of_getprop_uint32(phandle, "reg-shift", &val) == 0 ? val : 1;
	sc->sc_freq =
	    of_getprop_uint32(phandle, "clock-frequency", &val) == 0
	    ? val : DUART_DEFAULT_FREQ;

	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	if (OF_getprop(phandle, "interrupts", specifier,
		       sizeof(specifier)) != sizeof(specifier)) {
		aprint_error(": couldn't get interrupt specifier\n");
		return;
	}
	hwipl = be32toh(specifier[0]);
	ipl = hwipl >= 6 ? IPL_SCHED : IPL_VM;
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, ipl);

	const bool is_console = duart_cons.active && duart_cons.addr == addr;

	aprint_naive("\n");
	aprint_normal(": %s DUART, %u Hz\n", sc->sc_variant->dv_name,
	    sc->sc_freq);

	/* Quiesce everything except the console channel. */
	sc->sc_imr = 0;
	SC_WRITE(sc, DU_IMR, sc->sc_imr);
	if (be32toh(specifier[1]) != 0) {
		SC_WRITE(sc, DU_IVR, be32toh(specifier[1]));
	}
	if (is_console) {
		sc->sc_ct_rate = duart_cons.ct_rate;
		sc->sc_ct_users = duart_cons.ct_rate ? 1 : 0;
	} else {
		SC_WRITE(sc, DU_ACR, ACR_DEFAULT);
	}

	for (chan = 0; chan < 2; chan++) {
		struct duart_chan * const ch = &sc->sc_chan[chan];
		struct tty *tp;

		ch->ch_sc = sc;
		ch->ch_num = chan;
		ch->ch_console = is_console && duart_cons.chan == chan;

		if (!ch->ch_console) {
			du_write_settle(sc->sc_bst, sc->sc_bsh, sc->sc_shift,
			    chan, DU_CR, CR_RESET_RX);
			du_write_settle(sc->sc_bst, sc->sc_bsh, sc->sc_shift,
			    chan, DU_CR, CR_RESET_TX);
			du_write_settle(sc->sc_bst, sc->sc_bsh, sc->sc_shift,
			    chan, DU_CR, CR_RESET_ERR);
			du_write_settle(sc->sc_bst, sc->sc_bsh, sc->sc_shift,
			    chan, DU_CR, CR_RESET_BRKINT);
		} else {
			ch->ch_uses_ct = duart_cons.ct_rate != 0;
		}

		tp = tty_alloc();
		tp->t_oproc = duartstart;
		tp->t_param = duartparam;
		tty_attach(tp);
		ch->ch_tty = tp;

		if (ch->ch_console) {
			ch->ch_cndev = makedev(cdevsw_lookup_major(&duart_cdevsw),
			    DUART_MINOR(device_unit(self), chan));
			cn_tab->cn_dev = ch->ch_cndev;
			aprint_normal_dev(self, "channel %c: console\n",
			    'A' + chan);
		}
	}

	sc->sc_si = softint_establish(SOFTINT_SERIAL, duart_soft, sc);
	if (sc->sc_si == NULL) {
		aprint_error_dev(self, "couldn't establish soft interrupt\n");
		return;
	}

	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error_dev(self, "failed to decode interrupt\n");
		return;
	}
	sc->sc_ih = fdtbus_intr_establish_xname(phandle, 0, ipl,
	    FDT_INTR_MPSAFE, duart_intr, sc, device_xname(self));
	if (sc->sc_ih == NULL) {
		aprint_error_dev(self,
		    "failed to establish interrupt at %s\n", intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting at %s\n", intrstr);

	if (is_console) {
		/* Default magic is BREAK; listen for it from now on. */
		cn_init_magic(&duart_cnm_state);
		cn_set_magic("\047\001");

		mutex_spin_enter(&sc->sc_lock);
		sc->sc_imr |= (ISR_RXRDY | ISR_DBRK) <<
		    (duart_cons.chan * ISR_CHAN_SHIFT);
		SC_WRITE(sc, DU_IMR, sc->sc_imr);
		mutex_spin_exit(&sc->sc_lock);
	}
}

/*
 * Hardware interrupt handling.  Called with sc_lock held.
 */

static void
duart_rxintr(struct duart_softc *sc, struct duart_chan *ch, uint8_t isr)
{
	const int chan = ch->ch_num;
	struct tty * const tp = ch->ch_tty;
	const bool open = ISSET(tp->t_state, TS_ISOPEN);
	uint8_t sr, c;

	if (isr & ISR_DBRK) {
		SC_WRITE(sc, CH_REG(chan, DU_CR), CR_RESET_BRKINT);
	}

	while ((sr = SC_READ(sc, CH_REG(chan, DU_SR))) & SR_RXRDY) {
		c = SC_READ(sc, CH_REG(chan, DU_RHR));

		if (sr & (SR_OE | SR_PE | SR_FE | SR_RB)) {
			SC_WRITE(sc, CH_REG(chan, DU_CR), CR_RESET_ERR);
		}

		if (ch->ch_console) {
			if (sr & SR_RB) {
				cn_check_magic(ch->ch_cndev, CNC_BREAK,
				    duart_cnm_state);
			} else {
				cn_check_magic(ch->ch_cndev, c,
				    duart_cnm_state);
			}
		}

		/* A break arrives as a NUL with RB set; drop it. */
		if (!open || (sr & SR_RB)) {
			continue;
		}

		if (ch->ch_rbput - ch->ch_rbget >= DUART_RBUF_SIZE) {
			ch->ch_rbovf++;
			continue;
		}
		ch->ch_rbuf[ch->ch_rbput++ & (DUART_RBUF_SIZE - 1)] =
		    ((uint16_t)sr << 8) | c;
		ch->ch_rx_ready = true;
	}
}

static void
duart_txintr(struct duart_softc *sc, struct duart_chan *ch)
{
	const int chan = ch->ch_num;

	if (ch->ch_tbc > 0) {
		SC_WRITE(sc, CH_REG(chan, DU_THR), *ch->ch_tba);
		ch->ch_tba++;
		ch->ch_tbc--;
		return;
	}

	/* Done; TxRDY stays asserted while THR is empty, so mask it. */
	sc->sc_imr &= ~(ISR_TXRDY << (chan * ISR_CHAN_SHIFT));
	SC_WRITE(sc, DU_IMR, sc->sc_imr);

	if (ISSET(ch->ch_tty->t_state, TS_BUSY)) {
		ch->ch_tx_done = true;
	}
}

static int
duart_intr(void *arg)
{
	struct duart_softc * const sc = arg;
	uint8_t isr;
	int handled = 0;

	mutex_spin_enter(&sc->sc_lock);
	while ((isr = SC_READ(sc, DU_ISR) & sc->sc_imr) != 0) {
		handled = 1;
		for (int chan = 0; chan < 2; chan++) {
			struct duart_chan * const ch = &sc->sc_chan[chan];
			const uint8_t cisr =
			    (isr >> (chan * ISR_CHAN_SHIFT)) & 0x0f;

			if (cisr & (ISR_RXRDY | ISR_DBRK)) {
				duart_rxintr(sc, ch, cisr);
			}
			if (cisr & ISR_TXRDY) {
				duart_txintr(sc, ch);
			}
		}
	}
	mutex_spin_exit(&sc->sc_lock);

	if (handled) {
		softint_schedule(sc->sc_si);
	}
	return handled;
}

/*
 * Soft interrupt: hand received characters to the line discipline
 * and restart output.
 */
static void
duart_soft(void *arg)
{
	struct duart_softc * const sc = arg;

	for (int chan = 0; chan < 2; chan++) {
		struct duart_chan * const ch = &sc->sc_chan[chan];
		struct tty * const tp = ch->ch_tty;
		uint16_t ent;
		u_int ovf;
		bool tx_done;
		int c;

		mutex_spin_enter(&sc->sc_lock);
		ovf = ch->ch_rbovf;
		ch->ch_rbovf = 0;
		ch->ch_rx_ready = false;
		mutex_spin_exit(&sc->sc_lock);

		if (ovf) {
			log(LOG_WARNING, "%s: channel %c: %u receive "
			    "overflow(s)\n", device_xname(sc->sc_dev),
			    'A' + chan, ovf);
		}

		for (;;) {
			mutex_spin_enter(&sc->sc_lock);
			if (ch->ch_rbget == ch->ch_rbput) {
				mutex_spin_exit(&sc->sc_lock);
				break;
			}
			ent = ch->ch_rbuf[ch->ch_rbget++ &
			    (DUART_RBUF_SIZE - 1)];
			mutex_spin_exit(&sc->sc_lock);

			c = ent & 0xff;
			if (ent & (SR_FE << 8)) {
				SET(c, TTY_FE);
			}
			if (ent & (SR_PE << 8)) {
				SET(c, TTY_PE);
			}
			if (ISSET(tp->t_state, TS_ISOPEN)) {
				(*tp->t_linesw->l_rint)(c, tp);
			}
		}

		mutex_spin_enter(&sc->sc_lock);
		tx_done = ch->ch_tx_done;
		ch->ch_tx_done = false;
		mutex_spin_exit(&sc->sc_lock);

		if (tx_done) {
			CLR(tp->t_state, TS_BUSY);
			if (ISSET(tp->t_state, TS_FLUSH)) {
				CLR(tp->t_state, TS_FLUSH);
			} else {
				ndflush(&tp->t_outq,
				    (int)(ch->ch_tba - tp->t_outq.c_cf));
			}
			(*tp->t_linesw->l_start)(tp);
		}
	}
}

/*
 * tty glue.
 */

static struct duart_chan *
duart_dev_to_chan(dev_t dev)
{
	struct duart_softc *sc;

	sc = device_lookup_private(&duart_cd, DUART_UNIT(dev));
	if (sc == NULL || sc->sc_si == NULL) {
		return NULL;
	}
	return &sc->sc_chan[DUART_CHAN(dev)];
}

static void
duartstart(struct tty *tp)
{
	struct duart_chan * const ch = duart_dev_to_chan(tp->t_dev);
	struct duart_softc *sc;
	int s;

	if (ch == NULL) {
		return;
	}
	sc = ch->ch_sc;

	s = spltty();
	if (ISSET(tp->t_state, TS_BUSY | TS_TIMEOUT | TS_TTSTOP)) {
		goto out;
	}
	if (!ttypull(tp)) {
		goto out;
	}

	mutex_spin_enter(&sc->sc_lock);
	ch->ch_tba = tp->t_outq.c_cf;
	ch->ch_tbc = ndqb(&tp->t_outq, 0);
	SET(tp->t_state, TS_BUSY);
	/* THR is empty if TxRDY is set, so this interrupts immediately. */
	sc->sc_imr |= ISR_TXRDY << (ch->ch_num * ISR_CHAN_SHIFT);
	SC_WRITE(sc, DU_IMR, sc->sc_imr);
	mutex_spin_exit(&sc->sc_lock);
 out:
	splx(s);
}

static void
duartstop(struct tty *tp, int flag)
{
	struct duart_chan * const ch = duart_dev_to_chan(tp->t_dev);
	struct duart_softc *sc;

	if (ch == NULL) {
		return;
	}
	sc = ch->ch_sc;

	mutex_spin_enter(&sc->sc_lock);
	if (ISSET(tp->t_state, TS_BUSY)) {
		/* Stop transmitting at the next character boundary. */
		ch->ch_tbc = 0;
		if (!ISSET(tp->t_state, TS_TTSTOP)) {
			SET(tp->t_state, TS_FLUSH);
		}
	}
	mutex_spin_exit(&sc->sc_lock);
}

static int
duartparam(struct tty *tp, struct termios *t)
{
	struct duart_chan * const ch = duart_dev_to_chan(tp->t_dev);
	struct duart_softc *sc;
	struct duart_brg brg;
	uint8_t mr1, mr2;

	if (ch == NULL) {
		return ENXIO;
	}
	sc = ch->ch_sc;

	/* No split speeds, no hardware flow control or modem control. */
	if (t->c_ispeed && t->c_ispeed != t->c_ospeed) {
		return EINVAL;
	}
	if (ISSET(t->c_cflag, CRTSCTS | MDMBUF)) {
		return EINVAL;
	}

	if (!duart_speed_code(sc->sc_variant, sc->sc_freq, t->c_ospeed,
			      &brg)) {
		return EINVAL;
	}

	mutex_spin_enter(&sc->sc_lock);
	if (brg.db_ct_div != 0) {
		/* Someone else already has the C/T at a different rate? */
		const u_int others = sc->sc_ct_users - (ch->ch_uses_ct ? 1 : 0);
		if (others != 0 && sc->sc_ct_rate != (u_int)t->c_ospeed) {
			mutex_spin_exit(&sc->sc_lock);
			return EINVAL;
		}
		if (!ch->ch_uses_ct) {
			ch->ch_uses_ct = true;
			sc->sc_ct_users++;
		}
		if (sc->sc_ct_rate != (u_int)t->c_ospeed) {
			duart_program_ct(sc->sc_bst, sc->sc_bsh, sc->sc_shift,
			    brg.db_ct_div);
			sc->sc_ct_rate = t->c_ospeed;
		}
	} else if (ch->ch_uses_ct) {
		ch->ch_uses_ct = false;
		sc->sc_ct_users--;
	}

	duart_cflag_to_mr(t->c_cflag, &mr1, &mr2);
	duart_program_chan(sc->sc_variant, sc->sc_bst, sc->sc_bsh,
	    sc->sc_shift, ch->ch_num, mr1, mr2, &brg);
	mutex_spin_exit(&sc->sc_lock);

	tp->t_ispeed = 0;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	/* Carrier is always present. */
	(void)(*tp->t_linesw->l_modem)(tp, 1);

	return 0;
}

static void
duart_shutdown(struct duart_chan *ch)
{
	struct duart_softc * const sc = ch->ch_sc;

	if (ch->ch_console) {
		/* Keep listening for the debugger magic. */
		return;
	}

	mutex_spin_enter(&sc->sc_lock);
	sc->sc_imr &= ~((ISR_TXRDY | ISR_RXRDY | ISR_DBRK) <<
	    (ch->ch_num * ISR_CHAN_SHIFT));
	SC_WRITE(sc, DU_IMR, sc->sc_imr);
	mutex_spin_exit(&sc->sc_lock);
}

static int
duartopen(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct duart_chan * const ch = duart_dev_to_chan(dev);
	struct duart_softc *sc;
	struct tty *tp;
	int error, s;

	if (ch == NULL) {
		return ENXIO;
	}
	sc = ch->ch_sc;
	tp = ch->ch_tty;

	if (kauth_authorize_device_tty(l->l_cred, KAUTH_DEVICE_TTY_OPEN, tp)) {
		return EBUSY;
	}

	s = spltty();

	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		struct termios t;

		tp->t_dev = dev;

		t.c_ispeed = 0;
		if (ch->ch_console) {
			t.c_ospeed = duart_cons.speed;
			t.c_cflag = duart_cons.cflag;
		} else {
			t.c_ospeed = TTYDEF_SPEED;
			t.c_cflag = TTYDEF_CFLAG;
		}
		SET(t.c_cflag, CLOCAL);
		CLR(t.c_cflag, CRTSCTS | MDMBUF);

		/* Make sure duartparam() will do something. */
		tp->t_ospeed = 0;
		(void)duartparam(tp, &t);
		tp->t_iflag = TTYDEF_IFLAG;
		tp->t_oflag = TTYDEF_OFLAG;
		tp->t_lflag = TTYDEF_LFLAG;
		ttychars(tp);
		ttsetwater(tp);

		mutex_spin_enter(&sc->sc_lock);
		ch->ch_rbget = ch->ch_rbput = 0;
		sc->sc_imr |= (ISR_RXRDY | ISR_DBRK) <<
		    (ch->ch_num * ISR_CHAN_SHIFT);
		SC_WRITE(sc, DU_IMR, sc->sc_imr);
		mutex_spin_exit(&sc->sc_lock);
	}

	splx(s);

	error = ttyopen(tp, DUART_DIALOUT(dev), ISSET(flag, O_NONBLOCK));
	if (error) {
		goto bad;
	}

	error = (*tp->t_linesw->l_open)(dev, tp);
	if (error) {
		goto bad;
	}

	return 0;

 bad:
	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		duart_shutdown(ch);
	}
	return error;
}

static int
duartclose(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct duart_chan * const ch = duart_dev_to_chan(dev);
	struct tty *tp;

	if (ch == NULL) {
		return 0;
	}
	tp = ch->ch_tty;

	if (!ISSET(tp->t_state, TS_ISOPEN)) {
		return 0;
	}

	(*tp->t_linesw->l_close)(tp, flag);
	ttyclose(tp);

	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		duart_shutdown(ch);
	}

	return 0;
}

static int
duartread(dev_t dev, struct uio *uio, int flag)
{
	struct duart_chan * const ch = duart_dev_to_chan(dev);

	if (ch == NULL) {
		return ENXIO;
	}
	return (*ch->ch_tty->t_linesw->l_read)(ch->ch_tty, uio, flag);
}

static int
duartwrite(dev_t dev, struct uio *uio, int flag)
{
	struct duart_chan * const ch = duart_dev_to_chan(dev);

	if (ch == NULL) {
		return ENXIO;
	}
	return (*ch->ch_tty->t_linesw->l_write)(ch->ch_tty, uio, flag);
}

static int
duartpoll(dev_t dev, int events, struct lwp *l)
{
	struct duart_chan * const ch = duart_dev_to_chan(dev);

	if (ch == NULL) {
		return POLLHUP;
	}
	return (*ch->ch_tty->t_linesw->l_poll)(ch->ch_tty, events, l);
}

static struct tty *
duarttty(dev_t dev)
{
	struct duart_chan * const ch = duart_dev_to_chan(dev);

	return ch != NULL ? ch->ch_tty : NULL;
}

static int
duartioctl(dev_t dev, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct duart_chan * const ch = duart_dev_to_chan(dev);
	struct duart_softc *sc;
	struct tty *tp;
	int error;

	if (ch == NULL) {
		return ENXIO;
	}
	sc = ch->ch_sc;
	tp = ch->ch_tty;

	error = (*tp->t_linesw->l_ioctl)(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH) {
		return error;
	}

	error = ttioctl(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH) {
		return error;
	}

	switch (cmd) {
	case TIOCSBRK:
		mutex_spin_enter(&sc->sc_lock);
		SC_WRITE(sc, CH_REG(ch->ch_num, DU_CR), CR_START_BRK);
		mutex_spin_exit(&sc->sc_lock);
		break;

	case TIOCCBRK:
		mutex_spin_enter(&sc->sc_lock);
		SC_WRITE(sc, CH_REG(ch->ch_num, DU_CR), CR_STOP_BRK);
		mutex_spin_exit(&sc->sc_lock);
		break;

	default:
		return EPASSTHROUGH;
	}

	return 0;
}
