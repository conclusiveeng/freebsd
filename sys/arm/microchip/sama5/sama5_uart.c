/*-
 * Copyright (c) 2005 M. Warner Losh
 * Copyright (c) 2005 Olivier Houchard
 * Copyright (c) 2012 Ian Lepore
 * Copyright (c) 2020 Conclusive Engineering
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/cons.h>
#include <sys/tty.h>
#include <machine/bus.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_bus.h>
#include <arm/microchip/sama5/sama5_uartreg.h>

#include "uart_if.h"

#define	DEFAULT_RCLK			0 //83000000
#define	USART_DEFAULT_FIFO_BYTES	128

#define	USART_DCE_CHANGE_BITS	(USART_CSR_CTSIC | USART_CSR_DCDIC | \
				 USART_CSR_DSRIC | USART_CSR_RIIC)

struct sama5_uart_softc {
	struct uart_softc base;
};

#define	RD4(bas, reg)		\
	bus_space_read_4((bas)->bst, (bas)->bsh, uart_regofs(bas, reg))
#define	WR4(bas, reg, value)	\
	bus_space_write_4((bas)->bst, (bas)->bsh, uart_regofs(bas, reg), value)

#define	SIGCHG(c, i, s, d)				\
	do {						\
		if (c) {				\
			i |= (i & s) ? s : s | d;	\
		} else {				\
			i = (i & s) ? (i & ~s) | d : i;	\
		}					\
	} while (0);

#define BAUD2DIVISOR(b) \
	((((DEFAULT_RCLK * 10) / ((b) * 16)) + 5) / 10)

/*
 * Low-level UART interface.
 */
static int sama5_uart_probe(struct uart_bas *bas);
static void sama5_uart_init(struct uart_bas *bas, int, int, int, int);
static void sama5_uart_term(struct uart_bas *bas);
static void sama5_uart_putc(struct uart_bas *bas, int);
static int sama5_uart_rxready(struct uart_bas *bas);
static int sama5_uart_getc(struct uart_bas *bas, struct mtx *hwmtx);

extern SLIST_HEAD(uart_devinfo_list, uart_devinfo) uart_sysdevs;

static int
sama5_uart_param(struct uart_bas *bas, int baudrate, int databits,
    int stopbits, int parity)
{
	uint32_t mr;

	/*
	 * Assume 3-wire RS-232 configuration.
	 * XXX Not sure how uart will present the other modes to us, so
	 * XXX they are unimplemented.  maybe ioctl?
	 */
	mr = USART_MR_MODE_NORMAL;
	mr |= USART_MR_USCLKS_MCK;	/* Assume MCK */

	/*
	 * Or in the databits requested
	 */
	if (databits < 9)
		mr &= ~USART_MR_MODE9;
	switch (databits) {
	case 5:
		mr |= USART_MR_CHRL_5BITS;
		break;
	case 6:
		mr |= USART_MR_CHRL_6BITS;
		break;
	case 7:
		mr |= USART_MR_CHRL_7BITS;
		break;
	case 8:
		mr |= USART_MR_CHRL_8BITS;
		break;
	case 9:
		mr |= USART_MR_CHRL_8BITS | USART_MR_MODE9;
		break;
	default:
		return (EINVAL);
	}

	/*
	 * Or in the parity
	 */
	switch (parity) {
	case UART_PARITY_NONE:
		mr |= USART_MR_PAR_NONE;
		break;
	case UART_PARITY_ODD:
		mr |= USART_MR_PAR_ODD;
		break;
	case UART_PARITY_EVEN:
		mr |= USART_MR_PAR_EVEN;
		break;
	case UART_PARITY_MARK:
		mr |= USART_MR_PAR_MARK;
		break;
	case UART_PARITY_SPACE:
		mr |= USART_MR_PAR_SPACE;
		break;
	default:
		return (EINVAL);
	}

	/*
	 * Or in the stop bits.  Note: The hardware supports 1.5 stop
	 * bits in async mode, but there's no way to specify that
	 * AFAICT.  Instead, rely on the convention documented at
	 * http://www.lammertbies.nl/comm/info/RS-232_specs.html which
	 * states that 1.5 stop bits are used for 5 bit bytes and
	 * 2 stop bits only for longer bytes.
	 */
	if (stopbits == 1)
		mr |= USART_MR_NBSTOP_1;
	else if (databits > 5)
		mr |= USART_MR_NBSTOP_2;
	else
		mr |= USART_MR_NBSTOP_1_5;

	/*
	 * We want normal plumbing mode too, none of this fancy
	 * loopback or echo mode.
	 */
	mr |= USART_MR_CHMODE_NORMAL;

	mr &= ~USART_MR_MSBF;	/* lsb first */
	mr &= ~USART_MR_CKLO_SCK;	/* Don't drive SCK */

	WR4(bas, USART_MR, mr);

	/*
	 * Set the baud rate (only if we know our master clock rate)
	 */
	if (DEFAULT_RCLK != 0)
		WR4(bas, USART_BRGR, BAUD2DIVISOR(baudrate));

	/*
	 * Set the receive timeout based on the baud rate.  The idea is to
	 * compromise between being responsive on an interactive connection and
	 * giving a bulk data sender a bit of time to queue up a new buffer
	 * without mistaking it for a stopping point in the transmission.  For
	 * 19.2kbps and below, use 20 * bit time (2 characters).  For faster
	 * connections use 500 microseconds worth of bits.
	 */
	if (baudrate <= 19200)
		WR4(bas, USART_RTOR, 20);
	else 
		WR4(bas, USART_RTOR, baudrate / 2000);
	WR4(bas, USART_CR, USART_CR_STTTO);

	/* XXX Need to take possible synchronous mode into account */
	return (0);
}

static struct uart_ops sama5_uart_ops = {
	.probe = sama5_uart_probe,
	.init = sama5_uart_init,
	.term = sama5_uart_term,
	.putc = sama5_uart_putc,
	.rxready = sama5_uart_rxready,
	.getc = sama5_uart_getc,
};

#ifdef EARLY_PRINTF
/*
 * Early printf support. This assumes that we have the SoC "system" devices
 * mapped into AT91_BASE. To use this before we adjust the boostrap tables,
 * you'll need to define SOCDEV_VA to be 0xdc000000 and SOCDEV_PA to be
 * 0xfc000000 in your config file where you define EARLY_PRINTF
 */
volatile uint32_t *sama5_dbgu = (volatile uint32_t *)(AT91_BASE + AT91_DBGU0);

static void
eputc(int c)
{

	while (!(sama5_dbgu[USART_CSR / 4] & USART_CSR_TXRDY))
		continue;
	sama5_dbgu[USART_THR / 4] = c;
}

early_putc_t * early_putc = eputc;
#endif

static int
sama5_uart_probe(struct uart_bas *bas)
{

	/* We know that this is always here */
	return (0);
}

/*
 * Initialize this device for use as a console.
 */
static void
sama5_uart_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{

#ifdef EARLY_PRINTF
	if (early_putc != NULL) {
		printf("Early printf yielding control to the real console.\n");
		early_putc = NULL;
	}
#endif

	/*
	 * This routine is called multiple times, sometimes right after writing
	 * some output, and the last byte is still shifting out.  If that's the
	 * case delay briefly before resetting, but don't loop on TXRDY because
	 * we don't want to hang here forever if the hardware is in a bad state.
	 */
	if (!(RD4(bas, USART_CSR) & USART_CSR_TXRDY))
		DELAY(10000);
	
	sama5_uart_param(bas, baudrate, databits, stopbits, parity);

	/* Reset the rx and tx buffers and turn on rx and tx */
	WR4(bas, USART_CR, USART_CR_RSTSTA | USART_CR_RSTRX | USART_CR_RSTTX);
	WR4(bas, USART_CR, USART_CR_RXEN | USART_CR_TXEN);
	WR4(bas, USART_IDR, 0xffffffff);
}

/*
 * Free resources now that we're no longer the console.  This appears to
 * be never called, and I'm unsure quite what to do if I am called.
 */
static void
sama5_uart_term(struct uart_bas *bas)
{

	/* XXX */
}

/*
 * Put a character of console output (so we do it here polling rather than
 * interrupt driven).
 */
static void
sama5_uart_putc(struct uart_bas *bas, int c)
{

	while (!(RD4(bas, USART_CSR) & USART_CSR_TXRDY))
		continue;

	WR4(bas, USART_THR, c);
}

/*
 * Check for a character available.
 */
static int
sama5_uart_rxready(struct uart_bas *bas)
{

	return ((RD4(bas, USART_CSR) & USART_CSR_RXRDY) != 0 ? 1 : 0);
}

/*
 * Block waiting for a character.
 */
static int
sama5_uart_getc(struct uart_bas *bas, struct mtx *hwmtx)
{
	int c;

	uart_lock(hwmtx);
	while (!(RD4(bas, USART_CSR) & USART_CSR_RXRDY)) {
		uart_unlock(hwmtx);
		DELAY(4);
		uart_lock(hwmtx);
	}
	c = RD4(bas, USART_RHR) & 0xff;
	uart_unlock(hwmtx);
	return (c);
}

static int sama5_uart_bus_probe(struct uart_softc *sc);
static int sama5_uart_bus_attach(struct uart_softc *sc);
static int sama5_uart_bus_flush(struct uart_softc *, int);
static int sama5_uart_bus_getsig(struct uart_softc *);
static int sama5_uart_bus_ioctl(struct uart_softc *, int, intptr_t);
static int sama5_uart_bus_ipend(struct uart_softc *);
static int sama5_uart_bus_param(struct uart_softc *, int, int, int, int);
static int sama5_uart_bus_receive(struct uart_softc *);
static int sama5_uart_bus_setsig(struct uart_softc *, int);
static int sama5_uart_bus_transmit(struct uart_softc *);
static void sama5_uart_bus_grab(struct uart_softc *);
static void sama5_uart_bus_ungrab(struct uart_softc *);

static kobj_method_t sama5_uart_methods[] = {
	KOBJMETHOD(uart_probe,		sama5_uart_bus_probe),
	KOBJMETHOD(uart_attach,		sama5_uart_bus_attach),
	KOBJMETHOD(uart_flush,		sama5_uart_bus_flush),
	KOBJMETHOD(uart_getsig,		sama5_uart_bus_getsig),
	KOBJMETHOD(uart_ioctl,		sama5_uart_bus_ioctl),
	KOBJMETHOD(uart_ipend,		sama5_uart_bus_ipend),
	KOBJMETHOD(uart_param,		sama5_uart_bus_param),
	KOBJMETHOD(uart_receive,	sama5_uart_bus_receive),
	KOBJMETHOD(uart_setsig,		sama5_uart_bus_setsig),
	KOBJMETHOD(uart_transmit,	sama5_uart_bus_transmit),
	KOBJMETHOD(uart_grab,		sama5_uart_bus_grab),
	KOBJMETHOD(uart_ungrab,		sama5_uart_bus_ungrab),
	KOBJMETHOD_END
};

int
sama5_uart_bus_probe(struct uart_softc *sc)
{
	int value;

	value = USART_DEFAULT_FIFO_BYTES;
	resource_int_value(device_get_name(sc->sc_dev), 
	    device_get_unit(sc->sc_dev), "fifo_bytes", &value);
	value = roundup2(value, arm_dcache_align);
	sc->sc_txfifosz = value;
	sc->sc_rxfifosz = value;
	sc->sc_hwiflow = 0;
	return (0);
}

static int
sama5_uart_bus_attach(struct uart_softc *sc)
{
	struct sama5_uart_softc *samsc;

	samsc = (struct sama5_uart_softc *)sc;

	/* Turn on rx and tx */
	DELAY(1000);		/* Give pending character a chance to drain.  */
	WR4(&sc->sc_bas, USART_CR, USART_CR_RSTSTA | USART_CR_RSTRX | USART_CR_RSTTX);
	WR4(&sc->sc_bas, USART_CR, USART_CR_RXEN | USART_CR_TXEN);
	WR4(&sc->sc_bas, USART_IER, USART_CSR_RXRDY);
	WR4(&sc->sc_bas, USART_IER, USART_CSR_RXBRK | USART_DCE_CHANGE_BITS);

	/* Prime sc->hwsig with the initial hw line states. */
	sama5_uart_bus_getsig(sc);

	return (0);
}

static int
sama5_uart_bus_transmit(struct uart_softc *sc)
{
	int err;
	int i;

	err = 0;
	uart_lock(sc->sc_hwmtx);

	for (i = 0; i < sc->sc_txdatasz; i++) {
		sama5_uart_putc(&sc->sc_bas, sc->sc_txbuf[i]);
		uart_barrier(&sc->sc_bas);
	}

	uart_unlock(sc->sc_hwmtx);
	return (err);
}

static int
sama5_uart_bus_setsig(struct uart_softc *sc, int sig)
{
	uint32_t new, old, cr;
	struct sama5_uart_softc *samsc;

	samsc = (struct sama5_uart_softc *)sc;

	do {
		old = sc->sc_hwsig;
		new = old;
		if (sig & SER_DDTR)
			SIGCHG(sig & SER_DTR, new, SER_DTR, SER_DDTR);
		if (sig & SER_DRTS)
			SIGCHG(sig & SER_RTS, new, SER_RTS, SER_DRTS);
	} while (!atomic_cmpset_32(&sc->sc_hwsig, old, new));

	cr = 0;
	if (new & SER_DTR)
		cr |= USART_CR_DTREN;
	else
		cr |= USART_CR_DTRDIS;
	if (new & SER_RTS)
		cr |= USART_CR_RTSEN;
	else
		cr |= USART_CR_RTSDIS;

	uart_lock(sc->sc_hwmtx);
	WR4(&sc->sc_bas, USART_CR, cr);
	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
sama5_uart_bus_receive(struct uart_softc *sc)
{

	return (0);
}

static int
sama5_uart_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{

	return (sama5_uart_param(&sc->sc_bas, baudrate, databits, stopbits,
	    parity));
}

static __inline void
sama5_rx_put(struct uart_softc *sc, int key)
{

#if defined(KDB)
	if (sc->sc_sysdev != NULL && sc->sc_sysdev->type == UART_DEV_CONSOLE)
		kdb_alt_break(key, &sc->sc_altbrk);
#endif
	uart_rx_put(sc, key);
}

static int
sama5_uart_bus_ipend(struct uart_softc *sc)
{
	struct sama5_uart_softc *samsc;
	int ipend;
	uint32_t csr;

	ipend = 0;
	samsc = (struct sama5_uart_softc *)sc;
	uart_lock(sc->sc_hwmtx);
	csr = RD4(&sc->sc_bas, USART_CSR);

	if (csr & USART_CSR_OVRE) {
		WR4(&sc->sc_bas, USART_CR, USART_CR_RSTSTA);
		ipend |= SER_INT_OVERRUN;
	}

	if (csr & USART_DCE_CHANGE_BITS)
		ipend |= SER_INT_SIGCHG;

	if (csr & (USART_CSR_TXRDY | USART_CSR_ENDTX)) {
		if (sc->sc_txbusy)
			ipend |= SER_INT_TXIDLE;
		WR4(&sc->sc_bas, USART_IDR, csr & (USART_CSR_TXRDY |
		    USART_CSR_ENDTX));
	}

	if (csr & USART_CSR_RXRDY) {
		sama5_rx_put(sc, RD4(&sc->sc_bas, USART_RHR) & 0xff);
		ipend |= SER_INT_RXREADY;
	}

	if (csr & USART_CSR_RXBRK) {
		ipend |= SER_INT_BREAK;
		WR4(&sc->sc_bas, USART_CR, USART_CR_RSTSTA);
	}

	uart_unlock(sc->sc_hwmtx);
	return (ipend);
}

static int
sama5_uart_bus_flush(struct uart_softc *sc, int what)
{

	return (0);
}

static int
sama5_uart_bus_getsig(struct uart_softc *sc)
{
	uint32_t csr, new, old, sig;

	/*
	 * Note that the atmel channel status register DCE status bits reflect
	 * the electrical state of the lines, not the logical state.  Since they
	 * are logically active-low signals, we invert the tests here.
	 */
	do {
		old = sc->sc_hwsig;
		sig = old;
		csr = RD4(&sc->sc_bas, USART_CSR);
		SIGCHG(!(csr & USART_CSR_DSR), sig, SER_DSR, SER_DDSR);
		SIGCHG(!(csr & USART_CSR_CTS), sig, SER_CTS, SER_DCTS);
		SIGCHG(!(csr & USART_CSR_DCD), sig, SER_DCD, SER_DDCD);
		SIGCHG(!(csr & USART_CSR_RI),  sig, SER_RI,  SER_DRI);
		new = sig & ~SER_MASK_DELTA;
	} while (!atomic_cmpset_32(&sc->sc_hwsig, old, new));

	return (sig);
}

static int
sama5_uart_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{

	switch (request) {
	case UART_IOCTL_BREAK:
	case UART_IOCTL_IFLOW:
	case UART_IOCTL_OFLOW:
		break;
	case UART_IOCTL_BAUD:
		/* only if we know our master clock rate */
		if (DEFAULT_RCLK != 0)
			WR4(&sc->sc_bas, USART_BRGR,
			    BAUD2DIVISOR(*(int *)data));
		return (0);
	}
	return (EINVAL);
}


static void
sama5_uart_bus_grab(struct uart_softc *sc)
{

	uart_lock(sc->sc_hwmtx);
	WR4(&sc->sc_bas, USART_IDR, USART_CSR_RXRDY);
	uart_unlock(sc->sc_hwmtx);
}

static void
sama5_uart_bus_ungrab(struct uart_softc *sc)
{

	uart_lock(sc->sc_hwmtx);
	WR4(&sc->sc_bas, USART_IER, USART_CSR_RXRDY);
	uart_unlock(sc->sc_hwmtx);
}

struct uart_class sama5_uart_class = {
	"sama5_uart",
	sama5_uart_methods,
	sizeof(struct sama5_uart_softc),
	.uc_ops = &sama5_uart_ops,
	.uc_range = 8
};

static struct ofw_compat_data compat_data[] = {
	{"atmel,at91rm9200-usart",(uintptr_t)&sama5_uart_class},
	{"atmel,at91sam9260-usart",(uintptr_t)&sama5_uart_class},
	{NULL,			(uintptr_t)NULL},
};

UART_FDT_CLASS_AND_DEVICE(compat_data);
