/*-
 * Copyright (c) 2006 M. Warner Losh.  All rights reserved.
 * Copyright (c) 2010 Greg Ansley.  All rights reserved.
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

#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/timetc.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>
#include <arm/sama5/sama5reg.h>
#include <arm/sama5/sama5var.h>

#include <arm/sama5/sama5_pmcreg.h>
#include <arm/sama5/sama5_pmcvar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

struct sama5_pmc_softc {
	struct clkdom		*clkdom;
	bus_space_tag_t		sc_st;
	bus_space_handle_t	sc_sh;
	struct resource		*mem_res;	/* Memory resource */
	device_t		dev;
};

struct sama5_clknode_core_softc {
	device_t		clkdev;
	int			clkid;
};

struct sama5_clknode_pck_softc {
	device_t		clkdev;
	int			clkid;
};

static uint32_t pllb_init;

MALLOC_DECLARE(M_PMC);
MALLOC_DEFINE(M_PMC, "sama5_pmc_clocks", "AT91 PMC Clock descriptors");

#define AT91_PMC_BASE 0xffffc00

static void sama5_pmc_set_pllb_mode(struct sama5_pmc_clock *, int);
static void sama5_pmc_set_upll_mode(struct sama5_pmc_clock *, int);
static void sama5_pmc_set_sys_mode(struct sama5_pmc_clock *, int);
static void sama5_pmc_set_periph_mode(struct sama5_pmc_clock *, int);
static void sama5_pmc_clock_alias(const char *name, const char *alias);

static struct clk_fixed_def sama5_pmc_osc32khz = {
	.clkdef.id = PMC_OSC32KHZ,
	.clkdef.name = "osc32khz",
	.freq = 32768,
};

static struct clk_fixed_def sama5_pmc_osc12mhz = {
	.clkdef.id = 1,
	.clkdef.name = "osc12mhz",
	.freq = 12000000,
};

static struct clk_init_def sama5_pmc_slck = {
	.name = "slck",
	.id = PMC_SLOW,
	.parent_names = {"slow_xtal", "osc32khz"},
	.parent_cnt = 2,
	.flags = 0,
};

static struct clk_init_def sama5_pmc_mainck = {
	.name = "mainck",
	.id = PMC_MAIN,
	.parent_names = {"main_xtal", "osc12mhz"},
	.parent_cnt = 2,
	.flags = 0,
};

static struct clk_init_def sama5_pmc_upllck = {
	.name = "upllck",
	.id = PMC_UTMI,
	.parent_names = {"mainck"},
	.parent_cnt = 1,
	.flags = 0,
};

static struct clk_init_def sama5_pmc_pllack = {
	.name = "pllack",
	.id = PMC_PLLA,
	.parent_names = {"mainck"},
	.parent_cnd = 1,
	.flags = 0,
};

static struct clk_init_def sama5_pmc_pck = {
	.name = "pck",
	.id = PMC_PCK,
	.parent_names = {},
};

static struct clk_init_def sama5_pmc_mck = {
	.name = "mck",
	.id = PMC_MCK,
	.parent_names = {"sclk", "mainck", "pllack", "upllck"},
	.parent_cnt = 4,
	.flags = 0,
};

static clknode_method_t sama5_clknode_core_methods[] = {
	CLKNODEMETHOD(clknode_init,		sama5_clknode_core_init),
	CLKNODEMETHOD(clknode_set_mux,		sama5_clknode_core_set_mux),
	CLKNODMETHOD_END
};

static clknode_method_t sama5_clknode_pck_methods[] = {
	CLKNODEMETHOD(clknode_init,		sama5_clknode_pck_init),
	CLKNODEMETHOD(clknode_recalc_freq,	sama5_clknode_pck_recalc_freq),
	CLKNODEMETHOD(clknode_set_gate,		sama5_clknode_pck_set_gate),
	CLKNODEMETHOD(clknode_set_mux,		sama5_clknode_pck_set_mux),
	CLKNODMETHOD_END
};

DEFINE_CLASS_1(sama5_core, sama5_core_class, sama5_clknode_core_methods,
    sizeof(struct sama5_clknode_core_softc), clknode_class);

DEFINE_CLASS_1(sama5_pck, sama5_pck_class, sama5_clknode_pck_methods,
    sizeof(struct sama5_clknode_pck_softc), clknode_class);

static inline uint32_t
RD4(struct sama5_pmc_softc *sc, bus_size_t off)
{
	return (bus_read_4(sc->mem_res, off));
}

static inline void
WR4(struct sama5_pmc_softc *sc, bus_size_t off, uint32_t val)
{
	bus_write_4(sc->mem_res, off, val);
}

#if 0
static void
sama5_pmc_set_upll_mode(struct sama5_pmc_clock *clk, int on)
{
	struct sama5_pmc_softc *sc = pmc_softc;
	uint32_t value;

	if (on) {
		on = PMC_IER_LOCKU;
		value = CKGR_UCKR_UPLLEN | CKGR_UCKR_BIASEN;
	} else
		value = 0;

	WR4(sc, CKGR_UCKR, RD4(sc, CKGR_UCKR) | value);
	while ((RD4(sc, PMC_SR) & PMC_IER_LOCKU) != on)
		continue;

	WR4(sc, PMC_USB, PMC_USB_USBDIV(9) | PMC_USB_USBS);
	WR4(sc, PMC_SCER, PMC_SCER_UHP_SAM9);
}

static void
sama5_pmc_set_sys_mode(struct sama5_pmc_clock *clk, int on)
{
	struct sama5_pmc_softc *sc = pmc_softc;

	WR4(sc, on ? PMC_SCER : PMC_SCDR, clk->pmc_mask);
	if (on) {
		while ((RD4(sc, PMC_SCSR) & clk->pmc_mask) != clk->pmc_mask)
			continue;
	} else {
		while ((RD4(sc, PMC_SCSR) & clk->pmc_mask) == clk->pmc_mask)
			continue;
	}
}

static void
sama5_pmc_set_periph_mode(struct sama5_pmc_clock *clk, int on)
{
	struct sama5_pmc_softc *sc = pmc_softc;

	WR4(sc, on ? PMC_PCER : PMC_PCDR, clk->pmc_mask);
	if (on) {
		while ((RD4(sc, PMC_PCSR) & clk->pmc_mask) != clk->pmc_mask)
			continue;
	} else {
		while ((RD4(sc, PMC_PCSR) & clk->pmc_mask) == clk->pmc_mask)
			continue;
	}
}

static int
sama5_pmc_pll_rate(struct sama5_pmc_clock *clk, uint32_t reg)
{
	uint32_t mul, div, freq;

	freq = clk->parent->hz;
	div = (reg >> clk->pll_div_shift) & clk->pll_div_mask;
	mul = (reg >> clk->pll_mul_shift) & clk->pll_mul_mask;

#if 0
	printf("pll = (%d /  %d) * %d = %d\n",
	    freq, div, mul + 1, (freq/div) * (mul+1));
#endif

	if (div != 0 && mul != 0) {
		freq /= div;
		freq *= mul + 1;
	} else
		freq = 0;
	clk->hz = freq;

	return (freq);
}

static uint32_t
sama5_pmc_pll_calc(struct sama5_pmc_clock *clk, uint32_t out_freq)
{
	uint32_t i, div = 0, mul = 0, diff = 1 << 30;

	unsigned ret = 0x3e00;

	if (out_freq > clk->pll_max_out)
		goto fail;

	for (i = 1; i < 256; i++) {
		int32_t diff1;
		uint32_t input, mul1;

		input = clk->parent->hz / i;
		if (input < clk->pll_min_in)
			break;
		if (input > clk->pll_max_in)
			continue;

		mul1 = out_freq / input;
		if (mul1 > (clk->pll_mul_mask + 1))
			continue;
		if (mul1 == 0)
			break;

		diff1 = out_freq - input * mul1;
		if (diff1 < 0)
			diff1 = -diff1;
		if (diff > diff1) {
			diff = diff1;
			div = i;
			mul = mul1;
			if (diff == 0)
				break;
		}
	}
	if (diff > (out_freq >> PMC_PLL_SHIFT_TOL))
		goto fail;

	if (clk->set_outb != NULL)
		ret |= clk->set_outb(out_freq);

	return (ret |
		((mul - 1) << clk->pll_mul_shift) |
		(div << clk->pll_div_shift));
fail:
	return (0);
}

#if !defined(AT91C_MAIN_CLOCK)
static const unsigned int sama5_main_clock_tbl[] = {
	3000000, 3276800, 3686400, 3840000, 4000000,
	4433619, 4915200, 5000000, 5242880, 6000000,
	6144000, 6400000, 6553600, 7159090, 7372800,
	7864320, 8000000, 9830400, 10000000, 11059200,
	12000000, 12288000, 13560000, 14318180, 14745600,
	16000000, 17344700, 18432000, 20000000
};
#define	MAIN_CLOCK_TBL_LEN	nitems(sama5_main_clock_tbl)
#endif

static unsigned int
sama5_pmc_sense_main_clock(void)
{
#if !defined(AT91C_MAIN_CLOCK)
	unsigned int ckgr_val;
	unsigned int diff, matchdiff, freq;
	int i;

	ckgr_val = (RD4(NULL, CKGR_MCFR) & CKGR_MCFR_MAINF_MASK) << 11;

	/*
	 * Clocks up to 50MHz can be connected to some models.  If
	 * the frequency is >= 21MHz, assume that the slow clock can
	 * measure it correctly, and that any error can be adequately
	 * compensated for by roudning to the nearest 500Hz.  Users
	 * with fast, or odd-ball clocks will need to set
	 * AT91C_MAIN_CLOCK in the kernel config file.
	 */
	if (ckgr_val >= 21000000)
		return (rounddown(ckgr_val + 250, 500));

	/*
	 * Try to find the standard frequency that match best.
	 */
	freq = sama5_main_clock_tbl[0];
	matchdiff = abs(ckgr_val - sama5_main_clock_tbl[0]);
	for (i = 1; i < MAIN_CLOCK_TBL_LEN; i++) {
		diff = abs(ckgr_val - sama5_main_clock_tbl[i]);
		if (diff < matchdiff) {
			freq = sama5_main_clock_tbl[i];
			matchdiff = diff;
		}
	}
	return (freq);
#else
	return (AT91C_MAIN_CLOCK);
#endif
}
#endif

static int
sama5_clknode_core_init(struct clknode *clk, device_t dev)
{
	struct sama5_clknode_core_softc *sc;

	return (0);
}

static int
sama5_clknode_core_set_gate(struct clknode *clk, bool enable)
{
	struct sama5_clknode_core_softc *sc;

	switch (sc->clkid) {
	case PMC_UPLLCK:

	case PMC_PLLACK:

	default:
		panic("Attempt to gate invalid clock");
	}

	return (0);
}

static int
sama5_clknode_core_set_mux(struct clknode *clk, int idx)
{
	struct sama5_clknode_core_softc *sc;
	uint32_t mor;
	uint32_t sckc_cr;

	switch (sc->clkid) {
	case PMC_MAINCK:
		mor = RD4(sc->pmc_softc, PMC_MOR);
		mor &= ~PMC_MOR_MOSCSEL;

		if (idx == PMC_MAIN_XTAL)
			mor |= PMC_MOR_MOSCSEL;
		
		WR4(sc->pmc_softc, PMC_MOR, mor);
		break;

	case PMC_SLOWCK:
		WR4(idx == PMC_SLOW_XTAL ? SCKC_CR_OSCSEL : 0,
		    SCKC_CR, sckr_cr);
		break;
	
	default:
		panic("Attempt to mux invalid clock");
	}

	return (0);
}

static int
sama5_clknode_per_init(struct clknode *clk, device_t dev)
{

	struct sama5_clknode_per_softc *sc;

	return (0);
}

static int
sama5_clknode_per_set_gate(struct clknode *clk, bool enable)
{
	struct sama5_clknode_per_softc *sc;

	return (0);
}

static int
sama5_clknode_per_set_mux(struct clknode *clk, int idx)
{

	struct sama5_clknode_per_softc *sc;

	return (0);
}

static int
sama5_pmc_create_core_clk(struct sama5_pmc_softc *sc, struct clkdef *clkdef)
{
	struct clknode *clk;
	struct sama5_clknode_core_softc *sc;

	clk = clknode_create(sc->clkdom, &sama5_core_class, clkdef);
	if (clk == NULL)
		return (-1);

	sc = clknode_get_softc(clk);
	sc->clkdev = clknode_get_device(clk);
	
	if (clknode_register(clkdom, clk) == NULL)
		return (-1)

	return (0);
}

static int
sama5_pmc_create_per_clk(struct sama5_pmc_softc *sc, struct clkdef *clkdef)
{
	struct clknode *clk;
	struct sama5_clknode_per_softc *sc;

	clk = clknode_create(sc->clkdom, &sama5_per_class, clkdef);
	if (clk == NULL)
		return (-1);

	sc = clknode_get_softc(clk);
	sc->clkdev = clknode_get_device(clk);

	if (clknode_register(clkdom, clk) == NULL)
		return (-1)

	return (0);
}

static int
sama5_pmc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "atmel,sama5d2-pmc"))
		return (ENXIO);
	
	device_set_desc(dev, "Atmel SAMA5 PMC");
	return (0);
}

static int
sama5_pmc_attach(device_t dev)
{
	struct sama5_pmc_softc *sc;
	int err;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->clkdom = clkdom_create(dev);
	
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL)
		goto errout;

	clknode_fixed_register(sc->clkdom, &sama5_pmc_osc32khz);
	clknode_fixed_register(sc->clkdom, &sama5_pmc_osc12mhz);
	sama5_pmc_create_core_clk(sc, &sama5_pmc_sclk);
	sama5_pmc_create_core_clk(sc, &sama5_pmc_mainck);
	sama5_pmc_create_core_clk(sc, &sama5_pmc_upllck);
	sama5_pmc_create_core_clk(sc, &sama5_pmc_pllack);

	/*
	 * Configure main clock frequency.
	 */
	sama5_pmc_init_clock();

	/*
	 * Display info about clocks previously computed
	 */
	device_printf(dev,
	    "Primary: %d Hz PLLA: %d MHz CPU: %d MHz MCK: %d MHz\n",
	    main_ck.hz,
	    plla.hz / 1000000,
	    cpu.hz / 1000000, mck.hz / 1000000);

	return (0);
}

static device_method_t sama5_pmc_methods[] = {
	DEVMETHOD(device_probe, sama5_pmc_probe),
	DEVMETHOD(device_attach, sama5_pmc_attach),
	DEVMETHOD_END
};

static driver_t sama5_pmc_driver = {
	"sama5_pmc",
	sama5_pmc_methods,
	sizeof(struct sama5_pmc_softc),
};
static devclass_t sama5_pmc_devclass;

EARLY_DRIVER_MODULE(sama5_pmc, simplebus, sama5_pmc_driver, sama5_pmc_devclass,
    NULL, NULL, BUS_PASS_CPU);
