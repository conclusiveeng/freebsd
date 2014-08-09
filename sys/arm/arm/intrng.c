/*-
 * Copyright (c) 2012-2014 Jakub Wojciech Klama <jceel@FreeBSD.org>.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Mark Brinicombe
 *	for the NetBSD Project.
 * 4. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/syslog.h> 
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/conf.h>
#include <machine/atomic.h>
#include <machine/intr.h>
#include <machine/cpu.h>
#include <machine/smp.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "pic_if.h"

#define	IRQ_PIC_IDX(_irq)	((_irq >> 8) & 0xff)
#define	IRQ_VECTOR_IDX(_irq)	((_irq) & 0xff)
#define	IRQ_GEN(_pic, _irq)	(((_pic) << 8) | ((_irq) & 0xff))

#ifdef DEBUG
#define debugf(fmt, args...) do { printf("%s(): ", __func__);	\
    printf(fmt,##args); } while (0)
#else
#define debugf(fmt, args...)
#endif

typedef void (*mask_fn)(void *);

struct arm_intr_controller {
	device_t		ic_dev;
	phandle_t		ic_node;
};

struct arm_intr_handler {
	device_t		ih_dev;
	const char *		ih_ipi_name;
	int			ih_intrcnt_idx;
	int			ih_irq;
	struct intr_event *	ih_event;
	struct arm_intr_controller *ih_pic;
};

static void arm_mask_irq(void *);
static void arm_unmask_irq(void *);
static void arm_eoi(void *);

static struct arm_intr_handler arm_intrs[NIRQ];
static struct arm_intr_controller arm_pics[NPIC];
static struct arm_intr_controller *arm_ipi_pic;

static int intrcnt_index = 0;
static int last_printed = 0;

/* Data for statistics reporting. */
#define	INTRNAME_LEN	(MAXCOMLEN + 1)
u_long intrcnt[NIRQ];
char intrnames[NIRQ * INTRNAME_LEN];
size_t sintrcnt = sizeof(intrcnt);
size_t sintrnames = sizeof(intrnames);
int (*arm_config_irq)(int irq, enum intr_trigger trig,
    enum intr_polarity pol) = NULL;

void
arm_intrnames_init(void)
{
	/* nothing... */
}

void
arm_dispatch_irq(device_t dev, struct trapframe *tf, int irq)
{
	struct arm_intr_handler *ih = NULL;
	int i;

	debugf("pic %s, tf %p, irq %d\n", device_get_nameunit(dev), tf, irq);

	/*
	 * If we got null trapframe argument, that probably means
	 * a call from non-root interrupt controller. In that case,
	 * we'll just use the saved one.
	 */
	if (tf == NULL)
		tf = PCPU_GET(curthread)->td_intr_frame;

	for (i = 0; arm_intrs[i].ih_dev != NULL; i++) {
		if (arm_intrs[i].ih_pic->ic_dev == dev &&
		    arm_intrs[i].ih_irq == irq) {
			ih = &arm_intrs[i];
			break;
		}
	}

	if (ih == NULL)
		panic("arm_dispatch_irq: unknown irq");

	debugf("requested by %s\n", ih->ih_ipi_name != NULL
	    ? ih->ih_ipi_name
	    : device_get_nameunit(ih->ih_dev));

	intrcnt[ih->ih_intrcnt_idx]++;
	if (intr_event_handle(ih->ih_event, tf) != 0) {
		/* Stray IRQ */
		arm_mask_irq(ih);
	}

	debugf("done\n");
}

static struct arm_intr_handler *
arm_lookup_intr_handler(device_t pic, int irq)
{
	int i;

	for (i = 0; i < NIRQ; i++) {
		if (arm_intrs[i].ih_pic != NULL &&
		    arm_intrs[i].ih_pic->ic_dev == pic &&
		    arm_intrs[i].ih_irq == irq)
			return (&arm_intrs[i]);

		if (arm_intrs[i].ih_dev == NULL)
			return (&arm_intrs[i]);
	}

	return NULL;
}

int
arm_fdt_map_irq(phandle_t ic, int irq)
{
	int i;

	ic = OF_xref_phandle(ic);

	debugf("ic %08x irq %d\n", ic, irq);

	if (ic == CORE_PIC_NODE)
		return (IRQ_GEN(CORE_PIC_IDX, irq));

	for (i = 0; arm_pics[i].ic_node != 0; i++) {
		if (arm_pics[i].ic_node	== ic)
			return (IRQ_GEN(i, irq));
	}

	/* 
	 * Interrupt controller is not registered yet, so
	 * we map a stub for it. 'i' is pointing to free
	 * first slot in arm_pics table.
	 */
	arm_pics[i].ic_node = ic;
	return (IRQ_GEN(i, irq));
}

const char *
arm_describe_irq(int irq)
{
	struct arm_intr_controller *pic;
	static char buffer[32];
	static char name[32];

	pic = &arm_pics[IRQ_PIC_IDX(irq)];

	if (pic->ic_dev == NULL) {
		/*
		 * Interrupt controller not attached yet, so we'll use it's
		 * FDT "name" property instead
		 */
		OF_getprop(pic->ic_node, "name", name, sizeof(name));
		sprintf(buffer, "%s.%d", name, IRQ_VECTOR_IDX(irq));
		return (buffer);
	}

	sprintf(buffer, "%s.%d", device_get_nameunit(pic->ic_dev),
	    IRQ_VECTOR_IDX(irq));

	return (buffer);
}

void
arm_register_pic(device_t dev, int flags)
{
	struct arm_intr_controller *ic = NULL;
	phandle_t node;
	int i;

	node = ofw_bus_get_node(dev);

	/* Find room for IC */
	for (i = 0; i < NPIC; i++) {
		if (arm_pics[i].ic_dev != NULL)
			continue;

		if (arm_pics[i].ic_node == node) {
			ic = &arm_pics[i];
			break;
		}

		if (arm_pics[i].ic_node == 0) {
			ic = &arm_pics[i];
			break;
		}
	}

	if (ic == NULL)
		panic("not enough room to register interrupt controller");

	ic->ic_dev = dev;
	ic->ic_node = node;

	debugf("device %s node %08x slot %d\n", device_get_nameunit(dev), ic->ic_node, i);

	if (flags & PIC_FEATURE_IPI) {
		if (arm_ipi_pic != NULL)
			panic("there's already registered interrupt controller for serving IPIs");

		arm_ipi_pic = ic;
	}

	device_printf(dev, "registered as interrupt controller\n");
}

void
arm_setup_irqhandler(device_t dev, driver_filter_t *filt, 
    void (*hand)(void*), void *arg, int irq, int flags, void **cookiep)
{
	struct arm_intr_controller *pic;
	struct arm_intr_handler *ih;
	const char *name;
	int error;
	int ipi;

	if (irq < 0)
		return;

	ipi = (flags & INTR_IPI) != 0;
	pic = ipi ? arm_ipi_pic : &arm_pics[IRQ_PIC_IDX(irq)];
	ih = arm_lookup_intr_handler(pic->ic_dev, IRQ_VECTOR_IDX(irq));

	if (ipi) {
		name = (const char *)dev;
		debugf("setup ipi %d\n", irq);
	} else {
		name = device_get_nameunit(dev);
		debugf("setup irq %d on %s\n", IRQ_VECTOR_IDX(irq),
		    device_get_nameunit(pic->ic_dev));
	}

	debugf("pic %p, ih %p\n", pic, ih);

	if (ih->ih_event == NULL) {
		error = intr_event_create(&ih->ih_event, (void *)ih, 0, irq,
		    (mask_fn)arm_mask_irq, (mask_fn)arm_unmask_irq,
		    arm_eoi, NULL, "intr%d:", irq);
		
		if (error)
			return;

		ih->ih_dev = dev;
		ih->ih_ipi_name = ipi ? name : NULL;
		ih->ih_irq = IRQ_VECTOR_IDX(irq);
		ih->ih_pic = pic;

		arm_unmask_irq(ih);

		last_printed += 
		    snprintf(intrnames + last_printed,
		    MAXCOMLEN + 1, "%s:%d: %s",
		    device_get_nameunit(pic->ic_dev),
		    ih->ih_irq, name);
		
		last_printed++;
		ih->ih_intrcnt_idx = intrcnt_index;
		intrcnt_index++;
		
	}

	intr_event_add_handler(ih->ih_event, name, filt, hand, arg,
	    intr_priority(flags), flags, cookiep);

	/* Unmask IPIs immediately */
	if (ipi)
		arm_unmask_irq(ih);
}

int
arm_remove_irqhandler(int irq, void *cookie)
{
	struct arm_intr_controller *pic;
	struct arm_intr_handler *ih;
	int error;

	if (irq < 0)
		return (ENXIO);

	pic = &arm_pics[IRQ_PIC_IDX(irq)];
	ih = arm_lookup_intr_handler(pic->ic_dev, IRQ_VECTOR_IDX(irq));

	if (ih->ih_event == NULL)
		return (ENXIO);

	arm_mask_irq(ih);
	error = intr_event_remove_handler(cookie);

	if (!TAILQ_EMPTY(&ih->ih_event->ie_handlers))
		arm_unmask_irq(ih);

	return (error);
}

static void
arm_mask_irq(void *arg)
{
	struct arm_intr_handler *ih = (struct arm_intr_handler *)arg;

	PIC_MASK(ih->ih_pic->ic_dev, ih->ih_irq);
}

static void
arm_unmask_irq(void *arg)
{
	struct arm_intr_handler *ih = (struct arm_intr_handler *)arg;

	PIC_UNMASK(ih->ih_pic->ic_dev, ih->ih_irq);
}

static void
arm_eoi(void *arg)
{
	struct arm_intr_handler *ih = (struct arm_intr_handler *)arg;

	PIC_EOI(ih->ih_pic->ic_dev, ih->ih_irq);
}

int
arm_intrng_config_irq(int irq, enum intr_trigger trig, enum intr_polarity pol)
{
	struct arm_intr_controller *pic;
	struct arm_intr_handler *ih;

	pic = &arm_pics[IRQ_PIC_IDX(irq)];
	ih = arm_lookup_intr_handler(pic->ic_dev, IRQ_VECTOR_IDX(irq));

	if (ih == NULL)
		return (ENXIO);

	return PIC_CONFIG(pic->ic_dev, ih->ih_irq, trig, pol);
}

#ifdef SMP
void
arm_init_secondary_ic(void)
{

	KASSERT(arm_ipi_pic != NULL, ("no IPI PIC attached"));
	PIC_INIT_SECONDARY(arm_ipi_pic->ic_dev);
}

void
pic_ipi_send(cpuset_t cpus, u_int ipi)
{

	KASSERT(arm_ipi_pic != NULL, ("no IPI PIC attached"));
	PIC_IPI_SEND(arm_ipi_pic->ic_dev, cpus, ipi);
}

void
pic_ipi_clear(int ipi)
{
	
	KASSERT(arm_ipi_pic != NULL, ("no IPI PIC attached"));
	PIC_IPI_CLEAR(arm_ipi_pic->ic_dev, ipi);
}

int
pic_ipi_read(int ipi)
{

	KASSERT(arm_ipi_pic != NULL, ("no IPI PIC attached"));
	return (PIC_IPI_READ(arm_ipi_pic->ic_dev, ipi));
}

void
arm_unmask_ipi(int ipi)
{

	KASSERT(arm_ipi_pic != NULL, ("no IPI PIC attached"));
	PIC_UNMASK(arm_ipi_pic->ic_dev, ipi);
}

void
arm_mask_ipi(int ipi)
{

	KASSERT(arm_ipi_pic != NULL, ("no IPI PIC attached"));
	PIC_MASK(arm_ipi_pic->ic_dev, ipi);
}
#endif

void dosoftints(void);
void
dosoftints(void)
{
}

