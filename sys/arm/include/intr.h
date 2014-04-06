/* 	$NetBSD: intr.h,v 1.7 2003/06/16 20:01:00 thorpej Exp $	*/

/*-
 * Copyright (c) 1997 Mark Brinicombe.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 *
 */

#ifndef _MACHINE_INTR_H_
#define _MACHINE_INTR_H_

#include <machine/psl.h>
#include <machine/pcb.h>
#include <dev/ofw/openfirm.h>

#include "opt_global.h"

#if defined(ARM_INTRNG)

#define	NIRQ		255
#define	NPIC		16
#define	INTR_CONTROLLER	INTR_MD1
#define	INTR_IPI	INTR_MD2
#define	CORE_PIC_IDX	(0)
#define	CORE_PIC_NODE	(0xffffffff)

/* Interrupt controller features used in arm_register_pic(): */
#define	PIC_FEATURE_IPI	0x1

int arm_fdt_map_irq(phandle_t ic, int irq);
void arm_register_pic(device_t dev, int features);
void arm_unregister_pic(device_t dev);
void arm_dispatch_irq(device_t dev, struct trapframe *tf, int irq);
void arm_setup_irqhandler(device_t dev, int (*)(void*), void (*)(void*), 
    void *, int, int, void **);
int arm_remove_irqhandler(int, void *);
int arm_intrng_config_irq(int, enum intr_trigger, enum intr_polarity);

#ifdef SMP
void arm_init_secondary_ic(void);
void arm_unmask_ipi(int);
void arm_mask_ipi(int);
#endif

#else

/* XXX move to std.* files? */
#ifdef CPU_XSCALE_81342
#define NIRQ		128
#elif defined(CPU_XSCALE_PXA2X0)
#include <arm/xscale/pxa/pxareg.h>
#define	NIRQ		IRQ_GPIO_MAX
#elif defined(SOC_MV_DISCOVERY)
#define NIRQ		96
#elif defined(CPU_ARM9) || defined(SOC_MV_KIRKWOOD) || \
    defined(CPU_XSCALE_IXP435)
#define NIRQ		64
#elif defined(CPU_CORTEXA)
#define NIRQ		160
#elif defined(CPU_KRAIT)
#define NIRQ		288
#elif defined(CPU_ARM1136) || defined(CPU_ARM1176)
#define NIRQ		128
#elif defined(SOC_MV_ARMADAXP)
#define MAIN_IRQ_NUM		116
#define ERR_IRQ_NUM		32
#define ERR_IRQ			(MAIN_IRQ_NUM)
#define MSI_IRQ_NUM		32
#define MSI_IRQ			(ERR_IRQ + ERR_IRQ_NUM)
#define NIRQ			(MAIN_IRQ_NUM + ERR_IRQ_NUM + MSI_IRQ_NUM)
#else
#define NIRQ		32
#endif


int arm_get_next_irq(int);
void arm_mask_irq(uintptr_t);
void arm_unmask_irq(uintptr_t);
void arm_setup_irqhandler(const char *, int (*)(void*), void (*)(void*),
    void *, int, int, void **);
int arm_remove_irqhandler(int, void *);
extern void (*arm_post_filter)(void *);
void gic_init_secondary(void);

#endif	/* !ARM_INTRNG */

extern int (*arm_config_irq)(int irq, enum intr_trigger trig,
    enum intr_polarity pol);
const char *arm_describe_irq(int irq);
void arm_intrnames_init(void);

#endif	/* _MACHINE_INTR_H */
