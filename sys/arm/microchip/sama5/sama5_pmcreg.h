/*-
 * Copyright (c) 2020 Jakub Klama
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

/* $FreeBSD$ */

#ifndef ARM_MICROCHIP_SAMA5_SAMA5_PMCREG_H
#define	ARM_MICROCHIP_SAMA5_SAMA5_PMCREG_H

/* Clock node identifiers */
#define	PMC_OSC32KHZ	1
#define	PMC_OSC12MHZ	2
#define	PMC_SLOW	3
#define	PMC_MAIN	4
#define	PMC_UTMI	5
#define	PMC_PLLA	6
#define	PMC_PCK		7
#define	PMC_MCK		8

#define	PMC_SCER	0x00		/* System Clock Enable Register */
#define	PMC_SCDR	0x04		/* System Clock Disable Register */
#define	PMC_SCSR	0x08		/* System Clock Status Register */
#define	PMC_PCER	0x10		/* Peripheral Clock Enable Register 0 */
#define	PMC_PCDR	0x14		/* Peripheral Clock Disable Register 0 */
#define	PMC_PCSR	0x18		/* Peripheral Clock Status Register 0 */
#define	CKGR_UCKR	0x1c		/* UTMI Clock Configuration Register */
#define CKGR_MOR	0x20		/* Main Oscillator Register */
#define CKGR_MCFR	0x24		/* Main Clock Frequency Register */
#define CKGR_PLLAR	0x28		/* PLL A Register */
#define PMC_MCKR	0x30		/* Master Clock Register */
#define	PMC_USB		0x38		/* USB Clock Register */
#define PMC_PCK0	0x40		/* Programmable Clock 0 Register */
#define PMC_PCK1	0x44		/* Programmable Clock 1 Register */
#define PMC_PCK2	0x48		/* Programmable Clock 2 Register */
#define PMC_IER		0x60		/* Interrupt Enable Register */
#define PMC_IDR		0x64		/* Interrupt Disable Register */
#define PMC_SR		0x68		/* Status Register */
#define PMC_IMR		0x6c		/* Interrupt Mask Register */
#define	PMC_FSMR	0x70		/* Fast Startup Mode Register */
#define	PMC_FSPR	0x74		/* Fast Startup Polarity Register */
#define	PMC_FOCR	0x78		/* Fast Startup Clear Register */
#define	PMC_PLLICPR	0x80		/* PLL Charge Pump Current Register */
#define	PMC_WPMR	0xe4		/* Write Protection Mode Register */
#define	PMC_WPSR	0xe8		/* Write Protection Status Register */
#define	PMC_PCER1	0x100		/* Peripheral Clock Enable Register 0 */
#define	PMC_PCDR1	0x104		/* Peripheral Clock Disable Register 0 */
#define	PMC_PCSR1	0x108		/* Peripheral Clock Status Register 0 */
#define	PMC_OCR		0x110		/* Oscillator Calibration Register */

/* PMC System Clock Enable Register */
/* PMC System Clock Disable Register */
/* PMC System Clock Status Register */
#define PMC_SCER_PCK	(1UL << 0)	/* PCK: Processor Clock Enable */
#define PMC_SCER_UDP	(1UL << 1)	/* UDP: USB Device Port Clock Enable */
#define PMC_SCER_MCKUDP	(1UL << 2)	/* MCKUDP: Master disable susp/res */
#define PMC_SCER_UHP	(1UL << 4)	/* UHP: USB Host Port Clock Enable */
#define PMC_SCER_PCK0	(1UL << 8)	/* PCK0: Programmable Clock out en */
#define PMC_SCER_PCK1	(1UL << 9)	/* PCK1: Programmable Clock out en */
#define PMC_SCER_PCK2	(1UL << 10)	/* PCK2: Programmable Clock out en */
#define PMC_SCER_PCK3	(1UL << 11)	/* PCK3: Programmable Clock out en */
#define PMC_SCER_UHP_SAM9 (1UL << 6)	/* UHP: USB Host Port Clock Enable */
#define PMC_SCER_UDP_SAM9 (1UL << 7)	/* UDP: USB Device Port Clock Enable */

/* PMC Clock Generator Master Clock Register */
#define PMC_MCKR_PDIV      (1 << 12)			/* SAM9G20 Only */
#define PMC_MCKR_PLLADIV2  (1 << 12)			/* SAM9G45 Only */
#define PMC_MCKR_CSS_MASK  (3 << 0)		
#define PMC_MCKR_MDIV_MASK (3 << 8)		
#define PMC_MCKR_PRES_MASK (7 << 2)		

#endif /* ARM_MICROCHIP_SAMA5_SAMA5_PMCREG_H */
