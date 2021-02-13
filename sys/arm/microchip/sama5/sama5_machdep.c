/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Conclusive Engineering
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
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
#include <sys/bus.h>
#include <sys/devmap.h>

#include <vm/vm.h>

#include <dev/ofw/openfirm.h>

#include <machine/armreg.h>
#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/machdep.h>
#include <machine/platform.h> 
#include <machine/platformvar.h>

#include "platform_if.h"

static platform_devmap_init_t sama5_devmap_init;
static platform_cpu_reset_t sama5_cpu_reset;

static int
sama5_devmap_init(platform_t plat)
{

	devmap_add_entry(0xfc069000, 0x1000); /* CHIPID */
	devmap_add_entry(0xf8020000, 0x4000); /* UART */
	return (0);
}

static void
sama5_cpu_reset(platform_t plat)
{
	bus_space_handle_t bsh;

	bus_space_map(fdtbus_bs_tag, 0xf8048000, 0x1000, 0, &bsh);
	bus_space_write_4(fdtbus_bs_tag, bsh, 0, 0xa5000001);

	while (1);
}

static platform_method_t sama5_methods[] = {
	PLATFORMMETHOD(platform_devmap_init,	sama5_devmap_init),
	PLATFORMMETHOD(platform_cpu_reset,	sama5_cpu_reset),
	PLATFORMMETHOD_END,
};

FDT_PLATFORM_DEF(sama5, "sama5", 0, "atmel,sama5d2", 200);
