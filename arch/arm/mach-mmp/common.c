/*
 *  linux/arch/arm/mach-mmp/common.c
 *
 *  Code common to PXA168 processor lines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <asm/page.h>
#include <asm/mach/map.h>
#include <mach/addr-map.h>

#include <asm/cacheflush.h>
#include "common.h"

static struct map_desc standard_io_desc[] __initdata = {
	{
		.pfn		= __phys_to_pfn(APB_PHYS_BASE),
		.virtual	= APB_VIRT_BASE,
		.length		= APB_PHYS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.pfn		= __phys_to_pfn(AXI_PHYS_BASE),
		.virtual	= AXI_VIRT_BASE,
		.length		= AXI_PHYS_SIZE,
		.type		= MT_DEVICE,
	},
};

void __init pxa_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));
}

void release_RIPC(void)
{
	RIPC0_STATUS = 1;
}
void get_RIPC(void)
{
	volatile unsigned long status ;

	status = RIPC0_STATUS;
	while(status!=0) {
		if (!in_atomic() && !irqs_disabled())
			schedule();
		else
			cpu_relax();
		status = RIPC0_STATUS;
	}
}

EXPORT_SYMBOL(dmac_flush_range);
EXPORT_SYMBOL(dmac_clean_range);
EXPORT_SYMBOL(dmac_inv_range);


