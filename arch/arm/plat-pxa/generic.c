/*
 *  linux/arch/arm/plat-pxa/generic.c
 *
 *  Code to PXA processor lines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <asm/page.h>

#include <plat/generic.h>

static int android_project = 0;
static int __init android_setup(char *__unused)
{
	android_project = 1;
	return 1;
}
__setup("android", android_setup);

int is_android(void)
{
	return android_project;
}
EXPORT_SYMBOL(is_android);

static int lab_kernel = 0;
static int __init lab_setup(char *__unused)
{
	lab_kernel = 1;
	return 1;
}
__setup("lab", lab_setup);

int is_lab(void)
{
	return lab_kernel;
}
EXPORT_SYMBOL(is_lab);

static unsigned long long g_mlc_size = 1; /*FIXME: default is 4G ? */
static int __init mlc_size_setup(char *str)
{
	char *endchar;
	if (NULL == str) {
		return 1; /* FIXME*/
	}
	str++; /*skip = */

	/* g_mlc_size = memparse(str, &endchar); */
	g_mlc_size = simple_strtoull(str, &endchar, 0);

	return 1;
}
__setup("pxastorage", mlc_size_setup);

int get_mlc_size(void)
{
	return g_mlc_size;
}
EXPORT_SYMBOL(get_mlc_size);

#ifdef CONFIG_ANDROID_PMEM
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
void android_add_pmem(char *name, size_t size, int no_allocator, int cached)
{
	struct platform_device *android_pmem_device;
	struct android_pmem_platform_data *android_pmem_pdata;
	struct page *page;
	unsigned long addr, tmp;
	static int id;
	unsigned long paddr = 0;

	android_pmem_device = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if(android_pmem_device == NULL)
		return ;

	android_pmem_pdata = kzalloc(sizeof(struct android_pmem_platform_data), GFP_KERNEL);
	if(android_pmem_pdata == NULL) {
		kfree(android_pmem_device);
		return ;
	}
	
	page = alloc_pages(GFP_KERNEL, get_order(size));
	if (page == NULL)
		return ;

	addr = (unsigned long)page_address(page);
	paddr = virt_to_phys((void *)addr);
	tmp = size;
	dma_cache_maint(addr, size, DMA_FROM_DEVICE);
	while(tmp > 0) {
		SetPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		tmp -= PAGE_SIZE;
	}
	android_pmem_pdata->name = name;
	android_pmem_pdata->start = paddr;
	android_pmem_pdata->size = size;
	android_pmem_pdata->no_allocator = no_allocator ;
	android_pmem_pdata->cached = cached;

	android_pmem_device->name = "android_pmem";
	android_pmem_device->id = id++;
	android_pmem_device->dev.platform_data = android_pmem_pdata;

	platform_device_register(android_pmem_device);
}
EXPORT_SYMBOL(android_add_pmem);
#endif

