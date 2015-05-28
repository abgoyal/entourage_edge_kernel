#ifndef __ASM_MACH_DEVICES_H
#define __ASM_MACH_DEVICES_H
#include <linux/types.h>
#include <linux/platform_device.h>

#define MAX_RESOURCE_DMA	2

struct pxa_device_desc {
	const char	*dev_name;
	const char	*drv_name;
	int		id;
	int		irq;
	unsigned long	start;
	unsigned long	size;
	int		dma[MAX_RESOURCE_DMA];
};

#define PXA168_DEVICE(_name, _drv, _id, _irq, _start, _size, _dma...)	\
struct pxa_device_desc pxa168_device_##_name __initdata = {		\
	.dev_name	= "pxa168-" #_name,				\
	.drv_name	= _drv,						\
	.id		= _id,						\
	.irq		= IRQ_PXA168_##_irq,				\
	.start		= _start,					\
	.size		= _size,					\
	.dma		= { _dma },					\
};

#define PXA168_DEVICE_M(_name, _drv, _id, _irq, _start, _size, _dma...)	\
struct pxa_device_desc pxa168_device_##_name = {		\
	.dev_name	= "pxa168-" #_name,				\
	.drv_name	= _drv,						\
	.id		= _id,						\
	.irq		= IRQ_PXA168_##_irq,				\
	.start		= _start,					\
	.size		= _size,					\
	.dma		= { _dma },					\
};									\
EXPORT_SYMBOL(pxa168_device_##_name);

#define PXA910_DEVICE(_name, _drv, _id, _irq, _start, _size, _dma...)   \
struct pxa_device_desc pxa910_device_##_name __initdata = {             \
	.dev_name       = "pxa910-" #_name,                             \
	.drv_name       = _drv,                                         \
	.id             = _id,                                          \
	.irq            = IRQ_PXA910_##_irq,                            \
	.start          = _start,                                       \
	.size           = _size,                                        \
	.dma            = { _dma },                                     \
};

#define PXA910_DEVICE_M(_name, _drv, _id, _irq, _start, _size, _dma...)   \
struct pxa_device_desc pxa910_device_##_name = {             \
	.dev_name       = "pxa910-" #_name,                             \
	.drv_name       = _drv,                                         \
	.id             = _id,                                          \
	.irq            = IRQ_PXA910_##_irq,                            \
	.start          = _start,                                       \
	.size           = _size,                                        \
	.dma            = { _dma },                                     \
};									\
EXPORT_SYMBOL(pxa910_device_##_name);

extern struct platform_device pxa168_device_u2o;
extern struct platform_device pxa168_device_u2h;
extern struct platform_device pxa168_device_u2oehci;
extern struct platform_device pxa168_device_u2ootg;
extern struct pxa_device_desc pxa168_device_uart1;
extern struct pxa_device_desc pxa168_device_uart2;
extern struct pxa_device_desc pxa168_device_uart3;
extern struct pxa_device_desc pxa168_device_fb;
extern struct pxa_device_desc pxa168_device_fb_ovly;
extern struct pxa_device_desc pxa168_device_twsi0;
extern struct pxa_device_desc pxa168_device_twsi1;
extern struct pxa_device_desc pxa168_device_ssp0;
extern struct pxa_device_desc pxa168_device_ssp1;
extern struct pxa_device_desc pxa168_device_ssp2;
extern struct pxa_device_desc pxa168_device_ssp3;
extern struct pxa_device_desc pxa168_device_ssp4;
extern struct pxa_device_desc pxa168_device_keypad;
extern struct pxa_device_desc pxa168_device_nand;
extern struct pxa_device_desc pxa168_device_onenand;
extern struct pxa_device_desc pxa168_device_mfu;
extern struct pxa_device_desc pxa168_device_sdh0;
extern struct pxa_device_desc pxa168_device_sdh1;
extern struct pxa_device_desc pxa168_device_sdh2;
extern struct pxa_device_desc pxa168_device_sdh3;
extern struct platform_device pxa168_device_freq;
extern struct platform_device pxa168_device_cir;
extern struct pxa_device_desc pxa168_device_camera;
extern struct pxa_device_desc pxa168_device_msp;
extern struct pxa_device_desc pxa168_device_cf;
extern struct pxa_device_desc pxa168_device_icr;

/*PXA910 Specific*/
extern struct pxa_device_desc pxa910_device_uart1;
extern struct pxa_device_desc pxa910_device_uart2;
extern struct pxa_device_desc pxa910_device_uart3;
extern struct platform_device pxa910_device_acipc;
extern struct pxa_device_desc pxa910_device_ire;
extern struct pxa_device_desc pxa910_device_ssp0;
extern struct pxa_device_desc pxa910_device_ssp1;
extern struct pxa_device_desc pxa910_device_ssp2;
extern struct platform_device pxa910_device_imm;
extern struct platform_device pxa910_device_rtc;
extern struct pxa_device_desc pxa910_device_twsi0;
extern struct pxa_device_desc pxa910_device_twsi1;
extern struct platform_device pxa168_device_pwm0;
extern struct platform_device pxa168_device_pwm1;
extern struct platform_device pxa168_device_pwm2;
extern struct platform_device pxa168_device_pwm3;

extern int pxa_register_device(struct pxa_device_desc *, void *, size_t);
extern int pxa168_usb_phy_init(unsigned base);
extern int pxa168_usb_phy_deinit(unsigned base);

#endif /* __ASM_MACH_DEVICES_H */
