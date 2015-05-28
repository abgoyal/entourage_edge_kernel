/*
 * linux/arch/arm/mach-mmp/include/mach/regs-apbc.h
 *
 *   Application Peripheral Bus Clock Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_APBC_H
#define __ASM_MACH_REGS_APBC_H

#include <mach/addr-map.h>

#define APBC_VIRT_BASE	(APB_VIRT_BASE + 0x015000)
#define APBCP_VIRT_BASE	(APB_VIRT_BASE + 0x03B000)
#define APBC_REG(x)	(APBC_VIRT_BASE + (x))
#define APBCP_REG(x)	(APBCP_VIRT_BASE + (x))

/*
 * APB clock registers PXA910
 */
#define APBC_PXA910_ACIPC       APBC_REG(0x0024)
#define APBC_PXA910_SSP1	APBC_REG(0x0020)
#define APBC_PXA910_RIPC       APBCP_REG(0x0038)
/*
 * APB clock registers PXA168
 */
#define APBC_PXA168_UART0	APBC_REG(0x0000)
#define APBC_PXA168_UART1	APBC_REG(0x0004)
#define APBC_PXA168_GPIO	APBC_REG(0x0008)
#define APBC_PXA168_PWM0	APBC_REG(0x000c)
#define APBC_PXA168_PWM1	APBC_REG(0x0010)
#define APBC_PXA168_PWM2	APBC_REG(0x0014)
#define APBC_PXA168_PWM3	APBC_REG(0x0018)
#define APBC_PXA168_RTC		APBC_REG(0x0028)
#define APBC_PXA168_TWSI0	APBC_REG(0x002c)
#define APBC_PXA168_TWSI1	APBC_REG(0x006c) /* Power I2C */
#define APBC_PXA168_KPC		APBC_REG(0x0030)
#define APBC_PXA168_TIMERS	APBC_REG(0x0034)
#define APBC_PXA168_TB_ROTARY	APBC_REG(0x0038)
#define APBC_PXA168_AIB		APBC_REG(0x003c)
#define APBC_PXA168_SW_JTAG	APBC_REG(0x0040)
#define APBC_PXA168_TIMER1	APBC_REG(0x0044)
#define APBC_PXA168_ONEWIRE	APBC_REG(0x0048)
#define APBC_PXA168_ASFAR	APBC_REG(0x0050)
#define APBC_PXA168_ASSAR	APBC_REG(0x0054)
#define APBC_PXA168_UART2	APBC_REG(0x0070)
#define APBC_PXA168_TIMER2	APBC_REG(0x007c)
#define APBC_PXA168_AC97	APBC_REG(0x0084)

#define APBC_PXA168_SSP0	APBC_REG(0x081c)
#define APBC_PXA168_SSP1	APBC_REG(0x0820)
#define APBC_PXA168_SSP2	APBC_REG(0x084c)
#define APBC_PXA168_SSP3	APBC_REG(0x0858)
#define APBC_PXA168_SSP4	APBC_REG(0x085c)

#define APBC_PXA910_UART2       APBCP_REG(0x001c)
#define APBC_PXA910_TWSI0	APBC_REG(0x002c)
#define APBC_PXA910_TWSI1	APBCP_REG(0x0028) /* Power I2C */

#define APBC_APBCLK	(1 << 0)  /* APB Bus Clock Enable */
#define APBC_FNCLK	(1 << 1)  /* Functional Clock Enable */
#define APBC_RST	(1 << 2)  /* Reset Generation */

/* Functional Clock Selection Mask */
#define APBC_FNCLKSEL(x)	(((x) & 0xf) << 4)

/* UART High Speed Multiple of Regular Speed */
#define APBC_UART_HS_MULTI	4

#endif /* __ASM_MACH_REGS_APBC_H */
