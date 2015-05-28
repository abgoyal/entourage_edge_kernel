/*
 *  linux/arch/arm/mach-mmp/ipcam.c
 *
 *  Support for the Marvell PXA168-based IPCAM Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/mtd/partitions.h>
#include <linux/android_pmem.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <plat/pxa3xx_pmic.h>
#include <plat/pxa_u2o.h>

#include <plat/generic.h>
#include <mach/mmc.h>
#include <mach/camera.h>

#include "common.h"
#include <linux/delay.h>

/* MFP configurations */
static unsigned long ipcam_pin_config[] __initdata = {
	/* DEBUG UART3 */
	MFP_CFG(GPIO30, AF2),
	MFP_CFG(GPIO31, AF2),

	/* MMC3_CD */
	MFP_CFG(GPIO2, AF5),

	/* MMC3 DAT3-0/CLK/CMD */
	MFP_CFG(GPIO4,	AF6),
	MFP_CFG(GPIO5,	AF6),
	MFP_CFG(GPIO6,	AF6),
	MFP_CFG(GPIO7,	AF6),
	MFP_CFG(GPIO35,	AF6),
	MFP_CFG(GPIO36,	AF6),

	/* MMC4_CD */
	MFP_CFG(GPIO77, AF0),

	/* MMC4 DAT3-0/CLK/CMD */
	MFP_CFG(GPIO78,	AF5),
	MFP_CFG(GPIO79,	AF5),
	MFP_CFG(GPIO80,	AF5),
	MFP_CFG(GPIO81,	AF5),
	MFP_CFG(GPIO82,	AF5),
	MFP_CFG(GPIO83,	AF5),

	/* SMC */
	MFP_CFG(GPIO8,	AF0),
	MFP_CFG(GPIO9,	AF0),
	MFP_CFG(GPIO10,	AF0),
	MFP_CFG(GPIO11,	AF0),
	MFP_CFG(GPIO12,	AF0),
	MFP_CFG(GPIO13,	AF0),
	MFP_CFG(GPIO14,	AF0),
	MFP_CFG(GPIO15,	AF0),
	MFP_CFG(GPIO19,	AF0),
	MFP_CFG(GPIO21,	AF0),
	MFP_CFG(GPIO22,	AF0),
	MFP_CFG(GPIO28,	AF0),
	MFP_CFG(GPIO33,	AF0),

	/* Fast Ethernet */
	MFP_CFG(GPIO86,	AF5),
	MFP_CFG(GPIO87,	AF5),
	MFP_CFG(GPIO88,	AF5),
	MFP_CFG(GPIO89,	AF5),
	MFP_CFG(GPIO90,	AF5),
	MFP_CFG(GPIO91,	AF5),
	MFP_CFG(GPIO92,	AF5),
	MFP_CFG(GPIO93,	AF5),
	MFP_CFG(GPIO94,	AF5),
	MFP_CFG(GPIO95,	AF5),
	MFP_CFG(GPIO96,	AF5),
	MFP_CFG(GPIO97,	AF5),
	MFP_CFG(GPIO98,	AF5),
	MFP_CFG(GPIO99,	AF5),
	MFP_CFG(GPIO100, AF5),
	MFP_CFG(GPIO101, AF5),
	MFP_CFG(GPIO103, AF5),
	/* ENET_RESET_N */
	MFP_CFG(GPIO102, AF0),
	/* ENET_COMA_N */
	MFP_CFG(GPIO60, AF0),

	/* I2C */
	MFP_CFG(GPIO105, AF1),
	MFP_CFG(GPIO106, AF1),

	/* USB_C_HPEN */
	MFP_CFG(GPIO85,	AF7),

	/* board ID */
	MFP_CFG(GPIO20,	AF0),
	MFP_CFG(GPIO84,	AF0),

	/* not used */
	MFP_CFG(GPIO16, AF0),
	MFP_CFG(GPIO17, AF0),
	MFP_CFG(GPIO18, AF0),
	MFP_CFG(GPIO23, AF0),
	MFP_CFG(GPIO24, AF0),
	MFP_CFG(GPIO25, AF0),
	MFP_CFG(GPIO26, AF0),
	MFP_CFG(GPIO27, AF0),
	MFP_CFG(GPIO29, AF0),
	MFP_CFG(GPIO32, AF0),
	MFP_CFG(GPIO34, AF0),
	MFP_CFG(GPIO43, AF0),
	MFP_CFG(GPIO47, AF0),
	MFP_CFG(GPIO49, AF0),
	MFP_CFG(GPIO51, AF0),
	MFP_CFG(GPIO52, AF0),
	MFP_CFG(GPIO53, AF0),
	MFP_CFG(GPIO104, AF0),
	MFP_CFG(GPIO109, AF0),
	MFP_CFG(GPIO110, AF0),
	MFP_CFG(GPIO118, AF0),
	MFP_CFG(GPIO119, AF0),
	MFP_CFG(GPIO120, AF0),
	MFP_CFG(GPIO121, AF0),
	MFP_CFG(GPIO122, AF0),

	/* Camera */
	MFP_CFG(GPIO37, AF4),
	MFP_CFG(GPIO38, AF4),
	MFP_CFG(GPIO39, AF4),
	MFP_CFG(GPIO40, AF4),
	MFP_CFG(GPIO41, AF4),
	MFP_CFG(GPIO42, AF4),
	MFP_CFG(GPIO44, AF4),
	MFP_CFG(GPIO45, AF4),
	MFP_CFG(GPIO46, AF4),
	MFP_CFG(GPIO48, AF4),
	MFP_CFG(GPIO50, AF4),
	MFP_CFG(GPIO54, AF4),
	MFP_CFG(GPIO55, AF4),

	/* SSP2, SPI NOR */
	GPIO107_SPI_NOR_RXD,
	GPIO108_SPI_NOR_TXD,
	GPIO110_GPIO,
	GPIO111_SPI_NOR_CLK,

	/* SSP1, AUDIO */
	MFP_CFG(GPIO113, AF0),
	MFP_CFG(GPIO114, AF1),
	MFP_CFG(GPIO115, AF1),
	MFP_CFG(GPIO116, AF2),
	MFP_CFG(GPIO117, AF2),

	/* OV529 */
	MFP_CFG(GPIO61, AF0),	/* 529_RST */
	MFP_CFG(GPIO63, AF0),	/* 529_SNAP */
};

#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV7740)
/* sensor init */
static int sensor_power_onoff(int on, int sensor)
{
	int ov7740_pwr_down;

	ov7740_pwr_down = mfp_to_gpio(MFP_PIN_GPIO62);
	if (gpio_request(ov7740_pwr_down , "ov7740_pwr_down")) {
		printk(KERN_INFO "gpio %d request failed\n", ov7740_pwr_down);
		return -EIO;
	}

	if (on) {
		gpio_direction_output(ov7740_pwr_down, 0);
		printk(KERN_INFO"ov7740 power on\n");
	} else {
		gpio_direction_output(ov7740_pwr_down, 1);
		printk(KERN_INFO"ov7740 power off\n");
	}
	gpio_free(ov7740_pwr_down);

	return 0;
}


static struct sensor_platform_data ov7740_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};
#endif

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};


/* i2c bus: audio codec  */
static struct i2c_board_info ipcam_i2c_board_info[] = {
};

/* power i2c bus: camera  */
static struct i2c_board_info pwri2c_board_info[] =
{
#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV7740)
	{
		.type	= "ov7740",
		.addr	= 0x21,
		.platform_data	= &ov7740_sensor_data,
		/* .irq	= */
	},
#endif
};

#if defined(CONFIG_MMC_PXA_SDH)

/* quirks define should sync with drivers/mmc/host/sdhci.h */

static struct pxasdh_platform_data ipcam_sdh3_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init		= pxa_mci_init,
	.exit		= pxa_mci_exit,
	.get_cd		= pxa_mci_get_cd,
};

static struct pxasdh_platform_data ipcam_sdh4_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init		= pxa_mci_init,
	.exit		= pxa_mci_exit,
	.get_cd		= pxa_mci_get_cd
};

static void __init ipcam_init_mmc(void)
{
	/* mmc3, external mmc/sd socket */
	pxa_mmc_slot[2].gpio_detect = 1;
	pxa_mmc_slot[2].no_wp = 1;
	pxa_mmc_slot[2].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO2);
	pxa168_add_sdh(2, &ipcam_sdh3_platform_data);

	/* mmc4, internal mmc/sd socket */
	pxa_mmc_slot[3].gpio_detect = 1;
	pxa_mmc_slot[3].no_wp = 1;
	pxa_mmc_slot[3].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO77);
	pxa168_add_sdh(3, &ipcam_sdh4_platform_data);
}
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static int ipcam_u2o_vbus_status(unsigned base)
{
	return VBUS_HIGH;
}

static struct pxa_usb_plat_info ipcam_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_status	= ipcam_u2o_vbus_status,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
static int ipcam_u2h_vbus_set(int enable)
{
	return 0;
}
static struct pxa_usb_plat_info ipcam_u2h_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= ipcam_u2h_vbus_set,
};
#endif

#define ENET_RESET_N mfp_to_gpio(MFP_PIN_GPIO102)
#define ENET_COMA_N mfp_to_gpio(MFP_PIN_GPIO60)

static int ipcam_eth_init(void)
{
	if (gpio_request(ENET_RESET_N, "ENET_RESET_N")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", ENET_RESET_N);
		return -EIO;
	}

	if (gpio_request(ENET_COMA_N, "ENET_COMA_N")) {
		gpio_free(ENET_RESET_N);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", ENET_COMA_N);
		return -EIO;
	}

	gpio_direction_output(ENET_RESET_N, 1);
	gpio_direction_output(ENET_COMA_N, 1);

	gpio_free(ENET_RESET_N);
	gpio_free(ENET_COMA_N);

	return 0;
}

static struct pxa168_eth_platform_data pxa168_eth_data = {
	.phy_addr	= 0,	/* phy addr depends on boards */
	.force_phy_addr	= 1,
	.init		= ipcam_eth_init,
};



#if defined(CONFIG_SPI_PXA2XX) || defined(CONFIG_SPI_PXA2XX_MODULE)
extern void spi_flashinit(void);
static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect	= 1,
};

static void __init ipcam_init_spi(void)
{
	pxa168_add_ssp(1);
	pxa168_add_spi(2, &pxa_ssp_master_info);
	spi_flashinit();
}
#else
static inline void ipcam_init_spi(void) {}
#endif

DECLARE_SPI_PARTITIONS(ipcam_spi_partitions);

static void __init ipcam_init(void)
{
	pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	pxa168_mfp_set_fastio_drive(MFP_DS02X);

	mfp_config(ARRAY_AND_SIZE(ipcam_pin_config));

	/* on-chip devices */
	pxa168_add_uart(3);
	pxa168_add_ssp(0);
	pxa168_add_twsi(0, &pwri2c_info, ARRAY_AND_SIZE(ipcam_i2c_board_info));
	pxa168_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(pwri2c_board_info));
#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&ipcam_u2o_info);
#endif
#ifdef CONFIG_USB_EHCI_PXA_U2H
	pxa168_add_u2h(&ipcam_u2h_info);
#endif
#if defined(CONFIG_MMC_PXA_SDH)
	ipcam_init_mmc();
#endif
	pxa168_add_mfu(&pxa168_eth_data);

	/* off-chip devices */
#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV7740)
	pxa168_add_cam();
#endif
	ipcam_init_spi();
}

MACHINE_START(IPCAM, "PXA168 IPCAM Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = ipcam_init,
MACHINE_END
