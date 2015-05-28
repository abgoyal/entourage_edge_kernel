/*
 *  linux/arch/arm/mach-mmp/edge.c
 *
 *  Support for the Marvell PXA168-based Edge 2.0 Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/pcf857x.h>
#include <linux/i2c/ofm_pdata.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/spi/spi.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/android_pmem.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <linux/gpio_keys.h>
#ifdef CONFIG_INPUT_GPIO
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#endif
#include <mach/irqs.h>
#include <plat/pxa3xx_pmic.h>
#include <plat/pxa_u2o.h>

#include <mach/pxa3xx_nand.h>
#include <plat/part_table.h>
#include <plat/generic.h>
#include <mach/mmc.h>
#include <mach/camera.h>
#include <mach/pxa168fb.h>

#include "common.h"
#include <linux/delay.h>
#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif
#include <linux/mmc/sdhci.h>

#ifdef CONFIG_ESI_EDGE
#include <mach/regs-smc.h>
#endif

/* define ONLY for G2k dongle units */
#undef G2K_DONGLE_MODE

/* For GPIO Expander (from aspenite.c) */
#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))

/* Edge 2.0 MFP configurations */
static unsigned long edge_pin_config[] __initdata = {
	/* DEBUG UART2 */
	MFP_CFG(GPIO88, AF3),
	MFP_CFG(GPIO89, AF3),

	/* MMC2_DET/WP */
	MFP_CFG(GPIO86, AF0),
#ifndef CONFIG_ESI_EDGE_JR
	MFP_CFG(GPIO87, AF0),		/* SD Card write protect only on edge2 */
#else
	MFP_CFG(GPIO87, AF0),		/* for Jr, dummy SW_WIFI_ON switch */
#endif

	/* MMC2 DAT3-0/CLK/CMD */
	MFP_CFG(GPIO90, AF1),
	MFP_CFG(GPIO91,	AF1),
	MFP_CFG(GPIO92,	AF1),
	MFP_CFG(GPIO93,	AF1),
	MFP_CFG(GPIO94,	AF1),
	MFP_CFG(GPIO95,	AF1),

	/* SD_PWR_EN */
	MFP_CFG(GPIO105, AF0),

#ifndef CONFIG_ESI_EDGE_JR
	/* JBALL, JBALL_DOWN - EXT_WAKEUP */
	MFP_CFG(GPIO27, AF5),  /* JBALL_DOWN */
	MFP_CFG(GPIO96, AF0),  /* JBALL_PRESS */
	MFP_CFG(GPIO25, AF5),  /* JBALL_LEFT */
	MFP_CFG(GPIO104, AF0), /* JBALL_RIGHT */
	MFP_CFG(GPIO112, AF0), /* JBALL_UP */
#else
	/* jog ball not used on Jr */
	MFP_CFG(GPIO27, AF5),  /* LCD_LR */
	MFP_CFG(GPIO96, AF0),  /* OFM PRESS */
	MFP_CFG(GPIO25, AF5),  /* OFM PWR DOWN */
	MFP_CFG(GPIO104, AF0), /* OFM IRQ */
	MFP_CFG(GPIO112, AF0), /* AP_WPD# */
#endif

	/* PEN DIGITIZER */
	MFP_CFG(GPIO97, AF0), /* PEN_DETECT */
	MFP_CFG(GPIO100, AF0), /* PEN_SLEEP */
	MFP_CFG(GPIO109, AF0), /* PEN_RESETn */
	MFP_CFG_DRV(GPIO98, AF2, FAST), /* PEN_RXD, UART3_TXD */
	MFP_CFG_DRV(GPIO99, AF2, FAST), /* PEN_TXD, UART3_RXD */
	MFP_CFG_DRV(GPIO101, AF2, FAST), /* PEN_CTS, UART3_RTS */

	/* I2C */
	MFP_CFG_DRV(GPIO102, AF1, SLOW),
	MFP_CFG_DRV(GPIO103, AF1, SLOW),

	/* 3G Mini_PCIe HUAWEI EM660 Module */
	MFP_CFG(GPIO106, AF0), /* PERST# */
#ifndef CONFIG_ESI_EDGE_JR
	MFP_CFG(GPIO43, AF0),  /* W_DISABLE# */
#endif

#ifndef CONFIG_ESI_EDGE_JR
	/* SW_LCDn[0:3] */
	MFP_CFG(GPIO37, AF0),
	MFP_CFG(GPIO38, AF0),
	MFP_CFG(GPIO39, AF0),
	MFP_CFG(GPIO42, AF0),

	/* SW_VOL */
	MFP_CFG(GPIO40, AF0),
	MFP_CFG(GPIO41, AF0),

	/* SW_EPDn[0:3] */
	MFP_CFG(GPIO44, AF0),
	MFP_CFG(GPIO45, AF0),
	MFP_CFG(GPIO47, AF0),
	MFP_CFG(GPIO46, AF0),
#else
	/* buttons/switches moved to GPIO expander; replaced with camera signals */
	/* camera */
	MFP_CFG(GPIO37, AF4),
	MFP_CFG(GPIO38, AF4),
	MFP_CFG(GPIO39, AF4),
	MFP_CFG(GPIO42, AF4),

	MFP_CFG(GPIO40, AF4),
	MFP_CFG(GPIO41, AF4),

	MFP_CFG(GPIO44, AF4),
	MFP_CFG(GPIO45, AF4),
	/* MFP_CFG(GPIO47, AF4), */
	MFP_CFG(GPIO46, AF4),

	MFP_CFG(GPIO48, AF4),
	MFP_CFG(GPIO49, AF0),
	MFP_CFG(GPIO51, AF0),

        MFP_CFG(GPIO54, AF4),


	MFP_CFG(GPIO43, AF0),	/* GPIO Expander IRQ */
#endif

#ifndef CONFIG_ESI_EDGE_JR
	/* HALL sensor */
	MFP_CFG(GPIO48, AF0), /* UNIT_OPEN */
#endif

#ifndef CONFIG_ESI_EDGE_JR
	/* EC power control */
	MFP_CFG(GPIO49, AF0), /* SD_REQ# */
	MFP_CFG(GPIO51, AF0), /* SD_GNT# */
	MFP_CFG(GPIO52, AF0), /* SLP_REQ# */
	MFP_CFG(GPIO53, AF0), /* SLP_GNT# */
	MFP_CFG(GPIO54, AF0), /* VCORE_INT, gpio47 on ava ??? */
#endif

	/* LCD */
	GPIO56_LCD_FCLK_RD,
	GPIO57_LCD_LCLK_A0,
	GPIO58_LCD_PCLK_WR,
	GPIO59_LCD_DENA_BIAS,
	GPIO60_LCD_DD0,
	GPIO61_LCD_DD1,
	GPIO62_LCD_DD2,
	GPIO63_LCD_DD3,
	GPIO64_LCD_DD4,
	GPIO65_LCD_DD5,
	GPIO66_LCD_DD6,
	GPIO67_LCD_DD7,
	GPIO68_LCD_DD8,
	GPIO69_LCD_DD9,
	GPIO70_LCD_DD10,
	GPIO71_LCD_DD11,
	GPIO72_LCD_DD12,
	GPIO73_LCD_DD13,
	GPIO74_LCD_DD14,
	GPIO75_LCD_DD15,
	GPIO76_LCD_DD16,
	GPIO77_LCD_DD17,

	MFP_CFG(GPIO34, AF0), /* LCD_PWR_EN */
	MFP_CFG(GPIO35, AF0), /* LCD_PWDN# (LCD_UD for edgeJr) */

	MFP_CFG(GPIO78, AF0), /* EPSON_RESET# */
	MFP_CFG(GPIO79, AF0), /* EPSON_IRQ */
	MFP_CFG(GPIO80, AF0), /* EPSON_DC */
	MFP_CFG(GPIO81, AF0), /* EPSON_RDY */

	/* USB host port power control */
	MFP_CFG(GPIO122, AF0), /* USBH_RST# */
#ifndef CONFIG_ESI_EDGE_JR
	MFP_CFG(GPIO20, AF0), /* USB_CAM_EN# */
	MFP_CFG(GPIO82, AF0), /* USB_tp_P_EN# USB customer 2 */
	MFP_CFG(GPIO83, AF0), /* USB_kb_P_EN# USB customer 1 */
#else
	MFP_CFG(GPIO20, AF0), /* RST_WIFI# */
	MFP_CFG(GPIO82, AF0), /* USB_CUST_EN# */
	MFP_CFG(GPIO83, AF0), /* W_DISABLE# */
#endif

	/* backlight control */
	MFP_CFG(GPIO84, AF4), /* LCD_PWM */
	MFP_CFG(GPIO85, AF0), /* LCD_BL_PWR_EN */

	/* FLASH */
	GPIO0_DFI_D15,
	GPIO1_DFI_D14,
	GPIO2_DFI_D13,
	GPIO3_DFI_D12,
	GPIO4_DFI_D11,
	GPIO5_DFI_D10,
	GPIO6_DFI_D9,
	GPIO7_DFI_D8,
	GPIO8_DFI_D7,
	GPIO9_DFI_D6,
	GPIO10_DFI_D5,
	GPIO11_DFI_D4,
	GPIO12_DFI_D3,
	GPIO13_DFI_D2,
	GPIO14_DFI_D1,
	GPIO15_DFI_D0,
	GPIO16_ND_nCS0, /* MLC CS0# */
	GPIO26_ND_RnB1, /* MLC_NF_R_B0# */
	GPIO21_ND_ALE,  /* ??? and MLC NF_WE# */
	/* shared with EPSON panel */
	GPIO24_ND_nRE, /* AP_NF_RE#, only */
	GPIO17_ND_nWE, /* AP_NF_WE#, only */
	GPIO22_ND_CLE, /* AP_NF_CLE#, EPSON_RDn */

	/* EPSON panel */
	GPIO18_SMC_nCS0, /* EPSON_CS# */

	/* tsc2007 irq */
	MFP_CFG(GPIO19, AF5),

	/* RTC */
	MFP_CFG(GPIO23, AF5), /* RTC_INT# */

	/* SD_WIFI */
	MFP_CFG(GPIO28, AF1),
	MFP_CFG(GPIO29, AF1),
	MFP_CFG(GPIO30, AF1),
	MFP_CFG(GPIO31, AF1),
	MFP_CFG(GPIO32, AF1),
	MFP_CFG(GPIO33, AF1),

	/* WIFI */
#ifndef CONFIG_ESI_EDGE_JR
	MFP_CFG(GPIO50, AF0),	/* SW_WIFI_ON */
	MFP_CFG(GPIO55, AF0),	/* GP_PD_WLAN# */
#endif
#ifndef CONFIG_ESI_EDGE_JR
	MFP_CFG(GPIO118, AF0),	/* AP_WPD_1# */
	MFP_CFG(GPIO119, AF0),	/* AP_WPD_2# */
	MFP_CFG(GPIO120, AF0),	/* RST_WIFI# */
	MFP_CFG(GPIO121, AF0),	/* WIFI_LED# */
#else
	MFP_CFG(GPIO118, AF0),	/* SD_REQ# */
	MFP_CFG(GPIO119, AF0),	/* SD_GNT# */
	MFP_CFG(GPIO120, AF0),	/* SLP_REQ# */
	MFP_CFG(GPIO121, AF0),	/* SLP_GNT# */
#endif

	/* G-sensor BMA020 */
	MFP_CFG(GPIO36, AF0), /* G_INT */

	/* SSP2, SPI NOR */
	GPIO107_SPI_NOR_RXD,
	GPIO108_SPI_NOR_TXD,
	GPIO110_GPIO,
	GPIO111_SPI_NOR_CLK,

	/* SSP0, AUDIO */
	MFP_CFG(GPIO113, AF6),
	MFP_CFG(GPIO114, AF1),
	MFP_CFG(GPIO115, AF1),
	MFP_CFG(GPIO116, AF2),
	MFP_CFG(GPIO117, AF2),
};

#ifdef CONFIG_INPUT_GPIO	
struct edge_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
};
static bool nav_just_on;
static int nav_on_jiffies;

uint16_t edge_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct edge_axis_info *ai = container_of(info, struct edge_axis_info, info);
	uint16_t out = ai->out_state;
	
	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if((ai->in_state ^ in) & 1)
		out--;
	if((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	return out;
}

int edge_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
#if 0
	gpio_set_value(EDGE_GPIO_JOG_EN, on);
#endif
	if (on) {
		nav_just_on = 1;
		nav_on_jiffies = jiffies;
	}
	return 0;
}

#define EDGE_4_BALL_LEFT_0 	MFP_PIN_GPIO25
#define EDGE_4_BALL_RIGHT_0 	MFP_PIN_GPIO27
#define EDGE_4_BALL_UP_0 		MFP_PIN_GPIO104
#define EDGE_4_BALL_DOWN_0	 	MFP_PIN_GPIO112

#ifdef ESI_DEBUG_DISABLED
#define EDGE_5_BALL_LEFT_0 		MFP_PIN_GPIO25
#define EDGE_5_BALL_RIGHT_0 	MFP_PIN_GPIO27
#define EDGE_5_BALL_UP_0 		MFP_PIN_GPIO104
#define EDGE_5_BALL_DOWN_0	 	MFP_PIN_GPIO112
#else
/* Flip these around */
#define EDGE_5_BALL_LEFT_0 		MFP_PIN_GPIO27
#define EDGE_5_BALL_RIGHT_0 	MFP_PIN_GPIO25
#define EDGE_5_BALL_UP_0 		MFP_PIN_GPIO104
#define EDGE_5_BALL_DOWN_0	 	MFP_PIN_GPIO112
#endif

#define EDGE_GPIO_NAVI_ACT_N 	MFP_PIN_GPIO96

static uint32_t edge_5_x_axis_gpios[] = {
	EDGE_5_BALL_LEFT_0, EDGE_5_BALL_RIGHT_0
};

static uint32_t edge_5_y_axis_gpios[] = {
	EDGE_5_BALL_UP_0, EDGE_5_BALL_DOWN_0
};


static struct edge_axis_info edge_x_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(edge_5_x_axis_gpios),
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(edge_5_y_axis_gpios),
		.map = edge_axis_map,
		.gpio = edge_5_y_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION /* | GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */ 
	}
};

static struct edge_axis_info edge_y_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(edge_5_y_axis_gpios),
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(edge_5_x_axis_gpios),
		.map = edge_axis_map,
		.gpio = edge_5_x_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION /* | GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */
	}
};

static struct gpio_event_direct_entry edge_nav_buttons[] = {
	{ EDGE_GPIO_NAVI_ACT_N, BTN_MOUSE }
};

static struct gpio_event_input_info edge_nav_button_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = edge_nav_buttons,
	.keymap_size = ARRAY_SIZE(edge_nav_buttons)
};

static struct gpio_event_info *edge_nav_info[] = {
	&edge_x_axis.info.info,
	&edge_y_axis.info.info,
	&edge_nav_button_info.info
};

static struct gpio_event_platform_data edge_nav_data = {
	.name = "edge-nav",
	.info = edge_nav_info,
	.info_count = ARRAY_SIZE(edge_nav_info),
	.power = edge_nav_power,
};

static struct platform_device edge_nav_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 2,
	.dev		= {
		.platform_data	= &edge_nav_data,
	},
};

static int edge_reset_keys_up[] = {
	BTN_MOUSE,
	0
};

static struct keyreset_platform_data edge_reset_keys_pdata = {
	.keys_up = edge_reset_keys_up,
	.keys_down = {
		KEY_SEND,
		KEY_MENU,
		KEY_END,
		0
	},
};

struct platform_device edge_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &edge_reset_keys_pdata,
};
#endif // CONFIG_INPUT_GPIO

/* Button and Switch definitions */
#ifndef CONFIG_ESI_EDGE_JR
/* for daddy edge */
#define BACK_BTN		MFP_PIN_GPIO37	/* SW8, SW_LCDn[0] */
#define ROTATE_BTN		MFP_PIN_GPIO38	/* SW9, SW_LCDn[1] */
#define HOME_BTN		MFP_PIN_GPIO39	/* SW10, SW_LCDn[2] */
#define MENU_BTN		MFP_PIN_GPIO42	/* SW12, SW_LCDn[3] */
#define VOL_UP_BTN		MFP_PIN_GPIO41	/* SW1, SW_VOL_UP# */
#define VOL_DN_BTN		MFP_PIN_GPIO40	/* SW2, SW_VOL_DN# */
#define ZOOM_BTN		MFP_PIN_GPIO46	/* SW7 */
#define NEXT_BTN		MFP_PIN_GPIO47	/* SW6 */
#define PREV_BTN		MFP_PIN_GPIO45	/* SW5 */
#define WHEEL_BTN		MFP_PIN_GPIO44	/* SW3 */
#define LID_CLOSE_SW	MFP_PIN_GPIO48	/* U5 */
#define RF_KILL_SW		MFP_PIN_GPIO50	/* SW1 */
#define TABLET_MODE_SW	MFP_PIN_GPIO97	/* formerly PEN_DETECT */
#else
/* for edge Jr. */
#define BACK_BTN		GPIO_EXT0(3)
#define ROTATE_BTN		GPIO_EXT0(2)
#define HOME_BTN		GPIO_EXT0(1)
#define MENU_BTN		GPIO_EXT0(0)
#define VOL_UP_BTN		GPIO_EXT0(8)
#define VOL_DN_BTN		GPIO_EXT0(9)
#define ZOOM_BTN		GPIO_EXT0(4)
#define NEXT_BTN		GPIO_EXT0(5)
#define PREV_BTN		GPIO_EXT0(6)
#define WHEEL_BTN		GPIO_EXT0(7)
#define LID_CLOSE_SW	GPIO_EXT0(10)
#define RF_KILL_SW		MFP_PIN_GPIO87	/* DUMMY!!! - always 1 */
#define TABLET_MODE_SW	MFP_PIN_GPIO97	/* formerly PEN_DETECT */
#endif

/*
 * GPIO Keys
 */
static struct gpio_keys_button btn_button_table[] = {
	[0]={
		.code			=	KEY_F9,
		.gpio			=	BACK_BTN,
		.active_low		=	1,		//0 for down 0, up 1; 1 for down 1, up 0
		.desc			=	"BACK button",
		.type			=	EV_KEY,
		//.wakeup			=
		.debounce_interval	=	10,		//10 msec jitter elimination
		.side               = 0,
	},
	[1]={
		.code			=	KEY_F6,
		.gpio			=	ROTATE_BTN,
		.active_low		=	1,		//0 for down 0, up 1; 1 for down 1, up 0
		.desc			=	"ROTATE button",
		.type			=	EV_KEY,
		//.wakeup			=
		.debounce_interval	=	10,		//10 msec jitter elimination
		.side               = 0,
	},
	[2]={
		.code			=	KEY_F7,
		.gpio			=	HOME_BTN,
		.active_low		=	1,		//0 for down 0, up 1; 1 for down 1, up 0
		.desc			=	"HOME button",
		.type			=	EV_KEY,
		//.wakeup			=
		.debounce_interval	=	10,		//10 msec jitter elimination
		.side               = 0,
	},
	[3]={
		.code			=	KEY_F8,
		.gpio			=	MENU_BTN,  /* SW12, SW_LCDn[3] */
		.active_low		=	1,		//0 for down 0, up 1; 1 for down 1, up 0
		.desc			=	"MENU button",
		.type			=	EV_KEY,
		//.wakeup			=
		.debounce_interval	=	10,		//10 msec jitter elimination
		.side               = 0,
	},
	[4]={
		.code			=	KEY_VOLUMEUP,
		.gpio			=	VOL_UP_BTN,
		.active_low		=	1,		//0 for down 0, up 1; 1 for down 1, up 0
		.desc			=	"VOL_UP button",
		.type			=	EV_KEY,
		//.wakeup			=
		.debounce_interval	=	10,		//10 msec jitter elimination
		.side               = 0,
	},
	[5]={
		.code			=	KEY_VOLUMEDOWN,
		.gpio			=	VOL_DN_BTN,
		.active_low		=	1,		//0 for down 0, up 1; 1 for down 1, up 0
		.desc			=	"VOL_DN button",
		.type			=	EV_KEY,
		//.wakeup			=
		.debounce_interval	=	10,		//10 msec jitter elimination
		.side               = 0,
	},
	/* EPD-side buttons/gpios */
	[6] = {
		.code		= KEY_F13,
		.gpio		= WHEEL_BTN,
		.desc		= "WHEEL button",
		.debounce_interval	= 30,
		.active_low			= 1,
		.side 				= 1,		
	},
	[7] = {
		.code		= KEY_PAGEDOWN,
		.gpio		= NEXT_BTN,
		.desc		= "NEXT_PG button",
		.debounce_interval	= 30,
		.active_low			= 1,
		.side 				= 1,		
	},
	[8] = {
		.code		= KEY_PAGEUP,
		.gpio		= PREV_BTN,
		.desc		= "PREV_PG button",
		.debounce_interval	= 30,
		.active_low			= 1,
		.side 				= 1,
	},
	[9] = {
		.code		= KEY_F14,
		.gpio		= ZOOM_BTN,
		.desc		= "ZOOM button",
		.debounce_interval	= 30,
		.active_low			= 1,
		.side 				= 1,		
	},
	/* misc switches */
	[10] = {
		.code		= SW_LID,
		.gpio		= LID_CLOSE_SW,
		.desc		= "LID_CLOSED switch",
		.type			=	EV_SW,
		.debounce_interval	= 30,
		.active_low			= 1,
		.side 				= 0,		
	},
	[11] = {
		.code		= SW_RFKILL_ALL,
		.gpio		= RF_KILL_SW,
		.desc		= "RF_KILL switch",
		.type			=	EV_SW,
		.debounce_interval	= 30,
#ifndef CONFIG_ESI_EDGE_JR
		.active_low			= 1,
#else
		.active_low			= 0,
#endif
		.side 				= 0,		
	},
#ifdef CONFIG_ESI_EDGE
	[12] = {
		.code		= SW_DOCK,
		.gpio		= TABLET_MODE_SW,
		.desc		= "Tablet Mode switch",
		.type			=	EV_SW,
		.debounce_interval	= 30,
		.active_low			= 0,
		.side 				= 0,		
	},
#endif
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons  = btn_button_table,
	.nbuttons = ARRAY_SIZE(btn_button_table),
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
};

static void __init edge_gpio_keys_init(void)
{
	platform_device_register(&gpio_keys);
}

#ifndef CONFIG_ESI_EDGE_JR
static struct fb_videomode video_modes[] = {
	/* innolux WVGA mode info */
	[0] = {
		.pixclock	= 25700,	/* 39 MHz */
		.refresh	= 60,
		.xres		= 1024,
		.yres		= 600,
		.hsync_len	= 176,
		.left_margin	= 0,
		.right_margin	= 0,
		.vsync_len	= 25,
		.upper_margin	= 0,
		.lower_margin	= 0,
		.sync		= FB_SYNC_VERT_HIGH_ACT|FB_SYNC_HOR_HIGH_ACT,
	},
};
#else
/* for edge Junior */
static struct fb_videomode video_modes[] = {
	/* innolux WVGA mode info - from avengers_lite.c */
	[0] = {
		.pixclock	= 29000, /* div = 9 */
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.hsync_len	= 1,
		.left_margin	= 45,
		.right_margin	= 210,
		.vsync_len	= 1,
		.upper_margin	= 22,
		.lower_margin	= 132,
		.sync		= 0,
	},
};
#endif

static struct pxa168fb_mach_info edge_lcd_info __initdata = {
	.id			= "Base",
	.modes			= video_modes,
	.num_modes		= ARRAY_SIZE(video_modes),
	.pix_fmt		= PIX_FMT_RGB565,
	.io_pin_allocation_mode	= PIN_MODE_DUMB_18_GPIO,
	.dumb_mode		= DUMB_MODE_RGB666,
	.active			= 1,
	.panel_rbswap		= 0,
	.invert_pixclock	= 1,
	.max_fb_size		= (DEFAULT_FB_SIZE + 5 * 1024 * 1024),
};

struct pxa168fb_mach_info edge_lcd_ovly_info __initdata = {
        .id                     = "Ovly",
        .modes                  = video_modes,
        .num_modes              = ARRAY_SIZE(video_modes),
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_DUMB_18_GPIO,
        .dumb_mode              = DUMB_MODE_RGB666,
        .active                 = 1,
	.panel_rbswap		= 0,
	.max_fb_size		= (DEFAULT_FB_SIZE),
};

#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV3640) && defined(CONFIG_ESI_EDGE_JR)
/* sensor init */
static int sensor_power_onoff(int on, int sensor)
{

	int ov3640_pwr_down;
	int ov3640_reset;

	ov3640_pwr_down = MFP_PIN_GPIO51;
	if (gpio_request(ov3640_pwr_down , "ov3640_pwr_down")) {
		printk(KERN_INFO "gpio %d request failed\n", ov3640_pwr_down);
		return -EIO;
	}

	ov3640_reset = MFP_PIN_GPIO49;
	if (gpio_request(ov3640_reset , "ov3640_reset")) {
		printk(KERN_INFO "gpio %d request failed\n", ov3640_reset);
		return -EIO;
	}

	/* pwr_down vs reset sequence/timing may need to be revisited */
	if (on) {
		gpio_direction_output(ov3640_pwr_down, 0);
		/* Rest during initial power up */
		if (sensor == 0xFF) { 
		    msleep(1);
		    gpio_direction_output(ov3640_reset, 0);
		    printk(KERN_INFO"ov3640 power on (reset active)\n");
		    msleep(1);
		    gpio_direction_output(ov3640_reset, 1);
		    printk(KERN_INFO"ov3640 power on (reset inactive)\n");
		}
	} else {
		gpio_direction_output(ov3640_pwr_down, 1);
	}
	gpio_free(ov3640_pwr_down);
	gpio_free(ov3640_reset);

	return 0;
}


static struct sensor_platform_data ov3640_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

#endif

#define trace(FMT, ARGS...) if(1) printk("%s at line %d: "FMT"\n", __FILE__, __LINE__, ## ARGS)

#ifndef CONFIG_ESI_EDGE_JR
static int edge_lcd_power_enable(int state)
{
	int lcd_pwdn = MFP_PIN_GPIO35;
    int lcd_pwr_en = MFP_PIN_GPIO34;
	int lcd_bl_pwr_en = MFP_PIN_GPIO85;
	int on = !!state;
	static int oldState = -1;
#if 1
	if(on == oldState)
		return 0;

	if(on)
		{
		gpio_direction_output(lcd_pwr_en, on);
		gpio_direction_output(lcd_pwdn, on);

		mdelay(200);

		gpio_direction_output(lcd_bl_pwr_en, on);
		}
	else
		{
		gpio_direction_output(lcd_bl_pwr_en, on);

		mdelay(200);

		gpio_direction_output(lcd_pwdn, on);
		gpio_direction_output(lcd_pwr_en, on);
		}

	oldState = on;
	return 0;
#else
	int lcd_pwdn, lcd_pwr_en;

	/* LCD_PWR_EN */
	lcd_pwr_en = MFP_PIN_GPIO34;
	if(gpio_request(lcd_pwr_en, "lcd_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_pwr_en);
		return -EIO;
	}
	gpio_direction_output(lcd_pwr_en, !!state);
	gpio_free(lcd_pwr_en);

	/* LCD_PWDN# */
	lcd_pwdn = MFP_PIN_GPIO35;
	if(gpio_request(lcd_pwdn, "lcd_pwdn")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_pwdn);
		return -EIO;
	}
	gpio_direction_output(lcd_pwdn, !!state);
	gpio_free(lcd_pwdn);
	return 0;
#endif
}
#else
/* edge Jr version - taken from avengers_lite.c */
static int edge_lcd_power_enable(int state)
{
	int lcd_pwr_en;
	int lcd_bl_pwr_en;
	int lcd_lr, lcd_ud;

	lcd_pwr_en = MFP_PIN_GPIO34;
	if (gpio_request(lcd_pwr_en, "lcd_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n",
						lcd_pwr_en);
		goto out;
	}

	lcd_bl_pwr_en = MFP_PIN_GPIO85;
	if (gpio_request(lcd_bl_pwr_en, "lcd_bl_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n",
						lcd_bl_pwr_en);
		goto out1;
	}

	lcd_lr = MFP_PIN_GPIO27;
	if (gpio_request(lcd_lr, "lcd_lr")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_lr);
		goto out2;
	}

	lcd_ud = MFP_PIN_GPIO35;
	if (gpio_request(lcd_ud, "lcd_ud")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_ud);
		goto out3;
	}

	if (state) {
		/* re-config lcd_lr/ud pin to enable lcd */
		gpio_direction_output(lcd_pwr_en, 1);
		gpio_direction_output(lcd_bl_pwr_en, 1);
		gpio_direction_output(lcd_lr, 1);
		gpio_direction_output(lcd_ud, 0);
	} else {
		/* config lcd_lr/ud pin to gpio for power optimization */
		gpio_direction_output(lcd_pwr_en, 0);
		gpio_direction_output(lcd_bl_pwr_en, 0);
		gpio_direction_input(lcd_lr);
		gpio_direction_input(lcd_ud);
	}

	gpio_free(lcd_ud);
out3:
	gpio_free(lcd_lr);
out2:
	gpio_free(lcd_bl_pwr_en);
out1:
	gpio_free(lcd_pwr_en);
out:
	return 0;
}
#endif

static int __init edge_lcd_init(void)
{
#ifdef CONFIG_ESI_EDGE_JR
	int lcd_lr, lcd_ud;
#endif

	int ret = 0;
	if((ret = edge_lcd_power_enable(1)) != 0)
		return ret;

#ifdef CONFIG_ESI_EDGE_JR
	/* taken from avenger_lite.c */
	lcd_lr = MFP_PIN_GPIO27;
	lcd_ud = MFP_PIN_GPIO35;

	if (gpio_request(lcd_lr, "lcd_lr")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_lr);
		return 0;
	}
	gpio_direction_output(lcd_lr, 1);
	gpio_free(lcd_lr);

	if (gpio_request(lcd_ud, "lcd_ud")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_ud);
		return 0;
	}
	gpio_direction_output(lcd_ud, 0);
	gpio_free(lcd_ud);
#endif

	/* register device */
	pxa168_add_fb(&edge_lcd_info);
	pxa168_add_fb_ovly(&edge_lcd_ovly_info);

	return 0;
}

static int edge_lcd_brightness_notification(int brightness)
{
//	printk(KERN_CRIT "%s: brightness = %d\n", __FUNCTION__, brightness);

	edge_lcd_power_enable(!!brightness);

	return brightness;
}

static struct platform_pwm_backlight_data edge_backlight_data = {
	.pwm_id         = 0,
	.max_brightness = 255,
	.dft_brightness = 200,
	.pwm_period_ns  = 78770,
	.notify = edge_lcd_brightness_notification,
};

static struct platform_device edge_backlight_device = {
	.name           = "pwm-backlight",
	.dev            = {
		.parent = &pxa168_device_pwm0.dev,
		.platform_data = &edge_backlight_data,
	},
};

static int __init edge_backlight_register(void)
{
	int lcd_bl_pwr_en;
	int ret = platform_device_register(&edge_backlight_device);
	if (ret)
		printk(KERN_ERR "edge: failed to register backlight device: %d\n", ret);

	/* LCD_BL_PWR_EN */
	lcd_bl_pwr_en = MFP_PIN_GPIO85;
	if(gpio_request(lcd_bl_pwr_en, "lcd_bl_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_bl_pwr_en);
		return -EIO;
	}
	gpio_direction_output(lcd_bl_pwr_en, 1);
	gpio_free(lcd_bl_pwr_en);

	return 0;
}

#if defined(CONFIG_OFM_MOUSE)
static struct ofm_pin ofm_gpio_pins[] = {
		[OFM_POWER_DN] = {
				.pin = mfp_to_gpio(MFP_PIN_GPIO25),
				.name = "ofm_power_dn",
		},
		[OFM_MOTION_DETECT] = {
				.pin = mfp_to_gpio(MFP_PIN_GPIO104),
				.irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO104)),
				.name = "ofm_motion_detect",
		},
		[OFM_LEFT_BUTTON] = {
				.pin = mfp_to_gpio(MFP_PIN_GPIO96),
				.irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO96)),
				.name = "ofm_left_button",
		},
	};
#endif

#if defined(CONFIG_GPIO_PCF857X)
static struct pcf857x_platform_data pcf857x_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT0(0),
		.n_latch		= 0,
	},
};
#endif

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};

/* touch screen, rtc, audio codec  */
static struct i2c_board_info edge_i2c_board_info[] = {
#if defined(CONFIG_TSC2007)
       {
	       .type	= "tsc2007",
	       .addr	= 0x48,			/* 0x90/0x91 */
	       .irq	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO19)),
       },
#endif
#if defined(CONFIG_RTC_DRV_DS1307)
	{
		.type		= "ds1337",
		.addr           = 0x68, /* 0xD0 */
	},
#endif
#if defined(CONFIG_OFM_MOUSE)
	{
		.type	="ofm",
#if 0
		.addr	=0x10,	/* 0x20 */
#else
		.addr	=0x53,	/* 0xa6 (Partron M40C00FXC) */ 
#endif
//		.irq	=IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO104)), /* G_INT */
		.platform_data = &ofm_gpio_pins[0],
	},
#endif
#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV3640) && defined(CONFIG_ESI_EDGE_JR)
	{
		.type	= "ov3640",
		.addr	= 0x3c,
		.platform_data	= &ov3640_sensor_data,
		/* .irq	= */
	},
#endif
};

static struct i2c_board_info pwri2c_board_info[] =
{
#if defined(CONFIG_BMA020)
	{
		.type	="bma020",
		.addr	=0x38,
		.irq	=IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO36)), /* G_INT */
	},
#endif
#if defined(CONFIG_MCU_PM)
	{
		.type	="power_mcu",
		.addr	=0x2C, /* why not 0x3c as it's 0x78 in schematic ??? */
	},
#endif
#if defined(CONFIG_GPIO_PCF857X)
	{
		.type           = "pcf8575",
		.addr           = 0x20,                 /* 0x40 */
		.irq            = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO43)),
		.platform_data  = &pcf857x_data[0],
	},
#endif
};

static struct pxa27x_keypad_platform_data edge_keypad_info __initdata = {
	.direct_key_map = { 1, 2, 3, 4 },
	.direct_key_num = 4,
	.debounce_interval	= 30,
};

static struct pxa27x_keypad_platform_data edge_android_keypad_info \
						  __initdata = {

#ifdef CONFIG_ESI_EDGE_JR
	.direct_key_map = { 1 },
	.direct_key_num = 1,
	.debounce_interval	= 30,
#else
	.direct_key_map = { 1, 2, 3, 4 },
	.direct_key_num = 4,
	.debounce_interval	= 30,
#endif
};
#if defined(CONFIG_MMC_PXA_SDH)
static int edge_sdh_init(struct device *dev,
		     irq_handler_t detect_int, void *data)
{
	int sd_pwr_en;
	int ap_wpd_1n, ap_wpd_2n, rst_wifi;
	int err, cd_irq, gpio_cd;

	/* SD_PWR_EN */
	sd_pwr_en = MFP_PIN_GPIO105;
	if(gpio_request(sd_pwr_en, "sd_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n", sd_pwr_en);
		return -EIO;
	}
	gpio_direction_output(sd_pwr_en, 1);/* should fix to 0 for 1.5Q */
	gpio_free(sd_pwr_en);

#ifndef CONFIG_ESI_EDGE_JR
	/* V3P3_WLAN - AP_WPD_1#; V1P8_WLAN - AP_WPD_2# */
	ap_wpd_1n = mfp_to_gpio(MFP_PIN_GPIO118);
	ap_wpd_2n = mfp_to_gpio(MFP_PIN_GPIO119);

	if(gpio_request(ap_wpd_1n, "ap_wpd_1n")) {
		printk(KERN_INFO "gpio %d request failed\n", ap_wpd_1n);
		return -EIO;
	}
	if(gpio_request(ap_wpd_2n, "ap_wpd_2n")) {
		printk(KERN_INFO "gpio %d request failed\n", ap_wpd_2n);
		return -EIO;
	}

	gpio_direction_output(ap_wpd_1n, 1);
	gpio_direction_output(ap_wpd_2n, 1);
	gpio_free(ap_wpd_1n);
	gpio_free(ap_wpd_2n);
#endif

	/* RST_WIFI# to high */
#ifndef CONFIG_ESI_EDGE_JR
	rst_wifi = mfp_to_gpio(MFP_PIN_GPIO120);
#else
	rst_wifi = mfp_to_gpio(MFP_PIN_GPIO20);
#endif
	if(gpio_request(rst_wifi, "rst_wifi")) {
		printk(KERN_INFO "gpio %d request failed\n", rst_wifi);
		return -EIO;
	}
	gpio_direction_output(rst_wifi, 1);
	gpio_free(rst_wifi);

	/* MMC2_DET */
	gpio_cd = mfp_to_gpio(MFP_PIN_GPIO86);
	cd_irq = gpio_to_irq(gpio_cd);

	/*
	 * setup GPIO for MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	gpio_free(gpio_cd);
err_request_cd:
	return err;
}

static struct pxasdh_platform_data edge_sdh_platform_data = {
	.detect_delay	= 20,
	.max_speed		= 24000000, 
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init		= edge_sdh_init,
	.quirks		= SDHCI_QUIRK_NO_BUSY_IRQ,
};

static struct pxasdh_platform_data edge_sdh3_platform_data = {
	/* stagger detection relative to mmc slot (above) */
	.detect_delay	= 60,
#if 1
	/*
	 * Force clock to 39MHz to help with noise issues (talk to Adrian).
	 * This change is dependent on others in clock.c and pxa168.c
	 */
	.max_speed		= 39000000, 
#endif	
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
#ifdef CONFIG_SD8XXX_RFKILL
	.init           = sd8x_sdh_init,
#endif
	.quirks		= SDHCI_QUIRK_BROKEN_ADMA,
};

static void __init edge_init_mmc(void)
{
#ifdef CONFIG_SD8XXX_RFKILL
	void *pdata = NULL;

#ifndef CONFIG_ESI_EDGE_JR
	pdata = add_sd8x_rfkill_device(mfp_to_gpio(MFP_PIN_GPIO55), mfp_to_gpio(MFP_PIN_GPIO120));
#else
	pdata = add_sd8x_rfkill_device(mfp_to_gpio(MFP_PIN_GPIO112), mfp_to_gpio(MFP_PIN_GPIO20));
#endif
	if (pdata)
		edge_sdh3_platform_data.rfkill_pdata = pdata,
#endif

	/* mmc2, mmc/sd socket */
	pxa168_add_sdh(1, &edge_sdh_platform_data);

	/* mmc4, wifi */
	pxa168_add_sdh(3, &edge_sdh3_platform_data);
}
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static struct pxa_usb_plat_info edge_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.phy_deinit	= pxa168_usb_phy_deinit,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
static int edge_u2h_plat_init (struct device *dev)
{
	int usbh_rst = mfp_to_gpio(MFP_PIN_GPIO122);

	/* USBH_RST# */
	if(gpio_request(usbh_rst, "usbh_rst")) {
		printk(KERN_INFO "gpio %d request failed\n", usbh_rst);
		return -EIO;
	}
#ifndef G2K_DONGLE_MODE
	gpio_direction_output(usbh_rst, 1);
#else
	/*
	 * Force low so USB hub stays in reset.  DONGLE MODE!
	 */
	gpio_direction_output(usbh_rst, 1);
	gpio_set_value(usbh_rst, 0);
#endif
	gpio_free(usbh_rst);
	return 0;
}

static int edge_u2h_vbus_set (int enable)
{
#ifndef CONFIG_ESI_EDGE_JR
	int usb_cam_en = mfp_to_gpio(MFP_PIN_GPIO20);
	int usb_tp_p_en = mfp_to_gpio(MFP_PIN_GPIO82);
	int usb_kb_p_en = mfp_to_gpio(MFP_PIN_GPIO83);
#else
	int usb_cust_en = mfp_to_gpio(MFP_PIN_GPIO82);
#endif

#ifndef CONFIG_ESI_EDGE_JR
	/* USB_CAM_EN# */
	if(gpio_request(usb_cam_en, "usb_cam_en")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_cam_en);
		return -EIO;
	}
	gpio_direction_output(usb_cam_en, 1);	// default to OFF
	gpio_free(usb_cam_en);

	/* USB_tp_P_EN# */
	if(gpio_request(usb_tp_p_en, "usb_tp_p_en")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_tp_p_en);
		return -EIO;
	}
	gpio_direction_output(usb_tp_p_en, 0);
	gpio_free(usb_tp_p_en);

	/* USB_kb_P_EN# */
	if(gpio_request(usb_kb_p_en, "usb_kb_p_en")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_kb_p_en);
		return -EIO;
	}
	gpio_direction_output(usb_kb_p_en, 0);
	gpio_free(usb_kb_p_en);
#else
	/* USB_CUST_EN# */
	if(gpio_request(usb_cust_en, "usb_cust_en")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_cust_en);
		return -EIO;
	}
	gpio_direction_output(usb_cust_en, 0);
	gpio_free(usb_cust_en);
#endif

	return 0;
}

static struct pxa_usb_plat_info edge_u2h_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= edge_u2h_vbus_set,
	.plat_init	= edge_u2h_plat_init,
};
#endif

#if defined(CONFIG_MTD_M25P80)
#if 0
static struct pxa2xx_spi_chip m25pxx_spi_info = {
	.tx_threshold = 1,
	.rx_threshold = 1,
	.timeout = 1000,
	.gpio_cs = 110
};

static struct spi_board_info __initdata spi_board_info[] = {
	{
		.modalias = "m25p80",
		.mode = SPI_MODE_0,
		.max_speed_hz = 260000,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = NULL,
		.controller_data = &m25pxx_spi_info,
		.irq = -1,
	},
};
#else
extern void spi_flashinit(void);
#endif

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect	= 1,
};

static void __init edge_init_spi(void)
{
	pxa168_add_ssp(1);
	pxa168_add_spi(2, &pxa_ssp_master_info);
#if 0
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#else
	spi_flashinit();
#endif
}
#else
static inline void edge_init_spi(void) {}
#endif

struct platform_device edge_power_mcu = {
	.name		= "power_mcu",
};

struct platform_device edge_power_supply = {
	.name		= "battery",
};

struct platform_device edge_power_button = {
	.name		= "power-button",
	.id = -1,
};



static struct platform_device *devices[] __initdata = {
	&pxa168_device_pwm0,
	&edge_power_mcu,
	&edge_power_supply,
	&edge_power_button,
#ifdef CONFIG_INPUT_GPIO
	&edge_nav_device,
	&edge_reset_keys_device,
#endif
};

DECLARE_EDGE_MLC_4G_PARTITIONS(edge_mlc_4g_partitions);

static struct pxa3xx_nand_platform_data edge_nand_info;
static void __init edge_add_nand(void)
{
    unsigned long long mlc_size = 0;

    mlc_size = get_mlc_size();
    printk (KERN_INFO "@%s Get mlc size from CMDLINE:%d\n",
				   	__FUNCTION__, (int)mlc_size);

	switch (mlc_size)
	{
		case 4:  /* 4G */
		default:
			edge_nand_info.parts[0] = edge_mlc_4g_partitions;
			edge_nand_info.nr_parts[0] = ARRAY_SIZE(edge_mlc_4g_partitions);
			break;
	}

	edge_nand_info.use_dma = 0;
	edge_nand_info.RD_CNT_DEL = 0;
	edge_nand_info.enable_arbiter = 1;
	pxa168_add_nand((struct flash_platform_data *) &edge_nand_info);
}

static void gpio_ec_init(void)
{
	/* ??? no uart lines connected, how to control battery */
}

static void edge_power_off(void)
{
	pr_notice("notify EC to shutdown\n");
#ifndef CONFIG_ESI_EDGE_JR
	gpio_direction_output(MFP_PIN_GPIO51, 1);
	gpio_direction_output(MFP_PIN_GPIO53, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO51, 0);
	gpio_direction_output(MFP_PIN_GPIO53, 0);
#else
	gpio_direction_output(MFP_PIN_GPIO119, 1);
	gpio_direction_output(MFP_PIN_GPIO121, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO119, 0);
	gpio_direction_output(MFP_PIN_GPIO121, 0);
#endif

	/* Spin to death... */
	while (1);
}

void edge_suspend_to_ram(void)
{
//	pr_notice("notify EC to hibernate\n");
#ifndef CONFIG_ESI_EDGE_JR
	gpio_direction_output(MFP_PIN_GPIO53, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO53, 0);
#else
	gpio_direction_output(MFP_PIN_GPIO121, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO121, 0);
#endif

	// EC is gonna kill us shortly
}
EXPORT_SYMBOL(edge_suspend_to_ram);

#ifdef CONFIG_ESI_EDGE
/*
 * epd GPIO/misc init:
 * may have already been done in boot code, but don't count on it
 */
static void edge_init_epd(void)
{
	int gpio_num;

	/*
	 * MFP18 and 78-81 should already be configured (see above):
	 * MFP18 = SMC_CS0 = EPD Chip Select
	 * MFP78 = GPIO78 =  EPD reset# (output)
	 * MFP79 = GPIO79 =  EPD irq (input)
	 * MFP80 = GPIO80 =  EPD data/cmd (output)
	 * MFP81 = GPIO81 =  EPD ready (input)
	 */

	/* setup for EPD controller access (never got CS1 working) */
	/* SMC_CS0 */
	__raw_writel(0x42815243, SMC_MSC0);
	__raw_writel(0x52010008, SMC_CSDFICFG0);
	__raw_writel(0x10000000, SMC_CSADRMAP0);

	/*
	 * setup EPD GPIO directions:
	 * only set outputs, setting as inputs (arg=0) appears to
	 * actually set as an output! bug?
	 */
	/* epd reset */
	gpio_num = mfp_to_gpio(MFP_PIN_GPIO78);
	if(gpio_request(gpio_num, "epd_reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_num);
		return;
	}
	gpio_direction_output(gpio_num, 1);
	gpio_free(gpio_num);
	printk(KERN_INFO "gpio %d direction set to OUTPUT\n", gpio_num);

	/* epd data/cmd */
	gpio_num = mfp_to_gpio(MFP_PIN_GPIO80);
	if(gpio_request(gpio_num, "epd_data_cmd")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_num);
		return;
	}
	gpio_direction_output(gpio_num, 1);
	gpio_free(gpio_num);
	printk(KERN_INFO "gpio %d direction set to OUTPUT\n", gpio_num);

}

#ifndef CONFIG_ESI_EDGE_JR
/*
 * 3g modem GPIO/misc init:
 * may have already been done in boot code, but don't count on it
 */
static void edge_init_3gmodem(void)
{
	int gpio_num;

	/*
	 * MFP106 = GPIO106 =  PERST# (module reset: output)
	 * MFP43 = GPIO43 =  W_DISABLE* (module disable: output)
	 */

	/*
	 * setup GPIO directions:
	 * only set outputs, setting as inputs (arg=0) appears to
	 * actually set as an output! bug?
	 */
	/* module reset */
	gpio_num = mfp_to_gpio(MFP_PIN_GPIO106);
	if(gpio_request(gpio_num, "3g_reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_num);
		return;
	}
	gpio_direction_output(gpio_num, 1);
	gpio_set_value(gpio_num, 0);
	mdelay(5);
	gpio_set_value(gpio_num, 1);
	gpio_free(gpio_num);
	printk(KERN_INFO "gpio %d direction set to OUTPUT and 1\n", gpio_num);

	/* module enable */
#ifndef CONFIG_ESI_EDGE_JR
	gpio_num = mfp_to_gpio(MFP_PIN_GPIO43);
#else
	gpio_num = mfp_to_gpio(MFP_PIN_GPIO83);
#endif
	if(gpio_request(gpio_num, "3g_enable")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_num);
		return;
	}
#ifdef G2K_DONGLE_MODE		// radio on by default for adrian
	gpio_direction_output(gpio_num, 1);
	gpio_set_value(gpio_num, 0);
	mdelay(5);
	gpio_set_value(gpio_num, 1);
	gpio_free(gpio_num);
	printk(KERN_INFO "gpio %d direction set to OUTPUT and 1\n", gpio_num);
#else
	gpio_direction_output(gpio_num, 0);
	gpio_free(gpio_num);
#endif

}
#endif

#endif

static void __init edge_init(void)
{
	pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	pxa168_mfp_set_fastio_drive(MFP_DS02X);

	mfp_config(ARRAY_AND_SIZE(edge_pin_config));

	/* on-chip devices */
	pxa168_add_uart(2);		/* debug console */
	pxa168_add_uart(3);		/* edge digitizer */
	edge_add_nand();
	pxa168_add_ssp(0);
	pxa168_add_twsi(0, &pwri2c_info, ARRAY_AND_SIZE(edge_i2c_board_info));
	pxa168_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(pwri2c_board_info));

	if (is_android())
		pxa168_add_keypad(&edge_android_keypad_info);
	else
		pxa168_add_keypad(&edge_keypad_info);

#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&edge_u2o_info);
#endif
#ifdef CONFIG_USB_EHCI_PXA_U2H
	pxa168_add_u2h(&edge_u2h_info);
#endif
#if defined(CONFIG_MMC_PXA_SDH)
	edge_init_mmc();
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* off-chip devices */
	edge_lcd_init();
	edge_backlight_register();

	pxa168_add_freq();
	edge_init_spi();

#if defined(CONFIG_KEYBOARD_GPIO)
	edge_gpio_keys_init();
#endif

#ifdef CONFIG_ANDROID_PMEM
	android_add_pmem("pmem", 0x02000000UL, 1, 0);
	android_add_pmem("pmem_adsp", 0x01200000UL, 0, 1);
#endif

	gpio_ec_init();

#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV3640) && defined(CONFIG_ESI_EDGE_JR)
	pxa168_add_cam();
#endif
	pm_power_off = edge_power_off;
#ifdef CONFIG_ESI_EDGE
	edge_init_epd();
#ifndef CONFIG_ESI_EDGE_JR
	edge_init_3gmodem();
#endif
#endif
}

#ifdef CONFIG_ESI_EDGE

/*
 * This variable is passed to kernel by u-boot when "maintenance" mode
 * is invoked.  It is used by mtd NOR flash driver to force all partitions
 * to READ/WRITE mode, which in turn means any of the partitions can be
 * updated during maintenance mode.
 */
int esi_mmode = 0;

int get_esi_mmode(void)
{
	return esi_mmode;
}

static int manuf_setup(char *s)
{
	esi_mmode = simple_strtoul(s, NULL, 0);
	printk("mmode = %d\n", esi_mmode);
	return 1;
}

__setup("mmode=", manuf_setup);


/*
 * This variable is passed to kernel by u-boot.  It can by used by anyone
 * who needs to know the state of persist.sys.esi_is_flipped
 * (an android property).  This state dictates LCD/EPD screen orientation.
 */
int esi_flipped = 0;

int get_esi_flipped(void)
{
	return esi_flipped;
}

static int flipped_setup(char *s)
{
	esi_flipped = simple_strtoul(s, NULL, 0);
	printk("flipped = %d\n", esi_flipped);
	return 1;
}

__setup("flipped=", flipped_setup);

#endif

MACHINE_START(EDGE, "PXA168 Edge Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = edge_init,
MACHINE_END
