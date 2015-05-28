/*
 * linux/drivers/video/pxa168fb.c -- Marvell PXA168 LCD Controller
 *
 *  Copyright (C) 2008 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2009-02-16  adapted from original version for PXA168
 *		Green Wan <gwan@marvell.com>
 *              Jun Nie <njun@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/uaccess.h>

#include <mach/io.h>
#include <mach/irqs.h>
#include <mach/pxa168fb.h>
#include <mach/hardware.h>

#include <mach/gpio.h>
#include "pxa168fb.h"

#ifdef CONFIG_DVFM
#include <mach/dvfm.h>

static int dvfm_dev_idx;
static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_disable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_disable_op_name("sys_sleep", dvfm_dev_idx);
	dvfm_disable_op_name("26MHz", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_enable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_enable_op_name("sys_sleep", dvfm_dev_idx);
	dvfm_enable_op_name("26MHz", dvfm_dev_idx);
}

#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

#define MAX_HWC_SIZE		(64*64*2)
#define MAX_REFRESH_RATE	100	/* Hz */
#define MIN_REFRESH_RATE	47	/* Hz */
#define DEFAULT_REFRESH		60	/* Hz */

/* Compatibility mode global switch .....
 *
 * This is a secret switch for user space programs that may want to
 * select color spaces and set resolutions the same as legacy PXA display
 * drivers. The switch is set and unset by setting a specific value in the
 * var_screeninfo.nonstd variable.
 *
 * To turn on compatibility with older PXA, set the MSB of nonstd to 0xAA.
 * To turn off compatibility with older PXA, set the MSB of nonstd to 0x55.
 */

static unsigned int COMPAT_MODE;
static unsigned int max_fb_size = 0;

#ifndef DISABLE_CLOCK_SHIFTING
/*
 * This "pseudo spread spectrum" algorithm is designed to reduce noise.
 * It changes the LCD clock every time an interrupt is serviced.  The
 * clock is 312MHz divided by a divider within the clk_divs array,
 * with the array index continuously rotating through the array.  
 * The divider array is defined based on the min/max clock specified by the
 * LCD panel datasheet, and is therefore specific to each target device.
 */

u32 lcd_clk_div;
u32 next_lcd_clk_div;
u32 clk_div_index = 0;

#ifdef CONFIG_ESI_EDGE_JR
/* pocket edge */
u16 clk_divs[] = {
	11,		/* 28.3636 MHz */
	10,		/* 31.2000 MHz */
	9,		/* 34.6667 MHz */
	8,		/* 39.0000 MHz */
#if 0
	/* slim margins on amplitude, so don't use for now */
	7,		/* 44.5714 MHz */
#endif
};
#else
/* edge 2.0 */
u16 clk_divs[] = {
	8,		/* 39.0000 MHz */
#if 0
	/* results in issues in PCS band, so more harm than good */
	7,		/* 44.5714 MHz */
#endif
};
#endif

#define NUM_CLK_DIVS	ARRAY_SIZE(clk_divs)
#endif

#ifdef CONFIG_PM
static int _pxa168fb_resume(struct pxa168fb_info *fbi);
static int _pxa168fb_suspend(struct pxa168fb_info *fbi, pm_message_t mesg);
#endif

#ifndef DISABLE_CLOCK_SHIFTING
u16 inline get_next_clk_div(void)
{
	clk_div_index++;
	if (clk_div_index >= NUM_CLK_DIVS) {
		clk_div_index = 0;
	}
	return clk_divs[clk_div_index];
}
#endif

void pxa168fb_spi_send(struct pxa168fb_info *fbi, void *value, int count, unsigned int spi_gpio_cs)
{
	u32 x, spi_byte_len;
	u8 *cmd = (u8 *)value;
	int i, err, isr, iopad;

	if (spi_gpio_cs != -1) {
		err = gpio_request(spi_gpio_cs, "LCD_SPI_CS");
		if (err) {
			printk("failed to request GPIO for LCD CS\n");
			return;
		}
		gpio_direction_output(spi_gpio_cs, 1);
	}
	/* get spi data size */
	spi_byte_len = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
	spi_byte_len = (spi_byte_len >> 8) & 0xff;
	/* It should be (spi_byte_len + 7) >> 3, but spi controller request set one less than bit length */
	spi_byte_len = (spi_byte_len + 8) >> 3;
	/* spi command provided by platform should be 1, 2, or 4 byte aligned */
	if(spi_byte_len == 3)
		spi_byte_len = 4;

	iopad = readl(fbi->reg_base + SPU_IOPAD_CONTROL);
	for (i = 0; i < count; i++) {
		if ((iopad & CFG_IOPADMODE_MASK) != PIN_MODE_DUMB_18_SPI)
			writel(PIN_MODE_DUMB_18_SPI, fbi->reg_base + SPU_IOPAD_CONTROL);
		if (spi_gpio_cs != -1)
			gpio_set_value(spi_gpio_cs, 0);
		mdelay(2);
		isr = readl(fbi->reg_base + SPU_IRQ_ISR);
		writel((isr & ~SPI_IRQ_ENA_MASK), fbi->reg_base + SPU_IRQ_ISR);

		switch (spi_byte_len){
		case 1:
			writel(*cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		case 2:
			writel(*(u16*)cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		case 4:
			writel(*(u32*)cmd, fbi->reg_base + LCD_SPU_SPI_TXDATA);
			break;
		default:
			printk("Wrong spi bit length\n");
		}
		cmd += spi_byte_len;
		x = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
		x |= 0x1;
		writel(x, fbi->reg_base + LCD_SPU_SPI_CTRL);
		isr = readl(fbi->reg_base + SPU_IRQ_ISR);
		while(!(isr & SPI_IRQ_ENA_MASK)) {
			udelay(1);
			isr = readl(fbi->reg_base + SPU_IRQ_ISR);
		}
		x = readl(fbi->reg_base + LCD_SPU_SPI_CTRL);
		x &= ~0x1;
		writel(x, fbi->reg_base + LCD_SPU_SPI_CTRL);
		if (spi_gpio_cs != -1)
			gpio_set_value(spi_gpio_cs, 1);
		if ((iopad & CFG_IOPADMODE_MASK) != PIN_MODE_DUMB_18_SPI)
			writel(iopad, fbi->reg_base + SPU_IOPAD_CONTROL);
	}
	if (spi_gpio_cs != -1)
		gpio_free(spi_gpio_cs);
}
EXPORT_SYMBOL(pxa168fb_spi_send);

static int determine_best_pix_fmt(struct fb_var_screeninfo *var)
{
        unsigned char pxa_format;

        /* compatibility switch: if var->nonstd MSB is 0xAA then skip to 
         * using the nonstd variable to select the color space.
         */
        if(COMPAT_MODE != 0x2625) {

                /*
                 * Pseudocolor mode?
                 */
                if (var->bits_per_pixel == 8)
                        return PIX_FMT_PSEUDOCOLOR;

		/*
                 * Check for YUV422PACK.
                 */
                if (var->bits_per_pixel == 16 && var->red.length == 16 &&
                    var->green.length == 16 && var->blue.length == 16) {
                        if (var->red.offset >= var->blue.offset) {
			if (var->red.offset == 4)
				return PIX_FMT_YUV422PACK;
			else
				return PIX_FMT_YUYV422PACK;
                        } else
                                return PIX_FMT_YVU422PACK;
                }

                /*
                 * Check for 565/1555.
                 */
                if (var->bits_per_pixel == 16 && var->red.length <= 5 &&
                    var->green.length <= 6 && var->blue.length <= 5) {
                        if (var->transp.length == 0) {
                                if (var->red.offset >= var->blue.offset)
                                        return PIX_FMT_RGB565;
                                else
                                        return PIX_FMT_BGR565;
                        }

                        if (var->transp.length == 1 && var->green.length <= 5) {
                                if (var->red.offset >= var->blue.offset)
                                        return PIX_FMT_RGB1555;
                                else
                                        return PIX_FMT_BGR1555;
                        }

                        /* fall through */
                }

                /*
                 * Check for 888/A888.
                 */
                if (var->bits_per_pixel <= 32 && var->red.length <= 8 &&
                    var->green.length <= 8 && var->blue.length <= 8) {
                        if (var->bits_per_pixel == 24 && var->transp.length == 0) {
                                if (var->red.offset >= var->blue.offset)
                                        return PIX_FMT_RGB888PACK;
                                else
                                        return PIX_FMT_BGR888PACK;
                        }

			if (var->bits_per_pixel == 32 && var->transp.offset == 24) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGBA888;
				else
					return PIX_FMT_BGRA888;
			} else {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGB888UNPACK;
				else
					return PIX_FMT_BGR888UNPACK;
			}

			/* fall through */
		}

	}

        else {

                pxa_format = (var->nonstd >> 20) & 0xf;
        
                switch (pxa_format) {
                case 0:
                        return PIX_FMT_RGB565;
                        break;
                case 5:
                        return PIX_FMT_RGB1555;
                        break;
                case 6:
                        return PIX_FMT_RGB888PACK;
                        break;
                case 7:
                        return PIX_FMT_RGB888UNPACK;
                        break;
                case 8:
                        return PIX_FMT_RGBA888;
                        break;
                case 9:
                        return PIX_FMT_YUV422PACK;
                        break;

                default:
                        return -EINVAL;
                }
        }

        

	return -EINVAL;
}

static void set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt)
{
	switch (pix_fmt) {
	case PIX_FMT_RGB565:
		var->bits_per_pixel = 16;
		var->red.offset = 11;    var->red.length = 5;
		var->green.offset = 5;   var->green.length = 6;
		var->blue.offset = 0;    var->blue.length = 5;
		var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
		break;
	case PIX_FMT_BGR565:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 5;
		var->green.offset = 5;   var->green.length = 6;
		var->blue.offset = 11;   var->blue.length = 5;
		var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
		break;
	case PIX_FMT_RGB1555:
		var->bits_per_pixel = 16;
		var->red.offset = 10;    var->red.length = 5;
		var->green.offset = 5;   var->green.length = 5;
		var->blue.offset = 0;    var->blue.length = 5;
		var->transp.offset = 15; var->transp.length = 1;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 5 << 20;
		break;
	case PIX_FMT_BGR1555:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 5;
		var->green.offset = 5;   var->green.length = 5;
		var->blue.offset = 10;   var->blue.length = 5;
		var->transp.offset = 15; var->transp.length = 1;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 5 << 20;
                break;
	case PIX_FMT_RGB888PACK:
		var->bits_per_pixel = 24;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 6 << 20;
                break;
	case PIX_FMT_BGR888PACK:
		var->bits_per_pixel = 24;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 6 << 20;
                break;
	case PIX_FMT_RGB888UNPACK:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 7 << 20;
                break;
	case PIX_FMT_BGR888UNPACK:
		var->bits_per_pixel = 32;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 7 << 20;
                break;
	case PIX_FMT_RGBA888:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 24; var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 8 << 20;
                break;
	case PIX_FMT_BGRA888:
		var->bits_per_pixel = 32;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 24; var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 8 << 20;
                break;
        case PIX_FMT_YUYV422PACK:
                var->bits_per_pixel = 16;
                var->red.offset = 8;     var->red.length = 16;
                var->green.offset = 4;   var->green.length = 16;
                var->blue.offset = 0;   var->blue.length = 16;
                var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 9 << 20;
                break;
        case PIX_FMT_YVU422PACK:
                var->bits_per_pixel = 16;
                var->red.offset = 0;     var->red.length = 16;
                var->green.offset = 8;   var->green.length = 16;
                var->blue.offset = 12;   var->blue.length = 16;
                var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 9 << 20;
                break;
        case PIX_FMT_YUV422PACK:
                var->bits_per_pixel = 16;
                var->red.offset = 4;     var->red.length = 16;
                var->green.offset = 12;   var->green.length = 16;
                var->blue.offset = 0;    var->blue.length = 16;
                var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 9 << 20;
                break;

	case PIX_FMT_PSEUDOCOLOR:
		var->bits_per_pixel = 8;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 0;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		break;
	}
}

static void set_mode(struct pxa168fb_info *fbi, struct fb_var_screeninfo *var,
		     const struct fb_videomode *mode, int pix_fmt, int ystretch)
{

	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
        set_pix_fmt(var, pix_fmt);

	var->xres = mode->xres;
	var->yres = mode->yres;
	var->xres_virtual = max(var->xres, var->xres_virtual);
	if (ystretch)
                var->yres_virtual = var->yres *2;
	else
		var->yres_virtual = max(var->yres, var->yres_virtual);
	var->grayscale = 0;
	var->accel_flags = FB_ACCEL_NONE;
	var->pixclock = mode->pixclock;
	var->left_margin = mode->left_margin;
	var->right_margin = mode->right_margin;
	var->upper_margin = mode->upper_margin;
	var->lower_margin = mode->lower_margin;
	var->hsync_len = mode->hsync_len;
	var->vsync_len = mode->vsync_len;
	var->sync = mode->sync;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->rotate = FB_ROTATE_UR;

#ifndef CONFIG_ESI_EDGE_JR
	/* 
	 * Probably not the right spot for this, but otherwise we're having
	 * to hard code it in android libs/ui/EGLDisplaySurface.cpp...
	 */
	var->width = 220;
	var->height = 129;
#else
	var->width = 154;
	var->height = 86;
#endif
}

static int pxa168fb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);
        if (var->bits_per_pixel == 8)
		return -EINVAL;

        /* compatibility mode: if the MSB of var->nonstd is 0xAA then 
         * set xres_virtual and yres_virtual to xres and yres.
         */

        if((var->nonstd >> 24) == 0xAA) 
                COMPAT_MODE = 0x2625;

        if((var->nonstd >> 24) == 0x55)
                COMPAT_MODE = 0x0;

	/*
	 * Basic geometry sanity checks.
	 */
	if (var->xoffset + var->xres > var->xres_virtual)
		return -EINVAL;
	if (var->yoffset + var->yres > var->yres_virtual)
		return -EINVAL;
	if (var->xres + var->right_margin +
	    var->hsync_len + var->left_margin > 2048)
		return -EINVAL;
	if (var->yres + var->lower_margin +
	    var->vsync_len + var->upper_margin > 2048)
		return -EINVAL;

	/*
	 * Check size of framebuffer.
	 */
	if (var->xres_virtual * var->yres_virtual *
	    (var->bits_per_pixel >> 3) > max_fb_size)
		return -EINVAL;

	return 0;
}

/*
 * The hardware clock divider has an integer and a fractional
 * stage:
 *
 *	clk2 = clk_in / integer_divider
 *	clk_out = clk2 * (1 - (fractional_divider >> 12))
 *
 * Calculate integer and fractional divider for given clk_in
 * and clk_out.
 */
static void set_clock_divider(struct pxa168fb_info *fbi,
			      const struct fb_videomode *m)
{
	int divider_int;
	int needed_pixclk;
	u64 div_result;
	u32 x = 0;

	/*
	 * Notice: The field pixclock is used by linux fb
	 * is in pixel second. E.g. struct fb_videomode &
	 * struct fb_var_screeninfo
	 */

	/*
	 * Check input values.
	 */
	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
        if (!m || !m->pixclock || !m->refresh) {
		printk(KERN_ERR "Input refresh or pixclock is wrong.\n");
		return;
	}

	/*
	 * Using PLL/AXI clock.
	 */
	x = 0x80000000;

	/*
	 * Calc divider according to refresh rate.
	 */
	div_result = 1000000000000ll;
	do_div(div_result, m->pixclock);
	needed_pixclk = (u32)div_result;

	divider_int = clk_get_rate(fbi->clk) / needed_pixclk;

	/* check whether divisor is too small. */
	if (divider_int < 2) {
		printk(KERN_WARNING "Warning: clock source is too slow."
				 "Try smaller resolution\n");
		divider_int = 2;
	}

	/*
	 * Set setting to reg.
	 */
	x |= divider_int;
#ifndef DISABLE_CLOCK_SHIFTING
	lcd_clk_div = x;
#endif
	writel(x, fbi->reg_base + LCD_CFG_SCLK_DIV);
}

static void set_dma_control0(struct pxa168fb_info *fbi)
{
	u32 x;


        dev_dbg(fbi->fb_info->dev,"Enter %s\n", __FUNCTION__);
	/*
	 * Set bit to enable graphics DMA.
	 */
	x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
	x |= fbi->active ? 0x00000100 : 0;

	/*
	 * If we are in a pseudo-color mode, we need to enable
	 * palette lookup.
	 */
	if (fbi->pix_fmt == PIX_FMT_PSEUDOCOLOR)
		x |= 0x10000000;

	/*
	 * Configure hardware pixel format.
	 */
	x &= ~(0xF << 16);
	x |= (fbi->pix_fmt >> 1) << 16;

	/*
	 * Check YUV422PACK 
	 */
	x &= ~((1 << 9) | (1 << 11) | (1 << 10) | (1 << 12));
	if (((fbi->pix_fmt >> 1) == 5) || (fbi->pix_fmt & 0x1000)) {
		x |= 1 << 9;
		x |= (fbi->panel_rbswap) << 12;
		if (fbi->pix_fmt == 11)
			x |= 1 << 11;
		if (fbi->pix_fmt & 0x1000)
			x |= 1 << 10;
	} else {

		/*
		 * Check red and blue pixel swap.
		 * 1. source data swap. BGR[M:L] rather than RGB[M:L] is stored in memeory as source format.
		 * 2. panel output data swap
		 */
		x |= (((fbi->pix_fmt & 1) ^ 1) ^ (fbi->panel_rbswap)) << 12;
	}

        x |= CFG_ARBFAST_ENA(1);

	writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL0);
}

static void set_dma_control1(struct pxa168fb_info *fbi, int sync)
{
	u32 x;

	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
        /*
	 * Configure default bits: vsync triggers DMA, gated clock
	 * enable, power save enable, configure alpha registers to
	 * display 100% graphics, and set pixel command.
	 */
	x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1);
	
	/*
	 * We trigger DMA on the falling edge of vsync if vsync is
	 * active low, or on the rising edge if vsync is active high.
	 */
	if (!(sync & FB_SYNC_VERT_HIGH_ACT))
		x |= 0x08000000;


	writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL1);
}

static void set_graphics_start(struct fb_info *info, int xoffset, int yoffset)
{
	struct pxa168fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int pixel_offset;
	unsigned long addr;
        static int debugcount = 0;

	
        if (debugcount < 10)
                debugcount++;

        if (debugcount < 9) 
                dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

        pixel_offset = (yoffset * var->xres_virtual) + xoffset;

	addr = fbi->fb_start_dma + (pixel_offset * (var->bits_per_pixel >> 3));
	writel(addr, fbi->reg_base + LCD_CFG_GRA_START_ADDR0);
}

static void set_dumb_panel_control(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	u32 x;

        dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

        /*
	 * Preserve enable flag.
	 */
	x = readl(fbi->reg_base + LCD_SPU_DUMB_CTRL) & 0x00000001;

	x |= (fbi->is_blanked ? 0x7 : mi->dumb_mode) << 28;
	x |= mi->gpio_output_data << 20;
	x |= mi->gpio_output_mask << 12;
	x |= mi->panel_rgb_reverse_lanes ? 0x00000080 : 0;
	x |= mi->invert_composite_blank ? 0x00000040 : 0;
	x |= (info->var.sync & FB_SYNC_COMP_HIGH_ACT) ? 0x00000020 : 0;
	x |= mi->invert_pix_val_ena ? 0x00000010 : 0;
	x |= (info->var.sync & FB_SYNC_VERT_HIGH_ACT) ? 0x00000008 : 0;
	x |= (info->var.sync & FB_SYNC_HOR_HIGH_ACT) ? 0x00000004 : 0;
	x |= mi->invert_pixclock ? 0x00000002 : 0;

	writel(x, fbi->reg_base + LCD_SPU_DUMB_CTRL);
}

static void set_dumb_screen_dimensions(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct fb_var_screeninfo *v = &info->var;
	int x;
	int y;

	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

        x = v->xres + v->right_margin + v->hsync_len + v->left_margin;
	y = v->yres + v->lower_margin + v->vsync_len + v->upper_margin;

	writel((y << 16) | x, fbi->reg_base + LCD_SPUT_V_H_TOTAL);
}

static void pxa168fb_clear_framebuffer(struct fb_info *info)
{
        struct pxa168fb_info *fbi = info->par;

        memset(fbi->fb_start, 0, fbi->fb_size);
}

static int pxa168fb_set_par(struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	const struct fb_videomode *mode;
	int pix_fmt;
	u32 x;
	struct pxa168fb_mach_info *mi;

	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);
	
        mi = fbi->dev->platform_data;
	/*
	 * Determine which pixel format we're going to use.
	 */
	pix_fmt = determine_best_pix_fmt(&info->var);
	if (pix_fmt < 0)
		return pix_fmt;
	fbi->pix_fmt = pix_fmt;
        dev_dbg(info->dev, "Pixel Format = %d\n", pix_fmt);

	/*
	 * Set additional mode info.
	 */
	if (pix_fmt == PIX_FMT_PSEUDOCOLOR)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	/*
	 * Configure global panel parameters.
	 */
	writel((var->yres << 16) | var->xres,
		fbi->reg_base + LCD_SPU_V_H_ACTIVE);

        dev_dbg(info->dev, "xres=%d yres=%d\n", var->xres, var->yres);
	/*
	 * convet var to video mode
	 */

	mode = fb_find_best_mode(var, &info->modelist);
	set_mode(fbi, var, mode, pix_fmt, 1);
	
	info->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;
	info->fix.ypanstep = var->yres;

        info->fix.smem_len = var->xres_virtual * var->yres_virtual * var->bits_per_pixel/8;
        info->screen_size = info->fix.smem_len;

	/* Calculate clock divisor. */
	set_clock_divider(fbi, mode);

	/* Configure dma ctrl regs. */
	set_dma_control0(fbi);
	set_dma_control1(fbi, info->var.sync);

	/*
	 * Configure graphics DMA parameters.
	 */
	set_graphics_start(info, info->var.xoffset, info->var.yoffset);
	x = readl(fbi->reg_base + LCD_CFG_GRA_PITCH);
	x = (x & ~0xFFFF) | ((var->xres_virtual * var->bits_per_pixel) >> 3);
	writel(x, fbi->reg_base + LCD_CFG_GRA_PITCH);
	writel((var->yres << 16) | var->xres,
		fbi->reg_base + LCD_SPU_GRA_HPXL_VLN);
	writel((var->yres << 16) | var->xres,
		fbi->reg_base + LCD_SPU_GZM_HPXL_VLN);

	/*
	 * Configure dumb panel ctrl regs & timings.
	 */
	set_dumb_panel_control(info);
	set_dumb_screen_dimensions(info);

	writel((var->left_margin << 16) | var->right_margin,
			fbi->reg_base + LCD_SPU_H_PORCH);
	writel((var->upper_margin << 16) | var->lower_margin,
			fbi->reg_base + LCD_SPU_V_PORCH);


	x = readl(fbi->reg_base + LCD_SPU_DUMB_CTRL);
	if ((x & 1) == 0)
		writel(x | 1, fbi->reg_base + LCD_SPU_DUMB_CTRL);
	return 0;
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	return ((chan & 0xffff) >> (16 - bf->length)) << bf->offset;
}

static u32 to_rgb(u16 red, u16 green, u16 blue)
{
	red >>= 8;
	green >>= 8;
	blue >>= 8;

	return (red << 16) | (green << 8) | blue;
}

static int
pxa168fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
		 unsigned int blue, unsigned int trans, struct fb_info *info)
{
	struct pxa168fb_info *fbi = info->par;
	u32 val;

        if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	if (info->fix.visual == FB_VISUAL_TRUECOLOR && regno < 16) {
		val =  chan_to_field(red,   &info->var.red);
		val |= chan_to_field(green, &info->var.green);
		val |= chan_to_field(blue , &info->var.blue);
		fbi->pseudo_palette[regno] = val;
	}

	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR && regno < 256) {
		val = to_rgb(red, green, blue);
		writel(val, fbi->reg_base + LCD_SPU_SRAM_WRDAT);
		writel(0x8300 | regno, fbi->reg_base + LCD_SPU_SRAM_CTRL);
	}

	return 0;
}

static int pxa168fb_blank(int blank, struct fb_info *info)
{
#ifdef CONFIG_PM
	struct pxa168fb_info *fbi = info->par;

	pm_message_t mesg = PMSG_SUSPEND;

		switch (blank) {
			case FB_BLANK_POWERDOWN:
			case FB_BLANK_VSYNC_SUSPEND:
			case FB_BLANK_HSYNC_SUSPEND:
			case FB_BLANK_NORMAL:
				_pxa168fb_suspend(fbi, mesg);
				break;

			case FB_BLANK_UNBLANK:
				_pxa168fb_resume(fbi);
				break;
			default:
				break;
		}
		return 0;
#else

	fbi->is_blanked = (blank == FB_BLANK_UNBLANK) ? 0 : 1;
	set_dumb_panel_control(info);

	return 0;
#endif
}

static int pxa168fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);
        set_graphics_start(info, var->xoffset, var->yoffset);

	return 0;
}

static irqreturn_t pxa168fb_handle_irq(int irq, void *dev_id)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)dev_id;
	u32     isr;

#ifndef DISABLE_CLOCK_SHIFTING
	u32 clk_div = lcd_clk_div;
	clk_div &= ~CLK_INT_DIV_MASK;
	clk_div |= get_next_clk_div();
	writel(clk_div, fbi->reg_base + LCD_CFG_SCLK_DIV);
#endif
	isr = readl(fbi->reg_base+SPU_IRQ_ISR);

	if ((isr & GRA_FRAME_IRQ0_ENA_MASK)) {
		/* wake up queue. */
		atomic_set(&fbi->w_intr, 1);
		wake_up(&fbi->w_intr_wq);

		writel(isr & (~GRA_FRAME_IRQ0_ENA_MASK),
			fbi->reg_base+SPU_IRQ_ISR);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
static void debug_identify_called_ioctl(struct fb_info *info, int cmd, unsigned long arg)
{
        switch (cmd) {
        case FBIO_CLEAR_FRAMEBUFFER:
                dev_dbg(fi->dev," FBIO_CLEAR_FRAMEBUFFER\n");
                break;
        case FBIOPUT_SWAP_GRAPHIC_RED_BLUE:
                dev_dbg(info->dev," FBIOPUT_SWAP_GRAPHIC_RED_BLUE with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_SWAP_GRAPHIC_U_V:
                dev_dbg(info->dev," FBIOPUT_SWAP_GRAPHIC_U_V with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_SWAP_GRAPHIC_Y_UV:
                dev_dbg(info->dev," FBIOPUT_SWAP_GRAPHIC_Y_UV with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_VIDEO_ALPHABLEND:
                dev_dbg(info->dev," FBIOPUT_VIDEO_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_GLOBAL_ALPHABLEND:
                dev_dbg(info->dev," FBIOPUT_GLOBAL_ALPHABLEND with arg = %08x\n",(unsigned int) arg);
                break;
        case FBIOPUT_GRAPHIC_ALPHABLEND:
                dev_dbg(info->dev," FBIOPUT_GRAPHIC_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
                break;

        }
}
#endif

static int pxa168_graphic_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
        int blendval;
        int val;
        unsigned char param;
        struct pxa168fb_info *fbi = info->par;

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
        debug_identify_called_ioctl(info, cmd, arg);
#endif

        switch (cmd) {
        
        case FBIO_CLEAR_FRAMEBUFFER:
                pxa168fb_clear_framebuffer(info);
                return 0;
                break;

        case FBIOPUT_VIDEO_ALPHABLEND:
                /*
                 *  This puts the blending control to the Video layer.
                 */
                val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1);
                val &= ~CFG_ALPHA_MODE_MASK;
                val &= ~CFG_ALPHA_MASK;
                val |= CFG_ALPHA_MODE(0);
                val |= CFG_ALPHA(0xff);
                writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL1);
                return 0;
                break;

        case FBIOPUT_GLOBAL_ALPHABLEND:
                /*
                 *  The userspace application can specify a byte value for the amount of global blend 
                 *  between the video layer and the graphic layer. 
                 *
                 *  The alpha blending is per the formula below: 
                 *  P = (V[P] * blendval/255) + (G[P] * (1 - blendval/255))
                 *
                 *     where: P = Pixel value, V = Video Layer, and G = Graphic Layer
                 */
                blendval = (arg & 0xff);
                val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1);
                val &= ~CFG_ALPHA_MODE_MASK;
                val &= ~CFG_ALPHA_MASK;
                val |= CFG_ALPHA_MODE(2);
                val |= CFG_ALPHA(blendval);
                writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL1);
                return 0;
                break;

        case FBIOPUT_GRAPHIC_ALPHABLEND:
                /*
                 *  This puts the blending back to the default mode of allowing the 
                 *  graphic layer to do pixel level blending. 
                 */
                val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1);
                val &= ~CFG_ALPHA_MODE_MASK;
                val &= ~CFG_ALPHA_MASK;
                val |= CFG_ALPHA_MODE(1);
                val |= CFG_ALPHA(0x0);
                writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL1);
                return 0;
                break;

        case FBIOPUT_SWAP_GRAPHIC_RED_BLUE:
                param = (arg & 0x1);

                val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
                val &= ~CFG_GRA_SWAPRB_MASK;
                val |= CFG_GRA_SWAPRB(param);
                writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL0);
                return 0;
                break;

        case FBIOPUT_SWAP_GRAPHIC_U_V:
                param = (arg & 0x1);

                val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
                val &= ~CFG_GRA_SWAPUV_MASK;
                val |= CFG_GRA_SWAPUV(param);
                writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL0);
                return 0;
                break;

        case FBIOPUT_SWAP_GRAPHIC_Y_UV:
                param = (arg & 0x1);

                val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
                val &= ~CFG_GRA_SWAPYU_MASK;
                val |= CFG_GRA_SWAPYU(param);
                writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL0);
                return 0;
                break;



        default:
                break;
 
        }
        return 0;
}

static int pxa168fb_release(struct fb_info *info, int user)
{
        struct fb_var_screeninfo *var = &info->var;

        dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);
        
        /* Turn off compatibility mode */

        var->nonstd &= ~0xff000000;
        COMPAT_MODE = 0;
	return 0;
}


static struct fb_ops pxa168fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= pxa168fb_check_var,
	.fb_release	= pxa168fb_release,
	.fb_set_par	= pxa168fb_set_par,
	.fb_setcolreg	= pxa168fb_setcolreg,
	.fb_blank	= pxa168fb_blank,
	.fb_pan_display	= pxa168fb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
        .fb_ioctl       = pxa168_graphic_ioctl,
};

static int pxa168fb_init_mode(struct fb_info *info,
			      struct pxa168fb_mach_info *mi)
{
	struct pxa168fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int ret = 0;
	u32 total_w, total_h, refresh;
	u64 div_result;
	const struct fb_videomode *m;

        dev_dbg(info->dev, "Enter %s\n", __FUNCTION__);

	/*
	 * Set default value
	 */
	refresh = DEFAULT_REFRESH;

	/*
	 * If has bootargs, apply it first.
	 */
	if (fbi->dft_vmode.xres && fbi->dft_vmode.yres &&
	    fbi->dft_vmode.refresh) {
		/* set data according bootargs */
		var->xres = fbi->dft_vmode.xres;
		var->yres = fbi->dft_vmode.yres;
		refresh = fbi->dft_vmode.refresh;
	}

	/* try to find best video mode. */
	m = fb_find_best_mode(&info->var, &info->modelist);
	if (m)
		fb_videomode_to_var(&info->var, m);

	/* Init settings. */
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres * 2;

	/* correct pixclock. */
	total_w = var->xres + var->left_margin + var->right_margin +
		  var->hsync_len;
	total_h = var->yres + var->upper_margin + var->lower_margin +
		  var->vsync_len;

	div_result = 1000000000000ll;
	do_div(div_result, total_w * total_h * refresh);
	var->pixclock = (u32)div_result;

	return ret;
}

static void pxa168fb_set_default(struct pxa168fb_info *fbi, struct pxa168fb_mach_info *mi)
{
	/*
	 * Configure default register values.
	 */
	writel(0x00000000, fbi->reg_base + LCD_SPU_BLANKCOLOR);
	writel(mi->io_pin_allocation_mode, fbi->reg_base + SPU_IOPAD_CONTROL);
	writel(0x00000000, fbi->reg_base + LCD_CFG_GRA_START_ADDR1);
	writel(0x00000000, fbi->reg_base + LCD_SPU_GRA_OVSA_HPXL_VLN);
	writel(0x0, fbi->reg_base + LCD_SPU_SRAM_PARA0);
	writel(CFG_CSB_256x32(0x1)|CFG_CSB_256x24(0x1)|CFG_CSB_256x8(0x1),
		fbi->reg_base + LCD_SPU_SRAM_PARA1);
        writel(VIDEO_FIFO(1) | GRAPHIC_FIFO(1) , fbi->reg_base + LCD_FIFO_DEPTH);


        /*
	 * Configure default bits: vsync triggers DMA, 
         * power save enable, configure alpha registers to
	 * display 100% graphics, and set pixel command.
	 */
	writel(0x2011ff81, fbi->reg_base + LCD_SPU_DMA_CTRL1);
}

static void pxa168fb_power(struct pxa168fb_info *fbi, struct pxa168fb_mach_info *mi, int on)
{
	if (on)
		set_dvfm_constraint();

	if ((mi->spi_ctrl != -1) && (mi->spi_ctrl & CFG_SPI_ENA_MASK))
		writel(mi->spi_ctrl, fbi->reg_base + LCD_SPU_SPI_CTRL);

	if (mi->pxa168fb_lcd_power)
		mi->pxa168fb_lcd_power(fbi, mi->spi_gpio_cs, mi->spi_gpio_reset, on);

	if (!on)
		unset_dvfm_constraint();
}

static int __init get_fb_size(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	max_fb_size = n;
	return 1;
}
__setup("fb_size=", get_fb_size);

static int  pxa168fb_probe(struct platform_device *pdev)
{
	struct pxa168fb_mach_info *mi;
	struct fb_info *info = 0;
	struct pxa168fb_info *fbi = 0;
	struct resource *res;
	struct clk *clk;
	int irq, ret;

	mi = pdev->dev.platform_data;
	if (mi == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	clk = clk_get(&pdev->dev, "LCDCLK");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get LCDCLK");
		return PTR_ERR(clk);
	}
#ifndef DISABLE_CLOCK_SHIFTING
	printk("%s: NUM_CLK_DIVS = %d\n", __FUNCTION__, NUM_CLK_DIVS);
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no IO memory defined\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		return -ENOENT;
	}

	info = framebuffer_alloc(sizeof(struct pxa168fb_info), &pdev->dev);
	if (info == NULL)
		return -ENOMEM;

	/* Initialize private data */
	fbi = info->par;
	fbi->fb_info = info;
	platform_set_drvdata(pdev, fbi);
	fbi->clk = clk;
	fbi->dev = &pdev->dev;
	fbi->panel_rbswap = mi->panel_rbswap;
	fbi->is_blanked = 0;
	fbi->debug = 0;
	fbi->active = mi->active;

	/* Initialize boot setting */
	fbi->dft_vmode.xres = mi->modes->xres;
	fbi->dft_vmode.yres = mi->modes->yres;
	fbi->dft_vmode.refresh = mi->modes->refresh;

	init_waitqueue_head(&fbi->w_intr_wq);

	/*
	 * Initialise static fb parameters.
	 */
	info->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
			FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	info->node = -1;
	strcpy(info->fix.id, mi->id);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 0;
	info->fix.ypanstep = 0;
	info->fix.ywrapstep = 0;
	info->fix.mmio_start = res->start;
	info->fix.mmio_len = res->end - res->start + 1;
	info->fix.accel = FB_ACCEL_NONE;
	info->fbops = &pxa168fb_ops;
	info->pseudo_palette = fbi->pseudo_palette;

	/*
	 * Map LCD controller registers.
	 */
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		ret = -ENOMEM;
		goto failed;
	}

	/*
	 * Allocate framebuffer memory.
	 */

	if (!max_fb_size) {
		if (mi->max_fb_size)
			max_fb_size = mi->max_fb_size;
		else
			max_fb_size = DEFAULT_FB_SIZE;
	}
	max_fb_size = PAGE_ALIGN(max_fb_size);
	fbi->fb_size = max_fb_size;

	fbi->fb_start = dma_alloc_writecombine(fbi->dev, max_fb_size,
						&fbi->fb_start_dma,
						GFP_KERNEL);

	if (!fbi->fb_start || !fbi->fb_start_dma) {
		fbi->fb_start = (void *)__get_free_pages(GFP_DMA | GFP_KERNEL,
						get_order(fbi->fb_size));
		fbi->fb_start_dma = (dma_addr_t)__virt_to_phys(fbi->fb_start);
	}

	if (fbi->fb_start == NULL) {
		printk("%s: no enough memory!\n", __func__);
		ret = -ENOMEM;
		goto failed;
	}

	memset(fbi->fb_start, 0, fbi->fb_size);
	info->fix.smem_start = fbi->fb_start_dma;
	info->fix.smem_len = fbi->fb_size;
	info->screen_base = fbi->fb_start;
	info->screen_size = fbi->fb_size;

	/*
	 * Set video mode according to platform data.
	 */
	set_mode(fbi, &info->var, mi->modes, mi->pix_fmt, 1);

	fb_videomode_to_modelist(mi->modes, mi->num_modes, &info->modelist);

	/*
	 * init video mode data.
	 */
	pxa168fb_init_mode(info, mi);

	/*
	 * enable controller clock
	 */
	clk_enable(fbi->clk);

	/*
	 * Fill in sane defaults.
	 */
	pxa168fb_set_par(info);

	pxa168fb_set_default(fbi, mi);

	/*
	 * Allocate color map.
	 */
	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		ret = -ENOMEM;
		goto failed_free_clk;
	}

	/*
	 * Register irq handler.
	 */
	ret = request_irq(irq, pxa168fb_handle_irq, IRQF_SHARED, mi->id, fbi);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to request IRQ\n");
		ret = -ENXIO;
		goto failed_free_cmap;
	}
	writel(0x0, fbi->reg_base + SPU_IRQ_ISR);

	/*
	 * Enable GFX interrupt
	 */
	writel(GRA_FRAME_IRQ0_ENA(0x1), fbi->reg_base + SPU_IRQ_ENA);

	/*
	 * Register framebuffer.
	 */
	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pxa168-fb: %d\n", ret);
		ret = -ENXIO;
		goto failed_free_irq;
	}
        printk(KERN_INFO "pxa168fb: frame buffer device was loaded"
                " to /dev/fb%d <%s>.\n", info->node, info->fix.id);

	pxa168fb_power(fbi, mi, 1);
	return 0;

failed_free_irq:
	free_irq(irq, fbi);
failed_free_cmap:
	fb_dealloc_cmap(&info->cmap);
failed_free_clk:
	clk_disable(fbi->clk);
	clk_put(fbi->clk);
failed:
	pr_err("pxa168-fb: frame buffer device init failed\n");
	platform_set_drvdata(pdev, NULL);
	fb_dealloc_cmap(&info->cmap);

	if (fbi && fbi->reg_base) {
		iounmap(fbi->reg_base);
		kfree(fbi);
	}

	return ret;
}

#ifdef CONFIG_PM
static int _pxa168fb_suspend(struct pxa168fb_info *fbi, pm_message_t mesg)
{
	struct fb_info *info = fbi->fb_info;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	unsigned int reg;

//	printk(KERN_INFO "_pxa168fb_suspend(): state = %d.\n", mesg.event);

	if(FBINFO_STATE_SUSPENDED == info->state)
		return 0;

	pxa168fb_power(fbi, mi, 0);

	/* Disable interrupts */
	reg = readl(fbi->reg_base+SPU_IRQ_ENA);
	reg &= ~GRA_FRAME_IRQ0_ENA_MASK;
	writel(reg, fbi->reg_base+SPU_IRQ_ENA);

	fb_set_suspend(info, 1);

	clk_disable(fbi->clk);

	return 0;
}

static int _pxa168fb_resume(struct pxa168fb_info *fbi)
{
	struct fb_info *info = fbi->fb_info;
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;

//	printk(KERN_INFO "pxa168fb resuming...\n");

	if(FBINFO_STATE_RUNNING == info->state)
		return 0;

	clk_enable(fbi->clk);

	pxa168fb_set_par(info);

	pxa168fb_set_default(fbi, mi);

	fb_set_suspend(info, 0);

	/* Enable interrupts */
	writel(0x0, fbi->reg_base + SPU_IRQ_ISR);
	writel(readl(fbi->reg_base + SPU_IRQ_ENA) | GRA_FRAME_IRQ0_ENA_MASK,
			fbi->reg_base + SPU_IRQ_ENA);

	pxa168fb_power(fbi, mi, 1);
//	printk(KERN_INFO "pxa168fb resumed\n");
	return 0;
}

static int pxa168fb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);

	_pxa168fb_suspend(fbi, mesg);
	pdev->dev.power.power_state = mesg;

	return 0;
}

static int pxa168fb_resume(struct platform_device *pdev)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);
	_pxa168fb_resume(fbi);
	return 0;
}
#endif

static struct platform_driver pxa168fb_driver = {
	.driver		= {
		.name	= "pxa168-fb",
		.owner	= THIS_MODULE,
	},
	.probe		= pxa168fb_probe,
#ifdef CONFIG_PM
	.suspend	= pxa168fb_suspend,
	.resume		= pxa168fb_resume,
#endif
};

static int __devinit pxa168fb_init(void)
{
#ifdef CONFIG_DVFM
	dvfm_register("LCD Base", &dvfm_dev_idx);
#endif
	return platform_driver_register(&pxa168fb_driver);
}
module_init(pxa168fb_init);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@marvell.com>");
MODULE_DESCRIPTION("Framebuffer driver for PXA168");
MODULE_LICENSE("GPL");
