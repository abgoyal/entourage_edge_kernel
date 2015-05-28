/*
 *  Tablet driver for the Wacom Digitizer Sensor Unit Display Module
 *
 *  Copyright (c) 2008 Joe Kralowetz
 *
 *  Based on ser_mouse.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/ctype.h>
#include <asm/uaccess.h>

/**************************************/
/* Debug Defines and Macros           */
/**************************************/

#define ESI_DEBUG_CRIT
#define ESI_DEBUG_UNU
#define ESI_DEBUG_CMN
#define ESI_DEBUG_INFO
#define ESI_DEBUG_ASSERT
#undef  ESI_REPORT_PROXIMITY
#define ESI_INTER_MODULE
#define ESI_EMULATE_MOUSE
#undef ESI_ENABLE_PWR_MGMT
#define ESI_USE_GENERIC_GPIO_CALLS

enum dbg_lvls
{
	DBG_LVL_CRIT	= 0,
	DBG_LVL_UNU		= 1,
	DBG_LVL_CMN		= 2,
	DBG_LVL_INFO	= 3,
};

#ifdef ESI_DEBUG_CRIT
#define DBG_CRIT(fmt, args...) \
	{ printk(KERN_CRIT fmt, ## args); }
#else 
#define DBG_CRIT(fmt, args...) do { } while (0)
#endif

#ifdef ESI_DEBUG_UNU
#define DBG_UNU(fmt, args...) \
	{ if (wac_dbg_lvl >= DBG_LVL_UNU) printk(KERN_CRIT fmt, ## args); }
#else 
#define DBG_UNU(fmt, args...) do { } while (0)
#endif

#ifdef ESI_DEBUG_CMN
#define DBG_CMN(fmt, args...) \
	{ if (wac_dbg_lvl >= DBG_LVL_CMN) printk(KERN_CRIT fmt, ## args); }
#else 
#define DBG_CMN(fmt, args...) do { } while (0)
#endif

#ifdef ESI_DEBUG_INFO
#define DBG_INFO(fmt, args...) \
	{ if (wac_dbg_lvl >= DBG_LVL_INFO) printk(KERN_CRIT fmt, ## args); }
#else 
#define DBG_INFO(fmt, args...) do { } while (0)
#endif

#ifdef ESI_DEBUG_ASSERT
#define assert(expr) \
	if (!(expr)) { \
		printk(KERN_CRIT "Assertion failed! %s,%s,%s,line=%d\n", \
		#expr, __FILE__, __func__, __LINE__); \
	}
#else
#define assert(expr)
#endif

unsigned int wac_dbg_lvl = DBG_LVL_CRIT;

/**************************************/
/* Driver Defines                     */
/**************************************/

#define DRIVER_DESC	"Wacom tablet driver"

MODULE_AUTHOR("Joe Kralowetz");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*** Wacom Tablet Defines ***/
#define MIN_PRESSURE	1
#define MAX_PRESSURE	1024

#ifdef CONFIG_ESI_EDGE_JR
#define MIN_RAW_X_PIXELS	0
#define MAX_RAW_X_PIXELS	12200
#define MIN_RAW_Y_PIXELS	0
#define MAX_RAW_Y_PIXELS	9000

#define MIN_COOKED_X_PIXELS	0
#define MAX_COOKED_X_PIXELS	600
#define MIN_COOKED_Y_PIXELS	0
#define MAX_COOKED_Y_PIXELS	800

#define PTS_PER_X_PIXEL		15
#define PTS_PER_Y_PIXEL		15
#else
#define MIN_RAW_X_PIXELS	0
#define MAX_RAW_X_PIXELS	22000
#define MIN_RAW_Y_PIXELS	0
#define MAX_RAW_Y_PIXELS	14000

#define MIN_COOKED_X_PIXELS	0
#define MAX_COOKED_X_PIXELS	828
#define MIN_COOKED_Y_PIXELS	0
#define MAX_COOKED_Y_PIXELS	1200

#define PTS_PER_X_PIXEL		17
#define PTS_PER_Y_PIXEL		18
#endif

#undef ESI_AM300
#ifdef ESI_AM300
/*** Register Offsets for GPIO Control ***/
#define GPIO_WACDSU_46		46
#define GPIO_WACDSU_47		47

#define GAFR_AF1			1
#define GAFR_AF2			2
#else
/*** Wacom DSU GPIO Registers ***/
#ifdef CONFIG_MACH_OMAP3_EDGE
#ifdef CONFIG_MACH_OMAP3_EDGE_REV2
#define GPIO_WACDSU_RESET		42
#define GPIO_WACDSU_SLEEP		101
#else
#define GPIO_WACDSU_RESET		100
#define GPIO_WACDSU_SLEEP		101
#endif
#else
// Edge2
#define GPIO_WACDSU_SLEEP		100
#define GPIO_WACDSU_RESET		109
#endif
#endif

/*** WacDSU Commands ***/
#define WACDSU_STOP_SAMPLING				'0'
#define WACDSU_START_SAMPLING_133PTS_SEC	'1'
#define WACDSU_START_SAMPLING_80PTS_SEC		'2'
#define WACDSU_START_SAMPLING_40PTS_SEC		'3'
#define WACDSU_SURVEY_SCAN_ENABLE			'+'
#define WACDSU_SURVEY_SCAN_DISABLE			'-'
#define WACDSU_QUERY						'*'

/*** WacDSU Control Structure ***/
struct wacdsu {
	struct input_dev *dev;
	struct serio *serio;
	struct timer_list timer;
	wait_queue_head_t waitq;
	unsigned char buf[16];
	unsigned char count;
	unsigned int op_mode;
	unsigned int pm_mode;
	unsigned int pm_state;
	unsigned int pen_holstered;
	unsigned int pixel_mode;
	unsigned int pen_mode;
	unsigned int swap_xy;
	unsigned int last_x;
	unsigned int last_y;
	int proximity;
	int touch;
	int stylus;
	int eraser;
	unsigned long last;
	char phys[32];
};

/* Pixel, Pen, and Operational Modes */
#define PIX_MODE_RAW			0
#define PIX_MODE_RAW_DEBUG		1
#define PIX_MODE_MANUAL_COOKED	2
#define PIX_MODE_CAL_COOKED		3
#define PIX_MODE_MAX			3

#define PEN_MODE_PROXIMITY		0
#define PEN_MODE_TOUCH			1
#define PEN_MODE_MAX			1

#define OP_MODE_SLEEP				0
#define OP_MODE_SURVEY				1
#define OP_MODE_SAMPLING			2
#define OP_MODE_QUERY_THEN_SLEEP	3
#define OP_MODE_QUERY_THEN_SURVEY	4
#define OP_MODE_QUERY_THEN_SAMPLE	5
#define OP_MODE_MAX					5

#define PM_MODE_AUTO				0
#define PM_MODE_MANUAL				1
#define PM_MODE_MAX					1

#define PM_STATE_HOLSTERED			0		// Sleep
#define PM_STATE_INACTIVE			1		// Survey
#define PM_STATE_ACTIVE				2		// Sample
#define PM_STATE_MAX				2

#define SAMPLE_RATE_MIN				1
#define SAMPLE_RATE_133_PS			1
#define SAMPLE_RATE_80_PS			2
#define SAMPLE_RATE_40_PS			3
#define SAMPLE_RATE_MAX				3

static unsigned char wacdsu_sampling_rate;

/*
 ** - taken from tslib/plugins/linear.c
 ** - and modified to be used in this driver.
*/
#define NUM_CALIBRATION_VALUES	7

struct wacdsu_cal {
	unsigned int rotation;			// rotation val when calibration occurred
    int a[NUM_CALIBRATION_VALUES];	// Linear scaling and offset params for x,y
};

struct wacdsu_cal cal;
#undef TEST_CAL_COOKED
#ifdef TEST_CAL_COOKED
#ifdef CONFIG_ESI_EDGE_JR
struct wacdsu_cal defcal = { 90, {-36, 4629, -1133668, -4440, -23, 53131312, 65536} };
#else
//struct wacdsu_cal defcal = { 90, {-14, -3927, 54690100, 5376, 2, -4971056, 65536} };
struct wacdsu_cal defcal = { 90, {570, 4047, -6839216, -2, -1958, 55140456, 65536536} };
#endif
#else
struct wacdsu_cal defcal = { 90, {0, 0, 0, 0, 0, 0, 0} };
#endif

struct wacdsu_xy {
    int     x;
    int     y;
};

struct wacdsu_xy xy;

/**************************************/
/* Inter-Module Function Declarations */
/**************************************/

#ifdef ESI_INTER_MODULE
typedef void *epd_sidedoor_t(unsigned int touch, unsigned int X, unsigned int Y);
epd_sidedoor_t *p_epd_sidedoor = NULL;
extern epd_sidedoor_t epd_sidedoor;

#define UPDT_MODE_MENU_UP	32
#define UPDT_MODE_BOTTOM	1
unsigned int *p_epd_update_mode = NULL;
extern unsigned int epd_update_mode;
#endif

unsigned int *p_epd_rotate_mode = NULL;
extern unsigned int epd_rotate_mode;

/**************************************/
/* Driver Function Externs            */
/**************************************/

static void wacdsu_set_sleep_mode(void);
static void wacdsu_clear_sleep_mode(struct wacdsu *wacdsu);
static int wacdsu_pen_holstered(struct wacdsu *wacdsu);
static void wacdsu_enter_holstered_state(struct wacdsu *wacdsu);
static void wacdsu_enter_survey_state(struct wacdsu *wacdsu);
static void wacdsu_enter_sample_state(struct wacdsu *wacdsu,
	unsigned char sampling_rate);
static int wacdsu_get_rotation_skew(void);
#ifdef ESI_INTER_MODULE
static void wacdsu_knock_on_sidedoor(unsigned int touch,
	unsigned int x, unsigned int y);
#endif
static void wacdsu_process_synthesized_data(struct wacdsu *wacdsu,
	int touch, int pressure, unsigned int cookedX, unsigned int cookedY,
	int inter_module);

/**************************************/
/* Driver Functions                   */
/**************************************/

static void wacdsu_init_calibration_values(void)
{
	int i;

	for (i = 0; i < NUM_CALIBRATION_VALUES; i++) {
		cal.a[i] = defcal.a[i];
	}
}

static void
wacdsu_linearize(struct wacdsu *wacdsu, struct wacdsu_xy *sample_xy)
{
	struct wacdsu_cal *p_cal = &cal;
    int xtemp, ytemp;

    xtemp = sample_xy->x; ytemp = sample_xy->y;
    sample_xy->x =   ( p_cal->a[2] +
            p_cal->a[0]*xtemp +
            p_cal->a[1]*ytemp ) / p_cal->a[6];
    sample_xy->y =   ( p_cal->a[5] +
            p_cal->a[3]*xtemp +
            p_cal->a[4]*ytemp ) / p_cal->a[6];
	if (wacdsu->swap_xy)
	{
		int tmp = sample_xy->x;
		sample_xy->x = sample_xy->y;
		sample_xy->y = tmp;
	}
}

static ssize_t wacdsu_set_calibration_values(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t size)
{
	struct wacdsu_cal tmp_cal;
	int count;
	int i;

	if ((count = sscanf(buffer, "%d %d %d %d %d %d %d %d",
		&tmp_cal.rotation, &tmp_cal.a[0], &tmp_cal.a[1],
		&tmp_cal.a[2], &tmp_cal.a[3], &tmp_cal.a[4],
		&tmp_cal.a[5], &tmp_cal.a[6])) != (NUM_CALIBRATION_VALUES+1))
	{
		DBG_UNU("[%s] count %d != expect values %d\n",
			__FUNCTION__, count, (NUM_CALIBRATION_VALUES+1));
		return count;
	}

	/* Only copy into real calibration array after we now that we got */
	/* the correct number of values */
	cal.rotation = tmp_cal.rotation;
	DBG_CMN("[%s] cal_rotation = %d\n", __FUNCTION__, cal.rotation);
	for (i = 0; i < NUM_CALIBRATION_VALUES; i++)
	{
		cal.a[i] = tmp_cal.a[i];
		DBG_CMN("[%s] cal[%d] = %d\n", __FUNCTION__, i, cal.a[i]);
	}

	return count;
}
//---------------------------------------------------------------
DEVICE_ATTR(calibration, (S_IRUGO | S_IWUGO),
	NULL, wacdsu_set_calibration_values);
//---------------------------------------------------------------

static ssize_t wacdsu_get_pixel_mode(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	struct input_dev *idev = to_input_dev(dev);
	struct wacdsu *wacdsu = (struct wacdsu *)idev->private;
	char *pixel_mode_str;

	switch (wacdsu->pixel_mode)
	{
	case PIX_MODE_RAW:
		pixel_mode_str = "Raw";
		break;
	case PIX_MODE_RAW_DEBUG:
		pixel_mode_str = "Raw Debug";
		break;
	case PIX_MODE_MANUAL_COOKED:
		pixel_mode_str = "Manual Cooked";
		break;
	case PIX_MODE_CAL_COOKED:
		pixel_mode_str = "Calib Cooked";
		break;
	default:
		pixel_mode_str = "Out of Range";
		break;
	}

	return sprintf(buffer, "%u (%s)\n", wacdsu->pixel_mode, pixel_mode_str);
}

static ssize_t wacdsu_set_pixel_mode(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t size)
{
	char *after;
	unsigned long pm = simple_strtoul(buffer, &after, 10);
	size_t count = after - buffer;

	if (*after && isspace(*after))
		count++;

	if (pm <= PIX_MODE_MAX)
	{
		struct input_dev *idev = to_input_dev(dev);
		struct wacdsu *wacdsu = (struct wacdsu *)idev->private;

		DBG_CMN("[%s] pixel_mode = %ld\n", __FUNCTION__, pm);

		wacdsu->pixel_mode = pm;
		return count;
	}
	else
	{
		DBG_CRIT("[%s] pixel_mode (%ld) invalid\n", __FUNCTION__, pm);
		return -EINVAL;
	}
}
//---------------------------------------------------------------
DEVICE_ATTR(pixel_mode, (S_IRUGO | S_IWUGO),
	wacdsu_get_pixel_mode, wacdsu_set_pixel_mode);
//---------------------------------------------------------------

static ssize_t wacdsu_get_pen_mode(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	struct input_dev *idev = to_input_dev(dev);
	struct wacdsu *wacdsu = (struct wacdsu *)idev->private;
	char *pen_mode_str;

	switch (wacdsu->pen_mode)
	{
	case PEN_MODE_PROXIMITY:
		pen_mode_str = "Proximity";
		break;
	case PEN_MODE_TOUCH:
		pen_mode_str = "Touch";
		break;
	default:
		pen_mode_str = "Out of Range";
		break;
	}

	return sprintf(buffer, "%u (%s)\n", wacdsu->pen_mode, pen_mode_str);
}

static ssize_t wacdsu_set_pen_mode(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t size)
{
	char *after;
	unsigned long pm = simple_strtoul(buffer, &after, 10);
	size_t count = after - buffer;

	if (*after && isspace(*after))
		count++;

	if (pm <= PEN_MODE_MAX)
	{
		struct input_dev *idev = to_input_dev(dev);
		struct wacdsu *wacdsu = (struct wacdsu *)idev->private;

		DBG_CMN("[%s] pen_mode = %ld\n", __FUNCTION__, pm);

		wacdsu->pen_mode = pm;
		return count;
	}
	else
	{
		DBG_CRIT("[%s] pen_mode (%ld) invalid\n", __FUNCTION__, pm);
		return -EINVAL;
	}
}
//---------------------------------------------------------------
DEVICE_ATTR(pen_mode, (S_IRUGO | S_IWUGO),
	wacdsu_get_pen_mode, wacdsu_set_pen_mode);
//---------------------------------------------------------------

/*
 * wacdsu_start_sampling() starts the sampling at the specified rate.
 */

static void wacdsu_start_sampling(struct serio *serio, unsigned char sampling_rate)
{
	switch (sampling_rate)
	{
		case SAMPLE_RATE_133_PS:
			serio_write(serio, WACDSU_START_SAMPLING_133PTS_SEC);
			break;
		case SAMPLE_RATE_80_PS:
			serio_write(serio, WACDSU_START_SAMPLING_80PTS_SEC);
			break;
		case SAMPLE_RATE_40_PS:
		default:
			serio_write(serio, WACDSU_START_SAMPLING_40PTS_SEC);
			break;
	}
}

static ssize_t wacdsu_get_op_mode(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	struct input_dev *idev = to_input_dev(dev);
	struct wacdsu *wacdsu = (struct wacdsu *)idev->private;
	char *op_mode_str;

	switch (wacdsu->op_mode)
	{
	case OP_MODE_SLEEP:
		op_mode_str = "Sleep";
		break;
	case OP_MODE_SURVEY:
		op_mode_str = "Survey";
		break;
	case OP_MODE_SAMPLING:
		op_mode_str = "Sampling";
		break;
	case OP_MODE_QUERY_THEN_SLEEP:
		op_mode_str = "Query then Sleep";
		break;
	case OP_MODE_QUERY_THEN_SURVEY:
		op_mode_str = "Query then Survey";
		break;
	case OP_MODE_QUERY_THEN_SAMPLE:
		op_mode_str = "Query then Sample";
		break;
	default:
		op_mode_str = "Out of Range";
		break;
	}

	return sprintf(buffer, "%u (%s)\n", wacdsu->op_mode, op_mode_str);
}

static ssize_t wacdsu_set_op_mode(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t size)
{
	char *after;
	unsigned long pm = simple_strtoul(buffer, &after, 10);
	size_t count = after - buffer;

	if (*after && isspace(*after))
		count++;

	if (pm <= OP_MODE_MAX)
	{
		struct input_dev *idev = to_input_dev(dev);
		struct wacdsu *wacdsu = (struct wacdsu *)idev->private;

		DBG_CMN("[%s] op_mode = %ld\n", __FUNCTION__, pm);

		if (pm == OP_MODE_SLEEP)
		{
			wacdsu->op_mode = pm;

			/* Stop sampling */
			serio_write(wacdsu->serio, WACDSU_STOP_SAMPLING);

			wacdsu_set_sleep_mode();

			/* Cleanup counter for next round */
			wacdsu->count = 0;
		}
		else if (pm == OP_MODE_SURVEY)
		{
			if (wacdsu->op_mode == OP_MODE_SLEEP)
			{
				wacdsu_clear_sleep_mode(wacdsu);

				/* Cleanup counter for next round */
				wacdsu->count = 0;

				wacdsu->op_mode = pm;

				/* Start Survey (kicks in while in low sampling rate) */
				wacdsu_start_sampling(wacdsu->serio, SAMPLE_RATE_40_PS);
				serio_write(wacdsu->serio, WACDSU_SURVEY_SCAN_ENABLE);
			}
			else if (wacdsu->op_mode == OP_MODE_SAMPLING)
			{
				/* Stop sampling */
				serio_write(wacdsu->serio, WACDSU_STOP_SAMPLING);

				wacdsu->op_mode = pm;

				/* Cleanup counter for next round */
				wacdsu->count = 0;

				/* Start Survey (kicks in while in low sampling rate) */
				wacdsu_start_sampling(wacdsu->serio, SAMPLE_RATE_40_PS);
				serio_write(wacdsu->serio, WACDSU_SURVEY_SCAN_ENABLE);
			}
			else if ((wacdsu->op_mode == OP_MODE_QUERY_THEN_SLEEP) ||
				(wacdsu->op_mode == OP_MODE_QUERY_THEN_SAMPLE))
			{
				wacdsu->op_mode = OP_MODE_QUERY_THEN_SURVEY;
			}
			/* Else if current mode == SURVEY, no change */
			/* Else if current mode == QUERY_THEN_SURVEY, let query finish */
		}
		else if (pm == OP_MODE_SAMPLING)
		{
			if (wacdsu->op_mode == OP_MODE_SLEEP)
			{
				wacdsu_clear_sleep_mode(wacdsu);

				/* Start sampling */
				wacdsu_start_sampling(wacdsu->serio, wacdsu_sampling_rate);
				wacdsu->op_mode = OP_MODE_SAMPLING;
			}
			else if (wacdsu->op_mode == OP_MODE_SURVEY)
			{
				serio_write(wacdsu->serio, WACDSU_SURVEY_SCAN_DISABLE);

				/* Cleanup counter for next round */
				wacdsu->count = 0;

				wacdsu->op_mode = pm;

				/* Start sampling */
				wacdsu_start_sampling(wacdsu->serio, wacdsu_sampling_rate);
			}
			else if ((wacdsu->op_mode == OP_MODE_QUERY_THEN_SLEEP) ||
				(wacdsu->op_mode == OP_MODE_QUERY_THEN_SURVEY))
			{
				wacdsu->op_mode = OP_MODE_QUERY_THEN_SAMPLE;
			}
			/* Else if current mode == SAMPLING, no change */
			/* Else if current mode == QUERY_THEN_SAMPLE, let query finish */
		}
		else if ((pm == OP_MODE_QUERY_THEN_SLEEP) ||
				 (pm == OP_MODE_QUERY_THEN_SURVEY) ||
				 (pm == OP_MODE_QUERY_THEN_SAMPLE))
		{
			if (wacdsu->op_mode == OP_MODE_SLEEP)
				wacdsu_clear_sleep_mode(wacdsu);

			/* Stop sampling */
			serio_write(wacdsu->serio, WACDSU_STOP_SAMPLING);

			wacdsu->op_mode = pm;

			/* Cleanup counter for next round */
			wacdsu->count = 0;

			/* Now start the query */
			serio_write(wacdsu->serio, WACDSU_QUERY);
		}

		return count;
	}
	else
	{
		DBG_CRIT("[%s] op_mode (%ld) invalid\n", __FUNCTION__, pm);
		return -EINVAL;
	}
}
//---------------------------------------------------------------
DEVICE_ATTR(op_mode, (S_IRUGO | S_IWUGO),
	wacdsu_get_op_mode, wacdsu_set_op_mode);
//---------------------------------------------------------------

static ssize_t wacdsu_get_sample_rate(struct device *dev,
	struct device_attribute *attr, char *buffer)
{
	char *sample_rate_str;

	switch (wacdsu_sampling_rate)
	{
	case SAMPLE_RATE_133_PS:
		sample_rate_str = "133_PPS";
		break;
	case SAMPLE_RATE_80_PS:
		sample_rate_str = "80_PPS";
		break;
	case SAMPLE_RATE_40_PS:
		sample_rate_str = "40_PPS";
		break;
	default:
		sample_rate_str = "Out of Range";
		break;
	}

	return sprintf(buffer, "%u (%s)\n", wacdsu_sampling_rate, sample_rate_str);
}

static ssize_t wacdsu_set_sample_rate(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t size)
{
	char *after;
	unsigned long pm = simple_strtoul(buffer, &after, 10);
	size_t count = after - buffer;

	if (*after && isspace(*after))
		count++;

	if ((pm >= SAMPLE_RATE_MIN) && (pm <= SAMPLE_RATE_MAX))
	{
		struct input_dev *idev = to_input_dev(dev);
		struct wacdsu *wacdsu = (struct wacdsu *)idev->private;

		DBG_CMN("[%s] sample_rate = %ld\n", __FUNCTION__, pm);

		if (pm == wacdsu_sampling_rate)
			return count;

		/* Save new rate */
		wacdsu_sampling_rate = pm;

		if ((wacdsu->op_mode == OP_MODE_SLEEP) ||
			(wacdsu->op_mode == OP_MODE_SURVEY) ||
			(wacdsu->op_mode == OP_MODE_QUERY_THEN_SLEEP) ||
			(wacdsu->op_mode == OP_MODE_QUERY_THEN_SURVEY) ||
			(wacdsu->op_mode == OP_MODE_QUERY_THEN_SAMPLE))
		{
			/* Nothing more to do */
		}
		else if (wacdsu->op_mode == OP_MODE_SAMPLING)
		{
			/* Stop sampling */
			serio_write(wacdsu->serio, WACDSU_STOP_SAMPLING);

			/* Cleanup counter for next round */
			wacdsu->count = 0;

			/* Start sampling */
			wacdsu_start_sampling(wacdsu->serio, wacdsu_sampling_rate);
			wacdsu->op_mode = OP_MODE_SAMPLING;
		}

		return count;
	}
	else
	{
		DBG_CRIT("[%s] sample_rate (%ld) invalid\n", __FUNCTION__, pm);
		return -EINVAL;
	}
}
//---------------------------------------------------------------
DEVICE_ATTR(sample_rate, (S_IRUGO | S_IWUGO),
	wacdsu_get_sample_rate, wacdsu_set_sample_rate);
//---------------------------------------------------------------

static ssize_t 
wacdsu_show_dbg_lvl(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%u\n", wac_dbg_lvl);
	return strlen(buf) + 1;
}

static ssize_t 
wacdsu_set_dbg_lvl(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	ssize_t ret = -EINVAL;

	if (*after && isspace(*after))
		count++;

	if (count == size)
	{
		if (state <= DBG_LVL_INFO)
			wac_dbg_lvl = state;
		return count;
	}

	return ret;
}
//---------------------------------------------------------------
DEVICE_ATTR(dbg_lvl, (S_IRUGO | S_IWUGO),
		wacdsu_show_dbg_lvl, wacdsu_set_dbg_lvl);
//---------------------------------------------------------------

static ssize_t 
wacdsu_show_swapxy(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *idev = to_input_dev(dev);
	struct wacdsu *wacdsu = (struct wacdsu *)idev->private;

	sprintf(buf, "%u\n", wacdsu->swap_xy);
	return strlen(buf) + 1;
}

static ssize_t 
wacdsu_set_swapxy(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct input_dev *idev = to_input_dev(dev);
	struct wacdsu *wacdsu = (struct wacdsu *)idev->private;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	ssize_t ret = -EINVAL;

	if (*after && isspace(*after))
		count++;

	if (count == size)
	{
		if (state <= 1)
			wacdsu->swap_xy = state;
		return count;
	}

	return ret;
}
//---------------------------------------------------------------
DEVICE_ATTR(swapxy, (S_IRUGO | S_IWUGO),
		wacdsu_show_swapxy, wacdsu_set_swapxy);
//---------------------------------------------------------------

static ssize_t 
wacdsu_synthesize(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct input_dev *idev = to_input_dev(dev);
	struct wacdsu *wacdsu = (struct wacdsu *)idev->private;
	int touch, pressure, im, i;
	unsigned int x, y;
	ssize_t ret = -EINVAL;

	i = sscanf(buf, "%d %d %u %u %d", &touch, &pressure, &x, &y, &im);

	if (i < 5)
	{
		DBG_CRIT("[%s] Invalid arg count (%d). Must have 5 params.\n",
			__FUNCTION__, i);
		return ret;
	}

	DBG_CMN("[%s] Touch (%d), Pressure (%d), X (%d), Y (%d), IM (%d)\n",
		__FUNCTION__, touch, pressure, x, y, im);

	/* Synthesize the event */
	wacdsu_process_synthesized_data(wacdsu, touch, pressure, x, y, im);
	
	return size;
}
//---------------------------------------------------------------
DEVICE_ATTR(synthesize, S_IWUGO, NULL, wacdsu_synthesize);
//---------------------------------------------------------------

static struct attribute *wacdsu_attrs[] =
{
	&dev_attr_calibration.attr,
	&dev_attr_pixel_mode.attr,
	&dev_attr_pen_mode.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_sample_rate.attr,
	&dev_attr_dbg_lvl.attr,
	&dev_attr_swapxy.attr,
	&dev_attr_synthesize.attr,
	NULL
};

static struct attribute_group wacdsu_attr_group =
{
	.attrs = wacdsu_attrs,
};

/*
 * wacdsu_process_coordinate_data() analyzes the incoming bytestream and
 * handles the Coordinate Data Report from the Wacom DSU.
 */

#undef JOMAMA_TEST_SLOWER_SAMPLES
#ifdef JOMAMA_TEST_SLOWER_SAMPLES
unsigned long wacdsu_samples_count = 0;
#endif

static void wacdsu_process_synthesized_data(struct wacdsu *wacdsu,
	int touch, int pressure, unsigned int cookedX, unsigned int cookedY,
	int inter_module)
{
	struct input_dev *dev = wacdsu->dev;

	DBG_INFO("[%s] touch (%d), pressure (%d), X (%d), Y (%d)\n",
		__FUNCTION__, touch, pressure, cookedX, cookedY);

	/* Filter for minimum pressure */
	if (touch && (pressure < 10))
		touch = 0;

	if ((wacdsu->pen_mode != PEN_MODE_TOUCH) || touch)
	{
		wacdsu->last_x = cookedX;
		wacdsu->last_y = cookedY;

		input_report_abs(dev, ABS_X, cookedX);
		input_report_abs(dev, ABS_Y, cookedY);
		input_report_abs(dev, ABS_PRESSURE, pressure);
		if (touch && inter_module)
			wacdsu_knock_on_sidedoor(touch, cookedX, cookedY);
	}

#ifdef ESI_EMULATE_MOUSE
	if (touch != wacdsu->touch)
	{
		wacdsu->touch = touch;
		input_report_key(dev, BTN_LEFT, touch);
		if (!touch)
		{
			wacdsu->last_x = 0;		/* reset last values */
			wacdsu->last_y = 0;
			if (inter_module)
				wacdsu_knock_on_sidedoor(touch, 0, 0);
		}
	}
#else
	if (touch != wacdsu->touch)
	{
		wacdsu->touch = touch;
		input_report_key(dev, BTN_TOUCH, touch);
	}
#endif

	input_sync(dev);
}

static void wacdsu_process_coordinate_data(struct wacdsu *wacdsu,
	unsigned char data)
{
	struct input_dev *dev = wacdsu->dev;
	unsigned char *buf = wacdsu->buf;
	int touch, pressure, proximity, stylus, eraser;
	unsigned int rawX, rawY, cookedX, cookedY;

	DBG_INFO("[%s] count = %d\n", __FUNCTION__, wacdsu->count);

	switch (wacdsu->count) {

		case 0:
			/* Make sure this is the sync byte for coordinate data */
			if ((data & 0xc0) != 0x80)
				return;

#ifdef JOMAMA_TEST_SLOWER_SAMPLES
			if (++wacdsu_samples_count % 5)
				return;
#endif

			buf[wacdsu->count++] = data;
			break;

		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			buf[wacdsu->count++] = data;
			break;

		case 8:
			buf[wacdsu->count] = data;
			wacdsu->count = 0;

			proximity = buf[0] & 0x20;

#ifndef ESI_EMULATE_MOUSE
#ifdef ESI_REPORT_PROXIMITY
			if (proximity != wacdsu->proximity)
			{
				wacdsu->proximity = proximity;
				input_report_key(dev, BTN_TOOL_PEN, proximity);
			}
#endif
#endif

			if (proximity)
			{
				pressure = (buf[6] & 0x7);
				pressure = (pressure << 7) | buf[5];

				DBG_INFO("[%s] pressure = %d\n", __FUNCTION__, pressure);

				touch = buf[0] & 0x01;
				stylus = buf[0] & 0x02;
				eraser = buf[0] & 0x04;

				/* Do not accept touches from the eraser */
				if (touch && eraser)
					touch = 0;

				/* Filter for minimum pressure */
				if (touch && (pressure < 10))
					touch = 0;

				if (touch | stylus | eraser)
				{
					/* make this UNUSUAL because I want it to come out */
					/* when nothing else does (even though it is a CMN) */
					DBG_UNU("[%s] touch = %d, stylus = %d, eraser = %d\n",
						__FUNCTION__, touch, stylus, eraser);
					DBG_INFO(" cd = %x %x %x %x %x %x %x %x %x\n",
						buf[0], buf[1], buf[2], buf[3], buf[4],
						buf[5], buf[6], buf[7], buf[8]);
				}

				rawX = buf[1];
				rawX = (rawX << 7) | buf[2];
				rawX = (rawX << 2) | ((buf[6] >> 5) & 0x3);

				rawY = buf[3];
				rawY = (rawY << 7) | buf[4];
				rawY = (rawY << 2) | ((buf[6] >> 3) & 0x3);

				if ((wacdsu->pixel_mode == PIX_MODE_RAW) ||
					(wacdsu->pixel_mode == PIX_MODE_RAW_DEBUG))
				{
					if (wacdsu->pixel_mode == PIX_MODE_RAW_DEBUG)
						DBG_UNU("... rawX (%d) rawY (%d)\n", rawX, rawY);

					cookedX = rawX;
					cookedY = rawY;
				}
				else
				{
					if ((rawX < MIN_RAW_X_PIXELS) || (rawX > MAX_RAW_X_PIXELS))
					{
						/* invalid data, throw this packet away */
						DBG_UNU("[%s] *** rawX (%d) OOB (%d:%d)\n",
							__FUNCTION__, rawX, MIN_RAW_X_PIXELS,
							MAX_RAW_X_PIXELS);
						break;
					}

					if ((rawY < MIN_RAW_Y_PIXELS) || (rawY > MAX_RAW_Y_PIXELS))
					{
						/* invalid data, throw this packet away */
						DBG_UNU("[%s] *** rawY (%d) OOB (%d:%d)\n",
							__FUNCTION__, rawY, MIN_RAW_Y_PIXELS,
							MAX_RAW_Y_PIXELS);
						break;
					}

					if (wacdsu->pixel_mode == PIX_MODE_MANUAL_COOKED)
					{
#ifdef CONFIG_ESI_EDGE_JR
						cookedX = (rawY - MIN_RAW_Y_PIXELS) / PTS_PER_X_PIXEL;
						cookedY = (MAX_RAW_X_PIXELS - rawX) / PTS_PER_Y_PIXEL;
#else
						cookedX = (MAX_RAW_Y_PIXELS - rawY) / PTS_PER_X_PIXEL;
						cookedY = (rawX - MIN_RAW_X_PIXELS) / PTS_PER_Y_PIXEL;
#endif
					}
					else
					{
						xy.x = rawX;
						xy.y = rawY;

						wacdsu_linearize(wacdsu, &xy);

						switch (wacdsu_get_rotation_skew())
						{
							case 180:
							case -180:
								cookedX = (MAX_COOKED_X_PIXELS - xy.x);
								cookedY = (MAX_COOKED_Y_PIXELS - xy.y);
								break;
							case 0:
							default:
								cookedX = xy.x;
								cookedY = xy.y;
								break;
						}

						if (cookedX < MIN_COOKED_X_PIXELS)
						{
							/* invalid data, throw this packet away */
							DBG_UNU("[%s] *** Bogus MIN_X Data Sample.. Tossing\n",
								__FUNCTION__);
							break;
						}
						else if (cookedX > MAX_COOKED_X_PIXELS)
						{
							/* invalid data, throw this packet away */
							DBG_UNU("[%s] *** Bogus MAX_X Data Sample.. Tossing\n",
								__FUNCTION__);
							break;
						}

						if (cookedY < MIN_COOKED_Y_PIXELS)
						{
							/* invalid data, throw this packet away */
							DBG_UNU("[%s] *** Bogus MIN_Y Data Sample.. Tossing\n",
								__FUNCTION__);
							break;
						}
						else if (cookedX > MAX_COOKED_Y_PIXELS)
						{
							/* invalid data, throw this packet away */
							DBG_UNU("[%s] *** Bogus MAX_Y Data Sample.. Tossing\n",
								__FUNCTION__);
							break;
						}
					}
				}

				if (touch | stylus | eraser)
				{
					DBG_CMN("[%s] rX (%d), rY (%d), cX (%d), cY (%d)\n",
						__FUNCTION__, rawX, rawY, cookedX, cookedY);
				}

#undef TEST_PROXIMITY
#ifdef TEST_PROXIMITY
				if ((wacdsu->pen_mode != PEN_MODE_TOUCH) || (cookedY < 200) || touch)
#else

#undef SUPPORT_MENU_UP_PROXIMITY
#ifdef SUPPORT_MENU_UP_PROXIMITY
				if ((wacdsu->pen_mode != PEN_MODE_TOUCH) ||
#ifdef ESI_INTER_MODULE
					((p_epd_update_mode) &&
					(*p_epd_update_mode & UPDT_MODE_MENU_UP)) ||
#endif
					touch)
#else
				if ((wacdsu->pen_mode != PEN_MODE_TOUCH) || touch)
#endif

#endif
				{
#if 0
					/* Are we in survey mode? */
					if (wacdsu->op_mode == OP_MODE_SURVEY)
						wacdsu_enter_sample_state(wacdsu, SAMPLE_RATE_80_PS);
#endif
					/* Only report X,Y if they have changed */
#define MANUAL_DEFUZZ
#ifdef MANUAL_DEFUZZ
					if ((abs(cookedX - wacdsu->last_x) > 2) ||
					    (abs(cookedY - wacdsu->last_y) > 2))
#else
					if ((cookedX != wacdsu->last_x) ||
						(cookedY != wacdsu->last_y))
#endif
					{
						wacdsu->last_x = cookedX;
						wacdsu->last_y = cookedY;

						input_report_abs(dev, ABS_X, cookedX);
						input_report_abs(dev, ABS_Y, cookedY);
						input_report_abs(dev, ABS_PRESSURE, pressure);
#ifdef ESI_INTER_MODULE
						if (touch)
							wacdsu_knock_on_sidedoor(touch, cookedX, cookedY);
#endif
					}
				}

#ifdef ESI_EMULATE_MOUSE
				if (touch != wacdsu->touch)
				{
					wacdsu->touch = touch;
					input_report_key(dev, BTN_LEFT, touch);
					if (!touch)
					{
						wacdsu->last_x = 0;		/* reset last values */
						wacdsu->last_y = 0;
#ifdef ESI_INTER_MODULE
						wacdsu_knock_on_sidedoor(touch, 0, 0);
#endif
					}
				}
				if (stylus != wacdsu->stylus)
				{
					wacdsu->stylus = stylus;
//  Do NOT report stylus button events
//					input_report_key(dev, BTN_RIGHT, stylus);
				}
				if (eraser != wacdsu->eraser)
				{
					wacdsu->eraser = eraser;
//  Do NOT report eraser button events
//					input_report_key(dev, BTN_MIDDLE, eraser);
				}
#else
				if (touch != wacdsu->touch)
				{
					wacdsu->touch = touch;
					input_report_key(dev, BTN_TOUCH, touch);
				}
				if (stylus != wacdsu->stylus)
				{
					wacdsu->stylus = stylus;
//  Do NOT report stylus button events
//					input_report_key(dev, BTN_STYLUS, stylus);
				}
				if (eraser != wacdsu->eraser)
				{
					wacdsu->eraser = eraser;
//  Do NOT report eraser button events
//					input_report_key(dev, BTN_TOOL_RUBBER, eraser);
				}
#endif
			}

			input_sync(dev);
			break;
	}
}

/*
 * wacdsu_process_query_data() analyzes the incoming bytestream and
 * handles the Query Data Report from the Wacom DSU.
 */

static void wacdsu_process_query_data(struct wacdsu *wacdsu,
	unsigned char data)
{
	unsigned char *buf = wacdsu->buf;
	int maxX, maxY, maxPen;

	switch (wacdsu->count) {

		case 0:
			/* Make sure this is the sync byte for coordinate data */
			if ((data & 0xc0) != 0xc0)
				return;

			buf[wacdsu->count++] = data;
			break;

		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
			buf[wacdsu->count++] = data;
			break;

		case 10:
			buf[wacdsu->count] = data;
			wacdsu->count = 0;

#ifndef JOMAMA_JOMAMA_PRINT_QUERY
			DBG_CRIT("  Wacom DSU Query Results\n");
			DBG_CRIT("  -----------------------\n");
			DBG_CRIT("  Data ID          = %d\n", (buf[0] & 0x3f));

			maxX = buf[1];
			maxX = (maxX << 7) | buf[2];
			maxX = (maxX << 2) | ((buf[6] >> 5) & 0x3);
			DBG_CRIT("  Max X-axis Coord = %d\n", maxX);

			maxY = buf[3];
			maxY = (maxY << 7) | buf[4];
			maxY = (maxY << 2) | ((buf[6] >> 3) & 0x3);
			DBG_CRIT("  Max Y-axis Coord = %d\n", maxY);

			maxPen = buf[6] & 0x7;
			maxPen = (maxPen << 7) | buf[5];
			DBG_CRIT("  Max Pen Pressure = %d\n", maxPen);

			DBG_CRIT("  Max X-axis Tilt  = %d\n", buf[7]);
			DBG_CRIT("  Max Y-axis Tilt  = %d\n", buf[8]);
			DBG_CRIT("  F/W Version High = %d\n", buf[9]);
			DBG_CRIT("  F/W Version Low  = %d\n", buf[10]);
#endif

			/* Now update state */
			if (wacdsu->op_mode == OP_MODE_QUERY_THEN_SAMPLE)
			{
				wacdsu->op_mode = OP_MODE_SAMPLING;
			}
			else if (wacdsu->op_mode == OP_MODE_QUERY_THEN_SLEEP)
			{
				/* Stop sampling */
				serio_write(wacdsu->serio, WACDSU_STOP_SAMPLING);

				/* Sleep */
				wacdsu_set_sleep_mode();

				wacdsu->op_mode = OP_MODE_SLEEP;
			}
			else /* assuming OP_MODE_QUERY_THEN_SURVEY */
			{
				wacdsu->op_mode = OP_MODE_SURVEY;
			}
			break;
	}
}

/*
 * wacdsu_interrupt() handles incoming characters, either gathering them into
 * packets or passing them to the command routine as command output.
 */

static irqreturn_t wacdsu_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
	struct wacdsu *wacdsu = serio_get_drvdata(serio);
#ifdef MEASURE_INTERRUPT
	extern long omap_32k_read(void);

	DBG_UNU("[%s] entered %ld\n", __FUNCTION__, omap_32k_read());
#endif

	/* Do not process anything if SLEEPING */
	if (wacdsu->op_mode == OP_MODE_SLEEP)
		return IRQ_HANDLED;

	if (time_after(jiffies, wacdsu->last + HZ/10))
		wacdsu->count = 0;

	wacdsu->last = jiffies;

	if (wacdsu->op_mode == OP_MODE_SURVEY)
	{
		wacdsu_enter_sample_state(wacdsu, SAMPLE_RATE_133_PS);
		wacdsu_process_coordinate_data(wacdsu, data);
	}
	else if (wacdsu->op_mode == OP_MODE_SAMPLING)
	{
		wacdsu_process_coordinate_data(wacdsu, data);
	}
	else if ((wacdsu->op_mode == OP_MODE_QUERY_THEN_SLEEP) ||
			 (wacdsu->op_mode == OP_MODE_QUERY_THEN_SURVEY) ||
			 (wacdsu->op_mode == OP_MODE_QUERY_THEN_SAMPLE))
	{
		wacdsu_process_query_data(wacdsu, data);
	}
	else
	{
		DBG_UNU("[%s] bogus OP_MODE (%d)\n", __FUNCTION__, wacdsu->op_mode);
	}

	return IRQ_HANDLED;
}

/*
 * wacdsu_clear_sleep_mode(wacdsu) 
 */

static void wacdsu_clear_sleep_mode(struct wacdsu *wacdsu)
{
#ifdef ESI_USE_GENERIC_GPIO_CALLS
    gpio_set_value(GPIO_WACDSU_SLEEP, 0);
#else
    omap_set_gpio_dataout(GPIO_WACDSU_SLEEP, 0);
#endif

//   	interruptible_sleep_on_timeout(&wacdsu->waitq, 1); // msleep 1 jiffy
}

/*
 * wacdsu_set_sleep_mode() 
 */

static void wacdsu_set_sleep_mode(void)
{
#ifdef ESI_USE_GENERIC_GPIO_CALLS
    gpio_set_value(GPIO_WACDSU_SLEEP, 1);
#else
    omap_set_gpio_dataout(GPIO_WACDSU_SLEEP, 1);
#endif
}

/*
 * wacdsu_pen_holstered() returns whether or not the pen is holstered.
 */

static int wacdsu_pen_holstered(struct wacdsu *wacdsu)
{
    wacdsu->pen_holstered = 0;
	return wacdsu->pen_holstered;
}

/*
 * wacdsu_enter_holstered_state()
 */

static void wacdsu_enter_holstered_state(struct wacdsu *wacdsu)
{
	DBG_UNU("[%s] Entering HOLSTERED Mode\n", __FUNCTION__);

	/* Cleanup counter for next round */
	wacdsu->count = 0;

	/* Stop sampling and disable scan */
	serio_write(wacdsu->serio, WACDSU_STOP_SAMPLING);
	serio_write(wacdsu->serio, WACDSU_SURVEY_SCAN_DISABLE);

	wacdsu_set_sleep_mode();

	wacdsu->op_mode = OP_MODE_SLEEP;
	wacdsu->pm_state = PM_STATE_HOLSTERED;
}

/*
 * wacdsu_enter_survey_state()
 */

static void wacdsu_enter_survey_state(struct wacdsu *wacdsu)
{
	DBG_UNU("[%s] Entering SURVEY Mode\n", __FUNCTION__);

	/* Cleanup counter for next round */
	wacdsu->count = 0;

	wacdsu_clear_sleep_mode(wacdsu);

	/* Start Survey (kicks in while in low sampling rate) */
	wacdsu_start_sampling(wacdsu->serio, SAMPLE_RATE_40_PS);
	serio_write(wacdsu->serio, WACDSU_SURVEY_SCAN_ENABLE);

	wacdsu->op_mode = OP_MODE_SURVEY;
	wacdsu->pm_state = PM_STATE_INACTIVE;
}

static void wacdsu_enter_sample_state(struct wacdsu *wacdsu,
	unsigned char sampling_rate)
{
	DBG_UNU("[%s] Entering SAMPLING Mode at %d pps\n",
		__FUNCTION__, sampling_rate);

	/* Cleanup counter for next round */
	wacdsu->count = 0;

	wacdsu_clear_sleep_mode(wacdsu);
	serio_write(wacdsu->serio, WACDSU_SURVEY_SCAN_DISABLE);

	/* Start Survey (kicks in while in low sampling rate) */
	wacdsu_start_sampling(wacdsu->serio, sampling_rate);

	wacdsu->op_mode = OP_MODE_SAMPLING;
	wacdsu->pm_state = PM_STATE_ACTIVE;
}

/*
 * wacdsu_timer() runs the state machine
 */

static void wacdsu_timer(unsigned long data)
{
	struct wacdsu *wacdsu = (struct wacdsu *)data;

	if (wacdsu->pm_mode != PM_MODE_AUTO)
	{
		mod_timer(&wacdsu->timer, jiffies + HZ);	// re-arm for 1 second
		return;
	}

	DBG_INFO("[%s] op_mode = %d\n", __FUNCTION__, wacdsu->op_mode);

	switch (wacdsu->op_mode)
	{
	case OP_MODE_SLEEP:
		/* Only event we take action on is whipping your pen out! */
		if (!wacdsu_pen_holstered(wacdsu))
			wacdsu_enter_survey_state(wacdsu);
		break;

	case OP_MODE_SURVEY:
		/* Did we holster the pen? */
		if (wacdsu_pen_holstered(wacdsu))
			wacdsu_enter_holstered_state(wacdsu);
		break;

	case OP_MODE_SAMPLING:
		/* Did we holster the pen? */
		if (wacdsu_pen_holstered(wacdsu))
			wacdsu_enter_holstered_state(wacdsu);
		break;

	case OP_MODE_QUERY_THEN_SLEEP:
	case OP_MODE_QUERY_THEN_SURVEY:
	case OP_MODE_QUERY_THEN_SAMPLE:
		DBG_INFO("[%s] No state change action until Query is complete\n",
			__FUNCTION__);
		break;

	default:
		break;
	}

	mod_timer(&wacdsu->timer, jiffies + HZ);	// re-arm for 1 second
}

static int wacdsu_get_rotation_skew(void)
{
	if (!p_epd_rotate_mode)
	{
		DBG_CMN("[%s] **** TRYING TO RESOLVE symbol: epd_rotate_mode\n",
			__FUNCTION__);
		p_epd_rotate_mode = __symbol_get("epd_rotate_mode");

		if (!p_epd_rotate_mode)
		{
			DBG_CRIT("[%s] **** COULD NOT RESOLVE SYMBOL: epd_rotate_mode\n",
				__FUNCTION__);
			return 0;
		}
	}

	/* For now, we only support rotating between 90 and 270 */
	return (cal.rotation - *p_epd_rotate_mode);
}

#ifdef ESI_INTER_MODULE
static void wacdsu_knock_on_sidedoor(unsigned int touch,
							unsigned int x, unsigned int y)
{
	if (!p_epd_update_mode)
	{
		DBG_CMN("[%s] **** TRYING TO RESOLVE symbol: epd_update_mode\n",
			__FUNCTION__);
		p_epd_update_mode = __symbol_get("epd_update_mode");

		if (!p_epd_update_mode)
		{
			DBG_CRIT("[%s] **** COULD NOT RESOLVE SYMBOL: epd_update_mode\n",
				__FUNCTION__);
			return;
		}
	}

	/* If we are not allowed in, go home */
	if (!(*p_epd_update_mode & UPDT_MODE_BOTTOM))
		return;

	if (!p_epd_sidedoor)
	{
		DBG_CMN("[%s] **** TRYING TO RESOLVE symbol: epd_sidedoor\n",
			__FUNCTION__);
		p_epd_sidedoor = __symbol_get("epd_sidedoor");
	}

	if (p_epd_sidedoor)
	{
		(*p_epd_sidedoor)(touch, x, y);
	}
	else
	{
		DBG_CRIT("[%s] **** COULD NOT RESOLVE SYMBOL: epd_sidedoor\n",
			__FUNCTION__);
	}
}
#endif

/*
 * wacdsu_disconnect() cleans up after we don't want talk
 * to the tablet anymore.
 */

static void wacdsu_disconnect(struct serio *serio)
{
	struct wacdsu *wacdsu = serio_get_drvdata(serio);

	DBG_INFO("[%s] entered\n", __FUNCTION__);

	/* Stop sampling */
	serio_write(serio, WACDSU_STOP_SAMPLING);

	/* Sleep */
	wacdsu_set_sleep_mode();
	wacdsu->op_mode = OP_MODE_SLEEP;

	del_timer(&wacdsu->timer);

	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	sysfs_remove_group(&wacdsu->dev->dev.kobj, &wacdsu_attr_group);
	input_unregister_device(wacdsu->dev);
	kfree(wacdsu);
}

/*
 * wacdsu_connect() is a callback form the serio module when
 * an unhandled serio port is found.
 */

static int wacdsu_connect(struct serio *serio, struct serio_driver *drv)
{
	struct wacdsu *wacdsu;
	struct input_dev *input_dev;
	int err = -ENOMEM;

	DBG_INFO("[%s] entered\n", __FUNCTION__);

	wacdsu = kzalloc(sizeof(struct wacdsu), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!wacdsu || !input_dev)
		goto fail1;

	wacdsu->dev = input_dev;
	wacdsu->serio = serio;
	snprintf(wacdsu->phys, sizeof(wacdsu->phys), "%s/input0", serio->phys);

	init_waitqueue_head(&wacdsu->waitq);

	/* If we have calibration values, set to CAL_COOKED, else RAW */
	if (cal.a[0] | cal.a[1] | cal.a[2] | cal.a[3])
		wacdsu->pixel_mode = PIX_MODE_CAL_COOKED;
	else
		wacdsu->pixel_mode = PIX_MODE_RAW;

	wacdsu->pen_mode = PEN_MODE_TOUCH;
	wacdsu->pm_mode = PM_MODE_AUTO;

	wacdsu->count = 0;
	wacdsu->last = 0;
	wacdsu->swap_xy = 0;
	wacdsu->last_x = 0;
	wacdsu->last_y = 0;

	wacdsu->proximity = -1;
	wacdsu->touch = -1;
	wacdsu->stylus = -1;
	wacdsu->eraser = -1;

	input_dev->name = "Wacom DSU";
	input_dev->private = wacdsu;
	input_dev->phys = wacdsu->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor  = SERIO_WACDSU;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &serio->dev;
#ifdef CONFIG_ESI_EDGE
	input_dev->esi_id = 1;	
	printk(KERN_CRIT "%s: esi id set to %d\n", __FUNCTION__, input_dev->esi_id);
#endif

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#ifdef ESI_EMULATE_MOUSE
	input_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) |
		BIT_MASK(BTN_MIDDLE) | BIT_MASK(BTN_RIGHT);
//	input_dev->absbit[BIT_WORD(ABS_MISC)] |= BIT_MASK(ABS_MISC);
#else
#ifdef ESI_REPORT_PROXIMITY
	input_dev->keybit[BIT_WORD(BTN_DIGI)] = BIT_MASK(BTN_TOOL_PEN) |
		BIT_MASK(BTN_TOOL_RUBBER) | BIT_MASK(BTN_TOUCH) |
		BIT_MASK(BTN_STYLUS);
#else
	input_dev->keybit[BIT_WORD(BTN_DIGI)] = BIT_MASK(BTN_TOOL_RUBBER) |
		BIT_MASK(BTN_TOUCH) | BIT_MASK(BTN_STYLUS);
#endif
	input_dev->keybit[BIT_WORD(BTN_LEFT)] = BIT_MASK(BTN_LEFT) |
		BIT_MASK(BTN_RIGHT);
//	input_dev->absbit[BIT_WORD(ABS_MISC)] |= BIT_MASK(ABS_MISC);
//	input_dev->mscbit[0] |= BIT_MASK(MSC_SERIAL);
#endif

	input_set_abs_params (input_dev, ABS_X, 
		MIN_COOKED_X_PIXELS, MAX_COOKED_X_PIXELS, 4, 0);
	input_set_abs_params (input_dev, ABS_Y,
		MIN_COOKED_Y_PIXELS, MAX_COOKED_Y_PIXELS, 4, 0);
	input_set_abs_params (input_dev, ABS_PRESSURE,
		0, MAX_PRESSURE, 0, 0);

#define ESI_ENABLE_DEFUZZ
#ifdef ESI_ENABLE_DEFUZZ
	/* Turn defuzz off for this driver */
	input_dev->absfuzz[ABS_X] = 0;
	input_dev->absfuzz[ABS_Y] = 0;
#endif

	serio_set_drvdata(serio, wacdsu);

	err = serio_open(serio, drv);
	if (err)
		goto fail2;

	err = input_register_device(wacdsu->dev);
	if (err)
		goto fail3;

	err = sysfs_create_group(&wacdsu->dev->dev.kobj, &wacdsu_attr_group);
	if (err) {
		DBG_CRIT("[%s] Cannot Create SYSFS group\n", __FUNCTION__);
		goto fail4;
	}

	/* Get initial pen holstered state */
	wacdsu_pen_holstered(wacdsu);

	if (wacdsu->pen_holstered)
	{
		/* Go to sleep */
		wacdsu_set_sleep_mode();

		wacdsu->pm_state = PM_STATE_HOLSTERED;
	}
	else
	{
		/* Make sure we are not sleeping */
		wacdsu_clear_sleep_mode(wacdsu);

#ifdef ESI_ENABLE_PWR_MGMT
		wacdsu->pm_state = PM_STATE_INACTIVE;

		/* Start Survey (kicks in while in low sampling rate) */
		wacdsu_start_sampling(wacdsu->serio, SAMPLE_RATE_40_PS);
		serio_write(wacdsu->serio, WACDSU_SURVEY_SCAN_ENABLE);

		wacdsu->op_mode = OP_MODE_SURVEY;
#else
		wacdsu->pm_state = PM_STATE_ACTIVE;

		/* Start Survey (kicks in while in low sampling rate) */
		wacdsu_start_sampling(wacdsu->serio, SAMPLE_RATE_133_PS);

		wacdsu->op_mode = OP_MODE_SAMPLING;
#endif
	}

	/* Init the state machine timer */
	init_timer(&wacdsu->timer);
	wacdsu->timer.data = (long) wacdsu;
	wacdsu->timer.function = wacdsu_timer;
	mod_timer(&wacdsu->timer, jiffies + HZ);	// 1 second

	return 0;

 fail4:	input_unregister_device(wacdsu->dev);
 fail3:	serio_close(serio);
 fail2:	serio_set_drvdata(serio, NULL);
 fail1:	input_free_device(input_dev);
	kfree(wacdsu);
	return err;
}

static int wacdsu_reconnect(struct serio *serio)
{

	return 0;
}

#ifdef ESI_AM300
/* Sets the I/O direction of the specified pin to the specified value */
static void wacdsu_set_gpio_dir(unsigned int pin, unsigned int value)
{
	if (value & 0x1)
		GPDR(pin) |= (1 << (pin & 0x1f));
	else
		GPDR(pin) &= ~(1 << (pin & 0x1f));
}

/* Sets the pin mode of the specified pin to 'GPIO mode' */
static void wacdsu_set_gpio_mode(unsigned int pin, unsigned int alt_func)
{
	GAFR(pin) &= ~(3 << ((pin & 0xf) << 1));
	GAFR(pin) |= (alt_func & 3) << ((pin & 0xf) << 1);
}
#endif

static struct serio_device_id wacdsu_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_WACDSU,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, wacdsu_serio_ids);

static struct serio_driver wacdsu_drv = {
	.driver		= {
		.name	= "wacdsu",
	},
	.description	= DRIVER_DESC,
	.id_table	= wacdsu_serio_ids,
	.interrupt	= wacdsu_interrupt,
	.connect	= wacdsu_connect,
	.disconnect	= wacdsu_disconnect,
	.reconnect	= wacdsu_reconnect,
};

static int __init wacdsu_init(void)
{
	DBG_INFO("[%s] entered\n", __FUNCTION__);

#ifdef ESI_AM300
	wacdsu_set_gpio_dir(GPIO_WACDSU_46, 0);
	wacdsu_set_gpio_dir(GPIO_WACDSU_47, 1);

	wacdsu_set_gpio_mode(GPIO_WACDSU_46, GAFR_AF2);
	wacdsu_set_gpio_mode(GPIO_WACDSU_47, GAFR_AF1);
#else
#ifdef CONFIG_MACH_OMAP3_EDGE
#ifdef RESET_DIGITIZER_AT_STARTUP
	omap_request_gpio(GPIO_WACDSU_RESET);
#endif
	omap_request_gpio(GPIO_WACDSU_SLEEP);

	/* RESET and SLEEP GPIOs are outputs. */
#ifdef RESET_DIGITIZER_AT_STARTUP
	omap_set_gpio_direction(GPIO_WACDSU_RESET, 0);
#endif
	omap_set_gpio_direction(GPIO_WACDSU_SLEEP, 0);

#ifdef RESET_DIGITIZER_AT_STARTUP
	/* Pull high to remove from reset */
	omap_set_gpio_dataout(GPIO_WACDSU_RESET, 0);
	mdelay(50);
	omap_set_gpio_dataout(GPIO_WACDSU_RESET, 1);
#endif
#else
#ifdef RESET_DIGITIZER_AT_STARTUP
	gpio_request(GPIO_WACDSU_RESET, "RESET");
#endif
	gpio_request(GPIO_WACDSU_SLEEP, "SLEEP");

	/* RESET and SLEEP GPIOs are outputs. */
#ifdef RESET_DIGITIZER_AT_STARTUP
	gpio_direction_output(GPIO_WACDSU_RESET, 0);
#endif
	gpio_direction_output(GPIO_WACDSU_SLEEP, 0);

#ifdef RESET_DIGITIZER_AT_STARTUP
	/* Pull high to remove from reset */
	gpio_set_value(GPIO_WACDSU_RESET, 0);
	mdelay(50);
	gpio_set_value(GPIO_WACDSU_RESET, 1);
#endif
#endif
#endif

	/* Global that must survive between connects/disconnects */
	wacdsu_sampling_rate = SAMPLE_RATE_80_PS;

	wacdsu_init_calibration_values();

	return serio_register_driver(&wacdsu_drv);
}

static void __exit wacdsu_exit(void)
{
	DBG_INFO("[%s] entered\n", __FUNCTION__);

	serio_unregister_driver(&wacdsu_drv);
}

module_init(wacdsu_init);
module_exit(wacdsu_exit);

