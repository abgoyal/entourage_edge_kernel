/*
 *  linux/drivers/input/touchscreen/tsc2007.c
 *
 *  touch screen driver for tsc2007
 *
 *  Copyright (C) 2006, Marvell Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <mach/gpio.h>

#include <linux/sysctl.h>
#include <asm/system.h>

extern int ts_linear_scale(int *x, int *y, int swap_xy);

/* Use MAV filter */
#define TSC_CMD_SETUP 0xb0

/* Use 12-bit */
#ifndef CONFIG_ESI_EDGE_JR
#define TSC_CMD_X 0xc0
#define TSC_CMD_PLATEX 0x80
#define TSC_CMD_Y 0xd0
#define TSC_CMD_PLATEY 0x90
#else
/* x and y are reversed on Jr. */
#define TSC_CMD_X 0xd0
#define TSC_CMD_PLATEX 0x90
#define TSC_CMD_Y 0xc0
#define TSC_CMD_PLATEY 0x80
#endif

#if 0
#define TSC_X_MAX 4096
#define TSC_Y_MAX 4096
#define TSC_X_MIN 0
#define TSC_Y_MIN 0
#else
#define TSC_X_FULL 4095
#define TSC_Y_FULL 4095
#define TSC_X_MAX 4031
#define TSC_Y_MAX 4031
#define TSC_X_MIN 31
#define TSC_Y_MIN 191
#endif

/* delay time for compute x, y, computed as us */

#undef DEBUG
#ifdef DEBUG
#define TS_DEBUG(fmt,args...) printk(KERN_DEBUG fmt, ##args )
#else
#define TS_DEBUG(fmt,args...)
#endif
static int x_min=TSC_X_MIN;
static int y_min=TSC_Y_MIN;
static int x_max=TSC_X_MAX;
static int y_max=TSC_Y_MAX;
static int invert = 0;
static int debounce_time  = 150;
static int init_debounce = true;
static int delay_time = 1;

enum tsc2007_status {
	PEN_UP,
	PEN_DOWN,
};

struct _tsc2007 {
	struct input_dev *dev;
	int x;		/* X sample values */
	int y;		/* Y sample values */

	int status;
	struct work_struct irq_work;
	struct i2c_client *client;
	unsigned long last_touch;
#ifdef CONFIG_ESI_EDGE
    spinlock_t lock;
    unsigned long flags;
    int lockstate;
    int gpio;       /* of irq */
    struct hrtimer  timer;
#define TS_INITIAL_DELAY    (2*1000*1000)
#define TS_SETTLE_DELAY     (20*1000*1000)
#define TS_POLL_DELAY       (10*1000*1000)
#endif
};
struct _tsc2007 *g_tsc2007;

/* update abs params when min and max coordinate values are set */
int tsc2007_proc_minmax(struct ctl_table *table, int write, struct file *filp,
                     void __user *buffer, size_t *lenp, loff_t *ppos)
{
	struct _tsc2007 *tsc2007= g_tsc2007;
	struct input_dev *input = tsc2007->dev;

	/* update value */
	int ret = proc_dointvec(table, write, filp, buffer, lenp, ppos);

	/* updated abs params */
	if (input) {
		TS_DEBUG(KERN_DEBUG "update x_min %d x_max %d"
			" y_min %d y_max %d\n", x_min, x_max,
			y_min, y_max); 
		input_set_abs_params(input, ABS_X, x_min, x_max, 0, 0);
		input_set_abs_params(input, ABS_Y, y_min, y_max, 0, 0);
	}
	return ret;
}

static ctl_table tsc2007_proc_table[] = {
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "x-max",
		.data		= &x_max,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &tsc2007_proc_minmax,
	},
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "y-max",
		.data		= &y_max,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &tsc2007_proc_minmax,
	},
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "x-min",
		.data		= &x_min,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &tsc2007_proc_minmax,
	},
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "y-min",
		.data		= &y_min,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &tsc2007_proc_minmax,
	},
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "invert_xy",
		.data		= &invert,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "debounce_time",
		.data		= &debounce_time,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &proc_dointvec,
	},
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "delay_time",
		.data		= &delay_time,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &proc_dointvec,
	},
	{ .ctl_name = 0 }
};

static ctl_table tsc2007_proc_root[] = {
	{
		.ctl_name	= CTL_UNNUMBERED,
		.procname	= "ts_device",
		.mode		= 0555,
		.child		= tsc2007_proc_table,
	},
	{ .ctl_name = 0 }
};

static ctl_table tsc2007_proc_dev_root[] = {
	{
		.ctl_name	= CTL_DEV,
		.procname	= "dev",
		.mode		= 0555,
		.child		= tsc2007_proc_root,
	},
	{ .ctl_name = 0 }
};

static struct ctl_table_header *sysctl_header;

static int __init init_sysctl(void)
{
    sysctl_header = register_sysctl_table(tsc2007_proc_dev_root);
    return 0;
}

static void __exit cleanup_sysctl(void)
{
    unregister_sysctl_table(sysctl_header);
}

static int tsc2007_measure(struct i2c_client *client, int *x, int * y)
{
	u8 x_buf[2] = {0, 0};
	u8 y_buf[2] = {0, 0};

	i2c_smbus_write_byte(client, TSC_CMD_PLATEX);
#ifdef AVA
	msleep_interruptible(delay_time);
#else
	udelay(500);
#endif

	i2c_smbus_write_byte(client, TSC_CMD_X);
	i2c_master_recv(client, x_buf, 2);
#ifdef AVA
	*x = (x_buf[0]<<4) | (x_buf[1] >>4);
#else
    *x = 0xfffL - ((x_buf[0]<<4) | (x_buf[1] >>4));
#endif

	i2c_smbus_write_byte(client, TSC_CMD_PLATEY);
#ifdef AVA
	msleep_interruptible(delay_time);
#else
	udelay(500);
#endif

	i2c_smbus_write_byte(client, TSC_CMD_Y);
	i2c_master_recv(client, y_buf, 2);
#ifdef AVA
	*y = (y_buf[0]<<4) | (y_buf[1] >>4);
#else
    *y = 0xfffL - ((y_buf[0]<<4) | (y_buf[1] >>4));
#endif

	return 0;
}

#ifdef JOMAMA_DEBUG
static int drop_count = 0;
#endif

static enum hrtimer_restart tsc2007_timer(struct hrtimer *handle)
{
	struct _tsc2007 *tsc2007= g_tsc2007;
	struct i2c_client *client = tsc2007-> client;
	struct input_dev *input = tsc2007->dev;
	int x = -1, y = -1;
	int tmp_x = 0, tmp_y = 0;

    spin_lock(&tsc2007->lock);

	if (gpio_get_value(tsc2007->gpio)) {
		/*consider PEN_UP */
		tsc2007->status = PEN_UP;
		input_report_abs(input, ABS_PRESSURE, 0);
//		input_report_abs(input, ABS_TOOL_WIDTH, 1);
		input_report_key(input, BTN_TOUCH, 0);
		input_sync(input);
		tsc2007->last_touch = jiffies;
		tsc2007->x = tsc2007->y = -1;
		TS_DEBUG(KERN_DEBUG "pen up!\n"); 
#ifdef JOMAMA_DEBUG
		printk (KERN_CRIT "Drop Count = %d\n", drop_count);
		drop_count = 0;
#endif
		enable_irq(client->irq);
	} else {
		if ((jiffies_to_msecs(
			((long)jiffies - (long)tsc2007->last_touch)) < debounce_time &&
			tsc2007->status == PEN_DOWN) || init_debounce)
		{
			init_debounce = false;
			tsc2007_measure(client, &tmp_x, &tmp_y);
			TS_DEBUG(KERN_DEBUG "dropping pen touch %lu %lu (%u)\n",
				jiffies, tsc2007->last_touch,
				jiffies_to_msecs((long)jiffies - (long)tsc2007->last_touch));
#ifdef JOMAMA_DEBUG
			drop_count++;
#endif
		} else {
			tsc2007_measure(client, &x, &y);
		}

		/* continue report x, y */
		if (tsc2007->x > 0 && tsc2007->y > 0)
		{
#ifndef CONFIG_ESI_EDGE_JR
			TS_DEBUG(KERN_DEBUG "pen down x=%d y=%d!\n", tsc2007->x, tsc2007->y);
			input_report_abs(input, ABS_X, tsc2007->x);
#else
			/* x is +/- reversed on Jr */
			TS_DEBUG(KERN_DEBUG "pen down x=%d y=%d!\n",
						   	TSC_X_FULL - tsc2007->x, tsc2007->y);
			input_report_abs(input, ABS_X, TSC_X_FULL - tsc2007->x);
#endif
			input_report_abs(input, ABS_Y, tsc2007->y);
			input_report_abs(input, ABS_PRESSURE, 255);
//			input_report_abs(input, ABS_TOOL_WIDTH, 1);
			input_report_key(input, BTN_TOUCH, 1);
			input_sync(input);
		}

		tsc2007->x = x;
		tsc2007->y = y;
		tsc2007->status = PEN_DOWN;

		/* restart poll timer */
		hrtimer_start(&tsc2007->timer, ktime_set(0, TS_POLL_DELAY),
					HRTIMER_MODE_REL);
	}

    spin_unlock(&tsc2007->lock);

	return HRTIMER_NORESTART;
}

static irqreturn_t tsc2007_interrupt(int irq, void *dev_id)
{
//	struct _tsc2007 *tsc2007 = dev_id;
	struct _tsc2007 *tsc2007 = g_tsc2007;
	struct i2c_client *client = tsc2007->client;

	TS_DEBUG("pen down irq\n");
	if (tsc2007->status == PEN_UP) {
		/* pen must be down now */
		/* disable irq, will be re-enabled in timer handler */
		disable_irq(client->irq);
		//TS_DEBUG("starting timer\n");
		hrtimer_start(&tsc2007->timer, ktime_set(0, TS_INITIAL_DELAY),
					HRTIMER_MODE_REL);
	} else {
		/* Status says pen down, so pen must have come up and now back down. */
		printk(KERN_ERR "pen down but got IRQ?\n");
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_ESI_EDGE

/* 
 * User can write input triplets (t,c,v) to sysfs 
 * input node "/sys/devices/platform/gpio-keys/input"
 */
void turn_off_esi_touchscreen(void)
{
	struct _tsc2007 *tsc2007 = g_tsc2007;
	struct i2c_client *client = tsc2007-> client;

	spin_lock_irqsave(&tsc2007->lock, tsc2007->flags);
	if (tsc2007->lockstate == 0)
		disable_irq(client->irq);
	tsc2007->lockstate++;
	spin_unlock_irqrestore(&tsc2007->lock, tsc2007->flags);
}
EXPORT_SYMBOL(turn_off_esi_touchscreen);

void turn_on_esi_touchscreen(void)
{
	struct _tsc2007 *tsc2007 = g_tsc2007;
	struct i2c_client *client = tsc2007-> client;

	spin_lock_irqsave(&tsc2007->lock, tsc2007->flags);
	if (tsc2007->lockstate)
	{
		tsc2007->lockstate--;
		enable_irq(client->irq);
	}
	spin_unlock_irqrestore(&tsc2007->lock, tsc2007->flags);
}
EXPORT_SYMBOL(turn_on_esi_touchscreen);

static ssize_t
tsc2007write_state(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	int new_state;

	sscanf(buffer, "%d", &new_state);

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] entered, new_state %d\n", __FUNCTION__, new_state);
#endif

	if (new_state == 1)
		turn_on_esi_touchscreen();
	else
		turn_off_esi_touchscreen();

	return count;
}
/* Attach the sysfs write method */
DEVICE_ATTR(tsc2007state, 0666, NULL, tsc2007write_state);

/* Attribute Descriptor */
static struct attribute *tsc2007_attrs[] = {
	&dev_attr_tsc2007state.attr,
	NULL
};

/* Attribute group */
static struct attribute_group tsc2007_attr_group = {
	.attrs = tsc2007_attrs,
};
#endif

static int __devinit tsc2007_probe(struct i2c_client *client, 
				const struct i2c_device_id *id)
{
	struct _tsc2007 *tsc2007;
	struct input_dev *input_dev;
	int ret;

	tsc2007 = kzalloc(sizeof(struct _tsc2007), GFP_KERNEL);
	input_dev = input_allocate_device();

	g_tsc2007 = tsc2007;

	if (!tsc2007 || !input_dev) {
		ret = -ENOMEM;
		goto fail1;
	}

	i2c_set_clientdata(client, tsc2007);

	tsc2007->dev = input_dev;

	input_dev->name = "tsc2007";
	input_dev->phys = "tsc2007/input0";

	//input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_PRESSURE, input_dev->evbit);
	__set_bit(ABS_X, input_dev->evbit);
	__set_bit(ABS_Y, input_dev->evbit);

#ifdef CONFIG_ESI_EDGE
	input_dev->esi_id = 1;	

	tsc2007->lockstate = 0;
	tsc2007->gpio = irq_to_gpio(client->irq);
	if (gpio_request(tsc2007->gpio, "tsc2007 touch detect")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %X\n", tsc2007->gpio);
	}
	gpio_direction_input(tsc2007->gpio);	

	/* Create a sysfs node to allow touchscreen to be turned off */
	ret = sysfs_create_group(&client->dev.kobj, &tsc2007_attr_group);
#endif

	input_set_abs_params(input_dev, ABS_X, x_min, x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, y_min, y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	ret = request_irq(client->irq, tsc2007_interrupt, 
		IRQF_DISABLED | IRQF_TRIGGER_FALLING,
		 "tsc2007 irq", NULL);
	if (ret){
		printk(KERN_ERR "tsc2007 request irq failed\n");
		goto fail2;
	}

	ret = input_register_device(tsc2007->dev);
	if (ret){
		printk(KERN_ERR "tsc2007 register device fail\n");
		goto fail2;
	}

    spin_lock_init(&tsc2007->lock);

	/*init */
	tsc2007->status = PEN_UP;
	tsc2007->client = client;
	tsc2007->last_touch = jiffies;
	tsc2007->x = tsc2007->y = -1;

    hrtimer_init(&tsc2007->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    tsc2007->timer.function = tsc2007_timer;

	/* init tsc2007 */
	i2c_smbus_write_byte(client, TSC_CMD_SETUP);

	return 0;

 fail2:
	free_irq(client->irq, client);
 fail1:
	i2c_set_clientdata(client, NULL);
	input_free_device(input_dev);
	kfree(tsc2007);
	return ret;
}

static int __devexit tsc2007_remove(struct i2c_client *client)
{
	struct _tsc2007 *tsc2007 = i2c_get_clientdata(client);

#ifdef CONFIG_ESI_EDGE
	sysfs_remove_group(&client->dev.kobj, &tsc2007_attr_group);
	gpio_free(tsc2007->gpio);
#endif

	if(client->irq)
		free_irq(client->irq, client);
	
	i2c_set_clientdata(client, NULL);
	input_unregister_device(tsc2007->dev);
	kfree(tsc2007);

	return 0;
}

static struct i2c_device_id tsc2007_idtable[] = { 
	{ "tsc2007", 0 }, 
	{ } 
}; 

MODULE_DEVICE_TABLE(i2c, tsc2007_idtable);

static struct i2c_driver tsc2007_driver = {
	.driver = {
		.name 	= "tsc2007",
	},
	.id_table       = tsc2007_idtable,
	.probe		= tsc2007_probe,
	.remove		= __devexit_p(tsc2007_remove),
};

static int __init tsc2007_ts_init(void)
{
	init_sysctl();
	return i2c_add_driver(&tsc2007_driver);	 
}

static void __exit tsc2007_ts_exit(void)
{
	cleanup_sysctl();
	i2c_del_driver(&tsc2007_driver);
}

module_init(tsc2007_ts_init);
module_exit(tsc2007_ts_exit);

MODULE_DESCRIPTION("tsc2007 touch screen driver");
MODULE_LICENSE("GPL");
