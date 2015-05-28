/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <asm/gpio.h>

struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
};

struct gpio_keys_drvdata {
#ifdef CONFIG_ESI_EDGE
	int nbuttons;
#endif
	struct input_dev *input;
	struct gpio_button_data data[0];
};

#ifdef CONFIG_ESI_EDGE
// This is a pre-canned button value that will be used as the
// short-press value for the rotation/power button
struct gpio_keys_button lid_button =
{
	.code		= 1,
	.gpio		= 173,
	.desc		= "ev_sw",
	.debounce_interval	= 30,
	.active_low			= 1,
	.side 				= 0,		
	.type				= EV_SW,
};
#endif

#ifdef CONFIG_ESI_EDGE
static void gpio_keys_report_event(struct gpio_keys_button *button, unsigned int state_override, unsigned new_state);
#else
static void gpio_keys_report_event(struct gpio_keys_button *button,
				   struct input_dev *input);
#endif

#ifdef CONFIG_ESI_EDGE
struct platform_device *platform_device = NULL;

static void gpio_keys_report_input(unsigned int type, int code, int state)
{
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(platform_device);
	struct input_dev *input = ddata->input;
	
	input_event(input, type, code, !!state);
	//input_sync(input);
}

/* 
 * User can write input triplets (t,c,v) to sysfs 
 * input node "/sys/devices/platform/gpio-keys/input"
 */
static ssize_t
write_gpiokey(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	int key_type, key_code, key_value;

	sscanf(buffer, "%d%d%d", &key_type, &key_code, &key_value);

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] entered, type %d code 0x%x(%d) value %d\n", __FUNCTION__, key_type, key_code, key_code, key_value);
#endif

	/* If changing the EV_SW */
	if (key_type == 5)
	{
		if (key_code == lid_button.code)
		{
			gpio_keys_report_event(&lid_button, 1, key_value);
		}
		else
		{
			printk( KERN_CRIT "[%s] couldn't find switch button %d\n", __FUNCTION__, key_code);
		}
	}
	else if (key_type <= EV_MAX)
	{
		gpio_keys_report_input(key_type, key_code, key_value);

		/* Force a key up right after a key down to save typing */
		if (key_type == 1 && key_value == 1)
		{
			gpio_keys_report_input(key_type, key_code, 0);
		}
	}

	return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(ginput, 0666, NULL, write_gpiokey);

/* Attribute Descriptor */
static struct attribute *gpiokey_attrs[] = {
	&dev_attr_ginput.attr,
	NULL
};

/* Attribute group */
static struct attribute_group gpio_key_attr_group = {
	.attrs = gpiokey_attrs,
};
#endif

#ifdef CONFIG_ESI_EDGE
static void gpio_keys_report_event(struct gpio_keys_button *button, unsigned int override_state, unsigned int new_state)
#else
static void gpio_keys_report_event(struct gpio_keys_button *button,
				   struct input_dev *input)
#endif
{
	unsigned int type = button->type ?: EV_KEY;
#ifdef CONFIG_ESI_EDGE
	static int display_state = 1;
#endif
#ifdef CONFIG_ESI_EDGE
	int state = (override_state ? new_state : ((gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low));
#else
	int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;
#endif
#ifdef CONFIG_ESI_EDGE
	if (button->side == 0)
	{
#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] reporting(LCD), type %d code 0x%x(%d) value %d\n", __FUNCTION__, type, button->code, button->code, !!state);
#endif

		/*
		 * handle the POWER (display flip) button specially:
		 * only send the event when the button down is detected (state=1),
		 * and then send the opposite state that was sent previously.
		*/
		if (type == EV_SW && button->code == 1)
		{
			if (!!state)
			{
#ifndef FUG_DEBUG_DISABLED
				printk( KERN_CRIT "[%s] reporting type %d code 0x%x(%d) value %d\n", __FUNCTION__, type, button->code, button->code, display_state);
#endif
				gpio_keys_report_input(type, button->code, display_state);
				display_state = !display_state;
			}
			else
			{
#ifndef FUG_DEBUG_DISABLED
				printk( KERN_CRIT "[%s] IGNORING type %d code 0x%x(%d) value %d\n", __FUNCTION__, type, button->code, button->code, !!state);
#endif
			}
		}
		else
		{
			/* handle all other keys normally */
			gpio_keys_report_input(type, button->code, !!state);
		}
	}
	else	
	{
		extern void esivkbd_report_input(int type, int code, int value);

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] passing(EPD), type %d code 0x%x(%d) value %d\n", __FUNCTION__, type, button->code, button->code, !!state);
#endif

		esivkbd_report_input(type, button->code, !!state);
	}
#else
	input_event(input, type, button->code, !!state);
	input_sync(input);
#endif
}

static void gpio_check_button(unsigned long _data)
{
	struct gpio_button_data *data = (struct gpio_button_data *)_data;

#ifdef CONFIG_ESI_EDGE
	gpio_keys_report_event(data->button, 0, 0);
#else
	gpio_keys_report_event(data->button, data->input);
#endif
}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];

		if (irq == gpio_to_irq(button->gpio)) {
			struct gpio_button_data *bdata = &ddata->data[i];

			if (button->debounce_interval)
				mod_timer(&bdata->timer,
					  jiffies +
					  msecs_to_jiffies(button->debounce_interval));
			else
#ifdef CONFIG_ESI_EDGE
				gpio_keys_report_event(button, 0, 0);
#else
				gpio_keys_report_event(button, bdata->input);
#endif
			return IRQ_HANDLED;
		}
	}

	return IRQ_NONE;
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		error = -ENOMEM;
		goto fail1;
	}

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	ddata->input = input;
#ifdef CONFIG_ESI_EDGE
	ddata->nbuttons = pdata->nbuttons;
#endif

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		int irq;
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;
		setup_timer(&bdata->timer,
			    gpio_check_button, (unsigned long)bdata);

		error = gpio_request(button->gpio, button->desc ?: "gpio_keys");
		if (error < 0) {
			pr_err("gpio-keys: failed to request GPIO %d,"
				" error %d\n", button->gpio, error);
			goto fail2;
		}

		error = gpio_direction_input(button->gpio);
		if (error < 0) {
			pr_err("gpio-keys: failed to configure input"
				" direction for GPIO %d, error %d\n",
				button->gpio, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			pr_err("gpio-keys: Unable to get irq number"
				" for GPIO %d, error %d\n",
				button->gpio, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		error = request_irq(irq, gpio_keys_isr,
				    IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
				    button->desc ? button->desc : "gpio_keys",
				    pdev);
		if (error) {
			pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
				irq, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
	}

#ifdef CONFIG_ESI_EDGE
	input->esi_id = 1;	

	/* we have capability to send up a few special keys, left, right, up, down */
	input_set_capability(input, EV_KEY, KEY_UP);
	input_set_capability(input, EV_KEY, KEY_DOWN);
	input_set_capability(input, EV_KEY, KEY_LEFT);
	input_set_capability(input, EV_KEY, KEY_RIGHT);
	input_set_capability(input, EV_KEY, KEY_ENTER);

	// GPIO keys defines the longpress value for the rotation/power key
	// which is power.  So, define the shortpress operation as an option
	// The Android code will write down to ginput in order to cause
	// the shortpress (rotate) operation. Check out PhoneWindowManager.java
	input_set_capability(input, EV_SW, 1);

#endif

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, "
			"error: %d\n", error);
		goto fail2;
	}

#ifdef CONFIG_ESI_EDGE
	printk(KERN_CRIT "%s: esi id set to %d\n", __FUNCTION__, input->esi_id);

	platform_device = pdev;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		unsigned int type = button->type;

		if(type == EV_SW)
			gpio_keys_report_event(button, 0, 0);	// refresh switch states
	}

	/* Create a sysfs node to read simulated events */
	sysfs_create_group(&pdev->dev.kobj, &gpio_key_attr_group);

#endif

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), pdev);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

#ifdef CONFIG_ESI_EDGE
	sysfs_remove_group(&pdev->dev.kobj, &gpio_key_attr_group);
#endif

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, pdev);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		gpio_free(pdata->buttons[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int gpio_keys_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				enable_irq_wake(irq);
			}
		}
	}

	return 0;
}

#define trace(FMT, ARGS...) if(1) printk("%s at line %d: "FMT"\n", __FILE__, __LINE__, ## ARGS)
static int gpio_keys_resume(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;
	trace("%s", __FUNCTION__);

	if (device_may_wakeup(&pdev->dev)) {
		trace("%s", __FUNCTION__);
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				disable_irq_wake(irq);
			}
		}
	}
#ifdef CONFIG_ESI_EDGE_JR
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if(button->code == KEY_F8)	// MENU key
				{
				trace("%s: KEY_F8", __FUNCTION__);
				gpio_keys_report_event(button, 1, 1);
				gpio_keys_report_event(button, 1, 0);
				}
		}
#endif
	return 0;
}
#else
#define gpio_keys_suspend	NULL
#define gpio_keys_resume	NULL
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
