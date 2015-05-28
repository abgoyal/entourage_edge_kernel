/*
 *  Dummy Keyboard driver for the ESI eDGe board
 *
 *  Copyright (c) 2008 Joe Kralowetz
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>

static unsigned char esikbd_keycode[] = {
	/* These are the keys we'll receive from the EPD keypad */
#ifdef ESI_SUPPORT_UP_DOWN_KEYS
	KEY_UP, 
	KEY_DOWN, 
#endif
	KEY_PAGEUP, 
	KEY_PAGEDOWN, 
	KEY_REPLY, 
	KEY_F13,
	KEY_F14,

	/* These are here in order to support data input from USB keyboards, etc */
	KEY_ESC,
	KEY_1,
	KEY_2,
	KEY_3,
	KEY_4,
	KEY_5,
	KEY_6,
	KEY_7,
	KEY_8,
	KEY_9,
	KEY_0,
	KEY_MINUS,
	KEY_EQUAL,
	KEY_BACKSPACE,
	KEY_TAB,
	KEY_Q,
	KEY_W,
	KEY_E,
	KEY_R,
	KEY_T,
	KEY_Y,
	KEY_U,
	KEY_I,
	KEY_O,
	KEY_P,
	KEY_LEFTBRACE,
	KEY_RIGHTBRACE,
	KEY_ENTER,
	KEY_LEFTCTRL,
	KEY_A,
	KEY_S,
	KEY_D,
	KEY_F,
	KEY_G,
	KEY_H,
	KEY_J,
	KEY_K,
	KEY_L,
	KEY_SEMICOLON,
	KEY_APOSTROPHE,
	KEY_GRAVE,
	KEY_LEFTSHIFT,
	KEY_BACKSLASH,
	KEY_Z,
	KEY_X,
	KEY_C,
	KEY_V,
	KEY_B,
	KEY_N,
	KEY_M,
	KEY_COMMA,
	KEY_DOT,
	KEY_SLASH,
	KEY_RIGHTSHIFT,
	KEY_KPASTERISK,
	KEY_LEFTALT	,
	KEY_SPACE,
	KEY_CAPSLOCK,
	KEY_F1,
	KEY_F2,
	KEY_F3,
	KEY_F4,
	KEY_F5,
	KEY_F6,
	KEY_F7,
	KEY_F8,
	KEY_F9,
	KEY_F10,
	KEY_NUMLOCK,
	KEY_SCROLLLOCK,
	KEY_KP7,
	KEY_KP8,
	KEY_KP9,
	KEY_KPMINUS,
	KEY_KP4,
	KEY_KP5,
	KEY_KP6,
	KEY_KPPLUS,
	KEY_KP1,
	KEY_KP2,
	KEY_KP3,
	KEY_KP0	,
	KEY_KPDOT,
};

struct esikbd {
	unsigned char keycode[ARRAY_SIZE(esikbd_keycode)];
	struct input_dev *input_dev;
};

static struct platform_device *platform_dev;

extern int input_esi_get_switch(void);
extern void input_esi_set_switch(int value);

/* Report the event to the input handler */
void
esivkbd_report_input(int type, int code, int value)
{
	struct esikbd *esikbd;

	esikbd = (struct esikbd *)platform_get_drvdata(platform_dev);
	input_event(esikbd->input_dev, type, code, value);
	//input_sync(esikbd->input_dev);	
}

/* User can write input triplets (t,c,v) to sysfs input node "/sys/devices/platform/esi-keyboard/input" */
static ssize_t
write_esivkbd(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	int key_type, key_code, key_value;

	sscanf(buffer, "%d%d%d", &key_type, &key_code, &key_value);

	/* Protect against bogus key types */
	if ((key_type < 0) || (key_type >= EV_MAX))
		return count;

#ifdef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] entered, type %d code 0x%x(%d) value %d\n", __FUNCTION__, key_type, key_code, key_code, key_value);
#endif

	esivkbd_report_input(key_type, key_code, key_value);

	return count;
}

static ssize_t get_switch_esivkbd(struct device *dev, struct device_attribute *attr, char *buffer)
{
	int value = input_esi_get_switch();

	return sprintf(buffer, "%u\n", value);
}

/* 
 * User can write non-zero (bit 0 for Journal, bit 1 for Annotator),
 * to sysfs input node 
 * "/sys/devices/platform/esi-keyboard/switch" 
 * to redirect input from devices like the USB keyboard
 * to come through the virtual keyboard
 */
static ssize_t
set_switch_esivkbd(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	int value;

	sscanf(buffer, "%d", &value);

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] entered, val %d\n", __FUNCTION__, value);
#endif

	input_esi_set_switch(value);

	return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(esiinput, 0666, NULL, write_esivkbd);
DEVICE_ATTR(switch, 0666, get_switch_esivkbd, set_switch_esivkbd);

/* Attribute Descriptor */
static struct attribute *esivkbd_attrs[] = {
	&dev_attr_esiinput.attr,
	&dev_attr_switch.attr,
	NULL
};

/* Attribute group */
static struct attribute_group esi_vkbd_attr_group = {
	.attrs = esivkbd_attrs,
};

static int esikbd_probe(struct platform_device *pdev)
{
	struct esikbd *esikbd = NULL;
	struct input_dev *input_dev = NULL;
	int i;
	int error;

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] entered\n", __FUNCTION__ );
#endif

	/* Allocate the input device */
	esikbd = kzalloc(sizeof(struct esikbd), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!esikbd || !input_dev) {
		error = -ENOMEM;
		goto fail;
	}

	platform_set_drvdata(pdev, esikbd);

	memcpy(esikbd->keycode, esikbd_keycode, sizeof(esikbd->keycode));
	esikbd->input_dev = input_dev;

	input_dev->name = "ESI Virtual Keyboard";
	input_dev->phys = "esikbd/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;

//	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);

	/* Set up the key map */
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	input_dev->keycode = esikbd->keycode;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(esikbd_keycode);

	for (i = 0; i < ARRAY_SIZE(esikbd_keycode); i++)
		set_bit(esikbd->keycode[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);

	/* Mark that we are a core esi input device and Register with the input subsystem */
	input_dev->esi_id = 1;	
#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] registering esi-keys\n", __FUNCTION__ );
	error = input_register_device(esikbd->input_dev);
	if (error)
		goto fail;
#else
	printk( KERN_CRIT "[%s] skip starting esi-keys\n", __FUNCTION__ );
#endif

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] finished successfully\n", __FUNCTION__ );
#endif

	return 0;

 fail:	

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] finished with error\n", __FUNCTION__ );
#endif

	/* Free the allocated memory */
 	kfree(esikbd);
	if (input_dev)
		input_free_device(input_dev);
	return error;
}

static int __init esikbd_init(void)
{
#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] entered\n", __FUNCTION__ );
#endif

	platform_dev = platform_device_register_simple("esi-keyboard", -1, NULL, 0);
	if (IS_ERR(platform_dev)) {
		printk( KERN_CRIT "[%s] platform register err\n", __FUNCTION__ );
		return PTR_ERR(platform_dev);
	}

	/* Create a sysfs node to read simulated events */
	sysfs_create_group(&platform_dev->dev.kobj, &esi_vkbd_attr_group);

	esikbd_probe(platform_dev);

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] left\n", __FUNCTION__ );
#endif
	return 0;
}

static int esikbd_remove(struct platform_device *pdev)
{
	struct esikbd *esikbd = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &esi_vkbd_attr_group);
	input_unregister_device(esikbd->input_dev);
	input_free_device(esikbd->input_dev);
	kfree(esikbd);

	return 0;
}

static void __exit esikbd_exit(void)
{
#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] entered\n", __FUNCTION__ );
#endif

	esikbd_remove(platform_dev);
	platform_device_unregister(platform_dev);

#ifndef FUG_DEBUG_DISABLED
	printk( KERN_CRIT "[%s] left\n", __FUNCTION__ );
#endif
}

module_init(esikbd_init);
module_exit(esikbd_exit);

MODULE_AUTHOR("Entourage Systems, Inc");
MODULE_DESCRIPTION("ESI Virtual Keyboard Driver");
MODULE_LICENSE("GPL v2");
