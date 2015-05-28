#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa168.h>
#include <plat/mfp.h>
#include <linux/input.h>

#include <asm/mach-types.h>

/* For GPIO Expander (from aspenite.c) */
#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))

enum PM_EVENT{
    PM_STANDBY ,
    PM_SHUTDOWN,
    PM_STANDBY_NOW ,
    PM_SHUTDOWN_NOW,
    PM_NORMAL,
 };

struct work_struct irq_work;

struct input_dev *input;

atomic_t event;

enum PM_STATUS{
    PM_INIT,
    PM_SHUTDOWN_WAITING,
};

enum PM_STATUS status_pm;

static struct class *power_button_class;

#define SLEEP_CODE 0x58
#define SHUTDOWN_CODE 0x57

void shutdown_ok()
{
	printk("shutdown now\n");
       kernel_power_off();
}

void standby_ok()
{
	printk("standby now\n");
#ifndef CONFIG_ESI_EDGE_JR
	gpio_direction_output(MFP_PIN_GPIO53, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO53, 0);
#else
	/* the Jr way */
	gpio_direction_output(MFP_PIN_GPIO121, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO121, 0);
#endif
}

static ssize_t shutdown_show_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static ssize_t standby_show_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static ssize_t shutdown_store_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk("OS shutdown the system\n");
	atomic_set(&event, PM_SHUTDOWN_NOW);
	schedule_work(&irq_work);
	return 0;
}

static ssize_t standby_store_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk("OS make the systemto be standby\n");
	atomic_set(&event , PM_STANDBY_NOW);
	schedule_work(&irq_work);
	return 0;
}

static struct device_attribute bl_device_attributes[] = {
	__ATTR(pm_shutdown, 0644, shutdown_show_status, shutdown_store_status),
	__ATTR(pm_standby, 0644, standby_show_status,
		     standby_store_status),
	__ATTR_NULL,
};

static void power_button_irq_work(struct work_struct *work)
{
	int status;

	status = atomic_read(&event) ;
	switch (status) {
	case PM_SHUTDOWN_NOW:
		shutdown_ok();
		return;
	case PM_STANDBY_NOW:
		standby_ok();
		return;
	case PM_SHUTDOWN:
		/* ACK */
		printk("long press is detected\n");

#ifdef CONFIG_ESI_EDGE
#ifndef CONFIG_ESI_EDGE_JR
		if(!!gpio_get_value(MFP_PIN_GPIO42) == 0) {
			printk("Et tu, Brute?\n");
			return;
		}
#else
		if(!!gpio_get_value(GPIO_EXT0(0)) == 0) {
			printk("Et tu, Brute?\n");
			return;
		}
#endif
#endif
#ifndef CONFIG_ESI_EDGE_JR
		gpio_direction_output(MFP_PIN_GPIO51, 1);	/* call off the dogs */
		mdelay(100);
		gpio_direction_output(MFP_PIN_GPIO51, 0);
#else
		/* the Jr. way */
		gpio_direction_output(MFP_PIN_GPIO119, 1);	/* call off the dogs */
		mdelay(100);
		gpio_direction_output(MFP_PIN_GPIO119, 0);
#endif

		input_event(input, EV_KEY, SHUTDOWN_CODE, 1);	/* then tell android */
		input_sync(input);
		input_event(input, EV_KEY, SHUTDOWN_CODE, 0);
		input_sync(input);

		return;
	case PM_STANDBY:
		printk("short press is detected\n");
		input_event(input, EV_KEY, SLEEP_CODE, 1);
		input_sync(input);
		input_event(input, EV_KEY, SLEEP_CODE, 0);
		input_sync(input);

		return;
	default:
		return;
	}

}

static irqreturn_t standby_handler(int irq, void *dev_id)
{
	atomic_set(&event, PM_STANDBY);
	schedule_work(&irq_work);

	return IRQ_HANDLED;
}

static irqreturn_t shutdown_handler(int irq, void *dev_id)
{
	atomic_set(&event, PM_SHUTDOWN);
	schedule_work(&irq_work);

	return IRQ_HANDLED;
}

static void device_release(struct device *dev)
{
}

static struct device *dev;

static int power_button_probe(struct platform_device *pdev)
{
	int ret;
	int irq;

#ifndef CONFIG_ESI_EDGE_JR
	irq = gpio_to_irq(MFP_PIN_GPIO49);
	ret = request_irq(irq , shutdown_handler, \
				IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING , \
				"shutdown detect", NULL);
	if (ret) {
		printk(KERN_ERR "%s: can't request detect standby irq\n",
				__FUNCTION__);
		goto out;
	}

	irq = gpio_to_irq(MFP_PIN_GPIO52);
	ret = request_irq(irq, standby_handler , \
				IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING , \
				"standby detect", NULL);
	if (ret) {
		printk(KERN_ERR "%s: can't request detect shutdown irq\n",
				__FUNCTION__);
		goto out;
	}

	gpio_direction_input(MFP_PIN_GPIO49);
	gpio_direction_input(MFP_PIN_GPIO52);
	if (machine_is_avengers_lite())
		gpio_direction_input(MFP_PIN_GPIO109);
#else
	/* the Jr. way */
	irq = gpio_to_irq(MFP_PIN_GPIO118);
	ret = request_irq(irq , shutdown_handler, \
				IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING , \
				"shutdown detect", NULL);
	if (ret) {
		printk(KERN_ERR "%s: can't request detect standby irq\n",
				__FUNCTION__);
		goto out;
	}

	irq = gpio_to_irq(MFP_PIN_GPIO120);
	ret = request_irq(irq, standby_handler , \
				IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING , \
				"standby detect", NULL);
	if (ret) {
		printk(KERN_ERR "%s: can't request detect shutdown irq\n",
				__FUNCTION__);
		goto out;
	}

	gpio_direction_input(MFP_PIN_GPIO118);
	gpio_direction_input(MFP_PIN_GPIO120);
#endif

	INIT_WORK(&irq_work , power_button_irq_work);

	atomic_set(&event , PM_NORMAL);


	input = input_allocate_device();
	if (!input)
		goto out;


	input->name = "power-button";
	input->phys = "power-button/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
#ifdef CONFIG_ESI_EDGE
	input->esi_id = 1;
#endif

	input_set_capability(input, EV_KEY , SLEEP_CODE);

	input_set_capability(input, EV_KEY , SHUTDOWN_CODE);

	ret = input_register_device(input);
	if (ret) {
		pr_err("power button: Unable to register input device, "
			"error: %d\n", ret);
		goto out;
	}

	status_pm = PM_INIT;

	dev = kzalloc(sizeof(struct device), GFP_KERNEL);
	dev->class = power_button_class;
	/* dev->parent = &pdev->dev; */
	dev->release = device_release;
	strlcpy(dev->bus_id, "power-button", BUS_ID_SIZE);

	ret = device_register(dev);
	if (ret)
		printk("power button error:%d\n", -EEXIST);

	printk("power button probe finished\n");

	return 0;
out:
	return ret;
}

static struct platform_driver power_button_driver = {
	.probe		= power_button_probe,
	.driver 	= {
		.name	= "power-button",
		.owner	= THIS_MODULE,
	},
};

static int __init power_button_init(void)
{
	int ret;

	power_button_class = class_create(THIS_MODULE, "power-button");
	if (IS_ERR(power_button_class)) {
		printk(KERN_WARNING "Unable to create power_button class; errno = %ld\n",
				PTR_ERR(power_button_class));
		return PTR_ERR(power_button_class);
	}

	power_button_class->dev_attrs = bl_device_attributes;

	ret = platform_driver_register(&power_button_driver);
	if (ret) {
		printk(KERN_ERR "power_button_driver register failure\n");
		return ret;
	}


	return 0;
}

static void __exit power_button_exit(void)
{
	input_unregister_device(input);
	platform_driver_unregister(&power_button_driver);
	class_destroy(power_button_class);
}


module_init(power_button_init);
module_exit(power_button_exit);

MODULE_LICENSE("GPL");


