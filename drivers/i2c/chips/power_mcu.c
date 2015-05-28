#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <mach/power_mcu.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa168.h>
#include <plat/mfp.h>
#include <mach/gpio_ec.h>

static struct i2c_client *g_client;
static struct class *power_mcu_class;

static ssize_t led_blink_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
		unsigned char value = 0;

		if(power_mcu_read(MCU_LED_BLINK, &value))
			value = 255;

		sprintf(buf, "%d\n", value);
		return strlen(buf);
}

static ssize_t reset_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d\n", 0);
}

static ssize_t vcore_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
	unsigned short vcore;
	power_mcu_read(MCU_VCORE_ADJ, &vcore);
	return sprintf(buf, "%d\n", vcore);
}

static ssize_t ecversion_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
	unsigned char version;
	power_mcu_read(MCU_VERSION, &version);
	return sprintf(buf, "%d\n", version);
}

static ssize_t led_blink_store_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {
	int value = 0;
	if(count && sscanf(buf,"%d", &value) > 0)
		power_mcu_write_word(MCU_LED_BLINK, value );
	return count;
}

static ssize_t reset_store_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {
	return 0;
}

static ssize_t vcore_store_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {
	unsigned short flag = 0;
	unsigned long vcore;

	strict_strtoul(buf, 10, &vcore);
	if (vcore > 3) {
		printk(KERN_WARNING "vcore is invalid %lu\n", vcore);
		return 0;
	}

	/* vcore:
	 * 0	---- 1.0373V
	 * 1	---- 1.1049V
	 * 2	---- 1.1788V
	 * 3	---- 1.2457V
	 */
	power_mcu_write_word(MCU_VCORE_ADJ, 0x80 + vcore);

	msleep(100);
	power_mcu_read(MCU_VCORE_ADJ, &flag);
	if (flag != vcore)
		printk(KERN_WARNING "Vcore adjust failed\n");
	return count;
}

//#define FUEL_GUAGE_INSPECTOR
#ifdef FUEL_GUAGE_INSPECTOR
static ssize_t fgi_store_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {
	unsigned short flag = 0;
	unsigned short addr, data;
	int args = sscanf(buf, "%hx=%hx", &addr, &data);

	if (args == 0 || addr > 0x7f) {
		printk(KERN_WARNING "cmd is invalid: '%s'\n", buf);
		return count;
	}

	power_mcu_write_word(MCU_FG_ADDR, addr);

	if(args == 2) {
		power_mcu_write_word(MCU_FG_DATA, data);
		power_mcu_write_word(MCU_FG_CMD, 0x82);		// write
	} else {
		power_mcu_write_word(MCU_FG_CMD, 0x81);		// read
	}

	msleep(100);
	power_mcu_read(MCU_FG_CMD, &flag);
	if (flag & 0x80)
		printk(KERN_WARNING "fg inspect failed\n");
	return count;
}

static ssize_t fgi_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
	unsigned short data;
	power_mcu_read(MCU_FG_DATA, &data);
	return sprintf(buf, "0x%02x\n", data);
}
#endif
static struct device_attribute power_mcu_attributes[] = {
	__ATTR(led_blink, 0644, led_blink_show_status, led_blink_store_status),
	__ATTR(reset, 0644, reset_show_status, reset_store_status),
	__ATTR(vcore, 0644, vcore_show_status, vcore_store_status),
	__ATTR(ecversion, 0644, ecversion_show_status, NULL),
#ifdef FUEL_GUAGE_INSPECTOR
	__ATTR(fuel_guage, 0644, fgi_show_status, fgi_store_status),
#endif
	__ATTR_NULL,
};

/*
 * ESI - there is a bug in v2.1 EC firmware where it may hold interrupts
 * disabled too long while sampling the battery fuel gauge. This may cause
 * the avenger i2c access to intermittently fail.
*/ 
#define USE_EC_21_TIMEOUT_WORKAROUND

#ifdef USE_EC_21_TIMEOUT_WORKAROUND
#define retry(FUNC, RETRIES) ({ int zz = RETRIES, zx; do { if((zx = FUNC) < 0) mdelay(50); } while(zx < 0 && zz--); zx; })
#else
#define retry(FUNC, RETRIES) FUNC
#endif

int power_mcu_read(unsigned char reg, void *pval)
{
	int ret, status = -EIO;

	if (g_client == NULL)
		return -EFAULT;

	if (reg < MCU_OFFSET_WORD) {
		/*
		if (set_bus_busy() == -1)
			return -EINVAL;
		*/
		ret = retry(i2c_smbus_read_byte_data(g_client, reg), 2);
		/* set_bus_free(); */
		if (ret >= 0) {
			*((unsigned char *)pval) = (unsigned char)ret;
			status = 0;
		}
	} else if ((reg >= MCU_OFFSET_WORD) && (reg < MCU_END)) {
		/*
		if (set_bus_busy() == -1)
			return -EINVAL;
		*/
		ret = retry(i2c_smbus_read_word_data(g_client, reg), 2);
		/* set_bus_free(); */

		if (ret >= 0) {
			*((unsigned short *)pval) = (unsigned short)ret;
			status = 0;
		}
	} else
		return -EINVAL;

	return status;
}


EXPORT_SYMBOL(power_mcu_read);

int power_mcu_write_word(unsigned char reg, unsigned short val)
{
	int ret;

	if (g_client == NULL)
		return -EFAULT;
	if (reg >= MCU_END)
		return -EINVAL;

	/*
	if (set_bus_busy() == -1)
		return -EINVAL;
	*/
	ret = retry(i2c_smbus_write_word_data(g_client, reg, val), 2);
	/* set_bus_free(); */

	return ret;
}

int power_mcu_write_byte(unsigned char reg, unsigned char val)
{
	int ret;

	if (g_client == NULL)
		return -EFAULT;

	if (reg >= MCU_END)
		return -EINVAL;

	/*
	if (set_bus_busy() == -1)
		return -EINVAL;
	*/
	ret = retry(i2c_smbus_write_byte_data(g_client, reg, val), 2);
	/* set_bus_free(); */

	return ret;
}

EXPORT_SYMBOL(power_mcu_write_byte);
EXPORT_SYMBOL(power_mcu_write_word);

void power_mcu_write_rtc(u32 rtc)
{
	power_mcu_write_word(MCU_RTC_LSB, rtc);
	power_mcu_write_word(MCU_RTC_MSB , rtc >> 16);
}

void power_mcu_read_rtc(u32 *rtc)
{
	power_mcu_read(MCU_RTC_LSB, rtc);
	power_mcu_read(MCU_RTC_MSB, ((u8 *)rtc + 2));
}

EXPORT_SYMBOL(power_mcu_write_rtc);
EXPORT_SYMBOL(power_mcu_read_rtc);

void power_mcu_write_alarm(u32 rtc)
{
	power_mcu_write_word(MCU_ALARM_LSB , rtc);
	power_mcu_write_word(MCU_ALARM_MSB , rtc >> 16);
}

EXPORT_SYMBOL(power_mcu_write_alarm);

static int __devinit power_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int nretry = 25;
	unsigned char version;
	g_client = client;

	for (i = 0; i < nretry; i++) {
		if (power_mcu_read(MCU_VERSION, &version) == 0) {
			printk("power_mcu: probe reports mcu version: %02d\n", version);
			return 0;
		}
	}
	printk("power_mcu: probe failed\n");
	g_client = NULL;
	return -1;
}

static int __devexit power_i2c_remove(struct i2c_client *client)
{
	return 0;
}

#define trace(FMT, ARGS...) if(0) printk("%s at line %d: "FMT"\n",  \
				__FILE__, __LINE__, ## ARGS)

#ifdef CONFIG_PM
static	int power_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	trace();
	power_mcu_write_word(MCU_LED_BLINK, 1);
	return 0;
}

static	int power_i2c_resume(struct i2c_client *client)
{
	int ret = power_i2c_probe(client, NULL);

	trace("ret = %d", ret);
	if(!ret)
		power_mcu_write_word(MCU_LED_BLINK, 0);

	return ret;
}
#endif

static struct i2c_device_id power_mcu_idtable[] =
{
	{"power_mcu", 0},
	{},
};

static struct i2c_driver power_mcu_driver = {
	.driver = {
		.name	= "power_mcu",
	},
	.id_table		= power_mcu_idtable,
	.probe			= &power_i2c_probe,
	.remove			= &power_i2c_remove,
#ifdef CONFIG_PM
	.suspend		= &power_i2c_suspend,
	.resume			= &power_i2c_resume,
#endif
};

static struct device *dev;

static int __init power_mcu_init(void)
{
	int ret;

	printk("power_mcu_init\n");
	if ((ret = i2c_add_driver(&power_mcu_driver))) {
		printk(KERN_ERR "power mcu Driver registration failed\n");
		return ret;
	}

	power_mcu_class = class_create(THIS_MODULE, "power_mcu");
	if (IS_ERR(power_mcu_class)) {
		printk(KERN_WARNING "Unable to create \
		power_mcu class;errno=%ld\n", PTR_ERR(power_mcu_class));
		return PTR_ERR(power_mcu_class);
	}

	power_mcu_class->dev_attrs = power_mcu_attributes;
	dev = kzalloc(sizeof(struct device), GFP_KERNEL);
	dev->class = power_mcu_class;
	dev->release = NULL;
	strlcpy(dev->bus_id, "power_mcu", BUS_ID_SIZE);
	ret = device_register(dev);
	if (ret)
		printk("power_mcu error:%d\n", -EEXIST);
	return 0;
}

static void __exit power_mcu_exit(void)
{
	i2c_del_driver(&power_mcu_driver);
	class_destroy(power_mcu_class);
}

module_init(power_mcu_init);
module_exit(power_mcu_exit);


