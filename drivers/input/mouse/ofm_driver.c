
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/i2c/ofm_pdata.h>

#ifndef CONFIG_ESI_EDGE
#include <mach/gpio-core.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-gpio.h>
#include <plat/gpio-cfg-helpers.h>	
#endif

#include <linux/time.h>
#include <linux/workqueue.h>

//#define sensivility 1.5 //HZ/50


#define trace(FMT, ARGS...) if(0) printk("%s at line %d: "FMT"\n", __FILE__, __LINE__, ## ARGS)

#define OFM_DEBUG 0

static void ofm_motion_func(struct work_struct *work);

struct ofm{
	struct 	delayed_work work;
	struct	i2c_client *client;
	struct	input_dev *input;
	struct	ofm_pin *power_dn;
	struct	ofm_pin *ofm_motion;
	struct	ofm_pin *ofm_left;

};

struct workqueue_struct *ofm_workqueue;

static void ofm_motion_func(struct work_struct *work){
	struct ofm *ofm = container_of(work,
				struct ofm,
				work.work);
	struct input_dev *input = ofm->input;
	
	int error, state;
	s16  x,y;
	
	trace("%s", __FUNCTION__);

	do {
	#if OFM_DEBUG
		printk("XY DATA receiving\n");
	#endif	
	
	error = i2c_smbus_read_byte_data(ofm->client,0x22);
	
	if(error<0){
		#if OFM_DEBUG
			printk( "ofm : i2c error return(%d)\n",error);	
		#endif
		goto out;
	}
	y = error;
	
	error = i2c_smbus_read_byte_data(ofm->client,0x21);
	
	if(error<0){
		#if OFM_DEBUG
			printk( "ofm : i2c error return(%d)\n",error);	
		#endif
		goto out;
	}
	x = error;	 
		 
	x = x>128 ? x-256:x;
	y = y>128 ? y-256:y;
	
#if 1
	/* NEW magnitude checking to decrease sensitivity */
#define MAX_X_MAGNITUDE		10
#define MAX_Y_MAGNITUDE		10
	if (abs(x) > MAX_X_MAGNITUDE) {
//		printk(KERN_DEBUG "%s: throttle X (%3d)\n",__FUNCTION__, x); 	
		if (x >= 0)
			x = MAX_X_MAGNITUDE;
		else
			x = -MAX_X_MAGNITUDE;
	}
	if (abs(y) > MAX_Y_MAGNITUDE) {
//		printk(KERN_DEBUG "%s: throttle Y (%3d)\n",__FUNCTION__, y); 	
		if (y >= 0)
			y = MAX_Y_MAGNITUDE;
		else
			y = -MAX_Y_MAGNITUDE;
	}
#endif
	
	if (x) {
		input_report_rel(input,REL_X,x);
//		printk(KERN_DEBUG "%s: X :[%3d]\n",__FUNCTION__, x); 	
	}
	if (y) {
		input_report_rel(input,REL_Y,y);
//		printk(KERN_DEBUG "%s: Y :[%3d]\n",__FUNCTION__, y); 	
	}
	if (x || y) {
		input_sync(input);
	}

	state = !!gpio_get_value(ofm->ofm_motion->pin);

	/*
	 * We currently stay in a loop reading new x/y positions.  However,
	 * this ends up flooding events upstream and results in a very touchy
	 * setting. So put a sleep in this loop to slow the event rate.
	 * The sleep argument is in ms but is converted to jiffies - so you
	 * end up with increments of jiffies (10ms) only.  So effectively we
	 * want to sleep for just 1 jiffy.
	 * This also allows us to relax the MAX_X/Y_MAGNITUDE settings above, 
	 * which seems to improve mouse operation.
	 */
	msleep(0);

#if OFM_DEBUG	
	printk("X :[%3d]  Y:[%3d]\n",x,y); 	
	
	printk("%d END\n",!!gpio_get_value(ofm->ofm_motion->pin));
#endif		
	} while(!state);	
	
out: 
	enable_irq(ofm->ofm_motion->irq);

}

static irqreturn_t ofm_event(int irq, void *dev_id/*, struct pt_regs *regs*/)
{
	struct ofm *ofm = (struct ofm *)dev_id;

	trace("%s", __FUNCTION__);

	if (!ofm){
		printk("ofm :ofm_event interrupt error \n");
	return IRQ_HANDLED;
	}
	disable_irq_nosync(irq);

	schedule_delayed_work(&ofm->work, 0 /*HZ/50*/);
	
	return IRQ_HANDLED;
}


static irqreturn_t ofm_left_event(int irq, void *dev_id/*, struct pt_regs *regs*/)
{
	struct ofm *ofm = (struct ofm *)dev_id;
	int down;

	trace("%s", __FUNCTION__);

	if (!ofm){
		printk("ofm : ofm_left_event interrupt error \n");
	return IRQ_HANDLED;
	}
	disable_irq_nosync(irq);
	down = gpio_get_value(ofm->ofm_left->pin) ? 0 : 1; //  KEY_RELEASED : KEY_PRESSED
		
	input_report_key(ofm->input, BTN_LEFT, down);
	
	input_sync(ofm->input);
	
	enable_irq(irq);
	return IRQ_HANDLED;
}

	
static int ofm_i2c_write(struct i2c_client *client, u_int8_t index, u_int8_t data)
{
	int error;
	u_int8_t buf[2] = {index , data};	
		error= i2c_master_send(client, buf, 2);
		if(error>=0)
			return 0;
	
	printk("ofm : ERROR i2c send!!!index(%x) data(%x) return (%x)\n",index,data,error);
	return error;
}


static int __devinit ofm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ofm	*ofm;
	struct input_dev *input;
	int error = 0;
	struct ofm_pin *ofm_pins = client->dev.platform_data;
	struct ofm_pin *this_pin;
	
	printk("ofm : I2C device probe  \n");
	ofm = kzalloc(sizeof(struct ofm), GFP_KERNEL);
	if(!ofm){
		dev_err(&client->dev, "ofm : failed to allocate driver data\n");
		error = -ENOMEM;
		goto err0;
	}
	
	i2c_set_clientdata(client, ofm);
	
	
	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "ofm : Failed to allocate input device.\n");
		error = -ENOMEM;
		goto err1;
	}
	input->evbit[0] =BIT_MASK(EV_SYN)| BIT_MASK(EV_KEY)|BIT_MASK(EV_REL);
	input->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT);
	input->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
	
	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->phys = "ofm";
	input->id.vendor = 0xDEAD;
	input->id.product = 0xBEEF;
	input->id.version = 0x01;
	
	input_set_drvdata(input, ofm);
	
	ofm->client 	= client;
	ofm->input 	= input;
	ofm->ofm_left = &ofm_pins[OFM_LEFT_BUTTON];
	ofm->ofm_motion = &ofm_pins[OFM_MOTION_DETECT];
	ofm->power_dn = &ofm_pins[OFM_POWER_DN];
	
/*
*	pin setting
*/


#if 0   // ESI changed config done in edge.c
	s3c_gpio_cfgpin(ofm->ofm_left->pin, ofm->ofm_left->pin_setting);
       s3c_gpio_setpull(ofm->ofm_left->pin, S3C_GPIO_PULL_UP);
	
	s3c_gpio_cfgpin(ofm->power_dn->pin, ofm->power_dn->pin_setting);
        s3c_gpio_setpull(ofm->power_dn->pin,S3C_GPIO_PULL_UP);
	gpio_set_value(ofm->power_dn->pin, 1);
	
	s3c_gpio_cfgpin(ofm->ofm_motion->pin, ofm->ofm_motion->pin_setting);
	s3c_gpio_setpull(ofm->ofm_motion->pin, S3C_GPIO_PULL_UP);
#else
	this_pin = ofm->ofm_left;
	error = gpio_request(this_pin->pin, this_pin->name);
	if(error)
		{
		trace("gpio_request = %d name = %s failed! error = %d", 
			this_pin->pin, this_pin->name, error);
		goto err1;
		}

	this_pin = ofm->ofm_motion;
	error = gpio_request(this_pin->pin, this_pin->name);
	if(error)
		{
		trace("gpio_request = %d name = %s failed! error = %d", 
			this_pin->pin, this_pin->name, error);

		gpio_free(ofm->ofm_left->pin);
		goto err1;
		}

	this_pin = ofm->power_dn;
	error = gpio_request(this_pin->pin, this_pin->name);
	if(error)
		{
		trace("gpio_request = %d name = %s failed! error = %d", 
			this_pin->pin, this_pin->name, error);

		gpio_free(ofm->ofm_left->pin);
		gpio_free(ofm->ofm_motion->pin);
		goto err1;
		}

	gpio_direction_input(ofm->ofm_left->pin);	// input
	gpio_direction_input(ofm->ofm_motion->pin);	// input
	gpio_direction_output(ofm->power_dn->pin, 1);	// output

	mdelay(100);

	gpio_set_value(ofm->power_dn->pin, 0);
#endif
//end	


	error = input_register_device(input);
	
	if (error)
		goto err1;
	
/* 
*	 init  for ofm i2c
*
*/
	ofm_i2c_write(ofm->client,0x05,0x2D);
	ofm_i2c_write(ofm->client,0x27,0x12);	/* rotate 90 deg CW */
	ofm_i2c_write(ofm->client,0x2A,0x04);	/* scaling for X dir */
	ofm_i2c_write(ofm->client,0x2B,0x04);	/* scaling for Y dir */
#if 0
	printk(" 0x05 : 0x%02x \n",i2c_smbus_read_byte_data(ofm->client,0x05));
	printk(" 0x27 : 0x%02x \n",i2c_smbus_read_byte_data(ofm->client,0x27));
	printk(" 0x2a : 0x%02x \n",i2c_smbus_read_byte_data(ofm->client,0x2a));
	printk(" 0x2b : 0x%02x \n",i2c_smbus_read_byte_data(ofm->client,0x2b));
#endif
	
	trace("receiving data: r0 = %02x r1 = %02x r5 = %02x r2b = %02x", 
				i2c_smbus_read_byte_data(ofm->client, 0x00),
				i2c_smbus_read_byte_data(ofm->client, 0x01),
				i2c_smbus_read_byte_data(ofm->client, 0x05),
				i2c_smbus_read_byte_data(ofm->client, 0x2b));
	//printk("receving data : %x \n ",i2c_smbus_read_byte_data(ofm->client,0x2B));

	
	INIT_DELAYED_WORK(&ofm->work, ofm_motion_func);
	
/*
* 	init for motion interrupt
*/	
	
	error = request_irq (ofm->ofm_motion->irq, ofm_event,\
			IRQF_DISABLED | IRQF_TRIGGER_FALLING , ofm->ofm_motion->name, (void *)ofm);
		
	trace("%s: irq = %d error = %d", "request_irq", ofm->ofm_motion->irq, error);

	if (error) {
		printk("ofm : request_irq failed: %s\n", ofm->ofm_motion->name);
		goto err2;
	}

/*
*	init for left button interrupt
*/	
	error = request_irq (ofm->ofm_left->irq, ofm_left_event,\
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, ofm->ofm_left->name, (void *)ofm);
	
	trace("%s: irq = %d error = %d", "request_irq", ofm->ofm_left->irq, error);

	if (error) {
		printk("ofm : request_irq failed: %s\n", ofm->ofm_left->name);
		goto err2;
	}
//end
	
	
	return 0;		
	
err2:
	input_unregister_device(input);
	input = NULL; /* so we dont try to free it below */
err1:
	input_free_device(input);
	kfree(ofm);
err0:
	dev_set_drvdata(&client->dev, NULL);
	printk("error: I2C device probe (%d) \n",error);
	return error;
}

static __devexit int ofm_i2c_remove(struct i2c_client *client)
{
	struct ofm *ofm = i2c_get_clientdata(client);
	destroy_workqueue(ofm_workqueue);
	kfree(ofm);
	
	return 0;
}


#ifdef CONFIG_PM
static
int ofm_i2c_suspend(struct i2c_client *client, pm_message_t mesg) {
	struct ofm_pin *ofm_pins = client->dev.platform_data;
	gpio_set_value(ofm_pins[OFM_POWER_DN].pin, 1);
	return 0;
}

static
int ofm_i2c_resume(struct i2c_client *client) {
	struct ofm_pin *ofm_pins = client->dev.platform_data;
	gpio_set_value(ofm_pins[OFM_POWER_DN].pin, 0);
	return 0;
}
#else
#define ofm_i2c_suspend NULL
#define ofm_i2c_resume NULL
#endif

static const struct i2c_device_id ofm_i2c_id[]={
	{"ofm", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ofm_i2c);

static struct i2c_driver ofm_i2c_driver = {
	.driver	=	{
		.name	="ofm",
		.owner	=THIS_MODULE,
	},
	.probe	= ofm_i2c_probe,
	.remove	= ofm_i2c_remove,
	.id_table	=	ofm_i2c_id,

	.suspend	 = ofm_i2c_suspend,
	.resume		 = ofm_i2c_resume,
};


static int __init ofm_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ofm_i2c_driver);
	
	if(ret!=0)
		printk("ofm : I2C device init Faild! return(%d) \n",  ret);
	
	printk("ofm : I2C device init Success\n");
	return ret;
}
module_init(ofm_init);
static void __exit ofm_exit(void)
{
	i2c_del_driver(&ofm_i2c_driver);
}



module_exit(ofm_exit);
MODULE_DESCRIPTION("ofm mouse driver");
MODULE_AUTHOR("Partron-sensor-kimsoohwan");
MODULE_LICENSE("GPL");

