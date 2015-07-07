/***********************************************************************************/
/* File Name: aw2013.c */
/* File Description: this file is used to make aw2013 driver to be added in kernel or module. */

/*  Copyright (c) 2002-2012, ZTEMT, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: ZTEMT, Inc.,            */
/***********************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock_types.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include "aw2013.h"
#include  <../../include/linux/printk.h>
#include <linux/ctype.h>





static bool aw2013_SUSPEND_FLAG=false; 

#define DELAY_256MS_UNIT 1
#define DRV_NAME "class/leds/red/outn"





enum aw_outn_mode{
	AW_SW_RESET,	    // 0 to power off the IC
	AW_CONST_ON,	    // 1 to work on a constant lightness 
	AW_CONST_OFF,	    //AW_CONST_OFF,		// 2
	AW_AUTO_BREATH, 	// 3 used in sence such as charing issue, is the repeatation of mode 6
	AW_STEP_FADE_IN,	// 4 
	AW_STEP_FADE_OUT,	// 5
	AW_BREATH_ONCE,     // 6
	AW_RESERVED,		// 7
};

#define GRADE_PARAM_LEN 20
#define CONST_MIN_GRADE  10
#define CONST_MAX_GRADE  200
#define FADE_PARAM_LEN 20



#define AW2013_I2C_MAX_LOOP 		50   

/*
#define Imax          0x02   //LED Imax,0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x05   	 //t1, LED rise time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x05   	 //t2, LED max light time light 0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time     0x05   //t3, LED fall time,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time      0x05   //t4, LED off time ,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
*/

#define Delay_time   0x00    //t0, LED Delay time ,0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00     //rpt,LED breath period number,0x00=forever,0x01=1,0x02=2.....0x0f=15




static struct aw2013_regs_data aw2013_regs =
{
	.soft_reset = 0x00,
	.enable_led = 0x01,
	.irq_state = 0x20,
	.enable_ledx = 0x30,

	.led0_mode_conf = 0x31,
	.led1_mode_conf = 0x32,
	.led2_mode_conf = 0x33,

	.pwm0_max_lightness = 0x34,
	.pwm1_max_lightness = 0x35,
	.pwm2_max_lightness = 0x36,

	.led0_rise_and_hold_time = 0x37,
	.led0_fall_and_off_time = 0x38,
	.led0_delay_and_repeat_time = 0x39,

	.led1_rise_and_hold_time = 0x3A,
	.led1_fall_and_off_time = 0x3B,
	.led1_delay_and_repeat_time = 0x3C,

	.led2_rise_and_hold_time = 0x3D,
	.led2_fall_and_off_time = 0x3E,
	.led2_delay_and_repeat_time = 0x3F,
};

// for debug issue.
static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define AW_DBG(x...) do {if (debug_mask) pr_info("aw2013  " x); } while (0)

#if 0
#define LED_DEBUG(fmt,args...)  printk(KERN_ERR"ZTEMT:",fmt,##args)
#else
#define LED_DEBUG(fmt,args...)  do {} while (0)
#endif

#ifdef CONFIG_OF
static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "aw2013", },
	{}
};
#endif




// basic funtion---i2c function
static int aw2013_i2c_rx_byte_data(
		struct i2c_client *i2c,
		unsigned char  reg,
		unsigned char* buf)
{

	struct i2c_msg msgs[2];
	
	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;
	
	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed->\n", __func__);
		return -4;
	}
	pr_debug("return  buf[0]=0x%x!\n",buf[0]);

	return 0;
}

static int aw2013_i2c_tx_byte_data(
		struct i2c_client *i2c,
		unsigned char reg, 
		unsigned char buf)
{
	struct i2c_msg msgs;
	char bufwr[2];

	bufwr[0] = reg;
	bufwr[1] = buf;

	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = bufwr;
	
	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed->\n", __func__);
		return -4;
	}
	
	return 0;
}


unsigned char read_reg(struct aw2013_control_data *led,unsigned char regaddr) 
{
	unsigned char rdbuf[1], wrbuf[1], ret;

	wrbuf[0] = regaddr;

	ret = aw2013_i2c_rx_byte_data(led->i2c_client,regaddr,rdbuf); // then read the content at the regaddr
			
	if (ret <= 0)
		printk("**********************   5555   ning read_reg failed  %s \r\n", __func__);	
    return rdbuf[0];		
}


static int write_reg(struct aw2013_control_data *led,unsigned char reg,unsigned char data)
{

	int ret;
	unsigned char i;

	for (i=0; i<AW2013_I2C_MAX_LOOP; i++)
	{
		ret = aw2013_i2c_tx_byte_data(led->i2c_client,reg,data);
		if (ret >= 0) // ack success
			break;
		}
	
	return ret;	
}

/*
static int aw2013_modify_regs(int reg,char bitn,enum aw_reg_ctl set)
{
	char buf = 0;
	int ret;

	ret = aw2013_i2c_rx_byte_data(aw2013_data.i2c_client,reg,&buf);
	if(ret < 0)
		pr_err("%s: read reg[0x%x] fail!\n",__func__,reg);

	if(set == REG_BIT_SET)
		buf |= (0x01 << bitn);
	else
		buf &= ~(0x01 << bitn);

	ret = aw2013_i2c_tx_byte_data(aw2013_data.i2c_client,reg,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);

	return ret;
}
*/






static int aw2013_soft_reset( struct aw2013_control_data *led)
{
	char buf;
	int ret;
	//buf = 0x00;
	buf = 0x55;
	ret = aw2013_i2c_tx_byte_data(led->i2c_client,aw2013_regs.soft_reset,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw2013_regs.soft_reset);
	return ret;
}


//void led_const_on(int outn, unsigned int lightness)
 void led_const_on(struct aw2013_control_data *led)
{
	unsigned char buf;	

	buf = read_reg(led,aw2013_regs.enable_led);

	if ((buf&0x01)!=0x01)
	       write_reg(led,0x01, 0x01);
	
	write_reg(led,aw2013_regs.led0_mode_conf + led->outn,0x02);

	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);

	buf = read_reg(led,aw2013_regs.enable_ledx);

	write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn)); 

	msleep(6);
	
}



void led_const_off(struct aw2013_control_data *led)
{
	int buf;	

	buf = read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx,buf & (~(0x01 << led->outn))); // outn 0 --0xFE , 1--0xFD, 2--0xFB

	msleep(1);
}

void led_auto_breath(struct aw2013_control_data *led) // self breath 0 - 255 version.
{

	   unsigned char buf;
	   buf = read_reg(led,aw2013_regs.enable_led);
		if ((buf & 0x01)!= 0x01)
		   write_reg(led,aw2013_regs.enable_led, 0x01);


		// 3. set ledx as auto mode and set it's max current.as the Imax is a unchanged value, no need to set again.
		//buf = read_reg(led,aw2013_regs.led0_mode_conf + outn);
		write_reg(led,aw2013_regs.led0_mode_conf + led->outn, 0x72); //FO=FI=MD=1, Imax=10

		write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->max_grade);
		
		// 4. set auto mode's time
		/*
		write_reg(aw2013_regs.led0_rise_and_hold_time + outn*3, (Rise_time<<4) + Hold_time);
		write_reg(aw2013_regs.led0_fall_and_off_time + outn*3, (Fall_time<<4) + Off_time);
		write_reg(aw2013_regs.led0_delay_and_repeat_time+ outn*3, (Delay_time<<4) + Period_Num);		
		*/
		//  rise is equal to fall  rise , there are 3 time. rise_fall, hold, off
		write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Hold_time);

		write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Off_time);

		write_reg(led,aw2013_regs.led0_delay_and_repeat_time + led->outn*3, (Delay_time << 4) + Period_Num);

		
		// 5. enable ledx
       // read_reg(led,aw2013_regs.enable_ledx);
       
		write_reg(led,aw2013_regs.enable_ledx, 0x01 << led->outn);
		
		//printk("the enable_ledx's value after auto set is %x\n",read_reg(led,aw2013_regs.enable_ledx));
		//printk("the outn is %x\n",outn);
		
		msleep(1);

}



void led_step_fade_in(struct aw2013_control_data *led)
{
		int buf;

		led_const_on(led);

		buf = read_reg(led,aw2013_regs.led0_mode_conf + led->outn);
		write_reg(led,aw2013_regs.led0_mode_conf + led->outn, (buf & 0xEF) | 0x20); // set pwm mode and fade in mode
		
	
	    write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn, led->Rise_Fall_time <<4);
		
		buf = read_reg(led,aw2013_regs.enable_ledx);
		write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));

	    write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,  led->max_grade);
		
		msleep(1);				
}



void led_step_fade_out(struct aw2013_control_data *led)
{
	int buf;

	led_const_on(led);
	



	buf = read_reg(led,aw2013_regs.led0_mode_conf + led->outn);
	write_reg(led,aw2013_regs.led0_mode_conf+led->outn, buf | 0x42); // set pwm mode and fade in mode

	write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn, led->Rise_Fall_time << 4);

	//enable ledx
	buf = read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));

	// set lightness
	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);
	
	msleep(1);				
}



void led_breath_once(struct aw2013_control_data *led)
{
//	int ret;
	int buf;
	
    // 1. close outn before change to const_on mode 
	buf = read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx,buf & (~(0x01 << led->outn))); // outn 0 --0xFE , 1--0xFD, 2--0xFB

	// 2. const_on, then hold ,then fall ,then off 
	led_const_on(led);
	led_step_fade_in(led);
	led_step_fade_out(led);


}


static enum led_brightness aw2013_breath_mode_get(struct led_classdev *led_cdev)
{
	struct aw2013_control_data *led;

	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return led->brightness;

	
}


//static void aw2013_breath_mode_set(struct led_classdev *led_cdev,enum aw_outn_mode brightness)
static void aw2013_breath_mode_set(struct led_classdev *led_cdev,enum led_brightness brightness)

{
	int val = brightness;
	//int rc = 0;
	struct aw2013_control_data *led;
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	switch (val) {
		case AW_SW_RESET:
			aw2013_soft_reset(led);
			break;

		case AW_CONST_ON: 
			led_const_on(led);
			break;

		case AW_CONST_OFF:
			led_const_off(led);
			break;

		case AW_AUTO_BREATH:
			led_auto_breath(led);
			break;

			//smart breath mode
		case AW_STEP_FADE_IN:
		  led_step_fade_in(led);
			break;

		case AW_STEP_FADE_OUT:
			led_step_fade_out(led);
			break;

			//fade 1 cycle for press home key
		case AW_BREATH_ONCE:
		   led_breath_once(led);
			break;

		case AW_RESERVED:
			break;

		default:
			break;

	}
}




//EXPORT_SYMBOL_GPL(set_breath_mode);


// zqf added all leds breath
/*
void aw2013_breath_all(int led0,int led1,int led2)  //led on=0x01   ledoff=0x00
{  

	//write_reg(led,0x00, 0x55);				// Reset 
	write_reg(aw2013_regs.enable_led, 0x01);		// enable LED 		

	write_reg(led,aw2013_regs.led0_mode_conf, Imax|0x70);	//config mode, IMAX = 5mA	
	write_reg(led,aw2013_regs.led1_mode_conf, Imax|0x70);	//config mode, IMAX = 5mA	
	write_reg(led,aw2013_regs.led2_mode_conf, Imax|0x70);	//config mode, IMAX = 5mA	

	write_reg(led,aw2013_regs.pwm0_max_lightness, 0xff);	// LED0 level,
	write_reg(led,aw2013_regs.pwm1_max_lightness, 0xff);	// LED1 level,
	write_reg(led,aw2013_regs.pwm2_max_lightness, 0xff);	// LED2 level,
											
	write_reg(led,aw2013_regs.led0_rise_and_hold_time, Rise_time<<4 | Hold_time);	  //led0  				
	write_reg(led,aw2013_regs.led0_fall_and_off_time, Fall_time<<4 | Off_time);	  //led0 
	write_reg(led,aw2013_regs.led0_delay_and_repeat_time, Delay_time<<4| Period_Num);  //led0 

	write_reg(led,aw2013_regs.led1_rise_and_hold_time, Rise_time<<4 | Hold_time);	  //led1						
	write_reg(led,aw2013_regs.led1_fall_and_off_time, Fall_time<<4 | Off_time);	  //led1 
	write_reg(led,aw2013_regs.led1_delay_and_repeat_time, Delay_time<<4| Period_Num);  //led1  

	write_reg(led,aw2013_regs.led2_rise_and_hold_time, Rise_time<<4 | Hold_time);	  //led2 			
	write_reg(led,aw2013_regs.led2_fall_and_off_time, Fall_time<<4 | Off_time);    //led2 
	write_reg(led,aw2013_regs.led2_delay_and_repeat_time, Delay_time<<4| Period_Num);  //

	write_reg(led,0x30, led2<<2|led1<<1|led0);	      //led on=0x01 ledoff=0x00	
	//AW2013_delay_1us(8);//Delay >5us
	msleep(1);
}
*/


	static ssize_t fade_parameter_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
	{
		struct aw2013_control_data *led;
		char *after, *parm2,*parm3;
	
		
		unsigned long delay_off,delay_off_1;
		struct led_classdev *led_cdev = dev_get_drvdata(dev);
		unsigned long delay_on = simple_strtoul(buf, &after, 10);
		led = container_of(led_cdev, struct aw2013_control_data, cdev);
	
		while(isspace(*after))
			after++;
		parm2 = after;
		delay_off = simple_strtoul(parm2, &after, 10);
	
		while(isspace(*after))
			after++;
		parm3 = after;
		delay_off_1 = simple_strtoul(parm3, &after, 10);
		led->Rise_Fall_time = (int)delay_on;
		led->Hold_time = (int)delay_off;
		led->Off_time = (int)delay_off_1; 
		LED_DEBUG("%s : %d : fade_time=%d ,on_time=%d , off_time=%d\n",
			__func__,__LINE__,led->Rise_Fall_time,led->Hold_time,led->Off_time);
		return count;
	}
	
	static ssize_t fade_parameter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
	{
		struct aw2013_control_data *led;
		struct led_classdev *led_cdev = dev_get_drvdata(dev);
		led = container_of(led_cdev, struct aw2013_control_data, cdev);
	
		return snprintf(buf, FADE_PARAM_LEN, "%4d %4d %4d\n",
				led->Rise_Fall_time, led->Hold_time, led->Off_time);
	}
	
	static ssize_t grade_parameter_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
	{
	
		struct aw2013_control_data *led;
		char *after, *parm2;
		unsigned long parameter_two;
		struct led_classdev *led_cdev = dev_get_drvdata(dev);
		unsigned long parameter_one = simple_strtoul(buf, &after, 10);
	
		
		led = container_of(led_cdev, struct aw2013_control_data, cdev);
	
		while(isspace(*after))
			after++;
		parm2 = after;
		parameter_two = simple_strtoul(parm2, &after, 10);
	
		led->min_grade=(int)parameter_one;
	
		
		led->max_grade=(int)parameter_two;
		
		LED_DEBUG("%s : %d : min_grade=%d , max_grade=%d\n",
			__func__,__LINE__,led->min_grade,led->max_grade);
		return count;
	}
	
	static ssize_t grade_parameter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
	{
	
		struct aw2013_control_data *led;
		struct led_classdev *led_cdev = dev_get_drvdata(dev);
		led = container_of(led_cdev, struct aw2013_control_data, cdev);
	
		return snprintf(buf, GRADE_PARAM_LEN,	"%4d %4d\n",
				led->min_grade,led->max_grade);
	}

static ssize_t outn_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct aw2013_control_data *led;
	char *after;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	
	unsigned long parameter_one = simple_strtoul(buf, &after, 10);
	
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	led->outn =(int) parameter_one;
	LED_DEBUG("%s : %d : ztemt_channel=%d \n",__func__,__LINE__,led->outn);
	return count;
}

static ssize_t outn_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return sprintf(buf, "%d\n",led->outn);	
}




static DEVICE_ATTR(fade_parameter, 0664, fade_parameter_show, fade_parameter_store);
static DEVICE_ATTR(grade_parameter, 0664, grade_parameter_show, grade_parameter_store);
static DEVICE_ATTR(outn, 0664, outn_show, outn_store);




static struct attribute *aw2013_attrs[] = {
	&dev_attr_fade_parameter.attr,
	&dev_attr_grade_parameter.attr,
	&dev_attr_outn.attr,
	NULL
};




static const struct attribute_group aw2013_attr_group = {
	.attrs = aw2013_attrs,
};



static int  aw2013_probe(struct i2c_client *client,

		const struct i2c_device_id *dev_id)

{
	int ret = 0;
	char buf = 0x0;
	
	struct aw2013_control_data *aw2013_data;

	
	printk("%s: start probe:\n",__func__);
	
	aw2013_data = devm_kzalloc(&client->dev,
	sizeof(struct aw2013_control_data) , GFP_KERNEL);
	if (!aw2013_data) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		   return -ENOMEM;
	}

    //aw2013_data.aw2013_plat_data=pdata;
	aw2013_data->i2c_client = client;

    i2c_set_clientdata(client,aw2013_data);

	aw2013_data->cdev.brightness_set =  aw2013_breath_mode_set;

	aw2013_data->cdev.brightness_get = aw2013_breath_mode_get;

	aw2013_data->cdev.name = "nubia_led";

	

	
	ret = aw2013_i2c_rx_byte_data(aw2013_data->i2c_client,aw2013_regs.soft_reset,&buf);
	if(ret < 0)	{
		printk("aw2013_i2c_rx_byte_data  read chip id fail!!!\n");	
		return ret;	
		}	

	
	if(0x33 != buf){			
		printk(" the chip id is not 0x33!\n");		
		return -1;					
		}	
	else			
		printk(" the chip id  is 0x33\n");	
		

    aw2013_soft_reset(aw2013_data);
//	led_auto_breath(1,200);
//	led_const_on(2,255);
    
	ret = led_classdev_register(&client->dev, &aw2013_data->cdev);
	if (ret) {
		pr_err("unable to register breath_led ret=%d\n",ret);
		goto init_fail;
	}

     // create sys files
     
	ret = sysfs_create_group(&aw2013_data->cdev.dev->kobj,
			&aw2013_attr_group);
	if (ret)
		 goto init_fail;

	//INIT_WORK(&aw2013_data.work, aw2013_work_func);

	//hrtimer_init(&aw2013_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	//aw2013_data.timer.function = aw2013_timer;

	//aw2013_fade_data_init();
	//aw2013_control_init();
	printk("%s: finish probe:\n",__func__);
	return 0;

init_fail:
	return ret;

}
/*
static int aw2013_remove(struct i2c_client *client)
{	

	struct aw2013_control_data *aw2013_data;

	i2c_set_clientdata(client,aw2013_data);



	led_classdev_unregister(aw2013_data.cdev);
	//gpio_free(GPIO_PWDN);
	sysfs_remove_group(aw2013_data.cdev.dev->kobj, &aw2103_attr_group);
	
	return 0;
}
*/

static int aw2013_suspend(struct i2c_client *cl, pm_message_t mesg)
{
    aw2013_SUSPEND_FLAG=true;
	return 0;
};

static int aw2013_resume(struct i2c_client *cl)
{   
    aw2013_SUSPEND_FLAG=false;
	return 0;
};


static const struct i2c_device_id aw2013_id[] = {
	{ "aw2013", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27520_id);

static struct i2c_driver aw2013_driver = {
	.driver = {
		.name = "aw2013",
#ifdef CONFIG_OF
		.of_match_table = aw2013_match_table,
#endif
	},
	.id_table 	= aw2013_id,
	.probe 		= aw2013_probe,
	//.remove 	= aw2013_remove,
	.suspend	= aw2013_suspend,
	.resume 	= aw2013_resume,
};


static int __init aw2013_init(void)
{
	return i2c_add_driver(&aw2013_driver);
}

static void __exit aw2013_exit(void)
{
	i2c_del_driver(&aw2013_driver);
}


late_initcall(aw2013_init);
module_exit(aw2013_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("xuexiaojun,zhouqifang <xue.xiaojun@zte.com.cn,zhou.qifang7@zte.com.cn>");
MODULE_DESCRIPTION("aw2013 Linux driver");
MODULE_ALIAS("platform:aw2013");

