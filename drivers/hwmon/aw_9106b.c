/***********************************************************************************/
/* File Name: aw_9106b.c */
/* File Description: this file is used to make aw9106b driver to be added in kernel or module. */

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
#include "aw_9106b.h"
#include  <../../include/linux/printk.h>

static bool AW9106B_SUSPEND_FLAG=false; 
//#define GPIO_PWDN 28
#define DELAY_256MS_UNIT 1
#define DRV_NAME "class/leds/red/outn"
//#define AW_GPIO_CONFIG

#define AW_LED_DELAY_MS 650
#define TIME_MS_UNIT  1000000ULL
#define CONST_MIN_GRADE  10
#define CONST_MAX_GRADE  200
#define START_OUTN 3
#define END_OUTN 5
#define FADE_PARAM_LEN 20
#define GRADE_PARAM_LEN 20
enum aw_fade_time {
	FADE_0_MS,
	FADE_256_MS,
	FADE_512_MS,
	FADE_1024_MS,
	FADE_2048_MS,
	FADE_4096_MS,
};
enum aw_fullon_time {
	FULLON_0_MS,
	FULLON_256_MS,
	FULLON_512_MS,
	FULLON_1024_MS,
	FULLON_2048_MS,
	FULLON_4096_MS,
	FULLON_8192_MS,
	FULLON_16384_MS,
};

enum aw_fulloff_time {
	FULLOFF_0_MS,
	FULLOFF_256_MS,
	FULLOFF_512_MS,
	FULLOFF_1024_MS,
	FULLOFF_2048_MS,
	FULLOFF_4096_MS,
	FULLOFF_8192_MS,
	FULLOFF_16384_MS,
};
enum aw_max_current {
	MAX_37_MA,
	MAX_27_8__MA,
	MAX_18_5__MA,
	MAX_9_25__MA,
};

enum aw_ctl {
	AW_CTL_DISABLE,
	AW_CTL_ENABLE,
};

enum aw_reg_ctl {
	REG_BIT_CLEAR,
	REG_BIT_SET,
};

enum aw_gpio_led {
	AW_LED_MODE,
	AW_GPIO_MODE,
};

enum aw_smart_blink{
	AW_SMART_MODE,
	AW_BLINK_MODE,
};

enum aw_smart_fade{
	AW_FADE_OFF,
	AW_FADE_ON,
};

enum aw_led_close_mode{
	AW_CLOSE_NOW,
	AW_CLOSE_DELAY,
};

enum aw_power_state{
	AW_POWER_DOWN,
	AW_POWER_ON,
};

enum aw_out_val{
	AW_OUT_LOW,
	AW_OUT_HIGH,
};

enum aw_outn_mode{
	AW_POWER_OFF,// 0
	AW_CONST_ON,  // 1
	AW_CONST_OFF, // 2
	AW_FADE_AUTO, // 3 
	AW_FADE_ON_STEP,  // 4
	AW_FADE_OFF_STEP, // 5
	AW_FADE_CYCLE, // 6
	AW_RESERVED, //7
};
typedef struct {
	u16 in_mode;
	u16 dim_grade;
	u16 grade_updown;
} fade_data;

static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define AW_DBG(x...) do {if (debug_mask) pr_info("aw9106b  " x); } while (0)

static int max_current = MAX_9_25__MA;
module_param(max_current, int, 0644);

static int min_grade = CONST_MIN_GRADE;
static int max_grade = CONST_MAX_GRADE;
static int start_grade = CONST_MIN_GRADE;
static char grade_parameter[GRADE_PARAM_LEN];

static int fade_time= FADE_2048_MS;
static int fullon_time= FULLON_0_MS;
static int fulloff_time= FULLOFF_4096_MS;
static char fade_parameter[FADE_PARAM_LEN];

static int outn = 0;
static u16 aw_running = 0;
static int timer_running = 0;
static int fade_outn = 0;
fade_data fade[6];
struct aw9106b_control_data aw9106b_data;
static struct aw9106b_regs_data aw9106b_regs = 
{
	.en_bre = 0x14 ,
	.smart_blink = 0x05,
	.in_out = 0x05,
	.out_val = 0x03,
	.smart_fade = 0x03,
	.led_gpio = 0x13,
	.fade_tmr = 0x15,
	.full_tmr = 0x16,
	.delay_bre0 = 0x17,
	.dim0 = 0x20,
	.aw_reset = 0x7f,
	.ctl = 0x11,
};
#ifdef CONFIG_OF
static struct of_device_id aw_9106b_match_table[] = {
	{ .compatible = "aw,9106b", },
	{}
};
#endif

/******************************************************** 
 *					 I2C I/O function 				              *
 *********************************************************/

//read aw9106b i2c function
static int aw9106b_i2c_rx_byte_data(
		struct i2c_client *i2c,
		unsigned char  reg,
		unsigned char* buf)
{

	struct i2c_msg msgs[2];

	//write message: this is the sub address that we are reading from
	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	//read message: now we read into the buffer
	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		return -4;
	}
	pr_debug("return  buf[0]=0x%x!\n",buf[0]);

	return 0;
}

//write aw9106b i2c function
static int aw9106b_i2c_tx_byte_data(
		struct i2c_client *i2c,
		unsigned char reg, 
		unsigned char buf)
{
	struct i2c_msg msgs;
	char bufwr[2];

	bufwr[0] = reg;
	bufwr[1] = buf;

	//write message: this is the sub address that we are reading from
	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = bufwr;

	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		return -4;
	}

	return 0;
}

//modigy aw9106b i2c function
static int aw9106b_modify_regs(int reg,char bitn,enum aw_reg_ctl set)
{
	char buf = 0;
	int ret;

	ret = aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,reg,&buf);
	if(ret < 0)
		pr_err("%s: read reg[0x%x] fail!\n",__func__,reg);

	if(set == REG_BIT_SET)
		buf |= (0x01 << bitn);
	else
		buf &= ~(0x01 << bitn);

	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,reg,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);

	return ret;

}

static void aw9106b_power_set(enum aw_power_state power_set)
{
	int ret = 0;

	ret = gpio_direction_output(aw9106b_data.aw9106b_plat_data->power_ctl_gpio, power_set);
	if (ret) {
		printk(KERN_ERR"set_direction for reset gpio failed\n");
	}
    AW_DBG("aw9106b_power_set = %d,\n\r", power_set);
}

static int set_fade_time(int fade_t)
{
	char buf;
	int ret;

	buf = fade_t;
	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.fade_tmr,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.fade_tmr);

	return ret;
}

static int set_full_onoff_time(int full_on, int full_off)
{
	char buf;
	int ret;

	buf = full_on | full_off;

	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.full_tmr,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.full_tmr);
	return ret;
}
/******************************************************** 
 *                                Config AW Outn mode                                 *
 *********************************************************/

static int set_out_led_gpio(int out,enum aw_gpio_led gpmd_mode)
{
	enum aw_reg_ctl set;
	int ret;
	int reg;
	int shift;

	if( out <= AW_OUT_3 ){
		reg = aw9106b_regs.led_gpio;
		shift = out;
	}
	else{
		reg = 0x12;
		shift = out - AW_OUT_4;
	}

	if(gpmd_mode == AW_LED_MODE)
		set = REG_BIT_CLEAR;
	else
		set = REG_BIT_SET;

	ret = aw9106b_modify_regs(reg,shift,set);

	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
	return ret;
}

static int set_out_breath(int out,enum aw_ctl enable)
{
	enum aw_reg_ctl set;
	int ret;
	int reg;

	reg = aw9106b_regs.en_bre;
	if( enable == AW_CTL_DISABLE )
		set = REG_BIT_CLEAR;
	else
		set = REG_BIT_SET;

	ret = aw9106b_modify_regs(reg,out,set);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
	return ret;
}

static int set_out_smart_blink(int out,enum aw_smart_blink blink_cfg)
{
	enum aw_reg_ctl set;
	int ret;
	int reg;
	int shift;

	if( out <= AW_OUT_3 ){
		reg = aw9106b_regs.smart_blink;
		shift = out;
	}
	else{
		reg = 0x04;
		shift = out - AW_OUT_4;
	}

	if(blink_cfg == AW_SMART_MODE)
		set = REG_BIT_CLEAR;
	else
		set = REG_BIT_SET;

	ret = aw9106b_modify_regs(reg,shift,set);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
	return ret;
}

static int set_out_smart_fade(int out,enum aw_smart_fade fade_onoff)
{
	enum aw_reg_ctl set;
	int ret;
	int reg;
	int shift;

	if( out <= AW_OUT_3 ){
		reg = aw9106b_regs.smart_fade;
		shift = out;
	}
	else{
		reg = 0x02;
		shift = out - AW_OUT_4;
	}

	if(fade_onoff == AW_FADE_OFF)
		set = REG_BIT_CLEAR;
	else
		set = REG_BIT_SET;

	ret = aw9106b_modify_regs(reg,shift,set);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
	return ret;
}

static int set_out_delay_time(int out,int delay_unit)
{
	char buf;
	int ret;
	int reg;

	reg = aw9106b_regs.delay_bre0 + out;

	buf = delay_unit;

	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,reg,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
	return ret;
}

static int set_out_dim_grade(int out,char grade)
{
	char buf;
	int ret;
	int reg;

	reg = aw9106b_regs.dim0 + out;

	buf = grade;

	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,reg,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,reg);
	return ret;
}

static int set_const_current(enum aw_max_current max_i)
{
	char buf;
	int ret;

	buf = max_i;

	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.ctl,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.ctl);
	return ret;
}

static int start_blink_led(enum aw_max_current max_i)
{
	char buf;
	int ret;

	buf = max_i | 0x80;

	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.ctl,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.ctl);
	return ret;
}

static int close_out_blink_led(enum aw_led_close_mode close_mode)
{
	char buf;
	int ret;

	buf = 0;

	if(close_mode == AW_CLOSE_NOW)
		ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.en_bre,buf);
	else
		ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.smart_blink,buf);


	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.ctl);
	return ret;
}

int aw_full_fade_time_confg(int fade_t,	int full_on, int full_off)
{
	int ret;
	fade_t = (fade_t<<3) + fade_t;
	full_off = full_off<<3;
	AW_DBG("fade_t= %d, full_on=%d full_off=%d\n",fade_t,full_on,full_off);
	ret = set_fade_time(fade_t);
	if(ret < 0)
		pr_err("%s: config fade time  fail!\n",__func__);

	ret = set_full_onoff_time(full_on,full_off);
	if(ret < 0)
		pr_err("%s: config full time fail!\n",__func__);

	return ret;
}

static void aw9106b_control_init(void)
{
	outn = 0;
	aw_running = 0;
}

static void aw9106b_fade_data_init(void)
{
	int i;

	for(i=0; i<6;i++){
		fade[i].in_mode = 0;
		fade[i].dim_grade = 0;
		fade[i].grade_updown = 1;
	}
	timer_running = 0;
	fade_outn = 0;
}

static int aw9106b_solft_reset(void)
{
	char buf;
	int ret;
	buf = 0x00;

	ret = aw9106b_i2c_tx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.aw_reset,buf);
	if(ret < 0)
		pr_err("%s: write reg[0x%x] fail!\n",__func__,aw9106b_regs.aw_reset);
	return ret;
}

void enable_outn_fade_onoff(int val, int max_ma, enum aw_smart_fade fade_onoff)
{
	int rc = 0;
	int n = 0;

	for(n = START_OUTN; n <= END_OUTN; n++){
		if((outn & (1<<n))&&(fade[n].in_mode != val)){
			//store the corrent mode
			fade[n].in_mode = val;

			if(fade_onoff == AW_FADE_ON){
				rc = set_out_breath(n,AW_CTL_DISABLE);
				if(rc < 0)
					pr_err("%s: diable out[%d] breath mode fail!\n",__func__,n);
			}

			//set fade,fullon,fulloff time
			rc = aw_full_fade_time_confg(fade_time,FULLON_0_MS,FULLOFF_0_MS);
			if(rc < 0)
				pr_err("%s: config fade full on/off time fail!\n",__func__);

			//set outn led mode
			rc = set_out_led_gpio(n,AW_LED_MODE);
			if(rc < 0)
				pr_err("%s: set out[%d] LED mode fail!\n",__func__,n);

			//enable outn breath func
			rc = set_out_breath(n,AW_CTL_ENABLE);
			if(rc < 0)
				pr_err("%s: set out[%d] breath mode fail!\n",__func__,n);

			//set outn blink
			rc = set_out_smart_blink(n,AW_SMART_MODE);
			if(rc < 0)
				pr_err("%s: set out smart-fade mode fail!\n",__func__);

			//set Imax
			rc = set_const_current(max_ma);
			if(rc < 0)
				pr_err("%s: set imax fail!\n",__func__);

			//set outn delay time
			rc = set_out_delay_time(n,0*DELAY_256MS_UNIT);
			if(rc < 0)
				pr_err("%s: set out delay time fail!\n",__func__);

			//set outn smart fade
			rc = set_out_smart_fade(n,fade_onoff);
			if(rc < 0)
				pr_err("%s: start samrt-fade mode fail!\n",__func__);
		}
		outn &= ~(1<<n);
		AW_DBG("fade_onoff outn= 0x%x\n",outn);
	}
}


void enable_outn_const_led(int val,int max_ma, char grade)

{

	int rc = 0;
	int n = 0;

	for(n = START_OUTN; n <= END_OUTN; n++){
		if((outn & (1<<n)&&(fade[n].in_mode != val))){
			//store the corrent mode
			fade[n].in_mode = val;

			//disable breathing mode
			rc = set_out_breath(n,AW_CTL_DISABLE);
			if(rc < 0)
				pr_err("%s: set out[%d] breath mode fail!\n",__func__,n);

			//set outn led mode
			rc = set_out_led_gpio(n,AW_LED_MODE);
			if(rc < 0)
				pr_err("%s: set out[%d] LED mode fail!\n",__func__,n);

			//set Imax
			rc = set_const_current(max_ma);
			if(rc < 0)
				pr_err("%s: set imax fail!\n",__func__);

			//set const current = (grade/256)*imax
			rc = set_out_dim_grade(n,grade);
			if(rc < 0)
				pr_err("%s: set out[%d] dim grade fail!\n",__func__,n);
		}

		//clean the corresponding bit of outn
		outn &= ~(1<<n);
		AW_DBG("const_led outn= 0x%x\n",outn);
	}
}


void enable_outn_blink_led(int val, int max_ma)

{
	int rc = 0;
	int n = 0;

	aw9106b_fade_data_init();
	rc = aw9106b_solft_reset();
	if(rc < 0)
		pr_err("%s: solft reset fail!\n",__func__);

	for(n = START_OUTN; n <= END_OUTN; n++){
		if((outn & (1<<n))&&(fade[n].in_mode != val)){
			//store the corrent mode
			fade[n].in_mode = val;

			//set fade,fullon,fulloff time
			rc = aw_full_fade_time_confg(fade_time,fullon_time,fulloff_time);
			if(rc < 0)
				pr_err("%s: config fade full on/off time fail!\n",__func__);

			//set outn led mode
			rc = set_out_led_gpio(n,AW_LED_MODE);
			if(rc < 0)
				pr_err("%s: set out[%d] LED mode fail!\n",__func__,n);

			//enable outn breath func
			rc = set_out_breath(n,AW_CTL_ENABLE);
			if(rc < 0)
				pr_err("%s: set out[%d] breath mode fail!\n",__func__,n);

			//set outn blink
			rc = set_out_smart_blink(n,AW_BLINK_MODE);
			if(rc < 0)
				pr_err("%s: set out[%d] blink mode fail!\n",__func__,n);

			//set outn delay time
			rc = set_out_delay_time(n,0*DELAY_256MS_UNIT);
			if(rc < 0)
				pr_err("%s: set out[%d] delay time fail!\n",__func__,n);
		}

		//clean the corresponding bit of outn
		outn &= ~(1<<n);
		AW_DBG("blink_led outn= 0x%x\n",outn);
	}

	//start outn blink led
	rc |= start_blink_led(max_ma);
	if(rc < 0)
		pr_err("%s: start blink fail!\n",__func__);
}



static void aw9106b_work_func(struct work_struct *work)

{

	int rc;
	int n;

	for(n = START_OUTN; n <= END_OUTN; n++){
		if(fade_outn & (1<<n)){
			if(fade[n].in_mode != AW_FADE_CYCLE){
				//first time to set fade mode for this channel,disable breathing mode
				rc = set_out_breath(n,AW_CTL_DISABLE);

				//set outn led mode
				rc |= set_out_led_gpio(n,AW_LED_MODE);

				//set Imax
				rc |= set_const_current(max_current);
				if(rc < 0)
					pr_err("%s: set imax fail!\n",__func__);

				fade[n].in_mode = AW_FADE_CYCLE;
				fade[n].dim_grade = min_grade;
				fade[n].grade_updown = 1;
				timer_running |= (1<<n);
				AW_DBG("work_func:timer_running = 0x%x\n", timer_running);
			}

			if(fade[n].dim_grade == 0)
				fade[n].dim_grade = start_grade;

			if(fade[n].grade_updown == 1 && fade[n].dim_grade < max_grade)
				fade[n].dim_grade++;
			else if(fade[n].grade_updown == 2 && fade[n].dim_grade>min_grade)
				fade[n].dim_grade--;

			rc = set_out_dim_grade(n,fade[n].dim_grade);
			if(rc < 0){
				fade[n].in_mode = AW_CONST_ON;
				fade[n].dim_grade = 0;
				fade[n].grade_updown = 1;
				timer_running &=~(1<<n);
				fade_outn &=~(1<<n);
				pr_err("%s: set out dim grade fail!\n",__func__);
				return;
			}

			if(fade[n].grade_updown == 1 && fade[n].dim_grade>=max_grade)
				fade[n].grade_updown = 2;

			else if(fade[n].grade_updown == 2 && fade[n].dim_grade <= min_grade){
				//fade cycle finish
				fade[n].dim_grade = 0;
				fade[n].in_mode = AW_CONST_ON;
				fade[n].grade_updown = 1;
				timer_running &=~(1<<n);
				fade_outn &=~(1<<n);
				AW_DBG("work_func:timer_running = 0x%x, fade_outn = 0x%x\n",timer_running, fade_outn);
				AW_DBG("%s: end breath!\n",__func__);
			}

			if((timer_running & 0x3f) !=0)
				hrtimer_start(&aw9106b_data.timer,ktime_set(0,1*TIME_MS_UNIT),HRTIMER_MODE_REL);
		}
	}
}


static enum hrtimer_restart aw9106b_timer(struct hrtimer *timer)

{
	schedule_work(&aw9106b_data.work);
	return HRTIMER_NORESTART;
}

/*********************       aw9106b_breath_mode_set    *********************/

void aw9106b_breath_mode_set(struct led_classdev *led_cdev,

		enum led_brightness brightness)

{
	int val = brightness;
	int rc = 0;
	if(AW9106B_SUSPEND_FLAG==true)return;

	//gpio_tlmm_config(GPIO_CFG(GPIO_PWDN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	AW_DBG("val=%d fade_outn= 0x%x, outn= 0x%x \n",val,fade_outn, outn);

	if (aw_running == 0 && val != AW_POWER_OFF) { 
		aw9106b_power_set(AW_POWER_ON);
		rc = aw9106b_solft_reset();
		if(rc < 0)
			pr_err("%s: solft reset fail!\n",__func__);
		aw_running = 1;
	}

	switch (val) {
		case AW_POWER_OFF:
			aw9106b_fade_data_init();
			aw9106b_control_init();
			aw9106b_power_set(AW_POWER_DOWN);
			break;

		case AW_CONST_ON: 
			if((outn == 0x08)||(outn == 0x20))
				//enable_outn_const_led(val, max_current, min_grade + 100);
				enable_outn_const_led(val, max_current, 5*min_grade);
			else
				enable_outn_const_led(val, max_current, min_grade);
			break;

		case AW_CONST_OFF:
			enable_outn_const_led(val, max_current, 0);
			break;

			//blink breath mode
		case AW_FADE_AUTO:
			enable_outn_blink_led(val, max_current);
			break;

			//smart breath mode
		case AW_FADE_ON_STEP:
			enable_outn_fade_onoff(val, max_current,AW_FADE_ON);
			break;

		case AW_FADE_OFF_STEP:
			enable_outn_fade_onoff(val,max_current,AW_FADE_OFF);
			break;

			//fade 1 cycle for press home key
		case AW_FADE_CYCLE:
			fade_outn |= outn;
			if((timer_running & 0x3f) == 0){
				AW_DBG("timer_running = 0x%x, start fade mode.\n",timer_running);
				hrtimer_start(&aw9106b_data.timer,ktime_set(0,0),HRTIMER_MODE_REL);
			}
			break;

		case AW_RESERVED:
			break;

		default:
			break;

	}
}

EXPORT_SYMBOL_GPL(aw9106b_breath_mode_set);


static void aw9106b_show_regs(void)

{
	char buf[1];

	//read EN_BRE
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.en_bre,buf);
	printk("read  en_bre[0x%x]= 0x%x\n",aw9106b_regs.en_bre,buf[0]);

	//set led_gpio 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.led_gpio,buf);
	printk("read  led_gpio[0x%x] = 0x%x\n",aw9106b_regs.led_gpio,buf[0]);

	//set BLINK 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.smart_blink,buf);
	printk("read  smart_blink[0x%x] = 0x%x\n",aw9106b_regs.smart_blink,buf[0]);

	//set delay time 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.delay_bre0,buf);
	printk("read  delay_bre0[0x%x] = 0x%x\n",aw9106b_regs.delay_bre0,buf[0]);

	//set flade time 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.fade_tmr,buf);
	printk("read  fade_tmr[0x%x] = 0x%x\n",aw9106b_regs.fade_tmr,buf[0]);

	//set full on and off  time 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.full_tmr,buf);
	printk("read  full_tmr[0x%x] = 0x%x\n",aw9106b_regs.full_tmr,buf[0]);

	//set gpio in and out 
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.in_out,buf);
	printk("read  in_out[0x%x] = 0x%x\n",aw9106b_regs.in_out,buf[0]);

	//start
	buf[0] = 0x0;
	aw9106b_i2c_rx_byte_data(aw9106b_data.i2c_client,aw9106b_regs.ctl,buf);
	printk("read  ctl[0x%x] = 0x%x\n",aw9106b_regs.ctl,buf[0]);
}

//fade_parameter
static ssize_t show_fade_parameter(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	snprintf(fade_parameter, FADE_PARAM_LEN,	"%4d %4d %4d\n",
			fade_time, fullon_time, fulloff_time);

	return sprintf(buf, "%s\n", fade_parameter);
}

static ssize_t set_fade_parameter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	strncpy(fade_parameter, buf, FADE_PARAM_LEN);
	sscanf(fade_parameter, "%d %d %d", &fade_time, &fullon_time, &fulloff_time);
	AW_DBG("fade_time= %d, fullon_time=%d fulloff_time=%d\n",fade_time,fullon_time,fulloff_time);

	return count;
}

//path: sys/class/leds/red/
const static DEVICE_ATTR(fade_parameter, S_IRUGO | S_IWUSR,
		show_fade_parameter, set_fade_parameter);

//grade_parameter
static ssize_t show_grade_parameter(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	snprintf(grade_parameter, GRADE_PARAM_LEN,	"%4d %4d\n",
			min_grade, max_grade);

	return sprintf(buf, "%s\n", grade_parameter);
}

static ssize_t set_grade_parameter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	strncpy(grade_parameter, buf, GRADE_PARAM_LEN);
	sscanf(grade_parameter, "%d %d", &min_grade, &max_grade);
	AW_DBG("min_grade= %d, max_grade=%d \n",min_grade,max_grade);

	return count;
}
//path: sys/class/leds/red/
const static DEVICE_ATTR(grade_parameter, S_IRUGO | S_IWUSR,
		show_grade_parameter, set_grade_parameter);


//out_n
static ssize_t show_outn(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d\n", outn);
}

static ssize_t set_outn(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	sscanf(buf, "%d", &outn);
	AW_DBG("outn= %d \n",outn);
	return count;
}
//path: sys/class/leds/red/
const static DEVICE_ATTR(outn, S_IRUGO | S_IWUSR,
		show_outn, set_outn);

static int led_config;
static int set_led_mode(const char *val, struct kernel_param *kp)

{
	int ret;
	int rc = 0;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	printk("__%s: led_config=%d!\n",__func__,led_config);

	switch(led_config){
		case 0:
			aw9106b_power_set(AW_POWER_DOWN);
			break;

		case 1:
			outn = AW_OUT_0;
			break;

		case 2:
			outn = AW_OUT_1;
			break;

		case 3:
			aw9106b_show_regs();
			break;

		case 4:
			enable_outn_const_led(AW_CONST_ON, MAX_18_5__MA,min_grade);
			break;

		case 5:
			enable_outn_blink_led(AW_FADE_AUTO,max_current);
			rc |= start_blink_led(max_current);
			if(rc < 0)
				pr_err("%s: start blink fail!\n",__func__);
			break;

		case 6:
			close_out_blink_led(AW_CLOSE_NOW);
			break;

		case 7:
			close_out_blink_led(AW_CLOSE_DELAY);
			break;

		default:
			break;
	};
	return 0;
}

module_param_call(led_config, set_led_mode, param_get_uint,
		&led_config, 0644);


static struct led_classdev breath_led = {
	.name		= "red",
	.brightness_set	= aw9106b_breath_mode_set,
};


#ifdef CONFIG_OF
static int aw9106b_parse_dt(struct device *dev,
			struct aw9106b_plat_data *pdata)
{
	//int rc;
	struct device_node *np = dev->of_node;

/*
	pdata->name = "aw9106b_led";
	rc = of_property_read_string(np, "aw9106b_led,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}
*/
	/* reset, gpio info */
	pdata->power_ctl_gpio = of_get_named_gpio_flags(np, "aw9106b_led,power-gpio",
				0, &pdata->power_ctl_gpio_flags);
	if (pdata->power_ctl_gpio < 0)
		return pdata->power_ctl_gpio;
	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif


static int aw9106b_pinctrl_init(struct aw9106b_plat_data *aw9106b_plat_data,struct i2c_client *i2c_client)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	aw9106b_plat_data->aw9106b_led_pinctrl = devm_pinctrl_get(&(i2c_client->dev));
	if (IS_ERR_OR_NULL(aw9106b_plat_data->aw9106b_led_pinctrl)) {
		retval = PTR_ERR(aw9106b_plat_data->aw9106b_led_pinctrl);
		aw9106b_plat_data->aw9106b_led_pinctrl = NULL;
		return retval;
	}
	aw9106b_plat_data->gpio_power_active
		= pinctrl_lookup_state(aw9106b_plat_data->aw9106b_led_pinctrl,
			"aw9106b_active");
	if (IS_ERR_OR_NULL(aw9106b_plat_data->gpio_power_active)) {
		retval = PTR_ERR(aw9106b_plat_data->gpio_power_active);
		aw9106b_plat_data->aw9106b_led_pinctrl = NULL;
		return retval;
	}

	aw9106b_plat_data->gpio_power_supend
		= pinctrl_lookup_state(aw9106b_plat_data->aw9106b_led_pinctrl,
			"aw9106b_suspend");
	printk("zqf debug: kernel*****aw9106b_pinctrl_init passed !\n");
	if (IS_ERR_OR_NULL(aw9106b_plat_data->gpio_power_supend)) {
		retval = PTR_ERR(aw9106b_plat_data->gpio_power_supend);
		aw9106b_plat_data->aw9106b_led_pinctrl = NULL;
		return retval;
	}
	return 0;
}

static int aw9106b_pinctrl_select(struct aw9106b_plat_data *aw9106b_plat_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? aw9106b_plat_data->gpio_power_active
		: aw9106b_plat_data->gpio_power_supend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(aw9106b_plat_data->aw9106b_led_pinctrl, pins_state);
		if (ret) {
			return ret;
		}
	} 
	return 0;
}


static int  aw9106b_probe(struct i2c_client *client,

		const struct i2c_device_id *dev_id)

{
	int ret = 0;
	
	struct aw9106b_plat_data *pdata =
	  (struct aw9106b_plat_data *)client->dev.platform_data;

	printk("%s: start probe:\n",__func__);

	// xuexiaojun add for dts
	if (client->dev.of_node) {
		printk("[aw9106b]function:%s,client->dev.of_node\n",__func__);
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct aw9106b_plat_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		
		ret = aw9106b_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "DT parsing failed\n");
			return ret;
				}
			}
	//end add 
    aw9106b_data.aw9106b_plat_data=pdata;
	aw9106b_data.i2c_client = client;

	 ret = aw9106b_pinctrl_init(pdata,client);
	 if (!ret && pdata->aw9106b_led_pinctrl) {
		ret = aw9106b_pinctrl_select(pdata, true);
		if (ret < 0)
       return ret;
     }
	if (gpio_is_valid(pdata->power_ctl_gpio)) {
		ret = gpio_request(pdata->power_ctl_gpio, "aw9106b_power_gpio");
		if (ret) {
			dev_err(&client->dev, "power gpio request failed");
			return ret;
		}

		ret = gpio_direction_output(pdata->power_ctl_gpio, 1);
		if (ret) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
             return ret;
		}
		msleep(1000);
		gpio_set_value_cansleep(pdata->power_ctl_gpio, 1);
	}

	ret = led_classdev_register(NULL, &breath_led);
	if (ret) {
		pr_err("unable to register breath_led ret=%d\n",ret);
		goto init_fail;
	}

	ret = device_create_file(breath_led.dev, &dev_attr_fade_parameter);
	if (unlikely(ret < 0)) {
		dev_err(breath_led.dev, "failed: cannot create fade_parameter.\n");
	}

	ret = device_create_file(breath_led.dev, &dev_attr_grade_parameter);
	if (unlikely(ret < 0)) {
		dev_err(breath_led.dev, "failed: cannot create grade_parameter.\n");
	}

	ret = device_create_file(breath_led.dev, &dev_attr_outn);
	if (unlikely(ret < 0)) {
		dev_err(breath_led.dev, "failed: cannot create outn.\n");
	}

	INIT_WORK(&aw9106b_data.work, aw9106b_work_func);

	hrtimer_init(&aw9106b_data.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw9106b_data.timer.function = aw9106b_timer;

	aw9106b_fade_data_init();
	aw9106b_control_init();
	printk("%s: finish probe:\n",__func__);
	return 0;

init_fail:
	return ret;

}

static int aw9106b_remove(struct i2c_client *client)
{	
	led_classdev_unregister(&breath_led);
	//gpio_free(GPIO_PWDN);
	return 0;
}

static int aw9106b_suspend(struct i2c_client *cl, pm_message_t mesg)
{
    AW9106B_SUSPEND_FLAG=true;
	return 0;
};

static int aw9106b_resume(struct i2c_client *cl)
{   
    AW9106B_SUSPEND_FLAG=false;
	return 0;
};


static const struct i2c_device_id aw9106b_id[] = {
	{ "aw_9106b", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27520_id);

static struct i2c_driver aw9106b_driver = {
	.driver = {
		.name = "aw_9106b",
#ifdef CONFIG_OF
		.of_match_table = aw_9106b_match_table,
#endif
	},
	.id_table 	= aw9106b_id,
	.probe 		= aw9106b_probe,
	.remove 	= aw9106b_remove,
	.suspend	= aw9106b_suspend,
	.resume 	= aw9106b_resume,
};


static int __init aw9106b_init(void)
{
	printk( "%s:enter...\n", __func__);
	return i2c_add_driver(&aw9106b_driver);
}

static void __exit aw9106b_exit(void)
{
	printk( "%s:%d:aw9106b is exiting\n", __func__,__LINE__);
	i2c_del_driver(&aw9106b_driver);
}


late_initcall(aw9106b_init);
module_exit(aw9106b_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("xuexiaojun,wangshuai <xue.xiaojun@zte.com.cn,wang.shuai@>");
MODULE_DESCRIPTION("aw9106b Linux driver");
MODULE_ALIAS("platform:aw9106b");

