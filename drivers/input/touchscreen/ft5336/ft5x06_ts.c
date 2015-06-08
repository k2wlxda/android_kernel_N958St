/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
//#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>

//#include <mach/gpio.h>
//#include <mach/map.h>
//#include <mach/regs-clock.h>
//#include <mach/regs-gpio.h>
//#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "ft5x06_ts.h"
//#define FT5336_DOWNLOAD
#define SYSFS_DEBUG
#define FTS_APK_DEBUG
#define FTS_CTL_IIC  

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS 3
#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

//+++duguowei
#define HAVE_TOUCH_KEY 1
#define FT_PROTOCAL_A 1

#define MAX_KEY_NUM 3
u32 button_map[MAX_KEY_NUM];//map dts key map
u32 KEY_MAP[MAX_KEY_NUM] = {KEY_MENU,KEY_HOME,KEY_BACK};//map:button_map

#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2


#define KEY_STEP 50
#define KEY_Y_AVE 1350
#define KEY_Y_MIN (KEY_Y_AVE - KEY_STEP)
#define KEY_Y_MAX (KEY_Y_AVE + KEY_STEP)

//menu
#define KEY_MENU_X_AVE 160
#define KEY_MENU_X_MIN (KEY_MENU_X_AVE - KEY_STEP)
#define KEY_MENU_X_MAX (KEY_MENU_X_AVE + KEY_STEP)

//home
#define KEY_HOME_X_AVE 360
#define KEY_HOME_X_MIN (KEY_HOME_X_AVE - KEY_STEP)
#define KEY_HOME_X_MAX (KEY_HOME_X_AVE + KEY_STEP)
#define KEY_HOME_Y_AVE 1350
#define KEY_HOME_Y_MIN (KEY_HOME_Y_AVE - KEY_STEP)
#define KEY_HOME_Y_MAX (KEY_HOME_Y_AVE + KEY_STEP)

//back
#define KEY_BACK_X_AVE 580
#define KEY_BACK_X_MIN (KEY_BACK_X_AVE - KEY_STEP)
#define KEY_BACK_X_MAX (KEY_BACK_X_AVE + KEY_STEP)
#define KEY_BACK_Y_AVE 1350
#define KEY_BACK_Y_MIN (KEY_BACK_Y_AVE - KEY_STEP)
#define KEY_BACK_Y_MAX (KEY_BACK_Y_AVE + KEY_STEP)


#define FT_KEY_Y_CHECK(y)	\
	((y) >= KEY_Y_MIN && (y) <= KEY_Y_MAX)

#define FT_KEY_MENU_CHECK(x,y)	\
	((x) >= KEY_MENU_X_MIN && (x) <= KEY_MENU_X_MAX && (y) >= KEY_Y_MIN && (y) <= KEY_Y_MAX)
#define FT_KEY_HOME_CHECK(x,y)	\
	((x) >= KEY_HOME_X_MIN && (x) <= KEY_HOME_X_MAX && (y) >= KEY_Y_MIN && (y) <= KEY_Y_MAX)
#define FT_KEY_BACK_CHECK(x,y)	\
	((x) >= KEY_BACK_X_MIN && (x) <= KEY_BACK_X_MAX && (y) >= KEY_Y_MIN && (y) <= KEY_Y_MAX)
//---duguowei

static struct i2c_client *g_i2c_client = NULL;

#ifdef FT5336_DOWNLOAD
#include "ft5336_download_lib.h"

static unsigned char CTPM_MAIN_FW[]=
{
	#include "ft5336_all.i"
};
#endif

#ifdef SYSFS_DEBUG
#include "ft5x06_ex_fun.h"
#endif

//+++duguowei

#ifdef CONFIG_USB
static int usb_plug_status=0;
#endif


#ifdef CONFIG_PM_SLEEP
struct ft5x0x_ts_data *ft_ts_data;
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

//---duguowei

/*register address*/
#define FT5336_REG_IC_TYPE	0xA3
#define FT5336_REG_PMODE	0xA5
#define FT5336_REG_FW_VER	0xA6
#define FT5336_REG_POINT_RATE	0x88
#define FT5336_REG_THGROUP	0x80
#define FT5336_REG_FACTORY_ID		0xA8

/* power register bits*/
#define FT5336_PMODE_ACTIVE		0x00
#define FT5336_PMODE_MONITOR		0x01
#define FT5336_PMODE_STANDBY		0x02
#define FT5336_PMODE_HIBERNATE		0x03


struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft5x0x_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ft5x06_ts_platform_data *pdata;
	#ifdef CONFIG_PM
	struct early_suspend *early_suspend;
	#endif
	struct regulator *vdd;
	struct regulator *vcc_i2c;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
//+++duguowei,pinctrl
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
//---duguowei,pinctrl
};

#define ANDROID_INPUT_PROTOCOL_B

#define FT5X0X_RESET_PIN	12//S5PV210_GPB(2)
#define FT5X0X_RESET_PIN_NAME	"ft5x0x-reset"
#define FT5X0X_INT_PIN	13//S5PV210_GPB(2)
#define FT5X0X_INT_PIN_NAME	"ft5x0x-int"

bool hassleep = 0;

#ifdef CONFIG_PM_SLEEP

static int focaltech_ts_event(struct notifier_block *this, unsigned long event,void *ptr)
{
	int ret;

	switch(event)
	{
	case 0:
		//offline
		if ( usb_plug_status != 0 ){
	 		usb_plug_status = 0;
			printk("[ft5336]focaltech ts config change to offline status\n");
			i2c_smbus_write_byte_data( ft_ts_data->client, 0x8b,0x0);
		}
		break;
	case 1:
		//online
		if ( usb_plug_status != 1 ){
	 		usb_plug_status = 1;
			printk("[ft5336]focaltech ts config change to online status\n");
			i2c_smbus_write_byte_data( ft_ts_data->client, 0x8b,0x1);
		}
		break;
	default:
		printk("focaltech ts config change to other status action is %lu\n",event);
		break;
	}

	ret = NOTIFY_DONE;

	return ret;
}

static struct notifier_block ts_notifier = {
	.notifier_call = focaltech_ts_event,
};


static BLOCKING_NOTIFIER_HEAD(ts_chain_head);

int focaltech_register_ts_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ts_chain_head, nb);
}
EXPORT_SYMBOL_GPL(focaltech_register_ts_notifier);

int focaltech_unregister_ts_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ts_chain_head, nb);
}
EXPORT_SYMBOL_GPL(focaltech_unregister_ts_notifier);

int Ft5x0x_ts_notifier_call_chain(unsigned long val)
{
	printk("[ft5336]Ft5x0x_ts_notifier_call_chain,val = %lu\n",val);
	return (blocking_notifier_call_chain(&ts_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}
EXPORT_SYMBOL_GPL(Ft5x0x_ts_notifier_call_chain);
#endif



/*
*ft5x0x_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}
#ifdef FT5336_DOWNLOAD
int ft5x0x_download_i2c_Read(unsigned char *writebuf,
		    int writelen, unsigned char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = 0x38,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = 0x38,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&g_i2c_client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = 0x38,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&g_i2c_client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft5x0x_download_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = 0x38,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(g_i2c_client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&g_i2c_client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

#endif

//#ifdef FTS_SCAP_TEST
int focal_i2c_Read(unsigned char *writebuf,
		    int writelen, unsigned char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = g_i2c_client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = g_i2c_client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&g_i2c_client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = g_i2c_client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(g_i2c_client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&g_i2c_client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int focal_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = g_i2c_client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(g_i2c_client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&g_i2c_client->dev, "%s i2c write error.\n", __func__);

	return ret;
}
//#endif


/*Read touch point information when the interrupt  is asserted.*/
static int ft5x0x_read_Touchdata(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;
	u8 buf[POINT_READ_BUF] = {0};
	/*luochangyang For wakeup gesture 2014/04/29*/
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	/*luochangyang END*/

	ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	/*luochangyang For wakeup gesture 2014/04/29*/
	if (data->pdata->wakeup_gesture == 1) {
		uc_reg_addr = FT5x0x_REG_GEST_ONOFF;
		ft5x0x_i2c_Read(data->client, &uc_reg_addr, 1, &uc_reg_value, 1);

		if(uc_reg_value)
		{
			uc_reg_addr = 0xD3;
			ft5x0x_i2c_Read(data->client, &uc_reg_addr, 1, &uc_reg_value, 1);
			/* if */
			if(uc_reg_value == 0x24)
			{
				printk("[ft5x06] wakeup gesture....\n");
#if 1
				input_report_key(data->input_dev, KEY_F10, 1);
			    input_sync(data->input_dev);

			    input_report_key(data->input_dev, KEY_F10, 0);
			    input_sync(data->input_dev);
#else
				input_report_key(data->input_dev, KEY_POWER, 1);
			    input_sync(data->input_dev);

			    input_report_key(data->input_dev, KEY_POWER, 0);
			    input_sync(data->input_dev);
#endif
				return 1;
			}
		}
	}
	
	if(buf[1]==0x03)
	{
		//检测到 大面积触摸 休眠并通知host
		if(hassleep)
			return 1;
		hassleep = 1;
		printk("[ft5x06] palm to sleep....\n");

#if 1
		input_report_key(data->input_dev, BTN_TOUCH, 1);
		input_report_abs(data->input_dev, ABS_MT_PRESSURE,1000);
		input_mt_sync(data->input_dev);
		input_sync(data->input_dev);

		input_report_key(data->input_dev, BTN_TOUCH, 0);
		input_sync(data->input_dev);
#else
		input_report_key(data->input_dev, KEY_POWER, 1);
		input_sync(data->input_dev);

		input_report_key(data->input_dev, KEY_POWER, 0);
		input_sync(data->input_dev);
#endif
		
		return 1;
	}
	/*luochangyang END*/

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		#if 0
		pr_info("id=%d event=%d x=%d y=%d\n", event->au8_finger_id[i],
			event->au8_touch_event[i], event->au16_x[i], event->au16_y[i]);
		#endif
	}

	event->pressure = FT_PRESS;

	return 0;
}

/*
*report the point information
*/
static void ft5x0x_report_value(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;
	bool update_input = false;
	int x,y;

#if FT_PROTOCAL_A
	/*protocol A*/
	//printk("[ft5x06] event->touch_point.....  = %d\n",event->touch_point);
	for (i = 0; i < event->touch_point; i++) 
	{
		/* LCD view area */
		//if ( (event->au16_x[i] < data->x_max)&&(event->au16_y[i] < data->y_max)) 
		//{
			input_report_key(data->input_dev, BTN_TOUCH, 1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,event->au16_y[i]);
			
			x = event->au16_x[i];
			y = event->au16_y[i];
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,event->au8_finger_id[i]);
			if ((event->au8_touch_event[i] == FT_TOUCH_DOWN) || (event->au8_touch_event[i] == FT_TOUCH_CONTACT))
			{
				input_report_abs(data->input_dev,ABS_MT_TOUCH_MAJOR,event->pressure);
				input_report_abs(data->input_dev,ABS_MT_WIDTH_MAJOR,event->pressure);
				input_report_abs(data->input_dev, ABS_MT_PRESSURE,event->pressure);
			}
			else 
			{
				input_report_abs(data->input_dev,ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(data->input_dev,ABS_MT_WIDTH_MAJOR, 0);
				input_report_abs(data->input_dev, ABS_MT_PRESSURE,0);
				//input_report_key(data->input_dev, BTN_TOUCH, 0);
				uppoint++;
			}
		//}

		input_mt_sync(data->input_dev);
	}
	//input_sync(data->input_dev);

	#if HAVE_TOUCH_KEY
		if(FT_KEY_Y_CHECK(y))
		{
			printk("[ft5x06] FT_KEY_Y_CHECK....\n");
			update_input = false;
			if(FT_KEY_MENU_CHECK(x,y)){
				printk("[ft5x06] menu key pressed.....\n");
				if(event->touch_point == uppoint){
					printk("[ft5x06] menu key pressed0.....\n");
					input_report_key(data->input_dev, KEY_MAP[0], 0);
				}else{
					printk("[ft5x06] menu key pressed1.....\n");
					input_report_key(data->input_dev, KEY_MAP[0], 1);
				}
			}else if (FT_KEY_HOME_CHECK(x,y)){
				if(event->touch_point == uppoint)
					input_report_key(data->input_dev, KEY_MAP[1], 0);
				else
					input_report_key(data->input_dev, KEY_MAP[1], event->touch_point > 0);
			}else if(FT_KEY_BACK_CHECK(x,y)){
				if(event->touch_point == uppoint)
					input_report_key(data->input_dev, KEY_MAP[2], 0);
				else
					input_report_key(data->input_dev, KEY_MAP[2], event->touch_point > 0);
			}else{
				//other region,report nothing
				printk("[ft5x06] no key pressed..... x = %d\n",x);
			}
			//input_sync(data->input_dev);
		}
	#endif

	if (event->touch_point == uppoint)
	{
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_key(data->input_dev, BTN_TOUCH, 0);		
	}
	else
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	
	input_sync(data->input_dev);
#else
	/*protocol B*/	
	for (i = 0; i < event->touch_point; i++)
	{
		update_input = true;
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);
		
		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
		{
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
				true);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					event->au16_x[i]);
			x = event->au16_x[i];
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					event->au16_y[i]);
			y = event->au16_y[i];
		}
		else
		{
			uppoint++;
			x = event->au16_x[i];
			y = event->au16_y[i];
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
				false);
		}
		input_sync(data->input_dev);
	}


	#if HAVE_TOUCH_KEY
		if(FT_KEY_Y_CHECK(y))
		{
			update_input = false;
			if(FT_KEY_MENU_CHECK(x,y)){
				printk("[ft5x06] menu key pressed.....\n");
				if(event->touch_point == uppoint){
					printk("[ft5x06] menu key pressed0.....\n");
					input_report_key(data->input_dev, KEY_MAP[0], 0);
				}else{
					input_report_key(data->input_dev, KEY_MAP[0], 1);
				}
			}else if (FT_KEY_HOME_CHECK(x,y)){
				if(event->touch_point == uppoint)
					input_report_key(data->input_dev, KEY_MAP[1], 0);
				else
					input_report_key(data->input_dev, KEY_MAP[1], event->touch_point > 0);
			}else if(FT_KEY_BACK_CHECK(x,y)){
				if(event->touch_point == uppoint)
					input_report_key(data->input_dev, KEY_MAP[2], 0);
				else
					input_report_key(data->input_dev, KEY_MAP[2], event->touch_point > 0);
			}else{
				//other region,report nothing
				printk("[ft5x06] no key pressed..... x = %d\n",x);
			}
			input_sync(data->input_dev);
		}
	#endif
	if(update_input){
		if(event->touch_point == uppoint)
				input_report_key(data->input_dev, BTN_TOUCH, 0);
		else
				input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
		input_sync(data->input_dev);
	}
#endif

}

/*The ft5x0x device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	int ret = 0;
	disable_irq_nosync(ft5x0x_ts->irq);

	//printk("ft5x0x_ts_interrupt interrupt is coming\n");
	ret = ft5x0x_read_Touchdata(ft5x0x_ts);
	if (ret == 0)
		ft5x0x_report_value(ft5x0x_ts);

	enable_irq(ft5x0x_ts->irq);

	return IRQ_HANDLED;
}

void ft5x0x_reset_tp(int HighOrLow)
{
	pr_info("set tp reset pin to %d\n", HighOrLow);
	gpio_set_value(FT5X0X_RESET_PIN, HighOrLow);
}

void ft5x0x_Enable_IRQ(struct i2c_client *client, int enable)
{
	//if (FT5X0X_ENABLE_IRQ == enable)
	//if (FT5X0X_ENABLE_IRQ)
		//enable_irq(client->irq);
	//else
		//disable_irq_nosync(client->irq);
}

#if 0
static int fts_init_gpio_hw(struct ft5x0x_ts_data *ft5x0x_ts)
{

	int ret = 0;
	int i = 0;
	
	ret = gpio_request(FT5X0X_RESET_PIN, FT5X0X_RESET_PIN_NAME);
	if (ret) {
		pr_err("%s: request GPIO %s for reset failed, ret = %d\n",
				__func__, FT5X0X_RESET_PIN_NAME, ret);
		return ret;
	}
	s3c_gpio_cfgpin(FT5X0X_RESET_PIN, S3C_GPIO_OUTPUT);
	gpio_set_value(FT5X0X_RESET_PIN, 1);

	return ret;
}

static void fts_un_init_gpio_hw(struct ft5x0x_ts_data *ft5x0x_ts)
{
	gpio_free(FT5X0X_RESET_PIN);
}
#endif
#ifdef FT5336_DOWNLOAD

int ft5336_Enter_Debug(void)
{
	ft5x0x_reset_tp(0);
	msleep(4);
	ft5x0x_reset_tp(1);
	return ft5336_Lib_Enter_Download_Mode();
}
//if return 0, main flash is ok, else download.
int ft5336_IsDownloadMain(void)
{
	//add condition to check main flash
	return -1;
}
int ft5336_DownloadMain(void)
{
	unsigned short fwlen = 0;
	if (ft5336_Enter_Debug() < 0) {
		pr_err("-----enter debug mode failed\n");
		return -1;
	}
	fwlen = sizeof(CTPM_MAIN_FW);
	pr_info("----fwlen=%d\n", fwlen);

	//return ft6x06_Lib_DownloadMain(CTPM_MAIN_FW, fwlen);
	return ft5336_Lib_DownloadMain(CTPM_MAIN_FW, fwlen);
}
#endif

/*luochangyang 2014/04/30*/
static ssize_t ztemt_wakeup_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);
	ssize_t ret;

	pr_info("----pdata->wakeup_gesture =%d\n", ts->pdata->wakeup_gesture);
	
	ret = sprintf(buf, "0x%02X\n", ts->pdata->wakeup_gesture);

	return ret;
}

static ssize_t ztemt_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value > 0xFF && value < 0)
		return -EINVAL;

	ts->pdata->wakeup_gesture = (u8)value;

	return size;
}

static DEVICE_ATTR(wakeup_gesture, 0664, ztemt_wakeup_gesture_show, ztemt_wakeup_gesture_store);
/*luochangyang END*/

static int ft5x06_power_on(struct ft5x0x_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
	}

	return rc;
}

static int ft5x06_power_init(struct ft5x0x_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}


#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	//u32 button_map[MAX_BUTTONS];

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

extern int fts_ctpm_auto_upgrade(struct i2c_client * client);

//+++duguowei,pinctrl
static int ft5x06_ts_pinctrl_init(struct ft5x0x_ts_data *ft5x06_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		dev_dbg(&ft5x06_data->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	ft5x06_data->gpio_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			"pmx_ts_active");
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_active)) {
		dev_dbg(&ft5x06_data->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_active);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	ft5x06_data->gpio_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			"pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ft5x06_data->gpio_state_suspend)) {
		dev_err(&ft5x06_data->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(ft5x06_data->gpio_state_suspend);
		ft5x06_data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int ft5x06_ts_pinctrl_select(struct ft5x0x_ts_data *ft5x06_data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ft5x06_data->gpio_state_active
		: ft5x06_data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ft5x06_data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&ft5x06_data->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(&ft5x06_data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}

//---duguowei,pinctrl


#ifdef CONFIG_PM
static int ft5x0x_ts_suspend(struct device *dev)
{
	/*struct ft5x0x_ts_data *ts = container_of(handler, struct ft5x0x_ts_data,
						early_suspend);*/
	char txbuf[2],i;
	int err;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);
//+++duguowei,pinctrl
if(1){
	if (ts->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(ts, false);
		if (err < 0)
			dev_err(dev, "Cannot get idle pinctrl state\n");
	}
}
//---duguowei,pinctrl
	dev_dbg(&ts->client->dev, "[FTS]ft5x0x suspend\n");
	if (ts->pdata->wakeup_gesture == 1) {
		disable_irq_wake(ts->irq);
	} else {
	disable_irq(ts->irq);
	}
	/* release all touches */
	printk("ft5x0x ft5x0x_ts_suspend\n");
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);

	if (gpio_is_valid(ts->pdata->reset_gpio)) {
		/*luochangyang For wakeup gesture 2014/04/29*/
		if (ts->pdata->wakeup_gesture == 1) {
			txbuf[0] = FT5x0x_REG_GEST_ONOFF;
			txbuf[1] = 0x01;					//enter gesture
		} else {
			txbuf[0] = FT5336_REG_PMODE;
			txbuf[1] = FT5336_PMODE_HIBERNATE;	//enter deep sleep
		}
		ft5x0x_i2c_Write(ts->client, txbuf, sizeof(txbuf));
		/*luochangyang END*/
	}
	return 0;
}

static int ft5x0x_ts_resume(struct device *dev)
{
	/*struct ft5x0x_ts_data *ts = container_of(handler, struct ft5x0x_ts_data,
						early_suspend);*/
	int err;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);

//+++duguowei,pinctrl
if(1){
	if (ts->ts_pinctrl) {
		err = ft5x06_ts_pinctrl_select(ts, true);
		if (err < 0)
			dev_err(dev, "Cannot get default pinctrl state\n");
	}
}
//---duguowei,pinctrl
	dev_dbg(&ts->client->dev, "[FTS]ft5x0x resume.\n");
	gpio_set_value(ts->pdata->reset_gpio, 0);
	msleep(20);
	gpio_set_value(ts->pdata->reset_gpio, 1);
	mdelay(200);
//+++duguowei
		//fix bug: fts failed set reg when usb plug in under suspend mode
		printk("ft5336_ts_resume is processing..\n");
#if defined(CONFIG_USB)
		if(usb_plug_status==1){
			printk("ft5336_ts_resume is processing,usb is plug..\n");
			i2c_smbus_write_byte_data( ts->client, 0x8B,0x3);
		}else{
			printk("ft5336_ts_resume is processing,usb not plug..\n");
			i2c_smbus_write_byte_data( ts->client, 0x8B,0x1);
		}
#endif
//---duguowei

	if (ts->pdata->wakeup_gesture == 1) {
			enable_irq_wake(ts->irq);
		} else {
			enable_irq(ts->irq);
		}

	hassleep = 0;
	
	return 0;
}
#else
#define ft5x0x_ts_suspend	NULL
#define ft5x0x_ts_resume		NULL
#endif


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x0x_ts_data *ft5x06_data =
		container_of(self, struct ft5x0x_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK){
			printk("ft5336 ts resume...\n");
			ft5x0x_ts_resume(&(ft5x06_data->client->dev));
		}
		else if (*blank == FB_BLANK_POWERDOWN){
			printk("ft5336 ts suspend...\n");
			ft5x0x_ts_suspend(&(ft5x06_data->client->dev));
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif

/*** ZTEMT Added by luochangyang, 2014/04/29 ***/
#if defined(CONFIG_PM_SLEEP)
 static int ztemt_wakeup_gesture_suspend(struct device *dev)
 {
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);

//	struct ft5x06_ts_platform_data *pdata = dev_get_platdata(dev);

	if (!(ts->pdata->wakeup_gesture))
	 return 0;

	/*
	* This will not prevent resume
	* Required to prevent interrupts before i2c awake
	*/
	disable_irq(ts->irq);

	if (device_may_wakeup(dev)) {
	 dev_dbg(dev, "%s Device MAY wakeup\n", __func__);
	 if (!enable_irq_wake(ts->irq))
		 ts->pdata->irq_wake = 1;
	} else {
	 dev_dbg(dev, "%s Device may NOT wakeup\n", __func__);
	}

	return 0;
 }
 
 static int ztemt_wakeup_gesture_resume(struct device *dev)
 {
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);

	if (!(ts->pdata->wakeup_gesture))
		return 0;

	enable_irq(ts->irq);

	if (device_may_wakeup(dev)) {
	 dev_dbg(dev, "%s Device MAY wakeup\n", __func__);
	 if (ts->pdata->irq_wake) {
		 disable_irq_wake(ts->irq);
		 ts->pdata->irq_wake = 0;
	 }
	} else {
	 dev_dbg(dev, "%s Device may NOT wakeup\n", __func__);
	}

	hassleep = 0;
	
	return 0;
	}
#endif
/***ZTEMT END***/

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
/*** ZTEMT Modify by luochangyang, 2014/04/29 ***/
#if ZTEMT_WAKEUP_GESTURE
	SET_SYSTEM_SLEEP_PM_OPS(ztemt_wakeup_gesture_suspend, ztemt_wakeup_gesture_resume)
	SET_RUNTIME_PM_OPS(ft5x0x_ts_suspend, ft5x0x_ts_resume, NULL)
#else
	.suspend = ft5x06_ts_suspend,
	.resume  = ft5x06_ts_resume,
#endif
/***ZTEMT END***/
};

static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata =
	    (struct ft5x06_ts_platform_data *)client->dev.platform_data;
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;

//+++duguowei
#if HAVE_TOUCH_KEY
	int index;
#endif
//---duguowei
//+++
	if (client->dev.of_node) {
		printk("[ft5x06]function:%s,client->dev.of_node\n",__func__);
			pdata = devm_kzalloc(&client->dev,
				sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
			if (!pdata) {
				dev_err(&client->dev, "Failed to allocate memory\n");
				return -ENOMEM;
			}
	
			err = ft5x06_parse_dt(&client->dev, pdata);
			if (err) {
				dev_err(&client->dev, "DT parsing failed\n");
				return err;
			}
		}
//---

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);

	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, ft5x0x_ts);


	
#if 0
	if(fts_init_gpio_hw(ft5x0x_ts)<0)
		goto exit_init_gpio;	
#endif
#ifdef CONFIG_PM
#if 0
	err = gpio_request(pdata->reset, "ft5x0x reset");
	if (err < 0) {
		dev_err(&client->dev, "%s:failed to set gpio reset.\n",
			__func__);
		goto exit_request_reset;
	}
	#endif
#endif
	
	ft5x0x_ts->irq = client->irq;//gpio_to_irq(FT5X0X_INT_PIN);//IRQ_EINT(14);//client->irq;
	ft5x0x_ts->x_max = pdata->x_max - 1;
	ft5x0x_ts->y_max = pdata->y_max - 1;
	//ft5x0x_ts->pdata->reset_gpio = FT5X0X_RESET_PIN;
	//ft5x0x_ts->pdata->irq_gpio = FT5X0X_INT_PIN;//ft5x0x_ts->irq;
	//client->irq = ft5x0x_ts->irq;
	//pr_info("irq = %d\n", client->irq);
	//pr_info("FT5X0X_INT_PIN = %d\n", FT5X0X_INT_PIN);



	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;
	ft5x0x_ts->client = client;
	ft5x0x_ts->pdata = pdata;
	//+++duguowei
#if HAVE_TOUCH_KEY
	for (index = 0; index < MAX_KEY_NUM; index++)
	{
		input_set_capability(input_dev, EV_KEY, button_map[index]);  
	}
#endif
	//---duguowei
	input_set_drvdata(input_dev, ft5x0x_ts);

#if FT_PROTOCAL_A
	/*protocal A*/
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);
	set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ft5x0x_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, CFG_MAX_TOUCH_POINTS, 0, 0);
#else
	/*protocal B*/
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	
	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS,0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ft5x0x_ts->y_max, 0, 0);
#endif

	/*luochangyang For wakeup gesture 2014/04/29*/
	set_bit(KEY_POWER, input_dev->keybit);
	set_bit(KEY_F10, input_dev->keybit);

	/*luochangyang END*/

	input_dev->name = FT5X0X_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ft5x0x_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft5x06_power_init(ft5x0x_ts, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(ft5x0x_ts, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}
	//+++duguowei,pinctrl
		err = ft5x06_ts_pinctrl_init(ft5x0x_ts);
		if (!err && ft5x0x_ts->ts_pinctrl) {
			err = ft5x06_ts_pinctrl_select(ft5x0x_ts, true);
			if (err < 0)
				goto exit_alloc_data_failed;
		}
	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft5x06_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft5x06_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(ft5x0x_ts->pdata->hard_rst_dly);
		gpio_set_value_cansleep(ft5x0x_ts->pdata->reset_gpio, 1);
	}
			

	//---duguowei,pinctrl

	
	err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt,
				   IRQF_TRIGGER_FALLING|IRQF_ONESHOT, client->dev.driver->name,
				   ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);

	
	/*make sure CTP already finish startup process */
	msleep(150);
#ifdef SYSFS_DEBUG
		ft5x0x_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC
		if (ft_rw_iic_drv_init(client) < 0)
			dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
					__func__);
#endif
//+++duguowei
	fts_ctpm_auto_upgrade(ft5x0x_ts->client);
#ifdef CONFIG_PM_SLEEP
	ft_ts_data = ft5x0x_ts;
	focaltech_register_ts_notifier(&ts_notifier);
#endif

#if defined(CONFIG_FB)
	ft5x0x_ts->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&ft5x0x_ts->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	ft5x0x_ts->early_suspend.suspend = ft5x06_ts_early_suspend;
	ft5x0x_ts->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	/*luochangyang For wakeup gesture 2014/04/29*/
	device_init_wakeup(&(client->dev), 1);

	err = device_create_file(&(client->dev), &dev_attr_wakeup_gesture);
	if (err) {
		dev_err(&(client->dev), "%s: Error, could not create wakeup_gesture", __func__);
	}
	/*Default enable or disable*/
#if 0
	pdata->wakeup_gesture = 1;
#else
	pdata->wakeup_gesture = 0;
#endif
	/*luochangyang END*/

//---duguowei
#ifdef FTS_APK_DEBUG
	//ft5x0x_create_apk_debug_channel(client);
#endif

g_i2c_client = client;

#ifdef FT5336_DOWNLOAD
			FTS_I2c_Read_Function fun_i2c_read = ft5x0x_download_i2c_Read;
			FTS_I2c_Write_Function fun_i2c_write = ft5x0x_download_i2c_Write;
			Init_I2C_Read_Func(fun_i2c_read);
			Init_I2C_Write_Func(fun_i2c_write);
			 if(ft5336_IsDownloadMain() < 0) {
	 	#if 1
				pr_info("--------FTS---------download main\n");
				if(ft5336_DownloadMain()<0)
				{
					pr_err("---------FTS---------Download main failed\n");
				}
		#endif
			 } else
				pr_info("--------FTS---------no download main\n");
#endif


	/*get some register information */
	uc_reg_addr = FT5x0x_REG_FW_VER;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	//dev_dbg(&client->dev, "[FTS] Firmware version = 0x%x\n", uc_reg_value);
	pr_info( "[FTS] Firmware version = 0x%x\n", uc_reg_value);

	uc_reg_addr = FT5x0x_REG_POINT_RATE;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	//dev_dbg(&client->dev, "[FTS] report rate is %dHz.\n",
	//	uc_reg_value * 10);
	pr_info("[FTS] report rate is %dHz.\n", uc_reg_value * 10);

	uc_reg_addr = FT5X0X_REG_THGROUP;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	//dev_dbg(&client->dev, "[FTS] touch threshold is %d.\n",
	//	uc_reg_value * 4);
	pr_info("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

	enable_irq(client->irq);
	return 0;


free_reset_gpio:
		if (gpio_is_valid(pdata->reset_gpio))
			gpio_free(pdata->reset_gpio);
free_irq_gpio:
		if (gpio_is_valid(pdata->irq_gpio))
			gpio_free(pdata->irq_gpio);
pwr_off:
		if (pdata->power_on)
			pdata->power_on(false);
		else
			ft5x06_power_on(ft5x0x_ts, false);
pwr_deinit:
		if (pdata->power_init)
			pdata->power_init(false);
		else
			ft5x06_power_init(ft5x0x_ts, false);
unreg_inputdev:
		input_unregister_device(input_dev);
		input_dev = NULL;
exit_input_register_device_failed:
			input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ft5x0x_ts);
#ifdef CONFIG_PM
#if 0
exit_request_reset:
	gpio_free(ft5x0x_ts->pdata->reset);
#endif
#endif

#if 0
exit_init_gpio:
	fts_un_init_gpio_hw(ft5x0x_ts);
#endif

exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __exit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	ft5x0x_ts = i2c_get_clientdata(client);
	input_unregister_device(ft5x0x_ts->input_dev);
	#ifdef CONFIG_PM
	gpio_free(ft5x0x_ts->pdata->reset_gpio);
	#endif

	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	#ifdef SYSFS_DEBUG
		ft5x0x_remove_sysfs(client);
	#endif

	#ifdef FTS_APK_DEBUG
		//ft5x0x_release_apk_debug_channel();
	#endif
#ifdef CONFIG_PM_SLEEP
		focaltech_unregister_ts_notifier(&ts_notifier);
#endif

#if 0
	fts_un_init_gpio_hw(ft5x0x_ts);
#endif
	if (gpio_is_valid(ft5x0x_ts->pdata->reset_gpio))
		gpio_free(ft5x0x_ts->pdata->reset_gpio);

	if (gpio_is_valid(ft5x0x_ts->pdata->irq_gpio))
		gpio_free(ft5x0x_ts->pdata->irq_gpio);

	if (ft5x0x_ts->pdata->power_on)
		ft5x0x_ts->pdata->power_on(false);
	else
		ft5x06_power_on(ft5x0x_ts, false);

	if (ft5x0x_ts->pdata->power_init)
		ft5x0x_ts->pdata->power_init(false);
	else
		ft5x06_power_init(ft5x0x_ts, false);

	free_irq(client->irq, ft5x0x_ts);

	kfree(ft5x0x_ts);
	i2c_set_clientdata(client, NULL);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif


static const struct i2c_device_id ft5x0x_ts_id[] = {
	{FT5X0X_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe = ft5x0x_ts_probe,
	.remove = __exit_p(ft5x0x_ts_remove),
	.id_table = ft5x0x_ts_id,
	.driver = {
		   .name = FT5X0X_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x06_ts_pm_ops,
#endif
		   },
};

static int __init ft5x0x_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ft5x0x driver failed "
		       "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ft5x0x_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
