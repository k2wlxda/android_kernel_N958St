/*
 * MELFAS MMS Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Platform Data
 *
 * Default path : linux/platform_data/melfas_mms.h
 *
 */

#ifndef _LINUX_MMS_TOUCH_H
#define _LINUX_MMS_TOUCH_H

#ifdef CONFIG_OF
#define MMS_USE_DEVICETREE		1
#else
#define MMS_USE_DEVICETREE		0
#endif

#define MMS_USE_CALLBACK	1	// 0 or 1 : Callback for inform charger, display, power, etc...

#define MMS_DEVICE_NAME	"mms_ts"


#define TP_LOG_TAG "TP"
#define TP_LOG_ON
#define TP_DEBUG_ON

#ifdef  TP_LOG_ON
#define TP_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] "  fmt, TP_LOG_TAG, __FUNCTION__, __LINE__, ##args)
    #ifdef  TP_DEBUG_ON
#define TP_LOG_DEBUG(fmt, args...) printk(KERN_INFO "[%s] [%s: %d] "  fmt, TP_LOG_TAG, __FUNCTION__, __LINE__, ##args)                                              
    #else
#define TP_LOG_DEBUG(fmt, args...)
    #endif
#else
#define TP_LOG_ERROR(fmt, args...)
#define TP_LOG_DEBUG(fmt, args...)
#endif

struct mms_ts_pinctrl {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

/**
* Platform Data
*/
struct mms_platform_data {
	unsigned int max_x;
	unsigned int max_y;

	int gpio_intr;			//Required (interrupt signal)
	int gpio_reset;		//Optional

	//int gpio_vdd_en;		//Optional (power control)	

    struct mms_ts_pinctrl pin_res;

#if MMS_USE_CALLBACK
	void (*register_callback) (void *);
#endif
};

#endif

