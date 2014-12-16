/*******************************************************************************
*                                                                                           *
*   File Name:    tmd2772.c                                                        *
*   Description:  Linux device driver for Taos ambient light and          *
*                     proximity sensors.                                              *
*   Author:        John Koshi                                                        *
*   History:        09/16/2009 - Initial creation                               *
*                     10/09/2009 - Triton version                                *
*                     12/21/2009 - Probe/remove mode                        *
*                     02/07/2010 - Add proximity                                 *
*                                                                                           *
********************************************************************************
*    Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074 *
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/i2c/taos_common.h>
#include <linux/delay.h>
#include <linux/irq.h> 
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#define LOG_TAG "SENSOR_ALS_PROX"
#define DEBUG_ON

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : __FILE__

#ifdef  CONFIG_FEATURE_ZTEMT_SENSORS_LOG_ON
#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR   "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #ifdef  DEBUG_ON
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO  "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
                                              
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt,\
                                              LOG_TAG,__FUNCTION__, __LINE__, ##args)
    #else
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
    #endif
#else
#define SENSOR_LOG_ERROR(fmt, args...)
#define SENSOR_LOG_INFO(fmt, args...)
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

//globle varibles
struct taos_cfg *taos_cfgp = NULL;
struct taos_data *taos_datap = NULL;

//class
static struct class *proximity_class = NULL;
static struct class *light_class = NULL;

//device
static dev_t const tmd2772_proximity_dev_t = MKDEV(MISC_MAJOR, 101);
static dev_t const tmd2772_light_dev_t     = MKDEV(MISC_MAJOR, 102);

//debug flag
static bool pro_ft = false;
static bool flag_als_debug  = false;
static bool flag_prox_debug = false;
static unsigned int prox_debug_delay_time = 0;

//prox threshold calculation coefficient 
static u16 prox_calibrate_hi_param = 500;
static u16 prox_calibrate_lo_param = 330;

//als
static u16 als_saturation_value = 0;
// lux time scale
struct time_scale_factor  {
    u16 numerator;
    u16 denominator;
    u16 saturation;
}TritonTime = {1, 0, 0};

// gain table
u8 als_gain_table[] = {1, 8, 16, 120};

struct lux_data TritonFN_lux_data[] = {
    { 9830,  8320,  15360 },
    { 12452, 10554, 22797 },
    { 14746, 6234,  11430 },
    { 17695, 3968,  6400  },
    { 0,     0,     0     }
};
struct lux_data *lux_tablep = TritonFN_lux_data;
static int lux_history[ALS_FILTER_DEPTH] = {-ENODATA, -ENODATA, -ENODATA};

// device reg init values
u8 taos_triton_reg_init[16] = {0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 
                               0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};

static unsigned char g_reg_addr = 0;
static bool wakeup_from_sleep = false;

static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "ZTE",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "10000",
	.resolution = "1.0",
	.sensor_power = "0.5",
	.min_delay = 500000, /* in microseconds (us) */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 500,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "ZTE",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "1023",
	.resolution = "1.0",
	.sensor_power = "0.5",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static const struct i2c_device_id tmd2772_idtable_id[] = {
	{ "ams,ams-sensor", 0 },
	{ },
};

static struct of_device_id of_tmd2772_idtable[] = {
	{ .compatible = "ams,ams-sensor",},
	{}
};

MODULE_DEVICE_TABLE(i2c, tmd2772_idtable);

struct i2c_driver tmd2772_driver = {
	.driver = {
		.name = "ams-sensor",
        .of_match_table = of_tmd2772_idtable,
	},
	.id_table = tmd2772_idtable_id,
	.probe = tmd2772_probe,
	.remove = tmd2772_remove,
#ifdef CONFIG_PM_SLEEP
    .resume = taos_resume,
    .suspend = taos_suspend,
#endif
};

//function implementation
static ssize_t attr_chip_name_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "%s", taos_datap->chip_name);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_check_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	if (NULL != taos_datap)
    {
        return sprintf(buf, "%s(0x%x)", taos_datap->chip_name, TMD2772_DEVICE_ID);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *chip = dev_get_drvdata(dev);
	return sprintf(buf, "prox_on is %s\n", chip->prox_on ? "true" : "false");
}

static ssize_t attr_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct taos_data *chip = dev_get_drvdata(dev);
	bool value = false;

    SENSOR_LOG_INFO("enter\n");
	if (strtobool(buf, &value))
		return -EINVAL;

    mutex_lock(&chip->lock);

	if (value)
    {
        taos_prox_on();
    }
	else
    {
        taos_prox_off();
    }

    mutex_unlock(&chip->lock);
    SENSOR_LOG_INFO("exit\n");

	return size;
}

static ssize_t attr_prox_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int value = 1, ret = 0;

    SENSOR_LOG_INFO("enter\n");

	if (kstrtouint(buf, 10, &value))
		return -EINVAL;

    mutex_lock(&taos_datap->lock);

	if (value == 1)
    {
    	if((ret = taos_read_cal_value(PATH_PROX_OFFSET)) > 0)
		    taos_cfgp->prox_config_offset = ret;
        
    	if((ret = taos_read_cal_value(PATH_PROX_CAL_THRESHOLD)) < 0)
		{
		    SENSOR_LOG_ERROR("need to create %s\n", PATH_PROX_CAL_THRESHOLD);
            
		    ret = taos_write_cal_file(PATH_PROX_CAL_THRESHOLD, 0);
			if(ret < 0)
			{
				SENSOR_LOG_ERROR("failed to create %s\n", PATH_PROX_CAL_THRESHOLD);
				mutex_unlock(&taos_datap->lock);
				return -EINVAL;
			}
           	taos_datap->prox_calibrate_flag = true;
		}
		else if (ret == 0)
        {
	      	 taos_datap->prox_calibrate_flag = true;
			 SENSOR_LOG_ERROR("taos_read_cal_value %s return 0\n", PATH_PROX_CAL_THRESHOLD);
		}
		else if(ret > (taos_datap->prox_thres_hi_min))
        {
			taos_datap->prox_calibrate_flag = false;
			taos_datap->prox_manual_calibrate_threshold = ret;
	        taos_cfgp->prox_threshold_hi = ret;
	        taos_cfgp->prox_threshold_lo  = ret - PROX_THRESHOLD_DISTANCE;
            
			input_report_rel(taos_datap->p_idev, REL_Y, taos_cfgp->prox_threshold_hi);
			input_report_rel(taos_datap->p_idev, REL_Z, taos_cfgp->prox_threshold_lo);
			input_sync(taos_datap->p_idev);
            
			SENSOR_LOG_ERROR("get hi/lo = %d/%d from %s\n", taos_cfgp->prox_threshold_hi, 
                taos_cfgp->prox_threshold_lo, PATH_PROX_CAL_THRESHOLD);
		}

		if((ret = taos_read_cal_value(PATH_PROX_UNCOVER_DATA)) > 0)
        {
            taos_datap->prox_uncover_data = ret;
			//taos_datap->prox_thres_hi_min = taos_datap->prox_uncover_data + PROX_THRESHOLD_SAFE_DISTANCE + PROX_THRESHOLD_DISTANCE;
            taos_datap->prox_thres_hi_min = taos_datap->prox_uncover_data + PROX_THRESHOLD_SAFE_DISTANCE;
            taos_datap->prox_thres_hi_min = (taos_datap->prox_thres_hi_min > PROX_THRESHOLD_HIGH_MIN)? taos_datap->prox_thres_hi_min : PROX_THRESHOLD_HIGH_MIN;
        }
	}
	else
    {
		mutex_unlock(&taos_datap->lock);
        SENSOR_LOG_ERROR("input error, valid: 1\n");
		return -EINVAL;
	}

    mutex_unlock(&taos_datap->lock);
    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_led_pluse_cnt_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_led_pluse_cnt is %d\n", taos_cfgp->prox_pulse_cnt);
    }
    else
    {       
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_led_pluse_cnt_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned int val = 0;
    int ret = 0;

    SENSOR_LOG_ERROR("enter\n");

    if (kstrtouint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (NULL != taos_cfgp)
    {
        taos_cfgp->prox_pulse_cnt = val;
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT), taos_cfgp->prox_pulse_cnt))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write prox_pulse_cnt reg\n");
        }
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_adc_time_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_adc_time is 2.73 * %d ms\n", 256 - taos_cfgp->prox_adc_time);
    }
    else
    {       
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_adc_time_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned int val = 0;
    int ret = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtouint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (NULL != taos_cfgp)
    {
        taos_cfgp->prox_adc_time = 256 - val;
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME), taos_cfgp->prox_adc_time))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write prox_adc_time reg\n");
        }
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_led_strength_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    char *p_led_sth[4] = {"100", "50", "25", "12.5"};
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_led_strength is %s mA\n", p_led_sth[(taos_cfgp->prox_gain) >> 6]);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_led_strength_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    int val = 0, ret = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtoint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val > 4 || val <= 0)
    {
        SENSOR_LOG_ERROR("input error, valid: 1~4");
    }
    else
    {
        val = 4 - val;
        if (NULL != taos_cfgp)
        {
            taos_cfgp->prox_gain = (taos_cfgp->prox_gain & 0x3F) | (val << 6);

            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
                (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0)
            {
                SENSOR_LOG_ERROR("failed to write prox_led_strength reg\n");
            }
        }
        else
        {
            SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
        }
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_debug_delay_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_debug_delay is %d\n", prox_debug_delay_time);
    }
    else
    {       
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_debug_delay_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    prox_debug_delay_time = val;

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_set_prox_calibrate(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    int val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtoint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val > 1)
    {
        taos_datap->prox_calibrate_times = val;
        taos_prox_calibrate();
    }
    else
    {
        SENSOR_LOG_ERROR("input error, valid: > 1\n");
    }

	return size;
} 

static ssize_t attr_prox_threshold_high_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_threshold_hi is %d", taos_cfgp->prox_threshold_hi);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_threshold_high_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (NULL != taos_cfgp)
    {
        taos_cfgp->prox_threshold_hi = val;
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_threshold_low_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_threshold_lo is %d", taos_cfgp->prox_threshold_lo);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_threshold_low_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (NULL != taos_cfgp)
    {
        taos_cfgp->prox_threshold_lo = val;
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_calibrate_result_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "prox_calibrate_result is %d", taos_datap->prox_calibrate_result);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_cali_high_param_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_calibrate_hi_param is %d\n", prox_calibrate_hi_param);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_cali_high_param_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val > 0)
    {
        prox_calibrate_hi_param = val;
    }
    else
    {
        SENSOR_LOG_ERROR("input error, valid: > 0\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_cali_low_param_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_calibrate_lo_param is %d\n", prox_calibrate_lo_param);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_cali_low_param_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val > 0)
    {
        prox_calibrate_lo_param = val;
    }
    else
    {
        SENSOR_LOG_ERROR("input error, valid: > 0\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_thres_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_threshold_lo is %d, prox_threshold_hi is %d\n",
            taos_cfgp->prox_threshold_lo,taos_cfgp->prox_threshold_hi);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_thres_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	long value = 0;
	int rc = 0;

    SENSOR_LOG_ERROR("enter\n");
	if( kstrtol(buf, 10, &value))
	{
		return -EINVAL;
	}

	mutex_lock(&taos_datap->lock);

	if (value == 1)
    {
   		if( (rc = taos_read_cal_value(PATH_PROX_CAL_THRESHOLD)) < 0)
        {
            mutex_unlock(&taos_datap->lock);
            return -EINVAL;
        }
		else
		{
			if(rc > (taos_datap->prox_thres_hi_min))
            {
    			taos_datap->prox_calibrate_flag = false;
    			taos_datap->prox_manual_calibrate_threshold = rc;
    			taos_cfgp->prox_threshold_hi= rc;
    			taos_cfgp->prox_threshold_lo  = rc - PROX_THRESHOLD_DISTANCE;

                input_report_rel(taos_datap->p_idev, REL_Y, taos_cfgp->prox_threshold_hi);
    			input_report_rel(taos_datap->p_idev, REL_Z, taos_cfgp->prox_threshold_lo);
    			input_sync(taos_datap->p_idev);

    			SENSOR_LOG_ERROR("prox_th_high  = %d\n", taos_cfgp->prox_threshold_hi);
    			SENSOR_LOG_ERROR("prox_th_low   = %d\n", taos_cfgp->prox_threshold_lo);
           	}
		}
	}
	else
    {
		mutex_unlock(&taos_datap->lock);
		return -EINVAL;
	}

	mutex_unlock(&taos_datap->lock);

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "flag_prox_debug is %s\n", flag_prox_debug? "true" : "false");
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_debug_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    int val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtoint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val)
    {
        flag_prox_debug = true;
    }
    else
    {
        flag_prox_debug = false;
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_calibrate_start_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "flag_prox_debug  is %s\n", flag_prox_debug? "true" : "false");
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_thres_hi_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        SENSOR_LOG_ERROR( "prox_thres_hi_max is %d\n", taos_datap->prox_thres_hi_max);
        return sprintf(buf, "%d", taos_datap->prox_thres_hi_max);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_thres_lo_min_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        SENSOR_LOG_ERROR("prox_thres_hi_min is %d\n", taos_datap->prox_thres_hi_min);
        return sprintf(buf, "%d", taos_datap->prox_thres_hi_min);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_data_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "%d", taos_datap->prox_data_max);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_manual_cali_threshold_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "%d", taos_datap->prox_manual_calibrate_threshold);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_prox_wakelock_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *chip = dev_get_drvdata(dev);

    SENSOR_LOG_ERROR("proximity_wakelock is %s\n",chip->proximity_wakelock.locked ? "true" : "false");
	return sprintf(buf, "proximity_wakelock is %s\n",chip->proximity_wakelock.locked ? "true" : "false");
}

static ssize_t attr_prox_prox_wakelock_store(struct device *dev, struct device_attribute *attr,
                                            const char *buf, size_t size)
{
	struct taos_data *chip = dev_get_drvdata(dev);
	unsigned int recv = 0;
    if(kstrtouint(buf, 10, &recv))
    {
        return -EINVAL;
    }

    mutex_lock(&chip->lock);
    if (recv)
    {
        taos_wakelock_ops(&(chip->proximity_wakelock), true);
    }
    else
    {
    	hrtimer_cancel(&taos_datap->prox_unwakelock_timer);
    	taos_wakelock_ops(&(chip->proximity_wakelock), false);
    }
    mutex_unlock(&chip->lock);
	return size;
}

static void taos_irq_ops(bool enable, bool flag_sync)
{
    if (enable == taos_datap->irq_enabled)
    {
        SENSOR_LOG_ERROR("doubule %s irq, retern here\n",enable? "enable" : "disable");
        return;
    }

    if (enable)
    {
        enable_irq(taos_datap->client->irq);
    }
    else
    {
        if (flag_sync)
        {
            disable_irq(taos_datap->client->irq);
        }
        else
        {
            disable_irq_nosync(taos_datap->client->irq);
        }
    }
    taos_datap->irq_enabled  = enable;
}

static ssize_t attr_irq_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "flag_irq is %s\n", taos_datap->irq_enabled? "true" : "false");
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_irq_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned int val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtouint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val)
    {
        taos_irq_ops(true, true);
    }
    else
    {
        taos_irq_ops(false, true);
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_wait_time_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "wait_time is 2.73 * %d ms\n", 256 - taos_cfgp->wait_time);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_wait_time_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned int val = 0;
    int ret = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtoint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (NULL != taos_cfgp)
    {
        taos_cfgp->wait_time =  256 - val;
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME), taos_cfgp->wait_time))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write wait_time reg\n");
        }
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_offset_cal_start_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "flag_prox_debug is %s\n", flag_prox_debug? "true" : "false");
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_offset_cal_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int value = 1;

    SENSOR_LOG_INFO("enter\n");
    if(kstrtoint(buf, 10, &value))
	{
		return -EINVAL;
	}

	mutex_lock(&taos_datap->lock);
	if (value == 1)
    {
         schedule_delayed_work(&taos_datap->prox_offset_cal_work, msecs_to_jiffies(0));
	}
	else
    {
        SENSOR_LOG_ERROR("input error, valid: 1\n");
    }

    mutex_unlock(&taos_datap->lock);

	SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_offset_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "prox_config_offset is %d\n", taos_cfgp->prox_config_offset);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_offset_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;
    int ret = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (NULL != taos_cfgp)
    {
        taos_cfgp->prox_config_offset = val;
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write the prox_config_offset  reg\n");
        }
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_offset_cal_result_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "prox_offset_cal_result is %d", taos_datap->prox_offset_cal_result);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_data_safe_range_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        SENSOR_LOG_ERROR( "PROX_DATA_SAFE_RANGE_MAX is %d\n", PROX_DATA_SAFE_RANGE_MAX);
        return sprintf(buf, "%d", PROX_DATA_SAFE_RANGE_MAX);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_data_safe_range_min_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        SENSOR_LOG_ERROR("PROX_DATA_SAFE_RANGE_MIN is %d\n", PROX_DATA_SAFE_RANGE_MIN);
        return sprintf(buf, "%d", PROX_DATA_SAFE_RANGE_MIN);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_reg_addr_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    return sprintf(buf, "g_reg_addr is 0x%02x\n", g_reg_addr);
}

static ssize_t attr_reg_addr_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 16, &val))
    {
        return -EINVAL;
    }

    g_reg_addr = val;

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_reg_data_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    unsigned char i = 0;

    if (100 == g_reg_addr)
    {
        for (i = 0x00; i <= 0x0F; i++)
        {
            i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | i));
            SENSOR_LOG_ERROR("reg[0x%02x] = 0x%02x\n", i, i2c_smbus_read_byte(taos_datap->client));
        }

        for (i = 0x12; i <= 0x19; i++)
        {
            i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | i));
            SENSOR_LOG_ERROR("reg[0x%02x] = 0x%02x\n", i, i2c_smbus_read_byte(taos_datap->client));            
        }

        i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | 0x1E));
        SENSOR_LOG_ERROR("reg[0x1E] = 0x%02x\n", i2c_smbus_read_byte(taos_datap->client));
    }
    else
    {
        i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | g_reg_addr));
        return sprintf(buf, "reg[0x%02x] = 0x%02x", g_reg_addr, i2c_smbus_read_byte(taos_datap->client));
    }

	return strlen(buf);
}

static ssize_t attr_reg_data_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;
    int ret = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 16, &val))
    {
        return -EINVAL;
    }

    if (100 == g_reg_addr)
    {
        SENSOR_LOG_ERROR("reg addr error\n");
    }
    else
    {
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|g_reg_addr), val))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write 0x%02x reg\n", g_reg_addr);
        }
    }
    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_offset_cal_verify_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "%d", taos_datap->prox_offset_cal_verify);
    }
    else
    {
        sprintf(buf, "taos_datap is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_offset_cal_verify_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

	taos_datap->prox_offset_cal_verify = val;

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_prox_calibrate_verify_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "%d", taos_datap->prox_calibrate_verify);
    }
    else
    {
        sprintf(buf, "taos_datap is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_prox_calibrate_verify_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

	taos_datap->prox_calibrate_verify = val;

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

//als attributes
static ssize_t attr_als_light_check_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_datap)
    {
        return sprintf(buf, "%s(0x%x)", taos_datap->chip_name, TMD2772_DEVICE_ID);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *chip = dev_get_drvdata(dev);
	return sprintf(buf, "als_on is %s\n", chip->als_on ? "true" : "false");
}

static ssize_t attr_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct taos_data *chip = dev_get_drvdata(dev);
	bool value = false;

    SENSOR_LOG_ERROR("enter\n");
	if (strtobool(buf, &value))
		return -EINVAL;

    mutex_lock(&chip->lock);
	if (value)
    {
        taos_sensors_als_poll_on();
    }
    else
    {
        taos_sensors_als_poll_off();
    }

    mutex_unlock(&chip->lock);

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_als_gain_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "als gain is x%d\n", als_gain_table[taos_cfgp->als_gain]);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_als_gain_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    int val = 0, ret = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtoint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val > 4 || val <= 0)
    {
        SENSOR_LOG_ERROR("input error, valid: 1~4");
    }
    else
    {
        val = val - 1;

        if (NULL != taos_cfgp)
        {
            taos_cfgp->als_gain = val;
            taos_cfgp->prox_gain = (taos_cfgp->prox_gain & 0xFC) | val;

            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
                (TAOS_TRITON_CMD_REG|0x0F), taos_cfgp->prox_gain))) < 0)
            {
                SENSOR_LOG_ERROR("failed to write the prox_led_strength reg\n");
            }
        }
        else
        {
            SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
        }
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_als_debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "flag_als_debug is %s\n", flag_als_debug? "true" : "false");
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_als_debug_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned int val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtouint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val)
    {
        flag_als_debug = true;
    }
    else
    {
        flag_als_debug = false;
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_als_adc_time_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        return sprintf(buf, "als_adc_time is 2.73 * %d ms\n", 256 - taos_cfgp->als_adc_time);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_als_adc_time_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned int val = 0;
    int ret = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (kstrtouint(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (NULL != taos_cfgp)
    {
        taos_cfgp->als_adc_time = 256 - val;
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), taos_cfgp->als_adc_time))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write als_adc_time reg\n");
        }
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_scale_factor_als_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        sprintf(buf, "als_scale_factor is %d\n", taos_cfgp->als_scale_factor);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_scale_factor_als_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val > 0)
    {
        if (NULL != taos_cfgp)
        {
            taos_cfgp->als_scale_factor = val;
        }
        else
        {
            SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
        }
    }
    else
    {
        SENSOR_LOG_ERROR("input error, valid: > 0\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_scale_factor_prox_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
    if (NULL != taos_cfgp)
    {
        sprintf(buf, "prox_scale_factor is %d\n", taos_cfgp->prox_scale_factor);
    }
    else
    {
        sprintf(buf, "taos_cfgp is NULL\n");
    }
	return strlen(buf);
}

static ssize_t attr_scale_factor_prox_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
    unsigned long val = 0;

    SENSOR_LOG_ERROR("enter\n");
    if (strict_strtoul(buf, 10, &val))
    {
        return -EINVAL;
    }

    if (val > 0)
    {
        if (NULL != taos_cfgp)
        {
            taos_cfgp->prox_scale_factor = val;
        }
        else
        {
            SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
        }
    }
    else
    {
        SENSOR_LOG_ERROR("input error, valid: > 0\n");
    }

    SENSOR_LOG_ERROR("exit\n");
	return size;
}

static ssize_t attr_als_poll_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "als_poll_time = %d\n", taos_cfgp->als_poll_delay);
}

static ssize_t attr_als_poll_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long time = 0;

    if(kstrtoul(buf, 10, &time))
    {
    	return -EINVAL;
    }

	taos_cfgp->als_poll_delay = time;
	return size;
}

static struct device_attribute attrs_prox[] = {
    __ATTR(chip_name,                      0444,   attr_chip_name_show,                  NULL),
    __ATTR(prox_check,                     0444,   attr_prox_check_show,                 NULL),
	__ATTR(enable,                         0644,   attr_prox_enable_show,                attr_prox_enable_store),
	__ATTR(prox_init,                      0200,   NULL,                                 attr_prox_init_store),
	__ATTR(prox_led_pluse_cnt,             0644,   attr_prox_led_pluse_cnt_show,         attr_prox_led_pluse_cnt_store),
    __ATTR(prox_adc_time,                  0644,   attr_prox_adc_time_show,              attr_prox_adc_time_store),
    __ATTR(prox_led_strength_level,        0644,   attr_prox_led_strength_show,          attr_prox_led_strength_store),
    __ATTR(prox_debug_delay,               0644,   attr_prox_debug_delay_show,           attr_prox_debug_delay_store),
    __ATTR(prox_calibrate,                 0200,   NULL,                                 attr_set_prox_calibrate),
    __ATTR(prox_threshold_high,            0644,   attr_prox_threshold_high_show,        attr_prox_threshold_high_store),
    __ATTR(prox_threshold_low,             0644,   attr_prox_threshold_low_show,         attr_prox_threshold_low_store),
    __ATTR(prox_offset,                    0644,   attr_prox_offset_show,                attr_prox_offset_store),
    __ATTR(prox_calibrate_result,          0444,   attr_prox_calibrate_result_show,      NULL),
    __ATTR(prox_thres_param_high,          0644,   attr_prox_cali_high_param_show,       attr_prox_cali_high_param_store),
    __ATTR(prox_thres_param_low,           0644,   attr_prox_cali_low_param_show,        attr_prox_cali_low_param_store),
    __ATTR(prox_thres,                     0644,   attr_prox_thres_show,                 attr_prox_thres_store),
    __ATTR(prox_debug,                     0644,   attr_prox_debug_show,                 attr_prox_debug_store),
    __ATTR(prox_calibrate_start,           0644,   attr_prox_calibrate_start_show,       attr_prox_debug_store),
    __ATTR(prox_thres_max,                 0444,   attr_prox_thres_hi_max_show,          NULL),
    __ATTR(prox_thres_min,                 0444,   attr_prox_thres_lo_min_show,          NULL),
    __ATTR(prox_data_max,                  0444,   attr_prox_data_max_show,              NULL),
    __ATTR(prox_manual_calibrate_threshold,0444,   attr_prox_manual_cali_threshold_show, NULL),
    __ATTR(prox_wakelock,                  0644,   attr_prox_prox_wakelock_show,         attr_prox_prox_wakelock_store),
    __ATTR(irq_status,                     0644,   attr_irq_show,                        attr_irq_store),
    __ATTR(wait_time,                      0644,   attr_wait_time_show,                  attr_wait_time_store),
    __ATTR(prox_offset_cal_start,          0644,   attr_prox_offset_cal_start_show,      attr_prox_debug_store),
    __ATTR(prox_offset_cal,                0644,   attr_prox_offset_show,                attr_prox_offset_cal_store),
    __ATTR(prox_offset_cal_result,         0444,   attr_prox_offset_cal_result_show,     NULL),
    __ATTR(prox_data_safe_range_max,       0444,   attr_prox_data_safe_range_max_show,   NULL),
    __ATTR(prox_data_safe_range_min,       0444,   attr_prox_data_safe_range_min_show,   NULL),
    __ATTR(reg_addr,                       0644,   attr_reg_addr_show,                   attr_reg_addr_store),
    __ATTR(reg_data,                       0644,   attr_reg_data_show,                   attr_reg_data_store),
    __ATTR(prox_offset_cal_verify,         0644,   attr_prox_offset_cal_verify_show,     attr_prox_offset_cal_verify_store),
    __ATTR(prox_calibrate_verify,          0644,   attr_prox_calibrate_verify_show,      attr_prox_calibrate_verify_store),
};

static struct device_attribute attrs_light[] = {
	__ATTR(light_check,                    0444,   attr_als_light_check_show,            NULL),
	__ATTR(enable,                         0644,   attr_als_enable_show,                 attr_als_enable_store),
    __ATTR(light_gain,                     0644,   attr_als_gain_show,                   attr_als_gain_store),
    __ATTR(light_debug,                    0644,   attr_als_debug_show,                  attr_als_debug_store),
    __ATTR(light_adc_time,                 0644,   attr_als_adc_time_show,               attr_als_adc_time_store),
    __ATTR(scale_factor_als,               0644,   attr_scale_factor_als_show,           attr_scale_factor_als_store),
    __ATTR(scale_factor_prox,              0644,   attr_scale_factor_prox_show,          attr_scale_factor_prox_store),
    __ATTR(delay,                          0644,   attr_als_poll_time_show,              attr_als_poll_time_store),
};

static int create_sysfs_interfaces_prox(struct device *dev)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(attrs_prox); i++)
		if (device_create_file(dev, attrs_prox + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attrs_prox + i);

	SENSOR_LOG_ERROR("Unable to create interface\n");
	return -1;
}

static int create_sysfs_interfaces_light(struct device *dev)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(attrs_light); i++)
		if (device_create_file(dev, attrs_light + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attrs_light + i);

	SENSOR_LOG_ERROR("Unable to create interface\n");
	return -1;
}

static int taos_write_cal_file(char *file_path, unsigned int value)
{
    struct file *file_p = NULL;
    char write_buf[10] = { 0 };
    mm_segment_t old_fs;
    int vfs_write_retval = 0;

    if (NULL == file_path)
    {
        SENSOR_LOG_ERROR("file_path is NULL\n");
        goto error;
    }

    memset(write_buf, 0, sizeof(write_buf));
    sprintf(write_buf, "%d\n", value);

    file_p = filp_open(file_path, O_CREAT|O_RDWR , 0665);
    if (IS_ERR(file_p))
    {
        SENSOR_LOG_ERROR("[open file <%s>failed]\n", file_path);
        goto error;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    vfs_write_retval = vfs_write(file_p, (char*)write_buf, sizeof(write_buf), &file_p->f_pos);
    if (vfs_write_retval < 0)
    {
        SENSOR_LOG_ERROR("failed to write file <%s>\n", file_path);
        goto error;
    }

    set_fs(old_fs);
    filp_close(file_p, NULL);
    return 1;

error:
    return -1;
}

static int taos_read_cal_value(char *file_path)
{
    struct file *file_p = NULL;
    int vfs_read_retval = 0;
    mm_segment_t old_fs;
    char read_buf[32] = { 0 };
    unsigned short read_value = 0;

    if (NULL == file_path)
    {
        SENSOR_LOG_ERROR("file_path is NULL\n");
        goto error;
    }

    memset(read_buf, 0, 32);

    file_p = filp_open(file_path, O_RDONLY , 0);
    if (IS_ERR(file_p))
    {
        SENSOR_LOG_ERROR("failed to open file <%s>\n", file_path);
        goto error;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    vfs_read_retval = vfs_read(file_p, (char*)read_buf, 16, &file_p->f_pos);
    if (vfs_read_retval < 0)
    {
        SENSOR_LOG_ERROR("failed to read file <%s>\n", file_path);
        goto error;
    }

    set_fs(old_fs);
    filp_close(file_p, NULL);

    if (kstrtou16(read_buf, 10, &read_value) < 0)
    {
        SENSOR_LOG_ERROR("failed to kstrtou16 %s\n", read_buf);
        goto error;
    }

    SENSOR_LOG_ERROR("the content of %s is %s\n", file_path, read_buf);
    return read_value;

error:
    return -1;
}

static int taos_prox_on(void)
{
    int prox_sum = 0, prox_mean = 0, prox_max = 0;
    int  ret = 0;
    u8 reg_cntrl = 0, i = 0, j = 0;
    struct taos_prox_info prox_info = { 0 };

    taos_datap->prox_on = 1;
    taos_datap->als_poll_time_mul = 2;

    SENSOR_LOG_INFO("######## TAOS IOCTL PROX ON  ######## \n");

    if (true == taos_datap->als_on)
    {
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), ALS_ADC_TIME_PROX_ON))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
            return ret;
        }
    }
    else
    {
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), 0xFF))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
            return (ret);
        }
    }

    taos_update_sat_als();

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME), taos_cfgp->prox_adc_time))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_TIME reg\n");
        return ret;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME), taos_cfgp->wait_time))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_WAIT_TIME reg\n");
        return ret;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_INTERRUPT), taos_cfgp->prox_intr_filter))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_INTERRUPT reg\n");
        return ret;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_CFG), taos_cfgp->prox_config))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_CFG reg\n");
        return ret;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT), taos_cfgp->prox_pulse_cnt))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_COUNT reg\n");
        return ret;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_GAIN reg\n");
        return ret;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_OFFSET reg\n");
        return ret;
    }

    reg_cntrl = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_PROX_INT_ENBL | 
		        TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_WAIT_TMR_ENBL;
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
        return ret;
    }

	pro_ft = true;
    if (taos_datap->prox_calibrate_flag)
    {
        prox_sum = 0;
        prox_max = 0;

        mdelay(20);
        for (i = 0, j = 0; i < 5; i++)
        {
            if ((ret = taos_prox_poll(&prox_info)) < 0)
            {
                SENSOR_LOG_ERROR("failed to taos_prox_poll\n");
            }
			else
			{
				j++;
                prox_sum += prox_info.prox_data;
                if (prox_info.prox_data > prox_max)
                    prox_max = prox_info.prox_data;
                mdelay(20);
			}
        }

		if(j == 0)
		{
			ret = -1;
			goto error;
		}
        prox_mean = prox_sum / j;

        taos_cfgp->prox_threshold_hi = ((((prox_max - prox_mean) * prox_calibrate_hi_param) + 50)/100) + prox_mean + 110;
        taos_cfgp->prox_threshold_lo = ((((prox_max - prox_mean) * prox_calibrate_lo_param) + 50)/100) + prox_mean + 35;

        SENSOR_LOG_ERROR("TAOS:------------ taos_cfgp->prox_threshold_hi = %d\n", taos_cfgp->prox_threshold_hi);
        SENSOR_LOG_ERROR("TAOS:------------ taos_cfgp->prox_threshold_lo = %d\n", taos_cfgp->prox_threshold_lo);

        if( prox_mean > 800 || taos_cfgp->prox_threshold_hi > 1000 || taos_cfgp->prox_threshold_lo > 900)
        {
            taos_cfgp->prox_threshold_hi = 800;
            taos_cfgp->prox_threshold_lo = 750;
        }

        if(taos_cfgp->prox_threshold_hi < 200)
        {
        	taos_cfgp->prox_threshold_hi = 200;
            taos_cfgp->prox_threshold_lo = 100;
        }

        input_report_rel(taos_datap->p_idev, REL_Y, taos_cfgp->prox_threshold_hi);
        input_report_rel(taos_datap->p_idev, REL_Z, taos_cfgp->prox_threshold_lo);
        input_sync(taos_datap->p_idev);
    }

error:
    taos_prox_threshold_set();
    taos_irq_ops(true, true);
    return ret;
}

static int taos_prox_off(void)
{
    int ret = 0;
    SENSOR_LOG_INFO("########  TAOS IOCTL PROX OFF  ########\n");

    if (true == (taos_datap->proximity_wakelock).locked)
    {
    	hrtimer_cancel(&taos_datap->prox_unwakelock_timer);
        taos_wakelock_ops(&(taos_datap->proximity_wakelock), false);
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x00))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL\n");
        return ret;
    }

    taos_datap->prox_on = 0;
    taos_datap->als_poll_time_mul = 1;

	if (true == taos_datap->als_on)
    {
        taos_sensors_als_poll_on();
	}

    if (true == taos_datap->irq_enabled)
    {
        taos_irq_ops(false, true);
    }
    return (ret);
}

static int tmd2772_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	if ((enable != 0) && (enable != 1))
    {
		SENSOR_LOG_ERROR("invalid value(%d)\n", enable);
		return -EINVAL;
	}

	if (enable)
    {
        taos_prox_on();
    }
    else
    {
        taos_prox_off();
    }

	return 0;
}

static int taos_prox_calibrate(void)
{  
    int ret = 0;
    int prox_sum = 0, prox_mean = 0, prox_max = 0, prox_min = 1024;
    u8 reg_cntrl = 0;
    int i = 0, j = 0;
    struct taos_prox_info prox_info = { 0 };

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), taos_cfgp->als_adc_time))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
        goto error;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME), taos_cfgp->prox_adc_time))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_TIME reg\n");
        goto error;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME), taos_cfgp->wait_time))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_WAIT_TIME reg\n");
        goto error;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_CFG), taos_cfgp->prox_config))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_CFG reg\n");
        goto error;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT), taos_cfgp->prox_pulse_cnt))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_COUNT reg\n");
        goto error;
    }

	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_OFFSET reg\n");
        goto error;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_GAIN reg\n");
        goto error;
    }

    reg_cntrl = 0 | (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL);
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
        goto error;
    }

    prox_sum = 0;
    prox_max = 0;
	prox_min = 1024;
    mdelay(30);

    for (i = 0, j = 0; i < (taos_datap->prox_calibrate_times); i++)
    {
        if ((ret = taos_prox_poll(&prox_info)) < 0)
        {
            SENSOR_LOG_ERROR("failed to taos_prox_poll\n");
        }
        else
        {
			j++;
	        prox_sum += prox_info.prox_data;
	        if (prox_info.prox_data > prox_max)
	            prox_max = prox_info.prox_data;
		    if (prox_info.prox_data < prox_min)
	            prox_min = prox_info.prox_data;

	        SENSOR_LOG_ERROR("i = %d, data = %d", i, prox_info.prox_data);
	        mdelay(30);
        }
    }

    if(j == 0)
	    goto error;

    prox_mean = prox_sum / j;
    taos_cfgp->prox_threshold_hi = ((((prox_max - prox_mean) * prox_calibrate_hi_param) + 50)/100) + prox_mean + 120;
    taos_cfgp->prox_threshold_lo = ((((prox_max - prox_mean) * prox_calibrate_lo_param) + 50)/100) + prox_mean + 40;

	if( (prox_mean > 700) || (taos_cfgp->prox_threshold_hi > 1000) || (taos_cfgp->prox_threshold_lo > 900))
	{
		taos_cfgp->prox_threshold_hi  = PROX_DEFAULT_THRESHOLD_HIGH;
		taos_cfgp->prox_threshold_lo  = PROX_DEFAULT_THRESHOLD_LOW;
		taos_cfgp->prox_config_offset = 0x0;
	}
    
    SENSOR_LOG_ERROR("TAOS:--------- prox_threshold_hi = %d\n", taos_cfgp->prox_threshold_hi);
    SENSOR_LOG_ERROR("TAOS:--------- rox_threshold_lo = %d\n", taos_cfgp->prox_threshold_lo);
    SENSOR_LOG_ERROR("TAOS:--------- prox_mean = %d\n", prox_mean);
    SENSOR_LOG_ERROR("TAOS:--------- prox_max = %d\n", prox_max);
	SENSOR_LOG_ERROR("TAOS:--------- prox_min = %d\n", prox_min);

    for (i = 0; i < sizeof(taos_triton_reg_init); i++)
    {
        if(i != 11)
        {
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i)), taos_triton_reg_init[i]))) < 0)
            {
                SENSOR_LOG_ERROR("failed to write triton_init reg\n");
                goto error;
            }
         }
    }

    input_report_rel(taos_datap->p_idev, REL_Y, taos_cfgp->prox_threshold_hi);
    input_report_rel(taos_datap->p_idev, REL_Z, taos_cfgp->prox_threshold_lo);
    input_sync(taos_datap->p_idev);

	return 1;
error:
    SENSOR_LOG_ERROR("exit\n");
	return -1;
}

static void taos_wakelock_ops(struct taos_wake_lock *wakelock, bool enable)
{
    if (enable == wakelock->locked)
    {
        SENSOR_LOG_INFO("doubule %s %s, return here\n",enable? "lock" : "unlock", wakelock->name);
        return;
    }

    if (enable)
    {
        wake_lock(&wakelock->lock);
    }
    else
    {
        wake_unlock(&wakelock->lock);
    }

    wakelock->locked = enable;

    SENSOR_LOG_INFO("%s %s \n",enable? "lock" : "unlock", wakelock->name);
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id)
{
    SENSOR_LOG_ERROR("enter\n");

    taos_datap->irq_work_status = true;
    taos_irq_ops(false, false);
    taos_wakelock_ops(&(taos_datap->proximity_wakelock), true);
    if (0 == queue_work(taos_datap->irq_work_queue, &taos_datap->irq_work))
    {
        SENSOR_LOG_INFO("failed to schedule_work\n");
    }

    SENSOR_LOG_ERROR("exit\n");
    return IRQ_HANDLED;
}

static void taos_irq_work_func(struct work_struct * work) //iVIZM
{
    int retry_times = 0, ret = 0;

    SENSOR_LOG_ERROR("enter\n");

    mutex_lock(&taos_datap->lock);

    if (wakeup_from_sleep)
    {
        SENSOR_LOG_ERROR("wakeup_from_sleep is true\n");
        mdelay(50);
        wakeup_from_sleep = false;
    }

    for (retry_times = 0; retry_times <= 50; retry_times++)
    {
        ret = taos_get_data();
        if (ret >= 0)
        {
            break;
        }
        mdelay(20);
    }
    taos_interrupts_clear();

    hrtimer_cancel(&taos_datap->prox_unwakelock_timer);
    taos_datap->irq_work_status = false;
    hrtimer_start(&taos_datap->prox_unwakelock_timer, ktime_set(3, 0), HRTIMER_MODE_REL);

    taos_irq_ops(true, true);
    SENSOR_LOG_INFO(" retry_times = %d\n",retry_times);
    mutex_unlock(&taos_datap->lock);
}

static int taos_get_data(void)
{
    int ret = 0;

    ret = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS));

    if (ret < 0)
    {
        SENSOR_LOG_ERROR("failed to read TAOS_TRITON_STATUS\n");
        return ret;
    }
    else
    {
        ret = taos_prox_threshold_set();
    }
    return ret;
}

static int taos_interrupts_clear(void)
{
    int ret = 0;
    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07)))) < 0)
    {
        SENSOR_LOG_ERROR("failed to clear interrupts\n");
        return ret;
    }

    return ret;
}

static int taos_prox_poll(struct taos_prox_info *prxp)
{
    int i = 0, ret = 0;
    u8 chdata[6] = { 0 };

    for (i = 0; i < 6; i++)
    {
        chdata[i] = (i2c_smbus_read_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_AUTO | (TAOS_TRITON_ALS_CHAN0LO + i))));
    }

    prxp->prox_clear = chdata[1];
    prxp->prox_clear <<= 8;
    prxp->prox_clear |= chdata[0];

    if (prxp->prox_clear > als_saturation_value)
    {
		SENSOR_LOG_ERROR("prox_clear %u > als_saturation_value %u\n", prxp->prox_clear, als_saturation_value);
        return -ENODATA;
    }
    prxp->prox_data = chdata[5];
    prxp->prox_data <<= 8;
    prxp->prox_data |= chdata[4];

    return ret;
}

static void taos_prox_calibrate_work_func(struct work_struct *work)
{
	taos_prox_calibrate();
}

static int taos_prox_offset_cal_prepare(void)
{
    int ret = 1;

    if (NULL != taos_cfgp)
    {
        taos_cfgp->prox_config_offset = 0;
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write TAOS_TRITON_PRX_OFFSET reg\n");
	        return ret;
        }
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
	    return -EINVAL;
    }
	return ret;
}

static int taos_prox_offset_calculate(int data, int target)
{
    int offset = 0;

    if (data > target)
    {
		offset = (data - target) * 8 / taos_cfgp->prox_pulse_cnt / 5;
    }
    else
    {
		offset = (target - data) * 8 / taos_cfgp->prox_pulse_cnt / 5 + 128;
    }

    SENSOR_LOG_ERROR("prox_offset = %d\n", offset);

    return offset;
}

static int tmd2772_prox_uncover_data_get(void)
{
    u8 i = 0, j = 0;
    int prox_sum = 0, ret = 0;
    static struct taos_prox_info prox_info_temp;

    mdelay(20);
    for (i = 0, j = 0; i < PROX_OFFSET_CAL_BUFFER_SIZE / 5; i++)
    {
        if ((ret = taos_prox_poll(&prox_info_temp)) < 0)
        {
            SENSOR_LOG_ERROR("failed to tmd2772_prox_read_data\n");
        }
        else
        {
            j++;
            prox_sum += prox_info_temp.prox_data;
        }
        mdelay(20);
    }

    if(j == 0)
    {
        ret = -1;
        goto error;
    }

    taos_datap->prox_uncover_data = prox_sum / j;
	//taos_datap->prox_thres_hi_min = taos_datap->prox_uncover_data + PROX_THRESHOLD_SAFE_DISTANCE + PROX_THRESHOLD_DISTANCE;

	taos_datap->prox_thres_hi_min = taos_datap->prox_uncover_data + PROX_THRESHOLD_SAFE_DISTANCE;
    taos_datap->prox_thres_hi_min = (taos_datap->prox_thres_hi_min > PROX_THRESHOLD_HIGH_MIN)? taos_datap->prox_thres_hi_min : PROX_THRESHOLD_HIGH_MIN;
    SENSOR_LOG_ERROR("prox_uncover_data = %d, prox_thres_hi_min = %d\n", taos_datap->prox_uncover_data, taos_datap->prox_thres_hi_min);
    taos_write_cal_file(PATH_PROX_UNCOVER_DATA, taos_datap->prox_uncover_data);

    return 0;

error:
    return ret;
}

static int taos_prox_offset_cal_process(void)
{  
    int ret = 0;
    int prox_sum = 0, prox_mean = 0;
    int i = 0, j = 0;
    u8 reg_cntrl = 0;
    struct taos_prox_info prox_info = { 0 };

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), taos_cfgp->als_adc_time))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write als_adc_time reg\n");
        goto error;
    }
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME), taos_cfgp->prox_adc_time))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write prox_adc_time reg\n");
        goto error;
    }
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME), taos_cfgp->wait_time))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write wait_time reg\n");
        goto error;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_CFG), taos_cfgp->prox_config))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write prox_config reg\n");
        goto error;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT), taos_cfgp->prox_pulse_cnt))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write prox_pulse_cnt reg\n");
        goto error;
    }

	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write prox_config_offset reg\n");
        goto error;
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write prox_gain reg\n");
        goto error;
    }

    reg_cntrl = 0 | (TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL);
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0)
    {
       SENSOR_LOG_ERROR("failed to write cntrl reg\n");
       goto error;
    }

    mdelay(30);

    for (i = 0, j = 0; i < (PROX_OFFSET_CAL_BUFFER_SIZE); i++)
    {
        if ((ret = taos_prox_poll(&prox_info)) < 0)
        {
            SENSOR_LOG_ERROR("failed to taos_prox_poll\n");
        }
		else
		{
			j++;
            prox_sum += prox_info.prox_data;
            SENSOR_LOG_ERROR("i = %d, data = %d\n", i, prox_info.prox_data);
            mdelay(30);
		}
    }

    if(j == 0)
	    goto error;

    prox_mean = prox_sum / j;
    SENSOR_LOG_ERROR("prox_mean = %d\n", prox_mean);

    taos_cfgp->prox_config_offset = taos_prox_offset_calculate(prox_mean, PROX_DATA_TARGET);

    if((ret = taos_write_cal_file(PATH_PROX_OFFSET, taos_cfgp->prox_config_offset)) < 0)
    {
        SENSOR_LOG_ERROR("failed to taos_write_cal_file\n");
        goto error;
    }

	if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
		(TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_OFFSET), taos_cfgp->prox_config_offset))) < 0)
	{
		SENSOR_LOG_ERROR(KERN_ERR "failed to write prox_config_offset reg\n");
		goto error;
	}

	//read prox data register after update offset register
	ret = tmd2772_prox_uncover_data_get();
	if (ret < 0)
	{
	     SENSOR_LOG_ERROR("failed to tmd2772_prox_uncover_data_get\n");
	     goto error;
	}

    for (i = 0; i < sizeof(taos_triton_reg_init); i++)
    {
        if(i != 11)
        {
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, 
                (TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i)), taos_triton_reg_init[i]))) < 0)
            {
                SENSOR_LOG_ERROR("failed to write triton_init reg\n");
                goto error;
            }
         }
    }
	return 1;
error:
    SENSOR_LOG_ERROR("ERROR\n");
	return (-1);
}

static void taos_prox_offset_cal_finish(void)
{
    if (true == (taos_datap->prox_on))
    {
        taos_prox_on();
    }
    else
    {
        taos_prox_off();
    }
}

static int taos_prox_offset_cal(void)
{
    int ret = 0;
    taos_datap->prox_offset_cal_result = false;

    if ((ret = taos_prox_offset_cal_prepare()) < 0)
	    goto error;

    mdelay(50);

    if ((ret= taos_prox_offset_cal_process()) >= 0)
    {
        taos_datap->prox_offset_cal_result = true;
    }

    taos_prox_offset_cal_finish();

error:
    return ret;
}

static void taos_prox_offset_cal_work_func(struct work_struct *work)
{
	taos_prox_offset_cal();
}

static enum hrtimer_restart  taos_prox_unwakelock_work_func(struct hrtimer *timer)
{
	SENSOR_LOG_ERROR("######## taos_prox_unwakelock_timer_func #########\n");

	if(false == taos_datap->irq_work_status)
	taos_wakelock_ops(&(taos_datap->proximity_wakelock),false);

	return HRTIMER_NORESTART;
}

static int taos_prox_threshold_set(void)
{
    int i = 0, ret = 0;
    u8 chdata[6] = { 0 };
    u16 proxdata = 0, cleardata = 0;
    char pro_buf[4] = { 0 };

    for (i = 0; i < 6; i++)
    {
        chdata[i] = (i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW| (TAOS_TRITON_ALS_CHAN0LO + i))));
    }
    cleardata = chdata[0] + (chdata[1] << 8);
    proxdata = chdata[4] + (chdata[5] << 8);

	if (pro_ft || flag_prox_debug)
    {
        pro_buf[0] = 0xFF;
        pro_buf[1] = 0xFF;
        pro_buf[2] = 0xFF;
        pro_buf[3] = 0xFF;

        for( i = 0; i < 4; i++ )
        {
            if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x08) + i, pro_buf[i]))) < 0)
            {
                 SENSOR_LOG_ERROR("failed to access reg\n");
                 return ret;
            }
        }

        if (pro_ft)
        {
            SENSOR_LOG_ERROR( "init the prox threshold");
        }

        if (flag_prox_debug)
        {
            mdelay(prox_debug_delay_time);
            SENSOR_LOG_ERROR( "proxdata = %d", proxdata);
			input_report_rel(taos_datap->p_idev, REL_MISC, proxdata > 0? proxdata:1);
        }
		pro_ft = false;
	} 
    else
    {
        if (proxdata < taos_cfgp->prox_threshold_lo)//FAR
        {
            pro_buf[0] = 0x0;
            pro_buf[1] = 0x0;
            pro_buf[2] = taos_cfgp->prox_threshold_hi & 0x0FF;
            pro_buf[3] = taos_cfgp->prox_threshold_hi >> 8;

    		SENSOR_LOG_ERROR( "Far!!! proxdata = %d\n", proxdata);
            input_report_rel(taos_datap->p_idev, REL_X, proxdata > 0? proxdata:1);
        }
        else
        {
            if (proxdata > taos_cfgp->prox_threshold_hi)//NEAR
            {
                if (cleardata > als_saturation_value)
                {
                	SENSOR_LOG_ERROR("cleardata %d > als_saturation_value %u int data\n", cleardata, als_saturation_value);
                	msleep(100);
                    return -ENODATA;
                }

                pro_buf[0] = taos_cfgp->prox_threshold_lo & 0x0ff;
                pro_buf[1] = taos_cfgp->prox_threshold_lo >> 8;
                pro_buf[2] = 0xFF;
                pro_buf[3] = 0xFF;

                SENSOR_LOG_ERROR("Near!!! proxdata = %d\n", proxdata);
                input_report_rel(taos_datap->p_idev, REL_X, proxdata);
            }
            else
            {
                if( (taos_cfgp->prox_threshold_hi - proxdata) > (proxdata - taos_cfgp->prox_threshold_lo))//FAR
                {
                    pro_buf[0] = 0x0;
                    pro_buf[1] = 0x0;
                    pro_buf[2] = taos_cfgp->prox_threshold_hi & 0xFF;
                    pro_buf[3] = taos_cfgp->prox_threshold_hi >> 8;

            		SENSOR_LOG_ERROR( "Far!!! proxdata = %d\n", proxdata);
                    input_report_rel(taos_datap->p_idev, REL_X, taos_cfgp->prox_threshold_lo - 1);
                }
                else//NEAR
                {
                    if (cleardata > als_saturation_value)
                    {
                    	SENSOR_LOG_ERROR("cleardata %d > als_saturation_value %u int data\n", cleardata, als_saturation_value);
                    	msleep(100);
                        return -ENODATA;
                    }

                    pro_buf[0] = taos_cfgp->prox_threshold_lo & 0xFF;
                    pro_buf[1] = taos_cfgp->prox_threshold_lo >> 8;
                    pro_buf[2] = 0xFF;
                    pro_buf[3] = 0xFF;

                    SENSOR_LOG_ERROR( "Near!!! proxdata = %d\n", proxdata);
                    input_report_rel(taos_datap->p_idev, REL_X, taos_cfgp->prox_threshold_hi + 1);
                }
            }
        }
    }

    input_sync(taos_datap->p_idev);

    for( i = 0; i < 4; i++)
    {
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|0x08) + i, pro_buf[i]))) < 0)
        {
             printk(KERN_ERR "TAOS: i2c_smbus_write_byte_data failed in taos prox threshold set\n");
             return (ret);
        }
    }

    return ret;
}

//als function implementation
static void taos_update_sat_als(void)
{
    u8 reg_val = 0;
    int ret = 0;
    u16 sat_als = 0;

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME)))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME\n");
        return;
    }

    reg_val = i2c_smbus_read_byte(taos_datap->client);

    sat_als = (256 - reg_val) << 10;
    als_saturation_value = sat_als * 80 / 100;
}

static int taos_sensors_als_poll_on(void)
{
    int  ret = 0, i = 0;
    u8 reg_val = 0, reg_cntrl = 0;

    SENSOR_LOG_ERROR("######## TAOS IOCTL ALS ON #########\n");

    for (i = 0; i < ALS_FILTER_DEPTH; i++)
    {
        lux_history[i] = -ENODATA;
    }

    if (taos_datap->prox_on)
    {
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), ALS_ADC_TIME_PROX_ON))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
            return ret;
        }
    }
    else
    {
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), taos_cfgp->als_adc_time))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
            return ret;
        }
    }

    reg_val = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN));
    reg_val = reg_val & 0xFC;
    reg_val = reg_val | (taos_cfgp->als_gain & 0x03);
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN), reg_val))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_GAIN reg\n");
        return ret;
    }

    reg_cntrl = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL));

    SENSOR_LOG_ERROR("TAOS_TRITON_CNTRL reg = 0x%02x\n", reg_cntrl);

    reg_cntrl |= (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON);
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
        return ret;
    }

	schedule_delayed_work(&taos_datap->als_poll_work, msecs_to_jiffies(200));

    taos_datap->als_on = true;

    taos_update_sat_als();

	return ret;
}	

static int taos_sensors_als_poll_off(void)
{
    int  ret = 0, i = 0;
    u8  reg_val = 0;

    SENSOR_LOG_ERROR("######## TAOS IOCTL ALS OFF #########\n");
    for (i = 0; i < ALS_FILTER_DEPTH; i++)
    {
        lux_history[i] = -ENODATA;
    }

    reg_val = i2c_smbus_read_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL));
    if ((reg_val & TAOS_TRITON_CNTL_PROX_DET_ENBL) == 0x00 && (0 == taos_datap->prox_on))
    {
        SENSOR_LOG_ERROR("TAOS_TRITON_CNTL_PROX_DET_ENBL = 0\n");

        reg_val = 0x00;
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_val))) < 0)
        {
           SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
           return ret;
        }
    }

    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
        (TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME), 0XFF))) < 0)
    {
       SENSOR_LOG_ERROR("failed to write TAOS_TRITON_ALS_TIME reg\n");
       return ret;
    }

    taos_datap->als_on = false;
    cancel_delayed_work_sync(&taos_datap->als_poll_work);
    taos_update_sat_als();

    return ret;
}

static int tmd2772_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	if ((enable != 0) && (enable != 1))
    {
		SENSOR_LOG_ERROR("invalid value(%d)\n", enable);
		return -EINVAL;
	}

	if (enable)
    {
        taos_sensors_als_poll_on();
    }
    else
    {
        taos_sensors_als_poll_off();
    }

	return 0;
}

static int tmd2772_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	taos_cfgp->als_poll_delay = delay_msec;
	return 0;
}

static int taos_get_lux(void)
{
    int raw_clear = 0, raw_ir = 0, raw_lux = 0;
    u32 lux = 0, ratio = 0;
    u8 dev_gain = 0;
    u16 Tint = 0;
    struct lux_data *p = NULL;
    int ret = 0;
    u8 chdata[4] = { 0 };
    int tmp = 0, i = 0, tmp_gain = 1;

    for (i = 0; i < 4; i++)
    {
        if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i))))) < 0)
        {
            SENSOR_LOG_ERROR("failed to read chan0/1/lo/hi reg\n");
            return ret;
        }
        chdata[i] = i2c_smbus_read_byte(taos_datap->client);
    }

    //tmp = (taos_cfgp->als_adc_time + 25)/50;//if atime =100  tmp = (atime+25)/50=2.5   time = 2.7*(256-atime)=  412.5
    tmp = (ALS_TIME_PARAM + 25) / 50;//if atime =100  tmp = (atime+25)/50=2.5   time = 2.7*(256-atime)=  412.5

    TritonTime.numerator = 1;
    TritonTime.denominator = tmp;

    //tmp = 300 * taos_cfgp->als_adc_time;//tmp = 300*atime  400
    tmp = 300 * ALS_TIME_PARAM;//tmp = 300*atime  400

    if(tmp > 65535)
        tmp = 65535;

    TritonTime.saturation = tmp;

    raw_clear = chdata[1];
    raw_clear <<= 8;
    raw_clear |= chdata[0];
    raw_ir    =  chdata[3];
    raw_ir    <<= 8;
    raw_ir    |= chdata[2];

    raw_clear *= ((taos_cfgp->als_scale_factor) * tmp_gain);
    raw_ir *= (taos_cfgp->prox_scale_factor);

    if(raw_ir > raw_clear)
    {
        raw_lux = raw_ir;
        raw_ir = raw_clear;
        raw_clear = raw_lux;
    }

    dev_gain = als_gain_table[taos_cfgp->als_gain & 0x3];

    if(raw_clear >= TritonTime.saturation)
    {
        SENSOR_LOG_ERROR("raw_clear >= TritonTime.saturation\n");
        return ALS_MAX_LUX;
    }

    if(raw_ir >= TritonTime.saturation)
    {
        SENSOR_LOG_ERROR("raw_ir >= TritonTime.saturation\n");
        return ALS_MAX_LUX;
    }

    if(raw_clear == 0)
        return 0;

    if(dev_gain == 0 || dev_gain > 127)
    {
        SENSOR_LOG_ERROR("dev_gain = 0 or > 127\n");
        return (-1);
    }

    if(TritonTime.denominator == 0)
    {
        SENSOR_LOG_ERROR("TritonTime.denominator = 0\n");
        return (-1);
    }

    ratio = (raw_ir << 15) / raw_clear;

    for (p = lux_tablep; p->ratio && p->ratio < ratio; p++);

#ifdef WORK_UES_POLL_MODE
    if(!p->ratio)
    {
        if(lux_history[0] < 0)
            return 0;
        else
            return lux_history[0];
    }
#endif

    //Tint = taos_cfgp->als_adc_time;
    Tint = ALS_TIME_PARAM;

    raw_clear = ((raw_clear * 400 + (dev_gain >> 1)) / dev_gain + (Tint >> 1)) / Tint;
    raw_ir = ((raw_ir * 400 +(dev_gain >> 1)) / dev_gain + (Tint >> 1)) / Tint;
    lux = ((raw_clear * (p->clear)) - (raw_ir * (p->ir)));
    lux = (lux + 32000) / 64000;
    if(lux > ALS_MAX_LUX)
    {
        SENSOR_LOG_ERROR("lux > ALS_MAX_LUX\n");
        lux = ALS_MAX_LUX;
    }
    return lux;
}

static int taos_lux_filter(int lux)
{
    static u8 middle[] = {1, 0, 2, 0, 0, 2, 0, 1};
    int index = 0;

    lux_history[2] = lux_history[1];
    lux_history[1] = lux_history[0];
    lux_history[0] = lux;

    if(lux_history[2] < 0)
    {
        if(lux_history[1] > 0)
            return lux_history[1];
        else
            return lux_history[0];
    }

    index = 0;
    if(lux_history[0] > lux_history[1])
        index += 4;
    if(lux_history[1] > lux_history[2])
        index += 2;
    if(lux_history[0] > lux_history[2])
        index++;

    return(lux_history[middle[index]]);
}

static int taos_als_gain_set(unsigned als_gain)
{
    int ret = 0;
    if (NULL != taos_cfgp)
    {
        taos_cfgp->prox_gain = (taos_cfgp->prox_gain & 0xFC) | als_gain;
        taos_cfgp->als_gain = taos_cfgp->prox_gain & 0x03;

        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client,
            (TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN), taos_cfgp->prox_gain))) < 0)
        {
            SENSOR_LOG_ERROR("failed to write TAOS_TRITON_GAIN reg\n");
            return -EINVAL;
        }
    }
    else
    {
        SENSOR_LOG_ERROR("taos_cfgp is NULL\n");
        return -EINVAL;
    }

    return ret;
}

static int taos_als_get_data(void)
{
    int ret = 0;
    u8 reg_val = 0;
    int lux_val = 0;

    if((NULL == taos_datap) || (NULL == taos_cfgp))
    {
        SENSOR_LOG_ERROR("taos_datap is NULL\n");
        return -ENODATA;
    }

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL)))) < 0)
    {
        SENSOR_LOG_ERROR("failed to access TAOS_TRITON_CNTRL reg\n");
        return ret;
    }
    reg_val = i2c_smbus_read_byte(taos_datap->client);

    if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON)) != (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
        return -ENODATA;

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0)
    {
        SENSOR_LOG_ERROR("failed to access TAOS_TRITON_STATUS reg\n");
        return (ret);
    }
    reg_val = i2c_smbus_read_byte(taos_datap->client);

    if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID)
        return -ENODATA;

    if ((lux_val = taos_get_lux()) < 0)
    {
        SENSOR_LOG_ERROR("failed to taos_get_lux = %d\n", lux_val);
    }

    if (lux_val < ALS_GAIN_DIVIDE && taos_cfgp->als_gain != ALS_GAIN_8X)
    {
        taos_als_gain_set(ALS_GAIN_8X);
    }
    else
    {
        if (lux_val > ALS_GAIN_DIVIDE && taos_cfgp->als_gain != ALS_GAIN_1X)
        {
            taos_als_gain_set(ALS_GAIN_1X);
        }
    }

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME)))) < 0)
    {
        SENSOR_LOG_ERROR("failed to access TAOS_TRITON_ALS_TIME\n");
        return ret;
    }

    reg_val = i2c_smbus_read_byte(taos_datap->client);

    if (flag_als_debug)
    {
        SENSOR_LOG_ERROR("als_adc_time = %d lux_val = %d\n", reg_val, lux_val);
    }

    if (reg_val != ALS_ADC_TIME_DEFAULT)
    {
        lux_val = (lux_val * (101 - (0xFF - reg_val)))/20;
        SENSOR_LOG_ERROR("lux_val multiply \n");
    }

    lux_val = taos_lux_filter(lux_val);

    if (flag_als_debug)
    {
        SENSOR_LOG_ERROR("lux_val = %d", lux_val);
    }

    input_report_rel(taos_datap->a_idev, REL_X, lux_val+1);
    input_sync(taos_datap->a_idev);

    return ret;
}

static void taos_als_poll_work_func(struct work_struct *work)
{
	taos_als_get_data();
	schedule_delayed_work(&taos_datap->als_poll_work, msecs_to_jiffies(taos_datap->als_poll_time_mul * taos_cfgp->als_poll_delay));
}

static void tmd2772_data_init(void)
{
    taos_datap->als_on  = false;
    taos_datap->prox_on = false;
    taos_datap->als_poll_time_mul = 1;
    taos_datap->prox_name = "proximity";
    taos_datap->als_name  = "light";
    taos_datap->chip_name = "tmd2772";
    taos_datap->prox_calibrate_result = false;
    taos_datap->prox_offset_cal_result = false;
	taos_datap->prox_offset_cal_verify = true;
	taos_datap->prox_calibrate_verify = true;
    taos_datap->prox_thres_hi_max = PROX_THRESHOLD_HIGH_MAX;
    taos_datap->prox_thres_hi_min = PROX_THRESHOLD_HIGH_MIN;
    taos_datap->prox_data_max     = PROX_DATA_MAX;
    taos_datap->prox_calibrate_times = 10;
    taos_datap->prox_calibrate_flag = true;//true :auto_calibrate,false :manual_calibrate
    taos_datap->prox_manual_calibrate_threshold = 0;
	taos_datap->prox_uncover_data = 0;
    taos_datap->proximity_wakelock.name = "proximity-wakelock";
    taos_datap->proximity_wakelock.locked = false;
    taos_datap->irq_work_status = false;
    taos_datap->irq_enabled = true;
}

static int tmd2772_parse_dt(struct taos_data *chip)
{
  	int rc = 0;
	u32 temp_val = 0;
	struct device_node *np = chip->client->dev.of_node;

    /* irq gpio */
	rc = of_get_named_gpio_flags(np, "ams,irq-gpio", 0, NULL);
	if (rc < 0)
    {
		SENSOR_LOG_ERROR("Unable to read irq gpio\n");
		return rc;
	}
	chip->irq_pin_num = rc;
    SENSOR_LOG_INFO("irq_pin_num is %d\n", chip->irq_pin_num);

	rc = of_property_read_u32(np, "ams,pul_cnt", &temp_val);
	if (rc && (rc != -EINVAL)) {
		SENSOR_LOG_ERROR("Unable to read ams,pul_cnt\n");
		return rc;
	}
    else {
		chip->cfg->prox_pulse_cnt = temp_val;
		SENSOR_LOG_INFO("prox_pulse_cnt is %d\n", chip->cfg->prox_pulse_cnt);
	}
    return 0;
}

/* POWER SUPPLY VOLTAGE RANGE */
#define TMD2772_VDD_MIN_UV	2000000
#define TMD2772_VDD_MAX_UV	3300000
#define TMD2772_VIO_MIN_UV	1750000
#define TMD2772_VIO_MAX_UV	1950000

static int tmd2772_power_init(struct taos_data *chip, bool on)
{
	int rc = 0;

	SENSOR_LOG_ERROR("on = %d\n", on);

	if (!on) {
		if (regulator_count_voltages(chip->vdd) > 0)
			regulator_set_voltage(chip->vdd, 0,
				TMD2772_VDD_MAX_UV);
		regulator_put(chip->vdd);

		if (regulator_count_voltages(chip->vio) > 0)
			regulator_set_voltage(chip->vio, 0,
				TMD2772_VIO_MAX_UV);
		regulator_put(chip->vio);
	} else {
		chip->vdd = regulator_get(&chip->client->dev, "vdd");
		if (IS_ERR(chip->vdd)) {
			rc = PTR_ERR(chip->vdd);
			dev_err(&chip->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(chip->vdd) > 0) {
			rc = regulator_set_voltage(chip->vdd,
				TMD2772_VDD_MIN_UV, TMD2772_VDD_MAX_UV);
			if (rc) {
				dev_err(&chip->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto err_vdd_set;
			}
		}

		chip->vio = regulator_get(&chip->client->dev, "vio");
		if (IS_ERR(chip->vio)) {
			rc = PTR_ERR(chip->vio);
			dev_err(&chip->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}

		if (regulator_count_voltages(chip->vio) > 0) {
			rc = regulator_set_voltage(chip->vio,
				TMD2772_VIO_MIN_UV, TMD2772_VIO_MAX_UV);
			if (rc) {
				dev_err(&chip->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set;
			}
		}

		rc = regulator_enable(chip->vdd);
		if (rc) {
			dev_err(&chip->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(chip->vio);
		if (rc) {
			dev_err(&chip->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
	}
	return 0;

err_vio_enable:
	regulator_disable(chip->vdd);
err_vdd_enable:
	if (regulator_count_voltages(chip->vio) > 0)
		regulator_set_voltage(chip->vio, 0, TMD2772_VIO_MAX_UV);
err_vio_set:
	regulator_put(chip->vio);
err_vio_get:
	if (regulator_count_voltages(chip->vdd) > 0)
		regulator_set_voltage(chip->vdd, 0, TMD2772_VDD_MAX_UV);
err_vdd_set:
	regulator_put(chip->vdd);
	return rc;
}

static int tmd2772_probe(struct i2c_client *clientp, const struct i2c_device_id *idp)
{
    int ret = 0, chip_id = -1;
    u16 sat_als = 0;

    SENSOR_LOG_ERROR("probe start\n");

    if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
        SENSOR_LOG_ERROR("unsupport i2c smbus byte data functions\n");
        return -EOPNOTSUPP;
    }

    taos_datap = kmalloc(sizeof(struct taos_data), GFP_KERNEL);
    if (!taos_datap)
    {
         SENSOR_LOG_ERROR("failed to kmalloc for struct taos_data\n");
         return -ENOMEM;
    }

    taos_datap->client = clientp;
    i2c_set_clientdata(clientp, taos_datap);

	ret = tmd2772_power_init(taos_datap, 1);
	if (ret < 0)
		goto power_init_failed;

    chip_id = i2c_smbus_read_byte_data(clientp, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CHIPID));

	SENSOR_LOG_ERROR("chip_id = 0x%x TMD27713 = 0x30, TMD27723 = 0x39\n", chip_id);

    if(chip_id != TMD2772_DEVICE_ID)
    {
        SENSOR_LOG_ERROR("chip id does not match TMD27723(0x39)\n");
        ret = -ENODEV;
        goto read_chip_id_failed;
    }

    INIT_WORK(&(taos_datap->irq_work), taos_irq_work_func);

	mutex_init(&(taos_datap->lock));
    wake_lock_init(&taos_datap->proximity_wakelock.lock, WAKE_LOCK_SUSPEND, "proximity-wakelock");

    tmd2772_data_init();

    strlcpy(clientp->name, TAOS_DEVICE_NAME, I2C_NAME_SIZE);

    if (!(taos_cfgp = kmalloc(sizeof(struct taos_cfg), GFP_KERNEL)))
    {
        SENSOR_LOG_ERROR("failed to kmalloc for struct taos_cfg\n");
        ret = -ENOMEM;
        goto kmalloc_taos_cfgp_failed;
    }
	taos_datap->cfg = taos_cfgp;

    ret = tmd2772_parse_dt(taos_datap);
	if(ret)
		goto parse_dt_failed;

    taos_cfgp->als_scale_factor   = ALS_SCALE_FACTOR_DEFAULT;
    taos_cfgp->als_gain           = ALS_GAIN_DEFAULT;
    taos_cfgp->als_poll_delay     = ALS_POLL_DELAY_DEFAULT;

    taos_cfgp->als_adc_time       = ALS_ADC_TIME_DEFAULT;
    sat_als = (256 - taos_cfgp->als_adc_time) << 10;
    als_saturation_value = sat_als * 80 / 100;

    taos_cfgp->prox_threshold_hi  = PROX_DEFAULT_THRESHOLD_HIGH;
    taos_cfgp->prox_threshold_lo  = PROX_DEFAULT_THRESHOLD_LOW;
    taos_cfgp->prox_scale_factor  = PROX_SCALE_FACTOR_DEFAULT;
    taos_cfgp->prox_adc_time      = PROX_ADC_TIME_DEFAULT;
    taos_cfgp->prox_intr_filter   = PROX_INTR_FILTER_DEFAULT;
    taos_cfgp->prox_config        = PROX_CONFIG_DEFAULT;
    taos_cfgp->prox_gain          = PROX_GAIN_DEFAULT;
    taos_cfgp->prox_config_offset = PROX_CONFIG_OFFSET_DEFAULT;

    taos_cfgp->wait_time          = WAIT_TIME_DEFAULT;

    SENSOR_LOG_ERROR("power down\n");
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CNTRL), 0x00))) < 0)
    {
        SENSOR_LOG_ERROR("failed to write TAOS_TRITON_CNTRL reg\n");
        goto power_down_failed;
    }

    taos_datap->irq_work_queue = create_singlethread_workqueue("taos_work_queue");
    if (!taos_datap->irq_work_queue)
    {
        ret = -ENOMEM;
        SENSOR_LOG_ERROR( "failed to create irq_work_queue");
        goto power_down_failed;
    }

    ret = gpio_request(taos_datap->irq_pin_num, "ALS_PS_INT");
    if (ret)
    {
        SENSOR_LOG_ERROR("gpio %d is busy and then free it\n", taos_datap->irq_pin_num);

        gpio_free(taos_datap->irq_pin_num);
        ret = gpio_request(taos_datap->irq_pin_num, "ALS_PS_INT");
        if (ret) 
        {
            SENSOR_LOG_ERROR("gpio %d is busy and then free it\n",taos_datap->irq_pin_num);
            goto gpio_request_failed;
        }
    }
    else
    {
        SENSOR_LOG_ERROR("get gpio %d successfully\n", taos_datap->irq_pin_num);
    }

    taos_datap->client->irq = gpio_to_irq(taos_datap->irq_pin_num);

    ret = request_irq(taos_datap->client->irq, taos_irq_handler, IRQF_TRIGGER_FALLING, "taos_irq", taos_datap);
    if (ret)
        goto request_irq_failed;

    taos_irq_ops(false, true);

    INIT_DELAYED_WORK(&taos_datap->als_poll_work, taos_als_poll_work_func);
    INIT_DELAYED_WORK(&taos_datap->prox_calibrate_work, taos_prox_calibrate_work_func);
    INIT_DELAYED_WORK(&taos_datap->prox_offset_cal_work, taos_prox_offset_cal_work_func);

    hrtimer_init(&taos_datap->prox_unwakelock_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    (taos_datap->prox_unwakelock_timer).function = taos_prox_unwakelock_work_func;
    
    proximity_class = class_create(THIS_MODULE, "proximity");
	if (IS_ERR(proximity_class))
    {
		ret = PTR_ERR(proximity_class);
		proximity_class = NULL;
		goto request_irq_failed;
	}

    light_class = class_create(THIS_MODULE, "light");
    if (IS_ERR(light_class))
    {
		ret = PTR_ERR(light_class);
		light_class = NULL;
		goto create_light_class_failed;
	}

    taos_datap->proximity_dev = device_create(proximity_class, NULL, tmd2772_proximity_dev_t, &tmd2772_driver ,"proximity");
    if (IS_ERR(taos_datap->proximity_dev))
    {
        ret = PTR_ERR(taos_datap->proximity_dev);
        SENSOR_LOG_ERROR("failed to create device proximity\n");
        goto create_proximity_dev_failed;
    }

    taos_datap->light_dev = device_create(light_class, NULL, tmd2772_light_dev_t, &tmd2772_driver ,"light");
    if (IS_ERR(taos_datap->light_dev))
    {
        ret = PTR_ERR(taos_datap->light_dev);
        SENSOR_LOG_ERROR("failed to create device light\n");
        goto create_light_dev_failed;
    }

    taos_datap->p_idev = input_allocate_device();
    if (!taos_datap->p_idev)
    {
        SENSOR_LOG_ERROR("failed to create input_dev '%s'\n", taos_datap->prox_name);
        ret = -ENODEV;
        goto input_p_alloc_failed;
    }
    taos_datap->p_idev->name = taos_datap->prox_name;
    taos_datap->p_idev->id.bustype = BUS_I2C;
    dev_set_drvdata(&taos_datap->p_idev->dev, taos_datap);

    ret = input_register_device(taos_datap->p_idev);
    if (ret)
    {
        SENSOR_LOG_ERROR("failed to register input '%s'\n", taos_datap->prox_name);
        goto input_p_register_failed;
    }

    set_bit(EV_REL,   taos_datap->p_idev->evbit);
    set_bit(REL_X,    taos_datap->p_idev->relbit);
    set_bit(REL_Y,    taos_datap->p_idev->relbit);
    set_bit(REL_Z,    taos_datap->p_idev->relbit);
    set_bit(REL_MISC, taos_datap->p_idev->relbit);

    taos_datap->a_idev = input_allocate_device();
	if (!taos_datap->a_idev)
    {
		SENSOR_LOG_ERROR("failed to create input_dev '%s'\n", taos_datap->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	taos_datap->a_idev->name = taos_datap->als_name;
	taos_datap->a_idev->id.bustype = BUS_I2C;

    set_bit(EV_REL, taos_datap->a_idev->evbit);
    set_bit(REL_X,  taos_datap->a_idev->relbit);
    set_bit(REL_Y,  taos_datap->a_idev->relbit);

	dev_set_drvdata(&taos_datap->a_idev->dev, taos_datap);
	ret = input_register_device(taos_datap->a_idev);
	if (ret)
	{
		SENSOR_LOG_ERROR("fail to register input '%s'\n", taos_datap->prox_name);
		goto input_a_register_failed;
	}

	dev_set_drvdata(taos_datap->proximity_dev, taos_datap);
	dev_set_drvdata(taos_datap->light_dev, taos_datap);

    ret = create_sysfs_interfaces_prox(taos_datap->proximity_dev);
    if(ret)
        goto input_a_register_failed;

    ret = create_sysfs_interfaces_light(taos_datap->light_dev);
    if(ret)
        goto input_a_register_failed;

	/* Register to sensors class */
	taos_datap->als_cdev = sensors_light_cdev;
	taos_datap->als_cdev.sensors_enable = tmd2772_als_set_enable;
	taos_datap->als_cdev.sensors_poll_delay = tmd2772_als_poll_delay;
	taos_datap->ps_cdev = sensors_proximity_cdev;
	taos_datap->ps_cdev.sensors_enable = tmd2772_ps_set_enable;
	taos_datap ->ps_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&clientp->dev, &taos_datap->als_cdev);
	if (ret)
    {
		SENSOR_LOG_ERROR("Unable to register to sensors class: %d\n", ret);
		goto input_a_register_failed;
	}

	ret = sensors_classdev_register(&clientp->dev, &taos_datap->ps_cdev);
	if (ret)
    {
		SENSOR_LOG_ERROR("Unable to register to sensors class: %d\n", ret);
		goto classdev_register_failed;
	}

    SENSOR_LOG_ERROR("probe OK\n");

	return 0;

classdev_register_failed:
	sensors_classdev_unregister(&taos_datap->als_cdev);
    input_unregister_device(taos_datap->a_idev);
input_a_register_failed:
    input_free_device(taos_datap->a_idev);
input_a_alloc_failed:
    input_unregister_device(taos_datap->p_idev);
input_p_register_failed:
    input_free_device(taos_datap->p_idev);
input_p_alloc_failed:
    device_unregister(taos_datap->light_dev);
    taos_datap->light_dev = NULL;
create_light_dev_failed:
    device_unregister(taos_datap->proximity_dev);
    taos_datap->proximity_dev = NULL;
create_proximity_dev_failed:
    class_destroy(light_class);
create_light_class_failed:
    class_destroy(proximity_class);
request_irq_failed:
    gpio_free(taos_datap->irq_pin_num);
gpio_request_failed:
    destroy_workqueue(taos_datap->irq_work_queue);
power_down_failed:
parse_dt_failed:
    kfree(taos_cfgp);
kmalloc_taos_cfgp_failed:
read_chip_id_failed:
	tmd2772_power_init(taos_datap, 0);
power_init_failed:
    kfree(taos_datap);

    SENSOR_LOG_ERROR("probe failed\n");
    return ret;
}

#ifdef CONFIG_PM_SLEEP
static int taos_resume(struct i2c_client *client) 
{
	int ret = 0;

    SENSOR_LOG_ERROR("enter\n");
	if(1 == taos_datap->prox_on)
    {
        SENSOR_LOG_ERROR( "disable irq wakeup\n");
		ret = disable_irq_wake(taos_datap->client->irq);
        if(ret < 0)
		    SENSOR_LOG_ERROR("failed to disable_irq_wake\n");
	}

	if (true == taos_datap->als_on)
	{
		SENSOR_LOG_INFO("recovery light sensor work\n");
		schedule_delayed_work(&taos_datap->als_poll_work, msecs_to_jiffies(1000));
	}

    SENSOR_LOG_ERROR("exit\n");
    return ret;
}

static int taos_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;

    SENSOR_LOG_ERROR("enter\n");
	if(1 == taos_datap->prox_on)
    {
        SENSOR_LOG_ERROR("enable irq wakeup\n");
       	ret = enable_irq_wake(taos_datap->client->irq);
        if(ret < 0)
        {
    		SENSOR_LOG_ERROR("failed to enable_irq_wake\n");
        }
    }
    wakeup_from_sleep = true;

	if (true == taos_datap->als_on)
	{
	    SENSOR_LOG_INFO("cancel light sensor work start\n");
	    cancel_delayed_work(&(taos_datap->als_poll_work));
		SENSOR_LOG_INFO("cancel light sensor work ...\n");
	    flush_delayed_work(&(taos_datap->als_poll_work));
	    SENSOR_LOG_INFO("cancel light sensor work end\n");
	}

    SENSOR_LOG_ERROR("exit\n");
    return ret;
}
#endif

static int tmd2772_remove(struct i2c_client *client)
{
	tmd2772_power_init(taos_datap, 0);
    return 0;
}

// driver init
static int __init taos_init(void)
{
    return i2c_add_driver(&tmd2772_driver);
}

// driver exit
static void __exit taos_exit(void)
{
	i2c_del_driver(&tmd2772_driver);
}

MODULE_AUTHOR("John Koshi - Surya Software");
MODULE_DESCRIPTION("TAOS ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");

module_init(taos_init);
module_exit(taos_exit);
