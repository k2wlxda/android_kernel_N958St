/* drivers/misc/akm09911.c - akm09911 compass driver
 *
 * Copyright (C) 2013-2014 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/i2c/akm09911.h>

#define AKM_POWER_ALWAYS_ON

#define LOG_TAG "AKM_COMPASS"
//#define DEBUG_ON

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s: %d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s: %d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

#define AKM_DEBUG_IF			1
#define AKM_HAS_RESET			1
#define AKM_INPUT_DEVICE_NAME		"compass"
#define AKM_DRDY_TIMEOUT_MS		100
#define AKM_BASE_NUM			10

/* POWER SUPPLY VOLTAGE RANGE */
#define AKM09911_VDD_MIN_UV		2000000
#define AKM09911_VDD_MAX_UV		3300000
#define AKM09911_VIO_MIN_UV		1750000
#define AKM09911_VIO_MAX_UV		1950000

#define STATUS_ERROR(st)		(((st) & 0x09) != 0x01)
#define AKM_IS_MAG_DATA_ENABLED()	(akm->enable_flag & (1 << MAG_DATA_FLAG))

static char const *const device_link_name = "i2c";
static dev_t const akm_compass_device_dev_t = MKDEV(MISC_MAJOR, 240);

/* Save last device state for power down */
struct akm_sensor_state {
	bool power_on;
	uint8_t mode;
};

struct akm_compass_data {
	struct i2c_client	*i2c;
	struct input_dev	*input;
	struct device		*class_dev;
	struct class		*compass;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct pinctrl_state	*pin_sleep;
	struct sensors_classdev	cdev;
	struct delayed_work	dwork;
	struct mutex		op_mutex;

	wait_queue_head_t	drdy_wq;
	wait_queue_head_t	open_wq;

	/* These two buffers are initialized at start up.
	 * After that, the value is not changed
	 */
	uint8_t sense_info[AKM_SENSOR_INFO_SIZE];
	uint8_t sense_conf[AKM_SENSOR_CONF_SIZE];

	struct	mutex sensor_mutex;
	uint8_t	sense_data[AKM_SENSOR_DATA_SIZE];
	struct mutex accel_mutex;
	int16_t accel_data[3];

	/* Positive value means the device is working.
	 * 0 or negative value means the device is not woking,
	 * i.e. in power-down mode.
	 */
	int8_t	is_busy;

	struct mutex	val_mutex;
	uint32_t		enable_flag;
	int64_t			delay[AKM_NUM_SENSORS];

	atomic_t	active;
	atomic_t	drdy;

	char layout;
	int	irq;
	int	gpio_rstn;
	int	power_enabled;
	int	auto_report;
	struct regulator	*vdd;
	struct regulator	*vio;
	struct akm_sensor_state state;
};

static struct sensors_classdev sensors_cdev = {
	.name = "akm09911-mag",
	.vendor = "Asahi Kasei Microdevices Corporation",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1228.8",
	.resolution = "0.6",
	.sensor_power = "0.35",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
static struct akm_compass_data *s_akm;
static int mag_data[3] = {0};

static int akm_compass_power_set(struct akm_compass_data *data, bool on);

/***** I2C I/O function ***********************************************/
static int akm_i2c_rxdata(
	struct i2c_client *i2c,
	uint8_t *rxData,
	int length)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		SENSOR_LOG_ERROR("transfer failed.\n");
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		SENSOR_LOG_ERROR("transfer failed(size error).\n");
		return -ENXIO;
	}

	return 0;
}

static int akm_i2c_txdata(
	struct i2c_client *i2c,
	uint8_t *txData,
	int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		SENSOR_LOG_ERROR("transfer failed.\n");
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		SENSOR_LOG_ERROR("transfer failed(size error).\n");
		return -ENXIO;
	}

	return 0;
}

/***** akm miscdevice functions *************************************/
static int AKECS_Set_CNTL(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	uint8_t buffer[2];
	int err;

	mutex_lock(&akm->sensor_mutex);
	/* Busy check */
	if (akm->is_busy > 0) {
		SENSOR_LOG_ERROR("device is busy.\n");
		err = -EBUSY;
	} else {
		/* Set measure mode */
		buffer[0] = AKM_REG_MODE;
		buffer[1] = mode;
		err = akm_i2c_txdata(akm->i2c, buffer, 2);
		if (err < 0) {
			SENSOR_LOG_ERROR("Can not set CNTL.\n");
		} else {
			/* Set flag */
			akm->is_busy = 1;
			atomic_set(&akm->drdy, 0);
			/* wait at least 100us after changing mode */
			udelay(100);
		}
	}
	mutex_unlock(&akm->sensor_mutex);

	return err;
}

static int AKECS_Set_PowerDown(
	struct akm_compass_data *akm)
{
	uint8_t buffer[2];
	int err;

	mutex_lock(&akm->sensor_mutex);

	/* Set powerdown mode */
	buffer[0] = AKM_REG_MODE;
	buffer[1] = AKM_MODE_POWERDOWN;
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	if (err < 0) {
		SENSOR_LOG_ERROR("Can not set to powerdown mode.\n");
	} else {
		SENSOR_LOG_DEBUG("Powerdown mode is set.\n");
		/* wait at least 100us after changing mode */
		udelay(100);
	}
	/* Clear status */
	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);

	return err;
}

static int AKECS_Reset(
	struct akm_compass_data *akm,
	int hard)
{
	int err = 0;
	uint8_t buffer[2] = {0};

	mutex_lock(&akm->sensor_mutex);

	buffer[0] = AKM_REG_RESET;
	buffer[1] = AKM_RESET_DATA;
	err = akm_i2c_txdata(akm->i2c, buffer, 2);
	if (err < 0) {
		SENSOR_LOG_ERROR("Can not set SRST bit.\n");
	} else {
		SENSOR_LOG_DEBUG("Soft reset is done.\n");
	}

	/* Device will be accessible 100 us after */
	udelay(100);
	/* Clear status */
	akm->is_busy = 0;
	atomic_set(&akm->drdy, 0);
	mutex_unlock(&akm->sensor_mutex);

	return err;
}

static int AKECS_SetMode(
	struct akm_compass_data *akm,
	uint8_t mode)
{
	int err;

	switch (mode & 0x1F) {
	case AKM_MODE_SNG_MEASURE:
	case AKM_MODE_SELF_TEST:
	case AKM_MODE_FUSE_ACCESS:
		err = AKECS_Set_CNTL(akm, mode);
		break;
	case AKM_MODE_POWERDOWN:
		err = AKECS_Set_PowerDown(akm);
		break;
	default:
		SENSOR_LOG_ERROR("Unknown mode(%d).\n", mode);
		return -EINVAL;
	}

	return err;
}

static void AKECS_SetYPR(
	struct akm_compass_data *akm,
	int *rbuf)
{
	uint32_t ready;

	/* No events are reported */
	if (!rbuf[0]) {
		SENSOR_LOG_DEBUG("Don't waste a time.\n");
		return;
	}

	mutex_lock(&akm->val_mutex);
	ready = (akm->enable_flag & (uint32_t)rbuf[0]);
	mutex_unlock(&akm->val_mutex);

	/* Report acceleration sensor information */
	/*
	if (ready & ACC_DATA_READY) {
		input_report_abs(akm->input, ABS_X, rbuf[1]);
		input_report_abs(akm->input, ABS_Y, rbuf[2]);
		input_report_abs(akm->input, ABS_Z, rbuf[3]);
		input_report_abs(akm->input, ABS_RX, rbuf[4]);
	}
	*/
	/* Report magnetic vector information */
	if (ready & MAG_DATA_READY) {
		input_report_abs(akm->input, ABS_RY, rbuf[5]);
		input_report_abs(akm->input, ABS_RZ, rbuf[6]);
		input_report_abs(akm->input, ABS_THROTTLE, rbuf[7]);
		input_report_abs(akm->input, ABS_RUDDER, rbuf[8]);


		/* Report uncalibrated magnetic information */
		if (akm->enable_flag & UNCALI_MAG_DATA_READY) {
			input_report_abs(akm->input, ABS_MISC, mag_data[0]);
			input_report_abs(akm->input, ABS_MT_TOUCH_MAJOR, mag_data[1]);
			input_report_abs(akm->input, ABS_MT_TOUCH_MINOR, mag_data[2]);
		}
	}

	/* Report fusion sensor information */
	if (ready & FUSION_DATA_READY) {
		/* Gyroscope sensor */
		/*
		input_report_abs(akm->input, ABS_WHEEL, rbuf[9]);
		input_report_abs(akm->input, ABS_GAS,   rbuf[10]);
		input_report_abs(akm->input, ABS_BRAKE, rbuf[11]);
		input_report_abs(akm->input, ABS_HAT0X, rbuf[12]);
		*/
		/* Orientation */
		if (akm->enable_flag & ORI_DATA_READY) {
			input_report_abs(akm->input, ABS_HAT0Y, rbuf[13]);
			input_report_abs(akm->input, ABS_HAT1X, rbuf[14]);
			input_report_abs(akm->input, ABS_HAT1Y, rbuf[15]);
		}
		/* Gravity */
		/*
		input_report_abs(akm->input, ABS_HAT2X, rbuf[16]);
		input_report_abs(akm->input, ABS_HAT2Y, rbuf[17]);
		input_report_abs(akm->input, ABS_HAT3X, rbuf[18]);
		*/
		/* Linear Acceleration */
		/*
		input_report_abs(akm->input, ABS_HAT3Y, rbuf[19]);
		input_report_abs(akm->input, ABS_PRESSURE, rbuf[20]);
		input_report_abs(akm->input, ABS_DISTANCE, rbuf[21]);
		*/

		/* Geo Mag Rotation Vector */
		if (akm->enable_flag & GEO_MAG_DATA_READY) {
			input_report_abs(akm->input, ABS_TILT_X, rbuf[22]);
			input_report_abs(akm->input, ABS_TILT_Y, rbuf[23]);
			input_report_abs(akm->input, ABS_TOOL_WIDTH, rbuf[24]);
			input_report_abs(akm->input, ABS_VOLUME, rbuf[25]);
		}
	}

	input_sync(akm->input);
}

/* This function will block a process until the latest measurement
 * data is available.
 */
static int AKECS_GetData(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	int err;

	set_freezable();

	/* Block! */
	err = wait_event_freezable_timeout(
			akm->drdy_wq,
			atomic_read(&akm->drdy),
			msecs_to_jiffies(AKM_DRDY_TIMEOUT_MS));

	if (err < 0) {
		SENSOR_LOG_ERROR("wait_event failed (%d).", err);
		return err;
	}
	if (!atomic_read(&akm->drdy)) {
		SENSOR_LOG_ERROR("DRDY is not set.\n");
		return -ENODATA;
	}

	mutex_lock(&akm->sensor_mutex);

	memcpy(rbuf, akm->sense_data, size);
	atomic_set(&akm->drdy, 0);

	mutex_unlock(&akm->sensor_mutex);

	return 0;
}

static int AKECS_GetData_Poll(
	struct akm_compass_data *akm,
	uint8_t *rbuf,
	int size)
{
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	/* Read status */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, 1);
	if (err < 0) {
		SENSOR_LOG_ERROR("failed.\n");
		return err;
	}

	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		return -EAGAIN;

	/* Read rest data */
	buffer[1] = AKM_REG_STATUS + 1;
	err = akm_i2c_rxdata(akm->i2c, &(buffer[1]), AKM_SENSOR_DATA_SIZE-1);
	if (err < 0) {
		SENSOR_LOG_ERROR("failed.\n");
		return err;
	}

	memcpy(rbuf, buffer, size);
	atomic_set(&akm->drdy, 0);

	mutex_lock(&akm->sensor_mutex);
	akm->is_busy = 0;
	mutex_unlock(&akm->sensor_mutex);

	return 0;
}

static int AKECS_GetOpenStatus(
	struct akm_compass_data *akm)
{
	set_freezable();
	return wait_event_freezable(
			akm->open_wq, (atomic_read(&akm->active) > 0));
}

static int AKECS_GetCloseStatus(
	struct akm_compass_data *akm)
{
	set_freezable();
	return wait_event_freezable(
			akm->open_wq, (atomic_read(&akm->active) <= 0));
}

static int AKECS_Open(struct inode *inode, struct file *file)
{
	file->private_data = s_akm;
	return nonseekable_open(inode, file);
}

static int AKECS_Release(struct inode *inode, struct file *file)
{
	return 0;
}

static int akm_get_uncalib_data(struct akm_compass_data *akm, uint8_t *dat_buf, int *mag_data)
{
	int tmp = 0;

	tmp = (int)((int16_t)(dat_buf[2] << 8) + ((int16_t)dat_buf[1]));
	tmp = tmp * akm->sense_conf[0] / 128 + tmp;
	mag_data[0] = tmp;

	tmp = (int)((int16_t)(dat_buf[4] << 8) + ((int16_t)dat_buf[3]));
	tmp = tmp * akm->sense_conf[1] / 128 + tmp;
	mag_data[1] = tmp;

	tmp = (int)((int16_t)(dat_buf[6] << 8) + ((int16_t)dat_buf[5]));
	tmp = tmp * akm->sense_conf[2] / 128 + tmp;
	mag_data[2] = tmp;

	switch (akm->layout) {
	case 0:
	case 1:
		/* Fall into the default direction */
		break;
	case 2:
		tmp = mag_data[0];
		mag_data[0] = mag_data[1];
		mag_data[1] = -tmp;
		break;
	case 3:
		mag_data[0] = -mag_data[0];
		mag_data[1] = -mag_data[1];
		break;
	case 4:
		tmp = mag_data[0];
		mag_data[0] = -mag_data[1];
		mag_data[1] = tmp;
		break;
	case 5:
		mag_data[0] = -mag_data[0];
		mag_data[2] = -mag_data[2];
		break;
	case 6:
		tmp = mag_data[0];
		mag_data[0] = mag_data[1];
		mag_data[1] = tmp;
		mag_data[2] = -mag_data[2];
		break;
	case 7:
		mag_data[1] = -mag_data[1];
		mag_data[2] = -mag_data[2];
		break;
	case 8:
		tmp = mag_data[0];
		mag_data[0] = -mag_data[1];
		mag_data[1] = -tmp;
		mag_data[2] = -mag_data[2];
		break;
	}
	return 0;
}

static long AKECS_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct akm_compass_data *akm = file->private_data;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	uint8_t i2c_buf[AKM_RWBUF_SIZE];		/* for READ/WRITE */
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];/* for GET_DATA */
	int32_t ypr_buf[AKM_YPR_DATA_SIZE];		/* for SET_YPR */
	int64_t delay[AKM_NUM_SENSORS];	/* for GET_DELAY */
	int16_t acc_buf[3];	/* for GET_ACCEL */
	uint8_t mode;			/* for SET_MODE*/
	int status = 0;			/* for OPEN/CLOSE_STATUS */
	int ret = 0;		/* Return value. */

	switch (cmd) {
	case ECS_IOCTL_READ:
	case ECS_IOCTL_WRITE:
		if (argp == NULL) {
			SENSOR_LOG_ERROR("invalid argument.\n");
			return -EINVAL;
		}
		if (copy_from_user(&i2c_buf, argp, sizeof(i2c_buf))) {
			SENSOR_LOG_ERROR("copy_from_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL) {
			SENSOR_LOG_ERROR("invalid argument.\n");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			SENSOR_LOG_ERROR("copy_from_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL) {
			SENSOR_LOG_ERROR("invalid argument.\n");
			return -EINVAL;
		}
		if (copy_from_user(&ypr_buf, argp, sizeof(ypr_buf))) {
			SENSOR_LOG_ERROR("copy_from_user failed.\n");
			return -EFAULT;
		}
	case ECS_IOCTL_GET_INFO:
	case ECS_IOCTL_GET_CONF:
	case ECS_IOCTL_GET_DATA:
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
	case ECS_IOCTL_GET_DELAY:
	case ECS_IOCTL_GET_LAYOUT:
	case ECS_IOCTL_GET_ACCEL:
		/* Check buffer pointer for writing a data later. */
		if (argp == NULL) {
			SENSOR_LOG_ERROR("invalid argument.\n");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		if ((i2c_buf[0] < 1) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			SENSOR_LOG_ERROR("invalid argument.\n");
			return -EINVAL;
		}
		ret = akm_i2c_rxdata(akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_WRITE:
		if ((i2c_buf[0] < 2) || (i2c_buf[0] > (AKM_RWBUF_SIZE-1))) {
			SENSOR_LOG_ERROR("invalid argument.\n");
			return -EINVAL;
		}
		ret = akm_i2c_txdata(akm->i2c, &i2c_buf[1], i2c_buf[0]);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_RESET:
		ret = AKECS_Reset(akm, akm->gpio_rstn);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_MODE:
		ret = AKECS_SetMode(akm, mode);
		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_SET_YPR:
		AKECS_SetYPR(akm, ypr_buf);
		break;
	case ECS_IOCTL_GET_DATA:
		if (akm->irq)
			ret = AKECS_GetData(akm, dat_buf, AKM_SENSOR_DATA_SIZE);
		else
			ret = AKECS_GetData_Poll(
					akm, dat_buf, AKM_SENSOR_DATA_SIZE);

		akm_get_uncalib_data(akm, dat_buf, mag_data);

		if (ret < 0)
			return ret;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		ret = AKECS_GetOpenStatus(akm);
		if (ret < 0) {
			SENSOR_LOG_ERROR("Get Open returns error (%d).\n", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		ret = AKECS_GetCloseStatus(akm);
		if (ret < 0) {
			SENSOR_LOG_ERROR("Get Close returns error (%d).\n", ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		mutex_lock(&akm->val_mutex);
		delay[ACC_DATA_FLAG] = ((akm->enable_flag & ACC_DATA_READY) ?
				akm->delay[ACC_DATA_FLAG] : -1);
		delay[MAG_DATA_FLAG] = ((akm->enable_flag & MAG_DATA_READY) ?
				akm->delay[MAG_DATA_FLAG] : -1);
		delay[MAGUC_DATA_FLAG] = ((akm->enable_flag & MAGUC_DATA_READY) ?
				akm->delay[MAG_DATA_FLAG] : -1);
		delay[FUSION_DATA_FLAG] = ((akm->enable_flag & FUSION_DATA_READY) ?
				akm->delay[MAG_DATA_FLAG] : -1);
		delay[ORI_DATA_FLAG] = ((akm->enable_flag & ORI_DATA_READY) ?
				akm->delay[MAG_DATA_FLAG] : -1);
		delay[UNCALI_MAG_DATA_FLAG] = ((akm->enable_flag & UNCALI_MAG_DATA_READY) ?
				akm->delay[MAG_DATA_FLAG] : -1);
		delay[GEO_MAG_DATA_FLAG] = ((akm->enable_flag & GEO_MAG_DATA_READY) ?
				akm->delay[MAG_DATA_FLAG] : -1);
		mutex_unlock(&akm->val_mutex);
		break;
	case ECS_IOCTL_GET_INFO:
		break;
	case ECS_IOCTL_GET_CONF:
		break;
	case ECS_IOCTL_GET_LAYOUT:
		break;
	case ECS_IOCTL_GET_ACCEL:
		mutex_lock(&akm->accel_mutex);
		acc_buf[0] = akm->accel_data[0];
		acc_buf[1] = akm->accel_data[1];
		acc_buf[2] = akm->accel_data[2];
		mutex_unlock(&akm->accel_mutex);
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		/* +1  is for the first byte */
		if (copy_to_user(argp, &i2c_buf, i2c_buf[0]+1)) {
			SENSOR_LOG_ERROR("copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_INFO:
		if (copy_to_user(argp, &akm->sense_info,
					sizeof(akm->sense_info))) {
			SENSOR_LOG_ERROR("copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		if (copy_to_user(argp, &akm->sense_conf,
					sizeof(akm->sense_conf))) {
			SENSOR_LOG_ERROR("copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DATA:
		if (copy_to_user(argp, &dat_buf, sizeof(dat_buf))) {
			SENSOR_LOG_ERROR("copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		status = atomic_read(&akm->active);
		if (copy_to_user(argp, &status, sizeof(status))) {
			SENSOR_LOG_ERROR("copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			SENSOR_LOG_ERROR("copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_LAYOUT:
		if (copy_to_user(argp, &akm->layout, sizeof(akm->layout))) {
			SENSOR_LOG_ERROR("copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_ACCEL:
		if (copy_to_user(argp, &acc_buf, sizeof(acc_buf))) {
			SENSOR_LOG_ERROR("copy_to_user failed.\n");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static const struct file_operations AKECS_fops = {
	.owner = THIS_MODULE,
	.open = AKECS_Open,
	.release = AKECS_Release,
	.unlocked_ioctl = AKECS_ioctl,
};

static struct miscdevice akm_compass_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AKM_MISCDEV_NAME,
	.fops = &AKECS_fops,
};

/***** akm sysfs functions ******************************************/
static int create_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;
	int err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (err)
			break;
	}

	if (err) {
		for (--i; i >= 0 ; --i)
			device_remove_file(dev, &attrs[i]);
	}

	return err;
}

static void remove_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		device_remove_file(dev, &attrs[i]);
}

static int create_device_binary_attributes(
	struct kobject *kobj,
	struct bin_attribute *attrs)
{
	int i;
	int err = 0;

	err = 0;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i) {
		err = sysfs_create_bin_file(kobj, &attrs[i]);
		if (0 != err)
			break;
	}

	if (0 != err) {
		for (--i; i >= 0 ; --i)
			sysfs_remove_bin_file(kobj, &attrs[i]);
	}

	return err;
}

static void remove_device_binary_attributes(
	struct kobject *kobj,
	struct bin_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		sysfs_remove_bin_file(kobj, &attrs[i]);
}

/*********************************************************************
 *
 * SysFS attribute functions
 *
 * directory : /sys/class/compass/akmXXXX/
 * files :
 *  - enable_acc	[rw] [t] : enable flag for accelerometer
 *  - enable_mag	[rw] [t] : enable flag for magnetometer
 *  - enable_fusion [rw] [t] : enable flag for fusion sensor
 *  - delay_acc	 [rw] [t] : delay in nanosecond for accelerometer
 *  - delay_mag	 [rw] [t] : delay in nanosecond for magnetometer
 *  - delay_fusion  [rw] [t] : delay in nanosecond for fusion sensor
 *
 * debug :
 *  - mode	   [w]  [t] : E-Compass mode
 *  - bdata	  [r]  [t] : buffered raw data
 *  - asa	[r]  [t] : FUSEROM data
 *  - regs	   [r]  [t] : read all registers
 *
 * [b] = binary format
 * [t] = text format
 */

/***** sysfs enable *************************************************/
static void akm_compass_sysfs_update_status(
	struct akm_compass_data *akm)
{
	uint32_t en;
	mutex_lock(&akm->val_mutex);
	en = akm->enable_flag;
	mutex_unlock(&akm->val_mutex);

	if (en == 0) {
		if (atomic_cmpxchg(&akm->active, 1, 0) == 1) {
			wake_up(&akm->open_wq);
			SENSOR_LOG_DEBUG("Deactivated\n");
		}
	} else {
		if (atomic_cmpxchg(&akm->active, 0, 1) == 0) {
			wake_up(&akm->open_wq);
			SENSOR_LOG_DEBUG("Activated\n");
		}
	}
	SENSOR_LOG_DEBUG("Status updated: enable=0x%X, active=%d",
		en, atomic_read(&akm->active));
}

static int akm_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int ret = 0;
	struct akm_compass_data *akm = container_of(sensors_cdev,
			struct akm_compass_data, cdev);

	SENSOR_LOG_INFO("akm_enable_set enable = %d\n", enable);

	mutex_lock(&akm->val_mutex);
	akm->enable_flag &= ~(1<<MAG_DATA_FLAG);
	akm->enable_flag |= ((uint32_t)(enable))<<MAG_DATA_FLAG;
	mutex_unlock(&akm->val_mutex);

	akm_compass_sysfs_update_status(akm);
	mutex_lock(&akm->op_mutex);
	if (enable) {
#ifndef AKM_POWER_ALWAYS_ON
		ret = akm_compass_power_set(akm, true);
		if (ret) {
			SENSOR_LOG_ERROR("Fail to power on the device!\n");
			mutex_unlock(&akm->op_mutex);
			return ret;
		}
#endif

		if (akm->auto_report) {
			AKECS_SetMode(akm, AKM_MODE_SNG_MEASURE);
			schedule_delayed_work(&akm->dwork,
				(unsigned long)nsecs_to_jiffies64(
					akm->delay[MAG_DATA_FLAG]));
		}
	} else {
		if (akm->auto_report) {
			cancel_delayed_work_sync(&akm->dwork);
			AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
		}
#ifndef AKM_POWER_ALWAYS_ON
		ret = akm_compass_power_set(akm, false);
		if (ret)
			SENSOR_LOG_ERROR("Fail to power off the device!\n");
#endif
	}

	mutex_unlock(&akm->op_mutex);
	return ret;
}

static ssize_t akm_compass_sysfs_enable_show(
	struct akm_compass_data *akm, char *buf, int pos)
{
	int flag;

	mutex_lock(&akm->val_mutex);
	flag = ((akm->enable_flag >> pos) & 1);
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%d\n", flag);
}

static ssize_t akm_compass_sysfs_enable_store(
	struct akm_compass_data *akm, char const *buf, size_t count, int pos)
{
	long en = 0;
	int ret = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &en))
		return -EINVAL;

	en = en ? 1 : 0;

	SENSOR_LOG_INFO("akm_compass_sysfs_enable_store en = %ld, pos = %d\n", en, pos);

	mutex_lock(&akm->op_mutex);

#ifndef AKM_POWER_ALWAYS_ON
	ret = akm_compass_power_set(akm, en);
	if (ret) {
		SENSOR_LOG_ERROR("Fail to configure device power!\n");
		mutex_unlock(&akm->op_mutex);
		return ret;
	}
#endif

	mutex_lock(&akm->val_mutex);
	akm->enable_flag &= ~(1<<pos);
	akm->enable_flag |= ((uint32_t)(en))<<pos;
	mutex_unlock(&akm->val_mutex);

	akm_compass_sysfs_update_status(akm);

	mutex_unlock(&akm->op_mutex);
	return ret ? ret : count;
}

/***** IC check ***/
static ssize_t akm_mag_check_show(
	struct device *dev,	struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%s", AKM_I2C_NAME);
}

/***** Acceleration ***/
static ssize_t akm_enable_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}

static ssize_t akm_enable_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

/***** Magnetic field ***/
static ssize_t akm_enable_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}

static ssize_t akm_enable_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

/***** Fusion ***/
static ssize_t akm_enable_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}

static ssize_t akm_enable_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

/***** Orientation ***/
static ssize_t akm_enable_ori_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, ORI_DATA_FLAG);
}

static ssize_t akm_enable_ori_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, ORI_DATA_FLAG);
}

/***** Uncalibrated Mag ***/
static ssize_t akm_enable_unmag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, UNCALI_MAG_DATA_FLAG);
}

static ssize_t akm_enable_unmag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, UNCALI_MAG_DATA_FLAG);
}

/***** Geo Mag ***/
static ssize_t akm_enable_geomag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_enable_show(
		dev_get_drvdata(dev), buf, GEO_MAG_DATA_FLAG);
}

static ssize_t akm_enable_geomag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_enable_store(
		dev_get_drvdata(dev), buf, count, GEO_MAG_DATA_FLAG);
}

/***** sysfs delay **************************************************/
static int akm_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct akm_compass_data *akm = container_of(sensors_cdev,
			struct akm_compass_data, cdev);

	mutex_lock(&akm->val_mutex);
	akm->delay[MAG_DATA_FLAG] = delay_msec * 1000000;
	mutex_unlock(&akm->val_mutex);

	return 0;
}

static ssize_t akm_compass_sysfs_delay_show(
	struct akm_compass_data *akm, char *buf, int pos)
{
	int64_t val;

	mutex_lock(&akm->val_mutex);
	val = akm->delay[pos];
	mutex_unlock(&akm->val_mutex);

	return scnprintf(buf, PAGE_SIZE, "%lld\n", val);
}

static ssize_t akm_compass_sysfs_delay_store(
	struct akm_compass_data *akm, char const *buf, size_t count, int pos)
{
	long long val = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtoll(buf, AKM_BASE_NUM, &val))
		return -EINVAL;

	mutex_lock(&akm->val_mutex);
	akm->delay[pos] = val;
	mutex_unlock(&akm->val_mutex);

	return count;
}

/***** Accelerometer ***/
static ssize_t akm_delay_acc_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, ACC_DATA_FLAG);
}

static ssize_t akm_delay_acc_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, ACC_DATA_FLAG);
}

/***** Magnetic field ***/
static ssize_t akm_delay_mag_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, MAG_DATA_FLAG);
}

static ssize_t akm_delay_mag_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, MAG_DATA_FLAG);
}

/***** Fusion ***/
static ssize_t akm_delay_fusion_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return akm_compass_sysfs_delay_show(
		dev_get_drvdata(dev), buf, FUSION_DATA_FLAG);
}

static ssize_t akm_delay_fusion_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	return akm_compass_sysfs_delay_store(
		dev_get_drvdata(dev), buf, count, FUSION_DATA_FLAG);
}

/***** accel (binary) ***/
static ssize_t akm_bin_accel_write(
	struct file *file,
	struct kobject *kobj,
	struct bin_attribute *attr,
		char *buf,
		loff_t pos,
		size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int16_t *accel_data;

	if (size == 0)
		return 0;

	accel_data = (int16_t *)buf;

	mutex_lock(&akm->accel_mutex);
	akm->accel_data[0] = accel_data[0];
	akm->accel_data[1] = accel_data[1];
	akm->accel_data[2] = accel_data[2];
	mutex_unlock(&akm->accel_mutex);

	return size;
}

#if AKM_DEBUG_IF
static ssize_t akm_sysfs_mode_store(
	struct device *dev, struct device_attribute *attr,
	char const *buf, size_t count)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	long mode = 0;

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (strict_strtol(buf, AKM_BASE_NUM, &mode))
		return -EINVAL;

	if (AKECS_SetMode(akm, (uint8_t)mode) < 0)
		return -EINVAL;

	return 1;
}

static ssize_t akm_buf_print(
	char *buf, uint8_t *data, size_t num)
{
	int sz, i;
	char *cur;
	size_t cur_len;

	cur = buf;
	cur_len = PAGE_SIZE;
	sz = snprintf(cur, cur_len, "(HEX):");
	if (sz < 0)
		return sz;
	cur += sz;
	cur_len -= sz;
	for (i = 0; i < num; i++) {
		sz = snprintf(cur, cur_len, "%02X,", *data);
		if (sz < 0)
			return sz;
		cur += sz;
		cur_len -= sz;
		data++;
	}
	sz = snprintf(cur, cur_len, "\n");
	if (sz < 0)
		return sz;
	cur += sz;

	return (ssize_t)(cur - buf);
}

static ssize_t akm_sysfs_bdata_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	uint8_t rbuf[AKM_SENSOR_DATA_SIZE];

	mutex_lock(&akm->sensor_mutex);
	memcpy(&rbuf, akm->sense_data, sizeof(rbuf));
	mutex_unlock(&akm->sensor_mutex);

	return akm_buf_print(buf, rbuf, AKM_SENSOR_DATA_SIZE);
}

static ssize_t akm_sysfs_asa_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t asa[3];

	err = AKECS_SetMode(akm, AKM_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	asa[0] = AKM_FUSE_1ST_ADDR;
	err = akm_i2c_rxdata(akm->i2c, asa, 3);
	if (err < 0)
		return err;

	err = AKECS_SetMode(akm, AKM_MODE_POWERDOWN);
	if (err < 0)
		return err;

	return akm_buf_print(buf, asa, 3);
}

static ssize_t akm_sysfs_regs_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	/* The total number of registers depends on the device. */
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int err;
	uint8_t regs[AKM_REGS_SIZE];

	/* This function does not lock mutex obj */
	regs[0] = AKM_REGS_1ST_ADDR;
	err = akm_i2c_rxdata(akm->i2c, regs, AKM_REGS_SIZE);
	if (err < 0)
		return err;

	return akm_buf_print(buf, regs, AKM_REGS_SIZE);
}
#endif

static struct device_attribute akm_compass_attributes[] = {
	__ATTR(mag_check, 0444, akm_mag_check_show, NULL),
	__ATTR(enable_acc, 0660, akm_enable_acc_show, akm_enable_acc_store),
	__ATTR(enable_mag, 0660, akm_enable_mag_show, akm_enable_mag_store),
	__ATTR(enable_fusion, 0660, akm_enable_fusion_show, akm_enable_fusion_store),
	__ATTR(enable_ori, 0660, akm_enable_ori_show, akm_enable_ori_store),
	__ATTR(enable_unmag, 0660, akm_enable_unmag_show, akm_enable_unmag_store),
	__ATTR(enable_geomag, 0660, akm_enable_geomag_show, akm_enable_geomag_store),

	__ATTR(delay_acc,  0660, akm_delay_acc_show, akm_delay_acc_store),
	__ATTR(delay_mag,  0660, akm_delay_mag_show, akm_delay_mag_store),
	__ATTR(delay_ori, 0660, akm_delay_fusion_show, akm_delay_fusion_store),
#if AKM_DEBUG_IF
	__ATTR(mode,  0220, NULL, akm_sysfs_mode_store),
	__ATTR(bdata, 0440, akm_sysfs_bdata_show, NULL),
	__ATTR(asa,   0440, akm_sysfs_asa_show, NULL),
	__ATTR(regs,  0440, akm_sysfs_regs_show, NULL),
#endif
	__ATTR_NULL,
};

#define __BIN_ATTR(name_, mode_, size_, private_, read_, write_) \
	{ \
		.attr	= {.name = __stringify(name_), .mode = mode_ }, \
		.size	= size_, \
		.private = private_, \
		.read	= read_, \
		.write   = write_, \
	}

#define __BIN_ATTR_NULL \
	{ \
		.attr   = {.name = NULL }, \
	}

static struct bin_attribute akm_compass_bin_attributes[] = {
	__BIN_ATTR(accel, 0220, 6, NULL,
				NULL, akm_bin_accel_write),
	__BIN_ATTR_NULL
};

static int create_sysfs_interfaces(struct akm_compass_data *akm)
{
	int err = 0;

	if (NULL == akm)
		return -EINVAL;

	err = 0;

	akm->compass = class_create(THIS_MODULE, AKM_SYSCLS_NAME);
	if (IS_ERR(akm->compass)) {
		err = PTR_ERR(akm->compass);
		goto exit_class_create_failed;
	}

	akm->class_dev = device_create(
					akm->compass,
					NULL,
					akm_compass_device_dev_t,
					akm,
					AKM_SYSDEV_NAME);
	if (IS_ERR(akm->class_dev)) {
		err = PTR_ERR(akm->class_dev);
		goto exit_class_device_create_failed;
	}

	err = sysfs_create_link(
			&akm->class_dev->kobj,
			&akm->i2c->dev.kobj,
			device_link_name);
	if (0 > err)
		goto exit_sysfs_create_link_failed;

	err = create_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
	if (0 > err)
		goto exit_device_attributes_create_failed;

	err = create_device_binary_attributes(
			&akm->class_dev->kobj,
			akm_compass_bin_attributes);
	if (0 > err)
		goto exit_device_binary_attributes_create_failed;

	return err;

exit_device_binary_attributes_create_failed:
	remove_device_attributes(akm->class_dev, akm_compass_attributes);
exit_device_attributes_create_failed:
	sysfs_remove_link(&akm->class_dev->kobj, device_link_name);
exit_sysfs_create_link_failed:
	device_destroy(akm->compass, akm_compass_device_dev_t);
exit_class_device_create_failed:
	akm->class_dev = NULL;
	class_destroy(akm->compass);
exit_class_create_failed:
	akm->compass = NULL;
	return err;
}

static void remove_sysfs_interfaces(struct akm_compass_data *akm)
{
	if (NULL == akm)
		return;

	if (NULL != akm->class_dev) {
		remove_device_binary_attributes(
			&akm->class_dev->kobj,
			akm_compass_bin_attributes);
		remove_device_attributes(
			akm->class_dev,
			akm_compass_attributes);
		sysfs_remove_link(
			&akm->class_dev->kobj,
			device_link_name);
		akm->class_dev = NULL;
	}
	if (NULL != akm->compass) {
		device_destroy(
			akm->compass,
			akm_compass_device_dev_t);
		class_destroy(akm->compass);
		akm->compass = NULL;
	}
}


/***** akm input device functions ***********************************/
static int akm_compass_input_init(struct input_dev **input)
{
	int err = 0;

	/* Declare input device */
	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	/* Setup input device */
	set_bit(EV_ABS, (*input)->evbit);
	/* Accelerometer (720 x 16G)*/
	input_set_abs_params(*input, ABS_X,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_Y,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_Z,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_RX,
			0, 3, 0, 0);
	/* Magnetic field (limited to 16bit) */
	input_set_abs_params(*input, ABS_RY,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_RZ,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_THROTTLE,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_RUDDER,
			0, 3, 0, 0);
	/* Gyroscope (2000 deg/sec = 34.90 rad/sec in Q16 format) */
	/*
	input_set_abs_params(*input, ABS_WHEEL,
			-2287638, 2287638, 0, 0);
	input_set_abs_params(*input, ABS_GAS,
			-2287638, 2287638, 0, 0);
	input_set_abs_params(*input, ABS_BRAKE,
			-2287638, 2287638, 0, 0);
	input_set_abs_params(*input, ABS_HAT0X,
			0, 3, 0, 0);
	*/
	/* Orientation (degree in Q6 format) */
	/*  yaw[0,360) pitch[-180,180) roll[-90,90) */
	input_set_abs_params(*input, ABS_HAT0Y,
			0, 23040, 0, 0);
	input_set_abs_params(*input, ABS_HAT1X,
			-11520, 11520, 0, 0);
	input_set_abs_params(*input, ABS_HAT1Y,
			-5760, 5760, 0, 0);
	/* Gravity (2G = 19.61 m/s2 in Q16) */
	/*
	input_set_abs_params(*input, ABS_HAT2X,
			-1285377, 1285377, 0, 0);
	input_set_abs_params(*input, ABS_HAT2Y,
			-1285377, 1285377, 0, 0);
	input_set_abs_params(*input, ABS_HAT3X,
			-1285377, 1285377, 0, 0);
	*/
	/* Linear acceleration (16G = 156.9 m/s2 in Q16) */
	/*
	input_set_abs_params(*input, ABS_HAT3Y,
			-10283017, 10283017, 0, 0);
	input_set_abs_params(*input, ABS_PRESSURE,
			-10283017, 10283017, 0, 0);
	input_set_abs_params(*input, ABS_DISTANCE,
			-10283017, 10283017, 0, 0);
	*/
	/* Rotation Vector [-1,+1] in Q16 format */
	input_set_abs_params(*input, ABS_TILT_X,
			-65536, 65536, 0, 0);
	input_set_abs_params(*input, ABS_TILT_Y,
			-65536, 65536, 0, 0);
	input_set_abs_params(*input, ABS_TOOL_WIDTH,
			-65536, 65536, 0, 0);
	input_set_abs_params(*input, ABS_VOLUME,
			-65536, 65536, 0, 0);

	/*Uncalibrated  magnetic sensor*/
	input_set_abs_params(*input, ABS_MISC,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_MT_TOUCH_MAJOR,
			-32768, 32767, 0, 0);
	input_set_abs_params(*input, ABS_MT_TOUCH_MINOR,
					-32768, 32767, 0, 0);

	/* Set name */
	(*input)->name = AKM_INPUT_DEVICE_NAME;

	/* Register */
	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}

/***** akm functions ************************************************/
static irqreturn_t akm_compass_irq(int irq, void *handle)
{
	struct akm_compass_data *akm = handle;
	uint8_t buffer[AKM_SENSOR_DATA_SIZE];
	int err;

	memset(buffer, 0, sizeof(buffer));

	/***** lock *****/
	mutex_lock(&akm->sensor_mutex);

	/* Read whole data */
	buffer[0] = AKM_REG_STATUS;
	err = akm_i2c_rxdata(akm->i2c, buffer, AKM_SENSOR_DATA_SIZE);
	if (err < 0) {
		SENSOR_LOG_ERROR("IRQ I2C error.\n");
		akm->is_busy = 0;
		mutex_unlock(&akm->sensor_mutex);
		/***** unlock *****/

		return IRQ_HANDLED;
	}
	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		goto work_func_none;

	memcpy(akm->sense_data, buffer, AKM_SENSOR_DATA_SIZE);
	akm->is_busy = 0;

	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/

	atomic_set(&akm->drdy, 1);
	wake_up(&akm->drdy_wq);

	return IRQ_HANDLED;

work_func_none:
	mutex_unlock(&akm->sensor_mutex);
	/***** unlock *****/
	return IRQ_NONE;
}

static int akm_compass_suspend(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int ret = 0;

	if (AKM_IS_MAG_DATA_ENABLED() && akm->auto_report)
		cancel_delayed_work_sync(&akm->dwork);

	akm->state.power_on = akm->power_enabled;

#ifndef AKM_POWER_ALWAYS_ON
	if (akm->state.power_on)
		akm_compass_power_set(akm, false);
#endif

	if (s_akm->irq) {
		ret = pinctrl_select_state(akm->pinctrl, akm->pin_sleep);
		if (ret)
			SENSOR_LOG_ERROR("Can't select pinctrl state\n");
	}

	SENSOR_LOG_DEBUG("suspended\n");

	return ret;
}

static int akm_compass_resume(struct device *dev)
{
	struct akm_compass_data *akm = dev_get_drvdata(dev);
	int ret = 0;

	if (s_akm->irq) {
		ret = pinctrl_select_state(akm->pinctrl, akm->pin_default);
		if (ret)
			SENSOR_LOG_ERROR("Can't select pinctrl state\n");
	}

	if (akm->state.power_on) {
#ifndef AKM_POWER_ALWAYS_ON
		ret = akm_compass_power_set(akm, true);
		if (ret) {
			SENSOR_LOG_ERROR("Sensor power resume fail!\n");
			goto exit;
		}
#endif
		ret = AKECS_SetMode(akm, akm->state.mode);
		if (ret) {
			SENSOR_LOG_ERROR("Sensor state resume fail!\n");
			goto exit;
		}

	if (AKM_IS_MAG_DATA_ENABLED() && akm->auto_report)
		schedule_delayed_work(&akm->dwork,
			(unsigned long)nsecs_to_jiffies64(
			akm->delay[MAG_DATA_FLAG]));
	}

	SENSOR_LOG_DEBUG("resumed\n");

exit:
	return ret;
}

static int akm09911_i2c_check_device(
	struct i2c_client *client)
{
	/* AK09911 specific function */
	struct akm_compass_data *akm = i2c_get_clientdata(client);
	int err = 0;

	akm->sense_info[0] = AK09911_REG_WIA1;
	err = akm_i2c_rxdata(client, akm->sense_info, AKM_SENSOR_INFO_SIZE);
	if (err < 0)
		return err;

	/* Check read data */
	if ((akm->sense_info[0] != AK09911_WIA1_VALUE) ||
			(akm->sense_info[1] != AK09911_WIA2_VALUE)) {
		SENSOR_LOG_ERROR("The device is not AKM Compass.\n");
		return -ENXIO;
	}

	/* Set FUSE access mode */
	err = AKECS_SetMode(akm, AK09911_MODE_FUSE_ACCESS);
	if (err < 0)
		return err;

	akm->sense_conf[0] = AK09911_FUSE_ASAX;
	err = akm_i2c_rxdata(client, akm->sense_conf, AKM_SENSOR_CONF_SIZE);
	if (err < 0)
		return err;

	err = AKECS_SetMode(akm, AK09911_MODE_POWERDOWN);
	if (err < 0)
		return err;

	return err;
}

static int akm_compass_power_set(struct akm_compass_data *data, bool on)
{
	int rc = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->power_enabled = false;
		return rc;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->power_enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(80);
		return rc;
	} else {
		SENSOR_LOG_ERROR("Power on=%d. enabled=%d\n",
				on, data->power_enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		SENSOR_LOG_ERROR("Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int akm_compass_power_init(struct akm_compass_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				AKM09911_VDD_MAX_UV);
		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				AKM09911_VIO_MAX_UV);
		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				AKM09911_VDD_MIN_UV, AKM09911_VDD_MAX_UV);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->i2c->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			SENSOR_LOG_ERROR("Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				AKM09911_VIO_MIN_UV, AKM09911_VIO_MAX_UV);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}
	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, AKM09911_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int akm_compass_parse_dt(struct device *dev,
				struct akm_compass_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val = 0;
	int rc = 0;

	rc = of_property_read_u32(np, "akm,layout", &temp_val);
	if (rc && (rc != -EINVAL)) {
		SENSOR_LOG_ERROR("Unable to read akm,layout\n");
		return rc;
	}
	else {
		s_akm->layout = temp_val;
	}

	if (of_property_read_bool(np, "akm,auto-report")) {
		s_akm->auto_report = 1;
		SENSOR_LOG_INFO("s_akm->auto_report = 1\n");
	}
	else {
		s_akm->auto_report = 0;
		SENSOR_LOG_INFO("s_akm->auto_report = 0\n");
	}

	s_akm->gpio_rstn = of_get_named_gpio_flags(dev->of_node,
			"akm,gpio_rstn", 0, NULL);
	printk(KERN_ERR"gpio_rstn is %d\n", s_akm->gpio_rstn);

	if (!gpio_is_valid(s_akm->gpio_rstn)) {
		SENSOR_LOG_ERROR("gpio reset pin %d is invalid.\n", s_akm->gpio_rstn);
		return -EINVAL;
	}
	else {
		rc = gpio_request(s_akm->gpio_rstn, "akm09911_reset_pin");
		if (rc) {
			SENSOR_LOG_ERROR("reset gpio request failed\n");
			return -EINVAL;
		}

		rc = gpio_direction_output(s_akm->gpio_rstn, 0);
		if (rc) {
			SENSOR_LOG_ERROR("set_direction for reset gpio failed\n");
			return -EINVAL;
		}
		gpio_set_value_cansleep(s_akm->gpio_rstn, 1);
	}
	return 0;
}

static int akm_pinctrl_init(struct akm_compass_data *s_akm)
{
	struct i2c_client *client = s_akm->i2c;

	s_akm->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(s_akm->pinctrl)) {
		SENSOR_LOG_ERROR("Failed to get pinctrl\n");
		return PTR_ERR(s_akm->pinctrl);
	}

	s_akm->pin_default = pinctrl_lookup_state(s_akm->pinctrl, "ak09911_default");
	if (IS_ERR_OR_NULL(s_akm->pin_default)) {
		SENSOR_LOG_ERROR("Failed to look up default state\n");
		return PTR_ERR(s_akm->pin_default);
	}

	s_akm->pin_sleep = pinctrl_lookup_state(s_akm->pinctrl, "ak09911_sleep");
	if (IS_ERR_OR_NULL(s_akm->pin_sleep)) {
		SENSOR_LOG_ERROR("Failed to look up sleep state\n");
		return PTR_ERR(s_akm->pin_sleep);
	}

	return 0;
}

static int akm_report_data(struct akm_compass_data *akm)
{
	uint8_t dat_buf[AKM_SENSOR_DATA_SIZE];/* for GET_DATA */
	int ret;
	int mag_x, mag_y, mag_z;
	int tmp;
	int count = 10;

	do {
		/* The typical time for single measurement is 7.2ms */
		ret = AKECS_GetData_Poll(akm, dat_buf, AKM_SENSOR_DATA_SIZE);
		if (ret == -EAGAIN)
			usleep_range(1000, 10000);
	} while ((ret == -EAGAIN) && (--count));

	if (!count) {
		SENSOR_LOG_ERROR("Timeout get valid data.\n");
		return -EIO;
	}

	tmp = dat_buf[0] | dat_buf[7];
	if (STATUS_ERROR(tmp)) {
		SENSOR_LOG_ERROR("Status error(0x%x). Reset...\n", tmp);
		AKECS_Reset(akm, 0);
		return -EIO;
	}

	tmp = (int)((int16_t)(dat_buf[2] << 8) + ((int16_t)dat_buf[1]));
	tmp = tmp * akm->sense_conf[0] / 128 + tmp;
	mag_x = tmp;

	tmp = (int)((int16_t)(dat_buf[4] << 8) + ((int16_t)dat_buf[3]));
	tmp = tmp * akm->sense_conf[1] / 128 + tmp;
	mag_y = tmp;

	tmp = (int)((int16_t)(dat_buf[6] << 8) + ((int16_t)dat_buf[5]));
	tmp = tmp * akm->sense_conf[2] / 128 + tmp;
	mag_z = tmp;

/*
	SENSOR_LOG_DEBUG("mag_x:%d mag_y:%d mag_z:%d\n",
			mag_x, mag_y, mag_z);
	SENSOR_LOG_DEBUG("raw data: %d %d %d %d %d %d %d %d\n",
			dat_buf[0], dat_buf[1], dat_buf[2], dat_buf[3],
			dat_buf[4], dat_buf[5], dat_buf[6], dat_buf[7]);
	SENSOR_LOG_DEBUG("asa: %d %d %d\n", akm->sense_conf[0],
			akm->sense_conf[1], akm->sense_conf[2]);
*/
	switch (akm->layout) {
	case 0:
	case 1:
		/* Fall into the default direction */
		break;
	case 2:
		tmp = mag_x;
		mag_x = mag_y;
		mag_y = -tmp;
		break;
	case 3:
		mag_x = -mag_x;
		mag_y = -mag_y;
		break;
	case 4:
		tmp = mag_x;
		mag_x = -mag_y;
		mag_y = tmp;
		break;
	case 5:
		mag_x = -mag_x;
		mag_z = -mag_z;
		break;
	case 6:
		tmp = mag_x;
		mag_x = mag_y;
		mag_y = tmp;
		mag_z = -mag_z;
		break;
	case 7:
		mag_y = -mag_y;
		mag_z = -mag_z;
		break;
	case 8:
		tmp = mag_x;
		mag_x = -mag_y;
		mag_y = -tmp;
		mag_z = -mag_z;
		break;
	}

	input_report_abs(akm->input, ABS_X, mag_x);
	input_report_abs(akm->input, ABS_Y, mag_y);
	input_report_abs(akm->input, ABS_Z, mag_z);
	input_sync(akm->input);

	return 0;
}

static void akm_dev_poll(struct work_struct *work)
{
	struct akm_compass_data *akm;
	int ret;

	akm = container_of((struct delayed_work *)work,
			struct akm_compass_data,  dwork);

	ret = akm_report_data(akm);
	if (ret < 0)
		SENSOR_LOG_ERROR("Failed to report data\n");

	ret = AKECS_SetMode(akm, AKM_MODE_SNG_MEASURE);
	if ((ret < 0) && (ret != -EBUSY))
		SENSOR_LOG_ERROR("Failed to set mode\n");

	schedule_delayed_work(&akm->dwork,
		(unsigned long)nsecs_to_jiffies64(akm->delay[MAG_DATA_FLAG]));
}

static int akm_compass_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	int i = 0;

	SENSOR_LOG_INFO("start probing\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_LOG_ERROR("check_functionality failed\n");
		err = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	s_akm = kzalloc(sizeof(struct akm_compass_data), GFP_KERNEL);
	if (!s_akm) {
		SENSOR_LOG_ERROR("memory allocation failed\n");
		err = -ENOMEM;
		goto exit1;
	}

	/* initialize variables in akm_compass_data */
	init_waitqueue_head(&s_akm->drdy_wq);
	init_waitqueue_head(&s_akm->open_wq);

	mutex_init(&s_akm->sensor_mutex);
	mutex_init(&s_akm->accel_mutex);
	mutex_init(&s_akm->val_mutex);
	mutex_init(&s_akm->op_mutex);

	atomic_set(&s_akm->active, 0);
	atomic_set(&s_akm->drdy, 0);

	s_akm->is_busy = 0;
	s_akm->enable_flag = 0;

	/* Set to 1G in Android coordination, AKSC format */
	s_akm->accel_data[0] = 0;
	s_akm->accel_data[1] = 0;
	s_akm->accel_data[2] = 720;

	for (i = 0; i < AKM_NUM_SENSORS; i++)
		s_akm->delay[i] = -1;

	/* Set platform information */
	if (client->dev.of_node) {
		err = akm_compass_parse_dt(&client->dev, s_akm);
		if (err) {
			SENSOR_LOG_ERROR("Unable to parse platfrom data err = %d\n", err);
			goto exit2;
		}
	} else {
		SENSOR_LOG_ERROR("No platform data\n");
		err = -1;
		goto exit2;
	}

	/* I2C initialization */
	s_akm->i2c = client;
	/* set client data */
	i2c_set_clientdata(client, s_akm);

	/* initialize pinctrl */
	err = akm_pinctrl_init(s_akm);
	if (err) {
		SENSOR_LOG_ERROR("Pinctrl init failed.\n");
		goto exit2;
	}
	else {
		err = pinctrl_select_state(s_akm->pinctrl, s_akm->pin_sleep);
		if (err) {
			SENSOR_LOG_ERROR("Can't select pinctrl state\n");
			goto exit2;
		}
	}

	/* check connection */
	err = akm_compass_power_init(s_akm, 1);
	if (err < 0)
		goto exit2;
	err = akm_compass_power_set(s_akm, 1);
	if (err < 0)
		goto exit3;

	/* Pull up the reset pin */
	AKECS_Reset(s_akm, 1);

	err = akm09911_i2c_check_device(client);
	if (err < 0)
		goto exit4;

	/* input */
	err = akm_compass_input_init(&s_akm->input);
	if (err) {
		SENSOR_LOG_ERROR("input_dev register failed\n");
		goto exit4;
	}
	input_set_drvdata(s_akm->input, s_akm);

	/* IRQ setup */
	s_akm->irq = 0;

	SENSOR_LOG_INFO("IRQ is %d\n", s_akm->irq);

	if (s_akm->irq) {
		err = request_threaded_irq(
				s_akm->irq,
				NULL,
				akm_compass_irq,
				IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				dev_name(&client->dev),
				s_akm);
		if (err < 0) {
			SENSOR_LOG_ERROR("request irq failed.\n");
			goto exit5;
		}
	} else if (s_akm->auto_report) {
		INIT_DELAYED_WORK(&s_akm->dwork, akm_dev_poll);
	}

	/* misc */
	err = misc_register(&akm_compass_dev);
	if (err) {
		SENSOR_LOG_ERROR("akm_compass_dev register failed\n");
		goto exit6;
	}

	/* sysfs */
	err = create_sysfs_interfaces(s_akm);
	if (0 > err) {
		SENSOR_LOG_ERROR("create sysfs failed.\n");
		goto exit7;
	}

	s_akm->cdev = sensors_cdev;
	s_akm->cdev.sensors_enable = akm_enable_set;
	s_akm->cdev.sensors_poll_delay = akm_poll_delay_set;
	s_akm->delay[MAG_DATA_FLAG] = sensors_cdev.delay_msec * 1000000;

	err = sensors_classdev_register(&client->dev, &s_akm->cdev);
	if (err) {
		SENSOR_LOG_ERROR("class device create failed: %d\n", err);
		goto exit8;
	}

#ifndef AKM_POWER_ALWAYS_ON
	akm_compass_power_set(s_akm, false);
#endif
	SENSOR_LOG_INFO("successfully probed\n");
	return 0;

exit8:
	remove_sysfs_interfaces(s_akm);
exit7:
	misc_deregister(&akm_compass_dev);
exit6:
	if (s_akm->irq)
		free_irq(s_akm->irq, s_akm);
exit5:
	input_unregister_device(s_akm->input);
exit4:
	akm_compass_power_set(s_akm, 0);
exit3:
	akm_compass_power_init(s_akm, 0);
exit2:
	kfree(s_akm);
exit1:
exit0:
	return err;
}

static int akm_compass_remove(struct i2c_client *client)
{
	struct akm_compass_data *akm = i2c_get_clientdata(client);

	if (akm_compass_power_set(akm, 0))
		SENSOR_LOG_ERROR("power set failed.\n");
	if (akm_compass_power_init(akm, 0))
		SENSOR_LOG_ERROR("power deinit failed.\n");
	remove_sysfs_interfaces(akm);
	if (misc_deregister(&akm_compass_dev) < 0)
		SENSOR_LOG_ERROR("misc deregister failed.\n");
	if (akm->irq)
		free_irq(akm->irq, akm);
	input_unregister_device(akm->input);
	kfree(akm);
	SENSOR_LOG_INFO("successfully removed.");
	return 0;
}

static const struct i2c_device_id akm_compass_id[] = {
	{AKM_I2C_NAME, 0},
	{}
};

static const struct dev_pm_ops akm_compass_pm_ops = {
	.suspend	= akm_compass_suspend,
	.resume		= akm_compass_resume,
};

static struct of_device_id akm09911_match_table[] = {
	{.compatible = "ak,ak09911",},
	{.compatible = "akm,akm09911",},
	{},
};
static struct i2c_driver akm_compass_driver = {
	.probe		= akm_compass_probe,
	.remove		= akm_compass_remove,
	.id_table	= akm_compass_id,
	.driver = {
		.name	= AKM_I2C_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = akm09911_match_table,
		.pm		= &akm_compass_pm_ops,
	},
};

static int __init akm_compass_init(void)
{
	return i2c_add_driver(&akm_compass_driver);
}

static void __exit akm_compass_exit(void)
{
	i2c_del_driver(&akm_compass_driver);
}

module_init(akm_compass_init);
module_exit(akm_compass_exit);

MODULE_AUTHOR("viral wang <viral_wang@htc.com>");
MODULE_DESCRIPTION("AKM compass driver");
MODULE_LICENSE("GPL");

