/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * File Name		: lsm6dx0.c
 * Author		: MSH - C&I BU - Application Team
 *			: Giuseppe Barba (giuseppe.barba@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V.1.1.0
 * Date			: 2014/Apr/18
 * Description		: LSM6DX0 driver
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c/lsm6dx0.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#define LOG_TAG "SENSOR_ACC_GYRO"
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

/* POWER SUPPLY VOLTAGE RANGE */
#define LSM6DX0_VDD_MIN_UV	2000000
#define LSM6DX0_VDD_MAX_UV	3300000
#define LSM6DX0_VIO_MIN_UV	1750000
#define LSM6DX0_VIO_MAX_UV	1950000

#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
#define COMP_X_FACTOR_ADDR		(0x7F)
#define COMP_Y_FACTOR_ADDR		(0xF0)
#define COMP_Z_FACTOR_ADDR		(0xF1)
#define COMP_HW_DISABLE_MASK		(0xC0)
#endif

#ifdef CONFIG_SENSORS_LSM6DX0_LP
#define LSM6DX0_GYR_LPF1		(0)
#define LSM6DX0_GYR_LPF1_2		(1)
#define LSM6DX0_GYR_LP_TH_US		(LSM6DX0_ODR_US_100)

#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL
#define CONFIG_SENSORS_LSM6DX0_S_MODEL_LP
#endif
#endif

#define MS_TO_US(x)			(x * 1000L)
#define US_TO_NS(x)			(x * 1000L)
#define MS_TO_NS(x)			(US_TO_NS(MS_TO_US(x)))
#define NS_TO_US(x)			(x / 1000)
#define US_TO_MS(x)			(x / 1000)
#define NS_TO_MS(x)			(US_TO_MS(NS_TO_US(x)))

#define I2C_AUTO_INCREMENT		(0x80)
#define REFERENCE_G			(0x0B)

/* TODO: check the following values */
/* Sensitivity */
#define SENSITIVITY_ACC_2G		(61)	/** ug/LSB */
#define SENSITIVITY_ACC_4G		(122)	/** ug/LSB */
#define SENSITIVITY_ACC_8G		(244)	/** ug/LSB */
#define SENSITIVITY_GYR_250		(8750)	/** udps/LSB */
#define SENSITIVITY_GYR_500		(17500)	/** udps/LSB */
#define SENSITIVITY_GYR_2000		(70000)	/** udps/LSB */
#define SENSITIVITY_TEMP		(16)	/** LSB/C */
#define OFFSET_TEMP			(25)	/** Offset temperature */

#define ACC_G_MAX_POS			(1495040)/** max positive value acc [ug] */
#define ACC_G_MAX_NEG			(1495770)/** max negative value acc [ug] */
#define GYR_FS_MAX			(32768)

#define FUZZ				(0)
#define FLAT				(0)

#define FILTER_50			(50)/** Anti-Aliasing 50 Hz */
#define FILTER_100			(100)/** Anti-Aliasing 105 Hz */
#define FILTER_200			(200)/** Anti-Aliasing 211 Hz */
#define FILTER_400			(400)/** Anti-Aliasing 408 Hz */

#define RANGE_245DPS			(245)
#define RANGE_500DPS			(500)
#define RANGE_2000DPS			(2000)

#define ACT_THS				(0x04)
#define ACT_DUR				(0x05)
#define WHO_AM_I			(0x0F)
#define WHO_AM_I_VAL			(0x68)

/* Angular rate sensor Control Register 1 */
#define CTRL_REG1_G			(0x10)

#define BW_G_SHIFT			(0)
#define BW_G_MASK			(0x03)

#define FS_G_SHIFT			(3)
#define FS_G_MASK			(0x18)

/* Angular rate sensor Control Register 2 */
#define CTRL_REG2_G			(0x11)

#define OUT_SEL_SHIFT			(0)
#define OUT_SEL_MASK			(0x03)

#define INT_SEL_SHIFT			(2)
#define INT_SEL_MASK			(0x0C)

#define CTRL_REG3_G			(0x12)
#ifdef CONFIG_SENSORS_LSM6DX0_LP
#define CTRL_REG3_G_LP_MODE_MASK	(0x80)
#endif

/* Angular rate sensor sign and orientation register. */
#define ORIENT_CFG_G			(0x13)
#define ORIENT_CFG_G_SIGN_X_MASK	(0x20)
#define ORIENT_CFG_G_SIGN_Y_MASK	(0x10)
#define ORIENT_CFG_G_SIGN_Z_MASK	(0x08)
#define ORIENT_CFG_G_SIGN_ORIENT_MASK	(0x07)
#define OUT_TEMP_L			(0x15)
#define OUT_TEMP_H			(0x16)
#define STATUS_REG1			(0x17)
#define	OUT_X_L_G			(0x18) /* 1st AXIS OUT REG of 6 */
#define CTRL_REG4			(0x1E)
#define CTRL_REG4_DEF			(0x38)
#define CTRL_REG4_X_EN			(0x08)
#define CTRL_REG4_Y_EN			(0x10)
#define CTRL_REG4_Z_EN			(0x20)
#define CTRL_REG4_ALL_AXES_EN		(0x38)
#define CTRL_REG4_AXES_EN_MASK		(0x38)
#define CTRL_REG5_XL			(0x1F)
#define CTRL_REG5_XL_DEF		(0x38)
#define CTRL_REG6_XL			(0x20)
#define LSM6DX0_ACC_FS_DEF		(LSM6DX0_ACC_FS_2G)
#define BW_SCAL_ODR_SHIFT		(2)
#define BW_SCAL_ODR_MASK		(0x04)
#define BW_XL_50			(0x0C)
#define BW_XL_105			(0x08)
#define BW_XL_211			(0x04)
#define BW_XL_408			(0x00)
#define BW_XL_DEF			(BW_XL_408)
#define CTRL_REG7_XL			(0x21)
#define CTRL_REG8			(0x22)
#define CTRL_REG8_DEF			(0x44)
#define CTRL_REG9			(0x23)
#define CTRL_REG10			(0x24)
#define STATUS_REG2			(0x27)
#define OUT_X_L_XL			(0x28) /* 1st AXIS OUT REG of 6 */
#define FIFO_CTRL			(0x2E)
#define FIFO_SRC			(0x2F)
/* INT1_A/G pin control register. */
#define INT1_CTRL			(0x0C)
#define INT1_CTRL_IG_G_MASK		(0x80)
#define INT1_CTRL_IG_XL_MASK		(0x40)
#define INT1_CTRL_FSS5_MASK		(0x20)
#define INT1_CTRL_OVR_MASK		(0x10)
#define INT1_CTRL_FTH_MASK		(0x08)
#define INT1_CTRL_BOOT_MASK		(0x04)
#define INT1_CTRL_DRDY_G_MASK		(0x02)
#define INT1_CTRL_DRDY_XL_MASK		(0x01)

/* INT2_A/G pin control register. */
#define INT2_CTRL			(0x0D)
#define INT2_CTRL_INACT_MASK		(0x80)
#define INT2_CTRL_FSS5_MASK		(0x20)
#define INT2_CTRL_OVR_MASK		(0x10)
#define INT2_CTRL_FTH_MASK		(0x08)
#define INT2_CTRL_DRDY_TEMP_MASK	(0x04)
#define INT2_CTRL_DRDY_G_MASK		(0x02)
#define INT2_CTRL_DRDY_XL_MASK		(0x01)

/* Linear acceleration sensor interrupt source register. */
#define INT_GEN_SRC_XL			(0x26)
#define INT_GEN_SRC_XL_IA_MASK		(0x40)
#define INT_GEN_SRC_XL_ZH_MASK		(0x20)
#define INT_GEN_SRC_XL_ZL_MASK		(0x10)
#define INT_GEN_SRC_XL_YH_MASK		(0x08)
#define INT_GEN_SRC_XL_YL_MASK		(0x04)
#define INT_GEN_SRC_XL_XH_MASK		(0x02)
#define INT_GEN_SRC_XL_XL_MASK		(0x01)

/* Linear acceleration sensor interrupt generator configuration register. */
#define INT_GEN_CFG_XL			(0x06)
#define INT_GEN_CFG_XL_AOI_MASK	(0x80)
#define INT_GEN_CFG_XL_6D_MASK		(0x40)
#define INT_GEN_CFG_XL_ZHIE_MASK	(0x20)
#define INT_GEN_CFG_XL_ZLIE_MASK	(0x10)
#define INT_GEN_CFG_XL_YHIE_MASK	(0x08)
#define INT_GEN_CFG_XL_YLIE_MASK	(0x04)
#define INT_GEN_CFG_XL_XHIE_MASK	(0x02)
#define INT_GEN_CFG_XL_XLIE_MASK	(0x01)

/* Linear acceleration sensor interrupt threshold registers. */
#define INT_GEN_THS_X_XL		(0x07)
#define INT_GEN_THS_Y_XL		(0x08)
#define INT_GEN_THS_Z_XL		(0x09)

/* Linear acceleration sensor interrupt duration register. */
#define INT_GEN_DUR_XL			(0x0A)
#define INT_GEN_DUR_XL_WAIT_MASK	(0x80)
#define INT_GEN_DUR_XL_DUR_MASK	(0x7F)

/* Angular rate sensor interrupt source register. */
#define INT_GEN_SRC_G			(0x14)
#define INT_GEN_SRC_G_IA_MASK		(0x40)
#define INT_GEN_SRC_G_ZH_MASK		(0x20)
#define INT_GEN_SRC_G_ZL_MASK		(0x10)
#define INT_GEN_SRC_G_YH_MASK		(0x08)
#define INT_GEN_SRC_G_YL_MASK		(0x04)
#define INT_GEN_SRC_G_XH_MASK		(0x02)
#define INT_GEN_SRC_G_XL_MASK		(0x01)

/* Angular rate sensor interrupt generator configuration register. */
#define INT_GEN_CFG_G			(0x30)
#define INT_GEN_CFG_G_AOI_MASK		(0x80)
#define INT_GEN_CFG_G_LIR_MASK		(0x40)
#define INT_GEN_CFG_G_ZHIE_MASK	(0x20)
#define INT_GEN_CFG_G_ZLIE_MASK	(0x10)
#define INT_GEN_CFG_G_YHIE_MASK	(0x08)
#define INT_GEN_CFG_G_YLIE_MASK	(0x04)
#define INT_GEN_CFG_G_XHIE_MASK	(0x02)
#define INT_GEN_CFG_G_XLIE_MASK	(0x01)

/* Angular rate sensor interrupt generator threshold registers. */
#define INT_GEN_THS_XH_G		(0x31)
#define INT_GEN_THS_XL_G		(0x32)
#define INT_GEN_THS_YH_G		(0x33)
#define INT_GEN_THS_YL_G		(0x34)
#define INT_GEN_THS_ZH_G		(0x35)
#define INT_GEN_THS_ZL_G		(0x36)

/* Angular rate sensor interrupt generator duration register. */
#define INT_GEN_DUR_G			(0x37)
#define INT_GEN_DUR_G_WAIT_MASK	(0x80)
#define INT_GEN_DUR_G_DUR_MASK		(0x7F)

#define DEF_ZERO			(0x00)
#define UNDEF				(0x00)

#define LSM6DX0_ACC_MIN_POLL_PERIOD_MS  1
#define LSM6DX0_GYR_MIN_POLL_PERIOD_MS  2

#define LSM6DX0_PINCTRL_STATE_SLEEP	"lsm6ds0_sleep"
#define LSM6DX0_PINCTRL_STATE_DEFAULT	"lsm6ds0_default"

static char const *const device_link_name = "i2c";
static dev_t const lsm6dx0_acc_device_dev_t = MKDEV(MISC_MAJOR, 142);
static dev_t const lsm6dx0_gyr_device_dev_t = MKDEV(MISC_MAJOR, 143);

#define to_dev(obj) container_of(obj, struct device, kobj)

static const struct output_rate lsm6dx0_gyr_odr_table[] = {
	{LSM6DX0_ODR_US_110, (LSM6DX0_GYR_ODR_110 | (LSM6DX0_GYR_BW_11))},
	{LSM6DX0_ODR_US_101, (LSM6DX0_GYR_ODR_101 | (LSM6DX0_GYR_BW_11))},
	{LSM6DX0_ODR_US_100, (LSM6DX0_GYR_ODR_100 | (LSM6DX0_GYR_BW_11))},
	{LSM6DX0_ODR_US_011, (LSM6DX0_GYR_ODR_011 | (LSM6DX0_GYR_BW_11))},
	{LSM6DX0_ODR_US_010, (LSM6DX0_GYR_ODR_010 | (LSM6DX0_GYR_BW_11))},
	{LSM6DX0_ODR_US_001, (LSM6DX0_GYR_ODR_001)},
};

static const struct output_rate lsm6dx0_acc_odr_table[] = {
	{LSM6DX0_ODR_US_110, (LSM6DX0_GYR_ODR_110)},
	{LSM6DX0_ODR_US_101, (LSM6DX0_GYR_ODR_101)},
	{LSM6DX0_ODR_US_100, (LSM6DX0_GYR_ODR_100)},
	{LSM6DX0_ODR_US_011, (LSM6DX0_GYR_ODR_011)},
	{LSM6DX0_ODR_US_010, (LSM6DX0_GYR_ODR_010)},
	{LSM6DX0_ODR_US_001, (LSM6DX0_GYR_ODR_001)},
};

static const struct acc_turn_on_time lsm6dx0_acc_turn_on_time[] = {
	{LSM6DX0_ODR_US_110, {  {LSM6DX0_ACC_BW_400, 2},
				{LSM6DX0_ACC_BW_200, 4},
				{LSM6DX0_ACC_BW_100, 7},
				{LSM6DX0_ACC_BW_50,  14}}},
	{LSM6DX0_ODR_US_101, {  {LSM6DX0_ACC_BW_400, 1},
				{LSM6DX0_ACC_BW_200, 2},
				{LSM6DX0_ACC_BW_100, 4},
				{LSM6DX0_ACC_BW_50,  7}}},
	{LSM6DX0_ODR_US_100, {  {LSM6DX0_ACC_BW_400, 1},
				{LSM6DX0_ACC_BW_200, 1},
				{LSM6DX0_ACC_BW_100, 2},
				{LSM6DX0_ACC_BW_50,  4}}},
	{LSM6DX0_ODR_US_011, {  {LSM6DX0_ACC_BW_400, 1},
				{LSM6DX0_ACC_BW_200, 1},
				{LSM6DX0_ACC_BW_100, 1},
				{LSM6DX0_ACC_BW_50,  2}}},
	{LSM6DX0_ODR_US_010, {  {LSM6DX0_ACC_BW_400, 0},
				{LSM6DX0_ACC_BW_200, 0},
				{LSM6DX0_ACC_BW_100, 0},
				{LSM6DX0_ACC_BW_50,  0}}},
	{LSM6DX0_ODR_US_001, {  {LSM6DX0_ACC_BW_400, 0},
				{LSM6DX0_ACC_BW_200, 0},
				{LSM6DX0_ACC_BW_100, 0},
				{LSM6DX0_ACC_BW_50,  0}}},
};

#ifdef CONFIG_SENSORS_LSM6DX0_LP
static const struct gyr_turn_on_time lsm6dx0_gyr_turn_on_time[] = {
	{LSM6DX0_ODR_US_110, {  {LSM6DX0_GYR_LPF1,   8},
				{LSM6DX0_GYR_LPF1_2, 18}}},
	{LSM6DX0_ODR_US_101, {  {LSM6DX0_GYR_LPF1,   5},
				{LSM6DX0_GYR_LPF1_2, 15}}},
	{LSM6DX0_ODR_US_100, {  {LSM6DX0_GYR_LPF1,   4},
				{LSM6DX0_GYR_LPF1_2, 14}}},
	{LSM6DX0_ODR_US_011, {  {LSM6DX0_GYR_LPF1,   3},
				{LSM6DX0_GYR_LPF1_2, 13}}},
	{LSM6DX0_ODR_US_010, {  {LSM6DX0_GYR_LPF1,   3},
				{LSM6DX0_GYR_LPF1_2, 13}}},
	{LSM6DX0_ODR_US_001, {  {LSM6DX0_GYR_LPF1,   2},
				{LSM6DX0_GYR_LPF1_2, 0}}},
};
#endif

static const struct lsm6dx0_acc_platform_data default_lsm6dx0_acc_pdata = {
	.fs_range = LSM6DX0_ACC_FS_2G,
	.poll_interval = LSM6DX0_ACC_POLL_INTERVAL_DEF,
	.min_interval = LSM6DX0_ACC_MIN_POLL_PERIOD_US,
	.aa_filter_bandwidth = LSM6DX0_ACC_BW_400,
};

static const struct lsm6dx0_gyr_platform_data default_lsm6dx0_gyr_pdata = {
	.fs_range = LSM6DX0_GYR_FS_245DPS,
	.poll_interval = LSM6DX0_GYR_POLL_INTERVAL_DEF,
	.min_interval = LSM6DX0_GYR_MIN_POLL_PERIOD_US,
};

struct lsm6dx0_main_platform_data default_lsm6dx0_main_platform_data = {
	.rot_matrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	},
	.gpio_int1 = LSM6DX0_INT1_GPIO_DEF,
	.gpio_int2 = LSM6DX0_INT2_GPIO_DEF,
};

static struct status_registers status_registers = {
	.act_ths =
		{.address = ACT_THS,		.default_val = DEF_ZERO,},
	.act_dur =
		{.address = ACT_DUR,		.default_val = DEF_ZERO,},
	.int_gen_cfg_xl =
		{.address = INT_GEN_CFG_XL,	.default_val = DEF_ZERO,},
	.int_gen_ths_x_xl =
		{.address = INT_GEN_THS_X_XL,	.default_val = DEF_ZERO,},
	.int_gen_ths_y_xl =
		{.address = INT_GEN_THS_Y_XL,	.default_val = DEF_ZERO,},
	.int_gen_ths_z_xl =
		{.address = INT_GEN_THS_Z_XL,	.default_val = DEF_ZERO,},
	.int_gen_dur_xl =
		{.address = INT_GEN_DUR_XL,	.default_val = DEF_ZERO,},
	.reference_g =
		{.address = REFERENCE_G,	.default_val = DEF_ZERO,},
	.int1_ctrl =
		{.address = INT1_CTRL,		.default_val = DEF_ZERO,},
	.int2_ctrl =
		{.address = INT2_CTRL,		.default_val = DEF_ZERO,},
	.who_am_i =
		{.address = WHO_AM_I,		.default_val = WHO_AM_I_VAL,},
	.ctrl_reg1_g =
		{.address = CTRL_REG1_G,	.default_val = DEF_ZERO,},
	.ctrl_reg2_g =
		{.address = CTRL_REG2_G,	.default_val = DEF_ZERO,},
#ifdef CONFIG_SENSORS_LSM6DX0_L_MODEL
	/* LSM6L0 require the LP bit to be set to 0 */
	.ctrl_reg3_g =
		{.address = CTRL_REG3_G,
				.default_val = CTRL_REG3_G_LP_MODE_MASK,},
#else
	.ctrl_reg3_g =
		{.address = CTRL_REG3_G,	.default_val = DEF_ZERO,},
#endif
	.orient_cfg_g =
		{.address = ORIENT_CFG_G,	.default_val = DEF_ZERO,},
	.int_gen_src_g =
		{.address = INT_GEN_SRC_G,	.default_val = UNDEF,},
	.status_reg1 =
		{.address = STATUS_REG1,	.default_val = UNDEF,},
	.ctrl_reg4 =
		{.address = CTRL_REG4,		.default_val = CTRL_REG4_DEF,},
	.ctrl_reg5_xl =
		{.address = CTRL_REG5_XL,	.default_val = CTRL_REG5_XL_DEF,},
	.ctrl_reg6_xl =
		{.address = CTRL_REG6_XL,	.default_val = DEF_ZERO,},
	.ctrl_reg7_xl =
		{.address = CTRL_REG7_XL,	.default_val = DEF_ZERO,},
	.ctrl_reg8 =
		{.address = CTRL_REG8,		.default_val = CTRL_REG8_DEF,},
	.ctrl_reg9 =
		{.address = CTRL_REG9,		.default_val = DEF_ZERO,},
	.ctrl_reg10 =
		{.address = CTRL_REG10,		.default_val = DEF_ZERO,},
	.int_gen_src_xl =
		{.address = INT_GEN_SRC_XL,	.default_val = DEF_ZERO,},
	.status_reg2 =
		{.address = STATUS_REG2,	.default_val = UNDEF,},
	.fifo_ctrl =
		{.address = FIFO_CTRL,		.default_val = DEF_ZERO,},
	.fifo_src =
		{.address = FIFO_SRC,		.default_val = UNDEF,},
	.int_gen_cfg_g =
		{.address = INT_GEN_CFG_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_xh_g =
		{.address = INT_GEN_THS_XH_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_xl_g =
		{.address = INT_GEN_THS_XL_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_yh_g =
		{.address = INT_GEN_THS_YH_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_yl_g =
		{.address = INT_GEN_THS_YL_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_zh_g =
		{.address = INT_GEN_THS_ZH_G,	.default_val = DEF_ZERO,},
	.int_gen_ths_zl_g =
		{.address = INT_GEN_THS_ZL_G,	.default_val = DEF_ZERO,},
	.int_gen_dur_g =
		{.address = INT_GEN_DUR_G,	.default_val = DEF_ZERO,},
};
/*****************************************************************************/

static int32_t lsm6dx0_i2c_write(struct lsm6dx0_status *stat, uint8_t *buf,
								int32_t len)
{
	int32_t ret;
	uint8_t reg = buf[0];
	uint8_t value;
#ifdef DEBUG
	uint32_t ii;
#endif
	struct i2c_msg msg = {
		.addr = stat->client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = buf,
	};

	if (len > 1)
		reg |= I2C_AUTO_INCREMENT;

	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			SENSOR_LOG_INFO("i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
#ifdef DEBUG
			SENSOR_LOG_INFO("i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg);
			for (ii = 0; ii < (len + 1); ii++)
				SENSOR_LOG_INFO("value[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
			return ret;
		}
	}

	ret = i2c_transfer(stat->client->adapter, &msg, 1);

	return (ret == 1) ? 0 : 1;
}

static int32_t lsm6dx0_i2c_read(struct lsm6dx0_status *stat, uint8_t *buf,
								  int32_t len)
{
	int32_t ret;
	uint8_t cmd = buf[0];
#ifdef DEBUG
	uint32_t ii;
#endif
	struct i2c_msg msgs[] = {
		{
			.addr = stat->client->addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = stat->client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		}
	};

	if (len > 1)
		cmd |= I2C_AUTO_INCREMENT;

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			SENSOR_LOG_INFO("i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
#ifdef DEBUG
			SENSOR_LOG_INFO("i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			for (ii = 0; ii < len; ii++)
				SENSOR_LOG_INFO("buf[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			SENSOR_LOG_ERROR("read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0;
		}
		return len;
	}

	ret = i2c_transfer(stat->client->adapter, msgs, 2);

	return (ret == 2) ? 0 : 1;
}

static int32_t lsm6dx0_acc_device_power_off(struct lsm6dx0_status *stat)
{
	int32_t err = -1;
	uint8_t buf[2] = {0};

	buf[0] = status_registers.ctrl_reg6_xl.address;
	buf[1] = (LSM6DX0_ACC_ODR_MASK & LSM6DX0_ACC_ODR_OFF) |
	((~LSM6DX0_ACC_ODR_MASK) & status_registers.ctrl_reg6_xl.resume_val);

	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		SENSOR_LOG_ERROR("accelerometer soft power off "
							"failed: %d\n", err);

	if (stat->pdata_acc->power_off)
		stat->pdata_acc->power_off();

	atomic_set(&stat->enabled_acc, 0);
	SENSOR_LOG_INFO("accelerometer switched off.");

	return 0;
}

static int32_t lsm6dx0_gyr_device_power_off(struct lsm6dx0_status *stat)
{
	int32_t err = -1;
	uint8_t buf[2] = {0};

	buf[0] = status_registers.ctrl_reg1_g.address;
	buf[1] = (LSM6DX0_GYR_ODR_MASK & LSM6DX0_GYR_ODR_OFF) |
	((~LSM6DX0_GYR_ODR_MASK) & status_registers.ctrl_reg1_g.resume_val);

	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		SENSOR_LOG_ERROR("gyroscope soft power off failed: %d\n", err);

	if (stat->pdata_gyr->power_off)
		stat->pdata_gyr->power_off();

	atomic_set(&stat->enabled_gyr, 0);
	SENSOR_LOG_INFO("gyroscope switched off.");

	return 0;
}

static int32_t lsm6dx0_gyr_disable(struct lsm6dx0_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_gyr, 1, 0)) {
		SENSOR_LOG_INFO("disable gyro\n");
		cancel_work_sync(&stat->input_work_gyr);
		hrtimer_cancel(&stat->hr_timer_gyr);
		lsm6dx0_gyr_device_power_off(stat);
	}

	return 0;
}

static int32_t lsm6dx0_acc_disable(struct lsm6dx0_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_acc, 1, 0)) {
		SENSOR_LOG_INFO("disable acc\n");
		cancel_work_sync(&stat->input_work_acc);
		hrtimer_cancel(&stat->hr_timer_acc);
		lsm6dx0_acc_device_power_off(stat);
	}

	return 0;
}

static void lsm6dx0_acc_input_cleanup(struct lsm6dx0_status *stat)
{
	input_unregister_device(stat->input_dev_acc);
	input_free_device(stat->input_dev_acc);
}

static void lsm6dx0_gyr_input_cleanup(struct lsm6dx0_status *stat)
{
	input_unregister_device(stat->input_dev_gyr);
	input_free_device(stat->input_dev_gyr);
}

enum hrtimer_restart poll_function_read_acc(struct hrtimer *timer)
{
	struct lsm6dx0_status *stat;

	stat = container_of((struct hrtimer *)timer,
				struct lsm6dx0_status, hr_timer_acc);

	queue_work(stat->lsm6dx0_workqueue, &stat->input_work_acc);
	return HRTIMER_NORESTART;
}

enum hrtimer_restart poll_function_read_gyr(struct hrtimer *timer)
{
	struct lsm6dx0_status *stat;

	stat = container_of((struct hrtimer *)timer,
				struct lsm6dx0_status, hr_timer_gyr);

	queue_work(stat->lsm6dx0_workqueue, &stat->input_work_gyr);
	return HRTIMER_NORESTART;
}

static void lsm6dx0_validate_polling(uint32_t *min_interval,
					uint32_t *poll_interval,
					uint32_t min)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);
}

static int32_t lsm6dx0_acc_validate_pdata(struct lsm6dx0_status *stat)
{
	int32_t res = -EINVAL;

	lsm6dx0_validate_polling(&stat->pdata_acc->min_interval,
				 &stat->pdata_acc->poll_interval,
				(unsigned int)LSM6DX0_ACC_MIN_POLL_PERIOD_US);

	switch (stat->pdata_acc->aa_filter_bandwidth) {
	case LSM6DX0_ACC_BW_50:
		res = 1;
		break;
	case LSM6DX0_ACC_BW_100:
		res = 1;
		break;
	case LSM6DX0_ACC_BW_200:
		res = 1;
		break;
	case LSM6DX0_ACC_BW_400:
		res = 1;
		break;
	default:
		SENSOR_LOG_ERROR("invalid accelerometer bandwidth selected: %u\n",
			stat->pdata_acc->aa_filter_bandwidth);
	}

	return res;
}

static int32_t lsm6dx0_gyr_validate_pdata(struct lsm6dx0_status *stat)
{
	/* checks for correctness of minimal polling period */
	lsm6dx0_validate_polling(&stat->pdata_gyr->min_interval,
				 &stat->pdata_gyr->poll_interval,
				(unsigned int)LSM6DX0_GYR_MIN_POLL_PERIOD_US);

	/* Enforce minimum polling interval */
	if (stat->pdata_gyr->poll_interval < stat->pdata_gyr->min_interval) {
		SENSOR_LOG_ERROR("minimum poll interval violated\n");
		return -EINVAL;
	}
#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
	stat->compensation_temp = 0;
#endif
	return 0;
}

static int32_t lsm6dx0_acc_gyr_power_init(struct lsm6dx0_status *stat, bool on)
{
	int32_t rc = 0;

	SENSOR_LOG_INFO("on = %d\n", on);

	if (!on) {
		if (regulator_count_voltages(stat->vdd) > 0)
			regulator_set_voltage(stat->vdd, 0,
				LSM6DX0_VDD_MAX_UV);
		regulator_put(stat->vdd);

		if (regulator_count_voltages(stat->vio) > 0)
			regulator_set_voltage(stat->vio, 0,
				LSM6DX0_VIO_MAX_UV);
		regulator_put(stat->vio);
	} else {
		stat->vdd = regulator_get(&stat->client->dev, "vdd");
		if (IS_ERR(stat->vdd)) {
			rc = PTR_ERR(stat->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(stat->vdd) > 0) {
			rc = regulator_set_voltage(stat->vdd,
				LSM6DX0_VDD_MIN_UV, LSM6DX0_VDD_MAX_UV);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
					rc);
				goto err_vdd_set;
			}
		}

		stat->vio = regulator_get(&stat->client->dev, "vio");
		if (IS_ERR(stat->vio)) {
			rc = PTR_ERR(stat->vio);
			SENSOR_LOG_ERROR("Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}

		if (regulator_count_voltages(stat->vio) > 0) {
			rc = regulator_set_voltage(stat->vio,
				LSM6DX0_VIO_MIN_UV, LSM6DX0_VIO_MAX_UV);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set;
			}
		}

		rc = regulator_enable(stat->vdd);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(stat->vio);
		if (rc) {
			SENSOR_LOG_ERROR("Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
	}
	return 0;

err_vio_enable:
	regulator_disable(stat->vdd);
err_vdd_enable:
	if (regulator_count_voltages(stat->vio) > 0)
		regulator_set_voltage(stat->vio, 0, LSM6DX0_VIO_MAX_UV);
err_vio_set:
	regulator_put(stat->vio);
err_vio_get:
	if (regulator_count_voltages(stat->vdd) > 0)
		regulator_set_voltage(stat->vdd, 0, LSM6DX0_VDD_MAX_UV);
err_vdd_set:
	regulator_put(stat->vdd);
	return rc;
}

static int lsm6dx0_acc_gyr_pinctrl_init(struct lsm6dx0_status *stat)
{
	struct device *dev = &stat->client->dev;
	stat->lsm6dx0_pctrl.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(stat->lsm6dx0_pctrl.pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	stat->lsm6dx0_pctrl.gpio_state_active = pinctrl_lookup_state(
						stat->lsm6dx0_pctrl.pinctrl,
						LSM6DX0_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(stat->lsm6dx0_pctrl.gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	stat->lsm6dx0_pctrl.gpio_state_suspend = pinctrl_lookup_state(
						stat->lsm6dx0_pctrl.pinctrl,
						LSM6DX0_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(stat->lsm6dx0_pctrl.gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

static int32_t lsm6dx0_acc_gyr_hw_init(struct lsm6dx0_status *stat)
{
	int32_t err = -1;
	uint8_t buf = 0;

	SENSOR_LOG_INFO("%s: hw init start\n", LSM6DX0_ACC_GYR_DEV_NAME);

	buf = status_registers.who_am_i.address;
	err = lsm6dx0_i2c_read(stat, &buf, 1);
	if (err < 0) {
		SENSOR_LOG_ERROR("Error reading WHO_AM_I: is device"
			" available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf != status_registers.who_am_i.default_val) {
		SENSOR_LOG_ERROR("device unknown. Expected: 0x%02x, Replies: 0x%02x\n",
			status_registers.who_am_i.default_val, buf);
		err = -1;
		goto err_unknown_device;
	}

	status_registers.act_ths.resume_val =
				status_registers.act_ths.default_val;
	status_registers.act_dur.resume_val =
				status_registers.act_dur.default_val;
	status_registers.int_gen_cfg_xl.resume_val =
				status_registers.int_gen_cfg_xl.default_val;
	status_registers.int_gen_ths_x_xl.resume_val =
				status_registers.int_gen_ths_x_xl.default_val;
	status_registers.int_gen_ths_y_xl.resume_val =
				status_registers.int_gen_ths_y_xl.default_val;
	status_registers.int_gen_ths_z_xl.resume_val =
				status_registers.int_gen_ths_z_xl.default_val;
	status_registers.int_gen_dur_xl.resume_val =
				status_registers.int_gen_dur_xl.default_val;
	status_registers.reference_g.resume_val =
				status_registers.reference_g.default_val;
	status_registers.int1_ctrl.resume_val =
				status_registers.int1_ctrl.default_val;
	status_registers.int2_ctrl.resume_val =
				status_registers.int2_ctrl.default_val;
	status_registers.ctrl_reg1_g.resume_val =
				status_registers.ctrl_reg1_g.default_val;
	status_registers.ctrl_reg2_g.resume_val =
				status_registers.ctrl_reg2_g.default_val;
	status_registers.ctrl_reg3_g.resume_val =
				status_registers.ctrl_reg3_g.default_val;
	status_registers.orient_cfg_g.resume_val =
				status_registers.orient_cfg_g.default_val;
	status_registers.ctrl_reg4.resume_val =
				status_registers.ctrl_reg4.default_val;
	status_registers.ctrl_reg5_xl.resume_val =
				status_registers.ctrl_reg5_xl.default_val;
	status_registers.ctrl_reg6_xl.resume_val =
				status_registers.ctrl_reg6_xl.default_val;
	status_registers.ctrl_reg7_xl.resume_val =
				status_registers.ctrl_reg7_xl.default_val;
	status_registers.ctrl_reg8.resume_val =
				status_registers.ctrl_reg8.default_val;
	status_registers.ctrl_reg9.resume_val =
				status_registers.ctrl_reg9.default_val;
	status_registers.ctrl_reg10.resume_val =
				status_registers.ctrl_reg10.default_val;
	status_registers.fifo_ctrl.resume_val =
				status_registers.fifo_ctrl.default_val;
	status_registers.int_gen_cfg_g.resume_val =
				status_registers.int_gen_cfg_g.default_val;
	status_registers.int_gen_ths_xh_g.resume_val =
				status_registers.int_gen_ths_xh_g.default_val;
	status_registers.int_gen_ths_xl_g.resume_val =
				status_registers.int_gen_ths_xl_g.default_val;
	status_registers.int_gen_ths_yh_g.resume_val =
				status_registers.int_gen_ths_yh_g.default_val;
	status_registers.int_gen_ths_yl_g.resume_val =
				status_registers.int_gen_ths_yl_g.default_val;
	status_registers.int_gen_ths_zh_g.resume_val =
				status_registers.int_gen_ths_zh_g.default_val;
	status_registers.int_gen_ths_zl_g.resume_val =
				status_registers.int_gen_ths_zl_g.default_val;
	status_registers.int_gen_dur_g.resume_val =
				status_registers.int_gen_dur_g.default_val;

#ifdef CONFIG_SENSORS_LSM6DX0_LP
	atomic_set(&stat->low_power_state, 0);
#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL
	/** Disable Low Power mode at startup, device will be in normal mode */
	atomic_set(&stat->enabled_lp_mode, 0);
#else
	atomic_set(&stat->enabled_lp_mode, 1);
#endif
	stat->gyr_discard_samples = 0;
#endif
	stat->acc_discard_samples = 0;

	stat->hw_initialized = 1;
	SENSOR_LOG_INFO("%s: hw init done\n", LSM6DX0_ACC_GYR_DEV_NAME);

	return 0;

err_unknown_device:
err_firstread:
	stat->hw_working = 0;
	stat->hw_initialized = 0;
	return err;
}

static int32_t lsm6dx0_acc_device_power_on(struct lsm6dx0_status *stat)
{
	int32_t err = -1;
	uint8_t buf[9] = {0};

	if (stat->pdata_acc->power_on) {
		err = stat->pdata_acc->power_on();
		if (err < 0) {
			SENSOR_LOG_ERROR("accelerometer power_on failed: %d\n", err);
			return err;
		}
	}

	buf[0] = status_registers.ctrl_reg4.address;
	buf[1] = status_registers.ctrl_reg4.resume_val;
	buf[2] = status_registers.ctrl_reg5_xl.resume_val;
	buf[3] = status_registers.ctrl_reg6_xl.resume_val;
	buf[4] = status_registers.ctrl_reg7_xl.resume_val;
	buf[5] = status_registers.ctrl_reg8.resume_val;
	buf[6] = status_registers.ctrl_reg9.resume_val;
	buf[7] = status_registers.ctrl_reg10.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 7);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_gen_cfg_xl.address;
	buf[1] = status_registers.int_gen_cfg_xl.resume_val;
	buf[2] = status_registers.int_gen_ths_x_xl.resume_val;
	buf[3] = status_registers.int_gen_ths_y_xl.resume_val;
	buf[4] = status_registers.int_gen_ths_z_xl.resume_val;
	buf[5] = status_registers.int_gen_dur_xl.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 5);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int1_ctrl.address;
	buf[1] = status_registers.int1_ctrl.resume_val;
	buf[2] = status_registers.int2_ctrl.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.fifo_ctrl.address;
	buf[1] = status_registers.fifo_ctrl.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl_reg8.address;
	buf[1] = status_registers.ctrl_reg8.resume_val;
	buf[2] = status_registers.ctrl_reg9.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&stat->enabled_acc, 1);
	return 0;

err_resume_state:
	atomic_set(&stat->enabled_acc, 0);
	SENSOR_LOG_ERROR("accelerometer hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int32_t lsm6dx0_gyr_device_power_on(struct lsm6dx0_status *stat)
{
	int32_t err = -1;
	uint8_t buf[9] = {0};

	if (stat->pdata_gyr->power_on) {
		err = stat->pdata_gyr->power_on();
		if (err < 0) {
			SENSOR_LOG_ERROR("gyroscope power_on failed: %d\n", err);
			return err;
		}
	}

	buf[0] = status_registers.act_ths.address;
	buf[1] = status_registers.act_ths.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.reference_g.address;
	buf[1] = status_registers.reference_g.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl_reg1_g.address;
	buf[1] = status_registers.ctrl_reg1_g.resume_val;
	buf[2] = status_registers.ctrl_reg2_g.resume_val;
	buf[3] = status_registers.ctrl_reg3_g.resume_val;
	buf[4] = status_registers.orient_cfg_g.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 4);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.ctrl_reg4.address;
	buf[1] = status_registers.ctrl_reg4.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int_gen_cfg_g.address;
	buf[1] = status_registers.int_gen_cfg_g.resume_val;
	buf[2] = status_registers.int_gen_ths_xh_g.resume_val;
	buf[3] = status_registers.int_gen_ths_xl_g.resume_val;
	buf[4] = status_registers.int_gen_ths_yh_g.resume_val;
	buf[5] = status_registers.int_gen_ths_yl_g.resume_val;
	buf[6] = status_registers.int_gen_ths_zh_g.resume_val;
	buf[7] = status_registers.int_gen_ths_zl_g.resume_val;
	buf[8] = status_registers.int_gen_dur_g.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 8);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.int1_ctrl.address;
	buf[1] = status_registers.int1_ctrl.resume_val;
	buf[2] = status_registers.int2_ctrl.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.fifo_ctrl.address;
	buf[1] = status_registers.fifo_ctrl.resume_val;
	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&stat->enabled_gyr, 1);

	return 0;

err_resume_state:
	atomic_set(&stat->enabled_gyr, 0);
	SENSOR_LOG_ERROR("gyroscope hw power on error "
				"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int32_t lsm6dx0_acc_update_fs_range(struct lsm6dx0_status *stat,
							uint8_t new_fs_range)
{
	int32_t sensitivity, err = -1;
	uint8_t val, buf[2] = {0};

	switch (new_fs_range) {
	case LSM6DX0_ACC_FS_2G:
		sensitivity = SENSITIVITY_ACC_2G;
		break;
	case LSM6DX0_ACC_FS_4G:
		sensitivity = SENSITIVITY_ACC_4G;
		break;
	case LSM6DX0_ACC_FS_8G:
		sensitivity = SENSITIVITY_ACC_8G;
		break;
	default:
		SENSOR_LOG_ERROR("invalid accelerometer "
				"fs range requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	val = ((LSM6DX0_ACC_FS_MASK & new_fs_range) | ((~LSM6DX0_ACC_FS_MASK) &
				status_registers.ctrl_reg6_xl.resume_val));

	buf[0] = status_registers.ctrl_reg6_xl.address;
	buf[1] = val;

	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg6_xl.resume_val = val;

	mutex_lock(&stat->lock);
	stat->sensitivity_acc = sensitivity;
	mutex_unlock(&stat->lock);

	return err;

error:
	SENSOR_LOG_ERROR("update accelerometer fs range failed "
		"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}

static int32_t lsm6dx0_gyr_update_fs_range(struct lsm6dx0_status *stat,
							uint8_t new_fs_range)
{
	int32_t err = -1;
	uint8_t updated_val, buf[2] = {0};
	u32 sensitivity;
#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
	int8_t k_fs;
#endif

	switch (new_fs_range) {
	case LSM6DX0_GYR_FS_245DPS:
		sensitivity = SENSITIVITY_GYR_250;
#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
		k_fs = 8;
#endif
		break;
	case LSM6DX0_GYR_FS_500DPS:
		sensitivity = SENSITIVITY_GYR_500;
#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
		k_fs = 4;
#endif
		break;
	case LSM6DX0_GYR_FS_2000DPS:
		sensitivity = SENSITIVITY_GYR_2000;
#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
		k_fs = 1;
#endif
		break;
	default:
		SENSOR_LOG_ERROR("invalid g range requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	buf[0] = status_registers.ctrl_reg1_g.address;
	err = lsm6dx0_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;

	updated_val = ((LSM6DX0_GYR_FS_MASK & new_fs_range) |
				((~LSM6DX0_GYR_FS_MASK) & buf[0]));

	buf[0] = status_registers.ctrl_reg1_g.address;
	buf[1] = updated_val;

	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg1_g.resume_val = updated_val;

	mutex_lock(&stat->lock);
	stat->sensitivity_gyr = sensitivity;
#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
	stat->k_fs = k_fs;
#endif
	mutex_unlock(&stat->lock);

error:
	return err;
}

static int32_t lsm6dx0_acc_update_odr(struct lsm6dx0_status *stat,
					uint32_t poll_interval_us)
{
	int32_t err = -1;
	uint8_t buf[2] = {0};
	uint32_t i;

	for (i = ARRAY_SIZE(lsm6dx0_acc_odr_table) - 1; i >= 0; i--) {
		if (((uint32_t)lsm6dx0_acc_odr_table[i].cutoff_us <=
			poll_interval_us) || (i == 0))
			break;
	}

	if (atomic_read(&stat->enabled_acc)) {
		buf[0] = status_registers.ctrl_reg6_xl.address;
		buf[1] = LSM6DX0_ACC_ODR_MASK & lsm6dx0_acc_odr_table[i].value;
		buf[1] |= (~LSM6DX0_ACC_ODR_MASK) &
				status_registers.ctrl_reg6_xl.resume_val;

		err = lsm6dx0_i2c_write(stat, buf, 1);
		if (err < 0)
			goto error;

		status_registers.ctrl_reg6_xl.resume_val = buf[1];

		mutex_lock(&stat->lock);
		stat->ktime_acc = ktime_set(0, US_TO_NS(poll_interval_us));
		stat->pdata_acc->poll_interval = poll_interval_us;
		mutex_unlock(&stat->lock);
	}

	return err;

error:
	SENSOR_LOG_ERROR("update accelerometer odr failed "
			"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);

	return err;
}

static int32_t lsm6dx0_gyr_update_odr(struct lsm6dx0_status *stat,
					uint32_t poll_interval_us)
{
	uint8_t buf[2] = {0};
	uint32_t val, i;
	int32_t err = -1;
#ifdef CONFIG_SENSORS_LSM6DX0_LP
	uint8_t lp_mode;
#endif

	if (atomic_read(&stat->enabled_gyr)) {
		if (atomic_read(&stat->enabled_acc)) {
			val = min(poll_interval_us,
					stat->pdata_acc->poll_interval);
		} else {
			val = poll_interval_us;
		}

		for (i = ARRAY_SIZE(lsm6dx0_gyr_odr_table) - 1; i >= 0; i--) {
			if ((lsm6dx0_gyr_odr_table[i].cutoff_us <= val) ||
				(i == 0))
				break;
		}

		/* Set ODR value */
		buf[0] = status_registers.ctrl_reg1_g.address;
		buf[1] = LSM6DX0_GYR_ODR_MASK & lsm6dx0_gyr_odr_table[i].value;
		buf[1] |= (~LSM6DX0_GYR_ODR_MASK) &
					status_registers.ctrl_reg1_g.resume_val;

		err = lsm6dx0_i2c_write(stat, buf, 1);
		if (err < 0)
			goto error;

		status_registers.ctrl_reg1_g.resume_val = buf[1];

#ifdef CONFIG_SENSORS_LSM6DX0_LP
		if (lsm6dx0_gyr_odr_table[i].cutoff_us < LSM6DX0_GYR_LP_TH_US)
			lp_mode = 1;
		else
			lp_mode = 0;

		/* Discard samples only if switch between normal and low-power
		 * mode
		 */
		if (lp_mode != (uint8_t)atomic_read(&stat->low_power_state) &&
				atomic_read(&stat->enabled_lp_mode) == 1) {
			atomic_set(&stat->low_power_state, lp_mode);

			mutex_lock(&stat->lock);
			stat->gyr_discard_samples =
			lsm6dx0_gyr_turn_on_time[i].disc_list[LSM6DX0_GYR_LPF1].count;
		} else {
			mutex_lock(&stat->lock);
			stat->gyr_discard_samples = 0;
		}
		mutex_unlock(&stat->lock);
#endif
		/* Enable all axes */
		buf[0] = status_registers.ctrl_reg4.address;
		buf[1] = CTRL_REG4_ALL_AXES_EN |
				status_registers.ctrl_reg4.resume_val;

		err = lsm6dx0_i2c_write(stat, buf, 1);
		if (err < 0)
			goto error;

		status_registers.ctrl_reg4.resume_val = buf[1];

		mutex_lock(&stat->lock);
		stat->ktime_gyr = ktime_set(0, US_TO_NS(val));
		stat->pdata_gyr->poll_interval = val;
		mutex_unlock(&stat->lock);
	}
	return err;
error:
	SENSOR_LOG_ERROR("update accelerometer odr failed "
			"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);

	return err;
}

static int32_t lsm6dx0_acc_update_filter(struct lsm6dx0_status *stat,
							uint8_t new_bandwidth)
{
	int32_t err = -1;
	uint8_t updated_val, buf[2] = {0};

	switch (new_bandwidth) {
	case LSM6DX0_ACC_BW_50:
		break;
	case LSM6DX0_ACC_BW_100:
		break;
	case LSM6DX0_ACC_BW_200:
		break;
	case LSM6DX0_ACC_BW_400:
		break;
	default:
		SENSOR_LOG_ERROR("invalid accelerometer "
			"update bandwidth requested: %u\n", new_bandwidth);
		return -EINVAL;
	}

	buf[0] = status_registers.ctrl_reg6_xl.address;
	err = lsm6dx0_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg6_xl.resume_val = buf[0];

	updated_val = ((LSM6DX0_ACC_BW_MASK & new_bandwidth) |
					((~LSM6DX0_ACC_BW_MASK) & buf[0]));

	buf[0] = status_registers.ctrl_reg6_xl.address;
	buf[1] = updated_val;

	err = lsm6dx0_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;

	status_registers.ctrl_reg6_xl.resume_val = updated_val;

	return err;
error:
	SENSOR_LOG_ERROR("update accelerometer fs range failed "
		"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}


#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL_LP
static int32_t lsm6dx0_acc_change_pm_state(struct lsm6dx0_status *stat,
							uint8_t new_pm_state)
{
	uint8_t val, buf[2] = {0};
	int32_t err = -1, new_state = (new_pm_state != 0) ? 1 : 0;

	if (atomic_read(&stat->enabled_lp_mode) != new_state) {
		val = ((CTRL_REG3_G_LP_MODE_MASK & new_state) |
			((~CTRL_REG3_G_LP_MODE_MASK) &
			status_registers.ctrl_reg3_g.resume_val));

		buf[0] = status_registers.ctrl_reg3_g.address;
		buf[1] = val;

		err = lsm6dx0_i2c_write(stat, buf, 1);
		if (err < 0)
			goto error;

		status_registers.ctrl_reg3_g.resume_val = val;
		atomic_set(&stat->enabled_lp_mode, new_state);
	}
	return err;
error:
	SENSOR_LOG_ERROR("enable pm bit failed "
		"0x%02x,0x%02x: %d\n", buf[0], buf[1], err);
	return err;
}
#endif

static int32_t lsm6dx0_acc_enable(struct lsm6dx0_status *stat)
{
	int32_t err = -1;
	uint8_t j, bw;

	if (!atomic_cmpxchg(&stat->enabled_acc, 0, 1)) {
		SENSOR_LOG_INFO("enable acc\n");
		err = lsm6dx0_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_acc, 0);
			SENSOR_LOG_ERROR("enable accelerometer failed\n");
			return err;
		}

		for (j = ARRAY_SIZE(lsm6dx0_acc_turn_on_time) - 1; j >= 0; j--) {
			if ((lsm6dx0_acc_turn_on_time[j].odr_us <=
			  stat->pdata_acc->poll_interval) || (j == 0))
				break;
		}
		bw = stat->pdata_acc->aa_filter_bandwidth;

		stat->acc_discard_samples =
			lsm6dx0_acc_turn_on_time[j].disc_list[bw].count;

		hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc,
							HRTIMER_MODE_REL);
	}
	return 0;
}

static int32_t lsm6dx0_gyr_enable(struct lsm6dx0_status *stat)
{
	int32_t err = -1;

	if (!atomic_cmpxchg(&stat->enabled_gyr, 0, 1)) {
		SENSOR_LOG_INFO("enable gyro\n");
		err = lsm6dx0_gyr_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_gyr, 0);
			return err;
		}

		hrtimer_start(&(stat->hr_timer_gyr), stat->ktime_gyr,
							HRTIMER_MODE_REL);
	}
	return 0;
}

static int32_t lsm6dx0_acc_input_open(struct input_dev *input)
{
	/*
	struct lsm6dx0_status *stat = input_get_drvdata(input);
	return lsm6dx0_acc_enable(stat);
	*/
	return 0;
}

static void lsm6dx0_acc_input_close(struct input_dev *dev)
{
	/*
	struct lsm6dx0_status *stat = input_get_drvdata(dev);
	lsm6dx0_acc_disable(stat);
	*/
}

static int32_t lsm6dx0_gyr_input_open(struct input_dev *input)
{
	/*
	struct lsm6dx0_status *stat = input_get_drvdata(input);
	return lsm6dx0_gyr_enable(stat);
	*/
	return 0;
}

static void lsm6dx0_gyr_input_close(struct input_dev *dev)
{
	/*
	struct lsm6dx0_status *stat = input_get_drvdata(dev);
	lsm6dx0_gyr_disable(stat);
	*/
}

static int32_t lsm6dx0_temp_get_data(struct lsm6dx0_status *stat, int32_t *data)
{
	int32_t err = -1;
	uint8_t temp_data[2] = {0};

	temp_data[0] = OUT_TEMP_L;
	err = lsm6dx0_i2c_read(stat, temp_data, 2);
	if (err < 0)
		return err;

	(*data) = (int32_t)((int16_t)((temp_data[1] << 8) | (temp_data[0])));

	return err;
}

static int32_t lsm6dx0_acc_get_data(struct lsm6dx0_status *stat, int32_t *xyz)
{
	int32_t i, err = -1, hw_d[3] = {0};
	uint8_t acc_data[6];

	acc_data[0] = OUT_X_L_XL;
	err = lsm6dx0_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((int32_t)((int16_t)((acc_data[1] << 8) | (acc_data[0]))));
	hw_d[1] = ((int32_t)((int16_t)((acc_data[3] << 8) | (acc_data[2]))));
	hw_d[2] = ((int32_t)((int16_t)((acc_data[5] << 8) | (acc_data[4]))));

	mutex_lock(&stat->lock);
	hw_d[0] = hw_d[0] * stat->sensitivity_acc;
	hw_d[1] = hw_d[1] * stat->sensitivity_acc;
	hw_d[2] = hw_d[2] * stat->sensitivity_acc;

	for (i = 0; i < 3; i++) {
		xyz[i] = stat->pdata_main->rot_matrix[0][i] * hw_d[0] +
				stat->pdata_main->rot_matrix[1][i] * hw_d[1] +
				stat->pdata_main->rot_matrix[2][i] * hw_d[2];
	}
	mutex_unlock(&stat->lock);

	return err;
}

static int32_t lsm6dx0_gyr_get_data(struct lsm6dx0_status *stat, int32_t *xyz)
{
	int32_t i, err = 1, hw_d[3] = {0};
	uint8_t gyro_data[6];
#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
	int32_t temp, abs_temp;
#endif

	gyro_data[0] = OUT_X_L_G;
	err = lsm6dx0_i2c_read(stat, gyro_data, 6);

	if (err < 0)
		return err;

	hw_d[0] = (int32_t) ((int16_t)((gyro_data[1]) << 8) | gyro_data[0]);
	hw_d[1] = (int32_t) ((int16_t)((gyro_data[3]) << 8) | gyro_data[2]);
	hw_d[2] = (int32_t) ((int16_t)((gyro_data[5]) << 8) | gyro_data[4]);

	mutex_lock(&stat->lock);
#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
	err = lsm6dx0_temp_get_data(stat, &temp);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}

	if (temp >= stat->compensation_temp)
		abs_temp = temp - stat->compensation_temp;
	else
		abs_temp = stat->compensation_temp - temp;

	if (abs_temp > SENSITIVITY_TEMP)
		stat->compensation_temp = temp;

	hw_d[0] -= 10 * stat->compensation_temp *
			((int32_t)(stat->k_fs * stat->k_coeff[1])) / 896;
	hw_d[1] += 10 * stat->compensation_temp *
			((int32_t)(stat->k_fs * stat->k_coeff[0])) / 896;
	hw_d[2] -= 10 * stat->compensation_temp *
			((int32_t)(stat->k_fs * stat->k_coeff[2])) / 896;
#endif

	hw_d[0] = hw_d[0] * stat->sensitivity_gyr;
	hw_d[1] = hw_d[1] * stat->sensitivity_gyr;
	hw_d[2] = hw_d[2] * stat->sensitivity_gyr;

	for (i = 0; i < 3; i++) {
		xyz[i] = stat->pdata_main->rot_matrix[0][i] * hw_d[0] +
				stat->pdata_main->rot_matrix[1][i] * hw_d[1] +
				stat->pdata_main->rot_matrix[2][i] * hw_d[2];
	}
	mutex_unlock(&stat->lock);
	return err;
}

static void lsm6dx0_acc_report_values(struct lsm6dx0_status *stat,
								int32_t *xyz)
{
	input_report_abs(stat->input_dev_acc, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev_acc, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev_acc, ABS_Z, xyz[2]);
	input_sync(stat->input_dev_acc);
}

static void lsm6dx0_gyr_report_values(struct lsm6dx0_status *stat,
								int32_t *xyz)
{
	input_report_abs(stat->input_dev_gyr, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev_gyr, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev_gyr, ABS_Z, xyz[2]);
	input_sync(stat->input_dev_gyr);
}

static int32_t lsm6dx0_acc_input_init(struct lsm6dx0_status *stat)
{
	int32_t err = -1;

	mutex_lock(&stat->lock);
	stat->input_dev_acc = input_allocate_device();
	if (!stat->input_dev_acc) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("accelerometer input device allocation failed\n");
		mutex_unlock(&stat->lock);
		return err;
	}

	stat->input_dev_acc->open = lsm6dx0_acc_input_open;
	stat->input_dev_acc->close = lsm6dx0_acc_input_close;
	stat->input_dev_acc->name = LSM6DX0_ACC_DEV_NAME;
	stat->input_dev_acc->id.bustype = BUS_I2C;
	stat->input_dev_acc->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev_acc, stat);

	set_bit(EV_ABS, stat->input_dev_acc->evbit);

	input_set_abs_params(stat->input_dev_acc, ABS_X, -ACC_G_MAX_NEG,
						ACC_G_MAX_POS, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev_acc, ABS_Y, -ACC_G_MAX_NEG,
						ACC_G_MAX_POS, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev_acc, ABS_Z, -ACC_G_MAX_NEG,
						ACC_G_MAX_POS, FUZZ, FLAT);

	err = input_register_device(stat->input_dev_acc);
	if (err) {
		SENSOR_LOG_ERROR("unable to register accelerometer input device %s\n",
				stat->input_dev_acc->name);
		input_free_device(stat->input_dev_acc);
	}
	mutex_unlock(&stat->lock);

	return err;
}

static int32_t lsm6dx0_gyr_input_init(struct lsm6dx0_status *stat)
{
	int32_t err = -1;

	mutex_lock(&stat->lock);
	stat->input_dev_gyr = input_allocate_device();
	if (!stat->input_dev_gyr) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("input device allocation failed\n");
		mutex_unlock(&stat->lock);
		return err;
	}

	stat->input_dev_gyr->open = lsm6dx0_gyr_input_open;
	stat->input_dev_gyr->close = lsm6dx0_gyr_input_close;
	stat->input_dev_gyr->name = LSM6DX0_GYR_DEV_NAME;
	stat->input_dev_gyr->id.bustype = BUS_I2C;
	stat->input_dev_gyr->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev_gyr, stat);

	set_bit(EV_ABS, stat->input_dev_gyr->evbit);

	input_set_abs_params(stat->input_dev_gyr, ABS_X, -GYR_FS_MAX - 1,
							GYR_FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev_gyr, ABS_Y, -GYR_FS_MAX - 1,
							GYR_FS_MAX, 0, 0);
	input_set_abs_params(stat->input_dev_gyr, ABS_Z, -GYR_FS_MAX - 1,
							GYR_FS_MAX, 0, 0);

	err = input_register_device(stat->input_dev_gyr);
	if (err) {
		SENSOR_LOG_ERROR("unable to register input device %s\n",
			stat->input_dev_gyr->name);
		input_free_device(stat->input_dev_gyr);
	}
	mutex_unlock(&stat->lock);

	return err;
}
static void lsm6dx0_input_cleanup(struct lsm6dx0_status *stat)
{
	input_unregister_device(stat->input_dev_acc);
	input_free_device(stat->input_dev_acc);

	input_unregister_device(stat->input_dev_gyr);
	input_free_device(stat->input_dev_gyr);
}

static ssize_t attr_set_polling_rate_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long interval_us, interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_us = (unsigned long)max((unsigned int)MS_TO_US(interval_ms),
						stat->pdata_acc->min_interval);

	lsm6dx0_acc_update_odr(stat, interval_us);

	return size;
}

static ssize_t attr_get_polling_rate_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	uint32_t val = 0;
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%u\n", US_TO_MS(val));
}

static ssize_t attr_get_enable_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);

	int32_t val = (int)atomic_read(&stat->enabled_acc);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6dx0_acc_enable(stat);
	else
		lsm6dx0_acc_disable(stat);

	return size;
}

static ssize_t attr_get_range_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	uint8_t val;
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t range = 2;

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->fs_range;
	mutex_unlock(&stat->lock);

	switch (val) {
	case LSM6DX0_ACC_FS_2G:
		range = 2;
		break;
	case LSM6DX0_ACC_FS_4G:
		range = 4;
		break;
	case LSM6DX0_ACC_FS_8G:
		range = 8;
		break;
	default:
		SENSOR_LOG_ERROR("invalid param\n");
		break;
	}
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_acc(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t range;
	int32_t err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 2:
		range = LSM6DX0_ACC_FS_2G;
		break;
	case 4:
		range = LSM6DX0_ACC_FS_4G;
		break;
	case 8:
		range = LSM6DX0_ACC_FS_8G;
		break;
	default:
		SENSOR_LOG_ERROR("accelerometer invalid range "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	err = lsm6dx0_acc_update_fs_range(stat, range);
	if (err < 0)
		return err;

	mutex_lock(&stat->lock);
	stat->pdata_acc->fs_range = range;
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_get_aa_filter(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	uint8_t val;
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t frequency = FILTER_400;

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->aa_filter_bandwidth;
	mutex_unlock(&stat->lock);

	switch (val) {
	case LSM6DX0_ACC_BW_50:
		frequency = FILTER_50;
		break;
	case LSM6DX0_ACC_BW_100:
		frequency = FILTER_100;
		break;
	case LSM6DX0_ACC_BW_200:
		frequency = FILTER_200;
		break;
	case LSM6DX0_ACC_BW_400:
		frequency = FILTER_400;
		break;
	default:
		SENSOR_LOG_ERROR("invalid param\n");
		break;
	}
	return sprintf(buf, "%d\n", frequency);
}

static ssize_t attr_set_aa_filter(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t frequency;
	int32_t err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case FILTER_50:
		frequency = LSM6DX0_ACC_BW_50;
		break;
	case FILTER_100:
		frequency = LSM6DX0_ACC_BW_100;
		break;
	case FILTER_200:
		frequency = LSM6DX0_ACC_BW_200;
		break;
	case FILTER_400:
		frequency = LSM6DX0_ACC_BW_400;
		break;
	default:
		SENSOR_LOG_ERROR("accelerometer invalid filter "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	err = lsm6dx0_acc_update_filter(stat, frequency);
	if (err < 0)
		return err;

	mutex_lock(&stat->lock);
	stat->pdata_acc->aa_filter_bandwidth = frequency;
	mutex_unlock(&stat->lock);

	SENSOR_LOG_INFO("accelerometer anti-aliasing filter set to: %lu Hz\n", val);
	return size;
}

static ssize_t attr_get_temp(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t temp_decimal, temp_hw = 0, err = -1;
	uint32_t temp_float;

	err = lsm6dx0_temp_get_data(stat, &temp_hw);
	if (err < 0)
		return sprintf(buf, "null\n");

	temp_decimal = (int32_t)(temp_hw / SENSITIVITY_TEMP) + OFFSET_TEMP;
	temp_float = (((uint32_t)temp_hw) % SENSITIVITY_TEMP);

	return sprintf(buf, "%d.%d\n", temp_decimal, temp_float);
}

static ssize_t attr_get_polling_rate_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	uint32_t val;
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_gyr->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", US_TO_MS(val));
}

static ssize_t attr_set_polling_rate_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long interval_us, interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_us = (unsigned int)max((unsigned int)MS_TO_US(interval_ms),
					stat->pdata_gyr->min_interval);

	lsm6dx0_gyr_update_odr(stat, interval_us);

	return size;
}

static ssize_t attr_get_enable_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t val = atomic_read(&stat->enabled_gyr);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6dx0_gyr_enable(stat);
	else
		lsm6dx0_gyr_disable(stat);

	return size;
}

static ssize_t attr_get_range_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t range = 0;
	uint8_t val;

	mutex_lock(&stat->lock);
	val = stat->pdata_gyr->fs_range;
	switch (val) {
	case LSM6DX0_GYR_FS_245DPS:
		range = RANGE_245DPS;
		break;
	case LSM6DX0_GYR_FS_500DPS:
		range = RANGE_500DPS;
		break;
	case LSM6DX0_GYR_FS_2000DPS:
		range = RANGE_2000DPS;
		break;
	default:
		SENSOR_LOG_ERROR("invalid param\n");
		break;
	}
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_gyr(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t range;
	int32_t err = -1;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 245:
		range = LSM6DX0_GYR_FS_245DPS;
		break;
	case 500:
		range = LSM6DX0_GYR_FS_500DPS;
		break;
	case 2000:
		range = LSM6DX0_GYR_FS_2000DPS;
		break;
	default:
		SENSOR_LOG_ERROR("invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}

	err = lsm6dx0_gyr_update_fs_range(stat, range);
	if (err >= 0) {
		mutex_lock(&stat->lock);
		stat->pdata_gyr->fs_range = range;
		mutex_unlock(&stat->lock);
	}

	SENSOR_LOG_INFO("range set to: %lu dps\n", val);
	return size;
}

#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL_LP
static ssize_t attr_get_pmode(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	uint8_t val;

	val = atomic_read(&stat->enabled_lp_mode);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_pmode(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	struct device *dev = to_dev(kobj->parent);
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	lsm6dx0_acc_change_pm_state(stat, val);

	return size;
}
#endif

static struct kobj_attribute poll_attr_acc =
	__ATTR(pollrate_ms, 0644, attr_get_polling_rate_acc,
						attr_set_polling_rate_acc);
static struct kobj_attribute enable_attr_acc =
	__ATTR(enable_device, 0644, attr_get_enable_acc, attr_set_enable_acc);
static struct kobj_attribute fs_attr_acc =
	__ATTR(range, 0644, attr_get_range_acc, attr_set_range_acc);
static struct kobj_attribute aa_filter_attr  =
	__ATTR(anti_aliasing_frequency, 0644, attr_get_aa_filter,
							attr_set_aa_filter);
static struct kobj_attribute temp_attr  =
	__ATTR(temperature, 0644, attr_get_temp, NULL);
static struct kobj_attribute poll_attr_gyr =
	__ATTR(pollrate_ms, 0644, attr_get_polling_rate_gyr,
						attr_set_polling_rate_gyr);
static struct kobj_attribute enable_attr_gyr =
	__ATTR(enable_device, 0644, attr_get_enable_gyr, attr_set_enable_gyr);
static struct kobj_attribute range_attr_gyr =
	__ATTR(range, 0644, attr_get_range_gyr, attr_set_range_gyr);
#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL_LP
static struct kobj_attribute pmode_attr =
	__ATTR(pmode, 0644, attr_get_pmode, attr_set_pmode);
#endif

static struct attribute *attributes_acc[] = {
	&poll_attr_acc.attr,
	&enable_attr_acc.attr,
	&fs_attr_acc.attr,
	&aa_filter_attr.attr,
	&temp_attr.attr,
#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL_LP
	&pmode_attr.attr,
#endif
	NULL,
};

static struct attribute *attributes_gyr[] = {
	&poll_attr_gyr.attr,
	&enable_attr_gyr.attr,
	&range_attr_gyr.attr,
	&temp_attr.attr,
#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL_LP
	&pmode_attr.attr,
#endif
	NULL,
};

static struct attribute_group attr_group_acc = {
	.attrs = attributes_acc,
};

static struct attribute_group attr_group_gyr = {
	.attrs = attributes_gyr,
};

static int32_t create_sysfs_interfaces(struct lsm6dx0_status *stat)
{
	int32_t err = -1;
	struct device *dev = &stat->client->dev;

	stat->acc_kobj = kobject_create_and_add("accelerometer", &dev->kobj);
	if (!stat->acc_kobj) {
		err = -ENOMEM;
		goto failed;
	}

	stat->gyr_kobj = kobject_create_and_add("gyroscope", &dev->kobj);
	if (!stat->gyr_kobj) {
		err = -ENOMEM;
		goto gyr_kobj_create_failed;
	}

	err = sysfs_create_group(stat->acc_kobj, &attr_group_acc);
	if (err)
		goto acc_sysfs_create_failed;

	err = sysfs_create_group(stat->gyr_kobj, &attr_group_gyr);
	if (err)
		goto gyr_sysfs_create_failed;

	return 0;

gyr_sysfs_create_failed:
	sysfs_remove_group(stat->acc_kobj, &attr_group_acc);
acc_sysfs_create_failed:
	kobject_put(stat->gyr_kobj);
gyr_kobj_create_failed:
	kobject_put(stat->acc_kobj);
failed:
	return err;
}

static void remove_sysfs_interfaces(struct lsm6dx0_status *stat)
{
	sysfs_remove_group(stat->acc_kobj, &attr_group_acc);
	sysfs_remove_group(stat->gyr_kobj, &attr_group_gyr);
	kobject_put(stat->acc_kobj);
	kobject_put(stat->gyr_kobj);
}

static ssize_t attr_check_acc_show(struct device *dev,
	struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%s", LSM6DX0_ACC_GYR_DEV_NAME);
}

static ssize_t attr_polling_rate_acc_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	uint32_t val = 0;
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%u\n", US_TO_MS(val));
}

static ssize_t attr_polling_rate_acc_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long interval_us, interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_us = (unsigned long)max((unsigned int)MS_TO_US(interval_ms),
						stat->pdata_acc->min_interval);

	lsm6dx0_acc_update_odr(stat, interval_us);

	return size;
}

static ssize_t attr_enable_acc_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);

	int32_t val = (int)atomic_read(&stat->enabled_acc);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_acc_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6dx0_acc_enable(stat);
	else
		lsm6dx0_acc_disable(stat);

	return size;
}

static ssize_t attr_range_acc_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	uint8_t val;
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t range = 2;

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->fs_range;
	mutex_unlock(&stat->lock);

	switch (val) {
	case LSM6DX0_ACC_FS_2G:
		range = 2;
		break;
	case LSM6DX0_ACC_FS_4G:
		range = 4;
		break;
	case LSM6DX0_ACC_FS_8G:
		range = 8;
		break;
	default:
		SENSOR_LOG_ERROR("invalid param\n");
		break;
	}
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_acc_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t range;
	int32_t err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 2:
		range = LSM6DX0_ACC_FS_2G;
		break;
	case 4:
		range = LSM6DX0_ACC_FS_4G;
		break;
	case 8:
		range = LSM6DX0_ACC_FS_8G;
		break;
	default:
		SENSOR_LOG_ERROR("accelerometer invalid range "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	err = lsm6dx0_acc_update_fs_range(stat, range);
	if (err < 0)
		return err;

	mutex_lock(&stat->lock);
	stat->pdata_acc->fs_range = range;
	mutex_unlock(&stat->lock);

	return size;
}

static ssize_t attr_aa_filter_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	uint8_t val;
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t frequency = FILTER_400;

	mutex_lock(&stat->lock);
	val = stat->pdata_acc->aa_filter_bandwidth;
	mutex_unlock(&stat->lock);

	switch (val) {
	case LSM6DX0_ACC_BW_50:
		frequency = FILTER_50;
		break;
	case LSM6DX0_ACC_BW_100:
		frequency = FILTER_100;
		break;
	case LSM6DX0_ACC_BW_200:
		frequency = FILTER_200;
		break;
	case LSM6DX0_ACC_BW_400:
		frequency = FILTER_400;
		break;
	default:
		SENSOR_LOG_ERROR("invalid param\n");
		break;
	}
	return sprintf(buf, "%d\n", frequency);
}

static ssize_t attr_aa_filter_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t frequency;
	int32_t err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case FILTER_50:
		frequency = LSM6DX0_ACC_BW_50;
		break;
	case FILTER_100:
		frequency = LSM6DX0_ACC_BW_100;
		break;
	case FILTER_200:
		frequency = LSM6DX0_ACC_BW_200;
		break;
	case FILTER_400:
		frequency = LSM6DX0_ACC_BW_400;
		break;
	default:
		SENSOR_LOG_ERROR("accelerometer invalid filter "
					"request: %lu, discarded\n", val);
		return -EINVAL;
	}

	err = lsm6dx0_acc_update_filter(stat, frequency);
	if (err < 0)
		return err;

	mutex_lock(&stat->lock);
	stat->pdata_acc->aa_filter_bandwidth = frequency;
	mutex_unlock(&stat->lock);

	SENSOR_LOG_INFO("accelerometer anti-aliasing filter set to: %lu Hz\n", val);
	return size;
}

static ssize_t attr_temp_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t temp_decimal, temp_hw = 0, err = -1;
	uint32_t temp_float;

	err = lsm6dx0_temp_get_data(stat, &temp_hw);
	if (err < 0)
		return sprintf(buf, "null\n");

	temp_decimal = (int32_t)(temp_hw / SENSITIVITY_TEMP) + OFFSET_TEMP;
	temp_float = (((uint32_t)temp_hw) % SENSITIVITY_TEMP);

	return sprintf(buf, "%d.%d\n", temp_decimal, temp_float);
}

static ssize_t attr_null_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return size;
}

#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL_LP
static ssize_t attr_pmode_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	uint8_t val;

	val = atomic_read(&stat->enabled_lp_mode);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_pmode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	lsm6dx0_acc_change_pm_state(stat, val);

	return size;
}
#endif

static ssize_t attr_check_gyro_show(struct device *dev,
	struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%s", LSM6DX0_ACC_GYR_DEV_NAME);
}

static ssize_t attr_polling_rate_gyr_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	uint32_t val;
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_gyr->poll_interval;
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", US_TO_MS(val));
}

static ssize_t attr_polling_rate_gyr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long interval_us, interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

	interval_us = (unsigned int)max((unsigned int)MS_TO_US(interval_ms),
					stat->pdata_gyr->min_interval);

	lsm6dx0_gyr_update_odr(stat, interval_us);

	return size;
}

static ssize_t attr_enable_gyr_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t val = atomic_read(&stat->enabled_gyr);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_gyr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm6dx0_gyr_enable(stat);
	else
		lsm6dx0_gyr_disable(stat);

	return size;
}

static ssize_t attr_range_gyr_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	int32_t range = 0;
	uint8_t val;

	mutex_lock(&stat->lock);
	val = stat->pdata_gyr->fs_range;
	switch (val) {
	case LSM6DX0_GYR_FS_245DPS:
		range = RANGE_245DPS;
		break;
	case LSM6DX0_GYR_FS_500DPS:
		range = RANGE_500DPS;
		break;
	case LSM6DX0_GYR_FS_2000DPS:
		range = RANGE_2000DPS;
		break;
	default:
		SENSOR_LOG_ERROR("invalid param\n");
		break;
	}
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_gyr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	uint8_t range;
	int32_t err = -1;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	switch (val) {
	case 245:
		range = LSM6DX0_GYR_FS_245DPS;
		break;
	case 500:
		range = LSM6DX0_GYR_FS_500DPS;
		break;
	case 2000:
		range = LSM6DX0_GYR_FS_2000DPS;
		break;
	default:
		SENSOR_LOG_ERROR("invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}

	err = lsm6dx0_gyr_update_fs_range(stat, range);
	if (err >= 0) {
		mutex_lock(&stat->lock);
		stat->pdata_gyr->fs_range = range;
		mutex_unlock(&stat->lock);
	}

	SENSOR_LOG_INFO("range set to: %lu dps\n", val);
	return size;
}


static struct device_attribute lsm6dx0_acc_attributes[] = {
	__ATTR(acc_check, 0444, attr_check_acc_show, NULL),
	__ATTR(pollrate_ms, 0644, attr_polling_rate_acc_show,
			attr_polling_rate_acc_store),
	__ATTR(enable_device, 0644, attr_enable_acc_show,
			attr_enable_acc_store),
	__ATTR(range, 0644, attr_range_acc_show, attr_range_acc_store),
	__ATTR(anti_aliasing_frequency,  0644, attr_aa_filter_show,
			attr_aa_filter_store),
	__ATTR(temperature,  0644, attr_temp_show, attr_null_store),
#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL_LP
	__ATTR(pmode,  0644, attr_pmode_show, attr_pmode_store),
#endif
	__ATTR_NULL,
};

static struct device_attribute lsm6dx0_gyr_attributes[] = {
	__ATTR(gyro_check, 0444, attr_check_gyro_show, NULL),
	__ATTR(pollrate_ms, 0644, attr_polling_rate_gyr_show,
			attr_polling_rate_gyr_store),
	__ATTR(enable_device, 0644, attr_enable_gyr_show,
			attr_enable_gyr_store),
	__ATTR(range, 0644, attr_range_gyr_show, attr_range_gyr_store),
	__ATTR_NULL,
};

static int create_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;
	int err = 0;

	for (i = 0; NULL != attrs[i].attr.name; ++i) {
		err = device_create_file(dev, &attrs[i]);
		if (err)
			break;
	}

	if (err) {
		for (--i; i >= 0; --i)
			device_remove_file(dev, &attrs[i]);
	}

	return err;
}

static void remove_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;

	for (i = 0; NULL != attrs[i].attr.name; ++i)
		device_remove_file(dev, &attrs[i]);
}

static int create_class_sysfs_interfaces(struct lsm6dx0_status *stat)
{
	int err = 0;

	stat->acc = class_create(THIS_MODULE, "acc");
	if (IS_ERR(stat->acc)) {
		err = PTR_ERR(stat->acc);
		goto exit_class_create_failed;
	}

	stat->gyr = class_create(THIS_MODULE, "gyro");
	if (IS_ERR(stat->gyr)) {
		err = PTR_ERR(stat->gyr);
		goto exit_gyr_class_create_failed;
	}

	stat->acc_class_dev = device_create(
						stat->acc,
						NULL,
						lsm6dx0_acc_device_dev_t,
						stat,
						"acc");
	if (IS_ERR(stat->acc_class_dev)) {
		err = PTR_ERR(stat->acc_class_dev);
		goto exit_class_device_create_failed;
	}

	stat->gyr_class_dev = device_create(
						stat->gyr,
						NULL,
						lsm6dx0_gyr_device_dev_t,
						stat,
						"gyro");
	if (IS_ERR(stat->gyr_class_dev)) {
		err = PTR_ERR(stat->gyr_class_dev);
		goto exit_class_gyr_device_create_failed;
	}

	err = sysfs_create_link(
			&stat->acc_class_dev->kobj,
			&stat->client->dev.kobj,
			device_link_name);
	if (0 > err)
		goto exit_sysfs_create_link_failed;

	err = sysfs_create_link(
		&stat->gyr_class_dev->kobj,
		&stat->client->dev.kobj,
		device_link_name);
	if (0 > err)
		goto exit_sysfs_create_gyr_link_failed;

	err = create_device_attributes(
			stat->acc_class_dev,
			lsm6dx0_acc_attributes);
	if (0 > err)
		goto exit_device_attributes_create_failed;

	err = create_device_attributes(
			stat->gyr_class_dev,
			lsm6dx0_gyr_attributes);
	if (0 > err)
		goto exit_gyr_device_attributes_create_failed;

	return err;

exit_gyr_device_attributes_create_failed:
	remove_device_attributes(stat->acc_class_dev, lsm6dx0_acc_attributes);
exit_device_attributes_create_failed:
	sysfs_remove_link(&stat->gyr_class_dev->kobj, device_link_name);
exit_sysfs_create_gyr_link_failed:
	sysfs_remove_link(&stat->acc_class_dev->kobj, device_link_name);
exit_sysfs_create_link_failed:
	device_destroy(stat->gyr, lsm6dx0_gyr_device_dev_t);
exit_class_gyr_device_create_failed:
	device_destroy(stat->acc, lsm6dx0_acc_device_dev_t);
	stat->gyr_class_dev = NULL;
exit_class_device_create_failed:
	class_destroy(stat->gyr);
	stat->acc_class_dev = NULL;
exit_gyr_class_create_failed:
	stat->gyr = NULL;
	class_destroy(stat->acc);
exit_class_create_failed:
	stat->acc = NULL;
	return err;
}

static void remove_class_sysfs_interfaces(struct lsm6dx0_status *stat)
{
	if (NULL == stat)
		return;

	if (stat->acc_class_dev) {
		remove_device_attributes(
			stat->acc_class_dev,
			lsm6dx0_acc_attributes);
		sysfs_remove_link(
			&stat->acc_class_dev->kobj,
			device_link_name);
		stat->acc_class_dev = NULL;
	}

	if (stat->gyr_class_dev) {
			remove_device_attributes(
				stat->gyr_class_dev,
				lsm6dx0_gyr_attributes);
			sysfs_remove_link(
				&stat->gyr_class_dev->kobj,
				device_link_name);
			stat->gyr_class_dev = NULL;
		}

	if (stat->acc) {
		device_destroy(
			stat->acc,
			lsm6dx0_acc_device_dev_t);
		class_destroy(stat->acc);
		stat->acc = NULL;
	}

	if (NULL != stat->gyr) {
		device_destroy(
			stat->gyr,
			lsm6dx0_gyr_device_dev_t);
		class_destroy(stat->gyr);
		stat->gyr = NULL;
	}
}

static void poll_function_work_acc(struct work_struct *input_work_acc)
{
	struct lsm6dx0_status *stat;
	int32_t xyz[3] = {0}, err = -1;

	stat = container_of((struct work_struct *)input_work_acc,
			struct lsm6dx0_status, input_work_acc);

	err = lsm6dx0_acc_get_data(stat, xyz);

	if (err < 0)
		SENSOR_LOG_ERROR("get accelerometer data failed\n");
	else {
		mutex_lock(&stat->lock);
		if (stat->acc_discard_samples > 0) {
			stat->acc_discard_samples--;
			mutex_unlock(&stat->lock);
		} else {
			mutex_unlock(&stat->lock);
			lsm6dx0_acc_report_values(stat, xyz);
		}
	}

	hrtimer_start(&stat->hr_timer_acc, stat->ktime_acc, HRTIMER_MODE_REL);
}

static void poll_function_work_gyr(struct work_struct *input_work_gyr)
{
	struct lsm6dx0_status *stat;
	int32_t xyz[3] = {0}, err = -1;

	stat = container_of((struct work_struct *)input_work_gyr,
			struct lsm6dx0_status, input_work_gyr);

	err = lsm6dx0_gyr_get_data(stat, xyz);
	if (err < 0)
		SENSOR_LOG_ERROR("get gyroscope data failed.\n");
	else {
#ifdef CONFIG_SENSORS_LSM6DX0_LP
		mutex_lock(&stat->lock);
		if (stat->gyr_discard_samples > 0) {
			stat->gyr_discard_samples--;
			mutex_unlock(&stat->lock);
		} else {
			mutex_unlock(&stat->lock);
			lsm6dx0_gyr_report_values(stat, xyz);
		}
#else
		lsm6dx0_gyr_report_values(stat, xyz);
#endif
	}

	hrtimer_start(&stat->hr_timer_gyr, stat->ktime_gyr, HRTIMER_MODE_REL);
}

#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
int32_t lsm6dx0_hw_comp_read_k(struct lsm6dx0_status *stat, uint8_t reg,
								int8_t *val)
{
	int32_t err = -1;

	/* Read hardcoded factor */
	(*val) = reg;
	err = lsm6dx0_i2c_read(stat, val, 1);
	if (err < 0)
		return err;

	if ((*val) & (0x20)) {
		(*val) |= (0xE0);
		(*val) &= (0xFF);
	} else {
		(*val) &= (0x3F);
	}
	return err;
}

int32_t lsm6dx0_hw_compensation_disable(struct lsm6dx0_status *stat,
								uint8_t reg)
{
	uint8_t tmp[2];

	switch (reg) {
	case COMP_X_FACTOR_ADDR:
		tmp[1] = stat->k_coeff[0];
		break;

	case COMP_Y_FACTOR_ADDR:
		tmp[1] = stat->k_coeff[1];
		break;

	case COMP_Z_FACTOR_ADDR:
		tmp[1] = stat->k_coeff[2];
		break;
	default:
		SENSOR_LOG_ERROR("invalid param\n");
		break;
	}

	/* Disable HW compensation */
	tmp[0] = reg;
	tmp[1] &= COMP_HW_DISABLE_MASK;
	return lsm6dx0_i2c_write(stat, tmp, 1);
}

static int32_t lsm6dx0_gyr_temp_compensation_init(struct lsm6dx0_status *stat)
{
	int32_t err = -1;
	int8_t k_coeff[3] = {0};

	/* reset coefficients */
	mutex_lock(&stat->lock);
	stat->k_coeff[0] = 0;
	stat->k_coeff[1] = 0;
	stat->k_coeff[2] = 0;
	mutex_unlock(&stat->lock);

	err = lsm6dx0_hw_comp_read_k(stat, COMP_X_FACTOR_ADDR, &k_coeff[0]);
	if (err < 0)
		return err;

	err = lsm6dx0_hw_comp_read_k(stat, COMP_Y_FACTOR_ADDR, &k_coeff[1]);
	if (err < 0)
		return err;

	err = lsm6dx0_hw_comp_read_k(stat, COMP_Z_FACTOR_ADDR, &k_coeff[2]);
	if (err < 0)
		return err;

	mutex_lock(&stat->lock);
	err = lsm6dx0_hw_compensation_disable(stat, COMP_X_FACTOR_ADDR);
	if (err < 0)
		goto err_dis_comp;
	stat->k_coeff[0] = k_coeff[0];

	err = lsm6dx0_hw_compensation_disable(stat, COMP_Y_FACTOR_ADDR);
	if (err < 0)
		goto err_dis_comp;
	stat->k_coeff[1] = k_coeff[1];

	err = lsm6dx0_hw_compensation_disable(stat, COMP_Z_FACTOR_ADDR);
	if (err < 0)
		goto err_dis_comp;
	stat->k_coeff[2] = k_coeff[2];

err_dis_comp:
	mutex_unlock(&stat->lock);
	return err;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id lsm6dx0_acc_gyr_dt_id[] = {
	{.compatible = "st,lsm6ds0",},
	{},
};
MODULE_DEVICE_TABLE(of, lsm6dx0_acc_gyr_dt_id);

static int32_t lsm6dx0_acc_gyr_parse_dt(struct lsm6dx0_status *stat,
					struct device *dev)
{
	struct device_node *dn;
	uint8_t i, j;
	uint32_t val;
	short vect[9] = {0};

	if (of_match_device(lsm6dx0_acc_gyr_dt_id, dev)) {
		mutex_lock(&stat->lock);
		dn = dev->of_node;
		stat->pdata_main->of_node = dn;

		stat->pdata_main->gpio_int1 = of_get_gpio(dn, 0);
		if (!gpio_is_valid(stat->pdata_main->gpio_int1)) {
			SENSOR_LOG_ERROR("failed to get gpio_int1\n");
			stat->pdata_main->gpio_int1 = LSM6DX0_INT1_GPIO_DEF;
		}

		stat->pdata_main->gpio_int2 = of_get_gpio(dn, 1);
		if (!gpio_is_valid(stat->pdata_main->gpio_int2)) {
			SENSOR_LOG_ERROR("failed to get gpio_int2\n");
			stat->pdata_main->gpio_int2 = LSM6DX0_INT2_GPIO_DEF;
		}

		if (of_property_read_u16_array(dn, "rot-matrix", vect,
				  ARRAY_SIZE(vect)) >= 0) {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 3; i++) {
					stat->pdata_main->rot_matrix[i][j] =
						(short)vect[3 * j + i];
				}
			}
		} else {
			for (j = 0; j < 3; j++) {
				for (i = 0; i < 3; i++) {
					stat->pdata_main->rot_matrix[i][j] =
			default_lsm6dx0_main_platform_data.rot_matrix[i][j];
				}
			}
		}

		if (!of_property_read_u32(dn, "g-poll-interval", &val))
			stat->pdata_gyr->poll_interval = val;
		else
			stat->pdata_gyr->poll_interval =
				LSM6DX0_GYR_POLL_INTERVAL_DEF;

		if (!of_property_read_u32(dn, "g-min-interval", &val))
			stat->pdata_gyr->min_interval = val;
		else
			stat->pdata_gyr->min_interval =
				LSM6DX0_GYR_MIN_POLL_PERIOD_MS;

		if (!of_property_read_u32(dn, "g-fs-range", &val))
			stat->pdata_gyr->fs_range = val;
		else
			stat->pdata_gyr->fs_range = LSM6DX0_GYR_FS_245DPS;

		if (!of_property_read_u32(dn, "x-poll-interval", &val))
			stat->pdata_acc->poll_interval = val;
		else
			stat->pdata_acc->poll_interval =
				LSM6DX0_ACC_POLL_INTERVAL_DEF;

		if (!of_property_read_u32(dn, "x-min-interval", &val))
			stat->pdata_acc->min_interval = val;
		else
			stat->pdata_acc->min_interval =
				LSM6DX0_ACC_MIN_POLL_PERIOD_MS;

		if (!of_property_read_u32(dn, "x-fs-range", &val))
			stat->pdata_acc->fs_range = val;
		else
			stat->pdata_acc->fs_range = LSM6DX0_ACC_FS_2G;

		if (!of_property_read_u32(dn, "aa-filter-bw", &val))
			stat->pdata_acc->fs_range = val;
		else
			stat->pdata_acc->fs_range = LSM6DX0_ACC_BW_400;
		mutex_unlock(&stat->lock);
		return 0;
	}
	return -1;
}
#endif

static int32_t lsm6dx0_acc_gyr_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lsm6dx0_status *stat;
	int32_t err = -1;
	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	SENSOR_LOG_INFO("probe start.\n");

	stat = kzalloc(sizeof(struct lsm6dx0_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("failed to allocate memory for module data: "
					"%d\n", err);
		goto failed;
	}

	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_LOG_INFO("client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			SENSOR_LOG_INFO("client using SMBUS\n");
		} else {
			err = -ENODEV;
			SENSOR_LOG_ERROR("client nor SMBUS capable\n");
			goto exit_check_functionality_failed;
		}
	}

	stat->lsm6dx0_workqueue = create_workqueue("lsm6dx0_workqueue");
	if (!stat->lsm6dx0_workqueue) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("create_workqueue failed\n");
		goto exit_check_functionality_failed;
	}

	hrtimer_init(&stat->hr_timer_acc, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_acc.function = &poll_function_read_acc;
	hrtimer_init(&stat->hr_timer_gyr, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_gyr.function = &poll_function_read_gyr;

	mutex_init(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);

	stat->pdata_main = kzalloc(sizeof(*stat->pdata_main), GFP_KERNEL);
	stat->pdata_acc = kzalloc(sizeof(*stat->pdata_acc), GFP_KERNEL);
	stat->pdata_gyr = kzalloc(sizeof(*stat->pdata_gyr), GFP_KERNEL);
	if ((stat->pdata_main == NULL) || (stat->pdata_acc == NULL) ||
						  (stat->pdata_gyr == NULL)) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("failed to allocate memory for pdata: %d\n", err);
		goto err_memory_alloc;
	}
	stat->pdata_main->pdata_acc = stat->pdata_acc;
	stat->pdata_main->pdata_gyr = stat->pdata_gyr;

#ifdef CONFIG_OF
	lsm6dx0_acc_gyr_parse_dt(stat, &client->dev);
#else
	if (client->dev.platform_data == NULL) {
		memcpy(stat->pdata_main,
				&default_lsm6dx0_main_platform_data,
				sizeof(*stat->pdata_main));
		memcpy(stat->pdata_acc, &default_lsm6dx0_acc_pdata,
						sizeof(*stat->pdata_acc));
		memcpy(stat->pdata_gyr, &default_lsm6dx0_gyr_pdata,
						sizeof(*stat->pdata_gyr));
		SENSOR_LOG_INFO("using default plaform_data for "
					"accelerometer and gyroscope\n");
	} else {
		struct lsm6dx0_main_platform_data *platform_data;
		platform_data = client->dev.platform_data;

		if (platform_data == NULL) {
			memcpy(stat->pdata_main,
				&default_lsm6dx0_main_platform_data,
				sizeof(*stat->pdata_main));
			SENSOR_LOG_INFO("using default plaform_data for "
							"accelerometer\n");
		} else {
			memcpy(stat->pdata_main, platform_data,
						sizeof(*stat->pdata_acc));
		}

		if (platform_data->pdata_acc == NULL) {
			memcpy(stat->pdata_acc, &default_lsm6dx0_acc_pdata,
						sizeof(*stat->pdata_acc));
			SENSOR_LOG_INFO("using default plaform_data for "
							"accelerometer\n");
		} else {
			memcpy(stat->pdata_acc, platform_data->pdata_acc,
						sizeof(*stat->pdata_acc));
		}

		if (platform_data->pdata_gyr == NULL) {
			memcpy(stat->pdata_gyr, &default_lsm6dx0_gyr_pdata,
						sizeof(*stat->pdata_gyr));
			SENSOR_LOG_INFO("using default plaform_data for "
							"gyroscope\n");
		} else {
			memcpy(stat->pdata_gyr, platform_data->pdata_gyr,
						sizeof(*stat->pdata_gyr));
		}
	}
#endif

	err = lsm6dx0_acc_validate_pdata(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("failed to validate platform data for "
							"accelerometer\n");
		goto exit_kfree_pdata;
	}

	err = lsm6dx0_gyr_validate_pdata(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("failed to validate platform data for "
							"gyroscope\n");
		goto exit_kfree_pdata;
	}

	if (stat->pdata_acc->init) {
		err = stat->pdata_acc->init();
		if (err < 0) {
			SENSOR_LOG_ERROR("accelerometer init failed: "
							"%d\n", err);
			goto err_pdata_acc_init;
		}
	}
	if (stat->pdata_gyr->init) {
		err = stat->pdata_gyr->init();
		if (err < 0) {
			SENSOR_LOG_ERROR("gyroscope init failed: %d\n", err);
			goto err_pdata_gyr_init;
		}
	}

	err = lsm6dx0_acc_gyr_power_init(stat, 1);
	if (err < 0)
		goto err_power_init;

	err = lsm6dx0_acc_gyr_pinctrl_init(stat);
	if (err) {
		SENSOR_LOG_ERROR("pinctrl init failed\n");
		goto err_pinctrl_init;
	}
	err = pinctrl_select_state(stat->lsm6dx0_pctrl.pinctrl,
			stat->lsm6dx0_pctrl.gpio_state_suspend);
	if (err) {
		SENSOR_LOG_ERROR("%s:%d cannot set pin to gpio_state_active state",
			__func__, __LINE__);
		goto err_pinctrl_init;
	}

	err = lsm6dx0_acc_gyr_hw_init(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("hw init failed: %d\n", err);
		goto err_hw_init;
	}

	err = lsm6dx0_acc_device_power_on(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("accelerometer power on failed: %d\n", err);
		goto err_pdata_init;
	}

	err = lsm6dx0_gyr_device_power_on(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("gyroscope power on failed: %d\n", err);
		goto err_pdata_init;
	}

	err = lsm6dx0_acc_update_fs_range(stat, stat->pdata_acc->fs_range);
	if (err < 0) {
		SENSOR_LOG_ERROR("update accelerometer full scale range failed\n");
		goto  err_power_off_acc;
	}

	err = lsm6dx0_gyr_update_fs_range(stat, stat->pdata_gyr->fs_range);
	if (err < 0) {
		SENSOR_LOG_ERROR("update gyroscope full scale range failed\n");
		goto  err_power_off_gyr;
	}

	err = lsm6dx0_acc_update_odr(stat, stat->pdata_acc->poll_interval);
	if (err < 0) {
		SENSOR_LOG_ERROR("update accelerometer ODR failed\n");
		goto  err_power_off;
	}

	err = lsm6dx0_gyr_update_odr(stat, stat->pdata_gyr->poll_interval);
	if (err < 0) {
		SENSOR_LOG_ERROR("update gyroscope ODR failed\n");
		goto  err_power_off;
	}

	err = lsm6dx0_acc_update_filter(stat,
					stat->pdata_acc->aa_filter_bandwidth);
	if (err < 0) {
		SENSOR_LOG_ERROR("update accelerometer filter failed\n");
		goto  err_power_off;
	}

#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
	err = lsm6dx0_gyr_temp_compensation_init(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("sw temperature compensation init failed\n");
		goto  err_power_off;
	}
#endif

	err = lsm6dx0_acc_input_init(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("accelerometer input init failed\n");
		goto err_power_off;
	}

	err = lsm6dx0_gyr_input_init(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("gyroscope input init failed\n");
		goto err_power_off;
	}

	err = create_class_sysfs_interfaces(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("failed to create_class_sysfs_interfaces\n");
		goto err_input_cleanup;
	}

	err = create_sysfs_interfaces(stat);
	if (err < 0) {
		SENSOR_LOG_ERROR("device %s sysfs register failed\n",
			LSM6DX0_ACC_GYR_DEV_NAME);
		goto err_create_sysfs;
	}

	lsm6dx0_acc_device_power_off(stat);
	lsm6dx0_gyr_device_power_off(stat);

	INIT_WORK(&stat->input_work_acc, poll_function_work_acc);
	INIT_WORK(&stat->input_work_gyr, poll_function_work_gyr);

	SENSOR_LOG_INFO("%s: probed success\n", LSM6DX0_ACC_GYR_DEV_NAME);
	return 0;

err_create_sysfs:
	remove_class_sysfs_interfaces(stat);
err_input_cleanup:
	lsm6dx0_input_cleanup(stat);
err_power_off:
err_power_off_gyr:
	lsm6dx0_gyr_device_power_off(stat);
err_power_off_acc:
	lsm6dx0_acc_device_power_off(stat);
err_hw_init:
err_pinctrl_init:
	lsm6dx0_acc_gyr_power_init(stat, 0);
err_pdata_init:
err_power_init:
err_pdata_gyr_init:
	if (stat->pdata_gyr->exit)
		stat->pdata_gyr->exit();
err_pdata_acc_init:
	if (stat->pdata_acc->exit)
		stat->pdata_acc->exit();
exit_kfree_pdata:
	mutex_lock(&stat->lock);
	kfree(stat->pdata_acc);
	kfree(stat->pdata_gyr);
	kfree(stat->pdata_main);
	mutex_unlock(&stat->lock);
err_memory_alloc:
	flush_workqueue(stat->lsm6dx0_workqueue);
	destroy_workqueue(stat->lsm6dx0_workqueue);
exit_check_functionality_failed:
	SENSOR_LOG_ERROR("%s: porbe failed\n", LSM6DX0_ACC_GYR_DEV_NAME);
	kfree(stat);
failed:
	return err;
}

static int32_t lsm6dx0_acc_gyr_remove(struct i2c_client *client)
{
	struct lsm6dx0_status *stat = i2c_get_clientdata(client);

	if (lsm6dx0_acc_gyr_power_init(stat, 0))
		SENSOR_LOG_ERROR("power deinit failed.\n");

	if (atomic_read(&stat->enabled_gyr)) {
		lsm6dx0_gyr_disable(stat);
		lsm6dx0_gyr_input_cleanup(stat);

		if (stat->pdata_gyr->exit)
			stat->pdata_gyr->exit();
	}

	lsm6dx0_acc_disable(stat);
	lsm6dx0_acc_input_cleanup(stat);

	remove_class_sysfs_interfaces(stat);
	remove_sysfs_interfaces(stat);

	if (stat->pdata_acc->exit)
		stat->pdata_acc->exit();

	flush_workqueue(stat->lsm6dx0_workqueue);
	destroy_workqueue(stat->lsm6dx0_workqueue);

	kfree(stat->pdata_acc);
	kfree(stat->pdata_gyr);
	kfree(stat->pdata_main);
	kfree(stat);
	return 0;
}

static int lsm6dx0_acc_gyr_resume(struct device *dev)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);

	SENSOR_LOG_DEBUG("enter\n");

	if (atomic_read(&stat->enabled_acc) == 1) {
		SENSOR_LOG_DEBUG("schedule acc work\n");
		hrtimer_start(&stat->hr_timer_acc,
				stat->ktime_acc, HRTIMER_MODE_REL);
	}

	if (atomic_read(&stat->enabled_gyr) == 1) {
		SENSOR_LOG_DEBUG("schedule gyro work\n");
		hrtimer_start(&(stat->hr_timer_gyr),
				stat->ktime_gyr, HRTIMER_MODE_REL);
	}

	SENSOR_LOG_DEBUG("exit\n");
	return 0;
}

static int lsm6dx0_acc_gyr_suspend(struct device *dev)
{
	struct lsm6dx0_status *stat = dev_get_drvdata(dev);

	SENSOR_LOG_DEBUG("enter\n");

	if (atomic_read(&stat->enabled_acc) == 1) {
		SENSOR_LOG_DEBUG("cancel acc work\n");
		cancel_work_sync(&stat->input_work_acc);
		flush_work(&stat->input_work_acc);
		hrtimer_cancel(&stat->hr_timer_acc);
	}

	if (atomic_read(&stat->enabled_gyr) == 1) {
		SENSOR_LOG_DEBUG("cancel gyro work\n");
		cancel_work_sync(&stat->input_work_gyr);
		flush_work(&stat->input_work_gyr);
		hrtimer_cancel(&stat->hr_timer_gyr);
	}

	SENSOR_LOG_DEBUG("exit\n");
	return 0;
}

static const struct i2c_device_id lsm6dx0_acc_gyr_id[] = {
	{LSM6DX0_ACC_GYR_DEV_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, lsm6dx0_acc_gyr_id);

static const struct dev_pm_ops lsm6dx0_acc_gyr_pm_ops = {
	.suspend	= lsm6dx0_acc_gyr_suspend,
	.resume		= lsm6dx0_acc_gyr_resume,
};

static struct i2c_driver lsm6dx0_acc_gyr_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM6DX0_ACC_GYR_DEV_NAME,
#ifdef CONFIG_OF
			.of_match_table = lsm6dx0_acc_gyr_dt_id,
#endif
			.pm = &lsm6dx0_acc_gyr_pm_ops,
		  },
	.probe = lsm6dx0_acc_gyr_probe,
	.remove = lsm6dx0_acc_gyr_remove,
	.id_table = lsm6dx0_acc_gyr_id,
};

static int32_t __init lsm6dx0_acc_gyr_init(void)
{
	return i2c_add_driver(&lsm6dx0_acc_gyr_driver);
}

static void __exit lsm6dx0_acc_gyr_exit(void)
{
	i2c_del_driver(&lsm6dx0_acc_gyr_driver);
}

module_init(lsm6dx0_acc_gyr_init);
module_exit(lsm6dx0_acc_gyr_exit);

MODULE_DESCRIPTION(LSM6DX0_MOD_DESCRIPTION);
MODULE_AUTHOR("Giuseppe Barba,STMicroelectronics");
MODULE_LICENSE("GPL");

