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
 * File Name		: lsm6dx0.h
 * Author		: AMS - Motion Mems Division - Application Team
 *			: Giuseppe Barba (giuseppe.barba@st.com)
 * Version		: V.1.1.0
 * Date			: 2014/Apr/18
 */

#ifndef	__LSM6DX0_H__
#define	__LSM6DX0_H__

#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL
/** LSM6DS0 model */
#define LSM6DX0_ACC_GYR_DEV_NAME	"lsm6ds0"
#define LSM6DX0_ACC_DEV_NAME		"lsm6ds0_acc"
#define LSM6DX0_GYR_DEV_NAME		"lsm6ds0_gyr"
#define LSM6DX0_MOD_DESCRIPTION		"lsm6ds0 driver"
#else
/** LSM6DL0 model */
#define LSM6DX0_ACC_GYR_DEV_NAME	"lsm6dl0"
#define LSM6DX0_ACC_DEV_NAME		"lsm6dl0_acc"
#define LSM6DX0_GYR_DEV_NAME		"lsm6dl0_gyr"
#define LSM6DX0_MOD_DESCRIPTION		"lsm6dl0 driver"
#endif

/**********************************************/
/*      Accelerometer section defines         */
/**********************************************/

/* Accelerometer Sensor Full Scale */
#define LSM6DX0_ACC_FS_MASK		(0x18)
#define LSM6DX0_ACC_FS_2G		(0x00)	/* Full scale 2g */
#define LSM6DX0_ACC_FS_4G		(0x10)	/* Full scale 4g */
#define LSM6DX0_ACC_FS_8G		(0x18)	/* Full scale 8g */

/* Accelerometer Anti-Aliasing Filter */
#define LSM6DX0_ACC_BW_400		(0X00)
#define LSM6DX0_ACC_BW_200		(0X01)
#define LSM6DX0_ACC_BW_100		(0X02)
#define LSM6DX0_ACC_BW_50		(0X03)
#define LSM6DX0_ACC_BW_MASK		(0X03)

#define LSM6DX0_INT1_GPIO_DEF		(-EINVAL)
#define LSM6DX0_INT2_GPIO_DEF		(-EINVAL)

#define LSM6DX0_ACC_ODR_OFF		(0x00)
#define LSM6DX0_ACC_ODR_MASK		(0xE0)
#define LSM6DX0_ACC_ODR_14_9		(0x20)
#define LSM6DX0_ACC_ODR_59_5		(0x40)
#define LSM6DX0_ACC_ODR_119		(0x60)
#define LSM6DX0_ACC_ODR_238		(0x80)
#define LSM6DX0_ACC_ODR_476		(0xA0)
#define LSM6DX0_ACC_ODR_952		(0xC0)

#define LSM6DL0_ACC_ODR_20		(0x20)
#define LSM6DL0_ACC_ODR_80		(0x40)
#define LSM6DL0_ACC_ODR_160		(0x60)
#define LSM6DL0_ACC_ODR_320		(0x80)
#define LSM6DL0_ACC_ODR_640		(0xA0)
#define LSM6DL0_ACC_ODR_1280		(0xC0)

/**********************************************/
/* 	Gyroscope section defines	 	*/
/**********************************************/


#define LSM6DX0_GYR_FS_MASK		(0x18)
#define LSM6DX0_GYR_FS_245DPS		(0x00)
#define LSM6DX0_GYR_FS_500DPS		(0x08)
#define LSM6DX0_GYR_FS_2000DPS		(0x18)

#define LSM6DX0_GYR_ODR_OFF		(0x00)
#define LSM6DX0_GYR_ODR_MASK		(0xE0)
#define LSM6DX0_GYR_ODR_MASK_SHIFT	(5)
#define LSM6DX0_GYR_ODR_001		(2 << LSM6DX0_GYR_ODR_MASK_SHIFT)
#define LSM6DX0_GYR_ODR_010		(2 << LSM6DX0_GYR_ODR_MASK_SHIFT)
#define LSM6DX0_GYR_ODR_011		(3 << LSM6DX0_GYR_ODR_MASK_SHIFT)
#define LSM6DX0_GYR_ODR_100		(4 << LSM6DX0_GYR_ODR_MASK_SHIFT)
#define LSM6DX0_GYR_ODR_101		(5 << LSM6DX0_GYR_ODR_MASK_SHIFT)
#define LSM6DX0_GYR_ODR_110		(6 << LSM6DX0_GYR_ODR_MASK_SHIFT)

#ifdef CONFIG_SENSORS_LSM6DX0_LP
#define LSM6DX0_GYR_MAX_LP_ODR_US	(8)
#endif

#define LSM6DX0_GYR_BW_00		(0x00)
#define LSM6DX0_GYR_BW_01		(0x01)
#define LSM6DX0_GYR_BW_10		(0x02)
#define LSM6DX0_GYR_BW_11		(0x03)

/* ODR periods in msec */
#ifdef CONFIG_SENSORS_LSM6DX0_S_MODEL
/* LSM6DS0 model */
#define LSM6DS0_ODR_14_9_US		(67115)
#define LSM6DS0_ODR_59_5_US		(16807)
#define LSM6DS0_ODR_119_US		(8404)
#define LSM6DS0_ODR_238_US		(4202)
#define LSM6DS0_ODR_476_US		(2101)
#define LSM6DS0_ODR_952_US		(1051)

#define LSM6DX0_ODR_US_001		(LSM6DS0_ODR_14_9_US)
#define LSM6DX0_ODR_US_010		(LSM6DS0_ODR_59_5_US)
#define LSM6DX0_ODR_US_011		(LSM6DS0_ODR_119_US)
#define LSM6DX0_ODR_US_100		(LSM6DS0_ODR_238_US)
#define LSM6DX0_ODR_US_101		(LSM6DS0_ODR_476_US)
#define LSM6DX0_ODR_US_110		(LSM6DS0_ODR_952_US)
#else
/* LSM6DL0 model */
#define LSM6DL0_ODR_20_US		(50000)
#define LSM6DL0_ODR_80_US		(12500)
#define LSM6DL0_ODR_160_US		(6250)
#define LSM6DL0_ODR_320_US		(3125)
#define LSM6DL0_ODR_640_US		(1563)
#define LSM6DL0_ODR_1280_US		(782)

#define LSM6DX0_ODR_US_001		(LSM6DL0_ODR_20_US)
#define LSM6DX0_ODR_US_010		(LSM6DL0_ODR_80_US)
#define LSM6DX0_ODR_US_011		(LSM6DL0_ODR_160_US)
#define LSM6DX0_ODR_US_100		(LSM6DL0_ODR_320_US)
#define LSM6DX0_ODR_US_101		(LSM6DL0_ODR_640_US)
#define LSM6DX0_ODR_US_110		(LSM6DL0_ODR_1280_US)
#endif

#define LSM6DX0_GYR_MIN_POLL_PERIOD_US	(LSM6DX0_ODR_US_110)
#define LSM6DX0_ACC_MIN_POLL_PERIOD_US	(LSM6DX0_ODR_US_110)
#define LSM6DX0_GYR_POLL_INTERVAL_DEF	(LSM6DX0_ODR_US_001)
#define LSM6DX0_ACC_POLL_INTERVAL_DEF	(LSM6DX0_ODR_US_001)

struct lsm6dx0_acc_platform_data {
	uint32_t poll_interval;
	uint32_t min_interval;
	uint8_t fs_range;
	uint8_t aa_filter_bandwidth;

	int32_t (*init)(void);
	void (*exit)(void);
	int32_t (*power_on)(void);
	int32_t (*power_off)(void);
};

struct lsm6dx0_gyr_platform_data {
	uint32_t poll_interval;
	uint32_t min_interval;
	uint8_t fs_range;

	int32_t (*init)(void);
	void (*exit)(void);
	int32_t (*power_on)(void);
	int32_t (*power_off)(void);
};

struct lsm6dx0_main_platform_data {
	int32_t gpio_int1;
	int32_t gpio_int2;
	short rot_matrix[3][3];
	struct lsm6dx0_acc_platform_data *pdata_acc;
	struct lsm6dx0_gyr_platform_data *pdata_gyr;
#ifdef CONFIG_OF
	struct device_node	*of_node;
#endif
};

struct output_rate {
	uint32_t cutoff_us;
	uint8_t value;
};

struct discard_list {
	uint32_t param;
	uint8_t count;
};

struct acc_turn_on_time {
	uint32_t odr_us;
	struct discard_list disc_list[4];
};

#ifdef CONFIG_SENSORS_LSM6DX0_LP
struct gyr_turn_on_time {
	uint32_t odr_us;
	struct discard_list disc_list[2];
};
#endif

struct reg_rw {
	uint8_t address;
	uint8_t default_val;
	uint8_t resume_val;
};

struct reg_r {
	uint8_t address;
	uint8_t default_val;
};

struct status_registers {
	struct reg_rw act_ths;
	struct reg_rw act_dur;
	struct reg_rw int_gen_cfg_xl;
	struct reg_rw int_gen_ths_x_xl;
	struct reg_rw int_gen_ths_y_xl;
	struct reg_rw int_gen_ths_z_xl;
	struct reg_rw int_gen_dur_xl;
	struct reg_rw reference_g;
	struct reg_rw int1_ctrl;
	struct reg_rw int2_ctrl;
	struct reg_r who_am_i;
	struct reg_rw ctrl_reg1_g;
	struct reg_rw ctrl_reg2_g;
	struct reg_rw ctrl_reg3_g;
	struct reg_rw orient_cfg_g;
	struct reg_r int_gen_src_g;
	struct reg_r status_reg1;
	struct reg_rw ctrl_reg4;
	struct reg_rw ctrl_reg5_xl;
	struct reg_rw ctrl_reg6_xl;
	struct reg_rw ctrl_reg7_xl;
	struct reg_rw ctrl_reg8;
	struct reg_rw ctrl_reg9;
	struct reg_rw ctrl_reg10;
	struct reg_r int_gen_src_xl;
	struct reg_r status_reg2;
	struct reg_rw fifo_ctrl;
	struct reg_r fifo_src;
	struct reg_rw int_gen_cfg_g;
	struct reg_rw int_gen_ths_xh_g;
	struct reg_rw int_gen_ths_xl_g;
	struct reg_rw int_gen_ths_yh_g;
	struct reg_rw int_gen_ths_yl_g;
	struct reg_rw int_gen_ths_zh_g;
	struct reg_rw int_gen_ths_zl_g;
	struct reg_rw int_gen_dur_g;
};

struct lsm6dx0_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

struct lsm6dx0_status {
	struct i2c_client *client;
	struct lsm6dx0_main_platform_data *pdata_main;
	struct lsm6dx0_acc_platform_data *pdata_acc;
	struct lsm6dx0_gyr_platform_data *pdata_gyr;

	struct class		*acc;
	struct device		*acc_class_dev;

	struct class		*gyr;
	struct device		*gyr_class_dev;

	struct mutex lock;
	struct work_struct input_work_acc;
	struct work_struct input_work_gyr;

	struct hrtimer hr_timer_acc;
	ktime_t ktime_acc;
	struct hrtimer hr_timer_gyr;
	ktime_t ktime_gyr;

	struct input_dev *input_dev_acc;
	struct input_dev *input_dev_gyr;

	struct regulator *vdd;
	struct regulator *vio;

	int8_t hw_initialized;
	/* hw_working=-1 means not tested yet */
	int8_t hw_working;

	atomic_t enabled_acc;
	atomic_t enabled_gyr;
	atomic_t enabled_temp;

	int32_t on_before_suspend;
	int32_t use_smbus;

	uint32_t sensitivity_acc;
	uint32_t sensitivity_gyr;

	int32_t irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;

	int32_t irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	struct workqueue_struct *lsm6dx0_workqueue;

#ifdef CONFIG_SENSORS_LSM6DX0_SW_COMP
	int8_t k_fs;
	int8_t k_coeff[3];
	int32_t compensation_temp;
#endif
#ifdef CONFIG_SENSORS_LSM6DX0_LP
	atomic_t low_power_state;
	atomic_t enabled_lp_mode;

	/* gyr_discard_samples count the number of samples to be discarded
	 * after switching between low-power and normal mode
	 */
	uint8_t gyr_discard_samples;
#endif
	/* acc_discard_samples count the number of samples to be discarded
	 * after switching between power-down mode and normal mode
	 */
	uint8_t acc_discard_samples;

	struct kobject *acc_kobj;
	struct kobject *gyr_kobj;

	struct lsm6dx0_pinctrl_info lsm6dx0_pctrl;
};

#endif	/* __LSM6DX0_H__ */
