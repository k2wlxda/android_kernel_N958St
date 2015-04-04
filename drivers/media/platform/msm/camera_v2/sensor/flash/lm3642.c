/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include <mach/gpiomux.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"

#define FLASH_NAME "ti,lm3642"

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define LM3642_DBG(fmt, args...) pr_err(fmt, ##args)
#else
#define LM3642_DBG(fmt, args...)
#endif


static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3642_i2c_driver;

static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	{0x0A, 0x00},
	{0x08, 0x07},
	{0x09, 0x19},
};

static struct msm_camera_i2c_reg_array lm3642_off_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_release_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_low_array[] = {
	{0x0A, 0x22},
};

static struct msm_camera_i2c_reg_array lm3642_high_array[] = {
	{0x0A, 0x23},
};



static void __exit msm_flash_lm3642_i2c_remove(void)
{
	i2c_del_driver(&lm3642_i2c_driver);
	return;
}

static const struct of_device_id lm3642_trigger_dt_match[] = {
	{.compatible = "ti,lm3642", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, lm3642_trigger_dt_match);

static const struct i2c_device_id lm3642_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};


static int msm_flash_lm3642_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!id) {
		pr_err("msm_flash_lm3642_i2c_probe: id is NULL");
		id = lm3642_i2c_id;
	}
    pr_err("%s:%d called\n", __func__, __LINE__);
	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver lm3642_i2c_driver = {
	.id_table = lm3642_i2c_id,
	.probe  = msm_flash_lm3642_i2c_probe,
	.remove = __exit_p(msm_flash_lm3642_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_trigger_dt_match,
	},
};

static int msm_flash_lm3642_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_device(lm3642_trigger_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;
	return msm_flash_probe(pdev, match->data);
}
static struct platform_driver lm3642_platform_driver = {
	.probe = msm_flash_lm3642_platform_probe,
	.driver = {
		.name = "ti,lm3642",
		.owner = THIS_MODULE,
		.of_match_table = lm3642_trigger_dt_match,
	},
};

static int __init msm_flash_lm3642_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&lm3642_platform_driver);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&lm3642_i2c_driver);
}
static void __exit msm_flash_lm3642_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&lm3642_platform_driver);
	else
		i2c_del_driver(&lm3642_i2c_driver);
}

static struct msm_camera_i2c_client lm3642_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_off_setting = {
	.reg_setting = lm3642_off_array,
	.size = ARRAY_SIZE(lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_release_setting = {
	.reg_setting = lm3642_release_array,
	.size = ARRAY_SIZE(lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting = {
	.reg_setting = lm3642_low_array,
	.size = ARRAY_SIZE(lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting = {
	.reg_setting = lm3642_high_array,
	.size = ARRAY_SIZE(lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3642_regs = {
	.init_setting = &lm3642_init_setting,
	.off_setting = &lm3642_off_setting,
	.low_setting = &lm3642_low_setting,
	.high_setting = &lm3642_high_setting,
	.release_setting = &lm3642_release_setting,
};

static struct msm_flash_fn_t lm3642_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3642_i2c_client,
	.reg_setting = &lm3642_regs,
	.func_tbl = &lm3642_func_tbl,
};

module_init(msm_flash_lm3642_init_module);
module_exit(msm_flash_lm3642_exit_module);
MODULE_DESCRIPTION("lm3642 FLASH");
MODULE_LICENSE("GPL v2");
