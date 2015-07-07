/*
 * tps65312 driver
 *
 * Copyright (C) 2014 ZTEMT
 *
 * Create by mayu.
 * Modify by luochangyang 2014.06.01
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include "zte_lcd_avdd.h"

#define LCD_AVDD_DRIVER_NAME	"lcd_avdd"

#define LOG_TAG "LCD_AVDD"
#define DEBUG_ON //DEBUG SWITCH
#define LOG_ERROR(fmt, args...) printk(KERN_ERR   "[%s] [%s: %d] "  fmt, LOG_TAG,__FUNCTION__, __LINE__, ##args)

#ifdef  DEBUG_ON
#define LOG_INFO(fmt, args...)  printk(KERN_INFO  "[%s] [%s: %d] "  fmt, LOG_TAG,__FUNCTION__, __LINE__, ##args)                                              
#define LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s: %d] "  fmt, LOG_TAG,__FUNCTION__, __LINE__, ##args)
#else
#define LOG_INFO(fmt, args...)
#define LOG_DEBUG(fmt, args...)
#endif

struct lcd_avdd_data	{
	struct i2c_client	*client;
    struct regulator *vcc_i2c;

    int vsp_en_gpio;
    int vsn_en_gpio;
    
    bool chip_is_ti;
};

static struct lcd_avdd_data * lcd_avdd_dev;

static void lcd_avdd_parse_dt(struct lcd_avdd_data *chip)
{
	struct device_node *np = chip->client->dev.of_node;

    chip->vsp_en_gpio= of_get_named_gpio_flags(np, "vsp-en-gpio", 0, NULL);
    LOG_INFO("vsp_en_gpio is %d\n",chip->vsp_en_gpio);

    chip->vsn_en_gpio= of_get_named_gpio_flags(np, "vsn-en-gpio", 0, NULL);
    LOG_INFO("vsn_en_gpio is %d\n",chip->vsn_en_gpio);

}


static int lcd_avdd_write_reg_val(unsigned char reg, unsigned char val)
{
	int i = 0;
	int err = 0;

	while (i < 3)
	{
		err = i2c_smbus_write_byte_data(lcd_avdd_dev->client, reg, val);
		if(err < 0){
			LOG_ERROR("err=%d\n", err);
			i++;
		} else {
			break;
		}
	}

	return err;
}

static unsigned char lcd_avdd_read_reg( unsigned char reg)
{
	return i2c_smbus_read_byte_data(lcd_avdd_dev->client, reg);
}

void lcd_avdd_set_output_voltage(int vsp_voltage, int vsn_voltage)
{
	int pos_val=0;
	int neg_val=0;
	int regff_val=0;	
	pos_val= ( vsp_voltage - 4000)/100;
	neg_val= ( vsn_voltage - 4000)/100;

	lcd_avdd_write_reg_val(0x00,pos_val);
	lcd_avdd_write_reg_val(0x01,neg_val);
    if (lcd_avdd_dev->chip_is_ti)
    {
    	regff_val=lcd_avdd_read_reg(0xff);
    	regff_val |=(1<<8);
    	lcd_avdd_write_reg_val(0xff,regff_val);
    }
}

void lcd_avdd_on(int vsp_voltage, int vsn_voltage,int vsp_vsn_delay)
{
    LOG_ERROR("enter\n");
    gpio_direction_output(lcd_avdd_dev->vsp_en_gpio, 1);
    msleep(vsp_vsn_delay);  
    gpio_direction_output(lcd_avdd_dev->vsn_en_gpio, 1);
    lcd_avdd_set_output_voltage(vsp_voltage, vsn_voltage);
    msleep(vsp_vsn_delay);  
    LOG_ERROR("exit\n");
}

void lcd_avdd_off(int vsp_vsn_delay, int avdd_off_delay)
{
    LOG_ERROR("enter\n");
    msleep(avdd_off_delay);
    gpio_direction_output(lcd_avdd_dev->vsn_en_gpio, 0);
    msleep(vsp_vsn_delay);  
    gpio_direction_output(lcd_avdd_dev->vsp_en_gpio, 0);
    LOG_ERROR("exit\n");
}


void  lcd_avdd_show_reg(void )
{
	int val0,val1,regff_val;
	val0=lcd_avdd_read_reg(0x00);
	val1=lcd_avdd_read_reg(0x01);
	regff_val=lcd_avdd_read_reg(0xff);
}

void lcd_avdd_chip_is_ti(struct lcd_avdd_data *chip)
{
    LOG_INFO("%d\n",lcd_avdd_read_reg(0xff));

    if (lcd_avdd_read_reg(0xff) == 0)
    {
        chip->chip_is_ti = true;
    }
    else
    {
        chip->chip_is_ti = false;
    }

    LOG_INFO("lcd avdd chip is %s\n", (chip->chip_is_ti==true) ? "TPS65132" : "KTD2151");

}


static int lcd_avdd_power_init(struct lcd_avdd_data *chip)
{
    struct i2c_client *client = chip->client;
    int ret = 0;

    chip->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if (IS_ERR(chip->vcc_i2c ))
	{
		ret = PTR_ERR(chip->vcc_i2c );
		LOG_ERROR("Regulator get failed ret=%d\n", ret);
	}

    if (regulator_count_voltages(chip->vcc_i2c ) > 0)
    {
        ret = regulator_set_voltage(chip->vcc_i2c , 1800000, 1800000);
        if (ret)
        {
            LOG_ERROR("Regulator set vcc_i2c_power failed ret=%d\n", ret);
            goto error_set_voltage;
        }
    }
    
    ret = regulator_set_optimum_mode(chip->vcc_i2c , 600000);
    if (ret < 0)
    {
        LOG_ERROR("Regulator vcc_i2c_power set_opt failed ret=%d\n", ret);
        goto error_set_optimum;
    }
       
	ret = regulator_enable(chip->vcc_i2c);
	if (ret)
    {
        LOG_ERROR("Regulator vcc_i2c enable failed ret=%d\n", ret);
    }
    return ret;

error_set_optimum:
    regulator_set_voltage(chip->vcc_i2c , 0, 1800000);
    regulator_put(chip->vcc_i2c );

error_set_voltage:
	regulator_put(chip->vcc_i2c );
    LOG_ERROR("failed\n");
	return ret;

}

static int lcd_avdd_gpio_init(struct lcd_avdd_data *chip)
{
    int ret;
    
    LOG_INFO("start\n");
    if (gpio_is_valid(chip->vsp_en_gpio))
    {
        ret = gpio_request(chip->vsp_en_gpio, "lcd_vsp_enable");
        if (ret) 
        {
            LOG_ERROR("request vsp_en_gpio failed, ret = %d\n",ret);
            goto lcd_avdd_gpio_init_error;
        }
        else
        {
            gpio_direction_output(chip->vsp_en_gpio, 1);
        }
    }

    if (gpio_is_valid(chip->vsn_en_gpio))
    {
        ret = gpio_request(chip->vsn_en_gpio, "lcd_vsn_enable");
        if (ret) 
        {
            LOG_ERROR("request vsn_en_gpio failed, ret = %d\n",ret);
            goto lcd_avdd_gpio_init_error;
        }
        else
        {        
            gpio_direction_output(chip->vsn_en_gpio, 1);
        }
    }
    LOG_INFO("success\n");

    return 0;

lcd_avdd_gpio_init_error:
    LOG_INFO("failed\n");
    return ret;
}


static int lcd_avdd_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;  
    LOG_INFO("start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
		LOG_ERROR("need I2C_FUNC_I2C\n");
		return  -ENODEV;
	}

    lcd_avdd_dev = kzalloc(sizeof(struct lcd_avdd_data), GFP_KERNEL);
    if (lcd_avdd_dev == NULL) 
    {
        LOG_ERROR("failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

	lcd_avdd_dev->client = client;
    
   	i2c_set_clientdata(client, lcd_avdd_dev);

    lcd_avdd_parse_dt(lcd_avdd_dev);

    lcd_avdd_power_init(lcd_avdd_dev);

    lcd_avdd_gpio_init(lcd_avdd_dev);

    lcd_avdd_chip_is_ti(lcd_avdd_dev);

    LOG_INFO("success\n");

	return 0;

err_exit:
    LOG_INFO("failed\n");
	return ret;
	
}

static int lcd_avdd_remove(struct i2c_client *client)
{
	struct lcd_avdd_data *dev = i2c_get_clientdata(client);

	LOG_ERROR("\n");

	kfree(dev);

	lcd_avdd_dev = NULL;
	
	return 0;
}

static struct of_device_id dev_match_table[] = {
	{ .compatible = "lcd,avdd",},
	{ },
};

static const struct i2c_device_id lcd_avdd_id_table[] = {
	{ "lcd,avdd", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, lcd_avdd_id_table);

static struct i2c_driver lcd_avdd_driver = {
	.id_table	= lcd_avdd_id_table,
	.probe		= lcd_avdd_probe,
	.remove		= lcd_avdd_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= 	LCD_AVDD_DRIVER_NAME,
		.of_match_table = dev_match_table,
	},
};

// driver init
static int __init lcd_avdd_init(void) 
{
    return i2c_add_driver(&lcd_avdd_driver);
}

// driver exit
static void __exit lcd_avdd_exit(void) 
{
	i2c_del_driver(&lcd_avdd_driver);
}

module_init(lcd_avdd_init);
module_exit(lcd_avdd_exit);

MODULE_DESCRIPTION("I2C_TEST_TPS");
MODULE_LICENSE("GPL");

