/*
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Model dependent functions
 * 
 */

#include "melfas_mms400.h"

/* POWER SUPPLY VOLTAGE RANGE */
#define MELFAS_VDD_MIN_UV	2600000
#define MELFAS_VDD_MAX_UV	3300000
#define MELFAS_VIO_MIN_UV	1750000
#define MELFAS_VIO_MAX_UV	1950000

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

int mms_regulator_control(struct i2c_client *client, int enable)
{
	int rc = 0;
	struct device *dev = &client->dev;
    static struct regulator *vdd_ana;
    static struct regulator *vcc_i2c;

	if (enable) {
		vdd_ana = regulator_get(dev, "vdd_ana");
		if (IS_ERR(vdd_ana)) {
			rc = PTR_ERR(vdd_ana);
			dev_err(dev, "Regulator vdd_ana get failed rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(vdd_ana) > 0) {
			rc = regulator_set_voltage(vdd_ana, MELFAS_VDD_MIN_UV, MELFAS_VDD_MAX_UV);
			if (rc) {
				dev_err(dev, "Regulator vdd_ana set vtg failed rc=%d\n", rc);
				goto set_vtg_vdd_ana_failed;
			}
		}

	    rc = reg_set_optimum_mode_check(vdd_ana, 15000);
	    if (rc < 0) {
	        dev_err(dev, "Regulator vdd_ana set_opt failed rc=%d\n", rc);
	        goto set_opt_vdd_ana_failed;
	    }

	    rc = regulator_enable(vdd_ana);
	    if (rc) {
	        dev_err(dev, "Regulator vdd_ana enable failed rc=%d\n", rc);
	        goto en_vdd_ana_failed;
	    }

		vcc_i2c = regulator_get(dev, "vcc_i2c");
		if (IS_ERR(vcc_i2c)) {
			rc = PTR_ERR(vcc_i2c);
			dev_err(dev, "Regulator vcc_i2c get failed rc=%d\n", rc);
			goto get_vcc_i2c_failed;
		}

		if (regulator_count_voltages(vcc_i2c) > 0) {
			rc = regulator_set_voltage(vcc_i2c, MELFAS_VIO_MIN_UV, MELFAS_VIO_MAX_UV);
			if (rc) {
				dev_err(dev, "Regulator vcc_i2c set vtg failed rc=%d\n", rc);
				goto set_vtg_vcc_i2c_failed;
			}
		}

	    rc = regulator_enable(vcc_i2c);
	    if (rc) {
	        dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
	        goto en_vcc_i2c_failed;
	    }

	    msleep(100);
	} else {
		if (regulator_count_voltages(vcc_i2c) > 0)
			regulator_set_voltage(vcc_i2c, 0, MELFAS_VIO_MAX_UV);
			regulator_put(vcc_i2c);

		if (regulator_count_voltages(vdd_ana) > 0)
			regulator_set_voltage(vdd_ana, 0, MELFAS_VDD_MAX_UV);
		regulator_put(vdd_ana);
	}

    return 0;

en_vcc_i2c_failed:
    if (regulator_count_voltages(vcc_i2c) > 0)
		regulator_set_voltage(vcc_i2c, 0, MELFAS_VIO_MAX_UV);
set_vtg_vcc_i2c_failed:
	regulator_put(vcc_i2c);
get_vcc_i2c_failed:
    regulator_disable(vdd_ana);
en_vdd_ana_failed:
    reg_set_optimum_mode_check(vdd_ana, 0);
set_opt_vdd_ana_failed:
	if (regulator_count_voltages(vdd_ana) > 0)
		regulator_set_voltage(vdd_ana, 0, MELFAS_VDD_MAX_UV);
set_vtg_vdd_ana_failed:
	regulator_put(vdd_ana);
	return rc;
}

#define MELFAS_TP_PINCTRL_ACTIVE   "melfas_pin_active"
#define MELFAS_TP_PINCTRL_SUSPEND  "melfas_pin_suspend"

int mms_pinctrl_init(struct mms_ts_info *info)
{   
    info->pdata->pin_res.pinctrl = devm_pinctrl_get(&info->client->dev);
	if (IS_ERR_OR_NULL(info->pdata->pin_res.pinctrl)) {        
        TP_LOG_ERROR("Failed to get pinctrl\n");
	    return -EINVAL;
	} 

    info->pdata->pin_res.gpio_state_active =
		pinctrl_lookup_state(info->pdata->pin_res.pinctrl, MELFAS_TP_PINCTRL_ACTIVE);
    if (IS_ERR_OR_NULL(info->pdata->pin_res.gpio_state_active))
    {
        TP_LOG_ERROR("Failed to get [%s] state pinctrl\n", MELFAS_TP_PINCTRL_ACTIVE);
	    return -EINVAL;
    }

    info->pdata->pin_res.gpio_state_suspend =
		pinctrl_lookup_state(info->pdata->pin_res.pinctrl, MELFAS_TP_PINCTRL_SUSPEND);
    if (IS_ERR_OR_NULL(info->pdata->pin_res.gpio_state_suspend))
    {
        TP_LOG_ERROR("Failed to get [%s] state pinctrl\n", MELFAS_TP_PINCTRL_SUSPEND);
	    return -EINVAL;
    }

    TP_LOG_DEBUG("Success\n");

	return 0;
}

int mms_pinctrl_enable(struct mms_ts_info *info, bool active)
{       
	int rc = -EFAULT;
	struct pinctrl_state *pin_state;

	pin_state = active ? info->pdata->pin_res.gpio_state_active : info->pdata->pin_res.gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pin_state)) 
	{
		rc = pinctrl_select_state(info->pdata->pin_res.pinctrl, pin_state);
		if (rc)
		{   
            TP_LOG_ERROR("Failed to set [%s] pins\n", active ? MELFAS_TP_PINCTRL_ACTIVE : MELFAS_TP_PINCTRL_SUSPEND);
		}
		else
		{		    
            TP_LOG_ERROR("Success to set [%s] pins\n", active ? MELFAS_TP_PINCTRL_ACTIVE : MELFAS_TP_PINCTRL_SUSPEND);
		}
	} 
	else 
	{
	    TP_LOG_ERROR("invalid [%s] pinstate\n\n", active ? MELFAS_TP_PINCTRL_ACTIVE : MELFAS_TP_PINCTRL_SUSPEND);
	}

    return rc;
}

/**
* Turn off power supply
*/
int mms_power_off(struct mms_ts_info *info)
{
	if (info->power_en == false)
		return 0;

	dev_info(&info->client->dev, "%s [START]\n", __func__);

	//Control reset pin
	gpio_direction_output(info->pdata->gpio_reset, 0);
	msleep(10);
	dev_info(&info->client->dev, "%s [DONE]\n", __func__);

	info->power_en = false;

	return 0;
}

/**
* Turn on power supply
*/
int mms_power_on(struct mms_ts_info *info)
{
	if (info->power_en == true)
		return 0;

	dev_info(&info->client->dev, "%s [START]\n", __func__);

	//Control reset pin
	gpio_direction_output(info->pdata->gpio_reset, 1);
	msleep(50);
	dev_info(&info->client->dev, "%s [DONE]\n", __func__);

	info->power_en = true;

	return 0;
}

/**
* Clear touch input events
*/
void mms_clear_input(struct mms_ts_info *info)
{
	int i = 0;
	for (i = 0; i< MAX_FINGER_NUM; i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
	}
	
	input_sync(info->input_dev);

	return;
}

/**
* Input event handler - Report touch input event
*/
void mms_input_event_handler(struct mms_ts_info *info, u8 sz, u8 *buf)
{
	struct i2c_client *client = info->client;
	int i = 0;

	dev_dbg(&client->dev, "%s [START]\n", __func__);
	dev_dbg(&client->dev, "%s - sz[%d] buf[0x%02X]\n", __func__, sz, buf[0]);

	for (i = 1; i < sz; i += info->event_size) {
		u8 *tmp = &buf[i];

		int id = (tmp[0] & 0xf) - 1;
		int x = tmp[2] | ((tmp[1] & 0xf) << 8);
		int y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
		int touch_major = tmp[4];
		int pressure = tmp[5];

		// Report input data
		if ((tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0) {
			//Touchkey Event
			int key = tmp[0] & 0xf;
			int key_state = (tmp[0] & MIP_EVENT_INPUT_PRESS) ? 1 : 0;
			int key_code = 0;

			/////////////////////////////////
			// MODIFY REQUIRED
			//

			//Report touchkey event
			switch (key) {
				case 1:
					key_code = KEY_MENU;
					dev_err(&client->dev, "Key : KEY_MENU\n");
					break;
				case 2:
					key_code = KEY_HOME;
					dev_err(&client->dev, "Key : KEY_HOME\n");
					break;
				case 3:
					key_code = KEY_BACK;
					dev_err(&client->dev, "Key : KEY_BACK\n");
					break;
				default:
					dev_err(&client->dev, "%s [ERROR] Unknown key code [%d]\n", __func__, key);
					continue;
					break;
			}

			input_report_key(info->input_dev, key_code, key_state);

			dev_err(&client->dev, "%s - Key : ID[%d] Code[%d] State[%d]\n", __func__, key, key_code, key_state);
			
			//
			/////////////////////////////////
		}
		else
		{
			//Touchscreen Event

			/////////////////////////////////
			// MODIFY REQUIRED
			//

			//Report touchscreen event
			if((tmp[0] & MIP_EVENT_INPUT_PRESS) == 0) {
				//Release
				input_mt_slot(info->input_dev, id);
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);

				dev_dbg(&client->dev, "%s - Touch : ID[%d] Release\n", __func__, id);

				continue;
			}

			//Press or Move
			input_mt_slot(info->input_dev, id);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);

			dev_dbg(&client->dev, "%s - Touch : ID[%d] X[%d] Y[%d] P[%d] M[%d] \n", __func__, id, x, y, pressure, touch_major);

			//
			/////////////////////////////////
		}
	}

	input_sync(info->input_dev);

//EXIT:
	dev_dbg(&client->dev, "%s [DONE]\n", __func__);
	return;
}

/**
* Wake-up event handler
*/
int mms_wakeup_event_handler(struct mms_ts_info *info, u8 *rbuf)
{
	//u8 gesture_type = rbuf[2];

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/////////////////////////////////
	// MODIFY REQUIRED
	//

	//Report wake-up event

	//
	//
	/////////////////////////////////

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

//ERROR:
	//return 1;
}

#if MMS_USE_DEVICETREE
/**
* Parse device tree
*/
int mms_parse_devicetree(struct device *dev, struct mms_ts_info *info)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct mms_ts_info *info = i2c_get_clientdata(client);
	struct device_node *np = dev->of_node;
	u32 val = 0;
	int ret = 0;//gpio request需要分开写

	dev_info(dev, "%s [START]\n", __func__);

	//Read property
	ret = of_property_read_u32(np, MMS_DEVICE_NAME",max_x", &val);
	if (ret) {
		dev_err(dev, "%s [ERROR] max_x\n", __func__);
		info->pdata->max_x = 1080;
	}
	else {		
		info->pdata->max_x = val;
		dev_err(dev, "%s max_x is %d\n", __func__, info->pdata->max_x);
	}

	ret = of_property_read_u32(np, MMS_DEVICE_NAME",max_y", &val);
	if (ret) {
		dev_err(dev, "%s [ERROR] max_y\n", __func__);
		info->pdata->max_y = 1920;
	}
	else {
		info->pdata->max_y = val;
		dev_err(dev, "%s max_y is %d\n", __func__, info->pdata->max_y);
	}

	//Get GPIO
	ret = of_get_named_gpio_flags(np, MMS_DEVICE_NAME",irq-gpio", 0, NULL);
	if (!gpio_is_valid(ret)) {
		dev_err(dev, "%s [ERROR] of_get_named_gpio_flags : irq-gpio\n", __func__);
		goto ERROR;
	}
	else{
		info->pdata->gpio_intr = ret;
		dev_err(dev, "%s irq-gpio is %d\n", __func__, info->pdata->gpio_intr);
	}

	ret = of_get_named_gpio_flags(np, MMS_DEVICE_NAME",reset-gpio", 0, NULL);
	if (!gpio_is_valid(ret)) {
		dev_err(dev, "%s [ERROR] of_get_named_gpio_flags : reset-gpio\n", __func__);
		goto ERROR;
	}
	else{
		info->pdata->gpio_reset = ret;
		dev_err(dev, "%s reset-gpio is %d\n", __func__, info->pdata->gpio_reset);
	}

	//Config GPIO
	ret = gpio_request(info->pdata->gpio_intr, "melfas_int");
	if (ret < 0) {
		dev_err(dev, "%s [ERROR] gpio_request gpio_int\n", __func__);
		goto ERROR;
	}
	gpio_direction_input(info->pdata->gpio_intr);

	//Set IRQ
	info->client->irq = gpio_to_irq(info->pdata->gpio_intr);
	dev_info(dev, "%s - gpio_to_irq : irq[%d]\n", __func__, info->client->irq);

	ret = gpio_request(info->pdata->gpio_reset, "melfas_rst");
   	if (ret < 0) {
       	dev_err(dev, "%s [ERROR] gpio_request gpio_reset", __func__);
       	goto error_request_rst;
   	}
    gpio_direction_output(info->pdata->gpio_reset, 1);

	dev_info(dev, "%s [DONE]\n", __func__);
	return 0;

error_request_rst:
	gpio_free(info->pdata->gpio_intr);
ERROR:
	dev_err(dev, "%s [ERROR]\n", __func__);	
	return 1;
}
#endif

/**
* Config input interface	
*/
void mms_config_input(struct mms_ts_info *info)
{
	struct input_dev *input_dev = info->input_dev;

	dev_info(&info->client->dev, "%s [START]\n", __func__);

	//Screen
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	//input_mt_init_slots(input_dev, MAX_FINGER_NUM);
	input_mt_init_slots(input_dev, MAX_FINGER_NUM, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, INPUT_TOUCH_MAJOR_MAX, 0, 0);

	//Key
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);

#if MMS_USE_NAP_MODE
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_POWER, input_dev->keybit);
#endif

	dev_info(&info->client->dev, "%s [DONE]\n", __func__);
	return;
}

#if MMS_USE_CALLBACK
/**
* Callback - get charger status
*/
void mms_callback_charger(struct mms_callbacks *cb, int charger_status)
{
	struct mms_ts_info *info = container_of(cb, struct mms_ts_info, callbacks);

	dev_info(&info->client->dev, "%s [START]\n", __func__);

	dev_info(&info->client->dev, "%s - charger_status[%d]\n", __func__, charger_status);

	//...

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/**
* Callback - add callback funtions here
*/
//...

/**
* Config callback functions
*/
void mms_config_callback(struct mms_ts_info *info)
{
	dev_info(&info->client->dev, "%s [START]\n", __func__);

	info->register_callback = info->pdata->register_callback;

	//callback functions
	info->callbacks.inform_charger = mms_callback_charger;
	//info->callbacks.inform_display = mms_callback_display;
	//...

	if (info->register_callback){
		info->register_callback(&info->callbacks);
	}

	dev_info(&info->client->dev, "%s [DONE]\n", __func__);
	return;
}
#endif
