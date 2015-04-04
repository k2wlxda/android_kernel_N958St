/*
 * BQ24158 charger driver
 *
 * Copyright (C) 2013 ZTEMT
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#define pr_fmt(fmt)	"BQ: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/wakelock.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#include <bq24158_charger.h>

#define DRIVER_VERSION			"1.0.0"

#define PM_INFO 1
#define PM_DEBUG 4
//log level < bqlog_level will show
int bqlog_level = 5;  
module_param(bqlog_level, int, 0644);

#define BQLOG_INFO(fmt, args...) \
		if (PM_INFO < bqlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)
	
#define BQLOG_DEBUG(fmt, args...) \
		if (PM_DEBUG < bqlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)

struct bq24158_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq24158_chg_chip {
	struct device	  *dev;
	struct i2c_client *i2c;
	struct power_supply	   *batt_psy;
	struct delayed_work		chg_work;
	struct pinctrl *ce_pinctrl;
	struct pinctrl_state *gpio_state_active;	
	struct pinctrl_state *gpio_state_suspend;
	struct bq24158_otg_regulator otg_vreg;
	struct device_node *dev_node;
	struct wake_lock wlock;
	struct qpnp_vadc_chip	*vadc_dev;
	enum usb_chg_type chg_type;
	int chg_en_gpio;
	int otg_gpio;
	int psel_gpio;
	int irq_gpio;

	int charge_sense;
	
	//batt status info
	int batt_status; 
	int batt_temp;
	int chg_status;
	int batt_i;
	int batt_vol;
	int batt_soc;
	
	int ibatmax_ma;
	int vusb_min;
	int iusb_init_ma;
	int iterm_ma;
	int vbatt_max;
	
	int usb_in;
	int opa_mode;
	bool in_work;
	bool temp_abnormal;
	bool soc_chg_done;
	bool in_rechging;
	bool is_hvdcp_chg;
	bool is_chg_full;
};

static struct bq24158_chg_chip   *bq_chip;
static int usbin_current = 1500;
module_param(usbin_current, int, 0644);

static int ibat_current = 1200;
module_param(ibat_current, int, 0644);
static int charger_online = 0;

static DEFINE_MUTEX(bq24158_i2c_mutex);
extern int qpnp_get_battery_temp(void);
extern int qpnp_dcdc_enable(int enable);

static int bq24158_get_temp_status(const struct batt_status_map *pts,
		uint32_t tablesize, int input, int *batt_status);
static int bq24158_chg_hw_init(struct bq24158_chg_chip *chip);

static const struct batt_status_map batt_temp_map[] = {
	{CHG_TEMP_MIN, CHG_TEMP_COLD, BATT_STATUS_COLD, 0}, 
	{CHG_TEMP_COLD, CHG_TEMP_WARM, BATT_STATUS_GOOD, 1100}, 
	{CHG_TEMP_WARM, CHG_TEMP_HOT, BATT_STATUS_WARM, 800}, 
	{CHG_TEMP_HOT, CHG_TEMP_MAX, BATT_STATUS_HOT, 0}, 
};
/******************************************************** 
 *					 I2C I/O function 				              *
 *********************************************************/

static int bq24158_i2c_readb(
		struct i2c_client *i2c,
		u8  reg,
		u8* buf)
{
	struct i2c_msg msgs[2];
	int ret = 0;

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
	
	mutex_lock(&bq24158_i2c_mutex);
	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		ret = -4;
	}
	mutex_unlock(&bq24158_i2c_mutex);
	
	pr_debug("return  buf[0]=0x%x!\n",buf[0]);

	return ret;
}

//write bq24158 i2c function
static int bq24158_i2c_writeb(
		struct i2c_client *i2c,
		u8 reg, 
		u8 buf)
{
	struct i2c_msg msgs;
	u8 data[2];
	int ret = 0;

	data[0] = reg;
	data[1] = buf;

	//write message: this is the sub address that we are reading from
	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = data;
	
	mutex_lock(&bq24158_i2c_mutex);
	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.\n", __func__);
		ret = -4;
	}
	mutex_unlock(&bq24158_i2c_mutex);

	return ret;
}
/****************************************************************/

static int  bq24158_masked_write(struct bq24158_chg_chip *chip, u8 reg,
							u8 mask, u8 val)
{
	int rc;
	u8 buf;

	rc = bq24158_i2c_readb(chip->i2c,reg,&buf);
	if (rc) {
		pr_err("bq24158_i2c_readb failed: reg=0x%x, rc=%d\n", reg, rc);
		return rc;
	}
	
	buf &= ~mask;
	buf |= val & mask;

	rc = bq24158_i2c_writeb(chip->i2c,reg,buf);
	if (rc) {
		pr_err("bq24158_i2c_writeb failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}

	return 0;
}

static int bq24158_chg_gpio_select(struct bq24158_chg_chip *chip,int enable)
{
	struct pinctrl_state *pins_state;
	int ret;
	
	BQLOG_INFO(" select gpio state: %s\n",enable? "active":"suspend");

	pins_state = enable ? chip->gpio_state_active:chip->gpio_state_suspend;
	
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(chip->ce_pinctrl, pins_state);
		if (ret) {
			pr_err(	"can not set gpio pins(%d)\n",ret );
			return ret;
		}
	} else 
		pr_err("not a valid gpio pinstate\n");
	
	return 0;
}

static int bq24158_chg_gpio_enable(struct bq24158_chg_chip *chip,int enable)
{
	int ret;
	
	BQLOG_DEBUG("set enable gpio: %d\n",enable );

	ret = gpio_direction_output(chip->chg_en_gpio,enable);
	if(ret)
		pr_err("can not set gpio %d\n",chip->chg_en_gpio);
	return 0;
}

//Reg 0x0 ---------------------------------------------------------------
static int  bq24158_reset_wdog(struct bq24158_chg_chip *chip)
{
	return bq24158_masked_write(chip, BQ24158_REG_STATUS,BQ_TMR_OTG_MASK,BIT(7));
}

int  bq24158_get_chg_status(void)
{
	int rc;
	u8  buf;

	if (!bq_chip) {
		pr_err("called before init\n");
		return BQ_READY_CHGING;
	}

	rc = bq24158_i2c_readb(bq_chip->i2c,BQ24158_REG_STATUS,&buf);
    if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	buf &= BQ_CHG_STAT_MASK;
	buf = buf >> BQ24158_STAT_SHIFT;
	
	return buf;
}

//Reg 0x1 ---------------------------------------------------------------
static int  bq24158_hiz_mode_enable(struct bq24158_chg_chip *chip,int enable)
{
	int rc;
	u8 val;

	val = enable << BQ24158_HZ_MODE_SHIFT;
	
	rc = bq24158_masked_write(chip,
		                      BQ24158_REG_CONTROL,
		                      BQ_HZ_MODE_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

int  bq24158_is_hiz_mode(struct bq24158_chg_chip *chip)
{
	int rc;
	u8  buf;

	rc = bq24158_i2c_readb(chip->i2c,BQ24158_REG_CONTROL,&buf);
    if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	buf &= BQ_HZ_MODE_MASK;
	buf = buf >> BQ24158_HZ_MODE_SHIFT;
		
	return buf;
}

static int  bq24158_set_opa_mode(struct bq24158_chg_chip *chip,enum opa_mode mode)
{
	int rc;

	rc = bq24158_masked_write(chip,
		                      BQ24158_REG_CONTROL,
		                      BQ_OPA_MODE_MASK,
		                      (u8)mode);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	chip->opa_mode = mode;
		
	return 0;
}

int  bq24158_is_boost_mode(void)
{
	if (!bq_chip) {
		pr_err("called before init\n");
		return -1;
	}

	return (bq_chip->opa_mode == OPA_BOOST);
}

static int  bq24158_get_opa_mode(struct bq24158_chg_chip *chip)
{
	int rc;
	u8 buf;

	rc = bq24158_i2c_readb(chip->i2c,BQ24158_REG_CONTROL,&buf);
	if (rc) {
		pr_err("bq24158_i2c_readb failed: reg=0x%x, rc=%d\n", 0x01, rc);
		return rc;
	}
	
	buf &=  BQ_OPA_MODE_MASK;

	return buf;
}

static int	bq24158_set_chg_iusb(struct bq24158_chg_chip *chip, int iusb_ma)
{
	int rc;
	u8 val;

	if (iusb_ma <= 100)
		val = 0;
	else if (iusb_ma <= 500)
		val = 1;
	else if (iusb_ma <= 800)
		val = 2;
	else
		val = 3;
	
	BQLOG_INFO("REG[0x01] set iusb=%d val=%d\n",iusb_ma,val);
	
	val = val << BQ24158_CURRENT_LIMIT_SHIFT;
	rc = bq24158_masked_write(chip,
		                      BQ24158_REG_CONTROL,
		                      BQ_CURRENT_LIMIT_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

int  bq24158_charge_enable(int enable)
{
	int rc;
	u8 val;

    if (!bq_chip) {
		pr_err("called before init\n");
		return -1;
	}
		
	val = (!enable) << BQ24158_CE_SHIFT;
	rc = bq24158_masked_write(bq_chip,
		                      BQ24158_REG_CONTROL,
		                      BQ_CE_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

int  bq24158_is_charge_enable(void)
{
	int rc;
	u8 buf;

    if (!bq_chip) {
		pr_err("called before init\n");
		return 0;
	}

	rc = bq24158_i2c_readb(bq_chip->i2c,BQ24158_REG_CONTROL,&buf);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}

	buf &= BQ_CE_MASK;
	buf = buf >> BQ24158_CE_SHIFT;
	
	return (!buf);
}


//Reg 0x2 ---------------------------------------------------------------
 int  bq24158_set_vbattmax(struct bq24158_chg_chip *chip, int vbatt)
{
	u8 val;

	if (vbatt < BQ24158_CHG_MIN_VBATT || vbatt > BQ24158_CHG_MAX_VBATT) {
		pr_err("bad vbattmax = %dmV asked to set\n", vbatt);
		return -EINVAL;
	}

	val = (vbatt - BQ24158_CHG_MIN_VBATT)/BQ24158_CHG_VBATT_STEP_MV;
	BQLOG_INFO("REG[0x02]set vbattmax=%d val=%d\n",vbatt,val);
	
	val = val << BQ24158_BATT_REGULAT_SHIFT;
	return bq24158_masked_write(chip, 
		                        BQ24158_REG_VOL_CNTL,
		                        BQ_BATT_REGULAT_MASK,
		                        val);
}

//Reg 0x3 ---------------------------------------------------------------
static int  bq24158_get_dev_info(struct bq24158_chg_chip *chip)
{
	int rc;
	u8 buf;

	rc = bq24158_i2c_readb(chip->i2c,BQ24158_REG_VISION,&buf);
	if (rc) {
		pr_err("%s i2c read failed: rc=%d\n", __func__, rc);
		return rc;
	}
	
	buf &= BQ_PN_MASK;
	buf = buf >> BQ24158_PN_SHIFT;
	return (int)buf;
}

//Reg 0x4 ---------------------------------------------------------------
#define BQ_RESET_BIT		BIT(7)
static int  bq24158_reset_regs(struct bq24158_chg_chip *chip)
{
    BQLOG_INFO("reset_regs\n");
	return bq24158_masked_write(chip, BQ24158_REG_CURRENT, BQ_RESET_MASK,BIT(7));
}

static int  bq24158_set_chg_ibatt(struct bq24158_chg_chip *chip, int ichg_ma)
{
	u8 val;

	if (chip->charge_sense <= 0)
		return -ENOSYS;
	
	val = (ichg_ma * chip->charge_sense - 37400) / 6800;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;
	
	BQLOG_INFO("REG[0x04] set ibatt=%d val=%d charge_sense=%d\n",ichg_ma,val,chip->charge_sense);

	val = val << BQ24158_FAST_CURRENT_SHIFT;
	return bq24158_masked_write(chip,  
		                        BQ24158_REG_CURRENT,
		                        BQ_CHG_FAST_CURRENT_MASK,
		                        val);
}

 int  bq24158_set_chg_iterm(struct bq24158_chg_chip *chip, int iterm_ma)
{
	u8 val;

	if (chip->charge_sense <= 0)
		return -ENOSYS;

    bq24158_masked_write(chip, 
		                 BQ24158_REG_CONTROL,
		                 BQ_TE_MASK,
		                 BIT(3));
	
	val = (iterm_ma * chip->charge_sense - 3400) / 3400;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;
	
	BQLOG_INFO("REG[0x04] set iterm=%d val=%d\n",iterm_ma,val);
	return bq24158_masked_write(chip, 
		                        BQ24158_REG_CURRENT,
		                        BQ_CHG_TERM_CURRENT_MASK,
		                        val);
}

//Reg 0x5 ---------------------------------------------------------------
static int  bq24158_low_chg_sense(struct bq24158_chg_chip *chip,int enable)
{
	int rc;
	u8 val;

	val = enable << BQ24158_LOW_CHG_SHIFT;
	
	rc = bq24158_masked_write(chip,
		                      BQ24158_REG_CHARGER,
		                      BQ_LOW_CHG_MASK,
		                      val);
	if (rc) {
		pr_err("%s i2c r/w failed: rc=%d\n", __func__, rc);
		return rc;
	}
	return 0;
}

static int  bq24158_special_chg_vol(struct bq24158_chg_chip *chip, int vol_mv)
{
	u8 val;

	if (vol_mv < BQ24158_SPEC_CHG_MIN_VOL || vol_mv > BQ24158_SPEC_CHG_MAX_VOL) {
		pr_err("bad vbattmax = %dmV asked to set\n", vol_mv);
		return -EINVAL;
	}

	val = (vol_mv - BQ24158_SPEC_CHG_MIN_VOL)/BQ24158_SPEC_CHG_STEP;
	BQLOG_INFO("set vchgmin[0x02]=%d\n",val);
	
	return bq24158_masked_write(chip, 
		                        BQ24158_REG_CHARGER,
		                        BQ_VSREG_MASK,
		                        val);
}

//Reg 0x6 ---------------------------------------------------------------
static int  bq24158_set_safety_limit(struct bq24158_chg_chip *chip, int ibatt_ma,int vbatt_mv)
{
	u8 val = 0;
	int temp;
	int ret;

	if (chip->charge_sense <= 0)
		return -ENOSYS;
	
	temp = (ibatt_ma * chip->charge_sense - 37400) / 6800;
	if (temp < 0)
		temp = 0;
	else if (temp > 15)
		temp = 15;
	
	val = temp << BQ24158_SAFE_CURRENT_SHIFT;

	temp = (vbatt_mv - BQ24158_SAFE_MIN_VBATT) / BQ24158_SAFE_VBATT_STEP;
	if (temp < 0)
		temp = 0;
	else if (temp > 15)
		temp = 15;

	val |= temp;

	BQLOG_INFO("set ibatt_ma=%d vbatt_mv=%d val=0x%x\n",ibatt_ma,vbatt_mv,val);

	ret = bq24158_i2c_writeb(chip->i2c,BQ24158_REG_LIMIT,val);
	if (ret) 
		pr_err("bq24158_i2c_writeb failed: reg=%03X, rc=%d\n", 6, val);
	
	return ret;
}

//----------------------------------------------------------------------
static void bq24158_dump_regs(struct bq24158_chg_chip *chip)
{
	u8 buf;
	int i;

    if (PM_DEBUG > bqlog_level)
		return;
	for(i=0;i<7;i++){
		bq24158_i2c_readb(chip->i2c,i,&buf);
		printk("reg[%d] buf=0x%x\n",i,buf);
	}
}

static void bq24158_update_power_supply(struct bq24158_chg_chip *chip)
{
	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy > 0)
		power_supply_changed(chip->batt_psy);
}

static int bq_get_batt_present(struct bq24158_chg_chip *chip)
{
	union power_supply_propval ret = {0,};
	
	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if(chip->batt_psy)
  		chip->batt_psy->get_property(chip->batt_psy,
			    POWER_SUPPLY_PROP_PRESENT, &ret);
	
	return ret.intval;
}

#define DEFAULT_BATT_TEMP		250
static int  bq_get_batt_temp(struct bq24158_chg_chip *chip)
{
	int ret = 0;
	struct qpnp_vadc_result results;
	
	if (!bq_get_batt_present(chip)){
		pr_debug("battery is not present!\n");
		return DEFAULT_BATT_TEMP;
	}

	ret = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (ret) {
		pr_info("Unable to read batt temperature rc=%d\n", ret);
		return DEFAULT_BATT_TEMP;
	}
	
	pr_debug("adc_code=%d,temp=%lld\n", results.adc_code,results.physical);

	return (int)results.physical;
}

static int bq24158_get_batt_param(struct bq24158_chg_chip *chip)
{
    union power_supply_propval ret = {0,};
	
	chip->batt_temp = bq_get_batt_temp(chip);
	chip->chg_status = bq24158_get_chg_status();

	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if(!chip->batt_psy)
		return -1;

    chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CURRENT_NOW,&ret);
	chip->batt_i = ret.intval/1000;
	
	chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_VOLTAGE_NOW,&ret);
	chip->batt_vol = ret.intval/1000;

	chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&ret);
	chip->batt_soc = ret.intval;
	
	BQLOG_DEBUG("batt_temp=%d chg_status=%d  batt_i=%d  batt_vol=%d  batt_soc=%d\n",
		chip->batt_temp,chip->chg_status,chip->batt_i,chip->batt_vol,chip->batt_soc);

	return 0;
}
//battery  power supply property
int bq_prop_batt_status(void)
{
    int batt_st = bq24158_get_chg_status();

	if(batt_st == BQ_IN_CHGING)
		return POWER_SUPPLY_STATUS_CHARGING;
	else if(batt_st == BQ_READY_CHGING)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if(batt_st == BQ_CHGING_DONE)
		return POWER_SUPPLY_STATUS_FULL;
	else
		return POWER_SUPPLY_STATUS_UNKNOWN;
}

int bq_prop_charging_type(void)
{
    int batt_st = bq24158_get_chg_status();

	if(batt_st == BQ_IN_CHGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

int bq_prop_batt_health(void)
{
    int batt_status;
	batt_status = bq24158_get_batt_stauts();

	if(batt_status == BATT_STATUS_HOT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if(batt_status == BATT_STATUS_COLD)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

//Module Interface---------------------------------------------------------------------- 
int bq24158_is_charger_online(void)
{
	if (!bq_chip) {
	    pr_err("called before init\n");
		return 0;
	}

    return bq_chip->usb_in;
}

int bq24158_get_batt_stauts(void)
{
	int batt_temp;
	int batt_status;

	if (!bq_chip) {
	    pr_err("called before init\n");
		return BATT_STATUS_GOOD;
	}

	if(!bq_chip->in_work){
		batt_temp = bq_get_batt_temp(bq_chip);
		bq24158_get_temp_status(batt_temp_map, ARRAY_SIZE(batt_temp_map), batt_temp, &batt_status);
		return batt_status;
	}

    return bq_chip->batt_status;
}

int  bq24158_notify_charger(enum usb_chg_type chg_type)
{	
    int set_iusb;
	int ret;
	
	if (!bq_chip) {
		pr_err("called before init\n");
		return -1;
	}
	
	if(chg_type == USB_DCP_CHARGER 
	|| chg_type == USB_CDP_CHARGER 
	|| chg_type == USB_FLOATED_CHARGER)
		set_iusb = BQ_AC_CHG_I;
	else
		set_iusb = BQ_USB_CHG_I;
	
	usbin_current = set_iusb;
	bq_chip->chg_type = chg_type;
	BQLOG_INFO("chg->chg_type=%d iusb=%d\n",chg_type,set_iusb);
	
	ret = bq24158_set_chg_iusb(bq_chip,set_iusb);
	if (ret) {
		pr_err("%s	failed: rc=%d\n", __func__,ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(bq24158_notify_charger);

#define BATT_TEMTP_DELTA    20
#define TEMP_INIT    -500
static int bq24158_get_temp_status(const struct batt_status_map *pts,
		uint32_t tablesize, int input, int *batt_status)
{
	static int current_index = 0;
	static int init_status = 1;

	if ( pts == NULL || batt_status == NULL)
		return BATT_STATUS_UNKNOW;
		
	if(init_status){
		while (current_index < tablesize) {
			if ( (pts[current_index].low_temp <= input) && (input <= pts[current_index].high_temp) ) 
				break;
			else 
				current_index++;
		}
		init_status = 0;
		BQLOG_DEBUG("-First-input=%d  current_index=%d \n",input,current_index);
	}else{
		if(input < (pts[current_index].low_temp - BATT_TEMTP_DELTA))
			current_index--;
		else if(input > pts[current_index].high_temp)
			current_index++;
	}

    if(current_index < 0)
		*batt_status = BATT_STATUS_COLD;
	else if(current_index >= tablesize)
		*batt_status = BATT_STATUS_HOT;
	else
		*batt_status = pts[current_index].batt_st;

	BQLOG_DEBUG("input=%d  batt_status=%d \n",input,*batt_status);
	
	return current_index;

}

static void bq24158_chg_temp_cntl(struct bq24158_chg_chip *chip,int batt_temp)
{
    int battery_status;
	int batt_current = 0;
	int state_index;

	state_index = bq24158_get_temp_status(batt_temp_map,
		                                     ARRAY_SIZE(batt_temp_map),
		                                     batt_temp,
		                                     &battery_status);
	if(battery_status != BATT_STATUS_UNKNOW)
	    batt_current = batt_temp_map[state_index].batt_current;
		
	if(battery_status != chip->batt_status && chip->usb_in){
		BQLOG_INFO("last chip->batt_status=%d new_battery_status=%d\n",chip->batt_status,battery_status);
		if(batt_current > 0){
		    bq24158_charge_enable(1);
			bq24158_set_chg_ibatt(chip,batt_current);
			chip->temp_abnormal = 0;
			
		    BQLOG_INFO("batt_temp=%d batt_status=%d batt_current=%d start charging...\n",batt_temp,battery_status,batt_current);
		}else{
			bq24158_charge_enable(0);
			chip->temp_abnormal = 1;
			
			BQLOG_INFO("batt_temp=%d out of rangge,stop charging!\n",batt_temp);
		}
		bq24158_update_power_supply(chip);
	}
	
	chip->batt_status = battery_status;

}

static void bq24158_recharging_cntl(struct bq24158_chg_chip *chip)
{
	if(chip->batt_soc == 100 && chip->batt_vol>4300 && chip->usb_in)
		chip->soc_chg_done = 1;
	else if(chip->batt_soc <= 96 || !chip->usb_in)
		chip->soc_chg_done = 0;
	
    if((chip->soc_chg_done || chip->chg_status == BQ_CHGING_DONE) ){
        if(chip->batt_vol<4300 || chip->batt_soc<=99){
			bq24158_hiz_mode_enable(chip,1);
			mdelay(3);
			bq24158_hiz_mode_enable(chip,0);
			chip->soc_chg_done = 0;
			BQLOG_INFO("batt_vol=%d  restart charging...\n",chip->batt_vol);
        }
    }
}

static void bq24158_chg_status_reset(struct bq24158_chg_chip *chip)
{	
	chip->temp_abnormal = 0;
	chip->soc_chg_done = 0;
	chip->is_chg_full = 0;
	chip->batt_status = BATT_STATUS_UNKNOW;
	chip->chg_type = USB_INVALID_CHARGER;
}

#define START_CHG_MS	1000
#define CHG_PERIOD_MS	10000
static void bq24158_chg_worker(struct work_struct *work)
{
    int ret;
	struct delayed_work *cwork = to_delayed_work(work);
	struct bq24158_chg_chip *chip = container_of(cwork,	struct bq24158_chg_chip,chg_work);

	//console_printk[0] = 8;

	bq24158_reset_wdog(chip);
	
	if(!chip->usb_in)
		goto out_work;

	if(chip->opa_mode == OPA_BOOST){
		if(wake_lock_active(&chip->wlock))
			wake_unlock(&chip->wlock);
		goto next;
	}
	
	if(!chip->in_work){
		bq24158_chg_hw_init(chip);
		chip->in_work = 1;
	}

	ret = bq24158_get_batt_param(chip);
	if(ret < 0)
		goto next;
		
	bq24158_chg_temp_cntl(chip,chip->batt_temp);

    if(!chip->temp_abnormal && 0)
	    bq24158_recharging_cntl(chip);

	if(chip->chg_status == BQ_CHGING_DONE){
		if(!chip->is_chg_full){
			BQLOG_INFO("charging is full.\n");
			bq24158_update_power_supply(chip);
			chip->is_chg_full = 1;
		}
	}else
	    chip->is_chg_full = 0;

    bq24158_dump_regs(chip);
	BQLOG_INFO("usb_in=%d  status=%d\n",chip->usb_in,chip->chg_status);
	
next:
	schedule_delayed_work(&chip->chg_work,
		round_jiffies_relative(msecs_to_jiffies(CHG_PERIOD_MS)));
	return;

out_work:
	chip->in_work = 0;
	bq24158_chg_status_reset(chip);
	bq24158_set_chg_iusb(chip,0);
	
	if(wake_lock_active(&chip->wlock))
		wake_unlock(&chip->wlock);

}
int  bq24158_set_chg_status(int usb_in)
{
	BQLOG_INFO("usb_in=%d\n",usb_in);
		
	if (!bq_chip) {
		pr_err("called before init\n");
		charger_online = usb_in;
		return -1;
	}  

    bq_chip->usb_in = usb_in;  
	
	if(usb_in){
		bq24158_chg_gpio_enable(bq_chip,1);
		if(bq_chip->opa_mode == OPA_CHARGER)
			wake_lock(&bq_chip->wlock);
		schedule_delayed_work(&bq_chip->chg_work,
			   round_jiffies_relative(msecs_to_jiffies(START_CHG_MS)));
	}else
	    bq24158_chg_gpio_enable(bq_chip,0);
	
 	return 0;
}
EXPORT_SYMBOL_GPL(bq24158_set_chg_status);

static int debug_reg;
module_param(debug_reg, int, 0644);

static int bq24158_debug;
static int bq24158_debug_mode(const char *val, struct kernel_param *kp)
{
	int ret;
    int i;
    u8 buf;
	
	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	
	printk("__%s: bq24158_debug=%d!\n",__func__,bq24158_debug);

	if(bq24158_debug < 0){
		buf = abs(bq24158_debug);
        bq24158_i2c_writeb(bq_chip->i2c,debug_reg,buf);
		return 0;
	}
		
	switch(bq24158_debug){
    case 0:
		bq24158_charge_enable(0);
		break;
	case 1:
		bq24158_charge_enable(1);
		break;
	case 2:
		bq24158_set_chg_ibatt(bq_chip,ibat_current);
		break;
	case 3:
		bq24158_get_dev_info(bq_chip);
		break;
	case 4:
		for(i=0;i<6;i++){
			bq24158_i2c_readb(bq_chip->i2c,i,&buf);
            printk("---reg[%d] buf=0x%x\n",i,buf);
		}
		break;
    case 5:
		bq24158_reset_regs(bq_chip);
		break;
	case 6:
		bq24158_set_chg_iusb(bq_chip,1000);
		break;
	case 7:
		bq24158_hiz_mode_enable(bq_chip,0);
		break;
	case 8:
		bq24158_hiz_mode_enable(bq_chip,1);
		break;
	case 10:
		bq24158_chg_gpio_enable(bq_chip,0);
		break;
	case 11:
		bq24158_chg_gpio_enable(bq_chip,1);
		break;
	default:
		break;
	};
	printk("__%s: debug end\n",__func__);
	return 0;
}
module_param_call(bq24158_debug, bq24158_debug_mode, param_get_uint,
					&bq24158_debug, 0644);

static int
bq24158_charger_read_dt_props(struct bq24158_chg_chip *chip)
{
    int rc;
	
	rc = of_property_read_u32(chip->dev_node, "bq-ibatmax-ma", &chip->ibatmax_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-ibatmax-ma'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-vusb-min", &chip->vusb_min);
	if (rc) {
		pr_err( "Unable to parse 'bq-vusb-min'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-iterm-ma", &chip->iterm_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-iterm-ma'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-initusb-ma", &chip->iusb_init_ma);
	if (rc) {
		pr_err( "Unable to parse 'bq-initusb-ma'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-vbatmax-mv", &chip->vbatt_max);
	if (rc) {
		pr_err( "Unable to parse 'bq-vbatmax-mv'\n");
		return rc;
	}

	rc = of_property_read_u32(chip->dev_node, "bq-charge-sense", &chip->charge_sense);
	if (rc) {
		pr_err( "Unable to parse 'bq-charge-sense'\n");
		return rc;
	}
	
	chip->chg_en_gpio = of_get_named_gpio_flags(chip->dev_node,"bq24158,ce-gpio", 0, NULL);

	return 0;
}

 int bq24158_gpio_init(struct bq24158_chg_chip *chip)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	chip->ce_pinctrl = devm_pinctrl_get(&(chip->i2c->dev));
	if (IS_ERR_OR_NULL(chip->ce_pinctrl)) {
		pr_err(	"Target does not use pinctrl! \n");
		retval = PTR_ERR(chip->ce_pinctrl);
		chip->ce_pinctrl = NULL;
		return retval;
	}

	chip->gpio_state_active
		= pinctrl_lookup_state(chip->ce_pinctrl,"active");
	if (IS_ERR_OR_NULL(chip->gpio_state_active)) {
		pr_err("Can not get ts default pinstate! \n");
		retval = PTR_ERR(chip->gpio_state_active);
		chip->ce_pinctrl = NULL;
		return retval;
	}

	chip->gpio_state_suspend
		= pinctrl_lookup_state(chip->ce_pinctrl,
			"suspend");
	if (IS_ERR_OR_NULL(chip->gpio_state_suspend)) {
		pr_err("Can not get ts sleep pinstate! \n");
		retval = PTR_ERR(chip->gpio_state_suspend);
		chip->ce_pinctrl = NULL;
		return retval;
	}

	pr_info("Charge CE PinCtrl Init Success! \n");
	return 0;
}
 
static int bq24158_chg_hw_init(struct bq24158_chg_chip *chip)
{
    int ret;

	ret = bq24158_low_chg_sense(chip,0);
	if (ret) {
		pr_err("failed to set low chg sense rc=%d\n", ret);
		return ret;
	}

	ret = bq24158_special_chg_vol(chip,chip->vusb_min);
	if (ret) {
		pr_err("failed to set spec chg vol rc=%d\n", ret);
		return ret;
	}
	
    ret = bq24158_set_chg_iterm(chip,chip->iterm_ma);
	if (ret) {
		pr_err("failed set charging iterm rc=%d\n", ret);
		return ret;
	}

	ret = bq24158_set_vbattmax(chip,chip->vbatt_max);
	if (ret) {
		pr_err("failed set charging vbattmax rc=%d\n", ret);
		return ret;
	}
	
	return 0;
}

static int bq24158_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct bq24158_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	BQLOG_INFO("To Enable OTG \n" );
	
	bq24158_chg_gpio_enable(chip,1);

	rc = bq24158_set_opa_mode(chip,OPA_BOOST);
	if(rc)
		pr_err( "Fail to enable vbus_otg regulator.\n");

	return rc;
}

static int bq24158_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct bq24158_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;
	
	BQLOG_INFO("To Disable OTG \n" );
	
	bq24158_chg_gpio_enable(chip,0);

	rc = bq24158_set_opa_mode(chip,OPA_CHARGER);
	if(rc)
		pr_err( "Fail to disable vbus_otg regulator.\n");
   
	return rc;
}

static int bq24158_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq24158_chg_chip *chip = rdev_get_drvdata(rdev);
	int in_boost;
	
	in_boost = (bq24158_get_opa_mode(chip) == OPA_BOOST);
	
	BQLOG_INFO("in_boost=%d\n",in_boost);
	
	return in_boost;
}

struct regulator_ops bq24158_otg_reg_ops = {
	.enable		= bq24158_otg_regulator_enable,
	.disable	=  bq24158_otg_regulator_disable,
	.is_enabled	= bq24158_otg_regulator_is_enable,
};

static int bq24158_regulator_init(struct bq24158_chg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		pr_err( "Unable to allocate memory\n");
		return -ENOMEM;
	}

	BQLOG_INFO( "regulator name : %s \n", init_data->constraints.name);
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &bq24158_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				pr_err(	"OTG reg failed, rc=%d\n", rc);
		}
	}
  
	return rc;
}

static int  bq24158_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct bq24158_chg_chip *chip;
	int ret;

	chip = kzalloc(sizeof(struct bq24158_chg_chip),	GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate bq24158_chg_chip\n");
		return -ENOMEM;
	}
	
    BQLOG_INFO("enter driver probe:\n");
	
	chip->i2c = client;
	chip->dev = &client->dev;
	chip->dev_node = client->dev.of_node;
	chip->in_work = 0;
	chip->temp_abnormal = 0;
	chip->in_rechging = 0;
	chip->batt_status = BATT_STATUS_UNKNOW;
	
	chip->vadc_dev = qpnp_get_vadc(&chip->i2c->dev, "bq24158");
	if (IS_ERR(chip->vadc_dev)) {
		ret = PTR_ERR(chip->vadc_dev);
		if (ret == -EPROBE_DEFER){
			pr_err("vadc not found - defer ret=%d\n", ret);
			return ret;
		}else
			pr_err("vadc property missing, ret=%d\n", ret);
	}
	
	bq24158_charger_read_dt_props(chip);

	//bq24158_reset_regs(chip);

	ret = bq24158_set_safety_limit(chip,chip->ibatmax_ma,chip->vbatt_max);	
	if (ret) 
		pr_err("fail to set safety limit, ret=%d\n",ret); 
	
	ret = bq24158_chg_hw_init(chip);
	if (ret) 
		pr_err("bq24158 hw init fail, ret=%d\n",ret); 

	ret =  bq24158_regulator_init(chip);
	if (ret) {
		if (ret == -EPROBE_DEFER){
			pr_err("regulator not found - defer ret=%d\n", ret);
			return ret;
		}else
			pr_err("regulator property missing, ret=%d\n", ret);
	}

	
	wake_lock_init(&chip->wlock, WAKE_LOCK_SUSPEND, "bq24158_charger");
	INIT_DELAYED_WORK(&chip->chg_work, bq24158_chg_worker);
	
	//----------------------------------------------
	ret = bq24158_gpio_init(chip);
	if (ret) {
		pr_err("failed init gpio ret=%d\n", ret);
	}
	bq24158_chg_gpio_select(chip,1);
	ret = gpio_request(chip->chg_en_gpio, "CHARGE_EN");
	if (ret) {
		pr_err("%s:Fail To Request GPIO %d (%d)\n",__func__, chip->chg_en_gpio, ret);

	}
	bq24158_chg_gpio_enable(chip,1);
	//-----------------------------------------------

	bq_chip = chip;

	ret = bq24158_hiz_mode_enable(chip,0);
	if (ret) 
		pr_err("failed disable hiz mode rc=%d\n", ret);

	ret = bq24158_set_opa_mode(chip,OPA_CHARGER);
	if (ret) 
		pr_err("failed to set opa mode rc=%d\n", ret);

	ret = bq24158_charge_enable(1);
	if (ret) 
		pr_err("failed enable charging rc=%d\n", ret);
	
	bq24158_reset_wdog(chip);
	
	if(charger_online){
		BQLOG_INFO("start chg work:\n");
		bq24158_set_chg_status(1);
	}
	
	BQLOG_INFO("enter driver probe:end\n");

    return 0;
}

static int bq24158_charger_remove(struct i2c_client *client)
{	
	kfree(bq_chip);
	bq_chip = NULL;

	return 0;
}

static int bq24158_suspend(struct i2c_client *cl, pm_message_t mesg)
{
	BQLOG_DEBUG(" suspend:\n");

	return 0;
};

static int bq24158_resume(struct i2c_client *cl)
{
	BQLOG_DEBUG(" resume:\n");

	return 0;
};

static struct of_device_id bq_24158_match_table[] = {
	{ .compatible = "ti,bq24158",},
	{}
};

static const struct i2c_device_id bq24158_id[] = {
	{ "bq24158", 1 },
	{},
};

static struct i2c_driver bq24158_charger_driver = {
	.driver = {
		.name = "bq24158",
		.of_match_table = bq_24158_match_table,
	},
	.id_table 	= bq24158_id,
	.probe 		= bq24158_charger_probe,
	.remove 	= bq24158_charger_remove,

	.suspend	= bq24158_suspend,
	.resume 	= bq24158_resume,
};

static int __init bq24158_charger_init(void)
{
	printk( "%s:enter...\n", __func__);

	return i2c_add_driver(&bq24158_charger_driver);
}

static void __exit bq24158_charger_exit(void)
{
	printk( "%s:bq24158 is exiting\n", __func__);

	i2c_del_driver(&bq24158_charger_driver);
}

module_init(bq24158_charger_init);


module_exit(bq24158_charger_exit);

MODULE_AUTHOR("ztemt-swang<wang.shuai12@zte.com.cn>");
MODULE_DESCRIPTION("bq24158 charger driver");
MODULE_LICENSE("GPL");
