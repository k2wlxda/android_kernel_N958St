/*
 * bq24296 charger driver
 *
 * Copyright (C) 2014-2016  author: <ghzhou1978@163.com.cn>
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/power/bq24296_charger.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>


//打开调试接口
/*
#undef pr_debug
#define pr_debug   pr_info*/
#undef KERN_INFO
#define KERN_INFO KERN_ERR


static int debug_mask_charge = 1;
module_param_named(debug_mask_charge, debug_mask_charge, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define DBG_CHARGE(x...) do {if (debug_mask_charge) pr_info(">>ZTE_CHG>> " x); } while (0)

static int charger_online = 0;
char * bq24296_batt_power_props[] = {
	"bq24296-battery",
};

/* Kick wd every 10 seconds */
#define WD_KICKING_PERIOD_MS	10000
#define BQ24265_CHG_I_MAX_MIN_100		100

/********************************************************\ 
	Enum Definition
\*********************************************************/
 enum power_supply_property bq24296_pm_power_props_mains[] = {
   POWER_SUPPLY_PROP_STATUS,
   	POWER_SUPPLY_PROP_CHARGE_TYPE,
   POWER_SUPPLY_PROP_CHARGING_ENABLED,
   	POWER_SUPPLY_PROP_RESTART_CHARGING,
};
/*
 Charger Type
*/
enum bq24296_charge_type {
	CHARGE_TYPE_NONE	= 0,
	CHARGE_TYPE_USB	= 1,
	CHARGE_TYPE_ADAPTER	= 2,
	CHARGE_TYPE_OTG	= 3,
};

/* Charge status register values
*/
enum bq24296_charge_status {
	CHARGE_STATE_NO_CHG	= 0,
	CHARGE_STATE_PRE_CHG	= 1,
	CHARGE_STATE_FAST_CHG	= 2,
	CHARGE_STATE_CHG_DONE	= 3,
};

enum vbus_mode {
	VBUS_CHARGER =0,
	VBUS_BOOST,
};

struct bq24296_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

/********************************************************\ 
	Struct		Definition
\*********************************************************/
struct bq24296_chg_chip {
	struct device				*dev;
	struct i2c_client	*client;
	struct pinctrl *ce_pinctrl;
	struct pinctrl_state *gpio_state_active;	
	struct pinctrl_state *gpio_state_suspend;	
	struct bq24296_otg_regulator	otg_vreg;
	struct qpnp_vadc_chip		*vadc_dev;	
	const struct bq24296_platform_data	*pdata;
	struct wake_lock wlock;
	struct delayed_work		wd_work;
	struct power_supply		bq24296_batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*batt_psy;	
	u16	 chgr_base;
	bool  wd_reinit_flag;
	struct mutex			read_write_lock;
	int				skip_writes;
	int				skip_reads;
	 int gpio_ce;
//ADC
	int vbat_channel;
	int batt_therm_channel;
//Property
	int maxinput_chg_mv;
	int maxinput_chg_ma;
	int v_cutoff_mv;
	int maxbat_chg_current_ma;
	int term_current_ma;
	int prechg_current_ma;
	int maxvdd_voltage_mv;
	int wd_reset_ms;
	int max_chgcycle_mins;
	int boostoutput_chg_mv;
	int hot_temperature_threshold;
	int cold_temperature_threshold;						
	int thermal_regulation_threshold;

//flag
	int   usb_in;
	int batt_voltage;
	int in_work;
	enum vbus_mode  vbus_status;
};
struct bq24296_chg_chip   *bq24296_chip;

//For Reg00
struct iusb_current_config {
	int			iusb_current_ma;
	u8			charge_cur_reg;
};

struct fchg_time_config {
	int			fchg_time_min;
	u8			fchg_time_reg;
};

struct wd_time_config {
	int			wd_time_sec;
	u8			wd_time_reg;
};

struct bhot_c_config {
	int			bhot_c;
	u8			bhot_reg;
};

struct treg_c_config {
	int			treg_c;
	u8			treg_reg;
};

/*********************************************************\
  Reg	Definition	              
 \*********************************************************/
#define BQ24296_REG_IS_CNTL		0x00 	/*input source control*/
#define BQ24296_REG_PO_CNTL	0x01	/*power on configuration*/
#define BQ24296_REG_CC_CNTL		0x02	/*charge current control*/
#define BQ24296_REG_PT_CNTL		0x03	/*pre-charge/termination current control*/
#define BQ24296_REG_CV_CNTL		0x04	/*charge voltage control*/
#define BQ24296_REG_CT_CNTL		0x05	/*charge termination/timer control*/
#define BQ24296_REG_BT_CNTL		0x06	/*boost voltage/thermal regulation control*/
#define BQ24296_REG_MO_CNTL	0x07	/*misc operation control*/
#define BQ24296_REG_SS_CNTL		0x08	/*system status*/
#define BQ24296_REG_NF_CNTL	0x09	/*new fault*/
#define BQ24296_REG_VP_CNTL		0x0A /*vender/part/reversion status*/

/********************************************************\
	Function Declaration 				       
\*********************************************************/
static int bq24296_chg_hw_init(struct bq24296_chg_chip *chip);
static int bq24296_get_prop_batt_present(struct bq24296_chg_chip *chip);
static int bq24296_get_prop_batt_temp(struct bq24296_chg_chip *chip);
static int bq24296_get_prop_battery_voltage_now(struct bq24296_chg_chip *chip);
static int bq24296_chg_charge_en(struct bq24296_chg_chip *chip, int enable);
static int  bq24296_is_boost_mode(void);
static int  bq24296_get_vbus_mode(struct bq24296_chg_chip *chip);
static int bq24296_set_vbus_mode(struct bq24296_chg_chip *chip, 
	   enum vbus_mode  vbus_status_temp);
/********************************************************\
			 I2C I/O function 				       
\*********************************************************/
static int 
bq24296_i2c_read(struct bq24296_chg_chip *chip,
				unsigned char  reg,unsigned char* buf)
{
	struct i2c_msg msgs[2];
  struct i2c_client *i2c = chip->client;
  	int ret;
  	
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

	if ( (ret = i2c_transfer(i2c->adapter, msgs, 2)) < 0) {
		pr_err("%s:Transfer Failed.\n", 
			__func__);
		return ret;
	}
	
	pr_debug("%s:Return Buf[0]=0x%02X.\n",
			__func__,buf[0]);
	return 0;
}

static int 
bq24296_i2c_write(struct bq24296_chg_chip *chip,
				unsigned char reg, unsigned char buf)
{
	struct i2c_msg msgs;
	char bufwr[2];
 struct i2c_client *i2c = chip->client;
  	int ret;
  	
	bufwr[0] = reg;
	bufwr[1] = buf;
	//write message: this is the sub address that we are reading from
	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = bufwr;

	if ((ret  =i2c_transfer(i2c->adapter, &msgs, 1)) < 0) {
		pr_err("%s:Transfer Failed.\n",
				__func__);
		return  ret;
	}

	return 0;
}

static int  
bq24296_masked_write(struct bq24296_chg_chip *chip, 
				u8 reg,u8 mask, u8 val)
{
	int rc;
	u8 buf;

	rc = bq24296_i2c_read(chip,reg,&buf);
	if (rc) {
		pr_err("%s:Failed Read.Reg=0x%02X,rc=%d.\n",
				__func__,reg,rc);
		return rc;
	}
	
	buf &= ~mask;
	buf |= val & mask;
	rc = bq24296_i2c_write(chip,reg,buf);
	if (rc) {
		pr_err("%s:Failed Write.Reg=%02X,rc=%d.\n",
				__func__,reg,rc);
		return rc;
	}
	
	DBG_CHARGE("%s:Write Reg 0x%02X=0x%02X.\n", 
			__func__,reg,buf);
	return 0;
}

 int bq24296_gpio_select(struct bq24296_chg_chip *chip,bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ?chip->gpio_state_active: chip->gpio_state_suspend;
	
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(chip->ce_pinctrl, pins_state);
		if (ret) {
			DBG_CHARGE(	"can not set %s pins(%d)\n",
				             "gpio_state_default",ret );
			return ret;
		}
	} else {
		DBG_CHARGE("not a valid '%s' pinstate\n",
				          "gpio_state_default" );
	}

  DBG_CHARGE("Charge CE PinCtrl Select  Successfully! \n ");
	return 0;
}

 int bq24296_gpio_init(struct bq24296_chg_chip *chip)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	chip->ce_pinctrl = devm_pinctrl_get(&(chip->client->dev));
	if (IS_ERR_OR_NULL(chip->ce_pinctrl)) {
		DBG_CHARGE(	"Target does not use pinctrl! \n");
		retval = PTR_ERR(chip->ce_pinctrl);
		chip->ce_pinctrl = NULL;
		return retval;
	}

	chip->gpio_state_active
		= pinctrl_lookup_state(chip->ce_pinctrl,"active");
	if (IS_ERR_OR_NULL(chip->gpio_state_active)) {
		DBG_CHARGE("Can not get ts default pinstate! \n");
		retval = PTR_ERR(chip->gpio_state_active);
		chip->ce_pinctrl = NULL;
		return retval;
	}

	chip->gpio_state_suspend
		= pinctrl_lookup_state(chip->ce_pinctrl,
			"suspend");
	if (IS_ERR_OR_NULL(chip->gpio_state_suspend)) {
		DBG_CHARGE("Can not get ts sleep pinstate! \n");
		retval = PTR_ERR(chip->gpio_state_suspend);
		chip->ce_pinctrl = NULL;
		return retval;
	}

	DBG_CHARGE("Charge CE PinCtrl Init Success! \n");
	return 0;
}

/****************************************************************\
 REG 
\****************************************************************/
//REG 0x00
//Disable The HIZ State
#define HIZ_ENABLE		BIT(7)
#define HIZ_ENABLE_MASK		0x80
//Charge Voltage Setting
#define IS_VOLTAGE_MIN_MV		3880
#define IS_VOLTAGE_MAX_MV		5080
#define IS_VOLTAGE_STEP_MV		80
#define IS_VOLTAGE_MASK		0x78
#define IS_VOLTAGE_SHIFT		3
static int
bq24296_chg_vusbmax_set(struct bq24296_chg_chip *chip,
				int voltage_limit_mv)
{
	int setpoint = 0;
	u8 ret = 0;
	int rc = 0;
  
	rc = bq24296_i2c_read(chip, BQ24296_REG_IS_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read.Reg=0x%02X,Rc=%d.\n",
					__func__,BQ24296_REG_IS_CNTL,rc);
			return rc;
	}

/* EN_HIZ Disable
0:Disable.
1:Enable.
*/
//Disable
  ret &= ~HIZ_ENABLE;
/*Input Voltage Limit
*/
	if (voltage_limit_mv < IS_VOLTAGE_MIN_MV ||
	    voltage_limit_mv > IS_VOLTAGE_MAX_MV) {
		pr_err("%s:Voltage Out Of Bounds(%dmV).\n",
				__func__,voltage_limit_mv);
		return -EINVAL;
	}
	setpoint = (voltage_limit_mv - IS_VOLTAGE_MIN_MV)/IS_VOLTAGE_STEP_MV;
	ret &= ~IS_VOLTAGE_MASK;
	ret |= setpoint << IS_VOLTAGE_SHIFT;
	rc = bq24296_i2c_write(chip,BQ24296_REG_IS_CNTL,ret);
	if (!rc)
		pr_debug("%s:Write Reg 0x%02X=0x%02X,Voltage=%dmV.\n",
				__func__,BQ24296_REG_IS_CNTL,ret,
		   setpoint * IS_VOLTAGE_STEP_MV + IS_VOLTAGE_MIN_MV);
	
	return rc;
}

//Charge Current Setting
static struct iusb_current_config iusb_current_ma[] = {
	{ 100, 0x00},
	{ 150, 0x01},
	{500, 0x02},
	{900, 0x03},
	{1000, 0x04},
	{1500, 0x05},
	{2000, 0x06},
	{3000, 0x07},
};
#define IS_CURRENT_MASK	0x07
#define IS_CURRENT_SHIFT	0
static int
bq24296_chg_iusbmax_set(struct bq24296_chg_chip *chip,
				int current_limit_ma)
{
	int i_current_limit_ma = 0;
	u8 val = 0, ret  = 0 ;
	int rc = 0;
	int i;

	rc = bq24296_i2c_read(chip, BQ24296_REG_IS_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read.Reg=0x%02X,Rc=%d.\n",
					__func__,BQ24296_REG_IS_CNTL,rc);
			return rc;
	}
	
	for (i = ARRAY_SIZE(iusb_current_ma) - 1; i >= 0; i--) {
		if (current_limit_ma >= iusb_current_ma[i].iusb_current_ma) {
			val = iusb_current_ma[i].charge_cur_reg;
			i_current_limit_ma= iusb_current_ma[i].iusb_current_ma;
			break;
		}
	}
	if (i_current_limit_ma) {
		ret &= ~IS_CURRENT_MASK;
		ret |= val << IS_CURRENT_SHIFT;
		rc = bq24296_i2c_write(chip,BQ24296_REG_IS_CNTL,ret);
		if (!rc){
			DBG_CHARGE("%s:Write Reg 0x%02X=0x%02X,Current=%dmA.\n",
								__func__,BQ24296_REG_IS_CNTL,ret,	i_current_limit_ma);
			}
	} else {
		DBG_CHARGE("%s:Write Reg 0x%02X=0x%02X,Current Out Of Bounds(%dmA).\n",
				__func__,BQ24296_REG_IS_CNTL, ret,
				current_limit_ma);
		return -EINVAL;
	}
	
  DBG_CHARGE("i_current_limit_ma = %d, val = 0x%02X \n" , i_current_limit_ma, ret);
	return rc;
}

//REG 0x01
//Boost Limitation
#define BOOST_LIM_CONFIG	BIT(0)
//SysMin Voltage Setting
#define SYSMIN_VOLTAGE_MIN_MV		3000
#define SYSMIN_VOLTAGE_MAX_MV	3700
#define SYSMIN_VOLTAGE_STEP_MV		100
#define SYSMIN_VOLTAGE_MASK		0x0E
#define SYSMIN_VOLTAGE_SHIFT		1
//Charge Enable
#define CHG_ENABLE		BIT(4)
#define CHG_ENABLE_MASK		0x10
//Otg Enable
#define OTG_ENABLE		BIT(5)
//I2C Reset
#define WD_TIMER_RESET_CONFIG		BIT(6)
#define WD_TIMER_RESET_MASK  0x40
//Reg Reset
#define REG_RESET_CONFIG		BIT(7)
static int
bq24296_chg_power_on_set(struct bq24296_chg_chip *chip,
				int sysmin_voltage_limit_mv)
{
	int setpoint = 0;
	u8 ret = 0;
	int rc = 0;
	rc = bq24296_i2c_read(chip,BQ24296_REG_PO_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read.Reg=0x%02X,Rc=%d.\n",
					__func__,BQ24296_REG_PO_CNTL,rc );
			return rc;
	}

	if (sysmin_voltage_limit_mv < SYSMIN_VOLTAGE_MIN_MV ||
	    sysmin_voltage_limit_mv > SYSMIN_VOLTAGE_MAX_MV) {
		pr_err("%s:SysMin Voltage Out Of Bounds(%dmV).\n",
				__func__,sysmin_voltage_limit_mv);
		return -EINVAL;
	}
	setpoint = (sysmin_voltage_limit_mv - SYSMIN_VOLTAGE_MIN_MV)/SYSMIN_VOLTAGE_STEP_MV;
	ret &= ~SYSMIN_VOLTAGE_MASK;
	ret |= setpoint << SYSMIN_VOLTAGE_SHIFT;
	/*Boost Limitation
	 0:1A
	 1:1.5A
	*/
	ret  &= ~BOOST_LIM_CONFIG;	
	//Enable Charging
	ret |= CHG_ENABLE;
	//Disable OTG
	ret &= ~OTG_ENABLE;
	//I2C WD Reset
	ret &= ~ WD_TIMER_RESET_CONFIG;
	//Reg Reset-- Keep The Current Reg Settting
	ret &= ~REG_RESET_CONFIG;
	rc = bq24296_i2c_write(chip,BQ24296_REG_PO_CNTL,ret);
	if (!rc){
		pr_debug("%s:Write Reg 0x%02X=0x%02X,SysMin Voltage=%dmV.\n",
				__func__,BQ24296_REG_PO_CNTL,ret,
				setpoint * SYSMIN_VOLTAGE_STEP_MV + SYSMIN_VOLTAGE_MIN_MV);
		chip->vbus_status = VBUS_CHARGER;
	}

	/*wait 20us*/
	udelay(20);
		
	return rc;
}

//REG 0x02
//Fast Charge Current Setting
#define CC_CURRENT_MIN_MA		512
#define CC_CURRENT_MAX_MA		3008
#define CC_CURRENT_STEP_MA		64
#define CC_CURRENT_MASK		0xFC
#define CC_CURRENT_SHIFT		2
//BCold Enable
#define BCOLD_CONFIG  	BIT(1)
//Force_20PCT Enable
#define FORCE_20PCT_CONFIG  	BIT(0)
static int
bq24296_chg_ibatmax_set(struct bq24296_chg_chip *chip, 
				int chg_current_ma,int cold_temperature_threshold)
{
	int setpoint = 0;
	u8 ret = 0;
	int rc = 0;

	rc = bq24296_i2c_read(chip, BQ24296_REG_CC_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read,Reg=0x%02X,Rc=%d.\n",
					__func__, BQ24296_REG_CC_CNTL,rc );
			return rc;
	}

	if (chg_current_ma < CC_CURRENT_MIN_MA ||
	    chg_current_ma > CC_CURRENT_MAX_MA) {
		pr_err("%s:Current Out Of Bounds(%dmA).\n",
				__func__,chg_current_ma);
		return -EINVAL;
	}
	setpoint = (chg_current_ma - CC_CURRENT_MIN_MA)/CC_CURRENT_STEP_MA;
	ret &= ~CC_CURRENT_MASK;
	ret |= setpoint << CC_CURRENT_SHIFT;
	
	/*BCold Config
	 0: -10'C
	 1: -20'C
	*/
	if(cold_temperature_threshold)
    	ret |= BCOLD_CONFIG;
	else
    	ret &= ~BCOLD_CONFIG;
  /*Force_20PCT Config
  0: ichg & ipre-chg
  1:20%ichg & 50%ipre-chg 
  */
  ret &= ~FORCE_20PCT_CONFIG;
	rc = bq24296_i2c_write(chip, BQ24296_REG_CC_CNTL, ret);
	if (!rc)
		pr_debug("%s:Write Reg 0x%02X=0x%02X,Current=%dmA.\n",
				__func__,BQ24296_REG_CC_CNTL, ret,
		   setpoint * CC_CURRENT_STEP_MA + CC_CURRENT_MIN_MA);

	return rc;
}

//REG 0x03
//Pre-Charge Current Limit
#define PRE_CURRENT_MIN_MA		128
#define PRE_CURRENT_MAX_MA		2048
#define PRE_CURRENT_STEP_MA		128
#define PRE_CURRENT_MASK		0xF0
#define PRE_CURRENT_SHIFT		4
//Termination Current Limit
#define TERM_CURRENT_MIN_MA		128
#define TERM_CURRENT_MAX_MA		1024
#define TERM_CURRENT_STEP_MA		128
#define TERM_CURRENT_MASK		0x0F
#define TERM_CURRENT_SHIFT		0
static int
bq24296_chg_iprechg_and_iterm_set(struct bq24296_chg_chip *chip, 
				int prechg_current_ma,int term_current_ma)
{
	int pre_setpoint = 0, term_setpoint = 0;
	u8 ret = 0;
	int rc = 0;

	rc = bq24296_i2c_read(chip, BQ24296_REG_PT_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read:Reg=0x%02X,Rc=%d.\n",
					__func__,BQ24296_REG_PT_CNTL,rc );
			return rc;
	}

	if (prechg_current_ma < PRE_CURRENT_MIN_MA ||
	    prechg_current_ma > PRE_CURRENT_MAX_MA) {
		 pr_err("%s:Pre-charge Current Out Of Bounds(%dmA).\n",
		 	  __func__,prechg_current_ma);
		return -EINVAL;
	}

	if (term_current_ma < TERM_CURRENT_MIN_MA ||
	    term_current_ma > TERM_CURRENT_MAX_MA) {
		pr_err("%s:Termination Current Out Of Bounds(%dmA).\n",
			  __func__, term_current_ma);
		return -EINVAL;
	}
	
	pre_setpoint = (prechg_current_ma - PRE_CURRENT_MIN_MA)/PRE_CURRENT_STEP_MA;
	ret &= ~PRE_CURRENT_MASK;
	ret |= pre_setpoint << PRE_CURRENT_SHIFT;
	term_setpoint = (term_current_ma - TERM_CURRENT_MIN_MA)/TERM_CURRENT_STEP_MA;
	ret &= ~TERM_CURRENT_MASK;
	ret |= term_setpoint << TERM_CURRENT_SHIFT;
	rc = bq24296_i2c_write(chip,BQ24296_REG_PT_CNTL,ret);
	if (!rc)
		pr_debug("%s:Write Reg 0x%02X=0x%02X,Pre-Charge Current=%dmA,Term-Charge Current=%dmA.\n", 
						__func__, BQ24296_REG_PT_CNTL,ret,
						pre_setpoint * PRE_CURRENT_STEP_MA + PRE_CURRENT_MIN_MA,
		       term_setpoint * TERM_CURRENT_STEP_MA + TERM_CURRENT_MIN_MA);

	return rc;
}

//REG 0x04
//vdd Max
#define CV_VOLTAGE_MIN_MV		3504
#define CV_VOLTAGE_MAX_MV		4400
#define CV_VOLTAGE_STEP_MV		16
#define CV_VOLTAGE_MASK		0xFC
#define CV_VOLTAGE_SHIFT		2
//Pre-Charge To Fast Charge Voltage
#define BATLOWV_CONFIG BIT(1)
//ReCharge Voltage Setting
#define VRECHG_CONFIG  BIT(0)
static int
bq24296_chg_vddmax_and_trim_set(struct bq24296_chg_chip *chip,
				int bat_voltage_mv, int trim_mv)
{
	int setpoint = 0;
	u8 ret = 0;
	int rc = 0;

	rc = bq24296_i2c_read(chip,BQ24296_REG_CV_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read:Reg=0x%02X,Rc=%d.\n",
				  __func__,BQ24296_REG_CV_CNTL,rc );
			return rc;
	}

	if (bat_voltage_mv < CV_VOLTAGE_MIN_MV ||
	    bat_voltage_mv > CV_VOLTAGE_MAX_MV) {
		pr_err("%s:Voltage Out Of Bounds(%dmV).\n",
			  __func__,bat_voltage_mv);
		return -EINVAL;
	}

	setpoint = (bat_voltage_mv - CV_VOLTAGE_MIN_MV)/CV_VOLTAGE_STEP_MV;
	ret &= ~CV_VOLTAGE_MASK;
	ret |= setpoint << CV_VOLTAGE_SHIFT;
	/*
	 Pre-Charge To Fast Charge Setting.
	 0:2800mV.
	 1:3000mV.
	 */
	ret |= BATLOWV_CONFIG;
/*
  ReCharge Current Setting.
  0:100mA.
  1:300mA.
  */
	ret &= ~VRECHG_CONFIG;
	rc = bq24296_i2c_write(chip, BQ24296_REG_CV_CNTL, ret);
	if (!rc)
		pr_debug("%s:Write Reg 0x%02X=0x%02X,vdd Voltage=%dmV.\n",
       __func__,BQ24296_REG_CV_CNTL,ret, 
       (setpoint * CV_VOLTAGE_STEP_MV + CV_VOLTAGE_MIN_MV));

	return rc;
}

//REG 0x05
//Enable Charge Termination 
#define CHG_TERM_ENABLE  BIT(7) 
//WD Timer Setting
#define WD_TIME_DEFAULT 40
static struct wd_time_config wd_time_sec[] = {
	{ 0, 0x00}, //disable timer
	{ 40, 0x01},
	{ 80, 0x02},
	{ 160, 0x03},
};
#define CHG_WD_TIMER_CONFIG_MASK 0x30
#define CHG_WD_TIMER_CONFIG_SHIFT  4
//Enable Safety Charge Timer
#define CHG_SAFETY_TIMER_ENABLE  BIT(3)
//Fast Charge Timer Setting
#define FCHG_TIME_DEFAULT 12*60
static struct fchg_time_config fchg_time_min[] = {
	{ 5*60, 0x00},
	{ 8*60, 0x01},
	{12*60, 0x02},
	{20*60, 0x03},
};
#define CHG_FCHG_TIMER_CONFIG_MASK 0x06
#define CHG_FCHG_TIMER_CONFIG_SHIFT  1
static int
bq24296_chg_wd_fchg_timer_set(struct bq24296_chg_chip *chip,
				int wd_time_limit_sec, int fchg_time_limit_min)
{
	u8 ret = 0;
	u8 val = 0;
	int rc = 0;
	int s_wd_time_limit=0;
	int m_fchg_time_limit =0  ;
  int i ;
  int wd_val = 0;
	rc = bq24296_i2c_read(chip, BQ24296_REG_CT_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read:Reg=0x%02X,Rc=%d.\n",
				  __func__,BQ24296_REG_CT_CNTL,rc );
			return rc;
	}

  /*Charging Termination Enable
  0:Disable.
  1:Enable.
  */
   ret |= CHG_TERM_ENABLE;
  /*I2C Watchdog Timer Setting*/
   for (i = ARRAY_SIZE(wd_time_sec) - 1; i >= 0; i--) {
			if (wd_time_limit_sec >= wd_time_sec[i].wd_time_sec) {
				wd_val = val = wd_time_sec[i].wd_time_reg;
				s_wd_time_limit= wd_time_sec[i].wd_time_sec;
				break;
		}
	}
  if(wd_time_limit_sec == 0){
  		  wd_val = val = 0;
				s_wd_time_limit= 0 ;
  	}

	if (s_wd_time_limit ||
             (wd_time_limit_sec == 0)) {
		ret &= ~CHG_WD_TIMER_CONFIG_MASK;
		ret |= val << CHG_WD_TIMER_CONFIG_SHIFT;
	} else {
		pr_err("%s:WD Timer Disalbe(%dSec).\n",
			__func__,wd_time_limit_sec);
		//return -EINVAL;
	}
/*Safety Timer Enable*/
    ret |= CHG_SAFETY_TIMER_ENABLE;
/*Fast Charging Timer Setting*/
  val = 0;
	for (i = ARRAY_SIZE(fchg_time_min) - 1; i >= 0; i--) {
		if (fchg_time_limit_min >= fchg_time_min[i].fchg_time_min) {
			val = fchg_time_min[i].fchg_time_reg;
			m_fchg_time_limit= fchg_time_min[i].fchg_time_min;
			break;
		}
	}
	if (m_fchg_time_limit) {
		ret &= ~CHG_FCHG_TIMER_CONFIG_MASK;
		ret |= val << CHG_FCHG_TIMER_CONFIG_SHIFT;
	} else {
		pr_err("%s:Fchg Timer Out Of Bounds(%dMin).\n",
			  __func__,fchg_time_limit_min);
		return -EINVAL;
	}
//
		ret &= ~CHG_WD_TIMER_CONFIG_MASK;
		ret |= 0 << CHG_WD_TIMER_CONFIG_SHIFT;
	rc = bq24296_i2c_write(chip, BQ24296_REG_CT_CNTL, ret);
		if (!rc)
		  pr_debug("%s:Write Reg 0x%02X=0x%02X,WD Timer=%dSec,Fchg Timer=%dMin.\n",
	        __func__,BQ24296_REG_CT_CNTL, ret, 
	        s_wd_time_limit,m_fchg_time_limit  );

			ret &= ~CHG_WD_TIMER_CONFIG_MASK;
		ret |= wd_val << CHG_WD_TIMER_CONFIG_SHIFT;
	rc = bq24296_i2c_write(chip, BQ24296_REG_CT_CNTL, ret);
	if (!rc)
		  pr_debug("%s:Write Reg 0x%02X=0x%02X,WD Timer=%dSec,Fchg Timer=%dMin.\n",
	        __func__,BQ24296_REG_CT_CNTL, ret, 
	        s_wd_time_limit,m_fchg_time_limit  );
//
	return rc;
}

//REG 0x06
//Boost Voltage
#define BOOST_VOLTAGE_MIN_MV		4550
#define BOOST_VOLTAGE_MAX_MV		5510
#define BOOST_VOLTAGE_STEP_MV		64
#define BOOST_VOLTAGE_MASK		0xF0
#define BOOST_VOLTAGE_SHIFT		4
//BHot Thershold
#define BHOT_MASK		0x0C
#define BHOT_SHIFT		2
static struct bhot_c_config bhot_c[] = {
	{ 0, 0x03}, //Disable
	{ 55, 0x00},
	{ 60, 0x01},
	{ 65, 0x02},
};
//Thermal Regulation Threshold
#define TREG_MASK		0x03
#define TREG_SHIFT		0
static struct treg_c_config treg_c[] = {
	{ 60, 0x00},
	{ 80, 0x01},
	{ 100, 0x02},
	{ 120, 0x03},
};
static int
bq24296_chg_boost_thermal_set(struct bq24296_chg_chip *chip,
				int boost_voltage_limit_mv,int bhot_limit_c,int treg_limit_c)
{
	u8 ret = 0;
	u8 val = 0;
	int rc = 0;
  int setpoint = 0;
  int c_bhot_limit = 0;
  int c_treg_limit = 0;
  int i ;
  
	rc = bq24296_i2c_read(chip, BQ24296_REG_BT_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read:Reg=0x%02X,Rc=%d.\n",
				  __func__,BQ24296_REG_BT_CNTL,rc );
			return rc;
	}

	if (boost_voltage_limit_mv < BOOST_VOLTAGE_MIN_MV ||
	    boost_voltage_limit_mv > BOOST_VOLTAGE_MAX_MV) {
		pr_err("%s:Boost Voltage Out Of Bounds(%dmV).\n",
			  __func__, boost_voltage_limit_mv);
		return -EINVAL;
	}

	setpoint = (boost_voltage_limit_mv - BOOST_VOLTAGE_MIN_MV)/BOOST_VOLTAGE_STEP_MV;
	ret &= ~BOOST_VOLTAGE_MASK;
	ret |= setpoint << BOOST_VOLTAGE_SHIFT;
	  /*BHOT
	  */
   for (i = ARRAY_SIZE(bhot_c) - 1; i >= 0; i--) {
			if (bhot_limit_c >= bhot_c[i].bhot_c) {
				val = bhot_c[i].bhot_reg;
				c_bhot_limit= bhot_c[i].bhot_c;
				break;
		}
	}
	if (bhot_limit_c) {
		ret &= ~BHOT_MASK;
		ret |= val << BHOT_SHIFT;
	} else {
		pr_debug("%s:Disable Boost Mode Thermal Protection(%d'C)!\n",
			__func__,bhot_limit_c);
	}
 /*TREG
 */
   for (i = ARRAY_SIZE(treg_c) - 1; i >= 0; i--) {
			if (treg_limit_c >= treg_c[i].treg_c) {
				val = treg_c[i].treg_reg;
				c_treg_limit= treg_c[i].treg_c;
				break;
		}
	}
	if (treg_limit_c) {
		ret &= ~TREG_MASK;
		ret |= val << TREG_SHIFT;
	} else {
		pr_err("%s:TReg Out Of Bounds(%d'C).\n",
			__func__,treg_limit_c);
		return -EINVAL;
	}
//
	rc = bq24296_i2c_write(chip,BQ24296_REG_BT_CNTL,ret);
	if (!rc)
		pr_debug("%s:Write Reg 0x%02X=0x%02X,Boost Voltage=%dmV BHot=%d'C,Thermal=%d'C .\n", 
				__func__, BQ24296_REG_BT_CNTL,ret,
				setpoint * BOOST_VOLTAGE_STEP_MV + BOOST_VOLTAGE_MIN_MV,
				c_bhot_limit,c_treg_limit);

	return rc;

}

//REG 0x07
#define DPDM_ENABLE  BIT(7)
#define TMR2X_ENABLE  BIT(6)
#define CHGR_ON_USB_FORCE_BIT  BIT(5)
#define CHGR_FAULT_INT_CONFIG  BIT(1)
#define BAT_FAULT_INT_CONFIG  BIT(0)
static int
bq24296_chg_misc_operation_set(struct bq24296_chg_chip *chip)
{
	u8 ret = 0;
	int rc = 0;

	rc = bq24296_i2c_read(chip, BQ24296_REG_MO_CNTL,&ret);
	if (rc )
	{
			pr_err("%s:Failed Read:Reg=0x%02X,Rc=%d.\n",
				  __func__,BQ24296_REG_MO_CNTL,rc );
			return rc;
	}

	   /* DPDM
	     0:
	     1: Force D+/D-Detection When Vbus Is Present.
	     禁用BQ24296  D+/D- 检测
	   */
	  //ret |= DPDM_ENABLE;
	  ret &= ~DPDM_ENABLE; 
	   /*TMR2X
	   0:
	   1: Safety Timer Slowed By 2X During Input DPM Or Thermal Regulation.
	   */
	 ret &= ~TMR2X_ENABLE;
	   /* BATFET
	   0:Allow Batfet Turn on.
	   1:Turn Off Batfet.
	   */
  ret &= ~CHGR_ON_USB_FORCE_BIT;
	   /* INI_MASK CHGR_FAULT
	   0:No INI During Chgr Fault.
	   1:
	   */
	 ret |= CHGR_FAULT_INT_CONFIG;
	   /* INI_MASK BAT_FAULT
	   0:No INI During Bat Fault.
	   1:
	   */
	 	ret |= BAT_FAULT_INT_CONFIG;
		rc = bq24296_i2c_write(chip, BQ24296_REG_MO_CNTL, ret);
		if (!rc)
			  pr_debug("%s:Write Reg 0x%02X=0x%02X.\n",
		        __func__,BQ24296_REG_MO_CNTL, ret);
		//
		return rc;
	   
}

//REG 0x08

//REG 0x09
#define CHRG_FAULT_MASK  0x80
#define CHRG_FAULT_STEP  7
static int
bq24296_get_is_wd_expired(struct bq24296_chg_chip *chip)
{
  u8 wd_expired;
  int rc;
  
	rc = bq24296_i2c_read(chip, BQ24296_REG_NF_CNTL,&wd_expired);
	if (rc )
	{
		pr_err("%s:Failed Read:Reg=0x%02X,Rc=%d.\n",
				__func__,BQ24296_REG_NF_CNTL,rc );
		return rc;
	}

			pr_debug("%s: Read:Reg=0x%02X . value = %d \n",
				__func__,BQ24296_REG_NF_CNTL , wd_expired );
			
	return (wd_expired&CHRG_FAULT_MASK ? 1:0);	
}
//REG 0x0A
#define VERSION_INFO_MASK 0xE0
#define VERSION_INFO_SHIFT 5
static int
bq24296_chg_get_version_info(struct bq24296_chg_chip *chip)
{
	int rc;
	u8 version_info;
	rc = bq24296_i2c_read(chip, BQ24296_REG_VP_CNTL,&version_info);
	if (rc )
	{
		pr_err("%s:Failed Read:Reg=0x%02X,Rc=%d.\n",
													__func__,BQ24296_REG_VP_CNTL,rc );
		return rc;
	}
	return ((version_info&VERSION_INFO_MASK)>>VERSION_INFO_SHIFT);
}
EXPORT_SYMBOL(bq24296_chg_get_version_info);

/****************************************************************\
 Base Function 
\****************************************************************/
#define DEFAULT_TEMP		250
static int bq24296_get_prop_batt_temp(struct bq24296_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (!bq24296_get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	
	pr_debug("get_bat_temp %d, %lld\n", results.adc_code,
							results.physical);

	return (int)results.physical;
}

static int bq24296_get_prop_battery_voltage_now(struct bq24296_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}

	return results.physical;
}

static int bq24296_get_prop_batt_present(struct bq24296_chg_chip *chip)
{
	union power_supply_propval ret = {0,};
  if(!chip->batt_psy)
  		chip->batt_psy = power_supply_get_by_name("battery");

   if(chip->batt_psy){
  		chip->batt_psy->get_property(chip->batt_psy,
			    POWER_SUPPLY_PROP_PRESENT, &ret);
		return ret.intval ;
}
   else
   	 return 1;
}

int bq24296_is_charger_online(void)
{
	if (!bq24296_chip) {
	  pr_err("%s:called before init\n",__func__);
		return 0;
	}
    return bq24296_chip->usb_in;
}

int bq24296_is_usb_present(struct bq24296_chg_chip *chip)
{
	union power_supply_propval ret = {0,};
	if (!chip->usb_psy)
					chip->usb_psy = power_supply_get_by_name("usb");

	if(chip->usb_psy)
	  	chip->usb_psy->get_property(chip->usb_psy,
			  POWER_SUPPLY_PROP_ONLINE, &ret);

	 return ret.intval || bq24296_is_charger_online();
}

#ifdef CONFIG_ZTEMT_BATTERY_MONITOR
extern bool
is_chg_batt_temp_abnormal(void);
#endif
int
bq24296_chg_wd_reset(struct bq24296_chg_chip *chip, int wd_reset)
{
	int rc = 0;
  
	if(chip->vbus_status == VBUS_BOOST){
		if(bq24296_get_vbus_mode(chip) != VBUS_BOOST)
			bq24296_set_vbus_mode(chip,VBUS_BOOST);
	}else {
  if( bq24296_get_is_wd_expired(chip))
  	chip->wd_reinit_flag = true;
  
			if(chip->wd_reinit_flag){
				rc = bq24296_chg_hw_init(chip);
			#ifdef CONFIG_ZTEMT_BATTERY_MONITOR
				if(is_chg_batt_temp_abnormal()){
				   bq24296_chg_charge_en(chip,0);
					}/*else{
							bq24296_chg_charge_en(chip,1)
						}*/
			#endif
				chip->wd_reinit_flag = false;
				DBG_CHARGE("WD Reset! \n");
			}
		}
		
	rc = bq24296_masked_write(chip,chip->chgr_base + BQ24296_REG_PO_CNTL,
									WD_TIMER_RESET_MASK, 
									wd_reset?WD_TIMER_RESET_CONFIG:0);
	if (!rc)
	{
   	udelay(20);
	  DBG_CHARGE("Success Setting WD Reset Timer rc=%d\n", rc);
	}
	else{
		DBG_CHARGE("Failed Setting WD Reset Timer rc=%d\n", rc);
	}
	
	return rc ;
}
EXPORT_SYMBOL(bq24296_chg_wd_reset);

/*Batfet Disable 后, 电池将不能充放电*/
static int
bq24296_chg_batfet_disable(struct bq24296_chg_chip *chip, int disable)
{
	/* This bit forces the charger to run off of the battery rather
	 * than a connected charger */
		return bq24296_masked_write(chip, chip->chgr_base + BQ24296_REG_MO_CNTL,
																																CHGR_ON_USB_FORCE_BIT,
																																disable ? CHGR_ON_USB_FORCE_BIT:0);
}
EXPORT_SYMBOL(bq24296_chg_batfet_disable);


static int
bq24296_chg_charge_en(struct bq24296_chg_chip *chip, int enable)
{
	int rc;
	#if 1
		rc = bq24296_masked_write(chip,chip->chgr_base + BQ24296_REG_PO_CNTL,
									CHG_ENABLE_MASK, 
									enable ? CHG_ENABLE:0);
	#else
	rc = bq24296_masked_write(chip,chip->chgr_base + BQ24296_REG_IS_CNTL,
									HIZ_ENABLE_MASK, 
									enable ? HIZ_ENABLE:0);
	#endif
	if (rc){
		DBG_CHARGE("%s  i2c r/w failed: rc=%d\n", __func__, rc);
	}
	else{
		if(enable)
				chip->vbus_status = VBUS_CHARGER;
     DBG_CHARGE("%s  i2c r/w : rc=%d, enable = %d \n", __func__, rc, enable);
	}
	return rc;
}

static int 
bq24296_recharging_cntl(struct bq24296_chg_chip *chip)
{
	int rc;
		rc = bq24296_masked_write(chip,chip->chgr_base + BQ24296_REG_IS_CNTL,
									HIZ_ENABLE_MASK, 
									HIZ_ENABLE);
		
	mdelay(3);
	
		rc = bq24296_masked_write(chip,chip->chgr_base + BQ24296_REG_IS_CNTL,
									HIZ_ENABLE_MASK, 
									0);
		
	DBG_CHARGE("batt_vol=%d  restart charging...\n",chip->batt_voltage);
	return rc;
}

/********************************************\
\*********************************************/
#define CHARGE_CAPACITY_FULL 100
#define CHG_ON_STATUS_MASK 0x30
#define CHG_ON_STATUS_SHIFT 4
int bq24296_get_prop_batt_status(void)
{
	int rc;
	u8 chgr_sts;
 enum bq24296_charge_status charging_status;
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	union power_supply_propval ret = {0,};
	union power_supply_propval ret_capacity = {0,};
	int boost_mode ;

  if(!bq24296_chip)
  	return POWER_SUPPLY_STATUS_UNKNOWN;
  
  //No Battery
	if (!bq24296_get_prop_batt_present(bq24296_chip))
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

 //No Charger
  if(!bq24296_is_usb_present(bq24296_chip))
  	return POWER_SUPPLY_STATUS_DISCHARGING;
  
	rc = bq24296_i2c_read(bq24296_chip, BQ24296_REG_SS_CNTL , &chgr_sts);
	if (rc) {
		pr_err("failed to read chg status %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

  boost_mode = bq24296_is_boost_mode();
	charging_status = (chgr_sts & CHG_ON_STATUS_MASK) >> CHG_ON_STATUS_SHIFT;
	if (charging_status == CHARGE_STATE_NO_CHG){
		chg_type = POWER_SUPPLY_STATUS_DISCHARGING;
		if(!boost_mode){
		if(!bq24296_chip->batt_psy)
			bq24296_chip->batt_psy = power_supply_get_by_name("battery");
		if(bq24296_chip->batt_psy){
				bq24296_chip->batt_psy->get_property(bq24296_chip->batt_psy,
									POWER_SUPPLY_PROP_HEALTH, &ret);
				bq24296_chip->batt_psy->get_property(bq24296_chip->batt_psy,
					POWER_SUPPLY_PROP_CAPACITY, &ret_capacity);
				if((ret.intval ==POWER_SUPPLY_HEALTH_GOOD) && 
					  (ret_capacity.intval != CHARGE_CAPACITY_FULL))
						chg_type = POWER_SUPPLY_STATUS_CHARGING;
			}
	}
	}
	else if( (charging_status == CHARGE_STATE_PRE_CHG) ||
	          	(charging_status == CHARGE_STATE_FAST_CHG))
		chg_type = POWER_SUPPLY_STATUS_CHARGING;
	else if (charging_status == CHARGE_STATE_CHG_DONE)
		chg_type = POWER_SUPPLY_STATUS_FULL;
  else
  	 return POWER_SUPPLY_STATUS_NOT_CHARGING;
  
 pr_debug("charging_status = %d \n" , charging_status);  
 
	return chg_type;

}

int bq24296_get_prop_charge_type(void)
{
	int rc;
	u8 chgr_sts;
	enum bq24296_charge_status charging_status;
	int chg_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	if(!bq24296_chip)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	if (!bq24296_get_prop_batt_present(bq24296_chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if(!bq24296_is_usb_present(bq24296_chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	rc = bq24296_i2c_read(bq24296_chip, BQ24296_REG_SS_CNTL , &chgr_sts);
	if (rc) {
		pr_err("failed to read chg status %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	charging_status = (chgr_sts & CHG_ON_STATUS_MASK) >> CHG_ON_STATUS_SHIFT;
	if( (charging_status == CHARGE_STATE_PRE_CHG) ||
				(charging_status == CHARGE_STATE_FAST_CHG))
	chg_type = POWER_SUPPLY_STATUS_CHARGING;

	pr_debug("charging_status = %d \n" , charging_status);  
 
	return chg_type;
}

 int
bq24296_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct bq24296_chg_chip *chip = 
												container_of(psy, struct bq24296_chg_chip,
															bq24296_batt_psy);
	/* Check if called before init */	
	if (!chip)
	return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		 val->intval = bq24296_get_prop_batt_status();
		 break;	
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	val->intval = bq24296_get_prop_charge_type();
	break;		
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = (chip->vbus_status == VBUS_CHARGER)?1:0 ;
	break;
	case POWER_SUPPLY_PROP_RESTART_CHARGING:
		val->intval = 1;
	break;
	default:
		return -EINVAL;
	}
	return 0;
}

 int
bq24296_power_set_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct bq24296_chg_chip *chip = container_of(psy, struct bq24296_chg_chip,
								bq24296_batt_psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = bq24296_chg_charge_en(chip, val->intval);
		if (rc)
			DBG_CHARGE("Failed to enabled charging rc=%d\n", rc);
		break;	
		case POWER_SUPPLY_PROP_RESTART_CHARGING:
		rc = bq24296_recharging_cntl(chip);
		if (rc)
			DBG_CHARGE("Failed to Restart charging rc=%d\n", rc);
		break;		
	default:
		return -EINVAL;
	}
	DBG_CHARGE("psy changed bq24296_batt_psy\n");
	power_supply_changed(chip->batt_psy);
	return rc;
}

#define BQ24296_CHG_I_MAX_MIN_100		100
static void 
bq24296_pm_batt_external_power_changed(struct power_supply *psy)
{
	struct bq24296_chg_chip *chip = container_of(psy, struct bq24296_chg_chip,
								bq24296_batt_psy);
	union power_supply_propval ret = {0,};

	if (!chip)
		return;

	if (!chip->usb_psy)
					chip->usb_psy = power_supply_get_by_name("usb");

	/* Only honour requests while USB is present */
	if (bq24296_is_usb_present(chip)) {
		chip->usb_psy->get_property(chip->usb_psy,
			  POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		if (ret.intval <= 2 &&
				bq24296_get_prop_batt_present(chip)) {
			if (ret.intval ==  2){
				}
			} else {
					chip->maxinput_chg_ma = ret.intval/1000;
				  chip->wd_reinit_flag = true;
					schedule_delayed_work(&chip->wd_work,
								msecs_to_jiffies(WD_KICKING_PERIOD_MS));
			}
	 }
	else{
		  chip->maxinput_chg_ma = BQ24265_CHG_I_MAX_MIN_100;
	 	  //bq24296_chg_iusbmax_set(chip, BQ24265_CHG_I_MAX_MIN_100);
		}
}

static void
bq24296_chg_print_info(struct bq24296_chg_chip *chip)
{
  int index = 0;
  u8 ret = 0;
  for(index = 0;index<11;index++)
  {  
			unsigned char  reg = index;
			bq24296_i2c_read(chip, reg, &ret);
			DBG_CHARGE("Reg:%x = %x \n", reg , ret);
  }

 DBG_CHARGE("batt:voltage =%d, temp = %d , present = %d ,usb online = %d gpio_ce = %d \n ",
 	  chip->batt_voltage,
 	  bq24296_get_prop_batt_temp(chip),
 	  bq24296_get_prop_batt_present(chip),
 	  bq24296_is_usb_present(chip),
 	  gpio_get_value(chip->gpio_ce)  );
}

#define BQ24296_CONSECUTIVE_COUNT 	3
static void
bq24296_wd_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq24296_chg_chip *chip = container_of(dwork,
								struct bq24296_chg_chip, wd_work);
	static int count;
    int  usb_present;
    usb_present = bq24296_is_usb_present(chip);

    if(!chip){	return ; }

    if(chip->in_work == 0){
  	    chip->in_work = 1;
	    bq24296_chg_wd_fchg_timer_set(chip,
						  chip->wd_reset_ms/1000,chip->max_chgcycle_mins);
    }
	
  chip->batt_voltage = 
    	       bq24296_get_prop_battery_voltage_now(chip);
	if (!wake_lock_active(&chip->wlock))
		wake_lock(&chip->wlock);

	bq24296_chg_wd_reset(chip,true);
	bq24296_chg_print_info(chip);

	if( usb_present ){
			count = 0;
	} else {
		if (count == BQ24296_CONSECUTIVE_COUNT) {
			DBG_CHARGE(" End Of WD Work!\n");
			goto stop_work;
		}else{
			count += 1;
			DBG_CHARGE("WD Count = %d \n", count);
		}
    }
	schedule_delayed_work(&chip->wd_work,
				msecs_to_jiffies(WD_KICKING_PERIOD_MS));								
	return;

stop_work:
	DBG_CHARGE("Stop WD Work \n");
	count = 0;
	chip->wd_reinit_flag = true;
	chip->in_work = 0;
	bq24296_chg_wd_fchg_timer_set(chip, 0, chip->max_chgcycle_mins);
	wake_unlock(&chip->wlock);
}

int  bq24296_set_chg_status(int usb_in)
{
  	DBG_CHARGE("usb_in=%d\n",usb_in);
		
	if (!bq24296_chip) {
		pr_err("%s:called before init\n",__func__);
		charger_online = usb_in;
		return -1;
	}  

    bq24296_chip->usb_in = usb_in;  
	
	if(usb_in){
		wake_lock(&bq24296_chip->wlock);
		schedule_delayed_work(&bq24296_chip->wd_work,
			  	msecs_to_jiffies(WD_KICKING_PERIOD_MS));					
	}
	
 	return 0;
}
EXPORT_SYMBOL_GPL(bq24296_set_chg_status);

static int 
bq24296_power(bool enable, struct bq24296_chg_chip *chip)
{
	int rc = 0;
	const struct bq24296_platform_data *platdata;
	platdata = chip->pdata;

	if (enable){
		DBG_CHARGE("Power Success ! \n");
	}else{
	}
	return rc;
}

static int
bq24296_chg_hw_init(struct bq24296_chg_chip *chip)
{
	int rc ; 

//0x00 (0x1A)	
	rc = bq24296_chg_power_on_set(chip,chip->v_cutoff_mv);
	if (rc) {
		pr_err("failed setting power_on rc=%d\n", rc);
		return rc;
	}
	
	rc = bq24296_chg_vusbmax_set(chip,chip->maxinput_chg_mv);
	if (rc) {
		pr_err("failed setting vusbmax rc=%d\n", rc);
		return rc;
	}
	//0x01(0x32-500mA, 0x31-1500mA)
	rc = bq24296_chg_iusbmax_set(chip,chip->maxinput_chg_ma);
	if (rc) {
		pr_err("failed setting iusbmax rc=%d\n", rc);
		return rc;
	}

	//0x02(0x60)
	rc = bq24296_chg_ibatmax_set(chip,
							chip->maxbat_chg_current_ma,chip->cold_temperature_threshold);
	if (rc) {
		pr_err("failed setting ibatmax rc=%d\n", rc);
		return rc;
	}
	//0x03(0x10)
	rc = bq24296_chg_iprechg_and_iterm_set(chip,
									chip->prechg_current_ma,chip->term_current_ma);
	if (rc) {
		pr_err("failed setting iprechg_and_iterm rc=%d\n", rc);
		return rc;
	}
	//0x04(0xD6)
	rc = bq24296_chg_vddmax_and_trim_set(chip,chip->maxvdd_voltage_mv,0);
	if (rc) {
		pr_err("failed setting vddmax rc=%d\n", rc);
		return rc;
	}
	//0x05(AC)
	rc = bq24296_chg_wd_fchg_timer_set(chip,
	                chip->wd_reset_ms/1000,chip->max_chgcycle_mins);
	if (rc) {
		pr_err("failed setting wd_fchg_timer rc=%d\n", rc);
		return rc;
	}
	//0x06(0x73)
	rc = bq24296_chg_boost_thermal_set(chip, chip->boostoutput_chg_mv,
											chip->hot_temperature_threshold,chip->thermal_regulation_threshold);
	if (rc) {
		pr_err("failed setting boost_thermal rc=%d\n", rc);
		return rc;
	}
	//0x07(0X0B)
	rc = bq24296_chg_misc_operation_set(chip);
	if (rc) {
		pr_err("failed setting ibatmax rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static int  bq24296_apply_dt_configs(struct bq24296_chg_chip *chip)
{
	struct device *dev = &chip->client->dev;
	struct device_node *node = dev->of_node;
	int rc = 0;

	chip->gpio_ce = of_get_named_gpio_flags(node,
	   "bq24296,ce-gpio", 0, NULL);
	DBG_CHARGE("chip->gpio_ce = %d \n ", chip->gpio_ce);
	//property
	rc |= of_property_read_u32(node, "bq24296,maxinput-chg-mv", &chip->maxinput_chg_mv);
	rc |= of_property_read_u32(node, "bq24296,maxinput-chg-ma", &chip->maxinput_chg_ma);
	rc |= of_property_read_u32(node, "bq24296,v-cutoff-mv", &chip->v_cutoff_mv);
	rc |= of_property_read_u32(node, "bq24296,ibatmax-ma", &chip->maxbat_chg_current_ma);
	rc |= of_property_read_u32(node, "bq24296,ibatterm-ma", &chip->term_current_ma);
	rc |= of_property_read_u32(node, "bq24296,ibattprechg-ma", &chip->prechg_current_ma);
	rc |= of_property_read_u32(node, "bq24296,max-voltage-mv", &chip->maxvdd_voltage_mv);
	rc |= of_property_read_u32(node, "bq24296,watchdog-reset-ms", &chip->wd_reset_ms);
	rc |= of_property_read_u32(node, "bq24296,max-charger-cycle-mins", &chip->max_chgcycle_mins);
	rc |= of_property_read_u32(node, "bq24296,boost-output-chg-mv", &chip->boostoutput_chg_mv);
	rc |= of_property_read_u32(node, "bq24296,boost-hot-temp-threshold", &chip->hot_temperature_threshold);
	rc |= of_property_read_u32(node,"bq24296,boost-cold-temp-threshold", &chip->cold_temperature_threshold);
	rc |= of_property_read_u32(node,"bq24296,thermal_regulation-threshold", &chip->thermal_regulation_threshold);
	if (rc)
		pr_err("failed to read required dt parameters %d\n", rc);
	
		return rc;
}

static int  bq24296_is_boost_mode(void)
{
	if (!bq24296_chip) {
		DBG_CHARGE("called before init\n");
		return -1;
	}
	return (bq24296_chip->vbus_status == VBUS_BOOST);
}

static int  bq24296_get_vbus_mode(struct bq24296_chg_chip *chip)
{
	int rc = 0;
	u8 ret = 0;
	
   DBG_CHARGE("Get VBUS Mode! \n" );
	 if(!chip)
	 	  return -1;
	 
	rc = bq24296_i2c_read(chip,BQ24296_REG_PO_CNTL,&ret);
	if (rc )
	{
		pr_err("%s:Failed Read.Reg=0x%02X,Rc=%d.\n",
									__func__,BQ24296_REG_PO_CNTL,rc );
		return -1;
	}

	DBG_CHARGE("%s:Write Reg 0x%02X=0x%02X\n",
									__func__,BQ24296_REG_PO_CNTL,ret);
	
	return  (ret & OTG_ENABLE) ? VBUS_BOOST : VBUS_CHARGER;
}

static int bq24296_set_vbus_mode(struct bq24296_chg_chip *chip, 
	          enum vbus_mode  vbus_status_temp)
{
	int rc = 0;
	u8 ret = 0;
	
	 if(!chip)
	 	  return -1 ;

 	rc = bq24296_i2c_read(chip,BQ24296_REG_PO_CNTL,&ret);
	if (rc){
		pr_err("%s:Failed Read.Reg=0x%02X,Rc=%d.\n",
									__func__,BQ24296_REG_PO_CNTL,rc );
		return rc;
	}

	if(vbus_status_temp == VBUS_CHARGER){
		ret |= CHG_ENABLE;
		ret &= ~OTG_ENABLE;
		gpio_direction_output(chip->gpio_ce,1);
	}else if(vbus_status_temp == VBUS_BOOST){
		ret &= ~CHG_ENABLE;
		ret |= OTG_ENABLE;
		gpio_direction_output(chip->gpio_ce,0);
	}else{
	  DBG_CHARGE("Invalid VBUS Mode! \n");
		return -1;
	}

  rc = bq24296_i2c_write(chip,BQ24296_REG_PO_CNTL,ret);
	if (!rc){
		pr_debug("%s:Write Reg 0x%02X=0x%02X\n",
										__func__,BQ24296_REG_PO_CNTL,ret);
		chip->vbus_status = vbus_status_temp;
		DBG_CHARGE("VBUS Mode Set To:%d\n", chip->vbus_status);
	}
	else
			DBG_CHARGE("Couldn't Set VBUS Mode rc=%d\n", rc);
	
	/*wait 20us*/
	udelay(20);

	return rc;
}

static int bq24296_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct bq24296_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;

  DBG_CHARGE("To Enable OTG \n" );
  if(!chip)
 	  return rc;

  rc = bq24296_set_vbus_mode(chip,VBUS_BOOST);
  if(rc)
  	DBG_CHARGE("Couldn't enable  OTG mode rc=%d\n", rc);

	return rc;
}

static int bq24296_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct bq24296_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;

  DBG_CHARGE("To Disable OTG \n" );
  if(!chip)
 	  return rc;
	 
	 rc = bq24296_set_vbus_mode(chip,VBUS_CHARGER);
  if(rc)
  	DBG_CHARGE("Couldn't enable OTG mode rc=%d\n", rc);
   
	return rc;
}

static int bq24296_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct bq24296_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 ret = 0;

	 if(!chip)
	 	  return rc;
	 
	rc = bq24296_i2c_read(chip,BQ24296_REG_PO_CNTL,&ret);
	if (rc )
	{
		pr_err("%s:Failed Read.Reg=0x%02X,Rc=%d.\n",
									__func__,BQ24296_REG_PO_CNTL,rc );
		return rc;
	}

	DBG_CHARGE("%s:Write Reg 0x%02X=0x%02X\n",
										__func__,BQ24296_REG_PO_CNTL,ret);
		
	return  (ret & OTG_ENABLE) ? 1 : 0;
}

struct regulator_ops bq24296_otg_reg_ops = {
	.enable		= bq24296_otg_regulator_enable,
	.disable	=  bq24296_otg_regulator_disable,
	.is_enabled	= bq24296_otg_regulator_is_enable,
};

static int bq24296_regulator_init(struct bq24296_chg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		DBG_CHARGE( "Unable to allocate memory\n");
		return -ENOMEM;
	}

  DBG_CHARGE( "regulator name : %s \n", init_data->constraints.name);
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &bq24296_otg_reg_ops;
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
				DBG_CHARGE(	"OTG reg failed, rc=%d\n", rc);
		}
	}
  
	return rc;
}

static int  bq24296_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	const struct bq24296_platform_data  *pdata;
	struct bq24296_chg_chip *chip;
	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	int rc= 0;

//Control Debug Msg
	//console_loglevel = 7;
	 DBG_CHARGE("BQ24296 Driver!");

		usb_psy = power_supply_get_by_name("usb");
		if (!usb_psy) {
			dev_dbg(&client->dev, "USB supply not found; defer probe\n");
			return -EPROBE_DEFER;
		}

/*
		batt_psy = power_supply_get_by_name("battery");
		if (!batt_psy) {
			dev_dbg(&client->dev, "Battery supply not found; defer probe\n");
			return -EPROBE_DEFER;
		}
*/		
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->batt_psy = batt_psy;
	pdata = client->dev.platform_data;
	chip->pdata = pdata;

  chip->vadc_dev = qpnp_get_vadc(chip->dev, "bq24296");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc == -EPROBE_DEFER)
			pr_err("vadc not found - defer rc=%d\n", rc);
		else
			pr_err("vadc property missing, rc=%d\n", rc);

		return rc;
	}

	device_init_wakeup(chip->dev, 1);
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->read_write_lock);

	rc = bq24296_apply_dt_configs(chip);
	if (rc)
		return rc;

   bq24296_gpio_init(chip);
   bq24296_gpio_select(chip, true);

	rc = gpio_request(chip->gpio_ce, "CHARGE_EN");
	if (rc) {
		pr_err("%s:Fail To Request GPIO %d (%d)\n",
											__func__, chip->gpio_ce, rc);
		goto chg_failed_3;
	}
	//高电平使能充电
	gpio_direction_output(chip->gpio_ce,1);
	
	rc = bq24296_regulator_init(chip);
	if (rc) {
		DBG_CHARGE(	"Couldn't initialize bq24296 ragulator rc=%d\n", rc);
		return rc;
	}
  
	rc = bq24296_chg_hw_init(chip);
	if (rc) {
		pr_err("Failed To Configure Ret = %d\n", rc);
		goto chg_failed_2;
	}
	
	rc = bq24296_power(true, chip);
	if (rc) {
		pr_err("Failed To Powerup Ret = %d\n", rc);
		goto chg_failed_3;
	}

	wake_lock_init(&chip->wlock, WAKE_LOCK_SUSPEND, "bq24296_charger");
	INIT_DELAYED_WORK(&chip->wd_work,bq24296_wd_work);		
	chip->bq24296_batt_psy.name = "bq24296-battery";
	chip->bq24296_batt_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->bq24296_batt_psy.properties = bq24296_pm_power_props_mains;
	chip->bq24296_batt_psy.num_properties = ARRAY_SIZE(bq24296_pm_power_props_mains);
	chip->bq24296_batt_psy.get_property = bq24296_power_get_property_mains;
	chip->bq24296_batt_psy.set_property = bq24296_power_set_property_mains;
	chip->bq24296_batt_psy.external_power_changed = bq24296_pm_batt_external_power_changed;
	
	rc = power_supply_register(chip->dev, &chip->bq24296_batt_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto free_chip;
	}
 	bq24296_chip = chip;

	schedule_delayed_work(&chip->wd_work,
			 msecs_to_jiffies(WD_KICKING_PERIOD_MS));
  DBG_CHARGE("BQ24296 Successed!");
  return 0;

chg_failed_3:
chg_failed_2:	
free_chip:
		kfree(chip);
  return rc;
}

static int bq24296_charger_remove(struct i2c_client *client)
{	
	struct bq24296_chg_chip *chip = i2c_get_clientdata(client);
	//Set Device To Shipping Mode
	bq24296_chg_batfet_disable(chip,true);
	bq24296_power(false, chip);		

	mutex_destroy(&chip->read_write_lock);
	return 0;
}

static int bq24296_suspend(struct i2c_client *cl, pm_message_t mesg)
{
	return 0;
};

static int bq24296_resume(struct i2c_client *cl)
{
	return 0;
};

static struct of_device_id bq_24296_match_table[] = {
	{ .compatible = "ti,bq24296-chg",},
	{}
};

static const struct i2c_device_id bq24296_id[] = {
	{ "bq24296-chg", 1 },
	{},
};

static struct i2c_driver bq24296_charger_driver = {
	.driver = {
		.name = "bq24296-chg",
		.of_match_table = bq_24296_match_table,
	},
	.id_table 	= bq24296_id,
	.probe 		= bq24296_charger_probe,
	.remove 	= bq24296_charger_remove,

	.suspend	= bq24296_suspend,
	.resume 	= bq24296_resume,
};

static int __init bq24296_charger_init(void)
{
	return i2c_add_driver(&bq24296_charger_driver);
}

static void __exit bq24296_charger_exit(void)
{
	i2c_del_driver(&bq24296_charger_driver);
}

module_init(bq24296_charger_init);


module_exit(bq24296_charger_exit);

MODULE_AUTHOR("ghzhou<ghzhou1978@163.com.cn>");
MODULE_DESCRIPTION("bq24296 charger driver");
MODULE_LICENSE("GPL");


