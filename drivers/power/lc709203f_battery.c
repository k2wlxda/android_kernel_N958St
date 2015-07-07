/*
 *  lc709203f_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 ZTE Mobile  
 *   
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifdef CONFIG_ZTEMT_BATTERY_FG_LC709203F

#include <linux/module.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/regulator/consumer.h>
#include "linux/power/lc709203f_battery.h"

#define   LC709203F_DELAY  		2000
#define	dPOLYNOMIAL8			0x8380
#define   CHARGING_MODE		0x0001
#define   DISCHARGING_MODE		0xFFFF
#define	AUTO_MODE				0x0000
#define   FALSE					0
#define   TRUE					1
#define   DEFAULT_SOC			50
#define   BATT_TEMP_HIGH   		800
#define   BATT_TEMP_LOW    		-200
#define   DEFAULT_BATT_TEMP 	300
#define   DEFAULT_ALARM_SOC 	0x08

enum   
{
	NOEMAL_MODE,
	ECO_MODE,
	SLEEP_MODE
} lc709203f_mode;

struct lc709203f_chip {
	struct i2c_client		*client;
	struct device			*dev;
	struct delayed_work	work;
	struct wake_lock		low_voltage_wake_lock;
	struct work_struct		alarm_voltage_work;

	//struct lc709203f_platform_data	*pdata;
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
#ifdef CONFIG_ZTEMT_BQ24296_CHARGE	
	struct power_supply *bq24296_batt_psy;
#endif
	struct pinctrl *fg_pinctrl;
	struct pinctrl_state *gpio_state_default;
	//struct pinctrl_state *gpio_state_suspend;
	struct dentry		*dent;
	
	/* State Of charging */
	bool chg_status;
	/* State Of Connect */
	bool online;
	/* battery voltage */
	u32 vcell;
	/* battery capacity */
	u32 soc;
	/* State Of Charge */
	u32 batt_status;
	/* health Of Charge */
	u32 batt_health;	
	/*battery  temperture*/
	u32 batt_temperture;
	/*low soc update delay */
	u32 	low_soc_calculate_soc_ms;
	/*low voltage  update delay */
	u32	low_voltage_calculate_soc_ms;
	/*  update delay */
	u32	calculate_soc_ms;
	/*low soc  threshold*/
	u32 alarm_soc_threshold ;
	/*low voltage  threshold*/
	u32 alarm_voltage_threshold ;

	u32 low_soc_calculate_soc_threshold;

	/*resume charging soc*/
	u32 recharge_soc;

	bool chg_done;	
	
	bool irq_enabled;
	unsigned int irq_gpio;
	int irq;

	int first_resume;
	bool batt_por;
	bool is_sleep;
	u16 power_mode;
	bool recharge_flag;
	bool batt_removed_flag;
};

/**
 * struct lc709203f_soc_linear_voltage - Represent soc linear with voltage .
 * @dy: soc.
 * @dx: voltage .
 *
 * Each soc device has different offset and slope parameters with voltage
 */
struct lc709203f_soc_linear_voltage{
	int y;
	int x;
};

static const struct lc709203f_soc_linear_voltage soc_linear_voltage[] = {
	{3400,0  },
	{3459,1  },
	{3610,5  },
	{3671,10 },
	{3698,15 },
	{3710,20 },
	{3730,25 },
	{3760,30 },
	{3778,35 },
	{3800,40 },
	{3830,45 },
	{3862,50 },
	{3895,55 },
	{3925,60 },
	{3965,65 },
	{4010,70 },
	{4050,75 },
	{4080,80 },
	{4120,85 },
	{4165,90 },
	{4210,95 },
	{4275,100}
};

const int   expand_max_soc = 1000;     //unit 0.1
const int   expand_min_soc  = 0;
const int   high_soc = 980;
const int   low_soc = 5;
const int   scale_soc =10;

extern int qpnp_get_battery_temp(void);
extern int qpnp_is_batt_present(void);
extern int lc709203f_read_shutdown_ocv_soc(struct pwr_on_param *pwr_on_param);

static struct lc709203f_chip *global_chip ;
//static bool full_charge = FALSE;	

static void lc709203_alarm_voltage_work(struct work_struct *work);

//#define LC709203_DEBUG
#ifdef LC709203_DEBUG
static int lc709203f_read_reg(struct i2c_client *client, int reg);
static int lc709203f_write_reg(struct i2c_client *client, u8 reg, u16 data);

//uart debug
#define ZTEMT_UART_DEBUG_ENABLE

#ifdef ZTEMT_UART_DEBUG_ENABLE
#define MAX_INFO 1
#define MAX_DEBUG 4
//log level < maxlog_level will show    path:sys/module/lc709203f_battery/parameters
int maxlog_level = 3;  
module_param(maxlog_level, int, 0644);

#define MAXLOG_INFO(fmt, args...) \
		if (MAX_INFO < maxlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)
	
#define MAXLOG_DEBUG(fmt, args...) \
		if (MAX_DEBUG < maxlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)


static int debug_uart = 3;
module_param(debug_uart, int, 0644);

static void lc709203f_dump_regs(struct lc709203f_chip *chip)
{
    int i;
	u16 val;
	static int first_flag = 1;

	if(first_flag){
		first_flag = 0;
		printk("MDREG ");
		for(i=0;i<=0x4d;i++){
			printk("%4x ",i);
		}
		printk("%4x %4x \n",0xfb, 0xff);
	}
	printk("MDREG ");
	
	for(i=0x6;i<=0x1a;i++)
	{
		val = lc709203f_read_reg(chip->client, i);
        	printk("%4x ",val);
	}


}
#endif

static int debug_reg = 0;
module_param(debug_reg, int, 0644);


static int lc709203f_debug;
static int lc709203f_debug_mode(const char *val, struct kernel_param *kp)
{
	int ret;
    	u16 value = 0;
	
	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!global_chip) {
		pr_err("%s:called before init\n",__func__);
		return 1;
	}
	
	printk("__%s: lc709203f_debug=%d!\n",__func__,lc709203f_debug);
	switch(lc709203f_debug){
    case 0:
		value = lc709203f_read_reg(global_chip->client, debug_reg);
		printk("__%s: debug reg[0x%x]=0x%x\n",__func__,debug_reg,value);
		break;
	default:
		break;
	};
	
	return 0;
}
module_param_call(lc709203f_debug, lc709203f_debug_mode, param_get_uint,
					&lc709203f_debug, 0644);

//path: /sys/bus/i2c/drivers/ lc709203f/1-0036
static ssize_t lc709203f_show_power_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lc709203f_chip *chip = dev_get_drvdata(dev);
	
	return  sprintf(buf, "%d\n", chip->batt_por);
}
static ssize_t lc709203f_store_power_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct lc709203f_chip *chip = dev_get_drvdata(dev);
	
	chip->batt_por = *buf;
	return 1;
}
const static DEVICE_ATTR(batt_por, S_IRUGO | S_IWUSR,
	            lc709203f_show_power_status, lc709203f_store_power_status);

/***********************************************************
*  for debug reg , path: sys/kernel/debug/lc709203f
*  reg: the reg to read or write
*  data: 'echo x > data' to write the reg  and 'cat data 'to read the reg
************************************************************/
static u8 maxin_reg;
static int get_reg_addr(void *data, u64 * val)
{
	*val = maxin_reg;
	return 0;
}

static int set_reg_addr(void *data, u64 val)
{
	maxin_reg = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(set_reg_fops, get_reg_addr, set_reg_addr, "0x%02llx\n");

static int get_reg_data(void *data, u64 * val)
{
	struct lc709203f_chip *chip = (struct lc709203f_chip *)data;
	int ret;
	//u16 value;

    	ret = lc709203f_read_reg(chip->client, maxin_reg);
	if (ret<0) {
		pr_err("%s:fail to read batt temp\n",__func__);
		return ret;
	}

	*val = ret;
	return 0;
}

static int set_reg_data(void *data, u64 val)
{
	struct lc709203f_chip *chip = (struct lc709203f_chip *)data;
	int ret;
	u16  value = val;

	ret = lc709203f_write_reg(chip->client, maxin_reg, value);
	if (ret<0) {
	   pr_err("%s:fail to read batt temp\n",__func__);
	   return ret;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rw_reg_fops, get_reg_data, set_reg_data, "0x%02llx\n");

 void max_create_debugfs_entries(struct lc709203f_chip *chip)
{
	chip->dent = debugfs_create_dir("lc709203f", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("lc709203f couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("reg", 0644, chip->dent, chip, &set_reg_fops);
	debugfs_create_file("data", 0644, chip->dent, chip, &rw_reg_fops);
	return;
}

#endif


static unsigned char u1_CRC_8_u1u1( unsigned char u1ArgBeforeData , unsigned char u1ArgAfterData)
{
	unsigned char	u1TmpLooper = 0;
	unsigned char	u1TmpOutData = 0;
	unsigned short	u2TmpValue = 0;

	u2TmpValue = (unsigned short)(u1ArgBeforeData ^ u1ArgAfterData);
	u2TmpValue <<= 8;

	for( u1TmpLooper = 0 ; u1TmpLooper < 8 ; u1TmpLooper++ ){
		if( u2TmpValue & 0x8000 ){
			u2TmpValue ^= dPOLYNOMIAL8;
		}
		u2TmpValue <<= 1;
	}

	u1TmpOutData = (unsigned char)(u2TmpValue >> 8);

	return( u1TmpOutData );
}

unsigned char get_write_CRC( int reg,u16 data )
{
	 unsigned char	u1Debug = 0;
	 unsigned char	u1CRC8 = 0;

	// Write Word Protocol
	u1Debug = u1_CRC_8_u1u1( 0x00 , 0x16 );		// Address
	u1Debug = u1_CRC_8_u1u1( u1Debug , reg );	// Command
	u1Debug = u1_CRC_8_u1u1( u1Debug , data&0xff );	// low  Data
	u1CRC8  = u1_CRC_8_u1u1( u1Debug , (data&0xff00) >>8);	// high  Data

	return( u1CRC8 );			
}

static int lc709203f_write_reg(struct i2c_client *client, u8 reg, u16 data)
{
	s32 ret;
	u8 values[3] = {0,0,0};
	values[0] = data&0xff ;
	values[1] = (data&0xff00) >>8;
	values[2] =  get_write_CRC(  reg, data );
	
	ret = i2c_smbus_write_i2c_block_data(client, reg, sizeof(values)/sizeof(values[0]), values);

	if (ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	}
	return ret;
}

static int lc709203f_read_reg(struct i2c_client *client, int reg)
{
	s32 ret;
	u8 values[3] = {0,0,0};

	ret = i2c_smbus_read_i2c_block_data(client, reg, sizeof(values)/sizeof(values[0]), values);

	if (ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}		
	return (values[0]|(values[1]<<8));
}

static int lc709203f_get_prop_batt_status(struct lc709203f_chip* chip)
{
	union power_supply_propval ret = {0,};

	if (chip->batt_psy == NULL) {
		chip->batt_psy = power_supply_get_by_name("battery");
	}

	if (chip->batt_psy){
		chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_STATUS,&ret);
	}
	return chip->batt_status = ret.intval;
}

static int lc709203f_get_prop_batt_health(struct lc709203f_chip* chip)
{
    	union power_supply_propval ret = {0,};

	if (chip->batt_psy == NULL) { 	
		chip->batt_psy = power_supply_get_by_name("battery");
	}

	if (chip->batt_psy){
		chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_HEALTH,&ret);
	}
	return chip->batt_health = ret.intval;
}

static int lc709203f_is_charger_online(void)
{

	union power_supply_propval ret = {0,};
	if(global_chip){
		if (!global_chip->usb_psy){
				global_chip->usb_psy = power_supply_get_by_name("usb");
		}
		if (global_chip->usb_psy){
				global_chip->usb_psy->get_property(global_chip->usb_psy,
				  	POWER_SUPPLY_PROP_ONLINE, &ret);
			return ret.intval;
		}	
	}
	return FALSE;
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(100, soc);
	return soc;
}

static int lc709203f_rescaled_rsoc(u16 soc)
{
	int ret;
	
	if(soc<expand_min_soc||soc>expand_max_soc){
		return DEFAULT_SOC;
	}
	
	if(soc < low_soc){
		return 0;
	}else if(soc >= high_soc){
		return 100;
	}else{
		ret = ((soc-low_soc)*expand_max_soc)/(high_soc-low_soc)/scale_soc;	
	//	ret = soc/scale_soc;
		return ret;
	}
}

static int lc709203f_update_soc(struct lc709203f_chip* chip)
{
   	int ret;
	int rsoc = 0;
	static int once_flag = TRUE;

	if (!chip) {
		pr_err("%s:called before init\n",__func__);
		return 1;
	}
	
    	ret = lc709203f_read_reg(chip->client,  LC709203F_INDICATOR_TO_EMPTY);
	if (ret<0) {
		pr_err("%s:fail to read batt soc\n",__func__);
		if(once_flag){
			int rc;
			struct pwr_on_param	pwr_on_param;
			once_flag = FALSE;
			rc = lc709203f_read_shutdown_ocv_soc(&pwr_on_param);
			if (rc < 0){
				return 100 ;
				
			}else{
				return bound_soc(pwr_on_param.backup_soc);
			}					
		}else{
			return chip->soc;
		}
		
	}	
	rsoc = lc709203f_rescaled_rsoc(ret);
	printk("%s: soc=%d---  rsoc=%d\n",__func__,ret,rsoc);
	if(once_flag){
		once_flag = FALSE;
	}
	return bound_soc(rsoc);
}

int lc709203f_get_soc( void)
{
	if (!global_chip) {
		pr_err("%s:called before init\n",__func__);
		return 1;
	}

	if(global_chip->first_resume)
		return lc709203f_update_soc(global_chip);
#ifdef ZTEMT_UART_DEBUG_ENABLE	
	MAXLOG_DEBUG("report soc=%d\n",global_chip->soc);
#endif	
	printk("%s:  report soc=%d\n",__func__,global_chip->soc);
	return global_chip->soc;
}

static int  lc709203f_set_charge_mode(struct lc709203f_chip* chip,u16 charge_mode)
{
	//printk ("%s: charge_mode ==%d \n", __func__,charge_mode);
	return lc709203f_write_reg(chip->client,  CURRENT_DIRECTION,charge_mode);		
}

void  lc709203f_set_recharge_flag(bool flag)
{
	int usb_in ;
	int new_batt_health  ;
	usb_in   = lc709203f_is_charger_online( );
		
	if(global_chip)
	{
		new_batt_health = lc709203f_get_prop_batt_health(global_chip);
	}else{
		printk("%s:global_chip is null \n", __func__);
		return;
	}
	/**
	if(usb_in){
		if(global_chip->soc==100){
			full_charge = TRUE;
		}else{
			full_charge = FALSE;
		}
	}
	**/
	if(flag){
		if(usb_in){
			global_chip->chg_done = flag;
		}
	}else{
		if(!usb_in&&global_chip->chg_done) {
			global_chip->chg_done = flag;
		}
		else if(new_batt_health != POWER_SUPPLY_HEALTH_GOOD){
			global_chip->chg_done = flag;
		}
	} 
	
	if(usb_in&&(global_chip->power_mode==DISCHARGING_MODE)){
		lc709203f_set_charge_mode(global_chip,AUTO_MODE);
		global_chip->power_mode = AUTO_MODE;
	}else if((!usb_in)&&(global_chip->power_mode==AUTO_MODE)){
		lc709203f_set_charge_mode(global_chip,DISCHARGING_MODE);
		global_chip->power_mode = DISCHARGING_MODE;
	}
	printk ("%s: new_batt_health=%d  usb_in=%d chg_done ==%d \n", __func__,new_batt_health,usb_in,flag);
}

static int  lc709203f_update_vcell(struct lc709203f_chip* chip)	
{	
	u16 current_voltage = 0;
	
	if (chip->client){
		current_voltage = lc709203f_read_reg(chip->client,  BATTERY_VOLTAGE);
	}
	else {
		return chip->vcell;
	}
	 if (current_voltage <= 0){
	 	return chip->vcell;
	 }
	 else{
		return chip->vcell = current_voltage;
	 }
}

int lc709203f_get_voltage(void)
{
	if (!global_chip) {
		pr_err("%s:called before init\n",__func__);
		return 1;
	}

	if(global_chip->first_resume)
		return lc709203f_update_vcell(global_chip);
#ifdef ZTEMT_UART_DEBUG_ENABLE	
	MAXLOG_DEBUG("report soc=%d\n",global_chip->vcell);
#endif	
	//printk("%s:  report soc=%d\n",__func__,global_chip->vcell);
	return global_chip->vcell;
}

static void lc709203f_init(struct lc709203f_chip* chip)
{		
	lc709203f_write_reg(chip->client,LC709203F_POWER_MODE,ECO_MODE);
	msleep(200);
	lc709203f_write_reg(chip->client,LC709203F_POWER_MODE,ECO_MODE);
	lc709203f_write_reg(chip->client,LC709203F_ADJUSTMENT_PACK_APPLI,LC709203F_APA_PARAM);
	lc709203f_write_reg(chip->client,LC709203F_ALARM_LOWER_RSOC, chip->alarm_soc_threshold);
	lc709203f_write_reg(chip->client,LC709203F_ALARM_LOWER_VOLTAGE, chip->alarm_voltage_threshold);
}

static void lc709203f_check_battery_removed(struct lc709203f_chip* chip)
{
	int ret = 0;
	
	ret = lc709203f_read_reg(chip->client, LC709203F_ALARM_LOWER_RSOC );
	chip->batt_removed_flag  = (ret == DEFAULT_ALARM_SOC)?TRUE:FALSE;
	printk("%s:batt_removed_flag =%d\n",__func__,chip->batt_removed_flag);
}

static int lc709203f_map_soc_voltage(const struct lc709203f_soc_linear_voltage *pts,
		int tablesize, int input, int *output)
{
	bool descending = 1;
	int i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = ( ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y)) + pts[i-1].x ;
	}
	printk("%s:voltage =%d--soc =%d\n",__func__,input,*output);
	return 0;
}

#define DELTA_OFFSET_SOC		4
void lc709203_calculate_initial_soc(void)
{
	int rc = 0;
	int backup_soc_invalid = 0;
	int map_soc = 0;
	static bool once_flag = TRUE;
	bool  update_soc_flag = FALSE;
	struct pwr_on_param	pwr_on_param;
	
	//just  call once
	if(once_flag){
		once_flag = FALSE;
		return ;
	}
	once_flag = TRUE;

	rc = lc709203f_read_shutdown_ocv_soc(&pwr_on_param);
	/*
	 * if we have powered on from warm reset -
	 * Always donot initial LC708203F soc
	 */
	if(pwr_on_param.warm_reset){
		return ;
	}
	
	if ((rc < 0)||((!global_chip->soc)&&(global_chip->vcell>3420))){
		backup_soc_invalid = 1;
	}
	if(global_chip->batt_removed_flag){
		backup_soc_invalid = 1;
	}
	
	if(backup_soc_invalid){
		lc709203f_map_soc_voltage(soc_linear_voltage,ARRAY_SIZE(soc_linear_voltage),\
									pwr_on_param.current_ocv,&map_soc);
		if(abs(map_soc- global_chip->soc) >DELTA_OFFSET_SOC){
			lc709203f_write_reg(global_chip->client,LC709203F_INITIAL_RSOC,LC709203F_INITIAL_RSOC_DEFALUT_PARAM);
			lc709203f_init(global_chip);
			update_soc_flag = TRUE;
		}
	}

	if(update_soc_flag){
		global_chip->soc = lc709203f_update_soc(global_chip);
	}	
	printk("%s:  shutdown_soc=%d---shutdown_voltage=%d--ocv_volt=%d--update_soc =%d--map_soc =%d\n",\
			__func__,pwr_on_param.backup_soc,pwr_on_param.backup_voltage,pwr_on_param.current_ocv,\
			global_chip->soc,map_soc);
}

static void update_power_supply(struct lc709203f_chip *chip)
{
	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy > 0)
		power_supply_changed(chip->batt_psy);
}

static int get_calculation_delay_ms(struct lc709203f_chip *chip)
{
	if (wake_lock_active(&chip->low_voltage_wake_lock))
		return chip->low_voltage_calculate_soc_ms;
	else if (chip->soc < chip->alarm_soc_threshold)
		return chip->low_soc_calculate_soc_ms;
	else
		return chip->calculate_soc_ms;
}

static int  lc709203f_get_batt_temperture(struct lc709203f_chip* chip)
{
	union power_supply_propval ret = {300,};

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");
	}

	if (chip->batt_psy){
		chip->batt_psy->get_property(chip->batt_psy,POWER_SUPPLY_PROP_TEMP,&ret);
	}
   	if(ret.intval<BATT_TEMP_LOW || ret.intval>BATT_TEMP_HIGH ){
		chip->batt_temperture = DEFAULT_BATT_TEMP;
	}else{
		chip->batt_temperture = ret.intval;
	}
	
	return chip->batt_temperture ;
}

 static int  lc709203f_set_batt_temperture(struct lc709203f_chip* chip,int batt_temp )
{
	//printk ("%s: battery_temperture ==%d \n", __func__,batt_temp);
	return lc709203f_write_reg(chip->client,  BATTERY_TEMPERTURE,LC709203F_BASE_TEMPERTURE+batt_temp);		
}

#define CONSECUTIVE_COUNT	3

static int check_recharge_condition(struct lc709203f_chip* chip)
{
	static int count = 0;
	int ret = 0;
	
	printk ("%s:  count==%d \n", __func__,count);

	//batt_soc = lc709203f_update_soc(chip);
	if(chip->soc<chip->recharge_soc){
		count++;
	}else{
		count = 0;
	}
	
	if(count == CONSECUTIVE_COUNT){
		pr_info("start of recharging\n");
		if(chip->chg_done ){
#ifdef CONFIG_ZTEMT_BQ24296_CHARGE				
			union power_supply_propval enable_val = {1,};
			int rc = 0;
			if (!chip->bq24296_batt_psy) {
				chip->bq24296_batt_psy = power_supply_get_by_name("bq24296-battery");
			}
			if (!chip->bq24296_batt_psy) {
				printk("%s:  bq24296-battery supply not found; defer probe\n",__func__);
			}
			if(chip->bq24296_batt_psy){
				rc = chip->bq24296_batt_psy->set_property(chip->bq24296_batt_psy,
					POWER_SUPPLY_PROP_RESTART_CHARGING, &enable_val);
			}
			if(!rc){
				printk("%s:  psy changed batt_psy\n",__func__);
				chip->recharge_flag = TRUE;
				count = 0;
				return TRUE;
			}
#endif			
		}	
	} else {
		count++;
	}
	return ret;
}

static void lc709203f_update_soc_work(struct work_struct *work)
{
	struct lc709203f_chip *chip;
	int power_supply_change = 0;
    	int batt_soc,batt_temp,batt_mv;
	int last_batt_soc;
	int new_batt_status,new_batt_health;
	int usb_in;
	int rc;
	static bool once_flag = TRUE;
	
	chip = container_of(work, struct lc709203f_chip, work.work);

#ifdef ZTEMT_UART_DEBUG_ENABLE
	if(debug_uart){
		console_printk[0] = 7;
		debug_uart--;
	}
#endif

	batt_soc = lc709203f_update_soc(chip);
	batt_mv = lc709203f_update_vcell(chip);
	usb_in   = lc709203f_is_charger_online();
	batt_temp = lc709203f_get_batt_temperture(chip);
	rc = lc709203f_set_batt_temperture(chip,batt_temp);
	new_batt_status = lc709203f_get_prop_batt_status(chip);
	if(once_flag){
		last_batt_soc = batt_soc;
		chip->soc = batt_soc;
		once_flag = FALSE;
	}
	else{
		last_batt_soc = chip->soc;
	}
	
	//the soc is 100% when  the usb/ac is being inserted ,the soc should be 100%
	if( usb_in && (last_batt_soc==100 )){
		batt_soc = 100;
	}

	//after charging done and stop charging,but  soc < 100 and charger is online, so set soc  to 100%
	if( chip->chg_done){	
		if( usb_in && (batt_soc>=95 && batt_soc<100)){
			batt_soc = 100;
		}
	}
	//if  charging done and soc =99 and not  stop charging ,to set the soc =99 when remove the usb 
	if(!usb_in&&(!chip->chg_done)&&(last_batt_soc==99 )&&(batt_soc == 100))
	{
		batt_soc = 99;
	}
	//if usb is false ,the soc should not increase 
	if((batt_soc>last_batt_soc)&&(!usb_in))
	{
		batt_soc = last_batt_soc;
	}
	
	if(last_batt_soc >= 0){
		if(chip->first_resume){
          		  chip->first_resume = 0;
			}
	}

	if(batt_soc != last_batt_soc){
		chip->soc = bound_soc(batt_soc);
		power_supply_change = 1;
	}
	
	if(chip->batt_status != new_batt_status){
		chip->batt_status = new_batt_status;
		power_supply_change = 1;
	}
	
	if(usb_in&&(chip->batt_status  == POWER_SUPPLY_STATUS_FULL))
	{
		lc709203f_set_recharge_flag(TRUE);
	}

	new_batt_health = lc709203f_get_prop_batt_health(chip);
	if(chip->batt_health != new_batt_health){
		chip->batt_health = new_batt_health;
		power_supply_change = 1;
	}

	if((usb_in) && (chip->chg_done)){
		if((new_batt_status== POWER_SUPPLY_STATUS_FULL)&&(batt_soc<chip->recharge_soc)){
			if(check_recharge_condition(chip)){
				power_supply_change = 1;
			}
		}
	}

	if(power_supply_change){
		update_power_supply(chip);
	}	

  	lc709203_alarm_voltage_work(&chip-> alarm_voltage_work);

	printk("%s: BATT soc=%d temp=%d vol=%d usb_in=%d batt_status =%d chg_done =%d\n",
			__func__,chip->soc,batt_temp,batt_mv,usb_in,chip->batt_status,chip->chg_done);
	
#ifdef ZTEMT_UART_DEBUG_ENABLE
	MAXLOG_INFO("BATT soc=%d temp=%d vol=%d usb_in=%d\n",chip->soc,batt_temp,batt_mv,usb_in);
	if (MAX_DEBUG < maxlog_level)
		lc709203f_dump_regs(chip);
#endif	

	schedule_delayed_work(&chip->work,\
			  msecs_to_jiffies(get_calculation_delay_ms(chip)));
}
		
static void lc709203_alarm_voltage_work(struct work_struct *work)
{
	struct lc709203f_chip *chip = container_of(work,
				struct lc709203f_chip, alarm_voltage_work);
	u32 voltage = 0;
	 	 
	voltage = lc709203f_update_vcell(chip);	 
	printk ("%s: voltage== %d\n",__func__,voltage);
	// if battery is very low (v_cutoff voltage + 20mv) hold
	// a wakelock untill soc = 0%
	
	if (voltage <= chip->alarm_voltage_threshold
			&& !wake_lock_active(&chip->low_voltage_wake_lock)) {
		pr_debug("voltage = %d low holding wakelock\n", voltage);
		wake_lock(&chip->low_voltage_wake_lock);
	} else if (voltage > chip->alarm_voltage_threshold
			&& wake_lock_active(&chip->low_voltage_wake_lock)) {
		pr_debug("voltage = %d releasing wakelock\n", voltage);
		wake_unlock(&chip->low_voltage_wake_lock);
	}
}

void	start_update_soc_work(struct lc709203f_chip *chip)
{
	lc709203f_update_soc_work(&chip->work.work);
	lc709203_calculate_initial_soc();
}

static irqreturn_t low_voltage_alarm_irq_handler(int irq, void *dev_chip)
{
	struct lc709203f_chip* chip =  dev_chip;
	
	disable_irq_nosync(chip->irq);
	schedule_work(&chip->alarm_voltage_work);
	enable_irq(chip->irq);	
	
	return IRQ_HANDLED;
}


#define OF_PROP_READ_U32(chip,chip_prop, qpnp_node_property, retval)		\
do {									\
	if (retval)							\
		break;							\
	retval = of_property_read_u32(chip->dev->of_node,		\
				"qcom,"qpnp_node_property,		\
					&chip->chip_prop);		\
	if (retval) {							\
		pr_err("Error reading " #qpnp_node_property		\
					" property %d\n", retval);	\
	}								\
} while (0)

#define OF_PROP_READ_GPIO(chip,chip_prop, qpnp_node_property, retval)		\
do {									\
	chip->chip_prop = of_get_named_gpio(chip->dev->of_node,		\
				"qcom,"qpnp_node_property,		\
					retval);		\
								\
} while (0)

static int lc709203f_read_dt_props(struct lc709203f_chip *chip)
{
	int rc = 0;  
	
	OF_PROP_READ_U32(chip,low_soc_calculate_soc_ms, "low-soc-calculate-soc-ms",rc);
	OF_PROP_READ_U32(chip,low_voltage_calculate_soc_ms, "low-voltage-calculate-soc-ms",rc);
	OF_PROP_READ_U32(chip,calculate_soc_ms, "calculate-soc-ms",rc);
	OF_PROP_READ_U32(chip,alarm_soc_threshold, "soc-low-threshold",rc);
	OF_PROP_READ_U32(chip,alarm_voltage_threshold, "voltage-low-threshold",rc);
	OF_PROP_READ_U32(chip,low_soc_calculate_soc_threshold, "low-soc-calculate-soc-threshold",rc);
	OF_PROP_READ_U32(chip,recharge_soc, "resume-soc",rc);
		
	OF_PROP_READ_GPIO(chip,irq_gpio, "lc709203f-irq",0);

	printk("low_soc_calculate_soc_ms=%d--low_voltage_calculate_soc_ms=%d--calculate_soc_ms=%d\n alarm_soc_threshold =%d--alarm_voltage_threshold=%d--low_soc_calculate_soc_threshold=%d--recharge_soc=%d--irq_gpio=%d\n",\
		chip->low_soc_calculate_soc_ms,chip->low_voltage_calculate_soc_ms,\
		chip->calculate_soc_ms,chip->alarm_soc_threshold,chip->alarm_voltage_threshold,\
		chip->low_soc_calculate_soc_threshold,chip->recharge_soc,chip->irq_gpio);

	return rc;
}

static int lc709203_pinctrl_select(struct lc709203f_chip *chip)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state =  chip->gpio_state_default;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(chip->fg_pinctrl, pins_state);
		if (ret) {
			dev_err(&chip->client->dev,
				"can not set %s pins\n",
				 "gpio_state_default" );
			return ret;
		}
	} else {
		dev_err(&chip->client->dev,
			"not a valid '%s' pinstate\n",
				"gpio_state_default" );
	}

	return 0;

}

static int lc709203_pinctrl_init(struct lc709203f_chip *chip)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	chip->fg_pinctrl = devm_pinctrl_get(&(chip->client->dev));
	if (IS_ERR_OR_NULL(chip->fg_pinctrl)) {
		dev_dbg(&chip->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(chip->fg_pinctrl);
		chip->fg_pinctrl = NULL;
		return retval;
	}

	chip->gpio_state_default
		= pinctrl_lookup_state(chip->fg_pinctrl,
			"lc709203f_int_default");
	if (IS_ERR_OR_NULL(chip->gpio_state_default)) {
		dev_dbg(&chip->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(chip->gpio_state_default);
		chip->fg_pinctrl = NULL;
		return retval;
	}

	lc709203_pinctrl_select(chip);
		
	return 0;
}

static int  lc709203f_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct lc709203f_chip *chip = NULL;
	int ret;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
		return -EIO;
	}	

	chip = kzalloc(sizeof(struct lc709203f_chip),GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate lc709203f_chip\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);    
 	lc709203f_read_dt_props(chip);
	
	INIT_DELAYED_WORK(&chip->work, lc709203f_update_soc_work);
	wake_lock_init(&chip->low_voltage_wake_lock, WAKE_LOCK_SUSPEND,
			"low_voltage_lock");
	INIT_WORK(&chip->alarm_voltage_work,lc709203_alarm_voltage_work);
	lc709203_pinctrl_init(chip);
	
	if (gpio_is_valid(chip->irq_gpio)) {
		ret = gpio_request(chip->irq_gpio, "lc709203f_irq");
		if (unlikely(ret < 0)) {
			dev_err(&client->dev, "gpio request failed");
		}
		ret = gpio_direction_input(chip->irq_gpio);
		 
		chip->irq = gpio_to_irq(chip->irq_gpio);	
		if (unlikely(chip->irq < 0)) {
			dev_err(&client->dev, "gpio request to isr failed");
			return -ENODEV;
		}

		ret = request_irq(chip->irq, low_voltage_alarm_irq_handler,
				IRQF_TRIGGER_FALLING|IRQF_ONESHOT, "lc709203_irq", chip);
		if (unlikely(ret < 0)) {
			dev_err(&client->dev, "request_irq failed\n");	
			goto err_irq;
		}
		
	}

#ifdef lc709203_debug
	ret = device_create_file(&chip->client->dev, &dev_attr_batt_por);
	if (unlikely(ret < 0)) {
		dev_err(&chip->client->dev, "failed: cannot create power_lost.\n");
	}
	max_create_debugfs_entries(chip);
#endif

 	chip->batt_psy = power_supply_get_by_name("battery");
       if (!chip->batt_psy) {
               pr_err("batt_psy supply not found deferring probe\n");
       }
	global_chip = chip;

	lc709203f_check_battery_removed(chip);
	lc709203f_init( chip);
	start_update_soc_work(chip);
	return 0;
	
err_irq:
	free_irq(chip->irq, chip);
	gpio_free(chip->irq_gpio);	
	wake_lock_destroy(&chip->low_voltage_wake_lock);
	cancel_work_sync(&chip->alarm_voltage_work );
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

static int lc709203f_remove(struct i2c_client *client)
{	
	struct lc709203f_chip *chip = i2c_get_clientdata(client);

	free_irq(chip->irq, chip);
	gpio_free(chip->irq_gpio);	
	cancel_delayed_work(&chip->work);
	cancel_work_sync(&chip->alarm_voltage_work );
	wake_lock_destroy(&chip->low_voltage_wake_lock);
	kfree(chip);	
	global_chip = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void lc709203f_disable_irq(struct lc709203f_chip *chip)
{
	if(chip->irq_enabled){
		chip->irq_enabled = false;
		disable_irq_wake(chip->irq);
	}
}

static void lc709203f_enable_irq(struct lc709203f_chip *chip)
{
	if(!chip->irq_enabled){
		chip->irq_enabled = true;
		enable_irq_wake(chip->irq);
	}
}


static int lc709203f_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lc709203f_chip *chip = i2c_get_clientdata(client);

#ifdef ZTEMT_UART_DEBUG_ENABLE
	MAXLOG_DEBUG(" goto suspend.\n");
#endif
	chip->is_sleep = 1;	
	cancel_delayed_work_sync(&chip->work);
	lc709203f_enable_irq(chip);
	return 0;
};

static int lc709203f_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lc709203f_chip *chip = i2c_get_clientdata(client);

	chip->is_sleep = 0; 	
	chip->first_resume = 1;
	lc709203f_disable_irq(chip);		
	schedule_delayed_work(&chip->work, msecs_to_jiffies(LC709203F_DELAY));
	return 0;
}
static SIMPLE_DEV_PM_OPS(lc709203f_pm_ops, lc709203f_suspend, lc709203f_resume);
#define LC709203F_PM_OPS (&lc709203f_pm_ops)

#else

#define LC709203F_PM_OPS NULL

#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id lc709203f_id[] = {
	{ "lc709203f", 0 },
};
MODULE_DEVICE_TABLE(i2c, lc709203f_id);

static struct i2c_driver lc709203f_i2c_driver = {
	.driver	= {
		.name	= "lc709203f",
		.pm	=  LC709203F_PM_OPS,
	},
	.probe		= lc709203f_probe,
	.remove		= lc709203f_remove,
	.id_table	= lc709203f_id,
};
module_i2c_driver(lc709203f_i2c_driver);



MODULE_DESCRIPTION("LC709203F Fuel Gauge");
MODULE_AUTHOR("ztemt-RG");
MODULE_LICENSE("GPL");

#endif  



