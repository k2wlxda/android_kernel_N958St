#ifndef __LINUX_BQ24296_CHARGER_H__
#define __LINUX_BQ24296_CHARGER_H__

#include <linux/usb/msm_hsusb.h>

//regs
#define BQ24296_REG_SOURCE_CNTL      					0x00		//default 0x6x or 0x3x
#define BQ24296_REG_POWER_ON_CONFIG		   			0x01		//default 0x1B
#define BQ24296_REG_FAST_CURRENT_CNTL				0x02		//default 0x60
#define BQ24296_REG_PRE_AND_TERM_CURRENT_CNTL		0x03		//default 0x11
#define BQ24296_REG_CHARGE_VOLTAGE_CNTL  			0x04		//default 0xB2
#define BQ24296_REG_TIMER_CNTL 						0x05		//default 0x9A
#define BQ24296_REG_BOOST_AND_THERM_CNTL		    	0x06		//default 0x73
#define BQ24296_REG_OPERATION_CNTL		    			0x07		//default 0x4B
#define BQ24296_REG_SYSTEM_STATUS		    			0x08		
#define BQ24296_REG_FAULT_STATUS		    				0x09		


//REG 0x0
#define BQ24296_HZ_MODE_MASK		           		BIT(7)
#define BQ24296_HZ_MODE_SHIFT		    			7
	
#define BQ24296_INPUT_VOLT_LIMIT_MASK		BIT(6)|BIT(5)|BIT(4)|BIT(3)
#define BQ24296_INPUT_VOLT_LIMIT_SHIFT		3
	
#define BQ24296_INPUT_CURR_LIMIT_MASK		BIT(2)|BIT(1)|BIT(0)
#define BQ24296_INPUT_CURR_LIMIT_SHIFT		0


//REG 0x1
#define BQ24296_REG_RESET_MASK		            	BIT(7)
#define BQ24296_REG_RESET_SHIFT				7

#define BQ24296_WD_TIMER_RESET_MASK		       BIT(6)
#define BQ24296_WD_TIMER_RESET_SHIFT			6

#define BQ24296_OTG_EN_MASK		            		BIT(5)
#define BQ24296_OTG_EN_SHIFT					5

#define BQ24296_CHG_EN_MASK		            		BIT(4)
#define BQ24296_CHG_EN_SHIFT					4

#define BQ24296_MIN_SYS_VOLT_LIMIT_MASK		BIT(3)|BIT(2)|BIT(1)
#define BQ24296_MIN_SYS_VOLT_LIMIT_SHIFT		1

#define BQ24296_BOOST_LIMIT_MASK				BIT(0)
#define BQ24296_BOOST_LIMIT_SHIFT				0


//REG 0x2
#define BQ24296_FAST_CHG_CURR_LIMIT_MASK	BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2)
#define BQ24296_FAST_CHG_CURR_LIMIT_SHIFT	2
	
#define BQ24296_BOOST_COLD_MASK				BIT(1)
#define BQ24296_BOOST_COLD_SHIFT				1
	
#define BQ24296_FORCE_20PCT_MASK				BIT(0)
#define BQ24296_FORCE_20PCT_SHIFT				0


//REG 0x3
#define BQ24296_PRE_CHG_CURR_LIMIT_MASK		BIT(7)|BIT(6)|BIT(5)|BIT(4)
#define BQ24296_PRE_CHG_LIMIT_SHIFT			4
		
#define BQ24296_TERM_CURR_LIMIT_MASK		BIT(2)|BIT(1)|BIT(0)
#define BQ24296_TERM_CURR_LIMIT_SHIFT		0


//REG 0x4
#define BQ24296_CHG_VOLT_LIMIT_MASK			BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2)
#define BQ24296_CHG_VOLT_LIMIT_SHIFT			2
		
#define BQ24296_WEAK_VOLT_MASK				BIT(1)
#define BQ24296_WEAK_VOLT_SHIFT				1
		
#define BQ24296_RECHG_THRESHOLD_MASK		BIT(0)
#define BQ24296_RECHG_THRESHOLD_SHIFT		0
		
//REG 0x5		
#define BQ24296_CHG_TERM_EN_MASK			BIT(7)
#define BQ24296_CHG_TERM_EN_SHIFT			7
			
#define BQ24296_WD_TIMER_MASK				BIT(5)|BIT(4)
#define BQ24296_WD_TIMER_SHIFT				4
			
#define BQ24296_CHG_TIMER_EN_MASK			BIT(3)
#define BQ24296_CHG_TIMER_EN_SHIFT			3
			
#define BQ24296_CHG_TIMER_MASK				BIT(2)|BIT(1)
#define BQ24296_CHG_TIMER_SHIFT				1
			
		
//REG 0x6		
#define BQ24296_BOOST_VOLT_MASK				BIT(7)|BIT(6)|BIT(5)|BIT(4)
#define BQ24296_BOOST_VOLT_SHIFT				4
		
#define BQ24296_BOOST_TEMP_MASK				BIT(3)|BIT(2)
#define BQ24296_BOOST_TEMP_SHIFT				2
		
#define BQ24296_BOOST_TEMP_THRESHOLD_MASK			BIT(1)|BIT(0)
#define BQ24296_BOOST_TEMP_THRESHOLD_SHIFT			0


//REG 0x7
#define BQ24296_DPDM_DETECTION_EN_MASK		BIT(7)
#define BQ24296_DPDM_DETECTION_EN_SHIFT		7
		
#define BQ24296_TMR2X_EN_MASK				BIT(6)
#define BQ24296_TMR2X_EN_SHIFT				6
		
#define BQ24296_BATFET_EN_MASK				BIT(5)
#define BQ24296_BATFET_EN_SHIFT				5
		
#define BQ24296_INT_CHG_FAULT_MASK			BIT(1)
#define BQ24296_INT_CHG_FAULT_SHIFT			1
			
#define BQ24296_INT_BAT_FAULT_MASK			BIT(0)
#define BQ24296_INT_BAT_FAULT_SHIFT			0
			
			
//REG 0x8			
#define BQ24296_VBUS_STATUS_MASK				BIT(7)|BIT(6)
#define BQ24296_VBUS_STATUS_SHIFT				6
			
#define BQ24296_CHG_STATUS_MASK				BIT(5)|BIT(4)
#define BQ24296_CHG_STATUS_SHIFT				4
			
#define BQ24296_DPM_STATUS_MASK				BIT(3)
#define BQ24296_DPM_STATUS_SHIFT				3
			
#define BQ24296_POWER_STATUS_MASK			BIT(2)
#define BQ24296_POWER_STATUS_SHIFT			2
			
#define BQ24296_TEMP_STATUS_MASK				BIT(1)
#define BQ24296_TEMP_STATUS_SHIFT				1
		
#define BQ24296_VSYS_STATUS_MASK				BIT(0)
#define BQ24296_VSYS_STATUS_SHIFT				0


//REG 0x9
#define BQ24296_WD_FAULT_MASK				BIT(7)
#define BQ24296_WD_FAULT_SHIFT				7

#define BQ24296_OTG_FAULT_MASK				BIT(6)
#define BQ24296_OTG_FAULT_SHIFT				6

#define BQ24296_CHG_FAULT_MASK				BIT(5)|BIT(4)
#define BQ24296_CHG_FAULT_SHIFT				4

#define BQ24296_BAT_FAULT_MASK				BIT(3)
#define BQ24296_BAT_FAULT_SHIFT				3

#define BQ24296_NTC_FAULT_MASK				BIT(1)|BIT(0)
#define BQ24296_NTC_FAULT_SHIFT				0


#define BQ_AC_CHG_I 		    1000
#define BQ_USB_CHG_I		    500

//0.1 degree   ex:470 = 47C
#define CHG_TEMP_MAX    800
#define CHG_TEMP_HOT    500
#define CHG_TEMP_WARM   450
#define CHG_TEMP_GOOD   230
#define CHG_TEMP_COOL2   100
#define CHG_TEMP_COLD   -30
#define CHG_TEMP_MIN   -300
 
enum chg_config {
	UNKNOW_CONFIG = 0,
	CHG_USB_CONFIG ,
	CHG_AC_CONFIG ,
	OTG_CONFIG ,
	
};

enum input_current_limit{
	IUSB_100mA   ,
	IUSB_150mA,	
	IUSB_500mA   ,
	IUSB_900mA   ,	
	IUSB_1000mA ,
	IUSB_1500mA ,
	IUSB_2000mA ,
	IUSB_3000mA ,
	IUSB_MAX,
};

enum ibatt_current{
	IBATT_512mA   ,
	IBATT_768mA   ,	
	IBATT_1024mA ,
	IBATT_1536mA ,
	IBATT_2048mA ,
	IBATT_3008mA ,
	IBATT_MAX,
};


#define BQ24296_CHG_MIN_VBATT	       			3504
#define BQ24296_CHG_MAX_VBATT	       		4400
#define BQ24296_CHG_VBATT_STEP_MV			16	
enum vbatt_max{
	VBATT_4336mV ,
	VBATT_4352mV ,
	VBATT_4368mV ,
	VBATT_4384mV ,
	VBATT_4400mA ,
	VBATT_MAX,
};
//static int bq_vbatt_max_limit [VBATT_MAX]={0x34,0x35,0x36,0x37,0x38};
//static int vbatt_max_voltage [VBATT_MAX]  ={4336,4352,4368,4384,4400};

#define BQ24296_CHG_MIN_VCHG	   		3880
#define BQ24296_CHG_MAX_VCHG	   		5080
#define BQ24296_CHG_VCHG_STEP_MV	       80
#define VCHG_VOLTAGE  					4360


enum wdog_timer{
	DISABLE_WD_TIMER ,
	DELAY_40_S ,
	DELAY_80_S ,
	DELAY_160_S ,
	DELAY_MAX,
};

enum chg_safety_timer{
	CHG_SAFETY_TIME_5_H ,
	CHG_SAFETY_TIME_8_H ,
	CHG_SAFETY_TIME_12_H ,
	CHG_SAFETY_TIME_20_H ,
	CHG_SAFETY_MAX,
};


enum bq_chg_status{
	BQ_READY_CHGING,
	BQ_IN_CHGING,
	BQ_CHGING_DONE,
	BQ_CHGING_FAULT,
};

enum sys_min_voltage{
	SYS_MIN_VOLTAGE_3000MV = 0,
	SYS_MIN_VOLTAGE_3100MV,
	SYS_MIN_VOLTAGE_3200MV,
	SYS_MIN_VOLTAGE_3300MV,
	SYS_MIN_VOLTAGE_3400MV,
	SYS_MIN_VOLTAGE_3500MV,
	SYS_MIN_VOLTAGE_UNKNOW,
};

enum boost_current{
	BOOST_CURRENT_1000mA = 0,
	BOOST_CURRENT_1500mA,
	BOOST_CURRENT_UNKNOW,
};

enum bq_batt_status{
	BATT_STATUS_COLD = 0,
	BATT_STATUS_COOL1,
	BATT_STATUS_COOL2,
	BATT_STATUS_GOOD,
	BATT_STATUS_WARM,
	BATT_STATUS_HOT,
	BATT_STATUS_UNKNOW,
};

struct temp_status_map {
	int low_temp;
	int high_temp;
	enum bq_batt_status batt_st;
	int batt_current;
};

//int  bq24296_get_prop_chg_status(void);

int  bq24296_start_chg_work(int usb_in);

int  bq24296_notify_charger(enum usb_chg_type chg_type);

//int bq24296_get_batt_stauts(void);

//int  bq24296_is_boost_mode(void);

//int  bq24296_is_charge_enable(void);

//int  bq24296_get_prop_charge_type(void);

//int  bq24296_get_prop_batt_health(void);

#endif

