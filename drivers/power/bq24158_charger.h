#ifndef __LINUX_BQ24194_CHARGER_H__
#define __LINUX_BQ24194_CHARGER_H__

#include <linux/usb/msm_hsusb.h>

//regs
#define BQ24158_REG_STATUS      	0x00
#define BQ24158_REG_CONTROL		    0x01
#define BQ24158_REG_VOL_CNTL		0x02
#define BQ24158_REG_VISION   		0x03
#define BQ24158_REG_CURRENT  		0x04
#define BQ24158_REG_CHARGER  		0x05
#define BQ24158_REG_LIMIT		    0x06


//REG 0x0
#define BQ_TMR_OTG_MASK		        BIT(7)
#define BQ24158_TMR_OTG_SHIFT		7

#define BQ_EN_STAT_MASK		        BIT(6)
#define BQ24158_EN_STAT_SHIFT		6

#define BQ_CHG_STAT_MASK		    (BIT(5)|BIT(4))
#define BQ24158_STAT_SHIFT		    4

#define BQ_BOOST_MASK		        BIT(3)
#define BQ24158_BOOST_SHIFT		    3

#define BQ_FAULT_MASK		        (BIT(2)|BIT(1)|BIT(0))
#define BQ24158_FAULT_SHIFT		    0

//REG 0x1
#define BQ_CURRENT_LIMIT_MASK		    (BIT(7)|BIT(6))
#define BQ24158_CURRENT_LIMIT_SHIFT		6

#define BQ_WEAK_VOL_MASK		        (BIT(5)|BIT(4))
#define BQ24158_WEAK_VOL_SHIFT		    4

#define BQ_TE_MASK		                BIT(3)
#define BQ24158_TE_SHIFT		        3

#define BQ_CE_MASK		                BIT(2)
#define BQ24158_CE_SHIFT		        2

#define BQ_HZ_MODE_MASK		            BIT(1)
#define BQ24158_HZ_MODE_SHIFT		    1

#define BQ_OPA_MODE_MASK		        BIT(0)
#define BQ24158_OPA_MODE_SHIFT		    0

//REG 0x2
#define BQ_BATT_REGULAT_MASK		    (BIT(7)|BIT(6)|BIT(5)|BIT(4)|BIT(3)|BIT(2))
#define BQ24158_BATT_REGULAT_SHIFT		2
#define BQ24158_CHG_MIN_VBATT	       3500
#define BQ24158_CHG_MAX_VBATT	       4440
#define BQ24158_CHG_VBATT_STEP_MV	   20


#define BQ_OTG_PL_MASK		            BIT(1)
#define BQ24158_OTG_PL_SHIFT		    1

#define BQ_OTG_EN_MASK		            BIT(0)
#define BQ24158_OTG_EN_SHIFT		    0

//REG 0x3
#define BQ_VENDER_MASK		        (BIT(7)|BIT(6)|BIT(5))
#define BQ24158_VENDER_SHIFT		5

#define BQ_PN_MASK		            (BIT(4)|BIT(3))
#define BQ24158_PN_SHIFT		     3

#define BQ_REVISION_MASK		    (BIT(2)|BIT(1)|BIT(0))
#define BQ24158_REVISION_SHIFT	     0 

//REG 0x4
#define BQ_RESET_MASK		         BIT(7)
#define BQ24158_RESET_SHIFT		     7

#define BQ_CHG_FAST_CURRENT_MASK		(BIT(6)|BIT(5)|BIT(4))
#define BQ24158_FAST_CURRENT_SHIFT		4

#define BQ_CHG_TERM_CURRENT_MASK		(BIT(2)|BIT(1)|BIT(0))

//REG 0x5
#define BQ_LOW_CHG_MASK		            BIT(5)
#define BQ24158_LOW_CHG_SHIFT		    5

#define BQ_DPM_STATE_MASK		        BIT(4)
#define BQ24158_DPM_STATE_SHIFT		    4

#define BQ_CD_STATE_MASK		        BIT(3)
#define BQ24158_CD_STATE_SHIFT		    3

#define BQ24158_SPEC_CHG_MIN_VOL	   4200
#define BQ24158_SPEC_CHG_MAX_VOL	   4760
#define BQ24158_SPEC_CHG_STEP	       80

#define BQ_VSREG_MASK		            (BIT(2)|BIT(1)|BIT(0))

//REG 0x5
#define BQ_SAFE_CURRENT_MASK		    (BIT(7)|BIT(6)|BIT(5)|BIT(4))
#define BQ24158_SAFE_CURRENT_SHIFT		 4

#define BQ_SAFE_VBATT_MASK		        (BIT(3)|BIT(2)|BIT(1)|BIT(0))
#define BQ24158_SAFE_MAX_VBATT   		4440
#define BQ24158_SAFE_MIN_VBATT   		4200
#define BQ24158_SAFE_VBATT_STEP   		20

#define BQ_AC_CHG_I 		    1000
#define BQ_USB_CHG_I		    500

//0.1 degree   eg:470 = 47C
#define CHG_TEMP_MAX    800
#define CHG_TEMP_HOT    500
#define CHG_TEMP_WARM   450
#define CHG_TEMP_GOOD   230
#define CHG_TEMP_COOL   100
#define CHG_TEMP_COLD   -50
#define CHG_TEMP_MIN   -300

enum opa_mode {
	OPA_CHARGER,
	OPA_BOOST,
};

enum bq_chg_status{
	BQ_READY_CHGING,
	BQ_IN_CHGING,
	BQ_CHGING_DONE,
	BQ_CHGING_FAULT,
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

struct batt_status_map {
	int low_temp;
	int high_temp;
	enum bq_batt_status batt_st;
	int batt_current;
};

int  bq24158_get_chg_status(void);

int  bq24158_set_chg_status(int usb_in);

int  bq24158_notify_charger(enum usb_chg_type chg_type);

int bq24158_get_batt_stauts(void);

int  bq24158_is_boost_mode(void);

int  bq24158_is_charge_enable(void);

int bq_prop_batt_status(void);

int bq_prop_charging_type(void);

int bq_prop_batt_health(void);

#endif
