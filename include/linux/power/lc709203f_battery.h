#ifndef  LC709203F_BATTERY_H
#define   LC709203F_BATTERY_H

#define	  LC709203F_INITIAL_RSOC						 0x0004	//0x0007	
#define     BATTERY_TEMPERTURE         			 			 0x0008
#define     BATTERY_VOLTAGE    				 			 0x0009
#define     CURRENT_DIRECTION    	     			 			 0x000A
#define     LC709203F_ADJUSTMENT_PACK_APPLI    			 0x000b
#define     LC709203F_ADJUSTMENT_PACK_THERMISTOR  	 0x000c
#define     BATTERY_RSOC   					 			 0x000d
#define	  LC709203F_INDICATOR_TO_EMPTY				 0x000f
#define     LC709203F_IC_VERSION   					 	 0x0011
#define     LC709203F_ALARM_LOWER_RSOC   				 0x0013
#define     LC709203F_ALARM_LOWER_VOLTAGE 			 0x0014
#define     LC709203F_POWER_MODE   			  			 0x0015


#define    LC709203F_APA_PARAM				 			0x0043	//0x0020
#define    LC709203F_BASE_TEMPERTURE		 			0x0AAC    //0бу
#define	 LC709203F_INITIAL_RSOC_DEFALUT_PARAM		0xAA55

struct pwr_on_param{
	int backup_soc;
	int backup_voltage;
	bool warm_reset;
	int current_ocv;
};

int 	 lc709203f_get_soc(void);
int 	 lc709203f_get_voltage(void);
void  lc709203f_set_recharge_flag(bool flag);
void  lc709203_calculate_initial_soc(void);

#endif
































