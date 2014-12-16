/*******************************************************************************
*                                                                                              *
*       File Name:      taos_common.h                                               *
*       Description:    Common file for ioctl and configuration definitions. *
*       		Used by kernel driver and driver access applications.       *
*       		Please include this file, and <sys/ioctl.h> in your            *
*                       driver access application program source.	             *
*       Author:         John Koshi                                                       *
*       History:        09/16/2009 - Initial creation                               *
*       		02/07/2010 - Add proximity			                          *
*                                                                                               *
********************************************************************************
*       Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074  *
*******************************************************************************/
#include <linux/wakelock.h>
#include <linux/sensors.h>

// device id/address
#define TAOS_DEVICE_NAME                "tritonFN"
#define TAOS_ID_NAME_SIZE               10
#define TMD2772_DEVICE_ID               0x39

// Triton register 
#define TAOS_TRITON_CNTRL               0x00
#define TAOS_TRITON_ALS_TIME            0x01
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_INTERRUPT           0x0C
#define TAOS_TRITON_PRX_CFG             0x0D
#define TAOS_TRITON_PRX_COUNT           0x0E
#define TAOS_TRITON_GAIN                0x0F
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_PRX_OFFSET          0x1E

// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG             0x80
#define TAOS_TRITON_CMD_AUTO            0x20
#define TAOS_TRITON_CMD_WORD_BLK_RW     0x20
#define TAOS_TRITON_CMD_SPL_FN          0x60

// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL  0x20
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL  0x08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL  0x04
#define TAOS_TRITON_CNTL_ADC_ENBL       0x02
#define TAOS_TRITON_CNTL_PWRON          0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID     0x01

// lux constants parameters
#define ALS_MAX_LUX                     10000
#define ALS_FILTER_DEPTH                3
#define	ALS_ADC_TIME_PROX_ON	        0xF0
#define ALS_GAIN_DIVIDE                 1000
#define ALS_GAIN_1X                     0
#define ALS_GAIN_8X                     1
#define ALS_SCALE_FACTOR_DEFAULT        13
#define ALS_GAIN_DEFAULT                0
#define ALS_ADC_TIME_DEFAULT            0xF0
#define ALS_TIME_PARAM                  41
#define ALS_POLL_DELAY_DEFAULT          1000

//prox constants parameters
//#define PROX_LED_PULSE_CNT              12
#define PROX_THRESHOLD_DISTANCE         100
#define PROX_DATA_TARGET                150
#define PROX_DATA_MAX                   1023
#define PROX_OFFSET_CAL_BUFFER_SIZE     30
#define PROX_DATA_SAFE_RANGE_MIN        (PROX_DATA_TARGET - 50)
#define PROX_DATA_SAFE_RANGE_MAX        (PROX_DATA_TARGET + 350)
//#define PROX_OFFSET_CAL_PER_BIT         10/75
//#define PROX_OFFSET_CAL_PER_BIT         100/812

#define PROX_DEFAULT_THRESHOLD_HIGH     800
#define PROX_DEFAULT_THRESHOLD_LOW      700
#define PROX_THRESHOLD_HIGH_MAX         800
//#define PROX_THRESHOLD_HIGH_MIN         250
#define PROX_THRESHOLD_HIGH_MIN         500
//#define PROX_THRESHOLD_SAFE_DISTANCE    PROX_THRESHOLD_DISTANCE / 2
#define PROX_THRESHOLD_SAFE_DISTANCE    300

#define PROX_SCALE_FACTOR_DEFAULT       6
#define PROX_ADC_TIME_DEFAULT           0xFF
#define PROX_INTR_FILTER_DEFAULT        0x33
#define PROX_CONFIG_DEFAULT             0x00
#define PROX_GAIN_DEFAULT               0x20
#define PROX_CONFIG_OFFSET_DEFAULT      0x00

#define WAIT_TIME_DEFAULT               0xFF

//file saved
#define PATH_PROX_CAL_THRESHOLD         "/persist/proxdata/threshold"
#define PATH_PROX_OFFSET                "/persist/sensors/proximity/offset/proximity_offset"
#define PATH_PROX_UNCOVER_DATA          "/persist/sensors/proximity/uncover_data"

struct taos_wake_lock{
    struct wake_lock lock;
    bool   locked;
    char   *name;
};

// device configuration
struct taos_cfg
{
    u16	 prox_threshold_hi;
	u16  prox_threshold_lo;
	u8	 prox_adc_time;
	u16  prox_scale_factor;
 	u8	 prox_intr_filter;
	u8	 prox_config;
	u8	 prox_pulse_cnt;
	u8	 prox_gain;
	u8	 prox_config_offset;  
	
	u8	 wait_time;

    u16  als_adc_time;
    u16  als_scale_factor;
    u8   als_gain;
    unsigned int als_poll_delay;
};

struct taos_data {
	struct i2c_client *client;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
	struct taos_wake_lock proximity_wakelock;
	struct mutex lock;
	struct delayed_work als_poll_work;
	struct delayed_work prox_calibrate_work;
	struct delayed_work prox_offset_cal_work;
	struct hrtimer  prox_unwakelock_timer;
	struct input_dev *p_idev;
	struct input_dev *a_idev;

	struct device *proximity_dev;
	struct device *light_dev;

	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

	struct regulator	*vdd;
	struct regulator	*vio;

    struct taos_cfg *cfg;

	char *prox_name;
	char *als_name;
	bool prox_calibrate_flag;
	bool prox_calibrate_result;
	bool prox_offset_cal_result;

	bool prox_offset_cal_verify;
	bool prox_calibrate_verify;

	int  prox_calibrate_times;
	int  prox_thres_hi_max;
	int  prox_thres_hi_min;
	int  prox_data_max;
	int  prox_manual_calibrate_threshold;
	int  irq_pin_num;
	u8	 prox_uncover_data;

	char *chip_name;

	bool prox_on;
	bool als_on;
	bool irq_enabled;
	bool irq_work_status;
	int als_poll_time_mul;
};

// proximity data
struct taos_prox_info 
{
        u16 prox_clear;
        u16 prox_data;
        int prox_event;
};

// lux data
struct lux_data {
    u16 ratio;
    u16 clear;
    u16 ir;
};

// forward declarations
static int tmd2772_probe(struct i2c_client *clientp, const struct i2c_device_id *idp);
static int tmd2772_remove(struct i2c_client *client);
static int taos_suspend(struct i2c_client *client,pm_message_t mesg);
static int taos_resume(struct i2c_client *client);

static int taos_prox_on(void);
static int taos_prox_off(void);
static int taos_prox_poll(struct taos_prox_info *prxp);
static int taos_prox_threshold_set(void);
static int taos_interrupts_clear(void);
static int taos_prox_calibrate(void);
static void taos_prox_calibrate_work_func(struct work_struct *work);
static void taos_prox_offset_cal_work_func(struct work_struct *work);
static void taos_wakelock_ops(struct taos_wake_lock *wakelock, bool enable);
static int taos_write_cal_file(char *file_path,unsigned int value);
static int taos_read_cal_value(char *file_path);
static enum hrtimer_restart  taos_prox_unwakelock_work_func(struct hrtimer *timer);
static int taos_get_data(void);

static int taos_sensors_als_poll_on(void);
static int taos_sensors_als_poll_off(void);
static void taos_als_poll_work_func(struct work_struct *work);
static int taos_als_gain_set(unsigned als_gain);
static void taos_update_sat_als(void);
static int taos_als_get_data(void);
static int taos_get_lux(void);
static int taos_lux_filter(int raw_lux);

