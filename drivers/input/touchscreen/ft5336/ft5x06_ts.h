#ifndef __LINUX_FT5X0X_TS_H__
#define __LINUX_FT5X0X_TS_H__

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	5

#define PRESS_MAX	0xFF
#define FT_PRESS	0x08

#define FT5X0X_NAME	"ft5x06_ts"

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FT5x0x_REG_FW_VER		0xA6
#define FT5x0x_REG_POINT_RATE	0x88
#define FT5X0X_REG_THGROUP		0x80
#define FT5x0x_REG_GEST_ONOFF	0xD0

#define FT5X06_ID		0x55
#define FT5X16_ID		0x0A
#define FT5X36_ID		0x14
#define FT6X06_ID		0x06

#define ZTEMT_WAKEUP_GESTURE    1    // gesture wakeup

/* The platform data for the Focaltech ft5x0x touchscreen driver */
struct ft5x0x_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;	/*default:IRQF_TRIGGER_FALLING*/
	unsigned int irq;
	unsigned int reset;
};

/*luochangyang For wakeup gesture 2014/04/29*/
struct fw_upgrade_info {
	bool auto_cal;
	u16 delay_aa;
	u16 delay_55;
	u8 upgrade_id_1;
	u8 upgrade_id_2;
	u16 delay_readid;
	u16 delay_erase_flash;
};

struct ft5x06_ts_platform_data {
	struct fw_upgrade_info info;
	const char *name;
	const char *fw_name;
	int (*power_init) (bool);
	int (*power_on) (bool);
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
	/*** ZTEMT Added by luochangyang, 2014/04/29 ***/
    bool irq_wake;
    bool wakeup_gesture;
    /***ZTEMT END***/
};
/*luochangyang END*/

int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);
void ft5x0x_reset_tp(int HighOrLow);

#endif
