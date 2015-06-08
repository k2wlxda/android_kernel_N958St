#ifndef ZTE_LCD_CHECK_STATUS_H
#define ZTE_LCD_CHECK_STATUS_H

#include <linux/list.h>
#include <linux/mdss_io_util.h>
#include <linux/irqreturn.h>
#include <linux/pinctrl/consumer.h>

#include "mdss_panel.h"
#include "mdss_dsi_cmd.h"


#define MAX_PANEL_NAME_LEN 64

struct lcd_check_status_panel {	
    char panel_name[MAX_PANEL_NAME_LEN];
	int (*check_status) (struct mdss_dsi_ctrl_pdata *pdata);
};

int zte_lcd_reg_check(struct mdss_dsi_ctrl_pdata *ctrl, u8 reg_addr, u8 *reg_read_buf, u8 *reg_key_buf, u8 buf_len);
int zte_lcd_check_status_tianma_hx8394d_720p_video(struct mdss_dsi_ctrl_pdata *pdata);
int zte_lcd_check_status_oufei_hx8394d_720p_video(struct mdss_dsi_ctrl_pdata *pdata);
int zte_lcd_check_status_init(struct mdss_dsi_ctrl_pdata *pdata);
int zte_reg_buf_compare(u8 *buf_read, u8 *buf_key, u8 len);

#endif //ZTE_LCD_CHECK_STATUS_H
