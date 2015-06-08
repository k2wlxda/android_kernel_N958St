/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>

#include "mdss_dsi.h"
#include "zte_lcd_check_status.h"
#include "zte_lcd_log.h"
#include "zte_lcd_dsi.h"

//==========================================================================================================
/*
 *lcd check status arrary
 */
static struct lcd_check_status_panel lcd_check_status_panels[] = {
	{"tianma hx8394d 720p video mode dsi panel",    zte_lcd_check_status_tianma_hx8394d_720p_video},
	{"oufei hx8394d 720p video mode dsi panel",     zte_lcd_check_status_oufei_hx8394d_720p_video},
};
//==========================================================================================================



//==========================================================================================================
/*
 * funtion
 *      check lcd work status
 * input
 *      @pdata - the pointer to panel platform data
 * output
 *      NULL
 * return 
 *      @true  - check success
 *      @false - check failed
 */
int zte_lcd_check_status_default(struct mdss_dsi_ctrl_pdata *pdata)
{
    LCD_LOG_DEBUG("lcd check success\n");
    return true;     
}

int zte_lcd_check_status_tianma_hx8394d_720p_video(struct mdss_dsi_ctrl_pdata *pdata)
{
    u8 reg_addr = 0xD9;
    u8 reg_data_len   = 1;
    u8 reg_buf_read[1] = {0x00};
    u8 reg_buf_key [1] = {0x80}; 
    bool check_result= false;

    check_result = zte_lcd_reg_check(pdata, reg_addr, reg_buf_read, reg_buf_key, reg_data_len);
    if (false==check_result)
    {
        goto zte_lcd_check_status_tianma_hx8394d_720p_video_error;
    }

    LCD_LOG_DEBUG("lcd check success\n");
    return true;

zte_lcd_check_status_tianma_hx8394d_720p_video_error:
    LCD_LOG_ERROR("lcd check failed\n");
    return false;     
}

int zte_lcd_check_status_oufei_hx8394d_720p_video(struct mdss_dsi_ctrl_pdata *pdata)
{   
    bool check_result= false;

    u8 reg_buf_read_1[1] = {0x00};
    u8 reg_buf_read_2[2] = {0x00, 0x00};

    u8 reg_addr_1 = 0xD9;
    u8 reg_addr_2 = 0x09;

    u8 reg_data_len_1   = 1;
    u8 reg_data_len_2   = 2;

    u8 reg_buf_key_1 [1] = {0x80}; 
    u8 reg_buf_key_2 [2] = {0x80, 0x73};  

    check_result = zte_lcd_reg_check(pdata, reg_addr_1, reg_buf_read_1, reg_buf_key_1, reg_data_len_1);
    if (false==check_result)
    {
        goto zte_lcd_check_status_oufei_hx8394d_720p_video_error;
    }

    check_result = zte_lcd_reg_check(pdata, reg_addr_2, reg_buf_read_2, reg_buf_key_2, reg_data_len_2);
    if (false==check_result)
    {
        goto zte_lcd_check_status_oufei_hx8394d_720p_video_error;
    }

    LCD_LOG_DEBUG("lcd check success\n");
    return true;
    
zte_lcd_check_status_oufei_hx8394d_720p_video_error:
    LCD_LOG_ERROR("lcd check failed\n");
    return false; 

}
//==========================================================================================================


int zte_lcd_check_status_init(struct mdss_dsi_ctrl_pdata *pdata)
{
    int i = 0;
    for (i=0; i<ARRAY_SIZE(lcd_check_status_panels); i++)
    {
        if (!strcmp(pdata->panel_name, lcd_check_status_panels[i].panel_name))
        {
            pdata->check_status = lcd_check_status_panels[i].check_status;
            goto zte_esd_check_init_success;
        }
    }

    pdata->check_status = zte_lcd_check_status_default;
    LCD_LOG_ERROR("failed\n");
    return -1;

zte_esd_check_init_success:  
	LCD_LOG_DEBUG("success panel_name is %s\n",lcd_check_status_panels[i].panel_name);
    return 0;

}

int zte_reg_buf_compare(u8 *buf_read, u8 *buf_key, u8 buf_len)
{
    return (0 == strncmp(buf_read, buf_key, buf_len))? true : false;
} 

int zte_lcd_reg_check(struct mdss_dsi_ctrl_pdata *ctrl, u8 reg_addr, u8 *reg_read_buf, u8 *reg_key_buf, u8 buf_len)
{
    zte_dsi_panel_reg_read(ctrl, reg_addr, reg_read_buf, buf_len);
    return zte_reg_buf_compare(reg_read_buf, reg_key_buf, buf_len);
}


