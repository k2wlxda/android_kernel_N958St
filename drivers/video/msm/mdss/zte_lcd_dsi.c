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
#include "zte_lcd_dsi.h"
#include "zte_lcd_log.h"

static int zte_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;
    static char dcs_cmd[2] = {0x00, 0x00}; /* DTYPE_DCS_READ */
    static struct dsi_cmd_desc dcs_read_cmd = {
        {DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
        dcs_cmd
    };

    u8 *rbuf_temp = kzalloc(len+5, 0);

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len+5;
	cmdreq.rbuf = rbuf_temp;
	cmdreq.cb = fxn; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

    memcpy(rbuf, rbuf_temp, len);
    
    kfree(rbuf_temp);

	return 0;
}

#if 0
static void zte_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}
#endif

static int zte_dsi_panel_set_return_len(struct mdss_dsi_ctrl_pdata *ctrl, u8 read_len)
{
	struct dcs_cmd_req cmdreq;
    static char dcs_cmd[1] = {0x00};    /* DTYPE_MAX_PKTSIZE */
    static struct dsi_cmd_desc set_return_len_cmd = {
        {DTYPE_MAX_PKTSIZE, 1, 0, 0, 5, sizeof(dcs_cmd)},
        dcs_cmd
    };

    dcs_cmd[0] = read_len;

    memset(&cmdreq, 0, sizeof(cmdreq));
    cmdreq.cmds = &set_return_len_cmd;
    cmdreq.cmds_cnt = 1;
    cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
    cmdreq.rlen = 0;
    cmdreq.cb = NULL;

    mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return 0;
}

int zte_dsi_panel_reg_read(struct mdss_dsi_ctrl_pdata *ctrl, u8 reg_addr, u8 *reg_read_buf, u8 buf_len)
{
    unsigned char i = 0;

    zte_dsi_panel_set_return_len(ctrl, buf_len);

	zte_dsi_panel_cmd_read(ctrl, reg_addr, 0x00, NULL, reg_read_buf, buf_len);

    for(i=0; i<buf_len; i++)
    {
        LCD_LOG_DEBUG("red[0x%X]-->buf[%d] = 0x%X ==>> %d\n", reg_addr, i, reg_read_buf[i], reg_read_buf[i]);
    }

	return 0;
}

