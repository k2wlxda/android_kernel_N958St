#ifndef __ZTE_DISP_PREFERENCES_H__
#define __ZTE_DISP_PREFERENCES_H__

#include "mdss_dsi.h"
#include "mdss_mdp.h"

enum {
	INTENSITY_NORMAL = 24,
	INTENSITY_01,
	INTENSITY_02
};

enum {
	CABC_OFF = 0,
	CABC_LOW,
	CABC_NORMAL,
	CABC_HIGH
};


struct zte_enhance_type{
	int en_saturation;
	int en_colortmp;
	unsigned int saturation;
	unsigned int colortmp;
};

struct zte_enhance_type zte_get_lcd_enhance_param(void);
void zte_set_ctrl_point(struct mdss_dsi_ctrl_pdata * ctrl);
void zte_mipi_colortmp(void);
void zte_boot_begin_enhance(struct mdss_dsi_ctrl_pdata *ctrl);
void zte_mipi_disp_cabc(unsigned int state);

#endif

