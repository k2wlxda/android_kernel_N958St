#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include "mdss_dsi.h"
#include <linux/of.h>
#include <linux/of_address.h>

#include "zte_disp_preferences.h"

#include "zte_disp_preferences-sharp-r63417-1080p-5p5.h"
#include "zte_disp_preferences-sharp-nt35595-1080p-5p0.h"
#include "zte_disp_preferences-tianma-hx8394d-720p-5p5.h"
#include "zte_disp_preferences-oufei-hx8394d-720p-5p5.h"
#include "zte_lcd_dsi.h"

unsigned int zte_intensity_value;
static int boot_flag = 0;
struct mdss_dsi_ctrl_pdata *zte_mdss_dsi_ctrl;
void zte_send_cmd(struct dsi_cmd_desc *cmds, int len);



struct mdp_pcc_cfg_data zte_pcc_cfg_debug = {
	.block = 0x10,
	.ops = 0x5,
	{
		.c = 0,
		.r = 0x8000,
		.g = 0,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0x8000,
		.b = 0,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
	{
		.c = 0,
		.r = 0,
		.g = 0,
		.b = 0x8000,
		.rr = 0,
		.gg = 0,
		.bb = 0,
		.rg = 0,
		.gb = 0,
		.rb = 0,
		.rgb_0 = 0,
		.rgb_1 = 0
	},
};

static struct zte_enhance_type zte_enhance_val = {
	.en_saturation =1,
	.saturation = INTENSITY_01,
	.colortmp =  INTENSITY_NORMAL,
	.en_colortmp = 1,
};

struct zte_enhance_type zte_get_lcd_enhance_param(void)
{
	return zte_enhance_val;
}

void zte_mipi_colortmp(void)
{
	unsigned int value;
	value = zte_enhance_val.colortmp;

	if (!zte_enhance_val.en_colortmp || (NULL == zte_mdss_dsi_ctrl))
		return;

	if (!zte_mdss_dsi_ctrl) {
		pr_err("lcd:faild:%s zte_mdss_dsi_ctrl is null\n",__func__);
		return;
	}

	if (zte_mdss_dsi_ctrl->panel_name &&
		(!strcmp(zte_mdss_dsi_ctrl->panel_name, "sharp r63417 1080p 5p5 mipi cmd panel") ||
		!strcmp(zte_mdss_dsi_ctrl->panel_name, "sharp r63417 1080p 5p5 mipi video panel"))) {
		pr_err("lcd::%s sharp r63417 1080p 5p5 panel\n", __func__);
		switch (value) {
		case INTENSITY_NORMAL:
			zte_mdss_pcc_config(&sharp_r63417_1080p_5p5_pcc_cfg_warm);
			break;
		case INTENSITY_01:
			zte_mdss_pcc_config(&sharp_r63417_1080p_5p5_pcc_cfg_natural);
			break;
		case INTENSITY_02:
			zte_mdss_pcc_config(&sharp_r63417_1080p_5p5_pcc_cfg_cool);
			break;
		default:
			zte_mdss_pcc_config(&sharp_r63417_1080p_5p5_pcc_cfg_natural);
			break;
		}
	}
	else if (zte_mdss_dsi_ctrl->panel_name &&
		(!strcmp(zte_mdss_dsi_ctrl->panel_name, "sharp nt35595 1080p 5p0 video mode dsi panel") ||
		 !strcmp(zte_mdss_dsi_ctrl->panel_name, "sharp nt35595 1080p 5p0 command mode dsi panel"))) {
		pr_err("lcd::%s sharp nt35595 1080p 5p0 panel\n", __func__);
		switch (value) {
		case INTENSITY_NORMAL:
			zte_mdss_pcc_config(&sharp_nt35595_1080p_5p0_pcc_cfg_warm);
			break;
		case INTENSITY_01:
			zte_mdss_pcc_config(&sharp_nt35595_1080p_5p0_pcc_cfg_natural);
			break;
		case INTENSITY_02:
			zte_mdss_pcc_config(&sharp_nt35595_1080p_5p0_pcc_cfg_cool);
			break;
		default:
			zte_mdss_pcc_config(&sharp_nt35595_1080p_5p0_pcc_cfg_warm);
			break;
		}
	} 
	else if (zte_mdss_dsi_ctrl->panel_name &&
		(!strcmp(zte_mdss_dsi_ctrl->panel_name, "tianma hx8394d 720p video mode dsi panel"))) {
		pr_err("lcd::%s tianma hx8394d 720p video mode dsi panel\n", __func__);
		switch (value) {
		case INTENSITY_NORMAL:
			zte_mdss_pcc_config(&tianma_hx8394d_720p_5p5_pcc_cfg_warm);
			break;
		case INTENSITY_01:
			zte_mdss_pcc_config(&tianma_hx8394d_720p_5p5_pcc_cfg_natural);
			break;
		case INTENSITY_02:
			zte_mdss_pcc_config(&tianma_hx8394d_720p_5p5_pcc_cfg_cool);
			break;
		default:
			zte_mdss_pcc_config(&tianma_hx8394d_720p_5p5_pcc_cfg_warm);
			break;
		}
	} 
	else if (zte_mdss_dsi_ctrl->panel_name &&
		(!strcmp(zte_mdss_dsi_ctrl->panel_name, "oufei hx8394d 720p video mode dsi panel"))) {
		pr_err("lcd::%s oufei hx8394d 720p video mode dsi panel\n", __func__);
		switch (value) {
		case INTENSITY_NORMAL:
			zte_mdss_pcc_config(&oufei_hx8394d_720p_5p5_pcc_cfg_warm);
			break;
		case INTENSITY_01:
			zte_mdss_pcc_config(&oufei_hx8394d_720p_5p5_pcc_cfg_natural);
			break;
		case INTENSITY_02:
			zte_mdss_pcc_config(&oufei_hx8394d_720p_5p5_pcc_cfg_cool);
			break;
		default:
			zte_mdss_pcc_config(&oufei_hx8394d_720p_5p5_pcc_cfg_warm);
			break;
		}
	} 
    else
    {
		pr_err("lcd::%s thers is no panel named %s\n", __func__, zte_mdss_dsi_ctrl->panel_name);
	}
}

static ssize_t colortmp_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",	zte_enhance_val.colortmp);
}

static ssize_t colortmp_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t size)
{
	int val;

	if (!zte_enhance_val.en_colortmp)
		 return size;

	sscanf(buf, "%d", &val);

	zte_enhance_val.colortmp = val;

	zte_mipi_colortmp();
	return size;
}

static ssize_t colortmp_debug_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "r = 0x%x, g = 0x%x, b = 0x%x\n",
		zte_pcc_cfg_debug.r.r, zte_pcc_cfg_debug.g.g, zte_pcc_cfg_debug.b.b);
}

static ssize_t colortmp_debug_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t size)
{
	int val;

	if(!zte_enhance_val.en_colortmp)
		 return size;

	sscanf(buf, "%d", &val);

	if (val == 1)
		zte_mdss_pcc_config(&zte_pcc_cfg_debug);

	return size;
}

static ssize_t colortmp_r_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t size)
{
	uint32_t val;

	sscanf(buf, "%d", &val);

	zte_pcc_cfg_debug.r.r = 0x8000 - val;

	return size;
}

static ssize_t colortmp_g_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t size)
{
	uint32_t val;

	sscanf(buf, "%d", &val);

	zte_pcc_cfg_debug.g.g = 0x8000 - val;

	return size;
}

static ssize_t colortmp_b_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t size)
{
	uint32_t val;

	sscanf(buf, "%d", &val);

	zte_pcc_cfg_debug.b.b = 0x8000 - val;

	return size;
}

void zte_mipi_disp_inc(unsigned int state)
{
	unsigned int value;
	value = state;

	if (!zte_mdss_dsi_ctrl) {
		pr_err("lcd:faild:%s zte_mdss_dsi_ctrl is null\n",__func__);
		return;
	}
	else
	{	    
        pr_err("lcd::%s set inc to %d\n", __func__,state);
	}

	if (zte_mdss_dsi_ctrl->panel_name &&
		(!strcmp(zte_mdss_dsi_ctrl->panel_name, "sharp r63417 1080p 5p5 mipi cmd panel") ||
		!strcmp(zte_mdss_dsi_ctrl->panel_name, "sharp r63417 1080p 5p5 mipi video panel"))) {
		pr_err("lcd::%s sharp r63417 1080p 5p5 panel\n", __func__);
		switch (value) {
		case INTENSITY_NORMAL:
			zte_send_cmd(sharp_r63417_1080p_5p5_soft
				, sizeof(sharp_r63417_1080p_5p5_soft)/sizeof(sharp_r63417_1080p_5p5_soft[0]));
			break;
		case INTENSITY_01:
			zte_send_cmd(sharp_r63417_1080p_5p5_standard
				, sizeof(sharp_r63417_1080p_5p5_standard)/sizeof(sharp_r63417_1080p_5p5_standard[0]));
			break;
		case INTENSITY_02:
			zte_send_cmd(sharp_r63417_1080p_5p5_glow
				, sizeof(sharp_r63417_1080p_5p5_glow)/sizeof(sharp_r63417_1080p_5p5_glow[0]));
			break;
		default:
			zte_send_cmd(sharp_r63417_1080p_5p5_soft
				, sizeof(sharp_r63417_1080p_5p5_soft)/sizeof(sharp_r63417_1080p_5p5_soft[0]));
			break;
		}
	}
	else if	(zte_mdss_dsi_ctrl->panel_name &&
             (!strcmp(zte_mdss_dsi_ctrl->panel_name, "sharp nt35595 1080p 5p0 video mode dsi panel") ||
		     !strcmp(zte_mdss_dsi_ctrl->panel_name, "sharp nt35595 1080p 5p0 command mode dsi panel"))) {
    		pr_err("lcd::%s sharp nt35595 1080p 5p0 panel\n", __func__);
            switch (value)
            {
                case INTENSITY_NORMAL:
                {
                    zte_send_cmd(sharp_nt35595_1080p_5p0_soft
                        , sizeof(sharp_nt35595_1080p_5p0_soft)/sizeof(sharp_nt35595_1080p_5p0_soft[0]));
                    break;
                }
                case INTENSITY_01:
                {
                    zte_send_cmd(sharp_nt35595_1080p_5p0_standard
                        , sizeof(sharp_nt35595_1080p_5p0_standard)/sizeof(sharp_nt35595_1080p_5p0_standard[0]));
                    break;
                }
                case INTENSITY_02:
                {
                    zte_send_cmd(sharp_nt35595_1080p_5p0_glow
                        , sizeof(sharp_nt35595_1080p_5p0_glow)/sizeof(sharp_nt35595_1080p_5p0_glow[0]));
                    break;
                }
                default:
                {
                    zte_send_cmd(sharp_nt35595_1080p_5p0_soft
                        , sizeof(sharp_nt35595_1080p_5p0_soft)/sizeof(sharp_nt35595_1080p_5p0_soft[0]));
                    break;
                }
            }
    }
    else if	(zte_mdss_dsi_ctrl->panel_name &&
             (!strcmp(zte_mdss_dsi_ctrl->panel_name, "tianma hx8394d 720p video mode dsi panel"))) {
    		pr_err("lcd::%s tianma hx8394d 720p video mode dsi panel\n", __func__);
            switch (value) {
            case INTENSITY_NORMAL:
                zte_send_cmd(tianma_hx8394d_720p_5p5_soft
                    , sizeof(tianma_hx8394d_720p_5p5_soft)/sizeof(tianma_hx8394d_720p_5p5_soft[0]));
                break;
            case INTENSITY_01:
                zte_send_cmd(tianma_hx8394d_720p_5p5_standard
                    , sizeof(tianma_hx8394d_720p_5p5_standard)/sizeof(tianma_hx8394d_720p_5p5_standard[0]));
                break;
            case INTENSITY_02:
                zte_send_cmd(tianma_hx8394d_720p_5p5_glow
                    , sizeof(tianma_hx8394d_720p_5p5_glow)/sizeof(tianma_hx8394d_720p_5p5_glow[0]));
                break;
            default:
                zte_send_cmd(tianma_hx8394d_720p_5p5_soft
                    , sizeof(tianma_hx8394d_720p_5p5_soft)/sizeof(tianma_hx8394d_720p_5p5_soft[0]));
                break;
            }
    }
    else if (zte_mdss_dsi_ctrl->panel_name &&
             (!strcmp(zte_mdss_dsi_ctrl->panel_name, "oufei hx8394d 720p video mode dsi panel"))) {
    		pr_err("lcd::%s oufei hx8394d 720p video mode dsi panel\n", __func__);
            switch (value) {
            case INTENSITY_NORMAL:
                zte_send_cmd(oufei_hx8394d_720p_5p5_soft
                    , sizeof(oufei_hx8394d_720p_5p5_soft)/sizeof(oufei_hx8394d_720p_5p5_soft[0]));
                break;
            case INTENSITY_01:
                zte_send_cmd(oufei_hx8394d_720p_5p5_standard
                    , sizeof(oufei_hx8394d_720p_5p5_standard)/sizeof(oufei_hx8394d_720p_5p5_standard[0]));
                break;
            case INTENSITY_02:
                zte_send_cmd(oufei_hx8394d_720p_5p5_glow
                    , sizeof(oufei_hx8394d_720p_5p5_glow)/sizeof(oufei_hx8394d_720p_5p5_glow[0]));
                break;
            default:
                zte_send_cmd(oufei_hx8394d_720p_5p5_soft
                    , sizeof(oufei_hx8394d_720p_5p5_soft)/sizeof(oufei_hx8394d_720p_5p5_soft[0]));
                break;
            }
    }
    else 
    {
		pr_err("lcd::%s thers is no panel named %s\n", __func__, zte_mdss_dsi_ctrl->panel_name);
	}
}

void zte_send_cmd(struct dsi_cmd_desc *cmds, int len)
{
	struct dcs_cmd_req cmdreq;

	if (!zte_mdss_dsi_ctrl) {
		pr_err("lcd:faild:%s zte_mdss_dsi_ctrl is null\n",__func__);
		return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmds;
	cmdreq.cmds_cnt = len;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(zte_mdss_dsi_ctrl, &cmdreq);
}

void zte_disp_preferences(void)
{
	zte_mipi_disp_inc(zte_intensity_value);
	if (!boot_flag)
		boot_flag = 1;
}
EXPORT_SYMBOL(zte_disp_preferences);

static ssize_t intensity_show(struct kobject *kobj, struct kobj_attribute *attr,
   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", zte_intensity_value);
}

static ssize_t intensity_store(struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t size)
{
	int val;
	sscanf(buf, "%d", &val);

	zte_intensity_value = val;

	if (!boot_flag)
		return size;

	zte_mipi_disp_inc(val);

	return size;
}

static ssize_t disp_arg_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct device_node *chosen_node;
	static const char *cmd_line;
	char *disp_idx, *end_idx;
	int len = 0, name_len, cmd_len;

	chosen_node = of_find_node_by_name(NULL, "chosen");
	if (!chosen_node) {
		pr_err("%s: get chosen node failed\n", __func__);
		goto err;
	}

	cmd_line = of_get_property(chosen_node, "bootargs", &len);
	if (!cmd_line || len <= 0) {
		pr_err("%s: get bootargs failed\n", __func__);
		goto err;
	}

	name_len = strlen("mdss_mdp.panel=");
	cmd_len = strlen(cmd_line);
	disp_idx = strnstr(cmd_line, "mdss_mdp.panel=", cmd_len);
	if (!disp_idx) {
		pr_err("%s:%d:cmdline panel not set disp_idx=[%p]\n",
			__func__, __LINE__, disp_idx);
	goto err;
	}

	disp_idx += name_len;

	end_idx = strnstr(disp_idx, " ", 256);
	pr_debug("%s:%d: pan_name=[%s] end=[%s]\n", __func__, __LINE__,
		disp_idx, end_idx);
	if (!end_idx) {
		end_idx = disp_idx + strlen(disp_idx) + 1;
		pr_warn("%s:%d: pan_name=[%s] end=[%s]\n", __func__,
			__LINE__, disp_idx, end_idx);
	}

	return snprintf(buf, PAGE_SIZE, "%s\n",	disp_idx);
err:
	return snprintf(buf, PAGE_SIZE, "err\n");

}

static struct kobj_attribute disp_arg_attribute	 = __ATTR(disp_arg,	  0664,	   disp_arg_show, NULL);
static struct kobj_attribute disptype_attribute	 = __ATTR(disptype,	  0664,	   intensity_show, intensity_store);
static struct kobj_attribute colortmp_attribute	 = __ATTR(colortmp,	  0664,	   colortmp_show, colortmp_store);
static struct kobj_attribute colortmp_r_attribute	   = __ATTR(colortmp_r,	0664,	   NULL, colortmp_r_store);
static struct kobj_attribute colortmp_g_attribute	   = __ATTR(colortmp_g,	0664,	   NULL, colortmp_g_store);
static struct kobj_attribute colortmp_b_attribute	   = __ATTR(colortmp_b,	0664,	   NULL, colortmp_b_store);
static struct kobj_attribute colortmp_debug_attribute   = __ATTR(colortmp_debug,	0664,	   colortmp_debug_show, colortmp_debug_store);

static struct attribute *attrs[] = {
	&disp_arg_attribute.attr,
	&disptype_attribute.attr,
	&colortmp_attribute.attr,
	&colortmp_r_attribute.attr,
	&colortmp_g_attribute.attr,
	&colortmp_b_attribute.attr,
	&colortmp_debug_attribute.attr,
	NULL, /* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *id_kobj;

static int __init id_init(void)
{
	int retval;

	id_kobj = kobject_create_and_add("lcd_enhance", kernel_kobj);
	if (!id_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(id_kobj, &attr_group);
	if (retval)
		kobject_put(id_kobj);

	return retval;
}

static void __exit id_exit(void)
{
	kobject_put(id_kobj);
}

module_init(id_init);
module_exit(id_exit);

