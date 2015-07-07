#ifndef __ZTE_DISP_PERFERENCE_TIANMA_HX8394D_720P_5p5_H__
#define __ZTE_DISP_PERFERENCE_TIANMA_HX8394D_720P_5p5_H__

struct mdp_pcc_cfg_data tianma_hx8394d_720p_5p5_pcc_cfg_warm = {
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
      .b = 0x7b50,
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


struct mdp_pcc_cfg_data tianma_hx8394d_720p_5p5_pcc_cfg_natural = {
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

struct mdp_pcc_cfg_data tianma_hx8394d_720p_5p5_pcc_cfg_cool = {
	.block = 0x10,
	.ops = 0x5,
    {
      .c = 0,
      .r = 0x7d44,
      .g = 0,
      .b = 0,
    },
    {
      .c = 0,
      .r = 0,
      .g = 0x7d44,
      .b = 0,
    },
    {
      .c = 0,
      .r = 0,
      .g = 0,
      .b = 0x8000,
    },
};

static char tianma_hx8394d_720p_video_ce_soft [] = {0xE4, 0x00, 0x01};
static char tianma_hx8394d_720p_video_ce_standard [] = {0xE4, 0x01, 0x01};
static char tianma_hx8394d_720p_video_ce_glow [] = {0xE4, 0x02, 0x01};

static struct dsi_cmd_desc tianma_hx8394d_720p_5p5_glow[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(tianma_hx8394d_720p_video_ce_glow)}, tianma_hx8394d_720p_video_ce_glow},
};
static struct dsi_cmd_desc tianma_hx8394d_720p_5p5_soft[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(tianma_hx8394d_720p_video_ce_soft)}, tianma_hx8394d_720p_video_ce_soft},

};
static struct dsi_cmd_desc tianma_hx8394d_720p_5p5_standard[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(tianma_hx8394d_720p_video_ce_standard)}, tianma_hx8394d_720p_video_ce_standard},
};

#endif
