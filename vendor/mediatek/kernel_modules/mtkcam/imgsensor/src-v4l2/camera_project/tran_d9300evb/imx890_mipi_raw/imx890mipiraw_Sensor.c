// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 imx890mipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *-------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *===================================================================
 *******************************************************************/
#include "imx890mipiraw_Sensor.h"
#define IMX890_LOG_INF(format, args...) pr_info(LOG_TAG "[%s] " format, __func__, ##args)

#define FPT_PDAF_SUPPORT 0

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int imx890_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int imx890_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx890_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, imx890_set_test_pattern_data},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x010B00FF,
		.addr_header_id = 0x00000001,
		.i2c_write_id = 0xA2,

		.xtalk_support = TRUE,
		.xtalk_size = 2048,
		.addr_xtalk = 0x150F,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 0,
	.i4PitchY  = 0,
	.i4PairNum  =0,
	.i4SubBlkW  =0,
	.i4SubBlkH  =0,
	.i4PosL = {{0, 0},{0, 0},{0, 0},{0, 0}},
	.i4PosR = {{0, 0},{0, 0},{0, 0},{0, 0}},
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.iMirrorFlip = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
		{0, 0}, {128,456}, {0, 0}, {0, 0}, {128,456}, {2048,1536}
	},
	.iMirrorFlip = 3,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom4 = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX  = 8,
    .i4PitchY  = 8,
    .i4PairNum  =4,
    .i4SubBlkW  =8,
    .i4SubBlkH  =2,
    .i4PosL = {{1, 0},{3, 3},{7, 4},{5, 7}},
    .i4PosR = {{0, 0},{2, 3},{6, 4},{4, 7}},
    .i4BlockNumX = 504,
    .i4BlockNumY = 288,
    .iMirrorFlip = 0,
    .i4Crop = { {0, 0}, {0, 0}, {24, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {24, 384}, {0, 0} },
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 0,
	.i4PitchY  = 0,
	.i4PairNum  =0,
	.i4SubBlkW  =0,
	.i4SubBlkH  =0,
	.i4PosL = {{0, 0},{0, 0},{0, 0},{0, 0}},
	.i4PosR = {{0, 0},{0, 0},{0, 0},{0, 0}},
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.iMirrorFlip = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
		{0, 0}, {128,456}, {0, 0}, {0, 0}, {128,456}, {2048,1536}
	},
	.iMirrorFlip = 3,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom2 = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =2,
	.i4PosL = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.i4PosR = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4BlockNumX = 480,
	.i4BlockNumY = 268,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0}, {120, 456}, {0, 0}, {0, 0}, {0, 0} },
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
             .channel = 3,
             .data_type = 0x2b,
             .hsize = 0x1000,
             .vsize = 0x0240,
             .user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0600,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0780,
			.vsize = 0x0438,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 0x1000,
             .vsize = 0x0300,
             .user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0F00,
			.vsize = 0x0870,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
        .bus.csi2 = {
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 0x0F00,
             .vsize = 0x021C,
             .user_data_desc = VC_PDAF_STATS,
        },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x2000,
			.vsize = 0x1800,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0F00,
			.vsize = 0x0870,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 0x1000,
             .vsize = 0x0300,
             .user_data_desc = VC_PDAF_STATS,
         },
    },
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 0x1000,
             .vsize = 0x0300,
             .user_data_desc = VC_PDAF_STATS,
         },
    },
};


static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

#define WINSIZE_INFO_PRE  {8192, 6144,    0,    0, 8192, 6144, 4096, 3072, 0,  0, 4096, 3072,    0,    0, 4096, 3072}
#define WINSIZE_INFO_CAP  {8192, 6144,    0,    0, 8192, 6144, 4096, 3072, 0,  0, 4096, 3072,    0,    0, 4096, 3072}
#define WINSIZE_INFO_VID  {8192, 6144,    0,  768, 8192, 4608, 4096, 2304, 0,  0, 4096, 2304,    0,    0, 4096, 2304}
#define WINSIZE_INFO_HS   {8192, 6144,    0,    0, 8192, 6144, 2048, 1536, 0,  0, 2048, 1536,    0,    0, 2048, 1536}
#define WINSIZE_INFO_SLIM {8192, 6144,    0,  912, 8192, 4320, 2048, 1080, 64, 0, 1920, 1080,    0,    0, 1920, 1080}
#define WINSIZE_INFO_CUS1 {8192, 6144,    0,    0, 8192, 6144, 4096, 3072, 0,  0, 4096, 3072,    0,    0, 4096, 3072}
#define WINSIZE_INFO_CUS2 {8192, 6144,   256, 912, 7680, 4320, 3840, 2160, 0,  0, 3840, 2160,    0,    0, 3840, 2160}
#define WINSIZE_INFO_CUS3 {8192, 6144,    0,    0, 8192, 6144, 8192, 6144, 0,  0, 8192, 6144,    0,    0, 8192, 6144}
#define WINSIZE_INFO_CUS4 {8192, 6144,    0,  768, 8192, 4608, 4096, 2304, 0,  0, 4096, 2304,    0,    0, 4096, 2304}
#define WINSIZE_INFO_CUS5 {8192, 6144,   256, 912, 7680, 4320, 3840, 2160, 0,  0, 3840, 2160,    0,    0, 3840, 2160}
#define WINSIZE_INFO_CUS6 {8192, 6144,    0,    0, 8192, 6144, 4096, 3072, 0,  0, 4096, 3072,    0,    0, 4096, 3072}
#define WINSIZE_INFO_CUS7 {8192, 6144,    0,    0, 8192, 6144, 4096, 3072, 0,  0, 4096, 3072,    0,    0, 4096, 3072}
static struct subdrv_mode_struct mode_struct[] = {
	{//preview
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1526400000,
		.linelength = 15616,
		.framelength = 3258,
		.max_framerate = 300,
		.mipi_pixel_rate = 960000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_PRE,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x49,
		},
	},
	{//capture
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_capture,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_capture),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1526400000,
		.linelength = 15616,
		.framelength = 3258,
		.max_framerate = 300,
		.mipi_pixel_rate = 960000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_CAP,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x49,
		},
	},
	{//normal video
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = addr_data_pair_normal_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_normal_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1536000000,
		.linelength = 15616,
		.framelength = 3278,
		.max_framerate = 300,
		.mipi_pixel_rate = 586970000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_VID,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//hs video
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = addr_data_pair_hs_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_hs_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1881600000,
		.linelength = 8816,
		.framelength = 1776,
		.max_framerate = 1200,
		.mipi_pixel_rate = 582860000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_HS,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//slim video
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = addr_data_pair_slim_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_slim_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 3110400000,
		.linelength = 8816,
		.framelength = 1468,
		.max_framerate = 2400,
		.mipi_pixel_rate = 935310000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_SLIM,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom1
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = addr_data_pair_custom1,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom1),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1507200000,
		.linelength = 15616,
		.framelength = 3216,
		.max_framerate = 300,
		.mipi_pixel_rate = 576000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom2
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = addr_data_pair_custom2,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom2),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 2217600000,
		.linelength = 15616,
		.framelength = 2366,
		.max_framerate = 600,
		.mipi_pixel_rate = 806400000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS2,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_custom2,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom3
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = addr_data_pair_custom3,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom3),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1280000000,
		.linelength = 11552,
		.framelength = 7386,
		.max_framerate = 100,
		.mipi_pixel_rate = 995660000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS3,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom4
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = addr_data_pair_custom4,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom4),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 2246400000,
		.linelength = 15616,
		.framelength = 4792,
		.max_framerate = 300,
		.mipi_pixel_rate = 938060000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS4,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_custom4,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom5
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = addr_data_pair_custom5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom5),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 3340800000,
		.linelength = 15616,
		.framelength = 7128,
		.max_framerate = 300,
		.mipi_pixel_rate = 1231540000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS5,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = addr_data_pair_custom6,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom6),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 1526400000,
		.linelength = 15616,
		.framelength = 3258,
		.max_framerate = 300,
		.mipi_pixel_rate = 960000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS6,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{//custom7 200m
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = addr_data_pair_custom7,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom7),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 1526400000,
		.linelength = 15616,
		.framelength = 3258,
		.max_framerate = 300,
		.mipi_pixel_rate = 960000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS7,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = IMX890_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 2,
	.ana_gain_step = 1,
	.ana_gain_table = imx890_ana_gain_table,
	.ana_gain_table_size = sizeof(imx890_ana_gain_table),
	.min_gain_iso = 50,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max =  128 * (0xFFFC - 48),
	.exposure_step = 1,
	.exposure_margin = 48,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,
	.saturation_info = &imgsensor_saturation_info,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	//.hdr_type = HDR_SUPPORT_STAGGER_FDOL|HDR_SUPPORT_DCG|HDR_SUPPORT_LBMF,
	.hdr_type = 0,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = FALSE,
	//.reg_addr_exposure_lshift = 0x3128,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_frame_length = {0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,

	.checksum_value = 0x47a75476,
};

static struct subdrv_ops ops = {
	.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_PDN, 0, 0},
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2804000, 3},
	{HW_ID_AVDD1, 1804000, 3},
	{HW_ID_DVDD, 1104000, 4},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_PDN, 1, 0},
	{HW_ID_RST, 1, 5}
};

const struct subdrv_entry imx890_mipi_raw_entry = {
	.name = "imx890_mipi_raw",
	.id = IMX890_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static void set_group_hold(void *arg, u8 en)
{
	// struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	// if (en)
		// set_i2c_buffer(ctx, 0x0104, 0x01);
	// else
		// set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return 16384 - (16384 * BASEGAIN) / gain;
}

static int imx890_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("xxx set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
	//1:Solid Color 2:Color bar 5:Black
	// if (mode == 5)
		// subdrv_i2c_wr_u8(ctx, 0x020E, 0x00); /*100% Color bar*/
	// else if (ctx->test_pattern)
		// subdrv_i2c_wr_u8(ctx, 0x0601, mode); /*No pattern*/

	ctx->test_pattern = mode;

	return 0;
}

static int imx890_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	// subdrv_i2c_wr_u16(ctx, 0x0602, Gr);
	// subdrv_i2c_wr_u16(ctx, 0x0604, R);
	// subdrv_i2c_wr_u16(ctx, 0x0606, B);
	// subdrv_i2c_wr_u16(ctx, 0x0608, Gb);

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);

	return 0;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;

	return 0;
}

static int imx890_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	i2c_table_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));
	/*enable temperature sensor, TEMP_SEN_CTL:*/
	subdrv_i2c_wr_u8(ctx, 0x0138, 0x01);
	/* set MIPI auto ctrl */
	subdrv_i2c_wr_u8(ctx, 0x0808, 0x00);
	DRV_LOG(ctx, "X\n");

	return 0;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (common_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	imx890_sensor_init(ctx);

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->current_fps = 10 * ctx->pclk / ctx->line_length / ctx->frame_length;
	ctx->readout_length = ctx->s_ctx.mode[scenario_id].readout_length;
	ctx->read_margin = ctx->s_ctx.mode[scenario_id].read_margin;
	ctx->min_frame_length = ctx->frame_length;
	ctx->autoflicker_en = FALSE;
	ctx->test_pattern = 0;
	ctx->ihdr_mode = 0;
	ctx->pdaf_mode = 0;
	ctx->hdr_mode = 0;
	ctx->extend_frame_length_en = 0;
	ctx->is_seamless = 0;
	ctx->fast_mode_on = 0;
	ctx->sof_cnt = 0;
	ctx->ref_sof_cnt = 0;
	ctx->is_streaming = 0;

	return ERROR_NONE;
} /* open */
