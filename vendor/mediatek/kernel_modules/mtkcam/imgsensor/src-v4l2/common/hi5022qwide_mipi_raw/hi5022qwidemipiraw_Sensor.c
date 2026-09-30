// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 hi5022qwidemipiraw_Sensor.c
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
#include "hi5022qwidemipiraw_Sensor.h"
#define HI5022QWIDE_LOG_INF(format, args...) pr_info(LOG_TAG "[%s] " format, __func__, ##args)
#define FPT_PDAF_SUPPORT 0
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int hi5022qwide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int hi5022qwide_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int hi5022qwide_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int tran_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);

/* STRUCT */
static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, hi5022qwide_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, hi5022qwide_set_test_pattern_data},
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
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =2,
	.i4PosR = {{0, 0},{2, 3},{6, 4},{4, 7}},
	.i4PosL = {{1, 0},{3, 3},{7, 4},{5, 7}},
	.i4BlockNumX = 512,
	.i4BlockNumY = 384,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },

	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 1,
		.i4PDOrder = {1}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =2,
	.i4PosR = {{0, 0},{2, 3},{6, 4},{4, 7}},
	.i4PosL = {{1, 0},{3, 3},{7, 4},{5, 7}},
	.i4BlockNumX = 512,
	.i4BlockNumY = 288,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 384}, {0, 0} },
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom2 = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =2,
	.i4PosR = {{0, 0},{2, 3},{6, 4},{4, 7}},
	.i4PosL = {{1, 0},{3, 3},{7, 4},{5, 7}},
	.i4BlockNumX = 480,
	.i4BlockNumY = 270,
	.iMirrorFlip = IMAGE_HV_MIRROR,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {128, 456}, {0, 0}, {0, 0}, {128, 456} },
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
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
             .hsize = 0x0400,
             .vsize = 0x0600,
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
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 0x0400,
             .vsize = 0x0600,
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
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 0x0400,
             .vsize = 0x0480,
             .user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x07F8,
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
			.hsize = 0x0640,
			.vsize = 0x04B0,
			.user_data_desc = VC_STAGGER_NE,
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
             .hsize = 0x03C0,
             .vsize = 0x0438,
             .user_data_desc = VC_PDAF_STATS,
        },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1FE0,
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
			.user_data_desc = VC_RAW_PROCESSED_DATA,
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
			.hsize = 0x0FF0,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 0x01F8,
             .vsize = 0x0BF0,
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
             .hsize = 0x0400,
             .vsize = 0x0600,
             .user_data_desc = VC_PDAF_STATS,
         },
    },
};


static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

#define WINSIZE_INFO_PRE  {8224, 6176, 16,  16,  8192, 6144, 4096, 3072,	0, 0,  4096, 3072, 0, 0, 4096, 3072} //preview
#define WINSIZE_INFO_CAP  {8224, 6176, 16,  16,  8192, 6144, 4096, 3072,	0, 0,  4096, 3072, 0, 0, 4096, 3072} //capture
#define WINSIZE_INFO_VID  {8224, 6176, 16,  784, 8192, 4608, 4096, 2304,    0, 0,  4096, 2304, 0, 0, 4096, 2304} //video
#define WINSIZE_INFO_HS   {8224, 6176, 16,  16,  8192, 6144, 2048, 1536,	4, 0,  2040, 1536, 0, 0, 2040, 1536} //hs_video(2040*1536)@120fps
#define WINSIZE_INFO_SLIM {8224, 6176, 16,  928, 8192, 4320, 2048, 1080,   64, 0,  1920, 1080, 0, 0, 1920, 1080} //slim video(1920*1080)@240fps
#define WINSIZE_INFO_CUS1 {8224, 6176, 912, 688, 6400, 4800, 1600, 1200,	0, 0,  1600, 1200, 0, 0, 1600, 1200} //custom1 vsdof
#define WINSIZE_INFO_CUS2 {8224, 6176, 272, 928, 7680, 4320, 3840, 2160,	0, 0,  3840, 2160, 0, 0, 3840, 2160} //custom2 4k 60fps
#define WINSIZE_INFO_CUS3 {8224, 6176, 32,  16,  8160, 6144, 8160, 6144,	0, 0,  8160, 6144, 0, 0, 8160, 6144} //custom3 remosaic
#define WINSIZE_INFO_CUS4 {8224, 6176, 16,  784, 8192, 4608, 4096, 2304,    0, 0,  4096, 2304, 0, 0, 4096, 2304}
#define WINSIZE_INFO_CUS5 {8224, 6176, 272, 928, 7680, 4320, 3840, 2160,	0, 0,  3840, 2160, 0, 0, 3840, 2160}
#define WINSIZE_INFO_CUS6 {8224, 6176, 16,  784, 8192, 4608, 4096, 2304,    0, 0,  4096, 2304, 0, 0, 4096, 2304} //custom6
#define WINSIZE_INFO_CUS7 {8224, 6176, 16,  16,  8192, 6144, 4096, 3072,	0, 0,  4096, 3072, 0, 0, 4096, 3072} //custom7
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
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_PRE,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x60,
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
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_CAP,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x60,
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
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_VID,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x60,
		},
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
		.pclk = 124000000,
		.linelength = 499,
		.framelength = 2070,
		.max_framerate = 1200,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_HS,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x60,
		},
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
		.pclk = 124000000,
		.linelength = 341,
		.framelength = 1514,
		.max_framerate = 2400,
		.mipi_pixel_rate = 992000000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_SLIM,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x60,
		},
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
		.pclk = 124000000,
		.linelength = 499,
		.framelength = 8280,
		.max_framerate = 300,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
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
		.pclk = 124000000,
		.linelength = 894,
		.framelength = 2310,
		.max_framerate = 600,
		.mipi_pixel_rate = 992000000,
		.readout_length = 0,
		.read_margin = 4,
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
		.pclk = 124000000,
		.linelength = 1776,
		.framelength = 6975,
		.max_framerate = 100,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS3,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 0,
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
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS4,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.pclk = 124000000,
		.linelength = 894,
		.framelength = 2308,
		.max_framerate = 300,
		.mipi_pixel_rate = 992000000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS5,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom6 video iDCG
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = addr_data_pair_custom6,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom6),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_RAW_DCG_COMPOSE,
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS6,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{
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
		.pclk = 124000000,
		.linelength = 1158,
		.framelength = 3568,
		.max_framerate = 300,
		.mipi_pixel_rate = 598400000,
		.readout_length = 0,
		.read_margin = 4,
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
	.sensor_id = HI5022QWIDE_SENSOR_ID,
	.reg_addr_sensor_id = {0x0716, 0x0717},
	.i2c_addr_table = {0x42, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8160, 6144},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 2,
	.ana_gain_step = 3,
	.ana_gain_table = hi5022qwide_ana_gain_table,
	.ana_gain_table_size = sizeof(hi5022qwide_ana_gain_table),
	.min_gain_iso = 50,
	.exposure_def = 0x3D0,
	.exposure_min = 3,
	.exposure_max = 0xFFFF - 3,
	.exposure_step = 1,
	.exposure_margin = 4,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,
	.saturation_info = &imgsensor_saturation_info,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	//.hdr_type = HDR_SUPPORT_STAGGER_FDOL|HDR_SUPPORT_DCG|HDR_SUPPORT_LBMF,
	.hdr_type = 0,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0B00,
	.reg_addr_mirror_flip = 0x0202,
	.reg_addr_exposure = {{0x0510, 0x0511},},
	.long_exposure_support = FALSE,
	//.reg_addr_exposure_lshift = 0x0702,
	.reg_addr_ana_gain = {{0x050A, 0x050B},},
	.reg_addr_frame_length = {0x050C, 0x050D},
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

int tran_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
{
	u8 i = 0;
	u8 retry = 2;
	u32 addr_h = ctx->s_ctx.reg_addr_sensor_id.addr[0];
	u32 addr_l = ctx->s_ctx.reg_addr_sensor_id.addr[1];
	u32 addr_ll = ctx->s_ctx.reg_addr_sensor_id.addr[2];

	while (ctx->s_ctx.i2c_addr_table[i] != 0xFF) {
		ctx->i2c_write_id = ctx->s_ctx.i2c_addr_table[i];
		do {
			*sensor_id = (subdrv_i2c_rd_u8(ctx, addr_h) << 8) |
				subdrv_i2c_rd_u8(ctx, addr_l) + 1;
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOG(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == ctx->s_ctx.sensor_id)
				return ERROR_NONE;
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != ctx->s_ctx.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static struct subdrv_ops ops = {
	.get_id = tran_get_imgsensor_id,
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
	{HW_ID_RST, 0, 1},
	{HW_ID_DOVDD, 1804000, 1},
	{HW_ID_DVDD, 1104000, 1},
	{HW_ID_AVDD, 2804000, 1},
	{HW_ID_RST, 1, 1},
	{HW_ID_MCLK, 24, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 6, 20},
	//{HW_ID_AFVDD, 2804000, 3},
};

const struct subdrv_entry hi5022qwide_mipi_raw_entry = {
	.name = "hi5022qwide_mipi_raw",
	.id = HI5022QWIDE_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static void set_group_hold(void *arg, u8 en)
{
	//struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	// if (en)
		// set_i2c_buffer(ctx, 0x0104, 0x01);
	// else
		// set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return gain / BASEGAIN - 16;
}

static int hi5022qwide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
	//1:Solid Color 2:Color bar 5:Black
	if (mode == 5){
		subdrv_i2c_wr_u16(ctx, 0x0B04, 0x0201); /*100% Color bar*/
		subdrv_i2c_wr_u16(ctx, 0x0B06, 0x010D);
		subdrv_i2c_wr_u16(ctx, 0x0C0A, 0x0200);
	}else if(mode == 2){
		subdrv_i2c_wr_u16(ctx, 0x0B04, 0x0201);
		subdrv_i2c_wr_u16(ctx, 0x0B06, 0x010D);
		subdrv_i2c_wr_u16(ctx, 0x0C0A, 0x0100);
	}else{
		subdrv_i2c_wr_u16(ctx, 0x0C0A, 0x0000); /*No pattern*/
	}
	ctx->test_pattern = mode;

	return 0;
}

static int hi5022qwide_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int hi5022qwide_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	i2c_table_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));
	DRV_LOG(ctx, "X\n");

	return 0;
}

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (tran_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	hi5022qwide_sensor_init(ctx);

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
