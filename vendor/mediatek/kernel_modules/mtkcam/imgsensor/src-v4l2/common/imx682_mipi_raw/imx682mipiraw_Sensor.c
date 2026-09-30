// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 imx682mipiraw_Sensor.c
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
#include "imx682mipiraw_Sensor.h"
#define PFX "IMX682"
#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int imx682_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_sensor_temperature(void *arg);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int imx682_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int close(struct subdrv_ctx *ctx);
//static int get_csi_param(struct subdrv_ctx *ctx,enum SENSOR_SCENARIO_ID_ENUM scenario_id,struct mtk_csi_param *csi_param);
static int imx682_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int imx682_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int imx682_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx682_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx682_set_test_pattern},
	{SENSOR_FEATURE_SET_AWB_GAIN, imx682_set_awbgain},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, imx682_seamless_switch},
	{SENSOR_FEATURE_SET_STREAMING_RESUME,imx682_streaming_control_on},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x01,
		.addr_header_id = 0x00,
		.i2c_write_id = 0xA0,

		.xtalk_support = TRUE,
		.xtalk_size = 2048,
		.addr_xtalk = 0x150F,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = {
	.i4OffsetX = 17,
	.i4OffsetY = 16,
	.i4PitchX  = 8,
	.i4PitchY  = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	// .i4PosL = { {20, 17}, {24, 19}, {22, 21}, {18, 23},
	// 	   {20, 25}, {24, 27}, {22, 29}, {18, 31} },
	// .i4PosR = { {19, 17}, {23, 19}, {21, 21}, {17, 23},
	// 	   {19, 25}, {23, 27}, {21, 29}, {17, 31} },
	.i4PosL = { {20, 17}, {18, 19}, {22, 21}, {24, 23},
		{20, 25}, {18, 27}, {22, 29}, {24, 31} },
	.i4PosR = { {19, 17}, {17, 19}, {21, 21}, {23, 23},
		{19, 25}, {17, 27}, {21, 29}, {23, 31} },
	.i4BlockNumX = 574,
	.i4BlockNumY = 215,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	.iMirrorFlip = 3,
	.i4ModeIndex = 0,
	.i4FullRawW = 4624,
	.i4FullRawH = 3472,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0}, // R = 1, L = 0
	},
};
static struct SET_PD_BLOCK_INFO_T imgsensor_vr_pd_info = {
	.i4OffsetX = 17,
	.i4OffsetY = 16,
	.i4PitchX  =  8,
	.i4PitchY  = 16,
	.i4PairNum  = 8,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = { {20, 17}, {18, 19}, {22, 21}, {24, 23},
		{20, 25}, {18, 27}, {22, 29}, {24, 31} },
	.i4PosR = { {19, 17}, {17, 19}, {21, 21}, {23, 23},
		{19, 25}, {17, 27}, {21, 29}, {23, 31} },
	.i4BlockNumX = 574,
	.i4BlockNumY = 161,
	.i4Crop = { {0, 0}, {0, 0}, {0, 432}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	.iMirrorFlip = 3,
	.i4ModeIndex = 0,
	.i4FullRawW = 4624,
	.i4FullRawH = 2608,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0}, // R = 1, L = 0
	},
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
    .iMirrorFlip = 3,
    .i4Crop = { {0, 0}, {0, 0}, {24, 384}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {24, 384}, {0, 0} },
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_crop = {
	.i4OffsetX = 8,
	.i4OffsetY = 24,
	.i4PitchX  = 16,
	.i4PitchY  = 32,
	.i4PairNum  = 8,
	.i4SubBlkW  = 16,
	.i4SubBlkH  = 4,
	.i4PosL = { {13, 27}, {9, 31}, {17, 35}, {21, 39},
		   {13, 43}, {9, 47}, {17, 51}, {21, 55} },
	.i4PosR = { {14, 27}, {10, 31}, {18, 35}, {22, 39},
		   {14, 43}, {10, 47}, {18, 51}, {22, 55} },
	.i4BlockNumX = 288,
	.i4BlockNumY = 107,
	.i4LeFirst = 0,
	.i4ModeIndex = 3,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		{0, 0}, {0,0}, {0, 0}, {0, 0}, {0,0}, {2312,1736}
	},
	.i4FullRawW = 9248,
	.i4FullRawH = 6944,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
	},
	.iMirrorFlip = 3,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom2 = {
	.i4OffsetX = 17,
	.i4OffsetY = 16,
	.i4PitchX  = 8,
	.i4PitchY  = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	// .i4PosL = { {20, 17}, {24, 19}, {22, 21}, {18, 23},
	// 	   {20, 25}, {24, 27}, {22, 29}, {18, 31} },
	// .i4PosR = { {19, 17}, {23, 19}, {21, 21}, {17, 23},
	// 	   {19, 25}, {23, 27}, {21, 29}, {17, 31} },
	.i4PosL = { {20, 17}, {18, 19}, {22, 21}, {24, 23},
		{20, 25}, {18, 27}, {22, 29}, {24, 31} },
	.i4PosR = { {19, 17}, {17, 19}, {21, 21}, {23, 23},
		{19, 25}, {17, 27}, {21, 29}, {23, 31} },
	.i4BlockNumX = 478,
	.i4BlockNumY = 134,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {392, 656}, {0, 0}, {0, 0}, {0, 0} },
	.iMirrorFlip = 3,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 2,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom5 = {
	.i4OffsetX = 16,
	.i4OffsetY = 32,
	.i4PitchX  = 8,
	.i4PitchY  = 32,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =8,
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	//.i4VCPackNum = 1,
	.i4PosL = {{20, 33}, {20, 43}, {19, 48}, {19,58}},
	.i4PosR = {{16, 33}, {16, 43}, {23, 48}, {23,58}},
	.i4BlockNumX = 508,
	.i4BlockNumY = 70,
	.iMirrorFlip = 3,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {128,456}, {0, 0}, {0, 0}, {0, 0} },
	.i4ModeIndex = 0,
	//.i4FullRawW = 4096,
	//.i4FullRawH = 3072,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 8,
		.i4PDOrder = {1,1,0,0,0,0,1,1}, // R = 1, L = 0
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4624,
			.vsize = 3472,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 1152,
			.vsize = 1720,
			// .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4624,
			.vsize = 3472,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 1152,
			.vsize = 1720,
			// .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4624,
			.vsize = 2608,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 1152,
             .vsize = 1304,
            //  .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
             .user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2312,
			.vsize = 1736,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	// {
	// 	.bus.csi2 = {
	// 		.channel = 0,
	// 		.data_type = 0x30,
	// 		.hsize = 0x0800,
	// 		.vsize = 0x0120,
	// 		.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
	// 		.user_data_desc = VC_PDAF_STATS,
	// 	},
	// },
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1920,
			.vsize = 1080,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4624,
			.vsize = 3472,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 1152,
			.vsize = 1720,
			// .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
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
			.hsize = 0x3c0,
			.vsize = 0x430,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
         },
    },
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 9248,
			.vsize = 6944,
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
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_ME,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			// .hsize = 0x508,
			// .vsize = 0x230,
						.hsize = 0x200,
			.vsize = 0x240,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4624,
			.vsize = 3472,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    // {
    //     .bus.csi2 = {
	// 		.channel = 0,
	// 		.data_type = 0x30,
	// 		.hsize = 0x800,
	// 		.vsize = 0x600,
	// 		.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
	// 		.user_data_desc = VC_PDAF_STATS,
    //     },
    // },
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
    /*{
         .bus.csi2 = {
             .channel = 1,
             .data_type = 0x2b,
             .hsize = 0x1000,
             .vsize = 0x0300,
             .user_data_desc = VC_PDAF_STATS,
         },
    },*/
};


static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

static struct subdrv_mode_struct mode_struct[] = {
	{//preview
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx682_preview_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_preview_30fps_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = imx682_preview_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx682_preview_seamless_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1098000000,
		.linelength = 9432,
		.framelength = 3696,
		.max_framerate = 300,
		.mipi_pixel_rate = 871200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
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
		.mode_setting_table = imx682_preview_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_preview_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1098000000,
		.linelength = 9432,
		.framelength = 3696,
		.max_framerate = 300,
		.mipi_pixel_rate = 871200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
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
		.mode_setting_table = imx682_video_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_video_30fps_setting),
		.seamless_switch_group = 0,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1098000000,
		.linelength = 9432,
		.framelength = 3880,
		.max_framerate = 300,
		.mipi_pixel_rate = 871200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 864,
			.w0_size = 9248,
			.h0_size = 5216,
			.scale_w = 4624,
			.scale_h = 2608,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 2608,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 2608,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_vr_pd_info,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//hs video
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = imx682_hs_120fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_hs_120fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 855000000,
		.linelength = 3288,
		.framelength = 2164,
		.max_framerate = 1200,
		.mipi_pixel_rate = 919200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 2312,
			.scale_h = 1736,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2312,
			.h1_size = 1736,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2312,
			.h2_tg_size = 1736,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//slim video
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = imx682_slim_240fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_slim_240fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1089000000,
		.linelength = 3288,
		.framelength = 1380,
		.max_framerate = 2400,
		.mipi_pixel_rate = 919200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 784,
			.y0_offset = 1312,
			.w0_size = 7680,
			.h0_size = 4320,
			.scale_w = 1920,
			.scale_h = 1080,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1920,
			.h1_size = 1080,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1920,
			.h2_tg_size = 1080,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info =  &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom1
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = imx682_preview_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_preview_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1098000000,
		.linelength = 9432,
		.framelength = 3696,
		.max_framerate = 300,
		.mipi_pixel_rate = 871200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom2
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = imx682_video4k_60fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_video4k_60fps_setting),
		.seamless_switch_group = 0,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1089000000,
		.linelength = 9432,
		.framelength = 2391,
		.max_framerate = 600,
		.mipi_pixel_rate = 919200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 784,
			.y0_offset = 1312,
			.w0_size = 7680,
			.h0_size = 4320,
			.scale_w = 3840,
			.scale_h = 2160,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3840,
			.h1_size = 2160,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3840,
			.h2_tg_size = 2160,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_custom2,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom3
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = imx682_fullsize_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_fullsize_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1098000000,
		.linelength = 12608,
		.framelength = 7140,
		.max_framerate = 120,
		.mipi_pixel_rate = 919200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 9248,
			.scale_h = 6944,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 9248,
			.h1_size = 6944,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 9248,
			.h2_tg_size = 6944,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
	},
	{//custom4
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = PARAM_UNDEFINED,
		.mode_setting_len = PARAM_UNDEFINED,
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
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_custom4,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom5 dol HDR
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = PARAM_UNDEFINED,
		.mode_setting_len = PARAM_UNDEFINED,
		.seamless_switch_group = 0,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.hdr_mode = HDR_NONE,
		.pclk = 1357824000,
		.linelength = 7072,
		.framelength = 6400,
		.max_framerate = 300,
		.mipi_pixel_rate = 976460000,
		.readout_length = 4405,
		.read_margin = 24,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 4 * 2,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_custom5,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{  //custom6 ISZ
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = imx682_full_corp_setting,
		.mode_setting_len = ARRAY_SIZE(imx682_full_corp_setting),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = imx682_custom6_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx682_custom6_seamless_setting),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 1098000000,
		.linelength = 12608,
		.framelength = 3684,
		.max_framerate = 300,
		.mipi_pixel_rate = 919200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 2312,
			.y0_offset = 1736,
			.w0_size = 4624,
			.h0_size = 3472,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_crop,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
	},
		{//custom7 200m
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = PARAM_UNDEFINED,
		.mode_setting_len = PARAM_UNDEFINED,
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
		.imgsensor_winsize_info = {
			.full_w = 9248,
			.full_h = 6944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 9248,
			.h0_size = 6944,
			.scale_w = 4624,
			.scale_h = 3472,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4624,
			.h1_size = 3472,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4624,
			.h2_tg_size = 3472,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = IMX682_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {9248, 6944},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 50,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max =  128 * (0xFFFC - 48),
	.exposure_step = 1,
	.exposure_margin = 48,
	.saturation_info = &imgsensor_saturation_info,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 2093000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = 0,
	.seamless_switch_support = TRUE,
	.reg_addr_fast_mode = 0x3010,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3100,
	.reg_addr_frame_length = {{0x0340,0x0341},},
	.reg_addr_exposure = {
			{0x0202, 0x0203},
	},
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
	},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = 0x0005,

	.init_setting_table = PARAM_UNDEFINED,
	.init_setting_len = PARAM_UNDEFINED,
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.chk_s_off_sta = 1,
	.chk_s_off_end = 0,

	.checksum_value = 0xb340d5a6,
};

static struct subdrv_ops ops = {
	.get_id = imx682_get_imgsensor_id,
	.init_ctx = init_ctx,  
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = close,
	.vsync_notify = vsync_notify,
	.get_frame_desc = common_get_frame_desc,
	.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
	//.get_csi_param = get_csi_param,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2900000, 1},
	{HW_ID_AVDD1, 1800000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD,  1098000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_MCLK, 24, 5},
	{HW_ID_RST, 1, 3},
	{HW_ID_AFVDD, 2800000, 1},
};

const struct subdrv_entry imx682_mipi_raw_entry = {
	.name = "imx682_mipi_raw",
	.id = IMX682_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

#define SENSOR_QSC_ADDR 0x077F
#define EEPROM_QSC_ADDR 0xCA00
#define QSC_LENGTH 3024

#define SENSOR_LRC_ADDR 0x1351
#define EEPROM_LRC_ADDR1 0x7b00
#define EEPROM_LRC_ADDR2 0x7c00

static u16 imx682_QSC_setting[3024 * 2];
static u16 imx682_LRC_setting[504 * 2];
int lrc_flag=0,qsc_flag=0;
static void imx682_read_sensor_Cali(struct subdrv_ctx *ctx)
{

	u16 idx = 0, addr_qsc = 0, sensor_qsc = 0;
	int i = 0;
	u8 temp = 0;
	u8 Checksum_Qsc = 0;
	u8 data = 0;

	for (idx = 0; idx < QSC_LENGTH; idx++) {
		addr_qsc = SENSOR_QSC_ADDR + idx;
		sensor_qsc = EEPROM_QSC_ADDR + idx;
		imx682_QSC_setting[2 * idx] = sensor_qsc;
		adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, addr_qsc, (u8 *)&data, 1);
		imx682_QSC_setting[2 * idx + 1] = data;
	}

	for(i=0; i < QSC_LENGTH; i++)
	{
		temp= imx682_QSC_setting[i*2+1] + temp;
		//pr_info("imx682_remosaic_qsc Data[%d] = %d\n",i*2+1,imx682_QSC_setting[i*2+1]);
	}
	adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, SENSOR_QSC_ADDR+QSC_LENGTH,&Checksum_Qsc, 1);
	if (Checksum_Qsc != (temp%256)) {
		pr_err("imx682_remosaic_qsc checksum err, [0x%x] != [0x%x]\n", Checksum_Qsc, (temp%256));
		return;
	}else{
		pr_info("imx682_remosaic_qsc checksum success, [0x%x] = [0x%x]\n", Checksum_Qsc, (temp%256));
	}
}

static void LRC_interface(struct subdrv_ctx *ctx)
{

	u16 idx = 0, addr_lrc = 0, sensor_lrc = 0;
	int i = 0;
	u8 temp = 0;
	u8 Checksum_Lrc = 0;
	u8 data=0;
	/*read otp data to distinguish module*/

	for (idx = 0; idx < 252; idx++) {
		addr_lrc = SENSOR_LRC_ADDR + idx;
		sensor_lrc = EEPROM_LRC_ADDR1 + idx;
		imx682_LRC_setting[2 * idx] = sensor_lrc;
		adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, addr_lrc, (u8 *)&data, 1);
		imx682_LRC_setting[2 * idx + 1] = data;
	}
	for(; idx < 504; idx++) {
		addr_lrc = SENSOR_LRC_ADDR + idx;
		sensor_lrc = EEPROM_LRC_ADDR2 + i;
		imx682_LRC_setting[2 * idx] = sensor_lrc;
		adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, addr_lrc, (u8 *)&data, 1);
		imx682_LRC_setting[2 * idx + 1] = data;
		i++;
	}
	for(i=0; i < 504; i++)
	{
		temp= imx682_LRC_setting[2*i+1] + temp;
		//pr_err("LRC addr : 0x%x LRC data = 0x%x",imx682_LRC_setting[i*2],imx682_LRC_setting[2*i+1]);
	}
	adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, SENSOR_LRC_ADDR + 504, (u8 *)&Checksum_Lrc, 1);
	if (Checksum_Lrc != (temp%256)) {
		pr_err("imx682_remosaic_lrc checksum err, [0x%x] != [0x%x]\n", Checksum_Lrc, (temp%256));
		return;
	}else{
		pr_info("imx682_remosaic_lrc checksum success, [0x%x] = [0x%x]\n", Checksum_Lrc, (temp%256));
	}
}

static void write_sensor_QSC(struct subdrv_ctx *ctx)
{
	// calibration tool version 3.0 -> 0x4E
	//write_cmos_sensor_8(0x86A9, 0x4E);
	// set QSC from EEPROM to sensor
	//subdrv_i2c_wr_u8(ctx,0x0100, 0x00); //stream off

	i2c_table_write(ctx, imx682_QSC_setting, sizeof(imx682_QSC_setting)/sizeof(u16));
	subdrv_i2c_wr_u8(ctx,0x3621, 0x01);
	mdelay(10);
}

static void write_sensor_LRC(struct subdrv_ctx *ctx)
{
	// calibration tool version 3.0 -> 0x4E
	//write_cmos_sensor_8(0x86A9, 0x4E);
	// set QSC from EEPROM to sensor
	i2c_table_write(ctx, imx682_LRC_setting, sizeof(imx682_LRC_setting)/sizeof(u16));
}

/*write AWB gain to sensor*/
static int m_r_gain = 0, m_b_gain = 0, awb_flag = 0;
static u16 imx682_feedback_awbgain[] = {
    0x0b90, 0x00,
    0x0b91, 0x01,
    0x0b92, 0x00,
    0x0b93, 0x01,
};

static int feedback_awbgain(struct subdrv_ctx *ctx,u32 r_gain, u32 b_gain)
{
    u32 r_gain_int = 0;
    u32 b_gain_int = 0;

    LOG_INF("feedback_awbgain r_gain: %d %d, b_gain: %d %d mode:%d\n", r_gain, m_r_gain, b_gain, m_b_gain, ctx->current_scenario_id);
    if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6){
        awb_flag = 1;
        r_gain_int = r_gain / 512;
        b_gain_int = b_gain / 512;
        imx682_feedback_awbgain[1] = r_gain_int;
        imx682_feedback_awbgain[3] = (((r_gain * 100) / 512) - (2 * 100)) * 2;
        imx682_feedback_awbgain[5] = b_gain_int;
        imx682_feedback_awbgain[7] = (((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
        LOG_INF("feedback_awbgain awbgain[1]: %d, awbgain[3]: %d awbgain[5]: %d awbgain[7]: %d\n", imx682_feedback_awbgain[1], imx682_feedback_awbgain[3],imx682_feedback_awbgain[5],imx682_feedback_awbgain[7]);
        i2c_table_write(ctx,imx682_feedback_awbgain,sizeof(imx682_feedback_awbgain)/sizeof(u16));
    }else{
		awb_flag = 0;
	}

	m_r_gain = r_gain;
	m_b_gain = b_gain;

    return 0;
}

static int imx682_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 *feature_data_32 = (u32 *) para;

    feedback_awbgain(ctx, (u32)*(feature_data_32 + 1), (u32)*(feature_data_32 + 2));

    return 0;
}
static int imx682_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;

	if (*feature_data) {
		set_shutter(ctx, *feature_data);
	}
	streaming_control(ctx, TRUE);
	if((ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 ||  ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6) && awb_flag == 0){
		feedback_awbgain(ctx, m_r_gain, m_b_gain);
		awb_flag = 0;
		DRV_LOG_MUST(ctx, "imx682_streaming_control feedback_awbgain \n");
	}
    return 0;
}
static int imx682_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
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
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOGE(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == ctx->s_ctx.sensor_id){
				imx682_read_sensor_Cali(ctx);
				LRC_interface(ctx);
				return ERROR_NONE;
			}
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
/*
static int get_csi_param(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	struct mtk_csi_param *csi_param)
{
	pr_info("imx682 scenario_id:%u,aov_csi_clk:%u\n",scenario_id, ctx->aov_csi_clk);
	csi_param->legacy_phy = 0;
  	csi_param->not_fixed_trail_settle = 0;
	csi_param->cphy_settle = 84;
	return 0;
}
*/
static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;
	subdrv_i2c_wr_u8(ctx, 0x0138, 0x01);
	temperature = subdrv_i2c_rd_u8(ctx, 0x013a);

	if (temperature < 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature < 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 &&temperature < 0xE2)
		temperature_convert = -30;
	else{
		temperature_convert = (char)temperature;
	}

	DRV_LOG(ctx, "temperature_convert: %d,temperature =%d\n", temperature_convert,temperature);
	return temperature_convert;
}

static void set_group_hold(void *arg, u8 en)
{
	 struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	 if (en)
		 set_i2c_buffer(ctx, 0x0104, 0x01);
	 else
		 set_i2c_buffer(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	pr_info("imx682 get_gain2reg gain = %d,return = %d\n",gain,1024 - (1024*BASEGAIN)/gain);
	return 1024 - (1024*BASEGAIN)/gain;
}

static int imx682_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("imx682 set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
	//1:Solid Color 2:Color bar 5:Black
	if (mode == 2){
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x02);
	}else if (mode == 5)
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x01);//Dgain = 0
	else
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/

	ctx->test_pattern = mode;

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

static int imx682_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	i2c_table_write(ctx, imx682_init_setting, sizeof(imx682_init_setting)/sizeof(u16));
	/*enable temperature sensor, TEMP_SEN_CTL:*/
	//subdrv_i2c_wr_u8(ctx, 0x0138, 0x01);
	/* set MIPI auto ctrl */
	//subdrv_i2c_wr_u8(ctx, 0x0808, 0x00);
	if (!qsc_flag) {
		DRV_LOG(ctx,"write_sensor_QSC Start\n");
		write_sensor_QSC(ctx);
		DRV_LOG(ctx,"write_sensor_QSC End\n");
		qsc_flag = 1;
	}
	if (!lrc_flag) {
		DRV_LOG(ctx,"write_sensor_lrc Start\n");
		write_sensor_LRC(ctx);
		DRV_LOG(ctx,"write_sensor_lrc End\n");
		lrc_flag = 1;
	}

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
	imx682_sensor_init(ctx);

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

int close(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx,"E\n");

	/*No Need to implement this function*/

	subdrv_i2c_wr_u8(ctx,0x0100, 0x00);
	qsc_flag = 0;
	lrc_flag = 0;
	return 0;
}

static int imx682_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 frame_length_in_lut[IMGSENSOR_STAGGER_EXPOSURE_CNT] = {0};
	u32 exp_cnt = 0;

	if (feature_data == NULL) {
		DRV_LOG(ctx, "input scenario is null!");
		return ERROR_NONE;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOG(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, " E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOG(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOG(ctx, "seamless_switch not supported\n");
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOG(ctx, "Please implement seamless_switch setting\n");
		return ERROR_NONE;
	}

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x02);
	if (ctx->s_ctx.reg_addr_fast_mode_in_lbmf &&
		(ctx->s_ctx.mode[scenario_id].hdr_mode == HDR_RAW_LBMF ||
		ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF))
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode_in_lbmf, 0x4);

	update_mode_info(ctx, scenario_id);
	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_LBMF:
			set_multi_shutter_frame_length_in_lut(ctx,
				(u64 *)&ae_ctrl->exposure, exp_cnt, 0, frame_length_in_lut);
			set_multi_gain_in_lut(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		case HDR_RAW_DCG_RAW:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			if (ctx->s_ctx.mode[scenario_id].dcg_info.dcg_gain_mode
				== IMGSENSOR_DCG_DIRECT_MODE)
				set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			else
				set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		default:
			set_shutter(ctx, ae_ctrl->exposure.le_exposure);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt){
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		commit_i2c_buffer(ctx);
	}
	return 0;
}
