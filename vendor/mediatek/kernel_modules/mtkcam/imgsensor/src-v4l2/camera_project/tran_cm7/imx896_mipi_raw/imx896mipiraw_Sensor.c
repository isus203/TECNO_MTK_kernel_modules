// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 imx896mipiraw_Sensor.c
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
#include "imx896mipiraw_Sensor.h"
#define PFX "IMX896"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)

#define FPT_PDAF_SUPPORT 0

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int imx896_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx896_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_sensor_temperature(void *arg);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int imx896_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int close(struct subdrv_ctx *ctx);
static int get_csi_param(struct subdrv_ctx *ctx,enum SENSOR_SCENARIO_ID_ENUM scenario_id,struct mtk_csi_param *csi_param);
static int imx896_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int imx896_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int imx896_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx896_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int imx896_get_stagger_target_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx896_feature_get_chip_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);

/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx896_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, imx896_set_test_pattern_data},
	{SENSOR_FEATURE_SET_AWB_GAIN, imx896_set_awbgain},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, imx896_seamless_switch},
	{SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO, imx896_get_stagger_target_scenario},
	{SENSOR_FEATURE_SET_STREAMING_RESUME,imx896_streaming_control_on},
	{SENSOR_FEATURE_TRAN_GET_SENSOR_CHIP_ID, imx896_feature_get_chip_id},
};

static unsigned char chip_id[32];
#define CHIP_ID_SIZE 11
unsigned char *imx896_get_chip_id(struct subdrv_ctx *ctx,unsigned char *chip_id)
{
	int i = 0;
	unsigned char chipIdData[32];

	//sensor_init();

	for (i = 0; i < CHIP_ID_SIZE; i++) {
		chipIdData[i] = subdrv_i2c_rd_u8(ctx, 0x3AF0 + i);//OTP data read
		//pr_info("sunyu chipIdData[%d], %02X", i, chipIdData[i]);
		sprintf(chip_id+2*i, "%02X",chipIdData[i]);
	}

	pr_info("sunyu chip_id is %s", chip_id);

	return chip_id;
}

static int imx896_feature_get_chip_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *)para;
	char *feature_return_para_32 = (char *)(uintptr_t)(*(feature_data+1));
	int i = 0;
	*len = CHIP_ID_SIZE*2;

	for(i=0;i<CHIP_ID_SIZE*2;i++){
		feature_return_para_32[i] = chip_id[i];
	}

	//pr_info("feature_return_para_32 chip_id is %s", feature_return_para_32);
	return 0;
}

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
	.i4Crop = { {0, 0}, {0, 0}, {0, 384}, {1024, 960}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},{0, 0},{0, 0},{0, 384}},
	.iMirrorFlip = 3,
	.i4ModeIndex = 3,
	//.i4FullRawW = 4096,
	//.i4FullRawH = 3072,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,
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
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 0,
	.i4PitchY = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4ModeIndex = 3,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
		{0, 0}, {0,0}, {0, 0}, {0, 0}, {0,0}, {0, 0}, {0, 0}
	},
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,
		.i4BinFacX = 4,
		.i4BinFacY = 2,
	},
	.iMirrorFlip = 3,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom2 = {
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
	.i4BlockNumX = 480,
	.i4BlockNumY = 66,
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
	.i4BlockNumX = 504,
	.i4BlockNumY = 72,
	.iMirrorFlip = 3,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {128,456}, {0, 0}, {0, 0}, {0, 384} },
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
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
    {
         .bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
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
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
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
             .channel = 0,
             .data_type = 0x30,
             .hsize = 0x1000,
             .vsize = 0x0240,
             .dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
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
			.vsize = 0x0480,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0800,
			.vsize = 0x0120,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x500,
			.vsize = 0x2d0,
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
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
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
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x01e0,
			.vsize = 0x0210,
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
			.hsize = 0x1F8,
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
			.hsize = 0x1000,
			.vsize = 0x0c00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x800,
			.vsize = 0x600,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
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
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x800,
			.vsize = 0x480,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
//4096*3072 1exp seamless_switch cus9
static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
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
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x0240,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};
//4096*3072 2dol
static struct mtk_mbus_frame_desc_entry frame_desc_cus9[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		 .bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 4096,
			.vsize = 3072,
			.user_data_desc = VC_STAGGER_ME,
		 },
	},
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

#define WINSIZE_INFO_PRE  {8192, 6144, 000, 000, 8192, 6144, 4096, 3072, 0000, 0000, 4096, 3072, 0, 0, 4096, 3072}
#define WINSIZE_INFO_CAP  {8192, 6144, 000, 000, 8192, 6144, 4096, 3072, 0000, 0000, 4096, 3072, 0, 0, 4096, 3072}
#define WINSIZE_INFO_VID  {8192, 6144, 000, 768, 8192, 4608, 4096, 2304, 0000, 0000, 4096, 2304, 0, 0, 4096, 2304}
#define WINSIZE_INFO_HS   {8192, 6144, 000, 768, 8192, 4608, 2048, 1152, 0000, 0000, 2048, 1152, 0, 0, 2048, 1152}
#define WINSIZE_INFO_SLIM {8192, 6144, 000, 768, 8192, 4608, 2048, 1152, 0000, 0000, 2048, 1152, 0, 0, 2048, 1152}
#define WINSIZE_INFO_CUS1 {8192, 6144, 000, 000, 8192, 6144, 4096, 3072, 0000, 0000, 4096, 3072, 0, 0, 4096, 3072}
#define WINSIZE_INFO_CUS2 {8192, 6144, 256, 912, 7680, 4320, 3840, 2160, 0,  0, 3840, 2160,    0,    0, 3840, 2160}
#define WINSIZE_INFO_CUS3 {8192, 6144, 000, 000, 8192, 6144, 8192, 6144, 0000, 0000, 8192, 6144, 0, 0, 8192, 6144}
#define WINSIZE_INFO_CUS4 {8192, 6144,    0,  768, 8192, 4608, 4096, 2304, 0,  0, 4096, 2304,    0,    0, 4096, 2304}
#define WINSIZE_INFO_CUS5 {8192, 6144, 256, 912, 7680, 4320, 3840, 2160, 0,  0, 3840, 2160,    0,    0, 3840, 2160}
#define WINSIZE_INFO_CUS6 {8192, 6144, 2048, 1536, 4096, 3072, 4096, 3072, 0,  0, 4096, 3072,    0,    0, 4096, 3072}
#define WINSIZE_INFO_CUS7 {8192, 6144,    0,    0, 8192, 6144, 4096, 3072, 0,  0, 4096, 3072,    0,    0, 4096, 3072}
static struct subdrv_mode_struct mode_struct[] = {
	{//preview
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = imx896_preview_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_preview_30fps_setting),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = imx896_preview_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx896_preview_seamless_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1353841920,
		.linelength = 11904,
		.framelength = 3791,
		.max_framerate = 300,
		.mipi_pixel_rate = 1162970000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x49,
		},
	},
	{//capture
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = imx896_preview_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_preview_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1353841920,
		.linelength = 11904,
		.framelength = 3791,
		.max_framerate = 300,
		.mipi_pixel_rate = 1162970000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 0x49,
		},
	},
	{//normal video
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = imx896_video_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_video_30fps_setting),
		.seamless_switch_group = 4,
		.seamless_switch_mode_setting_table = imx896_video_30fps_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx896_video_30fps_seamless_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1353841920,
		.linelength = 11904,
		.framelength = 3791,
		.max_framerate = 300,
		.mipi_pixel_rate = 976460000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//hs video
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = imx896_hs_120fps_pdaf_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_hs_120fps_pdaf_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1356364800,
		.linelength = 6728,
		.framelength = 1680,
		.max_framerate = 1200,
		.mipi_pixel_rate = 680230000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 2048,
			.scale_h = 1152,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2048,
			.h1_size = 1152,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2048,
			.h2_tg_size = 1152,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//slim video
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = imx896_slim_240fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_slim_240fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1344695040,
		.linelength = 4264,
		.framelength = 1314,
		.max_framerate = 2400,
		.mipi_pixel_rate = 608910000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 1536,
			.y0_offset = 1632,
			.w0_size = 5120,
			.h0_size = 2880,
			.scale_w = 1280,
			.scale_h = 720,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1280,
			.h1_size = 720,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1280,
			.h2_tg_size = 720,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom1
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = imx896_preview_30fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_preview_30fps_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1353841920,
		.linelength = 11904,
		.framelength = 3791,
		.max_framerate = 300,
		.mipi_pixel_rate = 1162970000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom2
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = imx896_video4k_60fps_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_video4k_60fps_setting),
		.seamless_switch_group = 0,
		.seamless_switch_mode_setting_table = imx896_4k_60fps_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx896_4k_60fps_seamless_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1358979840,
		.linelength = 7136,
		.framelength = 3174,
		.max_framerate = 600,
		.mipi_pixel_rate = 970970000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 256,
			.y0_offset = 912,
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
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom3
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = imx896_fullsize_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_fullsize_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1357806480,
		.linelength = 14032,
		.framelength = 6451,
		.max_framerate = 150,
		.mipi_pixel_rate = 861260000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8192,
			.h0_size = 6144,
			.scale_w = 8192,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8192,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8192,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
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
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_custom4,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom5 dol HDR
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = addr_data_pair_custom5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom5),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = imx896_video_2dol_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx896_video_2dol_seamless_setting),
		.raw_cnt = 2,
		.exp_cnt = 2,
		.hdr_mode = HDR_RAW_STAGGER,
		.pclk = 1357824000,
		.linelength = 7072,
		.framelength = 6400,
		.max_framerate = 300,
		.mipi_pixel_rate = 976460000,
		.readout_length = 4405,
		.read_margin = 24,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 4 * 2,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_custom5,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{  //custom6 ISZ
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = imx896_custom6_seamless_setting,
		.mode_setting_len = ARRAY_SIZE(imx896_custom6_seamless_setting),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = imx896_custom6_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx896_custom6_seamless_setting),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 1357596000,
		.linelength = 14032,
		.framelength = 3225,
		.max_framerate = 300,
		.mipi_pixel_rate = 1162970000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 2048,
			.y0_offset = 1536,
			.w0_size = 4096,
			.h0_size = 3072,
			.scale_w = 4096,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_crop,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
	},
		{//custom7 video ISZ Reg_C-4-1
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = addr_data_pair_custom7,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom7),
		.seamless_switch_group = 4,
		.seamless_switch_mode_setting_table = imx896_custom7_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx896_custom7_seamless_setting),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 1357596000,
		.linelength = 14032,
		.framelength = 3225,
		.max_framerate = 300,
		.mipi_pixel_rate = 976460000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 2048,
			.y0_offset = 1920,
			.w0_size = 4096,
			.h0_size = 2304,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_crop,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom8 and custom5 seamless
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = addr_data_pair_custom8,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom8),
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = imx896_custom8_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx896_custom8_seamless_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1353841920,
		.linelength = 11904,
		.framelength = 3791,
		.max_framerate = 300,
		.mipi_pixel_rate = 976460000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8192,
			.h0_size = 4608,
			.scale_w = 4096,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4096,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4096,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{ //Reg_I-1 2dol
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = addr_data_pair_custom9,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom9),
		.seamless_switch_group = 4,
		.seamless_switch_mode_setting_table = imx896_custom9_seameless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx896_custom9_seameless_setting),
		.raw_cnt = 2,
		.exp_cnt = 2,
		.hdr_mode = HDR_RAW_STAGGER,
		.pclk = 1360000000,
		.linelength = 7072,
		.framelength = 6400,
		.max_framerate = 300,
		.mipi_pixel_rate = 976460000,
		.readout_length = 4405,
		.read_margin = 24,
		.imgsensor_winsize_info = {
                        .full_w = 8192,
                        .full_h = 6144,
                        .x0_offset = 0,
                        .y0_offset = 0,
                        .w0_size = 8192,
                        .h0_size = 6144,
                        .scale_w = 4096,
                        .scale_h = 3072,
                        .x1_offset = 0,
                        .y1_offset = 0,
                        .w1_size = 4096,
                        .h1_size = 3072,
                        .x2_tg_offset = 0,
                        .y2_tg_offset = 0,
                        .w2_tg_size = 4096,
                        .h2_tg_size = 3072,
                },
		.framelength_step = 4,
		.coarse_integ_step = 4,
		.min_exposure_line = 4 * 2,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1428,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = IMX896_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {8192, 6144},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
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
	.start_exposure_offset = 1200000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = TRUE,
	.reg_addr_fast_mode = 0x3010,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3160,
	.reg_addr_frame_length = {{0x0340,0x0341},},
	.reg_addr_exposure = {
			{0x0202, 0x0203},
			{0x0202, 0x0203},
			{0x0224, 0x0225},
	},
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
			{0x0204, 0x0205},
			{0x0216, 0x0217},
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

	.checksum_value = 0xaf3e324f,
};

static struct subdrv_ops ops = {
	.get_id = imx896_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = close,
	.vsync_notify = vsync_notify,
	.get_frame_desc = common_get_frame_desc,
	//.get_csi_param = common_get_csi_param,
	.update_sof_cnt = common_update_sof_cnt,
	.get_csi_param = get_csi_param,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_AVDD1, 1800000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD,  1098000, 1},
	{HW_ID_RST, 1, 3},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_MCLK, 24, 5},
};

const struct subdrv_entry imx896_mipi_raw_entry = {
	.name = "imx896_mipi_raw",
	.id = IMX896_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

#define SENSOR_QSC_ADDR 0x135a
#define EEPROM_QSC_ADDR 0xc000
#define QSC_LENGTH     3072

#define SENSOR_SPC_ADDR 0x1F5C
#define EEPROM_SPC_ADDR1 0xd200
#define EEPROM_SPC_ADDR2 0xd300

static u16 imx896_QSC_setting[3072 * 2];
static u16 imx896_SPC_setting[384 * 2];
int spc_flag=0,qsc_flag=0;
static void imx896_read_sensor_Cali(struct subdrv_ctx *ctx)
{

	u16 idx = 0, addr_qsc = 0, sensor_qsc = 0;
	int i = 0;
	u8 temp = 0;
	u8 Checksum_Qsc = 0;
	u8 data = 0;

	for (idx = 0; idx < 3072; idx++) {
		addr_qsc = SENSOR_QSC_ADDR + idx;
		sensor_qsc = EEPROM_QSC_ADDR + idx;
		imx896_QSC_setting[2 * idx] = sensor_qsc;
		adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, addr_qsc, (u8 *)&data, 1);
		imx896_QSC_setting[2 * idx + 1] = data;
	}

	for(i=0; i < QSC_LENGTH; i++)
	{
		temp= imx896_QSC_setting[i*2+1] + temp;
		//pr_info("imx896_remosaic_qsc Data[%d] = %d\n",i*2+1,imx896_QSC_setting[i*2+1]);
	}
	adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, SENSOR_QSC_ADDR+QSC_LENGTH,&Checksum_Qsc, 1);
	if (Checksum_Qsc != (temp%256)) {
		pr_err("imx896_remosaic_qsc checksum err, [0x%x] != [0x%x]\n", Checksum_Qsc, (temp%256));
		return;
	}else{
		pr_info("imx896_remosaic_qsc checksum success, [0x%x] = [0x%x]\n", Checksum_Qsc, (temp%256));
	}
}

static void SPC_interface(struct subdrv_ctx *ctx)
{

	u16 idx = 0, addr_spc = 0, sensor_spc = 0;
	int i = 0;
	u8 temp = 0;
	u8 Checksum_Spc = 0;
	u8 data=0;
	/*read otp data to distinguish module*/

	for (idx = 0; idx < 192; idx++) {
		addr_spc = SENSOR_SPC_ADDR + idx;
		sensor_spc = EEPROM_SPC_ADDR1 + idx;
		imx896_SPC_setting[2 * idx] = sensor_spc;
		adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, addr_spc, (u8 *)&data, 1);
		imx896_SPC_setting[2 * idx + 1] = data;
	}
	for(; idx < 384; idx++) {
		addr_spc = SENSOR_SPC_ADDR + idx;
		sensor_spc = EEPROM_SPC_ADDR2 + i;
		imx896_SPC_setting[2 * idx] = sensor_spc;
		adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, addr_spc, (u8 *)&data, 1);
		imx896_SPC_setting[2 * idx + 1] = data;
		i++;
	}
	for(i=0; i < 384; i++)
	{
		temp= imx896_SPC_setting[2*i+1] + temp;
		//pr_err("SPC addr : 0x%x SPC data = 0x%x",imx896_SPC_setting[i*2],imx896_SPC_setting[2*i+1]);
	}
	adaptor_i2c_rd_p8(ctx->i2c_client,0xA0 >> 1, SENSOR_SPC_ADDR+384, (u8 *)&Checksum_Spc, 1);
	if (Checksum_Spc != (temp%256)) {
		pr_err("imx896_remosaic_spc checksum err, [0x%x] != [0x%x]\n", Checksum_Spc, (temp%256));
		return;
	}else{
		pr_info("imx896_remosaic_spc checksum success, [0x%x] = [0x%x]\n", Checksum_Spc, (temp%256));
	}
}

static void write_sensor_QSC(struct subdrv_ctx *ctx)
{
	// calibration tool version 3.0 -> 0x4E
	//write_cmos_sensor_8(0x86A9, 0x4E);
	// set QSC from EEPROM to sensor
	subdrv_i2c_wr_u8(ctx,0x0100, 0x00); //stream off

	//i2c_table_write(ctx, imx896_QSC_setting, sizeof(imx896_QSC_setting)/sizeof(u16));
	i2c_table_cntinuburst_write(ctx, imx896_QSC_setting, sizeof(imx896_QSC_setting)/sizeof(u16));

	mdelay(10);
}

static void write_sensor_SPC(struct subdrv_ctx *ctx)
{
	// calibration tool version 3.0 -> 0x4E
	//write_cmos_sensor_8(0x86A9, 0x4E);
	// set QSC from EEPROM to sensor
	i2c_table_write(ctx, imx896_SPC_setting, sizeof(imx896_SPC_setting)/sizeof(u16));
}

/*write AWB gain to sensor*/
static int m_r_gain = 0, m_b_gain = 0, awb_flag = 0;
static u16 imx896_feedback_awbgain[] = {
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
    if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM7){
        awb_flag = 1;
        r_gain_int = r_gain / 512;
        b_gain_int = b_gain / 512;
        imx896_feedback_awbgain[1] = r_gain_int;
        imx896_feedback_awbgain[3] = (((r_gain * 100) / 512) - (2 * 100)) * 2;
        imx896_feedback_awbgain[5] = b_gain_int;
        imx896_feedback_awbgain[7] = (((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
        LOG_INF("feedback_awbgain awbgain[1]: %d, awbgain[3]: %d awbgain[5]: %d awbgain[7]: %d\n", imx896_feedback_awbgain[1], imx896_feedback_awbgain[3],imx896_feedback_awbgain[5],imx896_feedback_awbgain[7]);
        i2c_table_write(ctx,imx896_feedback_awbgain,sizeof(imx896_feedback_awbgain)/sizeof(u16));
    }else{
        awb_flag = 0;
    }

    m_r_gain = r_gain;
    m_b_gain = b_gain;

    return 0;
}

static int imx896_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 *feature_data_32 = (u32 *) para;

    feedback_awbgain(ctx, (u32)*(feature_data_32 + 1), (u32)*(feature_data_32 + 2));

    return 0;
}
static int imx896_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;

	if (*feature_data) {
		set_shutter(ctx, *feature_data);
	}
	streaming_control(ctx, TRUE);
	if((ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 ||  ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM7) && awb_flag == 0){
		feedback_awbgain(ctx, m_r_gain, m_b_gain);
		awb_flag = 0;
		DRV_LOG_MUST(ctx, "imx896_streaming_control feedback_awbgain \n");
	}
    return 0;
}
static int imx896_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
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
			LOG_INF("i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == ctx->s_ctx.sensor_id){
				imx896_read_sensor_Cali(ctx);
				SPC_interface(ctx);
				imx896_get_chip_id(ctx,chip_id);
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

static int get_csi_param(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	struct mtk_csi_param *csi_param)
{
	pr_info("imx896 scenario_id:%u,aov_csi_clk:%u\n",scenario_id, ctx->aov_csi_clk);
	csi_param->legacy_phy = 0;
  	csi_param->not_fixed_trail_settle = 0;
	csi_param->cphy_settle = 84;
	return 0;
}
static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;
	subdrv_i2c_wr_u8(ctx, 0x0138, 0x01);
	temperature = subdrv_i2c_rd_u8(ctx, 0x013a);

	if (temperature < 0x55)
		temperature_convert = temperature;
	else if (temperature>=0x55&&temperature < 0x80)
		temperature_convert = 85;
	else if (temperature>=0x80&&temperature < 0xED)
		temperature_convert = -20;
	else
		temperature_convert = (char)temperature;

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
	pr_info("imx896 get_gain2reg gain = %d,return = %d\n",gain,16384 - (16384 * BASEGAIN) / gain);
	return 16384 - (16384 * BASEGAIN) / gain;
}

static int imx896_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("imx896 set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
	//1:Solid Color 2:Color bar 5:Black
	if (mode == 2){
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x02);
	}else if (mode == 5)
		subdrv_i2c_wr_u8(ctx, 0x020E, 0x00);//Dgain = 0
	else
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/

	if ((ctx->test_pattern) && (mode != ctx->test_pattern)) {
		if (ctx->test_pattern == 5)
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);
		else if (mode == 0)
			subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /* No pattern */
	}
	ctx->test_pattern = mode;

	return 0;
}

static int imx896_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	 //subdrv_i2c_wr_u16(ctx, 0x0602, Gr);
	 //subdrv_i2c_wr_u16(ctx, 0x0604, R);
	 //subdrv_i2c_wr_u16(ctx, 0x0606, B);
	 //subdrv_i2c_wr_u16(ctx, 0x0608, Gb);

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",ctx->test_pattern, R, Gr, Gb, B);

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

static int imx896_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	//i2c_table_write(ctx, imx896_init_setting, sizeof(imx896_init_setting)/sizeof(u16));
	i2c_table_cntinuburst_write(ctx, imx896_init_setting, sizeof(imx896_init_setting)/sizeof(u16));
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
	if (!spc_flag) {
		DRV_LOG(ctx,"write_sensor_spc Start\n");
		write_sensor_SPC(ctx);
		DRV_LOG(ctx,"write_sensor_spc End\n");
		spc_flag = 1;
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
	imx896_sensor_init(ctx);

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	memset(ctx->ana_gain, 0, sizeof(ctx->gain));
	ctx->exposure[0] = ctx->s_ctx.exposure_def;
	ctx->ana_gain[0] = ctx->s_ctx.ana_gain_def;
	ctx->current_scenario_id = scenario_id;
	ctx->pclk = ctx->s_ctx.mode[scenario_id].pclk;
	ctx->line_length = ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length = ctx->s_ctx.mode[scenario_id].framelength;
	ctx->frame_length_rg = ctx->frame_length;
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
	spc_flag = 0;
	return 0;
}

static int imx896_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int imx896_get_stagger_target_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len) {
    u8 scenario_id;
    u8 hdr_mode;
    u8 *target_scenario;
	scenario_id = *para;
	hdr_mode = *(para + 1);
	target_scenario = para + 2;
    DRV_LOG(ctx,"imx896_get_stagger_target_scenario mode = %d\n", *para);
	if (scenario_id == SENSOR_SCENARIO_ID_NORMAL_VIDEO){
        switch (hdr_mode) {
            case HDR_RAW_STAGGER:
                *target_scenario = SENSOR_SCENARIO_ID_CUSTOM5;
                break;
            case HDR_NONE:
                *target_scenario = scenario_id;
                break;
            default:
                break;
        }
	}
    else if (scenario_id == SENSOR_SCENARIO_ID_CUSTOM5){
        switch (hdr_mode) {
            case HDR_NONE:
                *target_scenario = SENSOR_SCENARIO_ID_NORMAL_VIDEO;
                break;
            default:
                break;
        }
    }
	return 0;
}
