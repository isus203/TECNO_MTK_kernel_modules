// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 s5khm6sxmipiraw_Sensor.c
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
#include "s5khm6sxmipiraw_Sensor.h"
#define PFX "S5KHM6SX"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int s5khm6sx_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int tran_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);

static int s5khm6sx_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khm6sx_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khm6sx_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khm6sx_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khm6sx_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khm6sx_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khm6sx_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5khm6sx_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int get_sensor_temperature(void *arg);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5khm6sx_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5khm6sx_set_test_pattern_data},
	{SENSOR_FEATURE_SET_ESHUTTER,s5khm6sx_set_shutter},
	{SENSOR_FEATURE_SET_GAIN,s5khm6sx_set_gain},
 	{SENSOR_FEATURE_SET_AWB_GAIN,s5khm6sx_set_awbgain},
 	{SENSOR_FEATURE_SET_STREAMING_RESUME,s5khm6sx_streaming_control_on},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, s5khm6sx_seamless_switch},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME,s5khm6sx_set_multi_shutter_frame_length}
};

static int m_r_gain = 0, m_b_gain = 0, awb_flag = 0;
static int s5khm6sx_feedback_awbgain(struct subdrv_ctx *ctx,u32 r_gain, u32 b_gain)
{
	u32 r_gain_int = 0x0;
	u32 b_gain_int = 0x0;

	r_gain_int = r_gain * 2;
	b_gain_int = b_gain * 2;

	LOG_INF("set_awbgain: r_gain = 0x%x b_gain = 0x%x sensor mode = %d\n", r_gain_int, b_gain_int, ctx->current_scenario_id);
	if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5){
		awb_flag = 1;
		subdrv_i2c_wr_u16(ctx, 0x0D82, r_gain_int);
		subdrv_i2c_wr_u16(ctx, 0x0D86, b_gain_int);
	}else{
		awb_flag = 0;
	}

	m_r_gain = r_gain;
	m_b_gain = b_gain;

	return 0;
}

static int s5khm6sx_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{

	u32 *feature_data_32 = (u32 *) para;

	s5khm6sx_feedback_awbgain(ctx, (u32)*(feature_data_32 + 1), (u32)*(feature_data_32 + 2));

	return 0;
}

static int s5khm6sx_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;

	if (*feature_data) {
		set_shutter(ctx, *feature_data);
	}
	streaming_control(ctx, TRUE);
	if((ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 ||  ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5) && awb_flag == 0){
		s5khm6sx_feedback_awbgain(ctx, m_r_gain, m_b_gain);
		awb_flag = 0;
		DRV_LOG_MUST(ctx, "s5khm6sx_streaming_control feedback_awbgain \n");
	}
	return 0;
}

static void s5khm6sx_set_multi_shutter(struct subdrv_ctx *ctx,
				u32 *shutters, u16 shutter_cnt,
				u16 frame_length)
{
	if (shutter_cnt == 1) {
		ctx->shutter = shutters[0];

		if (shutters[0] > ctx->min_frame_length - ctx->s_ctx.exposure_margin)
			ctx->frame_length = shutters[0] + ctx->s_ctx.exposure_margin;
		else
			ctx->frame_length = ctx->min_frame_length;

		if (frame_length > ctx->frame_length)
			ctx->frame_length = frame_length;
		if (ctx->frame_length > ctx->s_ctx.frame_length_max)
			ctx->frame_length = ctx->s_ctx.frame_length_max;

		if (shutters[0] < ctx->s_ctx.exposure_min)
			shutters[0] = ctx->s_ctx.exposure_min;

		/* Update Shutter */
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xFFFF);
		subdrv_i2c_wr_u16(ctx, 0X0202, shutters[0] & 0xFFFF);
		ctx->frame_length_rg = ctx->frame_length;

		LOG_INF("shutters[0] =%d, framelength =%d\n", shutters[0], ctx->frame_length);
	}
}

static int s5khm6sx_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	s5khm6sx_set_multi_shutter(ctx, (UINT32 *)(*feature_data), (UINT16) (*(feature_data + 1)), (UINT16) (*(feature_data + 2)));

	return 0;
}

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 4,
	.i4PitchY  = 4,
	.i4PairNum  =2,
	.i4SubBlkW  =2,
	.i4SubBlkH  =4,
	.i4PosL = {{1, 0}, {3, 0}},
	.i4PosR = {{0, 0}, {2, 0}},
	.i4BlockNumX = 1000,
	.i4BlockNumY = 750,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	.iMirrorFlip = 0,
	.i4FullRawW = 4000,
	.i4FullRawH = 3000,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning_video_16_9 = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 4,
	.i4PitchY  = 4,
	.i4PairNum  =2,
	.i4SubBlkW  =2,
	.i4SubBlkH  =4,
	.i4PosL = {{1, 0}, {3, 0}},
	.i4PosR = {{0, 0}, {2, 0}},
	.i4BlockNumX = 1000,
	.i4BlockNumY = 563,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 374}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	.i4FullRawW = 4000,
	.i4FullRawH = 3000,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom2 = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 4,
	.i4PitchY  = 4,
	.i4PairNum  =2,
	.i4SubBlkW  =2,
	.i4SubBlkH  =4,
	.i4PosL = {{1, 0}, {3, 0}},
	.i4PosR = {{0, 0}, {2, 0}},
	.i4BlockNumX = 960,
	.i4BlockNumY = 540,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {80, 420}, {0, 0}, {0, 0}, {0, 0} },
	.i4FullRawW = 4000,
	.i4FullRawH = 3000,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0},
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_fullsizecrop_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 6,
	.i4PitchY  = 6,
	.i4PairNum  =2,
	.i4SubBlkW  =3,
	.i4SubBlkH  =6,
	.i4PosL = {{1, 0}, {4, 0}},
	.i4PosR = {{0, 0}, {3, 0}},
	.i4BlockNumX = 668,
	.i4BlockNumY = 500,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {4000, 3000} },
	.i4FullRawW = 12000,
	.i4FullRawH = 9000,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x07D0,
			.vsize = 0x05DC,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x07D0,
			.vsize = 0x05DC,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 2252,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x07D0,
			.vsize = 0x0466,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2000,
			.vsize = 1500,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1920,
			.vsize = 1080,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x07D0,
			.vsize = 0x05DC,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3840,
			.vsize = 2160,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			 .channel = 1,
			 .data_type = 0x2b,
			 .hsize = 0x0780,
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
			.hsize = 12000,
			.vsize = 9000,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x07D0,
			.vsize = 0x05DC,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
		},
	},
	{
		.bus.csi2 = {
			.channel = 1,
			.data_type = 0x2b,
			.hsize = 0x0538,
			.vsize = 0x03E8,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus6[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
//	{
//		 .bus.csi2 = {
//			 .channel = 1,
//			 .data_type = 0x2b,
//			 .hsize = 0x01F8,
//			 .vsize = 0x0BF0,
//			 .user_data_desc = VC_PDAF_STATS,
//		 },
//	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4000,
			.vsize = 3000,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
//	{
//		 .bus.csi2 = {
//			 .channel = 1,
//			 .data_type = 0x2b,
//			 .hsize = 0x01F8,
//			 .vsize = 0x0BF0,
//			 .user_data_desc = VC_PDAF_STATS,
//		 },
//	},
};

#define WINSIZE_INFO_PRE  {12000, 9000,    0,    0, 12000, 9000,  4000, 3000,    0,     0, 4000, 3000,     0,    0,  4000, 3000} // Preview
#define WINSIZE_INFO_CAP  {12000, 9000,    0,    0, 12000, 9000,  4000, 3000,    0,     0, 4000, 3000,     0,    0,  4000, 3000} // capture
#define WINSIZE_INFO_VID  {12000, 9000,    0, 1122, 12000, 6756,  4000, 2252,    0,     0, 4000, 2252,     0,    0,  4000, 2252} // video
#define WINSIZE_INFO_HS   {12000, 9000,    0,    0, 12000, 9000,  2000, 1500,    0,     0, 2000, 1500,     0,    0,  2000, 1500} // hight speed video
#define WINSIZE_INFO_SLIM {12000, 9000,  240, 1260, 11520, 6480,  1920, 1080,    0,     0, 1920, 1080,     0,    0,  1920, 1080} // slim video
#define WINSIZE_INFO_CUS1 {12000, 9000,    0,    0, 12000, 9000,  4000, 3000,    0,     0, 4000, 3000,     0,    0,  4000, 3000} // Custom1
#define WINSIZE_INFO_CUS2 {12000, 9000,  240, 1260, 11520, 6480,  3840, 2160,    0,     0, 3840, 2160,     0,    0,  3840, 2160} // Custom2
#define WINSIZE_INFO_CUS3 {12000, 9000,    0,    0, 12000, 9000, 12000, 9000,    0,     0, 12000, 9000,    0,    0, 12000, 9000} // custom3
#define WINSIZE_INFO_CUS4 {12000, 9000,    0,    0, 12000, 9000,  4000, 3000,    0,     0, 4000, 3000,     0,    0,  4000, 3000} // Custom4
#define WINSIZE_INFO_CUS5 {12000, 9000, 4000, 3000,  4000, 3000,  4000, 3000,    0,     0, 4000, 3000,     0,    0,  4000, 3000} // custom5
#define WINSIZE_INFO_CUS6 {12000, 9000,    0,    0, 12000, 9000,  4000, 3000,    0,     0, 4000, 3000,     0,    0,  4000, 3000} // Custom6
#define WINSIZE_INFO_CUS7 {12000, 9000,    0,    0, 12000, 9000,  4000, 3000,    0,     0, 4000, 3000,     0,    0,  4000, 3000} // Custom7
static struct subdrv_mode_struct mode_struct[] = {
	{//preview
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = addr_data_pair_preview,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_preview),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = addr_data_pair_preview_fmc,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(addr_data_pair_preview_fmc),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1280,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
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
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1280,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
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
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 1122,
			.w0_size = 12000,
			.h0_size = 6756,
			.scale_w = 4000,
			.scale_h = 2252,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 2252,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 2252,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning_video_16_9,
		.ae_binning_ratio = 1280,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
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
		.pclk = 1640000000,
		.linelength = 8704,
		.framelength = 1570,
		.max_framerate = 1200,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,
			.scale_w = 2000,
			.scale_h = 1500,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2000,
			.h1_size = 1500,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2000,
			.h2_tg_size = 1500,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1280,
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
		.pclk = 1640000000,
		.linelength = 5696,
		.framelength = 1198,
		.max_framerate = 2400,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 240,
			.y0_offset = 1260,
			.w0_size = 11520,
			.h0_size = 6480,
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
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1280,
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
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1280,
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
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 2433,
		.max_framerate = 600,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 240,
			.y0_offset = 1260,
			.w0_size = 11520,
			.h0_size = 6480,
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
		.ae_binning_ratio = 1280,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 85,
		},
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
		.pclk = 1640000000,
		.linelength = 20472,
		.framelength = 9984,
		.max_framerate = 80,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,
			.scale_w = 12000,
			.scale_h = 9000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 12000,
			.h1_size = 9000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 12000,
			.h2_tg_size = 9000,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.ana_gain_max = BASEGAIN * 16,
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
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1280,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom5
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = addr_data_pair_custom5,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom5),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = addr_data_pair_custom5_fmc,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(addr_data_pair_custom5_fmc),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 1640000000,
		.linelength = 16848,
		.framelength = 3242,
		.max_framerate = 300,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 4000,
			.y0_offset = 3000,
			.w0_size = 4000,
			.h0_size = 3000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_fullsizecrop_pd_info,
		.ae_binning_ratio = 1000,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.ana_gain_max = BASEGAIN * 16,
	},
	{//custom6
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = addr_data_pair_custom6,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom6),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1280,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{//custom7
		.frame_desc = frame_desc_cus7,
		.num_entries = ARRAY_SIZE(frame_desc_cus7),
		.mode_setting_table = addr_data_pair_custom7,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom7),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1640000000,
		.linelength = 11232,
		.framelength = 4864,
		.max_framerate = 300,
		.mipi_pixel_rate = 1492114286,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 12000,
			.full_h = 9000,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 12000,
			.h0_size = 9000,
			.scale_w = 4000,
			.scale_h = 3000,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4000,
			.h1_size = 3000,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4000,
			.h2_tg_size = 3000,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1280,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KHM6SX_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info =  PARAM_UNDEFINED,
	.eeprom_num =  PARAM_UNDEFINED,
	.resolution = {12000, 9000},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 2,
	.ana_gain_step = 2,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 50,
	.exposure_def = 0x3D0,
	.exposure_min = 12,//min shutter
	.exposure_max = 0xFFFF - 25,
	.exposure_step = 1,
	.exposure_margin = 8,//margin

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 1185000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = 0,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202,0x0203}},
	.long_exposure_support = TRUE,
	.reg_addr_ana_gain = {{0x0204,0x0205}},
	.reg_addr_frame_length = {{0x0340,0x0341}},
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

	.checksum_value = 0x13018390,
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
				subdrv_i2c_rd_u8(ctx, addr_l);
			if (addr_ll)
				*sensor_id = ((*sensor_id) << 8) | subdrv_i2c_rd_u8(ctx, addr_ll);
			DRV_LOG(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == (ctx->s_ctx.sensor_id & 0xFFFF)){
				return ERROR_NONE;
			}
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != (ctx->s_ctx.sensor_id & 0xFFFF)) {
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
	{HW_ID_RST, 0, 2},
	{HW_ID_DOVDD, 1800000, 2},
	{HW_ID_AVDD,  2200000, 2},
	{HW_ID_DVDD,  1050000, 2},
	{HW_ID_RST, 1, 2},
	{HW_ID_MCLK, 24, 6},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 20},
};

const struct subdrv_entry s5khm6sx_mipi_raw_entry = {
	.name = "s5khm6sx_mipi_raw",
	.id = S5KHM6SX_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static void set_group_hold(void *arg, u8 en)
{
	/*struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	if (en)
		set_i2c_buffer(ctx, 0x0104, 0x01);
	else
		set_i2c_buffer(ctx, 0x0104, 0x00);*/
}

static u16 get_gain2reg(u32 gain)
{
	return gain * 32 / BASEGAIN;
}

static int s5khm6sx_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
	//1:Solid Color 2:Color bar 5:Black
	if (mode == 5)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0001); /*100% Color bar*/
	else if(mode == 2)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0002);
	else if(mode == 1)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0001);
	else if (ctx->test_pattern)
		subdrv_i2c_wr_u16(ctx, 0x0600, 0x0000); /*No pattern*/

	ctx->test_pattern = mode;

	return 0;
}

static int s5khm6sx_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

/*	subdrv_i2c_wr_u16(ctx, 0x0602, Gr);
	subdrv_i2c_wr_u16(ctx, 0x0604, R);
	subdrv_i2c_wr_u16(ctx, 0x0606, B);
	subdrv_i2c_wr_u16(ctx, 0x0608, Gb);*/

	DRV_LOG(ctx, "mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);

	return 0;
}
/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void s5khm6sx_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en)
{
	/*  kal_int16 dummy_line;  */
	u32 frame_length = ctx->frame_length;

	LOG_INF("framerate = %d min_frame_length = %d, min framelength should enable %d\n",framerate, ctx->min_frame_length, min_framelength_en);

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	if (frame_length >= ctx->min_frame_length)
		ctx->frame_length = frame_length;
	else
		ctx->frame_length = ctx->min_frame_length;

	ctx->dummy_line
		= ctx->frame_length - ctx->min_frame_length;

	if (ctx->frame_length > ctx->s_ctx.frame_length_max) {
		ctx->frame_length = ctx->s_ctx.frame_length_max;
		ctx->dummy_line
			= ctx->frame_length - ctx->min_frame_length;
	}

	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;

}

static int s5khm6sx_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 shutter = *((u32 *) para);
	u16 realtime_fps = 0;
	u32 coarseIntegrationTime = 0x0;
	u32 frameLengthLine = 0x0;
	ctx->shutter = shutter;

	if (shutter > ctx->min_frame_length - ctx->s_ctx.exposure_margin)
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
	else
		ctx->frame_length = ctx->min_frame_length;

	if (ctx->frame_length > ctx->s_ctx.frame_length_max)
		ctx->frame_length = ctx->s_ctx.frame_length_max;

	if (shutter < ctx->s_ctx.exposure_min)
		shutter = ctx->s_ctx.exposure_min;

	if (ctx->autoflicker_en) {
		realtime_fps
			= ctx->pclk
			/ ctx->line_length * 10
			/ ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			s5khm6sx_set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			s5khm6sx_set_max_framerate(ctx, 146, 0);
	}

	/* long expsoure */
	if (shutter
		> (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {
		coarseIntegrationTime = shutter / 128;
		frameLengthLine = coarseIntegrationTime + 10;
		LOG_INF("Long shutter In shutter = %d frameLength = 0x%x coarseTime = 0x%x\n", shutter, frameLengthLine, coarseIntegrationTime);
		subdrv_i2c_wr_u16(ctx, 0x0340,frameLengthLine);
		subdrv_i2c_wr_u16(ctx, 0x0202,coarseIntegrationTime);
		subdrv_i2c_wr_u16(ctx, 0x0702, 0x0700);
		subdrv_i2c_wr_u16(ctx, 0x0704, 0x0700);

		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		/* Update Shutter */
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xFFFF);
		subdrv_i2c_wr_u16(ctx, 0x0202, shutter & 0xFFFF);
		subdrv_i2c_wr_u16(ctx, 0x0702, 0x0000);
		subdrv_i2c_wr_u16(ctx, 0x0704, 0x0000);
		ctx->current_ae_effective_frame = 2;
	}
	ctx->frame_length_rg = ctx->frame_length;

	LOG_INF("set_shutter shutter =%d, framelength =%d\n", shutter, ctx->frame_length);
	return 0;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static u32 s5khm6sx_gain2reg(struct subdrv_ctx *ctx, const u32 gain)
{
	u32 reg_gain = 0x0;

	reg_gain = gain / 32;
	return (u32) reg_gain;
}

static u32 set_gain_16Xgain(struct subdrv_ctx *ctx, u32 gain)
{
	u32 reg_gain;
	u32 min_gain = 1 * BASEGAIN;
	u32 max_gain = 16 * BASEGAIN;

	if (gain < min_gain || gain > max_gain) {
		LOG_INF("Error gain setting");

		if (gain < min_gain)
			gain = min_gain;
		else
			gain = max_gain;
	}

	reg_gain = s5khm6sx_gain2reg(ctx, gain);
	ctx->gain = reg_gain;

	LOG_INF("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));
	return gain;
} /* set_gain_16Xgain */

static u32 set_gain_64Xgain(struct subdrv_ctx *ctx, u32 gain)
{
	u32 reg_gain;
	u32 min_gain = 1 * BASEGAIN;//setuphere for mode use
	u32 max_gain = 64 * BASEGAIN;

	if (gain < min_gain || gain > max_gain) {
		LOG_INF("Error gain setting");

		if (gain < min_gain)
			gain = min_gain;
		else
			gain = max_gain;
	}

	reg_gain = s5khm6sx_gain2reg(ctx, gain);
	ctx->gain = reg_gain;

	LOG_INF("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));
	return gain;
} /* set_gain_40Xgain */

static int s5khm6sx_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	LOG_INF("set_gain mode =%d\n", ctx->current_scenario_id);
	switch(ctx->current_scenario_id){
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM4:
			set_gain_64Xgain(ctx, *((u32 *) para));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM5:
			set_gain_16Xgain(ctx, *((u32 *) para));
		break;
		default:
			set_gain_64Xgain(ctx, *((u32 *) para));
		break;
	}
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

static int s5khm6sx_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x01C0);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x1AD6);
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x6010, 0x0001);
	mdelay(30);
	i2c_table_cntinuburst_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));
	i2c_table_cntinuburst_write(ctx, sensor_fmc_init_addr_data, sizeof(sensor_fmc_init_addr_data)/sizeof(u16));
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
	s5khm6sx_sensor_init(ctx);

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

static int s5khm6sx_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	ctx->fast_mode_on = FALSE;

	//subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	//subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x02);
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
	//subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, 0x0020);

	if (temperature >= 0x0&&temperature <= 0x55)
		temperature_convert = temperature;
	else if (temperature>=0x55&&temperature <= 0x7F)
		temperature_convert = 85;
	else if (temperature>=0x80&&temperature <= 0xEC)
		temperature_convert = -20;
	else
		temperature_convert = (u8)temperature;

	DRV_LOG(ctx, "temperature_convert: %d,temperature =%d\n", temperature_convert,temperature);
	return temperature_convert;
}
