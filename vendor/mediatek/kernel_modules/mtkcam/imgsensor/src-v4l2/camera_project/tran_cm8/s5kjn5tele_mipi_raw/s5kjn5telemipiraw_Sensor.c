// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 s5kjn5telemipiraw_Sensor.c
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
#include "s5kjn5telemipiraw_Sensor.h"

static u16 get_gain2reg(u32 gain);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int s5kjn5tele_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int s5kjn5tele_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int s5kjn5tele_write_shutter(struct subdrv_ctx *ctx, u32 shutter);
static void tran_set_dummy(struct subdrv_ctx *ctx);
static int vsync_notify(struct subdrv_ctx *ctx, unsigned int sof_cnt);

static int s5kjn5tele_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_set_afinfmac_pos(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_get_stream_state(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_extend_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5tele_feature_get_chip_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int get_sensor_temperature(void *arg);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kjn5tele_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5kjn5tele_set_test_pattern_data},
	{SENSOR_FEATURE_SET_ESHUTTER,s5kjn5tele_set_shutter},
	{SENSOR_FEATURE_SET_GAIN,s5kjn5tele_set_gain},
	{SENSOR_FEATURE_SET_AWB_GAIN,s5kjn5tele_set_awbgain},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME,s5kjn5tele_set_multi_shutter_frame_length},
	{SENSOR_FEATURE_SET_CURRENT_AF_POSITION,s5kjn5tele_set_afinfmac_pos},
	{SENSOR_FEATURE_SET_STREAMING_RESUME,s5kjn5tele_streaming_control_on},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, s5kjn5tele_seamless_switch},
	{SENSOR_FEATURE_TRAN_GET_STREAM_STATE,s5kjn5tele_get_stream_state},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, s5kjn5tele_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH,s5kjn5tele_extend_frame_length},
	{SENSOR_FEATURE_TRAN_GET_SENSOR_CHIP_ID, s5kjn5tele_feature_get_chip_id},
};

#define EEPROM_READ_ID  0xA0
static int AF_inf = 0;
static int AF_macro = 0;
static int m_r_gain = 0, m_b_gain = 0, awb_flag = 0;

static int s5kjn5tele_extend_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	int i;
	u64 *feature_data = (u64 *) para;
	u32 ns = (u32) *feature_data;
	u32 last_exp_cnt = 1;
	u32 old_fl = ctx->frame_length;
	u32 calc_fl = 0;
	u32 readoutLength = 0;
	u32 readMargin = 0;
	u32 per_frame_ns = (u64)ctx->frame_length *
		(u64)ctx->line_length * 1000000000 / ctx->pclk;

	check_current_scenario_id_bound(ctx);
	if (ctx->s_ctx.mode[ctx->current_scenario_id].hdr_mode == HDR_RAW_LBMF)
		return 0;
	readoutLength = ctx->s_ctx.mode[ctx->current_scenario_id].readout_length;
	readMargin = ctx->s_ctx.mode[ctx->current_scenario_id].read_margin;

	for (i = 1; i < ARRAY_SIZE(ctx->exposure); i++)
		last_exp_cnt += ctx->exposure[i] ? 1 : 0;
	if (ctx->current_scenario_id == 10 && ns == 0)
		ns = 10000000;
	if (ns)
		ctx->frame_length = (u32)(((u64)(per_frame_ns + ns)) *
			ctx->frame_length / per_frame_ns);
	if (last_exp_cnt > 1) {
		calc_fl = (readoutLength + readMargin);
		for (i = 1; i < last_exp_cnt; i++)
			calc_fl += (ctx->exposure[i] + ctx->s_ctx.exposure_margin * last_exp_cnt);
		ctx->frame_length = max(calc_fl, ctx->frame_length);
	}
	set_dummy(ctx);
	ctx->extend_frame_length_en = TRUE;

	ns = (u64)(ctx->frame_length - old_fl) *
		(u64)ctx->line_length * 1000000000 / ctx->pclk;
	DRV_LOG(ctx, "fll(old/new):%u/%u, add %u ns", old_fl, ctx->frame_length, ns);
	return 0;
}

static unsigned char chip_id[32];
#define CHIP_ID_SIZE 6
unsigned char *s5kjn5tele_get_chip_id(struct subdrv_ctx *ctx,unsigned char *chip_id)
{
	int i = 0;
	unsigned char chipIdData[32];

	//sensor_init();
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x2400);
	//OTP mode select register control
	subdrv_i2c_wr_u16(ctx, 0x602A, 0x07C6);
	subdrv_i2c_wr_u16(ctx, 0x6F12, 0x0000);
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0100, 0x0100);
	mdelay(5);
	subdrv_i2c_wr_u16(ctx, 0x0A02, 0x0000);
	subdrv_i2c_wr_u16(ctx, 0x0A00, 0x0100);
	mdelay(2);

	for (i = 0; i < CHIP_ID_SIZE; i++) {
		chipIdData[i] = subdrv_i2c_rd_u8(ctx, 0x0A24 + i);//OTP data read
		//pr_info("sunyu chipIdData[%d], %02X", i, chipIdData[i]);
		sprintf(chip_id+2*i, "%02X",chipIdData[i]);
	}
	subdrv_i2c_wr_u16(ctx, 0x0100, 0x0000);
	subdrv_i2c_wr_u16(ctx, 0x0A00, 0x0000);

	pr_info("sunyu chip_id is %s", chip_id);

	return chip_id;
}

static int s5kjn5tele_feature_get_chip_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static void tran_set_dummy(struct subdrv_ctx *ctx)
{
	if (ctx->extend_frame_length_en == KAL_FALSE) {
		DRV_LOG(ctx,"fll in = %d\n", ctx->frame_length);
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xFFFF);
		ctx->frame_length_rg = ctx->frame_length;
	}
}

static int s5kjn5tele_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;
	u32 framerate = *(feature_data + 1);
	u32 frame_length;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	if (framerate == 0) {
		DRV_LOG(ctx, "framerate should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->s_ctx.mode[scenario_id].linelength == 0) {
		DRV_LOG(ctx, "linelength should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->line_length == 0) {
		DRV_LOG(ctx, "ctx->line_length should not be 0\n");
		return ERROR_NONE;
	}

	if (ctx->frame_length == 0) {
		DRV_LOG(ctx, "ctx->frame_length should not be 0\n");
		return ERROR_NONE;
	}

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;

	if((ctx->frame_length % 2) != 0){
		ctx->frame_length = ctx->frame_length + 1;
	}

	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;

	DRV_LOG(ctx, "max_fps(input/output):%u/%u(sid:%u), min_fl_en:1\n",
		framerate, ctx->current_fps, scenario_id);
	if (ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin)){
		tran_set_dummy(ctx);
	}

	ctx->frame_length_rg = ctx->frame_length;
	return ERROR_NONE;
}

static bool read_s5kjn5_eeprom_infmacro(struct subdrv_ctx *ctx)
{
	int AF_ADDR = 0x775;

	unsigned char data[6];

	adaptor_i2c_rd_p8(ctx->i2c_client,EEPROM_READ_ID >> 1, AF_ADDR, data, 6);

	AF_inf = (data[0] & 0x00FF) | ((data[1] << 8) & 0xFF00);
	AF_macro = (data[4] & 0x00FF) | ((data[5] << 8) & 0xFF00);

	DRV_LOG(ctx,"read_s5kjn5_eeprom_infmacro data[0]=%d data[1]=%d data[4]=%d data[5]=%d AF_inf=%d AF_macro=%d\n", data[0], data[1], data[4], data[5], AF_inf, AF_macro);
	return 0;
}

static void s5kjn5tele_set_af(struct subdrv_ctx *ctx, u32 af_pos, u32 inf_pos, u32 macro_pos)
{
	u32 lens_position = 0x00;
	u32 lens_position_h = 0x00;
	u32 lens_position_l = 0x00;
	lens_position=((af_pos - inf_pos) * 1023) / (macro_pos - inf_pos);
	if(lens_position > 1023)
		lens_position = 1023;
	if(lens_position < 0)
		lens_position = 0;
	DRV_LOG(ctx,"set_afinfmac_pos+  af_pos= %d inf_pos= %d macro_pos= %d lens_position=%d\n", af_pos, inf_pos, macro_pos, lens_position);

	if(ctx->is_seamless){
		return;
	}
	if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6){
		lens_position_h=(lens_position & 0xFF) << 8;
		lens_position_l=(lens_position & 0xFF00) >> 8;
		lens_position  =lens_position_h | lens_position_l;
		DRV_LOG(ctx,"start write\n");
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x2001);
		subdrv_i2c_wr_u16(ctx, 0x2566, lens_position);
		subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
		DRV_LOG(ctx,"set_afinfmac_pos-  lens_position=%d lens_position_h=%d lens_position_l=%d\n", lens_position, lens_position_h, lens_position_l);
	}
}

static int s5kjn5tele_set_afinfmac_pos(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_data_32 = (u32 *) para;

	DRV_LOG(ctx,"s5kjn5tele_set_afinfmac_pos sensor mode = %d\n", ctx->current_scenario_id);
	if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6){
		s5kjn5tele_set_af(ctx, (u32)(*feature_data_32), AF_inf, AF_macro);
	}

	return 0;
}

static int s5kjn5tele_feedback_awbgain(struct subdrv_ctx *ctx,u32 r_gain, u32 b_gain)
{
	u32 r_gain_int = 0x0;
	u32 b_gain_int = 0x0;

	r_gain_int = r_gain * 2;
	b_gain_int = b_gain * 2;

	DRV_LOG(ctx,"set_awbgain: r_gain = %d r_gain_int = %d b_gain = %d b_gain_int = %d sensor mode = %d\n", r_gain, r_gain_int, b_gain, b_gain_int, ctx->current_scenario_id);
	if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6){
		awb_flag = 1;
		subdrv_i2c_wr_u16(ctx, 0x0D82, r_gain_int);
		subdrv_i2c_wr_u16(ctx, 0x0D84, 0x0400);
		subdrv_i2c_wr_u16(ctx, 0x0D86, b_gain_int);
	}else{
		awb_flag = 0;
	}

	m_r_gain = r_gain;
	m_b_gain = b_gain;

	return 0;
}

static int s5kjn5tele_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{

	u32 *feature_data_32 = (u32 *) para;

	s5kjn5tele_feedback_awbgain(ctx, (u32)*(feature_data_32 + 1), (u32)*(feature_data_32 + 2));

	return 0;
}

static int s5kjn5tele_get_stream_state(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (UINT32 *) para;
	int state = 0;

	state = subdrv_i2c_rd_u8(ctx, 0x0100);

	*(feature_return_para_32 + 1) = state;
	DRV_LOG(ctx,"s5kjn5tele_get_stream_state state = %d\n", state);

	return 0;
}


static int s5kjn5tele_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;

	if (*feature_data) {
		s5kjn5tele_write_shutter(ctx, *feature_data);
	}
	streaming_control(ctx, TRUE);
	if((ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 ||  ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6) && awb_flag == 0){
		s5kjn5tele_feedback_awbgain(ctx, m_r_gain, m_b_gain);
		awb_flag = 0;
		DRV_LOG_MUST(ctx, "s5kjn5tele_streaming_control feedback_awbgain \n");
	}
	return 0;
}

static void s5kjn5tele_set_multi_shutter(struct subdrv_ctx *ctx,
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

		DRV_LOG(ctx,"shutters[0] =%d, framelength =%d\n", shutters[0], ctx->frame_length);
	}
}

static int s5kjn5tele_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	s5kjn5tele_set_multi_shutter(ctx, (UINT32 *)(*feature_data), (UINT16) (*(feature_data + 1)), (UINT16) (*(feature_data + 2)));

	return 0;
}

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
	.i4Crop = {
				{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
				{0, 0}, {128, 456}, {0, 0}, {0, 0}, {128, 456}, {2048,1536}
			},
	.iMirrorFlip = 1,
	.i4ModeIndex = 3,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_isz = {
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
	.i4Crop = {
				{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
				{0, 0}, {128, 456}, {0, 0}, {0, 0}, {128, 456}, {2048,1536}
			},
	.iMirrorFlip = 1,
	.i4ModeIndex = 3,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,
		.i4BinFacX = 4,
		.i4BinFacY = 8,
		.i4PDRepetition = 0,
		.i4PDOrder = {0}, //R=1, L=0
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
			.channel = 3,
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
			.channel = 3,
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
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	 {
			.bus.csi2 = {
			.channel = 3,
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
			.vsize = 0x0600,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0800,
			.vsize = 0x0480,
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
			.channel = 3,
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
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x0F00,
			.vsize = 0x021C,
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
			.vsize = 0x0C00,
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
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		 .bus.csi2 = {
			 .channel = 1,
			 .data_type = 0x2b,
			.hsize = 0x0F00,
			.vsize = 0x0870,
			 .user_data_desc = VC_STAGGER_ME,
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
			.channel = 3,
			.data_type = 0x30,
			.hsize = 0x800,
			.vsize = 0x180,
			.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2c,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	 /*{
			.bus.csi2 = {
				 .channel = 3,
				 .data_type = 0x2b,
				 .hsize = 0x1000,
				 .vsize = 0x0300,
				 .user_data_desc = VC_PDAF_STATS,
			},
	 },*/
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_12bit = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

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
		.pclk = 920000000,
		.linelength = 7128,
		.framelength = 4278,
		.max_framerate = 300,
		.mipi_pixel_rate = 1371428571,
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
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//capture
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = s5kjn5tele_seamless_cap,
		.mode_setting_len = ARRAY_SIZE(s5kjn5tele_seamless_cap),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = s5kjn5tele_seamless_cap,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(s5kjn5tele_seamless_cap),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 7128,
		.framelength = 4278,
		.max_framerate = 300,
		.mipi_pixel_rate = 1371428571,
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
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
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
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 6408,
		.max_framerate = 300,
		.mipi_pixel_rate = 1371428571,
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
		.ae_binning_ratio = 4,
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
		.pclk = 920000000,
		.linelength = 4784,
		.framelength = 1602,
		.max_framerate = 1200,
		.mipi_pixel_rate = 1371428571,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
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
		.pclk = 920000000,
		.linelength = 3072,
		.framelength = 1246,
		.max_framerate = 2400,
		.mipi_pixel_rate = 1371428571,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8192,
			.full_h = 6144,
			.x0_offset = 256,
			.y0_offset = 912,
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
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
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
		.pclk = 920000000,
		.linelength = 7128,
		.framelength = 4278,
		.max_framerate = 300,
		.mipi_pixel_rate = 1371428571,
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
		.ae_binning_ratio = 4,
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
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 6672,
		.framelength = 2293,
		.max_framerate = 600,
		.mipi_pixel_rate = 1371428571,
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
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 4,
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
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 920000000,
		.linelength = 12080,
		.framelength = 6320,
		.max_framerate = 120,
		.mipi_pixel_rate = 1371428571,
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
		.ae_binning_ratio = 5,
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
		.pclk = 920000000,
		.linelength = 7128,
		.framelength = 4278,
		.max_framerate = 300,
		.mipi_pixel_rate = 1371428571,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
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
		.raw_cnt = 2,
		.exp_cnt = 2,
		.hdr_mode = HDR_RAW_STAGGER,
		.pclk = 920000000,
		.linelength = 6000,
		.framelength = 5110,
		.max_framerate = 300,
		.mipi_pixel_rate = 1097142857,
		.readout_length = 0,
		.read_margin = 0,
		.coarse_integ_step = 1,//exp step
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 4,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].max = 0xFFFC,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].max = 0xFFFC,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].min = BASEGAIN * 1,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 80,
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_ME].max = BASEGAIN * 80,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
	{//custom6 AI ISZ
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = s5kjn5tele_seamless_custom6,
		.mode_setting_len = ARRAY_SIZE(s5kjn5tele_seamless_custom6),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = s5kjn5tele_seamless_custom6,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(s5kjn5tele_seamless_custom6),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 920000000,
		.linelength = 9200,
		.framelength = 3332,
		.max_framerate = 300,
		.mipi_pixel_rate = 1371428571,
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
		.imgsensor_pd_info = &imgsensor_pd_info_isz,
		.ae_binning_ratio = 5,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.ana_gain_max = BASEGAIN * 16,
	},
		{//custom7
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
		.pclk = 920000000,
		.linelength = 9568,
		.framelength = 3204,
		.max_framerate = 300,
		.mipi_pixel_rate = 1371428571,
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
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_R,
		.saturation_info = &imgsensor_saturation_info_12bit,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KJN5TELE_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info =  PARAM_UNDEFINED,
	.eeprom_num =  PARAM_UNDEFINED,
	.resolution = {8192, 6144},
	.mirror = IMAGE_H_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 80,
	.ana_gain_type = 2,
	.ana_gain_step = 2,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 50,
	.exposure_def = 0x3D0,
	.exposure_min = 4,//min shutter
	.exposure_max = 0xFFFF - 3,
	.exposure_step = 1,
	.exposure_margin = 4,//margin

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = PARAM_UNDEFINED,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
		{0x0226,0x0227},
		{0x0226,0x0227},
		{0x0202,0x0203}
	},
	.long_exposure_support = TRUE,
	//.reg_addr_exposure_lshift = 0x0702,
	.reg_addr_ana_gain = {
		{0x0206,0x0207},
		{0x0206,0x0207},
		{0x0204,0x0205},
	},
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
	.chk_s_off_end = 1,

	.checksum_value = 0xd086e5a5,
};

static struct subdrv_ops ops = {
	.get_id = s5kjn5tele_get_imgsensor_id,
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
	.vsync_notify = vsync_notify,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_AVDD,  2200000, 1},
	{HW_ID_DVDD,  1020000, 1},
	{HW_ID_RST, 1, 3},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_MCLK, 24, 8},
};

const struct subdrv_entry s5kjn5tele_mipi_raw_entry = {
	.name = "s5kjn5tele_mipi_raw",
	.id = S5KJN5TELE_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static u16 get_gain2reg(u32 gain)
{
	return gain / 32;
}

static int s5kjn5tele_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("s5kjn5tele_set_test_pattern set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
	if (mode == 2)
	{
		subdrv_i2c_wr_u16(ctx,0x0600, 0x0002);
	}
	else if (mode == 5)
	{
		subdrv_i2c_wr_u16(ctx,0x0600, 0x0001);
	} else {
		subdrv_i2c_wr_u16(ctx,0x0600, 0x0000);
	}
	ctx->test_pattern = mode;

	return 0;
}

static int s5kjn5tele_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	struct mtk_test_pattern_data *data = (struct mtk_test_pattern_data *)para;
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	/*subdrv_i2c_wr_u16(ctx, 0x0602, Gr);
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
static void s5kjn5tele_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en)
{
	/*  kal_int16 dummy_line;  */
	u32 frame_length = ctx->frame_length;

	DRV_LOG(ctx,"framerate = %d min_frame_length = %d, min framelength should enable %d\n",framerate, ctx->min_frame_length, min_framelength_en);

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

static int s5kjn5tele_write_shutter(struct subdrv_ctx *ctx, u32 shutter)
{
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
			s5kjn5tele_set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			s5kjn5tele_set_max_framerate(ctx, 146, 0);
	}

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = (u32) shutter;
	/* long expsoure */
	if (shutter
		 > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {
		coarseIntegrationTime = shutter / 128;
		frameLengthLine = coarseIntegrationTime + 10;
		DRV_LOG_MUST(ctx,"Long shutter In shutter = %d frameLength = 0x%x coarseTime = 0x%x\n", shutter, frameLengthLine, coarseIntegrationTime);
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

	DRV_LOG(ctx,"set_shutter shutter =%d, framelength =%d\n", shutter, ctx->frame_length);
	return 0;
}

static int s5kjn5tele_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 shutter = *((u32 *) para);

	s5kjn5tele_write_shutter(ctx, shutter);
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
static u32 s5kjn5tele_gain2reg(struct subdrv_ctx *ctx, const u32 gain)
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
		DRV_LOG(ctx,"Error gain setting");

		if (gain < min_gain)
			gain = min_gain;
		else
			gain = max_gain;
	}

	reg_gain = s5kjn5tele_gain2reg(ctx, gain);
	ctx->gain = reg_gain;

	DRV_LOG(ctx,"gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));
	return gain;
} /* set_gain_16Xgain */

static u32 set_gain_80Xgain(struct subdrv_ctx *ctx, u32 gain)
{
	u32 reg_gain,reg_gainD;
	//gain = 1024 = 1x real gain.
	DRV_LOG(ctx,"++ gain:%d\n",gain);
	if (gain < BASEGAIN || gain > 80 * BASEGAIN) {
		DRV_LOG(ctx,"Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 80 * BASEGAIN)
		{
			reg_gainD = gain / 128;
			DRV_LOG(ctx,"reg_gainD:%d",reg_gainD);
			subdrv_i2c_wr_u16(ctx, 0x020E, (reg_gainD & 0xFFFF));
			subdrv_i2c_wr_u16(ctx, 0x0456, (reg_gainD & 0xFFFF));
			ctx->gain = 0x0400;
			subdrv_i2c_wr_u16(ctx, 0x0204, 0x0400);
			return gain;
		}
	}

	DRV_LOG(ctx,"** gain:%d",gain);
	reg_gain = s5kjn5tele_gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	DRV_LOG(ctx,"-- reg_gain = 0x%x\n ",reg_gain);
	subdrv_i2c_wr_u16(ctx, 0x020E, 0x0100);
	subdrv_i2c_wr_u16(ctx, 0x0456, 0x0100);
	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));

	return gain;
} /* set_gain_80Xgain */

static int s5kjn5tele_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx,"set_gain mode =%d\n", ctx->current_scenario_id);
	switch(ctx->current_scenario_id){
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM5:
			set_gain_80Xgain(ctx, *((u32 *) para));
		break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM6:
			set_gain_16Xgain(ctx, *((u32 *) para));
		break;
		default:
			set_gain_80Xgain(ctx, *((u32 *) para));
		break;
	}
	return 0;
}

static int s5kjn5tele_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;

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
	update_mode_info(ctx, scenario_id);
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

	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		default:
			s5kjn5tele_write_shutter(ctx, ae_ctrl->exposure.le_exposure);
			if(scenario_id == SENSOR_SCENARIO_ID_CUSTOM6){
				set_gain_16Xgain(ctx, ae_ctrl->gain.le_gain);
			}else{
				set_gain_80Xgain(ctx, ae_ctrl->gain.le_gain);
			}
			break;
		}
	}
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	ctx->fast_mode_on = FALSE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}


static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;

	return 0;
}

static int s5kjn5tele_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x000E);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x38e5);
	subdrv_i2c_wr_u16(ctx, 0x6018, 0x0001);
	subdrv_i2c_wr_u16(ctx, 0x7002, 0x0408);
	subdrv_i2c_wr_u16(ctx, 0x6014, 0x0001);
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x2002);
	subdrv_i2c_wr_u16(ctx, 0x1E92, 0x8000);
	subdrv_i2c_wr_u16(ctx, 0x1E84, 0x282B);
	subdrv_i2c_wr_u16(ctx, 0x1E86, 0x0320);
	subdrv_i2c_wr_u16(ctx, 0xFCFC, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x7002, 0x0008);
	mdelay(5);
	//i2c_table_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));
	i2c_table_cntinuburst_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));
	mdelay(5);
	//i2c_table_write(ctx, sensor_fmc_init_addr_data, sizeof(sensor_fmc_init_addr_data)/sizeof(u16));
	i2c_table_cntinuburst_write(ctx, sensor_fmc_init_addr_data, sizeof(sensor_fmc_init_addr_data)/sizeof(u16));
	DRV_LOG(ctx, "X\n");

	return 0;
}

static int s5kjn5tele_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
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
				read_s5kjn5_eeprom_infmacro(ctx);
				s5kjn5tele_get_chip_id(ctx,chip_id);
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

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	pr_info("open+");
	if (s5kjn5tele_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	s5kjn5tele_sensor_init(ctx);

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
	ctx->autoflicker_en = KAL_TRUE;
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

	pr_info("open-");
	return ERROR_NONE;
} /* open */

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u32 temperature = 0;
	int temperature_convert = 0;

	temperature = (subdrv_i2c_rd_u8(ctx, 0x0020)<<8)|subdrv_i2c_rd_u8(ctx, 0x0021);

	if (temperature & 0x8000)
		temperature_convert = -(((~temperature)+1) / 256);
	else
		temperature_convert = temperature / 256;
	if(temperature_convert > 80)
		temperature_convert = 80;
	else if(temperature_convert < -20)
		temperature_convert = -20;

	pr_info("get_sensor_temperature temp_convert(%d), temp_reg(%d)\n", temperature_convert, temperature);

	return temperature_convert;
}

static int vsync_notify(struct subdrv_ctx *ctx, unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if ((ctx->fast_mode_on && ctx->s_ctx.reg_addr_fast_mode) && (sof_cnt > ctx->ref_sof_cnt)) {
		DRV_LOG(ctx, "seamless_switch disabled.");
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		commit_i2c_buffer(ctx);
	}
	ctx->fast_mode_on = FALSE;
	ctx->ref_sof_cnt = 0;
	return 0;
}