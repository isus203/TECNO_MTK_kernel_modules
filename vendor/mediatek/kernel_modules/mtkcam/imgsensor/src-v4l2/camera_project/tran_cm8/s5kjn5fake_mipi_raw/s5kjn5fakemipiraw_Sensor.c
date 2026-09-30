// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 s5kjn5fakemipiraw_Sensor.c
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
#include "s5kjn5fakemipiraw_Sensor.h"

static u32 previous_exp[2];
static u16 get_gain2reg(u32 gain);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int s5kjn5fake_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int tran_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static u32 s5kjn5fake_gain2reg(struct subdrv_ctx *ctx, const u32 gain);
static void s5kjn5fake_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en);
static void tran_set_dummy(struct subdrv_ctx *ctx);
static int s5kjn5fake_write_shutter(struct subdrv_ctx *ctx, u32 shutter);

static int s5kjn5fake_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_get_preisp_flag(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_get_stream_state(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_set_hdr_tri_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_streaming_control_off(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn5fake_set_tri_dgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int get_sensor_temperature(void *arg);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kjn5fake_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5kjn5fake_set_test_pattern_data},
	{SENSOR_FEATURE_SET_ESHUTTER,s5kjn5fake_set_shutter},
	{SENSOR_FEATURE_SET_GAIN,s5kjn5fake_set_gain},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME,s5kjn5fake_set_multi_shutter_frame_length},
	{SENSOR_FEATURE_CUSTOM_GET_PREISP_FLAG, s5kjn5fake_get_preisp_flag},
	{SENSOR_FEATURE_TRAN_GET_STREAM_STATE,s5kjn5fake_get_stream_state},
	{SENSOR_FEATURE_SET_HDR_SHUTTER,s5kjn5fake_set_hdr_tri_shutter},
	{SENSOR_FEATURE_SET_DUAL_GAIN,s5kjn5fake_set_hdr_tri_gain},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, s5kjn5fake_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_SET_STREAMING_RESUME,s5kjn5fake_streaming_control_on},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND,s5kjn5fake_streaming_control_off},
	{SENSOR_FEATURE_SET_MULTI_DIG_GAIN, s5kjn5fake_set_tri_dgain},
};

static void hdr_write_tri_dgain_w_gph(struct subdrv_ctx *ctx, u32 long_dgain, u32 short_dgain)
{
	u32 short_actual_dgain = short_dgain;
	const u32 short_min_dgain = ctx->s_ctx.dig_gain_min;
	const u32 short_max_dgain = ctx->s_ctx.dig_gain_max;
	const u32 dgain_step = max((u32)1, ctx->s_ctx.dig_gain_step);

	if (short_actual_dgain < short_min_dgain) {
		short_actual_dgain = short_min_dgain;
	} else if (short_actual_dgain > short_max_dgain) {
		short_actual_dgain = short_max_dgain;
	}

	subdrv_i2c_wr_u16(ctx, 0x020E, ((short_actual_dgain / dgain_step / 0x100) << 8) & 0xFFFF);
	DRV_LOG(ctx, "long dgain %u reg 0x%x short dgain %u reg 0x%x max:min %u %u\n",
		long_dgain, subdrv_i2c_rd_u16(ctx, 0x0230), short_actual_dgain, subdrv_i2c_rd_u16(ctx, 0x020E),
		short_max_dgain, short_min_dgain);
}

static int s5kjn5fake_set_tri_dgain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 preisp_mode = *((u32 *)(feature_data + 2));
	u32 *long_dgain = NULL;
	u32 *short_dgain = NULL;

	if ((1 == preisp_mode) && (5 != ctx->test_pattern)) {
		long_dgain = (u32 *)(*feature_data);
		short_dgain = (long_dgain + 1);
		DRV_LOG(ctx, "SENSOR_FEATURE_SET_MULTI_DIG_GAIN L_dig = %u S_dig=%u\n", *long_dgain, *short_dgain);
		hdr_write_tri_dgain_w_gph(ctx, *long_dgain, *short_dgain);
	} else {
		set_multi_dig_gain(ctx, (u32 *)(*feature_data), (u16) (*(feature_data + 1)));
	}

	return 0;
}

static void tran_set_dummy(struct subdrv_ctx *ctx)
{
	if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5){
		DRV_LOG(ctx,"tran_set_dummy before framelength =%d\n",  ctx->frame_length);
		ctx->frame_length /=1;
	}
	subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xFFFF);
	DRV_LOG(ctx,"tran_set_dummy framelength =%d\n",  ctx->frame_length);
}

//stream on
static void s5kjn5fake_streaming_on(struct subdrv_ctx *ctx, bool enable)
{
	struct adaptor_ctx *_adaptor_ctx = NULL;
	struct v4l2_subdev *sd = NULL;

	DRV_LOG(ctx,"E! enable:%u\n", enable);

	if (ctx->i2c_client)
		sd = i2c_get_clientdata(ctx->i2c_client);
	if (sd)
		_adaptor_ctx = to_ctx(sd);
	if (!_adaptor_ctx) {
		DRV_LOGE(ctx, "null _adaptor_ctx\n");
		return;
	}

	check_current_scenario_id_bound(ctx);

	if (enable) {
		tran_set_dummy(ctx);
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
	}else {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
		if (ctx->s_ctx.reg_addr_fast_mode) {
			ctx->fast_mode_on = FALSE;
			ctx->ref_sof_cnt = 0;
			DRV_LOG(ctx, "seamless_switch disabled.");
			subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
		}
		memset(ctx->exposure, 0, sizeof(ctx->exposure));
		memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
		ctx->autoflicker_en = FALSE;
		ctx->extend_frame_length_en = 0;
		ctx->is_seamless = 0;
		if (ctx->s_ctx.chk_s_off_end)
			check_stream_off(ctx);
		ctx->stream_ctrl_start_time = 0;
		ctx->stream_ctrl_end_time = 0;
	}
	ctx->sof_no = 0;
	ctx->is_streaming = enable;
	DRV_LOG(ctx,"X! enable:%u\n", enable);
}

static int s5kjn5fake_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;

	if (*feature_data) {
		s5kjn5fake_write_shutter(ctx, *feature_data);
	}
	s5kjn5fake_streaming_on(ctx, TRUE);
    return 0;
}

static int s5kjn5fake_streaming_control_off(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	s5kjn5fake_streaming_on(ctx, FALSE);

    return 0;
}

static int s5kjn5fake_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

	if((scenario_id == SENSOR_SCENARIO_ID_CUSTOM5 && ctx->s_ctx.mode[scenario_id].framelength == 5112) && framerate ==300){
		DRV_LOG(ctx, "custom5 max fps is 30\n");
		return ERROR_NONE;
	}

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;

	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	if((ctx->frame_length % 2) != 0){
		ctx->frame_length = ctx->frame_length - 1;
	}

	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;
	DRV_LOG(ctx, "max_fps(input/output):%u/%u(sid:%u), ctx->frame_length:%d exposure_margin:%d shutter:%d\n", framerate, ctx->current_fps, scenario_id, ctx->frame_length, ctx->exposure[0], ctx->s_ctx.exposure_margin);
	//if(ctx->current_scenario_id != SENSOR_SCENARIO_ID_CUSTOM5){
		if (ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin)){
			tran_set_dummy(ctx);
		}
	//}
	ctx->frame_length_rg = ctx->frame_length;
	return ERROR_NONE;
}

/*
1.L_shutter(current AE)+S_shutter(current AE)+48< framelenth(current AE)
2.L_shutter(next AE)+S_shutter(current AE)+48< framelenth(next AE)
3.L_shutter>=S_shutter
4.L_shutter ,S_shutter 需要2的整数倍
*/
static void s5kjn5fake_set_hdr_multi_shutter(struct subdrv_ctx *ctx, u64 *shutters)
{
	u32 TE = 0, temp_fll = 0;
	u16 realtime_fps = 0;

	DRV_LOG(ctx,"para shutters[0] =%llu, shutters[1] =%llu, framelength =%d\n", shutters[0], shutters[1], ctx->frame_length);

	shutters[0] = round_up((shutters[0]) / 2, 4) * 2;

	shutters[1] = round_up((shutters[1]) / 2, 4) * 2;

	TE = shutters[0] + shutters[1] + 24 * 2;
	ctx->frame_length = max(TE,ctx->min_frame_length);

	//（se > 742）fll = se +margin *2 + height *2
	temp_fll = shutters[1] / 2+ 2160 *2 + 24 * 2;
	ctx->frame_length = max(ctx->frame_length, temp_fll);

	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	DRV_LOG(ctx,"read before shutters[0] =%d, shutters[1] =%d framelength =%d frame_cnt = %d\n", subdrv_i2c_rd_u16(ctx, 0X0226), subdrv_i2c_rd_u16(ctx, 0X0202), subdrv_i2c_rd_u16(ctx, 0x0340),subdrv_i2c_rd_u8(ctx, 0x0005));
	DRV_LOG(ctx,"act shutters[0] =%llu, previous_exp[0] =%d, previous_exp[1] =%d, framelength =%d  autoflicker_en=%d\n", shutters[0], previous_exp[0], previous_exp[1], ctx->frame_length, ctx->autoflicker_en);
	if (ctx->autoflicker_en) {
		realtime_fps
			= ctx->pclk
			/ ctx->line_length * 10
			/ ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			s5kjn5fake_set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			s5kjn5fake_set_max_framerate(ctx, 146, 0);
	}
	//L_shutter(next AE)+S_shutter(current AE)+48 *2< framelenth(next AE)
	if (ctx->frame_length < (shutters[0] + previous_exp[1] + 24 * 2)) {
		ctx->frame_length = shutters[0] + previous_exp[1] + 24 * 2;
		DRV_LOG(ctx,"[case1] adjust fll: %d", ctx->frame_length);
	}

	subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xFFFF);
	subdrv_i2c_wr_u16(ctx, 0X0226, shutters[0]  & 0xFFFF);
	subdrv_i2c_wr_u16(ctx, 0X0202, shutters[1] & 0xFFFF);

	previous_exp[0] = shutters[0];
	previous_exp[1] = shutters[1];
	ctx->frame_length_rg = ctx->frame_length;
	DRV_LOG(ctx,"write shutters[0] =%llu, shutters[1] =%llu, framelength =%d\n", shutters[0], shutters[1], ctx->frame_length);
	DRV_LOG(ctx,"read after shutters[0] =%d, shutters[1] =%d framelength =%d frame_cnt = %d\n", subdrv_i2c_rd_u16(ctx, 0X0226), subdrv_i2c_rd_u16(ctx, 0X0202), subdrv_i2c_rd_u16(ctx, 0x0340),subdrv_i2c_rd_u8(ctx, 0x0005));
}

static int s5kjn5fake_set_hdr_tri_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	int i = 0;
	u64 values[2] = {0};
	u64 * temp_shutter = (u64 *)para;

	if (para != NULL) {
		for (i = 0; i < 2; i++)
			values[i] = *(temp_shutter + i);
	}

	s5kjn5fake_set_hdr_multi_shutter(ctx, values);
	return 0;
}

static int s5kjn5fake_set_hdr_tri_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	int i = 0;
	u64 values[2] = {0};
	u64 reg_gain[2] = {0};
	u32 min_gain = 1 * BASEGAIN;
	u32 max_gain = 80 * BASEGAIN;
	u64 * temp_gain = (u64 *)para;

	if (para != NULL) {
		for (i = 0; i < 2; i++){
			if(*(temp_gain + i) != 0){
				values[i] = *(temp_gain + i);
				if(values[i] < min_gain || values[i] > max_gain) {
					DRV_LOG(ctx,"Error gain setting");
					if (values[i] < min_gain)
						values[i] = min_gain;
					else
						values[i] = max_gain;
				}
			}
			reg_gain[i] = s5kjn5fake_gain2reg(ctx, values[i]);
			ctx->ana_gain[i] = values[i];
		}
	}

	DRV_LOG(ctx,"gain[0] = %llu, gian[1]=%llu, reg_gain[0] = %llu, reg_gain[1] = %llu\n ", values[0], values[1], reg_gain[0], reg_gain[1]);
	subdrv_i2c_wr_u16(ctx, 0x0206, (reg_gain[0] & 0xFFFF));
	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain[1] & 0xFFFF));
	return 0;
}


static int s5kjn5fake_get_stream_state(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (UINT32 *) para;
	int state = 0;

	state = subdrv_i2c_rd_u8(ctx, 0x0100);

	*(feature_return_para_32 + 1) = state;
	DRV_LOG(ctx,"s5kjn5fake_get_stream_state state = %d\n", state);

	return 0;
}

static int s5kjn5fake_get_preisp_flag(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	DRV_LOG(ctx,"[%s] mode: %lld\n", __func__, *feature_data);
	if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM5){
		*(u32 *)(uintptr_t)(*(feature_data + 1)) = 1;
	}else{
		*(u32 *)(uintptr_t)(*(feature_data + 1)) = 0;
	}

	return 0;
}

static void s5kjn5fake_set_multi_shutter(struct subdrv_ctx *ctx,
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

static int s5kjn5fake_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	s5kjn5fake_set_multi_shutter(ctx, (UINT32 *)(*feature_data), (UINT16) (*(feature_data + 1)), (UINT16) (*(feature_data + 2)));

	return 0;
}

static struct mtk_sensor_saturation_info imgsensor_saturation_info_14bit_dol = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 16383,
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
    /*{
         .bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1400,
			.vsize = 0x0300,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
         },
    },*/
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
    /*{
         .bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1400,
			.vsize = 0x0300,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
         },
    },*/
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
    /*{
         .bus.csi2 = {
             .channel = 0,
             .data_type = 0x2b,
             .hsize = 0x1400,
             .vsize = 0x0240,
             .user_data_desc = VC_PDAF_STATS,
         },
    },*/
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
	/*{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0a00,
			.vsize = 0x120,
			.user_data_desc = VC_PDAF_STATS,
		},
	},*/
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
    /*{
         .bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1400,
			.vsize = 0x0300,
			//.dt_remap_to_type = MTK_MBUS_FRAME_DESC_REMAP_TO_RAW10,
			.user_data_desc = VC_PDAF_STATS,
         },
    },*/
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
	/*{
        .bus.csi2 = {
             .channel = 0,
             .data_type = 0x30,
             .hsize = 0x0258,
             .vsize = 0x0210,
             .user_data_desc = VC_PDAF_STATS,
        },
    },*/
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
			.data_type = 0x2d,
			.hsize = 4096,
			.vsize = 2304,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
#if 1
    {
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x1000,
			.vsize = 0x11,
			.user_data_desc = VC_GENERAL_EMBEDDED,
		},
	},
#endif
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
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2d,
			.hsize = 0x0F00,
			.vsize = 0x0870,
		},
	},
#if 1
    {
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x30,
			.hsize = 0x0F00,
			.vsize = 0x1D,
			.user_data_desc = VC_GENERAL_EMBEDDED,
		},
	},
#endif
	{
		.bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x0F00,
			.vsize = 0x021C,
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

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
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
				{0, 0}, {128, 456}, {0, 0}, {0, 0}, {0, 384}, {2048,1536}
			},
	.iMirrorFlip = 1,
	.i4FullRawW = 3840,
	.i4FullRawH = 2160,
	.i4ModeIndex = 3,
	.sPDMapInfo[0] = {
		.i4PDPattern = 1,
	},
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
		.mipi_pixel_rate = 1097142857,
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
		.pclk = 920000000,
		.linelength = 7128,
		.framelength = 4278,
		.max_framerate = 300,
		.mipi_pixel_rate = 1097142857,
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
		.linelength = 9568,
		.framelength = 3204,
		.max_framerate = 300,
		.mipi_pixel_rate = 1097142857,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.linelength = 9600,
		.framelength = 6346,
		.max_framerate = 150,
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
		.linelength = 9568,
		.framelength = 3204,
		.max_framerate = 300,
		.mipi_pixel_rate = 1097142857,
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
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW14_R,
		.saturation_info = &imgsensor_saturation_info_14bit_dol,
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
		.pclk = 920000000,
		.linelength = 6000,
		.framelength = 5112,
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
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 4,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW14_R,
		.saturation_info = &imgsensor_saturation_info_14bit_dol,
		.csi_param = {0},
	},
	{//custom6
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
			if (*sensor_id == (ctx->s_ctx.sensor_id & 0xFFFF))
				return ERROR_NONE;
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

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KJN5FAKE_SENSOR_ID,
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
	.dig_gain_min = BASEGAIN * 1,
	.dig_gain_max = BASEGAIN * 16,
	.dig_gain_step = 4,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = FALSE,
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
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_AVDD,  2200000, 1},
	{HW_ID_DVDD,  1020000, 1},
	{HW_ID_RST, 1, 1},
	{HW_ID_MCLK, 24, 8},
};

const struct subdrv_entry s5kjn5fake_mipi_raw_entry = {
	.name = "s5kjn5fake_mipi_raw",
	.id = S5KJN5FAKE_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */

static u16 get_gain2reg(u32 gain)
{
	return gain / 32;
}

static int s5kjn5fake_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	DRV_LOG(ctx,"s5kjn5fake_set_test_pattern set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
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

static int s5kjn5fake_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
static void s5kjn5fake_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en)
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

static int s5kjn5fake_write_shutter(struct subdrv_ctx *ctx, u32 shutter)
{
	u16 realtime_fps = 0;
	u32 coarseIntegrationTime = 0x0;
	u32 frameLengthLine = 0x0;
	ctx->shutter = shutter;

	if((ctx->shutter % 2) != 0){
		ctx->shutter = ctx->shutter + 1;
	}

	if (shutter > ctx->min_frame_length - ctx->s_ctx.exposure_margin)
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
	else
		ctx->frame_length = ctx->min_frame_length;

	if (ctx->frame_length > ctx->s_ctx.frame_length_max)
		ctx->frame_length = ctx->s_ctx.frame_length_max;

	if (shutter < ctx->s_ctx.exposure_min)
		shutter = ctx->s_ctx.exposure_min;

	if((ctx->frame_length % 2) != 0){
		ctx->frame_length = ctx->frame_length + 1;
	}

	if (ctx->autoflicker_en) {
		realtime_fps
			= ctx->pclk
			/ ctx->line_length * 10
			/ ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			s5kjn5fake_set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			s5kjn5fake_set_max_framerate(ctx, 146, 0);
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

static int s5kjn5fake_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 shutter = *((u32 *) para);

	s5kjn5fake_write_shutter(ctx, shutter);
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
static u32 s5kjn5fake_gain2reg(struct subdrv_ctx *ctx, const u32 gain)
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

	reg_gain = s5kjn5fake_gain2reg(ctx, gain);
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
	reg_gain = s5kjn5fake_gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	DRV_LOG(ctx,"-- reg_gain = 0x%x\n ",reg_gain);
	subdrv_i2c_wr_u16(ctx, 0x020E, 0x0100);
	subdrv_i2c_wr_u16(ctx, 0x0456, 0x0100);
	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));
	return gain;
} /* set_gain_80Xgain */

static int s5kjn5fake_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;

	return 0;
}

static int s5kjn5fake_sensor_init(struct subdrv_ctx *ctx)
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

static int open(struct subdrv_ctx *ctx)
{
	u32 sensor_id = 0;
	u32 scenario_id = 0;

	/* get sensor id */
	if (tran_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	s5kjn5fake_sensor_init(ctx);

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

	DRV_LOG(ctx,"get_sensor_temperature temp_convert(%d), temp_reg(%d)\n", temperature_convert, temperature);

	return temperature_convert;
}
