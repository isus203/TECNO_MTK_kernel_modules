// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 gc02m3macromipiraw_Sensor.c
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
#include "gc02m3macromipiraw_Sensor.h"

#define FPT_PDAF_SUPPORT 0
#define PFX "GC02M3MACRO"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int gc02m3macro_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc02m3macro_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int gc02m3macro_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int tran_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int gc02m3macro_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc02m3macro_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);

//extern void gc02m3macro_read_sensor_otp(struct i2c_client *client);
/*static void gc02m3macro_read_sensor_otp(struct subdrv_ctx *ctx);
struct gc02m3macro_otp_struct {
	kal_uint8 awb_group_flag;
	kal_uint8 awb_data[14]; //12data 2reserved
	kal_uint8 awb_checksum;
	kal_uint8 af_group_flag;
	kal_uint8 af_data[14]; //12data 2reserved
	kal_uint8 af_checksum;
	kal_uint8 lsc_group_flag;
	kal_uint8 lsc_data[1868];
	kal_uint8 lsc_checksum;
};
struct gc02m3macro_otp_struct gc02m3macro_otp;*/

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc02m3macro_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, gc02m3macro_set_test_pattern_data},
	{SENSOR_FEATURE_SET_ESHUTTER,gc02m3macro_set_shutter},
	{SENSOR_FEATURE_SET_GAIN,gc02m3macro_set_gain},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1600,
			.vsize = 1200,
		},
	},

};


static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

#define WINSIZE_INFO_PRE  {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200} /* Preview */
#define WINSIZE_INFO_CAP  {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200} /* capture */
#define WINSIZE_INFO_VID  {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200} /* video */
#define WINSIZE_INFO_HS   {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200} /* hs video */
#define WINSIZE_INFO_SLIM {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200} /* slim video */
#define WINSIZE_INFO_CUS1 {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200} 
#define WINSIZE_INFO_CUS2 {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200} 
#define WINSIZE_INFO_CUS3 {1600, 1200, 0, 0, 1600, 1200, 1600, 1200, 0000, 0000, 1600, 1200, 0, 0, 1600, 1200} 
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
		.pclk = 109000000,
		.linelength = 2788,
		.framelength = 1302,
		.max_framerate = 300,
		.mipi_pixel_rate = 65400000,
		.readout_length = 0,
		.read_margin = 8,
		.imgsensor_winsize_info = WINSIZE_INFO_PRE,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.pclk = 109000000,
		.linelength = 2788,
		.framelength = 1302,
		.max_framerate = 300,
		.mipi_pixel_rate = 65400000,
		.readout_length = 0,
		.read_margin = 8,
		.imgsensor_winsize_info = WINSIZE_INFO_CAP,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.pclk = 109000000,
		.linelength = 2788,
		.framelength = 1302,
		.max_framerate = 300,
		.mipi_pixel_rate = 65400000,
		.readout_length = 0,
		.read_margin = 8,
		.imgsensor_winsize_info = WINSIZE_INFO_VID,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.pclk = 109000000,
		.linelength = 2788,
		.framelength = 1302,
		.max_framerate = 300,
		.mipi_pixel_rate = 65400000,
		.readout_length = 0,
		.read_margin = 8,
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
		.pclk = 109000000,
		.linelength = 2788,
		.framelength = 1302,
		.max_framerate = 300,
		.mipi_pixel_rate = 65400000,
		.readout_length = 0,
		.read_margin = 8,
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
		.pclk = 109000000,
		.linelength = 2788,
		.framelength = 1302,
		.max_framerate = 300,
		.mipi_pixel_rate = 65400000,
		.readout_length = 0,
		.read_margin = 8,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS1,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.pclk = 109000000,
		.linelength = 2788,
		.framelength = 1302,
		.max_framerate = 300,
		.mipi_pixel_rate = 65400000,
		.readout_length = 0,
		.read_margin = 8,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS2,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
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
		.pclk = 109000000,
		.linelength = 2788,
		.framelength = 1302,
		.max_framerate = 300,
		.mipi_pixel_rate = 65400000,
		.readout_length = 0,
		.read_margin = 8,
		.imgsensor_winsize_info = WINSIZE_INFO_CUS3,
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = GC02M3MACRO_SENSOR_ID,
	.reg_addr_sensor_id = {0x03f0, 0x03f1},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	//.eeprom_info = eeprom_info,
	//.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {1600, 1200},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_1_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 1,
	.ana_gain_step = 1,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 100,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xfffe,
	.exposure_step = 1,
	.exposure_margin = 16,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 1,
	.dig_gain_step = 4,
	.saturation_info = &imgsensor_saturation_info,

	.frame_length_max = 0xfffe,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_NA,
	//.hdr_type = HDR_SUPPORT_STAGGER_FDOL|HDR_SUPPORT_DCG|HDR_SUPPORT_LBMF,
	.hdr_type = 0,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,
	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {{0x0202},},
	.long_exposure_support = FALSE,
	//.reg_addr_exposure_lshift = 0x0702,
	.reg_addr_ana_gain = {{0x0204},},
	.reg_addr_frame_length = {{0x0340},},
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

	.checksum_value = 0x1246a0f3,
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
			DRV_LOGE(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == ctx->s_ctx.sensor_id){
				//gc02m3macro_read_sensor_otp(ctx->i2c_client);
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
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1200000, 1},
	{HW_ID_AVDD, 2800000, 1},
	{HW_ID_RST, 1, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_MCLK, 24, 5},
};

const struct subdrv_entry gc02m3macro_mipi_raw_entry = {
	.name = "gc02m3macro_mipi_raw",
	.id = GC02M3MACRO_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
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
static void gc02m3macro_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en)
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

	subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xfffe);

}

static int gc02m3macro_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 shutter = *((u32 *) para);
	u16 realtime_fps = 0;
	ctx->shutter = shutter;

	if (shutter > ctx->min_frame_length - ctx->s_ctx.exposure_margin)
		ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
	else
		ctx->frame_length = ctx->min_frame_length;

	if (ctx->frame_length > ctx->s_ctx.frame_length_max)
		ctx->frame_length = ctx->s_ctx.frame_length_max;

	if (shutter < ctx->s_ctx.exposure_min)
		shutter = ctx->s_ctx.exposure_min;

	LOG_INF("ctx->autoflicker_en =%d\n",ctx->autoflicker_en);
	if (ctx->autoflicker_en) {
		realtime_fps
			= ctx->pclk
			/ ctx->line_length * 10
			/ ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			gc02m3macro_set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			gc02m3macro_set_max_framerate(ctx, 146, 0);
		else
			subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xfffe);
	}
	else {
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
	}
	subdrv_i2c_wr_u16(ctx, 0x0202, shutter);
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
static u32 gc02m3macro_gain2reg(struct subdrv_ctx *ctx, const u32 gain)
{
	u32 reg_gain = gain;

	reg_gain = (reg_gain < BASEGAIN) ? BASEGAIN : reg_gain;
	reg_gain = (reg_gain > BASEGAIN * 16) ? BASEGAIN * 16 : reg_gain;

	return reg_gain;
}

static int gc02m3macro_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *) para);
	u32 reg_gain = 0;

	reg_gain = gc02m3macro_gain2reg(ctx, gain);
	ctx->gain = reg_gain;

	LOG_INF("gain = %d, reg_gain = 0x%x \n ", gain, reg_gain);
	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));

	return gain;
} /* set_gain*/

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
	return gain << 4;
}

static int gc02m3macro_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("gc02m3macro_set_test_pattern set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
	//1:Solid Color 2:Color bar 5:Black
	if (mode == 2)
	{
		subdrv_i2c_wr_u8(ctx,0x008c, 0x04);
	}
	else if (mode == 5) {
		    subdrv_i2c_wr_u8(ctx,0x008c, 0x04);
		    subdrv_i2c_wr_u8(ctx,0x008d, 0x00);
    }
	else
		subdrv_i2c_wr_u8(ctx,0x008c, 0x00);

	ctx->test_pattern = mode;

	return 0;
}

static int gc02m3macro_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

/*#define AWBINFO_FLAG_ADDR           0x1698
#define AFINFO_FLAG_ADDR            0x1790
#define LSCINFO_FLAG_ADDR           0x1888

#define AWB_GROUP1_START_ADDR       0x16A0
#define AWB_GROUP2_START_ADDR       0x1718
#define AWB_GROUP1_GOLDEN_DATA_ADDR 0x16D0
#define AWB_GROUP2_GOLDEN_DATA_ADDR 0x1748
#define AF_GROUP1_START_ADDR        0x1798
#define AF_GROUP2_START_ADDR        0x1810
#define LSC_GROUP1_START_ADDR       0x1890
#define LSC_GROUP2_START_ADDR       0x52F8

#define AWB_GROUP1_CHECKSUM_ADDR    0x1710
#define AWB_GROUP2_CHECKSUM_ADDR    0x1788
#define AF_GROUP1_CHECKSUM_ADDR     0x1808
#define AF_GROUP2_CHECKSUM_ADDR     0x1880
#define LSC_GROUP1_CHECKSUM_ADDR    0x52F0
#define LSC_GROUP2_CHECKSUM_ADDR    0x8D58

static void otp_init_setting(struct subdrv_ctx *ctx)
{
	subdrv_i2c_wr_u8(ctx ,0x031c, 0x60);
	subdrv_i2c_wr_u8(ctx ,0x0315, 0x80);
	mdelay(10);
	subdrv_i2c_wr_u8(ctx ,0x0324, 0x42);
	subdrv_i2c_wr_u8(ctx ,0x0316, 0x09);
	subdrv_i2c_wr_u8(ctx ,0x0a67, 0x80);
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x00);
	subdrv_i2c_wr_u8(ctx ,0x0a53, 0x0e);
	subdrv_i2c_wr_u8(ctx ,0x0a65, 0x17);
	subdrv_i2c_wr_u8(ctx ,0x0a68, 0xa1);
	subdrv_i2c_wr_u8(ctx ,0x0a47, 0x00);
	subdrv_i2c_wr_u8(ctx ,0x0a58, 0x00);
	subdrv_i2c_wr_u8(ctx ,0x0ace, 0x0c);
	mdelay(10);
}

static kal_uint8 gc02m3macro_read_otp_single(struct subdrv_ctx *ctx,kal_uint16 addr)
{
	kal_uint8 data;
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x00);
	subdrv_i2c_wr_u8(ctx ,0x0a69, (addr >> 8) & 0xff);
	subdrv_i2c_wr_u8(ctx ,0x0a6a, addr & 0xff);
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x20);
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x12);
	mdelay(5);

	data = (kal_uint8)subdrv_i2c_rd_u8(ctx,0x0a6c);
	pr_info("gc02m3macro_read_otp_single single data = %d", data);

	return data;
}

static void gc02m3macro_read_otp_continuous(struct subdrv_ctx *ctx,kal_uint16 addr, kal_uint16 length, kal_uint8 *data)
{
	int i;
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x00);
	subdrv_i2c_wr_u8(ctx ,0x0a69, (addr >> 8) & 0xff);
	subdrv_i2c_wr_u8(ctx ,0x0a6a, addr & 0xff);
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x20);
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x12);
	mdelay(5);

	for(i = 0;i < length; i++)
	{
		data[i] = (kal_uint8)subdrv_i2c_rd_u8(ctx,0x0a6c);
		pr_info("gc02m3macro_read_otp_continuous continuous data[%d] = %d", i, data[i]);
	}
}

static void gc02m3macro_read_sensor_otp(struct subdrv_ctx *ctx)
{
	kal_uint8 awb_flag, af_flag, lsc_flag;

	otp_init_setting(ctx);
	//awb data
	awb_flag = gc02m3macro_read_otp_single(ctx,AWBINFO_FLAG_ADDR);
	printk("gc02m3macro sensor otp awb_flag = %d", awb_flag);
	if(awb_flag == 0x01)
	{
		gc02m3macro_otp.awb_group_flag = 1;
		gc02m3macro_read_otp_continuous(ctx,AWB_GROUP1_START_ADDR, 14, gc02m3macro_otp.awb_data);
		gc02m3macro_otp.awb_checksum = gc02m3macro_read_otp_single(ctx,AWB_GROUP1_CHECKSUM_ADDR);
	}
	else if(awb_flag == 0x1F)
	{
		gc02m3macro_otp.awb_group_flag = 2;
		gc02m3macro_read_otp_continuous(ctx,AWB_GROUP2_START_ADDR, 14, gc02m3macro_otp.awb_data);
		gc02m3macro_otp.awb_checksum = gc02m3macro_read_otp_single(ctx,AWB_GROUP2_CHECKSUM_ADDR);
	}
	else
		printk("gc02m3macro sensor otp awb_flag invalid");

	//af data
	af_flag = gc02m3macro_read_otp_single(ctx,AFINFO_FLAG_ADDR);
	printk("gc02m3macro sensor otp af_flag = %d", af_flag);
	if(af_flag == 0x01)
	{
		gc02m3macro_otp.af_group_flag = 1;
		gc02m3macro_read_otp_continuous(ctx,AF_GROUP1_START_ADDR, 14, gc02m3macro_otp.af_data);
		gc02m3macro_otp.af_checksum = gc02m3macro_read_otp_single(ctx,AF_GROUP1_CHECKSUM_ADDR);
	}
	else if(af_flag == 0x1F)
	{
		gc02m3macro_otp.af_group_flag = 2;
		gc02m3macro_read_otp_continuous(ctx,AF_GROUP2_START_ADDR, 14, gc02m3macro_otp.af_data);
		gc02m3macro_otp.af_checksum = gc02m3macro_read_otp_single(ctx,AF_GROUP2_CHECKSUM_ADDR);
	}
	else
		printk("gc02m3macro sensor otp af_flag invalid");

	//lsc data
	lsc_flag = gc02m3macro_read_otp_single(ctx,LSCINFO_FLAG_ADDR);
	printk("gc02m3macro sensor otp lsc_flag = %d", lsc_flag);
	if(lsc_flag == 0x01)
	{
		gc02m3macro_otp.lsc_group_flag = 1;
		gc02m3macro_read_otp_continuous(ctx,LSC_GROUP1_START_ADDR, 1868, gc02m3macro_otp.lsc_data);
		gc02m3macro_otp.lsc_checksum = gc02m3macro_read_otp_single(ctx,LSC_GROUP1_CHECKSUM_ADDR);
	}
	else if(lsc_flag == 0x1F)
	{
		gc02m3macro_otp.lsc_group_flag = 2;
		gc02m3macro_read_otp_continuous(ctx,LSC_GROUP2_START_ADDR, 1868, gc02m3macro_otp.lsc_data);
		gc02m3macro_otp.lsc_checksum = gc02m3macro_read_otp_single(ctx,LSC_GROUP2_CHECKSUM_ADDR);
	}
	else
		printk("gc02m3macro sensor otp lsc_flag invalid");

	//otp_end_read
	subdrv_i2c_wr_u8(ctx,0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx,0x0a67, 0x00);
}

unsigned int gc02m3macro_read_region(struct i2c_client *client, unsigned int addr,
			unsigned char *data, unsigned int size)
{
	pr_info("gc02m3macro_read_region addr = 0x%x, size = %d", addr, size);

	if(addr == 0 && size == 0x1500) //Avoiding preloading, hal print error log "Preload data failed"
		return 0;
	else if(addr == AWB_GROUP1_START_ADDR && size == 4) //awb data
	{
		memcpy(data, gc02m3macro_otp.awb_data, size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == AWB_GROUP1_GOLDEN_DATA_ADDR && size == 4) //awb golden data
	{
		memcpy(data, &gc02m3macro_otp.awb_data[6], size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == AWB_GROUP1_START_ADDR && size == 14) //all awb data
	{
		memcpy(data, gc02m3macro_otp.awb_data, size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == AWB_GROUP1_CHECKSUM_ADDR)  //awb checksum
	{
		*data = gc02m3macro_otp.awb_checksum;
		return size;
	}
	else if(addr == AF_GROUP1_START_ADDR && size == 6)  //af data
	{
		memcpy(data, &gc02m3macro_otp.af_data[6], size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == AF_GROUP1_START_ADDR && size == 14) //all af data
	{
		memcpy(data, gc02m3macro_otp.af_data, size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == AF_GROUP1_CHECKSUM_ADDR)  //af checksum
	{
		*data = gc02m3macro_otp.af_checksum;
		return size;
	}
	else if(addr == (LSC_GROUP1_START_ADDR-1)) //lsc flag
	{
		*data = gc02m3macro_otp.lsc_group_flag ? 1 : 0;
		return size;
	}
	else if(addr == LSC_GROUP1_START_ADDR && size == 1868) //lsc data
	{
		memcpy(data, gc02m3macro_otp.lsc_data, size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == (LSC_GROUP1_START_ADDR+1868)) //lsc checksum
	{
		*data = gc02m3macro_otp.lsc_checksum;
		return size;
	}
	return 0;
}EXPORT_SYMBOL(gc02m3macro_read_region);*/


static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;

	return 0;
}
static int gc02m3macro_sensor_init(struct subdrv_ctx *ctx)
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
	DRV_LOG(ctx, "wufei open\n");
	/* get sensor id */
	if (tran_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	gc02m3macro_sensor_init(ctx);/*  */

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
