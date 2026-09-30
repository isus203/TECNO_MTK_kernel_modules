// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 gc13a0ffmipiraw_Sensor.c
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
#include "gc13a0ffmipiraw_Sensor.h"

#define FPT_PDAF_SUPPORT 0
#define PFX "GC13A0FF"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)
static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int gc13a0ff_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc13a0ff_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int gc13a0ff_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int tran_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int gc13a0ff_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc13a0ff_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);

//extern void gc13a0ff_read_sensor_otp(struct i2c_client *client);
/*static void gc13a0ff_read_sensor_otp(struct subdrv_ctx *ctx);
struct gc13a0ff_otp_struct {
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
struct gc13a0ff_otp_struct gc13a0ff_otp;*/

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc13a0ff_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, gc13a0ff_set_test_pattern_data},
	{SENSOR_FEATURE_SET_ESHUTTER,gc13a0ff_set_shutter},
	{SENSOR_FEATURE_SET_GAIN,gc13a0ff_set_gain},
};

static struct eeprom_info_struct eeprom_info[] = {
	{
		.header_id = 0x00,
		.addr_header_id = 0x00,
		.i2c_write_id = 0x22,

		.xtalk_support = FALSE,
		.xtalk_size = 2048,
		.addr_xtalk = 0x150F,
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2104,
			.vsize = 1560,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4208,
			.vsize = 3120,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4208,
			.vsize = 3120,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2104,
			.vsize = 1560,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1280,
			.vsize = 720,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4208,
			.vsize = 3120,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 2104,
			.vsize = 1560,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 4208,
			.vsize = 3120,
		},
	},

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
		.mode_setting_table = gc13a0ff_2104x1560_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc13a0ff_2104x1560_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 568800000,
		.linelength = 5850,
		.framelength = 3232,
		.max_framerate = 300,
		.mipi_pixel_rate = 241920000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 2104,
			.scale_h = 1560,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2104,
			.h1_size = 1560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2104,
			.h2_tg_size = 1560,
		},
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
		.mode_setting_table = gc13a0ff_4208x3120_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc13a0ff_4208x3120_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 568800000,
		.linelength = 5850,
		.framelength = 3232,
		.max_framerate = 300,
		.mipi_pixel_rate = 476160000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 4208,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 3120,
		},
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
		.mode_setting_table = gc13a0ff_normal_video,
		.mode_setting_len = ARRAY_SIZE(gc13a0ff_normal_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 568800000,
		.linelength = 5850,
		.framelength = 3232,
		.max_framerate = 300,
		.mipi_pixel_rate = 483840000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 378,
			.w0_size = 4208,
			.h0_size = 2364,
			.scale_w = 4208,
			.scale_h = 2364,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 2364,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 2364,
		},
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
		.mode_setting_table = gc13a0ff_2104x1560_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc13a0ff_2104x1560_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 379200000,
		.linelength = 3900,
		.framelength = 3232,
		.max_framerate = 300,
		.mipi_pixel_rate = 241920000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 2104,
			.scale_h = 1560,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2104,
			.h1_size = 1560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2104,
			.h2_tg_size = 1560,
		},
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
		.mode_setting_table = gc13a0ff_1280x720_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc13a0ff_1280x720_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 379200000,
		.linelength = 3900,
		.framelength = 810,
		.max_framerate = 30,
		.mipi_pixel_rate = 168960000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 2104,
			.scale_h = 1560,
			.x1_offset = 412,
			.y1_offset = 420,
			.w1_size = 1280,
			.h1_size = 720,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1280,
			.h2_tg_size = 720,
		},
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
		.mode_setting_table = gc13a0ff_4208x3120_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc13a0ff_4208x3120_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 568800000,
		.linelength = 5850,
		.framelength = 3232,
		.max_framerate = 300,
		.mipi_pixel_rate = 476160000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 4208,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 3120,
		},
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
		.mode_setting_table = gc13a0ff_cus2_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc13a0ff_cus2_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 568800000,
		.linelength = 5850,
		.framelength = 1616,
		.max_framerate = 600,
		.mipi_pixel_rate = 241920000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 2104,
			.scale_h = 1560,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2104,
			.h1_size = 1560,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2104,
			.h2_tg_size = 1560,
		},
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
		.mode_setting_table = gc13a0ff_4208x3120_addr_data,
		.mode_setting_len = ARRAY_SIZE(gc13a0ff_4208x3120_addr_data),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = PARAM_UNDEFINED,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 271200000,
		.readout_length = 0,
		.read_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 4208,
			.full_h = 3120,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 4208,
			.h0_size = 3120,
			.scale_w = 4208,
			.scale_h = 3120,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4208,
			.h1_size = 3120,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4208,
			.h2_tg_size = 3120,
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
	.sensor_id = GC13A0FF_SENSOR_ID,
	.reg_addr_sensor_id = {0x03f0, 0x03f1},
	.i2c_addr_table = {0x72, 0xFF},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {4208, 3120},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
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

	.checksum_value = 0x3353bfd3,
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
			DRV_LOGE(ctx, "i2c_write_id:0x%x sensor_id(cur/exp):0x%x/0x%x\n",
				ctx->i2c_write_id, *sensor_id, ctx->s_ctx.sensor_id);
			if (*sensor_id == ctx->s_ctx.sensor_id){
				//gc13a0ff_read_sensor_otp(ctx->i2c_client);
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

const struct subdrv_entry gc13a0ff_mipi_raw_entry = {
	.name = "gc13a0ff_mipi_raw",
	.id = GC13A0FF_SENSOR_ID,
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
static void gc13a0ff_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en)
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

static int gc13a0ff_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 shutter = *((u32 *) para);
	u16 realtime_fps = 0;
	static bool bNeedSetNormalMode = false;
	u32 cal_shutter = 0;
	u16 long_exp_l = 0;
	u16 long_exp_m = 0;
	u16 long_exp_h = 0;
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
			gc13a0ff_set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			gc13a0ff_set_max_framerate(ctx, 146, 0);
		else
			subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xfffe);
	}

	if (shutter >= 0x3C18 ) { //200ms
		LOG_INF("enter long shutter\n");
		bNeedSetNormalMode = true;
		cal_shutter = (shutter - 0xa00) / 4 - 1;
		long_exp_h = (cal_shutter >> 16) & 0xF;
		long_exp_m = (cal_shutter >> 8) & 0xFF;
		long_exp_l = cal_shutter & 0xFF;
		subdrv_i2c_wr_u16(ctx, 0x0202, 0x0a00);
		subdrv_i2c_wr_u16(ctx, 0x0340, 0x0a10);
		subdrv_i2c_wr_u8(ctx, 0x022f, long_exp_l);
		subdrv_i2c_wr_u8(ctx,0x022e, long_exp_m);
		subdrv_i2c_wr_u8(ctx,0x022d, (0x30 | long_exp_h));
	} else {
		if (bNeedSetNormalMode) {
			LOG_INF("exit long shutter\n");
			subdrv_i2c_wr_u8(ctx,0x022d, 0x20);
			subdrv_i2c_wr_u8(ctx,0x022e, 0x00);
			subdrv_i2c_wr_u8(ctx,0x022f, 0x00);
			bNeedSetNormalMode = false;
		}
		subdrv_i2c_wr_u16(ctx, 0x0202, shutter);
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
	}


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
static u32 gc13a0ff_gain2reg(struct subdrv_ctx *ctx, const u32 gain)
{
	u32 reg_gain = gain;

	reg_gain = (reg_gain < BASEGAIN) ? BASEGAIN : reg_gain;
	reg_gain = (reg_gain > BASEGAIN * 16) ? BASEGAIN * 16 : reg_gain;

	return reg_gain;
}

static int gc13a0ff_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *) para);
	u32 reg_gain = 0;

	reg_gain = gc13a0ff_gain2reg(ctx, gain);
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

static int gc13a0ff_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("gc13a0ff_set_test_pattern set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
	//1:Solid Color 2:Color bar 5:Black
	if (mode == 2)
	{
		subdrv_i2c_wr_u8(ctx,0x008c, 0x01);
	}
	else if (mode == 5) {
		    subdrv_i2c_wr_u8(ctx,0x008c, 0x01);
		    subdrv_i2c_wr_u8(ctx,0x008d, 0x00);
    }
	else
		subdrv_i2c_wr_u8(ctx,0x008c, 0x00);

	ctx->test_pattern = mode;

	return 0;
}

static int gc13a0ff_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
static int gc13a0ff_sensor_init(struct subdrv_ctx *ctx)
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
	gc13a0ff_sensor_init(ctx);

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
