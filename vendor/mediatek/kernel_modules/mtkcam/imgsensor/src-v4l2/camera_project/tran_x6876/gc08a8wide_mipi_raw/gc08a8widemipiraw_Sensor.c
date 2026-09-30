// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 gc08a8widemipiraw_Sensor.c
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
#include "gc08a8widemipiraw_Sensor.h"

static u16 get_gain2reg(u32 gain);
static int gc08a8wide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a8wide_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_sensor_temperature(void *arg);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int gc08a8wide_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int tran_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static int gc08a8wide_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a8wide_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc08a8_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
unsigned int gc08a8wide_read_region(struct i2c_client *client, unsigned int addr, unsigned char *data, unsigned int size);
static void gc08a8wide_read_sensor_otp(struct subdrv_ctx *ctx);
static int gc08a8wide_feature_get_chip_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static kal_uint8 gc08a8wide_read_otp_single(struct subdrv_ctx *ctx,kal_uint16 addr);
struct gc08a8wide_otp_struct {
	kal_uint8 awb_group_flag;
	kal_uint8 awb_data[14]; //12data 2reserved
	kal_uint8 awb_checksum;
	kal_uint8 awb_flag;
	kal_uint8 awb_group_flag_4000k;
	kal_uint8 awb_data_4000k[13];
	kal_uint8 awb_checksum_4000k;
	kal_uint8 awb_flag_4000k;
	kal_uint8 awb_group_flag_2800k;
	kal_uint8 awb_data_2800k[13];
	kal_uint8 awb_checksum_2800k;
	kal_uint8 awb_flag_2800k;
	kal_uint8 af_group_flag;
	kal_uint8 af_flag;
	kal_uint8 af_data[14]; //12data 2reserved
	kal_uint8 af_checksum;
	kal_uint8 lsc_group_flag;
	kal_uint8 lsc_data[1868];
	kal_uint8 lsc_checksum;
};
struct gc08a8wide_otp_struct gc08a8wide_otp;

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, gc08a8wide_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, gc08a8wide_set_test_pattern_data},
	{SENSOR_FEATURE_SET_ESHUTTER,gc08a8wide_set_shutter},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME,gc08a8_set_multi_shutter_frame_length},
	{SENSOR_FEATURE_SET_GAIN,gc08a8wide_set_gain},
 	{SENSOR_FEATURE_TRAN_GET_SENSOR_CHIP_ID, gc08a8wide_feature_get_chip_id},
};

static unsigned char chip_id[32];
#define CHIP_ID_SIZE 12
static unsigned char gc08a8wide_otp_read_byte(struct subdrv_ctx *ctx,u16 addr)
{
  subdrv_i2c_wr_u8(ctx, 0x0a69, (addr >> 8) & 0x1f);
  subdrv_i2c_wr_u8(ctx, 0x0a6a, addr & 0xff);
  subdrv_i2c_wr_u8(ctx, 0x0313, 0x20);
  return subdrv_i2c_rd_u8(ctx,0x0a6c);
}

unsigned char *gc08a8wide_get_chip_id(struct subdrv_ctx *ctx,unsigned char *chip_id)
{
  int i = 0,flag;
  unsigned char chipIdData[32];

  //sensor_init();
  subdrv_i2c_wr_u8(ctx, 0x031c, 0x60);
  subdrv_i2c_wr_u8(ctx, 0x0315, 0x80);
  subdrv_i2c_wr_u8(ctx, 0x0324, 0x42);
  //OTP mode select register control
  subdrv_i2c_wr_u8(ctx, 0x0316, 0x09);
  subdrv_i2c_wr_u8(ctx, 0x0a67, 0x80);
  subdrv_i2c_wr_u8(ctx, 0x0313, 0x00);
  subdrv_i2c_wr_u8(ctx, 0x0a53, 0x0e);
  subdrv_i2c_wr_u8(ctx, 0x0a65, 0x17);
  subdrv_i2c_wr_u8(ctx, 0x0a68, 0xa1);
  subdrv_i2c_wr_u8(ctx, 0x0a47, 0x00);
  subdrv_i2c_wr_u8(ctx, 0x0a58, 0x00);
  subdrv_i2c_wr_u8(ctx, 0x0ace, 0x0c);
  mdelay(10);

  flag = gc08a8wide_read_otp_single(ctx,0x0018);

  pr_info("gc08a8wide_get_chip_id flag = %d",flag);
  for (i = 0; i < CHIP_ID_SIZE; i++) {
    chipIdData[i] = gc08a8wide_otp_read_byte(ctx,(0x20 + 8 * i));//OTP data read
   // pr_info("chipIdData[%d], %02X", i, chipIdData[i]);
    sprintf(chip_id+2*i, "%02X",chipIdData[i]);
  }

  pr_info("chip_id is %s", chip_id);

  return chip_id;
}

static int gc08a8wide_feature_get_chip_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1632,
			.vsize = 1224,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 1632,
			.vsize = 1224,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 3264,
			.vsize = 2448,
		},
	},

};


static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

#define WINSIZE_INFO_PRE  { 3264, 2448,	 0,    0, 3264, 2448, 1632,  1224,    0,    0, 1632,  1224,    0,    0, 1632,  1224} /* Preview */
#define WINSIZE_INFO_CAP  { 3264, 2448,	 0,    0, 3264, 2448, 3264,  2448,    0,    0, 3264,  2448,    0,    0, 3264,  2448} /* capture */
#define WINSIZE_INFO_VID  { 3264, 2448,	 0,    0, 3264, 2448, 3264,  2448,    0,    0, 3264,  2448,    0,    0, 3264,  2448} /* video */
#define WINSIZE_INFO_HS   { 3264, 2448,	 0,    0, 3264, 2448, 1632,  1224,    0,    0, 1632,  1224,    0,    0, 1632,  1224} /* hs video */
#define WINSIZE_INFO_SLIM { 3264, 2448,	 0,    0, 3264, 2448, 1632,  1224,    0,    0, 1632,  1224,    0,    0, 1632,  1224} /* slim video */
#define WINSIZE_INFO_CUS1 { 3264, 2448,	 0,    0, 3264, 2448, 3264,  2448,    0,    0, 3264,  2448,    0,    0, 3264,  2448} //custom1@30fps
#define WINSIZE_INFO_CUS2 { 3264, 2448,	 0,    0, 3264, 2448, 3264,  2448,    0,    0, 3264,  2448,    0,    0, 3264,  2448}  //custom2@60fps
#define WINSIZE_INFO_CUS3 { 3264, 2448,	 0,    0, 3264, 2448, 3264,  2448,    0,    0, 3264,  2448,    0,    0, 3264,  2448}  //custom3@10fps
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
		.pclk = 280000000,
		.linelength = 3640,
		.framelength = 2548,
		.max_framerate = 300,
		.mipi_pixel_rate = 271200000,
		.readout_length = 0,
		.read_margin = 8,
		.imgsensor_winsize_info = {
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
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
		.mode_setting_table = addr_data_pair_capture,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_capture),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
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
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
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
		.mode_setting_table = addr_data_pair_normal_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_normal_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
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
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
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
		.mode_setting_table = addr_data_pair_hs_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_hs_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
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
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
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
		.mode_setting_table = addr_data_pair_slim_video,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_slim_video),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
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
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
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
		.mode_setting_table = addr_data_pair_custom1,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom1),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
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
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
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
		.mode_setting_table = addr_data_pair_custom2,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom2),
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
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
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
		.mode_setting_table = addr_data_pair_custom3,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom3),
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
			.full_w = 3264,
			.full_h = 2448,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
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
	.sensor_id = GC08A8WIDE_SENSOR_ID,
	.reg_addr_sensor_id = {0x03f0, 0x03f1},
	.i2c_addr_table = {0x22, 0xFF},
	.i2c_burst_write_support = FALSE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = eeprom_info,
	.eeprom_num = ARRAY_SIZE(eeprom_info),
	.resolution = {3264, 2448},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 1,
	.ana_gain_step = 1,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 50,
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = 0xfffe,
	.exposure_step = 1,
	.exposure_margin = 16,
	.saturation_info = &imgsensor_saturation_info,

	.frame_length_max = 0xfffe,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 3000000,

	.pdaf_type = PDAF_SUPPORT_NA,
	//.hdr_type = HDR_SUPPORT_STAGGER_FDOL|HDR_SUPPORT_DCG|HDR_SUPPORT_LBMF,
	.hdr_type = 0,
	.seamless_switch_support = FALSE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = PARAM_UNDEFINED,

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

	.checksum_value = 0x5d51ce33,
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
				if (gc08a8wide_otp.awb_group_flag == 0 || gc08a8wide_otp.af_group_flag ==0 || gc08a8wide_otp.lsc_group_flag ==0)
				{
					DRV_LOG(ctx,"gc08a8wide_read_sensor_otp \n");
					gc08a8wide_read_sensor_otp(ctx);
				}
				gc08a8wide_get_chip_id(ctx, chip_id);
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

const struct subdrv_entry gc08a8wide_mipi_raw_entry = {
	.name = "gc08a8wide_mipi_raw",
	.id = GC08A8WIDE_SENSOR_ID,
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
static void gc08a8_set_multi_shutter(struct subdrv_ctx *ctx,
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

static int gc08a8_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	gc08a8_set_multi_shutter(ctx, (UINT32 *)(*feature_data), (UINT16) (*(feature_data + 1)), (UINT16) (*(feature_data + 2)));

	return 0;
}
static void gc08a8wide_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en)
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

	subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xfffe);

}

static int gc08a8wide_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

	DRV_LOG(ctx,"ctx->autoflicker_en =%d\n",ctx->autoflicker_en);
	if (ctx->autoflicker_en) {
		realtime_fps
			= ctx->pclk
			/ ctx->line_length * 10
			/ ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			gc08a8wide_set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			gc08a8wide_set_max_framerate(ctx, 146, 0);
		else
			subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length & 0xfffe);
	}

	if (shutter >= 0x3C18 ) { //200ms
		DRV_LOG(ctx,"enter long shutter\n");
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
			DRV_LOG(ctx,"exit long shutter\n");
			subdrv_i2c_wr_u8(ctx,0x022d, 0x20);
			subdrv_i2c_wr_u8(ctx,0x022e, 0x00);
			subdrv_i2c_wr_u8(ctx,0x022f, 0x00);
			bNeedSetNormalMode = false;
		}
		subdrv_i2c_wr_u16(ctx, 0x0202, shutter);
		subdrv_i2c_wr_u16(ctx, 0x0340, ctx->frame_length);
	}

	ctx->frame_length_rg = ctx->frame_length;
	DRV_LOG(ctx,"set_shutter shutter =%d, framelength =%d\n", shutter, ctx->frame_length);
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
static u32 gc08a8wide_gain2reg(struct subdrv_ctx *ctx, const u32 gain)
{
	u32 reg_gain = gain;

	reg_gain = (reg_gain < BASEGAIN) ? BASEGAIN : reg_gain;
	reg_gain = (reg_gain > BASEGAIN * 16) ? BASEGAIN * 16 : reg_gain;

	return reg_gain;
}

static int gc08a8wide_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 gain = *((u32 *) para);
	u32 reg_gain = 0;

	reg_gain = gc08a8wide_gain2reg(ctx, gain);
	ctx->gain = reg_gain;

	DRV_LOG(ctx,"gain = %d, reg_gain = 0x%x \n ", gain, reg_gain);
	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));

	return gain;
} /* set_gain*/
static int get_sensor_temperature(void *arg)
{
	return -127000;
}

static u16 get_gain2reg(u32 gain)
{
	return gain << 4;
}

static int gc08a8wide_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	pr_info("gc08a8wide_set_test_pattern set_test_pattern_mode mode %d -> %d\n", ctx->test_pattern, mode);
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

static int gc08a8wide_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

#define AWBINFO_FLAG_ADDR           0x1698
#define AWBINFO_FLAG_ADDR_4000K     0x8D60
#define AWBINFO_FLAG_ADDR_2800K     0x8E38
#define AFINFO_FLAG_ADDR            0x1790
#define LSCINFO_FLAG_ADDR           0x1888

#define AWB_GROUP1_START_ADDR       0x16A0
#define AWB_GROUP2_START_ADDR       0x1718
#define AWB_GROUP1_GOLDEN_DATA_ADDR 0x16D0
#define AWB_GROUP2_GOLDEN_DATA_ADDR 0x1748

//4000K
#define AWB_GROUP1_START_ADDR_4000K 0x8D68
#define AWB_GROUP2_START_ADDR_4000K 0x8DD0
#define AWB_GROUP1_GOLDEN_DATA_ADDR_4000K  0x8D98
#define AWB_GROUP2_GOLDEN_DATA_ADDR_4000K  0x8E00
//2800K
#define AWB_GROUP1_START_ADDR_2800K 0x8E40
#define AWB_GROUP2_START_ADDR_2800K 0x8EA8
#define AWB_GROUP1_GOLDEN_DATA_ADDR_2800K  0x8E70
#define AWB_GROUP2_GOLDEN_DATA_ADDR_2800K  0x8ED8

#define AF_GROUP1_START_ADDR        0x1798
#define AF_GROUP2_START_ADDR        0x1810
#define LSC_GROUP1_START_ADDR       0x1890
#define LSC_GROUP1_CHECKSUM_ADDR    0x52F0
#define LSC_GROUP2_START_ADDR       0x52F8
#define LSC_GROUP2_CHECKSUM_ADDR    0x8D58

#define AWB_GROUP1_CHECKSUM_ADDR    0x1710
#define AWB_GROUP2_CHECKSUM_ADDR    0x1788
//4000K
#define AWB_GROUP1_CHECKSUM_ADDR_4000K    0x8DC8
#define AWB_GROUP2_CHECKSUM_ADDR_4000K    0x8E30
//2800K
#define AWB_GROUP1_CHECKSUM_ADDR_2800K    0x8EA0
#define AWB_GROUP2_CHECKSUM_ADDR_2800K    0x8F08
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

static kal_uint8 gc08a8wide_read_otp_single(struct subdrv_ctx *ctx,kal_uint16 addr)
{
	kal_uint8 data;
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x00);
	subdrv_i2c_wr_u8(ctx ,0x0a69, (addr >> 8) & 0xff);
	subdrv_i2c_wr_u8(ctx ,0x0a6a, addr & 0xff);
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x20);
	subdrv_i2c_wr_u8(ctx ,0x0313, 0x12);
	mdelay(5);

	data = (kal_uint8)subdrv_i2c_rd_u8(ctx,0x0a6c);
	DRV_LOG(ctx,"gc08a8wide_read_otp_single single data = %d", data);

	return data;
}

static void gc08a8wide_read_otp_continuous(struct subdrv_ctx *ctx,kal_uint16 addr, kal_uint16 length, kal_uint8 *data)
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
		DRV_LOG(ctx,"gc08a8wide_read_otp_continuous continuous data[%d] = %d", i, data[i]);
	}
}

static void gc08a8wide_read_sensor_otp(struct subdrv_ctx *ctx)
{
	kal_uint8 awb_flag, af_flag, lsc_flag;

	otp_init_setting(ctx);
	//awb data
	awb_flag = gc08a8wide_read_otp_single(ctx,AWBINFO_FLAG_ADDR);
	DRV_LOG(ctx,"gc08a8wide sensor otp awb_flag = %d", awb_flag);
    	gc08a8wide_otp.awb_flag = awb_flag;
	if(awb_flag == 0x01)
	{
		gc08a8wide_otp.awb_group_flag = 1;
		gc08a8wide_read_otp_continuous(ctx,AWB_GROUP1_START_ADDR, 14, gc08a8wide_otp.awb_data);
		gc08a8wide_otp.awb_checksum = gc08a8wide_read_otp_single(ctx,AWB_GROUP1_CHECKSUM_ADDR);
	}
	else if(awb_flag == 0x1F)
	{
		gc08a8wide_otp.awb_group_flag = 2;
		gc08a8wide_read_otp_continuous(ctx,AWB_GROUP2_START_ADDR, 14, gc08a8wide_otp.awb_data);
		gc08a8wide_otp.awb_checksum = gc08a8wide_read_otp_single(ctx,AWB_GROUP2_CHECKSUM_ADDR);
	}
	else
		DRV_LOG(ctx,"gc08a8wide sensor otp awb_flag invalid");

    //AWB 4000k start
	awb_flag = gc08a8wide_read_otp_single(ctx,AWBINFO_FLAG_ADDR_4000K);
	DRV_LOG(ctx,"gc08a8wide sensor otp 4000K awb_flag = %d", awb_flag);
	if(awb_flag == 0x01)
	{
		DRV_LOG(ctx,"gc08a8wide sensor otp 4000K awb_flag awb_flag == 0x01");
		gc08a8wide_otp.awb_group_flag_4000k = 1;
		gc08a8wide_read_otp_continuous(ctx,AWB_GROUP1_START_ADDR_4000K, 13, gc08a8wide_otp.awb_data_4000k);
		gc08a8wide_otp.awb_checksum_4000k = gc08a8wide_read_otp_single(ctx,AWB_GROUP1_CHECKSUM_ADDR_4000K);
		mdelay(2);
		gc08a8wide_otp.awb_flag_4000k = awb_flag;
	}
	else if(awb_flag == 0x1F)
	{
		DRV_LOG(ctx,"gc08a8wide sensor otp 4000K awb_flag awb_flag == 0x1F");
		gc08a8wide_otp.awb_group_flag_4000k = 2;
		gc08a8wide_read_otp_continuous(ctx,AWB_GROUP2_START_ADDR_4000K, 13, gc08a8wide_otp.awb_data_4000k);
		gc08a8wide_otp.awb_checksum_4000k = gc08a8wide_read_otp_single(ctx,AWB_GROUP2_CHECKSUM_ADDR_4000K);
		mdelay(2);
		gc08a8wide_otp.awb_flag_4000k = awb_flag;
	}
	else
		DRV_LOG(ctx,"gc08a8wide sensor otp 4000K awb_flag invalid");
	//AWB 4000k end
	//AWB 2800k start
	awb_flag = gc08a8wide_read_otp_single(ctx,AWBINFO_FLAG_ADDR_2800K);
	DRV_LOG(ctx,"gc08a8wide sensor otp 2800K awb_flag = %d", awb_flag);
	if(awb_flag == 0x01)
	{
		DRV_LOG(ctx,"gc08a8wide sensor otp 2800k awb_flag awb_flag == 0x01");
		gc08a8wide_otp.awb_group_flag_2800k = 1;
		gc08a8wide_read_otp_continuous(ctx,AWB_GROUP1_START_ADDR_2800K, 13, gc08a8wide_otp.awb_data_2800k);
		gc08a8wide_otp.awb_checksum_2800k = gc08a8wide_read_otp_single(ctx,AWB_GROUP1_CHECKSUM_ADDR_2800K);
		mdelay(2);
		gc08a8wide_otp.awb_flag_2800k = awb_flag;
	}
	else if(awb_flag == 0x1F)
	{
		DRV_LOG(ctx,"gc08a8wide sensor otp 2800k awb_flag awb_flag == 0x1F");
		gc08a8wide_otp.awb_group_flag_2800k = 2;
		gc08a8wide_read_otp_continuous(ctx,AWB_GROUP2_START_ADDR_2800K, 13, gc08a8wide_otp.awb_data_2800k);
		gc08a8wide_otp.awb_checksum_2800k = gc08a8wide_read_otp_single(ctx,AWB_GROUP2_CHECKSUM_ADDR_2800K);
		mdelay(2);
		gc08a8wide_otp.awb_flag_2800k = awb_flag;
	}
	else
		DRV_LOG(ctx,"gc08a8wide sensor otp 2800k awb_flag invalid");
	//AWB 2800k end

	//af data
	af_flag = gc08a8wide_read_otp_single(ctx,AFINFO_FLAG_ADDR);
	DRV_LOG(ctx,"gc08a8 sensor otp af_flag = %d", af_flag);
    	gc08a8wide_otp.af_flag = af_flag;
	if(af_flag == 0x01)
	{
		gc08a8wide_otp.af_group_flag = 1;
		gc08a8wide_read_otp_continuous(ctx,AF_GROUP1_START_ADDR, 14, gc08a8wide_otp.af_data);
		gc08a8wide_otp.af_checksum = gc08a8wide_read_otp_single(ctx,AF_GROUP1_CHECKSUM_ADDR);
	}
	else if(af_flag == 0x1F)
	{
		gc08a8wide_otp.af_group_flag = 2;
		gc08a8wide_read_otp_continuous(ctx,AF_GROUP2_START_ADDR, 14, gc08a8wide_otp.af_data);
		gc08a8wide_otp.af_checksum = gc08a8wide_read_otp_single(ctx,AF_GROUP2_CHECKSUM_ADDR);
	}
	else
		DRV_LOG(ctx,"gc08a8wide sensor otp af_flag invalid");

	//lsc data
	lsc_flag = gc08a8wide_read_otp_single(ctx,LSCINFO_FLAG_ADDR);
	DRV_LOG(ctx,"gc08a8 sensor otp lsc_flag = %d", lsc_flag);
	if(lsc_flag == 0x01)
	{
		gc08a8wide_otp.lsc_group_flag = 1;
		gc08a8wide_read_otp_continuous(ctx,LSC_GROUP1_START_ADDR, 1868, gc08a8wide_otp.lsc_data);
		gc08a8wide_otp.lsc_checksum = gc08a8wide_read_otp_single(ctx,LSC_GROUP1_CHECKSUM_ADDR);
	}
	else if(lsc_flag == 0x1F)
	{
		gc08a8wide_otp.lsc_group_flag = 2;
		gc08a8wide_read_otp_continuous(ctx,LSC_GROUP2_START_ADDR, 1868, gc08a8wide_otp.lsc_data);
		gc08a8wide_otp.lsc_checksum = gc08a8wide_read_otp_single(ctx,LSC_GROUP2_CHECKSUM_ADDR);
	}
	else
		DRV_LOG(ctx,"gc08a8wide sensor otp lsc_flag invalid");

	//otp_end_read
	subdrv_i2c_wr_u8(ctx,0x0316, 0x01);
	subdrv_i2c_wr_u8(ctx,0x0a67, 0x00);
}



unsigned int gc08a8wide_read_region(struct i2c_client *client, unsigned int addr,
			unsigned char *data, unsigned int size)
{
	pr_info("gc08a8wide_read_region addr = 0x%x, size = %d", addr, size);

	if(addr == 0 && size == 0x1500) //Avoiding preloading, hal print error log "Preload data failed"
		return 0;
	else if (addr == AWBINFO_FLAG_ADDR) 
	{
		pr_info("gc08a8wide_read_region awb flag:%d", gc08a8wide_otp.awb_flag);
		*data = gc08a8wide_otp.awb_flag;
		return size;
	}
	else if((addr == AWB_GROUP1_START_ADDR || addr == AWB_GROUP2_START_ADDR) && size == 6) //awb data
	{
		pr_info("gc08a8wide_read_region awb data");
		memcpy(data, gc08a8wide_otp.awb_data, size * sizeof(kal_uint8));
		return size;
	}
	else if((addr == AWB_GROUP1_GOLDEN_DATA_ADDR || addr == AWB_GROUP2_GOLDEN_DATA_ADDR) && size == 6) //awb golden data
	{
		pr_info("gc08a8wide_read_region awb golden data");
		memcpy(data, &gc08a8wide_otp.awb_data[6], size * sizeof(kal_uint8));
		return size;
	}
	else if((addr == AWB_GROUP1_START_ADDR || addr == AWB_GROUP2_START_ADDR) && size == 14) //all awb data
	{
		pr_info("gc08a8wide_read_region all awb data");
		memcpy(data, gc08a8wide_otp.awb_data, size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == AWB_GROUP1_CHECKSUM_ADDR || addr == AWB_GROUP2_CHECKSUM_ADDR)  //awb checksum
	{
		*data = gc08a8wide_otp.awb_checksum;
		return size;
	}
	//4000K	AWB start
	else if( (addr == AWB_GROUP1_START_ADDR_4000K || addr == AWB_GROUP2_START_ADDR_4000K) && size == 6) //awb data
	{
		pr_info("gc08a8wide_read_region 4000K awb data");
		memcpy(data, &gc08a8wide_otp.awb_data_4000k, size * sizeof(kal_uint8));
		return size;
	}
	else if((addr == AWB_GROUP1_GOLDEN_DATA_ADDR_4000K || addr == AWB_GROUP2_GOLDEN_DATA_ADDR_4000K)  && size == 6) //awb golden data
	{
		pr_info("gc08a8wide_read_region 4000K awb golden data");
		memcpy(data, &gc08a8wide_otp.awb_data_4000k[6], size * sizeof(kal_uint8));
		return size;
	}
	else if((addr == AWB_GROUP1_START_ADDR_4000K || addr == AWB_GROUP2_START_ADDR_4000K)  && size == 13) //all awb data
	{
		pr_info("gc08a8wide_read_region 4000K all awb data");
		memcpy(data, &gc08a8wide_otp.awb_data_4000k, size * sizeof(kal_uint8));
		return size;
	}
	else if( (addr == AWB_GROUP1_CHECKSUM_ADDR_4000K) || (addr == AWB_GROUP2_CHECKSUM_ADDR_4000K) )  //awb checksum
	{
		pr_info("gc08a8wide_read_region all awb data 4000k ckecksum =%d",gc08a8wide_otp.awb_checksum_4000k);
		*data = gc08a8wide_otp.awb_checksum_4000k;
		return size;
	}
	else if(addr == AWBINFO_FLAG_ADDR_4000K)  //awb checkflag
	{
		pr_info("gc08a8wide_read_region 4000K awb_flag_4000k is =0x%x",gc08a8wide_otp.awb_flag_4000k);
		*data = gc08a8wide_otp.awb_flag_4000k;
		return size;
	}
	//4000K	AWB end
	//2800K	AWB start
	else if((addr == AWB_GROUP1_START_ADDR_2800K || addr == AWB_GROUP2_START_ADDR_2800K)  && size == 6) //awb data
	{
		pr_info("gc08a8wide_read_region 2800K awb data");
		memcpy(data, &gc08a8wide_otp.awb_data_2800k, size * sizeof(kal_uint8));
		return size;
	}
	else if( (addr == AWB_GROUP1_GOLDEN_DATA_ADDR_2800K || addr == AWB_GROUP2_GOLDEN_DATA_ADDR_2800K) && size == 6) //awb golden data
	{
		pr_info("gc08a8wide_read_region 2800K awb golden data");
		memcpy(data, &gc08a8wide_otp.awb_data_2800k[6], size * sizeof(kal_uint8));
		return size;
	}
	else if((addr == AWB_GROUP1_START_ADDR_2800K  || addr == AWB_GROUP2_START_ADDR_2800K)&& size == 13) //all awb data
	{
		pr_info("gc08a8wide_read_region all 2800K awb data");
		memcpy(data, &gc08a8wide_otp.awb_data_2800k, size * sizeof(kal_uint8));
		return size;
	}
	else if( (addr == AWB_GROUP1_CHECKSUM_ADDR_2800K)||(addr == AWB_GROUP2_CHECKSUM_ADDR_2800K) ) //awb checksum
	{
		*data = gc08a8wide_otp.awb_checksum_2800k;
		return size;
	}
	else if(addr == AWBINFO_FLAG_ADDR_2800K)  //awb checkflag
	{
		pr_info("gc08a8wide_read_region 2800K awb_flag_2800k is =0x%x",gc08a8wide_otp.awb_flag_4000k);
		*data = gc08a8wide_otp.awb_flag_2800k;
		return size;
	}
	//2800K	AWB end
	else if (addr == AFINFO_FLAG_ADDR) 
	{
		pr_info("gc08a8wide_read_region af flag:%d", gc08a8wide_otp.af_flag);
		*data = gc08a8wide_otp.af_flag;
		return size;
	}
	else if((addr == AF_GROUP1_START_ADDR || addr == AF_GROUP2_START_ADDR) && size == 6)  //af data
	{
		memcpy(data, &gc08a8wide_otp.af_data[6], size * sizeof(kal_uint8));
		return size;
	}
	else if((addr == AF_GROUP1_START_ADDR || addr == AF_GROUP2_START_ADDR) && size == 14) //all af data
	{
		memcpy(data, gc08a8wide_otp.af_data, size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == AF_GROUP1_CHECKSUM_ADDR || addr == AF_GROUP2_CHECKSUM_ADDR)  //af checksum
	{
		*data = gc08a8wide_otp.af_checksum;
		return size;
	}
	else if(addr == (LSC_GROUP1_START_ADDR-1)) //lsc flag
	{
		*data = gc08a8wide_otp.lsc_group_flag ? 1 : 0;
		return size;
	}
	else if(addr == LSC_GROUP1_START_ADDR && size == 1868) //lsc data
	{
		memcpy(data, gc08a8wide_otp.lsc_data, size * sizeof(kal_uint8));
		return size;
	}
	else if(addr == LSC_GROUP1_CHECKSUM_ADDR) //lsc checksum
	{
		pr_info("gc08a8wide_read_region lsc checksum");
		*data = gc08a8wide_otp.lsc_checksum;
		return size;
	}
	return 0;
}EXPORT_SYMBOL(gc08a8wide_read_region);



static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;

	return 0;
}
static int gc08a8wide_sensor_init(struct subdrv_ctx *ctx)
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
	gc08a8wide_sensor_init(ctx);

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
	ctx->frame_length_rg = ctx->frame_length;
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
