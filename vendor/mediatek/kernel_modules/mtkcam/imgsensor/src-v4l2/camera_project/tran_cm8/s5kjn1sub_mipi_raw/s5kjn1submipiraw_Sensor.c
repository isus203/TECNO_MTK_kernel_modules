// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 s5kjn1submipiraw_Sensor.c
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
#include "s5kjn1submipiraw_Sensor.h"

#define SHUTTER_1S 10
#define SHUTTER_1_5S 15
#define SHUTTER_2S 20
#define SHUTTER_4S 41
#define SHUTTER_6S 62
#define SHUTTER_10S 1046250
#define SHUTTER_20S 2092491
#define SHUTTER_30S 3138731

static u16 get_gain2reg(u32 gain);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int s5kjn1sub_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int tran_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);

static int s5kjn1sub_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1sub_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1sub_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1sub_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1sub_read_4cell_from_eeprom(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1sub_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1sub_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int s5kjn1sub_streaming_control_off(struct subdrv_ctx *ctx, u8 *para, u32 *len);

static int get_sensor_temperature(void *arg);
/* STRUCT */

static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, s5kjn1sub_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, s5kjn1sub_set_test_pattern_data},
	{SENSOR_FEATURE_SET_ESHUTTER,s5kjn1sub_set_shutter},
	{SENSOR_FEATURE_SET_GAIN,s5kjn1sub_set_gain},
	{SENSOR_FEATURE_GET_4CELL_DATA,s5kjn1sub_read_4cell_from_eeprom},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME,s5kjn1sub_set_multi_shutter_frame_length},
	{SENSOR_FEATURE_SET_STREAMING_RESUME,s5kjn1sub_streaming_control_on},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND,s5kjn1sub_streaming_control_off},
};

//stream on
static void s5kjn1sub_streaming_on(struct subdrv_ctx *ctx, bool enable)
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
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
		mdelay(10);
	}else {
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);

		memset(ctx->exposure, 0, sizeof(ctx->exposure));
		memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
		ctx->autoflicker_en = FALSE;
		ctx->extend_frame_length_en = 0;
		ctx->is_seamless = 0;
		ctx->stream_ctrl_start_time = 0;
		ctx->stream_ctrl_end_time = 0;
	}
	ctx->sof_no = 0;
	ctx->is_streaming = enable;
	DRV_LOG(ctx,"X! enable:%u\n", enable);
}

static int s5kjn1sub_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;

	if (*feature_data) {
		set_shutter(ctx, *feature_data);
	}
	s5kjn1sub_streaming_on(ctx, TRUE);

	return 0;
}

static int s5kjn1sub_streaming_control_off(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	s5kjn1sub_streaming_on(ctx, FALSE);

	return 0;
}

static void s5kjn1sub_set_multi_shutter(struct subdrv_ctx *ctx,
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

static int s5kjn1sub_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	s5kjn1sub_set_multi_shutter(ctx, (UINT32 *)(*feature_data), (UINT16) (*(feature_data + 1)), (UINT16) (*(feature_data + 2)));

	return 0;
}

#define EEPROM_READ_ID  0xA2
#define EEPROM_WRITE_ID   0xA3

#define FOUR_CELL_SIZE 9490
//XTC
#define XTC_FLAG 0x1A5A
#define XTC_ADDR 0x1A5C
#define XTC_LENGTH 3502
//if XTC_Size is greater than XTC_LENGTH, it must be offset
#define XTC_OFFSET 594
//Sensor XTC
#define SENSOR_XTC_ADDR 0x280A
#define SENSOR_XTC_LENGTH 768

//PD XTC
#define PD_XTC_ADDR 0x2B0A
#define PD_XTC_LENGTH 4000

//SW GGC
#define SW_GGC_ADDR 0x3AAA
#define SW_GGC_LENGTH 626

#define HW_GGC_ADDR   0x3D1C
#define HW_GGC_LENGTH 346
#define SW_all_CHKSUM 0x3E76

static int Is_Read_4Cell;
static char Four_Cell_Array[FOUR_CELL_SIZE + 2];
static char Four_Cell_Array_HwGcc[HW_GGC_LENGTH];
static int s5kjn1sub_read_data_from_eeprom(struct subdrv_ctx *ctx)
{
	int ret;
	int temp;
	char flag = 0;
	int i = 0;

	if (Is_Read_4Cell != 1) {
		pr_info("Need to read xtc by I2C\n");

		/* Flag of XTC */
		ret = adaptor_i2c_rd_u8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, XTC_FLAG, &flag);
		if (ret != 0) {
			pr_err("Read Flag of XTC err, flag[%d]\n", flag);
			return 0;
		}

		Four_Cell_Array[0] = (FOUR_CELL_SIZE & 0xff);/*Low*/
		Four_Cell_Array[1] = ((FOUR_CELL_SIZE >> 8) & 0xff);/*High*/

		/* Multi-Read XTC Data */
		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, XTC_ADDR,
			&Four_Cell_Array[2], XTC_LENGTH);
		temp = 0;
		for(i=0; i < XTC_LENGTH; i++)
		{
			temp= Four_Cell_Array[i+2] + temp;
			//pr_info("XTC Data[%d] = %d\n",i,Four_Cell_Array[i+2]);
		}

		/* Offset of XTC */
		for(i=0; i < XTC_OFFSET; i++) {
			Four_Cell_Array[i+2+XTC_LENGTH] = 0;
		}

		/* Multi-Read Sensor XTC Data */
		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, SENSOR_XTC_ADDR,
			&Four_Cell_Array[2+XTC_LENGTH+XTC_OFFSET], SENSOR_XTC_LENGTH);

		for(i=0; i < SENSOR_XTC_LENGTH; i++)
		{
			temp= Four_Cell_Array[i+2+XTC_LENGTH+XTC_OFFSET] + temp;
			//pr_info("Sensor XTC Data[%d] = %d\n",i+4096,Four_Cell_Array[i+4098]);
		}

		/* Multi-Read Sensor PDXTC Data */
		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, PD_XTC_ADDR,
			&Four_Cell_Array[2+XTC_LENGTH+XTC_OFFSET+SENSOR_XTC_LENGTH], PD_XTC_LENGTH);

		for(i=0; i < PD_XTC_LENGTH; i++)
		{
			temp= Four_Cell_Array[i+2+XTC_LENGTH+XTC_OFFSET+SENSOR_XTC_LENGTH] + temp;
			//pr_info("PDXTC Data[%d] = %d\n",i+4864,Four_Cell_Array[i+4866]);
		}

		/* Multi-Read Sensor SWGGC Data */
		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, SW_GGC_ADDR,
			&Four_Cell_Array[2+XTC_LENGTH+XTC_OFFSET+SENSOR_XTC_LENGTH+PD_XTC_LENGTH], SW_GGC_LENGTH);

		for(i=0; i < SW_GGC_LENGTH; i++)
		{
			temp= Four_Cell_Array[i+2+XTC_LENGTH+XTC_OFFSET+SENSOR_XTC_LENGTH+PD_XTC_LENGTH] + temp;
			//pr_info("SWGGC Data[%d] = %d\n",i+8864,Four_Cell_Array[i+8866]);
		}
		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, HW_GGC_ADDR,
			&Four_Cell_Array_HwGcc[0], HW_GGC_LENGTH);

		for(i=0; i < HW_GGC_LENGTH; i++)
		{
			temp= Four_Cell_Array_HwGcc[i+0] + temp;
			//pr_info("s5kjn1sub Four_Cell_Array_HwGcc Data[%d] = %d\n",i,Four_Cell_Array_HwGcc[i+0]);
		}
		/* checksum of remosaic all xtc */
		ret = adaptor_i2c_rd_u8(ctx->i2c_client, EEPROM_READ_ID >> 1, SW_all_CHKSUM, &flag);
		if (flag != ((temp + 1)%256)) {
			pr_err("SWGGC chksum err, [0x%x] != [0x%x]\n", flag, (temp%256));
			return 0;
		}else{
			pr_info("s5kjn1sub remosaic_xtc checksum success");
		}
		Is_Read_4Cell = 1;
	}

	return 0;
}
static int s5kjn1sub_read_4cell_from_eeprom(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;
	int type = (u32)(*feature_data);
	char *data = (char *)(uintptr_t)(*(feature_data+1));

	if (Is_Read_4Cell != 1) {
		s5kjn1sub_read_data_from_eeprom(ctx);
	}

	if (data != NULL && type == FOUR_CELL_CAL_TYPE_GAIN_TBL) {
		pr_info("return xtc data type:%d\n", type);
		memcpy(data, Four_Cell_Array, FOUR_CELL_SIZE);
	}else{
		pr_info("xtc data type -\n");
	}
	return 0;
}

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =2,
	.i4PosL = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.i4PosR = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4BlockNumX = 504,
	.i4BlockNumY = 382,
	.i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0}, {120, 456}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	.iMirrorFlip = 0,
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.i4FullRawW = 4080,
	.i4FullRawH = 3072,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning_video_16_9 = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX  = 8,
	.i4PitchY  = 8,
	.i4PairNum  =4,
	.i4SubBlkW  =8,
	.i4SubBlkH  =2,
	.i4PosL = {{9, 8},{11, 11},{15, 12},{13, 15}},
	.i4PosR = {{8, 8},{10, 11},{14, 12},{12, 15}},
	.i4BlockNumX = 504,
	.i4BlockNumY = 286,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0}, {120, 456}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.i4FullRawW = 4080,
	.i4FullRawH = 2304,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0}, // R = 1, L = 0
	},
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
	.i4Crop = { {0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0}, {0, 0}, {120, 456}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	.iMirrorFlip = 0,
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.i4FullRawW = 3840,
	.i4FullRawH = 2160,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0}, // R = 1, L = 0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom4 = {// 1/4 binning 
	.i4OffsetX = 4,
	.i4OffsetY = 4,
	.i4PitchX  = 4,
	.i4PitchY  = 4,
	.i4PairNum  =1,
	.i4SubBlkW  =4,
	.i4SubBlkH  =4,
	.i4PosL = {{5, 4}},
	.i4PosR = {{4, 4}},
	.i4BlockNumX = 504,
	.i4BlockNumY = 382,
	.iMirrorFlip = 0,
	.i4Crop = { {0, 0}, {0, 0}, {0,0}, {0, 0}, {0, 0}, {0, 0}, {0,0}, {0, 0}, {766, 386}, {0, 0}, {0, 0}, {0, 0} },
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.i4FullRawW = 2040,
	.i4FullRawH = 1536,
	.i4VCPackNum = 1,
	.i4ModeIndex = 0,
	.sPDMapInfo[0] = {
		.i4PDPattern = 3,
		.i4PDRepetition = 2,
		.i4PDOrder = {1,0}, // R = 1, L = 0
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
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
			.hsize = 504,
			.vsize = 3056,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
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
			.hsize = 504,
			.vsize = 3056,
			.user_data_desc = VC_PDAF_STATS,
		 },
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x0900,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		 .bus.csi2 = {
			 .channel = 1,
			 .data_type = 0x2b,
			 .hsize = 0x01F8,
			 .vsize = 0x08F0,
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
			.hsize = 0x0FF0,
			.vsize = 0x0C00,
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
			 .hsize = 0x01E0,
			 .vsize = 0x0860,
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
			.hsize = 2040,
			.vsize = 1536,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
	{
		 .bus.csi2 = {
			 .channel = 1,
			 .data_type = 0x2b,
			 .hsize = 512,
			 .vsize = 764,
			 .user_data_desc = VC_PDAF_STATS,
		 },
	},

};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x07F8,
			.vsize = 0x0600,
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
//	{
//		.bus.csi2 = {
//			.channel = 1,
//			.data_type = 0x2b,
//			.hsize = 504,
//			.vsize = 3056,
//			.user_data_desc = VC_PDAF_STATS,
//		 },
//	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cus7[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x0C00,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
//	{
//		.bus.csi2 = {
//			.channel = 1,
//			.data_type = 0x2b,
//			.hsize = 504,
//			.vsize = 3056,
//			.user_data_desc = VC_PDAF_STATS,
//		 },
//	},
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
		.pclk = 544000000,
		.linelength = 5200,
		.framelength = 3486,
		.max_framerate = 300,
		.mipi_pixel_rate = 547200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
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
		.pclk = 544000000,
		.linelength = 5200,
		.framelength = 3486,
		.max_framerate = 300,
		.mipi_pixel_rate = 547200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
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
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.max_framerate = 300,
		.mipi_pixel_rate = 556800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 768,
			.w0_size = 8160,
			.h0_size = 4608,
			.scale_w = 4080,
			.scale_h = 2304,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 2304,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 2304,
		},
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning_video_16_9,
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
		.pclk = 600000000,
		.linelength = 2848,
		.framelength = 1754,
		.max_framerate = 1200,
		.mipi_pixel_rate = 652800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 2040,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2040,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2040,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.ana_gain_max = BASEGAIN * 16,
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
		.pclk = 600000000,
		.linelength = 2064,
		.framelength = 1208,
		.max_framerate = 2400,
		.mipi_pixel_rate = 792000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 240,
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
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
		.multi_exposure_ana_gain_range[IMGSENSOR_EXPOSURE_LE].max = BASEGAIN * 16,
		.ana_gain_max = BASEGAIN * 16,
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
		.pclk = 544000000,
		.linelength = 5200,
		.framelength = 3486,
		.max_framerate = 300,
		.mipi_pixel_rate = 547200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
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
		.pclk = 560000000,
		.linelength = 4096,
		.framelength = 2274,
		.max_framerate = 600,
		.mipi_pixel_rate = 1120000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 240,
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
		.pclk = 560000000,
		.linelength = 8688,
		.framelength = 6400,
		.max_framerate = 100,
		.mipi_pixel_rate = 556800000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 8160,
			.scale_h = 6144,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 8160,
			.h1_size = 6144,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 8160,
			.h2_tg_size = 6144,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
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
		.pclk = 600000000,
                .linelength = 4848,
                .framelength = 4124,
                .max_framerate = 300,
                .mipi_pixel_rate = 320000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 2040,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2040,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2040,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = TRUE,
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
		.pclk = 600000000,
		.linelength = 4848,
		.framelength = 4124,
		.max_framerate = 300,
		.mipi_pixel_rate = 320000000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 2040,
			.scale_h = 1536,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2040,
			.h1_size = 1536,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2040,
			.h2_tg_size = 1536,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1,
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
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 544000000,
		.linelength = 5200,
		.framelength = 3486,
		.max_framerate = 300,
		.mipi_pixel_rate = 547200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
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
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 544000000,
		.linelength = 5200,
		.framelength = 3486,
		.max_framerate = 300,
		.mipi_pixel_rate = 547200000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
			.full_w = 8160,
			.full_h = 6144,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 8160,
			.h0_size = 6144,
			.scale_w = 4080,
			.scale_h = 3072,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 4080,
			.h1_size = 3072,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 4080,
			.h2_tg_size = 3072,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = S5KJN1SUB_SENSOR_ID,
	.reg_addr_sensor_id = {0x0000, 0x0001},
	.i2c_addr_table = {0xAC, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_16,
	.eeprom_info =  PARAM_UNDEFINED,
	.eeprom_num =  PARAM_UNDEFINED,
	.resolution = {8160, 6144},
	.mirror = IMAGE_HV_MIRROR,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_Gb,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
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
	.start_exposure_offset = 1650000,

	.pdaf_type = PDAF_SUPPORT_CAMSV,
	.hdr_type = 0,
	.seamless_switch_support = FALSE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = PARAM_UNDEFINED,

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

	.checksum_value = 0xef23676a,
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
				s5kjn1sub_read_data_from_eeprom(ctx);
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
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD,  1050000, 1},
	{HW_ID_AVDD,  2800000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_RST, 1, 2},
	{HW_ID_MCLK, 24, 9},
};

const struct subdrv_entry s5kjn1sub_mipi_raw_entry = {
	.name = "s5kjn1sub_mipi_raw",
	.id = S5KJN1SUB_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
static u16 get_gain2reg(u32 gain)
{
	return gain * 32 / BASEGAIN;
}

static int s5kjn1sub_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int s5kjn1sub_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
static void s5kjn1sub_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en)
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

static int s5kjn1sub_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 shutter = *((u32 *) para);
	u16 realtime_fps = 0;
	u32 temp_shutter = 0;
	u32 long_exp_flag = 0;
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
			s5kjn1sub_set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			s5kjn1sub_set_max_framerate(ctx, 146, 0);
	}

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = (u32) shutter;
	/* long expsoure */
	if (shutter > (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {
		DRV_LOG(ctx,"Long shutter In shutter = %d \n", shutter);
		temp_shutter = shutter / 10000;
		if(temp_shutter < 100){
			shutter = temp_shutter;
		}
		switch (shutter) {
			case SHUTTER_1S:
			DRV_LOG(ctx,"1S\n");
			subdrv_i2c_wr_u16(ctx, 0x0340, 0x0341);
			subdrv_i2c_wr_u16(ctx, 0x0202, 0x0331);
			long_exp_flag = 1;
			break;
		case SHUTTER_1_5S:
			DRV_LOG(ctx,"1.5S\n");
			subdrv_i2c_wr_u16(ctx, 0x0340, 0x04D9);
			subdrv_i2c_wr_u16(ctx, 0x0202, 0x04C9);
			long_exp_flag = 1;
			break;
		case SHUTTER_2S:
			DRV_LOG(ctx,"2S\n");
			subdrv_i2c_wr_u16(ctx, 0x0340, 0x0672);
			subdrv_i2c_wr_u16(ctx, 0x0202, 0x0662);
			long_exp_flag = 1;
			break;
		case SHUTTER_4S:
			DRV_LOG(ctx,"4S\n");
			subdrv_i2c_wr_u16(ctx, 0x0340, 0x0CD5);
			subdrv_i2c_wr_u16(ctx, 0x0202, 0x0CC5);
			long_exp_flag = 1;
			break;
		case SHUTTER_6S:
			DRV_LOG(ctx,"6S\n");
			subdrv_i2c_wr_u16(ctx, 0x0340, 0x1337);
			subdrv_i2c_wr_u16(ctx, 0x0202, 0x1327);
			long_exp_flag = 1;
			break;
		case SHUTTER_10S:
			DRV_LOG(ctx,"10S\n");
			subdrv_i2c_wr_u16(ctx, 0x0340, 0x1FFD);
			subdrv_i2c_wr_u16(ctx, 0x0202, 0x1FED);
			long_exp_flag = 1;
			break;
		case SHUTTER_20S:
			DRV_LOG(ctx,"20S\n");
			subdrv_i2c_wr_u16(ctx, 0x0340, 0x3FEA);
			subdrv_i2c_wr_u16(ctx, 0x0202, 0x3FDA);
			long_exp_flag = 1;
			break;
		case SHUTTER_30S:
			DRV_LOG(ctx,"30S\n");
			subdrv_i2c_wr_u16(ctx, 0x0340, 0x5FD7);
			subdrv_i2c_wr_u16(ctx, 0x0202, 0x5FC7);
			long_exp_flag = 1;
			break;
		 default:
			long_exp_flag = 0;
			break;
		}
		if(long_exp_flag){
			subdrv_i2c_wr_u16(ctx, 0x0702, 0x0700);
			subdrv_i2c_wr_u16(ctx, 0x0704, 0x0700);
		}else{
			if(shutter < 100){
				shutter = *((u32 *) para);
			}
			coarseIntegrationTime = (ctx->pclk / (ctx->line_length * 64) * (shutter / 10000)) / 10;
			frameLengthLine = coarseIntegrationTime + 3;
			DRV_LOG_MUST(ctx,"Long shutter In shutter = %d frameLength = 0x%x coarseTime = 0x%x\n", shutter, frameLengthLine, coarseIntegrationTime);
			subdrv_i2c_wr_u16(ctx, 0x0340,frameLengthLine);
			subdrv_i2c_wr_u16(ctx, 0x0202,coarseIntegrationTime);
			subdrv_i2c_wr_u16(ctx, 0x0702, 0x0600);
			subdrv_i2c_wr_u16(ctx, 0x0704, 0x0600);
		}
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
		long_exp_flag = 0;
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
static u32 s5kjn1sub_gain2reg(struct subdrv_ctx *ctx, const u32 gain)
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

	reg_gain = s5kjn1sub_gain2reg(ctx, gain);
	ctx->gain = reg_gain;

	DRV_LOG(ctx,"gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));
	return gain;
} /* set_gain_16Xgain */

static u32 set_gain_64Xgain(struct subdrv_ctx *ctx, u32 gain)
{
	u32 reg_gain;
	u32 min_gain = 1 * BASEGAIN;//setuphere for mode use
	u32 max_gain = 64 * BASEGAIN;

	if (gain < min_gain || gain > max_gain) {
		DRV_LOG(ctx,"Error gain setting");

		if (gain < min_gain)
			gain = min_gain;
		else
			gain = max_gain;
	}

	reg_gain = s5kjn1sub_gain2reg(ctx, gain);
	ctx->gain = reg_gain;

	DRV_LOG(ctx,"gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

	subdrv_i2c_wr_u16(ctx, 0x0204, (reg_gain & 0xFFFF));
	return gain;
} /* set_gain_64Xgain */

static int s5kjn1sub_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	DRV_LOG(ctx,"set_gain mode =%d\n", ctx->current_scenario_id);
	switch(ctx->current_scenario_id){
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_CUSTOM1:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM4:
			set_gain_64Xgain(ctx, *((u32 *) para));
		break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
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

static int s5kjn1sub_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x0003);
	subdrv_i2c_wr_u16(ctx, 0x0000, 0x38E1);
	subdrv_i2c_wr_u16(ctx, 0x001E, 0x0007);
	subdrv_i2c_wr_u16(ctx, 0x6028, 0x4000);
	subdrv_i2c_wr_u16(ctx, 0x6010, 0x0001);
	mdelay(6);
	subdrv_i2c_wr_u16(ctx, 0x6226, 0x0001);
	mdelay(10);
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
	s5kjn1sub_sensor_init(ctx);

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
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, 0x013a);

	if (temperature >= 0x0&&temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature>=0x61&&temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature>=0x80&&temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (u8)temperature | 0xFFFFFF0;

	DRV_LOG(ctx, "temperature_convert: %d,temperature =%d\n", temperature_convert,temperature);
	return temperature_convert;
}

