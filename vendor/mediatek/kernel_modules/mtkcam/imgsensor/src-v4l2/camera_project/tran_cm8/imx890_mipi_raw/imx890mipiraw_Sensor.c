// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/********************************************************************
 *
 * Filename:
 * ---------
 *	 imx890mipiraw_Sensor.c
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
#include "imx890mipiraw_Sensor.h"

static u32 previous_exp[3];
static u16 previous_exp_cnt;

#define IMX890_EEPROM_READ_ID  0xA0
#define IMX890_EEPROM_WRITE_ID 0xA1

//QSC
#define QSC_FLAG       0x2D4E
#define QSC_ADDR       0x2D4F
#define QSC_Check_Flag 0x394F
#define QSC_LENGTH     3072
static char QSC_Array[QSC_LENGTH];

static void set_group_hold(void *arg, u8 en);
static u16 get_gain2reg(u32 gain);
static int imx890_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int imx890_sensor_init(struct subdrv_ctx *ctx);
static int open(struct subdrv_ctx *ctx);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int imx890_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id);
static void imx890_write_shutter(struct subdrv_ctx *ctx, u32 shutter, kal_bool gph);
static u32 get_cur_exp_cnt(struct subdrv_ctx *ctx);
static void write_frame_len(struct subdrv_ctx *ctx, u32 fll);
static void tran_set_dummy(struct subdrv_ctx *ctx);

static int imx890_get_exp_cnt(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_hdr_write_tri_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_hdr_write_tri_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_get_fine_integ_line_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_streaming_control_off(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_hdr_write_tri_dgain(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_get_preisp_flag(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int imx890_feature_get_chip_id(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int get_sensor_temperature(void *arg);

/*write AWB gain to sensor*/
static int m_r_gain = 0, m_b_gain = 0, awb_flag = 0;
static u16 imx890_feedback_awbgain[] = {
    0x0b90, 0x00,
    0x0b91, 0x01,
    0x0b92, 0x00,
    0x0b93, 0x01,
};

/* STRUCT */
static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_TEST_PATTERN, imx890_set_test_pattern},
	{SENSOR_FEATURE_SET_TEST_PATTERN_DATA, imx890_set_test_pattern_data},
	{SENSOR_FEATURE_CUSTOM_GET_EXPCNT_PREISP, imx890_get_exp_cnt},
	{SENSOR_FEATURE_SET_AWB_GAIN, imx890_set_awbgain},
	{SENSOR_FEATURE_SEAMLESS_SWITCH, imx890_seamless_switch},
	{SENSOR_FEATURE_SET_HDR_SHUTTER, imx890_hdr_write_tri_shutter},
	{SENSOR_FEATURE_SET_DUAL_GAIN, imx890_hdr_write_tri_gain},
	{SENSOR_FEATURE_GET_FINE_INTEG_LINE_BY_SCENARIO, imx890_get_fine_integ_line_by_scenario},
	{SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME,imx890_set_multi_shutter_frame_length},
	{SENSOR_FEATURE_SET_STREAMING_RESUME,imx890_streaming_control_on},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND,imx890_streaming_control_off},
	{SENSOR_FEATURE_SET_ESHUTTER,imx890_set_shutter},
	{SENSOR_FEATURE_SET_MULTI_DIG_GAIN, imx890_hdr_write_tri_dgain},
	{SENSOR_FEATURE_CUSTOM_GET_PREISP_FLAG, imx890_get_preisp_flag},
	{SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO, imx890_set_max_framerate_by_scenario},
	{SENSOR_FEATURE_TRAN_GET_SENSOR_CHIP_ID, imx890_feature_get_chip_id},
};

static unsigned char chip_id[32];
#define CHIP_ID_SIZE 11
unsigned char *imx890_get_chip_id(struct subdrv_ctx *ctx,unsigned char *chip_id)
{
	int i = 0,ret;
	unsigned char chipIdData[32];

	//sensor_init();
	subdrv_i2c_wr_u8(ctx, 0x0A02, 0x9F);
	//OTP mode select register control
	subdrv_i2c_wr_u8(ctx, 0x0A00, 0x01);
	mdelay(10);
	ret = subdrv_i2c_rd_u8(ctx, 0x0A01);
	pr_info("0x0A01 ret = %d", ret);

	for (i = 0; i < CHIP_ID_SIZE; i++) {
		chipIdData[i] = subdrv_i2c_rd_u8(ctx, 0x0A17 + i);//OTP data read
		//pr_info("sunyu chipIdData[%d], %02X", i, chipIdData[i]);
		sprintf(chip_id+2*i, "%02X",chipIdData[i]);
	}

	pr_info("sunyu chip_id is %s", chip_id);

	return chip_id;
}

static int imx890_feature_get_chip_id(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	write_frame_len(ctx, ctx->frame_length);
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

}

static int imx890_set_max_framerate_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

	if((scenario_id == SENSOR_SCENARIO_ID_CUSTOM5 && ctx->s_ctx.mode[scenario_id].framelength == 7128) && framerate ==300){
		DRV_LOG(ctx, "custom5 max fps is 30\n");
		return ERROR_NONE;
	}

	frame_length = ctx->s_ctx.mode[scenario_id].pclk / framerate * 10
		/ ctx->s_ctx.mode[scenario_id].linelength;
	ctx->frame_length =
		max(frame_length, ctx->s_ctx.mode[scenario_id].framelength);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	if((ctx->frame_length % 2) != 0){
		ctx->frame_length = ctx->frame_length + 1;
	}

	ctx->current_fps = ctx->pclk / ctx->frame_length * 10 / ctx->line_length;
	ctx->min_frame_length = ctx->frame_length;

	DRV_LOG(ctx, "max_fps(input/output):%u/%u(sid:%u), frame_length =%d, min_frame_length =%d, min_fl_en:1\n", framerate, ctx->current_fps, scenario_id, frame_length, ctx->min_frame_length);
	if (ctx->frame_length > (ctx->exposure[0] + ctx->s_ctx.exposure_margin)){
		tran_set_dummy(ctx);
	}

	ctx->frame_length_rg = ctx->frame_length;
	return ERROR_NONE;
}

//awb
static int feedback_awbgain(struct subdrv_ctx *ctx,u32 r_gain, u32 b_gain)
{
    u32 r_gain_int = 0;
    u32 b_gain_int = 0;

    DRV_LOG(ctx,"feedback_awbgain r_gain: %d %d, b_gain: %d %d mode:%d\n", r_gain, m_r_gain, b_gain, m_b_gain, ctx->current_scenario_id);
    if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 || ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6){
        awb_flag = 1;
        r_gain_int = r_gain / 512;
        b_gain_int = b_gain / 512;
        imx890_feedback_awbgain[1] = r_gain_int;
        imx890_feedback_awbgain[3] = (((r_gain * 100) / 512) - (2 * 100)) * 2;
        imx890_feedback_awbgain[5] = b_gain_int;
        imx890_feedback_awbgain[7] = (((b_gain * 100) / 512) - (b_gain_int * 100)) * 2;
        DRV_LOG(ctx,"feedback_awbgain awbgain[1]: %d, awbgain[3]: %d awbgain[5]: %d awbgain[7]: %d\n", imx890_feedback_awbgain[1], imx890_feedback_awbgain[3],imx890_feedback_awbgain[5],imx890_feedback_awbgain[7]);
        i2c_table_write(ctx,imx890_feedback_awbgain,sizeof(imx890_feedback_awbgain)/sizeof(u16));
    }else{
		awb_flag = 0;
	}

	m_r_gain = r_gain;
	m_b_gain = b_gain;

    return 0;
}

static int imx890_set_awbgain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
    u32 *feature_data_32 = (u32 *) para;

    feedback_awbgain(ctx, (u32)*(feature_data_32 + 1), (u32)*(feature_data_32 + 2));

    return 0;
}

static int imx890_get_preisp_flag(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

//QSC
static void read_QSC_from_eeprom(struct subdrv_ctx *ctx)
{
	int  ret;
	char flag = 0;
	int  i=0;
	char temp = 0;
	char Checksum_Qsc = 0;
	/* Flag of QSC */
	ret = adaptor_i2c_rd_p8(ctx->i2c_client,
			IMX890_EEPROM_READ_ID >> 1, QSC_FLAG, &flag, 1);
	if (ret != 0) {
		pr_err("Read Flag of XTC err, flag[%d]\n", flag);
		return;
	}

	/* Multi-Read QSC Data */
	adaptor_i2c_rd_p8(ctx->i2c_client,
			IMX890_EEPROM_READ_ID >> 1, QSC_ADDR,
			&QSC_Array[0], QSC_LENGTH);
	for(i=0; i < QSC_LENGTH; i++)
	{
		temp= QSC_Array[i] + temp;
		//DRV_LOG(ctx,"imx890_remosaic_qsc Data[%d] = %d\n",i,QSC_Array[i]);
	}

	adaptor_i2c_rd_p8(ctx->i2c_client,IMX890_EEPROM_READ_ID >> 1, QSC_Check_Flag, &Checksum_Qsc, 1);
	if (Checksum_Qsc != (temp%256)) {
		pr_err("imx890_remosaic_qsc checksum err, [0x%x] != [0x%x]\n", Checksum_Qsc, (temp%256));
		return;
	}else{
		DRV_LOG(ctx,"imx890_remosaic_qsc checksum success");
	}
}

//stream on
static void imx890_streaming_on(struct subdrv_ctx *ctx, bool enable)
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
		if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
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

static int imx890_streaming_control_on(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;

	if (*feature_data) {
		imx890_write_shutter(ctx, *feature_data, KAL_TRUE);
	}
	imx890_streaming_on(ctx, TRUE);
	if((ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM3 ||  ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM6) && awb_flag == 0){
		feedback_awbgain(ctx, m_r_gain, m_b_gain);
		awb_flag = 0;
		DRV_LOG_MUST(ctx, "imx890_streaming_control feedback_awbgain \n");
	}
    return 0;
}

static int imx890_streaming_control_off(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	imx890_streaming_on(ctx, FALSE);

    return 0;
}

static u32 imx890_get_fine_integ_line(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id, MUINT32 *fine_integ_line)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_CUSTOM5:
	case SENSOR_SCENARIO_ID_CUSTOM10:
		*fine_integ_line = 2555;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM1:
	case SENSOR_SCENARIO_ID_CUSTOM2:
	case SENSOR_SCENARIO_ID_CUSTOM3:
	case SENSOR_SCENARIO_ID_CUSTOM4:
	case SENSOR_SCENARIO_ID_CUSTOM6:
	case SENSOR_SCENARIO_ID_CUSTOM7:
	case SENSOR_SCENARIO_ID_CUSTOM8:
	case SENSOR_SCENARIO_ID_CUSTOM9:
	case SENSOR_SCENARIO_ID_CUSTOM11:
	case SENSOR_SCENARIO_ID_CUSTOM12:
	case SENSOR_SCENARIO_ID_CUSTOM13:
	case SENSOR_SCENARIO_ID_CUSTOM14:
	case SENSOR_SCENARIO_ID_CUSTOM15:
		*fine_integ_line = 0;
		break;
	default:
		break;
	}
	return 0;
}

static int imx890_get_fine_integ_line_by_scenario(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	imx890_get_fine_integ_line(ctx, (enum SENSOR_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data + 1)));
	return 0;
}


static u16 imx890_gain2reg(struct subdrv_ctx *ctx, const u32 gain)
{
	u16 reg_gain = 0x0;
	u32 gain_value = gain;
	u32 min_gain = 1 * BASEGAIN;//setuphere for mode use
	u32 max_gain = 64 * BASEGAIN;

	if (gain_value < min_gain || gain_value >  max_gain) {
		DRV_LOG(ctx,"Error: gain value out of range");

		if (gain_value < min_gain)
			gain_value = min_gain;
		else if (gain_value > max_gain)
			gain_value = max_gain;
	}

	reg_gain = 16384 - (16384 * BASEGAIN) / gain_value;

	return (u16) reg_gain;
}

static void hdr_write_tri_gain_w_gph(struct subdrv_ctx *ctx,
		u32 lg, u32 mg, u32 sg, kal_bool gph)
{
	u16 reg_lg, reg_mg, reg_sg;

	reg_lg = imx890_gain2reg(ctx, lg);
	reg_mg = mg ? imx890_gain2reg(ctx, mg) : 0;
	reg_sg = imx890_gain2reg(ctx, sg);

	ctx->gain = reg_lg;
	if (gph)
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	/* Long Gian */
	subdrv_i2c_wr_u8(ctx, 0x0204, (reg_lg>>8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0205, reg_lg & 0xFF);
	/* Middle Gian */
	if (mg != 0) {
		subdrv_i2c_wr_u8(ctx, 0x313C, (reg_mg>>8) & 0xFF);
		subdrv_i2c_wr_u8(ctx, 0x313D, reg_mg & 0xFF);
	}
	/* Short Gian */
	subdrv_i2c_wr_u8(ctx, 0x0216, (reg_sg>>8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0217, reg_sg & 0xFF);
	if (gph)
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	DRV_LOG(ctx,
		"lg:0x%x, reg_lg:0x%x, mg:0x%x, reg_mg:0x%x, sg:0x%x, reg_sg:0x%x\n",
		lg, reg_lg, mg, reg_mg, sg, reg_sg);
}

static int imx890_hdr_write_tri_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	DRV_LOG(ctx,"SENSOR_FEATURE_SET_DUAL_GAIN, Lg=%d, Sg=%d\n", (u32) *feature_data, (u32) *(feature_data + 1));
	hdr_write_tri_gain_w_gph(ctx, (u32)*feature_data, 0, (u32)*(feature_data + 1), KAL_TRUE);
	return 0;
}

static void hdr_write_tri_dgain_w_gph(struct subdrv_ctx *ctx, u32 long_dgain, u32 short_dgain)
{
	u32 long_min_dgain = 1 * BASE_DGAIN;
	u32 long_max_dgain = 1 * BASE_DGAIN;
	u32 short_min_dgain = 1 * BASE_DGAIN;
	u32 short_max_dgain = 16 * BASE_DGAIN;
	u32 step = max((u32)1, ctx->s_ctx.dig_gain_step);
	u8 integ = 0x01;
	u8 dec = 0;
	u32 long_actual_dgain = long_dgain;
	u32 short_actual_dgain = short_dgain;

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	/* long dgain */
	if (long_actual_dgain < long_min_dgain) {
		long_actual_dgain = long_min_dgain;
	} else if (long_actual_dgain > long_max_dgain) {
		long_actual_dgain = long_max_dgain;
	}
	integ = (u8)(long_actual_dgain / BASE_DGAIN);
	dec = (u8)((long_actual_dgain % BASE_DGAIN) / step);
	subdrv_i2c_wr_u8(ctx, 0x020E, integ);
	subdrv_i2c_wr_u8(ctx, 0x020F, dec);

	/* short dgain */
	if (short_actual_dgain < short_min_dgain) {
		short_actual_dgain = short_min_dgain;
	} else if (short_actual_dgain > short_max_dgain) {
		short_actual_dgain = short_max_dgain;
	}
	integ = (u8)(short_actual_dgain / BASE_DGAIN);
	dec = (u8)((short_actual_dgain % BASE_DGAIN) / step);
	subdrv_i2c_wr_u8(ctx, 0x0218, integ);
	subdrv_i2c_wr_u8(ctx, 0x0219, dec);
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
	DRV_LOG(ctx,"long dgain %u short dgain %u long[%u/%u] short[%u/%u]",
		long_actual_dgain, short_actual_dgain, subdrv_i2c_rd_u8(ctx, 0x020E), subdrv_i2c_rd_u8(ctx, 0x020F),
		subdrv_i2c_rd_u8(ctx, 0x0218), subdrv_i2c_rd_u8(ctx, 0x0219));

};

static int imx890_hdr_write_tri_dgain(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	u32 preisp_mode = *((u32 *)(feature_data + 2));
	u32 *long_dgain = NULL;
	u32 *short_dgain = NULL;

	if (1 == preisp_mode && ctx->test_pattern != 5) {
		long_dgain = (u32 *)(*feature_data);
		short_dgain = (long_dgain + 1);
		DRV_LOG(ctx,"SENSOR_FEATURE_SET_MULTI_DIG_GAIN L_dig = %u S_dig=%u\n", *long_dgain, *short_dgain);
		hdr_write_tri_dgain_w_gph(ctx, *long_dgain, *short_dgain);
	} else {
		set_multi_dig_gain(ctx, (u32 *)(*feature_data), (u16) (*(feature_data + 1)));
	}

	return 0;

}

static void imx890_set_max_framerate(struct subdrv_ctx *ctx, u16 framerate, kal_bool min_framelength_en)
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

static u32 get_cur_exp_cnt(struct subdrv_ctx *ctx)
{
	u32 exp_cnt = 1;

	if (0x1 == (subdrv_i2c_rd_u8(ctx, 0x33D0) & 0x1)) { // DOL_EN
		if (0x1 == (subdrv_i2c_rd_u8(ctx, 0x33D1) & 0x3)) { // DOL_MODE
			exp_cnt = 3;
		} else {
			exp_cnt = 2;
		}
	}

	return exp_cnt;
}

static void write_frame_len(struct subdrv_ctx *ctx, u32 fll)
{
	// //write_frame_len should be called inside GRP_PARAM_HOLD (0x0104)
	// FRM_LENGTH_LINES must be multiple of 4
	u32 exp_cnt = get_cur_exp_cnt(ctx);

	ctx->frame_length = round_up(fll / exp_cnt, 4) * exp_cnt;

	if (ctx->extend_frame_length_en == KAL_FALSE) {
		DRV_LOG(ctx,"fll in = %d  out = %d exp_cnt = %d\n", fll, ctx->frame_length, exp_cnt);
		subdrv_i2c_wr_u8(ctx, 0x0340, ctx->frame_length / exp_cnt >> 8);
		subdrv_i2c_wr_u8(ctx, 0x0341, ctx->frame_length / exp_cnt & 0xFF);
		ctx->frame_length_rg = ctx->frame_length;
	}
}

static kal_bool imx890_set_auto_flicker(struct subdrv_ctx *ctx)
{
	u16 realtime_fps = 0;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10
				/ ctx->frame_length;
		DRV_LOG(ctx,"autoflicker enable, realtime_fps = %d\n",
			realtime_fps);
		if (realtime_fps >= 587 && realtime_fps <= 615) {
			imx890_set_max_framerate(ctx, 586, 0);
			write_frame_len(ctx, ctx->frame_length);
			return KAL_TRUE;
		}
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			imx890_set_max_framerate(ctx, 296, 0);
			write_frame_len(ctx, ctx->frame_length);
			return KAL_TRUE;
		}
		if (realtime_fps >= 147 && realtime_fps <= 150) {
			imx890_set_max_framerate(ctx, 146, 0);
			write_frame_len(ctx, ctx->frame_length);
			return KAL_TRUE;
		}
	}

	return KAL_FALSE;
}

static void hdr_write_tri_shutter_w_gph(struct subdrv_ctx *ctx,
		u32 le, u32 me, u32 se, kal_bool gph)
{
	u16 exposure_cnt = 0;
	u32 fineIntegTime = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	int i;
	u32 pre_le = 0, pre_me = 0, pre_se = 0;
	u32 pre_fll = 0;
	u32 read_out_len = 0;
	u32 temp_value = 0;

	le = le > fineIntegTime ? FINE_INTEG_CONVERT(le, fineIntegTime) : le;
	me = me > fineIntegTime ? FINE_INTEG_CONVERT(me, fineIntegTime) : me;
	se = se > fineIntegTime ? FINE_INTEG_CONVERT(se, fineIntegTime) : se;

	if (le)
		exposure_cnt++;
	if (me)
		exposure_cnt++;
	if (se)
		exposure_cnt++;

	if (le) {
		le = (u16)max(ctx->s_ctx.exposure_min, (u32)le);
		le = round_up((le) / exposure_cnt, 4) * exposure_cnt;
	}
	if (me) {
		me = (u16)max(ctx->s_ctx.exposure_min, (u32)me);
		me = round_up((me) / exposure_cnt, 4) * exposure_cnt;
	}
	if (se) {
		se = (u16)max(ctx->s_ctx.exposure_min, (u32)se);
		se = round_up((se) / exposure_cnt, 4) * exposure_cnt;
	}

	pre_le = (subdrv_i2c_rd_u8(ctx, 0x0202) << 8) | subdrv_i2c_rd_u8(ctx, 0x0203);
	pre_me = (subdrv_i2c_rd_u8(ctx, 0x313A) << 8) | subdrv_i2c_rd_u8(ctx, 0x313B);
	pre_se = (subdrv_i2c_rd_u8(ctx, 0x0224) << 8) | subdrv_i2c_rd_u8(ctx, 0x0225);
	pre_fll =(subdrv_i2c_rd_u8(ctx, 0x0340) << 8) | subdrv_i2c_rd_u8(ctx, 0x0341);

	if (pre_le)
		pre_le = pre_le * exposure_cnt;
	if (pre_me)
		pre_me = pre_me * exposure_cnt;
	if (pre_se)
		pre_se = pre_se * exposure_cnt;
	if (pre_fll)
		pre_fll = pre_fll * exposure_cnt;

	ctx->frame_length =
		max((u32)(le + me + se + ctx->s_ctx.exposure_margin*exposure_cnt*exposure_cnt),
		ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	DRV_LOG(ctx,"## current fll: %d le: %d, me: %d, se: %d\n", ctx->frame_length, le, me, se);
	read_out_len = 36 + (((subdrv_i2c_rd_u8(ctx, 0x034A) << 8) | subdrv_i2c_rd_u8(ctx, 0x034B)) -
		       ((subdrv_i2c_rd_u8(ctx, 0x346) << 8)|subdrv_i2c_rd_u8(ctx, 0x347)) + 1) / 2;
	read_out_len += 16;

	temp_value = previous_exp[1] + read_out_len * exposure_cnt;
	if(temp_value > se){
		temp_value = temp_value - se;
	}else{
		temp_value = 0;
	}
	if (ctx->frame_length < temp_value) {
		ctx->frame_length = previous_exp[1] + read_out_len * exposure_cnt - se;
		DRV_LOG(ctx,"[case2]adjust fll :%d temp_value = %d", ctx->frame_length, temp_value);
	}

	if (ctx->frame_length < (le + previous_exp[1] + ctx->s_ctx.exposure_margin*exposure_cnt*exposure_cnt)) {
		ctx->frame_length = le + previous_exp[1] + ctx->s_ctx.exposure_margin*exposure_cnt*exposure_cnt;
		DRV_LOG(ctx,"[case1] adjust fll: %d", ctx->frame_length);
	}
	DRV_LOG(ctx,"pre_le: %d, pre_me %d pre_se %d pre_fll %d read_out_len %d"
		" le: %d, me: %d, se: %d, curfll: %d prev_me: %d, prev_se: %d \n",
		pre_le, pre_me, pre_se, pre_fll, read_out_len,
		le, me, se, ctx->frame_length, previous_exp[1], previous_exp[2]);

	for (i = 0; i < previous_exp_cnt; i++)
		previous_exp[i] = 0;
	previous_exp[0] = le;
	switch (exposure_cnt) {
	case 3:
		previous_exp[1] = me;
		previous_exp[2] = se;
		break;
	case 2:
		previous_exp[1] = se;
		previous_exp[2] = 0;
		break;
	case 1:
	default:
		previous_exp[1] = 0;
		previous_exp[2] = 0;
		break;
	}
	previous_exp_cnt = exposure_cnt;

	if (le)
		le = le / exposure_cnt;
	if (me)
		me = me / exposure_cnt;
	if (se)
		se = se / exposure_cnt;

	if (gph)
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);

	imx890_set_auto_flicker(ctx);
	write_frame_len(ctx, ctx->frame_length);

	/* Long exposure */
	subdrv_i2c_wr_u8(ctx, 0x0202, (le >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0203, le & 0xFF);
	/* Muddle exposure */
	if (me) {
		/*MID_COARSE_INTEG_TIME[15:8]*/
		subdrv_i2c_wr_u8(ctx, 0x313A, (me >> 8) & 0xFF);
		/*MID_COARSE_INTEG_TIME[7:0]*/
		subdrv_i2c_wr_u8(ctx, 0x313B, me & 0xFF);
	} else {
		/*MID_COARSE_INTEG_TIME[15:8]*/
		subdrv_i2c_wr_u8(ctx, 0x313A, 0x0);
		/*MID_COARSE_INTEG_TIME[7:0]*/
		subdrv_i2c_wr_u8(ctx, 0x313B, 0x0);
	}
	/* Short exposure */
	subdrv_i2c_wr_u8(ctx, 0x0224, (se >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0225, se & 0xFF);
	if (gph)
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	DRV_LOG(ctx,"X! le:0x%x, me:0x%x, se:0x%x autoflicker_en %d frame_length %d\n",
		le, me, se, ctx->autoflicker_en, ctx->frame_length);
}

static int imx890_hdr_write_tri_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	DRV_LOG(ctx,"SENSOR_FEATURE_SET_HDR_SHUTTER, LE=%d, SE=%d\n", (u32) *feature_data, (u32) *(feature_data + 1));
	hdr_write_tri_shutter_w_gph(ctx, (u32)*feature_data, 0, (u32)*(feature_data+1), KAL_TRUE);

	return 0;
}

#define MAX_CIT_LSHIFT 7
static void imx890_write_shutter(struct subdrv_ctx *ctx, u32 shutter, kal_bool gph)
{
	u16 l_shift = 1;
	u32 fineIntegTime = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	int i;

	DRV_LOG(ctx,"shutter+ =%d, framelength =%d\n",shutter, ctx->frame_length);
	shutter = FINE_INTEG_CONVERT(shutter, fineIntegTime);
	shutter = round_up(shutter, 4);

	DRV_LOG(ctx,"shutter++ =%d, framelength =%d\n",shutter, ctx->frame_length);
	// if (shutter > ctx->min_frame_length - ctx->s_ctx.exposure_margin)
		// ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
	// else
	ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > ctx->s_ctx.frame_length_max)
		ctx->frame_length = ctx->s_ctx.frame_length_max;
	if (shutter < ctx->s_ctx.exposure_min)
		shutter = ctx->s_ctx.exposure_min;

	/* restore current shutter value */
	for (i = 0; i < previous_exp_cnt; i++)
		previous_exp[i] = 0;
	previous_exp[0] = shutter;
	previous_exp_cnt = 1;

	if (gph)
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);

	imx890_set_auto_flicker(ctx);
	ctx->shutter = shutter;

	memset(ctx->exposure, 0, sizeof(ctx->exposure));
	ctx->exposure[0] = (u32) shutter;
	/* long expsoure */
	if (shutter >
		(ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) {

		for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
			if ((shutter >> l_shift) < (ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin))
				break;
		}
		if (l_shift > MAX_CIT_LSHIFT) {
			DRV_LOG(ctx,"Unable to set such a long exposure %d, set to max\n",shutter);

			l_shift = MAX_CIT_LSHIFT;
		}
		shutter = shutter >> l_shift;
		// ctx->frame_length = shutter + ctx->s_ctx.exposure_margin;
		DRV_LOG(ctx,"enter long exposure mode, time is %d", l_shift);
		subdrv_i2c_wr_u8(ctx, 0x3128, l_shift);
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		subdrv_i2c_wr_u8(ctx, 0x3128, 0x00);
		// write_frame_len(ctx, ctx->frame_length);
		ctx->current_ae_effective_frame = 2;
	}

	/* Update Shutter */
	subdrv_i2c_wr_u8(ctx, 0x0350, 0x01); /* Enable auto extend */
	subdrv_i2c_wr_u8(ctx, 0x0202, (shutter >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0203, shutter  & 0xFF);

	if (gph)
		subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	DRV_LOG(ctx,"shutter =%d, framelength =%d\n",shutter, ctx->frame_length);
}	/*	write_shutter  */

static int imx890_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;
	imx890_write_shutter(ctx, *feature_data, KAL_TRUE);
	return 0;
} /* set_shutter */


static void imx890_set_multi_shutter(struct subdrv_ctx *ctx,
				u32 *shutters, u16 shutter_cnt,
				u16 frame_length)
{
	int i;
	int readoutDiff = 0;
	u32 calc_fl = 0;
	u32 calc_fl2 = 0;
	u32 calc_fl3 = 0;
	u16 le = 0, me = 0, se = 0;
	u32 fineIntegTime = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;
	u32 readoutLength = ctx->readout_length;
	u32 readMargin = ctx->read_margin;

	DRV_LOG(ctx,"L!+ le:%d, me:%d, se:%d, fl:%d fl_para:%d  cnt:%d\n", le, me, se, ctx->frame_length, frame_length, shutter_cnt);

	/* convert & set current available shutter value */
	for (i = 0; i < shutter_cnt; i++) {
		shutters[i] = FINE_INTEG_CONVERT(shutters[i], fineIntegTime);
		shutters[i] = (u16)max(ctx->s_ctx.exposure_min,
					(u32)shutters[i]);
		shutters[i] = round_up((shutters[i]) / shutter_cnt, 4) * shutter_cnt;
	}

	/* fl constraint 1: previous se + previous me + current le */
	calc_fl = shutters[0];
	for (i = 1; i < previous_exp_cnt; i++)
		calc_fl += previous_exp[i];
	calc_fl += ctx->s_ctx.exposure_margin*shutter_cnt*shutter_cnt;

	/* fl constraint 2: current se + current me + current le */
	calc_fl2 = shutters[0];
	for (i = 1; i < shutter_cnt; i++)
		calc_fl2 += shutters[i];
	calc_fl2 += ctx->s_ctx.exposure_margin*shutter_cnt*shutter_cnt;

	/* fl constraint 3: readout time cannot be overlapped */
	calc_fl3 = (readoutLength + readMargin);
	if (previous_exp_cnt == shutter_cnt) {
		for (i = 1; i < shutter_cnt; i++) {
			readoutDiff = previous_exp[i] - shutters[i];
			calc_fl3 += readoutDiff > 0 ? readoutDiff : 0;
		}
	}

	/* using max fl of above value */
	calc_fl = max(calc_fl, calc_fl2);
	calc_fl = max(calc_fl, calc_fl3);

	/* set fl range */
	ctx->frame_length = max((u32)calc_fl, ctx->min_frame_length);
	ctx->frame_length = max(ctx->frame_length, (u32)frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);

	/* restore current shutter value */
	for (i = 0; i < previous_exp_cnt; i++)
		previous_exp[i] = 0;
	for (i = 0; i < shutter_cnt; i++)
		previous_exp[i] = shutters[i];
	previous_exp_cnt = shutter_cnt;

	/* register value conversion */
	switch (shutter_cnt) {
	case 3:
		le = shutters[0]/3;
		me = shutters[1]/3;
		se = shutters[2]/3;
		break;
	case 2:
		le = shutters[0]/2;
		me = 0;
		se = shutters[1]/2;
		break;
	case 1:
		le = shutters[0];
		me = 0;
		se = 0;
		break;
	}

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);

	if (!imx890_set_auto_flicker(ctx))
		write_frame_len(ctx, ctx->frame_length);
	/* Long exposure */
	subdrv_i2c_wr_u8(ctx, 0x0202, (le >> 8) & 0xFF);
	subdrv_i2c_wr_u8(ctx, 0x0203, le & 0xFF);
	/* Middle exposure */
	if (me) {
		/*MID_COARSE_INTEG_TIME[15:8]*/
		subdrv_i2c_wr_u8(ctx, 0x313A, (me >> 8) & 0xFF);
		/*MID_COARSE_INTEG_TIME[7:0]*/
		subdrv_i2c_wr_u8(ctx, 0x313B, me & 0xFF);
	}
	/* Short exposure */
	if (se) {
		subdrv_i2c_wr_u8(ctx, 0x0224, (se >> 8) & 0xFF);
		subdrv_i2c_wr_u8(ctx, 0x0225, se & 0xFF);
	}

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);

	DRV_LOG(ctx,"L! le:%d, me:%d, se:%d, fl:%d\n", le, me, se, ctx->frame_length);
}

static int imx890_set_multi_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	unsigned long long *feature_data = (unsigned long long *) para;

	imx890_set_multi_shutter(ctx, (UINT32 *)(*feature_data), (UINT16) (*(feature_data + 1)), (UINT16) (*(feature_data + 2)));

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
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 150}, {128,456}, {0, 0}, {0, 0}, {128,456},
		// <cust6> <cust7> <cust8> cust9 cust10
		{2048,1536}, {0, 0}, {0, 0},{0, 0},{0, 0},
	},
	.iMirrorFlip = 0,
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.i4VCPackNum = 1,
	.i4FullRawW = 4096,
	.i4FullRawH = 3072,
	.i4ModeIndex = 3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
	        .i4PDPattern = 1,
	        .i4BinFacX = 2,
	        .i4BinFacY = 4,
	        .i4PDRepetition = 0,
	        .i4PDOrder = {1}, //R=1, L=0
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
		// <pre> <cap> <normal_video> <hs_video> <<slim_video>>
		{0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
		// <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
		{0, 150}, {128,456}, {0, 0}, {0, 0}, {128,456},
		// <cust6> <cust7> <cust8> cust9 cust10
		{2048,1536}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = 0,
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.i4VCPackNum = 1,
	.i4FullRawW = 8192,
	.i4FullRawH = 6144,
	.i4ModeIndex = 3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
	        .i4PDPattern = 1,
	        .i4BinFacX = 4,
	        .i4BinFacY = 2,
	        .i4PDRepetition = 0,
	        .i4PDOrder = {1}, //R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
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
                // <pre> <cap> <normal_video> <hs_video> <<slim_video>>
                {0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
                // <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
                {0, 0}, {128,456}, {0, 0}, {0, 0}, {128,456},
                // <cust6> <cust7> <cust8> cust9 cust10
                {2048,1536}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = 0,
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.i4VCPackNum = 1,
	.i4FullRawW = 4096,
	.i4FullRawH = 2304,
	.i4ModeIndex = 3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
	        .i4PDPattern = 1,
	        .i4BinFacX = 2,
	        .i4BinFacY = 4,
	        .i4PDRepetition = 0,
	        .i4PDOrder = {1}, //R=1, L=0
	},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_custom2 = {
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
                // <pre> <cap> <normal_video> <hs_video> <<slim_video>>
                {0, 0}, {0, 0}, {0, 384}, {0, 0}, {0, 0},
                // <<cust1>> <<cust2>> <<cust3>> <cust4> <cust5>
                {0, 0}, {128,456}, {0, 0}, {0, 0}, {128,456},
                // <cust6> <cust7> <cust8> cust9 cust10
                {2048,1536}, {0, 0}, {0, 0},
	},
	.iMirrorFlip = 0,
	.i4VolumeX = 1,
	.i4VolumeY = 1,
	.i4VCPackNum = 1,
	.i4FullRawW = 3840,
	.i4FullRawH = 2160,
	.i4ModeIndex = 3,
	/* VC's PD pattern description */
	.sPDMapInfo[0] = {
	        .i4PDPattern = 1,
	        .i4BinFacX = 2,
	        .i4BinFacY = 4,
	        .i4PDRepetition = 0,
	        .i4PDOrder = {1}, //R=1, L=0
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
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
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
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
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
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0240,
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
			.hsize = 0x0F00,
			.vsize = 0x0870,
			.user_data_desc = VC_STAGGER_NE,
		},
	},
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
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x0F00,
			.vsize = 0x021C,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
#endif
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
			 .data_type = 0x2b,
			 .hsize = 0x0800,
			 .vsize = 0x0600,
			 .user_data_desc = VC_PDAF_STATS,
		 },
	},
};
//video isz
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
};
//4096*3072 1exp seamless_switch cus9
static struct mtk_mbus_frame_desc_entry frame_desc_cus8[] = {
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
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
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
    {
         .bus.csi2 = {
			.channel = 3,
			.data_type = 0x2b,
			.hsize = 0x1000,
			.vsize = 0x0300,
			.user_data_desc = VC_PDAF_STATS,
		},
	},
};

//mtk 2dol
static struct mtk_mbus_frame_desc_entry frame_desc_cus10[] = {
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


static struct mtk_sensor_saturation_info imgsensor_saturation_info = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 1023,
};

static struct mtk_sensor_saturation_info imgsensor_saturation_info_14bit_dol = {
	.gain_ratio = 1000,
	.OB_pedestal = 64,
	.saturation_level = 16383,
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
		.pclk = 1526400000,
		.linelength = 15616,
		.framelength = 3258,
		.max_framerate = 300,
		.mipi_pixel_rate = 960000000,
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
		.ae_binning_ratio = 1465,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 73,
		},
	},
	{//capture copy cl8
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = addr_data_pair_capture,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_capture),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = imx890_capture_seamless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx890_capture_seamless_setting),
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 1526400000,
		.linelength = 15616,
		.framelength = 3258,
		.max_framerate = 300,
		.mipi_pixel_rate = 960000000,
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
		.ae_binning_ratio = 1465,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 73,
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
		.pclk = 2390400000,
		.linelength = 15616,
		.framelength = 5098,
		.max_framerate = 300,
		.mipi_pixel_rate = 918860000,
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
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 73,
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
		.pclk = 1881600000,
		.linelength = 8816,
		.framelength = 1776,
		.max_framerate = 1200,
		.mipi_pixel_rate = 582860000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
                        .full_w = 8192,
                        .full_h = 6144,
                        .x0_offset = 0,
                        .y0_offset = 0,
                        .w0_size = 8192,
                        .h0_size = 6144,
                        .scale_w = 2048,
                        .scale_h = 1536,
                        .x1_offset = 0,
                        .y1_offset = 0,
                        .w1_size = 2048,
                        .h1_size = 1536,
                        .x2_tg_offset = 0,
                        .y2_tg_offset = 0,
                        .w2_tg_size = 2048,
                        .h2_tg_size = 1536,
                },
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1465,
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
		.pclk = 3110400000,
		.linelength = 8816,
		.framelength = 1468,
		.max_framerate = 2400,
		.mipi_pixel_rate = 935310000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
                        .full_w = 8192,
                        .full_h = 6144,
                        .x0_offset = 0,
                        .y0_offset = 912,
                        .w0_size = 8192,
                        .h0_size = 4320,
                        .scale_w = 2048,
                        .scale_h = 1080,
                        .x1_offset = 64,
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
		.ae_binning_ratio = 1465,
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
		.pclk = 1280000000,
		.linelength = 16704,
		.framelength = 2552,
		.max_framerate = 300,
		.mipi_pixel_rate = 342860000,
		.readout_length = 0,
		.read_margin = 0,
		.imgsensor_winsize_info = {
                        .full_w = 8192,
                        .full_h = 6144,
                        .x0_offset = 896,
                        .y0_offset = 672,
                        .w0_size = 6400,
                        .h0_size = 4800,
                        .scale_w = 1600,
                        .scale_h = 1200,
                        .x1_offset = 0,
                        .y1_offset = 0,
                        .w1_size = 1600,
                        .h1_size = 1200,
                        .x2_tg_offset = 0,
                        .y2_tg_offset = 0,
                        .w2_tg_size = 1600,
                        .h2_tg_size = 1200,
                },
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = &imgsensor_pd_info_binning,
		.ae_binning_ratio = 1465,
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
		.pclk = 2217600000,
		.linelength = 15616,
		.framelength = 2366,
		.max_framerate = 600,
		.mipi_pixel_rate = 806400000,
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
		.ae_binning_ratio = 1465,
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
		.pclk = 1280000000,
		.linelength = 11552,
		.framelength = 7386,
		.max_framerate = 100,
		.mipi_pixel_rate = 995660000,
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
		.pclk = 2390400000,
		.linelength = 15616,
		.framelength = 5098,
		.max_framerate = 300,
		.mipi_pixel_rate = 918860000,
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
		.imgsensor_pd_info = &imgsensor_pd_info_video,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {
			.cphy_settle = 73,
		},
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
		.pclk = 3340800000,
		.linelength = 15616,
		.framelength = 7128,
		.max_framerate = 300,
		.mipi_pixel_rate = 1231540000,
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
		.coarse_integ_step = 8,//exp step
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_LE].min = 8,
		.multi_exposure_shutter_range[IMGSENSOR_EXPOSURE_ME].min = 8,
		.pdaf_cap = TRUE,
		.imgsensor_pd_info = &imgsensor_pd_info_custom2,
		.ae_binning_ratio = 1465,
		.fine_integ_line = 2555,
		.delay_frame = 2,
		.csi_param = {0},
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW14_R,
		.saturation_info = &imgsensor_saturation_info_14bit_dol,
		.csi_param = {
			.cphy_settle = 69,
		},
	},
	{//custom6 AI CAM ISZ
		.frame_desc = frame_desc_cus6,
		.num_entries = ARRAY_SIZE(frame_desc_cus6),
		.mode_setting_table = addr_data_pair_custom6,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom6),
		.seamless_switch_group = 1,
		.seamless_switch_mode_setting_table = imx890_custom6_seameless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx890_custom6_seameless_setting),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 1526400000,
		.linelength = 11552,
		.framelength = 4404,
		.max_framerate = 300,
		.mipi_pixel_rate = 960000000,
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
		.ae_binning_ratio = 1000,
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
		.seamless_switch_group = 2,
		.seamless_switch_mode_setting_table = imx890_custom7_seameless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx890_custom7_seameless_setting),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 1536000000,
		.linelength = 11552,
		.framelength = 4432,
		.max_framerate = 300,
		.mipi_pixel_rate = 822860000,
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
		.ae_binning_ratio = 1465,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{
		.frame_desc = frame_desc_cus8,
		.num_entries = ARRAY_SIZE(frame_desc_cus8),
		.mode_setting_table = addr_data_pair_custom8,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom8),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = imx890_custom8_seameless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx890_custom8_seameless_setting),
		.raw_cnt = 1,
		.exp_cnt = 1,
		.hdr_mode = HDR_NONE,
		.pclk = 3513600000,
		.linelength = 15616,
		.framelength = 7500,
		.max_framerate = 300,
		.mipi_pixel_rate = 1365940000,
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
		.ae_binning_ratio = 1465,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{
		.frame_desc = frame_desc_cus9,
		.num_entries = ARRAY_SIZE(frame_desc_cus9),
		.mode_setting_table = addr_data_pair_custom9,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom9),
		.seamless_switch_group = 3,
		.seamless_switch_mode_setting_table = imx890_custom9_seameless_setting,
		.seamless_switch_mode_setting_len = ARRAY_SIZE(imx890_custom9_seameless_setting),
		.raw_cnt = 2,
		.exp_cnt = 2,
		.hdr_mode = HDR_RAW_STAGGER,
		.pclk = 3513600000,
		.linelength = 15616,
		.framelength = 7496,
		.max_framerate = 300,
		.mipi_pixel_rate = 1365940000,
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
		.ae_binning_ratio = 1465,
		.fine_integ_line = 0,
		.delay_frame = 2,
		.csi_param = {0},
	},
		{
		.frame_desc = frame_desc_cus10,
		.num_entries = ARRAY_SIZE(frame_desc_cus10),
		.mode_setting_table = addr_data_pair_custom10,
		.mode_setting_len = ARRAY_SIZE(addr_data_pair_custom10),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.raw_cnt = 2,
		.exp_cnt = 2,
		.hdr_mode = HDR_NONE,
		.pclk = 3340800000,
		.linelength = 15616,
		.framelength = 7128,
		.max_framerate = 300,
		.mipi_pixel_rate = 1231540000,
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
		.ae_binning_ratio = 1465,
		.fine_integ_line = 2555,
		.delay_frame = 2,
		.csi_param = {0},
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = IMX890_SENSOR_ID,
	.reg_addr_sensor_id = {0x0016, 0x0017},
	.i2c_addr_table = {0x20, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info =  PARAM_UNDEFINED,
	.eeprom_num =  PARAM_UNDEFINED,
	.resolution = {8192, 6144},
	.mirror = IMAGE_NORMAL,

	.mclk = 24,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_CPHY,
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_type = 0,
	.ana_gain_step = 1,
	.ana_gain_table = PARAM_UNDEFINED,
	.ana_gain_table_size = PARAM_UNDEFINED,
	.min_gain_iso = 50,
	.exposure_def = 0x3D0,
	.exposure_min = 8,
	.exposure_max =  (65532*128) - 48,
	.exposure_step = 4,
	.exposure_margin = 48,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = BASE_DGAIN * 16,
	.dig_gain_step = 4,
	.saturation_info = &imgsensor_saturation_info,

	.frame_length_max = 0xFFFF,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 3,
	.start_exposure_offset = 1600000,

	.pdaf_type = PDAF_SUPPORT_CAMSV_QPD,
	.hdr_type = HDR_SUPPORT_STAGGER_FDOL,
	.seamless_switch_support = TRUE,
	.temperature_support = TRUE,
	.g_temp = get_sensor_temperature,
	.g_gain2reg = get_gain2reg,
	.s_gph = set_group_hold,

	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = 0x0101,
	.reg_addr_exposure = {
			{0x0202, 0x0203},
			{0x313A, 0x313B},
			{0x0224, 0x0225},
	},
	.long_exposure_support = TRUE,
	.reg_addr_exposure_lshift = 0x3128,
	.reg_addr_ana_gain = {
			{0x0204, 0x0205},
			{0x313C, 0x313D},
			{0x0216, 0x0217},
	},
	.reg_addr_frame_length = {{0x0340, 0x0341}},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = 0x0350,
	.reg_addr_frame_count = 0x0005,
	.reg_addr_fast_mode = 0x3010,

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
	.get_id = imx890_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK,  24, 0},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 6},
	{HW_ID_AVDD,  2800000, 3},
	{HW_ID_AVDD1, 1800000, 3},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD,  1100000, 4},
	{HW_ID_RST,1, 5}
};

const struct subdrv_entry imx890_mipi_raw_entry = {
	.name = "imx890_mipi_raw",
	.id = IMX890_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
static void set_group_hold(void *arg, u8 en)
{
	 struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;

	 if (en)
		 subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	 else
		 subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);
}

static u16 get_gain2reg(u32 gain)
{
	return 16384 - (16384 * BASEGAIN) / gain;
}


static int imx890_get_exp_cnt(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 *feature_return_para_32 = (u32 *) para;
	unsigned long long *feature_data = (unsigned long long *) para;

	DRV_LOG(ctx,"imx890_get_exp_cnt mode: %lld\n", *feature_data);
	if (*feature_data == SENSOR_SCENARIO_ID_CUSTOM5)
		*(feature_return_para_32 + 1) = 2;

	return 0;
}

static int imx890_seamless_switch(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	enum SENSOR_SCENARIO_ID_ENUM scenario_id;
	struct mtk_hdr_ae *ae_ctrl = NULL;
	u64 *feature_data = (u64 *)para;
	u32 exp_cnt = 0;

	if (feature_data == NULL) {
		DRV_LOGE(ctx, "input scenario is null!");
		return ERROR_NONE;
	}
	scenario_id = *feature_data;
	if ((feature_data + 1) != NULL)
		ae_ctrl = (struct mtk_hdr_ae *)((uintptr_t)(*(feature_data + 1)));
	else
		DRV_LOGE(ctx, "no ae_ctrl input");

	check_current_scenario_id_bound(ctx);
	DRV_LOG(ctx, "E: set seamless switch %u %u\n", ctx->current_scenario_id, scenario_id);
	if (!ctx->extend_frame_length_en)
		DRV_LOGE(ctx, "please extend_frame_length before seamless_switch!\n");
	ctx->extend_frame_length_en = FALSE;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOGE(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_group == 0 ||
		ctx->s_ctx.mode[scenario_id].seamless_switch_group !=
			ctx->s_ctx.mode[ctx->current_scenario_id].seamless_switch_group) {
		DRV_LOGE(ctx, "seamless_switch not supported\n");
		return ERROR_NONE;
	}
	if (ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table == NULL) {
		DRV_LOGE(ctx, "Please implement seamless_switch setting\n");
		return ERROR_NONE;
	}

	exp_cnt = ctx->s_ctx.mode[scenario_id].exp_cnt;
	ctx->is_seamless = TRUE;
	update_mode_info(ctx, scenario_id);

	subdrv_i2c_wr_u8(ctx, 0x0104, 0x01);
	subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x02);//GPH Start

	i2c_table_write(ctx,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_table,
		ctx->s_ctx.mode[scenario_id].seamless_switch_mode_setting_len);

	if (ae_ctrl) {
		switch (ctx->s_ctx.mode[scenario_id].hdr_mode) {
		case HDR_RAW_STAGGER:
			set_multi_shutter_frame_length(ctx, (u64 *)&ae_ctrl->exposure, exp_cnt, 0);
			set_multi_gain(ctx, (u32 *)&ae_ctrl->gain, exp_cnt);
			break;
		default:
			imx890_write_shutter(ctx, ae_ctrl->exposure.le_exposure, KAL_TRUE);
			set_gain(ctx, ae_ctrl->gain.le_gain);
			break;
		}
	}
	subdrv_i2c_wr_u8(ctx, 0x0104, 0x00);//GPH End

	ctx->fast_mode_on = TRUE;
	ctx->ref_sof_cnt = ctx->sof_cnt;
	ctx->is_seamless = FALSE;
	DRV_LOG(ctx, "X: set seamless switch done\n");
	return ERROR_NONE;
}

static int imx890_set_test_pattern(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u32 mode = *((u32 *)para);

	DRV_LOG(ctx,"set_test_pattern_mode mode %d -> %d sensor mode = %d\n", ctx->test_pattern, mode, ctx->current_scenario_id);
	//1:Solid Color 2:Color bar 5:Black
	if (mode == 2){
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x02);
	}else if (mode == 5){
		subdrv_i2c_wr_u8(ctx, 0x020E, 0x00);//Long Dgain = 0
		if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5){
			subdrv_i2c_wr_u8(ctx, 0x0218, 0x00);//Short Dgain = 0
		}
	}
	else
		subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /*No pattern*/

	if ((ctx->test_pattern) && (mode != ctx->test_pattern)) {
		if (ctx->test_pattern == 5){
			subdrv_i2c_wr_u8(ctx, 0x020E, 0x01);//Long Dgain = 1x
			if(ctx->current_scenario_id == SENSOR_SCENARIO_ID_CUSTOM5){
				subdrv_i2c_wr_u8(ctx, 0x0218, 0x01);//Short Dgain = 1x
			}
		}else if (mode == 0)
			subdrv_i2c_wr_u8(ctx, 0x0601, 0x00); /* No pattern */
	}

	ctx->test_pattern = mode;

	return 0;
}

static int imx890_set_test_pattern_data(struct subdrv_ctx *ctx, u8 *para, u32 *len)
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

static int imx890_sensor_init(struct subdrv_ctx *ctx)
{
	DRV_LOG(ctx, "E\n");
	i2c_table_write(ctx, sensor_init_addr_data, sizeof(sensor_init_addr_data)/sizeof(u16));

	/*enable temperature sensor, TEMP_SEN_CTL:*/
	subdrv_i2c_wr_u8(ctx, 0x0138, 0x01);
	/* set MIPI auto ctrl */
	subdrv_i2c_wr_u8(ctx, 0x0808, 0x00);
	DRV_LOG(ctx, "X\n");

	return 0;
}

static int imx890_get_imgsensor_id(struct subdrv_ctx *ctx, u32 *sensor_id)
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
				read_QSC_from_eeprom(ctx);
				imx890_get_chip_id(ctx,chip_id);
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
	if (imx890_get_imgsensor_id(ctx, &sensor_id) != ERROR_NONE)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail setting */
	imx890_sensor_init(ctx);

	subdrv_i2c_wr_u8(ctx, 0x0100, 0x00);
	subdrv_i2c_wr_seq_p8(ctx, 0xC800, QSC_Array, sizeof(QSC_Array));

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

static int vsync_notify(struct subdrv_ctx *ctx, unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	if (ctx->fast_mode_on && (sof_cnt > ctx->ref_sof_cnt)) {
		ctx->fast_mode_on = FALSE;
		ctx->ref_sof_cnt = 0;
		DRV_LOG(ctx, "seamless_switch disabled.");
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
	}
	return 0;
}

static int get_sensor_temperature(void *arg)
{
	struct subdrv_ctx *ctx = (struct subdrv_ctx *)arg;
	u8 temperature = 0;
	int temperature_convert = 0;

	temperature = subdrv_i2c_rd_u8(ctx, 0x013a);

	if (temperature <= 0x60)
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
