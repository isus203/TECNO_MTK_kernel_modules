/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2019 MediaTek Inc. */

#ifndef __ADAPTOR_HW_H__
#define __ADAPTOR_HW_H__

int adaptor_hw_power_on(struct adaptor_ctx *ctx);
int adaptor_hw_power_off(struct adaptor_ctx *ctx);
int adaptor_hw_init(struct adaptor_ctx *ctx);
int adaptor_hw_sensor_reset(struct adaptor_ctx *ctx);
int tran_mipi_switch_onoff(struct adaptor_ctx *ctx, int enable);
//#define tran_d9300evb
#ifdef tran_d9300evb
extern int aw95016_set_dir(unsigned int port, int mode);
extern int aw95016_set_output(unsigned int port, int level);
#endif

#endif
