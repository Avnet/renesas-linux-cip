/*
 * Interface for the Renesas RZ/V2L ISP Library
 *
 * Copyright (C) 2022 Renesas Electronics Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef SIMPLE_ISP_H_
#define SIMPLE_ISP_H_

#include "errno.h"

/* Macro */
#define ISP_STATUS_INIT     (0)
#define ISP_STATUS_READY    (1)
#define ISP_STATUS_RUN      (2)

#define AE_MODE_OFF         (0)
#define AE_MODE_ON          (1)

#define IN_FORMAT_RAW8      (0)
#define IN_FORMAT_RAW10     (1)
#define IN_FORMAT_RAW12     (2)
#define IN_FORMAT_DEFAULT   (0xFFFF)

#define OUT_FORMAT_UYVY     (0x00)
#define OUT_FORMAT_YUY2     (0x01)
#define OUT_FORMAT_RGB888   (0x10)
#define OUT_FORMAT_ARGB8888 (0x11)
#define OUT_FORMAT_XRGB8888 (0x12)

#define ISP_FUNC_ALL        (0)
#define ISP_FUNC_BL         (1)
#define ISP_FUNC_WB         (2)
#define ISP_FUNC_GAMMA      (3)
#define ISP_FUNC_CMX        (4)
#define ISP_FUNC_2DNR       (5)
#define ISP_FUNC_3DNR       (6)
#define ISP_FUNC_EMP        (7)
#define ISP_FUNC_AWB        (8)

#define ISP_AEMODE_OFF      (0)
#define ISP_AEMODE_ON       (1)

/* ISP LIMIT Default value */
#define ISP_LIMIT_IN_FMT_MIN        (0)
#define ISP_LIMIT_IN_FMT_MAX        (2)
#define ISP_LIMIT_IN_FMT_STEP       (1)

#define ISP_LIMIT_FRM_INTERVAL_MIN  (1)
#define ISP_LIMIT_FRM_INTERVAL_MAX  (10000)
#define ISP_LIMIT_FRM_INTERVAL_STEP 1

#define ISP_LIMIT_BL_MIN            (0)
#define ISP_LIMIT_BL_MAX            (127)
#define ISP_LIMIT_BL_STEP           (1)

#define ISP_LIMIT_WB_MIN            (0)
#define ISP_LIMIT_WB_MAX            (3)
#define ISP_LIMIT_WB_STEP           (1)

#define ISP_LIMIT_GAMMA_MIN         (1)
#define ISP_LIMIT_GAMMA_MAX         (9999)
#define ISP_LIMIT_GAMMA_STEP        (1)

#define ISP_LIMIT_CMX_MIN           (0)
#define ISP_LIMIT_CMX_MAX           (3)
#define ISP_LIMIT_CMX_STEP          (1)

#define ISP_LIMIT_2DNR_MIN          (0)
#define ISP_LIMIT_2DNR_MAX          (100)
#define ISP_LIMIT_2DNR_STEP         (1)

#define ISP_LIMIT_3DNR_MIN          (0)
#define ISP_LIMIT_3DNR_MAX          (1)
#define ISP_LIMIT_3DNR_STEP         (1)

#define ISP_LIMIT_EMP_MIN           (0)
#define ISP_LIMIT_EMP_MAX           (3)
#define ISP_LIMIT_EMP_STEP          (1)

#define ISP_LIMIT_DRP_LV_MIN        (0)
#define ISP_LIMIT_DRP_LV_MAX        (0)
#define ISP_LIMIT_DRP_LV_STEP       (1)

#define ISP_LIMIT_AE_MIN            (0)
#define ISP_LIMIT_AE_MAX            (1)
#define ISP_LIMIT_AE_STEP           (1)

#define ISP_LIMIT_EXPOSE_LV_MIN     (-400)
#define ISP_LIMIT_EXPOSE_LV_MAX     (+400)
#define ISP_LIMIT_EXPOSE_LV_STEP    (1)

#define ISP_LIMIT_T_BL_MIN          (1)
#define ISP_LIMIT_T_BL_MAX          (254)
#define ISP_LIMIT_T_BL_STEP         (1)

#define ISP_LIMIT_THRESHOLD_MIN     (1)
#define ISP_LIMIT_THRESHOLD_MAX     (64)
#define ISP_LIMIT_THRESHOLD_STEP    (1)

#define ISP_LIMIT_ALL_MIN           (0)
#define ISP_LIMIT_ALL_MAX           (255)
#define ISP_LIMIT_ALL_STEP          (1)

#define ISP_LIMIT_AWB_MIN          (0)
#define ISP_LIMIT_AWB_MAX          (1)
#define ISP_LIMIT_AWB_STEP         (1)

typedef void (*func_isp_callback)(int result, unsigned long out_adr);

/* Function prototype */
int ISP_open(unsigned short width, unsigned short height, unsigned short stride, unsigned short frame_interval, unsigned short in_format, unsigned short out_format, unsigned char *cam_param_p);
int ISP_close(void);
int ISP_set(int isp_func, void *value);
int ISP_AE_Set(int ae_on, int expose_level, int target_y, int threshold);
void ISP_AE_Get(int *gain_p, int *expose_p);
int ISP_go(unsigned long in_image_adr, unsigned long out_image_adr, int level, func_isp_callback callback_p);
int ISP_get_in_buffer(unsigned long *cam_buffer_list);
int ISP_get_param(unsigned short *out_in_format, unsigned short *out_frame_interval,
                  int *out_level, int *out_ae_on, int *out_expose_level,
                  int *out_target_y, int *out_threshold);

int ISP_get(int fid, void *value);

#endif /* SIMPLE_ISP_H_ */
