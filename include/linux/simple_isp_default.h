/*
 * Default data for the Renesas RZ/V2L ISP Library
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

#ifndef SIMPLE_ISP_DEFAULT_H_
#define SIMPLE_ISP_DEFAULT_H_

/* Color matrix preset data */
/* Preset1 : Uncorrect */
#define ISP_CMX_P1_C11      (0x2000)
#define ISP_CMX_P1_C12      (0x0000)
#define ISP_CMX_P1_C13      (0x0000)
#define ISP_CMX_P1_C21      (0x0000)
#define ISP_CMX_P1_C22      (0x2000)
#define ISP_CMX_P1_C23      (0x0000)
#define ISP_CMX_P1_C31      (0x0000)
#define ISP_CMX_P1_C32      (0x0000)
#define ISP_CMX_P1_C33      (0x2000)

/* Preset2 : Standard correct */
#define ISP_CMX_P2_C11      (0x2f63)
#define ISP_CMX_P2_C12      (0xfa8a)
#define ISP_CMX_P2_C13      (0xfd9b)
#define ISP_CMX_P2_C21      (0xf4bd)
#define ISP_CMX_P2_C22      (0x2b7d)
#define ISP_CMX_P2_C23      (0x0a95)
#define ISP_CMX_P2_C31      (0xfed5)
#define ISP_CMX_P2_C32      (0xdd8e)
#define ISP_CMX_P2_C33      (0x4aa3)

/* Preset3 : Vivid correct */
#define ISP_CMX_P3_C11      (0x3d05)
#define ISP_CMX_P3_C12      (0xf9f7)
#define ISP_CMX_P3_C13      (0xf372)
#define ISP_CMX_P3_C21      (0xf17a)
#define ISP_CMX_P3_C22      (0x3992)
#define ISP_CMX_P3_C23      (0x00a1)
#define ISP_CMX_P3_C31      (0xf7df)
#define ISP_CMX_P3_C32      (0xd5c0)
#define ISP_CMX_P3_C33      (0x5a06)

/* Preset4 : Sepia correct */
#define ISP_CMX_P4_C11      (0x076d)
#define ISP_CMX_P4_C12      (0x27ff)
#define ISP_CMX_P4_C13      (0x02ef)
#define ISP_CMX_P4_C21      (0x0580)
#define ISP_CMX_P4_C22      (0x24dc)
#define ISP_CMX_P4_C23      (0x0171)
#define ISP_CMX_P4_C31      (0x054f)
#define ISP_CMX_P4_C32      (0x166f)
#define ISP_CMX_P4_C33      (0x0a02)


/* White balance preset data */
/* Preset1 : Day light */
#define ISP_WB_P1_BLUE      (0x1EFC)
#define ISP_WB_P1_GREEN     (0x1581)
#define ISP_WB_P1_RED       (0x1913)

/* Preset2 : Neutral light */
#define ISP_WB_P2_BLUE      (0x1C2B)
#define ISP_WB_P2_GREEN     (0x1581)
#define ISP_WB_P2_RED       (0x1B95)

/* Preset3 : Interior light */
#define ISP_WB_P3_BLUE      (0x195A)
#define ISP_WB_P3_GREEN     (0x1581)
#define ISP_WB_P3_RED       (0x1EA6)

/* Preset4 : Lamp light */
#define ISP_WB_P4_BLUE      (0x1689)
#define ISP_WB_P4_GREEN     (0x1581)
#define ISP_WB_P4_RED       (0x227A)

/* ISP LIMIT Default value */
#define ISP_LIMIT_IN_FMT_DEFAULT        (1)
#define ISP_LIMIT_FRM_INTERVAL_DEFAULT  (33)
#define ISP_LIMIT_BL_DEFAULT            (0)
#define ISP_LIMIT_WB_DEFAULT            (1)
#define ISP_LIMIT_GAMMA_DEFAULT         (100)
#define ISP_LIMIT_CMX_DEFAULT           (1)
#define ISP_LIMIT_2DNR_DEFAULT          (100)
#define ISP_LIMIT_3DNR_DEFAULT          (1)
#define ISP_LIMIT_EMP_DEFAULT           (0)
#define ISP_LIMIT_DRP_LV_DEFAULT        (0)
#define ISP_LIMIT_AE_DEFAULT            (0)
#define ISP_LIMIT_EXPOSE_LV_DEFAULT     (100)
#define ISP_LIMIT_T_BL_DEFAULT          (106)
#define ISP_LIMIT_THRESHOLD_DEFAULT     (10)
#define ISP_LIMIT_ALL_DEFAULT           (0)
#define ISP_LIMIT_AWB_DEFAULT           (1)

/* 3DNR param */
#define ISP_PARAM_3DNR_Y_THRES_A    (8)
#define ISP_PARAM_3DNR_Y_THRES_B    (16)
#define ISP_PARAM_3DNR_Y_TILT       (512)
#define ISP_PARAM_3DNR_C_THRES_A    (8)
#define ISP_PARAM_3DNR_C_THRES_B    (16)
#define ISP_PARAM_3DNR_C_TILT       (512)

#define ISP_PARAM_3DNR_Y_COEF       (64)
#define ISP_PARAM_3DNR_C_COEF       (32)
#define ISP_PARAM_3DNR_Y_ALPHA_MAX  (128)
#define ISP_PARAM_3DNR_C_ALPHA_MAX  (128)

#endif /* SIMPLE_ISP_DEFAULT_H_ */
