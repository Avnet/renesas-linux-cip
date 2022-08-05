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
/* Preset0 : Truecolor */
#define ISP_CMX_P1_C11                  (0x2125)
#define ISP_CMX_P1_C12                  (0x02e2)
#define ISP_CMX_P1_C13                  (0xfbc8)
#define ISP_CMX_P1_C21                  (0xfffd)
#define ISP_CMX_P1_C22                  (0x1df6)
#define ISP_CMX_P1_C23                  (0x0291)
#define ISP_CMX_P1_C31                  (0xfec5)
#define ISP_CMX_P1_C32                  (0xf74b)
#define ISP_CMX_P1_C33                  (0x289b)
                                                
/* Preset1 : Standard correct */                
#define ISP_CMX_P2_C11                  (0x2c69)
#define ISP_CMX_P2_C12                  (0xf78e)
#define ISP_CMX_P2_C13                  (0xfb2c)
#define ISP_CMX_P2_C21                  (0xfddd)
#define ISP_CMX_P2_C22                  (0x22c9)
#define ISP_CMX_P2_C23                  (0x0163)
#define ISP_CMX_P2_C31                  (0xfdf0)
#define ISP_CMX_P2_C32                  (0xf68a)
#define ISP_CMX_P2_C33                  (0x2c82)
                                                
/* Preset2 : Vivid correct */                   
#define ISP_CMX_P3_C11                  (0x2521)
#define ISP_CMX_P3_C12                  (0xfa89)
#define ISP_CMX_P3_C13                  (0xfcdd)
#define ISP_CMX_P3_C21                  (0xfbe1)
#define ISP_CMX_P3_C22                  (0x27ad)
#define ISP_CMX_P3_C23                  (0xff26)
#define ISP_CMX_P3_C31                  (0x00f1)
#define ISP_CMX_P3_C32                  (0xf172)
#define ISP_CMX_P3_C33                  (0x2c25)
                                                
/* Preset3 : Sepia correct */                   
#define ISP_CMX_P4_C11                  (0x0a20)
#define ISP_CMX_P4_C12                  (0x195a)
#define ISP_CMX_P4_C13                  (0x0509)
#define ISP_CMX_P4_C21                  (0x0848)
#define ISP_CMX_P4_C22                  (0x1f51)
#define ISP_CMX_P4_C23                  (0xfd2f)
#define ISP_CMX_P4_C31                  (0x07d7)
#define ISP_CMX_P4_C32                  (0x1092)
#define ISP_CMX_P4_C33                  (0x0731)
                                                
/* White balance preset data */                 
/* Preset0 : Day light */                       
#define ISP_WB_P1_BLUE                  (0x1a16)
#define ISP_WB_P1_GREEN                 (0x1041)
#define ISP_WB_P1_RED                   (0x167a)
                                                
/* Preset1 : Neutral light */                   
#define ISP_WB_P2_BLUE                  (0x1656)
#define ISP_WB_P2_GREEN                 (0x1041)
#define ISP_WB_P2_RED                   (0x18d8)
                                                
/* Preset2 : Interior light */                  
#define ISP_WB_P3_BLUE                  (0x1297)
#define ISP_WB_P3_GREEN                 (0x1041)
#define ISP_WB_P3_RED                   (0x1b3d)
                                                
/* Preset3 : Lamp light */                      
#define ISP_WB_P4_BLUE                  (0x10dd)
#define ISP_WB_P4_GREEN                 (0x1041)
#define ISP_WB_P4_RED                   (0x21e4)
                                                
/* ISP LIMIT Default value */                   
#define ISP_LIMIT_BL_R_DEFAULT            (  16)
#define ISP_LIMIT_BL_G_DEFAULT            (  16)
#define ISP_LIMIT_BL_B_DEFAULT            (  16)
#define ISP_LIMIT_WB_DEFAULT              (   1)
#define ISP_LIMIT_GAMMA_DEFAULT           ( 100)
#define ISP_LIMIT_CMX_DEFAULT             (   1)
#define ISP_LIMIT_2DNR_DEFAULT            (  25)
#define ISP_LIMIT_3DNR_DEFAULT            (   1)
#define ISP_LIMIT_EMP_DEFAULT             (   0)
#define ISP_LIMIT_AE_DEFAULT              (   0)
#define ISP_LIMIT_EXPOSE_LV_DEFAULT       ( 100)
#define ISP_LIMIT_T_BRIGHTNESS_DEFAULT    ( 118)
#define ISP_LIMIT_THRESHOLD_DEFAULT       (  10)
#define ISP_LIMIT_AWB_DEFAULT             (   0)
                                                
/* 3DNR param */                                
#define ISP_PARAM_3DNR_Y_COEF             (  64)
#define ISP_PARAM_3DNR_C_COEF             (  32)
#define ISP_PARAM_3DNR_Y_ALPHA_MAX        ( 128)
#define ISP_PARAM_3DNR_Y_THRES_A          (   8)
#define ISP_PARAM_3DNR_Y_THRES_B          (  16)
#define ISP_PARAM_3DNR_Y_TILT             ( 512)
#define ISP_PARAM_3DNR_C_ALPHA_MAX        ( 128)
#define ISP_PARAM_3DNR_C_THRES_A          (   8)
#define ISP_PARAM_3DNR_C_THRES_B          (  16)
#define ISP_PARAM_3DNR_C_TILT             ( 512)

/* else */
#define ISP_LIMIT_IN_FMT_DEFAULT          (   1)
#define ISP_LIMIT_FRM_INTERVAL_DEFAULT    (  33)
#define ISP_LIMIT_BL_DEFAULT              (  16)
#define ISP_LIMIT_DRP_LV_DEFAULT          (   0)
#define ISP_LIMIT_T_BL_DEFAULT            ISP_LIMIT_T_BRIGHTNESS_DEFAULT
#define ISP_LIMIT_ALL_DEFAULT             (   0)


#endif /* SIMPLE_ISP_DEFAULT_H_ */
