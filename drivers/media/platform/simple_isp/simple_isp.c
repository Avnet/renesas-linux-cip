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

#include <asm/cacheflush.h>
#include <asm/io.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/simple_isp.h>
#include <linux/drpai.h>
#include <linux/simple_isp_ae.h>
#include <linux/simple_isp_config.h>
#include <linux/simple_isp_gamma.h>
#include <linux/simple_isp_default.h>

#define ISP_DEBUG 0
#define ISP_GET_FUNC_DBUG 0

/* DRPAI I/F */
extern int drpai_open_k(void);
extern int drpai_close_k(void);
extern int drpai_start_k(drpai_data_t *arg, void (*isp_finish)(int result));

/* ISP operating state */
static int isp_status = ISP_STATUS_INIT;

/* ISP work memory address */
static unsigned int isp_work_adr;

/* ISP image output memory address */
static unsigned long isp_out_image_adr;

/* ISP callback function */
func_isp_callback isp_callback_p;

/* DRP Config data size */
static int isp_config_data_size;

/* Expose level */
static int isp_AE_expose_level;

/* Expose level history init value */
#define ISP_AE_HISTORY_INIT     (0xFF)
/* Expose level history */
#define ISP_AE_HISTORY          (4)
static int isp_AE_expose_history[ISP_AE_HISTORY];

/* AE ON/OFF */
static int isp_AE_mode = AE_MODE_OFF;

/* AE target luminance limit */
static unsigned char isp_AE_target;

/* AE threshold */
static int isp_AE_threshold;

/* AWB ON/OFF */
static int isp_AWB_mode = AWB_MODE_ON;

/* AWB manual */
static int isp_AWB_manual = AWB_MANUAL;

/* AWB manual gain */
static int isp_AWB_manual_rgain = ISP_PARAM_AWB_DEFAULT;
static int isp_AWB_manual_bgain = ISP_PARAM_AWB_DEFAULT;

/* AWB Gain */
static int isp_AWB_gain[] = {ISP_PARAM_AWB_DEFAULT, ISP_PARAM_AWB_DEFAULT, ISP_PARAM_AWB_DEFAULT};

/* AWB Gain Prev */
static unsigned long long isp_AWB_gain_prev[] = {1 << ISP_PARAM_AWB_SHIFT, 1 << ISP_PARAM_AWB_SHIFT, 1 << ISP_PARAM_AWB_SHIFT};


/* Waiting frame for the camera settings to be activated. */
#define AE_CAMERA_WAIT  (2)
/* AE camera frame until the settings are reflected  */
static int isp_AE_wait;

/* Frame interval */
static int isp_frame_interval;

/* DRP priority level */
static int isp_drp_priority_level;

/* Setting ISP data */
static int int_val_bl;
static int int_val_wb;
static int int_val_gamma;
static int int_val_cmx;
static int int_val_2dnr;
static int int_val_3dnr;
static int int_val_emp;
static int int_val_awb;

/* ISP CAMERA BUFFER NUM */
#define ISP_CAM_BUFFER_NUM      (4)

/* ISP SIZE LIMIT */
#define ISP_WIDTH_MAX      (2592)
#define ISP_HEIGHT_MAX     (1944)

/* ISP WORK MEMORY MAP */
#define ISP_WORK_OFFSET_3DNR        (0x0000000)
#define ISP_WORK_OFFSET_CONFIG      (0x0A00000)
#define ISP_WORK_OFFSET_AE_TBL      (0x0B00000)
#define ISP_WORK_OFFSET_PARAM       (0x0B10000)
#define ISP_WORK_OFFSET_GAMMA       (0x0B10100)
#define ISP_WORK_OFFSET_ACCUM       (0x0B10200)
#define ISP_WORK_OFFSET_DRPIF       (0x0C00000)
#define ISP_WORK_OFFSET_RBUF        (0x1000000)

#define ISP_RBUF_SIZE               (0x0800000)
#define ISP_PARAM_OFFSET_SRC        (ISP_WORK_OFFSET_PARAM + 0x0000000)
#define ISP_PARAM_OFFSET_DST        (ISP_WORK_OFFSET_PARAM + 0x0000004)
#define ISP_PARAM_OFFSET_3DNR       (ISP_WORK_OFFSET_PARAM + 0x0000010)
#define ISP_PARAM_OFFSET_RECT       (ISP_WORK_OFFSET_PARAM + 0x0000014)
#define ISP_PARAM_OFFSET_WB         (ISP_WORK_OFFSET_PARAM + 0x000001A)
#define ISP_PARAM_OFFSET_ACCAR      (ISP_WORK_OFFSET_PARAM + 0x0000020)
#define ISP_PARAM_OFFSET_CMX        (ISP_WORK_OFFSET_PARAM + 0x0000038)
#define ISP_PARAM_OFFSET_3DNRD      (ISP_WORK_OFFSET_PARAM + 0x000004A)
#define ISP_PARAM_OFFSET_BL         (ISP_WORK_OFFSET_PARAM + 0x000005A)
#define ISP_PARAM_OFFSET_GAMMA      (ISP_WORK_OFFSET_PARAM + 0x0000060)
#define ISP_PARAM_OFFSET_ACCON      (ISP_WORK_OFFSET_PARAM + 0x0000061)
#define ISP_PARAM_OFFSET_2DNR       (ISP_WORK_OFFSET_PARAM + 0x0000062)
#define ISP_PARAM_OFFSET_EMP        (ISP_WORK_OFFSET_PARAM + 0x0000064)
#define ISP_PARAM_OFFSET_FMT        (ISP_WORK_OFFSET_PARAM + 0x0000066)
#define ISP_PARAM_OFFSET_WHITE      (ISP_WORK_OFFSET_PARAM + 0x0000068)
#define ISP_PARAM_OFFSET_SAT        (ISP_WORK_OFFSET_PARAM + 0x000006A)
#define ISP_PARAM_OFFSET_RGBL       (ISP_WORK_OFFSET_PARAM + 0x000006C)
#define ISP_PARAM_OFFSET_RGBH       (ISP_WORK_OFFSET_PARAM + 0x000006E)
#define ISP_PARAM_OFFSET_RGAIN      (ISP_WORK_OFFSET_PARAM + 0x0000070)
#define ISP_PARAM_OFFSET_BGAIN      (ISP_WORK_OFFSET_PARAM + 0x0000074)
#define ISP_PARAM_OFFSET_WHITE_CLIP (ISP_WORK_OFFSET_PARAM + 0x0000076)
#define ISP_PARAM_OFFSET_SIZE       (0x78)

/* Color matrix preset data */
static unsigned short ISP_Preset_CMX[] = {
    /* Preset1: Uncorrect */
    ISP_CMX_P1_C11, ISP_CMX_P1_C12, ISP_CMX_P1_C13,
    ISP_CMX_P1_C21, ISP_CMX_P1_C22, ISP_CMX_P1_C23,
    ISP_CMX_P1_C31, ISP_CMX_P1_C32, ISP_CMX_P1_C33,
    /* Preset2: Standard correct */
    ISP_CMX_P2_C11, ISP_CMX_P2_C12, ISP_CMX_P2_C13,
    ISP_CMX_P2_C21, ISP_CMX_P2_C22, ISP_CMX_P2_C23,
    ISP_CMX_P2_C31, ISP_CMX_P2_C32, ISP_CMX_P2_C33,
    /* Preset3: Vivid correct */
    ISP_CMX_P3_C11, ISP_CMX_P3_C12, ISP_CMX_P3_C13,
    ISP_CMX_P3_C21, ISP_CMX_P3_C22, ISP_CMX_P3_C23,
    ISP_CMX_P3_C31, ISP_CMX_P3_C32, ISP_CMX_P3_C33,
    /* Preset4: Sepia correct */
    ISP_CMX_P4_C11, ISP_CMX_P4_C12, ISP_CMX_P4_C13,
    ISP_CMX_P4_C21, ISP_CMX_P4_C22, ISP_CMX_P4_C23,
    ISP_CMX_P4_C31, ISP_CMX_P4_C32, ISP_CMX_P4_C33
};

/* White balance preset data */
static unsigned short ISP_Preset_WB[] = {
    /* Preset1: Day light */
    ISP_WB_P1_BLUE, ISP_WB_P1_GREEN,    ISP_WB_P1_RED,
    /* Preset2: Neutral light */
    ISP_WB_P2_BLUE, ISP_WB_P2_GREEN,    ISP_WB_P2_RED,
    /* Preset3: Interior light */
    ISP_WB_P3_BLUE, ISP_WB_P3_GREEN,    ISP_WB_P3_RED,
    /* Preset4: Lamp light */
    ISP_WB_P4_BLUE, ISP_WB_P4_GREEN,    ISP_WB_P4_RED
};
static int wb_red,wb_green,wb_blue;

/* Function prototype */
static void ISP_callback( int result);

/* ISP Alldata size */
#define ISP_SET_ALLDATA_SIZE_V10    (429)
#define ISP_SET_ALLDATA_SIZE_V11    (512)

#define ISP_SET_ALLDATA_BIT_OUTFORMAT   (0x0001)    /* bit0 */
#define ISP_SET_ALLDATA_BIT_ACCUMLATE   (0x0002)    /* bit1 */
#define ISP_SET_ALLDATA_CMX_PRESET      (0x0004)    /* bit2 */
#define ISP_SET_ALLDATA_WB_PRESET       (0x0008)    /* bit3 */
#define ISP_SET_ALLDATA_CMX_PRESET_SEL  (0x0010)    /* bit4 */
#define ISP_SET_ALLDATA_WB_PRESET_SEL   (0x0020)    /* bit5 */
#define ISP_SET_ALLDATA_IMGSIZE         (0x0040)    /* bit6 */
#define ISP_SET_ALLDATA_3DNR            (0x0080)    /* bit7 */
#define ISP_SET_ALLDATA_BL              (0x0100)    /* bit8 */
#define ISP_SET_ALLDATA_2DNR            (0x0200)    /* bit9 */
#define ISP_SET_ALLDATA_EMP             (0x0400)    /* bit10 */
/* V1.0 */
#define ISP_SET_ALLDATA_GAMMA10         (0x0800)    /* bit11 */
/* V1.1 */
#define ISP_SET_ALLDATA_AE              (0x0800)    /* bit11 */
#define ISP_SET_ALLDATA_AWB             (0x1000)    /* bit12 */
#define ISP_SET_ALLDATA_GAMMA11         (0x8000)    /* bit15 */

#define ISP_SET_ALLDATA_POS_SIZE_L  (0x00)
#define ISP_SET_ALLDATA_POS_SIZE_H  (0x01)
#define ISP_SET_ALLDATA_POS_MAP_L   (0x02)
#define ISP_SET_ALLDATA_POS_MAP_H   (0x03)

#define ISP_SET_ALLDATA_POS_SIG1    (0x18)
#define ISP_SET_ALLDATA_POS_SIG2    (0x19)
#define ISP_SET_ALLDATA_POS_SIG3    (0x1A)
#define ISP_SET_ALLDATA_POS_SIG4    (0x1B)
#define ISP_SET_ALLDATA_POS_SIG5    (0x1C)
#define ISP_SET_ALLDATA_POS_SIG6    (0x1D)
#define ISP_SET_ALLDATA_POS_SIG7    (0x1E)
#define ISP_SET_ALLDATA_POS_SIG8    (0x1F)

#define ISP_SET_ALLDATA_POS_OUTFMT  (0x20)

#define ISP_SET_ALLDATA_POS_ACCON   (0x21)

#define ISP_SET_ALLDATA_POS_AREA    (0x22)

#define ISP_SET_ALLDATA_POS_AREAXL  (0x22)
#define ISP_SET_ALLDATA_POS_AREAXH  (0x23)
#define ISP_SET_ALLDATA_POS_AREAYL  (0x24)
#define ISP_SET_ALLDATA_POS_AREAYH  (0x25)
#define ISP_SET_ALLDATA_POS_AREAWL  (0x26)
#define ISP_SET_ALLDATA_POS_AREAWH  (0x27)
#define ISP_SET_ALLDATA_POS_AREAHL  (0x28)
#define ISP_SET_ALLDATA_POS_AREAHH  (0x29)

#define ISP_SET_ALLDATA_POS_CMX_P   (0x2A)
#define ISP_SET_ALLDATA_POS_WB_P    (0x72)
#define ISP_SET_ALLDATA_POS_CMX_SEL (0x8A)
#define ISP_SET_ALLDATA_POS_WB_SEL  (0x8B)

#define ISP_SET_ALLDATA_POS_IMGSIZE (0x8C)
#define ISP_SET_ALLDATA_POS_3DNR_D  (0x92)
#define ISP_SET_ALLDATA_POS_BIAS    (0xA2)
#define ISP_SET_ALLDATA_POS_2DNR    (0xA8)
#define ISP_SET_ALLDATA_POS_EMP     (0xAA)
/* V1.0 */
#define ISP_SET_ALLDATA_POS_GAMMAON10 (0xAC)
#define ISP_SET_ALLDATA_POS_GAMMAD10  (0xAD)
/* V1.1 */
#define ISP_SET_ALLDATA_POS_AE_ONOFF        (0xAC)
#define ISP_SET_ALLDATA_POS_AE_TARGET       (0xAD)
#define ISP_SET_ALLDATA_POS_AE_EXPOSE_L     (0xAE)
#define ISP_SET_ALLDATA_POS_AE_EXPOSE_H     (0xAF)
#define ISP_SET_ALLDATA_POS_AE_THRESHOLD    (0xB0)

#define ISP_SET_ALLDATA_POS_AWB_MANUAL      (0xB1)
#define ISP_SET_ALLDATA_POS_AWB_MANUAL_R_L  (0xB2)
#define ISP_SET_ALLDATA_POS_AWB_MANUAL_R_H  (0xB3)
#define ISP_SET_ALLDATA_POS_AWB_MANUAL_B_L  (0xB4)
#define ISP_SET_ALLDATA_POS_AWB_MANUAL_B_H  (0xB5)

#define ISP_SET_ALLDATA_POS_GAMMAON11 (0xFF)
#define ISP_SET_ALLDATA_POS_GAMMAD11  (0x100)

/* Gamma table limiter */
#define GAMMA_MIN           (1)
#define GAMMA_DEFAULT       (100)
#define GAMMA_MAX           (9999)
#define GAMMA_TBL1_LIMIT    (100)
#define GAMMA_TBL2_LIMIT    (1000)
#define GAMMA_TBL3_LIMIT    (9999)

/* Emphasis param */
#define PARAM_EMP_OFF       (0)
#define PARAM_EMP_WEAK      (30)
#define PARAM_EMP_NORMAL    (60)
#define PARAM_EMP_STRONG    (90)
#define PARAM_CORING        (28)


/* ISP Accumlate */
#define ISP_ACCUMLATE_OFF   (0)
#define ISP_ACCUMLATE_ON    (1)
/* ISP Accumlate mode */
static unsigned char isp_accumlate_mode;

/* ISP Accumlate area */
static unsigned short isp_accumlatearea_x;
static unsigned short isp_accumlatearea_y;
static unsigned short isp_accumlatearea_w;
static unsigned short isp_accumlatearea_h;

/* Camera Input format */
#define AE_THRESHOLD_RATIO  6
static unsigned short isp_in_format;
#define AE_THRESHOLD_HIGH   (3)
#define AE_THRESHOLD_MID    (2)

/* Expose table data */
static int AE_tbl[801][2];

/* RZV ISP Version Information */
static unsigned char ver_info;

/* ALL Setting data */

/* Ver1.0 */
static unsigned char all_v10_tbl[ISP_SET_ALLDATA_SIZE_V10];
static unsigned char all_v10_tbl_data[ISP_SET_ALLDATA_SIZE_V10] =
{
                                                                                                                /* [Header area]                                */
   0xAD, 0x01,                                                                                                  /* [0000 - 0001] total size                     */
   0xFF, 0x0F,                                                                                                  /* [0002 - 0003] renew data (bit sum)           */
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                      /* [0004 - 000F] reserved area                  */
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                                              /* [0010 - 0017] reserved area                  */
   0x52, 0x5A, 0x56, 0x5F, 0x49, 0x53, 0x50, 0x30,                                                              /* [0018 - 001F] signature                      */
                                                                                                                /* [ISP data area]                              */
   0x00,                                                                                                        /* [0020       ] (bit00) output format          */
   0x01,                                                                                                        /* [0021       ] (bit01) accumulate ON/OFF      */
   0x40, 0x01, 0xB4, 0x00, 0x80, 0x02, 0x68, 0x01,                                                              /* [0022 - 0029] (bit01) accumulate area        */
   0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,  /* [002A - 003B] (bit02) CMX Preset0 data       */
   0x63, 0x2F, 0x8A, 0xFA, 0x9B, 0xFD, 0xBD, 0xF4, 0x7D, 0x2B, 0x95, 0x0A, 0xD5, 0xFE, 0x8E, 0xDD, 0xA3, 0x4A,  /* [003C - 004D] (bit02) CMX Preset1 data       */
   0x05, 0x3D, 0xF7, 0xF9, 0x72, 0xF3, 0x7A, 0xF1, 0x92, 0x39, 0xA1, 0x00, 0xDF, 0xF7, 0xC0, 0xD5, 0x06, 0x5A,  /* [004E - 005F] (bit02) CMX Preset2 data       */
   0x6D, 0x07, 0xFF, 0x27, 0xEF, 0x02, 0x80, 0x05, 0xDC, 0x24, 0x71, 0x01, 0x4F, 0x05, 0x6F, 0x16, 0x02, 0x0A,  /* [0060 - 0071] (bit02) CMX Preset3 data       */
   0xE3, 0x1B, 0x5B, 0x13, 0x91, 0x16,                                                                          /* [0072 - 0077] (bit03) WB Preset0 data        */
   0x5A, 0x19, 0x5B, 0x13, 0xD3, 0x18,                                                                          /* [0078 - 007D] (bit03) WB Preset1 data        */
   0xD1, 0x16, 0x5B, 0x13, 0x95, 0x1B,                                                                          /* [007E - 0083] (bit03) WB Preset2 data        */
   0x48, 0x14, 0x5B, 0x13, 0x07, 0x1F,                                                                          /* [0084 - 0089] (bit03) WB Preset3 data        */
   0x01,                                                                                                        /* [008A       ] (bit04) CMX Preset select      */
   0x01,                                                                                                        /* [008B       ] (bit05) WB Preset select       */
   0x00, 0x05, 0xD0, 0x02, 0xB0, 0x06,                                                                          /* [008C - 0091] (bit06) image size,stride      */
   0x40, 0x20, 0x80, 0x08, 0x00, 0x10, 0x00, 0x00, 0x02, 0x80, 0x08, 0x00, 0x10, 0x00, 0x00, 0x02,              /* [0092 - 00A1] (bit07) 3DNR                   */
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                                                          /* [00A2 - 00A7] (bit08) R/G/B Bias             */
   0x00, 0x01,                                                                                                  /* [00A8 - 00A9] (bit09) 2DNR(median) blend     */
   0x00,                                                                                                        /* [00AA       ] (bit10) Unsharp mask strength  */
   0x1C,                                                                                                        /* [00AB       ] (bit10) Unsharp mask coring    */
   0x00,                                                                                                        /* [00AC       ] (bit11) gamma ON/OFF           */
   0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,              /* [00AD - 01AC] (bit11) gamma table            */
   0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
   0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
   0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
   0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
   0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
   0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
   0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
   0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
   0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
   0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
   0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
   0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
   0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
   0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
   0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF
};
/* Ver1.1 */
static unsigned char all_v11_tbl[ISP_SET_ALLDATA_SIZE_V11];
static unsigned char all_v11_tbl_data[ISP_SET_ALLDATA_SIZE_V11] =
{
                                                                                                                      /* [Header area]                                      */
    0x00, 0x02,                                                                                                       /* [0000 - 0001] total size                           */
    0xFF, 0x9F,                                                                                                       /* [0002 - 0003] renew data (bit sum)                 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                           /* [0004 - 000F] reserved area                        */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                                                   /* [0010 - 0017] reserved area                        */
    0x52, 0x5A, 0x56, 0x5F, 0x49, 0x53, 0x50, 0x31,                                                                   /* [0018 - 001F] signature                            */
                                                                                                                      /* [ISP data area]                                    */
    0x00,                                                                                                             /* [0020       ] (bit00) output format                */
    0x01,                                                                                                             /* [0021       ] (bit01) accumulate ON/OFF            */
    0x40, 0x01, 0xB4, 0x00, 0x80, 0x02, 0x68, 0x01,                                                                   /* [0022 - 0029] (bit01) accumulate area              */
    0x25, 0x21, 0xe2, 0x02, 0xc8, 0xfb, 0xfd, 0xff, 0xf6, 0x1d, 0x91, 0x02, 0xc5, 0xfe, 0x4b, 0xf7, 0x9b, 0x28,       /* [003C - 004D] (bit02) CMX Preset0 data             */
    0x69, 0x2c, 0x8e, 0xf7, 0x2c, 0xfb, 0xdd, 0xfd, 0xc9, 0x22, 0x63, 0x01, 0xf0, 0xfd, 0x8a, 0xf6, 0x82, 0x2c,       /* [004E - 005F] (bit02) CMX Preset1 data             */
    0x21, 0x25, 0x89, 0xfa, 0xdd, 0xfc, 0xe1, 0xfb, 0xad, 0x27, 0x26, 0xff, 0xf1, 0x00, 0x72, 0xf1, 0x25, 0x2c,       /* [002A - 003B] (bit02) CMX Preset2 data             */
    0x20, 0x0a, 0x5a, 0x19, 0x09, 0x05, 0x48, 0x08, 0x51, 0x1f, 0x2f, 0xfd, 0xd7, 0x07, 0x92, 0x10, 0x31, 0x07,       /* [0060 - 0071] (bit02) CMX Preset3 data             */
    0x16, 0x1a, 0x41, 0x10, 0x7a, 0x16,                                                                               /* [0072 - 0077] (bit03) WB Preset0 data              */
    0x56, 0x16, 0x41, 0x10, 0xd8, 0x18,                                                                               /* [0078 - 007D] (bit03) WB Preset1 data              */
    0x97, 0x12, 0x41, 0x10, 0x3d, 0x1b,                                                                               /* [007E - 0083] (bit03) WB Preset2 data              */
    0xdd, 0x10, 0x41, 0x10, 0xe4, 0x21,                                                                               /* [0084 - 0089] (bit03) WB Preset3 data              */
    0x01,                                                                                                             /* [008A       ] (bit04) CMX Preset select            */
    0x01,                                                                                                             /* [008B       ] (bit05) WB Preset select             */
    0x00, 0x05, 0xD0, 0x02, 0xB0, 0x06,                                                                               /* [008C - 0091] (bit06) image size,stride            */
    0x40, 0x20, 0x80, 0x08, 0x00, 0x10, 0x00, 0x00, 0x02, 0x80, 0x08, 0x00, 0x10, 0x00, 0x00, 0x02,                   /* [0092 - 00A1] (bit07) 3DNR                               */
    0xF0, 0xFF, 0xF0, 0xFF, 0xF0, 0xFF,                                                                               /* [00A2 - 00A7] (bit08) R/G/B Bias                   */
    0x40, 0x00,                                                                                                       /* [00A8 - 00A9] (bit09) 2DNR(median) blend           */
    0x00,                                                                                                             /* [00AA       ] (bit10) Unsharp mask strength        */
    0x1C,                                                                                                             /* [00AB       ] (bit10) Unsharp mask coring          */
    0x00,                                                                                                             /* [00AC       ] (bit11) AE ON/OFF                    */
    0x76,                                                                                                             /* [00AD       ] (bit11) AE target brightness level   */
    0x64, 0x00,                                                                                                       /* [00AE - 00AF] (bit11) AE default expose level      */
    0x0A,                                                                                                             /* [00B0       ] (bit11) AE threshold                 */
    0x00,                                                                                                             /* [00B1       ] (bit12) AWB On(auto)/Off(Manual)     */
    0x00, 0x01, 0x00, 0x01,                                                                                           /* [00B2 - 00B5] (bit12) AWB Gain R/B                 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       /* [00B6 - 00FE] reserved area                        */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,
    0x00,                                                                                                             /* [00FF       ] (bit15) gamma ON/OFF                 */
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,                   /* [0100 - 01FF] (bit15) gamma table                  */
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
    0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
    0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
    0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
    0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
    0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF
};
#if ISP_DEBUG
static void pmem_print(unsigned int src_adr, int print_size)
{
    char *read_p;
    int cnt;
    char c0,c1,c2,c3,c4,c5,c6,c7;
    read_p = (char *)phys_to_virt(src_adr);
    for( cnt = 0; cnt < print_size; cnt+=8 )
    {
        c0 = *read_p++;
        c1 = *read_p++;
        c2 = *read_p++;
        c3 = *read_p++;
        c4 = *read_p++;
        c5 = *read_p++;
        c6 = *read_p++;
        c7 = *read_p++;
        printk( "MEM %x:%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",src_adr+cnt,c0,c1,c2,c3,c4,c5,c6,c7 );
    }
}
#endif

/* Copy from virtual address to physical address */
static void pmem_write(unsigned int dest_adr, void *src_pointer, int copy_size)
{
    void *write_p;
    
    write_p = phys_to_virt(dest_adr);
    if( write_p != 0 )
    {
        memcpy(write_p, src_pointer, copy_size);
        __flush_dcache_area(write_p, copy_size);
    }
}

/* Memory fill */
static void pmem_set(unsigned int dest_adr, unsigned int set_data, size_t set_size)
{
    void *write_p;
    int cnt;
    
    write_p = phys_to_virt(dest_adr);
    if( write_p != 0 )
    {
        for( cnt = 0; cnt < set_size/sizeof(unsigned int); cnt++ )
        {
            ((unsigned int *)write_p)[cnt] = set_data;
        }
        __flush_dcache_area(write_p, set_size);
    }
}

/* Get gamma table */
static unsigned char *get_gamma_tbl(unsigned short gamma)
{
    unsigned char *out_data;
    if( ( gamma == 0 ) || ( gamma > GAMMA_MAX ) )
    {
        gamma = GAMMA_DEFAULT;    //default value
    }
    
    if( gamma < GAMMA_TBL1_LIMIT )
    {
        out_data = &gamma_tbl_0[gamma - 1][0];
    }
    else if( gamma < GAMMA_TBL2_LIMIT )
    {
        out_data = &gamma_tbl_1[(gamma - GAMMA_TBL1_LIMIT)/10][0];
    }
    else
    {
        out_data = &gamma_tbl_2[(gamma - GAMMA_TBL2_LIMIT)/100][0];
    }
    return out_data;
}

/* Create AE table */
static void set_ae_table( int frame_interval )
{
    int expose_cnt;
    int limit_time = 1;
    int extra_gain = 0;

    for( expose_cnt = ISP_LIMIT_EXPOSE_LV_MAX - ISP_LIMIT_EXPOSE_LV_MIN; expose_cnt >= 0 ; expose_cnt-- )
    {
        if( AE_tbl_default[expose_cnt][1] > (frame_interval * 10) )
        {
            extra_gain++;
        }
        else
        {
            limit_time = AE_tbl_default[expose_cnt][1];
        }
        
        AE_tbl[expose_cnt][0] = AE_tbl_default[expose_cnt][0] + extra_gain;
        AE_tbl[expose_cnt][1] = limit_time;
    }
}

/* ISP Open(from V4L2) */
int ISP_open(unsigned short width, unsigned short height, unsigned short stride,
             unsigned short frame_interval, unsigned short in_format, 
             unsigned short out_format, unsigned char *cam_param_p)
{
    int ret_data = 0;
    int ret;
    int drp_opend_f = 0;
    int cnt_i;
    unsigned char *config_data_p;
    unsigned char isp_param[0x100];
    struct device_node *np;
    struct resource reserved_res;
    unsigned char o_format;

    
    /* Check the operating state of ISP (="INIT") */
    if( isp_status != ISP_STATUS_INIT )
    {
        ret_data = (-EACCES);
        goto end;
    }

    /* Check arguments  */
    if( ( (width & 1) != 0 ) || ( (height & 1) != 0) || ( (stride & 1) != 0) ||                                 // Image size is odd
        ( width == 0 ) || (height == 0) || (stride == 0) ||                                                      // Image size is zero
        ( width > ISP_WIDTH_MAX ) || ( height > ISP_HEIGHT_MAX ) ||                                               // Image size over
        ( (in_format != IN_FORMAT_RAW8) && (in_format != IN_FORMAT_RAW10) && (in_format != IN_FORMAT_RAW12) && (in_format != IN_FORMAT_DEFAULT) ) ||  // Input image format error
        ( (out_format != OUT_FORMAT_UYVY) && (out_format != OUT_FORMAT_YUY2) && (out_format != OUT_FORMAT_RGB888) &&
          (out_format != OUT_FORMAT_ARGB8888) && (out_format != OUT_FORMAT_XRGB8888) ) )                        // Output image format error
    {
        ret_data = (-EINVAL);
        goto end;
    }

    /* Get work memory address */
    np = of_find_node_by_path( "/" );  // Root node
    np = of_get_child_by_name( np, "reserved-memory");
    np = of_get_child_by_name( np, "SIMPLE_ISP");
    ret = of_address_to_resource(np, 0, &reserved_res);
    if( ret )
    {
        ret_data = (-EACCES);
        goto end;
    }
    isp_work_adr = (reserved_res.start + 0x3F) & 0xFFFFFFC0;  // ISP work memory address(64byte align)

    /* Open DRP */
    ret = drpai_open_k();
    if( ret )
    {
        ret_data = ret;
        goto end;
    }
    drp_opend_f = 1;

    /* Set input format */
    if( in_format == IN_FORMAT_DEFAULT )
    {
        isp_in_format = IN_FORMAT_RAW10;
    }
    else
    {
        isp_in_format = in_format;
    }

    /* Copy DRP config data */
    if( ( out_format == OUT_FORMAT_UYVY ) || ( out_format == OUT_FORMAT_YUY2 ) )
    {
        if( isp_in_format == IN_FORMAT_RAW8 ){
            config_data_p = &ISP_Config_RAW8_YUV[0];
            isp_config_data_size = ISP_Config_RAW8_YUV_len;
        }
        else if( isp_in_format == IN_FORMAT_RAW10 ){
            config_data_p = &ISP_Config_RAW10_YUV[0];
            isp_config_data_size = ISP_Config_RAW10_YUV_len;
        }
        else{
            config_data_p = &ISP_Config_RAW12_YUV[0];
            isp_config_data_size = ISP_Config_RAW12_YUV_len;
        }
    }
    else
    {
        if( isp_in_format == IN_FORMAT_RAW8 ){
            config_data_p = &ISP_Config_RAW8_RGB[0];
            isp_config_data_size = ISP_Config_RAW8_RGB_len;
        }
        else if( isp_in_format == IN_FORMAT_RAW10 ){
            config_data_p = &ISP_Config_RAW10_RGB[0];
            isp_config_data_size = ISP_Config_RAW10_RGB_len;
        }
        else{
            config_data_p = &ISP_Config_RAW12_RGB[0];
            isp_config_data_size = ISP_Config_RAW12_RGB_len;
        }
    }
    pmem_write( isp_work_adr + ISP_WORK_OFFSET_CONFIG, config_data_p, isp_config_data_size);

    /* Initialize variables */
    isp_AE_expose_level = ISP_LIMIT_EXPOSE_LV_DEFAULT;  // Expose level = 0dB
    isp_AE_target = ISP_LIMIT_T_BL_DEFAULT;         // AE target luminance (= 128/255)
    isp_AE_threshold = ISP_LIMIT_THRESHOLD_DEFAULT;     // AE threshold = 10
    isp_AE_mode = AE_MODE_OFF;                           // AE MODE OFF
	isp_AWB_mode = AWB_MODE_ON;                         // AWB MODE ON
	isp_AWB_manual = AWB_MANUAL;                        // AWB MANUAL
    isp_AE_wait = AE_CAMERA_WAIT;                       // Camera setting wait frames = 2
    for( cnt_i = 0; cnt_i < ISP_AE_HISTORY; cnt_i++ )
    {
        isp_AE_expose_history[cnt_i] = ISP_AE_HISTORY_INIT-cnt_i; // Expose level history clear
    }
    if( frame_interval == 0 )
    {
        isp_frame_interval = ISP_LIMIT_FRM_INTERVAL_DEFAULT;    //default value = 33ms
    }
    else
    {
        isp_frame_interval = frame_interval;
    }
    isp_drp_priority_level = 0;
    isp_accumlate_mode = ISP_ACCUMLATE_ON;
    isp_accumlatearea_x = width/4;
    isp_accumlatearea_y = height/4;
    isp_accumlatearea_w = width/2;
    isp_accumlatearea_h = height/2;
    wb_red = ISP_Preset_WB[ISP_LIMIT_WB_DEFAULT*3];
    wb_green =  ISP_Preset_WB[ISP_LIMIT_WB_DEFAULT*3+1];
    wb_blue =  ISP_Preset_WB[ISP_LIMIT_WB_DEFAULT*3+2];

    /* Creat AE table */
    set_ae_table( isp_frame_interval );

    /* Clear the last image area */
    pmem_set( isp_work_adr + ISP_WORK_OFFSET_3DNR, 0x80008000 , 0x960000 );    // 3DNR last image format is always "UYVY"

    /* Set default parameter for SimpleISP */
    *(unsigned int*)(&isp_param[0x08])   = isp_work_adr + ISP_WORK_OFFSET_ACCUM;    // Accumlate data area
    *(unsigned int*)(&isp_param[0x0C])   = isp_work_adr + ISP_WORK_OFFSET_GAMMA;    // Gamma table area
    if( ISP_LIMIT_3DNR_DEFAULT == 0 )
    {
        *(unsigned int*)(&isp_param[0x10])   = 0;                                   // 3DNR OFF
    }
    else
    {
        *(unsigned int*)(&isp_param[0x10])   = isp_work_adr + ISP_WORK_OFFSET_3DNR; // Work area for 3DNR 
    }
    *(unsigned short*)(&isp_param[0x14]) = width;
    *(unsigned short*)(&isp_param[0x16]) = height;
    *(unsigned short*)(&isp_param[0x18]) = stride;
    memcpy( &isp_param[0x1A], &ISP_Preset_WB[ISP_LIMIT_WB_DEFAULT*3], 6);           // Gain R/G/B area size (2byte * 3)
    *(unsigned short*)(&isp_param[0x20])   = isp_accumlatearea_x;                   // Accumlate image size
    *(unsigned short*)(&isp_param[0x22])   = isp_accumlatearea_y;
    *(unsigned short*)(&isp_param[0x24])   = isp_accumlatearea_w;
    *(unsigned short*)(&isp_param[0x26])   = isp_accumlatearea_h;
    *(unsigned int*)(&isp_param[0x28])   = 0;                                       // Reserve
    *(unsigned int*)(&isp_param[0x2C])   = 0;
    *(unsigned int*)(&isp_param[0x30])   = 0;
    *(unsigned int*)(&isp_param[0x34])   = 0;
    memcpy( &isp_param[0x38], &ISP_Preset_CMX[ISP_LIMIT_CMX_DEFAULT*9], 18);        // Color matrix area size (2byte * 9)
    *(unsigned short*)(&isp_param[0x4A])   = ISP_PARAM_3DNR_Y_THRES_A;              // 3DNR parameter (Fixed data)
    *(unsigned short*)(&isp_param[0x4C])   = ISP_PARAM_3DNR_Y_THRES_B;
    *(unsigned short*)(&isp_param[0x4E])   = ISP_PARAM_3DNR_Y_TILT;
    *(unsigned short*)(&isp_param[0x50])   = ISP_PARAM_3DNR_C_THRES_A;
    *(unsigned short*)(&isp_param[0x52])   = ISP_PARAM_3DNR_C_THRES_B;
    *(unsigned short*)(&isp_param[0x54])   = ISP_PARAM_3DNR_C_TILT;
    *(unsigned char*)(&isp_param[0x56])   = ISP_PARAM_3DNR_Y_COEF;
    *(unsigned char*)(&isp_param[0x57])   = ISP_PARAM_3DNR_C_COEF;
    *(unsigned char*)(&isp_param[0x58])   = ISP_PARAM_3DNR_Y_ALPHA_MAX;
    *(unsigned char*)(&isp_param[0x59])   = ISP_PARAM_3DNR_C_ALPHA_MAX;
    *(short*)(&isp_param[0x5A])   = -ISP_LIMIT_BL_R_DEFAULT;                                 // Black level
    *(short*)(&isp_param[0x5C])   = -ISP_LIMIT_BL_G_DEFAULT;
    *(short*)(&isp_param[0x5E])   = -ISP_LIMIT_BL_B_DEFAULT;
    if( ISP_LIMIT_GAMMA_DEFAULT == 100 )
    {
        *(unsigned char*)(&isp_param[0x60])   = 0;                                  // Gamma OFF(gamma value = 1.00)
    }
    else
    {
        *(unsigned char*)(&isp_param[0x60])   = 1;                                  // Gamma ON
    }
    *(unsigned char*)(&isp_param[0x61])   = ISP_ACCUMLATE_ON;                       // Accumlate ON
    *(unsigned short*)(&isp_param[0x62])  = (unsigned short)((ISP_LIMIT_2DNR_DEFAULT * 256uL + 50) / 100); // 2DNR
    *(unsigned char*)(&isp_param[0x64])   = ISP_LIMIT_EMP_DEFAULT;                  // Emphasis OFF
    *(unsigned char*)(&isp_param[0x65])   = PARAM_CORING;                           // Coring ON
    *(unsigned char*)(&isp_param[0x66])   = out_format;                             // Output format
    *(unsigned char*)(&isp_param[0x67])   = 0;                                      // Padding
	
	/* add AWB param */
	*(unsigned short*)(&isp_param[0x68])   = ISP_PARAM_AWB_THRESH_WHITE;            //threshWhite
	*(unsigned short*)(&isp_param[0x6A])   = ISP_PARAM_AWB_THRESH_SAT;              //threshSaturation
	*(unsigned short*)(&isp_param[0x6C])   = ISP_PARAM_AWB_THRESH_RGBL;             //threshRGBL
	*(unsigned short*)(&isp_param[0x6E])   = ISP_PARAM_AWB_THRESH_RGBH;             //threshRGBH
	*(unsigned short*)(&isp_param[0x70])   = isp_AWB_gain[AWB_COLOR_R];             //awbGainR
	*(unsigned short*)(&isp_param[0x74])   = isp_AWB_gain[AWB_COLOR_B];             //awbGainB
	*(unsigned short*)(&isp_param[0x76])   = ISP_PARAM_AWB_WHITE_CLIP;              //whiteclip
	
	
    pmem_write( isp_work_adr + ISP_WORK_OFFSET_PARAM, &isp_param[0], ISP_PARAM_OFFSET_SIZE);
#if ISP_DEBUG
    printk( "[ISP DEFAULT PARAM]\n");
    pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif
    /* Calculate the value of the default gamma table */
    pmem_write( isp_work_adr + ISP_WORK_OFFSET_GAMMA, get_gamma_tbl(ISP_LIMIT_GAMMA_DEFAULT), 0x100);
#if ISP_DEBUG
    printk( "[GAMMA TBL]\n");
    pmem_print( isp_work_adr + ISP_WORK_OFFSET_GAMMA, 0x100);
#endif

    /* Set ISP operating state to "READY" */
    isp_status = ISP_STATUS_READY;      // ISP operating ready
    
    /* Set ISP status */
    ver_info        = 1; /* Version 1.1 */
    int_val_bl      = ISP_LIMIT_BL_DEFAULT;
    int_val_wb      = ISP_LIMIT_WB_DEFAULT;
    int_val_gamma   = ISP_LIMIT_GAMMA_DEFAULT;
    int_val_cmx     = ISP_LIMIT_CMX_DEFAULT;
    int_val_2dnr    = ISP_LIMIT_2DNR_DEFAULT;
    int_val_3dnr    = ISP_LIMIT_3DNR_DEFAULT;
    int_val_emp     = ISP_LIMIT_EMP_DEFAULT;
    int_val_awb     = ISP_LIMIT_AWB_DEFAULT;
    
    /* Set Table data */
    memcpy(&all_v10_tbl[0] , &all_v10_tbl_data[0], sizeof(all_v10_tbl));
    memcpy(&all_v11_tbl[0] , &all_v11_tbl_data[0], sizeof(all_v11_tbl));
end:
    /* Close DRP, if error occurred and DRP opened */
    if( ret_data != 0 )
    {
        if( drp_opend_f == 1 )
        {
            drpai_close_k();
        }
    }

    return ret_data;
}

/* ISP Close(from V4L2) */
int ISP_close(void)
{
    int ret_data = 0;
    int ret;

    /* Check the operating state of ISP (!="INIT") */
    if( isp_status == ISP_STATUS_INIT )
    {
        ret_data = (-EACCES);
        goto end;
    }

    /* Close DRP */
    ret = drpai_close_k();
    if( ret )
    {
        ret_data = ret;
        goto end;
    }

    /* Set ISP operating state to "INIT" */
    isp_status = ISP_STATUS_INIT;

end:
    return ret_data;
}

/* ISP Set Param(from V4L2) */
int ISP_set(int isp_func, void *value)
{
    int ret_data = 0;
    int int_val;
    unsigned char *all_data;
    unsigned short all_set_bit;
    unsigned short all_data_size;
    unsigned char isp_param[0x100];

    /* Check the operating state of ISP (="READY") */
    if( isp_status != ISP_STATUS_READY )
    {
        ret_data = (-EACCES);
        goto end;
    }

    switch( isp_func )
    {
        case ISP_FUNC_ALL:
            /* Set All parameter */
            all_data = (unsigned char *)value;
            all_set_bit = (all_data[ISP_SET_ALLDATA_POS_MAP_H] << 8)| all_data[ISP_SET_ALLDATA_POS_MAP_L];
            all_data_size = (all_data[ISP_SET_ALLDATA_POS_SIZE_H] << 8)| all_data[ISP_SET_ALLDATA_POS_SIZE_L];
            #if ISP_GET_FUNC_DBUG
            printk("all_set_bit%x\n",all_set_bit);
            #endif /* ISP_GET_FUNC_DBUG */
            /* Check datasize and signature */
            /* Ver1.0 */
            if( ( all_data_size == ISP_SET_ALLDATA_SIZE_V10 ) &&
                ( (all_set_bit & 0xF000) == 0 ) &&
                ( all_data[ISP_SET_ALLDATA_POS_SIG1] == 'R' ) && ( all_data[ISP_SET_ALLDATA_POS_SIG2] == 'Z' ) &&
                ( all_data[ISP_SET_ALLDATA_POS_SIG3] == 'V' ) && ( all_data[ISP_SET_ALLDATA_POS_SIG4] == '_' ) &&
                ( all_data[ISP_SET_ALLDATA_POS_SIG5] == 'I' ) && ( all_data[ISP_SET_ALLDATA_POS_SIG6] == 'S' ) &&
                ( all_data[ISP_SET_ALLDATA_POS_SIG7] == 'P' ) && ( all_data[ISP_SET_ALLDATA_POS_SIG8] == '0' ) )
            {
                ver_info = 0;
                #if ISP_GET_FUNC_DBUG
                printk("all_set_bit%x\n",all_set_bit);
                printk("Verinfo%x\n",ver_info);
                printk("ISP_set Ver1.0%x\n",all_v10_tbl[0]);
                #endif /* ISP_GET_FUNC_DBUG */
            }
            /* Ver1.1 */
            else if( ( all_data_size == ISP_SET_ALLDATA_SIZE_V11 ) &&
                ( all_data[ISP_SET_ALLDATA_POS_SIG1] == 'R' ) && ( all_data[ISP_SET_ALLDATA_POS_SIG2] == 'Z' ) &&
                ( all_data[ISP_SET_ALLDATA_POS_SIG3] == 'V' ) && ( all_data[ISP_SET_ALLDATA_POS_SIG4] == '_' ) &&
                ( all_data[ISP_SET_ALLDATA_POS_SIG5] == 'I' ) && ( all_data[ISP_SET_ALLDATA_POS_SIG6] == 'S' ) &&
                ( all_data[ISP_SET_ALLDATA_POS_SIG7] == 'P' ) && ( all_data[ISP_SET_ALLDATA_POS_SIG8] == '1' ) )
            {
                ver_info = 1;
            }
            else
            {
                ret_data = (-EINVAL);
                #if ISP_GET_FUNC_DBUG
                printk( "ISP:Version Error\n");
                printk( "SIZEL%x\n",all_data[0]);
                printk( "SIZEH%x\n",all_data[1]);
                printk( "MAPL%x\n",all_data[2]);
                printk( "MAPH%x\n",all_data[3]);

                printk( "SIG1%x\n",all_data[ISP_SET_ALLDATA_POS_SIG1]);
                printk( "SIG2%x\n",all_data[ISP_SET_ALLDATA_POS_SIG2]);
                printk( "SIG3%x\n",all_data[ISP_SET_ALLDATA_POS_SIG3]);
                printk( "SIG4%x\n",all_data[ISP_SET_ALLDATA_POS_SIG4]);
                printk( "SIG5%x\n",all_data[ISP_SET_ALLDATA_POS_SIG5]);
                printk( "SIG6%x\n",all_data[ISP_SET_ALLDATA_POS_SIG6]);
                printk( "SIG7%x\n",all_data[ISP_SET_ALLDATA_POS_SIG7]);
                printk( "SIG8%x\n",all_data[ISP_SET_ALLDATA_POS_SIG8]);
                #endif /* ISP_GET_FUNC_DBUG */
                goto end;
            }

            /* Check invalid param */
            if( (all_set_bit & ISP_SET_ALLDATA_BIT_ACCUMLATE) != 0 ){
                if( (all_data[ISP_SET_ALLDATA_POS_AREAWH] == 0) && (all_data[ISP_SET_ALLDATA_POS_AREAWL] == 0) )
                {
                    ret_data = (-EINVAL);
                    goto end;
                }
                if( (all_data[ISP_SET_ALLDATA_POS_AREAHH] == 0) && (all_data[ISP_SET_ALLDATA_POS_AREAHL] == 0) )
                {
                    ret_data = (-EINVAL);
                    goto end;
                }
            }
            
            /* Set Output format */
            if( (all_set_bit & ISP_SET_ALLDATA_BIT_OUTFORMAT) != 0 ){
                unsigned char *config_data_p;
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_FMT, &all_data[ISP_SET_ALLDATA_POS_OUTFMT], 1);   // Output format(1byte)

                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_OUTFMT], &all_data[ISP_SET_ALLDATA_POS_OUTFMT], 1);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_OUTFMT], &all_data[ISP_SET_ALLDATA_POS_OUTFMT], 1);

                /* Select DRP Config data */
                if( ( all_data[ISP_SET_ALLDATA_POS_OUTFMT] == OUT_FORMAT_UYVY ) || ( all_data[ISP_SET_ALLDATA_POS_OUTFMT] == OUT_FORMAT_YUY2 ) )
                {
                    if( isp_in_format == IN_FORMAT_RAW8 ){
                        config_data_p = &ISP_Config_RAW8_YUV[0];
                        isp_config_data_size = ISP_Config_RAW8_YUV_len;
                    }
                    else if( isp_in_format == IN_FORMAT_RAW10 ){
                        config_data_p = &ISP_Config_RAW10_YUV[0];
                        isp_config_data_size = ISP_Config_RAW10_YUV_len;
                    }
                    else{
                        config_data_p = &ISP_Config_RAW12_YUV[0];
                        isp_config_data_size = ISP_Config_RAW12_YUV_len;
                    }
                }
                else
                {
                    if( isp_in_format == IN_FORMAT_RAW8 ){
                        config_data_p = &ISP_Config_RAW8_RGB[0];
                        isp_config_data_size = ISP_Config_RAW8_RGB_len;
                    }
                    else if( isp_in_format == IN_FORMAT_RAW10 ){
                        config_data_p = &ISP_Config_RAW10_RGB[0];
                        isp_config_data_size = ISP_Config_RAW10_RGB_len;
                    }
                    else{
                        config_data_p = &ISP_Config_RAW12_RGB[0];
                        isp_config_data_size = ISP_Config_RAW12_RGB_len;
                    }
                }
                pmem_write( isp_work_adr + ISP_WORK_OFFSET_CONFIG, config_data_p, isp_config_data_size);
            }
            
            /* Set accumlate ON/OFF, and accumlate area */
            if( (all_set_bit & ISP_SET_ALLDATA_BIT_ACCUMLATE) != 0 ){
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_ACCON, &all_data[ISP_SET_ALLDATA_POS_ACCON], 1); // Accumlate ON/OFF(1byte)
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_ACCAR, &all_data[ISP_SET_ALLDATA_POS_AREA], 8); // Accumlate Area(8byte)
                isp_accumlatearea_x = (all_data[ISP_SET_ALLDATA_POS_AREAXH] << 8)| all_data[ISP_SET_ALLDATA_POS_AREAXL];            // Accumlate image size
                isp_accumlatearea_y = (all_data[ISP_SET_ALLDATA_POS_AREAYH] << 8)| all_data[ISP_SET_ALLDATA_POS_AREAYL];
                isp_accumlatearea_w = (all_data[ISP_SET_ALLDATA_POS_AREAWH] << 8)| all_data[ISP_SET_ALLDATA_POS_AREAWL];
                isp_accumlatearea_h = (all_data[ISP_SET_ALLDATA_POS_AREAHH] << 8)| all_data[ISP_SET_ALLDATA_POS_AREAHL];

                if( ( all_data[ISP_SET_ALLDATA_POS_ACCON] == 1 ) && ( isp_accumlatearea_w != 0 ) && ( isp_accumlatearea_h != 0 ) )
                {
                    isp_accumlate_mode = ISP_ACCUMLATE_ON;
                }
                else
                {
                    isp_accumlate_mode = ISP_ACCUMLATE_OFF;
                }

                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_ACCON], &all_data[ISP_SET_ALLDATA_POS_ACCON], 9);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_ACCON], &all_data[ISP_SET_ALLDATA_POS_ACCON], 9);
            }
            
            /* Set color matrix data */
            if( (all_set_bit & ISP_SET_ALLDATA_CMX_PRESET) != 0 ){
                memcpy( &ISP_Preset_CMX[0], &all_data[ISP_SET_ALLDATA_POS_CMX_P], 72);  // Color Matrix Preset (2byte * 9data * 4preset = 72byte)
                int_val_cmx = -1;
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_CMX_P], &all_data[ISP_SET_ALLDATA_POS_CMX_P], 72);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_CMX_P], &all_data[ISP_SET_ALLDATA_POS_CMX_P], 72);
            }

            /* Set white balance data */
            if( (all_set_bit & ISP_SET_ALLDATA_WB_PRESET) != 0 ){
                memcpy( &ISP_Preset_WB[0], &all_data[ISP_SET_ALLDATA_POS_WB_P], 24);    // White Balance Preset (2byte * 3data * 4preset = 24byte)
                int_val_wb = -1;
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_WB_P], &all_data[ISP_SET_ALLDATA_POS_WB_P], 24);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_WB_P], &all_data[ISP_SET_ALLDATA_POS_WB_P], 24);
            }

            /* Set color matrix preset select */
            if( (all_set_bit & ISP_SET_ALLDATA_CMX_PRESET_SEL) != 0 ){
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_CMX, &ISP_Preset_CMX[all_data[ISP_SET_ALLDATA_POS_CMX_SEL]*9], sizeof(unsigned short)*9);// Color matrix area (2byte * 9)
                int_val_cmx = -1;
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_CMX_SEL], &all_data[ISP_SET_ALLDATA_POS_CMX_SEL], 1);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_CMX_SEL], &all_data[ISP_SET_ALLDATA_POS_CMX_SEL], 1);
            }

            /* Set white balance preset select */
            if( (all_set_bit & ISP_SET_ALLDATA_WB_PRESET_SEL) != 0 ){
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_WB, &ISP_Preset_WB[all_data[ISP_SET_ALLDATA_POS_WB_SEL]*3], sizeof(unsigned short)*3);// Gain R/G/B area (2byte * 3)
                int_val_wb = -1;
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_WB_SEL], &all_data[ISP_SET_ALLDATA_POS_WB_SEL], 1);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_WB_SEL], &all_data[ISP_SET_ALLDATA_POS_WB_SEL], 1);
            }

            /* Set image width/height/stride */
            if( (all_set_bit & ISP_SET_ALLDATA_IMGSIZE) != 0 ){
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_RECT, &all_data[ISP_SET_ALLDATA_POS_IMGSIZE], 6);  // Image Size data area(6byte)
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_IMGSIZE], &all_data[ISP_SET_ALLDATA_POS_IMGSIZE], 6);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_IMGSIZE], &all_data[ISP_SET_ALLDATA_POS_IMGSIZE], 6);
            }

            /* Set 3DNR Parameter */
            if( (all_set_bit & ISP_SET_ALLDATA_3DNR) != 0 ){
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_3DNRD, &all_data[ISP_SET_ALLDATA_POS_3DNR_D], 16);// 3DNR Parameter area(16byte)
                int_val_3dnr = -1;
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_3DNR_D], &all_data[ISP_SET_ALLDATA_POS_3DNR_D], 16);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_3DNR_D], &all_data[ISP_SET_ALLDATA_POS_3DNR_D], 16);
            }

            /* Set R/G/B Bias data */
            if( (all_set_bit & ISP_SET_ALLDATA_BL) != 0 ){
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_BL, &all_data[ISP_SET_ALLDATA_POS_BIAS], 6);   // Bias R/G/B area (2byte * 3)
                int_val_bl = -1;
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_BIAS], &all_data[ISP_SET_ALLDATA_POS_BIAS], 6);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_BIAS], &all_data[ISP_SET_ALLDATA_POS_BIAS], 6);
            }

            /* Set median filter blend ratio */
            if( (all_set_bit & ISP_SET_ALLDATA_2DNR) != 0 ){
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_2DNR, &all_data[ISP_SET_ALLDATA_POS_2DNR], 2);  // Madian filter blend parameter area (2byte)
                int_val_2dnr = -1;
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_2DNR], &all_data[ISP_SET_ALLDATA_POS_2DNR], 2);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_2DNR], &all_data[ISP_SET_ALLDATA_POS_2DNR], 2);
            }

            /* Set emphasys parameter */
            if( (all_set_bit & ISP_SET_ALLDATA_EMP) != 0 ){
                pmem_write( isp_work_adr + ISP_PARAM_OFFSET_EMP, &all_data[ISP_SET_ALLDATA_POS_EMP], 2);   // Unsharp Mask strength and coring parameter area (2byte)
                int_val_emp = -1;
                /* Save data using ISP_set function */
                memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_EMP], &all_data[ISP_SET_ALLDATA_POS_EMP], 2);
                memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_EMP], &all_data[ISP_SET_ALLDATA_POS_EMP], 2);
            }

            /* Ver1.0 */
            if (ver_info == 0) {
                /* Set gamma correct ON/OFF and gamma table */
                if( (all_set_bit & ISP_SET_ALLDATA_GAMMA10) != 0 ){
                    pmem_write( isp_work_adr + ISP_PARAM_OFFSET_GAMMA, &all_data[ISP_SET_ALLDATA_POS_GAMMAON10], 1);    // Gamma ON/OFF (1byte)
                    pmem_write( isp_work_adr + ISP_WORK_OFFSET_GAMMA, &all_data[ISP_SET_ALLDATA_POS_GAMMAD10], 0x100);  // Gamma table data (256byte)
                    int_val_gamma = -1;
                    /* Save data using ISP_set function */
                    memcpy( &all_v10_tbl[ISP_SET_ALLDATA_POS_GAMMAON10], &all_data[ISP_SET_ALLDATA_POS_GAMMAON10], 0x101);
                }
            }
            /* Ver1.1 */
            else
            {
                if( (all_set_bit & ISP_SET_ALLDATA_AE) != 0 ){
                    /* Set AE Parameters */
                    int ae_on;
                    short expose_level;
                    int target_y;
                    int threshold;

                    ae_on = all_data[ISP_SET_ALLDATA_POS_AE_ONOFF];
                    target_y = all_data[ISP_SET_ALLDATA_POS_AE_TARGET];
                    expose_level = ( all_data[ISP_SET_ALLDATA_POS_AE_EXPOSE_H] << 8 | all_data[ISP_SET_ALLDATA_POS_AE_EXPOSE_L] );
                    threshold = all_data[ISP_SET_ALLDATA_POS_AE_THRESHOLD];

                    /* Check arguments  */
                    if( ( (ae_on != ISP_AEMODE_OFF) && (ae_on != ISP_AEMODE_ON)) ||  // AE mode error
                        ( (expose_level < ISP_LIMIT_EXPOSE_LV_MIN ) || (expose_level > ISP_LIMIT_EXPOSE_LV_MAX)) ||   // Expose level error
                        ( (target_y < ISP_LIMIT_T_BL_MIN ) || (target_y > ISP_LIMIT_T_BL_MAX)) ||   // Target Y error
                        ( (threshold < ISP_LIMIT_THRESHOLD_MIN ) || (threshold > ISP_LIMIT_THRESHOLD_MAX)) )   // Threshold error
                    {
                        ret_data = (-EINVAL);
                        goto end;
                    }

                    /* Save data using ISP_set function */
                    memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_AE_ONOFF], &all_data[ISP_SET_ALLDATA_POS_AE_ONOFF], 5);

                    isp_AE_mode = ae_on;
                    isp_AE_expose_level = expose_level;
                    isp_AE_wait = AE_CAMERA_WAIT;
                    isp_AE_target = target_y;
                    isp_AE_threshold = threshold;
                }
                if( (all_set_bit & ISP_SET_ALLDATA_GAMMA11) != 0 ){
                    pmem_write( isp_work_adr + ISP_PARAM_OFFSET_GAMMA, &all_data[ISP_SET_ALLDATA_POS_GAMMAON11], 1);    // Gamma ON/OFF (1byte)
                    pmem_write( isp_work_adr + ISP_WORK_OFFSET_GAMMA, &all_data[ISP_SET_ALLDATA_POS_GAMMAD11], 0x100);  // Gamma table data (256byte)
                    int_val_gamma = -1;
                    /* Save data using ISP_set function */
                    memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_GAMMAON11], &all_data[ISP_SET_ALLDATA_POS_GAMMAON11], 0x101);
                }
            	
            	if( (all_set_bit & ISP_SET_ALLDATA_AWB) != 0 ){
                    /* Set AWB Parameters */
                    int awb_manual;
                    int awb_manual_rgain;
                    int awb_manual_bgain;

                    awb_manual = all_data[ISP_SET_ALLDATA_POS_AWB_MANUAL];
                    awb_manual_rgain = ( all_data[ISP_SET_ALLDATA_POS_AWB_MANUAL_R_H] << 8 | all_data[ISP_SET_ALLDATA_POS_AWB_MANUAL_R_L] );
                    awb_manual_bgain = ( all_data[ISP_SET_ALLDATA_POS_AWB_MANUAL_B_H] << 8 | all_data[ISP_SET_ALLDATA_POS_AWB_MANUAL_B_L] );

                    /* Check arguments  */
                    if( ( (awb_manual != AWB_AUTO) && (awb_manual != AWB_MANUAL)) ||  // AWB manual error
                        ( (awb_manual_rgain < ISP_LIMIT_AWB_GAIN_MIN ) || (awb_manual_rgain > ISP_LIMIT_AWB_GAIN_MAX)) ||   // manual rgain error
                        ( (awb_manual_bgain < ISP_LIMIT_AWB_GAIN_MIN ) || (awb_manual_bgain > ISP_LIMIT_AWB_GAIN_MAX)) )   // manual bgain error
                    {
                        ret_data = (-EINVAL);
                        goto end;
                    }

                    /* Save data using ISP_set function */
                    memcpy( &all_v11_tbl[ISP_SET_ALLDATA_POS_AWB_MANUAL], &all_data[ISP_SET_ALLDATA_POS_AWB_MANUAL], 5);

                    isp_AWB_manual = awb_manual;
                    isp_AWB_manual_rgain = awb_manual_rgain;
                    isp_AWB_manual_bgain = awb_manual_bgain;
                    int_val_awb = -1;
                }
            }
            break;
        case ISP_FUNC_BL:
            /* Set Bias of SimpleISP parameter */
            int_val = *((int *)value);
            if( ( int_val < 0 ) || ( int_val > 0x7F ) )
            {
                ret_data = (-EINVAL);
                goto end;
            }
            *(short*)(&isp_param[0x00])   = -int_val;
            *(short*)(&isp_param[0x02])   = -int_val;
            *(short*)(&isp_param[0x04])   = -int_val;
            pmem_write( isp_work_adr + ISP_PARAM_OFFSET_BL, &isp_param[0], sizeof(unsigned short)*3);// Bias R/G/B area size (2byte * 3)
#if ISP_DEBUG
            printk( "[ISP PARAM]\n");
            pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif
            /* Set data for ISP_get function */
            int_val_bl = int_val;
            break;
        case ISP_FUNC_WB:
            int_val = *((int *)value);
            /* Set Gain of SimpleISP parameter */
            if( ( int_val < 0 ) || ( int_val > 3 ) )
            {
                ret_data = (-EINVAL);
                goto end;
            }
            pmem_write( isp_work_adr + ISP_PARAM_OFFSET_WB, &ISP_Preset_WB[int_val*3], sizeof(unsigned short)*3);// Gain R/G/B area size (2byte * 3)
#if ISP_DEBUG
            printk( "[ISP PARAM]\n");
            pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif
            /* Set data for ISP_get function */
            int_val_wb = int_val;
            break;
        case ISP_FUNC_GAMMA:
            int_val = *((int *)value);
            /* Set Gamma of SimpleISP parameter */
            if( ( int_val <= 0 ) || ( int_val > GAMMA_MAX ) )
            {
                ret_data = (-EINVAL);
                goto end;
            }
            pmem_write( isp_work_adr + ISP_WORK_OFFSET_GAMMA, get_gamma_tbl(int_val), 0x100);
#if ISP_DEBUG
            printk( "[GAMMA TBL] val = %d\n",int_val);
            pmem_print( isp_work_adr + ISP_WORK_OFFSET_GAMMA, 0x100);
#endif
            if( int_val == 100 )
            {
                /* gamma int_val is 1.00 ( = gamma off) */
                isp_param[0x00] = 0;
            }
            else
            {
                /* gamma int_val is not 1.00 ( = gamma on) */
                isp_param[0x00] = 1;
            }
            pmem_write( isp_work_adr + ISP_PARAM_OFFSET_GAMMA, &isp_param[0], sizeof(unsigned char));
#if ISP_DEBUG
            printk( "[ISP PARAM]\n");
            pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif
            /* Set data for ISP_get function */
            int_val_gamma = int_val;
            break;
        case ISP_FUNC_CMX:
            int_val = *((int *)value);
            /* Set Color Matrix elements of SimpleISP parameter */
            if( ( int_val < 0 ) || ( int_val > 3 ) )
            {
                ret_data = (-EINVAL);
                goto end;
            }
            pmem_write( isp_work_adr + ISP_PARAM_OFFSET_CMX, &ISP_Preset_CMX[int_val*9], sizeof(unsigned short)*9);// Color matrix area size (2byte * 9)
#if ISP_DEBUG
            printk( "[ISP PARAM]\n");
            pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif
            /* Set data for ISP_get function */
            int_val_cmx = int_val;
            break;
        case ISP_FUNC_2DNR:
            int_val = *((int *)value);
            /* Set Median blend of SimpleISP parameter */
            if( ( int_val < 0 ) || ( int_val > 100 ) )
            {
                ret_data = (-EINVAL);
                goto end;
            }
            *(unsigned short*)(&isp_param[0x00])   = (unsigned short)((int_val * 256uL + 50) / 100);  // 
            pmem_write( isp_work_adr + ISP_PARAM_OFFSET_2DNR, &isp_param[0], sizeof(unsigned short));
#if ISP_DEBUG
            printk( "[ISP PARAM]\n");
            pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif
            /* Set data for ISP_get function */
            int_val_2dnr = int_val;
            break;
        case ISP_FUNC_3DNR:
            int_val = *((int *)value);
            /* Set 3DNR of SimpleISP parameter */
            if( ( int_val != 0 ) && ( int_val != 1 ) )
            {
                ret_data = (-EINVAL);
                goto end;
            }
            *(unsigned int*)(&isp_param[0x00]) = int_val == 0 ? 0 : isp_work_adr + ISP_WORK_OFFSET_3DNR;     // Work area for 3DNR 
            pmem_write( isp_work_adr + ISP_PARAM_OFFSET_3DNR, &isp_param[0], sizeof(unsigned int));
#if ISP_DEBUG
            printk( "[ISP PARAM]\n");
            pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif
            /* Set data for ISP_get function */
            int_val_3dnr = int_val;
            break;
        case ISP_FUNC_EMP:
            int_val = *((int *)value);
            /* Set Unsharp Mask of SimpleISP parameter */
            if( int_val == 0 )
            {
                isp_param[0x00] = PARAM_EMP_OFF;     // Emphasis off;
            }
            else if ( int_val == 1 )
            {
                isp_param[0x00] = PARAM_EMP_WEAK;    // Emphasis weak;
            }
            else if ( int_val == 2 )
            {
                isp_param[0x00] = PARAM_EMP_NORMAL;  // Emphasis normal;
            }
            else if ( int_val == 3 )
            {
                isp_param[0x00] = PARAM_EMP_STRONG;  // Emphasis strong;
            }
            else
            {
                ret_data = (-EINVAL);
                goto end;
            }
            pmem_write( isp_work_adr + ISP_PARAM_OFFSET_EMP, &isp_param[0], sizeof(unsigned char));
#if ISP_DEBUG
            printk( "[ISP PARAM]\n");
            pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif
            /* Set data for ISP_get function */
            int_val_emp = int_val;
            break;
        case ISP_FUNC_AWB:
            int_val = *((int *)value);
            /* Set Gain of SimpleISP parameter */
            if( ( int_val < 0 ) || ( int_val > 1 ) )
            {
                ret_data = (-EINVAL);
                goto end;
            }
            int_val_awb = int_val;
            isp_AWB_manual = int_val_awb;
            isp_AWB_manual_rgain = ISP_PARAM_AWB_DEFAULT;
            isp_AWB_manual_bgain = ISP_PARAM_AWB_DEFAULT;
            break;
        default:
            /* return EINVAL */
            ret_data = (-EINVAL);
            break;
    }

end:
    return ret_data;
}

/* ISP AE ON/OFF and AE setting(from V4L2) */
int ISP_AE_Set(int ae_on, int expose_level, int target_y, int threshold)
{
    int ret_data = 0;
    int cnt_i;

    /* Check the operating state of ISP (="READY") */
    if( isp_status != ISP_STATUS_READY )
    {
        ret_data = (-EACCES);
        goto end;
    }

    /* Check arguments  */
    if( ( (ae_on != ISP_AEMODE_OFF) && (ae_on != ISP_AEMODE_ON)) ||  // AE mode error
        ( (expose_level < ISP_LIMIT_EXPOSE_LV_MIN ) || (expose_level > ISP_LIMIT_EXPOSE_LV_MAX)) ||   // Expose level error
        ( (target_y < ISP_LIMIT_T_BL_MIN ) || (target_y > ISP_LIMIT_T_BL_MAX)) ||   // Target Y error
        ( (threshold < ISP_LIMIT_THRESHOLD_MIN ) || (threshold > ISP_LIMIT_THRESHOLD_MAX)) )   // Threshold error
    {
        ret_data = (-EINVAL);
        goto end;
    }

    /* Set AE Parameters */
    isp_AE_mode = ae_on;
    isp_AE_expose_level = expose_level;
    isp_AE_wait = AE_CAMERA_WAIT;
    isp_AE_target = target_y;
    isp_AE_threshold = threshold;
    
    /* Set AE Default value */
    for( cnt_i = 0; cnt_i < ISP_AE_HISTORY; cnt_i++ )
    {
        isp_AE_expose_history[cnt_i] = ISP_AE_HISTORY_INIT-cnt_i; // Expose level history clear
    }

    /* Creat AE table */
    set_ae_table( isp_frame_interval );

end:
    return ret_data;
}

/* Get gain and expose value(from V4L2) */
void ISP_AE_Get(int *gain_p, int *expose_p)
{
    if( gain_p != NULL )
    {
        *gain_p = AE_tbl[ISP_LIMIT_EXPOSE_LV_MAX - isp_AE_expose_level][0];
    }
    if( expose_p != NULL )
    {
        *expose_p = AE_tbl[ISP_LIMIT_EXPOSE_LV_MAX - isp_AE_expose_level][1];
    }
}

/* Start ISP (from V4L2) */
int ISP_go(unsigned long in_image_adr, unsigned long out_image_adr,
             int level, func_isp_callback callback_p)
{
    unsigned int in_drp_adr;
    unsigned int out_drp_adr;
    drpai_data_t drp_arg[2];
    int ret_data = 0;
    int ret;
    
    /* Check the operating state of ISP (="READY") */
    if( isp_status != ISP_STATUS_READY )
    {
        ret_data = (-EACCES);
        goto end;
    }

    /* Check arguments  */
    if( (in_image_adr == 0) ||       // Illegal input image address
        (out_image_adr == 0 ) ||     // Illegal output image address
        (callback_p == 0) )          // Illegal callback function pointer
    {
        ret_data = (-EINVAL);
        goto end;
    }

    /* Set SimpleISP Parameters */
    in_drp_adr = (unsigned int)(in_image_adr & 0xFFFFFFFF);
    out_drp_adr = (unsigned int)(out_image_adr & 0xFFFFFFFF);
    pmem_write( isp_work_adr + ISP_PARAM_OFFSET_SRC, &in_drp_adr, sizeof(unsigned int));
    pmem_write( isp_work_adr + ISP_PARAM_OFFSET_DST, &out_drp_adr, sizeof(unsigned int));
	
	pmem_write( isp_work_adr + ISP_PARAM_OFFSET_RGAIN, &isp_AWB_gain[AWB_COLOR_R], sizeof(unsigned short)); //Rgain
	pmem_write( isp_work_adr + ISP_PARAM_OFFSET_BGAIN, &isp_AWB_gain[AWB_COLOR_B], sizeof(unsigned short)); //Bgain
	
#if ISP_DEBUG
    printk( "[ISP PARAM]\n");
    pmem_print( isp_work_adr + ISP_WORK_OFFSET_PARAM, ISP_PARAM_OFFSET_SIZE);
#endif

    /* Save out_image adr */
    isp_out_image_adr = out_image_adr;

    /* Save callback function pointer */
    isp_callback_p = callback_p;
    
    /* Set DRP parameters */
    drp_arg[0].address = isp_work_adr + ISP_WORK_OFFSET_CONFIG;
    drp_arg[0].size    = isp_config_data_size;
    drp_arg[1].address = isp_work_adr + ISP_WORK_OFFSET_PARAM;
    drp_arg[1].size    = ISP_PARAM_OFFSET_SIZE;
    /* Start DRP */
    ret = drpai_start_k(&drp_arg[0], ISP_callback);
    if( ret )
    {
        ret_data = ret;
        goto end;
    }

    /* Set ISP operating state to "RUN" */
    isp_status = ISP_STATUS_RUN;      // ISP operating running
end:
    return ret_data;

}

int ISP_get_in_buffer(unsigned long *cam_buffer_list)
{
    int ret_data = ISP_CAM_BUFFER_NUM;
    int cnt_i;

    if( ret_data > 4 )
    {
        ret_data = 4;
    }

    /* Check the operating state of ISP (!="INIT") */
    if( isp_status == ISP_STATUS_INIT )
    {
        ret_data = (-EACCES);
        goto end;
    }

    /* Check arguments  */
    if( cam_buffer_list == NULL )
    {
        ret_data = (-EINVAL);
        goto end;
    }

    for( cnt_i = 0; cnt_i < ret_data; cnt_i++ )
    {
        cam_buffer_list[cnt_i] = isp_work_adr + ISP_WORK_OFFSET_RBUF + ISP_RBUF_SIZE * cnt_i;
    }

end:
    return ret_data;   //Fixed num

}

/* Get ISP parameter (from V4L2) */
int ISP_get_param(unsigned short *out_in_format, unsigned short *out_frame_interval,
                  int *out_level, int *out_ae_on, int *out_expose_level,
                  int *out_target_y, int *out_threshold)
{
    int ret_data = 0;

    /* Check the operating state of ISP (="READY") */
    if( isp_status != ISP_STATUS_READY )
    {
        ret_data = (-EACCES);
        goto end;
    }

    /* Get input format (RAW8/10/12), default is RAW10 */
    if( out_in_format != NULL )
    {
        *out_in_format = isp_in_format;
    }

    /* Get frame interval(ms), default is 33 */
    if( out_frame_interval != NULL )
    {
        *out_frame_interval = isp_frame_interval;
    }

    /* Get DRP priority level (not used) */
    if( out_level != NULL )
    {
        *out_level = isp_drp_priority_level;
    }

    /* Get AE ON/OFF */
    if( out_ae_on != NULL )
    {
        *out_ae_on = isp_AE_mode;
    }

    /* Get Expose level */
    if( out_expose_level != NULL )
    {
        *out_expose_level = isp_AE_expose_level;
    }

    /* Get AE target Y value(from 1 to 254) */
    if( out_target_y != NULL )
    {
        *out_target_y = isp_AE_target;
    }

    /* Get AE threshold */
    if( out_threshold != NULL )
    {
        *out_threshold = isp_AE_threshold;
    }
end:
    return ret_data;

}

/* Callback for V4L2 */
static void ISP_callback( int result)
{
    int threshold = isp_AE_threshold;
    unsigned int *luminance_p;
    unsigned int red;
    unsigned int green;
    unsigned int blue;
    unsigned int luminance;
    int last_isp_AE_expose_level;
    
    /* Check the operating state of ISP (="RUN") */
    if( isp_status != ISP_STATUS_RUN )
    {
        goto end;
    }
	
#if ISP_DEBUG
		printk ("call ISP callbackx\n");
#endif

    /* Adjust AE Level? */
    if( ( isp_AE_mode == ISP_AEMODE_ON ) && (isp_accumlate_mode == ISP_ACCUMLATE_ON) )
    {
        /* AE ON */
        if( isp_AE_wait > 0 )
        {
            /* Skip AE */
            isp_AE_wait--;
        }
        else
        {
            if( ( isp_AE_expose_history[0] == isp_AE_expose_history[2] ) && ( isp_AE_expose_history[1] == isp_AE_expose_history[3] ) )
            {
                /* Last expose level value was oscillating, increased the threshold of changing the expose level */
                threshold = isp_AE_threshold*2;
            }

            /* Get the average value of red/green/blue */
            luminance_p = ((unsigned int*)phys_to_virt(isp_work_adr + ISP_WORK_OFFSET_ACCUM));
            red = (*luminance_p)*4 / (isp_accumlatearea_w * isp_accumlatearea_h);       //red = 0 to 0x1000
            red = (red * wb_red) / 0x1000;
            luminance_p++;
            green = (*luminance_p)*2 / (isp_accumlatearea_w * isp_accumlatearea_h);     //green = 0 to 0x1000
            green = (green * wb_green) / 0x1000;
            luminance_p++;
            blue = (*luminance_p)*4 / (isp_accumlatearea_w * isp_accumlatearea_h);     //blue = 0 to 0x1000
            blue = (blue * wb_blue) / 0x1000;
            /* Y = red * 0.229 + green * 0.687 + blue * 0.114 */
            luminance = (red*299 + green*687+ blue*114)/(1000 * 0x10);

            /* Calculate the new isp_AE_expose_level */
            last_isp_AE_expose_level = isp_AE_expose_level;
            if( (int)luminance < (int)isp_AE_target - threshold * AE_THRESHOLD_HIGH )
            {
                isp_AE_expose_level+=20;    //+2dB
            }
            else if( (int)luminance < (int)isp_AE_target - threshold * AE_THRESHOLD_MID )
            {
                isp_AE_expose_level+=5;    //+0.5dB
            }
            else if( (int)luminance < (int)isp_AE_target - threshold )
            {
                isp_AE_expose_level++;    //+0.1dB
            }
            if( (int)luminance > (int)isp_AE_target + threshold * AE_THRESHOLD_HIGH )
            {
                isp_AE_expose_level-=20;    //-2dB
            }
            else if( (int)luminance > (int)isp_AE_target + threshold * AE_THRESHOLD_MID )
            {
                isp_AE_expose_level-=5;     //-0.5dB
            }
            else if( (int)luminance > (int)isp_AE_target + threshold )
            {
                isp_AE_expose_level--;      //-0.1dB
            }

            /* Limit check */
            if ( isp_AE_expose_level > ISP_LIMIT_EXPOSE_LV_MAX )
            {
                 isp_AE_expose_level = ISP_LIMIT_EXPOSE_LV_MAX;
            }
            if ( isp_AE_expose_level < ISP_LIMIT_EXPOSE_LV_MIN )
            {
                 isp_AE_expose_level = ISP_LIMIT_EXPOSE_LV_MIN;
            }
            
            if( isp_AE_expose_level != last_isp_AE_expose_level )
            {
                isp_AE_wait = AE_CAMERA_WAIT;
            }

            /* Update expose level history list */
            if( isp_AE_expose_level != isp_AE_expose_history[0] )
            {
                isp_AE_expose_history[3] = isp_AE_expose_history[2];
                isp_AE_expose_history[2] = isp_AE_expose_history[1];
                isp_AE_expose_history[1] = isp_AE_expose_history[0];
                isp_AE_expose_history[0] = isp_AE_expose_level;
            }
            #if ISP_GET_FUNC_DBUG
            printk ("Expose level %d",isp_AE_expose_level);
            #endif /* ISP_GET_FUNC_DBUG */
        }
    }
	
	/* AWB Control */
	if(isp_AWB_mode == AWB_MODE_ON)
	{
		unsigned int *result;
		unsigned int detectSum[AWB_COLOR_NUM];
		unsigned int detectCnt;
		unsigned long long detectSumFixed[AWB_COLOR_NUM];
		unsigned long long ratioB;
		unsigned long long ratioR;
		unsigned long long ratioBLimit;
		unsigned long long ratioRLimit;
		unsigned long long gainRatioB;
		unsigned long long gainRatioR;
		unsigned long long applicationRatio;
		unsigned long long wbGainCtrl[AWB_COLOR_NUM];
		int wbGainFixed[AWB_COLOR_NUM];
		int enReliability;
		int enPreventBlack;
		
		/* Get the integral value of red/green/blue and Number of pixels */
		result = ((unsigned int*)phys_to_virt(isp_work_adr + ISP_WORK_OFFSET_ACCUM));
		result += 9; // awbCmt offset
		detectCnt = *result;
		result++; //awbSumR 
		detectSum[AWB_COLOR_R] = *result;
		result++; //awbSumG 
		detectSum[AWB_COLOR_G] = *result;
		result++; //awbSumB 
		detectSum[AWB_COLOR_B] = *result;
		
#if ISP_DEBUG
        printk ("detectCnt %d\n",detectCnt);
		printk ("detectSumR %d\n",detectSum[AWB_COLOR_R]);
		printk ("detectSumG %d\n",detectSum[AWB_COLOR_G]);
		printk ("detectSumB %d\n",detectSum[AWB_COLOR_B]);
#endif
		
		/* for Fixed */
		detectSumFixed[AWB_COLOR_R] = (long long)detectSum[AWB_COLOR_R] << ISP_PARAM_AWB_SHIFT;
		detectSumFixed[AWB_COLOR_G] = (long long)detectSum[AWB_COLOR_G] << ISP_PARAM_AWB_SHIFT;
		detectSumFixed[AWB_COLOR_B] = (long long)detectSum[AWB_COLOR_B] << ISP_PARAM_AWB_SHIFT;
		
#if ISP_DEBUG
		printk ("detectSumFixedR %llx\n",detectSumFixed[AWB_COLOR_R]);
		printk ("detectSumFixedG %llx\n",detectSumFixed[AWB_COLOR_G]);
		printk ("detectSumFixedB %llx\n",detectSumFixed[AWB_COLOR_B]);
#endif

		/* ratio */
		if (detectSum[AWB_COLOR_B] == 0) {
			ratioB = 1 << ISP_PARAM_AWB_SHIFT;
		}
		else if (detectSum[AWB_COLOR_G] == 0) {
			ratioB = 1 << ISP_PARAM_AWB_SHIFT;
		}
		else {
			ratioB = (detectSumFixed[AWB_COLOR_G] << ISP_PARAM_AWB_SHIFT) / detectSumFixed[AWB_COLOR_B];
		}
		if (detectSum[AWB_COLOR_R] == 0) {
			ratioR = 1 << ISP_PARAM_AWB_SHIFT;
		}
		else if (detectSum[AWB_COLOR_G] == 0) {
			ratioR = 1 << ISP_PARAM_AWB_SHIFT;
		}
		else {
			ratioR = (detectSumFixed[AWB_COLOR_G] << ISP_PARAM_AWB_SHIFT) / detectSumFixed[AWB_COLOR_R];
		}

		/* Limit */
		if (ratioB > ISP_PARAM_AWB_RATIO_B_LIMIT_H) {
			ratioBLimit = ISP_PARAM_AWB_RATIO_B_LIMIT_H;
		}
		else if (ratioB < ISP_PARAM_AWB_RATIO_B_LIMIT_L) {
			ratioBLimit = ISP_PARAM_AWB_RATIO_B_LIMIT_L;
		}
		else {
			ratioBLimit = ratioB;
		}
		if (ratioR > ISP_PARAM_AWB_RATIO_R_LIMIT_H) {
			ratioRLimit = ISP_PARAM_AWB_RATIO_R_LIMIT_H;
		}
		else if (ratioR < ISP_PARAM_AWB_RATIO_R_LIMIT_L) {
			ratioRLimit = ISP_PARAM_AWB_RATIO_R_LIMIT_L;
		}
		else {
			ratioRLimit = ratioR;
		}
		
#if ISP_DEBUG
		printk ("ratioRLimit %llx\n",ratioRLimit);
		printk ("ratioBLimit %llx\n",ratioBLimit);
#endif

		/* reliability */
		if (detectCnt >= ISP_PARAM_AWB_RELIABILITY_TH) {
			enReliability = 1;
		}
		else {
			enReliability = 0;
		}

		/* prevent black */
		if (detectCnt == 0) {
			enPreventBlack = 0;
		}
		else if (detectSum[AWB_COLOR_G] / detectCnt >= ISP_PARAM_AWB_PREVENT_BLACK_TH) {
			enPreventBlack = 1;
		}
		else {
			enPreventBlack = 0;
		}
		
#if ISP_DEBUG
		printk ("enReliability %d\n",enReliability);
		printk ("enPreventBlack %d\n",enPreventBlack);
#endif

		/* variable speed */
		gainRatioB = (ratioBLimit << ISP_PARAM_AWB_SHIFT) / isp_AWB_gain_prev[AWB_COLOR_B];
		gainRatioR = (ratioRLimit << ISP_PARAM_AWB_SHIFT) / isp_AWB_gain_prev[AWB_COLOR_R];
		if (gainRatioB >= ISP_PARAM_AWB_VARIABLE_SPEED_TH_3H
			|| gainRatioR >= ISP_PARAM_AWB_VARIABLE_SPEED_TH_3H
			|| gainRatioB <= ISP_PARAM_AWB_VARIABLE_SPEED_TH_3L
			|| gainRatioR <= ISP_PARAM_AWB_VARIABLE_SPEED_TH_3L
			) {
			applicationRatio = ISP_PARAM_AWB_VARIABLE_SPEED_COEF_3;
		}
		else if (gainRatioB >= ISP_PARAM_AWB_VARIABLE_SPEED_TH_2H
			|| gainRatioR >= ISP_PARAM_AWB_VARIABLE_SPEED_TH_2H
			|| gainRatioB <= ISP_PARAM_AWB_VARIABLE_SPEED_TH_2L
			|| gainRatioR <= ISP_PARAM_AWB_VARIABLE_SPEED_TH_2L
			) {
			applicationRatio = ISP_PARAM_AWB_VARIABLE_SPEED_COEF_2;
		}
		else if (gainRatioB >= ISP_PARAM_AWB_VARIABLE_SPEED_TH_1H
			|| gainRatioR >= ISP_PARAM_AWB_VARIABLE_SPEED_TH_1H
			|| gainRatioB <= ISP_PARAM_AWB_VARIABLE_SPEED_TH_1L
			|| gainRatioR <= ISP_PARAM_AWB_VARIABLE_SPEED_TH_1L
			) {
			applicationRatio = ISP_PARAM_AWB_VARIABLE_SPEED_COEF_1;
		}
		else {
			applicationRatio = 0;
		}
		
#if ISP_DEBUG
		printk ("applicationRatio %llx\n",applicationRatio);
#endif

		/* gain update */
		if (isp_AWB_manual == AWB_MANUAL) {
			wbGainCtrl[AWB_COLOR_R] = isp_AWB_manual_rgain;
			wbGainCtrl[AWB_COLOR_B] = isp_AWB_manual_bgain;
		}
		else if (enReliability == 1 && enPreventBlack == 1) {
			long long wbGainTargetR = ratioRLimit;
			long long wbGainTargetB = ratioBLimit;
			long long wbGainApplyR = ((long long)(wbGainTargetR - isp_AWB_gain_prev[AWB_COLOR_R]) * (long long)applicationRatio) >> ISP_PARAM_AWB_SHIFT;
			long long wbGainApplyB = ((long long)(wbGainTargetB - isp_AWB_gain_prev[AWB_COLOR_B]) * (long long)applicationRatio) >> ISP_PARAM_AWB_SHIFT;
			wbGainCtrl[AWB_COLOR_R] = isp_AWB_gain_prev[AWB_COLOR_R] + wbGainApplyR;
			wbGainCtrl[AWB_COLOR_B] = isp_AWB_gain_prev[AWB_COLOR_B] + wbGainApplyB;
		}
		else {
			wbGainCtrl[AWB_COLOR_R] = isp_AWB_gain_prev[AWB_COLOR_R];
			wbGainCtrl[AWB_COLOR_B] = isp_AWB_gain_prev[AWB_COLOR_B];
		}
		
#if ISP_DEBUG
		printk ("wbGainCtrlR %llx\n",wbGainCtrl[AWB_COLOR_R]);
		printk ("wbGainCtrlB %llx\n",wbGainCtrl[AWB_COLOR_B]);
#endif

		if (isp_AWB_manual == AWB_MANUAL) {
			wbGainFixed[AWB_COLOR_R] = (int)wbGainCtrl[AWB_COLOR_R];
			wbGainFixed[AWB_COLOR_B] = (int)wbGainCtrl[AWB_COLOR_B];
			wbGainFixed[AWB_COLOR_G] = (int)(ISP_PARAM_AWB_WB_GAIN_G >> ISP_PARAM_AWB_GAINDIV);
		}
		else
		{
			/* fixed */
			wbGainFixed[AWB_COLOR_R] = (int)(wbGainCtrl[AWB_COLOR_R] >> ISP_PARAM_AWB_GAINDIV);
			wbGainFixed[AWB_COLOR_B] = (int)(wbGainCtrl[AWB_COLOR_B] >> ISP_PARAM_AWB_GAINDIV);
			wbGainFixed[AWB_COLOR_G] = (int)(ISP_PARAM_AWB_WB_GAIN_G >> ISP_PARAM_AWB_GAINDIV);
		}
#if ISP_DEBUG
		printk ("wbGainFixedR %d\n",wbGainFixed[AWB_COLOR_R]);
		printk ("wbGainFixedB %d\n",wbGainFixed[AWB_COLOR_B]);
#endif

		/* gain limit */
		if (wbGainFixed[AWB_COLOR_R] > ISP_LIMIT_AWB_GAIN_MAX) {
			isp_AWB_gain[AWB_COLOR_R] = ISP_LIMIT_AWB_GAIN_MAX;
		}
	    else if(wbGainFixed[AWB_COLOR_R] < ISP_LIMIT_AWB_GAIN_MIN){
	        isp_AWB_gain[AWB_COLOR_R] = ISP_LIMIT_AWB_GAIN_MIN;
	    }
		else {
			isp_AWB_gain[AWB_COLOR_R] = wbGainFixed[AWB_COLOR_R];
		}
		if (wbGainFixed[AWB_COLOR_G] > ISP_LIMIT_AWB_GAIN_MAX) {
			isp_AWB_gain[AWB_COLOR_G] = ISP_LIMIT_AWB_GAIN_MAX;
		}
	    else if(wbGainFixed[AWB_COLOR_G] < ISP_LIMIT_AWB_GAIN_MIN){
	        isp_AWB_gain[AWB_COLOR_G] = ISP_LIMIT_AWB_GAIN_MIN;
	    }
		else {
			isp_AWB_gain[AWB_COLOR_G] = wbGainFixed[AWB_COLOR_G];
		}
		if (wbGainFixed[AWB_COLOR_B] > ISP_LIMIT_AWB_GAIN_MAX) {
			isp_AWB_gain[AWB_COLOR_B] = ISP_LIMIT_AWB_GAIN_MAX;
		}
	    else if(wbGainFixed[AWB_COLOR_B] < ISP_LIMIT_AWB_GAIN_MIN){
	        isp_AWB_gain[AWB_COLOR_B] = ISP_LIMIT_AWB_GAIN_MIN;
	    }
		else {
			isp_AWB_gain[AWB_COLOR_B] = wbGainFixed[AWB_COLOR_B];
		}

		/* gain save */
		isp_AWB_gain_prev[AWB_COLOR_R] = wbGainCtrl[AWB_COLOR_R];
		isp_AWB_gain_prev[AWB_COLOR_B] = wbGainCtrl[AWB_COLOR_B];
		
#if ISP_DEBUG
		printk ("wbGainR %d\n",isp_AWB_gain[AWB_COLOR_R]);
		printk ("wbGainB %d\n",isp_AWB_gain[AWB_COLOR_B]);
		printk ("isp_AWB_gain_prevR %llx\n",isp_AWB_gain_prev[AWB_COLOR_R]);
		printk ("isp_AWB_gain_prevB %llx\n",isp_AWB_gain_prev[AWB_COLOR_B]);
#endif
	}
	//AWB_MODE_OFF
	else
	{
		//Default value
		isp_AWB_gain[AWB_COLOR_R] = ISP_PARAM_AWB_DEFAULT;
		isp_AWB_gain[AWB_COLOR_G] = ISP_PARAM_AWB_DEFAULT;
		isp_AWB_gain[AWB_COLOR_B] = ISP_PARAM_AWB_DEFAULT;
		isp_AWB_gain_prev[AWB_COLOR_R] = 1 << ISP_PARAM_AWB_SHIFT;
		isp_AWB_gain_prev[AWB_COLOR_G] = 1 << ISP_PARAM_AWB_SHIFT;
		isp_AWB_gain_prev[AWB_COLOR_B] = 1 << ISP_PARAM_AWB_SHIFT;
	}

    /* Callback for V4L2 */
    isp_callback_p( result, isp_out_image_adr);
    
    /* Set ISP operating state to "READY" */
    isp_status = ISP_STATUS_READY;      // ISP operating ready
end:
    return;
}

int ISP_get(int isp_func, void *value)
{
    int ret_data = 0;
    int * tmp = (int*) value;

    /* Check the operating state of ISP (="READY") */
    if( isp_status != ISP_STATUS_READY )
    {
        ret_data = (-EACCES);
        goto end;
    }

    switch( isp_func )
    {
        case ISP_FUNC_ALL:
            /* Ver1.0 */
            if (ver_info == 0) {
                
                memcpy(tmp, &all_v10_tbl[0], ISP_SET_ALLDATA_SIZE_V10);
                
                #if ISP_GET_FUNC_DBUG
                printk("ISP_get Ver1.0%x\n",*tmp);
                #endif /* ISP_GET_FUNC_DBUG */
            }
            /* Ver1.1 */
            else
            {
                memcpy(tmp, &all_v11_tbl[0], ISP_SET_ALLDATA_SIZE_V11);
                #if ISP_GET_FUNC_DBUG
                printk("ISP_get Ver1.1%x\n",*tmp);
                #endif /* ISP_GET_FUNC_DBUG */
            }
            break;
        case ISP_FUNC_BL:
            *tmp = int_val_bl;
            #if ISP_GET_FUNC_DBUG
            printk("Bl%x\n",*tmp);
            #endif /* ISP_GET_FUNC_DBUG */
            break;
        case ISP_FUNC_WB:
            *tmp = int_val_wb;
            #if ISP_GET_FUNC_DBUG
            printk("WB%x\n",*tmp);
            #endif /* ISP_GET_FUNC_DBUG */
            break;
        case ISP_FUNC_GAMMA:
            *tmp = int_val_gamma;
            #if ISP_GET_FUNC_DBUG
            printk("Gamma%x\n",*tmp);
            #endif /* ISP_GET_FUNC_DBUG */
            break;
        case ISP_FUNC_CMX:
            *tmp = int_val_cmx;
            #if ISP_GET_FUNC_DBUG
            printk("cmx%x\n",*tmp);
            #endif /* ISP_GET_FUNC_DBUG */
            break;
        case ISP_FUNC_2DNR:
            *tmp = int_val_2dnr;
            #if ISP_GET_FUNC_DBUG
            printk("2dnr%x\n",*tmp);
            #endif /* ISP_GET_FUNC_DBUG */
            break;
        case ISP_FUNC_3DNR:
            *tmp = int_val_3dnr;
            #if ISP_GET_FUNC_DBUG
            printk("3dnr%x\n",*tmp);
            #endif /* ISP_GET_FUNC_DBUG */
            break;
        case ISP_FUNC_EMP:
            *tmp = int_val_emp;
            #if ISP_GET_FUNC_DBUG
            printk("emp%x\n",*tmp);
            #endif /* ISP_GET_FUNC_DBUG */
            break;
        case ISP_FUNC_AWB:
            *tmp = int_val_awb;
            #if ISP_GET_FUNC_DBUG
            printk("AWB%x\n",*tmp);
            #endif /* ISP_GET_FUNC_DBUG */
            break;
        default:
            /* return EINVAL */
            ret_data = (-EINVAL);
            break;
    }

end:
    return ret_data;
}
