/*
 * Driver for the Renesas RZ/V2L DRP-AI unit
 *
 * Copyright (C) 2021 Renesas Electronics Corporation
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

#ifndef R_DRPAI_CORE_H
#define R_DRPAI_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/semaphore.h>
#else /* __KERNEL__ */
#include "r_typedefs.h"
#endif /* __KERNEL__ */

#define CH0                             (0)
#define CH1                             (1)
#define CH2                             (2)
#define CH3                             (3)
#define DRP_CH_NUM                      (1)
#define AIMAC_CH_NUM                    (1)
#define MAC256_CH_NUM                   (4)

/* Error code */
#define R_DRPAI_SUCCESS                 (0)
#define R_DRPAI_ERR_INVALID_ARG         (-1)
#define R_DRPAI_ERR_RESET               (-2)

#define RST_CPG_WAIT (10)
#define RST_MAX_TIMEOUT (100)

// #define   DRPAI_DRV_DEBUG
#ifdef  DRPAI_DRV_DEBUG
#define DRPAI_DEBUG_PRINT(...)      printk(__VA_ARGS__)
#else
#define DRPAI_DEBUG_PRINT(...)
#endif

/* Type definitions */
struct drpai_priv {
    struct platform_device *pdev;
    const char *dev_name;
    drpai_status_t drpai_status;
    spinlock_t lock;
    struct semaphore sem;
    refcount_t count;
    void __iomem *drp_base;
    void __iomem *aimac_base;
    struct clk *clk_int;
    struct clk *clk_aclk_drp;
    struct clk *clk_mclk;
    struct clk *clk_dclkin;
    struct clk *clk_aclk;
    struct reset_control *rstc;
    uint32_t aimac_irq_flag;
/* ISP */
    void (*isp_finish_loc)(int);
/* ISP */
};

/* ISP */
typedef struct drpai_odif_intcnto
{
    uint32_t    ch0;
    uint32_t    ch1;
    uint32_t    ch2;
    uint32_t    ch3;
} drpai_odif_intcnto_t;
/* ISP */

int32_t R_DRPAI_DRP_Open(int32_t ch);
int32_t R_DRPAI_DRP_Start(int32_t ch, uint32_t desc);
/* ISP */
int32_t R_DRPAI_DRP_Nmlint(int32_t ch, drpai_odif_intcnto_t *odif_intcnto);
/* ISP */
int32_t R_DRPAI_DRP_Errint(int32_t ch);
int32_t R_DRPAI_AIMAC_Open(int32_t ch);
int32_t R_DRPAI_AIMAC_Start(int32_t ch, uint32_t desc);
int32_t R_DRPAI_AIMAC_Nmlint(int32_t ch);
int32_t R_DRPAI_AIMAC_Errint(int32_t ch);
int32_t R_DRPAI_Status(int32_t ch, drpai_status_t *drpai_status);
int32_t R_DRPAI_DRP_Reset(int32_t ch);
int32_t R_DRPAI_AIMAC_Reset(int32_t ch);
int32_t R_DRPAI_CPG_Reset(void);

#endif /* R_DRPAI_CORE_H */
