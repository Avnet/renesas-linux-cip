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

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/delay.h>
#else
#include "r_typedefs.h"
#endif
#include <linux/drpai.h>
#include "drpai-core.h"
#include "drpai-reg.h"
#include <linux/reset.h>

#define DRP_ERRINT_MSK_REG_NUM              (6)
#define DRP_ERRINT_STATUS_REG_NUM           (6)
#define AIMAC_ERRINT_MSK_REG_NUM            (34)
#define AIMAC_ERRINT_STATUS_REG_NUM         (35)
#define SET_STPC_CLKGEN_CTRL                (0x00000001)
#define SET_STPC_CLKGEN_RST                 (0x00000000)
#define SET_STPC_CLKGEN_STBYWAIT_EN         (0x00000001)
#define SET_STPC_CLKGEN_STBYWAIT_DI         (0x00000000)
#define SET_STPC_CLKGEN_DIV                 (0x00020001)
#define SET_STPC_CLKE_EN                    (0x100F030F)
#define SET_STPC_CLKE_DI                    (0x00000000)
#define SET_STPC_SFTRST_EN                  (0xFFFFFFFF)
#define SET_STPC_SFTRST_DI                  (0x21F000F0)
#define SET_DSCC_DCTL_CR                    (0x00000000)
#define SET_DSCC_DCTL                       (0x00310001)
#define SET_EXD0_STPC_CLKGEN_CTRL           (0x00000001)
#define SET_EXD0_STPC_CLKGEN_RST            (0x00000000)
#define SET_EXD0_STPC_CLKGEN_STBYWAI_EN     (0x00000001)
#define SET_EXD0_STPC_CLKGEN_STBYWAI_DI     (0x00000000)
#define SET_EXD0_STPC_CLKGEN_DIV            (0x00010000)
#define SET_EXDX_STPC_CLKE_EN               (0x100F000F)
#define SET_EXDX_STPC_CLKE_DI               (0x00000000)
#define SET_CLKRSTCON_CLKE_EN               (0x00000001)
#define SET_CLKRSTCON_CLKE_DI               (0x00000000)
#define SET_EXDX_STPC_SFTRST_EN             (0xB1FF03FF)
#define SET_EXDX_STPC_SFTRST_DI             (0xA1F003F0)
#define SET_CLKRSTCON_SFTRST_EN             (0x0000001F)
#define SET_CLKRSTCON_SFTRST_DI             (0x00000000)
#define SET_EXD1_ODIF_INTMSK                (0x0300FFF7)
#define SET_AID_DSCC_DCTL_CR                (0x00000000)
#define SET_AID_DSCC_DCTL                   (0x00000001)
#define SET_SYNCTBL_TBLX                    (0x0000FFFF)
#define SET_IDMACIF_DSC_EN                  (0x00040001)
#define SET_IDMACIF_MEMR_EN                 (0x00040101)
#define SET_DSCC_DMA_EN                     (0x00000001)
#define SET_DSCC_DMA_DI                     (0x00000000)
#define DRPAI_RESERVED_DSCC_PAMON           (0)
#define DRPAI_RESERVED_AID_DSCC_PAMON       (1)
#define DRPAI_RESERVED_STPC_ERRINT_STS      (2)
#define DRPAI_RESERVED_INTMON_ERRINT        (3)
#define DRPAI_RESERVED_EXD1_ODIF_INT_IRQ    (4)
#define DRPAI_RESERVED_EXD1_ODIF_INT_NOW    (5)
#define DRPAI_RESERVED_SYNCTBL_TBL12        (6)
#define DRPAI_RESERVED_SYNCTBL_TBL13        (7)
#define DRPAI_RESERVED_SYNCTBL_TBL14        (8)
#define DRPAI_RESERVED_SYNCTBL_TBL15        (9)

/* Base address */
extern void __iomem *g_drp_base_addr[DRP_CH_NUM];
extern void __iomem *g_aimac_base_address[AIMAC_CH_NUM];
extern struct drpai_priv *drpai_priv;

static int32_t drp_init(int32_t ch);
static int32_t drp_start(int32_t ch, uint32_t desc);
static int32_t aimac_init(int32_t ch);
static int32_t aimac_start(int32_t ch, uint32_t desc);
static void reg_bit_clear(volatile void __iomem *reg_address, uint32_t bit);
static void aimac_clear_synctbl_tbl(int32_t ch);
static int8_t check_dma_reg_stop(void __iomem *base, uint32_t offset);
static int8_t check_dma_stop(void __iomem *base, uint32_t *offset, uint32_t num_offset);
static int8_t wait_for_dma_stop(void __iomem *base, uint32_t *offset, uint32_t num_offset);
static int8_t wait_for_desc_prefetch_stop(void __iomem *base, uint32_t offset);

static uint32_t exd0_odif_int_val;
static uint32_t stpc_errint_sts_val;
static uint32_t intmon_errint_val;

/* DRP ERRINT */
const static uint32_t drp_errint_msk_reg_tbl[DRP_ERRINT_MSK_REG_NUM][2] =
{
    {IDIF_EINTMSK       ,0x00F0F0F0}, {IDIF_EINTMSK_DSC ,0x00000000},
    {ODIF_EINTMSK       ,0x07F0F0F0}, {IDMAC_INTME      ,0x55550000},
    {ODMAC_INTME        ,0x5555FF80}, {RAC_EINTM        ,0x00000000},
};
const static uint32_t drp_errint_status_reg_tbl[DRP_ERRINT_STATUS_REG_NUM] =
{
    IDIF_EINT, IDIF_EINT_DSC, ODIF_EINT, IDMAC_INTSE,
    ODMAC_INTSE, RAC_EINTS
};
const static char* drp_errint_status_reg_name_tbl[DRP_ERRINT_STATUS_REG_NUM] =
{
    "IDIF_EINT","IDIF_EINT_DSC","ODIF_EINT","IDMAC_INTSE",
    "ODMAC_INTSE","RAC_EINTS"
};
/* AI-MAC ERRINT */
const static uint32_t aimac_errint_msk_reg_tbl[AIMAC_ERRINT_MSK_REG_NUM][2] =
{
    {AID_IDIF_EINTMSK   ,0x07FEFEFE}, {AID_IDMAC_INTME      ,0x55550000},
    {AIMRAC_EINTM       ,0x00000000}, {CMDSEL_ERRMSK        ,0x00000000},
    {PRAM_INTMASK       ,0x00000000}, {OSEL_DO_MSK0         ,0x00000000},
    {OSEL_DO_MSK1       ,0x00000000}, {OSEL_DO_MSK2         ,0x00000000},
    {OSEL_DO_MSK3       ,0x00000000}, {OSEL_DO_MSK4         ,0x00000000},
    {OSEL_DO_MSK5       ,0x00000000}, {OSEL_DO_MSK6         ,0x00000000},
    {OSEL_DO_MSK7       ,0x00000000}, {OSEL_DO_MSK8         ,0x00000000},
    {OSEL_DO_EN0        ,0x00000001}, {OSEL_DO_EN1          ,0x00000001},
    {OSEL_DO_EN2        ,0x00000001}, {OSEL_DO_EN3          ,0x00000001},
    {OSEL_DO_EN4        ,0x00000001}, {OSEL_DO_EN5          ,0x00000001},
    {OSEL_DO_EN6        ,0x00000001}, {OSEL_DO_EN7          ,0x00000001},
    {OSEL_DO_EN8        ,0x00000001}, {MACTOP_ERR_MSK       ,0x00000000},
    {EXD0_IDIF_EINTMSK  ,0x07F0F0F0}, {EXD1_IDIF_EINTMSK    ,0x07F0F0F0},
    {EXD0_ODIF_EINTMSK  ,0x07F0F0F0}, {EXD1_ODIF_EINTMSK    ,0x07F0F0F0},
    {EXD0_IDMAC_INTME   ,0x55550000}, {EXD1_IDMAC_INTME     ,0x55550000},
    {EXD0_ODMAC_INTME   ,0x55550000}, {EXD1_ODMAC_INTME     ,0x55550000},
    {EXD0_RAC_EINTM     ,0x00000000}, {EXD1_RAC_EINTM       ,0x00000000},
};
const static uint32_t aimac_errint_status_reg_tbl[AIMAC_ERRINT_STATUS_REG_NUM] =
{
    AID_IDIF_EINT,AID_IDMAC_INTSE,AIMRAC_EINTS,CMDSEL_ERRSTS,PRAM_INT,
    OSEL_DO_ESTS0,OSEL_DO_ESTS1,OSEL_DO_ESTS2,OSEL_DO_ESTS3,OSEL_DO_ESTS4,
    OSEL_DO_ESTS5,OSEL_DO_ESTS6,OSEL_DO_ESTS7,OSEL_DO_ESTS8,OSEL_DO_FESTS0,
    OSEL_DO_FESTS1,OSEL_DO_FESTS2,OSEL_DO_FESTS3,OSEL_DO_FESTS4,OSEL_DO_FESTS5,
    OSEL_DO_FESTS6,OSEL_DO_FESTS7,OSEL_DO_FESTS8,MACTOP_ERR_STS,MACCTL_FERR_STS,
    EXD0_IDIF_EINT,EXD1_IDIF_EINT,EXD0_ODIF_EINT,EXD1_ODIF_EINT,EXD0_IDMAC_INTSE,
    EXD1_IDMAC_INTSE,EXD0_ODMAC_INTSE,EXD1_ODMAC_INTSE,EXD0_RAC_EINTS,EXD1_RAC_EINTS
};
const static char* aimac_errint_status_reg_name_tbl[AIMAC_ERRINT_STATUS_REG_NUM] =
{
    "AID_IDIF_EINT","AID_IDMAC_INTSE","AIMRAC_EINTS","CMDSEL_ERRSTS","PRAM_INT",
    "OSEL_DO_ESTS0","OSEL_DO_ESTS1","OSEL_DO_ESTS2","OSEL_DO_ESTS3","OSEL_DO_ESTS4",
    "OSEL_DO_ESTS5","OSEL_DO_ESTS6","OSEL_DO_ESTS7","OSEL_DO_ESTS8","OSEL_DO_FESTS0",
    "OSEL_DO_FESTS1","OSEL_DO_FESTS2","OSEL_DO_FESTS3","OSEL_DO_FESTS4","OSEL_DO_FESTS5",
    "OSEL_DO_FESTS6","OSEL_DO_FESTS7","OSEL_DO_FESTS8","MACTOP_ERR_STS","MACCTL_FERR_STS",
    "EXD0_IDIF_EINT","EXD1_IDIF_EINT","EXD0_ODIF_EINT","EXD1_ODIF_EINT","EXD0_IDMAC_INTSE",
    "EXD1_IDMAC_INTSE","EXD0_ODMAC_INTSE","EXD1_ODMAC_INTSE","EXD0_RAC_EINTS","EXD1_RAC_EINTS"
};

static int32_t drp_init(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);
    
    int32_t ret;
    uint32_t loop;
    unsigned long flags;
    struct drpai_priv *priv = drpai_priv;

    if (DRP_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    /* Enable DRP clock */
    iowrite32(SET_STPC_CLKGEN_CTRL, g_drp_base_addr[ch] + STPC_CLKGEN_CTRL);

    /* Unreset the DRPCLKGEN module */
    iowrite32(SET_STPC_CLKGEN_RST, g_drp_base_addr[ch] + STPC_CLKGEN_RST);

    /* Shift to standby mode */
    iowrite32(SET_STPC_CLKGEN_STBYWAIT_EN, g_drp_base_addr[ch] + STPC_CLKGEN_STBYWAIT);

    /* DRP clock operating frequency setting */
    /* Div divided by 4 (DCLK=315MHz), set to dynamic frequency mode */
    /* Div divided by 4 (DCLK=315MHz) -> When Config is loaded       */
    /* Dynamic frequency mode -> When DRP application is running     */
    iowrite32(SET_STPC_CLKGEN_DIV, g_drp_base_addr[ch] + STPC_CLKGEN_DIV);
    iowrite32(SET_STPC_CLKGEN_STBYWAIT_DI, g_drp_base_addr[ch] + STPC_CLKGEN_STBYWAIT);

    /* Enable DMA channel clock */
    iowrite32(SET_STPC_CLKE_EN, g_drp_base_addr[ch] + STPC_CLKE);

    /* Release Soft reset */
    spin_lock_irqsave(&priv->lock, flags);
    iowrite32(SET_STPC_SFTRST_DI, g_drp_base_addr[ch] + STPC_SFTRST);
    spin_unlock_irqrestore(&priv->lock, flags);

    /* Data input channel settings */
    iowrite32(SET_IDMACIF_DSC_EN, g_drp_base_addr[ch] + IDIF_DMACTLI0);
    iowrite32(SET_IDMACIF_MEMR_EN, g_drp_base_addr[ch] + IDIF_DMACTLI1);
    iowrite32(SET_IDMACIF_MEMR_EN, g_drp_base_addr[ch] + IDIF_DMACTLI2);
    iowrite32(SET_IDMACIF_MEMR_EN, g_drp_base_addr[ch] + IDIF_DMACTLI3);

    /* Data output channel settings */
    iowrite32(SET_IDMACIF_MEMR_EN, g_drp_base_addr[ch] + ODIF_DMACTLO0);
    iowrite32(SET_IDMACIF_MEMR_EN, g_drp_base_addr[ch] + ODIF_DMACTLO1);
    iowrite32(SET_IDMACIF_MEMR_EN, g_drp_base_addr[ch] + ODIF_DMACTLO2);
    iowrite32(SET_IDMACIF_MEMR_EN, g_drp_base_addr[ch] + ODIF_DMACTLO3);

    /* Configuration light */
    iowrite32(SET_IDMACIF_DSC_EN, g_drp_base_addr[ch] + IDIF_DMACTLCW);

/* ISP */
    /* Unmasks ODMAC interrupts */
    iowrite32(0xFFFFFFF0, g_drp_base_addr[ch] + ODIF_INTMSK);
/* ISP */

    /* DRP error interrupt mask release */
    for (loop = 0; loop < DRP_ERRINT_MSK_REG_NUM; loop++)
    {
        iowrite32(drp_errint_msk_reg_tbl[loop][1],
               g_drp_base_addr[ch] + drp_errint_msk_reg_tbl[loop][0]);
    }

    ret =  R_DRPAI_SUCCESS;
    goto end;

err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;

end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

static int32_t drp_start(int32_t ch, uint32_t desc)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;

    if (DRP_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

/* ISP */
    uint32_t reg_val;

    /* Clear interrupt factor */
    reg_val = readl(g_drp_base_addr[ch] + ODIF_INT);
    writel(reg_val, g_drp_base_addr[ch] + ODIF_INT);            /* Clear */
    reg_val = readl(g_drp_base_addr[ch] + ODIF_INT);            /* Dummy read */

    /* Reading the number of interrupts */
    reg_val = readl(g_drp_base_addr[ch] + ODIF_INTCNTO0);
    reg_val = readl(g_drp_base_addr[ch] + ODIF_INTCNTO1);
    reg_val = readl(g_drp_base_addr[ch] + ODIF_INTCNTO2);
    reg_val = readl(g_drp_base_addr[ch] + ODIF_INTCNTO3);

    /* Unmasks ODMAC interrupts */
    iowrite32(0xFFFFFFF0, g_drp_base_addr[ch] + ODIF_INTMSK);
/* ISP */

    /* Set descriptor start address */
    iowrite32(desc, g_drp_base_addr[ch] + DSCC_DPA);

    /* Clear prefetch of input descriptor */
    iowrite32(SET_DSCC_DCTL_CR, g_drp_base_addr[ch] + DSCC_DCTL);

    /* Start prefetch of input descriptor */
    iowrite32(SET_DSCC_DCTL, g_drp_base_addr[ch] + DSCC_DCTL);

    ret =  R_DRPAI_SUCCESS;
    goto end;

err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;

end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

static int32_t aimac_init(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;
    uint32_t loop;

    if (AIMAC_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    /* Unreset MCLKGEN module */
    iowrite32(SET_EXD0_STPC_CLKGEN_RST, g_aimac_base_address[ch] + EXD0_STPC_CLKGEN_RST);

    /* Shift to standby mode */
    iowrite32(SET_EXD0_STPC_CLKGEN_STBYWAI_EN, g_aimac_base_address[ch] + EXD0_STPC_CLKGEN_STBYWAIT);

    /* AIMAC clock operating frequency setting */
    /* Div divided by 2 (MCLK=630MHz), set to fixed frequency mode */
    iowrite32(SET_EXD0_STPC_CLKGEN_DIV, g_aimac_base_address[ch] + EXD0_STPC_CLKGEN_DIV);

    /* Clock activation */
    iowrite32(SET_EXD0_STPC_CLKGEN_STBYWAI_DI, g_aimac_base_address[ch] + EXD0_STPC_CLKGEN_STBYWAIT);

    /* Enable Clock */
    iowrite32(SET_EXDX_STPC_CLKE_EN, g_aimac_base_address[ch] + EXD0_STPC_CLKE);
    iowrite32(SET_EXDX_STPC_CLKE_EN, g_aimac_base_address[ch] + EXD1_STPC_CLKE);
    iowrite32(SET_CLKRSTCON_CLKE_EN, g_aimac_base_address[ch] + CLKRSTCON_CLKE);

    /* Release soft reset */
    iowrite32(SET_EXDX_STPC_SFTRST_DI, g_aimac_base_address[ch] + EXD0_STPC_SFTRST);
    iowrite32(SET_EXDX_STPC_SFTRST_DI, g_aimac_base_address[ch] + EXD1_STPC_SFTRST);
    iowrite32(SET_CLKRSTCON_SFTRST_DI, g_aimac_base_address[ch] + CLKRSTCON_SFTRST);

    /* DMA channel settings */
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI0);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI1);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI2);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI3);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO0);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO1);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO2);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO3);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI0);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI1);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI2);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI3);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO0);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO1);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO2);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO3);
    iowrite32(SET_IDMACIF_MEMR_EN, g_aimac_base_address[ch] + AID_IDIF_DMACTLI0);

    /* DRP-AI processing completion interrupt mask release */
    iowrite32(SET_EXD1_ODIF_INTMSK, g_aimac_base_address[ch] + EXD1_ODIF_INTMSK);

    /* AI-MAC error interrupt mask release */
    for (loop = 0; loop < AIMAC_ERRINT_MSK_REG_NUM; loop++)
    {
        iowrite32(aimac_errint_msk_reg_tbl[loop][1],
               g_aimac_base_address[ch] + aimac_errint_msk_reg_tbl[loop][0]);
    }


    ret =  R_DRPAI_SUCCESS;
    goto end;

err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;

end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

static int32_t aimac_start(int32_t ch, uint32_t desc)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;
    unsigned long flags;
    struct drpai_priv *priv = drpai_priv;

    if (AIMAC_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    /* Initialization of register value storage variable */
    spin_lock_irqsave(&priv->lock, flags);
    exd0_odif_int_val = 0;
    spin_unlock_irqrestore(&priv->lock, flags);

    /* Set the start address of AIMAC descriptor */
    iowrite32(desc, g_aimac_base_address[ch] + AID_DSCC_DPA);

    /* Start descriptor read */
    iowrite32(SET_AID_DSCC_DCTL_CR, g_aimac_base_address[ch] + AID_DSCC_DCTL);
    iowrite32(SET_AID_DSCC_DCTL, g_aimac_base_address[ch] + AID_DSCC_DCTL);

    ret =  R_DRPAI_SUCCESS;
    goto end;

err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;

end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

int32_t R_DRPAI_DRP_Open(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;

    if (DRP_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    ret = drp_init(ch);
    if (R_DRPAI_SUCCESS != ret)
    {
        return ret;
    }

    ret =  R_DRPAI_SUCCESS;
    goto end;
err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

int32_t R_DRPAI_DRP_Start(int32_t ch, uint32_t desc)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;

    if (DRP_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    ret = drp_start(ch, desc);
    if (R_DRPAI_SUCCESS != ret)
    {
        return ret;
    }

    ret =  R_DRPAI_SUCCESS;
    goto end;
err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

/* ISP */
int32_t R_DRPAI_DRP_Nmlint(int32_t ch, drpai_odif_intcnto_t *odif_intcnto)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    uint32_t reg_val;

    /* Clear interrupt factor */
    reg_val = readl(g_drp_base_addr[ch] + ODIF_INT);
    writel(reg_val, g_drp_base_addr[ch] + ODIF_INT);            /* Clear */
    reg_val = readl(g_drp_base_addr[ch] + ODIF_INT);            /* Dummy read */

    /* Reading the number of interrupts */
    odif_intcnto->ch0 = readl(g_drp_base_addr[ch] + ODIF_INTCNTO0);
    odif_intcnto->ch1 = readl(g_drp_base_addr[ch] + ODIF_INTCNTO1);
    odif_intcnto->ch2 = readl(g_drp_base_addr[ch] + ODIF_INTCNTO2);
    odif_intcnto->ch3 = readl(g_drp_base_addr[ch] + ODIF_INTCNTO3);

    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return 0;
}
/* ISP */

int32_t R_DRPAI_DRP_Errint(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    uint32_t reg_val;
    uint32_t loop;

    printk(KERN_ERR "DRP-AI DRP ERROR\n");

    /* Show descriptor pointer */
    reg_val = ioread32(g_drp_base_addr[ch] + DSCC_PAMON);
    printk(KERN_ERR "DSCC_PAMON      : 0x%08X\n", reg_val);
    reg_val = ioread32(g_aimac_base_address[ch] + AID_DSCC_PAMON);
    printk(KERN_ERR "AID_DSCC_PAMON  : 0x%08X\n", reg_val);

    /* Error interrupt cause register */
    stpc_errint_sts_val = ioread32(g_drp_base_addr[ch] + STPC_ERRINT_STS);
    printk(KERN_ERR "STPC_ERRINT_STS : 0x%08X\n", stpc_errint_sts_val);

    /* Error display of each module */
    reg_val = ioread32(g_drp_base_addr[ch] + DRP_ERRINTSTATUS);
    printk(KERN_ERR "DRP_ERRINTSTATUS : 0x%08X\n", reg_val);
    if (0 != reg_val)
    {
        reg_val = ioread32(g_drp_base_addr[ch] + STPC_SFTRST);
        reg_val |= DRPAI_BIT31;
        iowrite32(reg_val, g_drp_base_addr[ch] + STPC_SFTRST);
    }
    for (loop = 0; loop < DRP_ERRINT_STATUS_REG_NUM; loop++)
    {
        reg_val = ioread32(g_drp_base_addr[ch] + drp_errint_status_reg_tbl[loop]);
        iowrite32(reg_val, g_drp_base_addr[ch] + drp_errint_status_reg_tbl[loop]);
        printk(KERN_ERR "%s : 0x%08X\n", drp_errint_status_reg_name_tbl[loop], reg_val);
    }
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return 0;
}

int32_t R_DRPAI_AIMAC_Open(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;

    if (AIMAC_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    ret = aimac_init(ch);
    if (R_DRPAI_SUCCESS != ret)
    {
        return ret;
    }

    ret =  R_DRPAI_SUCCESS;
    goto end;
err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

int32_t R_DRPAI_AIMAC_Start(int32_t ch, uint32_t desc)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;

    if (AIMAC_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    ret = aimac_start(ch, desc);
    if (R_DRPAI_SUCCESS != ret)
    {
        return ret;
    }

    ret =  R_DRPAI_SUCCESS;
    goto end;
err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

int32_t R_DRPAI_AIMAC_Nmlint(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);
    volatile uint32_t dummy;

    /* Debug information */
    exd0_odif_int_val = ioread32(g_aimac_base_address[ch] + EXD1_ODIF_INT);

    /* Clear interrupt factor */
    iowrite32(0x00000008, g_aimac_base_address[ch] + EXD1_ODIF_INT);   /* Clear */
    dummy = ioread32(g_aimac_base_address[ch] + EXD1_ODIF_INT);        /* Dummy read */

    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return 0;
}

int32_t R_DRPAI_AIMAC_Errint(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    uint32_t reg_val;
    uint8_t i;

    printk(KERN_ERR "DRP-AI AI-MAC ERROR\n");

    /* Show descriptor pointer */
    reg_val = ioread32(g_drp_base_addr[ch] + DSCC_PAMON);
    printk(KERN_ERR "DSCC_PAMON      : 0x%08X\n", reg_val);
    reg_val = ioread32(g_aimac_base_address[ch] + AID_DSCC_PAMON);
    printk(KERN_ERR "AID_DSCC_PAMON  : 0x%08X\n", reg_val);

    /* Error interrupt cause register */
    intmon_errint_val = ioread32(g_aimac_base_address[ch] + INTMON_ERRINT);
    printk(KERN_ERR "INTMON_ERRINT : 0x%08X\n", intmon_errint_val);

    for (i = 0; i < AIMAC_ERRINT_STATUS_REG_NUM; i++)
    {
        reg_val = ioread32(g_aimac_base_address[ch] + aimac_errint_status_reg_tbl[i]);
        iowrite32(reg_val, g_aimac_base_address[ch] + aimac_errint_status_reg_tbl[i]);
        printk(KERN_ERR "%s : 0x%08X\n",aimac_errint_status_reg_name_tbl[i], reg_val);
    }

    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);
    return 0;
}

int32_t R_DRPAI_Status(int32_t ch, drpai_status_t *drpai_status)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;

    if (DRP_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }
    if (AIMAC_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    /* DRP Address of descriptor */
    drpai_status->reserved[DRPAI_RESERVED_DSCC_PAMON] = ioread32(g_drp_base_addr[ch] + DSCC_PAMON);

    /* AI-MAC Address of descriptor */
    drpai_status->reserved[DRPAI_RESERVED_AID_DSCC_PAMON] = ioread32(g_aimac_base_address[ch] + AID_DSCC_PAMON);

    /* DRP-AI processing complete interrupt status */
    drpai_status->reserved[DRPAI_RESERVED_EXD1_ODIF_INT_IRQ] = exd0_odif_int_val;
    drpai_status->reserved[DRPAI_RESERVED_EXD1_ODIF_INT_NOW] = ioread32(g_aimac_base_address[ch] + EXD1_ODIF_INT);

    /* AI-MAC synchronization information */
    drpai_status->reserved[DRPAI_RESERVED_SYNCTBL_TBL12] = ioread32(g_aimac_base_address[ch] + SYNCTBL_TBL12);
    drpai_status->reserved[DRPAI_RESERVED_SYNCTBL_TBL13] = ioread32(g_aimac_base_address[ch] + SYNCTBL_TBL13);
    drpai_status->reserved[DRPAI_RESERVED_SYNCTBL_TBL14] = ioread32(g_aimac_base_address[ch] + SYNCTBL_TBL14);
    drpai_status->reserved[DRPAI_RESERVED_SYNCTBL_TBL15] = ioread32(g_aimac_base_address[ch] + SYNCTBL_TBL15);

    /* DRP error information */
    drpai_status->reserved[DRPAI_RESERVED_STPC_ERRINT_STS] = stpc_errint_sts_val;

    /* AI-MAC error information */
    drpai_status->reserved[DRPAI_RESERVED_INTMON_ERRINT] = intmon_errint_val;

    ret = R_DRPAI_SUCCESS;
    goto end;

err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

static void reg_bit_clear(volatile void __iomem *reg_address, uint32_t bit)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);
    uint32_t tmp_reg;

    /* Read register value */
    tmp_reg = ioread32(reg_address);
    /* Clear the target bit */
    tmp_reg = tmp_reg & (~bit);

    iowrite32(tmp_reg, reg_address);

    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);
}

static void aimac_clear_synctbl_tbl(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL0);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL1);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL2);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL3);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL4);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL5);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL6);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL7);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL8);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL9);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL10);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL11);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL12);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL13);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL14);
    iowrite32(0x0000FFFF, g_aimac_base_address[ch] + SYNCTBL_TBL15);

    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);
}

static int8_t check_dma_reg_stop(void __iomem *base, uint32_t offset)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    uint32_t reg_val;
    int8_t ret;

    reg_val = ioread32(base + offset);
    if(!((0 == (reg_val & DRPAI_BIT1)) || (DRPAI_BIT19 == (reg_val & DRPAI_BIT19))))
    {
        ret = -1;
    }
    else
    {
        ret = 0;
    }

    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);
    return ret;
}

static int8_t check_dma_stop(void __iomem *base, uint32_t *offset, uint32_t num_offset)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t i;
    int32_t ret;

    for(i = 0; i < num_offset; i++)
    {
        if(0 != check_dma_reg_stop(base, offset[i]))
        {
            goto not_stop;
        }
        else
        {
            ; // Do nothing
        }
    }

    ret = 0;
    goto end;
not_stop:
    ret = -1;
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

static int8_t wait_for_dma_stop(void __iomem *base, uint32_t *offset, uint32_t num_offset)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    bool is_stop = false;
    int8_t ret;
    int32_t i;

    i = 0;
    while((RST_MAX_TIMEOUT > i) && (false == is_stop))
    {
        udelay(1);
        i++;
        if(0 != check_dma_stop(base, offset, num_offset))
        {
            ; // Do nothing
        }
        else
        {
            is_stop = true;
            break;
        }
    }

    i = 0;
    while((RST_MAX_TIMEOUT > i) && (false == is_stop))
    {
        usleep_range(100, 200);
        i++;
        if(0 != check_dma_stop(base, offset, num_offset))
        {
            ; // Do nothing
        }
        else
        {
            is_stop = true;
            break;
        }
    }

    if(true == is_stop)
    {
        ret = 0;
    }
    else
    {
        ret = -1;
#ifdef  DRPAI_DRV_DEBUG
        uint32_t reg_val;
        for(i = 0; i < num_offset; i++) {
            reg_val = ioread32(base + offset[i]);
            DRPAI_DEBUG_PRINT(KERN_INFO "%s: offset: %08X = 0x%08X\n", __func__, offset[i], reg_val);
        }
#endif
    }
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

static int8_t wait_for_desc_prefetch_stop(void __iomem *base, uint32_t offset)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    bool is_stop = false;
    int8_t ret;
    int32_t i;
    uint32_t reg_val;

    i = 0;
    while((RST_MAX_TIMEOUT > i) && (false == is_stop))
    {
        udelay(1);
        i++;
        reg_val = ioread32(base + offset);
        if(0 != (reg_val & DRPAI_BIT1))
        {
            ; // Do nothing
        }
        else
        {
            is_stop = true;
            break;
        }
    }

    i = 0;
    while((RST_MAX_TIMEOUT > i) && (false == is_stop))
    {
        usleep_range(100, 200);
        i++;
        reg_val = ioread32(base + offset);
        if(0 != (reg_val & DRPAI_BIT1))
        {
            ; // Do nothing
        }
        else
        {
            is_stop = true;
            break;
        }
    }

    if(true == is_stop)
    {
        ret = 0;
    }
    else
    {
        ret = -1;
        DRPAI_DEBUG_PRINT(KERN_INFO "%s: offset: %08X = 0x%08X\n", __func__, offset, reg_val);
    }
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

int32_t R_DRPAI_DRP_Reset(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;
    uint32_t offset_buf[4];
    unsigned long flags;
    struct drpai_priv *priv = drpai_priv;

    DRPAI_DEBUG_PRINT(KERN_INFO "%s: (pid %d)\n", __func__, current->pid);
    
    if (DRP_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    /* Descriptor prefetch stop */
    reg_bit_clear(g_drp_base_addr[ch] + DSCC_DCTL, DRPAI_BIT0);
    if(0 != wait_for_desc_prefetch_stop(g_drp_base_addr[ch], DSCC_DCTL))
    {
        goto err_reset;
    }
    
    /* Forced stop of writing configuration data */
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLCW, DRPAI_BIT18);
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLCW, DRPAI_BIT0);
    offset_buf[0] = IDIF_DMACTLCW;
    if(0 != wait_for_dma_stop(g_drp_base_addr[ch], offset_buf, 1))
    {
        goto err_reset;
    }

    /* Forced stop of data input / output */
    /* IDIF_DMACTLI0,I1,I2,I3 */
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLI0, DRPAI_BIT18);
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLI1, DRPAI_BIT18);
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLI2, DRPAI_BIT18);
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLI3, DRPAI_BIT18);

    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLI0, DRPAI_BIT0);
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLI1, DRPAI_BIT0);
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLI2, DRPAI_BIT0);
    reg_bit_clear(g_drp_base_addr[ch] + IDIF_DMACTLI3, DRPAI_BIT0);

    aimac_clear_synctbl_tbl(ch);

    offset_buf[0] = IDIF_DMACTLI0;
    offset_buf[1] = IDIF_DMACTLI1;
    offset_buf[2] = IDIF_DMACTLI2;
    offset_buf[3] = IDIF_DMACTLI3;
    if(0 != wait_for_dma_stop(g_drp_base_addr[ch], offset_buf, 4)){
        goto err_reset;
    }

    /* ODIF_DMACTLO0,O1,O2,O3 */
    reg_bit_clear(g_drp_base_addr[ch] + ODIF_DMACTLO0, DRPAI_BIT18);
    reg_bit_clear(g_drp_base_addr[ch] + ODIF_DMACTLO1, DRPAI_BIT18);
    reg_bit_clear(g_drp_base_addr[ch] + ODIF_DMACTLO2, DRPAI_BIT18);
    reg_bit_clear(g_drp_base_addr[ch] + ODIF_DMACTLO3, DRPAI_BIT18);

    reg_bit_clear(g_drp_base_addr[ch] + ODIF_DMACTLO0, DRPAI_BIT0);
    reg_bit_clear(g_drp_base_addr[ch] + ODIF_DMACTLO1, DRPAI_BIT0);
    reg_bit_clear(g_drp_base_addr[ch] + ODIF_DMACTLO2, DRPAI_BIT0);
    reg_bit_clear(g_drp_base_addr[ch] + ODIF_DMACTLO3, DRPAI_BIT0);

    aimac_clear_synctbl_tbl(ch);

    offset_buf[0] = ODIF_DMACTLO0;
    offset_buf[1] = ODIF_DMACTLO1;
    offset_buf[2] = ODIF_DMACTLO2;
    offset_buf[3] = ODIF_DMACTLO3;
    if(0 != wait_for_dma_stop(g_drp_base_addr[ch], offset_buf, 4))
    {
        goto err_reset;
    }

    /* Set DRP core as fixed frequency mode  */
    iowrite32((SET_STPC_CLKGEN_DIV & 0xFFFFFFFE), g_drp_base_addr[ch] + STPC_CLKGEN_DIV);

    /* Soft reset */
    spin_lock_irqsave(&priv->lock, flags);
    iowrite32(0xFFFFFFFF, g_drp_base_addr[ch] + STPC_SFTRST);
    spin_unlock_irqrestore(&priv->lock, flags);

    ret =  R_DRPAI_SUCCESS;
    goto end;
err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;
err_reset:
    ret = R_DRPAI_ERR_RESET;
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

int32_t R_DRPAI_AIMAC_Reset(int32_t ch)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;
    uint32_t offset_buf[4];

    DRPAI_DEBUG_PRINT(KERN_INFO "%s: (pid %d)\n", __func__, current->pid);

    if (DRP_CH_NUM <= ch)
    {
        goto err_invalid_arg;
    }

    /* Descriptor prefetch stop */
    reg_bit_clear(g_aimac_base_address[ch] + AID_DSCC_DCTL, DRPAI_BIT0);
    if(0 != wait_for_desc_prefetch_stop(g_aimac_base_address[ch], AID_DSCC_DCTL))
    {
        goto err_reset;
    }

    /* Forced stop of inputting parameters (weights and bias values) */
    reg_bit_clear(g_aimac_base_address[ch] + AID_IDIF_DMACTLI0, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + AID_IDIF_DMACTLI0, DRPAI_BIT0);

    aimac_clear_synctbl_tbl(ch);

    offset_buf[0] = AID_IDIF_DMACTLI0;
    if(0 != wait_for_dma_stop(g_aimac_base_address[ch], offset_buf, 1))
    {
        goto err_reset;
    }

    /* Forced stop of data input / output */
    /* EXD0_IDIF_DMACTLI0,I1,I2,I3 */
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI0, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI1, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI2, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI3, DRPAI_BIT18);

    reg_bit_clear(g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI0, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI1, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI2, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_IDIF_DMACTLI3, DRPAI_BIT0);

    aimac_clear_synctbl_tbl(ch);

    offset_buf[0] = EXD0_IDIF_DMACTLI0;
    offset_buf[1] = EXD0_IDIF_DMACTLI1;
    offset_buf[2] = EXD0_IDIF_DMACTLI2;
    offset_buf[3] = EXD0_IDIF_DMACTLI3;
    if(0 != wait_for_dma_stop(g_aimac_base_address[ch], offset_buf, 4))
    {
        goto err_reset;
    }

    /* EXD1_IDIF_DMACTLI0,I1,I2,I3 */
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI0, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI1, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI2, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI3, DRPAI_BIT18);

    reg_bit_clear(g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI0, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI1, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI2, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_IDIF_DMACTLI3, DRPAI_BIT0);

    aimac_clear_synctbl_tbl(ch);

    offset_buf[0] = EXD1_IDIF_DMACTLI0;
    offset_buf[1] = EXD1_IDIF_DMACTLI1;
    offset_buf[2] = EXD1_IDIF_DMACTLI2;
    offset_buf[3] = EXD1_IDIF_DMACTLI3;
    if(0 != wait_for_dma_stop(g_aimac_base_address[ch], offset_buf, 4))
    {
        goto err_reset;
    }

    /* EXD0_ODIF_DMACTLO0,O1,O2,O3 */
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO0, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO1, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO2, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO3, DRPAI_BIT18);

    reg_bit_clear(g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO0, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO1, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO2, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD0_ODIF_DMACTLO3, DRPAI_BIT0);

    aimac_clear_synctbl_tbl(ch);

    offset_buf[0] = EXD0_ODIF_DMACTLO0;
    offset_buf[1] = EXD0_ODIF_DMACTLO1;
    offset_buf[2] = EXD0_ODIF_DMACTLO2;
    offset_buf[3] = EXD0_ODIF_DMACTLO3;
    if(0 != wait_for_dma_stop(g_aimac_base_address[ch], offset_buf, 4))
    {
        goto err_reset;
    }

    /* EXD1_ODIF_DMACTLO0,O1,O2,O3 */
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO0, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO1, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO2, DRPAI_BIT18);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO3, DRPAI_BIT18);

    reg_bit_clear(g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO0, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO1, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO2, DRPAI_BIT0);
    reg_bit_clear(g_aimac_base_address[ch] + EXD1_ODIF_DMACTLO3, DRPAI_BIT0);

    aimac_clear_synctbl_tbl(ch);

    offset_buf[0] = EXD1_ODIF_DMACTLO0;
    offset_buf[1] = EXD1_ODIF_DMACTLO1;
    offset_buf[2] = EXD1_ODIF_DMACTLO2;
    offset_buf[3] = EXD1_ODIF_DMACTLO3;
    if(0 != wait_for_dma_stop(g_aimac_base_address[ch], offset_buf, 4))
    {
        goto err_reset;
    }

    /* Soft reset */
    iowrite32(0xB1FF03FF, g_aimac_base_address[ch] + EXD0_STPC_SFTRST);
    iowrite32(0xB1FF03FF, g_aimac_base_address[ch] + EXD1_STPC_SFTRST);
    iowrite32(0x0000001F, g_aimac_base_address[ch] + CLKRSTCON_SFTRST);

    /* Stop clock */
    iowrite32(0x00000000, g_aimac_base_address[ch] + EXD0_STPC_CLKE);
    iowrite32(0x00000000, g_aimac_base_address[ch] + EXD1_STPC_CLKE);
    iowrite32(0x00000000, g_aimac_base_address[ch] + CLKRSTCON_CLKE);

    /* Stop MCLKGEN */
    iowrite32(0x00000001, g_aimac_base_address[ch] + EXD0_STPC_CLKGEN_STBYWAIT);

    ret =  R_DRPAI_SUCCESS;
    goto end;

err_invalid_arg:
    ret = R_DRPAI_ERR_INVALID_ARG;
    goto end;
err_reset:
    ret = R_DRPAI_ERR_RESET;
    goto end;
end:
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);

    return ret;
}

int32_t R_DRPAI_CPG_Reset(void)
{
    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) start.\n", __func__, current->pid);

    int32_t ret;
    int32_t i = 0;
    int r_data;
    struct drpai_priv *priv = drpai_priv;
    bool is_stop = false;
	
	/* Access reset controller interface */
	reset_control_assert(priv->rstc);
    reset_control_deassert(priv->rstc);

	/* Check reset status */
    i = 0;
    while((RST_MAX_TIMEOUT > i) && (false == is_stop))
    {
        udelay(1);
        i++;
		r_data = reset_control_status(priv->rstc);
        if(1 == r_data)
        {
            is_stop = true;
            break;
        }
    }

    i = 0;
    while((RST_MAX_TIMEOUT > i) && (false == is_stop))
    {
        usleep_range(100, 200);
        i++;
		r_data = reset_control_status(priv->rstc);
        if(1 == r_data)
        {
            is_stop = true;
            break;
        }
    }

    if(true == is_stop)
    {
        ret =  R_DRPAI_SUCCESS;
    }
    else
    {
        ret = R_DRPAI_ERR_RESET;
        DRPAI_DEBUG_PRINT(KERN_INFO "%s: CPG Reset failed. Reset Control Status: %d\n", __func__,  r_data);
    }

    DRPAI_DEBUG_PRINT(KERN_INFO "[%s](pid %d) end.\n", __func__, current->pid);
    return ret;
}


MODULE_DESCRIPTION("RZ/V2L DRP-AI driver");
MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_LICENSE("GPL v2");
