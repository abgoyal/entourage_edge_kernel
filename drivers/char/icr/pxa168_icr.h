/*
 *  linux/drivers/icr/pxa168_icr.h
 *
 *  Copyright:	(C) Copyright 2009 Marvell International Ltd.
 *
 *  Author: Alan Guenther <>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 *
 */

/*
 * ICR hardware control register addresses
 */

#ifndef __PXA168_ICR_H
#define __PXA168_ICR_H

#define ICR_CONTROL_REG_BASE_ADDR    0xC0802000
#define ICR_BASE_ADDR                0
#define ICR_DMA_BASE_ADDR            ICR_BASE_ADDR
#define ICR_SVF0RGB_STRT_ADDR_REG    (ICR_DMA_BASE_ADDR+0x0)	//CFG_DMA_SA_RGB0
#define ICR_SVF1RGB_STRT_ADDR_REG    (ICR_DMA_BASE_ADDR+0x4)	//CFG_DMA_SA_RGB1
#define ICR_SVF0_SIZ_REG             (ICR_DMA_BASE_ADDR+0x8)
#define ICR_SVF1_SIZ_REG             (ICR_DMA_BASE_ADDR+0xc)
#define ICR_DF0DISP_STRT_ADDR_REG    (ICR_DMA_BASE_ADDR+0x10)
#define ICR_DF1DISP_STRT_ADDR_REG    (ICR_DMA_BASE_ADDR+0x14)
#define ICR_DF0DISP_SIZE_REG         (ICR_DMA_BASE_ADDR+0x18)
#define ICR_DF1DISP_SIZE_REG         (ICR_DMA_BASE_ADDR+0x1c)
#define ICR_DF0DISP_SPSCR_REG        (ICR_DMA_BASE_ADDR+0x20)
#define ICR_DF1DISP_SPSCR_REG        (ICR_DMA_BASE_ADDR+0x24)
#define ICR_DF0_VID_SIZ_REG          (ICR_DMA_BASE_ADDR+0x28)
#define ICR_DF1_VID_SIZ_REG          (ICR_DMA_BASE_ADDR+0x2C)
#define ICR_RX_DMA_CTRL0_REG         (ICR_DMA_BASE_ADDR+0x30)
#define ICR_RX_DMA_CTRL1_REG         (ICR_DMA_BASE_ADDR+0x34)
#define ICR_TX_DMA_CTRL0_REG         (ICR_DMA_BASE_ADDR+0x38)
#define ICR_TX_DMA_CTRL1_REG         (ICR_DMA_BASE_ADDR+0x3c)
#define ICR_DMA_GCR_REG              (ICR_DMA_BASE_ADDR+0x40)
#define ICR_INTR_MASK_REG            (ICR_DMA_BASE_ADDR+0x44)
#define ICR_INTR_CLR_SEL_REG         (ICR_DMA_BASE_ADDR+0x48)
#define ICR_INTR_STAT_MASK_REG       (ICR_DMA_BASE_ADDR+0x4c)
#define ICR_INTR_STATUS_REG          (ICR_DMA_BASE_ADDR+0x50)
//#define ICR_MEM_CF_CFG_REG           (ICR_DMA_BASE_ADDR+0x40)
#define ICR_SRAM_WTC_RTC_REG         (ICR_DMA_BASE_ADDR+0x54)
//#define ICR_INTR_CTRL_REG            (ICR_DMA_BASE_ADDR+0x48)
//#define ICR_INTR_STAT_REG            (ICR_DMA_BASE_ADDR+0x4c)
#define ICR_DBL_BUF_TRG_REG          (ICR_DMA_BASE_ADDR+0x58)

#define ICR_ICSC_BASE_ADDR           ICR_BASE_ADDR+0x400
#define ICR_ICSC_M_C0_L              (ICR_ICSC_BASE_ADDR+4*0x00)
#define ICR_ICSC_M_C0_H              (ICR_ICSC_BASE_ADDR+4*0x01)
#define ICR_ICSC_M_C1_L              (ICR_ICSC_BASE_ADDR+4*0x02)
#define ICR_ICSC_M_C1_H              (ICR_ICSC_BASE_ADDR+4*0x03)
#define ICR_ICSC_M_C2_L              (ICR_ICSC_BASE_ADDR+4*0x04)
#define ICR_ICSC_M_C2_H              (ICR_ICSC_BASE_ADDR+4*0x05)
#define ICR_ICSC_M_C3_L              (ICR_ICSC_BASE_ADDR+4*0x06)
#define ICR_ICSC_M_C3_H              (ICR_ICSC_BASE_ADDR+4*0x07)
#define ICR_ICSC_M_C4_L              (ICR_ICSC_BASE_ADDR+4*0x08)
#define ICR_ICSC_M_C4_H              (ICR_ICSC_BASE_ADDR+4*0x09)
#define ICR_ICSC_M_C5_L              (ICR_ICSC_BASE_ADDR+4*0x0A)
#define ICR_ICSC_M_C5_H              (ICR_ICSC_BASE_ADDR+4*0x0B)
#define ICR_ICSC_M_C6_L              (ICR_ICSC_BASE_ADDR+4*0x0C)
#define ICR_ICSC_M_C6_H              (ICR_ICSC_BASE_ADDR+4*0x0D)
#define ICR_ICSC_M_C7_L              (ICR_ICSC_BASE_ADDR+4*0x0E)
#define ICR_ICSC_M_C7_H              (ICR_ICSC_BASE_ADDR+4*0x0F)
#define ICR_ICSC_M_C8_L              (ICR_ICSC_BASE_ADDR+4*0x10)
#define ICR_ICSC_M_C8_H              (ICR_ICSC_BASE_ADDR+4*0x11)
#define ICR_ICSC_M_O1_0              (ICR_ICSC_BASE_ADDR+4*0x14)
#define ICR_ICSC_M_O1_1              (ICR_ICSC_BASE_ADDR+4*0x15)
#define ICR_ICSC_M_O1_2              (ICR_ICSC_BASE_ADDR+4*0x16)
#define ICR_ICSC_M_O2_0              (ICR_ICSC_BASE_ADDR+4*0x18)
#define ICR_ICSC_M_O2_1              (ICR_ICSC_BASE_ADDR+4*0x19)
#define ICR_ICSC_M_O2_2              (ICR_ICSC_BASE_ADDR+4*0x1A)
#define ICR_ICSC_M_O3_0              (ICR_ICSC_BASE_ADDR+4*0x1C)
#define ICR_ICSC_M_O3_1              (ICR_ICSC_BASE_ADDR+4*0x1D)
#define ICR_ICSC_M_O3_2              (ICR_ICSC_BASE_ADDR+4*0x1E)
#define ICR_ICSC_P_C0_L              (ICR_ICSC_BASE_ADDR+4*0x20)
#define ICR_ICSC_P_C0_H              (ICR_ICSC_BASE_ADDR+4*0x21)
#define ICR_ICSC_P_C1_L              (ICR_ICSC_BASE_ADDR+4*0x22)
#define ICR_ICSC_P_C1_H              (ICR_ICSC_BASE_ADDR+4*0x23)
#define ICR_ICSC_P_C2_L              (ICR_ICSC_BASE_ADDR+4*0x24)
#define ICR_ICSC_P_C2_H              (ICR_ICSC_BASE_ADDR+4*0x25)
#define ICR_ICSC_P_C3_L              (ICR_ICSC_BASE_ADDR+4*0x26)
#define ICR_ICSC_P_C3_H              (ICR_ICSC_BASE_ADDR+4*0x27)
#define ICR_ICSC_P_C4_L              (ICR_ICSC_BASE_ADDR+4*0x28)
#define ICR_ICSC_P_C4_H              (ICR_ICSC_BASE_ADDR+4*0x29)
#define ICR_ICSC_P_C5_L              (ICR_ICSC_BASE_ADDR+4*0x2A)
#define ICR_ICSC_P_C5_H              (ICR_ICSC_BASE_ADDR+4*0x2B)
#define ICR_ICSC_P_C6_L              (ICR_ICSC_BASE_ADDR+4*0x2C)
#define ICR_ICSC_P_C6_H              (ICR_ICSC_BASE_ADDR+4*0x2D)
#define ICR_ICSC_P_C7_L              (ICR_ICSC_BASE_ADDR+4*0x2E)
#define ICR_ICSC_P_C7_H              (ICR_ICSC_BASE_ADDR+4*0x2F)
#define ICR_ICSC_P_C8_L              (ICR_ICSC_BASE_ADDR+4*0x30)
#define ICR_ICSC_P_C8_H              (ICR_ICSC_BASE_ADDR+4*0x31)
#define ICR_ICSC_P_O1_0              (ICR_ICSC_BASE_ADDR+4*0x34)
#define ICR_ICSC_P_O1_1              (ICR_ICSC_BASE_ADDR+4*0x35)
#define ICR_ICSC_P_O1_2              (ICR_ICSC_BASE_ADDR+4*0x36)
#define ICR_ICSC_P_O2_0              (ICR_ICSC_BASE_ADDR+4*0x38)
#define ICR_ICSC_P_O2_1              (ICR_ICSC_BASE_ADDR+4*0x39)
#define ICR_ICSC_P_O2_2              (ICR_ICSC_BASE_ADDR+4*0x3A)
#define ICR_ICSC_P_O3_0              (ICR_ICSC_BASE_ADDR+4*0x3C)
#define ICR_ICSC_P_O3_1              (ICR_ICSC_BASE_ADDR+4*0x3D)
#define ICR_ICSC_P_O3_2              (ICR_ICSC_BASE_ADDR+4*0x3E)

#define ICR_FTDC_M_EN                (ICR_ICSC_BASE_ADDR+4*0xA0)
#define ICR_FTDC_P_EN                (ICR_ICSC_BASE_ADDR+4*0xA1)
#define ICR_FTDC_INLOW_L             (ICR_ICSC_BASE_ADDR+4*0xA2)
#define ICR_FTDC_INLOW_H             (ICR_ICSC_BASE_ADDR+4*0xA3)
#define ICR_FTDC_INHIGH_L            (ICR_ICSC_BASE_ADDR+4*0xA4)
#define ICR_FTDC_INHIGH_H            (ICR_ICSC_BASE_ADDR+4*0xA5)
#define ICR_FTDC_OUTLOW_L            (ICR_ICSC_BASE_ADDR+4*0xA6)
#define ICR_FTDC_OUTLOW_H            (ICR_ICSC_BASE_ADDR+4*0xA7)
#define ICR_FTDC_OUTHIGH_L           (ICR_ICSC_BASE_ADDR+4*0xA8)
#define ICR_FTDC_OUTHIGH_H           (ICR_ICSC_BASE_ADDR+4*0xA9)
#define ICR_FTDC_YLOW                (ICR_ICSC_BASE_ADDR+4*0xAA)
#define ICR_FTDC_YHIGH               (ICR_ICSC_BASE_ADDR+4*0xAB)
#define ICR_FTDC_CH1                 (ICR_ICSC_BASE_ADDR+4*0xAC)
#define ICR_FTDC_CH2_L               (ICR_ICSC_BASE_ADDR+4*0xAE)
#define ICR_FTDC_CH2_H               (ICR_ICSC_BASE_ADDR+4*0xAF)
#define ICR_FTDC_CH3_L               (ICR_ICSC_BASE_ADDR+4*0xB0)
#define ICR_FTDC_CH3_H               (ICR_ICSC_BASE_ADDR+4*0xB1)
#define ICR_FTDC_1_C00               (ICR_ICSC_BASE_ADDR+4*0xB2)
#define ICR_FTDC_2_C00               (ICR_ICSC_BASE_ADDR+4*0xB3)
#define ICR_FTDC_3_C00               (ICR_ICSC_BASE_ADDR+4*0xB4)
#define ICR_FTDC_4_C00               (ICR_ICSC_BASE_ADDR+4*0xB5)
#define ICR_FTDC_5_C00               (ICR_ICSC_BASE_ADDR+4*0xB6)
#define ICR_FTDC_6_C00               (ICR_ICSC_BASE_ADDR+4*0xB7)
#define ICR_FTDC_1_C01               (ICR_ICSC_BASE_ADDR+4*0xB8)
#define ICR_FTDC_2_C01               (ICR_ICSC_BASE_ADDR+4*0xB9)
#define ICR_FTDC_3_C01               (ICR_ICSC_BASE_ADDR+4*0xBA)
#define ICR_FTDC_4_C01               (ICR_ICSC_BASE_ADDR+4*0xBB)
#define ICR_FTDC_5_C01               (ICR_ICSC_BASE_ADDR+4*0xBC)
#define ICR_FTDC_6_C01               (ICR_ICSC_BASE_ADDR+4*0xBD)
#define ICR_FTDC_1_C11               (ICR_ICSC_BASE_ADDR+4*0xBE)
#define ICR_FTDC_2_C11               (ICR_ICSC_BASE_ADDR+4*0xBF)
#define ICR_FTDC_3_C11               (ICR_ICSC_BASE_ADDR+4*0xC0)
#define ICR_FTDC_4_C11               (ICR_ICSC_BASE_ADDR+4*0xC1)
#define ICR_FTDC_5_C11               (ICR_ICSC_BASE_ADDR+4*0xC2)
#define ICR_FTDC_6_C11               (ICR_ICSC_BASE_ADDR+4*0xC3)
#define ICR_FTDC_1_C10               (ICR_ICSC_BASE_ADDR+4*0xC4)
#define ICR_FTDC_2_C10               (ICR_ICSC_BASE_ADDR+4*0xC5)
#define ICR_FTDC_3_C10               (ICR_ICSC_BASE_ADDR+4*0xC6)
#define ICR_FTDC_4_C10               (ICR_ICSC_BASE_ADDR+4*0xC7)
#define ICR_FTDC_5_C10               (ICR_ICSC_BASE_ADDR+4*0xC8)
#define ICR_FTDC_6_C10               (ICR_ICSC_BASE_ADDR+4*0xC9)
#define ICR_FTDC_1_OFF00             (ICR_ICSC_BASE_ADDR+4*0xCA)
#define ICR_FTDC_2_OFF00             (ICR_ICSC_BASE_ADDR+4*0xCB)
#define ICR_FTDC_3_OFF00             (ICR_ICSC_BASE_ADDR+4*0xCC)
#define ICR_FTDC_4_OFF00             (ICR_ICSC_BASE_ADDR+4*0xCD)
#define ICR_FTDC_5_OFF00             (ICR_ICSC_BASE_ADDR+4*0xCE)
#define ICR_FTDC_6_OFF00             (ICR_ICSC_BASE_ADDR+4*0xCF)
#define ICR_FTDC_1_OFF10             (ICR_ICSC_BASE_ADDR+4*0xD0)
#define ICR_FTDC_2_OFF10             (ICR_ICSC_BASE_ADDR+4*0xD1)
#define ICR_FTDC_3_OFF10             (ICR_ICSC_BASE_ADDR+4*0xD2)
#define ICR_FTDC_4_OFF10             (ICR_ICSC_BASE_ADDR+4*0xD3)
#define ICR_FTDC_5_OFF10             (ICR_ICSC_BASE_ADDR+4*0xD4)
#define ICR_FTDC_6_OFF10             (ICR_ICSC_BASE_ADDR+4*0xD5)

#define ICR_HSC_BASE_ADDR            ICR_BASE_ADDR+0x800
#define ICR_HS_M_EN                  (ICR_HSC_BASE_ADDR+4*0x00)
#define ICR_HS_M_AX1_L               (ICR_HSC_BASE_ADDR+4*0x02)
#define ICR_HS_M_AX1_H               (ICR_HSC_BASE_ADDR+4*0x03)
#define ICR_HS_M_AX2_L               (ICR_HSC_BASE_ADDR+4*0x04)
#define ICR_HS_M_AX2_H               (ICR_HSC_BASE_ADDR+4*0x05)
#define ICR_HS_M_AX3_L               (ICR_HSC_BASE_ADDR+4*0x06)
#define ICR_HS_M_AX3_H               (ICR_HSC_BASE_ADDR+4*0x07)
#define ICR_HS_M_AX4_L               (ICR_HSC_BASE_ADDR+4*0x08)
#define ICR_HS_M_AX4_H               (ICR_HSC_BASE_ADDR+4*0x09)
#define ICR_HS_M_AX5_L               (ICR_HSC_BASE_ADDR+4*0x0A)
#define ICR_HS_M_AX5_H               (ICR_HSC_BASE_ADDR+4*0x0B)
#define ICR_HS_M_AX6_L               (ICR_HSC_BASE_ADDR+4*0x0C)
#define ICR_HS_M_AX6_H               (ICR_HSC_BASE_ADDR+4*0x0D)
#define ICR_HS_M_AX7_L               (ICR_HSC_BASE_ADDR+4*0x0E)
#define ICR_HS_M_AX7_H               (ICR_HSC_BASE_ADDR+4*0x0F)
#define ICR_HS_M_AX8_L               (ICR_HSC_BASE_ADDR+4*0x10)
#define ICR_HS_M_AX8_H               (ICR_HSC_BASE_ADDR+4*0x11)
#define ICR_HS_M_AX9_L               (ICR_HSC_BASE_ADDR+4*0x12)
#define ICR_HS_M_AX9_H               (ICR_HSC_BASE_ADDR+4*0x13)
#define ICR_HS_M_AX10_L              (ICR_HSC_BASE_ADDR+4*0x14)
#define ICR_HS_M_AX10_H              (ICR_HSC_BASE_ADDR+4*0x15)
#define ICR_HS_M_AX11_L              (ICR_HSC_BASE_ADDR+4*0x16)
#define ICR_HS_M_AX11_H              (ICR_HSC_BASE_ADDR+4*0x17)
#define ICR_HS_M_AX12_L              (ICR_HSC_BASE_ADDR+4*0x18)
#define ICR_HS_M_AX12_H              (ICR_HSC_BASE_ADDR+4*0x19)
#define ICR_HS_M_AX13_L              (ICR_HSC_BASE_ADDR+4*0x1A)
#define ICR_HS_M_AX13_H              (ICR_HSC_BASE_ADDR+4*0x1B)
#define ICR_HS_M_AX14_L              (ICR_HSC_BASE_ADDR+4*0x1C)
#define ICR_HS_M_AX14_H              (ICR_HSC_BASE_ADDR+4*0x1D)
#define ICR_HS_M_H1                 (ICR_HSC_BASE_ADDR+4*0x1E)
#define ICR_HS_M_H2                 (ICR_HSC_BASE_ADDR+4*0x1F)
#define ICR_HS_M_H3                 (ICR_HSC_BASE_ADDR+4*0x20)
#define ICR_HS_M_H4                 (ICR_HSC_BASE_ADDR+4*0x21)
#define ICR_HS_M_H5                 (ICR_HSC_BASE_ADDR+4*0x22)
#define ICR_HS_M_H6                 (ICR_HSC_BASE_ADDR+4*0x23)
#define ICR_HS_M_H7                 (ICR_HSC_BASE_ADDR+4*0x24)
#define ICR_HS_M_H8                 (ICR_HSC_BASE_ADDR+4*0x25)
#define ICR_HS_M_H9                 (ICR_HSC_BASE_ADDR+4*0x26)
#define ICR_HS_M_H10                 (ICR_HSC_BASE_ADDR+4*0x27)
#define ICR_HS_M_H11                 (ICR_HSC_BASE_ADDR+4*0x28)
#define ICR_HS_M_H12                 (ICR_HSC_BASE_ADDR+4*0x29)
#define ICR_HS_M_H13                 (ICR_HSC_BASE_ADDR+4*0x2A)
#define ICR_HS_M_H14                 (ICR_HSC_BASE_ADDR+4*0x2B)
#define ICR_HS_M_S1                 (ICR_HSC_BASE_ADDR+4*0x2C)
#define ICR_HS_M_S2                 (ICR_HSC_BASE_ADDR+4*0x2D)
#define ICR_HS_M_S3                 (ICR_HSC_BASE_ADDR+4*0x2E)
#define ICR_HS_M_S4                 (ICR_HSC_BASE_ADDR+4*0x2F)
#define ICR_HS_M_S5                 (ICR_HSC_BASE_ADDR+4*0x30)
#define ICR_HS_M_S6                 (ICR_HSC_BASE_ADDR+4*0x31)
#define ICR_HS_M_S7                 (ICR_HSC_BASE_ADDR+4*0x32)
#define ICR_HS_M_S8                 (ICR_HSC_BASE_ADDR+4*0x33)
#define ICR_HS_M_S9                 (ICR_HSC_BASE_ADDR+4*0x34)
#define ICR_HS_M_S10                 (ICR_HSC_BASE_ADDR+4*0x35)
#define ICR_HS_M_S11                 (ICR_HSC_BASE_ADDR+4*0x36)
#define ICR_HS_M_S12                 (ICR_HSC_BASE_ADDR+4*0x37)
#define ICR_HS_M_S13                 (ICR_HSC_BASE_ADDR+4*0x38)
#define ICR_HS_M_S14                 (ICR_HSC_BASE_ADDR+4*0x39)
#define ICR_HS_M_GL                 (ICR_HSC_BASE_ADDR+4*0x3A)
#define ICR_HS_M_MAXSAT_RGB_Y_L             (ICR_HSC_BASE_ADDR+4*0x3C)
#define ICR_HS_M_MAXSAT_RGB_Y_H             (ICR_HSC_BASE_ADDR+4*0x3D)
#define ICR_HS_M_MAXSAT_RCR_L             (ICR_HSC_BASE_ADDR+4*0x3E)
#define ICR_HS_M_MAXSAT_RCR_H             (ICR_HSC_BASE_ADDR+4*0x3F)
#define ICR_HS_M_MAXSAT_RCB_L             (ICR_HSC_BASE_ADDR+4*0x40)
#define ICR_HS_M_MAXSAT_RCB_H             (ICR_HSC_BASE_ADDR+4*0x41)
#define ICR_HS_M_MAXSAT_GCR_L             (ICR_HSC_BASE_ADDR+4*0x42)
#define ICR_HS_M_MAXSAT_GCR_H             (ICR_HSC_BASE_ADDR+4*0x43)
#define ICR_HS_M_MAXSAT_GCB_L             (ICR_HSC_BASE_ADDR+4*0x44)
#define ICR_HS_M_MAXSAT_GCB_H             (ICR_HSC_BASE_ADDR+4*0x45)
#define ICR_HS_M_MAXSAT_BCR_L             (ICR_HSC_BASE_ADDR+4*0x46)
#define ICR_HS_M_MAXSAT_BCR_H             (ICR_HSC_BASE_ADDR+4*0x47)
#define ICR_HS_M_MAXSAT_BCB_L             (ICR_HSC_BASE_ADDR+4*0x48)
#define ICR_HS_M_MAXSAT_BCB_H             (ICR_HSC_BASE_ADDR+4*0x49)
#define ICR_HS_M_ROFF_L                 (ICR_HSC_BASE_ADDR+4*0x4A)
#define ICR_HS_M_ROFF_H                 (ICR_HSC_BASE_ADDR+4*0x4B)
#define ICR_HS_M_GOFF_L                 (ICR_HSC_BASE_ADDR+4*0x4C)
#define ICR_HS_M_GOFF_H                 (ICR_HSC_BASE_ADDR+4*0x4D)
#define ICR_HS_M_BOFF_L                 (ICR_HSC_BASE_ADDR+4*0x4E)
#define ICR_HS_M_BOFF_H                 (ICR_HSC_BASE_ADDR+4*0x4F)
#define ICR_HS_P_EN                    (ICR_HSC_BASE_ADDR+4*0x50)
#define ICR_HS_P_AX1_L                 (ICR_HSC_BASE_ADDR+4*0x52)
#define ICR_HS_P_AX1_H                 (ICR_HSC_BASE_ADDR+4*0x53)
#define ICR_HS_P_AX2_L                 (ICR_HSC_BASE_ADDR+4*0x54)
#define ICR_HS_P_AX2_H                 (ICR_HSC_BASE_ADDR+4*0x55)
#define ICR_HS_P_AX3_L                 (ICR_HSC_BASE_ADDR+4*0x56)
#define ICR_HS_P_AX3_H                 (ICR_HSC_BASE_ADDR+4*0x57)
#define ICR_HS_P_AX4_L                 (ICR_HSC_BASE_ADDR+4*0x58)
#define ICR_HS_P_AX4_H                 (ICR_HSC_BASE_ADDR+4*0x59)
#define ICR_HS_P_AX5_L                 (ICR_HSC_BASE_ADDR+4*0x5A)
#define ICR_HS_P_AX5_H                 (ICR_HSC_BASE_ADDR+4*0x5B)
#define ICR_HS_P_AX6_L                 (ICR_HSC_BASE_ADDR+4*0x5C)
#define ICR_HS_P_AX6_H                 (ICR_HSC_BASE_ADDR+4*0x5D)
#define ICR_HS_P_AX7_L                 (ICR_HSC_BASE_ADDR+4*0x5E)
#define ICR_HS_P_AX7_H                 (ICR_HSC_BASE_ADDR+4*0x5F)
#define ICR_HS_P_AX8_L                 (ICR_HSC_BASE_ADDR+4*0x60)
#define ICR_HS_P_AX8_H                 (ICR_HSC_BASE_ADDR+4*0x61)
#define ICR_HS_P_AX9_L                 (ICR_HSC_BASE_ADDR+4*0x62)
#define ICR_HS_P_AX9_H                 (ICR_HSC_BASE_ADDR+4*0x63)
#define ICR_HS_P_AX10_L                 (ICR_HSC_BASE_ADDR+4*0x64)
#define ICR_HS_P_AX10_H                 (ICR_HSC_BASE_ADDR+4*0x65)
#define ICR_HS_P_AX11_L                 (ICR_HSC_BASE_ADDR+4*0x66)
#define ICR_HS_P_AX11_H                 (ICR_HSC_BASE_ADDR+4*0x67)
#define ICR_HS_P_AX12_L                 (ICR_HSC_BASE_ADDR+4*0x68)
#define ICR_HS_P_AX12_H                 (ICR_HSC_BASE_ADDR+4*0x69)
#define ICR_HS_P_AX13_L                 (ICR_HSC_BASE_ADDR+4*0x6A)
#define ICR_HS_P_AX13_H                 (ICR_HSC_BASE_ADDR+4*0x6B)
#define ICR_HS_P_AX14_L                 (ICR_HSC_BASE_ADDR+4*0x6C)
#define ICR_HS_P_AX14_H                 (ICR_HSC_BASE_ADDR+4*0x6D)
#define ICR_HS_P_H1                 (ICR_HSC_BASE_ADDR+4*0x6E)
#define ICR_HS_P_H2                 (ICR_HSC_BASE_ADDR+4*0x6F)
#define ICR_HS_P_H3                 (ICR_HSC_BASE_ADDR+4*0x70)
#define ICR_HS_P_H4                 (ICR_HSC_BASE_ADDR+4*0x71)
#define ICR_HS_P_H5                 (ICR_HSC_BASE_ADDR+4*0x72)
#define ICR_HS_P_H6                 (ICR_HSC_BASE_ADDR+4*0x73)
#define ICR_HS_P_H7                 (ICR_HSC_BASE_ADDR+4*0x74)
#define ICR_HS_P_H8                 (ICR_HSC_BASE_ADDR+4*0x75)
#define ICR_HS_P_H9                 (ICR_HSC_BASE_ADDR+4*0x76)
#define ICR_HS_P_H10                 (ICR_HSC_BASE_ADDR+4*0x77)
#define ICR_HS_P_H11                 (ICR_HSC_BASE_ADDR+4*0x78)
#define ICR_HS_P_H12                 (ICR_HSC_BASE_ADDR+4*0x79)
#define ICR_HS_P_H13                 (ICR_HSC_BASE_ADDR+4*0x7A)
#define ICR_HS_P_H14                 (ICR_HSC_BASE_ADDR+4*0x7B)
#define ICR_HS_P_S1                 (ICR_HSC_BASE_ADDR+4*0x7C)
#define ICR_HS_P_S2                 (ICR_HSC_BASE_ADDR+4*0x7D)
#define ICR_HS_P_S3                 (ICR_HSC_BASE_ADDR+4*0x7E)
#define ICR_HS_P_S4                 (ICR_HSC_BASE_ADDR+4*0x7F)
#define ICR_HS_P_S5                 (ICR_HSC_BASE_ADDR+4*0x80)
#define ICR_HS_P_S6                 (ICR_HSC_BASE_ADDR+4*0x81)
#define ICR_HS_P_S7                 (ICR_HSC_BASE_ADDR+4*0x82)
#define ICR_HS_P_S8                 (ICR_HSC_BASE_ADDR+4*0x83)
#define ICR_HS_P_S9                 (ICR_HSC_BASE_ADDR+4*0x84)
#define ICR_HS_P_S10                 (ICR_HSC_BASE_ADDR+4*0x85)
#define ICR_HS_P_S11                 (ICR_HSC_BASE_ADDR+4*0x86)
#define ICR_HS_P_S12                 (ICR_HSC_BASE_ADDR+4*0x87)
#define ICR_HS_P_S13                 (ICR_HSC_BASE_ADDR+4*0x88)
#define ICR_HS_P_S14                 (ICR_HSC_BASE_ADDR+4*0x89)
#define ICR_HS_P_GL                 (ICR_HSC_BASE_ADDR+4*0x8A)
#define ICR_HS_P_MAXSAT_RGB_Y_L             (ICR_HSC_BASE_ADDR+4*0x8C)
#define ICR_HS_P_MAXSAT_RGB_Y_H             (ICR_HSC_BASE_ADDR+4*0x8D)
#define ICR_HS_P_MAXSAT_RCR_L             (ICR_HSC_BASE_ADDR+4*0x8E)
#define ICR_HS_P_MAXSAT_RCR_H             (ICR_HSC_BASE_ADDR+4*0x8F)
#define ICR_HS_P_MAXSAT_RCB_L             (ICR_HSC_BASE_ADDR+4*0x90)
#define ICR_HS_P_MAXSAT_RCB_H             (ICR_HSC_BASE_ADDR+4*0x91)
#define ICR_HS_P_MAXSAT_GCR_L             (ICR_HSC_BASE_ADDR+4*0x92)
#define ICR_HS_P_MAXSAT_GCR_H             (ICR_HSC_BASE_ADDR+4*0x93)
#define ICR_HS_P_MAXSAT_GCB_L             (ICR_HSC_BASE_ADDR+4*0x94)
#define ICR_HS_P_MAXSAT_GCB_H             (ICR_HSC_BASE_ADDR+4*0x95)
#define ICR_HS_P_MAXSAT_BCR_L             (ICR_HSC_BASE_ADDR+4*0x96)
#define ICR_HS_P_MAXSAT_BCR_H             (ICR_HSC_BASE_ADDR+4*0x97)
#define ICR_HS_P_MAXSAT_BCB_L             (ICR_HSC_BASE_ADDR+4*0x98)
#define ICR_HS_P_MAXSAT_BCB_H             (ICR_HSC_BASE_ADDR+4*0x99)
#define ICR_HS_P_ROFF_L                 (ICR_HSC_BASE_ADDR+4*0x9A)
#define ICR_HS_P_ROFF_H                 (ICR_HSC_BASE_ADDR+4*0x9B)
#define ICR_HS_P_GOFF_L                 (ICR_HSC_BASE_ADDR+4*0x9C)
#define ICR_HS_P_GOFF_H                 (ICR_HSC_BASE_ADDR+4*0x9D)
#define ICR_HS_P_BOFF_L                 (ICR_HSC_BASE_ADDR+4*0x9E)
#define ICR_HS_P_BOFF_H                 (ICR_HSC_BASE_ADDR+4*0x9F)

#define ICR_GCSC_BASE_ADDR           ICR_HSC_BASE_ADDR+0x0
#define ICR_GCSC_M_C0_L              (ICR_GCSC_BASE_ADDR+4*0xA0)
#define ICR_GCSC_M_C0_H              (ICR_GCSC_BASE_ADDR+4*0xA1)
#define ICR_GCSC_M_C1_L              (ICR_GCSC_BASE_ADDR+4*0xA2)
#define ICR_GCSC_M_C1_H              (ICR_GCSC_BASE_ADDR+4*0xA3)
#define ICR_GCSC_M_C2_L              (ICR_GCSC_BASE_ADDR+4*0xA4)
#define ICR_GCSC_M_C2_H              (ICR_GCSC_BASE_ADDR+4*0xA5)
#define ICR_GCSC_M_C3_L              (ICR_GCSC_BASE_ADDR+4*0xA6)
#define ICR_GCSC_M_C3_H              (ICR_GCSC_BASE_ADDR+4*0xA7)
#define ICR_GCSC_M_C4_L              (ICR_GCSC_BASE_ADDR+4*0xA8)
#define ICR_GCSC_M_C4_H              (ICR_GCSC_BASE_ADDR+4*0xA9)
#define ICR_GCSC_M_C5_L              (ICR_GCSC_BASE_ADDR+4*0xAA)
#define ICR_GCSC_M_C5_H              (ICR_GCSC_BASE_ADDR+4*0xAB)
#define ICR_GCSC_M_C6_L              (ICR_GCSC_BASE_ADDR+4*0xAC)
#define ICR_GCSC_M_C6_H              (ICR_GCSC_BASE_ADDR+4*0xAD)
#define ICR_GCSC_M_C7_L              (ICR_GCSC_BASE_ADDR+4*0xAE)
#define ICR_GCSC_M_C7_H              (ICR_GCSC_BASE_ADDR+4*0xAF)
#define ICR_GCSC_M_C8_L              (ICR_GCSC_BASE_ADDR+4*0xB0)
#define ICR_GCSC_M_C8_H              (ICR_GCSC_BASE_ADDR+4*0xB1)
#define ICR_GCSC_M_O1_0              (ICR_GCSC_BASE_ADDR+4*0xB4)
#define ICR_GCSC_M_O1_1              (ICR_GCSC_BASE_ADDR+4*0xB5)
#define ICR_GCSC_M_O1_2              (ICR_GCSC_BASE_ADDR+4*0xB6)
#define ICR_GCSC_M_O2_0              (ICR_GCSC_BASE_ADDR+4*0xB8)
#define ICR_GCSC_M_O2_1              (ICR_GCSC_BASE_ADDR+4*0xB9)
#define ICR_GCSC_M_O2_2              (ICR_GCSC_BASE_ADDR+4*0xBA)
#define ICR_GCSC_M_O3_0              (ICR_GCSC_BASE_ADDR+4*0xBC)
#define ICR_GCSC_M_O3_1              (ICR_GCSC_BASE_ADDR+4*0xBD)
#define ICR_GCSC_M_O3_2              (ICR_GCSC_BASE_ADDR+4*0xBE)
#define ICR_GCSC_P_C0_L              (ICR_GCSC_BASE_ADDR+4*0xC0)
#define ICR_GCSC_P_C0_H              (ICR_GCSC_BASE_ADDR+4*0xC1)
#define ICR_GCSC_P_C1_L              (ICR_GCSC_BASE_ADDR+4*0xC2)
#define ICR_GCSC_P_C1_H              (ICR_GCSC_BASE_ADDR+4*0xC3)
#define ICR_GCSC_P_C2_L              (ICR_GCSC_BASE_ADDR+4*0xC4)
#define ICR_GCSC_P_C2_H              (ICR_GCSC_BASE_ADDR+4*0xC5)
#define ICR_GCSC_P_C3_L              (ICR_GCSC_BASE_ADDR+4*0xC6)
#define ICR_GCSC_P_C3_H              (ICR_GCSC_BASE_ADDR+4*0xC7)
#define ICR_GCSC_P_C4_L              (ICR_GCSC_BASE_ADDR+4*0xC8)
#define ICR_GCSC_P_C4_H              (ICR_GCSC_BASE_ADDR+4*0xC9)
#define ICR_GCSC_P_C5_L              (ICR_GCSC_BASE_ADDR+4*0xCA)
#define ICR_GCSC_P_C5_H              (ICR_GCSC_BASE_ADDR+4*0xCB)
#define ICR_GCSC_P_C6_L              (ICR_GCSC_BASE_ADDR+4*0xCC)
#define ICR_GCSC_P_C6_H              (ICR_GCSC_BASE_ADDR+4*0xCD)
#define ICR_GCSC_P_C7_L              (ICR_GCSC_BASE_ADDR+4*0xCE)
#define ICR_GCSC_P_C7_H              (ICR_GCSC_BASE_ADDR+4*0xCF)
#define ICR_GCSC_P_C8_L              (ICR_GCSC_BASE_ADDR+4*0xD0)
#define ICR_GCSC_P_C8_H              (ICR_GCSC_BASE_ADDR+4*0xD1)
#define ICR_GCSC_P_O1_0              (ICR_GCSC_BASE_ADDR+4*0xD4)
#define ICR_GCSC_P_O1_1              (ICR_GCSC_BASE_ADDR+4*0xD5)
#define ICR_GCSC_P_O1_2              (ICR_GCSC_BASE_ADDR+4*0xD6)
#define ICR_GCSC_P_O2_0              (ICR_GCSC_BASE_ADDR+4*0xD8)
#define ICR_GCSC_P_O2_1              (ICR_GCSC_BASE_ADDR+4*0xD9)
#define ICR_GCSC_P_O2_2              (ICR_GCSC_BASE_ADDR+4*0xDA)
#define ICR_GCSC_P_O3_0              (ICR_GCSC_BASE_ADDR+4*0xDC)
#define ICR_GCSC_P_O3_1              (ICR_GCSC_BASE_ADDR+4*0xDD)
#define ICR_GCSC_P_O3_2              (ICR_GCSC_BASE_ADDR+4*0xDE)

#define ICR_CPCB_PIXVAL_M_EN         (ICR_GCSC_BASE_ADDR+4*0xE0)
#define ICR_CPCB_PIXVAL_P_EN         (ICR_GCSC_BASE_ADDR+4*0xE1)

#define ICR_REG_SIZE                 4096

/////////////////////////////////////////////////////////////////////////
//       GCR Control bits

#define ICR_GCR_SOFT_RESET           0x1000
#define ICR_GCR_RX_WPTR_RESET        0x0800
#define ICR_GCR_RX_RPTR_RESET        0x0400
#define ICR_GCR_TX_WPTR_RESET        0x0200
#define ICR_GCR_TX_RPTR_RESET        0x0100
#define ICR_GCR_DMA_RX_MOA_ENABLE    0x0080
#define ICR_GCR_DMA_TX_MOA_ENABLE    0x0040
#define ICR_GCR_SRC_FORMAT           0x0020
#define ICR_GCR_DST_FORMAT           0x0010
#define ICR_GCR_DMA1_RX_ENABLE      0x0008
#define ICR_GCR_DMA0_RX_ENABLE      0x0004
#define ICR_GCR_DMA1_TX_ENABLE      0x0002
#define ICR_GCR_DMA0_TX_ENABLE      0x0001
#define ICR_GCR_FULL_RESET     (   ICR_GCR_SOFT_RESET        \
                                 | ICR_GCR_RX_WPTR_RESET     \
                                 | ICR_GCR_RX_RPTR_RESET     \
                                 | ICR_GCR_TX_WPTR_RESET     \
				 | ICR_GCR_TX_RPTR_RESET )
#define ICR_GCR_FULL_ENABLE    ( 0 \
                                 /*| ICR_GCR_DMA_RX_MOA_ENABLE */ \
                                 /*| ICR_GCR_DMA_TX_MOA_ENABLE */ \
                                 /*| ICR_GCR_SRC_FORMAT*/    \
                                 /*| ICR_GCR_DST_FORMAT*/    \
                                 | ICR_GCR_DMA1_RX_ENABLE    \
                                 | ICR_GCR_DMA0_RX_ENABLE    \
                                 | ICR_GCR_DMA1_TX_ENABLE    \
                                 | ICR_GCR_DMA0_TX_ENABLE )
#define ICR_DMA_TRANSFER_32 (0x1<<1)
#define ICR_DMA_TRANSFER_64 (0x2<<1)
#define ICR_DMA_TRANSFER_128 (0x3<<1)

#define ICR_INTR_STATUS_ERRORS(a) (((a) & 0xfffffff0) != 0)
#define ICR_INTR_STATUS_DMA0_DONE(a) (((a) & 5) == 5)
#define ICR_INTR_STATUS_DMA0_READ_DONE(a) (((a) & 4) == 4)
#define ICR_INTR_STATUS_DMA0_CLEAR(a) (a) &= ~5
#define ICR_INTR_STATUS_DMA1_DONE(a) (((a) & 0xa) == 0xa)
#define ICR_INTR_STATUS_DMA1_READ_DONE(a) (((a) & 0x8) == 0x8)
#define ICR_INTR_STATUS_DMA1_CLEAR(a) (a) &= ~0xa

#define ICR_DMA_FORMAT_MASK (ICR_GCR_SRC_FORMAT | ICR_GCR_DST_FORMAT)

#endif /* __PXA168_ICR_H */