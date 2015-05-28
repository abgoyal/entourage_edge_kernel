/*
 * ep3521ioctl.h - definitions for the Epson 3521 FB IOCTL interface
 *
 * Copyright (C) Entourage Systems, Inc. 2008, Joe Kralowetz
 *
 * This code is based on E-Ink demo source code, Epson framebuffer driver
 * source code, and Jaya Kumars broadsheetfb framebuffer driver source code.
 *
 */

#ifndef _EP3521IOCTL_H_
#define _EP3521IOCTL_H_

/********************************/
/** EP3521 PUBLIC DEFINITIONS  **/
/********************************/

/*** Operation Mode Defines ***/
#define EP_STATUS_MODE_OFF		0x00
#define EP_STATUS_MODE_SLEEP	0x01
#define EP_STATUS_MODE_STDBY	0x02
#define EP_STATUS_MODE_RUN		0x04

/********************************/
/** EP3521 CONTROLLER COMMANDS **/
/********************************/

  /* System Commands */
#define EP_INIT_CMD_SET		0x00	// Init instr code from FLASH addr
#define EP_INIT_PLL_STDBY	0x01	// Init PLL, go into STDBY
#define EP_RUN_SYS			0x02	// Run system using PLL
#define EP_STDBY			0x04	// Go to STDBY mode
#define EP_SLP				0x05	// Go to SLEEP mode
#define EP_INIT_SYS_RUN		0x06	// Init system, go into RUN
#define EP_INIT_SYS_STDBY	0x07	// Init system, go into STDBY
#define EP_INIT_SDRAM		0x08	// Init SDRAM
#define EP_INIT_DSPE_CFG	0x09	// Init Display Engine
#define EP_INIT_DSPE_TMG	0x0A	// Init Driver Timings
#define EP_INIT_ROTMODE		0x0B	// Init Rotation Mode Timings
  /* Register and Memory Commands */
#define EP_RD_REG			0x10	// Read Reg
#define EP_WR_REG			0x11	// Write Reg
#define EP_RD_SFM			0x12	// Trigger Serial FLASH Read Operation
#define EP_WR_SFM			0x13	// Trigger Serial FLASH Write Operation
#define EP_END_SFM			0x14	// End SFM Operation
  /* Burst Access Commands */
#define EP_BST_RD_SDR		0x1C	// Start Burst Read SDRAM Memory
#define EP_BST_WR_SDR		0x1D	// Start Burst Write SDRAM Memory
#define EP_BST_END			0x1E	// Burst End
  /* Image Loading Commands */
#define EP_LD_IMG			0x20	// Load Image Full
#define EP_LD_IMG_AREA		0x22	// Load Image Area with Parameters
#define EP_LD_IMG_END		0x23	// Load Image End
#define EP_LD_IMG_WAIT		0x24	// Load Image Wait End
#define EP_LD_IMG_SETADR	0x25	// Set Load Image Manual Addr
#define EP_LD_IMG_DSPEADR	0x26	// Set Load Image to use Display Engs Addr
  /* Polling Commands */
#define EP_WAIT_DSPE_TRG	0x28	// Wait for Display Eng Trigger Done
#define EP_WAIT_DSPE_FREND	0x29	// Wait for Disp Eng Frame End
#define EP_WAIT_DSPE_LUTFREE  0x2A	// Wait for Disp Eng at Least 1 LUT free
#define EP_WAIT_DSPE_MLUTFREE 0x2B	// Wait for Disp Eng at Least 1 Masked LUT free
  /* Waveform Update Commands */
#define EP_RD_WFM_INFO		0x30	// Read Waveform Info
#define EP_UPD_INIT			0x32	// Update Buffer Initialization
#define EP_UPD_FULL			0x33	// Update Buffer Full
#define EP_UPD_FULL_AREA	0x34	// Update Buffer Full Area
#define EP_UPD_PART			0x35	// Update Buffer Partial
#define EP_UPD_PART_AREA	0x36	// Update Buffer Partial Area
#define EP_UPD_GDRV_CLR		0x37	// Gate Driver Clear Command
#define EP_UPD_SET_IMGADR	0x38	// Set Image Buffer Start Addr

/********************************/
/******* EP3521 REGISTERS *******/
/********************************/

  /* System Configuration Registers */
#define EP_REG_REV_CODE			0x000	// Revision Code Reg
#define EP_REG_PROD_CODE		0x002	// Product Code Reg
#define EP_REG_CFG_PIN_RD_VAL	0x004	// Cfg Pin Read Value Reg
#define EP_REG_PWR_SAVE_MODE	0x006	// Power Save Mode Reg
#define EP_REG_SW_RESET			0x008	// Software Reset Reg
#define EP_REG_SYS_STATUS		0x00A	// System Status Reg
  /* Clock Configuration Registers */
#define EP_REG_PLL_CFG_0		0x010	// PLL Cfg_0 Reg
#define EP_REG_PLL_CFG_1		0x012	// PLL Cfg_1 Reg
#define EP_REG_PLL_CFG_2		0x014	// PLL Cfg_2 Reg
#define EP_REG_CLK_CFG			0x016	// Clock Cfg Reg
#define EP_REG_PIXEL_CLK_CFG	0x018	// Pixel Clock Cfg Reg
#define EP_REG_THERM_SENS_CC	0x01A	// I2C Thermal Sensor Clk Cfg Reg
  /* Component Configuration Registers */
#define EP_REG_PERIPH_DEV_CFG	0x020	// Peripheral Device Cfg Reg
  /* Memory Controller Configuration Registers */
#define EP_REG_SDRAM_CFG		0x100	// SDRAM Cfg Reg
#define EP_REG_SDRAM_INIT		0x102	// SDRAM Init Reg
#define EP_REG_SDRAM_STATE_TRIG	0x104	// SDRAM State Trigger Reg
#define EP_REG_SDRAM_REFR_CC	0x106	// SDRAM Refresh Clk Cfg Reg
#define EP_REG_SDRAM_RD_TDS		0x108	// SDRAM Read Tap Delay Select Reg
#define EP_REG_SDRAM_EXTMODE	0x10A	// SDRAM Extended Mode Cfg Reg
#define EP_REG_SDRAM_SW_RESET	0x10C	// SDRAM Controller SW Reset Cfg
  /* Host Interface Memory Access Configuration Registers */
#define EP_REG_HMEM_SC_STATUS	0x140	// Host Mem Access Cfg and Status Reg
#define EP_REG_HMEM_TRIGGERS	0x142	// Host Mem Access Triggers Reg
#define EP_REG_HRAWMEM_ADDR_0	0x144	// Host Raw Mem Access Addr_0 Reg
#define EP_REG_HRAWMEM_ADDR_1	0x146	// Host Raw Mem Access Addr_1 Reg
#define EP_REG_HRAWMEM_COUNT_0	0x148	// Host Raw Mem Access Count_0 Reg
#define EP_REG_HRAWMEM_COUNT_1	0x14A	// Host Raw Mem Access Count_1 Reg
#define EP_REG_PPIXRECT_XSTART	0x14C	// Packed Pixel Rectangular X-Start Reg
#define EP_REG_PPIXRECT_YSTART	0x14E	// Packed Pixel Rectangular Y-Start Reg
#define EP_REG_PPIXRECT_WIDTH	0x150	// Packed Pixel Rectangular Width Reg
#define EP_REG_PPIXRECT_HEIGHT	0x152	// Packed Pixel Rectangular Height Reg
#define EP_REG_HMEM_PORT		0x154	// Host Mem Access Port Reg
#define EP_REG_HMEM_CHECKSUM	0x156	// Host Mem Access Checksum Reg
#define EP_REG_HRAWMEM_FIFO_LVL	0x158	// Host Raw Mem FIFO Level Reg
  /* SPI FLASH Memory Interface Registers */
#define EP_REG_SPI_READ_DATA	0x200	// SPI Flash Read Data Register
#define EP_REG_SPI_WRITE_DATA	0x202	// SPI Flash Write Data Register
#define EP_REG_SPI_CTRL			0x204	// SPI Flash Control Register
#define EP_REG_SPI_STATUS		0x206	// SPI Flash Status Register
#define EP_REG_SPI_CHIPSEL_CTRL	0x208	// SPI Flash Chip Select Control Reg
  /* I2C Thermal Sensor Interface Registers */
#define EP_REG_THRM_SENS_CFG	0x210	// I2C Thermal Sensor Cfg Reg
#define EP_REG_THRM_SENS_STATUS	0x212	// I2C Thermal Sensor Status Reg
#define EP_REG_THRM_SENS_RDTRIG	0x214	// I2C Thermal Sensor Read Trigger Reg
#define EP_REG_THRM_SENS_VALUE	0x216	// I2C Thermal Sensor Temp Value Reg
  /* 3-Wire Chip Interface Registers */
#define EP_REG_3W_CFG			0x220	// 3-Wire Chip Cfg Reg
#define EP_REG_3W_STATUS		0x222	// 3-Wire Chip Status Reg
#define EP_REG_3W_ADDR_WR_BYTE	0x224	// 3-Wire Chip Addr/Write Data Byte Reg
#define EP_REG_3W_ADDR_RD_BYTE	0x226	// 3-Wire Chip Addr/Read Data Byte Reg
  /* Power Pin Control Configuration Registers */
#define EP_REG_PP_CTRL			0x230	// Power Pin Control Reg
#define EP_REG_PP_CFG			0x232	// Power Pin Cfg Reg
#define EP_REG_PP_TIMING_DLY_01	0x234	// Power Pin Timing Delay 0-1 Reg
#define EP_REG_PP_TIMING_DLY_12	0x236	// Power Pin Timing Delay 1-2 Reg
#define EP_REG_PP_TIMING_DLY_23	0x238	// Power Pin Timing Delay 2-3 Reg
  /* Interrupt Configuration Registers */
#define EP_REG_INTR_RAW_STATUS	0x240	// Interrupt Raw Status Reg
#define EP_REG_INTR_MASK_STATUS	0x242	// Interrupt Raw Masked Status Reg
#define EP_REG_INTR_CTRL		0x244	// Interrupt Control Reg
  /* GPIO Control Registers */
#define EP_REG_GPIO_CFG			0x250	// GPIO Configuration Reg
#define EP_REG_GPIO_STAT_CTRL	0x252	// GPIO Status/Control Reg
#define EP_REG_GPIO_INTR_ENABLE	0x254	// GPIO Interrupt Enable Reg
#define EP_REG_GPIO_INTR_STATUS	0x256	// GPIO Interrupt Status Reg
  /* Command RAM Controller Registers */
#define EP_REG_CRC_CTRL			0x290	// Cmd RAM Ctlr Cfg Reg
#define EP_REG_CRC_ADDR			0x292	// Cmd RAM Ctlr Address Reg
#define EP_REG_CRC_ACCESS_PORT	0x294	// Cmd RAM Ctlr Access Port Reg
  /* Display Engine: Display Timing Configuration Registers */
#define EP_REG_DE_FR_DATA_LEN	0x300	// Frame Data Length Reg
#define EP_REG_DE_FR_SYNC_LEN	0x302	// Frame Sync Length Reg
#define EP_REG_DE_FR_BEG_END_LEN 0x304	// Frame Begin/End Length Reg
#define EP_REG_DE_LINE_DATA_LEN	0x306	// Line Data Length Reg
#define EP_REG_DE_LINE_SYNC_LEN	0x308	// Line Sync Length Reg
#define EP_REG_DE_LINE_BEG_END_LEN 0x30A	// Line Begin/End Length Reg
  /* Display Engine: Driver Configurations Registers */
#define EP_REG_DE_SRC_DRV_CFG	0x30C	// Source Driver Configuration Reg
#define EP_REG_DE_GATE_DRV_CFG	0x30E	// Gate Driver Configuration Reg
  /* Display Engine: Memory Region Configuration Registers */
#define EP_REG_DE_IMBUF_START_0	0x310	// Image Buffer Start Addr_0 Reg
#define EP_REG_DE_IMBUF_START_1	0x312	// Image Buffer Start Addr_1 Reg
#define EP_REG_DE_UPBUF_START_0	0x314	// Update Buffer Start Addr_0 Reg
#define EP_REG_DE_UPBUF_START_1	0x316	// Update Buffer Start Addr_1 Reg
  /* Display Engine: Component Control Registers */
#define EP_REG_DE_TEMPDEV_SEL	0x320	// Temperature Device Select Reg
#define EP_REG_DE_TEMP_VALUE	0x322	// Temperature Value Reg
#define EP_REG_DE_BORDER_CFG	0x326	// Border Cfg Reg
#define EP_REG_DE_PWR_CTRL_CFG	0x32A	// Power Control Configuration Reg
#define EP_REG_DE_GEN_CFG		0x32C	// General Configuration Reg
#define EP_REG_DE_LUT_MASK		0x32E	// LUT Mask Reg
  /* Display Engine: Control/Triggers Registers */
#define EP_REG_DE_UPDT_BUF_CFG	0x330	// Update Buffer Configuration Reg
#define EP_REG_DE_UPDT_BUF_PIXSET 0x332	// Update Buffer Pixel Set Value Reg
#define EP_REG_DE_DE_CTRL_TRIG	0x334	// Display Engine Ctrl/Trigger Reg
  /* Display Engine: Update Buffer Status Registers */
#define EP_REG_DE_LOOKUPTBL_STAT 0x336	// Lookup Table Status Reg
#define EP_REG_DE_BUSY_STATUS	0x338	// Display Engine Busy Status Reg
  /* Display Engine: Interrupt Registers */
#define EP_REG_DE_INT_RAW_STATUS 0x33A	// Display Eng Inter Raw Status Reg
#define EP_REG_DE_INT_MASK_STATUS 0x33C	// Display Eng Inter Masked Status Reg
#define EP_REG_DE_INT_ENABLE	0x33E	// Display Eng Inter Enable Reg
  /* Display Engine: Partial Update Configuration Registers */
#define EP_REG_AU_PIX_XSTART	0x340	// Area Update Pixel Rect X-Start Reg
#define EP_REG_AU_PIX_YSTART	0x342	// Area Update Pixel Rect Y-Start Reg
#define EP_REG_AU_PIX_XEND		0x344	// Area Update Pixel Rect X-End Reg
#define EP_REG_AU_PIX_YEND		0x346	// Area Update Pixel Rect Y-End Reg
#define EP_REG_HST_PIX_XSTART	0x348	// Host Pixel Rect X-Start Reg
#define EP_REG_HST_PIX_YSTART	0x34A	// Host Pixel Rect Y-Start Reg
#define EP_REG_HST_PIX_XEND		0x34C	// Host Pixel Rect X-End Reg
#define EP_REG_HST_PIX_YEND		0x34E	// Host Pixel Rect Y-End Reg
  /* Display Engine: Serial Flash Waveform Registers */
#define EP_REG_SFW_FLASH_ADDR_0	0x350	// Waveform Hdr Serial Flash Addr_0 Reg
#define EP_REG_SFW_FLASH_ADDR_1	0x352	// Waveform Hdr Serial Flash Addr_1 Reg
#define EP_REG_SFW_LUTS_FVSN	0x354	// Waveform Hdr Serial LUTS/FVSN Info
#define EP_REG_SFW_TRC_MC		0x356	// Waveform Hdr Serial TRC/MC Info
#define EP_REG_SFW_SB_EB		0x358	// Waveform Hdr Serial SB/EB MC Info
#define EP_REG_SFW_WTMA_0		0x35C	// Waveform Hdr Serial WTMA_0
#define EP_REG_SFW_WTMA_1		0x35E	// Waveform Hdr Serial WTMA_1
  /* Auto Waveform Mode Configuration Registers */
#define EP_AWF_MODE_COMP_0_CFG	0x360	// Auto WFM Compare 0 Cfg Reg
#define EP_AWF_MODE_COMP_0_CN	0x362	// Auto WFM Compare 0 Curr/Next Reg
#define EP_AWF_MODE_COMP_1_CFG	0x364	// Auto WFM Compare 1 Cfg Reg
#define EP_AWF_MODE_COMP_1_CN	0x366	// Auto WFM Compare 1 Curr/Next Reg
#define EP_AWF_MODE_COMP_2_CFG	0x368	// Auto WFM Compare 2 Cfg Reg
#define EP_AWF_MODE_COMP_2_CN	0x36A	// Auto WFM Compare 2 Curr/Next Reg
#define EP_AWF_MODE_COMP_3_CFG	0x36C	// Auto WFM Compare 3 Cfg Reg

/********************************/
/*** EP3521 IOCTL DEFINITIONS ***/
/********************************/

#pragma pack()

/*** IOCTL : Major Number ***/
#define EPFBIO_MAJOR			0x4500

/*** IOCTL : EP3521 Commands ***/

#define EPFBIO_INIT_CMD_SET			(EPFBIO_MAJOR | EP_INIT_CMD_SET)
#define EPFBIO_INIT_PLL_STDBY		(EPFBIO_MAJOR | EP_INIT_PLL_STDBY)
#define EPFBIO_RUN_SYS				(EPFBIO_MAJOR | EP_RUN_SYS)
#define EPFBIO_STDBY				(EPFBIO_MAJOR | EP_STDBY)
#define EPFBIO_SLP					(EPFBIO_MAJOR | EP_SLP)
#define EPFBIO_INIT_SYS_RUN			(EPFBIO_MAJOR | EP_INIT_SYS_RUN)
#define EPFBIO_INIT_SYS_STDBY		(EPFBIO_MAJOR | EP_INIT_SYS_STDBY)
#define EPFBIO_INIT_SDRAM			(EPFBIO_MAJOR | EP_INIT_SDRAM)
#define EPFBIO_INIT_DSPE_CFG		(EPFBIO_MAJOR | EP_INIT_DSPE_CFG)
#define EPFBIO_INIT_DSPE_TMG		(EPFBIO_MAJOR | EP_INIT_DSPE_TMG)
#define EPFBIO_INIT_ROTMODE			(EPFBIO_MAJOR | EP_INIT_ROTMODE)
#define EPFBIO_RD_REG				(EPFBIO_MAJOR | EP_RD_REG)
#define EPFBIO_WR_REG				(EPFBIO_MAJOR | EP_WR_REG)
#define EPFBIO_RD_SFM				(EPFBIO_MAJOR | EP_RD_SFM)
#define EPFBIO_WR_SFM				(EPFBIO_MAJOR | EP_WR_SFM)
#define EPFBIO_END_SFM				(EPFBIO_MAJOR | EP_END_SFM)

// Burst access commands
#define EPFBIO_BST_RD_SDR			(EPFBIO_MAJOR | EP_BST_RD_SDR)
#define EPFBIO_BST_WR_SDR			(EPFBIO_MAJOR | EP_BST_WR_SDR)
#define EPFBIO_BST_END				(EPFBIO_MAJOR | EP_BST_END)

// Image loading IOCTL commands
#define EPFBIO_LD_IMG				(EPFBIO_MAJOR | EP_LD_IMG)
#define EPFBIO_LD_IMG_AREA			(EPFBIO_MAJOR | EP_LD_IMG_AREA)
#define EPFBIO_LD_IMG_END			(EPFBIO_MAJOR | EP_LD_IMG_END)
#define EPFBIO_LD_IMG_WAIT			(EPFBIO_MAJOR | EP_LD_IMG_WAIT)
#define EPFBIO_LD_IMG_SETADR		(EPFBIO_MAJOR | EP_LD_IMG_SETADR)
#define EPFBIO_LD_IMG_DSPEADR		(EPFBIO_MAJOR | EP_LD_IMG_DSPEADR)

// Polling commands
#define EPFBIO_WAIT_DSPE_TRG		(EPFBIO_MAJOR | EP_WAIT_DSPE_TRG)
#define EPFBIO_WAIT_DSPE_FREND		(EPFBIO_MAJOR | EP_WAIT_DSPE_FREND)
#define EPFBIO_WAIT_DSPE_LUTFREE	(EPFBIO_MAJOR | EP_WAIT_DSPE_LUTFREE)
#define EPFBIO_WAIT_DSPE_MLUTFREE	(EPFBIO_MAJOR | EP_WAIT_DSPE_MLUTFREE)

// Waveform update IOCTL commands
#define EPFBIO_RD_WFM_INFO			(EPFBIO_MAJOR | EP_RD_WFM_INFO)
#define EPFBIO_UPD_INIT				(EPFBIO_MAJOR | EP_UPD_INIT)
#define EPFBIO_UPD_FULL				(EPFBIO_MAJOR | EP_UPD_FULL)
#define EPFBIO_UPD_FULL_AREA		(EPFBIO_MAJOR | EP_UPD_FULL_AREA)
#define EPFBIO_UPD_PART				(EPFBIO_MAJOR | EP_UPD_PART)
#define EPFBIO_UPD_PART_AREA		(EPFBIO_MAJOR | EP_UPD_PART_AREA)
#define EPFBIO_UPD_GDRV_CLR			(EPFBIO_MAJOR | EP_UPD_GDRV_CLR)
#define EPFBIO_UPD_SET_IMGADR		(EPFBIO_MAJOR | EP_UPD_SET_IMGADR)

/*** IOCTL : Extended Commands ***/

#define EPFBIOX_MEMBURSTREAD		(EPFBIO_MAJOR | 0x80)
#define EPFBIOX_MEMBURSTWRITE		(EPFBIO_MAJOR | 0x81)
#define EPFBIOX_MANUAL_REFRESH		(EPFBIO_MAJOR | 0x82)
#define EPFBIOX_UPDATE_PART			(EPFBIO_MAJOR | 0x83)
#define EPFBIOX_DEFIO_DELAY			(EPFBIO_MAJOR | 0x84)
#define EPFBIOX_PWR_MODE			(EPFBIO_MAJOR | 0x85)
#define EPFBIOX_GET_VARS			(EPFBIO_MAJOR | 0x86)
#define EPFBIOX_CREGION_SETUP		(EPFBIO_MAJOR | 0x87)
#define EPFBIOX_CREGION_STATE		(EPFBIO_MAJOR | 0x88)
#define EPFBIOX_CREGION_DISPLAY_MODE (EPFBIO_MAJOR | 0x89)
#define EPFBIOX_CREGION_UPDATE_MODE (EPFBIO_MAJOR | 0x8a)
#define EPFBIOX_CREGION_PEN_WIDTH   (EPFBIO_MAJOR | 0x8b)
#define EPFBIOX_CREGION_PEN_COLOR   (EPFBIO_MAJOR | 0x8c)
#define EPFBIOX_INHIBIT_FLASHING	(EPFBIO_MAJOR | 0x8d)
#define EPFBIOX_AREA_REFRESH		(EPFBIO_MAJOR | 0x8e)
#define EPFBIOX_PAINT_SCREEN		(EPFBIO_MAJOR | 0x8f)
#define EPFBIOX_CHECK_FOR_INITED	(EPFBIO_MAJOR | 0x90)
#define EPFBIOX_CREGION_OVERLAP_MODE (EPFBIO_MAJOR | 0x91)
#define EPFBIOX_MENU_UP				(EPFBIO_MAJOR | 0x92)
#define EPFBIOX_FRAME_BUFFER_READ	(EPFBIO_MAJOR | 0x93)

#define AREA_POPUP			1			// Defines for AREA_REFRESH
#define AREA_ABOVE			2
#define AREA_RIGHT_SIDE		4
#define AREA_BELOW			8
#define AREA_NONCREGION		16

/*** IOCTL : Interface Structures ***/

#pragma pack(1)

typedef struct
{
	unsigned short param[12];
} ep3521_cmd_params;

#endif
