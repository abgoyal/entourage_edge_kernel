/*
 * ep3522ioctl.h - definitions for the Epson 3522 FB IOCTL interface
 *
 * Copyright (C) Entourage Systems, Inc. 2008, Joe Kralowetz
 *
 * This code is based on E-Ink demo source code, Epson framebuffer driver
 * source code, and Jaya Kumars broadsheetfb framebuffer driver source code.
 *
 */

#ifndef _EP3522IOCTL_H_
#define _EP3522IOCTL_H_

/********************************/
/** EP3522 PUBLIC DEFINITIONS  **/
/********************************/

/*** Operation Mode Defines ***/
#define EP_STATUS_MODE_OFF		0x00
#define EP_STATUS_MODE_SLEEP	0x01
#define EP_STATUS_MODE_STDBY	0x02
#define EP_STATUS_MODE_RUN		0x04

/********************************/
/** EP3522 CONTROLLER COMMANDS **/
/********************************/

  /* System Commands */
#define EP_INIT_CMD_SET		0x00	// Init instr code from FLASH addr
#define EP_INIT_PLL_STDBY	0x01	// Init PLL, go into STDBY
#define EP_RUN_SYS			0x02	// Run system using PLL
#define EP_STDBY			0x04	// Go to STDBY mode
#define EP_SLP				0x05	// Go to SLEEP mode
#define EP_INIT_SYS_RUN		0x06	// Init system, go into RUN
// #define EP_INIT_SYS_STDBY	0x07	// Init system, go into STDBY
// #define EP_INIT_SDRAM		0x08	// Init SDRAM
#define EP_INIT_DSPE_CFG	0x09	// Init Display Engine
#define EP_INIT_DSPE_TMG	0x0A	// Init Driver Timings
#define EP_INIT_ROTMODE		0x0B	// Init Rotation Mode Timings
#define EP_INIT_WAVEDEV		0x0C	// Init Waveform Device
  /* Register and Memory Commands */
#define EP_RD_REG			0x10	// Read Reg
#define EP_WR_REG			0x11	// Write Reg
// #define EP_RD_SFM			0x12	// Trigger Serial FLASH Read Operation
// #define EP_WR_SFM			0x13	// Trigger Serial FLASH Write Operation
// #define EP_END_SFM			0x14	// End SFM Operation
  /* PIP and Cursor Init Commands */
#define EP_PIP_DISABLE		0x14	// PIP Disable
#define EP_PIP_ENABLE		0x15	// PIP Enable
#define EP_PIP_ADRCFG		0x16	// PIP Data Storage Location
#define EP_PIP_XYSETUP		0x17	// PIP Setup Position on Screen
#define EP_CSR_MAINCFG		0x18	// Init Cursor
#define EP_CSR_XYSETUP		0x19	// Cursor Setup Position on Screen
#define EP_CSR_ADRCFG		0x1A	// Cursor Address Pointer Setup
  /* Burst Access Commands */
#define EP_BST_RD_SDR		0x1C	// Start Burst Read SDRAM Memory
#define EP_BST_WR_SDR		0x1D	// Start Burst Write SDRAM Memory
#define EP_BST_END			0x1E	// Burst End
  /* Image Loading Commands */
#define EP_LD_IMG			0x20	// Load Image Full
#define EP_LD_IMG_AREA		0x22	// Load Image Area with Parameters
#define EP_LD_IMG_END		0x23	// Load Image End
// #define EP_LD_IMG_WAIT		0x24	// Load Image Wait End
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
#define EP_PEN_DRAW			0x3A	// Pen Drawing Start
#define EP_PEN_MENU			0x3B	// Pen Menu Select Wait
#define EP_PEN_LINE			0x3C	// Pen Line Draw Command

/********************************/
/******* EP3522 REGISTERS *******/
/********************************/

  /* System Configuration Registers */
#define EP_REG_REV_CODE			0x000	// Revision Code Reg
#define EP_REG_PROD_CODE		0x002	// Product Code Reg
#define EP_REG_CFG_PIN_RD_VAL	0x004	// Cfg Pin Read Value Reg
#define EP_REG_PWR_SAVE_MODE	0x006	// Power Save Mode Reg
// #define EP_REG_SW_RESET			0x008	// Software Reset Reg
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
  /* Arithmetic Logic Unit Registers */
#define EP_REG_ALU_TMP_MIN		0x080	// ALU Temp Holding Reg Base (Reg 0)
#define EP_REG_ALU_TMP_MAX		0x0BE	// ALU Temp Holding Reg Base (Reg 31)
#define EP_REG_ALU_OP_RESULT_0	0x0C0	// ALU Operand/Result Reg 0
#define EP_REG_ALU_OP_RESULT_1	0x0C2	// ALU Operand/Result Reg 1
#define EP_REG_ALU_ADD_OP		0x0C4	// ALU Addition Operand Reg
#define EP_REG_ALU_SUB_OP		0x0C6	// ALU Subtraction Operand Reg
#define EP_REG_ALU_MULT_OP		0x0C8	// ALU Multiplication Operand Reg
#define EP_REG_ALU_DIV_DENOM	0x0CA	// ALU Division Denominator Reg
#define EP_REG_ALU_DIV_REMAIN	0x0CC	// ALU Division Remainder Reg
#define EP_REG_ALU_MD_STATUS	0x0CE	// ALU Multiplcation/Division Status Reg
#define EP_REG_ALU_INCR_CNTR	0x0D0	// ALU Increment Counter Reg
#define EP_REG_ALU_DECR_CNTR	0x0D2	// ALU Decrement Counter Reg
#define EP_REG_ALU_SHIFT_CTL	0x0D4	// ALU Shift/Rotate Control Reg
#define EP_REG_ALU_CMP_OP_0		0x0D6	// ALU Compare Operand Reg 0
#define EP_REG_ALU_CMP_OP_1		0x0D8	// ALU Compare Operand Reg 1
#define EP_REG_ALU_CMP_RESULT	0x0DA	// ALU Compare Result Reg
#define EP_REG_ALU_LOGIC_OP_OP	0x0DC	// ALU Logic Operation Operand Reg
#define EP_REG_ALU_LOGIC_OP_CTL	0x0DE	// ALU Logic Operation Control Reg
  /* Memory Controller Configuration Registers */
// #define EP_REG_SDRAM_CFG			0x100	// SDRAM Cfg Reg
#define EP_REG_SDRAM_INIT		0x102	// SDRAM Init Reg
#define EP_REG_SDRAM_STATE_TRIG	0x104	// SDRAM State Trigger Reg
#define EP_REG_SDRAM_REFR_CC	0x106	// SDRAM Refresh Clk Cfg Reg
// #define EP_REG_SDRAM_RD_TDS		0x108	// SDRAM Read Tap Delay Select Reg
// #define EP_REG_SDRAM_EXTMODE		0x10A	// SDRAM Extended Mode Cfg Reg
#define EP_REG_SDRAM_SW_RESET	0x10C	// SDRAM Controller SW Reset Cfg Reg
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
#define EP_REG_HPK_WR_TRANS		0x15A	// Host Packed Write Transparency Reg
#define EP_REG_HMEM_PIX_SWPCFG	0x15E	// Host Mem Access Pixel Swap Cfg Reg
  /* SPI FLASH Memory Interface Registers */
#define EP_REG_SPI_READ_DATA	0x200	// SPI Flash Read Data Reg
#define EP_REG_SPI_WRITE_DATA	0x202	// SPI Flash Write Data Reg
#define EP_REG_SPI_CTRL			0x204	// SPI Flash Control Reg
#define EP_REG_SPI_STATUS		0x206	// SPI Flash Status Reg
#define EP_REG_SPI_CHIPSEL_CTRL	0x208	// SPI Flash Chip Select Control Reg
  /* I2C Thermal Sensor Interface Registers */
#define EP_REG_THRM_SENS_CFG	0x210	// I2C Thermal Sensor Cfg Reg
#define EP_REG_THRM_SENS_STATUS	0x212	// I2C Thermal Sensor Status Reg
#define EP_REG_THRM_SENS_RDTRIG	0x214	// I2C Thermal Sensor Read Trigger Reg
#define EP_REG_THRM_SENS_VALUE	0x216	// I2C Thermal Sensor Temp Value Reg
#define EP_REG_I2C_STATUS_SETUP	0x218	// I2C Status/Setup Reg
#define EP_REG_I2C_CMD			0x21A	// I2C Command Reg
#define EP_REG_READ_DATA		0x21C	// I2C Read Data Reg
#define EP_REG_WRITE_DATA		0x21E	// I2C Write Data Reg
  /* 3-Wire Chip Interface Registers */
// #define EP_REG_3W_CFG			0x220	// 3-Wire Chip Cfg Reg
// #define EP_REG_3W_STATUS			0x222	// 3-Wire Chip Status Reg
// #define EP_REG_3W_ADDR_WR_BYTE	0x224	// 3-Wire Chip Addr/Write Data Reg
// #define EP_REG_3W_ADDR_RD_BYTE	0x226	// 3-Wire Chip Addr/Read Data Reg
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
  /* Waveform Read Control Registers */
#define EP_REG_WAVE_READ_CFG	0x260	// Waveform Read Cfg Reg
  /* Command RAM Controller Registers */
#define EP_REG_CRC_CTRL			0x290	// Cmd RAM Ctlr Cfg Reg
#define EP_REG_CRC_ADDR			0x292	// Cmd RAM Ctlr Address Reg
#define EP_REG_CRC_ACCESS_PORT	0x294	// Cmd RAM Ctlr Access Port Reg
  /* Command Sequencer Controller Registers */
#define EP_REG_CSC_INDEX		0x2A0	// Cmd Seq Ctlr Index Reg
#define EP_REG_CSC_DATA_PORT	0x2A2	// Cmd Seq Ctlr Data Port Reg
#define EP_REG_CSC_VER_REG_0	0x2A4	// Cmd Seq Ctlr Version # Reg 0
#define EP_REG_CSC_VER_REG_1	0x2A6	// Cmd Seq Ctlr Version # Reg 1
#define EP_REG_CSC_AL_STATUS	0x2A8	// Cmd Seq Ctlr Auto-Load Status Reg
#define EP_REG_CSC_WAKE_INT_SEL	0x2AA	// Cmd Seq Ctlr Wake Up Int Select Reg
#define EP_REG_CSC_SLEEP_CTL	0x2AC	// Cmd Seq Ctlr Sleep Mode Control Reg
#define EP_REG_CSC_INT_CTL		0x2AE	// Cmd Seq Ctlr Int Control Reg
  /* Multi-Function Serializer Registers */
#define EP_REG_MFS_INTF_SEL		0x2C0	// Multi-Func Interface Select Reg
#define EP_REG_MFS_SPI_RD_DATA	0x2C2	// Multi-Func SPI Read Data Reg
#define EP_REG_MFS_SPI_WR_DATA	0x2C4	// Multi-Func SPI Write Data Reg
#define EP_REG_MFS_SPI_CTRL		0x2C6	// Multi-Func SPI Control Reg
#define EP_REG_MFS_SPI_STATUS	0x2C8	// Multi-Func SPI Status Reg
#define EP_REG_MFS_SPI_CHIP_SEL	0x2CA	// Multi-Func SPI Chip Select Ctrl Reg
#define EP_REG_MFS_UART_CTRL	0x2CC	// Multi-Func UART Control Reg
#define EP_REG_MFS_UART_TRG_LVL	0x2CE	// Multi-Func UART Trigger Lvls Reg
#define EP_REG_MFS_UART_BID		0x2D0	// Multi-Func UART Baud Integer Div Reg
#define EP_REG_MFS_UART_BFD		0x2D2	// Multi-Func UART Baud Frac Div Reg
#define EP_REG_MFS_UART_TX_DATA	0x2D4	// Multi-Func UART TX Write Data Reg
#define EP_REG_MFS_UART_RX_DATA	0x2D6	// Multi-Func UART RX Read Data Reg
#define EP_REG_MFS_UART_ISC		0x2D8	// Multi-Func UART Int Status/Clear Reg
#define EP_REG_MFS_UART_FRESET	0x2DA	// Multi-Func UART FIFO Reset Reg
#define EP_REG_MFS_UART_STATUS	0x2DC	// Multi-Func UART TX/RX Status Reg
#define EP_REG_MFS_UART_FLVL	0x2DE	// Multi-Func UART FIFO Level Reg
#define EP_REG_MFS_UART_INT_EN	0x2E0	// Multi-Func UART Int Enable Reg
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
#define EP_REG_DE_PIPBUF_START_0 0x318	// PIP Buffer Start Addr_0 Reg
#define EP_REG_DE_PIPBUF_START_1 0x31A	// PIP Buffer Start Addr_1 Reg
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
  /*   NOTE:  These moved from 0x340.... in 3521 to 0x380.... in 3522   */
#define EP_REG_AU_PIX_XSTART	0x380	// Area Update Pixel Rect X-Start Reg
#define EP_REG_AU_PIX_YSTART	0x382	// Area Update Pixel Rect Y-Start Reg
#define EP_REG_AU_PIX_XEND		0x384	// Area Update Pixel Rect X-End Reg
#define EP_REG_AU_PIX_YEND		0x386	// Area Update Pixel Rect Y-End Reg
#define EP_REG_HST_PIX_XSTART	0x388	// Host Pixel Rect X-Start Reg
#define EP_REG_HST_PIX_YSTART	0x38A	// Host Pixel Rect Y-Start Reg
#define EP_REG_HST_PIX_XEND		0x38C	// Host Pixel Rect X-End Reg
#define EP_REG_HST_PIX_YEND		0x38E	// Host Pixel Rect Y-End Reg
  /* Display Engine: Serial Flash Waveform Registers */
  /*   NOTE:  These moved from 0x350.... in 3521 to 0x390.... in 3522   */
#define EP_REG_SFW_FLASH_ADDR_0	0x390	// Waveform Hdr Serial Flash Addr_0 Reg
#define EP_REG_SFW_FLASH_ADDR_1	0x392	// Waveform Hdr Serial Flash Addr_1 Reg
// #define EP_REG_SFW_LUTS_FVSN		0x354	// Waveform Hdr Ser LUTS/FVSN Info
// #define EP_REG_SFW_TRC_MC		0x356	// Waveform Hdr Serial TRC/MC Info
// #define EP_REG_SFW_SB_EB			0x358	// Waveform Hdr Serial SB/EB MC Info
// #define EP_REG_SFW_WTMA_0		0x35C	// Waveform Hdr Serial WTMA_0
// #define EP_REG_SFW_WTMA_1		0x35E	// Waveform Hdr Serial WTMA_1
  /* Auto Waveform Mode Configuration Registers */
  /*   NOTE:  These moved from 0x360.... in 3521 to 0x3A0.... in 3522   */
#define EP_AWF_MOD_0_DU_CFG		0x3A0	// Auto WFM Module 0 DU Cfg Reg
#define EP_AWF_MOD_1_GS4_CFG	0x3A2	// Auto WFM Module 1 GS 4 Lvls Cfg Reg
#define EP_AWF_MOD_2_GS8_CFG	0x3A4	// Auto WFM Module 2 GS 8 Lvls Cfg Reg
#define EP_AWF_MOD_3_GS16_CFG	0x3A6	// Auto WFM Module 3 GS 16 Lvls Cfg Reg
// #define EP_AWF_MODE_COMP_2_CFG	0x368	// Auto WFM Compare 2 Cfg Reg
// #define EP_AWF_MODE_COMP_2_CN	0x36A	// Auto WFM Compare 2 Curr/Next Reg
// #define EP_AWF_MODE_COMP_3_CFG	0x36C	// Auto WFM Compare 3 Cfg Reg
  /* Picture in Picture Control Registers */
#define EP_PIP_CFG				0x3C0	// PIP Configuration Register
#define EP_PIP_WIDTH			0x3C2	// PIP Width Register
#define EP_PIP_HEIGHT			0x3C4	// PIP Height Register
#define EP_PIP_X_START			0x3C6	// PIP X-Start Register
#define EP_PIP_Y_START			0x3C8	// PIP Y-Start Register
#define EP_PIP_LUT_XLATOR		0x3CA	// PIP LUT Val Xlator 1 bpp - 2 Reg
  /* Cursor Control Registers */
#define EP_CSR_CFG				0x3D0	// Cursor Configuration Register
#define EP_CSR_WIDTH			0x3D2	// Cursor Width Register
#define EP_CSR_HEIGHT			0x3D4	// Cursor Height Register
#define EP_CSR_X_START			0x3D6	// Cursor X-Start Register
#define EP_CSR_Y_START			0x3D8	// Cursor Y-Start Register
#define EP_CSR_LUT_XLATOR		0x3DA	// Cursor LUT Val Xlator 1 bpp - 2 Reg
#define EP_CSR_MEM_PTR			0x3DC	// Cursor Memory Ptr Reg
#define EP_CSR_MEM_ACCESS_PORT	0x3DE	// Cursor Memory Access Port Reg

/********************************/
/*** EP3522 IOCTL DEFINITIONS ***/
/********************************/

#pragma pack()

/*** IOCTL : Major Number ***/
#define EPFBIO_MAJOR			0x4500

/*** IOCTL : EP3522 Commands ***/

// System commands
#define EPFBIO_INIT_CMD_SET			(EPFBIO_MAJOR | EP_INIT_CMD_SET)
#define EPFBIO_INIT_PLL_STDBY		(EPFBIO_MAJOR | EP_INIT_PLL_STDBY)
#define EPFBIO_RUN_SYS				(EPFBIO_MAJOR | EP_RUN_SYS)
#define EPFBIO_STDBY				(EPFBIO_MAJOR | EP_STDBY)
#define EPFBIO_SLP					(EPFBIO_MAJOR | EP_SLP)
#define EPFBIO_INIT_SYS_RUN			(EPFBIO_MAJOR | EP_INIT_SYS_RUN)
// #define EPFBIO_INIT_SYS_STDBY		(EPFBIO_MAJOR | EP_INIT_SYS_STDBY)
// #define EPFBIO_INIT_SDRAM			(EPFBIO_MAJOR | EP_INIT_SDRAM)
#define EPFBIO_INIT_DSPE_CFG		(EPFBIO_MAJOR | EP_INIT_DSPE_CFG)
#define EPFBIO_INIT_DSPE_TMG		(EPFBIO_MAJOR | EP_INIT_DSPE_TMG)
#define EPFBIO_INIT_ROTMODE			(EPFBIO_MAJOR | EP_INIT_ROTMODE)
#define EPFBIO_INIT_WAVEDEV			(EPFBIO_MAJOR | EP_INIT_WAVEDEV)

// Register and memory commands
#define EPFBIO_RD_REG				(EPFBIO_MAJOR | EP_RD_REG)
#define EPFBIO_WR_REG				(EPFBIO_MAJOR | EP_WR_REG)
// #define EPFBIO_RD_SFM				(EPFBIO_MAJOR | EP_RD_SFM)
// #define EPFBIO_WR_SFM				(EPFBIO_MAJOR | EP_WR_SFM)
// #define EPFBIO_END_SFM				(EPFBIO_MAJOR | EP_END_SFM)

// PIP and cursor init commands
#define EPFBIO_PIP_DISABLE			(EPFBIO_MAJOR | EP_PIP_DISABLE)
#define EPFBIO_PIP_ENABLE			(EPFBIO_MAJOR | EP_PIP_ENABLE)
#define EPFBIO_PIP_ADRCFG			(EPFBIO_MAJOR | EP_PIP_ADRCFG)
#define EPFBIO_PIP_XYSETUP			(EPFBIO_MAJOR | EP_PIP_XYSETUP)
#define EPFBIO_CSR_MAINCFG			(EPFBIO_MAJOR | EP_CSR_MAINCFG)
#define EPFBIO_CSR_XYSETUP			(EPFBIO_MAJOR | EP_CSR_XYSETUP)
#define EPFBIO_CSR_ADRCFG			(EPFBIO_MAJOR | EP_CSR_ADRCFG)

// Burst access commands
#define EPFBIO_BST_RD_SDR			(EPFBIO_MAJOR | EP_BST_RD_SDR)
#define EPFBIO_BST_WR_SDR			(EPFBIO_MAJOR | EP_BST_WR_SDR)
#define EPFBIO_BST_END				(EPFBIO_MAJOR | EP_BST_END)

// Image loading IOCTL commands
#define EPFBIO_LD_IMG				(EPFBIO_MAJOR | EP_LD_IMG)
#define EPFBIO_LD_IMG_AREA			(EPFBIO_MAJOR | EP_LD_IMG_AREA)
#define EPFBIO_LD_IMG_END			(EPFBIO_MAJOR | EP_LD_IMG_END)
// #define EPFBIO_LD_IMG_WAIT			(EPFBIO_MAJOR | EP_LD_IMG_WAIT)
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
#define EPFBIO_PEN_DRAW				(EPFBIO_MAJOR | EP_PEN_DRAW)
#define EPFBIO_PEN_MENU				(EPFBIO_MAJOR | EP_PEN_MENU)
#define EPFBIO_PEN_LINE				(EPFBIO_MAJOR | EP_PEN_LINE)

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
} ep3522_cmd_params;

#endif
