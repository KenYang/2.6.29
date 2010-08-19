#ifndef __SOCLE_SDHC_REG_H_INCLUDED
#define __SOCLE_SDHC_REG_H_INCLUDED

/**
 * @struct STANDARD_REGISTERS
 * @brief Structure contains a list of pointers to all standadard SDIO registers.
 */

/*
 *  Registers for SDHC host controller
 *  */
#define SOCLE_SDHC_HRS0 		0x00 
#define SOCLE_SDHC_HRS1 		0x04 
#define SOCLE_SDHC_HRS2		0x08 
#define SOCLE_SDHC_HRS3 		0x0C 
#define SOCLE_SDHC_HRS4 		0x10 
#define SOCLE_SDHC_HRS5		0x14 
#define SOCLE_SDHC_HRS6 		0x18 
#define SOCLE_SDHC_HRS7 		0x1C 
#define SOCLE_SDHC_HRS8		0x20 
#define SOCLE_SDHC_HRS9		0x24 
#define SOCLE_SDHC_HRS10 		0x28 
#define SOCLE_SDHC_HRS11 		0x2C 
#define SOCLE_SDHC_HRS12		0x30 
#define SOCLE_SDHC_HRS13 		0x34 
#define SOCLE_SDHC_HRS14 		0x38 
#define SOCLE_SDHC_HRS15		0x3C 
#define SOCLE_SDHC_HRS16 		0x40 
#define SOCLE_SDHC_HRS17 		0x44 
#define SOCLE_SDHC_HRS18		0x48 
#define SOCLE_SDHC_HRS19		0x4C 
#define SOCLE_SDHC_HRS20		0x50 
#define SOCLE_SDHC_HRS21		0x54

#define SOCLE_SDHC_HRS63		0xFC

#define SOCLE_SDHC_SRS0(x) 			(0x100*(x+1) + 0x00)
#define SOCLE_SDHC_SRS1(x) 			(0x100*(x+1) + 0x04) 
#define SOCLE_SDHC_SRS2(x)			(0x100*(x+1) + 0x08) 
#define SOCLE_SDHC_SRS3(x) 			(0x100*(x+1) + 0x0C) 
#define SOCLE_SDHC_SRS4(x) 			(0x100*(x+1) + 0x10) 
#define SOCLE_SDHC_SRS5(x)			(0x100*(x+1) + 0x14) 
#define SOCLE_SDHC_SRS6(x) 			(0x100*(x+1) + 0x18) 
#define SOCLE_SDHC_SRS7(x) 			(0x100*(x+1) + 0x1C) 
#define SOCLE_SDHC_SRS8(x)			(0x100*(x+1) + 0x20) 
#define SOCLE_SDHC_SRS9(x)			(0x100*(x+1) + 0x24) 
#define SOCLE_SDHC_SRS10(x) 		(0x100*(x+1) + 0x28) 
#define SOCLE_SDHC_SRS11(x) 		(0x100*(x+1) + 0x2C) 
#define SOCLE_SDHC_SRS12(x)		(0x100*(x+1) + 0x30) 
#define SOCLE_SDHC_SRS13(x) 		(0x100*(x+1) + 0x34) 
#define SOCLE_SDHC_SRS14(x) 		(0x100*(x+1) + 0x38) 
#define SOCLE_SDHC_SRS15(x)		(0x100*(x+1) + 0x3C) 
#define SOCLE_SDHC_SRS16(x) 		(0x100*(x+1) + 0x40) 
#define SOCLE_SDHC_SRS17(x) 		(0x100*(x+1) + 0x44) 
#define SOCLE_SDHC_SRS18(x)		(0x100*(x+1) + 0x48)
#define SOCLE_SDHC_SRS19(x)		(0x100*(x+1) + 0x4C) 
#define SOCLE_SDHC_SRS20(x)		(0x100*(x+1) + 0x50) 
#define SOCLE_SDHC_SRS21(x)		(0x100*(x+1) + 0x54)
#define SOCLE_SDHC_SRS22(x)		(0x100*(x+1) + 0x58)

//---------------------------------------------------------------------------
/// @anchor Commands
/// @name Definitions of the command types
//---------------------------------------------------------------------------
//@{
/// Ordinary type of command ( none of below three )
#define SDIOHOST_CMD_TYPE_OTHER     0x00
/// Abort command
#define SDIOHOST_CMD_TYPE_ABORT     0x03
/// Suspend command
#define SDIOHOST_CMD_TYPE_SUSPEND   0x01
/// Resume command
#define SDIOHOST_CMD_TYPE_RESUME    0x02
//@}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @anchor Responses
/// @name Definitions of the command responses
//---------------------------------------------------------------------------
//@{
/// No response
#define SDIOHOST_RESPONSE_NO_RESP   0x0
/// Response R1
#define SDIOHOST_RESPONSE_R1        0x1
/// Response R1 with busy check
#define SDIOHOST_RESPONSE_R1B       0x2
/// Response R2
#define SDIOHOST_RESPONSE_R2        0x3
/// Response R3
#define SDIOHOST_RESPONSE_R3        0x4
/// Response R4
#define SDIOHOST_RESPONSE_R4        0x5
/// Response R5
#define SDIOHOST_RESPONSE_R5        0x6
/// Response R5 with busy check
#define SDIOHOST_RESPONSE_R5B       0x7
/// Response R6
#define SDIOHOST_RESPONSE_R6        0x8
/// Response R7
#define SDIOHOST_RESPONSE_R7        0x9
//@}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @anchor Devices
/// @name Device types definitions
//---------------------------------------------------------------------------
//@{
/// SDIO card 
#define SDIOHOST_CARD_TYPE_SDIO     0x1
/// SD memory only card 
#define SDIOHOST_CARD_TYPE_SDMEM    0x2
/// SD combo card ( SDIOHOST_CARD_TYPE_SDIO | SDIOHOST_CARD_TYPE_SDMEM )
#define SDIOHOST_CARD_TYPE_COMBO    0x3
/// MMC memory card
#define SDIOHOST_CARD_TYPE_MMC      0x4
//@}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @anchor Capacities
/// @name Device capacity definitions
//---------------------------------------------------------------------------
//@{
/// Standard capacity card
#define SDIOHOST_CAPACITY_NORMAL    0x1
/// Height capacity card
#define SDIOHOST_CAPACITY_HIGH      0x2
//@}
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
/// @anchor BusModes
/// @name Host bus mode definitions
//---------------------------------------------------------------------------
//@{
/// SPI bus mode
#define SDIOHOST_BUS_MODE_SPI       0x1
/// SD bus mode
#define SDIOHOST_BUS_MODE_SD        0x2
//@}
//---------------------------------------------------------------------------


/// System addres register it contains physical memory address for DMA operations
#define SDIO_REG_SLOT_SFR0_OFFSET	0x00
/// Block count and size register it contains informations about transmission size
#define SDIO_REG_SLOT_SFR1_OFFSET	0x04
/// Argument register it contains command argument of SD device
#define SDIO_REG_SLOT_SFR2_OFFSET	0x08
/// Transfer mode and command information register
#define SDIO_REG_SLOT_SFR3_OFFSET	0x0C
/// Response #0 register
#define SDIO_REG_SLOT_SFR4_OFFSET	0x10
/// Response #1 register
#define SDIO_REG_SLOT_SFR5_OFFSET	0x14
/// Response #2 register
#define SDIO_REG_SLOT_SFR6_OFFSET	0x18
/// Response #3 register
#define SDIO_REG_SLOT_SFR7_OFFSET	0x1C
/// Buffer data port register
#define SDIO_REG_SLOT_SFR8_OFFSET	0x20
/// Preset state register it contains the status of the SLOT
#define SDIO_REG_SLOT_SFR9_OFFSET	0x24
/// Host control settings #0 register
#define SDIO_REG_SLOT_SFR10_OFFSET	0x28
/// Host control settings #1 register 
#define SDIO_REG_SLOT_SFR11_OFFSET	0x2C
/// Interrupt status register
#define SDIO_REG_SLOT_SFR12_OFFSET	0x30
/// Interrupt status enable register
#define SDIO_REG_SLOT_SFR13_OFFSET	0x34
/// Interrupt signal enable register
#define SDIO_REG_SLOT_SFR14_OFFSET	0x38
/// Auto CMD12 error status register
#define SDIO_REG_SLOT_SFR15_OFFSET	0x3C
/// Capabilities register
#define SDIO_REG_SLOT_SFR16_OFFSET	0x40
/// Maximum current capabilities
#define SDIO_REG_SLOT_SFR18_OFFSET	0x48
/// Event Trigger Register
#define SDIO_REG_SLOT_SFR20_OFFSET	0x50
/// ADMA Error Status Register
#define SDIO_REG_SLOT_SFR21_OFFSET	0x54
/// ADMA System Address Register
#define SDIO_REG_SLOT_SFR22_OFFSET	0x58

/// register from common register area it contains host controller version and slot interrupt status information
#define SDIO_REG_COMM_SFR63     (*(volatile DWORD *) (SDIO_REGISTERS_OFFSET + 0x00FC))
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name General information register (HSFR0) - masks
//-------------------------------------------------------------------------------------------
//@{
//	/// MMC 8 mask
#define HSFR0_MMC8_MASK     (0xf << 24)
/// 4 slots are accesible in the SDIO host
#define HSFR0_4_ACCESABLE_SLOTS     (0x1 << 19)
/// 3 slots are accesible in the SDIO host
#define HSFR0_3_ACCESABLE_SLOTS     (0x1 << 18)
/// 2 slots are accesible in the SDIO host
#define HSFR0_2_ACCESABLE_SLOTS     (0x1 << 17)
/// 1 slots are accesible in the SDIO host
#define HSFR0_1_ACCESABLE_SLOT      (0x1 << 16)

#define HSFR0_AVAILABLE_SLOT		(0xf<< 16)
/// Reset all internal registers including SFR RAM. The card detection unit is reset also.
#define HSFR0_SOFTWARE_RESET        (1 << 0)
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Initialization setting register (HSFR1) - masks and macros
//-------------------------------------------------------------------------------------------
//@{
/// SDIO host uses SD mode to initialize card
#define HSFR1_INITIALIZATION_MODE_SD    0x10000000
/// SDIO host uses SPI mode to initialize card
#define HSFR1_INITIALIZATION_MODE_SPI   0x00000000
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name DMA settings register (HSFR2) - values   
//-------------------------------------------------------------------------------------------
//@{
/// set 2048 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_2048    0xC
/// set 1024 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_1024    0xB
/// set 512 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_512     0xA
/// set 256 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_256     0x9
/// set 128 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_128     0x8
/// set 64 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_64      0x7
/// set 32 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_32      0x6
/// set 16 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_16      0x5
/// set 8 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_8       0x4
/// set 4 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_4       0x3
/// set 2 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_2       0x2
/// set 1 words as a maximum that can be transfered within one DMA transaction
#define HSFR2_PROGRAMMABLE_BURST_LENGTH_1       0x1
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Block count and size register (SFR1) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Block count for current transfer mask
#define SFR1_BLOCK_COUNT            0xFFFF0000
/// DMA buffer size 4kB
#define SFR1_DMA_BUFF_SIZE_4KB      0x00000000
/// DMA buffer size 8kB
#define SFR1_DMA_BUFF_SIZE_8KB      0x00001000
/// DMA buffer size 16kB
#define SFR1_DMA_BUFF_SIZE_16KB     0x00002000
/// DMA buffer size 32kB
#define SFR1_DMA_BUFF_SIZE_32KB     0x00003000
/// DMA buffer size 64kB
#define SFR1_DMA_BUFF_SIZE_64KB     0x00004000
/// DMA buffer size 128kB
#define SFR1_DMA_BUFF_SIZE_128KB    0x00005000
/// DMA buffer size 265kB
#define SFR1_DMA_BUFF_SIZE_256KB    0x00006000
/// DMA buffer size 512kB
#define SFR1_DMA_BUFF_SIZE_512KB    0x00007000
/// DMA buffer size mask            
#define SFR1_DMA_BUFF_SIZE_MASK     0x00007000
/// Transfer block size mask
#define SFR1_BLOCK_SIZE             0x00000FFF
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Transfer mode and command information register (SFR3) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Abort CMD12, CMD52 for writing “I/O Abort?in CCCR 
#define SFR3_ABORT_CMD          0xC00000
/// Resume CMD52 for writing “Function Select?in CCCR
#define SRF3_RESUME_CMD         0x800000
///Suspend CMD52 for writing “Bus Suspend?in CCCR  
#define SFR3_SUSPEND_CMD        0x400000
/// data is present and will be transferred using the DAT line
#define SFR3_DATA_PRESENT       0x200000
/// index check enable
#define SFR3_INDEX_CHECK_EN     0x100000
/// response CRC check enable
#define SFR3_CRC_CHECK_EN       0x080000
/// response type - no response
#define SFR3_NO_RESPONSE        0x000000
/// response type - response length 136 
#define SFR3_RESP_LENGTH_136    0x010000
/// response type - response length 48
#define SFR3_RESP_LENGTH_48     0x020000
/// response type - response length 48 and check Busy after response 
#define SFR3_RESP_LENGTH_48B    0x030000
/// multi block DAT line data transfers
#define SFR3_MULTI_BLOCK_SEL    0x000020
/// data transfer direction - write
#define SFR3_TRANS_DIRECT_WRITE 0x000000
/// data transfer direction - read
#define SFR3_TRANS_DIRECT_READ  0x000010
/// Auto CMD12 enable
#define SFR3_AUTOCMD12_ENABLE   0x000004
/// Block count enable
#define SFR3_BLOCK_COUNT_ENABLE 0x000002
/// DMA enable
#define SFR3_DMA_ENABLE         0x000001
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Present state register masks (SFR9) - masks
//-------------------------------------------------------------------------------------------
//@{
/// CMD line signal level
#define SFR9_CMD_SIGNAL_LEVEL   0x1000000
/// DAT3 signal level
#define SFR9_DAT3_SIGNAL_LEVEL  0x0800000
/// DAT2 signal level
#define SFR9_DAT2_SIGNAL_LEVEL  0x0400000
/// DAT1 signal level
#define SFR9_DAT1_SIGNAL_LEVEL  0x0200000  
/// DAT0 signal level
#define SFR9_DAT0_SIGNAL_LEVEL  0x0100000 
/// Write protect switch pin level
#define SFR9_WP_SWITCH_LEVEL    0x0080000 
/// Card detect pin level
#define SFR9_CARD_DETECT_LEVEL  0x0040000 
/// Card state stable
#define SFR9_CARD_STATE_STABLE  0x0020000
/// Card inserted
#define SFR9_CARD_INSERTED      0x0010000 
/// Buffer read enable
#define SFR9_BUFF_READ_EN       0x0000800
/// Buffer write enable
#define SFR9_BUFF_WRITE_EN      0x0000400
/// Read transfer active
#define SFR9_READ_TRANS_ACTIVE  0x0000200
/// Write transfer active
#define SFR9_WRITE_TRANS_ACTIVE 0x0000100
/// DAT line active
#define SFR9_DAT_LINE_ACTIVE    0x0000004
/// Command Inhibit (DAT)
#define SFR9_CMD_INHIBIT_DAT    0x0000002
/// Command Inhibit (CMD)
#define SFR9_CMD_INHIBIT_CMD    0x0000001

//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @anchor Voltages
/// @name Host control settings #0 (SFR10) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Wakeup event enable on SD card removal
#define SFR10_WAKEUP_EVENT_CARD_REM_ENABLE  0x4000000
/// Wakeup event enable on SD card inserted
#define SFR10_WAKEUP_EVENT_CARD_INS_ENABLE  0x2000000
/// Wakeup event enable on SD card interrupt
#define SFR10_WAKEUP_EVENT_CARD_INT_ENABLE  0x1000000
/// Interrupt at block gap for a muliple transfer
#define SFR10_INTERRUPT_BLOCK_GAP           0x0080000
/// Read wait control
#define SFR10_READ_WAIT_CONTROL             0x0040000
/// Continue request
#define SFR10_CONTINUE_REQUEST              0x0020000
/// Stop at block gap request
#define SFR10_STOP_AT_BLOCK_GAP             0x0010000
/// SD bus voltage - 3.3V
#define SFR10_SET_3_3V_BUS_VOLTAGE          0x0000E00
/// SD bus voltage - 3.0V
#define SFR10_SET_3_0V_BUS_VOLTAGE          0x0000C00
/// SD bus voltage - 1.8V
#define SFR10_SET_1_8V_BUS_VOLTAGE          0x0000A00
/// SD bus voltage mask
#define SFR10_BUS_VOLTAGE_MASK              0x0000E00
/// SD bus power. The SD device is powered.
#define SFR10_SD_BUS_POWER                  0x0000100


#define SFR10_DMA_SELECT_SDMA   (0x0 << 3)
#define SFR10_DMA_SELECT_ADMA1  (0x1 << 3)
#define SFR10_DMA_SELECT_ADMA2  (0x2 << 3)
#define SFR10_DMA_SELECT_MASK   (0x3 << 3)

/// High speed enable.
#define SFR10_HIGH_SPEED_ENABLE             0x0000004
/// Set 4 bit data transfer width
#define SFR10_DATA_WIDTH_4BIT               0x0000002
/// Turning on the LED.
#define SFR10_TURN_ON_LED                   0x0000001
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Host control settings #1 (SFR11) - masks
//-------------------------------------------------------------------------------------------
//{@
/// Software reset for DAT line
#define SFR11_SOFT_RESET_DAT_LINE           0x4000000
/// Software reset for CMD line
#define SFR11_SOFT_RESET_CMD_LINE           0x2000000
/// Software reset for all. Restart entrie controller except the card detection circuit.
#define SFR11_SOFT_RESET_ALL                0x1000000

/// Data timeout TMCLK x 2 raised to the 27-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_27    0x00E0000
/// Data timeout TMCLK x 2 raised to the 26-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_26    0x00D0000
/// Data timeout TMCLK x 2 raised to the 25-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_25    0x00C0000
/// Data timeout TMCLK x 2 raised to the 24-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_24    0x00B0000
/// Data timeout TMCLK x 2 raised to the 23-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_23    0x00A0000
/// Data timeout TMCLK x 2 raised to the 22-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_22    0x0090000
/// Data timeout TMCLK x 2 raised to the 21-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_21    0x0080000
/// Data timeout TMCLK x 2 raised to the 20-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_20    0x0070000
/// Data timeout TMCLK x 2 raised to the 19-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_19    0x0060000
/// Data timeout TMCLK x 2 raised to the 18-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_18    0x0050000
/// Data timeout TMCLK x 2 raised to the 17-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_17    0x0040000
/// Data timeout TMCLK x 2 raised to the 16-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_16    0x0030000
/// Data timeout TMCLK x 2 raised to the 15-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_15    0x0020000
/// Data timeout TMCLK x 2 raised to the 14-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_14    0x0010000
/// Data timeout TMCLK x 2 raised to the 13-th power
#define SFR11_TIMEOUT_TMCLK_X_2_POWER_13    0x0000000
/// Data timeout mask
#define SFR11_TIMEOUT_MASK                  0x00F0000

/// SDCLK Frequency select. Divide base clock by 256.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_256     0x0008000
/// SDCLK Frequency select. Divide base clock by 128.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_128     0x0004000
/// SDCLK Frequency select. Divide base clock by 64.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_64      0x0002000
/// SDCLK Frequency select. Divide base clock by 32.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_32      0x0001000
/// SDCLK Frequency select. Divide base clock by 16.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_16      0x0000800
/// SDCLK Frequency select. Divide base clock by 8.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_8       0x0000400
/// SDCLK Frequency select. Divide base clock by 4.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_4       0x0000200
/// SDCLK Frequency select. Divide base clock by 2.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_2       0x0000100
/// SDCLK Frequency select. Divide base clock by 1.
#define SFR11_SEL_FREQ_BASE_CLK_DIV_1       0x0000000
/// SDCLK Frequency mask                    
#define SFR11_SEL_FREQ_BASE_MASK            0x000FF00

/// SD clock enable
#define SFR11_SD_CLOCK_ENABLE              0x0000004
/// Internal clock stable
#define SFR11_INT_CLOCK_STABLE             0x0000002
/// internal clock enable
#define SFR11_INT_CLOCK_ENABLE             0x0000001
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Interrupt status register (SFR12) - masks
//-------------------------------------------------------------------------------------------
//@{
/// ADMA error
#define SFR12_ADMA_ERROR	        0x2000000
/// Auto CMD12 error
#define SFR12_AUTO_CMD12_ERROR      0x1000000
/// Current limit error host controller is not supplying power to SD card due some failure.
#define SFR12_CURRENT_LIMIT_ERROR   0x0800000
/// Data end bit error
#define SFR12_DATA_END_BIT_ERROR    0x0400000
/// Data CRC error
#define SFR12_DATA_CRC_ERROR        0x0200000
/// Data timeout error
#define SFR12_DATA_TIMEOUT_ERROR    0x0100000
/// Command index error. Index error occurs in the command response.
#define SFR12_COMMAND_INDEX_ERROR   0x0080000
/// Command end bit error
#define SFR12_COMMAND_END_BIT_ERROR 0x0040000
/// Command CRC error
#define SFR12_COMMAND_CRC_ERROR     0x0020000
/// Command timeout error
#define SFR12_COMMAND_TIMEOUT_ERROR 0x0010000
/// Error interrupt
#define SFR12_ERROR_INTERRUPT       0x0008000
/// Card interrupt
#define SFR12_CARD_INTERRUPT        0x0000100
/// Card removal
#define SFR12_CARD_REMOVAL          0x0000080
/// Card insertion
#define SFR12_CARD_INSERTION        0x0000040
/// Buffer read ready. Host is ready to read the buffer.
#define SFR12_BUFFER_READ_READY     0x0000020
/// Buffer write ready. Host is ready for writing data to the buffer.
#define SFR12_BUFFER_WRITE_READY    0x0000010
/// DMA interrupt
#define SFR12_DMA_INTERRUPT         0x0000008
/// Block gap event
#define SFR12_BLOCK_GAP_EVENT       0x0000004
/// Transfer complete
#define SFR12_TRANSFER_COMPLETE     0x0000002
/// Command complete
#define SFR12_COMMAND_COMPLETE      0x0000001
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Interrupt status enable register (SFR13) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Auto CMD12 error status enable
#define SFR13_AUTO_CMD12_ERR_STAT_EN        0x1000000
/// Current limit error status enable
#define SFR13_CURRENT_LIMIT_ERR_STAT_EN     0x0800000
/// Data end bit error status enable
#define SFR13_DATA_END_BIT_ERR_STAT_EN      0x0400000
/// Data CRC error status enable
#define SFR13_DATA_CRC_ERR_STAT_EN          0x0200000
/// Data timeout error status enable
#define SFR13_DATA_TIMEOUT_ERR_STAT_EN      0x0100000
/// Command index error status enable
#define SFR13_COMMAND_INDEX_ERR_STAT_EN     0x0080000
/// Command end bit error status enable
#define SFR13_COMMAND_END_BIT_ERR_STAT_EN   0x0040000
/// Command CRC error status enable
#define SFR13_COMMAND_CRC_ERR_STAT_EN       0x0020000
/// Command timeout error status enable
#define SFR13_COMMAND_TIMEOUT_ERR_STAT_EN   0x0010000
/// Card interrupt status enable
#define SFR13_CARD_INTERRUPT_STAT_EN        0x0000100
/// Card removal status enable
#define SFR13_CARD_REMOVAL_STAT_EN          0x0000080
/// Card insertion status enable
#define SFR13_CARD_INERTION_STAT_EN         0x0000040
/// Buffer read ready status enable
#define SFR13_BUFFER_READ_READY_STAT_EN     0x0000020
/// Buffer write ready status enable
#define SFR13_BUFFER_WRITE_READY_STAT_EN    0x0000010
/// DMA interrupt status enable
#define SFR13_DMA_INTERRUPT_STAT_EN         0x0000008
/// Block gap event status enable
#define SFR13_BLOCK_GAP_EVENT_STAT_EN       0x0000004
/// Transfer complete status enable
#define SFR13_TRANSFER_COMPLETE_STAT_EN     0x0000002
/// Command complete status enable
#define SFR13_COMMAND_COMPLETE_STAT_EN      0x0000001
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Interrupt signal enable register (SFR14) - masks
//-------------------------------------------------------------------------------------------
//{@
/// Auto CMD12 error signal enable
#define SFR14_AUTO_CMD12_ERR_SIG_EN        0x1000000
/// Current limit error signal enable
#define SFR14_CURRENT_LIMIT_ERR_SIG_EN     0x0800000
/// Data end bit error signal enable
#define SFR14_DATA_END_BIT_ERR_SIG_EN      0x0400000
/// Data CRC error signal enable
#define SFR14_DATA_CRC_ERR_SIG_EN          0x0200000
/// Data timeout error signal enable
#define SFR14_DATA_TIMEOUT_ERR_SIG_EN      0x0100000
/// Command index error signal enable
#define SFR14_COMMAND_INDEX_ERR_SIG_EN     0x0080000
/// Command end bit error signal enable
#define SFR14_COMMAND_END_BIT_ERR_SIG_EN   0x0040000
/// Command CRC error signal enable
#define SFR14_COMMAND_CRC_ERR_SIG_EN       0x0020000
/// Command timeout error signal enable
#define SFR14_COMMAND_TIMEOUT_ERR_SIG_EN   0x0010000
/// Card interrupt signal enable
#define SFR14_CARD_INTERRUPT_SIG_EN        0x0000100
/// Card removal signal enable
#define SFR14_CARD_REMOVAL_SIG_EN          0x0000080
/// Card insertion signal enable
#define SFR14_CARD_INERTION_SIG_EN         0x0000040
/// Buffer read ready signal enable
#define SFR14_BUFFER_READ_READY_SIG_EN     0x0000020
/// Buffer write ready signal enable
#define SFR14_BUFFER_WRITE_READY_SIG_EN    0x0000010
/// DMA interrupt signal enable
#define SFR14_DMA_INTERRUPT_SIG_EN         0x0000008
/// Block gap event signal enable
#define SFR14_BLOCK_GAP_EVENT_SIG_EN       0x0000004
/// Transfer complete signal enable
#define SFR14_TRANSFER_COMPLETE_SIG_EN     0x0000002
/// Command complete signal enable
#define SFR14_COMMAND_COMPLETE_SIG_EN      0x0000001
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Auto CMD12 error status register (SFR15) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Command not issued bu auto CMD12 error
#define SFR15_CMD_NOT_ISSUED_ERR        0x80
/// Auto CMD12 index error
#define SFR15_AUTO_CMD12_INDEX_ERR      0x10
/// Auto CMD12 end bit error
#define SFR15_AUTO_CMD12_END_BIT_ERR    0x08
/// Auto CMD12 CRC error
#define SFR15_AUTO_CMD12_CRC_ERR        0x04
/// Auto CMD12 timeout error
#define SFR15_AUTO_CMD12_TIMEOUT_ERR    0x02
/// Autp CMD12 not executed
#define SFR15_AUTO_CMD12_NOT_EXECUTED   0x01
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Capabilities register (SFR16) - masks
//-------------------------------------------------------------------------------------------
//@{
/// 64-bit System Bus Support
#define SFR16_VOLTAGE_64BIT_SUPPORT     0x08000000
/// Voltage 1.8V is supported
#define SFR16_VOLTAGE_1_8V_SUPPORT      0x04000000
/// Voltage 3.0V is supported
#define SFR16_VOLTAGE_3_0V_SUPPORT      0x02000000
/// Voltage 3.3V is supported
#define SFR16_VOLTAGE_3_3V_SUPPORT      0x01000000
/// Suspend and resume functionality are supported
#define SFR16_RESUME_SUPPORT            0x00800000
/// Host controller is capable of using SDMA
#define SFR16_DMA_SUPPORT               0x00400000
/// Host controller and the host system support High Speed mode.
#define SFR16_HIGH_SPEED_SUPPORT        0x00200000
/// Host controller is capable of using ADMA1
#define SFR16_ADMA1_SUPPORT        		0x00100000
/// Host controller is capable of using ADMA2
#define SFR16_ADMA2_SUPPORT        		0x00080000
/// 512 is the maximum block size that can be written to the buffer in the Host Controller.
#define SFR16_MAX_BLOCK_LENGTH_512      0x00000000
/// 1024 is the maximum block size that can be written to the buffer in the Host Controller.
#define SFR16_MAX_BLOCK_LENGTH_1024     0x00010000
/// 2048 is the maximum block size that can be written to the buffer in the Host Controller.
#define SFR16_MAX_BLOCK_LENGTH_2048     0x00020000
/// timeout unit clock is MHz
#define SFR16_TIMEOUT_CLOCK_UNIT_MHZ    0x00000100
/// this macro can be used to get base clock frequency for SD clock, from a value which was read from the SFR16 register
#define SFR16_GET_BASE_CLK_FREQ_MHZ(VALUE) ( ( VALUE >> 8 ) & 0x3F )
/// this macro can be used to get timeout clock frequency from a value which was read from the SFR16 register
#define SFR16_GET_TIMEOUT_CLK_FREQ(VALUE) ( VALUE & 0x1F )
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Maximum current capabilities register (SFR18) - masks
//-------------------------------------------------------------------------------------------
//@{
//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name ADMA Error Status Register
//-------------------------------------------------------------------------------------------
//@{
/// ADMA Length Mismatch Error
#define SFR21_ADMA_ERROR_LENGTH_MISMATCH	(0x1uL << 2)
/// ADMA machine Stopped
#define SFR21_ADMA_ERROR_STATE_STOP			(0x0uL << 0)
/// ADMA Fetching descriptor
#define SFR21_ADMA_ERROR_STATE_FDS			(0x1uL << 0)
/// ADMA machine Transfer data
#define SFR21_ADMA_ERROR_STATE_TRF			(0x3uL << 0)

//@}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
/// @name Host controller version/ slot interrupt status register (SFR63) - masks
//-------------------------------------------------------------------------------------------
//@{
/// Interrupt signal for slot 0
#define SFR63_SLOT0_INTERRUPT       0x0
/// Interrupt signal for slot 1
#define SFR63_SLOT1_INTERRUPT       0x1
/// Interrupt signal for slot 2
#define SFR63_SLOT2_INTERRUPT       0x2
/// Interrupt signal for slot 3
#define SFR63_SLOT3_INTERRUPT       0x3
/// Interrupt signal for slot 4
#define SFR63_SLOT4_INTERRUPT       0x4
/// Interrupt signal for slot 5
#define SFR63_SLOT5_INTERRUPT       0x5
/// Interrupt signal for slot 6
#define SFR63_SLOT6_INTERRUPT       0x6
/// Interrupt signal for slot 7
#define SFR63_SLOT7_INTERRUPT       0x7
//-------------------------------------------------------------------------------------------

/// Maximum size of response in bytes
#define MAX_CARD_RESPONSE_BYTES 120 

//---------------------------------------------------------------------------
/// @name Endian conversions.
//---------------------------------------------------------------------------
//@{
/// Convert 2 bytes data to little endian format
#define CPUToLE16(a)                (a)
/// Convert 4 bytes data to little endian format
#define CPUToLE32(a)                (a)
/// Convert 2 bytes data from little endian format
#define LEToCPU16(a)                (a)
/// Convert 4 bytes data from little endian format
#define LEToCPU32(a)                (a)
//@}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @anchor WakeupCondition
/// @name Wakeup condition definition
//---------------------------------------------------------------------------
//@{
/// Wake up host when card is instert
#define SDIOHOST_WAKEUP_COND_CARD_INS       0x1
/// Wake up host when card is remove
#define SDIOHOST_WAKEUP_COND_CARD_REM       0x2
/// Wake up host when card interrupt occur
#define SDIOHOST_WAKEUP_COND_CARD_INT       0x4
//@}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @name CCCR transfer direction definitions
//---------------------------------------------------------------------------
//@{
/// Read data from CCCR register
#define SDIOHOST_CCCR_READ  0
/// Write data to CCCR register
#define SDIOHOST_CCCR_WRITE 1
//@}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/// @anchor HostConfigureCmd
/// @name Host configure commands
//---------------------------------------------------------------------------
//@{
/// Set host clock frequency
#define SDIOHOST_CONFIG_SET_CLK             0x1
/// Set host bus width
#define SDIOHOST_CONFIG_SET_BUS_WIDTH       0x2
/// Set timout on DAT line
#define SDIOHOST_CONFIG_SET_DAT_TIMEOUT     0x3
/// Disable signal interrupts
#define SDIOHOST_DISABLE_SIGNAL_INTERRUPT   0x4
/// Restore signal interrupts
#define SDIOHOST_RESTORE_SIGNAL_INTERRUPT   0x5
//@}
//---------------------------------------------------------------------------

////////////SDIO

/// Enable interrupt signal support form SDIO Host driver 
#define SDIO_INTERRUPT_SUPPORT  1

/// SDMA support for SDIO host driver
#define SDIO_SDMA_SUPPORTED     1
/// ADMA1 support for SDIO host driver
#define SDIO_ADMA1_SUPPORTED    0
/// ADMA2 support for SDIO host driver
#define SDIO_ADMA2_SUPPORTED    1

/// size of DSR circual buffer, it shodn't be greater than 127
#define DSR_COUNT               30
/// Debug level possible values (0, 1, 2, 3)
#define DEBUG_LEVEL             1
/// Choose SD bus mode SD or SPI ( currently only SD bus mode is supported )
#define HOST_BUS_MODE           SDIOHOST_BUS_MODE_SD
/// Set debouncing period
#define DEBOUNCING_TIME         0x300000
/// Commands timeout is a iteration count after which timeout error will be report
/// if a command will not execute (mainly using in WaitForValue function)
#define COMMANDS_TIMEOUT        100000
/// define this definition if you use Tasking compiler. Define it as 0 otherwise.
#define TASKING                 1
// system clock in Hz 
#define SYTEM_CLK_KHZ (8000)


////////////DMA


#define ADMA1_SIZE_OF_DESCRIPTOR 32
#define ADMA2_SIZE_OF_DESCRIPTOR 64


/** ADMA1 Descriptor Fields */

/// Set address/lenght field
#define ADMA1_DESCRIPTOR_ADDRESS_LENGHT(Val) (Val << 12)
/// No operation ?go to next descriptor on the list.
#define ADMA1_DESCRIPTOR_TYPE_NOP   (0x0 << 4)
/// Set data page length and go to next descriptor on the list.
#define ADMA1_DESCRIPTOR_TYPE_SET   (0x1 << 4)
/// Transfer data from the pointed page and go to next descriptor on the list.
#define ADMA1_DESCRIPTOR_TYPE_TRAN  (0x2 << 4)
/// Go to the next descriptor list
#define ADMA1_DESCRIPTOR_TYPE_LINK  (0x3 << 4)
/// the ADMA interrupt is generated 
/// when the ADMA1 engine finishes processing the descriptor.
#define ADMA1_DESCRIPTOR_INT        (0x1 << 2)
/// it signals termination of the transfer
/// and generates Transfer Complete Interrupt 
/// when this transfer is completed
#define ADMA1_DESCRIPTOR_END        (0x1 << 1)
/// it indicates the valid descriptor on a list
#define ADMA1_DESCRIPTOR_VAL        (0x1 << 0)

/// Set address/lenght field
#define ADMA2_DESCRIPTOR_LENGHT(Val) ((u32) Val << 16)
/// No operation ?go to next descriptor on the list.
#define ADMA2_DESCRIPTOR_TYPE_NOP   (0x0 << 4)
/// Transfer data from the pointed page and go to next descriptor on the list.
#define ADMA2_DESCRIPTOR_TYPE_TRAN  (0x2 << 4)
/// Go to the next descriptor list
#define ADMA2_DESCRIPTOR_TYPE_LINK  (0x3 << 4)
/// the ADMA interrupt is generated 
/// when the ADMA1 engine finishes processing the descriptor.
#define ADMA2_DESCRIPTOR_INT        (0x1 << 2)
/// it signals termination of the transfer
/// and generates Transfer Complete Interrupt 
/// when this transfer is completed
#define ADMA2_DESCRIPTOR_END        (0x1 << 1)
/// it indicates the valid descriptor on a list
#define ADMA2_DESCRIPTOR_VAL        (0x1 << 0)

//---------------------------------------------------------------------------
/// @name OCR register bits defnitions of SD memory cards
//---------------------------------------------------------------------------
//{@
#define SDCARD_REG_OCR_2_7_2_8  (1 << 15)
#define SDCARD_REG_OCR_2_8_2_9  (1 << 16)
#define SDCARD_REG_OCR_2_9_3_0  (1 << 17)
#define SDCARD_REG_OCR_3_0_3_1  (1 << 18)
#define SDCARD_REG_OCR_3_1_3_2  (1 << 19)
#define SDCARD_REG_OCR_3_2_3_3  (1 << 20)
#define SDCARD_REG_OCR_3_3_3_4  (1 << 21)
#define SDCARD_REG_OCR_3_4_3_5  (1 << 22)
#define SDCARD_REG_OCR_3_5_3_6  (1 << 23)
/// card capacity status (this bit is don't aviable in the SDIO cards)
#define SDCARD_REG_OCR_CCS      (1 << 30 )
/// card power up busy status (this bit is don't aviable in the SDIO cards)
#define SDCARD_REG_OCR_READY    (1 << 31 )
//@}
//---------------------------------------------------------------------------

//*****************************************************************************************************
//#define MMC_BUS_WIDTH_8		4
//*****************************************************************************************************
#endif
