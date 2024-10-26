#ifndef MCP2515_H
#define MCP2515_H



/**
 * File: MCP2515.h
 * Description: This header file contains the memory map of the MCP2515.
 * Author: Piyush Choudhary
 * Date: October 3, 2024
 * Version: 1.0
 */

/**
 * @defgroup Control_commands CC
 * @brief Macro definitions for control commands for MCP2515 operations.
 * @{
 */

#define RESET 			0xC0
#define READ 			0x03
#define WRITE 			0x02
#define READ_RX0_IR		0x90 
#define READ_RX0_DR	 	0x92 
#define READ_RX1_IR 	0x94
#define READ_RX1_DR	 	0X96
#define LOAD_TX0_IR 	0x40 
#define LOAD_TX0_DR		0X41 
#define LOAD_TX1_IR 	0x42
#define LOAD_TX1_DR		0X43
#define LOAD_TX2_IR 	0x44
#define LOAD_TX2_DR	 	0X45
#define RTS_TX0 		0x81 
#define RTS_TX1 		0x82
#define RTS_TX2 		0x84
#define READ_STATUS 	0xA0
#define RX_STATUS 		0xB0
#define BIT_MODIFY 		0x05

/**
 * @}
 */

/**
 * @defgroup Memory_map MM
 * @brief Macros representing memory address of the device registers and buffers.
 * @{
 */

#define RXF0SIDH 	(const uint8_t*)0x000
#define RXF0SIDL 	(const uint8_t*)0x001
#define RXF0EID8 	(const uint8_t*)0x002
#define RXF0EID0 	(const uint8_t*)0x003
#define RXF1SIDH 	(const uint8_t*)0x004
#define RXF1SIDL 	(const uint8_t*)0x005
#define RXF1EID8 	(const uint8_t*)0x006
#define RXF1EID0 	(const uint8_t*)0x007
#define RXF2SIDH 	(const uint8_t*)0x008
#define RXF2SIDL 	(const uint8_t*)0x009
#define RXF2EID8 	(const uint8_t*)0x00A
#define RXF2EID0 	(const uint8_t*)0x00B
#define _BFPCTRL 	(const uint8_t*)0x00C
#define _TXRTSCTRL 	(const uint8_t*)0x00D
#define CANSTAT		(const uint8_t*)0x00E
#define _CANCTRL 	(const uint8_t*)0x00F

#define RXF3SIDH 	(const uint8_t*)0x010
#define RXF3SIDL 	(const uint8_t*)0x011
#define RXF3EID8 	(const uint8_t*)0x012
#define RXF3EID0 	(const uint8_t*)0x013
#define RXF4SIDH 	(const uint8_t*)0x014
#define RXF4SIDL 	(const uint8_t*)0x015
#define RXF4EID8 	(const uint8_t*)0x016
#define RXF4EID0	(const uint8_t*)0x017
#define RXF5SIDH	(const uint8_t*)0x018
#define RXF5SIDL	(const uint8_t*)0x019
#define RXF5EID8	(const uint8_t*)0x01A
#define RXF5EID0	(const uint8_t*)0x01B
#define TEC		    (const uint8_t*)0x01C
#define REC	        (const uint8_t*)0x01D
//#define CANSTAT	(const uint8_t*)0x01E
//#define _CANCTRL	(const uint8_t*)0x01F

#define RXM0SIDH 	(const uint8_t*)0x020
#define RXM0SIDL	(const uint8_t*)0x021
#define RXM0EID8	(const uint8_t*)0x022
#define RXM0EID0	(const uint8_t*)0x023
#define RXM1SIDH	(const uint8_t*)0x024
#define RXM1SIDL	(const uint8_t*)0x025
#define RXM1EID8	(const uint8_t*)0x026
#define RXM1EID0	(const uint8_t*)0x027
#define _CNF3		(const uint8_t*)0x028
#define _CNF2		(const uint8_t*)0x029
#define _CNF1		(const uint8_t*)0x02A
#define _CANINTE	(const uint8_t*)0x02B
#define _CANINTF	(const uint8_t*)0x02C
#define _EFLG		(const uint8_t*)0x02D
//#define CANSTAT	(const uint8_t*)0x02E
//#define _CANCTRL	(const uint8_t*)0x02F

#define _TXB0CTRL	(const uint8_t*)0x030
#define TXB0SIDH	(const uint8_t*)0x031
#define TXB0SIDL	(const uint8_t*)0x032
#define TXB0EID8	(const uint8_t*)0x033
#define TXB0EID0	(const uint8_t*)0x034
#define TXB0DLC		(const uint8_t*)0x035
#define TXB0D0		(const uint8_t*)0x036
#define TXB0D1		(const uint8_t*)0x037
#define TXB0D2		(const uint8_t*)0x038
#define TXB0D3		(const uint8_t*)0x039
#define TXB0D4		(const uint8_t*)0x03A
#define TXB0D5		(const uint8_t*)0x03B
#define TXB0D6		(const uint8_t*)0x03C
#define TXB0D7		(const uint8_t*)0x03D
//#define CANSTAT	(const uint8_t*)0x03E
//#define _CANCTRL	(const uint8_t*)0x03F

#define _TXB1CTRL	(const uint8_t*)0x040
#define	TXB1SIDH	(const uint8_t*)0x041
#define TXB1SIDL	(const uint8_t*)0x042
#define TXB1EID8	(const uint8_t*)0x043
#define TXB1EID0	(const uint8_t*)0x044
#define TXB1DLC		(const uint8_t*)0x045
#define TXB1D0		(const uint8_t*)0x046
#define TXB1D1		(const uint8_t*)0x047
#define TXB1D2		(const uint8_t*)0x048
#define TXB1D3		(const uint8_t*)0x049
#define	TXB1D4		(const uint8_t*)0x04A
#define TXB1D5		(const uint8_t*)0x04B
#define TXB1D6		(const uint8_t*)0x04C
#define TXB1D7		(const uint8_t*)0x04D
//#define CANSTAT	(const uint8_t*)0x04E
//#define _CANCTRL	(const uint8_t*)0x04F

#define _TXB2CTRL 	(const uint8_t*)0x050
#define TXB2SIDH	(const uint8_t*)0x051
#define TXB2SIDL	(const uint8_t*)0x052
#define TXB2EID8	(const uint8_t*)0x053
#define TXB2EID0	(const uint8_t*)0x054
#define TXB2DLC		(const uint8_t*)0x055
#define TXB2D0		(const uint8_t*)0x056
#define TXB2D1		(const uint8_t*)0x057
#define	TXB2D2		(const uint8_t*)0x058
#define	TXB2D3		(const uint8_t*)0x059
#define	TXB2D4		(const uint8_t*)0x05A
#define	TXB2D5		(const uint8_t*)0x05B
#define TXB2D6		(const uint8_t*)0x05C
#define TXB2D7		(const uint8_t*)0x05D
//#define CANSTAT	(const uint8_t*)0x05E
//#define _CANCTRL	(const uint8_t*)0x05F

#define _RXB0CTRL	(const uint8_t*)0x060
#define RXB0SIDH	(const uint8_t*)0x061
#define RXB0SIDL	(const uint8_t*)0x062
#define RXB0EID8	(const uint8_t*)0x063
#define RXB0EID0	(const uint8_t*)0x064
#define RXB0DLC		(const uint8_t*)0x065
#define RXB0D0		(const uint8_t*)0x066
#define RXB0D1		(const uint8_t*)0x067
#define RXB0D2		(const uint8_t*)0x068
#define RXB0D3		(const uint8_t*)0x069
#define RXB0D4		(const uint8_t*)0x06A
#define RXB0D5		(const uint8_t*)0x06B
#define RXB0D6		(const uint8_t*)0x06C
#define RXB0D7		(const uint8_t*)0x06D
//#define CANSTAT	(const uint8_t*)0x06E
//#define _CANCTRL	(const uint8_t*)0x06F

#define _RXB1CTRL	(const uint8_t*)0x070
#define RXB1SIDH	(const uint8_t*)0x071
#define RXB1SIDL	(const uint8_t*)0x072
#define RXB1EID8	(const uint8_t*)0x073
#define RXB1EID0	(const uint8_t*)0x074
#define RXB1DLC		(const uint8_t*)0x075
#define RXB1D0		(const uint8_t*)0x076
#define RXB1D1		(const uint8_t*)0x077
#define RXB1D2		(const uint8_t*)0x078
#define RXB1D3		(const uint8_t*)0x079
#define RXB1D4		(const uint8_t*)0x07A
#define RXB1D5		(const uint8_t*)0x07B
#define RXB1D6		(const uint8_t*)0x07C
#define RXB1D7		(const uint8_t*)0x07D
//#define CANSTAT	(const uint8_t*)0x07E
//#define _CANCTRL	(const uint8_t*)0x07F

/**
 * @}
 */

#define STANDARD_FRAME_TYPE 0x00 /*!< for standard CAN frame, passed to MCP2515_WriteCAN_ID(). */
#define EXTENDED_FRAME_TYPE 0x01 /*!< for extended CAN frame, passed to MCP2515_WriteCAN_ID(). */
#define DATA_FRAME_TYPE     0x02 /*!< for data frames.   */
#define REMOTE_FRAME_TYPE   0x04 /*!< for remote frames. */

/**
 * @defgroup CONFIG_MODE config_mode
 * @brief Macros representing the identifier values for each mode.
 * @{
 */

#define NORMAL_MODE 		0x00
#define SLEEP_MODE  		0x01
#define LOOPBACK_MODE		0x02
#define LISTEN_ONLY_MODE	0x03
#define CONFIGURATION_MODE	0x04

/**
 * @}
 */

#define RXB_FILTER_ON	0x00	/*!< Enables filtering of frames for specific RX buffer.  */
#define RXB_FILTER_OFF	0x03	/*!< Disables filtering of frames for specific RX buffer. */


/**
 * @defgroup INTERRUPT_MACROS interrupt_macros
 * @brief Macros representing the identifier vaues for each interrupt.
 *        ORing of these macros can be done to represent more than just one interrupt.
 * @{
 */

#define INT_MERRE (uint8_t)0x080
#define INT_WAKIE (uint8_t)0x040
#define INT_ERRIE (uint8_t)0x020
#define INT_TX2IE (uint8_t)0x010
#define INT_TX1IE (uint8_t)0x008
#define INT_TX0IE (uint8_t)0x004
#define INT_RX1IE (uint8_t)0x002
#define INT_RX0IE (uint8_t)0x001

/**
 * @}
 */

/**
 * @defgroup TXRX_BUFF_IDENTIFIER txrx_buff_identifier
 * @brief Macros representing addresses for buffers, these addresses are the base address or address of TXnRXnCTRL registers, will be used to read or write on these RXnTXn buffers.
 * @{
 */

#define RX0 _RXB0CTRL
#define RX1 _RXB1CTRL
#define TX0 _TXB0CTRL
#define TX1 _TXB1CTRL
#define TX2 _TXB2CTRL

/**
 * @}
 */

/**
 * @defgroup MCP2515_BAUD_RATE mcp2515_baud_rate
 * @brief Macros for setting the Standard Baud rate for the CAN communication.
 * @{
 */

#define BR_100KBPS 0x00186A0 /*!< Baud rate 100 kilobits per second */
#define BR_200KBPS 0x0030D40 /*!< Baud rate 200 kilobits per second */
#define BR_300KBPS 0x00493E0 /*!< Baud rate 300 kilobits per second */
#define BR_400KBPS 0x0061A80 /*!< Baud rate 400 kilobits per second */
#define BR_500KBPS 0x007A120 /*!< Baud rate 500 kilobits per second */
#define BR_600KBPS 0x00927C0 /*!< Baud rate 600 kilobits per second */
#define BR_700KBPS 0x00AAE60 /*!< Baud rate 700 kilobits per second */
#define BR_800KBPS 0x00C3500 /*!< Baud rate 800 kilobits per second */
#define BR_900KBPS 0x00DBBA0 /*!< Baud rate 900 kilobits per second */
#define BR_1MBPS   0x00F4240 /*!< Baud rate 1 megabits per second   */


/**
 * @}
 */



#define TXPriority_1 0x03 /*!< Highest transmit priority for specific TX buffer. */
#define TXPriority_2 0x02 /*!< Intermediate High transmit priority for specific TX buffer. */
#define TXPriority_3 0x01 /*!< Intermediate Low transmit priority for specific TX buffer. */
#define TXPriority_4 0x00 /*!< Lowest transmit priority for specific TX buffer. */



/**
 * @brief Macros for configuring TXnRTS pins of MCP2515, disabled pin will be set to digital input mode.
 *	  ORing can be done for complex configurations.
 */

#define DI_RTS_ALL 	0x00 		/*!< Disables all RTS pins    */
#define EN_RTS0 	0x01 		/*!< Enables RTS pin for TXB0 */
#define EN_RTS1 	0x02 		/*!< Enables RTS pin for TXB1 */
#define EN_RTS2 	0x04 		/*!< Enables RTS pin for TXB2 */



/**
 * @brief Macros for configuring RXnBF pins of MCP2515.
 *	  pin function must be enabled for enabling each RXnBF pin i.e. ORing must be done else the RXnBF pin will only be activated and will remain in digital input mode.
 */

#define DI_RXnBF_ALL_PINFUNC	0x00
#define EN_RX0BF_PINFUNC	    0x04
#define EN_RX1BF_PINFUNC	    0x08
#define EN_RX0BF		        0x01
#define EN_RX1BF		        0x02	

/**
 * @brief Filter macros for accessing and configuring filters in configuration mode.
 *      filter0 and filter1 are for RXB0 and filter2 to filter5 are for RXB1.
 */

#define FILT0 RXF0SIDH
#define FILT1 RXF1SIDH
#define FILT2 RXF2SIDH
#define FILT3 RXF3SIDH
#define FILT4 RXF4SIDH
#define FILT5 RXF5SIDH

/**
 * @brief Mask macros for accessing and configuring filters in configuration mode.
 *      Mask0 i.e. RXM0 for RXB0 and Mask1 i.e. RXM1 for RXB1.
 */

#define MASK0 RXM0SIDH
#define MASK1 RXM1SIDH


/**
 * @brief common error codes for debugging.
 */

#define ETXBFULL 	0x0A  		/* Error occured when a Write operation to registers of certain TXBn fails due to all TXB being full, this happens when the transmission is in process. */
#define ERXBEMPTY	0x0B		/* Error occured when a read operation to registers of certain RXBn fails due to all RXB being empty. */
#define EINVALARG	0x0C        /* Error occured when invalid argument is passed to the specific function/routine. */
#define EINVALMODE  0x0D        /* Error occured when invalid operation mode is detected for certain operation. */


#endif /* MCP2515_H */
