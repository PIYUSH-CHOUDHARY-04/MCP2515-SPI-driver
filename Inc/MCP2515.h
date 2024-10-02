#ifndef MCP2515_H
#define MCP2515_H



/**
 * File: MCP2515.h
 * Description: This header file contains the declaration of drivers for MCP2515 standalone CAN controller with SPI interface.
 * Author: Piyush Choudhary
 * Date: September 30, 2024
 * Version: 1.0
 */

#include "HAL_SPI.h"   /*!< provides access to processor's SPI drivers.  */
#include "HAL_GPIO.h"  /*!< provides access to processor's GPIO drivers. */

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
#define READ_RX1_IR 		0x94
#define READ_RX1_DR	 	0X96
#define LOAD_TX0_IR 		0x40 
#define LOAD_TX0_DR		0X41 
#define LOAD_TX1_IR 		0x42
#define LOAD_TX1_DR		0X43
#define LOAD_TX2_IR 		0x44
#define LOAD_TX2_DR	 	0X45
#define RTS_TX0 		0x81 
#define RTS_TX1 		0x82
#define RTS_TX2 		0x84
#define READ_STATUS 		0xA0
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
#define TEC		(const uint8_t*)0x01C
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

#define STANDARD_FRAME_TYPE 0 /*!< for standard CAN frame, passed to MCP2515_WriteCAN_ID(). */
#define EXTENDED_FRAME_TYPE 1 /*!< for extended CAN frame, passed to MCP2515_WriteCAN_ID(). */


/**
 * @defgroup CONFIG_MODE config_mode
 * @brief Macros representing the identifier values for each mode.
 * @{
 */

#define NORMAL_MODE 		0x000
#define SLEEP_MODE  		0x001
#define LOOPBACK_MODE		0x010
#define LISTEN_ONLY_MODE	0x011
#define CONFIGURATION_MODE	0x100

/**
 * @}
 */


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
 * @brief Prototypes of HAL GPIO drivers to be used by MCP2515 drivers, already included via HAL_GPIO.h
 * @{
 */

/* GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); 						*/
/* void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);				*/
/* void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); 							*/


/**
 *@}
 */


/**
 * @brief Prototypes of HAL SPI drivers to be used by MCP2515 drivers, already included via HAL_SPI.h
 * @{
 */

/* HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, const const uint8_t *pData, uint16_t Size, uint32_t Timeout);	*/ 
/* HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size, uint32_t Timeout);		*/ 


/**
 * @}
 */



/**
 * @defgroup MCP2515_drivers MCP2515_drivers
 * @brief function prototypes for MCP2515 drivers.
 * @{
 */


/**
 * @defgroup MCP2515_powerup_and_modeconfig PU_MC
 * @brief Group of functions for power up handling and mode configuration.
 * @{
 */

/**
 * @brief Initializes the MCP2515 CAN controller hardware in Normal mode by configuring configuration registers to their default values.
 * @param void
 * @retvalue void
 */
void MCP2515_Init(void);

/**
 * @brief Configures the operating mode of MCP2515
 * @param uint8_t mode passes a mode value.
 * @retvalue void
 */
void MCP2515_SwitchMode(uint8_t mode);
 
/**
 * @}
 */


/**
 * @defgroup MCP2515_IO MCP2515_IO
 * @brief Group of functions for register and buffer IO.
 * @{

/**
 * @brief Writes to a specific register.
 * @param const uint8_t* reg_addr passes address of register to which data has to be written.
 * @param uint8_t data passes data byte to be stored in specific register.
 * @retvalue void
 */
void MCP2515_WriteRegister(const uint8_t* reg_addr, uint8_t data);

/**
 * @brief Reads from a specific register.
 * @param const uint8_t* reg_addr passes address of register from which data has to be read.
 * @param uint8_t* byte passes the address of the byte where data has to be copied from register.
 * @retvalue void
 */
void MCP2515_ReadRegister(const uint8_t* reg_addr, uint8_t* byte);

/**
 * @brief Writes the CAN ID to any of the empty TX buffer, write 2 bytes for standard CAN frame and will write 4 bytes for extended CAN frame.
 *	  - for standard frame : TXBnSIDH and TXBnSIDL will be populated.
 *	  - for extended frame : TXBnSIDH, TXBnSIDL, TXBnEID8, TXBnEID0 will be populated.
 * @param uint8_t Frame_type passes the information about type of frame, whether its standard or extended.
 * @param uint8_t* CAN_ID passes the pointer to a uint8_t array holding CAN ID.
 * 	  This array must be of 2/4 bytes containing the CAN ID,  
 *	  - for 11 bit CAN ID, 2 bytes array holding CAN ID bits in sequence, 1st byte holding bits 8 to 10 and 2nd byte holding lower 8 bits of CAN ID i.e. bits 0 to 7.
 *	  - for 29 bit CAN ID, 4 bytes array holding CAN ID bits in sequence, 1st byte holding bits 24 to 28 ( initial 5 bits of CAN ID ), 2nd byte holds bits 16 to 23, 3rd byte holds bits 8 to 15, 4th byte holding bits 0 to 7.
 * @retvalue void
 */
void MCP2515_WriteCAN_ID(uint8_t Frame_type, uint8_t* CAN_ID);

/**
 * @brief Reads the 2 bytes of CAN ID if the frame type is standard as dictated by RXBnSIDL[3] and reads 4 bytes for extended frame.
 * @param uint8_t* CAN_ID_buff pases the pointer to the array where CAN ID will be compied.
 * @retvalue uint8_t returns 0 for standard frame and 1 for extended frame.
 */
uint8_t MCP2515_ReadCAN_ID(uint8_t* CAN_ID_Buff);

/**
 * @brief Writes the entire CAN frame data to the TXn buffers ( from TXBnSIDH to TXBnD7 )
 * 		- for standard CAN frame i.e. TXBnSIDL[3]=0 , 2 bytes for CAN ID will be written , 1 byte (lower 4 bits) for DLC will be written, 8 bytes of data will be written.
 * 		- for extended CAN frame i.e. TXBnSIDL[3]=1 , 4 bytes for CAN ID will be written , 1 byte (lower 4 bits) for DLC will be written, 8 bytes of data will be written.
 * @param uint8_t* usr_data passes a pointer to an array which contains bytes for CAN ID, DLC, Data.
 *		- array size is 3 to 11 bytes for standard CAN frame.
 *		- array size is 5 to 13 bytes for extended CAN frame.
 * @retvalue void
 */
void MCP2515_WriteCAN_Frame(uint8_t* usr_data);

/**
 * @brief Reads the entire CAN frame from RXn register ( from RXBnSIDH to RXBnD7 )
 * 		- for standard CAN frame i.e. RXBnSIDL[3]=0 , 3 to 11 bytes will be read.
 * 		- for extended CAN frame i.e. RXBnSIDL[3]=1 , 5 to 13 bytes will be read.
 * @param uint8_t usr_data passes a pointer to an array which contains sequential space for CAN ID, DLC, Data.
 * 		- array size is 3 to 11 bytes for standard CAN frame. 
 *		- array size is 5 to 13 bytes for extended CAN frame.
 * @retvalue void
 */
void MCP2515_ReadCAN_Frame(uint8_t* usr_data);

/**
 * @brief Writes data field of CAN frame to the TXn buffers ( from TXnD0 to TXnD7 ) using DLC.
 * @param uint8_t* data passes a pointer to an uint8_t array of size 'DLC' bytes.
 * @param uint8_t DLC passes the length of the data for that frame, DLC number of bytes will be copied from user buffer to the TXBs.
 * @retvalue void
 */
void MCP2515_WriteData(uint8_t* data, uint8_t DLC);

/**
 * @brief Reads data from the RXBs and copies it to the user data buffer.
 * @param uint8_t* data passes a pointer to an uint8_t array of size 8 bytes since data length can be upto 8 bytes.
 * @retvalue uint8_t returns DLC for that frame.
 */
uint8_t MCP2515_ReadData(uint8_t* data);



/**
 * @defgroup MCP2515_INTERRUPT_CONFIG MCP2515_interrupt_handlers
 * @brief Group of functions for Interrupt handling and interrupt configuration.
 * @{

 /**
 * @brief Gets the information whether interrupts are enabled or not.
 * @param void
 * retvalue returns the value stored in register CANINTF
 */
uint8_t getInterrupts(void);

/**
 * @brief Sets the interrupt, previous interrupts flags will be cleared, any value passed to this will be set into the CANINTE register.
 * @param uint8_t interrupt passed by passing the INTERRUPT_MACROS.
 * @retvalue void
 */
void setInterrupts(uint8_t interrupt);

/**
 * @brief Enables specific interrupt.
 * @param uint8_t interrupt passes the interrupt identifier.
 * retvalue void
 */
void enableInterrupt(uint8_t interrupt);

/**
 * @brief Disable specific interrupt.
 * @param uint8_t interrupt passes the interrupt identifier.
 * @retvalue void
 */
void disableInterrupt(uint8_t interrupt);

#include "IRQHandlers.h"

/**
 * @}
 */















/**
 * @}
 */












#endif /* MCP2515_H */
