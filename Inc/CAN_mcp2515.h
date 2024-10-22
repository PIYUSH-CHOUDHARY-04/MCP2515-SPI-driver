#ifndef CAN_MCP2515_H 
#define CAN_MCP2515_H

/**
 * File: CAN_mcp2515.h
 * Description: This header file contains the declaration of IO and Config APIs of MCP2515 and relevant objects.
 * Author: Piyush Choudhary
 * Date: October 3, 2024
 * Version: 1.0
 */

/**
 * 			MCP2515 standalone CAN controller with SPI interface's CAN API Prototypes.
 */


#include "HAL_SPI.h"
#include "HAL_GPIO.h"

#include "MCP2515.h"
#include "Pin_connection.h"




/**
 * @brief CAN frame structure for frame transmission and reception.
 * @param uint32_t CAN_ID will hold CAN ID of the specific CAN frame.
 * @param uint_8 data[8] will hold the data transported by CAN frame.
 * @param uint8_t dlc will hold DLC of the CAN frame i.e. the length of data carried by frame ( ranges between 0 to 8 bytes )
 */

typedef struct{
	uint32_t CAN_ID;
	uint8_t dlc;
	uint8_t data[8];
} can_t ;


/**
 * @brief mode switch structure is used during switiching mode from one to another operational mode.
 * @param uint8_t Switch2Mode will hold the CONFIG_MODE macros.
 * @param uint8_t clkout will tell whether to enable or disable the clkout pin in new mode irrespective of what it is in current mode. it have some possible values listed below. 
 * 		- 0x00 for disabling the the CLKOUT pin in new mode.
 *		- 0x04 for enabling the CLKOUT pin at (OSC freq)
 *		- 0x05 for enabling the CLKOUT pin at (OSC freq)/2
 *		- 0x06 for enabling the CLKOUT pin at (OSC freq)/4
 *		- 0x07 for enabling the CLKOUT pin at (OSC freq)/8
 * @param uint8_t txnrts will tell whether to enable or disable a specific TXnRTS pin of MCP2515 in new mode or not irrespective of what their states are in current mode.
 *		- 0x00 for disabling all TXnRTS pins in new mode.
 *		- 0x01 for enabling the TX0RTS pin , disabling the remaining two.
 * 		- 0x02 for enabling the TX1RTS pin , disabling the remaining two.
 *		- 0x03 for enabling both TX0RTS and TX1RTS , disabling the remaining one.
 *		- 0x04 for enabling the TX2RTS pin , disabling the remaining two.
 *		- 0x05 for enabling both TX0RTS and TX2RTS , disabling the remaining one.
 *		- 0x06 for enabling both TX1RTS and TX2RTS , disabling the remaining one.
 *		- 0x07 for enabling all TXnRTS pins. 
 *		ORing can be done to enable or disable mulitple TXnRTS pins in new mode.
 * @param uint8_t rxnbf will tell whether to enable or disable a specific RXnBF pin of MCP2515 in new mode or not irrespective of what their states are in current mode.
 *		- 0x12 for disabling both RXnBF pins in new mode (pin function is enabled for default thus programmer can choose to use it as digital input pin or for RTS.)
 *		- 0x13 for enabling RX0BF, disabling RX1BF.
 *		- 0x14 for enabling RX1BF, disabling RX0BF.
 *		- 0x15 for enabling both RXnBF pins.
 */
typedef struct {
	uint8_t Switch2Mode;
	uint8_t clkout;
	uint8_t txnrts;
	uint8_t rxnbf;
} mode_switch ;



/*====================================================================<<Config APIs>>==============================================================*/

/**
 * @brief Performs hard reset by driving ~RESET low.
 * @param void
 * @retval void
 */
void CAN_HardReset(void);

/**
 * @brief Performs soft reset by sending RESET control command ( 0xC0 ) over SPI.
 * @param void
 * @retval void
 */
void CAN_SoftReset(void);

/**
 * @brief Writes to a specific MCP2515 register.
 * @param uint8_t* reg_addr passes the address of the specific register.
 * @param const uint8_t byte passes the 8 bit value to be stored in specific register.
 * @retval void
 */
void CAN_WriteRegister(uint8_t* reg_addr, const uint8_t* byte);

/**
 * @brief Reads 8 bit value from a specific register.
 * @param uint8_t* reg_addr passes the address of the specific register.
 * @param uint8_t* byte passes the address of the byte where the read value will be copied.
 * @retval void
 */
void CAN_ReadRegister(uint8_t* reg_addr, uint8_t* byte);

/**
 * @brief Modifies the specific bit of a given register.
 * @param uint8_t* reg_addr passes the address of the register which has to be modified.
 * @param uint8_t* mask passes the address of the mask which will tell which bits of the correspoding register have to be modified.
 * @param uint8_t* data will tells the value of bit for which mask bits are 1.
 * @retval void
 */
void CAN_BitModify( uint8_t* reg_addr, uint8_t* mask , uint8_t* data);

/**
 * @brief Writes the entire CAN frame to the specific TXB.
 * @param uint8_t* TXB passes the address of the specific TX buffer where we need to write the CAN frame.
 * @param can_t* can_frame passes the address of the frame (created by user program.)
 * @retval uint8_t tells whether the write operation is successful or not, on success returns 0 and on failure returns ETXBFULL.
uint8_t CAN_WriteFrame(uint8_t* TXB, can_t* can_frame );

/**
 * @brief Reads the entire CAN frame from the CAN controller into a specified program buffer.
 * @param uint8_t* passes the address of the specific RX buffer from where we need to copy the CAN frame.
 * @param can_t* can_frame passes the address of the CAN frame structure in the program memory where the CAN frame will be copied.
 * @retval uint8_t tell whether the read operation is successful or not, on success returns 0 and on failure returns ERXBEMPTY.
 */
uint8_t CAN_ReadFrame(uint8_t* RXB,can_t* can_frame);

/**
 * @brief Switches the operational mode of the MCP2515 CAN controller.
 * @param mode_switch* switch_info passes the address of the mode_switch structure which contains all details about mode change.
 * @retval void
 */
void CAN_SwitchMode(mode_switch* switch_info);

/**
 * @brief Clears all the previously enabled interrupt and only enables the specific interrupt which is passed to it as its argument.
 * @param uint8_t interrupt passes the interrupt value which is set into CANINTE register of MCP2515.
 * @retval void.
 */
void CAN_SetInterrupt(uint8_t interrupt);

/**
 * @brief Reads the CANINTE register and tells about the interrupts which are enable at that time.
 * @param uint8_t* interrupt passes the address of a byte where the CANINTE register will be copied for examination.
 * @retval void 
 */
void CAN_GetInterrupt(uint8_t* interrupt);

/**
 * @brief Enables specific interrupt, won't touch the previously enabled interrupts.
 * @param uint8_t interrupt
 * @retval void
 */
void CAN_EnableInterrupt(uint8_t interrupt);

/**
 * @brief Disables specific interrupt, won't touch the previously disabled interrupts.
 * @param uint8_t interrupt
 * @retval void
 */
void CAN_DisableInterrupt(uint8_t interrupt);

/**
 * @brief Sets specific baud rate for the subsequent communication over the CAN bus.
 * @param uint8_t baudrate passes the identifier for specific baudrate.
 * @retval void
 */
void CAN_SetBaudRate(uint32_t baudrate);

/**
 * @brief Gets the baudrate at which the CAN system is communicating.
 * @param uint32_t* passes address of the variable where the baudrate will be copied.
 * @retval void
 */
void CAN_GetBaudRate(uint32_t* baudrate);

/**
 * @brief Trigger request to transmit a CAN frame from a specific TXB over SPI.
 * @param uint8_t* TXB passes memory address of the TXn buffer corresponding to which RTS need to be triggered.
 * @retval uint8_t tells whether the TXnB contains CAN frame and can be transmitted or the specified TXBn is empty thus can't trigger RTS for that specified TXBn
 * 		- 0 if TXBn is empty.
 *		- 1 if TXBn is not empty and RTS has been triggered.
 */
uint8_t CAN_TriggerRTS_SPI(uint8_t* TXB);

/**
 * @brief Trigger request to transmit a CAN frame from a specific TXB via TXnRTS pin.
 * @param uint8_t* TXB passes memory address of the TXn buffer corresponding to which RTS need to be triggered.
 * @retval uint8_t tells whether the TXnB contains CAN frame and can be transmitted or the specified TXBn is empty thus can't trigger RTS for that specified TXBn
 * 		- 0 if TXBn is empty.
 *		- 1 if TXBn is not empty and RTS has been triggered.
 */
uint8_t CAN_TriggerRTS_PIN(uint8_t* TXB);

/**
 * @brief Immediately aborts the transmission for the specific TX buffer.
 * @param uint8_t* TXB will pass the address of the specific TXB whose transmission is supposed to be aborted.
 * @retval void
 */
void CAN_AbortTX(uint8_t* TXB);

/**
 * @brief Immediately aborts the trasnmission of all TXBs, generally used during mode switch.
 * @param void
 * @retval void
 */
void CAN_AbortAllTX(void);

/**
 * @brief Clears the ABAT bit of the CANCTRL in order to continue the transmission.
 * @param void
 * @retval void
 */
void CAN_UnAbortAllTX(void)

/**
 * @brief Gets the information whether CLKOUT pin is configured or not by reading the _CANCTRL register.
 * @param void
 * @retval uint8_t tells whether the CLKOUT pin is enabled or not.
 * 	- 1 means enabled.
 *	- 0 means disabled.
 */
uint8_t CAN_GetClkOut(void);

/**
 * @brief Enable or disable the CLKOUT pin of MCP2515.
 * @param uint8_t clkout_mode passes the CLKOUT mode i.e. whether to disable or enable the CLKOUT pin.
 *	- 0x00 : means disable the CLKOUT mode.
 *	- 0x04 : means enable the CLKOUT mode.
 * @retval void
 */
void CAN_SetClkOut(uint8_t clkout_mode);

/**
 * @brief Sets the clock frequency for external devices on pin 3 of MCP2515.
 * @param uint8_t prescalar passes the scalar value corresponding to which the clock frequency of CLKOUT will change.
 * 	- 0 : CLKOUT_freq = OSC_freq
 *	- 1 : CLKOUT_freq = OSC_freq/2
 * 	- 2 : CLKOUT_freq = OSC_freq/4
 * 	- 3 : CLKOUT_freq = OSC_freq/8
 * @retval void
 */
void CAN_SetClkOutFreq(uint8_t prescalar);

/**
 * @brief Transmits specified frame if TXB is not empty.
 * @param uint8_t* TXB passes the address of the TXB whose frame is attempted to be transmitted.
 * @retval uint8_t tells whether the transmission is successful or failed, returns 0 on success, returns ETXBFULL on failure.
uint8_t CAN_Transmit(uint8_t* TXB);

/**
 * @brief Marks the specified RX buffer as read i.e. new frame cane be received into that specific RX buffer.
 * @param uint8_t* passes the address of the RX buffer which has to be marked as read.
 * @retval void
 */
void CAN_MarkRead(uint8_t* RXB);

/**
 * @brief Enables One shot mode for transmission.
 * @param void
 * @retval void
 */
void CAN_EnableOSM(void);

/**
 * @brief Disables One shot mode for transmission.
 * @param void
 * @retval void
 */
void CAN_DisableOSM(void);

/**
 * @brief Changes the priority of a specific TX to a specific value (TXB with most priority will be given the CAN bus first for transmission).
 * @param uint8_t* TXB passes the address of the TXB whose priority has to be changed.
 * @param uint8_t Priority passes the priority value for the TXB.
 * @retval void
void CAN_ChangeTXPriority(uint8_t* TXB, uint8_t Priority);

/**
 * @brief Sets the filtering mode, i.e. whether to turn on or off filtering for specific RXB.
 * @param uint8_t* RXB passes the address of the RXB for which filtering has to be enabled or disabled.
 * @param uint8_t RXB_mode passes the mode value.
 * 	- 0x00 : Disables filtering i.e. all frames with both SID and EID will be received in the specific RXB.
 *	- 0x03 : Enables filtering i.e. frame's whose CAN ID satisfies specific requirement will be received into that specific buffer.
 * @retval void 
 */
void CAN_SetRXBMode(uint8_t* RXB, uint8_t RXB_mode);

/**
 * @brief Configure TXnRTS pins of the specific TXB.
 * @param uint8_t txnrts will tell whether to enable or disable a specific TXnRTS pin of MCP2515 in current mode.
 * 	  Macros can be used to configure the TXnRTS pins of MCP2515.
 * @retval void
 */
void CAN_ConfigTXnRTS(uint8_t txnrts);

/**
 * @brief Configure RXnBF pins of the specific RXB, must be used in configuration mode only!
 * @param uint8_t rxnbf will tell whether to enable or disable a specific RXnBF pin of MCP2515 in new mode or not irrespective of what their states are in current mode.
 *	  Macros cann be used to configure the RXnBF pins of MCP2515.
 * @retval void 
 */
void CAN_ConfigRXnBF(uint8_t rxnbf);

/**
 * @brief Tells whether the current TX/RX frame is a normal frame or RTR frame.
 * @param uint8_t* mXBn passes the address of TXBs/RXBs for which RTR bit has to be checked.
 * @retval uint8_t tells whether the specific frame is set/obtained as RTR frame or not.
 */
uint8_t CAN_GetRTR(uint8_t* mXBn);

/**
 * @brief Sets the specific TXBn frame to be normal or RTR frame.
 * @param uint8_t* TXBn passes the address of the TXB whose frame type property has to be changed.
 * @param uint8_t rtr_val tell whether to set the RTR bit to be 0 or 1, setting 0 means normal frame , setting it 1 means RTR frame.
 * @retval void
 */
void CAN_SetRTR(uint8_t* TXBn, uint8_t rtr_val);

/**
 * @brief Tells whether the specific TXB/RXB is normal or extended frame.
 * @param uint8_t* mXBn passes the address of RXB/TXB whose frame type has to be checked.
 * @retval uint8_t tells whether the frame type of TXB/RXB is normal or extended.
 */
uint8_t CAN_GetFrameType(uint8_t* mXBn);

/**
 * @brief Sets the frame type information i.e. sets the EXIDE bit of TXBnSIDL.
 * @param uint8_t* TXBn passes the address of the 
 * @param uint8_t exide_val pass the information whether to set exide bit to 0 or 1, if exide_val=0, exide bit is set to 0 i.e. normal frame type, and if exide_val=1, frame type is extended.
 * @retval void
 */
void CAN_SetFrameType(uint8_t* TXBn, uint8_t exide_val);

/**
 * @brief enables/disables roll over to RX1 in case RX0 is full or its read but not marked as read.
 * @param uint8_t roll_ovr passes the information whether to enable or disable the frame roll over to RX1 in case RX0 is full or not marked as read.
 * @retval void
 */
void CAN_RollOver2RX1(uint8_t roll_ovr);

/**
 * @brief Get the filters for RXBs.
 * @param uint8_t* RXB passes the address of the specific RXB.
 *
/*  

	FILTER AND MASK MODIFICATION.
	CONFIG MODE REGISTER MODIFICATION.
	'EFLG' REGISTER MODIFICATION.

*/


/*====================================================================<<IO APIS>>===================================================================*/



/**
 * @brief Initializes CAN MCP2515 hardware and prepare it for frame reception and transmission.
 * @param void
 * @retval void
 */
void CAN_init(void);

/**
 * @brief Transmits the CAN frame 
 * @param can_t* frame passes the CAN frame loaded with CAN ID, DLC, Data.
 * @retval uint8_t is 0 for tx failure, is 1 for tx success.
 */
uint8_t CAN_tx(can_t* frame);

/**
 * @brief Check if their is a CAN frame, if yes then it copies it from RXBn to memory via SPI buffers.
 * @param cant_t* frame passes the pointer to the CAN frame into which frame will be copied.
 * @retval uint8_t is 0 for rx failure, is 1 for rx success.
 */
uint8_t CAN_tx(can_t* frame);


#endif /* CAN_MCP2515_H */

