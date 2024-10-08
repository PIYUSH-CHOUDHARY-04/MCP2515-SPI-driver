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
void CAN_WriteRegister(uint8_t* reg_addr,const uint8_t* byte);

/**
 * @brief Reads 8 bit value from a specific register.
 * @param uint8_t* reg_addr passes the address of the specific register.
 * @param uint8_t* byte passes the address of the byte where the read value will be copied.
 * @retval void
 */
void CAN_ReadRegister(uint8_t* reg_addr, uint8_t* byte);

/**
 * @brief Writes the entire CAN frame to the specific TXB.
 * @param uint8_t* TXB passes the address of the specific TX buffer where we need to write the CAN frame.
 * @param can_t* can_frame passes the address of the frame (created by user program.)
 * @retval uint8_t tells whether the write operation is successful or not, on success returns 0 and on failure returns ETXBFULL.
uint8_t CAN_WriteFrame(uint8_t* TXB,can_t* can_frame );

/**
 * @brief Reads the entire CAN frame from the CAN controller into a specified program buffer.
 * @param uint8_t* passes the address of the specific RX buffer from where we need to copy the CAN frame.
 * @param can_t* can_frame passes the address of the CAN frame structure in the program memory where the CAN frame will be copied.
 * @retval uint8_t tell whether the read operation is successful or not, on success returns 0 and on failure returns ERXBEMPTY.
 */
uint8_t CAN_ReadFrame(uint8_t* RXB,can_t* can_frame);

/**
 * @brief Switches the operational mode of the MCP2515 CAN controller.
 * @param uint8_t mode passes the identifier of the mode to which the CAN controller must switch to.
 * @retval void
 */
void CAN_SwitchMode(uint8_t mode);

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
 * @brief Trigger request to transmit a CAN frame from a specific TXB.
 * @param uint8_t* TXB passes memory address of the TXn buffer corresponding to which RTS need to be triggered.
 * @retval uint8_t tells whether the TXnB contains CAN frame and can be transmitted or the specified TXnB is empty thus can't trigger RTS for that specified TXnB
 * 		- 0 if TXnB is empty.
 *		- 1 if TXnB is not empty and RTS has been triggered.
 */
uint8_t CAN_TriggerRTS(uint8_t* TXB);

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
 * @brief Activates the CLKOUT pin of MCP2515 i.e. Pin 3.
 * @param void
 * @retval void
 */
void EnableClkOut(void);

/**
 * @brief Disables the CLKOUT pin.
 * @param void
 * @retval void
 */
void Disable ClkOut(void);

/**
 * @brief Sets the clock frequency for external devices on pin 3 of MCP2515.
 * @param uint8_t prescalar passes the scalar value corresponding to which the clock frequency of CLKOUT will change.
 * 	- 0 : CLKOUT_freq = OSC_freq
 *	- 1 : CLKOUT_freq = OSC_freq/2
 * 	- 2 : CLKOUT_freq = OSC_freq/4
 * 	- 3 : CLKOUT_freq = OSC_freq/8
 * @retval void
 */
void SetClkOutFreq(uint8_t prescalar);

/**
 * @brief Transmits specified frame if TXB is not empty.
 * @param uint8_t* TXB passes the address of the TXB whose frame is attempted to be transmitted.
 * @retval uint8_t tells whether the transmission is successful or failed, returns 0 on success, returns ETXBFULL on failure.
uint8_t CAN_Transmit(uint8_t* TXB);

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
 * @brief Changes the priority of a specific TX to a specific value.
 * @param uint8_t* TXB passes the address of the TXB whose priority has to be changed.
 * @param uint8_t Priority passes the priority value for the TXB.
 * @retval void
void ChangeTXPriority(uint8_t* TXB, uint8_t Priority);




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

