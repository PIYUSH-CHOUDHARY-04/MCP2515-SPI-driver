#ifndef CAN_IOCTL_H
#define CAN_IOCTL_H

/**
 * File: MCP2515.h
 * Description: This header file contains the prototype declaration of IO Control and configuration functions.
 * Author: Piyush Choudhary
 * Date: October 3, 2024
 * Version: 1.0
 */

#include "HAL_SPI.h"
#include "HAL_GPIO.h"

#include "memory_maps/MCP2515.h"
#include "memory_maps/Pin_connection.h"





/**
 * @defgroup Group of low level functions for controlling MCP2515 CAN controller.
 * @brief Certain routines are defined here for configuring the MCP2515 CAN controller.
 * @{
 */

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
 * @param uint8_t byte passes the 8 bit value to be stored in specific register.
 * @retval void
 */
void CAN_WriteRegister(uint8_t* reg_addr, uint8_t* byte);

/**
 * @brief Reads 8 bit value from a specific register.
 * @param uint8_t* reg_addr passes the address of the specific register.
 * @param uint8_t* byte passes the address of the byte where the read value will be copied.
 * @retval void
 */
void CAN_ReadRegister(uint8_t* reg_addr, uint8_t* byte);

/**
 * @brief Writes CAN ID to one of the TX buffer's CAN ID registers.
 * @param uint8_t Frame_type passes information about the type of frame i.e. standard or extended frame.
 * 		- 0 for standard frame type
 *		- 1 for extended frame type
 * @param uint8_t* TXB passes the TX buffer address to which the user needs to write ( look at the Macros for buffer addresses in MCP2515.h )
 * @param const uint8_t* can_id passes memory address of the CAN ID array, if Frame_type is 0 i.e. standard CAN frame, then this will write 2 bytes to MCP2515 ( one byte to TXnSIDH and one byte to TXnSIDL. )
 *			and if Frame_type is 1 i.e. extended CAN frame, then this will write 4 bytes to MCP2515 ( one byte to each TXnSIDH, TXnSIDL, TXnEID8, TXnEID0 ).
 *			The array in user program holding CAN ID should have CAN ID alligned in bytes as per the CAN protocol.
 * @retval void
 */
void CAN_WriteID(uint8_t Frame_type, uint8_t* TXB, const uint8_t* can_id);

/**
 * @brief Reads CAN ID from one of the RX buffer and copies it into user buffer.
 * @param uint8_t* RXB passes the RXn buffer address.
 * @param uint8_t* can_id passes memory address of the user CAN ID buffer, must be atleast 4 byte long, will write to the first 2 to 4 bytes of can_id buffer in same way as implemented in CAN protocol.
 * @ retval uint8_t gives the info about Frame_type, 0 if standard CAN frame and 1 if extended CAN frame, can be used to traverse through can_id buffer.
 */
uint8_t CAN_ReadID(uint8_t* RXB, uint8_t* can_id);

/**
 * @brief Writes to the DLC field of the CAN frame.
 * @param uint8_t* TXB passes the buffer address for TXn buffers.
 * @param uint8_t* dlc passes the address of DLC to be stored in DLC field of CAN frame.
 * @retval void
 */
void CAN_WriteDLC(uint8_t* TXB, uint8_t* dlc);

/**
 * @brief Reads the DLC registers and wire the DLC value into specific byte.
 * @param uint8_t* RXB passes the buffer address for RXn buffers.
 * @param uint8_t* dlc passes memory address of a byte where DLC will be transferred.
 * @ retval void
 */
void CAN_ReadDLC(uint8_t* RXB, uint8_t* dlc);

/**
 * @brief Write 'dlc' bytes of data from program memory to MCP2515 TXn buffers
 * @param uint8_t* TXB passes the address for the TXn buffers.
 * @param uint8_t* dlc passes the length of the data toe be sent to CAN controller ( lies between 0 to 8 bytes ).
 * @param uint8_t dlc passes the number of bytes of data to be written to the Data field of the TXnB.
 * @retval void
 */
void CAN_WriteData(uint8_t* TXB, uint8_t* data, uint8_t dlc);

/**
 * @brief Read the data field from the RXn buffers and copies it to specified location.
 * @param uint8_t* RXB passes the address for RXn buffers.
 * @param uint8_t* data passes address of the array where the received CAN frame's data has to be copied ( array must be of sizeof(uint8_t)*8 atleast ).
 * @retval uint8_t tells the length of the data i.e. returns 'DLC' for the frame.
 */
uint8_t CAN_ReadData(uint8_t* RXB, uint8_t* data);

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

more error handling routines for error frames etc, go through all the registers for knowing about need of more routines for fine-grain control over MCP2515.

*/





/**
 * @}
 */

#endif /* CAN_IOCTL_H */