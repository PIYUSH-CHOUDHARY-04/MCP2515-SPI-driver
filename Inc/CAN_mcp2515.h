#ifndef CAN_MCP2515_H 
#define CAN_MCP2515_H

/**
 * File: CAN_mcp2515.h
 * Description: This header file contains the declaration of IO API of MCP2515 and relevant objects.
 * Author: Piyush Choudhary
 * Date: October 3, 2024
 * Version: 1.0
 */

#include "CAN_ioctl_config.h" /*!< Including low level CAN functionalities. */
#include "EXTI_IRQHandlers.h" 		/*!< Including prototypes for IRQHandlers. */


/**
 * @brief CAN frame structure for frame transmission and reception.
 * @attr uint32_t CAN_ID will hold CAN ID of the specific CAN frame.
 * @attr uint_8 data[8] will hold the data transported by CAN frame.
 * @attr uint8_t dlc will hold DLC of the CAN frame i.e. the length of data carried by frame ( ranges between 0 to 8 bytes )
 */

typedef struct{
	uint32_t CAN_ID;
	uint8_t data[8];
	uint8_t dlc;
} can_t ;



/**
 * 			MCP2515 standalone CAN controller with SPI interface's CAN API Prototypes
 * @brief This API set will contain 3 basic functions and some ioctl functions, though in this file only 3 are defined: CAN_init(), CAN_rx(), CAN_tx(), CAN_ioctl()
 */


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




