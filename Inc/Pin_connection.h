#ifndef PIN_CONN_H
#define PIN_CONN_H

/**
 * File: Pin_connection.h
 * Description: This header file contains MCP2515 pin connection macrosu.
 * Author: Piyush Choudhary
 * Date: October 3, 2024
 * Version: 1.0
 */


/**
 *			MCP2515 PIN CONFIGURATION FILE.
 * @brief This is a configuration file, everytime you connect your MCP2515 standalone CAN controller with SPI interface with the MCU, you need to declare here the Pin connections of MCP2515 with MCU.
 *	 X_PORT is a pointer of type GPIO_TypeDef* and these structures and pointers are initialized by the HAL library for each port, and X_PIN is a uint32_t block which represents the specific pin of the  
 *	 corresponding port.
 *	 Look at the HAL_GPIO.h file, X_PIN macros are defined there.
 * 	 For no connection of the any of the following pin, both X_PORT and X_PIN can be set to 0x0000.
 */



#define nRESET_PORT  	<some-port-number>
#define nRESET_PIN   	<some-pin-number>	/*!< Connect with GPIO_OUTPUT pin of the MCU. */

#define nCS_PORT	<some-port-number>	
#define nCS_PIN   	<some-pin-number>	/*!< Connect with GPIO_OUTPUT pin of the MCU. */

#define SO_PORT		<some-port-number>
#define SO_PIN   	<some-pin-number>	/*!< Connect with SPIn_MISO pin of the MCU. */

#define SI_PORT		<some-port-number>
#define SI_PIN   	<some-pin-number>	/*!< Connect with SPIn_MOSI pin of the MCU. */

#define SCK_PORT	<some-port-number>
#define SCK_PIN   	<some-pin-number>	/*!< Connect with SPI_SCK pin of the MCU. */

#define nINT_PORT	<some-port-number>
#define nINT_PIN   	<some-pin-number>	/*!< Connect with GPIO_EXTIn pin of the MCU. */

#define nRX0BF_PORT	<some-port-number>	
#define nRX0BF_PIN   	<some-pin-number>	/*!< Connect with any available GPIO_EXTIn pin of the MCU. */

#define nRX1BF_PORT	<some-port-number>
#define nRX1BF_PIN   	<some-pin-number>	/*!< Connect with any available GPIO_EXTIn pin of the MCU. */	

#define nTX0RTS_PORT	<some-port-number>
#define nTX0RTS_PIN   	<some-pin-number>	/*!< Connect with GPIO_OUTPUT pin of the MCU. */

#define nTX1RTS_PORT	<some-port-number>
#define nTX1RTS_PIN   	<some-pin-number>	/*!< Connect with GPIO_OUTPUT pin of the MCU. */

#define nTX2RTS_PORT	<some-port-number>
#define nTX2RTS_PIN   	<some-pin-number>	/*!< Connect with GPIO_OUTPUT pin of the MCU. */



/** 
 * @brief Macro for Oscillator frequency.
 */

#define OSC_FREQ <freq_value_in_hertz>  /*!< One need to put here the actual frequency of the crystal oscillator he/she is using. */


/**
 * @brief EXTIn line mapping to the GPIO pins.
 *
 *		- EXTI0: PA0, PB0, PC0, PD0, PE0
 *		- EXTI1: PA1, PB1, PC1, PD1, PE1
 *		- EXTI2: PA2, PB2, PC2, PD2, PE2
 * 		- EXTI3: PA3, PB3, PC3, PD3, PE3
 *		- EXTI4: PA4, PB4, PC4, PD4, PE4
 *		- EXTI5: PA5, PB5, PC5, PD5, PE5
 *		- EXTI6: PA6, PB6, PC6, PD6, PE6
 *		- EXTI7: PA7, PB7, PC7, PD7, PE7
 *		- EXTI8: PA8, PB8, PC8, PD8, PE8
 *		- EXTI9: PA9, PB9, PC9, PD9, PE9
 *		- EXTI10: PA10, PB10, PC10, PD10, PE10
 *		- EXTI11: PA11, PB11, PC11, PD11, PE11
 *		- EXTI12: PA12, PB12, PC12, PD12, PE12
 *		- EXTI13: PA13, PB13, PC13, PD13, PE13
 * 		- EXTI14: PA14, PB14, PC14, PD14, PE14
 *		- EXTI15: PA15, PB15, PC15, PD15, PE15
 *
 * 	so if one of the EXTIn line is used by a device, same EXTIn line can't be used by other device.
 */

#endif /* PIN_CONN_H */
