#ifndef IRQH_H
#define IRQH_H

#include "CAN_ioctl_config.h"


/**
 * 		CAREFUL MODIFICATION ALLOWED	
 * 
 * @defgroup IRQH_PROTOTYPES irqh_prototypes
 * @brief Prototypes for MCU specific interrupt handlers and naming conventions.
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
 *	Just go through above values, and thus connect any interrupt accordingly, it will help you use appropriate IRQHandler and its syntax as well.
 * @{
 */
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandelr(void);
void EXTI3_IRQHandelr(void);
void EXTI4_IRQHandelr(void);
void EXTI9_5_IRQHandelr(void);
void EXTI15_10_IRQHandelr(void);

/**
 * @}
 */

