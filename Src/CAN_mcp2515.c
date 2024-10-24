#include "CAN_mcp2515.h"

/**
 * File: CAN_mcp2515.c
 * Description: This header file contains the definition of MCP2515 IO API.
 * Author: Piyush Choudhary
 * Date: October 3, 2024
 * Version: 1.0
 */




/**
 * @brief Resets the MCP2515 CAN controller by pulling the ~RESET pin of the MCP2515 low.
 * @param void
 * @retval void
 */
void CAN_HardReset(void){
	HAL_GPIO_WritePin(nRESET_PORT,nRESET_PIN,GPIO_PIN_RESET); /*!< driving the ~RESET pin of MCP2515 for some time to reset the CAN controller. */
	HAL_Delay(20);
	HAL_GPIO_WritePin(nRESET_PORT,nRESET_PIN,GPIO_PIN_SET);
	
}

/**
 * @brief Resets the MCP2515 CAN controller by sending RESET command over SPI.
 * @param void
 * @retval void
 */
void CAN_SoftReset(void){
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET); 	/*!< lowering the ~CS pin to select the controller.  */
	HAL_SPI_Transmit(&hspi1,RESET,1,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);  	/*!< rising the ~CS pin to de-select the controller. */
}

/**
 * @brief Writes to a specific MCP2515 register.
 * @param uint8_t* reg_addr passes the address of the specific register.
 * @param uint8_t byte passes the 8 bit value to be stored in specific register.
 * @retval void
 */
void CAN_WriteRegister(uint8_t* reg_addr, uint8_t* byte){
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET); 	/*!< lowering the ~CS pin to select the controller. 	*/
	const uint8_t cmd=WRITE;
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);		/*!< transmitting the WRITE command 			*/
	HAL_SPI_Transmit(&hspi1,reg_addr,1,HAL_MAX_DELAY);	/*!< transmitting the register address 			*/
	HAL_SPI_Transmit(&hspi1,byte,1,HAL_MAX_DELAY);		/*!< transmitting the data 				*/
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET); 	/*!< rising the ~CS pin to de-select the controller. 	*/
}

/**
 * @brief Reads 8 bit value from a specific register.
 * @param uint8_t* reg_addr passes the address of the specific register.
 * @param uint8_t* byte passes the address of the byte where the read value will be copied.
 * @retval void
 */
void CAN_ReadRegister(uint8_t* reg_addr, uint8_t* byte){
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET); 	/*!< lowering the ~CS pin to select the controller. 	*/
	const uint8_t cmd=READ;
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);		/*!< transmitting the WRITE command 											 */
 	HAL_SPI_Transmit(&hspi1,reg_addr,1,HAL_MAX_DELAY);	/*!< transmitting the register address from which one need to read. 							 */
	HAL_SPI_Receive(&hspi1,byte,1,HAL_MAX_DELAY);		/*!< receives the register value into specific memory location, actually copies data from SPI buffers to program memory. */
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);	/*!< rising the ~CS pin to de-select the controller. 	*/
}

/**
 * @brief Modifies the specific bit of a given register.
 * @param uint8_t* reg_addr passes the address of the register which has to be modified.
 * @param uint8_t* mask passes the address of the mask which will tell which bits of the correspoding register have to be modified.
 * @param uint8_t* data will tells the value of bit for which mask bits are 1.
 * @retval void
 */
void CAN_BitModify( uint8_t* reg_addr, uint8_t* mask , uint8_t* data){
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
	const uint8_t cmd=BIT_MODIFY;
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1,reg_addr,1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1,mask,1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1,data,1,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);

}

/**
 * @brief Writes the entire CAN frame to the specific TXB.
 * @param uint8_t* TXB passes the address of the specific TX buffer where we need to write the CAN frame.
 * @param can_t* can_frame passes the address of the frame (created by user program.)
 * @retval uint8_t tells whether the write operation is successful or not, on success returns 0 and on failure returns ETXBFULL or EINVALARG.
 */
uint8_t CAN_WriteFrame(uint8_t* TXB,can_t* can_frame ){
	/* First check whether the specified TXB is empty or not */
	uint8_t cmd=0;
	uint8_t register=0;
	CAN_ReadRegister(TXB,&register);
	
	if(((*register) & 0x08)==1){
	/* Transmission in progress. */
		return ETXBFULL;
	}
	
	register&=(0x02); /* keeping TXB priority as it is. */

	/* Since TXREQ bit is clear, we can write new frame to that TXB, now one need to clear TXBnCTRL and CANINTF register's bits.  */
	
	CAN_WriteRegister(TXB,&register);

	register=0x00;
	
	CAN_ReadRegister(_CANINTF,&register);
	
	if(TXB==TX0){
		register&=(0xFB); /* ANDing to create a value to be written to _CANINTF register to clear the flag bit for previously transmitted CAN frame. */
		cmd=LOAD_TX0_IR;
	}else if(TXB==TX1){
		register&=(0xF7);
		cmd=LOAD_TX1_IR;
	}else if(TXB==TX2){
		register&=(0xEF);
		cmd=LOAD_TX2_IR;
	}else{
		return EINVALARG;
	}

	CAN_WriteRegister(_CANINTF,&register);

	/* We can now write next frame, it's programmer's responsibility to load the each details of CAN frame before calling CAN_Transmit(), if CAN_Transmit is not called, the corresponding
	   TXBn won't be considered as full. */
	
	/* We've pointer to the can_t structure, thus we can directly map the structure to the specific TXB. */
	/* We'll write the dlc and data attribute of the structure as it is, but will first reformat the CAN ID field and then write it.*/

	uint32_t canid_usr_format=can_frame->CAN_ID;

	/* Let's determine whether the CAN ID is standard or Extended ( SID or EID ) */

	/* Checking whether the 1st,2nd byte of user passed CAN array are 0 and upper 5 bits of 3rd byte are all 0 or not, if yes, its a standard frame. */

	if(((((uint8_t*)&(can_frame->CAN_ID))[0]==0x00) & (((uint8_t*)&(can_frame->CAN_ID))[1]==0x00) & (((((uint8_t*)&(can_frame->CAN_ID))[2])&(0x07))==0x07) ){
		
		/* Reformating the can_frame->CAN_ID attribute to reflect the CAN_ID in the format expected by the MCP2515 to write correctly into its TX buffers */
		((uint8_t*)&(can_frame->CAN_ID))[0]=((uint8_t*)&(can_frame->CAN_ID))[2];
		((uint8_t*)&(can_frame->CAN_ID))[1]=((uint8_t*)&(can_frame->CAN_ID))[3];
	
		((uint8_t*)&(can_frame->CAN_ID))[0]=(((uint8_t*)&(can_frame->CAN_ID))[0]<<5)&(0xE0);
		((uint8_t*)&(can_frame->CAN_ID))[0]|=(((uint8_t*)&(can_frame->CAN_ID))[1]>>3)&(0x1F);
	
		((uint8_t*)&(can_frame->CAN_ID))[2]=0x00;
		((uint8_t*)&(can_frame->CAN_ID))[3]=0x00;
		
		/* Adding EXIDE bit. */
		((uint8_t*)&(can_frame->CAN_ID))[1]&=(0xF7);
		
		/* CAN Frame is ready for being transmitted to MCP2515. */
	
	}else{
		/* Extended CAN frame. */
		/* last 2 bytes of CAN_ID will remain same, now need to only adjust initial 2 bytes. */
		((uint8_t*)&(can_frame->CAN_ID))[0]=(((uint8_t*)&(can_frame->CAN_ID))[0]<<3)&(0xF8);
		((uint8_t*)&(can_frame->CAN_ID))[0]|=(((uint8_t*)&(can_frame->CAN_ID))[1]>>5)&(0x07);
		((uint8_t*)&(can_frame->CAN_ID))[1]= ((((uint8_t*)&(can_frame->CAN_ID))[1])&(0x03)) | (((uint8_t*)&(can_frame->CAN_ID))[1]<<3)&(0xE0);
			
		/* Adding EXIDE bit. */
		((uint8_t*)&(can_frame->CAN_ID))[1]|=(0x08);

		/* CAN Frame is ready for being transmitted to MCP2515. */
	}

	/* Ensuring all register for the specific TXB are all initialized to 0x00.  */

	can_t temp={
		.CAN_ID=0,
		.dlc=0,
		.data={0}
	};

	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)(&temp),13,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);
	

	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)(&can_frame),5+(can_frame->dlc),HAL_MAX_DELAY);
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);

	

	return 0;
}

/**
 * @brief Reads the entire CAN frame from the CAN controller into a specified program buffer.
 * @param uint8_t* passes the address of the specific RX buffer from where we need to copy the CAN frame.
 * @param can_t* can_frame passes the address of the CAN frame structure in the program memory where the CAN frame will be copied.
 * @retval uint8_t tell whether the read operation is successful or not, on success returns 0 for standard frame and 1 for extended frame, and on failure returns ERXBEMPTY.
 */
uint8_t CAN_ReadFrame(uint8_t* RXB,can_t* can_frame){

	/* first need to check whether the specific RX buffer has some frame or not. */
	uint8_t cmd=0;
	uint8_t register=0;
	CAN_ReadRegister(_CANINTF,&register);
	
	
	if(RXB==RX0){
		if(register&(0x01)==0){
		/* No Frame is received into the specified RX buffer. */
			return ERXBEMPTY;
		}
		cmd=READ_RX0_IR;
		/* Clearing RXB0CTRL register. */
		register=0;
		CAN_ReadRegister(RXB,&register);
		register&=(0xF6); /* clearing RTR bit as well */
		CAN_WriteRegister(RXB,&register);


	}else if(RXB==RX1){
		if(register&(0x02)==0){
			return ERXBEMPTY;
		}
		cmd=READ_RX1_IR;
		/* Clearing RXB1CTRL register */
		register=0;
		CAN_ReadRegister(RXB,&register);
		register&=(0xF0); /* clearing RTR bit as well. */
		CAN_WriteRegister(RXB,&register);
	}else{
		return EINVALARG;
	}

	

	/* Let's get the DLC field for the specific RX buffer in order to know how many bytes of data needed to be copied to program buffer. */
	uint8_t dlc=0;
	CAN_ReadRegister(RXB+5,&dlc);

	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY)
	HAL_SPI_Receive(&hspi1,(uint8_t*)(&can_frame),dlc,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);


	/* Getting the IDE bit of the SIDL register to know whether the frame is extended or standard. */

	register=0;
	CAN_ReadRegister(RXB0SIDL,&register);
	
	if((register&(0x08))==0){
		/* Frame is standard. */
		/* RXBnEID0 and RXBnEID8 registers will be empty.*/	
			
		((uint8_t*)&(can_frame->CAN_ID))[3]=(((uint8_t*)&(can_frame->CAN_ID))[1]>>5)&(0x07);
		((uint8_t*)&(can_frame->CAN_ID))[3]|=(((uint8_t*)&(can_frame->CAN_ID))[0]<<3)&(0xF8);
		((uint8_t*)&(can_frame->CAN_ID))[2]=(((uint8_t*)&(can_frame->CAN_ID))[0]>>5)&(0x07);

		((uint8_t*)&(can_frame->CAN_ID))[0]=0;
		((uint8_t*)&(can_frame->CAN_ID))[1]=0;

		return 0;
		

	}else{
		/* Frame is extended. */
		/* RXBnEID0 and RXBnEID8 will contain 16 bits of extended identifier (EID) */
		
		((uint8_t*)&(can_frame->CAN_ID))[1]&=(0xE3);
		((uint8_t*)&(can_frame->CAN_ID))[1]= ( ((((uint8_t*)&(can_frame->CAN_ID))[1])&(0x03)) | ((((uint8_t*)&(can_frame->CAN_ID))[1]>>3)&(0x1F)) | ((((uint8_t*)&(can_frame->CAN_ID))[0]<<5)&(0xE0)) );
		((uint8_t*)&(can_frame->CAN_ID))[0]= (((uint8_t*)&(can_frame->CAN_ID))[0]>>2)&(0x07);
		
		return 1;
	}
	
}

/**
 * @brief Switches the operational mode of the MCP2515 CAN controller.
 * @param mode_switch* switch_info passes the address of the mode_switch structure which contains all details about mode change.
 * @retval uint8_t tells whether the mode has been changed or not, returns 0 on success and 1 on failure (failure may happen due to the MCP2515 being already in request mode).
 */
uint8_t CAN_SwitchMode(mode_switch* switch_info){

	uint8_t register=0;
	uint8_t mask=0x07;
	uint8_t data=switch_info->clkout;


	/* Checking whether the device is already in required mode or not */
	
	CAN_ReadRegister(_CANCTRL,&register);
	if((register&(0xE0))==(((switch_info->Switch2mode)<<5)&(0xE0))){
		/* controller already in requested mode. */
		return 1;
	}

	/* Modifying _CANCTRL register. */

	/* Clearing ABAT and OSM bits, writing CLKOUT bits. */

	CAN_BitModify(_CANCTRL,&mask,&data);

	
	/* Disabling all interrupts and clearing interrupt flags */
	register=0;
	CAN_WriteRegister(_CANINTE,&register);
	CAN_WriteRegister(_CANINTF,&register);

	/* Clearing previous frames data from TXnCTRL and RXnCTRL registers. */
	/* In default mode switch mode, all TXB priorities are set to lowest but still equal to each other. */
	CAN_WriteRegister(TX0,&register);
	CAN_WriteRegister(TX1,&register);
	CAN_WriteRegister(TX2,&register);

	CAN_WriteRegister(RX1,&register);
	CAN_WriteRegister(RX0,&register);

	/* Configuring TXnRTS and RXnBF pins of MCP2515. */
	CAN_WriteRegister(_TXRTSCTRL,&(switch_info->txnrts));
	CAN_WriteRegister(_BFPCTRL,&(switch_info->rxnbf));

	/* Writing the mode switch bits. */
	mask=0xE0;
	data=(switch_info->Switch2Mode)<<5)&(0xE0);
	CAN_BitModify(_CANCTRL,&mask,&data);
	return 0;

}

/**
 * @brief Clears all the previously enabled interrupt and only enables the specific interrupt which is passed to it as its argument.
 * @param uint8_t interrupt passes the interrupt value which is set into CANINTE register of MCP2515.
 * @retval void.
 */
void CAN_SetInterrupt(uint8_t interrupt){
	uint8_t intr=interrupt;
	CAN_WriteRegister(_CANINTE,&intr);
}

/**
 * @brief Reads the CANINTE register and tells about the interrupts which are enable at that time.
 * @param uint8_t* interrupt passes the address of a byte where the CANINTE register will be copied for examination.
 * @retval void 
 */
void CAN_GetInterrupt(uint8_t* interrupt){
	CAN_ReadRegister(_CANINTE,interrupt);
}

/**
 * @brief Enables specific interrupt, won't touch the previously enabled interrupts.
 * @param uint8_t interrupt passes the specific interrupt macro from the INTERRUPT_MACROS.
 * @retval void
 */
void CAN_EnableInterrupt(uint8_t interrupt){
	/* Enabling means the specific bit must be set to 1, thus both mask and data can be taken same of enabling specific interrupt. */
	uint8_t mask_data=interrupt;
	CAN_BitModify(_CANINTE,&mask_data,&mask_data);	
}

/**
 * @brief Disables specific interrupt, won't touch the previously disabled interrupts.
 * @param uint8_t interrupt
 * @retval void
 */
void CAN_DisableInterrupt(uint8_t interrupt){
	/* disabling means setting the specific interrupt bit to 0 thus data will be 0x00. */
	uint8_t mask=interrupt;
	uint8_t data=0x00;
	CAN_BitModify(_CANINTE,&mask,&data);
}

/**
 * @brief Sets specific baud rate for the subsequent communication over the CAN bus.
 * @param uint8_t baudrate passes the identifier for specific baudrate.
 * @retval void
 */
void CAN_SetBaudRate(uint32_t baudrate){




}

/**
 * @brief Gets the baudrate at which the CAN system is communicating.
 * @param uint32_t* passes address of the variable where the baudrate will be copied.
 * @retval void
 */
void CAN_GetBaudRate(uint32_t* baudrate){
	uint8_t register=0;
	CAN_ReadRegister(_CNF1,&register);
	/* Getting Baud Rate Prescalars (BRPs) and calculating baudrate. */

}

/**
 * @brief Trigger request to transmit a CAN frame from a specific TXB over SPI.
 * @param uint8_t* TXB passes memory address of the TXn buffer corresponding to which RTS need to be triggered.
 * @retval uint8_t tells whether the TXBn contains CAN frame undergoing transmissionn or the specified TXBn is empty.
 * 		- 0 if TXBn is not under any ongoing transmission.
 *		- ERXBFULL if TXBn is undergoing a transmission, thus can't trigger RTS for that specific TXB.
 *		- EINVALARG if invalid buffer address is passed.
 */
uint8_t CAN_TriggerRTS_SPI(uint8_t* TXB){

	if(((TXB!=TX0)&(TXB!=TX1)&(TXB!=TX2))==1){
		return EINVALARG;
	}
	/* First check whether the specified TXB is already in transmission or not. */
	uint8_t register;
	CAN_ReadRegister(TXB,&register);
	if((register&(0x08))==1){
		/* ongoing transmission, TXREQ bit already set in TXBnCTRL register. */
		return ERXBFULL;
	}
	register=0x08;
	CAN_BitModify(TXB,&register,&register);
	
	return 0;
	
}

/**
 * @brief Trigger request to transmit a CAN frame from a specific TXB via TXnRTS pin, falling edge when detected on TXnRTS pins is responsible for triggering RTS for specific TXB.
 * @param uint8_t* TXB passes memory address of the TXn buffer corresponding to which RTS need to be triggered.
 * @retval uint8_t tells whether the TXBn contains CAN frame undergoing transmission or the specified TXBn is empty.
 * 		- 0 if TXBn is not under any ongoing transmission.
 *		- ERXBFULL if TXBn is undergoing a transmission, thus can't trigger RTS for that specific TXB.
 *		- EINVALARG if the specified buffer address is invalid.
 */
uint8_t CAN_TriggerRTS_PIN(uint8_t* TXB){
	/* First check whether the specified TXB is already in transmission or not. */
	uint8_t register;
	CAN_ReadRegister(TXB,&register);
	if((register&(0x08))==1){
		/* ongoing transmission, TXREQ bit already set in TXBnCTRL register. */
		return ERXBFULL;
	}
	
	if(TXB==TX0){
		HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(nTX0RTS_PORT,nTX0RTS_PIN,GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(nTX0RTS_PORT,nTX0RTS_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);
	}else if(TXB==TX1){
		HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(nTX1RTS_PORT,nTX0RTS_PIN,GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(nTX1RTS_PORT,nTX0RTS_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);
		
	}else if(TXB==TX2){
		HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(nTX2RTS_PORT,nTX0RTS_PIN,GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(nTX2RTS_PORT,nTX0RTS_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);

	}else{
		return EINVALARG;
	}

}

/**
 * @brief Immediately aborts the transmission for the specific TX buffer by clearing the TXREQ bit of TXBnCTRL register.
 * @param uint8_t* TXB will pass the address of the specific TXB whose transmission is supposed to be aborted.
 * @retval void
 */
void CAN_AbortTX(uint8_t* TXB){
	uint8_t mask=0x08;
	uint8_t data=0x00;
	CAN_BitModify(TXB,&mask,&data);
	
}

/**
 * @brief Immediately aborts the transmission of all TXBs by writing to ABAT bit of CANCTRL register, generally used during mode switch.
 * @param void
 * @retval void
 */
void CAN_AbortAllTX(void){
	uint8_t mask_data=0x10;
	CAN_BitModify(_CANCTRL,&mask_data,&mask_data);
}

/**
 * @brief Clears the ABAT bit of the CANCTRL in order to continue the transmission.
 * @param void
 * @retval void
 */
void CAN_UnAbortAllTX(void){
	uint8_t mask=0x10;
	uint8_t data=0x00;
	CAN_BitModify(_CANCTRL,&mask,&data);
}

/**
 * @brief Gets the information whether CLKOUT pin is configured or not by reading the _CANCTRL register.
 * @param void
 * @retval uint8_t tells whether the CLKOUT pin is enabled or not.
 * 	- 1 means enabled.
 *	- 0 means disabled.
 */
uint8_t CAN_GetClkOut(void){
	uint8_t register=0;
	CAN_ReadRegister(_CANCTRL,&register);
	if((register&(0x04))==1){
		/* CLKOUT enabled. */
		return 1;
	}else{
		/* CLKOUT disabled. */
		return 0;
	}
}


/**
 * @brief Enable or disable the CLKOUT pin of MCP2515.
 * @param uint8_t clkout_mode passes the CLKOUT mode i.e. whether to disable or enable the CLKOUT pin.
 *	- 0x00 : means disable the CLKOUT mode.
 *	- 0x04 : means enable the CLKOUT mode.
 * @retval void
 */
void CAN_SetClkOut(uint8_t clkout_mode){
	uint8_t mask=0x04;
	uint8_t data=clkout_mode;
	CAN_BitModify(_CANCTRL,&mask,&data);
}

/**
 * @brief Sets the clock frequency for external devices on pin 3 of MCP2515.
 * @param uint8_t prescalar passes the scalar value corresponding to which the clock frequency of CLKOUT will change.
 * 	- 0x01 : CLKOUT_freq = OSC_freq
 *	- 0x01 : CLKOUT_freq = OSC_freq/2
 * 	- 0x02 : CLKOUT_freq = OSC_freq/4
 * 	- 0x03 : CLKOUT_freq = OSC_freq/8
 * @retval void
 */
void CAN_SetClkOutFreq(uint8_t prescalar){
	uint8_t mask=0x03;
	uint8_t data=prescalar;
	CAN_BitModify(_CANCTRL,&mask,&data);
}

/**
 * @brief Transmits specified frame if TXB is not empty.
 * @param uint8_t* TXB passes the address of the TXB whose frame is attempted to be transmitted.
 * @retval uint8_t tells whether the transmission is successful or failed, returns 0 on success, returns ETXBFULL on failure.
 */
uint8_t CAN_Transmit(uint8_t* TXB){
	/* First we'll check whether the TXREQ bit of TXBnCTRL register of specific TX buffer is already high or not, if high, TXB is already undergoing transmission. */
	uint8_t register=0;
	CAN_ReadRegister(TXB,&register);
	if((register&(0x08))==(0x08)){
		/* TXREQ already set, TXBn undergoing transmission already. */
		return ETXBFULL;
	}
	register=0x08;
	CAN_BitModify(TXB,&register,&register);
	return 0;
}

/**
 * @brief Marks the specified RX buffer as read i.e. new frame cane be received into that specific RX buffer.
 * @param uint8_t* passes the address of the RX buffer which has to be marked as read.
 * @retval uint8_t tells whether the specified RXB is successfully marked as read or not, on success return 0 and on failure returns ERXBEMPTY, EINVALARG if the address passed is wrong.
 */
uint8_t CAN_MarkRead(uint8_t* RXB){

	/* First checking whether specified RXB is empty or have some CAN frame. */
	uint8_t register=0;
	
	uint8_t mask=0x09;
	
	CAN_ReadRegister(_CANINTF,&register);
	
	
	
	if(RXB==RX0){
		if((register&(0x01))!=(0x01)){
		/* RXB0 is empty, thus can't mark the RXB0 as read. */
			return ERXBEMPTY;
		}
		register=0;
		CAN_BitModify(RXB,&mask,&register); /* clearing RXRTR and FILHIT0 bits of RXBnCTRL register. */
		mask=0x01;
				
	}else if(RX=RX1){
		if((register1&(0x02))!=(0x02)){
		/* RXB1 is empty, thus can't mark the RXB1 as read. */
			return ERXBEMPTY;
		}
		register=0;
		CAN_BitModify(RXB,&mask,&register); /* clearing RXRTR and FILHIT0 bits of RXBnCTRL register. */
		mask=0x02;

	}else{
		return EINVALARG;
	}
	
	CAN_BitModify(_CANINTF,&mask,&register); /* clearing interrupt flags from CANINTF register. */
	return 0;
}

/**
 * @brief Enables One shot mode for transmission.
 * @param void
 * @retval void
 */
void CAN_EnableOSM(void){
	uint8_t mask_data=0x08;
	CAN_BitModify(_CANCTRL,&mask_data,&mask_data);
}

/**
 * @brief Disables One shot mode for transmission.
 * @param void
 * @retval void
 */
void CAN_DisableOSM(void){
	uint8_t mask=0x08;
	uint8_t data=0x00;
	CAN_BitModify(_CANCTRL,&mak,&data);
}

/**
 * @brief Changes the priority of a specific TX to a specific value (TXB with most priority will be given the CAN bus first for transmission).
 * @param uint8_t* TXB passes the address of the TXB whose priority has to be changed.
 * @param uint8_t Priority passes the priority value for the TXB.
 * @retval void
void CAN_ChangeTXPriority(uint8_t* TXB, uint8_t Priority){
	uint8_t mask=0x03;
	uint8_t data=Priority;
	CAN_BitModify(TXB,&mask,&data);
}

/**
 * @brief Sets the filtering mode, i.e. whether to turn on or off filtering for specific RXB.
 * @param uint8_t* RXB passes the address of the RXB for which filtering has to be enabled or disabled.
 * @param uint8_t RXB_mode passes the mode value.
 * 	- 0x00 : disables filtering i.e. all frames with both SID and EID will be received in the specific RXB.
 *	- 0x03 : Enables filtering i.e. frame's whose CAN ID satisfies specific requirement will be received into that specific buffer.
 * @retval void 
 */
void CAN_SetRXBMode(uint8_t* RXB, uint8_t RXB_mode){
	uint8_t mask=0x60;
	uint8_t data=(RXB_mode<<5)&(0x60);
	CAN_BitModify(RXB,&mask,&data);
}

/**
 * @brief Configure TXnRTS pins of the specific TXB, must be used in configuration mode only!
 * @param uint8_t txnrts will tell whether to enable or disable a specific TXnRTS pin of MCP2515 in current mode.
 * 	  Macros can be used to configure the TXnRTS pins of MCP2515.
 * @retval void
 */
void CAN_ConfigTXnRTS(uint8_t txnrts){
	uint8_t mask=0x07;
	uint8_t data=txnrts;
	CAN_BitModify(_TXRTSCTRL,&mask,&data);
}

/**
 * @brief Configure RXnBF pins of the specific RXB.
 * @param uint8_t rxnbf will tell whether to enable or disable a specific RXnBF pin of MCP2515 in new mode or not irrespective of what their states are in current mode.
 *	  Macros can be used to configure the RXnBF pins of MCP2515.
 * @retval void 
 */
void CAN_ConfigRXnBF(uint8_t rxnbf){
	uint8_t mask=0x0F;
	uint8_t data rxnbf;
	CAN_BitModify(_BFPCTRL,&mask,&data);
}


/**
 * @brief Tells whether the current TX/RX frame is a normal frame or RTR frame.
 * 	  Its programmer's responsibility to use this routine carefully, it will only tell you whether the RTR bit is set for specific TXbn or RXBn, it won't tell about whether the buffer is full or empty, programmer must ensure 
 * 	  at its own.
 * @param uint8_t* mXBn passes the address of TXBs/RXBs for which RTR bit has to be checked.
 * @retval uint8_t tells whether the specific frame is set/obtained as RTR frame or not.
 *		- 0 if RTR is not set.
 *		- 1 if RTR in set.
 *		- EINVALARG if invalid mXBn value. 
 */
uint8_t CAN_GetRTR(uint8_t* mXBn){
	
	uint8_t register=0;
	if(mXBn==TXB0 || mXBn==TXB1 || mXBn==TXB2){
	/* Getting RTR bit for specified TX frame to be transmitted. */	
	
		CAN_ReadRegister(mXBn+5,&register);  /* reading DLC register. */
		if((register&(0x40))==(0x40)){
		/* RTR bit is set. */
			return 1;
		}else{
		/* RTR bit in not set. */
			return 0;
		}
		
	}else if(mXBn==RX0 || mXBn==RX1){
	/* Getting RTR bit for received RX frame. */
		CAN_ReadRegister(mXBn,&register);
		if((register&(0x08))==(0x08)){
		/* RTR bit is set. */
			return 1;
		}else{
		/* RTR bit is not set. */
			return 0;
		}
		
	}else{
		return ENIVALARG;
	}
}

/**
 * @brief Sets the specific TXBn frame to be normal or RTR frame, it only sets the RTR bit for specific TXBn.
 *	  This must be used after CAN_WriteFrame(), CAN_WriteFrame() by default don't include setting up of RTR bit, thus must be explicitly done using this function. 
 * @param uint8_t* TXBn passes the address of the TXB whose frame type property has to be changed.
 * @param uint8_t rtr_val tell whether to set the RTR bit to be 0 or 1, setting 0 means normal frame , setting it 1 means RTR frame.
 * @retval void
 */
void CAN_SetRTR(uint8_t* TXBn, uint8_t rtr_val){
	uint8_t register=rtr_val;
	CAN_ReadRegister(TXBn+5,&register);   /* Reading DLC register for RTR bit. */
	if(rtr_val==0){
		register&=(0xBF);
	}else{
		register|=(0x40);
	}
	CAN_WriteRegister(TXBn+5,&register);
}

/**
 * @brief Tells whether the specific TXB/RXB is normal or extended frame.
 * @param uint8_t* mXBn passes the address of RXB/TXB whose frame type has to be checked.
 * @retval uint8_t tells whether the frame type of TXB/RXB is normal or extended.
 */
uint8_t CAN_GetFrameType(uint8_t* mXBn){
	uint8_t* register=0;
	CAN_ReadRegister(mXBn+2,&register);
	if((register&(0x08))==(0x08)){
	/* Frametype is extended. */
		return 1;
	}else{
	/* Frametype is standard. */
		return 0;
	}
}

/**
 * @brief Sets the frame type information i.e. sets the EXIDE bit of TXBnSIDL.
 * @param uint8_t* TXBn passes the address of the TX buffer for which the Frame type of specific TX must be set, CAN_WriteFrame() by default writes to this bit by identifying the frame type depending on the CAN ID structure.
 * @param uint8_t exide_val pass the information whether to set exide bit to 0 or 1, if exide_val=0, exide bit is set to 0 i.e. normal frame type, and if exide_val=1, frame type is extended.
 * @retval void
 */
void CAN_SetFrameType(uint8_t* TXBn, uint8_t exide_val){
	uint8_t register=0;
	CAN_ReadRegister(TXBn+2,&register);   /* Reading the TXBnSIDL register  */
	if(exide_val==0){
	/* standard frame type. */
		register&=(0xF7);
	}else{
	/* extended frame type. */
		register|=(0x08);
	}
	CAN_WriteRegister(TXBn+2,&register);
}

/**
 * @brief enables/disables roll over to RX1 in case RX0 is full or its read but not marked as read.
 * @param uint8_t roll_ovr passes the information whether to enable or disable the frame roll over to RX1 in case RX0 is full or not marked as read.
 *	- 0x00 for disabling roll over.
 *	- 0x04 for enabling roll over.
 * @retval void
 */
void CAN_RollOver2RX1(uint8_t roll_ovr){
	uint8_t mask=0x04;
	uint8_t data=roll_ovr;
	CAN_BitModify(RX0,&mask,&data);
}



















