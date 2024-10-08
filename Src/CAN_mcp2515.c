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
	
	register=0x02; /* Higher intermediate TX priority. */

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
	 * TXBn won't be considered as full. 
	*/
	
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
		
		/* CAN Frame is ready for being transmitted to MCP2515. */
	
	}else{
		/* Extended CAN frame. */
		/* last 2 bytes of CAN_ID will remain same, now need to only adjust initial 2 bytes. */
		((uint8_t*)&(can_frame->CAN_ID))[0]=(((uint8_t*)&(can_frame->CAN_ID))[0]<<3)&(0xF8);
		((uint8_t*)&(can_frame->CAN_ID))[0]|=(((uint8_t*)&(can_frame->CAN_ID))[1]>>5)&(0x07);
		((uint8_t*)&(can_frame->CAN_ID))[1]= ((((uint8_t*)&(can_frame->CAN_ID))[1])&(0x03)) | (((uint8_t*)&(can_frame->CAN_ID))[1]<<3)&(0xE0);

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

	/* Now writing to the EXIDE bit of the TXBnSIDL register. */

	register=0;
	CAN_ReadRegister(TXB+2,&register);
	register|=(0x08);	
	CAN_WriteRegister(TXB+2,&register);

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
		register&=(0xFE);
		CAN_WriteRegister(RXB,&register);


	}else if(RXB==RX1){
		if(register&(0x02)==0){
			return ERXBEMPTY;
		}
		cmd=READ_RX1_IR;
		/* Clearing RXB1CTRL register */
		register=0;
		CAN_ReadRegister(RXB,&register);
		register&=(0xF8);
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
 * @param uint8_t mode passes the identifier of the mode to which the CAN controller must switch to.
 * @paran uint8_t CLKOUT passes the info about altering CLKOUT pin state on mode change.
 * 	- 0 means lower the CLKOUT bit in CANCTRL register for disabling CLKOUT pin in new mode. 
 *	- 1 means don't lower the CLKOUT bit in CANCTRL register to ensure continuous functioning of CLKOUT pin.
 * @retval uint8_t tells whether the mode has been changed or not, returns 0 on success and 1 on failure (failure may happen due to the MCP2515 being already in request mode).
 */
uint8_t CAN_SwitchMode(uint8_t mode, uint8_t CLKOUT){

	uint8_t register=0;

	
	CAN_ReadRegister(_CANCTRL,&register);
	if((register&(0xE0))==((mode<<5)&(0xE0))){
		/* controller already in requested mode. */
		return 1;
	}

	/* Preparing for mode switch, aborting all transmitting frames, clearing RXBs , disabling interrupts.*/
	if(CLKOUT==0){
		register&=(0xE3); /* disabling OSM and aborting all transmissions, CLKOUT is disabled */
	}else if(CLKOUT==1){
		register&=(0xE7); /* disabling OSM and aborting all transmissions, CLKOUT not disabled */
	}else{	
		return EINVALARG;
	}
	
	CAN_WriteRegister(_CANCTRL,&register); /* writing transmission abort, CKKOUT and OSM changes to CAN controller  */

	register=0x00;
	CAN_WriteRegister(_CANINTE,&register); /* disabling all interrupts. */
	CAN_WriteRegister(_CANINTF,&register); /* Clearing previous interrupt flags. */
	
	CAN_ReadRegister(_CANCTRL,&register);
	register&=(0x1F);
	register|=((mode<<5)&(E0));
	CAN_WriteRegister(_CANCTRL,&register);

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
	uint8_t intr=0;
	CAN_GetInterrupt(&intr);
	intr|=interrupt;	
}

/**
 * @brief Disables specific interrupt, won't touch the previously disabled interrupts.
 * @param uint8_t interrupt
 * @retval void
 */
void CAN_DisableInterrupt(uint8_t interrupt){
	uint8_t intr=0;
	CAN_GetInterrupt(&intr);
	intr|=(~interrupt);

}

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
	register|=(0x08);
	CAN_WriteRegister(TXB,&register);
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
 * @brief Immediately aborts the transmission for the specific TX buffer.
 * @param uint8_t* TXB will pass the address of the specific TXB whose transmission is supposed to be aborted.
 * @retval void
 */
void CAN_AbortTX(uint8_t* TXB){
	uint8_t register=0;
	CAN_ReadRegister(TXB,&register);
	register|=(0x40);
	CAN_WriteRegister(TXB,&register);
	
}

/**
 * @brief Immediately aborts the transmission of all TXBs, generally used during mode switch.
 * @param void
 * @retval void
 */
void CAN_AbortAllTX(void){
	uint8_t register=0;
	CAN_ReadRegister(_CANCTRL,&register);
	register|=(0x10);
	CAN_WriteRegister(_CANCTRL,&register);
}

/**
 * @brief Gets the information whether CLKOUT pin is configured or not.
 * @param void
 * @retval uint8_t tells whether the CLKOUT pin is enabled or not.
 * 	- 0 means enabled.
 *	- 1 means disabled.
 */
uint8_t CAN_GetClkOut(void);

/**
 * @brief Enable or disable the CLKOUT pin of MCP2515.
 * @param uint8_t clkout_mode passes the CLKOUT mode i.e. whether to disable or enable the CLKOUT pin.
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
 * 	- 0x00 : disables filtering i.e. all frames with both SID and EID will be received in the specific RXB.
 *	- 0x03 : Enables filtering i.e. frame's whose CAN ID satisfies specific requirement will be received into that specific buffer.
 * @retval void 
 */
void CAN_SetRXBMode(uint8_t* RXB, uint8_t RXB_mode);
