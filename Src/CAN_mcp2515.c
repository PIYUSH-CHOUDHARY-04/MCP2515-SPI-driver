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
 * @brief Writes to a specific MCP2515 Register.
 * @param uint8_t* reg_addr passes the address of the specific Register.
 * @param uint8_t byte passes the 8 bit value to be stored in specific Register.
 * @retval void
 */
void CAN_WriteRegister(uint8_t* reg_addr, uint8_t* byte){
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET); 	/*!< lowering the ~CS pin to select the controller. 	*/
	const uint8_t cmd=WRITE;
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);		/*!< transmitting the WRITE command 			*/
	HAL_SPI_Transmit(&hspi1,reg_addr,1,HAL_MAX_DELAY);	/*!< transmitting the Register address 			*/
	HAL_SPI_Transmit(&hspi1,byte,1,HAL_MAX_DELAY);		/*!< transmitting the data 				*/
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET); 	/*!< rising the ~CS pin to de-select the controller. 	*/
}

/**
 * @brief Reads 8 bit value from a specific Register.
 * @param uint8_t* reg_addr passes the address of the specific Register.
 * @param uint8_t* byte passes the address of the byte where the read value will be copied.
 * @retval void
 */
void CAN_ReadRegister(uint8_t* reg_addr, uint8_t* byte){
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET); 	/*!< lowering the ~CS pin to select the controller. 	*/
	const uint8_t cmd=READ;
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);		/*!< transmitting the WRITE command 											 */
 	HAL_SPI_Transmit(&hspi1,reg_addr,1,HAL_MAX_DELAY);	/*!< transmitting the Register address from which one need to read. 							 */
	HAL_SPI_Receive(&hspi1,byte,1,HAL_MAX_DELAY);		/*!< receives the Register value into specific memory location, actually copies data from SPI buffers to program memory. */
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);	/*!< rising the ~CS pin to de-select the controller. 	*/
}

/**
 * @brief Modifies the specific bit of a given Register.
 * @param uint8_t* reg_addr passes the address of the Register which has to be modified.
 * @param uint8_t* mask passes the address of the mask which will tell which bits of the correspoding Register have to be modified.
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
	uint8_t Register=0;
	uint8_t mask=0;
	CAN_ReadRegister(TXB,&Register);
	
	if(((*Register) & 0x08)==1){
	/* Transmission in progress. */
		return ETXBFULL;
	}
	
	Register&=(0x02); /* keeping TXB priority as it is. */

	/* Since TXREQ bit is clear, we can write new frame to that TXB, now one need to clear TXBnCTRL and CANINTF Register's bits.  */
	
	CAN_WriteRegister(TXB,&Register);

	Register=0x00;
	
	
	
	if(TXB==TX0){
		mask=(0x04); /* ANDing to create a value to be written to _CANINTF Register to clear the flag bit for previously transmitted CAN frame. */
		cmd=LOAD_TX0_IR;
	}else if(TXB==TX1){
		mask=(0x08);
		cmd=LOAD_TX1_IR;
	}else if(TXB==TX2){
		mask=(0x10);
		cmd=LOAD_TX2_IR;
	}else{
		return EINVALARG;
	}

	CAN_BitModify(_CANINTF,&mask,&Register);

    /* Ensuring all Register for the specific TXB are all initialized to 0x00.  */

	can_t temp={
		.CAN_ID=0,
		.dlc=0,
		.data={0}
	};

	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)(&temp),13,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);

    temp=*(can_frame);

	/* We can now write next frame, it's programmer's responsibility to load the each details of CAN frame before calling CAN_Transmit(), if CAN_Transmit is not called, the corresponding
	   TXBn won't be considered as full. */
	
	/* We've pointer to the can_t structure, thus we can directly map the structure to the specific TXB. */
	/* We'll write the dlc and data attribute of the structure as it is, but will first reformat the CAN ID field and then write it.*/


	/* Let's determine whether the CAN ID is standard or Extended ( SID or EID ) */
    /* Reformating the can_frame->CAN_ID attribute to reflect the CAN_ID in the format expected by the MCP2515 to write correctly into its TX buffers */

    

	/* Checking whether the 4th,3rd byte of user passed CAN array are 0 and lower 2 bits of 2nd byte are all 0 or not, if yes, its a standard frame. */
    

	if(((((uint8_t*)&(temp.CAN_ID))[2]==0x00) & (((uint8_t*)&(temp.CAN_ID))[3]==0x00) & (((((uint8_t*)&(temp.CAN_ID))[1])&(0x02))==0x00) )){
		/* Standard CAN frame. */

        ((uint8_t*)&(temp.CAN_ID))[0]=((uint8_t*)&(temp.CAN_ID))[2];
        ((uint8_t*)&(temp.CAN_ID))[1]=((uint8_t*)&(temp.CAN_ID))[3];
        
        ((uint8_t*)&(temp.CAN_ID))[0]=((((uint8_t*)&(temp.CAN_ID))[0]<<3)&(0xF8) | (((uint8_t*)&(temp.CAN_ID))[1]>>5)&(0x07));  
        ((uint8_t*)&(temp.CAN_ID))[1]=(((uint8_t*)&(temp.CAN_ID))[1]<<3)&(0xE0);

        ((uint8_t*)&(temp.CAN_ID))[2]=0x00;
        ((uint8_t*)&(temp.CAN_ID))[3]=0x00;        
    

		/* Adding EXIDE bit. */
		((uint8_t*)&(temp.CAN_ID))[1]&=(0xF7);
		
		/* CAN Frame is ready for being transmitted to MCP2515. */
	
	}else{
		/* Extended CAN frame. */
        ((uint8_t*)&(temp.CAN_ID))[0]=((((uint8_t*)&(temp.CAN_ID))[0]<<3)&(0xF8) | (((uint8_t*)&(temp.CAN_ID))[1]>>5)&(0x07));
        ((uint8_t*)&(temp.CAN_ID))[1]=((((uint8_t*)&(temp.CAN_ID))[1]<<3)&(0xE0) | ((uint8_t*)&(temp.CAN_ID))[1]&(0x03));

			
		/* Adding EXIDE bit. */
		((uint8_t*)&(temp.CAN_ID))[1]|=(0x08);

		/* CAN Frame is ready for being transmitted to MCP2515. */
	}


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
	uint8_t Register=0;
	CAN_ReadRegister(_CANINTF,&Register);
	
	
	if(RXB==RX0){
		if(Register&(0x01)==0){
		/* No Frame is received into the specified RX buffer. */
			return ERXBEMPTY;
		}
		cmd=READ_RX0_IR;
	}else if(RXB==RX1){
		if(Register&(0x02)==0){
			return ERXBEMPTY;
		}
		cmd=READ_RX1_IR;
	}else{
		return EINVALARG;
	}

	

	/* Let's get the DLC field for the specific RX buffer in order to know how many bytes of data needed to be copied to program buffer. */
	CAN_ReadRegister(RXB+5,&(can_frame->dlc));
    (can_frame->dlc)&=(0x40);

	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&cmd,1,HAL_MAX_DELAY)
	HAL_SPI_Receive(&hspi1,(uint8_t*)(&can_frame),(can_frame->dlc)+5,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(nCS_PORT,nCS_PIN,GPIO_PIN_SET);

    /* dlc field needs to be reformat to indicate only data length i.e. removing RTR bit */


	/* Getting the IDE bit of the SIDL Register to know whether the frame is extended or standard. */

	Register=0;
	CAN_ReadRegister(RXB+2,&Register); /* getting RXBnSIDL register. */
	
	if((Register&(0x08))==0){
		/* Frame is standard. */
		/* RXBnEID0 and RXBnEID8 Registers will be empty.*/	
			
		((uint8_t*)&(can_frame->CAN_ID))[3]=((uint8_t*)&(can_frame->CAN_ID))[1];
        ((uint8_t*)&(can_frame->CAN_ID))[2]=((uint8_t*)&(can_frame->CAN_ID))[0];
    
        ((uint8_t*)&(can_frame->CAN_ID))[3]=((((uint8_t*)&(can_frame->CAN_ID))[3]>>5)&(0x07) | (((uint8_t*)&(can_frame->CAN_ID))[2]<<3)&(0xF8));
        ((uint8_t*)&(can_frame->CAN_ID))[2]=(((uint8_t*)&(can_frame->CAN_ID))[2]>>5)&(0x07);

		((uint8_t*)&(can_frame->CAN_ID))[1]=0x00;
		((uint8_t*)&(can_frame->CAN_ID))[0]=0x00;
        
        (can_frame->dlc)&(0x40);
        /* Checking SRR bit of RXBnSIDL register */

        if((Register)&(0x0F)==(0x0F)){
            return STANDARD_FRAME_TYPE|REMOTE_FRAME_TYPE;
        }else{
            return STANDARD_FRAME_TYPE|DATA_FRAME_TYPE;
        }
		

	}else{
		/* Frame is extended. */
		/* RXBnEID0 and RXBnEID8 will contain 16 bits of extended identifier (EID) */
		
		((uint8_t*)&(can_frame->CAN_ID))[1]&=(0xE3);
		((uint8_t*)&(can_frame->CAN_ID))[1]= ( ((((uint8_t*)&(can_frame->CAN_ID))[1])&(0x03)) | ((((uint8_t*)&(can_frame->CAN_ID))[1]>>3)&(0x1F)) | ((((uint8_t*)&(can_frame->CAN_ID))[0]<<5)&(0xE0)) );
		((uint8_t*)&(can_frame->CAN_ID))[0]= (((uint8_t*)&(can_frame->CAN_ID))[0]>>2)&(0x07);
		
		if((can_frame->dlc))&(0x40) == (0x40)){
            (can_frame->dlc)&=(0x0F);
            return EXTENDED_FRAME_TYPE|REMOTE_FRAME_TYPE;    
        }else{
            (can_frame->dlc)&=(0x0F);
            return EXTENDED_FRAME_TYPE|DATA_FRAME_TYPE;
        }
	}
	
}

/**
 * @brief Switches the operational mode of the MCP2515 CAN controller.
 * @param mode_switch* switch_info passes the address of the mode_switch structure which contains all details about mode change.
 * @retval uint8_t tells whether the mode has been changed or not, returns 0 on success and 1 on failure (failure may happen due to the MCP2515 being already in request mode).
 */
uint8_t CAN_SwitchMode(mode_switch* switch_info){

	uint8_t Register=0;
	uint8_t mask=0x07;
	uint8_t data=switch_info->clkout;


	/* Checking whether the device is already in required mode or not */
	
	CAN_ReadRegister(_CANCTRL,&Register);
	if((Register&(0xE0))==(((switch_info->Switch2mode)<<5)&(0xE0))){
		/* controller already in requested mode. */
		return 1;
	}

	/* Modifying _CANCTRL Register. */

	/* Clearing ABAT and OSM bits, writing CLKOUT bits. */

	CAN_BitModify(_CANCTRL,&mask,&data);

	
	/* Disabling all interrupts and clearing interrupt flags */
	Register=0;
	CAN_WriteRegister(_CANINTE,&Register);
	CAN_WriteRegister(_CANINTF,&Register);

	/* Clearing previous frames data from TXnCTRL and RXnCTRL Registers. */
	/* In default mode switch mode, all TXB priorities are set to lowest but still equal to each other. */
	CAN_WriteRegister(TX0,&Register);
	CAN_WriteRegister(TX1,&Register);
	CAN_WriteRegister(TX2,&Register);
    
    Register=0x60;

	CAN_WriteRegister(RX1,&Register);
	CAN_WriteRegister(RX0,&Register);

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
 * @param uint8_t interrupt passes the interrupt value which is set into CANINTE Register of MCP2515.
 * @retval void.
 */
void CAN_SetInterrupt(uint8_t interrupt){
	uint8_t intr=interrupt;
	CAN_WriteRegister(_CANINTE,&intr);
}

/**
 * @brief Reads the CANINTE Register and tells about the interrupts which are enable at that time.
 * @param uint8_t* interrupt passes the address of a byte where the CANINTE Register will be copied for examination.
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
	uint8_t Register;
	CAN_ReadRegister(TXB,&Register);
	if((Register&(0x08))==1){
		/* ongoing transmission, TXREQ bit already set in TXBnCTRL Register. */
		return ERXBFULL;
	}
	Register=0x08;
	CAN_BitModify(TXB,&Register,&Register);
	
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
	uint8_t Register;
	CAN_ReadRegister(TXB,&Register);
	if((Register&(0x08))==1){
		/* ongoing transmission, TXREQ bit already set in TXBnCTRL Register. */
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
 * @brief Immediately aborts the transmission for the specific TX buffer by clearing the TXREQ bit of TXBnCTRL Register.
 * @param uint8_t* TXB will pass the address of the specific TXB whose transmission is supposed to be aborted.
 * @retval void
 */
void CAN_AbortTX(uint8_t* TXB){
	uint8_t mask=0x08;
	uint8_t data=0x00;
	CAN_BitModify(TXB,&mask,&data);
	
}

/**
 * @brief Immediately aborts the transmission of all TXBs by writing to ABAT bit of CANCTRL Register, generally used during mode switch.
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
 * @brief Gets the information whether CLKOUT pin is configured or not by reading the _CANCTRL Register.
 * @param void
 * @retval uint8_t tells whether the CLKOUT pin is enabled or not.
 * 	- 1 means enabled.
 *	- 0 means disabled.
 */
uint8_t CAN_GetClkOut(void){
	uint8_t Register=0;
	CAN_ReadRegister(_CANCTRL,&Register);
	if((Register&(0x04))==1){
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
	/* First we'll check whether the TXREQ bit of TXBnCTRL Register of specific TX buffer is already high or not, if high, TXB is already undergoing transmission. */
	uint8_t Register=0;
	CAN_ReadRegister(TXB,&Register);
	if((Register&(0x08))==(0x08)){
		/* TXREQ already set, TXBn undergoing transmission already. */
		return ETXBFULL;
	}
	Register=0x08;
	CAN_BitModify(TXB,&Register,&Register);
	return 0;
}

/**
 * @brief Marks the specified RX buffer as read i.e. new frame cane be received into that specific RX buffer.
 * @param uint8_t* passes the address of the RX buffer which has to be marked as read.
 * @retval uint8_t tells whether the specified RXB is successfully marked as read or not, on success return 0 and on failure returns ERXBEMPTY, EINVALARG if the address passed is wrong.
 */
uint8_t CAN_MarkRead(uint8_t* RXB){

	/* First checking whether specified RXB is empty or have some CAN frame. */
	uint8_t Register=0;
	
	uint8_t mask=0x09;
	
	CAN_ReadRegister(_CANINTF,&Register);
	
	
	
	if(RXB==RX0){
		if((Register&(0x01))!=(0x01)){
		/* RXB0 is empty, thus can't mark the RXB0 as read. */
			return ERXBEMPTY;
		}
		Register=0;
		CAN_BitModify(RXB,&mask,&Register); /* clearing RXRTR and FILHIT0 bits of RXBnCTRL Register. */
		mask=0x01;
				
	}else if(RXB=RX1){
		if((Register1&(0x02))!=(0x02)){
		/* RXB1 is empty, thus can't mark the RXB1 as read. */
			return ERXBEMPTY;
		}
		Register=0;
		CAN_BitModify(RXB,&mask,&Register); /* clearing RXRTR and FILHIT0 bits of RXBnCTRL Register. */
		mask=0x02;

	}else{
		return EINVALARG;
	}
	
	CAN_BitModify(_CANINTF,&mask,&Register); /* clearing interrupt flags from CANINTF Register. */
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
 */
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
 * @retval uint8_t tells whether the configuration of TXRTS pins is successful or not, 0 for success and 1 for failure.
 */
uint8_t CAN_ConfigTXnRTS(uint8_t txnrts){
    /* TXRTXCTRL can only be modified in configuration mode, thus checking for configuration mode first. */
    uint8_t mask=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return 1; 
    }
	mask=0x07;
	uint8_t data=txnrts;
	CAN_BitModify(_TXRTSCTRL,&mask,&data);
    return 0;
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
 * 	  at its own, must be called before freeing up a RXB
 * @param uint8_t* mXBn passes the address of TXBs/RXBs for which RTR bit has to be checked.
 * @retval uint8_t tells whether the specific frame is set/obtained as RTR frame or not.
 *		- 0 if RTR is not set.
 *		- 1 if RTR in set.
 *		- EINVALARG if invalid mXBn value. 
 */
uint8_t CAN_GetRTR(uint8_t* mXBn){
	
	uint8_t Register=0;
    uint8_t Register2=0;

	if(mXBn==TXB0 || mXBn==TXB1 || mXBn==TXB2){
	/* Getting RTR/SRR bit for specified TX frame to be transmitted. */	
	
		CAN_ReadRegister(mXBn+5,&Register);  /* reading DLC Register. */
		if((Register&(0x40))==(0x40)){
		/* RTR bit is set. */
			return 1;
		}else{
		/* RTR bit is not set. */
			return 0;
		}
		
	}else if(mXBn==RX0 || mXBn==RX1){
	/* Getting RTR bit for received RX frame. */
		CAN_ReadRegister(mXBn+2,&Register);
       		CAN_ReadRegister(mXBn+5,&Register2);
		if(((Register&(0x0F))==(0x0F)) | ((Register2&(0x40)) == (0x40))){
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
	uint8_t Register=rtr_val;
	CAN_ReadRegister(TXBn+5,&Register);   /* Reading DLC Register for RTR bit. */
	if(rtr_val==0){
		Register&=(0xBF);
	}else{
		Register|=(0x40);
	}
	CAN_WriteRegister(TXBn+5,&Register);
}

/**
 * @brief Tells whether the specific TXB/RXB is normal or extended frame.
 * @param uint8_t* mXBn passes the address of RXB/TXB whose frame type has to be checked.
 * @retval uint8_t tells whether the frame type of TXB/RXB is normal or extended.
 */
uint8_t CAN_GetFrameType(uint8_t* mXBn){
	uint8_t* Register=0;
	CAN_ReadRegister(mXBn+2,&Register);
	if((Register&(0x08))==(0x08)){
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
	uint8_t Register=0;
	CAN_ReadRegister(TXBn+2,&Register);   /* Reading the TXBnSIDL Register  */
	if(exide_val==0){
	/* standard frame type. */
		Register&=(0xF7);
	}else{
	/* extended frame type. */
		Register|=(0x08);
	}
	CAN_WriteRegister(TXBn+2,&Register);
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

/**
 * @brief Retrieves the specific filter value.
 * @param uint8_t* FILTn passes the address of the filter whose value has to be retrieved.
 * @param uint32_t* filt_val passes the address of the 4-byte where the filter value will be stored or copied to.
 * @retval uint8_t tells whether the filter contains the exide value set to 0 or 1 i.e. the filter allows extended or standard frame, can be neglected by setting exide bit to be 0 in corresponding mask.
 *          - 0 if the filter is for standard frame.
 *          - 1 if the filter is for extended frame.
 *          - EINVALMODE if the mode is not configuration mode since dealing with filters and masks needs the device to be in configuration mdoe only.
 */
uint8_t CAN_GetFilter(uint8_t* FILTn,uint32_t* filt_val){
    /* Filters and Masks Registers are read to '0' in any mode other than configuration mode, thus device must be in configuration mode read the exact value of the filters and masks. */
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    Register=0;
    for(Register;Register<4;Register++){
        CAN_ReadRegister(FILTn+Register,((uint8_t*)filt_val)+Register);
    }
    
    /* Getting the Exide bit. */
    Register=(&((uint8_t*)filt_val + 1))&(0x08);
    /* Need to rearrange the filter bits to match the standard 29 bit identifier format. */
   

    ((uint8_t*)filt_val)[1]=((((uint8_t*)filt_val)[1])&(0x03))|((((uint8_t*)filt_val)[1])>>3)&(0x1C);
    ((uint8_t*)filt_val)[1]=(((uint8_t*)filt_val)[1])|((((uint8_t*)filt_val)[1])<<5)&(0xE0);
    ((uint8_t*)filt_val)[0]=((((uint8_t*)filt_val)[0])>>3)&(0x1F);

    return Register;

}

/**
 * @brief Sets the specific filter.
 * @param uint8_t* FILTn passes the address of the filter whose value has to be set.
 * @param uint32_t* passes the address of the 4-byte filter which has to be set.
 * @param int8_t exide_val passes the value for exide bit i.e. whether the filter is intended for standard or extended frame, 0 for standard frame and 1 for extended frame, works only if the corresponding exide bit in mask is set to 1.
 * @retval void
 */
void CAN_SetFilter(uint8_t* FILTn,uint32_t* filt_val, uint8_t exide_val){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    /* Need to reformat the filter value. */
    uint32_t filt_cpy=*filt_val;
    
    ((uint8_t*)(&filt_cpy))[0]=((((uint8_t*)(&filt_cpy))[0]<<3)&(0xF8)) | ((((uint8_t*)(&filt_cpy))[1]>>5)&(0x07));
    ((uint8_t*)(&filt_cpy))[1]=((((uint8_t*)(&filt_cpy))[1])&(0x03)) | ((((uint8_t*)(&filt_cpy))[1]<<3)&(0xE0));

    /* Adding EXIDE bit filter. */
    ((uint8_t*)(&filt_cpy))[1]|=(0x08);
    for(Register;Register<4;Register++){
        CAN_WriteRegister(FILTn+Register,((uint8_t*)&filt_cpy)+Register);
    }

}

/**
 * @brief Retrieves the specific mask value.
 * @param uint8_t* MASKn passes the address of the mask whose value has to be retrieved.
 * @param uint32_t* mask_val passes the address of the 4-byte where the mask value will be stored or copied to.
 * @retval uint8_t tells whether the mask contains the exide value set to 0 or 1 i.e. 0 means frame type checing is disabled and 1 means frame type checking is enabled.
 *          - 0 : frame type checking by filters disabled.
 *          - 1 : frame type checking by filters enabled.
 *          - EINVALMODE if the mode is not configuration mode since dealing with filters and masks needs the device to be in configuration mdoe only.
 */
uint8_t CAN_GetMask(uint8_t* MASKn,uint32_t* mask_val){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }   
    Register=0;
    for(Register;Register<4;Register++){
        CAN_ReadRegister(MASKn+Register,((uint8_t*)mask_val)+Register);
    }

    /* Getting the exide bit. */
    Register=(&((uint8_t*)filt_val + 1))&(0x08);

    /* Rearranging the retrieved mask. */    
    ((uint8_t*)mask_val)[1]=((((uint8_t*)mask_val)[1])&(0x03))|((((uint8_t*)mask_val)[1])>>3)&(0x1C);
    ((uint8_t*)mask_val)[1]=(((uint8_t*)mask_val)[1])|((((uint8_t*)mask_val)[1])<<5)&(0xE0);
    ((uint8_t*)mask_val)[0]=((((uint8_t*)mask_val)[0])>>3)&(0x1F);


    return Register;
}

/**
 * @brief Sets the specific mask.
 * @param uint8_t* MASKn passes the address of the mask whose value has to be set.
 * @param uint32_t* mask_val passes the address of the 4-byte mask which has to be set.
 * @param uint8_t exide_val passes the value for exide bit, if 0 means you are disabling frame type checking and if 1 means you are enabling frame type checking. 
 * @retval void
 */
void CAN_SetMask(uint8_t* MASKn,uint32_t* mask_val, uint8_t exide_val){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    uint32_t mask_cpy=*mask_val;
        

    /* Rearranging the mask. */
    ((uint8_t*)(&mask_cpy))[0]=((((uint8_t*)(&mask_cpy))[0]<<3)&(0xF8)) | ((((uint8_t*)(&mask_cpy))[1]>>5)&(0x07));
    ((uint8_t*)(&mask_cpy))[1]=((((uint8_t*)(&mask_cpy))[1])&(0x03)) | ((((uint8_t*)(&mask_cpy))[1]<<3)&(0xE0));

    /* Adding EXIDE bit filter. */
    ((uint8_t*)(&filt_cpy))[1]|=(0x08);
    for(Register;Register<4;Register++){
        CAN_WriteRegister(MASKn+Register,((uint8_t*)&mask_cpy)+Register);
    }
} 


/*!< CAN BIT TIMING CONFIGURATION AND SYNCHRONIZATION.  */

/**
 * CAN Bit timing can be configured by modifying certain registers, these registers are CNFn registers (n=1,2,3).
 * These registers are only accessible in Configuration mode only, thus one need to make sure that the device is in configuration mode before trying accessing the timing registers.  
 *
 *
 *
 * Bit_Time = t(SyncSeg)+t(PropSeg)+t(PS1)+t(PS2),      Baudrate or Bitrate = 1/Bit_Time.
 * t(SyncSeg) is 1Tq for MCP2515.
 * Tq=(2*BRP)/F_OSC , BRP must be atleast 1 else Baudrate will diverge.
 *   • PropSeg + PS1 >= PS2
 *   • PropSeg + PS1 >= TDELAY
 *   • PS2 > SJW
 *
 *       Tq = 2.(BRP)/F_OSC = 2.(BV_brp+1)/F_OSC (BV_brp means the bit value of the BRP bits segment.)
 *       T_bit = (T_syncseg + T_propseg + T_ps1 + T_ps20)*Tq
 *       T_bit = (4 + BV_propseg + BV_ps1 + BV_ps2)*Tq
 *       baudrate = 1/(Tq*T_bit)         or          baudrate = F_OSC/(2*(1+BV_brp)*(4+BV_propseg+BV_ps1+BV_ps2))
 *       propagation segment is generally not changed because it accounts for propagation delays in CAN communication
 *
 *       The device boots up with the following values of these variables.
 *       F_OSC : OSC_FREQ (macro defined in /Inc/Pin_connection.h)
 *       BV_brp : 0  ::  BRP : 1
 *       BV_propseg : 0  ::  T_propseg : 1Tq
 *       BV_ps1 : 3  ::  T_ps1 : 4Tq
 *       BV_ps2 : 1  ::  T_ps2 : 2Tq
 *                       T_syncseg : 1Tq
 *       I.e. Baudrate is 500Kbps and sample point is at 75% bit time.
 *
 *
*/



/**
 * @brief Configures the SJW, i.e. gets and sets SJW value depending upon the passed arguments.
 * @param uint8_t set_SJW passes the SJW value that has to be set.
 *          - 0xFF : if user don't want to modify the SJW value.
 *          - 0x00 : SJW = 1Tq
 *          - 0x01 : SJW = 2Tq
 *          - 0x10 : SJW = 3Tq
 *          - 0x11 : SJW = 4Tq
 * @param uint8_t Read tells whether to read the SJW or not.
 *          - 0 : don't read SJW.
 *          - 1 : read SJW.
 * @retval uint8_t tells the value of SJW if Read is 1 else returns 0xFF, EINVALMODE if not in configuration mode.
 */
uint8_t CAN_ConfigureSJW(uint8_t set_SJW,uint8_t Read){
    uint8_t Register=0;

    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }

    if(set_SJW!=(0xFF)){
        /* Setting SJW value in CNF1 register. */
        Register=(set_SJW<<6)&(0xC0);
        uint8_t mask=0xC0;
        CAN_BitModify(_CNF1,&mask,&Register);
    }
    Register=0;
    if(Read!=0){
        CAN_ReadRegister(_CNF1,&Register);
        return ((Register>>6)&(0x03));
    }
    return 0xFF;
}

/**
 * @brief Configures the BRP, i.e. gets and sets BRP value depending upon the passed arguments.
 *          The actual BRP value that will be used to calculate the baudrate etc will be the value of BRP indicated by these 6 bits + 1 since BRP has to be atleast 1.
 * @param uint8_t set_BRP passes the BRP value that has to be set.
 *          - 0xFF : if user don't want to modify the BRP value.
 *          - 0x00 to 0x3F : prescalar values by which the baudrate will be scaled.
 * @param uint8_t Read tells whether to read the SJW or not.
 *          - 0 : don't read BRP.
 *          - 1 : read BRP.
 * @retval uint8_t tells the value of BRP if Read is 1 else returns 0xFF, EINVALMODE if not in configuration mode.
*/
uint8_t CAN_ConfigureBRP(uint8_t set_BRP,uint8_t Read){
    uint8_t Register=0;
    
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    
    if(set_BRP!=(0xFF)){
    /* Setting the BRP value in CNF1 register. */
        Register=set_BRP;
        uint8_t mask=0x3F;
        CAN_BitModify(_CNF1,&mask,&Register);
    }  
    Register=0;
    if(Read!=0){
        CAN_ReadRegister(_CNF1,&Register);
        return (Register&(0x3F));
    }
    return 0xFF;
}

/**
 * @brief Configures the PropSeg, i.e. gets and sets PropSeg time length value depending upon the passed arguments.
 *          The actual value of PropSeg will be value denoted by PropSeg bits + 1 with units of Tq (Time Quanta of CAN communication), this is because the PropSeg must be atleast of size 1Tq.
 * @param uint8_t set_PropSeg passes the PropSeg value that has to be set.
 *          - 0xFF : if user don't want to modify the PropSeg time lenght value.
 *          - 0x00 to 0x07 : PropSeg time length values, actual value will be (PropSeg bit values+1)Tq.
 * @param uint8_t Read tells whether to read the PropSeg or not.
 *          - 0 : don't read PropSeg.
 *          - 1 : read PropSeg.
 * @retval uint8_t tells the value of PropSeg if Read is 1 else returns 0xFF, EINVALMODE if not in configuration mode.
*/
uint8_t CAN_ConfigurePropSeg(uint8_t set_PropSeg,uint8_t Read){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }

    if(set_PropSeg!=(0xFF)){
        Register=set_PropSeg;
        uint8_t mask=0x07;
        CAN_BitModify(_CNF2,&mask,&Register);
    }
    Register=0;
    if(Read!=0){
        CAN_ReadRegister(_CNF2,&Register);
        return (Register&(0x07));
    }
    return 0xFF;
}

/**
 * @brief Configures the PS1, i.e. gets and sets PS1 time length value depending upon the passed arguments.
 *          The actual value of PS1 will be value denoted by PS1 bits + 1 with units of Tq (Time Quanta of CAN communication), this is because the PS1 must be atleast of size 1Tq.
 * @param uint8_t set_PS1 passes the PS1 value that has to be set.
 *          - 0xFF : if user don't want to modify the PS1 time length value.
 *          - 0x00 to 0x07 : PS1 time length values, actual value will be (PS1 bit values+1)Tq.
 * @param uint8_t Read tells whether to read the PS1 or not.
 *          - 0 : don't read PS1.
 *          - 1 : read PS1.
 * @retval uint8_t tells the value of PS1 if Read is 1 else returns 0xFF, EINVALMODE if not in configuration mode.
 */
uint8_t CAN_ConfigurePS1(uint8_t set_PS1,uint8_t Read){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    if(set_PS1!=(0xFF)){
        Register=(set_PS1<<3)&(0x38);
        uint8_t mask=0x38;
        CAN_BitModify(_CNF2,&mask,&Register);    
    }
    Register=0;
    if(Read!=0){
        CAN_ReadRegister(_CNF2,&Register);
        return ((Register>>3)&(0x07));
    }
    return 0xFF;
   
}

/**
 * @brief Configures the PS2, i.e. gets and sets PS2 time length value depending upon the passed arguments.
 *          The actual value of PS2 will be value denoted by PS2 bits + 1 with units of Tq (Time Quanta of CAN communication), minimum value of PS2 time length is 2Tq, thus set_PS2 must be atleast 1 (0x00 is invalid configuration)
 * @param uint8_t set_PS2 passes the PS2 value that has to be set.
 *          - 0xFF : if user don't want to modify the PS2 time length value.
 *          - 0x01 to 0x07 : PS2 time length values, actual value will be (PS2 bit values+1)Tq.
 * @param uint8_t Read tells whether to read the PS2 or not.
 *          - 0 : don't read PS2.
 *          - 1 : read PS2.
 * @retval uint8_t tells the value of PS2 if Read is 1 else returns 0xFF, EINVALMODE if not in configuration mode.
 */
uint8_t CAN_ConfigurePS2(uint8_t set_PS2,uint8_t Read){
     uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    if(set_PS2!=(0xFF)){
        Register=set_PS2;
        uint8_t mask=0x07;
        CAN_BitModify(_CNF3,&mask,&Register);
    }
    Register=0;
    if(Read!=0){
        CAN_ReadRegister(_CNF3,&Register);
        return (Register&(0x07));
    }
    return 0xFF;

}

/**
 * @brief Gets the baudrate at which the CAN system is communicating.
 * @param uint32_t* passes address of the variable where the baudrate will be copied.
 * @retval uint8_t tells whether the baud rate retrieval is successful or not, 0 for success , EINVALMODE for invalid mode.
 */
uint8_t CAN_GetBaudRate(uint32_t* baudrate){
	uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    Register=0;
	CAN_ReadRegister(_CNF2,&Register);
    /* To get the baud rate, we need to read all CNF registers */
	baudrate=1 + ((Register&(0x07))+1) + (((Register>>3)&(0x07))+1);
    /* Units in Tq, 1 for SyncSeg; Register&(0x07) + 1 for PropSeg (+1 because should be 1Tq atleast); (((Register>>3)&(0x07))+1) for PS1 (+1 because should be 1Tq atleast). */
    /* Now adding the PS2 time length. */
    Register=0;
    CAN_ReadRegister(_CNF3,&Register);
    baudrate+=((Register&(0x07))+1);
    
    /* Now we have the Baudrate which at current represents the bit time in Tq units, i.e. (t(SyncSeg)+t(PropSeg)+t(PS1)+t(PS2))*Tq  */
    /* Now multiplying baudrate with Tq value which can be calculated as following, we get baudrate variable as bit time in seconds, which on inverting gives baudrate. */
    /* Tq = (2*(BRP_Bit_Val+1))/F_OSC */
    /* Getting BRP values. */
    Register=0;
    CAN_ReadRegister(_CNF1,&Register);
    baudrate*=((2*((Register&(0x3F))+1))/OSC_FREQ);
    baudrate=(1/baudrate);      /* This should be float value but not all MCUs have FPU, thus this will return the integral part of the baudrate i.e. the actual baudrate can be in range [baudrate,baudrate+1] */
    return 0;
}

/**
 * @brief Sets specific baud rate for the subsequent communication over the CAN bus.
 *          As explained above at the begining of " CAN BIT TIMING CONFIGURATION AND SYNCHRONIZATION " section, the baudrate depends over several variables which includes BV_brp, BV_propseg, BV_ps1, BV_ps2 and F_OSC.
 *          So, to set a specific baudrate for the device, one need to set all of those variables to specific allowed values, the baudrate follows the mathematical relation : 
 *          
 *          baudrate(F_OSC, BV_brp, BV_propseg, BV_ps1, BV_ps2) = F_OSC/(2*(1+BV_brp)*(4+BV_propseg+BV_ps1+BV_ps2))
 *      
 *          CAN_SetBaudRate() assumes the F_OSC variable as OSC_FREQ macro declared in /Inc/Pin_connection.h , rest of all have to be passed by the user.
 *          Since the library boots up the device with specific configurations of configuration registers as explained above, one can relate the variables value for their own needs with respect to what this library sets during bootup.
 *          
 * @param uint8_t BV_brp passes the value of the BRP segment, between 0x00-0x063
 * @param uint8_t BV_propseg passes the value of the propseg segment, between 0x00-0x07
 * @param uint8_t BV_ps1 passes the value of the PS1 segment, between 0x00-0x07
 * @param uint8_t BV_ps2 passes the value of the PS2 segment, between 0x00-0x07
 * @retval uint8_t tells whether the new baudrate has been set or not, return 0 on success and EINVALMODE if not in configuration mode.
 */
uint8_t CAN_SetBaudRate(uint8_t BV_brp, uint8_t BV_propseg, uint8_t BV_ps1, uint8_t BV_ps2){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    
    /* Writing to CNFn registers. */
    Register=0;
    /* Modifying CNF1 */
    Register=BV_brp;
    uint8_t mask=(0x3F);
    CAN_BitModify(_CNF1,&mask,&Register);
    Register=0;
    /* Modifying CNF2 */
    Register=(BV_propseg&(0x03))|((BV_ps1<<3)&(0x38))
    mask=0x3F;
    CAN_BitModify(_CNF2,&mask,&Register);
    Register=0;
    /* Modifying CNF3 */
    Register=BV_ps2; 
    mask=0x07;
    CAN_BitModify(_CNF3,&mask,&Register);
    return 0;
}

/**
 * @brief Enables and disables triple sampling for the bits by modifying the SAM bit of CNF2
 * @param uint8_t set_TS_bit tells whether to set the SAM bit to 0 or 1, passing 0 means setting the bit to 0 and passing 1 means setting it to 1.
 * @retval 0 on success and EINVALMODE if not in configuration mode.
 */
uint8_t CAN_ConfigureTripleSampling(uint8_t set_TS_bit){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    Register=0;
    Register=set_TS_bit;
    uint8_t mask=0x80;
    CAN_BitModify(_CNF2,&mask,set_TS_bit); 
    return 0;
}

/**
 * @brief Enables and disables BTLMODE i.e. if set to 0, BV_ps2 is set automatically equals to BV_ps1 and if set to 1, BV_ps2 value will be taken as bit value of PS2 segment, i.e. user can program PS2 independently of PS1,
 *        This can be useful while adjusting the sampling point and baudrate.
 * @param uint8_t set_BTLMODE_bit tells whether to set the BTLMODE bit of CNF2 register to 0 or 1, passing 0 sets it to 0 and passing 1 sets it to 1.
 * @retval 0 on success and EINVALMODE if not in configuration mode.
 */
uint8_t CAN_ConfigureBTLMODE(uint8_t set_BTLMODE_bit){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    Register=0;
    Register=set_BTLMODE_bit;
    uint8_t mask=0x40;
    CAN_BitModfy()CNF2,&mask,&Register);
}

/**
 * @brief Enables and disables Wake up mode by setting WAKFIL bit of CNF3 register.
 * @param uint8_t set_WAKFIL_bit tells whether to set the WAKFIL bit to 0 or 1, passing 0 sets it to 0 and passing 1 sets it to 1.
 * @retval 0 on success and EINVALMODE if not in configuration mode.
 */
uint8_t CAN_ConfigureWakeUp(uint8_t set_WAKFIL_bit){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    Register=0;
    Register=set_WAKFIL_bit;
    uint8_t mask=0x40;
    CAN_BitModify(_CNF3,&mask,&Register);
}

/**
 * @brief Retrieves the value of TEC and REC register which can be used for identifying the error state of the device.
 * @param uint8_t TEC_or_REC tells whether to read REC or TEC, 0 means reading TEC and 1 means reading REC register.
 * @retval uint8_t tells the value of REC/TEC register depending upon the argument TEC_or_REC. 
 */
uint8_t CAN_Get_TEC_REC(uint8_t TEC_or_REC){
    uint8_t Register=0;
    if(TEC_or_REC==0){
        CAN_ReadRegister(TEC,&Register);
    }else{
        CAN_ReadRegister(REC,&Register);
    }
    return Register;
}

/**
 * @brief Configures the SOF bit of CNF3 register, used to put CLKOUT pin into two different modes, 1.) putting CLKOUT pin in SOF mode in which whenever the SOF is observed on bus by the node, it will generate 
 *        signals on this pin, its used for synchronization with external devices. 2.) putting CLKOUT in a mode where it emits the frequency to drive the peripherals. marking the bit 0 means in second mode, while 
 *        marking the bit 1 means in SOF mode, this bit is only considered if the CLKEN bit in CANCTRL register is set to 1, else this bit not be considered.
 * @param uint8_t set_or_unset tell whether to set the SOF bit 0 or 1, passing 0 means setting the bit to 0 and passing 1 does the opposite.
 * @retval 0 on success and EINVALMODE if not in configuration mode.
 */
uint8_t CAN_ConfigureSOF_CLKOUT(uint8_t set_or_unset){
    uint8_t Register=0;
    CAN_ReadRegister(_CANCTRL,&Register);
    if(((Register>>5)&(0x07))!=(0x04)){
       return EINVALMODE; 
    }
    Register=set_or_unset;
    uint8_t mask=0x80;
    CAN_BitModify(_CNF3,&mask,&Register);
    return 0; 
}

/**
 * @brief Retrieves the value of EFLG register.
 * @param uint8_t* eflg_val passes the address of the variable where the EFLG register value will be stored.
 * @retval void
 */
void CAN_GetEFLG(uint8_t eflg_val){
    uint8_t CAN_ReadRegister(EFLG,&eflg_val);
}

/**
 * @brief Sets the RX0OVR and RX1OVR bits, other bits are read only.
 * @param uint8_t RXnOVR_val passes the value for those two bits. thus it can have the values 0x00, 0x01, 0x02, 0x03.
 * @retval void
 */
void CAN_SetEFLG(uint8_t RXnOVR_val){
    uint8_t mask=0xC0;
    uint8_t Register=RXnOVR_val;
    CAN_BitModify(EFLG,&mask,&Register);
}
 
/*
 *	CAN IO API and Initialization routine.
 *
*/

/*
 * @brief 
 *
*/
void CAN_Init(){
	
	
}
