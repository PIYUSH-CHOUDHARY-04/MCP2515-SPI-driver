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

/**
 * @defgroup HAL_HEADER_INCLUSION hal_header_inclusion
 * @brief This inclusion section is modifiable, do appropriate inclusion of the SPI and GPIO header files of your specific microprocessor.
 * @{
 */
#include "HAL_SPI.h"
#include "HAL_GPIO.h"

/**
 * @}
 */


#include "MCP2515.h"
#include "Pin_connection.h"




/**
 * @brief CAN frame structure for frame transmission and reception.
 * @param uint32_t CAN_ID will hold CAN ID of the specific CAN frame.
 *          The CAN ID field holds CAN ID in format : SID[0:10]<id10,id9,...,id0>|EID[11:28]<id11,id12,...,id28>
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
 *	- 0x00 for disabling roll over.
 *	- 0x04 for enabling roll over.
 * @retval void
 */
void CAN_RollOver2RX1(uint8_t roll_ovr);

/**
 * @brief Retrieves the specific filter value.
 * @param uint8_t* FILTn passes the address of the filter whose value has to be retrieved.
 * @param uint32_t* filt_val passes the address of the 4-byte where the filter value will be stored or copied to.
 * @retval uint8_t tells whether the filter contains the exide value set to 0 or 1 i.e. the filter allows extended or standard frame, can be neglected by setting exide bit to be 0 in corresponding mask.
 *          - 0 if the filter is for standard frame.
 *          - 1 if the filter is for extended frame.
 *          - EINVALMODE if the mode is not configuration mode since dealing with filters and masks needs the device to be in configuration mdoe only.
 */
uint8_t CAN_GetFilter(uint8_t* FILTn,uint32_t* filt_val);

/**
 * @brief Sets the specific filter.
 * @param uint8_t* FILTn passes the address of the filter whose value has to be set.
 * @param uint32_t* filt_val passes the address of the 4-byte filter which has to be set.
 * @param int8_t exide_val passes the value for exide bit i.e. whether the filter is intended for standard or extended frame, 0 for standard frame and 1 for extended frame, works only if the corresponding exide bit in mask is set to 1.
 * @retval void
 */
void CAN_SetFilter(uint8_t* FILTn,uint32_t* filt_val, uint8_t exide_val);

/**
 * @brief Retrieves the specific mask value.
 * @param uint8_t* MASKn passes the address of the mask whose value has to be retrieved.
 * @param uint32_t* mask_val passes the address of the 4-byte where the mask value will be stored or copied to.
 * @retval uint8_t tells whether the mask contains the exide value set to 0 or 1 i.e. 0 means frame type checing is disabled and 1 means frame type checking is enabled.
 *          - 0 : frame type checking by filters disabled.
 *          - 1 : frame type checking by filters enabled.
 *          - EINVALMODE if the mode is not configuration mode since dealing with filters and masks needs the device to be in configuration mdoe only.
 */
uint8_t CAN_GetMask(uint8_t* MASKn,uint32_t* mask_val);

/**
 * @brief Sets the specific mask.
 * @param uint8_t* MASKn passes the address of the mask whose value has to be set.
 * @param uint32_t* mask_val passes the address of the 4-byte mask which has to be set.
 * @param uint8_t exide_val passes the value for exide bit, if 0 means you are disabling frame type checking and if 1 means you are enabling frame type checking. 
 * @retval void
 */
void CAN_SetMask(uint8_t* MASKn,uint32_t* mask_val, uint8_t exide_val); 






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
 *       The following routines can be used to adjust sampling point as well.
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
 * @retval uint8_t tells the value of SJW if Read is 1 else returns 0xFF.
 */
uint8_t CAN_ConfigureSJW(uint8_t set_SJW,uint8_t Read);

/**
 * @brief Configures the BRP, i.e. gets and sets BRP value depending upon the passed arguments.
 * @param uint8_t set_BRP passes the BRP value that has to be set.
 *          - 0xFF : if user don't want to modify the BRP value.
 *          - 0x00 to 0x3F : prescalar values by which the baudrate will be scaled.
 * @param uint8_t Read tells whether to read the SJW or not.
 *          - 0 : don't read BRP.
 *          - 1 : read BRP.
 * @retval uint8_t tells the value of BRP if Read is 1 else returns 0xFF.
 */
uint8_t CAN_ConfigureBRP(uint8_t set_BRP,uint8_t Read);

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
uint8_t CAN_ConfigurePropSeg(uint8_t set_PropSeg,uint8_t Read);

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
uint8_t CAN_ConfigurePS1(uint8_t set_PS1,uint8_t Read);

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
uint8_t CAN_ConfigurePS2(uint8_t set_PS2,uint8_t Read);

/**
 * @brief Gets the baudrate at which the CAN system is communicating.
 * @param uint32_t* passes address of the variable where the baudrate will be copied.
 * @retval uint8_t tells whether the baud rate retrieval is successful or not, 0 for success , EINVALMODE for invalid mode.
 */
uint8_t CAN_GetBaudRate(uint32_t* baudrate);

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
uint8_t CAN_SetBaudRate(uint8_t BV_brp, uint8_t BV_propseg, uint8_t BV_ps1, uint8_t BV_ps2);

/**
 * @brief Enables and disables triple sampling for the bits by modifying the SAM bit of CNF2
 * @param uint8_t set_TS_bit tells whether to set the SAM bit to 0 or 1, passing 0 means setting the bit to 0 and passing 1 means setting it to 1.
 * @retval void
 */
void CAN_ConfigureTripleSampling(uint8_t set_TS_bit);

/**
 * @brief Enables and disables BTLMODE i.e. if set to 0, BV_ps2 is set automatically equals to BV_ps1 and if set to 1, BV_ps2 value will be taken as bit value of PS2 segment, i.e. user can program PS2 independently of PS1,
 *        This can be useful while adjusting the sampling point and baudrate.
 * @param uint8_t set_BTLMODE_bit tells whether to set the BTLMODE bit of CNF2 register to 0 or 1, passing 0 sets it to 0 and passing 1 sets it to 1.
 * @retval void
 */
void CAN_ConfigureBTLMODE(uint8_t set_BTLMODE_bit);

/**
 * @brief Enables and disables Wake up mode by setting WAKFIL bit of CNF3 register.
 * @param uint8_t set_WAKFIL_bit tells whether to set the WAKFIL bit to 0 or 1, passing 0 sets it to 0 and passing 1 sets it to 1.
 * @retval void
 */
void CAN_ConfigureWakeUp(uint8_t set_WAKFIL_bit);

/**
 * @brief Retrieves the value of TEC and REC register which can be used for identifying the error state of the device, for clearing these register, device reset is necessary.
 * @param uint8_t TEC_or_REC tells whether to read REC or TEC, 0 means reading TEC and 1 means reading REC register.
 * @retval uint8_t tells the value of REC/TEC register depending upon the argument TEC_or_REC. 
 */
uint8_t CAN_Get_TEC_REC(uint8_t TEC_or_REC);

/**
 * @brief Configures the SOF bit of CNF3 register, used to put CLKOUT pin into two different modes, 1.) putting CLKOUT pin in SOF mode in which whenever the SOF is observed on bus by the node, it will generate 
 *        signals on this pin, its used for synchronization with external devices. 2.) putting CLKOUT in a mode where it emits the frequency to drive the peripherals. marking the bit 0 means in second mode, while 
 *        marking the bit 1 means in SOF mode, this bit is only considered if the CLKEN bit in CANCTRL register is set to 1, else this bit not be considered.
 * @param uint8_t set_or_unset tell whether to set the SOF bit 0 or 1, passing 0 means setting the bit to 0 and passing 1 does the opposite.
 * @retval void
 */
void CAN_ConfigureSOF_CLKOUT(uint8_t set_or_unset);
 
/**
 * @brief Retrieves the value of EFLG register.
 * @param uint8_t* eflg_val passes the address of the variable where the EFLG register value will be stored.
 * @retval void
 */
void CAN_GetEFLG(uint8_t* eflg_val);

/**
 * @brief Sets the RX0OVR and RX1OVR bits, other bits are read only.
 * @param uint8_t RXnOVR_val passes the value for those two bits. thus it can have the values 0x00, 0x01, 0x02, 0x03.
 * @retval void
 */
void CAN_SetEFLG(uint8_t RXnOVR_val);




/*====================================================================<<IO APIS>>===================================================================*/



/**
 * @brief Initializes CAN MCP2515 hardware and prepare it for frame reception and transmission.
 * @param void
 * @retval void
 */
void CAN_Init(void);

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
uint8_t CAN_rx(can_t* frame);


#endif /* CAN_MCP2515_H */

