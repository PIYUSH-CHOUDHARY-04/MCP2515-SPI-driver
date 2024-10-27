<h1>MCP2515 Standalone SPI CAN controller driver v1.0</h1>
<h2> Author : Piyush Choudhary</h2>

    Project Structure :
    /MCP2515-SPI-driver
    |
    |---/docs
    |    |--- MCP2515-Standalone-CAN-controller-User-manual.pdf
    |    |
    |    |
    |
    |
    |---/Inc
    |    |--- CAN_mcp2515.h
    |    |--- MCP2515.h
    |    |--- Pin_connection.h
    |
    |
    |---/Src
    |    |--- CAN_mcp2515.c
    |    
    |---/resources
    |    |--- MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf
    |    |
    |
    |--- Readme.md
    

<p> This driver offers you a high level API for interacting with CAN controller as well as low level API for those who want more fine-grain control over the CAN controller. <p>
<p>NOTE00 : Before using this driver, make sure that you've initialized HAL library by calling HAL_Init() and also its sub-sections like HAL_GPIO, HAL_SPI, HAL_NVIC etc by calling HAL_GPIO_Init(), HAL_SPI_Init(), HAL_NVIC_Init() respectively etc.<p>
<p>NOTE01 : The SPI structure 'SPI_HandleTypeDef' should be initialized with name 'hspi1', 'hspi1' is used by default in this entire library for SPI communication.<p>
<p>NOTE02 : This library is written for the "Little Endian" architecture.<p>

<p> MCP2515 CAN API is partitioned into a stack of lower to higher level routines.<p>

<table>
    <tr>
        <th><h2>CAN IO API Routines</h2></th>
        <th><h2>Use case</h2></th>
    </tr>
    <tr>
        <th>CAN_init()</th>
        <th>Initializes MCP2515 CAN controller in default state.</th>
    </tr>
    <tr>
        <th>CAN_rx()</th>
        <th>Receives a CAN frame.</th>
    </tr>
    <tr>
        <th>CAN_tx()</th>
        <th>Transmits a CAN frame.</th>
    </tr>
</table>

<table>
    <tr>
        <th><h2>CAN Config API Routines</h2></th>
        <th><h2>Use case</h2></th>
    </tr>
    <tr>
        <th>CAN_HardReset()</th>
        <th>Resets the CAN hardware by lowering its ~RESET pin</th>
    </tr>
    <tr>
        <th>CAN_SoftReset()</th>
        <th>Resets the CAN hardware by sending RESET control command over SPI</th>
    </tr>
    <tr>
        <th>CAN_SwitchMode()</th>
        <th>Changes the operating mode of MCP2515</th>
    </tr>
    <tr>
        <th>CAN_WriteRegister()</th>
        <th>Writes to specific MCP2515 register</th>
    </tr>
    <tr>
        <th>CAN_ReadRegister()</th>
        <th>Reads from the specific register of MCP2515</th>
    </tr>
    <tr>
        <th>CAN_WriteFrame()</th>
        <th>Writes the entire CAN frame to the specific TX buffer.</th>
    </tr>
    <tr>
        <th>CAN_ReadFrame()</th>
        <th>Reads the entire CAN frame from the MCP2515.</th>
    </tr>
    <tr>
        <th>CAN_SetInterrupt()</th>
        <th>Sets specific interrupts ( remove pre-existing interrupts )</th>
    </tr>
    <tr>
        <th>CAN_GetInterrupt()</th>
        <th>Gets the currently set interrupts for MCP2515</th>
    </tr>
    <tr>
        <th>CAN_EnableInterrupt()</th>
        <th>Enables specific interrupt, do not clear pre-existing interrupts</th>
    </tr>
    <tr>
        <th>CAN_DisableInterrup()</th>
        <th>Disables specific interrupt for the MCP2515</th>
    </tr>
    <tr>
        <th>CAN_SetBaudRate()</th>
        <th>Sets a specific standard baud rate for CAN communication</th>
    </tr>
    <tr>
        <th>CAN_GetBaudRate()</th>
        <th>Gets the current baud rate of the CAN communication</th>
    </tr>
     <tr>
        <th>CAN_TriggerRTS()</th>
        <th>Triggers the RTS for specific TXBn</th>
    </tr>
    <tr>
         <th>CAN_AbortTX()</th>
         <th>Aborts an ongoing transmission for a specific TX buffer</th>
    </tr>
    <tr>
         <th>CAN_AbortAllTX()</th>
         <th>Aborts all ongoing transmission for each TX buffer</th>
    </tr>
    <tr>
         <th>CAN_EnableClkOut()</th>
         <th>Enables the CLKOUT pin of the MCP2515 CAN controller</th>
    </tr>
     <tr>
         <th>CAN_DisableClkOut()</th>
         <th>Disables the CLKOUT pin of the MCP2515 CAN controller</th>
    </tr>
     <tr>
         <th>CAN_SetClkOutFreq()</th>
         <th>Sets the clock frequency for CLKOUT pin of the MCP2515 CAN controller</th>
    </tr>
     <tr>
         <th>CAN_EnableOSM()</th>
         <th>Enables the One Shot Mode (OSM) for all TX buffers</th>
    </tr>
     <tr>
         <th>CAN_DisableOSM()</th>
         <th>Disables the One Shot Mode (OSM) for all TX buffers</th>
    </tr>
    <tr>
        <th>CAN_ChangeTXPriority()</th>
        <th>Changes the transmission priority of each TX buffer</th>
    </tr>


</table>

<p>Under development...</p>


