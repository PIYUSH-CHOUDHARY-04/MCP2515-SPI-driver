<h1>MCP2515 Standalone SPI CAN controller driver v1.0</h1>
<h2> Author : Piyush Choudhary</h2>
<h3> This driver offers you a high level API for interacting with CAN controller as well as low level API for those who want more fine-grain control over the CAN controller. </h3>
<h3>NOTE : Before using this driver, make sure that you've initialized HAL library by calling HAL_Init() and also its sub-sections like HAL_GPIO, HAL_SPI, HAL_NVIC etc by calling HAL_GPIO_Init(), HAL_SPI_Init(), HAL_NVIC_Init() respectively etc.</h3>
<h3> MCP2515 CAN API is partitioned into two parts : CAN IO API and CAN Config API</h3>

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
        <th>CAN_Write_ID()</th>
        <th>Writes the CAN ID of the frame to the MCP2515</th>
    </tr>
    <tr>
        <th>CAN_Read_ID()</th>
        <th>Reads the CAN ID of a specific frame from MCP2515</th>
    </tr>
    <tr>
        <th>CAN_WriteData()</th>
        <th>Writes data field of CAN frame to the MCP2515</th>
    </tr>
    <tr>
        <th>CAN_ReadData()</th>
        <th>Reads the data field of a specific CAN frame from MCP2515</th>
    </tr>
    <tr>
        <th>CAN_setDLC()</th>
        <th>Write the DLC field of the CAN frame to MCP2515</th>
    </tr>
    <tr>
        <th>CAN_getDLC()</th>
        <th>Reads the DLC field of the CAN frame from MCP2515</th>
    </tr>
    <tr>
        <th>CAN_setInterrupt()</th>
        <th>Sets specific interrupts ( remove pre-existing interrupts )</th>
    </tr>
    <tr>
        <th>CAN_getInterrupt()</th>
        <th>Gets the currently set interrupts for MCP2515</th>
    </tr>
    <tr>
        <th>CAN_enableInterrupt()</th>
        <th>Enables specific interrupt, do not clear pre-existing interrupts</th>
    </tr>
    <tr>
        <th>CAN_disableInterrup()</th>
        <th>Disables specific interrupt for the MCP2515</th>
    </tr>
    <tr>
        <th>CAN_SetBaudRate()</th>
        <th>Sets a specific standard baud rate for CAN communication</th>
    </tr>
    <tr>
        <th>CAN_GetBaudrate()</th>
        <th>Gets the current baud rate of the CAN communication</th>
    </tr>
    
</table>

<p>Under development...</p>


