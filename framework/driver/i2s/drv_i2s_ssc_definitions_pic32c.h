/*******************************************************************************
  I2S(SSC Module) PIC32C Driver Definitions Header File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2s_ssc_definitions_pic32c.h

  Summary:
    I2S(SSC Module) PIC32C Driver Definitions Header File

  Description:
    This file will provide enumerations and other dependencies needed by
    I2S driver to manage the I2S(SPI) module on PIC32 microcontrollers.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/

#include "system/int/sys_int.h"
#include "system/dma/sys_dma.h"
#include "driver/driver_common.h"

#ifndef DRV_I2S__SSC_DEFINITIONS_PIC32C_H_
#define DRV_I2S__SSC_DEFINITIONS_PIC32C_H_

// *****************************************************************************
/* SSC Module Receiver Clock Selection

  Summary:
    Identifies the Receiver clock selection options for SSC Module.

  Description:
    This enumeration identifies SSC receiver clock options available.

  Remarks:
    None.
*/

typedef enum
{
    /* Divided Clock */
    DRV_I2S_SSC_RX_DIVIDED_CLOCK,

    /* TK signal */
    DRV_I2S_SSC_RX_TK_SIGNAL,
    
    /* RK pin */
    DRV_I2S_SSC_RX_RK_PIN,

} DRV_I2S_SSC_RX_CLK_SEL;

// *****************************************************************************
/* SSC Module Receiver Clock Output Mode Selection

  Summary:
    Identifies the Receiver Clock output mode selection.

  Description:
    This enumeration identifies receiver clock output mode options available.
    RK pin can be used as input, or used to output signal continuously or
    used output only during data transfers.
    
  Remarks:
    None.
*/

typedef enum 
{
     /* RK Pin is an input */
     I2S_SSC_NONE_RK_PIN_AS_INPUT = 0,
     
     /* RK Pin outputs clock continuously */
     I2S_SSC_CONTINUOUS_RX_CLK_OUTPUT,
     
    /* RK pin outputs clock only during data transfers */
    I2S_SSC_CLK_ONLY_DURING_DATA_TRANSFER,
    
} DRV_I2S_SSC_RX_CLK_OUTPUT_MODE_SEL;

// *****************************************************************************
/* SSC Module Receiver Clock inversion Options

  Summary:
    Identifies clock inversion option available for SSC Module.

  Description:
    This enumeration identifies data inputs are sampled on which RX clock edge 
    and RX Frame-sync output signal is shifted out on which RX clock edge.

  Remarks:
    None.
*/

typedef enum 
{
     /* Data inputs are sampled on RX clock falling edge and FS output is 
      * shifted out on RX clock rising edge */
     I2S_SSC_RX_FALLING_RISING_EDGE = 0,
     
     /* Data inputs are sampled on RX clock rising edge and FS output is 
      * shifted out on RX clock falling edge */
     I2S_SSC_RX_RISING_FALLING_EDGE,
     
} DRV_SSC_RX_CLOCK_INVERSION;

// *****************************************************************************
/* SSC Module Receiver Clock gating Selection

  Summary:
    Identifies Receiver clock gating options available for SSC Module.

  Description:
    This enumeration identifies gating options available for SSC Module
    receive clock. This clock can be enabled always(when receiver is enabled),
    or based on RF signal low or high level.

  Remarks:
    None.
*/

typedef enum 
{
     /* None. Receive clock is enabled always(when receiver is enabled) */
     I2S_SSC_RX_CLK_CONTINUOUS = 0,
     
     /* Receive clock is enabled only when RF(Rx Frame sync) signal is low */
     I2S_SSC_RX_CLK_EN_RF_LOW,
     
     /* Receive clock is enabled only when RF(Rx Frame sync) signal is high */
     I2S_SSC_RX_CLK_EN_RF_HIGH,
     
} DRV_I2S_SSC_RX_CLOCK_GATING_SEL;

// *****************************************************************************
/* SSC Module Receive Start Selection

  Summary:
    Identifies Receive start sleection options available for SSC Module

  Description:
    This enumeration identifes Receive start options available in SSC Module.
    Rx Start can be continuous one or based on tranmsit start or can be set
    based on change on RF signal edge/level.

  Remarks:
    None.
*/

typedef enum
{
    /* Contiuous, means reception starts as soon as receiver is enabled */
    I2S_SSC_RX_CONTINUOUS = 0,
    
    /* Receive start on detecting a transmit start  */ 
    I2S_SSC_RX_TRANSMIT,
    
    /* Receive starts on detecting a low level on RF signal */
    I2S_SSC_RX_RF_LOW,
    
    /* Receive starts on detecting a high level on RF signal */
    I2S_SSC_RX_RF_HIGH,
    
    /* Receive starts on detecting a falling edge on RF signal */
    I2S_SSC_RX_RF_FALLING,
    
    /* Receive starts on detecting a rising edge on RF signal */
    I2S_SSC_RX_RF_RISING,
    
    /* Receive starts on detecting any level change on RF signal */
    I2S_SSC_RX_RF_LEVEL,
    
    /* Receive starts on detecting any edge on RF signal */
    I2S_SSC_RX_RF_EDGE,
    
    /* Receive starts when received data(last n bits) matches with compare0
     * register */
    I2S_SSC_RX_COMP_0,
                
} DRV_SSC_RX_START_SEL;

// *****************************************************************************
/* SSC Module Receiver STOP condition

  Summary:
    Identifies stop conditions available for data reception in SSC Module when
    receive start is triggered by Compare 0 register

  Description:
    This enumeration identifies stop conditions available for receiver when 
    receive start is triggered by Compare 0 register macth. Can either stop
    receptiona nd wait for new compare 0 match or operate in continuous mode
    till compare 1 match occurs.
    
  Remarks:
    None.
*/

typedef enum
{
    /* After completion of data reception when starting with Compare 0 register,
     * the receiver stops the data reception & waits for new compare 0 match */
    I2S_SSC_RX_WAIT_NEW_CMP_0 = 0,
    
    /* After starting a receive with Compare 0, receiver operates in
     * continuous mode until a compare 1 is detected */
    I2S_SSC_RX_CONTINUOUS_AFTER_COMP_0,                
        
} DRV_SSC_RX_STOP;

// *****************************************************************************
/* SSC Module Receiver Loop Mode

  Summary:
    Identifies receiver should operate in normal or loop mode.

  Description:
    This enumeration identifies that whether receiver should operate in
    normal or loop mode.

  Remarks:
    None.
*/

typedef enum
{
    /* SSC operates in Normal mode */
    I2S_SSC_NORMAL_MODE = 0,
    
    /* SSC operates in Loop modes, means TD drives RD, TK drives RK, and
     * TF drives RF. */
    I2S_SSC_LOOP_MODE,                
        
} DRV_SSC_LOOP_MODE;

// *****************************************************************************
/* SSC Module Receiver LSB or MSB sampling order

  Summary:
    Identifies sampling order of SSC module receiver

  Description:
    This enumeration Identifies MSB or LSB should be sampled first by SSC
    module receiver.

  Remarks:
    None.
*/

typedef enum
{
    /* The Least Significant bit of the data register is sampled first*/
    I2S_SSC_RX_LSB_SAMPLED_FIRST = 0,
    
    /* The Most signiicant bit of the data register is sampled first */
    I2S_SSC_RX_MSB_SAMPLED_FIRST,                
        
} DRV_SSC_RX_MSBF;

// *****************************************************************************
/* SSC Module Receiver Frame Sync Output selection

  Summary:
    Identifies options available for receiver's frame sync signal of SSC module. 

  Description:
    This enumeration identifes the options available for receiver's frame sync
    signal. FS signal can be made as input, can output a positive or negative
    pulse, can be driver low or high during data transfer and can be toggled at
    each data transfer.

  Remarks:
    None.
*/

typedef enum
{
    /* None. RF pin will act as an input pin */
    I2S_SSC_RX_FS_OUTPUT_NONE = 0,
    
    /* RF pin outputs a negative pulse during data transfer */
    I2S_SSC_RX_FS_OUTPUT_NEGATIVE,
    
    /* RF pin outputs a positive pulse during data transfer */
    I2S_SSC_RX_FS_OUTPUT_POSITIVE,
    
    /* RF pin driven low during data transfer */
    I2S_SSC_RX_FS_OUTPUT_LOW,
    
    /* RF pin driven high during data transfer */
    I2S_SSC_RX_FS_OUTPUT_HIGH,
    
    /* RF pin toggles at the start of each data transfer */
    I2S_SSC_RX_FS_OUTPUT_TOGGLING,                
        
} DRV_SSC_RX_FS_OUTPUT_SEL;

// *****************************************************************************
/* SSC Module Receiver Frame Sync Edge detection

  Summary:
    Identifies which edge of frame sync will generate RXSYN interrupt.

  Description:
    This enumeration identifies which edge of receiver's frame sync signal will
    generate RXSYN interrupt in SSC status register.

  Remarks:
    None.
*/

typedef enum
{    
    /* Postive edge of RF signal will generate RXSYN interrupt */
    I2S_SSC_RX_FS_EDGE_POSITIVE = 0,
    
    /* Negative edge of RF signal will generate RXSYN interrupt */
    I2S_SSC_RX_FS_EDGE_NEGATIVE,
                        
} DRV_SSC_RX_FS_EDGE_DETECTION;

// *****************************************************************************
/* SSC Module Transmitter Clock Selection

  Summary:
    Identifies the transmitter clock selection options for SSC Module.

  Description:
    This enumeration identifies SSC transmitter clock options available.

  Remarks:
    None.
*/

typedef enum
{
    /* Divided Clock */
    DRV_I2S_SSC_TX_DIVIDED_CLOCK,

    /* RK Clock signal */
    DRV_I2S_SSC_TX_RK_SIGNAL,
    
    /* TK pin of transmitter. Means transmitter clock is fed from outside to 
     * MCU */
    DRV_I2S_SSC_TX_TK_PIN,

} DRV_I2S_SSC_TX_CLK_SEL;

// *****************************************************************************
/* SSC Module Transmitter Clock Output Mode Selection

  Summary:
    Identifies the Transmitter Clock output mode selection.

  Description:
    This enumeration identifies transitter clock output mode options available.
    TK pin can be used as input, or used to output signal continuously or
    used output only during data transfers.
    
  Remarks:
    None.
*/

typedef enum 
{
     /* TK Pin is an input */
     I2S_SSC_NONE_TK_PIN_AS_INPUT = 0,
     
     /* TK Pin outputs clock continuously */
     I2S_SSC_TX_CONTINUOUS_CLK_OUTPUT,
     
    /* TK pin outputs clock only during data transfers */
    I2S_SSC_TX_CLK_ONLY_DURING_DATA_TRANSFER,
    
} DRV_I2S_SSC_TX_CLK_OUTPUT_MODE_SEL;

// *****************************************************************************
/* SSC Module Transmitter Clock inversion Options

  Summary:
    Identifies clock inversion option available for SSC Module.

  Description:
    This enumeration identifies data outputs are shifted on which TX clock edge 
    and TX Frame-sync input signal is sampled on which TX clock edge.

  Remarks:
    None.
*/

typedef enum 
{
     /* Data outputs are shifted out on TX clock falling edge and FS input is 
      * sampled on TX clock rising edge */
     I2S_SSC_TX_FALLING_RISING_EDGE = 0,
     
     /* Data outputs are shifted out on TX clock rising edge and FS input is 
      * sampled on TX clock falling edge */
     I2S_SSC_TX_RISING_FALLING_EDGE,
     
} DRV_SSC_TX_CLOCK_INVERSION;

// *****************************************************************************
/* SSC Module Transmitter Clock gating Selection

  Summary:
    Identifies Transmitter clock gating options available for SSC Module.

  Description:
    This enumeration identifies gating options available for SSC Module
    transmit clock. This clock can be enabled always(when transmitter is 
    enabled), or based on TF signal low or high level.

  Remarks:
    None.
*/

typedef enum 
{
    /* None. Transmit clock is enabled always(when transmitter is enabled) */
    I2S_SSC_TX_CLK_CONTINUOUS = 0,
     
    /* Transmit clock is enabled only when TF(TX Frame sync) signal is low */
    I2S_SSC_TX_CLK_EN_RF_LOW,
     
    /* Transmit clock is enabled only when TF(TX Frame sync) signal is high */
    I2S_SSC_TX_CLK_EN_RF_HIGH,
     
} DRV_I2S_SSC_TX_CLOCK_GATING_SEL;

// *****************************************************************************
/* SSC Module Transmit Start Selection

  Summary:
    Identifies Transmit start sleection options available for SSC Module

  Description:
    This enumeration identifes Tranmist start options available in SSC Module.
    TX Start can be continuous one or based on Receive start or can be set
    based on change on TF signal edge/level.

  Remarks:
    None.
*/

typedef enum
{
    /* Contiuous, means transmission starts as soon as receiver is enabled */
    I2S_SSC_TX_CONTINUOUS = 0,
    
    /* Transmit starts on detecting a reception start  */ 
    I2S_SSC_TX_TRANSMIT,
    
    /* Transmit starts on detecting a low level on TF signal */
    I2S_SSC_TX_TF_LOW,
    
    /* Transmit starts on detecting a high level on TF signal */
    I2S_SSC_TX_TF_HIGH,
    
    /* Transmit starts on detecting a falling edge on TF signal */
    I2S_SSC_TX_TF_FALLING,
    
    /* Transmit starts on detecting a rising edge on TF signal */
    I2S_SSC_TX_TF_RISING,
    
    /* Transmit starts on detecting  any level change on TF signal */
    I2S_SSC_TX_TF_LEVEL,
    
    /* Transmit starts on detecting  any edge on TF signal */
    I2S_SSC_TX_TF_EDGE,
        
} DRV_SSC_TX_START_SEL;

// *****************************************************************************
/* SSC Module Transmitter Data default value

  Summary:
    Identifies the default data to be driven on TD pin while out of transmission

  Description:
    This enumeration identifies the default data to be transferred on TD pin
    when current data is sent out and no new data is written transmit data 
    register.

  Remarks:
    None.
*/

typedef enum
{
    /* TD will output zeros when no new data is written */
    I2S_SSC_TX_TD_OUTPUT_ZEROS = 0,
    
    /* TD will output ones when no new data is written */
    I2S_SSC_TX_TD_OUTPUT_ONES,                
        
} DRV_SSC_DATA_DEFAULT;

// *****************************************************************************
/* SSC Module Transmitter LSB or MSB sampling order

  Summary:
    Identifies sampling order of SSC module transmitter

  Description:
    This enumeration Identifies MSB or LSB should be shifted out by SSC
    module transmitter.

  Remarks:
    None.
*/

typedef enum
{
    /* The Least Significant bit of the data register is shifted out first */
    I2S_SSC_TX_LSB_SAMPLED_FIRST = 0,
    
    /* The Most Significant bit of the data register is shifted out first */
    I2S_SSC_TX_MSB_SAMPLED_FIRST,           
        
} DRV_SSC_TX_MSBF;

// *****************************************************************************
/* SSC Module Transmitter Frame Sync Output selection

  Summary:
    Identifies options available for transmitter's frame sync outout signal of 
    SSC module. 

  Description:
    This enumeration identifes the options available for transmitter's frame 
    sync output signal. FS signal can be made as input, can output a positive 
    or negative pulse, can be driver low or high during data transfer and
    can be toggled at each data transfer.

  Remarks:
    None.
*/

typedef enum
{
    /* None. TF pin will act as an input pin */
    I2S_SSC_TX_FS_OUTPUT_NONE = 0,
    
    /* TF pin outputs a negative pulse during data transfer */
    I2S_SSC_TX_FS_OUTPUT_NEGATIVE,
    
    /* TF pin outputs a positive pulse during data transfer */
    I2S_SSC_TX_FS_OUTPUT_POSITIVE,
    
    /* TF pin driven low during data transfer */
    I2S_SSC_TX_FS_OUTPUT_LOW,
    
    /* TF pin driven high during data transfer */
    I2S_SSC_TX_FS_OUTPUT_HIGH,
    
    /* TF pin toggles at the start of each data transfer */
    I2S_SSC_TX_FS_OUTPUT_TOGGLING,                
        
} DRV_SSC_TX_FS_OUTPUT_SEL;

// *****************************************************************************
/* SSC Module Transmitter Frame Sync Data Enable

  Summary:
    Identifies the Transmit Frame Sync Data options available for transmitter in
    SSC Module.

  Description:
    This enumeration identifies whether default data to be shifted out when 
    data is not available or Transmit SYNC holding register value should be 
    shifted out.

  Remarks:
    None.
*/

typedef enum
{
    /* The TD line is driver with deafult(DATDEF) value during Transmit Frame
     * Sync signal */
    I2S_SSC_TX_OUT_DEF_VALUE = 0,
    
    /* Transmit Sync holding register value is shifted out during the 
     * Transmit frame signal */
    I2S_SSC_TX_OUT_TX_SYNC_HOLD_REG,
                       
} DRV_SSC_TX_DATAEN;

// *****************************************************************************
/* SSC Module Transmitter Frame Sync Edge detection

  Summary:
    Identifies which edge of frame sync will generate TXSYN interrupt.

  Description:
    This enumeration identifies which edge of transmitter's frame sync signal 
    will generate TXSYN interrupt in SSC status register.

  Remarks:
    None.
*/

typedef enum
{    
    /* Postive edge of TF signal will generate TXSYN interrupt */
    I2S_SSC_TX_FS_EDGE_POSITIVE = 0,
    
    /* Negative edge of TF signal will generate TXSYN interrupt */
    I2S_SSC_TX_FS_EDGE_NEGATIVE,
                        
} DRV_SSC_TX_FS_EDGE_DETECTION;

// *****************************************************************************
/* I2S Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the I2S driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    I2S driver. If the driver is built statically, some members of this data
    structure are statically over-ridden by static overrides in the
    system_config.h file.

  Remarks:
    None.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT moduleInit;

    /* Identifies I2S hardware module ID */
    SSC_MODULE_ID sscID;
    
    /* Divided clock input for Transmitter and Receiver Blocks */
    uint16_t clockDivider;
    
    /* Identifies the Receiver clock selection options for SSC Module */
    DRV_I2S_SSC_RX_CLK_SEL rxClockSel;
    
    /* Identifies the Receiver Clock output mode selection */
    DRV_I2S_SSC_RX_CLK_OUTPUT_MODE_SEL rxClockOutputMode;
    
    /* Identifies clock inversion option available for SSC Module */
    DRV_SSC_RX_CLOCK_INVERSION rxClockInverse;
    
    /* Identifies Receiver clock gating options available for SSC Module */
    DRV_I2S_SSC_RX_CLOCK_GATING_SEL rxClockGating;
    
    /* Identifies Receive start sleection options available for SSC Module */
    DRV_SSC_RX_START_SEL rxStartSel;
    
    /* Identifies stop conditions available for data reception in SSC Module 
     * when receive start is triggered by Compare 0 register */
    DRV_SSC_RX_STOP rxStop;
    
    /* Delay to be inserted between start event and actual data reception */
    uint8_t rxStartDelay;
    
    /* Divider selection to be applied to generate frame sync signal */
    uint8_t rxPeriodDividerSel;

    /* Size of data word to be transmitted */
    uint8_t rxDataLen;
    
    /* Identifies receiver should operate in normal or loop mode */
    DRV_SSC_LOOP_MODE loopMode;

    /* Identifies sampling order of SSC module receiver */
    DRV_SSC_RX_MSBF rxMSBFirst;
    
    /* Number of Data Words to be transferred at each transfer start */
    uint8_t rxDataNumberperFrame;

    /* Field defines length of Frame sync Signal */
    uint8_t rxFrameSyncLen;
    
    /* Identifies options available for receiver's frame sync signal of 
     * SSC module */
    DRV_SSC_RX_FS_OUTPUT_SEL rxFSOutSel;

    /* Identifies which edge of frame sync will generate RXSYN interrupt */
    DRV_SSC_RX_FS_EDGE_DETECTION rxFSEdgeDetection;

    /* Extension for FSLEN field */
    uint8_t rxFSLenExtn;
    
    /* Identifies the transmitter clock selection options for SSC Module */
    DRV_I2S_SSC_TX_CLK_SEL txClockSel;

    /* Identifies the Transmitter Clock output mode selection */
    DRV_I2S_SSC_TX_CLK_OUTPUT_MODE_SEL txClockOutputMode;

    /* Identifies clock inversion option available for SSC Module */
    DRV_SSC_TX_CLOCK_INVERSION txClockInverse;

    /* Identifies Transmitter clock gating options available for SSC Module */
    DRV_I2S_SSC_TX_CLOCK_GATING_SEL txClockGating;

    /* Identifies Transmit start sleection options available for SSC Module */
    DRV_SSC_TX_START_SEL txStartSel;

    /* Delay to be inserted between start event and actual data transmission */
    uint8_t txStartDelay;
    
    /* Divider selection to be applied to generate frame sync signal */
    uint8_t txPeriodDividerSel;

    /* Size of data word to be transmitted */
    uint8_t txDataLen;

    /* Identifies the default data to be driven on TD pin while out of 
     * transmission */
    DRV_SSC_DATA_DEFAULT txDefaultData;

    /* Identifies sampling order of SSC module transmitter */
    DRV_SSC_TX_MSBF txMSBFirst;

    /* Number of Data Words to be transferred at each transfer start */
    uint8_t txDataNumberperFrame;

    /* Field defines length of Frame sync Signal */
    uint8_t txFrameSyncLen;
    
    /* Identifies options available for transmitter's frame sync output 
     * signal of SSC module */
    DRV_SSC_TX_FS_OUTPUT_SEL txFSOutSel;
    
    /* Identifies the Transmit Frame Sync Data options available for 
     * transmitter in SSC Module */
    DRV_SSC_TX_DATAEN txFrameSyncDataEnable;

    /* Identifies which edge of frame sync will generate TXSYN interrupt */
    DRV_SSC_TX_FS_EDGE_DETECTION txFSEdgeDetection;
    
    /* Extension for FSLEN field */
    uint8_t txFSLenExtn;
    
    /* SSC Interrupt source */
    IRQn_Type interruptSSC;
    
    /* This is the transmit buffer queue size. This is the maximum
       number of write requests that driver will queue. */
    uint32_t queueSizeTransmit;

    /* This is the receive buffer queue size. This is the maximum
       number of read requests that driver will queue. */
    uint32_t queueSizeReceive;

    /* DMA Channel for SSC Transmit operation */
    DMA_CHANNEL dmaChannelSSCTransmit;
    
    /* SSC Transmit trigger source for DMA */
    DMA_TRIGGER_SOURCE dmaChannelTransmitTrigger;
    
    /* DMA Channel for SSC Receive operation */
    DMA_CHANNEL dmaChannelSSCReceive;
    
    /* SSC Reception trigger source for DMA */
    DMA_TRIGGER_SOURCE dmaChannelReceiveTrigger;

    /* DMA Interrupt Source */
    INT_SOURCE interruptDMA;

} DRV_I2S_SSC_INIT;

#endif /* DRV_I2S__SSC_DEFINITIONS_PIC32C_H_ */