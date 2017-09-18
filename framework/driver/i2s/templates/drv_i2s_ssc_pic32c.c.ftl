<#--
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
 -->
 // <editor-fold defaultstate="collapsed" desc="DRV_I2S Initialization Data">
/*** I2S Driver Initialization Data ***/
<#-- Instance 0 -->
<#if CONFIG_DRV_I2S_INST_IDX0 == true>
DRV_I2S_SSC_INIT drvI2SSSC0InitData =
{
<#if CONFIG_DRV_I2S_PERIPHERAL_ID_IDX0?has_content>
    .sscID = DRV_I2S_PERIPHERAL_ID_IDX0, 
</#if>
<#if CONFIG_DRV_I2S_SSC_CLOCK_DIVIDER0?has_content>
    .clockDivider = DRV_I2S_SSC_CLOCK_DIVIDER0, 
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_CLOCK_SELECTION0?has_content>
    .rxClockSel = DRV_I2S_SSC_RX_CLOCK_SELECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_CLOCK_OUTPUT_MODE0?has_content>
    .rxClockOutputMode = DRV_I2S_SSC_RX_CLOCK_OUTPUT_MODE0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_CLOCK_INVERSE0?has_content>
    .rxClockInverse = DRV_I2S_SSC_RX_CLOCK_INVERSE0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_CLOCK_GATING0?has_content>
    .rxClockGating = DRV_I2S_SSC_RX_CLOCK_GATING0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_START_SELECTION0?has_content>
    .rxStartSel = DRV_I2S_SSC_RX_START_SELECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_LOOPMODE0?has_content>
    .loopMode = DRV_I2S_SSC_RX_LOOPMODE0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_START_DELAY0?has_content>
    .rxStartDelay = DRV_I2S_SSC_RX_START_DELAY0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_PERIOD_DIVIDER_SELECTION0?has_content>
    .rxPeriodDividerSel = DRV_I2S_SSC_RX_PERIOD_DIVIDER_SELECTION0,
</#if>

<#if CONFIG_DRV_I2S_SSC_RX_DATA_LENGTH0?has_content>
    .rxDataLen = DRV_I2S_SSC_RX_DATA_LENGTH0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_MSBFIRST0?has_content>
    .rxMSBFirst = DRV_I2S_SSC_RX_MSBFIRST0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_FS_LENGTH0?has_content>
    .rxFrameSyncLen = DRV_I2S_SSC_RX_FS_LENGTH0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_DATA_NUM_PER_FRAME0?has_content>
    .rxDataNumberperFrame = DRV_I2S_SSC_RX_DATA_NUM_PER_FRAME0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_FS_OUTPUT_SELECTION0?has_content>
    .rxFSOutSel = DRV_I2S_SSC_RX_FS_OUTPUT_SELECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_RX_FS_EDGEDETECTION0?has_content>
    .rxFSEdgeDetection = DRV_I2S_SSC_RX_FS_EDGEDETECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_CLOCK_SELECTION0?has_content>
    .txClockSel = DRV_I2S_SSC_TX_CLOCK_SELECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_CLOCK_OUTPUT_MODE0?has_content>
    .txClockOutputMode = DRV_I2S_SSC_TX_CLOCK_OUTPUT_MODE0,
</#if>

<#if CONFIG_DRV_I2S_SSC_TX_CLOCK_INVERSE0?has_content>
    .txClockInverse = DRV_I2S_SSC_TX_CLOCK_INVERSE0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_CLOCK_GATING0?has_content>
    .txClockGating = DRV_I2S_SSC_TX_CLOCK_GATING0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_START_SELECTION0?has_content>
    .txStartSel = DRV_I2S_SSC_TX_START_SELECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_START_DELAY0?has_content>
    .txStartDelay = DRV_I2S_SSC_TX_START_DELAY0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_PERIOD_DIVIDER_SELECTION0?has_content>
    .txPeriodDividerSel = DRV_I2S_SSC_TX_PERIOD_DIVIDER_SELECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_DATA_LENGTH0?has_content>
    .txDataLen = DRV_I2S_SSC_TX_DATA_LENGTH0,
</#if>

<#if CONFIG_DRV_I2S_SSC_TX_DEFAULTDATA0?has_content>
    .txDefaultData = DRV_I2S_SSC_TX_DEFAULTDATA0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_MSBFIRST0?has_content>
    .txMSBFirst = DRV_I2S_SSC_TX_MSBFIRST0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_DATA_NUM_PER_FRAME0?has_content>
    .txDataNumberperFrame = DRV_I2S_SSC_TX_DATA_NUM_PER_FRAME0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_FS_LENGTH0?has_content>
    .txFrameSyncLen = DRV_I2S_SSC_TX_FS_LENGTH0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_FS_OUTPUT_SELECTION0?has_content>
    .txFSOutSel = DRV_I2S_SSC_TX_FS_OUTPUT_SELECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_FS_DATAEN0?has_content>
    .txFrameSyncDataEnable = DRV_DRV_I2S_SSC_TX_FS_DATAEN0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_FS_EDGEDETECTION0?has_content>
    .txFSEdgeDetection = DRV_I2S_SSC_TX_FS_EDGEDETECTION0,
</#if>
<#if CONFIG_DRV_I2S_SSC_TX_FSLENGTH_EXTENSION?has_content>
    .txFSLenExtn = DRV_I2S_SSC_TX_FSLENGTH_EXTENSION0,
</#if>
 <#--   .interruptSSC = DRV_I2S_TX_INT_SRC_IDX0 -->
    
<#if CONFIG_QUEUE_SIZE_TX_IDX0?has_content>
    .queueSizeTransmit = QUEUE_SIZE_TX_IDX0,
</#if>
<#if CONFIG_QUEUE_SIZE_RX_IDX0?has_content>
    .queueSizeReceive = QUEUE_SIZE_RX_IDX0,
</#if>
    .dmaChannelSSCTransmit = DRV_I2S_TX_DMA_CHANNEL_IDX0,
    .dmaChannelTransmitTrigger = DRV_I2S_TX_DMA_TRIGGER, 
    .interruptDMA = DRV_I2S_TX_DMA_SOURCE_IDX0,    
    .dmaChannelSSCReceive = DRV_I2S_RX_DMA_CHANNEL_IDX0,
    .dmaChannelReceiveTrigger = DRV_I2S_RX_DMA_TRIGGER    
</#if>
};

DRV_I2S_INTERFACE drvI2SSCInterface = 
{
    .initialize = &DRV_I2S_SSC_Initialize,
    .open = &DRV_I2S_SSC_Open,
    .bufferAddWrite = &DRV_I2S_SSC_BufferAddWrite,
    .bufferAddRead = &DRV_I2S_SSC_BufferAddRead,
    .eventHandlerSet = &DRV_I2S_SSC_BufferEventHandlerSet,
    .bufferAddWriteRead = &DRV_I2S_SSC_BufferAddWriteRead
    
};

DRV_I2S_INIT i2sInit = 
{
      .init = &drvI2SSSC0InitData,
      .driverInterface = &drvI2SSCInterface
};

// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
