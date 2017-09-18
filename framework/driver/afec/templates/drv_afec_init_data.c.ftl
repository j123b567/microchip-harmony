<#--
/*******************************************************************************
  AFEC Driver Initialization File

  File Name:
    drv_afec_init.c.ftl

  Summary:
    This file contains source code necessary to initialize the AFEC driver.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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
<#if CONFIG_USE_DRV_AFEC == true>
#include "driver/afec/drv_afec.h"

<#if CONFIG_DRV_AFEC_INST_IDX0 == true>
/* Channelset data for AFEC driver instance 0 */
DRV_AFEC_CHANNEL_SET gAFEC0ChannelSet[DRV_AFEC0_CHANNEL_SET_NUMBER] = {
    <#if CONFIG_DRV_AFEC0_CHANNELSET_IDX0?has_content>
    DRV_AFEC0_CHANNELSET_IDX0,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNELSET_IDX1?has_content>
    DRV_AFEC0_CHANNELSET_IDX1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNELSET_IDX2?has_content>
    DRV_AFEC0_CHANNELSET_IDX2,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNELSET_IDX3?has_content>
    DRV_AFEC0_CHANNELSET_IDX3
    </#if>
    };

/* Offset data for channels of instance 0 */
uint16_t gAFEC0ChannelOffset[DRV_AFEC_NUMBER_OF_CHANNELS] = {
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET0?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET0},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET1?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET1},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET2?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET2},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET3?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET3},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET4?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET4},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET5?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET5},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET6?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET6},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET7?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET7},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET8?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET8},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET9?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET9},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET10?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET10},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_OFFSET11?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_OFFSET11},
    <#else>
    512,
    </#if>
};

/* Gain data for channels of instance 0 */
DRV_AFEC_CHANNEL_GAIN gAFEC0ChannelGain[DRV_AFEC_NUMBER_OF_CHANNELS] = {
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN0?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN0},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN1?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN1},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN2?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN2},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN3?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN3},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN4?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN4},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN5?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN5},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN6?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN6},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN7?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN7},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN8?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN8},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN9?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN9},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN10?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN10},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC0_CHANNEL_GAIN11?has_content>
    ${CONFIG_DRV_AFEC0_CHANNEL_GAIN11},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
};

/* AFEC Instance 0 Configuration */
const DRV_AFEC_INIT drvAfec0InitData =
{
    .afecID = DRV_AFEC_PERIPHERAL_ID_IDX0,
    .clockPrescaler = DRV_AFEC_CLOCK_DIV0,
    .dualSamplingChannels = DRV_AFEC_DUAL_CHANNELS0,
    .differentialModeChannels = DRV_AFEC_DIFFERENTIAL_CHANNELS0,
    .biasCurrent = DRV_AFEC_BIAS_CONTROL0,
    .interruptSource = DRV_AFEC_INTERRUPT_SOURCE0,
    .channelSetTable = gAFEC0ChannelSet,
    .startupTime = DRV_AFEC_STARTUP_TIME0,
    .channelSetTableSize = DRV_AFEC0_CHANNEL_SET_NUMBER,
    .resolution = DRV_AFEC_RESOLUTION0,
    .signMode = DRV_AFEC_SIGN_MODE0,
    .channelOffset = gAFEC0ChannelOffset,
    .channelGain = gAFEC0ChannelGain
};

</#if>

<#if CONFIG_DRV_AFEC_INST_IDX1 == true>
/* Channelset data for AFEC driver instance 1 */
DRV_AFEC_CHANNEL_SET gAFEC1ChannelSet[DRV_AFEC1_CHANNEL_SET_NUMBER] = {
    <#if CONFIG_DRV_AFEC1_CHANNELSET_IDX0?has_content>
    DRV_AFEC1_CHANNELSET_IDX0,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNELSET_IDX1?has_content>
    DRV_AFEC1_CHANNELSET_IDX1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNELSET_IDX2?has_content>
    DRV_AFEC1_CHANNELSET_IDX2,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNELSET_IDX3?has_content>
    DRV_AFEC1_CHANNELSET_IDX3
    </#if>
    };

/* Offset data for channels of instance 1 */
uint16_t gAFEC1ChannelOffset[DRV_AFEC_NUMBER_OF_CHANNELS] = {
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET0?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET0},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET1?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET1},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET2?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET2},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET3?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET3},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET4?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET4},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET5?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET5},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET6?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET6},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET7?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET7},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET8?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET8},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET9?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET9},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET10?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET10},
    <#else>
    512,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_OFFSET11?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_OFFSET11},
    <#else>
    512,
    </#if>
};

/* Gain data for channels of instance 1 */
DRV_AFEC_CHANNEL_GAIN gAFEC1ChannelGain[DRV_AFEC_NUMBER_OF_CHANNELS] = {
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN0?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN0},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN1?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN1},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN2?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN2},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN3?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN3},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN4?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN4},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN5?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN5},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN6?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN6},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN7?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN7},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN8?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN8},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN9?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN9},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN10?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN10},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
    <#if CONFIG_DRV_AFEC1_CHANNEL_GAIN11?has_content>
    ${CONFIG_DRV_AFEC1_CHANNEL_GAIN11},
    <#else>
    DRV_AFEC_CHANNEL_GAIN1,
    </#if>
};

/* AFEC Instance 1 Configuration */
const DRV_AFEC_INIT drvAfec1InitData =
{
    .afecID = DRV_AFEC_PERIPHERAL_ID_IDX1,
    .clockPrescaler = DRV_AFEC_CLOCK_DIV1,
    .dualSamplingChannels = DRV_AFEC_DUAL_CHANNELS1,
    .differentialModeChannels = DRV_AFEC_DIFFERENTIAL_CHANNELS1,
    .biasCurrent = DRV_AFEC_BIAS_CONTROL1,
    .interruptSource = DRV_AFEC_INTERRUPT_SOURCE1,
    .channelSetTable = gAFEC1ChannelSet,
    .startupTime = DRV_AFEC_STARTUP_TIME1,
    .channelSetTableSize = DRV_AFEC1_CHANNEL_SET_NUMBER,
    .resolution = DRV_AFEC_RESOLUTION1,
    .signMode = DRV_AFEC_SIGN_MODE1,
    .channelOffset = gAFEC1ChannelOffset,
    .channelGain = gAFEC1ChannelGain
};
</#if>

</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
