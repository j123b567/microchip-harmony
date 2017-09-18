<#--
/*******************************************************************************
  AFEC Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_afec.h.ftl

  Summary:
    AFEC Driver Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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
-->
<#if CONFIG_USE_DRV_AFEC == true>

#define DRV_AFEC_INSTANCES_NUMBER              ${CONFIG_DRV_AFEC_INSTANCE_NUMBER}
#define DRV_AFEC_CLIENTS_NUMBER                ${CONFIG_DRV_AFEC_CLIENTS_NUMBER}
<#if CONFIG_DRV_AFEC0_CHANNEL_SET_NUMBER?has_content>
#define DRV_AFEC0_CHANNEL_SET_NUMBER           ${CONFIG_DRV_AFEC0_CHANNEL_SET_NUMBER}
<#else>
#define DRV_AFEC0_CHANNEL_SET_NUMBER           0
</#if>
<#if CONFIG_DRV_AFEC1_CHANNEL_SET_NUMBER?has_content>
#define DRV_AFEC1_CHANNEL_SET_NUMBER           ${CONFIG_DRV_AFEC1_CHANNEL_SET_NUMBER}
<#else>
#define DRV_AFEC1_CHANNEL_SET_NUMBER           0
</#if>

/* Channelset definition */
<#if CONFIG_DRV_AFEC0_CHANNELSET_IDX0?has_content>
#define DRV_AFEC0_CHANNELSET_IDX0               ${CONFIG_DRV_AFEC0_CHANNELSET_IDX0}
</#if>
<#if CONFIG_DRV_AFEC0_CHANNELSET_IDX1?has_content>
#define DRV_AFEC0_CHANNELSET_IDX1               ${CONFIG_DRV_AFEC0_CHANNELSET_IDX1}
</#if>
<#if CONFIG_DRV_AFEC0_CHANNELSET_IDX2?has_content>
#define DRV_AFEC0_CHANNELSET_IDX2               ${CONFIG_DRV_AFEC0_CHANNELSET_IDX2}
</#if>
<#if CONFIG_DRV_AFEC0_CHANNELSET_IDX3?has_content>
#define DRV_AFEC0_CHANNELSET_IDX3               ${CONFIG_DRV_AFEC0_CHANNELSET_IDX3}
</#if>

<#if CONFIG_DRV_AFEC1_CHANNELSET_IDX0?has_content>
#define DRV_AFEC1_CHANNELSET_IDX0               ${CONFIG_DRV_AFEC1_CHANNELSET_IDX0}
</#if>
<#if CONFIG_DRV_AFEC1_CHANNELSET_IDX1?has_content>
#define DRV_AFEC1_CHANNELSET_IDX1               ${CONFIG_DRV_AFEC1_CHANNELSET_IDX1}
</#if>
<#if CONFIG_DRV_AFEC1_CHANNELSET_IDX2?has_content>
#define DRV_AFEC1_CHANNELSET_IDX2               ${CONFIG_DRV_AFEC1_CHANNELSET_IDX2}
</#if>
<#if CONFIG_DRV_AFEC1_CHANNELSET_IDX3?has_content>
#define DRV_AFEC1_CHANNELSET_IDX3               ${CONFIG_DRV_AFEC1_CHANNELSET_IDX3}
</#if>

<#if CONFIG_DRV_AFEC_INST_IDX0 == true>
/*** AFEC Driver Instance 0 Configuration ***/
#define DRV_AFEC_PERIPHERAL_ID_IDX0            ${CONFIG_DRV_AFEC_MODULE_ID0}
#define DRV_AFEC_STARTUP_TIME0                 ${CONFIG_DRV_AFEC_STARTUP_TIME_VALUE0}
#define DRV_AFEC_CLOCK_DIV0                    ${CONFIG_DRV_AFEC_CLOCK_DIV0}
#define DRV_AFEC_DUAL_CHANNELS0                ${CONFIG_DRV_AFEC_DUAL_CHANNELS0}
#define DRV_AFEC_DIFFERENTIAL_CHANNELS0        ${CONFIG_DRV_AFEC_DIFFERENTIAL_CHANNELS0}
#define DRV_AFEC_BIAS_CONTROL0                 ${CONFIG_DRV_AFEC_BIAS_CONTROL_VALUE0}
#define DRV_AFEC_INTERRUPT_SOURCE0             ${CONFIG_DRV_AFEC_INTERRUPT_SOURCE0}
#define DRV_AFEC_RESOLUTION0                   ${CONFIG_DRV_AFEC_RESOLUTION_VALUE0}
#define DRV_AFEC_SIGN_MODE0                    ${CONFIG_DRV_AFEC_SIGN_MODE_VALUE0}
</#if>

<#if CONFIG_DRV_AFEC_INST_IDX1 == true>
/*** AFEC Driver Instance 1 Configuration ***/
#define DRV_AFEC_PERIPHERAL_ID_IDX1            ${CONFIG_DRV_AFEC_MODULE_ID1}
#define DRV_AFEC_STARTUP_TIME1                 ${CONFIG_DRV_AFEC_STARTUP_TIME_VALUE1}
#define DRV_AFEC_CLOCK_DIV1                    ${CONFIG_DRV_AFEC_CLOCK_DIV1}
#define DRV_AFEC_DUAL_CHANNELS1                ${CONFIG_DRV_AFEC_DUAL_CHANNELS1}
#define DRV_AFEC_DIFFERENTIAL_CHANNELS1        ${CONFIG_DRV_AFEC_DIFFERENTIAL_CHANNELS1}
#define DRV_AFEC_BIAS_CONTROL1                 ${CONFIG_DRV_AFEC_BIAS_CONTROL_VALUE1}
#define DRV_AFEC_INTERRUPT_SOURCE1             ${CONFIG_DRV_AFEC_INTERRUPT_SOURCE1}
#define DRV_AFEC_RESOLUTION1                   ${CONFIG_DRV_AFEC_RESOLUTION_VALUE1}
#define DRV_AFEC_SIGN_MODE1                    ${CONFIG_DRV_AFEC_SIGN_MODE_VALUE1}
</#if>

</#if> <#--CONFIG_USE_DRV_AFEC == true-->
<#--
/*******************************************************************************
 End of File
*/
-->

