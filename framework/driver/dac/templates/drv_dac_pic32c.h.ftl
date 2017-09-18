<#--
/*******************************************************************************
  DAC Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_dac_pic32c.h.ftl

  Summary:
    DAC Driver Freemarker Template File

  Description:

*******************************************************************************/

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
-->

<#if CONFIG_USE_DRV_DAC == true>
// *****************************************************************************
/* DAC Driver Configuration Options */
<#if CONFIG_DRV_DAC_INTERRUPT == true>
#define DRV_DAC_INTERRUPT_MODE                   true
<#else>
#define DRV_DAC_INTERRUPT_MODE                   false
</#if>
#define DRV_DAC_PRESCALER_VALUE                  ${CONFIG_DRV_DAC_PRESACLER_VALUE}
#define DRV_DAC_OUTPUT_TYPE                      DRV_DAC_OUTPUT_MODE_${CONFIG_DRV_DAC_DRIVER_MODE}
<#if CONFIG_DRV_DAC_DRIVER_MODE == "SINGLE_ENDED">
#define DRV_DAC_CHANNEL_INSTANCES_NUMBER         ${CONFIG_DRV_DAC_CHANNEL_NUMBER}
</#if>
<#if CONFIG_DRV_DAC_DRIVER_MODE == "DIFFERENTIAL">
#define DRV_DAC_CHANNEL_INSTANCES_NUMBER         1
</#if>

<#-- If Channel 0 is Enabled -->
/* DAC Driver Channel 0 Configuration Options */
<#if (CONFIG_DRV_DAC_INST_IDX0 == true)>
#define DRV_DAC_INST_IDX0                        DRV_DAC_CHANNEL_INDEX_0
<#if CONFIG_DRV_DAC_MODE_OPTIONS_IDX0?has_content>
#define DRV_DAC_MODE_OPTIONS_IDX0                DRV_DAC_OPERATION_${CONFIG_DRV_DAC_MODE_OPTIONS_IDX0}
</#if>
<#if CONFIG_DRV_DAC_SPEED_MODE1_IDX0?has_content>
#define DRV_DAC_SPEED_MODE_IDX0                  DRV_DAC_SPEED_${CONFIG_DRV_DAC_SPEED_MODE1_IDX0}
</#if>
<#if CONFIG_DRV_DAC_SPEED_MODE2_IDX0?has_content>
#define DRV_DAC_SPEED_MODE_IDX0                  DRV_DAC_SPEED_${CONFIG_DRV_DAC_SPEED_MODE2_IDX0}
</#if>
<#if CONFIG_DRV_DAC_SPEED_MODE3_IDX0?has_content>
#define DRV_DAC_SPEED_MODE_IDX0                  DRV_DAC_SPEED_${CONFIG_DRV_DAC_SPEED_MODE3_IDX0}
</#if>
<#if CONFIG_DRV_DAC_TRIGGER_EVENT_SELECT_IDX0?has_content>
#define DRV_DAC_TRIGGER_EVENT_SELECT_IDX0        DRV_DAC_TRIGGER_SOURCE_${CONFIG_DRV_DAC_TRIGGER_EVENT_SELECT_IDX0}
</#if>
<#if CONFIG_DRV_DAC_INTERPOLATE_SELECT_IDX0?has_content>
#define DRV_DAC_INTERPOLATE_SELECT_IDX0          DRV_DAC_${CONFIG_DRV_DAC_INTERPOLATE_SELECT_IDX0}
</#if>
</#if>

<#-- If Channel 1 is Enabled -->
<#if (CONFIG_DRV_DAC_INST_IDX1 == true)>
/* DAC Driver Channel 1 Configuration Options */
#define DRV_DAC_INST_IDX1                        DRV_DAC_CHANNEL_INDEX_1
<#if CONFIG_DRV_DAC_MODE_OPTIONS_IDX1?has_content>
#define DRV_DAC_MODE_OPTIONS_IDX1                DRV_DAC_OPERATION_${CONFIG_DRV_DAC_MODE_OPTIONS_IDX1}
</#if>
<#if CONFIG_DRV_DAC_SPEED_MODE1_IDX1?has_content>
#define DRV_DAC_SPEED_MODE_IDX1                  DRV_DAC_SPEED_${CONFIG_DRV_DAC_SPEED_MODE1_IDX1}
</#if>
<#if CONFIG_DRV_DAC_SPEED_MODE2_IDX1?has_content>
#define DRV_DAC_SPEED_MODE_IDX1                  DRV_DAC_SPEED_${CONFIG_DRV_DAC_SPEED_MODE2_IDX1}
</#if>
<#if CONFIG_DRV_DAC_SPEED_MODE3_IDX1?has_content>
#define DRV_DAC_SPEED_MODE_IDX1                  DRV_DAC_SPEED_${CONFIG_DRV_DAC_SPEED_MODE3_IDX1}
</#if>
<#if CONFIG_DRV_DAC_TRIGGER_EVENT_SELECT_IDX1?has_content>
#define DRV_DAC_TRIGGER_EVENT_SELECT_IDX1        DRV_DAC_TRIGGER_SOURCE_${CONFIG_DRV_DAC_TRIGGER_EVENT_SELECT_IDX1}
</#if>
<#if CONFIG_DRV_DAC_INTERPOLATE_SELECT_IDX1?has_content>
#define DRV_DAC_INTERPOLATE_SELECT_IDX1          DRV_DAC_${CONFIG_DRV_DAC_INTERPOLATE_SELECT_IDX1}
</#if>
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->

