<#--
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
// <editor-fold defaultstate="collapsed" desc="DRV_DAC Initialization Data">
<#if CONFIG_USE_DRV_DAC == true>
const DRV_DAC_INIT drvDACInitData[] =
{
<#if (CONFIG_DRV_DAC_INST_IDX0 == true)>
	/*Channel 0 Initialization data*/
	{	
		.dacID = DACC_ID_0,
		.dacIndex = DRV_DAC_INST_IDX0, 
		.outputMode = DRV_DAC_OUTPUT_TYPE,
		.operationMode = DRV_DAC_MODE_OPTIONS_IDX0,
		.dacSpeed = DRV_DAC_SPEED_MODE_IDX0,
        <#if CONFIG_DRV_DAC_TRIGGER_EVENT_SELECT_IDX0?has_content>
		.triggerSource = DRV_DAC_TRIGGER_EVENT_SELECT_IDX0,
        </#if>
        <#if CONFIG_DRV_DAC_INTERPOLATE_SELECT_IDX0?has_content>
		.overSampleRatio = DRV_DAC_INTERPOLATE_SELECT_IDX0,
        </#if>
	    .interruptMode = DRV_DAC_INTERRUPT_MODE
	},
</#if>	
<#if (CONFIG_DRV_DAC_INST_IDX1 == true)>

    /*Channel 1 Initialization data*/
	{	
		.dacID = DACC_ID_0,
		.dacIndex = DRV_DAC_INST_IDX1, 
		.outputMode = DRV_DAC_OUTPUT_TYPE,
		.operationMode = DRV_DAC_MODE_OPTIONS_IDX1,
		.dacSpeed = DRV_DAC_SPEED_MODE_IDX1,
        <#if CONFIG_DRV_DAC_TRIGGER_EVENT_SELECT_IDX1?has_content>
		.triggerSource = DRV_DAC_TRIGGER_EVENT_SELECT_IDX1,
        </#if>
        <#if CONFIG_DRV_DAC_INTERPOLATE_SELECT_IDX1?has_content>
		.overSampleRatio = DRV_DAC_INTERPOLATE_SELECT_IDX1,
        </#if>
	    .interruptMode = DRV_DAC_INTERRUPT_MODE
	}
</#if>	
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->

