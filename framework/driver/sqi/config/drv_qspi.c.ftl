<#--
/*******************************************************************************
Copyright (c) 2016-2017 released Microchip Technology Inc.  All rights reserved.

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
// <editor-fold defaultstate="collapsed" desc="DRV_QSPI Initialization Data">
/*** QSPI Driver Initialization Data ***/
<#if CONFIG_USE_DRV_SQI == true>
<#assign SCBR_Val = ((CONFIG_SYS_CLK_MASTERCLK_FREQ?number / CONFIG_DRV_QSPI_CLK_FREQ?number) -1)?floor >

const DRV_SQI_INIT drvSqiInit =
{
<#if CONFIG_DRV_QSPI_PERIPHERAL_ID?has_content>
    .sqiId = CAST(qspi_registers_t,${CONFIG_DRV_QSPI_PERIPHERAL_ID}),
</#if>
<#if CONFIG_DRV_QSPI_INTERRUPT_SOURCE?has_content>
    .interruptSource = ${CONFIG_DRV_QSPI_INTERRUPT_SOURCE},
</#if>
<#if SCBR_Val < 256 >
    .clockDivider = ${SCBR_Val},
<#else>
    .clockDivider = 255,
</#if>
	.spiMode = ${CONFIG_DRV_QSPI_SPI_OP_MODE}
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
