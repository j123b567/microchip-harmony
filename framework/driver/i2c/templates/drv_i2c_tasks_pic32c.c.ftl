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

<#if CONFIG_DRV_I2C_INTERRUPT_MODE == false>
<#if CONFIG_DRV_I2C_INST_IDX0 == true>
    DRV_I2C_Tasks(sysObj.drvI2C0);
</#if> <#-- CONFIG_DRV_I2C_INST_IDX0 == true -->
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
    DRV_I2C_Tasks(sysObj.drvI2C1);
</#if> <#-- CONFIG_DRV_I2C_INST_IDX1 == true -->
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
    DRV_I2C_Tasks(sysObj.drvI2C2);
</#if> <#-- CONFIG_DRV_I2C_INST_IDX2 == true -->
</#if> <#-- CONFIG_DRV_I2C_INTERRUPT_MODE == false -->
<#--
/*******************************************************************************
 End of File
*/
-->
