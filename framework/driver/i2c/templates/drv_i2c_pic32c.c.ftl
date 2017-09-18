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
// <editor-fold defaultstate="collapsed" desc="DRV_I2C Initialization Data">
<#if CONFIG_DRV_I2C_DRIVER_MODE == "DYNAMIC">
// *****************************************************************************
/* I2C Driver Initialization Data
*/
<#if CONFIG_DRV_I2C_INST_IDX0 == true> <#-- Instance 0 -->
const DRV_I2C_INIT drvI2C0InitData =
{
    .i2cId              = DRV_I2C_PERIPHERAL_ID_IDX0,
    .i2cMode            = DRV_I2C_OPERATION_MODE_IDX0,
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX0 == true>
    .baudRate           = DRV_I2C_BAUD_RATE_IDX0,
</#if>
<#if CONFIG_DRV_I2C_SLAVE_MODE_IDX0 == true>
    .slaveaddvalue      = DRV_I2C_SLAVE_ADDRESS_VALUE_IDX0,
    .maskslaveaddress   = DRV_I2C_SLAVE_ADDRESS_MASK_IDX0,
    .highSpeedMode      = DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX0,
    .clockStretchEnable = DRV_I2C_CLOCK_STRETCH_IDX0,
</#if>
<#if CONFIG_USE_SYS_INT == true && CONFIG_DRV_I2C_INTERRUPT_MODE == true>
    .interruptSource    = DRV_I2C_INT_SRC_IDX0,
</#if>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX0 == true>
    .queueSize          = DRV_I2C_QUEUE_SIZE_IDX0,
</#if>
};
</#if> <#-- Instance 0 -->
<#if CONFIG_DRV_I2C_INST_IDX1 == true> <#-- Instance 1 -->
const DRV_I2C_INIT drvI2C1InitData =
{
    .i2cId              = DRV_I2C_PERIPHERAL_ID_IDX1,
    .i2cMode            = DRV_I2C_OPERATION_MODE_IDX1,
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX1 == true>
    .baudRate           = DRV_I2C_BAUD_RATE_IDX1,
</#if>
<#if CONFIG_DRV_I2C_SLAVE_MODE_IDX1 == true>
    .slaveaddvalue      = DRV_I2C_SLAVE_ADDRESS_VALUE_IDX1,
    .maskslaveaddress   = DRV_I2C_SLAVE_ADDRESS_MASK_IDX1,
    .highSpeedMode      = DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX1,
    .clockStretchEnable = DRV_I2C_CLOCK_STRETCH_IDX1,
</#if>
<#if CONFIG_USE_SYS_INT == true && CONFIG_DRV_I2C_INTERRUPT_MODE == true>
    .interruptSource    = DRV_I2C_INT_SRC_IDX1,
</#if>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX1 == true>
    .queueSize          = DRV_I2C_QUEUE_SIZE_IDX1,
</#if>
};
</#if> <#-- Instance 1 -->
<#if CONFIG_DRV_I2C_INST_IDX2 == true> <#-- Instance 2 -->
const DRV_I2C_INIT drvI2C2InitData =
{
    .i2cId              = DRV_I2C_PERIPHERAL_ID_IDX2,
    .i2cMode            = DRV_I2C_OPERATION_MODE_IDX2,
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX2 == true>
    .baudRate           = DRV_I2C_BAUD_RATE_IDX2,
</#if>
<#if CONFIG_DRV_I2C_SLAVE_MODE_IDX2 == true>
    .slaveaddvalue      = DRV_I2C_SLAVE_ADDRESS_VALUE_IDX2,
    .maskslaveaddress   = DRV_I2C_SLAVE_ADDRESS_MASK_IDX2,
    .highSpeedMode      = DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX2,
    .clockStretchEnable = DRV_I2C_CLOCK_STRETCH_IDX2,
</#if>
<#if CONFIG_USE_SYS_INT == true && CONFIG_DRV_I2C_INTERRUPT_MODE == true>
    .interruptSource    = DRV_I2C_INT_SRC_IDX2,
</#if>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX2 == true>
    .queueSize          = DRV_I2C_QUEUE_SIZE_IDX2,
</#if>
};
</#if> <#-- Instance 2 -->
</#if> <#-- CONFIG_DRV_I2C_DRIVER_MODE == "DYNAMIC" -->
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
