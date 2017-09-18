<#--
/*******************************************************************************
  I2C Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_pic32c.h.ftl

  Summary:
    I2C Driver Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
<#if CONFIG_USE_DRV_I2C == true>
<#assign DRV_I2C_QUEUE_DEPTH_COMBINED = 0>
// *****************************************************************************
/* I2C Driver Configuration Options
*/
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
#define DRV_I2C_INTERRUPT_MODE                    		true
<#else> <#-- CONFIG_DRV_I2C_INTERRUPT_MODE -->
#define DRV_I2C_INTERRUPT_MODE                    		false
</#if>  <#-- CONFIG_DRV_I2C_INTERRUPT_MODE -->
<#if CONFIG_DRV_I2C_DRIVER_MODE == "DYNAMIC">
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX0 == true ||
     CONFIG_DRV_I2C_MASTER_MODE_IDX1 == true ||
     CONFIG_DRV_I2C_MASTER_MODE_IDX2 == true >
#define DRV_I2C_MASTER_MODE_INSTANCE                    true
<#else>
#define DRV_I2C_MASTER_MODE_INSTANCE                    false
</#if>
<#if CONFIG_DRV_I2C_SLAVE_MODE_IDX0 == true ||
     CONFIG_DRV_I2C_SLAVE_MODE_IDX1 == true ||
     CONFIG_DRV_I2C_SLAVE_MODE_IDX2 == true>
#define DRV_I2C_SLAVE_MODE_INSTANCE                     true
<#else>
#define DRV_I2C_SLAVE_MODE_INSTANCE                     false
</#if>
</#if>
#define DRV_I2C_CLIENTS_NUMBER                    		${CONFIG_DRV_I2C_CLIENTS_NUMBER}
#define DRV_I2C_INSTANCES_NUMBER                  		${CONFIG_DRV_I2C_INSTANCES_NUMBER}
<#if CONFIG_DRV_I2C_DRIVER_MODE == "DYNAMIC">
<#if CONFIG_DRV_I2C_INST_IDX0 == true>
#define DRV_I2C_PERIPHERAL_ID_IDX0                		${CONFIG_DRV_I2C_PERIPHERAL_ID_IDX0}
#define DRV_I2C_OPERATION_MODE_IDX0               		${CONFIG_DRV_I2C_OPERATION_MODE_IDX0}
<#if CONFIG_USE_SYS_INT == true && CONFIG_DRV_I2C_INTERRUPT_MODE == true>
#define DRV_I2C_INT_SRC_IDX0               		        ${CONFIG_DRV_I2C_INT_SRC_IDX0}
</#if>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX0 == true>
#define DRV_I2C_BAUD_RATE_IDX0                    		${CONFIG_DRV_I2C_BAUD_RATE_IDX0}
#define DRV_I2C_QUEUE_SIZE_IDX0                         ${CONFIG_DRV_I2C_QUEUE_SIZE_IDX0}
</#if>
<#if CONFIG_DRV_I2C_SLAVE_MODE_IDX0 == true>
<#if CONFIG_DRV_I2C_CLOCK_STRETCH_IDX0 == true>
#define DRV_I2C_CLOCK_STRETCH_IDX0                      true
<#else>
#define DRV_I2C_CLOCK_STRETCH_IDX0                      false
</#if>
#define DRV_I2C_SLAVE_ADDRESS_MASK_IDX0                 ${CONFIG_DRV_I2C_SLAVE_ADDRESS_MASK_IDX0}
#define DRV_I2C_SLAVE_ADDRESS_VALUE_IDX0                ${CONFIG_DRV_I2C_SLAVE_ADDRESS_VALUE_IDX0}
<#if CONFIG_DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX0 == true>
#define DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX0              true
<#else>
#define DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX0              false
</#if>
</#if>
</#if> <#-- CONFIG_DRV_I2C_INST_IDX0 == true -->
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
#define DRV_I2C_PERIPHERAL_ID_IDX1                		${CONFIG_DRV_I2C_PERIPHERAL_ID_IDX1}
#define DRV_I2C_OPERATION_MODE_IDX1               		${CONFIG_DRV_I2C_OPERATION_MODE_IDX1}
<#if CONFIG_USE_SYS_INT == true && CONFIG_DRV_I2C_INTERRUPT_MODE == true>
#define DRV_I2C_INT_SRC_IDX1               		        ${CONFIG_DRV_I2C_INT_SRC_IDX1}
</#if>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX1 == true>
#define DRV_I2C_BAUD_RATE_IDX1                    		${CONFIG_DRV_I2C_BAUD_RATE_IDX1}
#define DRV_I2C_QUEUE_SIZE_IDX1                         ${CONFIG_DRV_I2C_QUEUE_SIZE_IDX1}
</#if>
<#if CONFIG_DRV_I2C_SLAVE_MODE_IDX1 == true>
<#if CONFIG_DRV_I2C_CLOCK_STRETCH_IDX1 == true>
#define DRV_I2C_CLOCK_STRETCH_IDX1                      true
<#else>
#define DRV_I2C_CLOCK_STRETCH_IDX1                      false
</#if>
#define DRV_I2C_SLAVE_ADDRESS_MASK_IDX1                 ${CONFIG_DRV_I2C_SLAVE_ADDRESS_MASK_IDX1}
#define DRV_I2C_SLAVE_ADDRESS_VALUE_IDX1                ${CONFIG_DRV_I2C_SLAVE_ADDRESS_VALUE_IDX1}
<#if CONFIG_DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX1 == true>
#define DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX1              true
<#else>
#define DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX1              false
</#if>
</#if>
</#if> <#-- CONFIG_DRV_I2C_INST_IDX1 == true -->
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
#define DRV_I2C_PERIPHERAL_ID_IDX2                		${CONFIG_DRV_I2C_PERIPHERAL_ID_IDX2}
#define DRV_I2C_OPERATION_MODE_IDX2               		${CONFIG_DRV_I2C_OPERATION_MODE_IDX2}
<#if CONFIG_USE_SYS_INT == true && CONFIG_DRV_I2C_INTERRUPT_MODE == true>
#define DRV_I2C_INT_SRC_IDX2               		        ${CONFIG_DRV_I2C_INT_SRC_IDX2}
</#if>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX2 == true>
#define DRV_I2C_BAUD_RATE_IDX2                    		${CONFIG_DRV_I2C_BAUD_RATE_IDX2}
#define DRV_I2C_QUEUE_SIZE_IDX2                         ${CONFIG_DRV_I2C_QUEUE_SIZE_IDX2}
</#if>
<#if CONFIG_DRV_I2C_SLAVE_MODE_IDX2 == true>
<#if CONFIG_DRV_I2C_CLOCK_STRETCH_IDX2 == true>
#define DRV_I2C_CLOCK_STRETCH_IDX2                      true
<#else>
#define DRV_I2C_CLOCK_STRETCH_IDX2                      false
</#if>
#define DRV_I2C_SLAVE_ADDRESS_MASK_IDX2                 ${CONFIG_DRV_I2C_SLAVE_ADDRESS_MASK_IDX2}
#define DRV_I2C_SLAVE_ADDRESS_VALUE_IDX2                ${CONFIG_DRV_I2C_SLAVE_ADDRESS_VALUE_IDX2}
<#if CONFIG_DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX2 == true>
#define DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX2              true
<#else>
#define DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX2              false
</#if>
</#if>
</#if> <#-- CONFIG_DRV_I2C_INST_IDX2 == true -->
</#if>
<#if CONFIG_DRV_I2C_INST_IDX0 == true>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX0 == true>
<#assign DRV_I2C_QUEUE_DEPTH_COMBINED = DRV_I2C_QUEUE_DEPTH_COMBINED + CONFIG_DRV_I2C_QUEUE_SIZE_IDX0?number>
</#if>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX1 == true>
<#assign DRV_I2C_QUEUE_DEPTH_COMBINED = DRV_I2C_QUEUE_DEPTH_COMBINED + CONFIG_DRV_I2C_QUEUE_SIZE_IDX1?number>
</#if>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
<#if CONFIG_DRV_I2C_MASTER_MODE_IDX2 == true>
<#assign DRV_I2C_QUEUE_DEPTH_COMBINED = DRV_I2C_QUEUE_DEPTH_COMBINED + CONFIG_DRV_I2C_QUEUE_SIZE_IDX2?number>
</#if>
</#if>
#define DRV_I2C_QUEUE_DEPTH_COMBINED                    ${DRV_I2C_QUEUE_DEPTH_COMBINED}
</#if> <#-- CONFIG_USE_DRV_I2C -->

<#--
/*******************************************************************************
 End of File
*/
-->
