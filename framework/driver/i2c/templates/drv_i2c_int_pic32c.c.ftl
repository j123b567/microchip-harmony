<#--
/*******************************************************************************
  I2C Driver Interrupt Handler Template File

  File Name:
    drv_i2c_int.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
 *******************************************************************************/

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
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
  <#if CONFIG_DRV_I2C_INST_IDX0 == true> <#--- I2C0 --->    
void ${CONFIG_DRV_I2C_INT_HANDLER_IDX0} (void)
{
    DRV_I2C_Tasks(sysObj.drvI2C0);
}
  </#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true> <#--- I2C1 --->    
void ${CONFIG_DRV_I2C_INT_HANDLER_IDX1} (void)
{
    DRV_I2C_Tasks(sysObj.drvI2C1);
}
  </#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true> <#--- I2C2 --->    
void ${CONFIG_DRV_I2C_INT_HANDLER_IDX2} (void)
{
    DRV_I2C_Tasks(sysObj.drvI2C2);
}
  </#if>
</#if> <#-- CONFIG_DRV_I2C_INTERRUPT_MODE -->
<#--
/*******************************************************************************
 End of File
*/
-->
