<#--
/*******************************************************************************
  QSPI Driver Interrupt Handler Template File

  File Name:
    drv_qspi_int.c.ftl

  Summary:
    This file contains source code for interrupt handler.

  Description:
    This file contains source code necessary to initialize the system. It implements
    interrupt calls necessary for QSPI driver in the application.
 *******************************************************************************/

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
<#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_QSPI_ISR_VECTOR}, ipl${CONFIG_DRV_QSPI_INT_IPL}SOFT) _${CONFIG_DRV_QSPI_INTERRUPT_HANDLER_NAME} (void)
  <#else>
    <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void ${CONFIG_DRV_QSPI_INTERRUPT_HANDLER_NAME}_ISR( void );
    </#if>
void ${CONFIG_DRV_QSPI_INTERRUPT_HANDLER_NAME} (void)
  </#if>
<#else>
void ${CONFIG_DRV_QSPI_INTERRUPT_HANDLER_NAME} (void)
</#if>
{
<#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
   /* Call ThreadX context save.  */
   _tx_thread_context_save();
  </#if>
  <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
  </#if>
  </#if>
    DRV_SQI_Tasks(sysObj.drvSqi);
  <#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
   /* Call ThreadX context restore.  */
   _tx_thread_context_restore();
  </#if>
  <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
  </#if>
</#if>
}
<#--
/*******************************************************************************
 End of File
*/
-->