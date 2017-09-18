<#--
/*******************************************************************************
  Systick Interrupt Handler Template File

  File Name:
    core_timer_int.c.ftl

  Summary:
    This file contains source code for ARM Cortex-M Systick interrupt handler.

  Description:
    This file implements interrupt handler for ARM Cortex-M Systick in the
    application. This handler would need to be updated by user for proper use.
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DICUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICRICHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PRICUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
 -->
<#if (CONFIG_ARM_CORTEX_M_SYSTICK_USE_INTERRUPT == true) && (CONFIG_ARM_CORTEX_M_SYSTICK_HANDLER_PROVIDED_BY_3RD_PARTY == false)>
void ${CONFIG_ARM_CORTEX_M_SYSTICK_HANDLER_NAME}(void)
{
  /* Fill me :-) */
}
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
