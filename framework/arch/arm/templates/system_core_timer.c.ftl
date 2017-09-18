/*******************************************************************************
  Systick System Initialization File

  File Name:
    system_core_timer.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    Initializes the system core timer (ARM Cortex-M Systick) according to the
    selected configuration.
 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#include "system_definitions.h"

/******************************************************************************
  Function:
    ARCH_CORE_TIMER_Initialize(void)

  Summary:
    Initializes Core Timer Service (ARM Cortex-M Systick)

  Description:
    This function initializes the system core timer (ARM Cortex-M Systick)
    according to the selected configuration.

  Remarks:
    None.
*/
void ARCH_CORE_TIMER_Initialize(void)
{
<#if CONFIG_PROJECT_USES_CORE_TIMER == true>
  <#if CONFIG_ARM_CORTEX_M_SYSTICK_USE_INTERRUPT == true>
  SYS_INT_VectorPrioritySet(SysTick_IRQn, ${CONFIG_ARM_CORTEX_M_SYSTICK_INTERRUPT_PRIORITY});

  </#if>
  /* Set Systick, common to all Cortex-M variants, to given interval */
  if (SysTick_Config( SYS_CLK_SystemFrequencyGet() / ${CONFIG_ARM_CORTEX_M_SYSTICK_PERIOD} ) != 0)
  {
    /* Capture error */
    while ( 1 ) ;
  }
</#if>
}
