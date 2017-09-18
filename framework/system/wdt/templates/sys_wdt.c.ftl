 /*******************************************************************************
  Watchdog Timer System Service Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    sys_wdt.c

  Summary:
    Watchdog Timer (WDT) System Service implementation.

  Description:
    The WDT System Service provides a simple interface to manage the
    Watchdog Timer module on Microchip microcontrollers.  This file implements
    the core interface routines for the WDT System Service. While building the
    system service from source, ALWAYS include this file in the build.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

<#if CONFIG_ARCH_ARM == true>
<#--
 ########  PIC32C Variant  ########
 -->
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system/wdt/sys_wdt.h"
#include "system/common/sys_common.h"
#include "system/int/sys_int.h"
#include "system/clk/sys_clk.h"
<#if (CONFIG_USE_SYS_WDT == true) || (CONFIG_USE_SYS_RSWDT == true)>

static void _SYS_WDT_Slowclock_Delay ( void );

</#if>
static SYS_WDT_MODULE_INSTANCE instance;
<#else>
<#--
 ########  PIC32M Variant  ########
 -->
#include "system/wdt/sys_wdt.h"
#include "peripheral/wdt/plib_wdt.h"
</#if>

void SYS_WDT_Enable ( bool windowModeEnable )
{
    <#if CONFIG_ARCH_ARM == true>
    <#--
    ########  PIC32C Variant  ########
    -->
    /* Not possible on PIC32C device (WDT config set during initialization) */
    return ;
    <#else>
    <#--
    ########  PIC32M Variant  ########
    -->
#if defined(PLIB_WDT_ExistsEnableControl)
    /* Enable/disable the watchdog timer. */
    if (true == PLIB_WDT_ExistsEnableControl(WDT_PLIB_ID))
    {
        PLIB_WDT_Enable(WDT_PLIB_ID);
    }
#endif

#if defined(PLIB_WDT_ExistsWindowEnable)
    /* Enable/disable the watchdog timer window mode. */
    if (true == PLIB_WDT_ExistsWindowEnable(WDT_PLIB_ID))
    {
        if (true == windowModeEnable)
        {
            PLIB_WDT_WindowEnable(WDT_PLIB_ID);
        }
        else
        {
            PLIB_WDT_WindowDisable(WDT_PLIB_ID);
        }
    }
#endif
  </#if>
}

void SYS_WDT_Disable ( void )
{
<#if CONFIG_ARCH_ARM == true>
    <#--
    ########  PIC32C Variant  ########
    -->
    /* Not possible on PIC32C device (WDT config set during initialization) */
    return ;
<#else>
    <#--
    ########  PIC32M Variant  ########
    -->
    PLIB_WDT_Disable(WDT_PLIB_ID);
</#if>
}

void SYS_WDT_TimerClear ( void )
{
<#if CONFIG_ARCH_ARM == true>
<#--
  ########  PIC32C Variant  ########
-->
    <#if CONFIG_USE_SYS_WDT == true>
    /* Perform WDT Software restart */
    _WDT_REGS->WDT_CR.w = (WDT_CR_KEY_PASSWD|WDT_CR_WDRSTT_Msk);
    /* Wait 3 slow clock cycles (see WDT_CR register description from product datasheet) */
    _SYS_WDT_Slowclock_Delay( );
  </#if>
    <#if CONFIG_USE_SYS_RSWDT == true>
  /* Perform RSWDT Software restart */
  _RSWDT_REGS->RSWDT_CR.w = (RSWDT_CR_KEY_PASSWD|RSWDT_CR_WDRSTT_Msk);
  /* Wait 3 slow clock cycles (see RSWDT_CR register description from product datasheet) */
    _SYS_WDT_Slowclock_Delay( );
  </#if>
  <#if (CONFIG_USE_SYS_WDT == false) && (CONFIG_USE_SYS_RSWDT == false)>
  /* Not possible as both WDT and RSWDT are disabled */
  return ;
  </#if>
<#else>
<#--
  ########  PIC32M Variant  ########
-->
    #if defined(PLIB_WDT_ExistsTimerClear)
    if(PLIB_WDT_ExistsTimerClear(WDT_PLIB_ID))
    {
        PLIB_WDT_TimerClear(WDT_PLIB_ID);
    }
  #endif
</#if>
}

<#if CONFIG_ARCH_ARM == true>
<#--
 ########  PIC32C Variant  ########
 -->
void SYS_WDT_Initialize (void)
{
  <#if CONFIG_USE_SYS_WDT == true>
  /* - Initialize system watchdog (WDT) */
  instance.timeoutPeriod = ${CONFIG_SYS_WDT_WDV};
  instance.interruptMode = ${CONFIG_SYS_WDT_INTERRUPT_MODE?c};
  instance.resetEnable   = ${CONFIG_SYS_WDT_RSTEN?c};
  instance.interruptSource = WDT_IRQn;
  <#if CONFIG_USE_SYS_INT == true && CONFIG_SYS_WDT_INTERRUPT_MODE == true>
  SYS_INT_SourceEnable(WDT_IRQn);
  </#if>
  _WDT_REGS->WDT_MR.w = \
                WDT_MR_WDV(${CONFIG_SYS_WDT_WDV}) \
                | WDT_MR_WDD(${CONFIG_SYS_WDT_WDD}) \
    <#if CONFIG_SYS_WDT_RSTEN == true>
                | WDT_MR_WDRSTEN_Msk \
    </#if>
    <#if CONFIG_SYS_WDT_DEBUG_HALT == true>
                | WDT_MR_WDDBGHLT_Msk \
    </#if>
    <#if CONFIG_SYS_WDT_IDLE_HALT == true>
                | WDT_MR_WDIDLEHLT_Msk \
    </#if>
    <#if CONFIG_USE_SYS_INT == true && CONFIG_SYS_WDT_INTERRUPT_MODE == true>
                | WDT_MR_WDFIEN_Msk \
    </#if>
  ;
  <#else>
    /* - Disable system watchdog (WDT) */
  _WDT_REGS->WDT_MR.WDDIS = true;
  </#if>
  <#if CONFIG_USE_SYS_RSWDT == true>
  /* - Initialize system reinforced safety watchdog (WDT) */
  _RSWDT_REGS->RSWDT_MR.w = \
                RSWDT_MR_WDV(${CONFIG_SYS_RSWDT_WDV}) \
                | RSWDT_MR_ALLONES_Msk \
    <#if CONFIG_SYS_WDT_RSTEN == true>
                | RSWDT_MR_WDRSTEN_Msk \
    </#if>
    <#if CONFIG_SYS_WDT_DEBUG_HALT == true>
                | RSWDT_MR_WDDBGHLT_Msk \
    </#if>
    <#if CONFIG_SYS_WDT_IDLE_HALT == true>
                | RSWDT_MR_WDIDLEHLT_Msk \
    </#if>
    <#if CONFIG_USE_SYS_INT == true && CONFIG_SYS_RSWDT_INTERRUPT_MODE == true>
                | RSWDT_MR_WDFIEN_Msk \
    </#if>
  ;
  <#else>
    /* - Disable system reinforced safety watchdog (WDT) */
  _RSWDT_REGS->RSWDT_MR.WDDIS = true;
  </#if>
}
void SYS_WDT_RegisterCallback(SYS_WDT_TIMEOUT_CALLBACK callback, 
    uintptr_t context )
{
	instance.callback = callback;
	instance.context  = context;
}

void SYS_WDT_ProcessEvents(void)
{
	(instance.callback)(instance.context);
}
  <#if (CONFIG_USE_SYS_WDT == true) || (CONFIG_USE_SYS_RSWDT == true)>

static void _SYS_WDT_Slowclock_Delay ( void )
{
    volatile uint32_t wdt_synchro_delay;
    wdt_synchro_delay = 3 * (SYS_CLK_SystemFrequencyGet()/32768)  ;
    for(;wdt_synchro_delay;wdt_synchro_delay--)
    {
        //Do nothing
    }
}
  </#if>
</#if>

/*******************************************************************************
End of File
*/
