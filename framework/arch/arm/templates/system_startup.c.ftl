/**
 * Copyright (c) 2016-2017 Microchip Technology Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
<#--  =====================
      MACRO mhc_expand_vectors_list
      list format is "vectors_structure_member_name" "function_symbol_name".
      This macro will take the 2 words and create the proper line into the vector table.
      ===================== -->
<#macro mhc_expand_vectors_list vectors>
  <#list vectors as vector>
    <#assign test = vector?split(" ")>
      <#list test[0..0] as item>
        <#assign struct_member = item?ensure_starts_with("  .")?right_pad(33)>
      </#list>
      <#list test[1..1] as item>
      <#assign symbol = item?ensure_starts_with("= (void*) ")?ensure_ends_with(",")>
      </#list>
      <#assign resulting_line = struct_member + symbol?ensure_ends_with("\xa")>
      <#assign interpreted_line = resulting_line?interpret>
      <@interpreted_line />
  </#list>
</#macro>

#include "arch/arm/devices_pic32c.h" /* for vectors structure and default handler names and CMSIS API */
#include "arch/arch.h" /* for ARCH_CORE_MPU_Initialize() defintion */
<#if CONFIG_PROJECT_USES_MPU == true>
#include "arch/arm/arm_cm_mpu.h"
</#if>
#include "system_definitions.h" /* for potential custom handler names */

/* Symbols exported from linker script */
extern uint32_t __etext ;
extern uint32_t __ram_vectors__;
extern uint32_t __data_start__ ;
extern uint32_t __data_end__ ;
extern uint32_t __bss_start__ ;
extern uint32_t __bss_end__ ;
extern uint32_t __StackTop;

extern int main( void ) ;
/* symbols from libc */
extern void __libc_init_array(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Declaration of Reset handler (may be custom) */
<#if CONFIG_PROJECT_USES_CUSTOMIZABLE_VECTORS == true>
void ${CONFIG_PROJECT_RESET_HANDLER_CUSTOM_NAME}(void);
<#else>
void Reset_Handler(void);
</#if>

/* Cortex-M7 core handlers */
/*
 * Warn about debug specific handlers usage:
 * - DEBUG=1 may be set by user on toolchain command line
 * - __DEBUG is set by MPLAB X while Building Application for Debug
 */
#if (defined DEBUG && (DEBUG == 1)) || (defined __DEBUG)
#pragma message "DEBUG handlers activated"

/*
 * These handlers are unitary implemented as weak (coming from ARM CMSIS-Core definitions) to let user defined its own.
 */
__attribute__((weak)) void NonMaskableInt_Handler(void)
{
  __BKPT(14);
  while (1);
}

__attribute__((weak)) void HardFault_Handler(void)
{
  __BKPT(13);
  while (1);
}

<#if (CONFIG_ARCH_ARM_CORTEX_M4 == true) || (CONFIG_ARCH_ARM_CORTEX_M7 == true)>
__attribute__((weak)) void MemoryManagement_Handler(void)
{
  __BKPT(12);
  while (1);
}

__attribute__((weak)) void BusFault_Handler(void)
{
  __BKPT(11);
  while (1);
}

__attribute__((weak)) void UsageFault_Handler(void)
{
  __BKPT(10);
  while (1);
}
</#if>

__attribute__((weak)) void SVCall_Handler(void)
{
  __BKPT(5);
  while (1);
}

<#if (CONFIG_ARCH_ARM_CORTEX_M4 == true) || (CONFIG_ARCH_ARM_CORTEX_M7 == true)>
__attribute__((weak)) void DebugMonitor_Handler(void)
{
  __BKPT(4);
  while (1);
}
</#if>

__attribute__((weak)) void PendSV_Handler(void)
{
  __BKPT(2);
  while (1);
}

__attribute__((weak)) void SysTick_Handler(void)
{
  __BKPT(1);
  while (1);
}

#else

void NonMaskableInt_Handler   ( void ) __attribute__((weak, alias("Dummy_Handler")));
void HardFault_Handler        ( void ) __attribute__((weak, alias("Dummy_Handler")));
<#if (CONFIG_ARCH_ARM_CORTEX_M4 == true) || (CONFIG_ARCH_ARM_CORTEX_M7 == true)>
void MemoryManagement_Handler ( void ) __attribute__((weak, alias("Dummy_Handler")));
void BusFault_Handler         ( void ) __attribute__((weak, alias("Dummy_Handler")));
void UsageFault_Handler       ( void ) __attribute__((weak, alias("Dummy_Handler")));
</#if>
void SVCall_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
<#if (CONFIG_ARCH_ARM_CORTEX_M4 == true) || (CONFIG_ARCH_ARM_CORTEX_M7 == true)>
void DebugMonitor_Handler     ( void ) __attribute__((weak, alias("Dummy_Handler")));
</#if>
void PendSV_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));

#endif // DEBUG=1

/* Exception Table */
__attribute__ ((section(".flash_vectors")))
const DeviceVectors exception_table=
{
  /* Configure Initial Stack Pointer, using linker-generated symbols */
  .pvStack = (void*) (&__StackTop),

<#if CONFIG_PROJECT_USES_CUSTOMIZABLE_VECTORS == true>
  .pfnReset_Handler              = (void*) ${CONFIG_PROJECT_RESET_HANDLER_CUSTOM_NAME},
<#else>
  .pfnReset_Handler              = (void*) Reset_Handler,
</#if>
  .pfnNonMaskableInt_Handler     = (void*) NonMaskableInt_Handler,
  .pfnHardFault_Handler          = (void*) HardFault_Handler,
<#if CONFIG_ARCH_ARM_CORTEX_M0PLUS == true>
  .pvReservedC12                 = (void*) (0UL), /* Reserved */
  .pvReservedC11                 = (void*) (0UL), /* Reserved */
  .pvReservedC10                 = (void*) (0UL), /* Reserved */
  .pvReservedC4                  = (void*) (0UL), /* Reserved */
</#if>
<#if (CONFIG_ARCH_ARM_CORTEX_M4 == true) || (CONFIG_ARCH_ARM_CORTEX_M7 == true)>
  .pfnMemoryManagement_Handler   = (void*) MemoryManagement_Handler,
  .pfnBusFault_Handler           = (void*) BusFault_Handler,
  .pfnUsageFault_Handler         = (void*) UsageFault_Handler,
  .pfnDebugMonitor_Handler       = (void*) DebugMonitor_Handler,
</#if>
  .pvReservedC9                  = (void*) (0UL), /* Reserved */
  .pvReservedC8                  = (void*) (0UL), /* Reserved */
  .pvReservedC7                  = (void*) (0UL), /* Reserved */
  .pvReservedC6                  = (void*) (0UL), /* Reserved */
  .pfnSVCall_Handler             = (void*) SVCall_Handler,
  .pvReservedC3                  = (void*) (0UL), /* Reserved */
  .pfnPendSV_Handler             = (void*) PendSV_Handler,

<#if LIST_SYSTEM_STARTUP_PIC32C_INTERRUPT_HANDLERS??>
  <#if LIST_SYSTEM_STARTUP_PIC32C_INTERRUPT_HANDLERS?has_content>
    <@mhc_expand_vectors_list vectors=LIST_SYSTEM_STARTUP_PIC32C_INTERRUPT_HANDLERS/>
  </#if>
</#if>
};

/**
 * This is the code that gets called on processor reset.
 * - Initializes the data sections
 * - Optionally calls CMSIS SystemInit()
 * - Initializes the (nano-)newlib libc
 * - Then call main()
 */
<#if CONFIG_PROJECT_USES_CUSTOMIZABLE_VECTORS == true>
void ${CONFIG_PROJECT_RESET_HANDLER_CUSTOM_NAME}(void)
<#else>
void Reset_Handler(void)
</#if>
{
    uint32_t *pSrc, *pDest;

  /* call present for MIPS, not sure it is needed here */
//  _on_reset();

<#if CONFIG_PROJECT_USES_MPU == true>
    ARCH_CORE_MPU_Initialize();
</#if>

<#if CONFIG_PROJECT_USES_CACHE == true>
    /* Enable Caches using CMSIS API */
    SCB_EnableICache();
    SCB_EnableDCache();
</#if>

<#if CONFIG_PROJECT_USES_FPU == true>
    /* enable FPU */
    SCB->CPACR |= (0x3u << 20) | (0x3u << 22);
    __DSB();
    __ISB();
</#if>

    /* Initialize the initialized data section */
    pSrc = &__etext;
    pDest = &__data_start__;

    if ( (&__data_start__ != &__data_end__) && (pSrc != pDest) )
    {
       for (; pDest < &__data_end__ ; pDest++, pSrc++ )
       {
           *pDest = *pSrc ;
       }
    }

    /* Clear the zero section */
    if ( &__bss_start__ != &__bss_end__ )
    {
        for ( pDest = &__bss_start__ ; pDest < &__bss_end__ ; pDest++ )
        {
            *pDest = 0ul ;
        }
    }

    /* Call SYS_MEMORY init */
    /* todo() */

<#if CONFIG_PROJECT_USES_RAM_VECTORS == true>
    /* Set the vector table base address */
    pSrc = &__ram_vectors__;
    SCB->VTOR = ((uint32_t)pSrc & SCB_VTOR_TBLOFF_Msk);
</#if>

#if !defined DONT_USE_CMSIS_INIT
    /* Initialize the system */
    SystemInit() ;
#endif /* DONT_USE_CMSIS_INIT */

    /* calls _init() functions, C++ constructors included */
    __libc_init_array();

    /* Call the "on bootstrap" procedure */
//    _on_bootstrap();

    /* Branch to main function */
    main() ;

  /* Infinite loop */
#if (defined DEBUG && (DEBUG == 1)) || (defined __DEBUG)
    __BKPT(15);
#endif

    while (1)
    {
    }
}

/**
 * Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void)
{
  /* Software breakpoint */
#if (defined DEBUG && (DEBUG == 1)) || (defined __DEBUG)
    __BKPT(0);
#endif

	/* halt CPU */
    while (1)
    {
    }
}
