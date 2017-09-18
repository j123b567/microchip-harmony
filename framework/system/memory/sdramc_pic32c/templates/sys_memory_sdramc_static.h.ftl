/*******************************************************************
  Company:
    Microchip Technology Inc.
    Memory System Service SDRAMC Initialization File

  File Name:
    sys_memory_sdramc_static.h.ftl

  Summary:
    SDRAM Controller (SDRAMC).
	This file contains the source code to initialize the SDRAM controller

  Description:
    SDRAM Controller configuration interface
    The SDRAMC System memory interface provides a simple interface to 
    manage the externally connected SDRAM device.

  *******************************************************************/

/*******************************************************************************
Copyright (c) 2014-2017 released Microchip Technology Inc.  All rights reserved.

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


#ifndef _SYS_MEMORY_SDRAMC_STATIC_H
#define _SYS_MEMORY_SDRAMC_STATIC_H

<#if CONFIG_USE_SYS_MEMORY_SDRAMC == true>
#include "framework/system/memory/sdramc_pic32c/sys_memory_sdramc.h"

/* SDRAM Device features. */
<#if CONFIG_SYS_SDRAMC_DEVICE_TYPE == "SDRAM">
#define SYS_MEMORY_SDRAMC_DEVICE_TYPE       (SDRAM)
<#else>
#define SYS_MEMORY_SDRAMC_DEVICE_TYPE       (SDRAM_LOW_POWER)
</#if>
<#if CONFIG_SYS_SDRAMC_NUM_ROWS == "2048_ROWS">
#define SYS_MEMORY_SDRAMC_NUM_ROWS          (SDRAMC_NUM_ROWS_2048)
<#elseif CONFIG_SYS_SDRAMC_NUM_ROWS == "4096_ROWS">
#define SYS_MEMORY_SDRAMC_NUM_ROWS          (SDRAMC_NUM_ROWS_4096)
<#else>
#define SYS_MEMORY_SDRAMC_NUM_ROWS          (SDRAMC_NUM_ROWS_8192)
</#if>
#define SYS_MEMORY_SDRAMC_NUM_ROWS_BITS     (SYS_MEMORY_SDRAMC_NUM_ROWS + 11)
<#if CONFIG_SYS_SDRAMC_NUM_COLS == "256_COLS">
#define SYS_MEMORY_SDRAMC_NUM_COLS          (SDRAMC_NUM_COLS_256)
<#elseif CONFIG_SYS_SDRAMC_NUM_COLS == "512_COLS">
#define SYS_MEMORY_SDRAMC_NUM_COLS          (SDRAMC_NUM_COLS_512)
<#elseif CONFIG_SYS_SDRAMC_NUM_COLS == "1024_COLS">
#define SYS_MEMORY_SDRAMC_NUM_COLS          (SDRAMC_NUM_COLS_1024)
<#else>
#define SYS_MEMORY_SDRAMC_NUM_COLS          (SDRAMC_NUM_COLS_2048)
</#if>
#define SYS_MEMORY_SDRAMC_NUM_COLS_BITS     (SYS_MEMORY_SDRAMC_NUM_COLS + 8)
<#if CONFIG_SYS_SDRAMC_NUM_BANKS == "2_BANKS">
#define SYS_MEMORY_SDRAMC_NUM_BANKS         (SDRAMC_NUM_BANKS_2)
<#else>
#define SYS_MEMORY_SDRAMC_NUM_BANKS         (SDRAMC_NUM_BANKS_4)
</#if>
#define SYS_MEMORY_SDRAMC_BUS_WIDTH         (SDRAMC_DATA_BUS_WIDTH_16BIT)
<#if CONFIG_SYS_SDRAMC_MEM_SCRAMBLING == true>

/* Off-chip Memory Scrambling Keys. */
#define SYS_MEMORY_SDRAMC_MEMORY_SCRAMBING_KEY1 ${CONFIG_SYS_SDRAMC_OCMS_KEY1}
#define SYS_MEMORY_SDRAMC_MEMORY_SCRAMBING_KEY2 ${CONFIG_SYS_SDRAMC_OCMS_KEY2}
</#if>

/* SDRAM Mode configuration parameters. */
#define SYS_MEMORY_SDRAMC_BURST_LENGTH      ${CONFIG_SYS_SDRAMC_BURST_LENGTH}
/* Burst Type. */
<#if CONFIG_SYS_SDRAMC_BURST_TYPE == "SEQUENTIAL">
#define SYS_MEMORY_SDRAMC_BURST_TYPE        (SDRAMC_BURST_TYPE_SEQUENTIAL)
<#else>
#define SYS_MEMORY_SDRAMC_BURST_TYPE        (SDRAMC_BURST_TYPE_INTERLEAVED)
</#if>

/* SDRAM timing parameters. */
#define SYS_MEMORY_SDRAMC_TRCD_DELAY        ${CONFIG_SYS_SDRAMC_TRCD_DELAY}
#define SYS_MEMORY_SDRAMC_CAS_LATENCY       ${CONFIG_SYS_SDRAMC_CAS_LATENCY}
#define SYS_MEMORY_SDRAMC_TRAS_DELAY        ${CONFIG_SYS_SDRAMC_TRAS_DELAY}
#define SYS_MEMORY_SDRAMC_TRP_DELAY         ${CONFIG_SYS_SDRAMC_TRP_DELAY}
#define SYS_MEMORY_SDRAMC_TRC_TRFC_DELAY    ${CONFIG_SYS_SDRAMC_TRC_TRFC_DELAY}
#define SYS_MEMORY_SDRAMC_TWR_DELAY         ${CONFIG_SYS_SDRAMC_TWR_DELAY}
#define SYS_MEMORY_SDRAMC_TMRD_DELAY        ${CONFIG_SYS_SDRAMC_TMRD_DELAY}

<#if CONFIG_SYS_SDRAMC_DEVICE_TYPE == "Low-power SDRAM">
/* Low-power configuration parameters. */
<#if CONFIG_SYS_SDRAMC_LP_CONFIG == "DISABLED">
#define SYS_MEMORY_SDRAMC_LOW_POWER_CONFIG  (SDRAMC_LP_DISABLED)
<#elseif CONFIG_SYS_SDRAMC_LP_CONFIG == "SELF_REFRESH">
#define SYS_MEMORY_SDRAMC_LOW_POWER_CONFIG  (SDRAMC_LP_SELF_REFRESH)
<#elseif CONFIG_SYS_SDRAMC_LP_CONFIG == "POWER_DOWN">
#define SYS_MEMORY_SDRAMC_LOW_POWER_CONFIG  (SDRAMC_LP_POWER_DOWN)
<#else>
#define SYS_MEMORY_SDRAMC_LOW_POWER_CONFIG  (SDRAMC_LP_DEEP_POWER_DOWN)
</#if>
<#if CONFIG_SYS_SDRAMC_LP_TIMEOUT == "LAST_TRANSFER_NO_WAIT">
#define SYS_MEMORY_SDRAMC_LOW_POWER_TIMEOUT (LAST_TRANSFER_NO_WAIT)
<#elseif CONFIG_SYS_SDRAMC_LP_TIMEOUT == "LAST_TRANSFER_64_CYCLES">
#define SYS_MEMORY_SDRAMC_LOW_POWER_TIMEOUT (LAST_TRANSFER_64_CYCLES)
<#else>
#define SYS_MEMORY_SDRAMC_LOW_POWER_TIMEOUT (LAST_TRANSFER_128_CYCLES)
</#if>
#define SYS_MEMORY_SDRAMC_TXSR_DELAY        ${CONFIG_SYS_SDRAMC_TRCD_DELAY}
#define SYS_MEMORY_SDRAMC_PASR              ${CONFIG_SYS_SDRAMC_PASR}
#define SYS_MEMORY_SDRAMC_TCSR              ${CONFIG_SYS_SDRAMC_TCSR}
#define SYS_MEMORY_SDRAMC_DS                ${CONFIG_SYS_SDRAMC_DS}
</#if>

//******************************************************************************
/* Function:
    void SYS_MEMORY_SDRAMC_Initialize ( void )

  Summary:
    Initializes and Enables the SDRAM Controller.

  Description:
    This function Enables the external memory controller module(s).

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
    SYS_MEMORY_SDRAMC_Initialize();
  </code>

  Remarks:
    This routine must be called before any attempt to access external
    memory.

    Not all features are available on all devices. Refer to the specific
	device data sheet to determine availability.
*/
void SYS_MEMORY_SDRAMC_Initialize
(
    void
);
</#if>
#endif // #ifndef _SYS_MEMORY_SDRAMC_STATIC_H

/*******************************************************************************
 End of File
*/
