<#--
/*******************************************************************************
  SDRAMC System Memory Configuration

  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory_sdramc.h.ftl

  Summary:
     SDRAM Controller (SDRAMC)configuration file

  Description:
    The SDRAMC System Memory configuration set the timings of the relevant SDRAMC

  *******************************************************************************/

-->

<#if CONFIG_USE_SYS_MEMORY_SDRAMC == true>
/*******************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory_sdramc.h

  Summary:
    SDRAM Controller (SDRAMC)configuration file

  Description:
    The SDRAMC System Memory configuration set the timings of the relevant SDRAMC

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

#ifndef _SYS_MEMORY_SDRAMC_H
#define _SYS_MEMORY_SDRAMC_H


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "system/system.h"
#include "arch/arm/devices_pic32c.h" /* PIC32C system header. */
#include "system/clk/sys_clk.h"
#include "framework/system/memory/sdramc_pic32c/src/sys_memory_sdramc_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/* Data Bus Width

  Summary:
    Enumerated data type listing supported data bus widths.

  Description:
    These enumerated values list the various data bus widths supported by the
    SDRAM Controller.

  Remarks:
    None.
*/
typedef enum {
    SDRAMC_DATA_BUS_WIDTH_16BIT = 1,
} SDRAMC_DATA_BUS_WIDTH;

//*******************************************************************************
/* Burst Type

  Summary:
    Enumerated data type listing supported burst types.

  Description:
    These enumerated values list the burst types supported by the SDRAM
    Controller.

  Remarks:
    None.
*/
typedef enum {
    SDRAMC_BURST_TYPE_SEQUENTIAL = 0,
    SDRAMC_BURST_TYPE_INTERLEAVED
} SDRAMC_BURST_TYPE;

//*******************************************************************************
/* Row configuration

  Summary:
    Enumerated data type listing the various Row sizes supported.

  Description:
    These enumerated values list the various Row sizes supported.

  Remarks:
    None.
*/
typedef enum {
    SDRAMC_NUM_ROWS_2048 = 0,
    SDRAMC_NUM_ROWS_4096,
    SDRAMC_NUM_ROWS_8192,
} SDRAMC_NUM_ROWS;

//*******************************************************************************
/* Column configuration

  Summary:
    Enumerated data type listing the various Column sizes supported.

  Description:
    These enumerated values list the various Column sizes supported.

  Remarks:
    None.
*/
typedef enum {
    SDRAMC_NUM_COLS_256 = 0,
    SDRAMC_NUM_COLS_512,
    SDRAMC_NUM_COLS_1024,
    SDRAMC_NUM_COLS_2048,
} SDRAMC_NUM_COLS;

//*******************************************************************************
/* Bank configuration

  Summary:
    Enumerated data type listing number of banks supported.

  Description:
    These enumerated values list the number of banks supported.

  Remarks:
    None.
*/
typedef enum {
    SDRAMC_NUM_BANKS_2 = 0,
    SDRAMC_NUM_BANKS_4
} SDRAMC_NUM_BANKS;

//*******************************************************************************
/* SDRAM Device Type

  Summary:
    Enumerated data type listing type of SDRAM devices supported.

  Description:
    These enumerated values list the type of SDRAM devices supported.

  Remarks:
    None.
*/
typedef enum
{
    SDRAM = 0,
    SDRAM_LOW_POWER
} SDRAMC_DEVICE_TYPE;

//*******************************************************************************
/* SDRAM Low Power configuration

  Summary:
    Enumerated data type listing the low power configuration options.

  Description:
    These enumerated values list the SDRAM low power configuration options.

  Remarks:
    None.
*/
typedef enum
{
    SDRAMC_LP_DISABLED = 0,
    SDRAMC_LP_SELF_REFRESH,
    SDRAMC_LP_POWER_DOWN,
    SDRAMC_LP_DEEP_POWER_DOWN,
} SDRAMC_LP_CONFIG;

//*******************************************************************************
/* SDRAM Low Power Timeout

  Summary:
    Enumerated data type listing the times when low power is enabled.

  Description:
    These enumerated values list the times when the low power is enabled.

  Remarks:
    None.
*/
typedef enum
{
    LAST_TRANSFER_NO_WAIT = 0,
    LAST_TRANSFER_64_CYCLES,
    LAST_TRANSFER_128_CYCLES,
} SDRAMC_LP_TIMEOUT;

/* SDRAMC hardware instance configuration

  Summary:
    Selects the maximum number of hardware instances that can be supported.

  Description:
    This definition selects the maximum number of hardware instances that can be
    supported by the pic32c device.

  Remarks:
    None.
*/

#define SYS_MEM_SDRAMC_INSTANCES_NUMBER     (SDRAMC_NUMBER_OF_MODULES)

typedef struct
{
    /* System module initialization data */
    SYS_MODULE_INIT moduleInit;

    /* Identifies SDRAMC hardware module ID. */
    uint32_t sdramcID;

} SYS_SDRAMC_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

#endif // #ifndef _SYS_MEMORY_SDRAMC_H

/*******************************************************************************
 End of File
*/
</#if>
