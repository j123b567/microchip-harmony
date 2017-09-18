<#--
/*******************************************************************************
  SMC System Memory Configuration

  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory_smc.h.ftl

  Summary:
     Static Memory Controller (SMC)configuration file

  Description:
    The SMC System Memory configuration set the timings of the relevant SMC Chip select

  *******************************************************************************/

-->

<#if CONFIG_USE_SYS_MEMORY_SMC == true>
/*******************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    sys_memory_smc.h

  Summary:
    Static Memory Controller(SMC) configuration file

  Description:
    The SMC System Memory configuration set the timings of the relevant SMC Chip select

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

#ifndef _SYS_MEMORY_SMC_H
#define _SYS_MEMORY_SMC_H


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "system/system.h"
#include "arch/arm/devices_pic32c.h" /* PIC32C system header. */
#include "framework/system/memory/smc_pic32c/src/sys_memory_smc_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

typedef enum {
	SMC_CS0 = 0x0, /* SMC Chip Select 0 */
	SMC_CS1 = 0x1, /* SMC Chip Select 1 */
    SMC_CS2 = 0x2, /* SMC Chip Select 2 */
    SMC_CS3 = 0x3  /* SMC Chip Select 3 */
}SMC_CS_NUM; /* SMC Chip Select list */

typedef enum {
    SMC_DATA_BUS_WIDTH_8BIT  = SMC_MODE_DBW_8_BIT, /* 8-bit data bus width*/
    SMC_DATA_BUS_WIDTH_16BIT  =SMC_MODE_DBW_16_BIT /* 16-bit data bus width*/
} SMC_DATA_BUS_WIDTH; /* Data bus size */

typedef enum {
  WRITE_PROTECT_OFF =0x0, /* Write Protection is OFF */
  WRITE_PROTECT_ON =0x1 /* Write Protection is ON, ALL the chips Select timings registers are in read-only access */
}SMC_WRITE_PROTECT; /* SMC Timings register write protection feature */

typedef enum {
	BAT_BYTE_SELECT = SMC_MODE_BAT_BYTE_SELECT, /* Write operation is controlled using NCS,NWE,NBS0-1 Read operation is controlled using NCS,NRD,NBS0-1 */
	BAT_BYTE_WRITE = SMC_MODE_BAT_BYTE_WRITE /* Write operation is controlled using NCS,NWR0-1 Read operation is controlled using NCS,NRD */
}SMC_BAT; /* Byte Access Type */

typedef enum {
  NWAIT_DISABLED_MODE = SMC_MODE_EXNW_MODE_DISABLED, /* NWAIT input signal is ignored on the corresponding chip select */
  NWAIT_FROZEN_MODE = SMC_MODE_EXNW_MODE_FROZEN, /* NWAIT signal freezes the current read or write cycle */
  NWAIT_READY_MODE = SMC_MODE_EXNW_MODE_READY /* NWAIT signal indicated the availability of the external device at the end of the pulse read or wirte signal */
}SMC_NWAIT_MODE; /* NWAIT signal is used to extend the current read or write signal */

typedef enum {
  MEM_SCRAMBLING_OFF =0x0, /* SMC Memory scrambling is disabled */
  MEM_SCRAMBLING_ON= 0x1 /* SMC Memory scrambling is enabled */
}SMC_MEM_SCRAMBLING; /* The External data bus can be scrambled to prevent recovery of data*/

typedef enum {
  WRITE_MODE_OFF =(0x0u << SMC_MODE_WRITE_MODE_Pos), /* Write mode operation is controlled by NCS signal*/
  WRITE_MODE_ON  = SMC_MODE_WRITE_MODE_Msk /* Write mode opeeration is controlled by NWE signal */
}SMC_WRITE_MODE;

typedef enum {
  STD_READ_PAGE_MODE =(0x0u << SMC_MODE_PMEN_Pos), /* Standard Read*/
  ASYNC_READ_PAGE_MODE = SMC_MODE_PMEN_Msk /* Asynchronous burst resd in page mode is applied on the corresponding chip select*/
}SMC_PAGE_MODE;


/* SMC hardware instance configuration

  Summary:
    Selects the maximum number of hardware instances that can be supported.

  Description:
    This definition selects the maximum number of hardware instances that can be
    supported by the pic32c device.

  Remarks:
    None.
*/

#define SYS_MEM_SMC_INSTANCES_NUMBER                        SMC_NUMBER_OF_MODULES


// *****************************************************************************
/* SMC maximum number of clients

  Summary:
    Selects the maximum number of clients .ie Chip Select

  Description:
    This definition select the maximum number of client that the SMC interface can
    support at run time.

  Remarks:
    None.

*/

#define SYS_SMC_CLIENTS_NUMBER SMCCSNUMBER_NUMBER


typedef struct
{
    /* System module initialization data */
    SYS_MODULE_INIT moduleInit;

    /* Identifies SMC hardware module ID. */
    uint32_t smcID;
} SYS_SMC_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

//********************** SMC Global Features ***********************************

<#if CONFIG_SYS_MEMORY_SMC_WRITE_PROTECTION == true>
#define SMC_WRITE_PROTECTION 0x1
<#else> #define SMC_WRITE_PROTECTION 0x0
</#if>

<#if CONFIG_SYS_MEMORY_SMC_MEM_SCRAMBLING == true>
#define SMC_MEMORY_SCRAMBLING 0x1
<#else>#define SMC_MEMORY_SCRAMBLING 0x0
</#if>

// *****************************************************************************
// ********** Chip Select 0 Configuration (Enabled per default)*****************
// *****************************************************************************
<#if CONFIG_SYS_MEMORY_SMC_CS0 == true>
<#if CONFIG_SYS_MEMORY_SMC_CS0_MEM_SCRAMBLING == true>
#define SMC_MEM_SCRAMBLING_CS0 0x1
<#else> #define SMC_MEM_SCRAMBLING_CS0 0x0
</#if>

// Setup Register
#define SYS_MEMORY_SMC_CS0 SMC_CS0
#define SMC_NWE_SETUP_CS0 ${CONFIG_SYS_MEMORY_SMC_NWE_SETUP_CS0}
#define SMC_NCS_WR_SETUP_CS0 ${CONFIG_SYS_MEMORY_SMC_NCS_WR_SETUP_CS0}
#define SMC_NRD_SETUP_CS0 ${CONFIG_SYS_MEMORY_SMC_NRD_SETUP_CS0}
#define SMC_NCS_RD_SETUP_CS0 ${CONFIG_SYS_MEMORY_SMC_NCS_RD_SETUP_CS0}

// Pulse Register
#define SMC_NWE_PULSE_CS0 ${CONFIG_SYS_MEMORY_SMC_NWE_PULSE_CS0}
#define SMC_NCS_WR_PULSE_CS0 ${CONFIG_SYS_MEMORY_SMC_NCS_WR_PULSE_CS0}
#define SMC_NRD_PULSE_CS0 ${CONFIG_SYS_MEMORY_SMC_NRD_PULSE_CS0}
#define SMC_NCS_RD_PULSE_CS0 ${CONFIG_SYS_MEMORY_SMC_NCS_RD_PULSE_CS0}

//Cycle Register
#define SMC_NWE_CYCLE_CS0 ${CONFIG_SYS_MEMORY_SMC_NWE_CYCLE_CS0}
#define SMC_NRD_CYCLE_CS0 ${CONFIG_SYS_MEMORY_SMC_NRD_CYCLE_CS0}

// Mode Register
<#if CONFIG_SYS_MEMORY_SMC_PMEN_CS0 == true>
#define SMC_PMEN_CS0 ASYNC_READ_PAGE_MODE
#define SMC_PS_CS0 ${CONFIG_SYS_MEMORY_SMC_PS_CS0}
<#else>
#define SMC_PMEN_CS0 STD_READ_PAGE_MODE
#define SMC_PS_CS0 SMC_MODE_PS_4_BYTE
</#if>
#define SMC_DATA_BUS_CS0 ${CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS0}
<#if CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS0 == "SMC_DATA_BUS_WIDTH_16BIT">
#define SMC_BAT_CS0 ${CONFIG_SYS_MEMORY_SMC_BAT_CS0}
<#else>#define SMC_BAT_CS0 0x0
</#if>
<#if CONFIG_SYS_MEMORY_SMC_TDF_MODE_CS0 == true>
#define SMC_TDF_MODE_CS0 0x1
#define SMC_TDF_CYCLES_CS0 ${CONFIG_SYS_MEMORY_SMC_TDF_CYCLES_CS0}
<#else>
#define SMC_TDF_MODE_CS0 0x0
#define SMC_TDF_CYCLES_CS0 0x0
</#if>
#define SMC_NWAIT_MODE_CS0 ${CONFIG_SYS_MEMORY_SMC_NWAIT_MODE_CS0}
<#if CONFIG_SYS_MEMORY_SMC_WRITE_MODE_CS0 == true >
#define SMC_WRITE_MODE_CS0 0x1
<#else>#define SMC_WRITE_MODE_CS0 0x0
</#if>
 <#if CONFIG_SYS_MEMORY_SMC_READ_MODE_CS0 == true>
#define SMC_READ_MODE_CS0 0x1
<#else>
#define SMC_READ_MODE_CS0 0x0
</#if>

</#if> // End Chip Select 0 settings

// *****************************************************************************
// ********** Chip Select 1 Configuration **************************************
// *****************************************************************************

<#if CONFIG_SYS_MEMORY_SMC_CS1 == true>
#define  SYS_MEMORY_SMC_CS1 SMC_CS1
<#if CONFIG_SYS_MEMORY_SMC_CS1_MEM_SCRAMBLING == true>
#define SMC_MEM_SCRAMBLING_CS1 0x1
<#else>
#define SMC_MEM_SCRAMBLING_CS1 0x0
</#if>

// Setup Register
#define SMC_NWE_SETUP_CS1 ${CONFIG_SYS_MEMORY_SMC_NWE_SETUP_CS1}
#define SMC_NCS_WR_SETUP_CS1 ${CONFIG_SYS_MEMORY_SMC_NCS_WR_SETUP_CS1}
#define SMC_NRD_SETUP_CS1 ${CONFIG_SYS_MEMORY_SMC_NRD_SETUP_CS1}
#define SMC_NCS_RD_SETUP_CS1 ${CONFIG_SYS_MEMORY_SMC_NCS_RD_SETUP_CS1}

// Pulse Register
#define SMC_NWE_PULSE_CS1 ${CONFIG_SYS_MEMORY_SMC_NWE_PULSE_CS1}
#define SMC_NCS_WR_PULSE_CS1 ${CONFIG_SYS_MEMORY_SMC_NCS_WR_PULSE_CS1}
#define SMC_NRD_PULSE_CS1 ${CONFIG_SYS_MEMORY_SMC_NRD_PULSE_CS1}
#define SMC_NCS_RD_PULSE_CS1 ${CONFIG_SYS_MEMORY_SMC_NCS_RD_PULSE_CS1}

// Cycle Register
#define SMC_NWE_CYCLE_CS1 ${CONFIG_SYS_MEMORY_SMC_NWE_CYCLE_CS1}
#define SMC_NRD_CYCLE_CS1 ${CONFIG_SYS_MEMORY_SMC_NRD_CYCLE_CS1}

// Mode Register
<#if CONFIG_SYS_MEMORY_SMC_PMEN_CS1 == true>
#define SMC_PMEN_CS1 ASYNC_READ_PAGE_MODE
#define SMC_PS_CS1 ${CONFIG_SYS_MEMORY_SMC_PS_CS1}
<#else>
#define SMC_PMEN_CS1 STD_READ_PAGE_MODE
#define SMC_PS_CS1 SMC_MODE_PS_4_BYTE
</#if>
#define SMC_DATA_BUS_CS1 ${CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS1}
<#if CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS1 == "SMC_DATA_BUS_WIDTH_16BIT">
#define SMC_BAT_CS1 ${CONFIG_SYS_MEMORY_SMC_BAT_CS1}
<#else>
#define SMC_BAT_CS1 0x0
</#if>
<#if CONFIG_SYS_MEMORY_SMC_TDF_MODE_CS1 == true>
#define SMC_TDF_MODE_CS1 0x1
#define SMC_TDF_CYCLES_CS1 ${CONFIG_SYS_MEMORY_SMC_TDF_CYCLES_CS1}
<#else>
#define SMC_TDF_MODE_CS1 0x0
#define SMC_TDF_CYCLES_CS1 0x0
</#if>
#define SMC_NWAIT_MODE_CS1 ${CONFIG_SYS_MEMORY_SMC_NWAIT_MODE_CS1}
<#if CONFIG_SYS_MEMORY_SMC_WRITE_MODE_CS1 == true >
#define SMC_WRITE_MODE_CS1 0x1
<#else>
#define SMC_WRITE_MODE_CS1 0x0
</#if>
<#if CONFIG_SYS_MEMORY_SMC_READ_MODE_CS1 == true>
#define SMC_READ_MODE_CS1 0x1
<#else>
#define SMC_READ_MODE_CS1 0x0
</#if>
</#if> // End Chip Select 1 settings

// *****************************************************************************
// ********* Chip Select 2 Configuration ***************************************
// *****************************************************************************

<#if CONFIG_SYS_MEMORY_SMC_CS2 == true>
#define  SYS_MEMORY_SMC_CS2 SMC_CS2
<#if CONFIG_SYS_MEMORY_SMC_CS2_MEM_SCRAMBLING == true>
#define SMC_MEM_SCRAMBLING_CS2 0x1
<#else>
#define SMC_MEM_SCRAMBLING_CS2 0x0
</#if>

// Setup Register
#define SMC_NWE_SETUP_CS2 ${CONFIG_SYS_MEMORY_SMC_NWE_SETUP_CS2}
#define SMC_NCS_WR_SETUP_CS2 ${CONFIG_SYS_MEMORY_SMC_NCS_WR_SETUP_CS2}
#define SMC_NRD_SETUP_CS2 ${CONFIG_SYS_MEMORY_SMC_NRD_SETUP_CS2}
#define SMC_NCS_RD_SETUP_CS2 ${CONFIG_SYS_MEMORY_SMC_NCS_RD_SETUP_CS2}

// Pulse Register
#define SMC_NWE_PULSE_CS2 ${CONFIG_SYS_MEMORY_SMC_NWE_PULSE_CS2}
#define SMC_NCS_WR_PULSE_CS2 ${CONFIG_SYS_MEMORY_SMC_NCS_WR_PULSE_CS2}
#define SMC_NRD_PULSE_CS2 ${CONFIG_SYS_MEMORY_SMC_NRD_PULSE_CS2}
#define SMC_NCS_RD_PULSE_CS2 ${CONFIG_SYS_MEMORY_SMC_NCS_RD_PULSE_CS2}

// Cycle Register
#define SMC_NWE_CYCLE_CS2 ${CONFIG_SYS_MEMORY_SMC_NWE_CYCLE_CS2}
#define SMC_NRD_CYCLE_CS2 ${CONFIG_SYS_MEMORY_SMC_NRD_CYCLE_CS2}

// Mode Register
<#if CONFIG_SYS_MEMORY_SMC_PMEN_CS2 == true>
#define SMC_PMEN_CS2 ASYNC_READ_PAGE_MODE
#define SMC_PS_CS2 ${CONFIG_SYS_MEMORY_SMC_PS_CS2}
<#else>
#define SMC_PMEN_CS2 STD_READ_PAGE_MODE
#define SMC_PS_CS2 SMC_MODE_PS_4_BYTE
</#if>
#define SMC_DATA_BUS_CS2 ${CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS2}
<#if CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS2 == "SMC_DATA_BUS_WIDTH_16BIT">
#define SMC_BAT_CS2 ${CONFIG_SYS_MEMORY_SMC_BAT_CS2}
<#else>
#define SMC_BAT_CS2 0x0
</#if>
<#if CONFIG_SYS_MEMORY_SMC_TDF_MODE_CS2 == true>
#define SMC_TDF_MODE_CS2 0x1
#define SMC_TDF_CYCLES_CS2 ${CONFIG_SYS_MEMORY_SMC_TDF_CYCLES_CS2}
<#else>
#define SMC_TDF_MODE_CS2 0x0
#define SMC_TDF_CYCLES_CS2 0x0
</#if>
#define SMC_NWAIT_MODE_CS2 ${CONFIG_SYS_MEMORY_SMC_NWAIT_MODE_CS2}
<#if CONFIG_SYS_MEMORY_SMC_WRITE_MODE_CS2 == true >
#define SMC_WRITE_MODE_CS2 0x1
<#else>
#define SMC_WRITE_MODE_CS2 0x0
</#if>
<#if CONFIG_SYS_MEMORY_SMC_READ_MODE_CS2 == true>
#define SMC_READ_MODE_CS2 0x1
<#else>
#define SMC_READ_MODE_CS2 0x0
</#if>
</#if> // End Chip Select 2 settings

// *****************************************************************************
// ************ Chip Select 3 Configuration  ***********************************
// *****************************************************************************

<#if CONFIG_SYS_MEMORY_SMC_CS3 == true>
#define  SYS_MEMORY_SMC_CS3 SMC_CS3

<#if CONFIG_SYS_MEMORY_SMC_CS3_MEM_SCRAMBLING == true>
#define SMC_MEM_SCRAMBLING_CS3 0x1
<#else>
#define SMC_MEM_SCRAMBLING_CS3 0x0
</#if>

// Setup Register
#define SMC_NWE_SETUP_CS3 ${CONFIG_SYS_MEMORY_SMC_NWE_SETUP_CS3}
#define SMC_NCS_WR_SETUP_CS3 ${CONFIG_SYS_MEMORY_SMC_NCS_WR_SETUP_CS3}
#define SMC_NRD_SETUP_CS3 ${CONFIG_SYS_MEMORY_SMC_NRD_SETUP_CS3}
#define SMC_NCS_RD_SETUP_CS3 ${CONFIG_SYS_MEMORY_SMC_NCS_RD_SETUP_CS3}

// Pulse Register
#define SMC_NWE_PULSE_CS3 ${CONFIG_SYS_MEMORY_SMC_NWE_PULSE_CS3}
#define SMC_NCS_WR_PULSE_CS3 ${CONFIG_SYS_MEMORY_SMC_NCS_WR_PULSE_CS3}
#define SMC_NRD_PULSE_CS3 ${CONFIG_SYS_MEMORY_SMC_NRD_PULSE_CS3}
#define SMC_NCS_RD_PULSE_CS3 ${CONFIG_SYS_MEMORY_SMC_NCS_RD_PULSE_CS3}

// Cycle Register
#define SMC_NWE_CYCLE_CS3 ${CONFIG_SYS_MEMORY_SMC_NWE_CYCLE_CS3}
#define SMC_NRD_CYCLE_CS3 ${CONFIG_SYS_MEMORY_SMC_NRD_CYCLE_CS3}

// Mode Register
<#if CONFIG_SYS_MEMORY_SMC_PMEN_CS3 == true>
#define SMC_PMEN_CS3 ASYNC_READ_PAGE_MODE
#define SMC_PS_CS3 ${CONFIG_SYS_MEMORY_SMC_PS_CS3}
<#else>
#define SMC_PMEN_CS3 STD_READ_PAGE_MODE
#define SMC_PS_CS3 SMC_MODE_PS_4_BYTE
</#if>
#define SMC_DATA_BUS_CS3 ${CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS3}
<#if CONFIG_SYS_MEMORY_SMC_DATA_BUS_CS3 == "SMC_DATA_BUS_WIDTH_16BIT">
#define SMC_BAT_CS3 ${CONFIG_SYS_MEMORY_SMC_BAT_CS3}
<#else>
#define SMC_BAT_CS3 0x0
</#if>
<#if CONFIG_SYS_MEMORY_SMC_TDF_MODE_CS3 == true>
#define SMC_TDF_MODE_CS3 0x1
#define SMC_TDF_CYCLES_CS3 ${CONFIG_SYS_MEMORY_SMC_TDF_CYCLES_CS3}
<#else>
#define SMC_TDF_MODE_CS3 0x0
#define SMC_TDF_CYCLES_CS3 0x0
</#if>
#define SMC_NWAIT_MODE_CS3 ${CONFIG_SYS_MEMORY_SMC_NWAIT_MODE_CS3}
<#if CONFIG_SYS_MEMORY_SMC_WRITE_MODE_CS3 == true >
#define SMC_WRITE_MODE_CS3 0x1
<#else>
#define SMC_WRITE_MODE_CS3 0x0
</#if>
<#if CONFIG_SYS_MEMORY_SMC_READ_MODE_CS3 == true>
#define SMC_READ_MODE_CS3 0x1
<#else>
#define SMC_READ_MODE_CS3 0x0
</#if>
</#if> // End Chip Select 3 settings

#endif // #ifndef _SYS_MEMORY_SMC_H

/*******************************************************************************
 End of File
*/
</#if>
