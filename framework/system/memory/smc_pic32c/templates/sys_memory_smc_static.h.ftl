/*******************************************************************
  Company:
    Microchip Technology Inc.
    Memory System Service SMC Initialization File

  File Name:
    sys_memory_smc_static.h.ftl

  Summary:
    Static Memory Controller (SMC).
	This file contains the source code to initialize the SMC controller

  Description:
    SMC configuration interface
    The SMC System memory interface provides a simple interface to manage 8-bit and 16-bit
    parallel devices.

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


#ifndef _SYS_MEMORY_SMC_STATIC_H
#define _SYS_MEMORY_SMC_STATIC_H

<#if CONFIG_USE_SYS_MEMORY_SMC == true>
#include "framework/system/memory/smc_pic32c/sys_memory_smc.h"
<#if CONFIG_USE_BSP== false>
void SYS_MEMORY_SMC_SetupPort(void);
</#if>
void SYS_MEMORY_SMC_Initialize(void);
</#if>
#endif // #ifndef _SYS_MEMORY_SMC_STATIC_H

/*******************************************************************************
 End of File
*/
