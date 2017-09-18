/*******************************************************************************
  SQI Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sqi_init.h

  Summary:
    SQI driver initialization data structures

  Description:
    This file contains the SQI driver's initialization data structures.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 - 2017 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_QSPI_INIT_H
#define _DRV_QSPI_INIT_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "arch/arm/devices_pic32c.h" /* PIC32C system header. */
#include "driver/sqi/drv_sqi.h"
//#include "system/int/sys_int.h"
//#include "osal/osal.h"
#include "system/debug/sys_debug.h"
#include "system/common/sys_queue.h"

// *****************************************************************************
/* QSPI Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the QSPI driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    QSPI driver.

  Remarks:
    Not all initialization features are available for all devices. Please
	refer to the specific device data sheet to determine availability.
*/
typedef struct
{
    qspi_registers_t *sqiId;
    uint8_t clockDivider;
    DRV_SQI_SPI_OPERATION_MODE spiMode;
	INT_SOURCE interruptSource;
} DRV_SQI_INIT;


#endif //#ifndef _DRV_QSPI_INIT_H

/*******************************************************************************
 End of File
*/

