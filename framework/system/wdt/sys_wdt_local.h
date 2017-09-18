/*******************************************************************************
  System Watchdog Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    sys_wdt_local.h

  Summary:
    Watchdog Timer (WDT) System Service interface definition.

  Description:
    This file contains the interface definition for the WDT System Service.  It
    provides a way to interact with the WDT subsystem to manage the
    timing requests supported by the system.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED 'AS IS' WITHOUT WARRANTY OF ANY KIND,
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
#ifndef _SYS_WDT_LOCAL_H
#define _SYS_WDT_LOCAL_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "system/int/sys_int.h"
// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

/* SYS_WDT_TIMEOUT_CALLBACK

  Summary:
    Use to register a callback with the RTCC.

  Description:
    When the alarm is asserted, a callback can be activated. 
    Use SYS_RTCC_ALARM_CALLBACK as the function pointer to register the callback
    with the alarm.

  Remarks:
    The callback should look like: 
      void callback(context);
	Make sure the return value and parameters of the callback are correct.
*/

typedef void (*SYS_WDT_TIMEOUT_CALLBACK)(uintptr_t context);

/* Watchdog System service Instance Object

  Summary:
    Defines the object required for the maintenance of the hardware instance.

  Description:
    This defines the object required for the maintenance of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/
typedef struct
{

    /* Timeout Period Value */
    uint32_t                                                timeoutPeriod;
	
	/* Interrupt mode */
	bool													interruptMode;
	
	/* Reset Enable */
	bool													resetEnable;
	
    /* Interrupt Source for TMR Interrupt */
    INT_SOURCE                                              interruptSource;
	
	/*Callback for the watchdog timeout*/
	SYS_WDT_TIMEOUT_CALLBACK								callback;
	
	/*Event Context that will be passed to callback */
    
	uintptr_t 												context;
	
} SYS_WDT_MODULE_INSTANCE;

/*This function is called by the ISR to handle the WDT Timeout interrupt*/
void SYS_WDT_ProcessEvents(void);

#endif //#ifndef _SYS_WDT_LOCAL_H