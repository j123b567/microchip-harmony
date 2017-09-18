/*******************************************************************************
  AFEC Driver Local Header File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_afec_local.h

  Summary:
    AFEC Driver Local Header File

  Description:
    This file will provide enumerations and other dependencies
    which are internal to the AFEC driver implementation.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_AFEC_LOCAL_H
#define _DRV_AFEC_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include "driver/afec/drv_afec.h"
#include "system_config.h"
#include "system/system.h"
#include "driver/driver_common.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
/* AFEC Driver Object

  Summary:
    Defines the AFEC driver object

  Description:
    The driver object associates the driver to a hardware instance.
    It is also used for registering a callback function
    
  Remarks:
    None.
*/
typedef struct
{
    /* The module index associated with the object*/
    afec_registers_t *moduleId;

    /* The status of the driver */
    SYS_STATUS status;

    /* Flag to indicate this object is in use  */
    bool inUse;
    
    /* Interrupt Source */
    INT_SOURCE interruptSource;
    
    /* Mutex to control access to the drive object */
    //OSAL_MUTEX_DECLARE(mutexDriverInstance);
    
    /* Channel to channelset mapping */
    uint8_t channelSetMap[12];
    
    /* Channelset array associated with this driver object */
    DRV_AFEC_CHANNEL_SET * channelSetTable;
    
} DRV_AFEC_OBJ;

// *****************************************************************************
/* AFEC Driver Client Object

  Summary:
    Defines the AFEC driver client object

  Description:
    The client object holds details about the driver object used by the client
    the channel set used, and status of the client.
    
  Remarks:
    None.
*/
typedef struct _DRV_AFEC_CLIENT_OBJ
{
    /* Driver object associated with this client */
    int afecDrvIndex;

    /* This flags indicates if the object is in use or is available */
    bool inUse;

    /* Unique identifier to validate a client object */
    uint32_t uniqueClientIdentifier;
    
    /* Callback functions */
    DRV_AFEC_CONVERSION_COMPLETE_CALLBACK afecCallback;
    
} DRV_AFEC_CLIENT_OBJ;

extern void _DRV_AFEC_HardwareSetup(afec_registers_t *moduleId, DRV_AFEC_INIT * afec_init);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


#endif // #ifndef _DRV_AFEC_LOCAL_H
/*******************************************************************************
 End of File
*/

