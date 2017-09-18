/*******************************************************************************
  PIC32C I2S Driver Abstraction Layer Implementation Local Header File.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2s_pic32c_local.h

  Summary:
    PIC32C I2S Driver Abstraction Layer Local Header file.

  Description:
    This file contains data types which are local to the abstraction layer
    implementation.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

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
 ******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_I2S_PIC32C_LOCAL_H_
#define _DRV_I2S_PIC32C_LOCAL_H_

#include "system_config.h"
#include "driver/i2s/drv_i2s.h"
#include "osal/osal.h"

// *****************************************************************************
/* PIC32C I2S Driver Initialization Data Structure

  Summary:
    PIC32C I2S Driver Initialization Data Structure

  Description:
    This type defintions defines the type of the PIC32C I2S Driver
    Initialization data structure. A data structure of this type should be
    provide to the DRV_I2S_Initialize function when the a PIC32C driver needs to
    be initialized.

  Remarks:
    None.
*/

typedef struct
{
    /* Pointer to the underlying peripheral specific I2S driver initialization
     * data structure. */
    void * init;

    /* Pointer to the underlying peripheral specific I2S driver interface 
     * The pointed structure should contain function pointers for underlying 
     * driver layer */
    DRV_I2S_INTERFACE * driverInterface;

} DRV_I2S_INIT;

/********************************************
 * Abstraction Layer Driver Instance Object
 * type.
 ********************************************/

typedef struct
{
    /* True if the object is allocated */
    bool inUse;

    /* Status of this driver instance */
    SYS_STATUS status;
    /* Pointer to the interface of the underlying hardware specific I2S driver
     * */
    DRV_I2S_INTERFACE * driverInterface;

    /* Driver handles */
    DRV_HANDLE driverHandles[DRV_I2S_CLIENTS_NUMBER];

    /* System Module Index returned by the intialize function of the underlying
     * driver. */
    SYS_MODULE_OBJ moduleObj;

    /* Mutex for the driver handles */
    OSAL_MUTEX_DECLARE(mutexDriverHandle);

} DRV_I2S_OBJ;

#endif
