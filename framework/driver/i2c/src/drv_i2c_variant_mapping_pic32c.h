/*******************************************************************************
  PIC32C I2C Driver Variant Mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_variant_mapping_pic32c.h

  Summary:
    PIC32C I2C Driver Variant Mapping

  Description:
    This file provides feature and build variant mapping macros allowing the
    driver to easily be built with different implementation variations based
    on static build-time configuration selections.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_I2C_FEATURE_MAPPING_PIC32C_H
#define _DRV_I2C_FEATURE_MAPPING_PIC32C_H


// *****************************************************************************
// *****************************************************************************
// Section: Interrrupt Variations
// *****************************************************************************
// *****************************************************************************
/* Mapping of the interrupt mode variations
*/

#if defined (DRV_I2C_INTERRUPT_MODE) && \
            (DRV_I2C_INTERRUPT_MODE == true)

    #define _DRV_I2C_InterruptSourceIsEnabled(source)     SYS_INT_SourceIsEnabled( source )
    #define _DRV_I2C_InterruptSourceEnable(source)        SYS_INT_SourceEnable( source )
    #define _DRV_I2C_InterruptSourceDisable(source)       SYS_INT_SourceDisable( source )
    #define _DRV_I2C_InterruptSourceStatusClear(source)   SYS_INT_SourceStatusClear( source )

    #define _DRV_I2C_SEM_POST(x)                          OSAL_SEM_PostISR(x)
    #define _DRV_I2C_TAKE_MUTEX(x,y)                      (OSAL_RESULT_TRUE)
    #define _DRV_I2C_RELEASE_MUTEX(x)

    #define _DRV_I2C_ALWAYS_NON_BLOCKING                  0

#elif defined (DRV_I2C_INTERRUPT_MODE) && \
            (DRV_I2C_INTERRUPT_MODE == false)

    /* Driver is configured for polled mode */
    #define _DRV_I2C_InterruptSourceIsEnabled(source)     false
    #define _DRV_I2C_InterruptSourceEnable(source)
    #define _DRV_I2C_InterruptSourceDisable(source)       false
    #define _DRV_I2C_InterruptSourceStatusClear(source)   SYS_INT_SourceStatusClear( source )
    #define _DRV_I2C_SEM_POST(x)                          OSAL_SEM_Post(x)
    #define _DRV_I2C_TAKE_MUTEX(x,y)                      OSAL_MUTEX_Lock(x,y)
    #define _DRV_I2C_RELEASE_MUTEX(x)                     OSAL_MUTEX_Unlock(x)

    #ifndef OSAL_USE_RTOS

        /* This means the driver is being built in a baremetal application.
           We cannot let a client open the driver in blocking mode */

           #define _DRV_I2C_ALWAYS_NON_BLOCKING           (DRV_IO_INTENT_NONBLOCKING)

    #else

        /* This means the driver is being built with RTOS support. We can
           support blocking operation */

            #define _DRV_I2C_ALWAYS_NON_BLOCKING          0

    #endif

#else
    #error "No Task mode chosen at build, interrupt or polling needs to be selected."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Buffer Queue support
// *****************************************************************************
// *****************************************************************************

#if defined (DRV_I2C_MASTER_MODE_INSTANCE) && \
            (DRV_I2C_MASTER_MODE_INSTANCE == true)

#define _DRV_I2C_MASTER_RECEIVE(l,m,n,o,p)                   _DRV_I2C_MasterReceive(l,m,n,o,p)
#define _DRV_I2C_MASTER_TRANSMIT(l,m,n,o,p,q)                _DRV_I2C_MasterTransmit(l,m,n,o,p,q)
#define _DRV_I2C_MASTER_TRANSMIT_THEN_RECEIVE(l,m,n,o,p,q,r) _DRV_I2C_MasterTransmitThenReceive(l,m,n,o,p,q,r)
#define _DRV_I2C_MASTER_BUFFER_QUEUE_TASKS(x)                _DRV_I2C_MasterTasks(x)

#elif defined (DRV_I2C_MASTER_MODE_INSTANCE) && \
              (DRV_I2C_MASTER_MODE_INSTANCE == false)

#define _DRV_I2C_MASTER_RECEIVE(l,m,n,o,p)                   DRV_I2C_BUFFER_HANDLE_INVALID
#define _DRV_I2C_MASTER_TRANSMIT(l,m,n,o,p,q)                DRV_I2C_BUFFER_HANDLE_INVALID
#define _DRV_I2C_MASTER_TRANSMIT_THEN_RECEIVE(l,m,n,o,p,q,r) DRV_I2C_BUFFER_HANDLE_INVALID
#define _DRV_I2C_MASTER_BUFFER_QUEUE_TASKS(x)

#endif

#if defined (DRV_I2C_SLAVE_MODE_INSTANCE) && \
            (DRV_I2C_SLAVE_MODE_INSTANCE == true)

#define _DRV_I2C_SLAVE_RECEIVE(l,m,n,o)                      _DRV_I2C_SlaveReceive(l,m,n,o)
#define _DRV_I2C_SLAVE_TRANSMIT(l,m,n,o)                     _DRV_I2C_SlaveTransmit(l,m,n,o)
#define _DRV_I2C_SLAVE_TRANSMIT_THEN_RECEIVE(l,m,n,o,p,q)    DRV_I2C_BUFFER_HANDLE_INVALID
#define _DRV_I2C_SLAVE_BUFFER_TASKS(x)                       _DRV_I2C_SlaveTasks(x)

#elif defined (DRV_I2C_SLAVE_MODE_INSTANCE) && \
              (DRV_I2C_SLAVE_MODE_INSTANCE == false)

#define _DRV_I2C_SLAVE_RECEIVE(l,m,n,o)                      DRV_I2C_BUFFER_HANDLE_INVALID
#define _DRV_I2C_SLAVE_TRANSMIT(l,m,n,o)                     DRV_I2C_BUFFER_HANDLE_INVALID
#define _DRV_I2C_SLAVE_TRANSMIT_THEN_RECEIVE(l,m,n,o,p,q)    DRV_I2C_BUFFER_HANDLE_INVALID
#define _DRV_I2C_SLAVE_BUFFER_TASKS(x)

#endif

#define _DRV_I2C_CLIENT_BUFFER_QUEUE_OBJECTS_REMOVE(x)       _DRV_I2C_ClientBufferQueueObjectsRemove(x)

// *****************************************************************************
// *****************************************************************************
// Initializtion Parameter Static Overrides
// *****************************************************************************
// *****************************************************************************


#endif //_DRV_I2C_FEATURE_MAPPING_PIC32C_H

/*******************************************************************************
 End of File
*/
