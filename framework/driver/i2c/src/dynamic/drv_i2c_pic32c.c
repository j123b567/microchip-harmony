/*******************************************************************************
  I2C Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_pic32c.c

  Summary:
    I2C Device Driver Dynamic Multiple Client Implementation

  Description:
    The I2C device driver provides a simple interface to manage the I2C
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines for the I2C driver.

    While building the driver from source, ALWAYS use this file in the build.
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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "../drv_i2c_local_pic32c.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array. */
DRV_I2C_OBJ             gDrvI2CObj      [ DRV_I2C_INSTANCES_NUMBER ] ;

/* This is the client object array. */
DRV_I2C_CLIENT_OBJ      gDrvI2CClientObj[ DRV_I2C_CLIENTS_NUMBER ];

/* This object maintains data that is required by all I2C
   driver instances. */
DRV_I2C_COMMON_DATA_OBJ gDrvI2CCommonDataObj;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

SYS_MODULE_OBJ DRV_I2C_Initialize ( const SYS_MODULE_INDEX    index,
                                    const SYS_MODULE_INIT * const init )
{
    DRV_I2C_OBJ  * drvObj  = ( DRV_I2C_OBJ * )  NULL;
    DRV_I2C_INIT * i2cInit = ( DRV_I2C_INIT * ) NULL;
    
    /* Check if the specified driver index is in valid range */
    if ( index >= DRV_I2C_INSTANCES_NUMBER )
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    
    /* Check if the init data pointer is valid */
    if( (SYS_MODULE_INIT *)NULL == init )
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    
    /* Check if this hardware instance was already initialized */
    if ( true == gDrvI2CObj[index].inUse )
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    
    /* Assign to the local pointer the init data passed */
    i2cInit = ( DRV_I2C_INIT * ) init; 
    
    /* Check if the module id is set to valid id */
    if( 0 == i2cInit->i2cId )
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    
    /* Disable the I2C Module */
    _DRV_I2C_ResetAndDisable( (twi_registers_t *)i2cInit->i2cId ); 
    
    /* Allocate the driver object and set the operation flag to be in use */
    drvObj = &gDrvI2CObj[index];
    drvObj->inUse = true;
    
    /* Update the I2C parameters. */
    drvObj->nClients              = 0;
    drvObj->i2cModule             = (twi_registers_t *) i2cInit->i2cId;
    drvObj->operationMode         = i2cInit->i2cMode;
    drvObj->queueSize             = i2cInit->queueSize;
    drvObj->interruptSource       = i2cInit->interruptSource;
    drvObj->operationStarting     = NULL;
    drvObj->isExclusive           = false;
    drvObj->interruptNestingCount = 0;
    drvObj->queueSizeCurrent      = 0;
    drvObj->queue                 = NULL;
    
    /* Initialize the task state based on Driver Mode */
    if( DRV_I2C_MODE_MASTER == drvObj->operationMode )
    {
        drvObj->taskState = DRV_I2C_DATA_OBJ_TASK_ADDRESS_SEND;
    }
    else
    {
        drvObj->taskState = DRV_I2C_DATA_OBJ_TASK_ADDRESS_RECEIVED;
    }
    
    /* Initialize the I2C Module */
    if ( !_DRV_I2C_HardwareInitialize( (twi_registers_t *) i2cInit->i2cId, i2cInit ) )
    {
        drvObj->inUse = false;
        return SYS_MODULE_OBJ_INVALID;
    }
    
    /* Clear Interrupt status */
    SYS_INT_SourceStatusClear  ( drvObj->interruptSource );
    
    /* Enable the interrupt the source in case of interrupt mode */
    _DRV_I2C_InterruptSourceEnable( drvObj->interruptSource );
    
    /* Create the hardware instance mutex. */
    if(OSAL_MUTEX_Create(&(drvObj->mutexDriverInstance)) != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Check if the global mutexes have been created. If not
       then create these. */

    if(!gDrvI2CCommonDataObj.membersAreInitialized)
    {
         /* This means that mutexes where not created. Create them. */
        if(OSAL_MUTEX_Create(&(gDrvI2CCommonDataObj.mutexClientObjects)) != OSAL_RESULT_TRUE)
        {
                 return SYS_MODULE_OBJ_INVALID;
        }
        if(OSAL_MUTEX_Create(&(gDrvI2CCommonDataObj.mutexBufferQueueObjects)) != OSAL_RESULT_TRUE)
        {
                 return SYS_MODULE_OBJ_INVALID;
        }
         
        /* Set this flag so that global mutexes get allocated only once */
        gDrvI2CCommonDataObj.membersAreInitialized = true;
    }
    
    /* Enable the I2c Module */
    _DRV_I2C_Enable( (twi_registers_t *)i2cInit->i2cId, i2cInit->i2cMode );
            
    /* Update the status */
    drvObj->status = SYS_STATUS_READY;
    
    /* Return the object structure */
    return ( (SYS_MODULE_OBJ)index );
}

// *****************************************************************************
/* Function:
    void DRV_I2C_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_I2C_Deinitialize system interface function.

  Description:
    This is the dynamic implementation of DRV_I2C_Deinitialize system interface
    function.

  Remarks:
    See drv_i2c.h for usage information.
*/

void  DRV_I2C_Deinitialize(SYS_MODULE_OBJ object)
{
    DRV_I2C_OBJ * dObj;
    DRV_I2C_BUFFER_OBJ * iterator;

    bool status;

    /* Check that the object is valid */
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        return;
    }

    if(object >= DRV_I2C_INSTANCES_NUMBER)
    {
        return;
    }

    dObj = (DRV_I2C_OBJ*) &gDrvI2CObj[object];

    if(!dObj->inUse)
    {
        return;
    }

    /* The driver will not have clients when it is
       being deinitialized. So the order in which
       we do the following steps is not that important */

    /* Indicate that this object is not is use */
    dObj->inUse = false;

    /* Deinitialize the I2C status */
    dObj->status =  SYS_STATUS_UNINITIALIZED ;

    /* Disable the interrupt */
    status = _DRV_I2C_InterruptSourceDisable(dObj->interruptSource) ;
    
    /* Ignore the warning */
    (void)status;

    /* Disable I2C module */
    _DRV_I2C_ResetAndDisable ((twi_registers_t *)dObj->i2cModule);

    /* Deallocate all mutexes */
    if(OSAL_MUTEX_Delete(&(dObj->mutexDriverInstance)) != OSAL_RESULT_TRUE)
    {
        return;
    }

    /* Remove all objects from the read and write queue */
    iterator = dObj->queue;
    while(iterator != NULL)
    {
        /* Return the buffer object to the pool */
        iterator->inUse = false;
        iterator = iterator->next;
    }
    
    return;
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_I2C_Status( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_I2C_Status system interface function.

  Description:
    This is the dynamic implementation of DRV_I2C_Status system interface
    function.

  Remarks:
    See drv_i2c.h for usage information.
*/

SYS_STATUS DRV_I2C_Status( SYS_MODULE_OBJ object)
{
    /* Check if we have a valid object */
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        return(SYS_STATUS_UNINITIALIZED);
    }

    if(object > DRV_I2C_INSTANCES_NUMBER)
    {
        return(SYS_STATUS_UNINITIALIZED);
    }

    /* Return the system status of the hardware instance object */
    return (gDrvI2CObj[object].status);
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_I2C_Open( const SYS_MODULE_INDEX index,
                             const DRV_IO_INTENT    ioIntent )

  Summary:
    Dynamic implementation of DRV_I2C_Open client interface function.

  Description:
    This is the dynamic implementation of DRV_I2C_Open client interface
    function.

  Remarks:
    See drv_i2c.h for usage information.
*/

DRV_HANDLE DRV_I2C_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_I2C_CLIENT_OBJ *clientObj;
    DRV_I2C_OBJ *dObj;
    unsigned int iClient;

    if (drvIndex >= DRV_I2C_INSTANCES_NUMBER)
    {
        /* Invalid driver index */
        return (DRV_HANDLE_INVALID);
    }

    dObj = &gDrvI2CObj[drvIndex];

    if((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false))
    {
        /* The I2C module should be ready */
        return DRV_HANDLE_INVALID;
    }

    if(dObj->isExclusive)
    {
        /* This means the another client has opened the driver in exclusive
           mode. The driver cannot be opened again */
        return ( DRV_HANDLE_INVALID ) ;
    }

    if((dObj->nClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        /* This means the driver was already opened and another driver was
           trying to open it exclusively.  We cannot give exclusive access in
           this case */
        return(DRV_HANDLE_INVALID);
    }

    /* Grab client object mutex here */
    if(OSAL_MUTEX_Lock(&(gDrvI2CCommonDataObj.mutexClientObjects), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Enter here only if the lock was obtained (applicable in
           RTOS only). If the mutex lock fails due to time out then
           this code does not get executed */

        for(iClient = 0; iClient != DRV_I2C_CLIENTS_NUMBER; iClient ++)
        {
            if(!gDrvI2CClientObj[iClient].inUse)
            {
                /* This means we have a free client object to use */
                clientObj = &gDrvI2CClientObj[iClient];
                clientObj->inUse        = true;

                /* We have found a client object. Release the mutex */

                OSAL_MUTEX_Unlock(&(gDrvI2CCommonDataObj.mutexClientObjects));

                clientObj->hDriver      = dObj;

                /* In a case where the driver is configured for polled
                   and bare metal operation, it will not support blocking operation */

                clientObj->ioIntent     = (ioIntent | _DRV_I2C_ALWAYS_NON_BLOCKING);
                clientObj->eventHandler = NULL;
                clientObj->context      = (uintptr_t)NULL;
                clientObj->error        = DRV_I2C_ERROR_NONE;

                if(ioIntent & DRV_IO_INTENT_EXCLUSIVE)
                {
                    /* Set the driver exclusive flag */
                    dObj->isExclusive = true;
                }

                dObj->nClients ++;

                /* Create the semaphores */
                if(OSAL_SEM_Create(&(clientObj->semTransferDone), OSAL_SEM_TYPE_COUNTING, 1, 0) != OSAL_RESULT_TRUE)
                {
                    return(DRV_HANDLE_INVALID);
                }
                
                /* Update the client status */
                clientObj->status = DRV_I2C_CLIENT_STATUS_READY;
                return ((DRV_HANDLE) clientObj );
            }
        }

        /* Could not find a client object. Release the mutex and
           return with an invalid handle. */
        OSAL_MUTEX_Unlock(&(gDrvI2CCommonDataObj.mutexClientObjects));
    }

    /* If we have reached here, it means either we could not find a spare
       client object or the mutex timed out in a RTOS environment. */

    return DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    DRV_I2C_STATUS DRV_I2C_Close ( DRV_HANDLE handle )

  Summary:
    Dynamic implementation of DRV_I2C_Close client interface function.

  Description:
    This is the dynamic implementation of DRV_I2C_Close client interface
    function.

  Remarks:
    See drv_i2c.h for usage information.
*/

void DRV_I2C_Close ( DRV_HANDLE handle)
{
    /* This function closes the client, The client
       object is deallocated and returned to the
       pool. */

    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ * dObj;

    /* Validate the handle */
    clientObj = _DRV_I2C_DriverHandleValidate(handle);

    if(clientObj == NULL)
    {
        /* Driver handle is not valid */
        return;
    }

    dObj = (DRV_I2C_OBJ *)clientObj->hDriver;

    /* Remove all buffers that this client owns from the driver queue. This
       function will map to _DRV_I2C_ClientBufferQueueObjectsRemove() if the
       driver was built for buffer queue support. Else this condition always
       maps to true. */

    if(!_DRV_I2C_CLIENT_BUFFER_QUEUE_OBJECTS_REMOVE(clientObj))
    {
        /* The function could fail if the mutex time out occurred */
        clientObj->status = DRV_I2C_CLIENT_STATUS_ERROR;
        return;
    }

    /* Deallocate all semaphores */
    if(OSAL_SEM_Delete(&(clientObj->semTransferDone)) != OSAL_RESULT_TRUE)
    {
        clientObj->status = DRV_I2C_CLIENT_STATUS_ERROR;
        return;
    }
    
    /* Reduce the number of clients */
    dObj->nClients --;

    /* Reset the exclusive flag */
    dObj->isExclusive = false;

    /* De-allocate the object */
    clientObj->status = DRV_I2C_CLIENT_STATUS_CLOSED;
    clientObj->inUse = false;

    return;
}

/*******************************************************************************
  Function:
    DRV_I2C_BUFFER_HANDLE DRV_I2C_Receive   (   DRV_HANDLE handle,
                                                uint16_t slaveaddress,
                                                void *rxBuffer,
                                                size_t size,
                                                void * callbackContext )

  Summary:
    This function reads data written from either Master or Slave.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Master calls this function to read data send by Slave. The Slave calls this
    function to read data send by Master.
    In case of Master, a START condition is initiated on the I2C bus.

  Precondition:
    The DRV_I2C_Initialize routine must have been called for the specified I2C
    device instance and the DRV_I2C_Status must have returned SYS_STATUS_READY.

    DRV_I2C_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's open
                  routine
    address     - Device address of slave. If this API is used in Slave mode,
                  then a dummy value can be used
    buffer      - This buffer holds data is received
    size        - The number of bytes that the Master expects to read from Slave.
                  This value can be kept as the MAX BUFFER SIZE for slave.
                  This is because the Master controls when the READ operation
                  is terminated.
    callbackContext     - Not implemented, future expansion

  Returns:
    A valid BUFFER HANDLE, NULL if the handle is not obtained.

  Remarks:
    See drv_i2c.h for usage information.
*/

DRV_I2C_BUFFER_HANDLE DRV_I2C_Receive (   DRV_HANDLE handle, 
                                          uint16_t   slaveAddress,
                                          void *     buffer,
                                          size_t     size,
                                          void *     callbackContext )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    
        /* Validate the driver handle */
    clientObj = _DRV_I2C_DriverHandleValidate(handle);
    if(clientObj == NULL)
    {
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    if((size == 0) || (NULL == buffer))
    {
        /* We either got an invalid source pointer or 0 bytes to transfer */
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    hDriver   = clientObj->hDriver;
    
    /* Call the local receive function based on operation mode. */
    if( DRV_I2C_MODE_MASTER == hDriver->operationMode )
    {
        return (_DRV_I2C_MASTER_RECEIVE(handle,slaveAddress,buffer,size,callbackContext));
    }
    else
    {
        return (_DRV_I2C_SLAVE_RECEIVE(handle,buffer,size,callbackContext));
    }
}

/*******************************************************************************
  Function:
    DRV_I2C_BUFFER_HANDLE DRV_I2C_Transmit( DRV_HANDLE  handle, 
                                            uint16_t    slaveaddress,
                                            void       *txBuffer,
                                            size_t      size,
                                            void       *context);

  Summary:
    This function writes data to Master or Slave.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Master calls this function to write data to Slave. The Slave calls this 
    function to write data to Master.

  Precondition:
    The DRV_I2C_Initialize routine must have been called for the specified I2C
    device instance and the DRV_I2C_Status must have returned SYS_STATUS_READY.

    DRV_I2C_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's open
                  routine
    address     - Device address of slave. If this API is used in Slave mode,
                  then a dummy value can be used
    buffer      - Contains data to be transferred
    size        - The number of bytes that the Master expects to write to Slave.
                  This value can be kept as the MAX BUFFER SIZE for slave.
                  This is because the Master controls when the WRITE operation
                  is terminated.
    callbackContext     - Not implemented, future expansion

  Returns:
    A valid BUFFER HANDLE, NULL if the handle is not obtained.

  Remarks:
    See drv_i2c.h for usage information.  
*/

DRV_I2C_BUFFER_HANDLE DRV_I2C_Transmit (  DRV_HANDLE handle, 
                                          uint16_t   slaveAddress,
                                          void *     buffer,
                                          size_t     size,
                                          void *     context )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    
        /* Validate the driver handle */
    clientObj = _DRV_I2C_DriverHandleValidate(handle);
    if(clientObj == NULL)
    {
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    if((size == 0) || (NULL == buffer))
    {
        /* We either got an invalid source pointer or 0 bytes to transfer */
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    hDriver = clientObj->hDriver;
    
    /* Call the local transmit function based on operation mode. */
    if( DRV_I2C_MODE_MASTER == hDriver->operationMode )
    {
        return (_DRV_I2C_MASTER_TRANSMIT(handle,slaveAddress,buffer,size,DRV_I2C_HALT_ON_ERROR,context));
    }
    else
    {
        return (_DRV_I2C_SLAVE_TRANSMIT(handle,buffer,size,context));
    }
}

/*******************************************************************************
  Function:
    DRV_I2C_BUFFER_HANDLE DRV_I2C_TransmiThenReceive ( DRV_HANDLE handle,
                                                       uint16_t   deviceaddress,
                                                       void       *txBuffer,
                                                       size_t     writeSize,
                                                       void       *rxBuffer,
                                                       size_t     readSize,
                                                       void       *context)

  Summary:
    This function writes data to Slave, inserts restart and requests read from
    slave.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Master calls this function to send a register address value to the slave and
    then queries the slave with a read request to read the contents indexed by
    the register location. The Master sends a restart condition after the
    initial write before sending the device address with R/W = 1. The restart
    condition prevents the Master from relinquishing the control of the bus. The
    slave should not use this function.

  Precondition:
    The DRV_I2C_Initialize routine must have been called for the specified I2C
    device instance and the DRV_I2C_Status must have returned SYS_STATUS_READY.

    DRV_I2C_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's open
                  routine
    address     - Device address of slave. If this API is used in Slave mode,
                  then a dummy value can be used
    writeBuffer - Contains data to be transferred
    writeSize   - The number of bytes that the Master expects to write to Slave.
                  This value can be kept as the MAX BUFFER SIZE for slave.
                  This is because the Master controls when the WRITE operation
                  is terminated.
    readBuffer  - This buffer holds data that is send back from slave after
                  read operation.
    readSize       - The number of bytes the Master expects to be read from the
                  slave
    callbackContext     - Not implemented, future expansion

  Returns:
    A valid BUFFER HANDLE, NULL if the handle is not obtained.

    </code>

  Remarks:
    See drv_i2c.h for usage information.
*/

DRV_I2C_BUFFER_HANDLE DRV_I2C_TransmitThenReceive (  DRV_HANDLE handle, 
                                                     uint16_t   slaveAddress,
                                                     void *     writeBuffer,
                                                     size_t     writeSize,
                                                     void *     readBuffer,
                                                     size_t     readSize,
                                                     void *     callbackContext )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    
        /* Validate the driver handle */
    clientObj = _DRV_I2C_DriverHandleValidate(handle);
    if(clientObj == NULL)
    {
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    if((readSize == 0)        || (writeSize == 0) ||
       (NULL  == writeBuffer) || (NULL == readBuffer))
    {
        /* We either got an invalid source pointer or 0 bytes to transfer */
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }
    
    hDriver   = clientObj->hDriver;
    
    /* Call the local Transmit Then Receive function (Applicable for Master Mode only). */
    if( DRV_I2C_MODE_MASTER == hDriver->operationMode )
    {
        return (_DRV_I2C_MASTER_TRANSMIT_THEN_RECEIVE(handle,slaveAddress,writeBuffer,writeSize,readBuffer,readSize,callbackContext));
    }
    else
    {
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }
}

/*******************************************************************************
  Function:
DRV_I2C_BUFFER_HANDLE DRV_I2C_TransmitForced ( DRV_HANDLE handle, 
                                               uint16_t deviceaddress, 
                                               uint8_t* txBuffer,   
                                               uint16_t txbuflen, 
                                               DRV_I2C_BUS_ERROR_EVENT eventFlag, 
                                               void * calbackContext)

  Summary:
    This function writes data to Master or Slave.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Master calls this function to transmit the entire buffer to the slave even
    if the slave ACKs or NACKs the address or any of the data bytes. This is 
    typically used for slaves that have to initiate a reset sequence by sending
    a dummy I2C transaction. Since the slave is still in reset, any or all the 
    bytes can be NACKed. In the normal operation of the driver if the address 
    or data byte is NACKed, then the transmission is aborted and a STOP condition
    is asserted on the bus.

  Precondition:
    The DRV_I2C_Initialize routine must have been called for the specified I2C
    device instance and the DRV_I2C_Status must have returned SYS_STATUS_READY.

    DRV_I2C_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's open
                  routine
    address     - Device address of slave. If this API is used in Slave mode,
                  then a dummy value can be used
    buffer      - Contains data to be transferred
    size        - The number of bytes that the Master expects to write to Slave.
                  This value can be kept as the MAX BUFFER SIZE for slave.
                  This is because the Master controls when the WRITE operation
                  is terminated.
    eventFlag   - This field is left for future implementation
    callbackContext     - Not implemented, future expansion

  Returns:
    A valid BUFFER HANDLE, NULL if the handle is not obtained.

  Remarks:
    The handle that is passed into the function, drvI2CHandle is obtained by 
    calling the DRV_I2C_OPEN function.  If the function could not return a 
    valid buffer handle, then a NULl value is returned.
    Once all the bytes are transferred the buffer status is set as             
    then DRV_I2C_BUFFER_EVENT_COMPLETE .  
*/

DRV_I2C_BUFFER_HANDLE DRV_I2C_TransmitForced ( DRV_HANDLE handle, 
                                               uint16_t deviceaddress, 
                                               void* txBuffer,   
                                               size_t txbuflen, 
                                               DRV_I2C_BUS_ERROR_EVENT eventFlag, 
                                               void * calbackContext)
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    
    /* Validate the driver handle */
    clientObj = _DRV_I2C_DriverHandleValidate(handle);
    if(clientObj == NULL)
    {
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    if((txbuflen == 0) || (NULL == txBuffer))
    {
        /* We either got an invalid source pointer or 0 bytes to transfer */
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    hDriver   = clientObj->hDriver;
    
    /* Call the local Forced Transmit function (Applicable for Master Mode only). */
    if( DRV_I2C_MODE_MASTER == hDriver->operationMode )
    {
        return (_DRV_I2C_MASTER_TRANSMIT(handle,deviceaddress,txBuffer,txbuflen,eventFlag,calbackContext));
    }
    else
    {
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }
}

// *****************************************************************************
/* Function:
    void DRV_I2C_BufferEventHandlerSet
    (
        const DRV_HANDLE hClient,
        const DRV_I2C_BUFFER_EVENT_HANDLER eventHandler,
        const uintptr_t context
    )

  Summary:
    Dynamic implementation of DRV_I2C_BufferEventHandlerSet client interface
    function.

  Description:
    This is the dynamic implementation of DRV_I2C_BufferEventHandlerSet
    client interface function.

  Remarks:
    See drv_i2c.h for usage information.
*/

void DRV_I2C_BufferEventHandlerSet
(
    const DRV_HANDLE handle,
    const DRV_I2C_BUFFER_EVENT_HANDLER eventHandler,
    const uintptr_t context
)
{
    DRV_I2C_CLIENT_OBJ * clientObj;

    /* Validate the driver handle */
    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        return;
    }

    clientObj = (DRV_I2C_CLIENT_OBJ *)handle;

    if(!clientObj->inUse)
    {
        return;
    }

    /* Register the event handler with the client */
    clientObj->eventHandler = eventHandler;
    clientObj->context = context;
}

// *****************************************************************************
/* Function:
    void DRV_I2C_Tasks ( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_I2C_Tasks system interface function.

  Description:
    This is the dynamic implementation of DRV_I2C_Tasks system interface
    function.

  Remarks:
    See drv_i2c.h for usage information.
*/

void DRV_I2C_Tasks( SYS_MODULE_OBJ object )
{
    /* This is the I2C Driver tasks routine.
       In this function, the driver checks if a i2c 
       interrupt is active and if there are any buffers in
       queue. If so the buffer is serviced. A buffer that
       is serviced completely is removed from the queue.
     */

    DRV_I2C_OBJ * hDriver = &gDrvI2CObj[object];

    if((!hDriver->inUse) || (hDriver->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't
         * do anything */
        return;
    }

    /* The I2C driver is configured to generate an
       interrupt when the FIFO is empty. Additionally
       the queue is not empty. Which means there is
       work to done in this routine. */

    if( DRV_I2C_MODE_MASTER == hDriver->operationMode )
    {
        _DRV_I2C_MASTER_BUFFER_QUEUE_TASKS(object);
    }
    else
    {
        _DRV_I2C_SLAVE_BUFFER_TASKS(object);
    }

    /* Clear up the interrupt flag */
    SYS_INT_SourceStatusClear(hDriver->interruptSource);

}

// *****************************************************************************
/* Function:
    void DRV_I2C_QueueFlush ( DRV_HANDLE handle )

  Summary:
    The existing transactions in the queue are voided and the queue   
    pointers are reset to their initial state. This renders the queue
    empty.

  Description:
    The existing transactions in the queue are voided and the queue   
    pointers are reset to their initial state. This renders the queue
    empty.

  Parameters:
    handle          -  A valid open-instance handle, returned from the driver's 
                       open routine 

  Returns:
    None
  
  Remarks:
    See drv_i2c.h for usage information.

*/
void DRV_I2C_QueueFlush ( DRV_HANDLE handle )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    
    /* Validate the handle */
    clientObj = _DRV_I2C_DriverHandleValidate(handle);

    if(clientObj == NULL)
    {
        /* Driver handle is not valid */
        return;
    }
    
    /* Remove all buffers that this client owns from the driver queue. This
       function will map to _DRV_I2C_ClientBufferQueueObjectsRemove() if the
       driver was built for buffer queue support. Else this condition always
       maps to true. */
    if(!_DRV_I2C_CLIENT_BUFFER_QUEUE_OBJECTS_REMOVE(clientObj))
    {
        /* The function could fail if the mutex time out occurred */
        clientObj->status = DRV_I2C_CLIENT_STATUS_ERROR;
        return;
    }
}

// *****************************************************************************
/* Function:
    uint32_t DRV_I2C_BytesTransferred ( DRV_I2C_BUFFER_HANDLE bufferHandle )

  Summary:
    Returns the number of bytes transmitted or received in a particular I2C 
    transaction. The transaction is identified by the handle.

  Description:
    This returns the transmitter and receiver transfer status.

  Parameters:
    handle          -  A valid open-instance handle, returned from the driver's 
                       open routine 
    bufferHandle    -  A valid buffer handle obtained when calling   
                       Transmit/Receive/TransmitThenReceive/TransmitForced or
                       BufferAddRead/BufferAddWrite/BufferAddReadWrite function

  Returns:
    The number of bytes transferred in a particular I2C transaction. 
 
  Remarks:
    See drv_i2c.h for usage information.

*/

uint32_t DRV_I2C_BytesTransferred ( DRV_HANDLE handle,
                                    DRV_I2C_BUFFER_HANDLE bufferHandle )
{
    DRV_I2C_BUFFER_OBJ * bufferObj;
    
    if( 0 == bufferHandle || DRV_I2C_BUFFER_HANDLE_INVALID == bufferHandle )
    {
        return 0;
    }
    
    bufferObj = ( DRV_I2C_BUFFER_OBJ *) bufferHandle;
    
    return (bufferObj->nCurrentBytes);

}

/*****************************************************************************
  Function:
    DRV_I2C_BUFFER_EVENT DRV_I2C_TransferStatusGet ( DRV_HANDLE handle,
                             DRV_I2C_BUFFER_HANDLE bufferHandle )

  Summary:
    Returns status of data transfer when Master or Slave acts either as a
    transmitter or a receiver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    The bufferHandle parameter contains the buffer handle of the buffer that
    associated with the event.
    If the event is DRV_I2C_BUFFER_EVENT_COMPLETE, it means that the data was
    transferred successfully.
    If the event is DRV_I2C_BUFFER_EVENT_ERROR, it means that the data was not
    transferred successfully.

  Parameters:
    handle          -  A valid open-instance handle, returned from the driver's 
                       open routine 
    bufferHandle    -  A valid buffer handle obtained when calling   
                       Transmit/Receive/TransmitThenReceive/TransmitForced or
                       BufferAddRead/BufferAddWrite/BufferAddReadWrite function
  
  Returns:
    A DRV_I2C_TRANSFER_STATUS value describing the current status of the
    transfer.
   
   Remarks:
    See drv_i2c.h for usage information.

*/

DRV_I2C_BUFFER_EVENT DRV_I2C_TransferStatusGet ( DRV_HANDLE handle,
                             DRV_I2C_BUFFER_HANDLE bufferHandle )
{
    DRV_I2C_BUFFER_OBJ * bufferObj;

    if( bufferHandle == 0 || bufferHandle == DRV_I2C_BUFFER_HANDLE_INVALID )
    {
        return DRV_I2C_BUFFER_EVENT_ERROR_INVALID_HANDLE;
    }
    
    bufferObj = ( DRV_I2C_BUFFER_OBJ * ) bufferHandle;
    
    return(bufferObj->event);
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

void _DRV_I2C_ResetAndDisable( twi_registers_t * i2cModule )
{    
    /* Disable the I2C Interrupts */
    i2cModule->TWI_IDR.w = ~0UL;
    
    /* Status register dummy read */
    i2cModule->TWI_SR.w;
    
    /* Reset the i2c Module */
    i2cModule->TWI_CR.w = TWI_CR_SWRST_Msk;
    i2cModule->TWI_RHR.w;
        
    /* Disable the I2C Master/Slave Mode */
    i2cModule->TWI_CR.w = TWI_CR_MSDIS_Msk | TWI_CR_SVDIS_Msk;
    
    return;
}

bool _DRV_I2C_BaudRateSet( twi_registers_t * i2cModule, uint32_t baudRate )
{
    uint32_t ckdiv = 0;
	uint32_t c_lh_div;
	uint32_t cldiv, chdiv;
    uint32_t clkSpeed;

    /* Set Clock */
    if( DRV_I2C_MASTER_MAX_BAUDRATE < baudRate  )
    {
        return (false);
    }
    
    /* Get Value of clock speed */
    clkSpeed = SYS_CLK_FrequencyGet(SYS_CLK_MASTER);
    
	/* Low level time not less than 1.3us of I2C Fast Mode. */
	if ( baudRate > DRV_I2C_LOW_LEVEL_TIME_LIMIT ) 
    {
		/* Low level of time fixed for 1.3us. */
		cldiv = clkSpeed / ( DRV_I2C_LOW_LEVEL_TIME_LIMIT * 
                             DRV_I2C_CLK_DIVIDER ) - 
                             DRV_I2C_CLK_CALC_ARGU;
        
		chdiv = clkSpeed / (( baudRate + 
                            ( baudRate - DRV_I2C_LOW_LEVEL_TIME_LIMIT)) * 
                              DRV_I2C_CLK_DIVIDER ) - 
                              DRV_I2C_CLK_CALC_ARGU;
		
		/* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
		while (( cldiv > DRV_I2C_CLK_DIV_MAX ) && 
               ( ckdiv < DRV_I2C_CLK_DIV_MIN )) 
        {
			/* Increase clock divider */
			ckdiv++;
            
			/* Divide cldiv value */
			cldiv /= DRV_I2C_CLK_DIVIDER;
		}
        
		/* chdiv must fit in 8 bits, ckdiv must fit in 3 bits */
		while (( chdiv > DRV_I2C_CLK_DIV_MAX ) && 
               ( ckdiv < DRV_I2C_CLK_DIV_MIN )) 
        {
			/* Increase clock divider */
			ckdiv++;
            
			/* Divide cldiv value */
			chdiv /= DRV_I2C_CLK_DIVIDER;
		}

		/* set clock waveform generator register */
        i2cModule->TWI_CWGR.w = ( TWI_CWGR_HOLD_Msk & i2cModule->TWI_CWGR.w ) |
                                  ( TWI_CWGR_CLDIV(cldiv) | 
                                    TWI_CWGR_CHDIV(chdiv) |
                                    TWI_CWGR_CKDIV(ckdiv) );
	} 
    else 
    {
		c_lh_div = clkSpeed / ( baudRate * DRV_I2C_CLK_DIVIDER ) - 
                   DRV_I2C_CLK_CALC_ARGU;

		/* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
		while (( c_lh_div > DRV_I2C_CLK_DIV_MAX ) && 
               ( ckdiv < DRV_I2C_CLK_DIV_MIN )) 
        {
			/* Increase clock divider */
			ckdiv++;
            
			/* Divide cldiv value */
			c_lh_div /= DRV_I2C_CLK_DIVIDER;
		}

		/* set clock waveform generator register */
        i2cModule->TWI_CWGR.w = ( TWI_CWGR_HOLD_Msk & i2cModule->TWI_CWGR.w ) |
                                  ( TWI_CWGR_CLDIV(c_lh_div) | 
                                    TWI_CWGR_CHDIV(c_lh_div) |
                                    TWI_CWGR_CKDIV(ckdiv) )  ;
	}
    
    return (true);
}

bool _DRV_I2C_HardwareInitialize( twi_registers_t * i2cModule, DRV_I2C_INIT *i2cInit )
{   
    switch( i2cInit->i2cMode )
    {
        case DRV_I2C_MODE_MASTER:
        {   
            /* Calculate value of TWI_CWGR */
            if(!_DRV_I2C_BaudRateSet( i2cModule, i2cInit->baudRate ))
            {
                return (false);
            }
            
            /* Starts the transfer by clearing the transmit hold register  */
            i2cModule->TWI_CR.w = TWI_CR_THRCLR_Msk;
            
            /* Enables interrupt on nack and arbitration lost */
            i2cModule->TWI_IER.w = TWI_IER_NACK_Msk | TWI_IER_ARBLST_Msk;
            
            break;
        }
        
        case DRV_I2C_MODE_SLAVE:
        {
            i2cModule->TWI_SMR.w   = 0;
            i2cModule->TWI_FILTR.w = 0;
            
            /* Enable High-Speed mode if selected in MHC */
            if( true == i2cInit->highSpeedMode )
            {
                i2cModule->TWI_CR.w = TWI_CR_HSEN_Msk;
                
                /* Enable the Pad filter and 
                   disable the input filtering */
                i2cModule->TWI_FILTR.w |= TWI_FILTR_PADFEN_Msk;
				i2cModule->TWI_FILTR.w &= ~TWI_FILTR_FILT_Msk;
            }
            else
            {
                i2cModule->TWI_CR.w = TWI_CR_HSDIS_Msk;
            }
            
            /* Set slave address and address mask in slave mode */
            
	        i2cModule->TWI_SMR.w = (~(TWI_SMR_SADR_Msk | TWI_SMR_MASK_Msk) & i2cModule->TWI_SMR.w) |
                                     ( TWI_SMR_SADR(((uint8_t *)&i2cInit->slaveaddvalue)[0] >> 1)        |
                                       TWI_SMR_MASK(i2cInit->maskslaveaddress));
            
            /* Disbale the clockstretch feature */
            if( false == i2cInit->clockStretchEnable )
            {
                i2cModule->TWI_SMR.w |= TWI_SMR_SCLWSDIS_Msk;
                
                /* Enable Underrun and overrun interrupts */
                i2cModule->TWI_IER.w = TWI_IER_UNRE_Msk | TWI_IER_OVRE_Msk;
            }
            
            /* Enable Slave access, End of Slave access and NACK interrupts */
            i2cModule->TWI_IER.w = TWI_IER_SVACC_Msk | TWI_IER_EOSACC_Msk | TWI_IER_NACK_Msk;
            
            break;
        }
    }
    
    return (true);
}

void _DRV_I2C_Enable( twi_registers_t * i2cModule, DRV_I2C_MODE i2cMode )
{   
    switch( i2cMode )
    {
        case DRV_I2C_MODE_MASTER:
        {
            i2cModule->TWI_CR.w = TWI_CR_MSEN_Msk;
            break;
        }
        
        case DRV_I2C_MODE_SLAVE:
        {
            /* Enable slave mode */
            i2cModule->TWI_CR.w = TWI_CR_SVEN_Msk;
            break;
        }
    }
            
    return;
}

DRV_I2C_CLIENT_OBJ * _DRV_I2C_DriverHandleValidate(DRV_HANDLE handle)
{
    /* This function returns the pointer to the client object that is
       associated with this handle if the handle is valid. Returns NULL
       otherwise. */

    DRV_I2C_CLIENT_OBJ * client;

    if((DRV_HANDLE_INVALID == handle) ||
            (0 == handle))
    {
        return(NULL);
    }

    client = (DRV_I2C_CLIENT_OBJ *)handle;

    if(!client->inUse)
    {
        return(NULL);
    }

    return(client);
}

uint32_t _DRV_I2C_MakeInternalAddress( uint8_t *address, uint32_t length )
{
    uint32_t result;
    
    if( length == 0 )
    {
        return 0;
    }
    
    result = address[0];
    
    if( length > 1 )
    {
        result <<= 8;
        result |= address[1];
    }
    
    if( length > 2 )
    {
        result <<= 8;
        result |= address[2];
    }
    
    return result;
}

bool _DRV_I2C_ClientBufferQueueObjectsRemove(DRV_I2C_CLIENT_OBJ * clientObj)
{
    DRV_I2C_OBJ * dObj = clientObj->hDriver;
    bool interruptWasEnabled = false;
    DRV_I2C_BUFFER_OBJ * iterator = NULL;

    if(OSAL_MUTEX_Lock(&(dObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Disable the transmit interrupt */
        interruptWasEnabled = _DRV_I2C_InterruptSourceDisable(dObj->interruptSource);
        iterator = dObj->queue;
        while(iterator != NULL)
        {
            if(clientObj == (DRV_I2C_CLIENT_OBJ *)iterator->hClient)
            {
                /* That means this buffer object is owned
                   by this client. This buffer object should
                   be removed. The following code removes
                   the object from a doubly linked list queue. */

                iterator->inUse = false;
                if(iterator->previous != NULL)
                {
                    iterator->previous->next = iterator->next;
                }
                if(iterator->next != NULL)
                {
                    iterator->next->previous = iterator->previous;
                }
                /* Decrementing Current queue size */
                dObj->queueSizeCurrent --;

            }
            iterator = iterator->next;
        }
        /* If there are no buffers in the queue.
         * Make the head pointer point to NULL */
        if(dObj->queueSizeCurrent == 0)
        {
            dObj->queue = NULL;
        }
        else
        {
            /* Iterate to update the head pointer to point
             * the first valid buffer object in the queue */
            iterator = dObj->queue;
            while(iterator != NULL)
            {
                if(iterator->inUse == true)
                {
                    dObj->queue = iterator;
                    break;
                }
                iterator = iterator->next;
            }
        }


        /* Re-enable the interrupt if it was enabled */
        if(interruptWasEnabled)
        {
	        _DRV_I2C_InterruptSourceEnable(dObj->interruptSource);
        }

        /* Unlock the mutex */
        OSAL_MUTEX_Unlock(&(dObj->mutexDriverInstance));
    }
    else
    {
        /* The case where the mutex lock timed out and the
           client buffer objects could not be removed from
           the driver queue, the close function should fail. */

        return false;
    }

    return true;
}

/*******************************************************************************
 End of File
 */
