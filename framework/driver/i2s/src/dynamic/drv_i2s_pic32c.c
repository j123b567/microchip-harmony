/*******************************************************************************
  PIC32C I2S Driver Abstraction Layer Implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2s_pic32c.c

  Summary:
    PIC32C I2S Driver Abstraction Layer Implementation.

  Description:
    This file contains the implementation of the abstraction layer for the
    PIC32C I2S driver. Some PIC32C Device contains multiple peripherals that
    support I2S protocol. A I2S driver is implemented for each such peripheral.
    This file contains an abstraction layer implementation that provided
    peripheral abstraction I2S driver API.
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

/*************************************************************
 * Include files.
 ************************************************************/
#include "driver/i2s/drv_i2s.h"
#include "driver/i2s/src/drv_i2s_pic32c_local.h"
#include "system/debug/sys_debug.h"

/**********************************************
 * Hardware instance objects.
 *********************************************/
DRV_I2S_OBJ gDrvI2SObj[DRV_I2S_INSTANCES_NUMBER];

// *****************************************************************************
// *****************************************************************************
// Section: I2S Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_I2S_Initialize 
    ( 
        const SYS_MODULE_INDEX drvIndex,
        const SYS_MODULE_INIT *const init 
    );

  Summary:
    Initializes hardware and data for the instance of the I2S module.

  Description:
    This routine initializes the I2S driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the init parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized. The driver instance index is
    independent of the I2S module ID. For example, driver instance 0 can be
    assigned to I2S2.  If the driver is built statically, then some of the
    initialization parameters are overridden by configuration macros. Refer to
    the description of the DRV_I2S_INIT data structure for more details on which
    members on this data structure are overridden.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/
 
SYS_MODULE_OBJ  DRV_I2S_Initialize
( 
    const SYS_MODULE_INDEX moduleIndex,
    const SYS_MODULE_INIT *const init
)
{
    DRV_I2S_OBJ *driverObj = NULL;
	DRV_I2S_INIT * moduleInit = (DRV_I2S_INIT *) init;
	
    SYS_MODULE_OBJ retVal = SYS_MODULE_OBJ_INVALID;
    SYS_MODULE_OBJ driverModuleObj = SYS_MODULE_OBJ_INVALID;
    
    DRV_I2S_INTERFACE * driverInterface = NULL;
	
    int i = 0;

    /* Make sure that driver index is valid */
    if(moduleIndex < DRV_I2S_INSTANCES_NUMBER)
    {
        if(init != NULL)
        {
            /* Get a pointer to the driver object */
            driverObj = (DRV_I2S_OBJ *)&gDrvI2SObj[moduleIndex];

            if(driverObj->inUse)
            {
                /* This means this driver object is already initialized. We will
                 * simply return the module index as module object */
                retVal = (SYS_MODULE_OBJ)(moduleIndex);
            }
            else
            {
                /* Get a pointer to the underlying driver initialize function */
                driverInterface = moduleInit->driverInterface;

				driverObj->inUse = true;
				driverObj->status = SYS_STATUS_BUSY;
					
                /* Call the driver initialize function */
                driverModuleObj = driverInterface->initialize(moduleIndex, moduleInit->init);

                if(driverModuleObj != SYS_MODULE_OBJ_INVALID)
                {
                    /* The underlying driver could be initialized. Update the
                     * abstraction layer module object and return the module
                     * index as system module index. Note the we store the
                     * driver module object returned by the underlying
                     * driver and pointer to the driver interface in the
                     * abstraction driver object. */

                    driverObj->inUse = true;
					driverObj->status = SYS_STATUS_READY;
                    driverObj->moduleObj = driverModuleObj;
                    driverObj->driverInterface = driverInterface;
					

                    for(i = 0; i < DRV_I2S_CLIENTS_NUMBER; i++)
                    {
                        /* Initialize the driver handle array */
                        driverObj->driverHandles[i] = DRV_HANDLE_INVALID;
                    }

                    retVal = (SYS_MODULE_OBJ)driverObj;

                    /* Each driver instance needs a mutex to manage multiple
                     * clients */
                    if(OSAL_MUTEX_Create(&driverObj->mutexDriverHandle) != OSAL_RESULT_TRUE)
                    {
                        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Error while creating mutex");
						retVal = SYS_MODULE_OBJ_INVALID;
					}
				}  
            }
        }
        else
        {
            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Initialization Parameter is NULL!");
        }
    }
    else
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Driver Index is not valid");
		retVal = SYS_MODULE_OBJ_INVALID;
    }

    return(retVal);
}

// *****************************************************************************
/* Function:
    void DRV_I2S_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the I2S driver module.

  Description:
    Deinitializes the specified instance of the I2S driver module, disabling
    its operation (and any hardware).  Invalidates all the internal data.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void DRV_I2S_Deinitialize
( 
    SYS_MODULE_OBJ object
)
{
    /* The module object is the index into the driver object array */
    uint32_t index = (uint32_t)(object);
    DRV_I2S_OBJ * driverObj = NULL;

    if(index < DRV_I2S_INSTANCES_NUMBER)
    {
        /* Get the pointer to the driver object */
        driverObj = &gDrvI2SObj[index];
        if(driverObj->inUse)
        {
            /* Call the de-initialize function of the underlying driver and then
             * return the driver object back to an unallocated state. */

            driverObj->driverInterface->deinitialize(driverObj->moduleObj);
            driverObj->inUse = false;
            driverObj->moduleObj = SYS_MODULE_OBJ_INVALID;
        }
        else
        {
            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
        }
    }
    else
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
    }
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_I2S_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the I2S driver module.

  Description:
    This routine provides the current status of the I2S driver module.

  Remarks:
    Refer to drv_i2s.h for usage informatioin.
*/

SYS_STATUS DRV_I2S_Status
( 
    SYS_MODULE_OBJ object
)
{
    /* The module object is the index into the driver object array */
    uint32_t index = (uint32_t)(object);
    DRV_I2S_OBJ * driverObj = NULL;
    SYS_STATUS retVal = SYS_STATUS_UNINITIALIZED;

    if(index < DRV_I2S_INSTANCES_NUMBER)
    {
        /* Get the pointer to the driver object */
        driverObj = &gDrvI2SObj[index];
        if(driverObj->inUse)
        {
            /* Call the status function of the underlying driver */
            retVal = driverObj->driverInterface->status(driverObj->moduleObj);
        }
        else
        {
            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
        }
    }
    else
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
    }

    return(retVal);
}

// *****************************************************************************
/* Function:
    void DRV_I2S_Tasks(SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's receive state machine and implements its ISR.

  Description:
    This routine is used to maintain the driver's internal receive state machine
    and implement its transmit and receive ISR for interrupt-driven
    implementations.  In polling mode, this function should be called from the
    SYS_Tasks function.  In interrupt mode, this function should be called from
    the interrupt service routine of the I2S that is associated with this I2S
    driver hardware instance.
    
    In DMA mode of operation, this function should be called from the interrupt
    service routine of the channel associated with the transmission/reception
    of the I2s driver hardware instance.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void  DRV_I2S_Tasks
(
    SYS_MODULE_OBJ object
)
{
    /* The module object is the index into the driver object array */
    uint32_t index = (uint32_t)(object);
    DRV_I2S_OBJ * driverObj = NULL;

    if(index < DRV_I2S_INSTANCES_NUMBER)
    {
        /* Get the pointer to the driver object */
        driverObj = &gDrvI2SObj[index];
        if(driverObj->inUse)
        {
            /* Call the driver tasks function of the underlying driver */
            driverObj->driverInterface->tasks(driverObj->moduleObj);
        }
        else
        {
            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
        }
    }
    else
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
    }
}

// *****************************************************************************
/* Function:
    void DRV_I2S_TasksError (SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's error state machine and implements its ISR.

  Description:
    This routine is used to maintain the driver's internal error state machine
    and implement its error ISR for interrupt-driven implementations.  In
    polling mode, this function should be called from the SYS_Tasks() function.
    In interrupt mode, this function should be called in the error interrupt
    service routine of the I2S that is associated with this I2S driver hardware
    instance.
    
    In DMA mode of operation, this function should be called from the interrupt
    service routine of the channel associated with the transmission/reception of
    the I2s driver hardware instance.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void DRV_I2S_TasksError 
( 
    SYS_MODULE_OBJ object 
)
{
    /* The module object is the index into the driver object array */
    uint32_t index = (uint32_t)(object);
    DRV_I2S_OBJ * driverObj = NULL;

    if(index < DRV_I2S_INSTANCES_NUMBER)
    {
        /* Get the pointer to the driver object */
        driverObj = &gDrvI2SObj[index];
        if(driverObj->inUse)
        {
            /* Call the error tasks function of the underlying driver */
            driverObj->driverInterface->tasksError(driverObj->moduleObj);
        }
        else
        {
            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
        }
    }
    else
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: I2S Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_I2S_Open
    ( 
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT    ioIntent 
    )

  Summary:
    Opens the specified I2S driver instance and returns a handle to it.

  Description:
    This routine opens the specified I2S driver instance and provides a handle
    that must be provided to all other client-level operations to identify the
    caller and the instance of the driver. The ioIntent parameter defines how
    the client interacts with this driver instance.

    The DRV_IO_INTENT_BLOCKING and DRV_IO_INTENT_NONBLOCKING ioIntent options
    additionally affect the behavior of the DRV_I2S_Read() and DRV_I2S_Write()
    functions. If the ioIntent is DRV_IO_INTENT_NONBLOCKING, then these function
    will not block even if the required amount of data could not be processed.
    If the ioIntent is DRV_IO_INTENT_BLOCKING, these functions will block until
    the required amount of data is processed.

    If ioIntent is DRV_IO_INTENT_READ, the client will only be read from the
    driver. If ioIntent is DRV_IO_INTENT_WRITE, the client will only be able to
    write to the driver. If the ioIntent in DRV_IO_INTENT_READWRITE, the client
    will be able to do both, read and write.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any other
    client.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

DRV_HANDLE DRV_I2S_Open
( 
    const SYS_MODULE_INDEX iDriver, 
    const DRV_IO_INTENT ioIntent
)
{
	DRV_I2S_OBJ * driverObj = NULL;
    uint32_t index = (uint32_t)iDriver;
    uint32_t i = 0;
    
	DRV_HANDLE retVal = DRV_HANDLE_INVALID;
    DRV_HANDLE driverHandle = DRV_HANDLE_INVALID;

    if(index < DRV_I2S_INSTANCES_NUMBER)
    {
        /* The driver instance is valid. Get a pointer to the driver object */
        driverObj = (DRV_I2S_OBJ *)&gDrvI2SObj[index];

        if (driverObj->status == SYS_STATUS_READY)
		{
			if(driverObj->inUse)
			{
				/* This is the call to the open function of the underlying
				 * driver. Note that the I2S abstraction assumes that the open
				 * function implementation of the underlying driver is thread
				 * safe. The abstraction does not implement any thread safety
				 * around this call. */

				driverHandle = driverObj->driverInterface->open(iDriver, ioIntent);

				if(driverHandle != DRV_HANDLE_INVALID)
				{
					/* This means the driver could be opened. We should save the
					 * driver handle some where in the driver handle array of
					 * the abstraction object. This should be made thread safe. 
                     */

					if(OSAL_MUTEX_Lock(&driverObj->mutexDriverHandle, OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
					{
						for (i = 0; i < DRV_I2S_CLIENTS_NUMBER; i ++)
						{
							if(driverObj->driverHandles[i] == DRV_HANDLE_INVALID)
							{
								/* This slot is available.Save the handle here */
								driverObj->driverHandles[i] = driverHandle;
								break;
							}
						}

						/* Release the mutex */
						OSAL_MUTEX_Unlock(&driverObj->mutexDriverHandle);

						if(i >= DRV_I2S_CLIENTS_NUMBER)
						{
							/* This means there is no place in the array. Close
							 * the underlying driver and return an invalid
							 * handle. */

							SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Insufficent clients configured. Check value of DRV_I2S_CLIENTS_NUMBER");
							driverObj->driverInterface->close(driverHandle);
						}
						else
						{
							/* The driver handle returned by the underlying 
                             * driver open function is stored in the driver 
                             * object. We have to return a handle to the client.
                             * This handle is formed by combining the driver 
                             * index and driver handle array index. */

							retVal = (DRV_HANDLE)(((index << 16) & 0xFFFF0000)|i);
						}
					}
					else
					{
						SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Could not grab mutex");
					}
				}
			}
			else
			{
				SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module object");
			}
		}
		else
		{
			SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Status in not ready");
			retVal = DRV_HANDLE_INVALID;
		}		

    }
    else
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid module index");
		retVal = DRV_HANDLE_INVALID;
    }

    return(retVal);
}

// *****************************************************************************
/* Function:
    void DRV_I2S_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the I2S driver.

  Description:
    This routine closes an opened-instance of the I2S driver, invalidating the
    handle. Any buffers in the driver queue that were submitted by this client
    will be removed.  After calling this routine, the handle passed in "handle"
    must not be used with any of the remaining driver routines.  A new handle
    must be obtained by calling DRV_I2S_Open before the caller may use the
    driver again

  Remarks:
    Refer to drv_i2s.h for usage information.
 
*/

void DRV_I2S_Close
( 
    const DRV_HANDLE handle
)
{
    uint32_t index = 0;
    uint32_t driverHandleIndex = 0;
    DRV_I2S_OBJ * driverObj = NULL;
    
    if(handle != DRV_HANDLE_INVALID)
    {
        /* Obtain the driver object index and the driver handle index  from the
         * driver handle. */ 

        index = (((uint32_t)handle) & 0xFFFF0000) >> 16;
        driverHandleIndex = ((uint32_t)handle) & 0x0000FFFF;

        if((index < DRV_I2S_INSTANCES_NUMBER) && (driverHandleIndex < DRV_I2S_CLIENTS_NUMBER))
        {
            driverObj = &gDrvI2SObj[index];
            
            /* Call the close function of the underlying driver */
            driverObj->driverInterface->close(driverObj->driverHandles[driverHandleIndex]);

            /* Invalidate the driver handle that was stored in the driver object
             * */
            driverObj->driverHandles[driverHandleIndex] = DRV_HANDLE_INVALID;
        }
        else
        {
            SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid driver handle");
        }
    }
    else
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid driver handle");
    }
}

// *****************************************************************************
/* Function:
    void DRV_I2S_BufferAddWrite
    (
        const DRV_HANDLE handle,
        DRV_I2S_BUFFER_HANDLE *bufferHandle,
        void * buffer, size_t size
    );

  Summary:
    Schedule a non-blocking driver write operation.

  Description:
    This function schedules a non-blocking write operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the write request
    was scheduled successfully. The function adds the request to the hardware
    instance transmit queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_I2S_BUFFER_HANDLE_INVALID:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read-only
    - if the buffer size is 0
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_I2S_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_I2S_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Remarks:
    Refer to drv_i2s.h for usage information.

*/

void DRV_I2S_BufferAddWrite
(
    const DRV_HANDLE handle,
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *buffer, 
    size_t size
)
{
    uint32_t index = 0;
    uint32_t driverHandleIndex = 0;
    uint32_t i = 0;
    
    DRV_I2S_OBJ *driverObj;
    DRV_HANDLE driverHandle = DRV_HANDLE_INVALID;
    
    index = (((uint32_t)handle) & 0xFFFF0000) >> 16;
    driverHandleIndex = ((uint32_t)handle) & 0x0000FFFF;
    driverObj = &gDrvI2SObj[index];
	
	/* We first check the arguments and initialize the
     * buffer handle */
    
    if(bufferHandle != NULL)
    {
        *bufferHandle = DRV_I2S_BUFFER_HANDLE_INVALID;
    }
    
    /* Call the bufferAddWrite function of the underlying driver */
    driverObj->driverInterface->bufferAddWrite(driverObj->driverHandles[driverHandleIndex],bufferHandle,buffer,size);

    return;
}

// *****************************************************************************
/* Function:
    void DRV_I2S_BufferAddRead
    ( 
        const DRV_HANDLE handle,
        DRV_I2S_BUFFER_HANDLE *bufferHandle,
        void * buffer, size_t size
    )

  Summary:
    Schedule a non-blocking driver read operation.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function schedules a non-blocking read operation. The function returns
    with a valid buffer handle  in the bufferHandle argument if the read request
    was scheduled successfully. The function adds the request to the hardware
    instance receive queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_I2S_BUFFER_HANDLE_INVALID:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for write-only
    - if the buffer size is 0
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_I2S_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_I2S_BUFFER_EVENT_ERROR event if the buffer
    was not processed successfully.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void DRV_I2S_BufferAddRead
(
    const DRV_HANDLE handle,
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *buffer, 
    size_t size
)
{
    uint32_t index = 0;
    uint32_t driverHandleIndex = 0;
    uint32_t i = 0;
    
    DRV_I2S_OBJ *driverObj = NULL;
    DRV_HANDLE driverHandle = DRV_HANDLE_INVALID;
    
    index = (((uint32_t)handle) & 0xFFFF0000) >> 16;
    driverHandleIndex = ((uint32_t)handle) & 0x0000FFFF;
    driverObj = &gDrvI2SObj[index];
	
	/* We first check the arguments and initialize the
     * buffer handle */
    
    if(bufferHandle != NULL)
    {
        *bufferHandle = DRV_I2S_BUFFER_HANDLE_INVALID;
    }
    
    /* Call the bufferAddRead function of the underlying driver */
    driverObj->driverInterface->bufferAddRead(driverObj->driverHandles[driverHandleIndex],bufferHandle,buffer,size);

	return;
}

// *****************************************************************************
/* Function:
	void DRV_I2S_BufferAddWriteRead
    (
        const DRV_HANDLE handle,
        DRV_I2S_BUFFER_HANDLE	*bufferHandle,
        void *transmitBuffer, void *receiveBuffer,
        size_t size
    )

  Summary:
    Schedule a non-blocking driver write-read operation.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function schedules a non-blocking write-read operation. The function
    returns with a valid buffer handle in the bufferHandle argument if the
    write-read request was scheduled successfully. The function adds the request
    to the hardware instance queue and returns immediately. While the request is
    in the queue, the application buffer is owned by the driver and should not
    be modified. The function returns DRV_I2S_BUFFER_HANDLE_INVALID:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only or write only
    - if the buffer size is 0
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_I2S_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_I2S_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void DRV_I2S_BufferAddWriteRead
(
    const DRV_HANDLE handle,
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *transmitBuffer, 
    void *receiveBuffer,
    size_t size
)
{
    uint32_t index = 0;
    uint32_t driverHandleIndex = 0;
    uint32_t i = 0;
        
    DRV_I2S_OBJ *driverObj = NULL;
    DRV_HANDLE driverHandle = DRV_HANDLE_INVALID;

    
    index = (((uint32_t)handle) & 0xFFFF0000) >> 16;
    driverHandleIndex = ((uint32_t)handle) & 0x0000FFFF;
    driverObj = &gDrvI2SObj[index];
    
    /* We first check the arguments and initialize the buffer handle */
    if(bufferHandle != NULL)
    {
        *bufferHandle = DRV_I2S_BUFFER_HANDLE_INVALID;
    }

    /* Call the bufferAddWriteRead function of the underlying driver */
    driverObj->driverInterface->bufferAddWriteRead(driverObj->driverHandles[driverHandleIndex],bufferHandle,transmitBuffer, receiveBuffer, size);

	return;
}

// *****************************************************************************
/* Function:
    void DRV_I2S_BufferEventHandlerSet
    ( 
        const DRV_HANDLE handle,
        DRV_I2S_BUFFER_EVENT_HANDLER eventHandler,
	    uintptr_t contextHandle
    )

  Summary:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.

  Description:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
    When a client calls either the DRV_I2S_BufferAddRead, DRV_I2S_BufferAddWrite
    or DRV_I2S_BufferAddWriteRead  function, it is provided with a handle
    identifying  the buffer that was added to the driver's buffer queue.  The
    driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "buffer add"
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which
    could be a "NULL" pointer to indicate no callback).

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void DRV_I2S_BufferEventHandlerSet
(
    DRV_HANDLE handle,
	const DRV_I2S_BUFFER_EVENT_HANDLER eventHandler,
	const uintptr_t contextHandle
)
{
    uint32_t index = 0;
    uint32_t driverHandleIndex = 0;
    
    DRV_I2S_OBJ *driverObj = NULL;
    
    index = (((uint32_t)handle) & 0xFFFF0000) >> 16;
    driverHandleIndex = ((uint32_t)handle) & 0x0000FFFF;
    driverObj = &gDrvI2SObj[index];
    
	if(!(DRV_HANDLE_INVALID == handle))
	{
        /* Call the eventHandlerSet function of the underlying driver */
        driverObj->driverInterface->eventHandlerSet(driverObj->driverHandles[driverHandleIndex],eventHandler,contextHandle);
	}	
	else
	{
		SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid handle");
	}
    
    return;
}

// *****************************************************************************
/* Function:
    void DRV_I2S_BufferQueueFlush(DRV_HANDLE handle)

  Summary:
    This function flushes off the buffers associated with the client object.

  Description:
    This function flushes off the buffers associated with the client object and
    disables the DMA channel used for transmission.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void DRV_I2S_BufferQueueFlush
(
    DRV_HANDLE handle
)
{
    uint32_t index = 0;
    uint32_t driverHandleIndex = 0;
    
    DRV_I2S_OBJ *driverObj = NULL;
    
    index = (((uint32_t)handle) & 0xFFFF0000) >> 16;
    driverHandleIndex = ((uint32_t)handle) & 0x0000FFFF;
    driverObj = &gDrvI2SObj[index];
    
    if(!(DRV_HANDLE_INVALID == handle))
	{
        /* Call the bufferQueueFlush function of the underlying driver */
        driverObj->driverInterface->bufferQueueFlush(driverObj->driverHandles[driverHandleIndex]);
	}	
	else
	{
		SYS_DEBUG_PRINT(SYS_ERROR_INFO, "\r\nI2S Driver: Invalid handle");
	}
    
    return;
}

// *****************************************************************************
/* Function:
    void DRV_I2S_TransmitErrorIgnore
    (
        DRV_HANDLE handle, 
        bool errorIgnore
    )

  Summary:
    This function enable/disable ignoring of the transmit underrun error.

  Description:
    A Transmit underrun error is not a critical error and zeros are transmitted
    until the SPIxTXB is not empty. Ignore Transmit underrun error is needed for
    cases when software does not care or does not need to know about underrun
    conditions.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void DRV_I2S_TransmitErrorIgnore
(
    DRV_HANDLE handle, 
    bool errorIgnore
)
{
    /* Empty function as pic32c doesn't support this */
}

// *****************************************************************************
/* Function:
    void DRV_I2S_ReceiveErrorIgnore
    (
        DRV_HANDLE handle, 
        bool errorEnable
    )

  Summary:
    This function enable/disable ignoring of the receive overflow error.

  Description:
    A receive overflow is not a critical error; during receive overflow data in
    the FIFO is not overwritten by receive data. Ignore receive overflow is
    needed for cases when there is a general performance problem in the system
    that software must handle properly.

  Remarks:
    Refer to drv_i2s.h for usage information.
*/

void DRV_I2S_ReceiveErrorIgnore
(
    DRV_HANDLE handle, 
    bool errorEnable
)
{
    /* Empty function as pic32c doesn't support this */
}

//*******************************************************************************
/* Function:
    void DRV_I2S_BaudSet
    (
        DRV_HANDLE handle, 
        uint32_t clockFrequency, 
        uint32_t baudRate
    )

  Summary:
    This function sets the baudrate.

  Description:
    This function sets the baud rate for the I2S operation.

  Remarks:
    None.
*/

void DRV_I2S_BaudSet
(
    DRV_HANDLE handle, 
    uint32_t clockFrequency, 
    uint32_t baudRate
)
{
    /* Empty function as pic32c doesn't support this */
}

//******************************************************************************
/* Function:
    void DRV_I2S_SetAudioCommunicationMode
    (
        const DRV_HANDLE handle, 
        uint8_t audioCommWidth
    )

  Summary:
    This function sets the audio communication width.

  Description:
    This function sets the audio communication width for the I2S operation.

  Remarks:
    None.
*/

void DRV_I2S_SetAudioCommunicationMode
(
    const DRV_HANDLE handle, 
    uint8_t audioCommWidth
)
{
    /* Empty function as pic32c doesn't support this */
}