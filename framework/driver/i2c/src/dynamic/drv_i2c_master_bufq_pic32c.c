/*******************************************************************************
  I2C Device Driver Buffer Queue Implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_buffer_queue_pic32c.c

  Summary:
    I2C Device Driver buffer queue Implementation.

  Description:
 This file contains the Dynamic mode implementation of the I2C driver 
 buffer queue routines.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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
extern DRV_I2C_OBJ             gDrvI2CObj      [ DRV_I2C_INSTANCES_NUMBER ] ;

/* This is the client object array. */
extern DRV_I2C_CLIENT_OBJ      gDrvI2CClientObj[ DRV_I2C_CLIENTS_NUMBER ];

/* This is the buffer object array */
DRV_I2C_BUFFER_OBJ             gDrvI2CBufferObj[ DRV_I2C_QUEUE_DEPTH_COMBINED ];

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

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

DRV_I2C_BUFFER_HANDLE _DRV_I2C_MasterReceive (   DRV_HANDLE handle, 
                                                 uint16_t   slaveAddress,
                                                 void *     buffer,
                                                 size_t     size,
                                                 void *     callbackContext )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    bool                 interruptWasEnabled = false;
    DRV_I2C_BUFFER_OBJ * bufferObj           = NULL;
    DRV_I2C_BUFFER_OBJ * iterator;
    twi_registers_t  * i2cModule;
    uint32_t             i;
    uint32_t             flag10BitAddr = 0;
    
        /* Validate the driver handle */
    clientObj = (DRV_I2C_CLIENT_OBJ *)handle;
    hDriver   = clientObj->hDriver;
    i2cModule = (twi_registers_t *) hDriver->i2cModule;

    if(hDriver->queueSizeCurrent >= hDriver->queueSize)
    {
        /* This means the queue is full. We cannot add
           this request */
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    /* We will allow buffers to be added in the interrupt
       context of this I2C driver. But we must make
       sure that if we are in interrupt, then we should
       not modify mutexes. */

    if(0 == hDriver->interruptNestingCount)
    {
        /* Grab a mutex. This is okay because we are not in an
           interrupt context */

        if(OSAL_MUTEX_Lock(&(hDriver->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            /* We will disable interrupts so that the queue
               status does not get updated asynchronously.
               This code will always execute. */
            interruptWasEnabled = _DRV_I2C_InterruptSourceDisable(hDriver->interruptSource);
        }
        else
        {
            /* The mutex acquisition timed out. Return with an
               invalid handle. This code will not execute
               if there is no RTOS. */
            return DRV_I2C_BUFFER_HANDLE_INVALID;
        }
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_I2C_QUEUE_DEPTH_COMBINED; i ++)
    {
        if(!gDrvI2CBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvI2CBufferObj[i];
            bufferObj->inUse              = true;
            bufferObj->readSize           = size;
            bufferObj->writeSize          = 0;
            
            if ( DRV_I2C_7BIT_ADDR_UPPER_LIMIT < slaveAddress  )
            {
                flag10BitAddr = 1;
                bufferObj->internalAddress[0] = ((uint8_t *)&slaveAddress)[0];
            }
            else
            {
                flag10BitAddr = 0;
            }
            
            bufferObj->slave7BitAddress   = ((uint8_t *)&slaveAddress)[flag10BitAddr] >> 1;
            bufferObj->internalAddrSize   = flag10BitAddr;
            bufferObj->internalAddress[1] = 0;
            bufferObj->internalAddress[2] = 0;
            bufferObj->readBuffer         = ( uint8_t* ) buffer;
            bufferObj->writeBuffer        = NULL;
            bufferObj->hClient            = clientObj;
            bufferObj->nCurrentBytes      = 0;
            bufferObj->next               = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->previous           = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->currentState       = DRV_I2C_BUFFER_IS_IN_QUEUE;
            bufferObj->flags              = DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_READ;
            bufferObj->event              = DRV_I2C_BUFFER_EVENT_PENDING;
            bufferObj->errorEvent         = DRV_I2C_HALT_ON_ERROR;

            break;
        }
    }

    if( DRV_I2C_QUEUE_DEPTH_COMBINED == i )
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_I2C_QUEUE_DEPTH
           parameter is configured to be less */

        /* Enable the interrupt if it was enabled */
        if(interruptWasEnabled)
        {   
            _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
        }
        
        if( 0 == hDriver->interruptNestingCount )
        {
            /* Release mutex */
            OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
        }

        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    /* Increment the current queue size*/
    hDriver->queueSizeCurrent ++;

    /* Check if the queue is empty */
    if( NULL == hDriver->queue )
    {
        /* This is the first buffer in the
           queue */
        hDriver->queue = bufferObj;

        /* Because this is the first buffer in the queue, we need to send the
         first byte (In this case, the address of the client to communicate,
         so that we trigger the transmit interrupt generation. The
         rest of the buffer then gets processed in the task routine, which
         may or may not be called from the interrupt service routine. */
          
        /* Send I2C Address */
        i2cModule->TWI_MMR.w = 0;
        i2cModule->TWI_MMR.w = TWI_MMR_MREAD_Msk | 
                                 TWI_MMR_DADR(bufferObj->slave7BitAddress) |
                                 TWI_MMR_IADRSZ(bufferObj->internalAddrSize);

        /* if slave address is 10bit 
         * set MSB 3 bits in the internal address registers*/
        i2cModule->TWI_IADR.w  = 
                TWI_IADR_IADR( _DRV_I2C_MakeInternalAddress ( bufferObj->internalAddress, 
                                                                bufferObj->internalAddrSize ));

        /* Send a START Condition */
        i2cModule->TWI_CR.w = TWI_CR_START_Msk;

        /* Transmission to be stopped at the same time as start
           if the transmission size is of 1 byte*/
        if( 1 == bufferObj->readSize )
        {
            i2cModule->TWI_CR.w = TWI_CR_STOP_Msk;
        }

        hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
        
        /* If the driver is configured for interrupt mode, then the following
         statement should enable the interrupt. */
        i2cModule->TWI_IER.w = TWI_IER_TXCOMP_Msk | TWI_IER_RXRDY_Msk;
        
        _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
    }
    else
    {
        /* This means the write queue is not empty. We must add
         * the buffer object to the end of the queue */
        iterator = hDriver->queue;
        while( NULL != iterator->next )
        {
            /* Get the next buffer object */
            iterator = iterator->next;
        }

        /* At this point, iterator will point to the
           last object in the queue. We add the buffer
           object to the linked list. Note that we
           need to set up the previous pointer as well
           because buffer should be deleted when the
           client closes the driver */
        iterator->next = bufferObj;
        bufferObj->previous = iterator;

        /* We are done. Restore the interrupt enable status
           and return. */
        if(interruptWasEnabled)
        {
            _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
        }

    }
    
    if( 0 == hDriver->interruptNestingCount )
    {
        /* Release mutex */
        OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
    }
    
    return (DRV_I2C_BUFFER_HANDLE) bufferObj;
    
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

DRV_I2C_BUFFER_HANDLE _DRV_I2C_MasterTransmit (  DRV_HANDLE handle,
                                                 uint16_t   slaveAddress,
                                                 void *     buffer,
                                                 size_t     size,
                                                 DRV_I2C_BUS_ERROR_EVENT eventFlag,
                                                 void *     context )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    bool                 interruptWasEnabled = false;
    DRV_I2C_BUFFER_OBJ * bufferObj           = NULL;
    DRV_I2C_BUFFER_OBJ * iterator;
    twi_registers_t  * i2cModule;
    uint32_t             i;
    uint32_t             flag10BitAddr;
    
    clientObj = (DRV_I2C_CLIENT_OBJ *)handle;
    hDriver   = clientObj->hDriver;
    i2cModule = (twi_registers_t *) hDriver->i2cModule;
    
    if(hDriver->queueSizeCurrent >= hDriver->queueSize)
    {
        /* This means the queue is full. We cannot add
           this request */
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    /* We will allow buffers to be added in the interrupt
       context of this I2C driver. But we must make
       sure that if we are in interrupt, then we should
       not modify mutexes. */

    if( 0 == hDriver->interruptNestingCount )
    {
        /* Grab a mutex. This is okay because we are not in an
           interrupt context */

        if(OSAL_MUTEX_Lock(&(hDriver->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            /* We will disable interrupts so that the queue
               status does not get updated asynchronously.
               This code will always execute. */

            interruptWasEnabled = _DRV_I2C_InterruptSourceDisable(hDriver->interruptSource);
        }
        else
        {
            /* The mutex acquisition timed out. Return with an
               invalid handle. This code will not execute
               if there is no RTOS. */
            return DRV_I2C_BUFFER_HANDLE_INVALID;
        }
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_I2C_QUEUE_DEPTH_COMBINED; i ++)
    {
        if(!gDrvI2CBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvI2CBufferObj[i];
            bufferObj->inUse              = true;
            bufferObj->readSize           = 0;
            bufferObj->writeSize          = size;
            
            if ( DRV_I2C_7BIT_ADDR_UPPER_LIMIT < slaveAddress )
            {
                flag10BitAddr = 1;
                bufferObj->internalAddress[0] = ((uint8_t *)&slaveAddress)[0];
            }
            else
            {
                flag10BitAddr = 0;
            }
            
            bufferObj->slave7BitAddress   = ((uint8_t *)&slaveAddress)[flag10BitAddr] >> 1;
            bufferObj->internalAddrSize   = flag10BitAddr;
            bufferObj->internalAddress[1] = 0;
            bufferObj->internalAddress[2] = 0;
            bufferObj->readBuffer         = NULL;
            bufferObj->writeBuffer        = (uint8_t*)buffer;
            bufferObj->hClient            = clientObj;
            bufferObj->nCurrentBytes      = 0;
            bufferObj->next               = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->previous           = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->currentState       = DRV_I2C_BUFFER_IS_IN_QUEUE;
            bufferObj->flags              = DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_WRITE;
            bufferObj->event              = DRV_I2C_BUFFER_EVENT_PENDING;
            bufferObj->errorEvent         = eventFlag;
            
            break;
        }
    }

    if( DRV_I2C_QUEUE_DEPTH_COMBINED == i )
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_I2C_QUEUE_DEPTH
           parameter is configured to be less */

        /* Enable the interrupt if it was enabled */
        if(interruptWasEnabled)
        {
            _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
        }
        
        if( 0 == hDriver->interruptNestingCount )
        {
            /* Release mutex */
            OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
        }

        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    /* Increment the current queue size*/
    hDriver->queueSizeCurrent ++;

    /* Check if the queue is empty */
    if( NULL == hDriver->queue )
    {
        /* This is the first buffer in the
           queue */

        hDriver->queue = bufferObj;

        /* Because this is the first buffer in the queue, we need to send the
         first byte (In this case, the address of the client to communicate,
         so that we trigger the transmit interrupt generation. The
         rest of the buffer then gets processed in the task routine, which
         may or may not be called from the interrupt service routine. */

        /* Send I2C Address */
        i2cModule->TWI_MMR.w        = 0;
        i2cModule->TWI_MMR.w = TWI_MMR_DADR(bufferObj->slave7BitAddress) |
                                 TWI_MMR_IADRSZ(bufferObj->internalAddrSize);

        /* Set internal address */
        i2cModule->TWI_IADR.w = 
                TWI_IADR_IADR( _DRV_I2C_MakeInternalAddress ( bufferObj->internalAddress, 
                                                                bufferObj->internalAddrSize ));

        hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE;
        
        /* If the driver is configured for interrupt mode, then the following
         statement should enable the interrupt. */
        i2cModule->TWI_IER.w = TWI_IER_TXCOMP_Msk | TWI_IER_TXRDY_Msk;
        
        _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
        
    }
    else
    {
        /* This means the write queue is not empty. We must add
         * the buffer object to the end of the queue */

        iterator = hDriver->queue;
        while( NULL != iterator->next )
        {
            /* Get the next buffer object */
            iterator = iterator->next;
        }

        /* At this point, iterator will point to the
           last object in the queue. We add the buffer
           object to the linked list. Note that we
           need to set up the previous pointer as well
           because buffer should be deleted when the
           client closes the driver */

        iterator->next = bufferObj;
        bufferObj->previous = iterator;

        /* We are done. Restore the interrupt enable status
           and return. */
        if(interruptWasEnabled)
        {
            _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
        }

    }
    
    if( 0 == hDriver->interruptNestingCount )
    {
        /* Release mutex */
        OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
    }
    
    return (DRV_I2C_BUFFER_HANDLE) bufferObj;
    
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

DRV_I2C_BUFFER_HANDLE _DRV_I2C_MasterTransmitThenReceive (  DRV_HANDLE handle, 
                                                            uint16_t   slaveAddress,
                                                            void *     writeBuffer,
                                                            size_t     writeSize,
                                                            void *     readBuffer,
                                                            size_t     readSize,
                                                            void *     callbackContext )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    bool                 interruptWasEnabled = false;
    DRV_I2C_BUFFER_OBJ * bufferObj           = NULL;
    DRV_I2C_BUFFER_OBJ * iterator;
    twi_registers_t              * i2cModule;
    uint32_t             i = 0;
    uint32_t             j = 0;
    uint32_t             flag10BitAddr = 0;
    
    clientObj = (DRV_I2C_CLIENT_OBJ *)handle;
    hDriver   = clientObj->hDriver;
    i2cModule = (twi_registers_t *) hDriver->i2cModule;

    if(hDriver->queueSizeCurrent >= hDriver->queueSize)
    {
        /* This means the queue is full. We cannot add
           this request */
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    /* We will allow buffers to be added in the interrupt
       context of this I2C driver. But we must make
       sure that if we are in interrupt, then we should
       not modify mutexes. */

    if( 0 == hDriver->interruptNestingCount )
    {
        /* Grab a mutex. This is okay because we are not in an
           interrupt context */

        if(OSAL_MUTEX_Lock(&(hDriver->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            /* We will disable interrupts so that the queue
               status does not get updated asynchronously.
               This code will always execute. */

            interruptWasEnabled = _DRV_I2C_InterruptSourceDisable(hDriver->interruptSource);
        }
        else
        {
            /* The mutex acquisition timed out. Return with an
               invalid handle. This code will not execute
               if there is no RTOS. */
            return DRV_I2C_BUFFER_HANDLE_INVALID;
        }
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_I2C_QUEUE_DEPTH_COMBINED; i ++)
    {
        if(!gDrvI2CBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvI2CBufferObj[i];
            bufferObj->readSize         = readSize;
            bufferObj->inUse            = true;
            bufferObj->writeBuffer      = NULL;
            bufferObj->writeSize        = 0;
            bufferObj->internalAddrSize = 0;
            bufferObj->flags            = DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_READ;
            
            /* 10-bit address */
            if ( DRV_I2C_7BIT_ADDR_UPPER_LIMIT < slaveAddress )
            {
                flag10BitAddr = 1;
                bufferObj->internalAddress[0] = ((uint8_t *)&slaveAddress)[0];
            }
            else /* 7-bit address */
            {
                flag10BitAddr = 0;
            }
            
            bufferObj->slave7BitAddress = ((uint8_t *)&slaveAddress)[flag10BitAddr] >> 1;
            bufferObj->internalAddrSize = flag10BitAddr;
            
            if(writeSize > (DRV_I2C_INTERNAL_ADDRESS_SIZE - flag10BitAddr))
            {
                bufferObj->writeBuffer = (uint8_t *)writeBuffer;
                bufferObj->writeSize   = writeSize;
                bufferObj->flags      |= DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_WRITE;
            }
            else
            {
                bufferObj->internalAddrSize += writeSize;
                
                for( j = 0; j < writeSize; j++ )
                {
                    bufferObj->internalAddress[j + flag10BitAddr] = ((uint8_t *) writeBuffer)[j] ; 
                }
            }

            bufferObj->readBuffer    = (uint8_t*) readBuffer;
            bufferObj->hClient       = clientObj;
            bufferObj->nCurrentBytes = 0;
            bufferObj->next          = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->previous      = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->currentState  = DRV_I2C_BUFFER_IS_IN_QUEUE;
            bufferObj->event         = DRV_I2C_BUFFER_EVENT_PENDING;
            bufferObj->errorEvent    = DRV_I2C_HALT_ON_ERROR;
            
            break;
        }
    }

    if( DRV_I2C_QUEUE_DEPTH_COMBINED == i )
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_I2C_QUEUE_DEPTH
           parameter is configured to be less */

        /* Enable the interrupt if it was enabled */
        if(interruptWasEnabled)
        {
            _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
        }
        
        if( 0 == hDriver->interruptNestingCount )
        {
            /* Release mutex */
            OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
        }

        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    /* Increment the current queue size*/
    hDriver->queueSizeCurrent ++;

    /* Check if the queue is empty */
    if( NULL == hDriver->queue )
    {
        /* This is the first buffer in the
           queue */

        hDriver->queue = bufferObj;

        /* Because this is the first buffer in the queue, we need to send the
         first byte (In this case, the address of the client to communicate,
         so that we trigger the transmit interrupt generation. The
         rest of the buffer then gets processed in the task routine, which
         may or may not be called from the interrupt service routine. */

        /* Send I2C Address */
        i2cModule->TWI_MMR.w = 0;
        i2cModule->TWI_MMR.w = TWI_MMR_DADR(bufferObj->slave7BitAddress) |
                                 TWI_MMR_IADRSZ(bufferObj->internalAddrSize);

        /* Set internal address */
        i2cModule->TWI_IADR.w = 
                TWI_IADR_IADR( _DRV_I2C_MakeInternalAddress ( bufferObj->internalAddress, 
                                                                bufferObj->internalAddrSize ));

        /* If only internal address registers are used, 
         * read operation performs both transmit, restart and receive */
        if( 0 == bufferObj->writeSize )
        {
            i2cModule->TWI_MMR.w |= TWI_MMR_MREAD_Msk;

            /* Send a START Condition */
            i2cModule->TWI_CR.w = TWI_CR_START_Msk;

            /* Transmission to be stopped at the same time as start
           if the transmission size is of 1 byte*/
            if( 1 == bufferObj->readSize )
            {
                i2cModule->TWI_CR.w = TWI_CR_STOP_Msk;
            }
            
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
            i2cModule->TWI_IER.w = TWI_IER_RXRDY_Msk;
        }
        else
        {
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE;
            i2cModule->TWI_IER.w = TWI_IER_TXRDY_Msk;
        }

        /* If the driver is configured for interrupt mode, then the following
         statement should enable the interrupt. */
        i2cModule->TWI_IER.w = TWI_IER_TXCOMP_Msk;
        
        _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
    }
    else
    {
        /* This means the write queue is not empty. We must add
         * the buffer object to the end of the queue */

        iterator = hDriver->queue;
        while( NULL != iterator->next )
        {
            /* Get the next buffer object */
            iterator = iterator->next;
        }

        /* At this point, iterator will point to the
           last object in the queue. We add the buffer
           object to the linked list. Note that we
           need to set up the previous pointer as well
           because buffer should be deleted when the
           client closes the driver */

        iterator->next = bufferObj;
        bufferObj->previous = iterator;

        /* We are done. Restore the interrupt enable status
           and return. */
        if(interruptWasEnabled)
        {
            _DRV_I2C_InterruptSourceEnable(hDriver->interruptSource);
        }

    }
    
    if( 0 == hDriver->interruptNestingCount )
    {
        /* Release mutex */
        OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
    }
    
    return (DRV_I2C_BUFFER_HANDLE) bufferObj;
    
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

void _DRV_I2C_MasterTasks ( SYS_MODULE_OBJ object )
{
    DRV_I2C_OBJ        * hDriver = &gDrvI2CObj[object];
    DRV_I2C_BUFFER_OBJ * bufferObj;
    DRV_I2C_CLIENT_OBJ * client;
    twi_registers_t  * i2cModule;
    __TWI_SR_bits_t    status;
    
    /* If this driver is configured for polled mode in an RTOS, the tasks
       routine would be called from another thread. We need to get the driver
       instance mutex before updating the queue. If the driver is configured for
       interrupt mode, then _DRV_I2C_TAKE_MUTEX will compile to true */

    if(DRV_I2C_INTERRUPT_MODE == false)
    {
        if(OSAL_MUTEX_Lock(&(hDriver->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            /* We were able to take the mutex */
        }
        else
        {
            /* The mutex acquisition timed out. Return with an
               invalid handle. This code will not execute
               if there is no RTOS. */
            return;
        }
    }

    /* In this function, the driver checks if there are any buffers in queue. If
       so the buffer is serviced. A buffer that is serviced completely is
       removed from the queue. Start by getting the buffer at the head of the
       queue */

    i2cModule = hDriver->i2cModule;
    
    if( NULL != hDriver->queue )
    {
        bufferObj  = hDriver->queue;
        
        /* gets the module status */
        status.w = i2cModule->TWI_SR.w;

        /* checks if Slave has Nacked */
        if( DRV_I2C_TRUE == status.NACK ) 
        {
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_ERROR;
        }

        /* checks if the arbitration is lost in multi-master scenario */
        if( DRV_I2C_TRUE == status.ARBLST )
        {
            /* Re-initiate the transfer if arbitration is lost in 
             * between of the transfer 
             */
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_ADDRESS_SEND;
            bufferObj->nCurrentBytes = 0;
        }

        switch( hDriver->taskState )
        {
            case DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE:
            {
                /* checks if master is ready to transmit */
                if( DRV_I2C_TRUE == status.TXRDY )
                {
                    /* byte by byte data transmit */
                    i2cModule->TWI_THR.w = TWI_THR_TXDATA(bufferObj->writeBuffer[bufferObj->nCurrentBytes]);
                    bufferObj->nCurrentBytes++;

                    /* checks if transmission reached at the end */
                    if( bufferObj->nCurrentBytes == bufferObj->writeSize )
                    {
                        /* checks if it is a transmit only request */
                        if( 0 == bufferObj->readSize )
                        {
                            /* Sends a stop bit */
                            i2cModule->TWI_CR.w = TWI_CR_STOP_Msk;

                            /* read the module status again to know if 
                               tranmission complete flag is set */
                            status.w = i2cModule->TWI_SR.w;
                            if( DRV_I2C_TRUE != status.TXCOMP )
                            {   
                                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_COMPLETE;
                            }
                            else
                            {
                                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_DONE;
                            }
                        }
                        else /* It is a transmit then receive request 
                              * without using internal address register */
                        {
                            bufferObj->nCurrentBytes = 0;

                            i2cModule->TWI_MMR.w = 0;
                            i2cModule->TWI_MMR.w = TWI_MMR_MREAD_Msk | 
                                                     TWI_MMR_DADR(bufferObj->slave7BitAddress);

                            /* Send a START Condition */
                            i2cModule->TWI_CR.w = TWI_CR_START_Msk;

                            /* Send Stop at the same time as start 
                             * if size of data to be received is 1*/
                            if( 1 == bufferObj->readSize )
                            {
                                i2cModule->TWI_CR.w = TWI_CR_STOP_Msk;
                            }

                            /* Enable the RXRDY interrupt */
                            i2cModule->TWI_IER.w = TWI_IER_RXRDY_Msk;
                            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
                        }
                        
                        /* Disable the TXRDY interrupt*/
                        i2cModule->TWI_IDR.w = TWI_IDR_TXRDY_Msk;
                    }
                }
                
                break;
            }

            case DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ:
            {
                /* checks if master has received the data */
                if( DRV_I2C_TRUE == status.RXRDY )
                {
                    /* Send Stop bit if it is second last byte to be received */
                    if( 1 < bufferObj->readSize  &&
                        bufferObj->nCurrentBytes == bufferObj->readSize - 2 )
                    {
                        i2cModule->TWI_CR.w = TWI_CR_STOP_Msk;
                    }

                    /* read the received data */
                    bufferObj->readBuffer[bufferObj->nCurrentBytes] = i2cModule->TWI_RHR.RXDATA;
                    bufferObj->nCurrentBytes++;

                    /* checks if transmission has reached at the end */
                    if( bufferObj->nCurrentBytes == bufferObj->readSize )
                    {
                        /* read the module status again to know if 
                           tranmission complete flag is set */
                        status.w = i2cModule->TWI_SR.w;
                        if( DRV_I2C_TRUE != status.TXCOMP )
                        {   
                            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_COMPLETE;
                        }
                        else
                        {
                            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_DONE;
                        }
                        
                        /* Disable the RXRDY interrupt*/
                        i2cModule->TWI_IDR.w = TWI_IDR_RXRDY_Msk;
                    }
                }
                
                break;
            }

            case DRV_I2C_DATA_OBJ_TASK_TRANSFER_COMPLETE:
            {
                /* check if tranmission complete flag is set */
                if( DRV_I2C_TRUE == status.TXCOMP )
                {
                    hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_DONE;
                }
                
                break;
            }

            default:
            {
                break;
            }
        }

        /* Check if this buffer is done */
        if( DRV_I2C_DATA_OBJ_TASK_DONE  == hDriver->taskState ||
            DRV_I2C_DATA_OBJ_TASK_ERROR == hDriver->taskState )
        {
            /*Sets the buffer event as complete/error.*/
            if( DRV_I2C_DATA_OBJ_TASK_DONE == hDriver->taskState )
            {
                bufferObj->event = DRV_I2C_BUFFER_EVENT_COMPLETE;
            }
            else
            {
                bufferObj->event = DRV_I2C_BUFFER_EVENT_ERROR;
            }
            
            /* This means the buffer is completed. If there
               is a callback registered with client, then
               call it */
            client = (DRV_I2C_CLIENT_OBJ *)bufferObj->hClient;
            if( NULL != client->eventHandler )
            {   
                /* Before calling the event handler, the interrupt nesting
                   counter is incremented. This will allow driver routine that
                   are called from the event handler to know the interrupt
                   nesting level. Events are only generated for buffers that
                   were submitted using the buffer add routine */

                hDriver->interruptNestingCount ++;

                client->eventHandler( bufferObj->event,
                                      (DRV_I2C_BUFFER_HANDLE)bufferObj,
                                      client->context );

                /* Decrement the nesting count */
                hDriver->interruptNestingCount -- ;
                
            }
            
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_ADDRESS_SEND;
            
             /* Get the next buffer in the queue and deallocate
             * this buffer */
            hDriver->queue   = bufferObj->next;
            bufferObj->inUse = false;
            hDriver->queueSizeCurrent --;

            /* Reset the next and previous pointers */
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            
            /* Reset the current head's previous pointer */
            if ( NULL != hDriver->queue )
            {
                hDriver->queue->previous = NULL;
            }
            
            /* This means we should post the semaphore */
            //_DRV_I2C_SEM_POST(&(client->semTransferDone));

        }
    } 
    
    /* Process next buffer if queue is not empty */
    if( NULL != hDriver->queue )
    {
        /* check if task state is set to send the address */
		if( DRV_I2C_DATA_OBJ_TASK_ADDRESS_SEND == hDriver->taskState )
		{
            bufferObj = hDriver->queue;
        
            i2cModule->TWI_MMR.w = 0;
            i2cModule->TWI_MMR.w = TWI_MMR_DADR(bufferObj->slave7BitAddress) |
                                     TWI_MMR_IADRSZ(bufferObj->internalAddrSize);

            /* Set internal address */
            i2cModule->TWI_IADR.w  = 
                    TWI_IADR_IADR( _DRV_I2C_MakeInternalAddress ( bufferObj->internalAddress, 
                                                                    bufferObj->internalAddrSize ));
            
            /* check if request is receive only or 
             * transmit then receive with only internal address register used */
            if( 0 == bufferObj->writeSize )
            {
                i2cModule->TWI_MMR.w |= TWI_MMR_MREAD_Msk;

                /* Send a START Condition */
                i2cModule->TWI_CR.w = TWI_CR_START_Msk;

                if( 1 == bufferObj->readSize )
                {
                    /* Send Stop at the same time as start 
                     * if size of data to be received is 1*/
                    i2cModule->TWI_CR.w = TWI_CR_STOP_Msk;
                }

                /* Enable the receive interrupt */
                i2cModule->TWI_IER.w = TWI_IER_RXRDY_Msk;
                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
            }
            else
            {
                /* Enable the transmit interrupt */
                i2cModule->TWI_IER.w = TWI_IER_TXRDY_Msk;
                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE;
            }
		}
    }
	else
    {
        /* If the queue is empty, then disable the Txcomp, TX and Rx interrupt */
        i2cModule->TWI_IDR.w = TWI_IDR_TXCOMP_Msk | TWI_IDR_TXRDY_Msk |
                                 TWI_IDR_RXRDY_Msk;
        
        _DRV_I2C_InterruptSourceDisable(hDriver->interruptSource);
    }
    
    /* Release the mutex */
    _DRV_I2C_RELEASE_MUTEX(&(hDriver->mutexDriverInstance));
    
}

/*******************************************************************************
 End of File
 */
