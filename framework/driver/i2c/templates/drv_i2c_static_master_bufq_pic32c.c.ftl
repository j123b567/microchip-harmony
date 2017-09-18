/*******************************************************************************
  I2C Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_static_master_bufq_pic32c.c
	
  Summary:
    I2C driver implementation for the Master mode static single instance driver.

  Description:
    The I2C Master mode device driver provides a simple interface to manage the 
    I2C modules in Master mode on Microchip microcontrollers. This file contains 
    implementation for the I2C driver in master mode.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
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
#include "system_config.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the buffer object array */
DRV_I2C_BUFFER_OBJ             gDrvI2CBufferObj[ DRV_I2C_QUEUE_DEPTH_COMBINED ];

<#macro DRV_I2C_STATIC_VALUES DRV_INSTANCE HARDWARE_INSTANCE OPERATION_MODE I2C_INT_SRC QUEUE_SIZE>
<#if OPERATION_MODE == "DRV_I2C_MODE_MASTER">
extern DRV_I2C_OBJ  gDrvI2C${DRV_INSTANCE}Obj ;

/* Global variable to access hardware instance */
<#if HARDWARE_INSTANCE == "TWI_ID_0">
    <#assign I2C_MODULE = "TWI0_Module">
static twi_registers_t volatile *TWI0_Module = (twi_registers_t *)TWI_ID_0;
<#elseif HARDWARE_INSTANCE == "TWI_ID_1">
    <#assign I2C_MODULE = "TWI1_Module">
static twi_registers_t volatile *TWI1_Module = (twi_registers_t *)TWI_ID_1;
<#elseif HARDWARE_INSTANCE == "TWI_ID_2">
    <#assign I2C_MODULE = "TWI2_Module">
static twi_registers_t volatile *TWI2_Module = (twi_registers_t *)TWI_ID_2;
</#if>

// *****************************************************************************
// *****************************************************************************
// Section: Instance ${DRV_INSTANCE} static driver functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Receive   ( uint16_t slaveaddress,
                                                             void *rxBuffer,
                                                             size_t size,
                                                             void * callbackContext )

  Summary:
    This function reads data written from either Master or Slave.
    <p><b>Implementation:</b> Static</p>

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

DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Receive ( uint16_t   slaveAddress,
                                                       void *     buffer,
                                                       size_t     size,
                                                       void *     callbackContext )
{
    DRV_I2C_OBJ        * hDriver;
    DRV_I2C_BUFFER_OBJ * bufferObj = NULL;
    DRV_I2C_BUFFER_OBJ * iterator;
    uint32_t             i;
    uint32_t             flag10BitAddr = 0;

    if((size == 0) || (NULL == buffer))
    {
        SYS_ASSERT(false, "Invalid parameters");
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    hDriver = &gDrvI2C${DRV_INSTANCE}Obj;

    if(hDriver->queueSizeCurrent >= ${QUEUE_SIZE})
    {
        /* This means the queue is full. We cannot add
           this request */
        SYS_ASSERT(false, "Queue is full");
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    /* We will allow buffers to be added in the interrupt
       context of this I2C driver. But we must make
       sure that if we are in interrupt, then we should
       not modify mutexes. */

    if(0 == hDriver->interruptNestingCount)
    {
<#if CONFIG_USE_3RDPARTY_RTOS>
        /* Grab a mutex. This is okay because we are not in an
           interrupt context */

        if(OSAL_MUTEX_Lock(&(hDriver->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            /* We were able to take the mutex */            
        }
        else
        {
            /* The mutex acquisition timed out. Return with an
               invalid handle. This code will not execute
               if there is no RTOS. */
            return DRV_I2C_BUFFER_HANDLE_INVALID;
        }
</#if>
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        SYS_INT_SourceDisable(${I2C_INT_SRC});
</#if>
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
            bufferObj->nCurrentBytes      = 0;
            bufferObj->next               = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->previous           = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->drvInstance        = ${DRV_INSTANCE};
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
        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        SYS_INT_SourceEnable(${I2C_INT_SRC});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
        if( 0 == hDriver->interruptNestingCount )
        {
            /* Release mutex */
            OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
        }
</#if>
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
        ${I2C_MODULE}->TWI_MMR.w = 0;
        ${I2C_MODULE}->TWI_MMR.w = TWI_MMR_MREAD_Msk | 
                                     TWI_MMR_DADR(bufferObj->slave7BitAddress) |
                                     TWI_MMR_IADRSZ(bufferObj->internalAddrSize);

        /* if slave address is 10bit 
         * set MSB 3 bits in the internal address registers*/
        ${I2C_MODULE}->TWI_IADR.w  = 
                TWI_IADR_IADR( _DRV_I2C_MakeInternalAddress ( bufferObj->internalAddress, 
                                                                bufferObj->internalAddrSize ));

        /* Send a START Condition */
        ${I2C_MODULE}->TWI_CR.w = TWI_CR_START_Msk;

        /* Transmission to be stopped at the same time as start
           if the transmission size is of 1 byte*/
        if( 1 == bufferObj->readSize )
        {
            ${I2C_MODULE}->TWI_CR.w = TWI_CR_STOP_Msk;
        }

        hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        /* If the driver is configured for interrupt mode, then the following
         statement should enable the interrupt. */
        ${I2C_MODULE}->TWI_IER.w = TWI_IER_TXCOMP_Msk | TWI_IER_RXRDY_Msk;
</#if>
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
    }

<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>        
    SYS_INT_SourceEnable(${I2C_INT_SRC});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
    if( 0 == hDriver->interruptNestingCount )
    {
        /* Release mutex */
        OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
    }
</#if>
    return (DRV_I2C_BUFFER_HANDLE) bufferObj;    
}

/*******************************************************************************
  Function:
    DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Transmit( DRV_HANDLE  handle, 
                                                           uint16_t    slaveaddress,
                                                           void       *txBuffer,
                                                           size_t      size,
                                                           void       *context);

  Summary:
    This function writes data to Master or Slave.
    <p><b>Implementation:</b>Static</p>

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

DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Transmit ( uint16_t   slaveAddress,
                                                        void *     buffer,
                                                        size_t     size,
                                                        void *     context )
{
    DRV_I2C_OBJ        * hDriver;
    DRV_I2C_BUFFER_OBJ * bufferObj = NULL;
    DRV_I2C_BUFFER_OBJ * iterator;
    uint32_t             i;
    uint32_t             flag10BitAddr;
    
    if((size == 0) || (NULL == buffer))
    {
        /* We either got an invalid source pointer or 0 bytes to transfer */
        SYS_ASSERT(false, "Invalid parameters");
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }

    hDriver = &gDrvI2C${DRV_INSTANCE}Obj;

    if(hDriver->queueSizeCurrent >= ${QUEUE_SIZE})
    {
        /* This means the queue is full. We cannot add
           this request */
        SYS_ASSERT(false, "Queue Size is full");
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
<#if CONFIG_USE_3RDPARTY_RTOS>
        if(OSAL_MUTEX_Lock(&(hDriver->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {

        }
        else
        {
            /* The mutex acquisition timed out. Return with an
               invalid handle. This code will not execute
               if there is no RTOS. */
            return DRV_I2C_BUFFER_HANDLE_INVALID;
        }
</#if>
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        SYS_INT_SourceDisable(${I2C_INT_SRC});
</#if>
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
            bufferObj->nCurrentBytes      = 0;
            bufferObj->next               = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->previous           = ( DRV_I2C_BUFFER_OBJ * ) NULL;
            bufferObj->drvInstance        = ${DRV_INSTANCE};
            bufferObj->currentState       = DRV_I2C_BUFFER_IS_IN_QUEUE;
            bufferObj->flags              = DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_WRITE;
            bufferObj->event              = DRV_I2C_BUFFER_EVENT_PENDING;
            
            break;
        }
    }

    if( DRV_I2C_QUEUE_DEPTH_COMBINED == i )
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_I2C_QUEUE_DEPTH
           parameter is configured to be less */
        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        SYS_INT_SourceEnable(${I2C_INT_SRC});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
        if( 0 == hDriver->interruptNestingCount )
        {
            /* Release mutex */
            OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
        }
</#if>
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
        ${I2C_MODULE}->TWI_MMR.w = 0;
        ${I2C_MODULE}->TWI_MMR.w = TWI_MMR_DADR(bufferObj->slave7BitAddress) |
                                     TWI_MMR_IADRSZ(bufferObj->internalAddrSize);

        /* Set internal address */
        ${I2C_MODULE}->TWI_IADR.w = 
                TWI_IADR_IADR( _DRV_I2C_MakeInternalAddress ( bufferObj->internalAddress, 
                                                                bufferObj->internalAddrSize ));

        hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE;
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>   
        /* If the driver is configured for interrupt mode, then the following
         statement should enable the interrupt. */
        ${I2C_MODULE}->TWI_IER.w = TWI_IER_TXCOMP_Msk | TWI_IER_TXRDY_Msk;
</#if>
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
    }
    
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>   
    SYS_INT_SourceEnable(${I2C_INT_SRC});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
    if( 0 == hDriver->interruptNestingCount )
    {
        /* Release mutex */
        OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
    }
</#if>
    
    return (DRV_I2C_BUFFER_HANDLE) bufferObj;    
}

/*******************************************************************************
  Function:
    DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_TransmiThenReceive ( uint16_t   deviceaddress,
                                                       void       *txBuffer,
                                                       size_t     writeSize,
                                                       void       *rxBuffer,
                                                       size_t     readSize,
                                                       void       *context)

  Summary:
    This function writes data to Slave, inserts restart and requests read from
    slave.
    <p><b>Implementation:</b>Static</p>

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

DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_TransmitThenReceive ( uint16_t   slaveAddress,
                                                                   void *     writeBuffer,
                                                                   size_t     writeSize,
                                                                   void *     readBuffer,
                                                                   size_t     readSize,
                                                                   void *     callbackContext )
{
    DRV_I2C_OBJ        * hDriver;
    DRV_I2C_BUFFER_OBJ * bufferObj = NULL;
    DRV_I2C_BUFFER_OBJ * iterator;
    uint32_t             i = 0;
    uint32_t             j = 0;
    uint32_t             flag10BitAddr = 0;
    
    if((readSize == 0)        || (writeSize == 0) ||
       (NULL  == writeBuffer) || (NULL == readBuffer))
    {
        /* We either got an invalid source pointer or 0 bytes to transfer */
        return DRV_I2C_BUFFER_HANDLE_INVALID;
    }
    
    hDriver = &gDrvI2C${DRV_INSTANCE}Obj;

    if(hDriver->queueSizeCurrent >= ${QUEUE_SIZE})
    {
        /* This means the queue is full. We cannot add
           this request */
        SYS_ASSERT(false, "Invalid parameters");
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
<#if CONFIG_USE_3RDPARTY_RTOS>
        if(OSAL_MUTEX_Lock(&(hDriver->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            
        }
        else
        {
            /* The mutex acquisition timed out. Return with an
               invalid handle. This code will not execute
               if there is no RTOS. */
            return DRV_I2C_BUFFER_HANDLE_INVALID;
        }
</#if>
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        SYS_INT_SourceDisable(${I2C_INT_SRC});
</#if>
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
            bufferObj->nCurrentBytes = 0;
            bufferObj->drvInstance   = ${DRV_INSTANCE};
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
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        SYS_INT_SourceEnable(${I2C_INT_SRC});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
        if( 0 == hDriver->interruptNestingCount )
        {
            /* Release mutex */
            OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
        }
</#if>
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
        ${I2C_MODULE}->TWI_MMR.w = 0;
        ${I2C_MODULE}->TWI_MMR.w = TWI_MMR_DADR(bufferObj->slave7BitAddress) |
                                     TWI_MMR_IADRSZ(bufferObj->internalAddrSize);

        /* Set internal address */
        ${I2C_MODULE}->TWI_IADR.w = 
                TWI_IADR_IADR( _DRV_I2C_MakeInternalAddress ( bufferObj->internalAddress, 
                                                                bufferObj->internalAddrSize ));

        /* If only internal address registers are used, 
         * read operation performs both transmit, restart and receive */
        if( 0 == bufferObj->writeSize )
        {
            ${I2C_MODULE}->TWI_MMR.w |= TWI_MMR_MREAD_Msk;

            /* Send a START Condition */
            ${I2C_MODULE}->TWI_CR.w = TWI_CR_START_Msk;

            /* Transmission to be stopped at the same time as start
           if the transmission size is of 1 byte*/
            if( 1 == bufferObj->readSize )
            {
                ${I2C_MODULE}->TWI_CR.w = TWI_CR_STOP_Msk;
            }
            
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
            ${I2C_MODULE}->TWI_IER.w = TWI_IER_RXRDY_Msk;
</#if>
        }
        else
        {
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE;
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
            ${I2C_MODULE}->TWI_IER.w = TWI_IER_TXRDY_Msk;
</#if>
        }
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        /* If the driver is configured for interrupt mode, then the following
         statement should enable the interrupt. */
        ${I2C_MODULE}->TWI_IER.w = TWI_IER_TXCOMP_Msk;
</#if>
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

    }
    
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>   
    SYS_INT_SourceEnable(${I2C_INT_SRC});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
    if( 0 == hDriver->interruptNestingCount )
    {
        /* Release mutex */
        OSAL_MUTEX_Unlock(&(hDriver->mutexDriverInstance));
    }
</#if>
    
    return (DRV_I2C_BUFFER_HANDLE) bufferObj;
    
}

void DRV_I2C${DRV_INSTANCE}_Tasks ( void )
{
    DRV_I2C_OBJ        * hDriver = &gDrvI2C${DRV_INSTANCE}Obj;
    DRV_I2C_BUFFER_OBJ * bufferObj;
    __TWI_SR_bits_t    status;
    
<#if CONFIG_USE_3RDPARTY_RTOS>
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
</#if>
    /* In this function, the driver checks if there are any buffers in queue. If
       so the buffer is serviced. A buffer that is serviced completely is
       removed from the queue. Start by getting the buffer at the head of the
       queue */
    
    if( NULL != hDriver->queue )
    {
        bufferObj = hDriver->queue;
        
        /* gets the module status */
        status.w = ${I2C_MODULE}->TWI_SR.w;

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
                    ${I2C_MODULE}->TWI_THR.w =  TWI_THR_TXDATA(bufferObj->writeBuffer[bufferObj->nCurrentBytes]);
                    bufferObj->nCurrentBytes++;

                    /* checks if transmission reached at the end */
                    if( bufferObj->nCurrentBytes == bufferObj->writeSize )
                    {
                        /* checks if it is a transmit only request */
                        if( 0 == bufferObj->readSize )
                        {
                            /* Sends a stop bit */
                            ${I2C_MODULE}->TWI_CR.w = TWI_CR_STOP_Msk;

                            /* read the module status again to know if 
                               tranmission complete flag is set */
                            status.w = ${I2C_MODULE}->TWI_SR.w;
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

                            ${I2C_MODULE}->TWI_MMR.w = 0;
                            ${I2C_MODULE}->TWI_MMR.w = TWI_MMR_MREAD_Msk | 
                                                         TWI_MMR_DADR(bufferObj->slave7BitAddress);

                            /* Send a START Condition */
                            ${I2C_MODULE}->TWI_CR.w = TWI_CR_START_Msk;

                            /* Send Stop at the same time as start 
                             * if size of data to be received is 1*/
                            if( 1 == bufferObj->readSize )
                            {
                                ${I2C_MODULE}->TWI_CR.w = TWI_CR_STOP_Msk;
                            }
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
                            /* Enable the RXRDY interrupt */
                            ${I2C_MODULE}->TWI_IER.w = TWI_IER_RXRDY_Msk;
</#if>
                            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
                        }
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>                   
                        /* Disable the TXRDY interrupt*/
                        ${I2C_MODULE}->TWI_IDR.w = TWI_IDR_TXRDY_Msk;
</#if>
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
                        ${I2C_MODULE}->TWI_CR.w = TWI_CR_STOP_Msk;
                    }

                    /* read the received data */
                    bufferObj->readBuffer[bufferObj->nCurrentBytes] = ${I2C_MODULE}->TWI_RHR.RXDATA;
                    bufferObj->nCurrentBytes++;

                    /* checks if transmission has reached at the end */
                    if( bufferObj->nCurrentBytes == bufferObj->readSize )
                    {
                        /* read the module status again to know if 
                           tranmission complete flag is set */
                        status.w = ${I2C_MODULE}->TWI_SR.w;
                        if( DRV_I2C_TRUE != status.TXCOMP )
                        {   
                            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_COMPLETE;
                        }
                        else
                        {
                            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_DONE;
                        }
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
                        /* Disable the RXRDY interrupt*/
                        ${I2C_MODULE}->TWI_IDR.w = TWI_IDR_RXRDY_Msk;
</#if>
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
            /* Sets the buffer event to completed/error. */
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
            if( NULL != hDriver->eventHandler )
            {                
                /* Before calling the event handler, the interrupt nesting
                   counter is incremented. This will allow driver routine that
                   are called from the event handler to know the interrupt
                   nesting level. Events are only generated for buffers that
                   were submitted using the buffer add routine */

                hDriver->interruptNestingCount ++;

                hDriver->eventHandler( bufferObj->event,
                                      (DRV_I2C_BUFFER_HANDLE)bufferObj,
                                      hDriver->context );

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
        }
    } 
    
    /* Process next buffer if queue is not empty */
    if( NULL != hDriver->queue )
    {
        /* check if task state is set to send the address */
		if( DRV_I2C_DATA_OBJ_TASK_ADDRESS_SEND == hDriver->taskState )
		{
            bufferObj = hDriver->queue;
        
            ${I2C_MODULE}->TWI_MMR.w = 0;
            ${I2C_MODULE}->TWI_MMR.w = TWI_MMR_DADR(bufferObj->slave7BitAddress) |
                                         TWI_MMR_IADRSZ(bufferObj->internalAddrSize);

            /* Set internal address */
            ${I2C_MODULE}->TWI_IADR.w  = 
                    TWI_IADR_IADR( _DRV_I2C_MakeInternalAddress ( bufferObj->internalAddress, 
                                                                    bufferObj->internalAddrSize ));
            
            /* check if request is receive only or 
             * transmit then receive with only internal address register used */
            if( 0 == bufferObj->writeSize )
            {
                ${I2C_MODULE}->TWI_MMR.w |= TWI_MMR_MREAD_Msk;

                /* Send a START Condition */
                ${I2C_MODULE}->TWI_CR.w = TWI_CR_START_Msk;

                if( 1 == bufferObj->readSize )
                {
                    /* Send Stop at the same time as start 
                     * if size of data to be received is 1*/
                    ${I2C_MODULE}->TWI_CR.w = TWI_CR_STOP_Msk;
                }
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
                /* Enable the receive interrupt */
                ${I2C_MODULE}->TWI_IER.w = TWI_IER_RXRDY_Msk;
</#if>
                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
            }
            else
            {
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
                /* Enable the transmit interrupt */
                ${I2C_MODULE}->TWI_IER.w = TWI_IER_TXRDY_Msk;
</#if>
                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE;
            }
		}
    }
	else
    {
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        /* If the queue is empty, then disable the Txcomp, TX and Rx interrupt */
        ${I2C_MODULE}->TWI_IDR.w = TWI_IDR_TXCOMP_Msk | TWI_IDR_TXRDY_Msk |
                                     TWI_IDR_RXRDY_Msk;      
        SYS_INT_SourceDisable(${I2C_INT_SRC});
</#if>
    }
<#if CONFIG_USE_3RDPARTY_RTOS>
    /* Release the mutex */
    _DRV_I2C_RELEASE_MUTEX(&(hDriver->mutexDriverInstance));
</#if>
}
</#if>
</#macro>

<#if CONFIG_DRV_I2C_INST_IDX0 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="0"
HARDWARE_INSTANCE=CONFIG_DRV_I2C_PERIPHERAL_ID_IDX0
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX0
I2C_INT_SRC=CONFIG_DRV_I2C_INT_SRC_IDX0
QUEUE_SIZE=CONFIG_DRV_I2C_QUEUE_SIZE_IDX0
/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="1"
HARDWARE_INSTANCE=CONFIG_DRV_I2C_PERIPHERAL_ID_IDX1
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX1 
I2C_INT_SRC=CONFIG_DRV_I2C_INT_SRC_IDX1
QUEUE_SIZE=CONFIG_DRV_I2C_QUEUE_SIZE_IDX1
/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="2"
HARDWARE_INSTANCE=CONFIG_DRV_I2C_PERIPHERAL_ID_IDX2
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX2 
I2C_INT_SRC=CONFIG_DRV_I2C_INT_SRC_IDX2
QUEUE_SIZE=CONFIG_DRV_I2C_QUEUE_SIZE_IDX2
/>
</#if>

/*******************************************************************************
 End of File
 */