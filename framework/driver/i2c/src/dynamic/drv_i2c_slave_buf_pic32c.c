/*******************************************************************************
  I2C Device Driver Slave Mode Implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_buffer_queue_pic32c.c

  Summary:
    I2C Device Driver Slave Mode Implementation.

  Description:
 This file contains the Dynamic mode implementation of the I2C driver 
 Slave mode routines.
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
extern DRV_I2C_OBJ             gDrvI2CObj      [ DRV_I2C_INSTANCES_NUMBER ] ;

/* This is the client object array. */
extern DRV_I2C_CLIENT_OBJ      gDrvI2CClientObj[ DRV_I2C_CLIENTS_NUMBER ];

/* This is the buffer object array */
static DRV_I2C_BUFFER_OBJ      gDrvI2CSlaveBufferObj [ DRV_I2C_INSTANCES_NUMBER ];

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void DRV_I2C_SlaveCallbackSet ( const DRV_HANDLE handle,
                                    const DRV_I2C_CallBack callback,
                                    const uintptr_t context )

  Summary:
    Allows a client to identify a Slave Callback function for the driver
    to call back when drivers needs to initiate a read or write operation.

  Description:
    This function allows a client to identify a Slave Callback function
    for the driver to call back when drivers needs to initiate a read or write 
    operation. When the I2C Slave driver receives a read or write event from 
    master the callback is called to initiate the user defined action.
    The callback should be set before the master requests for read or write 
    operations. The event handler once set, persists until the client closes 
    the driver or sets another event handler. 
    
    In Slave mode, a callback event is registered to let the application 
    know that master has requested for either read or write operation. 
    DRV_I2C_BUFFER_SLAVE_READ_REQUESTED is set when the master sends data and 
    wants slave to read.
    DRV_I2C_BUFFER_SLAVE_WRITE_REQUESTED is set when the master tries to read 
    data from slave and wants slave to send the data.

  Parameters:
    handle    - A valid open-instance handle, returned from the driver's 
                open routine

    callback  - pointer to the callback function.

    context   - The value of parameter will be passed back to the client unchanged, 
                when the callback function is called. It can be used to identify 
                any client specific data

  Returns:
    None
  
*/

void DRV_I2C_SlaveCallbackSet
(
    const DRV_HANDLE handle,
    const DRV_I2C_CallBack callback,
    const uintptr_t context
)
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ * hDriver;

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
    
    hDriver = clientObj->hDriver;

    /* Register the event handler with the client */
    hDriver->operationStarting = callback;
    hDriver->context = (void *) context;

}

/*******************************************************************************
  Function:
    DRV_I2C_BUFFER_HANDLE _DRV_I2C_Receive   (   DRV_HANDLE handle,
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

DRV_I2C_BUFFER_HANDLE _DRV_I2C_SlaveReceive (   DRV_HANDLE handle, 
                                                void *     buffer,
                                                size_t     size,
                                                void *     callbackContext )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    DRV_I2C_BUFFER_OBJ * bufferObj = NULL;
    uint32_t             drvIndex;
    
    clientObj = (DRV_I2C_CLIENT_OBJ *) handle;
    hDriver   = clientObj->hDriver;
    drvIndex  = hDriver - &gDrvI2CObj[0];

    /* This means this object is free.
     * Configure the object and then
     * break */
    bufferObj = &gDrvI2CSlaveBufferObj[drvIndex];
    bufferObj->readBuffer    = ( uint8_t* ) buffer;
    bufferObj->readSize      = size;
    bufferObj->writeBuffer   = NULL;
    bufferObj->hClient       = clientObj;
    bufferObj->nCurrentBytes = 0;
    bufferObj->flags         = DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_READ;

    hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
    
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

DRV_I2C_BUFFER_HANDLE _DRV_I2C_SlaveTransmit (  DRV_HANDLE handle, 
                                                void *     buffer,
                                                size_t     size,
                                                void *     context )
{
    DRV_I2C_CLIENT_OBJ * clientObj;
    DRV_I2C_OBJ        * hDriver;
    DRV_I2C_BUFFER_OBJ * bufferObj = NULL;
    uint32_t             drvIndex;
    
    /* Validate the driver handle */
    clientObj = (DRV_I2C_CLIENT_OBJ *)handle;
    hDriver   = clientObj->hDriver;
    drvIndex  = hDriver - &gDrvI2CObj[0];
            
    bufferObj = &gDrvI2CSlaveBufferObj[drvIndex];
    bufferObj->readSize           = 0;
    bufferObj->writeSize          = size;
    bufferObj->readBuffer         = NULL;
    bufferObj->writeBuffer        = (uint8_t*)buffer;
    bufferObj->hClient            = clientObj;
    bufferObj->nCurrentBytes      = 0;
    bufferObj->next               = ( DRV_I2C_BUFFER_OBJ * ) NULL;
    bufferObj->previous           = ( DRV_I2C_BUFFER_OBJ * ) NULL;
    bufferObj->flags              = DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_WRITE;

    hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE;
            
    return (DRV_I2C_BUFFER_HANDLE) bufferObj; 
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

void _DRV_I2C_SlaveTasks ( SYS_MODULE_OBJ object )
{
    DRV_I2C_OBJ        * hDriver = &gDrvI2CObj[object];
    DRV_I2C_BUFFER_OBJ * bufferObj;
    DRV_I2C_CLIENT_OBJ * client;
    twi_registers_t  * i2cModule;
    __TWI_SR_bits_t    status;

    i2cModule  = hDriver->i2cModule;
    status.w = i2cModule->TWI_SR.w;
    bufferObj  = &gDrvI2CSlaveBufferObj[object];
    
    /* checks if the slave address is matched */
    if ( DRV_I2C_TRUE == status.SVACC ||
         DRV_I2C_TRUE == status.GACC ) 
    {             
        /* checks if it is a general access, 
         * overrun error or under run error */
        if( NULL == hDriver->operationStarting ||
            DRV_I2C_TRUE == status.GACC )
        {
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_SLAVE_OPERATION_START;
            if( NULL == hDriver->operationStarting )
            {
                bufferObj->event = DRV_I2C_BUFFER_EVENT_SLAVE_ERROR_INVALID_CALLBACK;
            }
            else
            {
                bufferObj->event = DRV_I2C_BUFFER_EVENT_SLAVE_GENERAL_CALL;
            }
        }
        
        if( DRV_I2C_TRUE == status.NACK )
        {
            i2cModule->TWI_IDR.w = TWI_IDR_TXRDY_Msk;
            i2cModule->TWI_IER.w = TWI_IER_EOSACC_Msk;
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_COMPLETE;
        }

        if ( DRV_I2C_DATA_OBJ_TASK_ADDRESS_RECEIVED == hDriver->taskState )
        {    
            /* Master is going to read data from slave */
            if( DRV_I2C_TRUE == status.SVREAD )
            {
                bufferObj->event = DRV_I2C_BUFFER_SLAVE_WRITE_REQUESTED;
            }
            else /* Master is going to write data to slave */
            {
                bufferObj->event = DRV_I2C_BUFFER_SLAVE_READ_REQUESTED;
            }
            
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_SLAVE_OPERATION_START;
        }
        
        if( DRV_I2C_DATA_OBJ_TASK_SLAVE_OPERATION_START == hDriver->taskState )
        {
            /* Master is going to read data from slave */
            if( DRV_I2C_TRUE == status.SVREAD )
            {
                i2cModule->TWI_IER.w = TWI_IER_EOSACC_Msk | TWI_IER_TXRDY_Msk;
                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_DUMMY_WRITE;
            }
            else /* Master is going to write data to slave */
            {
                i2cModule->TWI_IER.w = TWI_IER_EOSACC_Msk | TWI_IER_RXRDY_Msk;
                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_DUMMY_READ;
            }
            
            i2cModule->TWI_IDR.w = TWI_IDR_SVACC_Msk;
            
            /* calls slave callback function */
            if( NULL != hDriver->operationStarting )
            {
                hDriver->operationStarting( bufferObj->event, hDriver->context );
            }
        }
        
        switch( hDriver->taskState )
        {
            case DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ:
            {
                if( DRV_I2C_TRUE == status.OVRE )
                {
                    bufferObj->event = DRV_I2C_BUFFER_EVENT_SLAVE_ERROR_OVER_RUN;
                }
                
                /* checks if data is received */
                if( DRV_I2C_TRUE != status.RXRDY )
                {
                    break;
                }

                /* reads the received data */
                bufferObj->readBuffer[bufferObj->nCurrentBytes] = i2cModule->TWI_RHR.RXDATA;
                bufferObj->nCurrentBytes++;

                if( bufferObj->readSize == bufferObj->nCurrentBytes )
                {
                    /* Add selectively */
                    bufferObj->event   = DRV_I2C_BUFFER_EVENT_SLAVE_EXCESS_READ_REQUESTED;
                    hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_SLAVE_OPERATION_START;
                }
                
                break;
            }
            
            case DRV_I2C_DATA_OBJ_TASK_TRANSFER_DUMMY_READ:
            {
                if( DRV_I2C_TRUE == status.OVRE )
                {
                    bufferObj->event = DRV_I2C_BUFFER_EVENT_SLAVE_ERROR_OVER_RUN;
                }
                
                /* checks if data is received */
                if( DRV_I2C_TRUE != status.RXRDY )
                {
                    break;
                }
                
                i2cModule->TWI_RHR.RXDATA;
                bufferObj->nCurrentBytes++;
                
                break;
            }

            case DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE:
            {   
                if( DRV_I2C_TRUE == status.UNRE )
                {
                    bufferObj->event = DRV_I2C_BUFFER_EVENT_SLAVE_ERROR_UNDER_RUN;
                }
                
                /* checks if the slave is ready to transmit the next byte */
                if( DRV_I2C_TRUE != status.TXRDY )
                {
                    break;
                }

                /* transmits the next byte */
                i2cModule->TWI_THR.w = TWI_THR_TXDATA(bufferObj->writeBuffer[bufferObj->nCurrentBytes]);
                bufferObj->nCurrentBytes++;

                if( bufferObj->writeSize == bufferObj->nCurrentBytes )
                {
                    bufferObj->event   = DRV_I2C_BUFFER_EVENT_SLAVE_EXCESS_WRITE_REQUESTED;
                    hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_SLAVE_OPERATION_START;
                }
                
                break;
            }
            
            case DRV_I2C_DATA_OBJ_TASK_TRANSFER_DUMMY_WRITE:
            {   
                if( DRV_I2C_TRUE == status.UNRE )
                {
                    bufferObj->event = DRV_I2C_BUFFER_EVENT_SLAVE_ERROR_UNDER_RUN;
                }
                
                /* checks if the slave is ready to transmit the next byte */
                if( DRV_I2C_TRUE != status.TXRDY )
                {
                    break;
                }

                /* transmits the next byte */
                i2cModule->TWI_THR.w = TWI_THR_TXDATA(0xFF);
                bufferObj->nCurrentBytes++;
                
                break;
            }
            
            default:
            {
                break;
            }
        }       
    }
    else
    { 
        /* Checks whether Master has ended the slave access */
        if ( DRV_I2C_TRUE == status.EOSACC )
        {
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_DONE;
            
            /* sets the task state to done */
            if( DRV_I2C_BUFFER_EVENT_SLAVE_ERROR_INVALID_CALLBACK != bufferObj->event &&
                DRV_I2C_BUFFER_EVENT_SLAVE_ERROR_OVER_RUN         != bufferObj->event &&
                DRV_I2C_BUFFER_EVENT_SLAVE_ERROR_UNDER_RUN        != bufferObj->event )
            {
                bufferObj->event = DRV_I2C_BUFFER_EVENT_COMPLETE;   
            }
                           
            /* This means the buffer is completed. If there
               is a callback registered with client, then
               call it */
            client = (DRV_I2C_CLIENT_OBJ *)bufferObj->hClient;
            if( NULL != client )
            {
                if( NULL != client->eventHandler )
                {
                    /* calls the event handler */
                    client->eventHandler( bufferObj->event,
                                          (DRV_I2C_BUFFER_HANDLE)bufferObj,
                                          client->context );
                }
            }
            
            /* resets the task state */
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_ADDRESS_RECEIVED;
            
            /* reset the buffer object */
            bufferObj->readBuffer    = NULL;
            bufferObj->writeBuffer   = NULL;
            bufferObj->readSize      = 0;
            bufferObj->writeSize     = 0;
            bufferObj->hClient       = NULL;
            bufferObj->nCurrentBytes = 0;
            bufferObj->flags         = 0;
        }
        
        /* disables all interrupt */
        i2cModule->TWI_IDR.w = TWI_IDR_EOSACC_Msk | TWI_IDR_TXRDY_Msk | 
                                 TWI_IDR_RXRDY_Msk;
        i2cModule->TWI_IER.w = TWI_IER_SVACC_Msk;
    }
    
    return;
}

/*******************************************************************************
 End of File
 */
