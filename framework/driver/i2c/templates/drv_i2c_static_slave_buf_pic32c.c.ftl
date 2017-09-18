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
<#macro DRV_I2C_STATIC_VALUES DRV_INSTANCE HARDWARE_INSTANCE OPERATION_MODE I2C_INT_SRC QUEUE_SIZE>
<#if OPERATION_MODE == "DRV_I2C_MODE_SLAVE">
extern DRV_I2C_OBJ  gDrvI2C${DRV_INSTANCE}Obj ;
DRV_I2C_BUFFER_OBJ  gDrvI2C${DRV_INSTANCE}BufferObj;

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

// *****************************************************************************
/* Function:
    void DRV_I2C${DRV_INSTANCE}_SlaveCallbackSet ( const DRV_I2C_CallBack callback,
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
    callback  - pointer to the callback function.

    context   - The value of parameter will be passed back to the client unchanged, 
                when the callback function is called. It can be used to identify 
                any client specific data

  Returns:
    None
  
*/

void DRV_I2C${DRV_INSTANCE}_SlaveCallbackSet
(
    const DRV_I2C_CallBack callback,
    const uintptr_t context
)
{
    DRV_I2C_OBJ * hDriver = &gDrvI2C${DRV_INSTANCE}Obj;
    
    /* Register the event handler with the client */
    hDriver->operationStarting = callback;
    hDriver->callbackContext = (void *) context;
}

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

DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Receive ( uint16_t slaveaddress,
                                                       void *     buffer,
                                                       size_t     size,
                                                       void *     callbackContext )
{
    DRV_I2C_OBJ        * hDriver = &gDrvI2C${DRV_INSTANCE}Obj;
    DRV_I2C_BUFFER_OBJ * bufferObj = &gDrvI2C${DRV_INSTANCE}BufferObj;

    bufferObj->readBuffer    = ( uint8_t* ) buffer;
    bufferObj->readSize      = size;
    bufferObj->writeBuffer   = NULL;
    bufferObj->drvInstance   = ${DRV_INSTANCE};
    bufferObj->nCurrentBytes = 0;
    bufferObj->flags         = DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_READ;

    hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ;
    
    return (DRV_I2C_BUFFER_HANDLE) bufferObj;
}

/*******************************************************************************
  Function:
    DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Transmit( uint16_t    slaveaddress,
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

DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Transmit ( uint16_t slaveaddress,
                                                        void *   buffer,
                                                        size_t   size,
                                                        void *   context )
{
    DRV_I2C_OBJ        * hDriver = &gDrvI2C${DRV_INSTANCE}Obj;
    DRV_I2C_BUFFER_OBJ * bufferObj = &gDrvI2C${DRV_INSTANCE}BufferObj;

    bufferObj->readSize      = 0;
    bufferObj->writeSize     = size;
    bufferObj->readBuffer    = NULL;
    bufferObj->writeBuffer   = (uint8_t*)buffer;
    bufferObj->drvInstance   = ${DRV_INSTANCE};
    bufferObj->nCurrentBytes = 0;
    bufferObj->next          = ( DRV_I2C_BUFFER_OBJ * ) NULL;
    bufferObj->previous      = ( DRV_I2C_BUFFER_OBJ * ) NULL;
    bufferObj->flags         = DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_WRITE;

    hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE;
            
    return (DRV_I2C_BUFFER_HANDLE) bufferObj; 
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

void DRV_I2C${DRV_INSTANCE}_Tasks ( void )
{
    DRV_I2C_OBJ        * hDriver = &gDrvI2C${DRV_INSTANCE}Obj;
    DRV_I2C_BUFFER_OBJ * bufferObj = &gDrvI2C${DRV_INSTANCE}BufferObj;
    __TWI_SR_bits_t    status;

    status.w = ${I2C_MODULE}->TWI_SR.w;
    
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
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
            ${I2C_MODULE}->TWI_IDR.w = TWI_IDR_TXRDY_Msk;
            ${I2C_MODULE}->TWI_IER.w = TWI_IER_EOSACC_Msk;
</#if>
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
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
                ${I2C_MODULE}->TWI_IER.w = TWI_IER_EOSACC_Msk | TWI_IER_TXRDY_Msk;
</#if>
                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_DUMMY_WRITE;
            }
            else /* Master is going to write data to slave */
            {
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
                ${I2C_MODULE}->TWI_IER.w = TWI_IER_EOSACC_Msk | TWI_IER_RXRDY_Msk;
</#if>
                hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_TRANSFER_DUMMY_READ;
            }
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>            
            ${I2C_MODULE}->TWI_IDR.w = TWI_IDR_SVACC_Msk;
</#if>            
            /* calls slave callback function */
            if( NULL != hDriver->operationStarting )
            {
                hDriver->operationStarting( bufferObj->event, hDriver->callbackContext );
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
                bufferObj->readBuffer[bufferObj->nCurrentBytes] = ${I2C_MODULE}->TWI_RHR.RXDATA;
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
                
                ${I2C_MODULE}->TWI_RHR.RXDATA;
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
                ${I2C_MODULE}->TWI_THR.w = TWI_THR_TXDATA(bufferObj->writeBuffer[bufferObj->nCurrentBytes]);
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
                ${I2C_MODULE}->TWI_THR.w = TWI_THR_TXDATA(0xFF);
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
               is a callback registered with driver, then
               call it */
            
            if( NULL != hDriver->eventHandler )
            {
                /* calls the event handler */
                hDriver->eventHandler( bufferObj->event,
                                      (DRV_I2C_BUFFER_HANDLE)bufferObj,
                                       hDriver->context );
            }
            
            /* resets the task state */
            hDriver->taskState = DRV_I2C_DATA_OBJ_TASK_ADDRESS_RECEIVED;
            
            /* reset the buffer object */
            bufferObj->readBuffer    = NULL;
            bufferObj->writeBuffer   = NULL;
            bufferObj->readSize      = 0;
            bufferObj->writeSize     = 0;
            bufferObj->nCurrentBytes = 0;
            bufferObj->flags         = 0;
        }
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        /* disables all interrupt */
        ${I2C_MODULE}->TWI_IDR.w = TWI_IDR_EOSACC_Msk | TWI_IDR_TXRDY_Msk | 
                                     TWI_IDR_RXRDY_Msk;
        ${I2C_MODULE}->TWI_IER.w = TWI_IER_SVACC_Msk;
</#if>
    }
    
    return;
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