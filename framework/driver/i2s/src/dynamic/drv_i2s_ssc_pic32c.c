/*******************************************************************************
  I2S Driver Dynamic implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2s_ssc_pic32c.c

  Summary:
    I2S Driver Dynamic Implementation.

  Description:
    This file contains the Dynamic mode implementation of the SSC driver,
    a underlying driver of I2S interface driver. This driver implementation is
    applicable only to SSC module. The driver supports DMA based 
    implementation only.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
#include "driver/i2s/src/drv_i2s_ssc_pic32c_local.h"

// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects that are available on the part

  Description:
    This data type defines the hardware instance objects that are available on
    the part, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all micro-controllers.
 */
DRV_I2S_SSC_OBJ gDrvI2SSSCObj[DRV_I2S_SSC_INSTANCES_NUMBER];

// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the client instances objects

  Description:
    This data type defines the client instance objects that are available on
    the part, so as to capture the client state of the instance.
    It uses the configuration of maximum number of clients which can get
    registered per hardware instance.

  Remarks:
    Not all modes are available on all micro-controllers.
 */
DRV_I2S_CLIENT_OBJ gDrvI2SClientObj[DRV_I2S_SSC_CLIENTS_NUMBER];

// *****************************************************************************
/* Driver buffer instance objects.

  Summary:
    Defines the readwrite buffer instances objects

  Description:
    This data type defines the buffer instance objects

  Remarks:
    None
 */
DRV_I2S_BUFFER_OBJECT gDrvI2SBufferQObj[DRV_I2S_QUEUE_DEPTH_COMBINED];

// *****************************************************************************
/* Driver buffer instance objects.

  Summary:
    Defines the read buffer instances objects

  Description:
    This data type defines the buffer instance objects

  Remarks:
    None
 */
DRV_I2S_BUFFER_OBJECT gDrvI2SReadBufferQObj[QUEUE_SIZE_RX_IDX0];
// *****************************************************************************
/* Driver buffer instance objects.

  Summary:
    Defines the write buffer instances objects

  Description:
    This data type defines the buffer instance objects

  Remarks:
    None
 */
DRV_I2S_BUFFER_OBJECT gDrvI2SWriteBufferQObj[QUEUE_SIZE_TX_IDX0];

// *****************************************************************************
/* Driver common data object

  Summary:
    Defines the common data object

  Description:
    This object maintains data that is required by all I2S
   driver instances

  Remarks:
    None
 */
DRV_I2S_COMMON_DATA_OBJ gDrvI2SCommonDataObj;

// *****************************************************************************
/* Driver Unique buffer handle

  Summary:
    Defines the unique buffer handle

  Description:
    This data type defines Unique buffer handle.

  Remarks:
    None
 */
static uint16_t gDrvI2SUniqueBufferHandle;

// *****************************************************************************
// *****************************************************************************
// Section: I2S Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*   Function:
    SYS_MODULE_OBJ DRV_I2S_Initialize ( const SYS_MODULE_INDEX drvIndex,
                                        const SYS_MODULE_INIT *const init )

  Summary:
    Initializes hardware and data for the instance of the I2S module

  Description:
    This routine initializes the I2S driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the init parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized. The driver instance index is
    independent of the I2S module ID. For example, driver instance 0 can be
    assigned to I2S2.  If the driver is built statically, then some of the
    initialization parameters are overridden by configuration macros. Refer to
    the description of the DRV_I2S_INIT data structure for more details on
    which members on this data structure are overridden.

  Remarks:
    This routine must be called before any other I2S routine is called.

    This routine should only be called once during system initialization
    unless DRV_I2S_Deinitialize is called to de-initialize the driver
    instance. This routine will NEVER block for hardware access.

 */

SYS_MODULE_OBJ DRV_I2S_SSC_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT * const init
)
{
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    DRV_I2S_SSC_INIT *sscInit;
    SYS_MODULE_OBJ returnValue;

    sscInit = (DRV_I2S_SSC_INIT *) init;
    drvObj = (DRV_I2S_SSC_OBJ *)&gDrvI2SSSCObj[drvIndex];

    /* Disable the I2S module */ 
    ssc_registers_t volatile * ssc = ((ssc_registers_t *)(sscInit->sscID));
    ssc->SSC_CR.w = (SSC_CR_TXDIS_Msk | SSC_CR_RXDIS_Msk);
    
    /* Setup the Hardware */
    _DRV_I2S_SSC_HardwareSetup(drvObj, sscInit);
    
    drvObj->audioCommWidth = sscInit->txDataLen;
    drvObj->isInInterruptContext = false;
    drvObj->queueSizeTransmit = sscInit->queueSizeTransmit;
    drvObj->queueSizeReceive = sscInit->queueSizeReceive;
    drvObj->queueCurrentTransmitSize = 0;
    drvObj->queueCurrentReceiveSize = 0;
    drvObj->queueHead = (DRV_I2S_BUFFER_OBJECT *) NULL;
    drvObj->readQueueHead = (DRV_I2S_BUFFER_OBJECT *) NULL;
    drvObj->writeQueueHead = (DRV_I2S_BUFFER_OBJECT *) NULL;
    drvObj->task = DRV_I2S_TASK_PROCESS_QUEUE;
    drvObj->numClients = 0;
    drvObj->status = SYS_STATUS_READY;
    drvObj->moduleID = (ssc_registers_t *) sscInit->sscID;

    /* Clear the interrupts */
    _DRV_I2S_InterruptSourceClear(drvObj->interruptSSC);
    _DRV_I2S_InterruptSourceClear(drvObj->interruptDMA);

    /* Create the hardware instance mutex. */
    if(OSAL_MUTEX_Create(&(drvObj->mutexDriverInstance)) != OSAL_RESULT_TRUE)
    {
       SYS_DEBUG(SYS_ERROR_INFO, "\r\nI2S SSC: Error while creating mutex");
       returnValue = SYS_MODULE_OBJ_INVALID;
       return returnValue;
    }
    
    /* Check if the global mutexes have been created.If not then create these.*/
    if(!gDrvI2SCommonDataObj.membersAreInitialized)
    {
        /* This means that mutexes where not created. Create them. */
        if((OSAL_MUTEX_Create(&(gDrvI2SCommonDataObj.mutexClientObjects)) != OSAL_RESULT_TRUE))
        {
           returnValue = SYS_MODULE_OBJ_INVALID;
           return returnValue;
        }
        /* Set this flag so that global mutexes get allocated only once */
        gDrvI2SCommonDataObj.membersAreInitialized = true;
    }

    /* Enable the I2S module */
    ssc->SSC_CR.w = (SSC_CR_TXEN_Msk | SSC_CR_RXEN_Msk);
    
    returnValue = ((SYS_MODULE_OBJ) drvObj);
    
    /* Return the object structure */
    return returnValue;

}

// *****************************************************************************
/* Function:
    void DRV_I2S_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the I2S driver module

  Description:
    Deinitializes the specified instance of the I2S driver module, disabling
    its operation (and any hardware).  Invalidates all the internal data.

  Remarks:
    Once the Initialize operation has been called, the De-initialize operation
    must be called before the Initialize operation can be called again. This
    routine will NEVER block waiting for hardware.
*/

void DRV_I2S_SSC_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    DRV_I2S_BUFFER_OBJECT * iterator, *itRead, *itWrite;

    if (object == SYS_MODULE_OBJ_INVALID || object == (SYS_MODULE_OBJ)NULL)
    {
        /* Invalid object */
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid object \r\n");
        return;
    }

    drvObj = (DRV_I2S_SSC_OBJ *) object;
    ssc_registers_t volatile * ssc = ((ssc_registers_t *)(drvObj->sscID));
    
    if (false == drvObj->inUse)
    {
        /* Cannot de-initialize an object that is not already in use. */
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Instance not in use \r\n");
        return;
    }
    
    /* Disable the interrupt */
    _DRV_I2S_InterruptSourceDisable(drvObj->interruptSSC);
    _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);

    /* Deallocate the allocated channel handles  */
    if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleWrite)
    {
        SYS_DMA_ChannelRelease(drvObj->dmaChannelHandleWrite);
    }
    if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleRead)
    {
        SYS_DMA_ChannelRelease(drvObj->dmaChannelHandleRead);
    }
    
    /* Turn off I2S module */
    ssc->SSC_CR.w = (SSC_CR_TXDIS_Msk | SSC_CR_RXDIS_Msk);

	/* Deallocate all the mutexes */
     if((OSAL_MUTEX_Delete(&(drvObj->mutexDriverInstance)) != OSAL_RESULT_TRUE))
     {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to delete client handle mutex \r\n");
        return;
     }

    /* Remove all objects from the queue */
    iterator = drvObj->queueHead;
    itRead = drvObj->readQueueHead;
    itWrite = drvObj->writeQueueHead;
    while(iterator != NULL)
    {
        /* Return the buffer object to the pool */
        iterator->inUse = false;
        iterator = iterator->next;
    }
    while(itRead != NULL)
    {
        /* Return the buffer object to the pool */
        itRead->inUse = false;
        itRead = itRead->next;
    }
    while(itWrite != NULL)
    {
        /* Return the buffer object to the pool */
        itWrite->inUse = false;
        itWrite = itWrite->next;
    }
    
    /* Indicate that this object is not is use */
    drvObj->inUse = false;
    
    /* Deallocate the buffer queue Object */
    drvObj->queueHead = (DRV_I2S_BUFFER_OBJECT *) NULL;
    drvObj->readQueueHead = (DRV_I2S_BUFFER_OBJECT *) NULL;
    drvObj->writeQueueHead = (DRV_I2S_BUFFER_OBJECT *) NULL;
    
    /* Set number of clients to zero */
    drvObj->numClients = 0;
    drvObj->status = SYS_STATUS_UNINITIALIZED;
    
    return;
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_I2S_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the I2S driver module.

  Description:
    This routine provides the current status of the I2S driver module.

  Remarks:
    A driver can opened only when its status is SYS_STATUS_READY.
*/

SYS_STATUS DRV_I2S_SSC_Status
(
    SYS_MODULE_OBJ object
)
{
    DRV_I2S_SSC_OBJ *drvObj = NULL;

    if (object == SYS_MODULE_OBJ_INVALID || object < DRV_I2S_SSC_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: System Module Object is invalid \r\n");
        return SYS_STATUS_ERROR;
    }
	drvObj = (DRV_I2S_SSC_OBJ *) object;

    /* Return the status of the driver object */
    return drvObj->status;
}

// *****************************************************************************
/* Function:
    void DRV_I2S_Tasks(SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's receive state machine and implements its ISR,
    This function only works for WRITEREAD mode.

  Description:
    This routine is used to maintain the driver's internal receive state machine
    and implement its transmit and receive ISR for interrupt-driven 
    implementations.
    In polling mode, this function should be called from SYS_Tasks() function.
    In interrupt mode, this function should be called from the interrupt
    service routine of the I2S that is associated with this I2S driver
    hardware instance.
    In DMA mode of operation, this function should be called from the interrupt
    service routine of the channel associated with the transmission/reception
    of the I2s driver hardware instance.

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This routine may execute in an ISR context & will never block or access any
    resources that may cause it to block.
*/

void DRV_I2S_SSC_Tasks
(
    SYS_MODULE_OBJ object
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    DRV_I2S_BUFFER_OBJECT *lQueueObj;
    bool txChannelInterruptWasEnabled = false;
    bool rxChannelInterruptWasEnabled = false;

    drvObj = (DRV_I2S_SSC_OBJ *)object;
    ssc_registers_t volatile * ssc = ((ssc_registers_t *)(drvObj->sscID));
    
    if((false == drvObj->inUse) || (drvObj->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't do anything */
        return;
    }

    _DRV_I2S_isInInterruptContextSet(drvObj->isInInterruptContext);
    
    /* Setup the state of the I2S task based on the contents at the head 
     * of queue and client IO intent */

    txChannelInterruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
    rxChannelInterruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
    
    lQueueObj = drvObj->queueHead;
    if(lQueueObj == (DRV_I2S_BUFFER_OBJECT*) NULL)
    {        
        return;
    }        
    clientObj = lQueueObj->clientObject;
    if ((lQueueObj != (DRV_I2S_BUFFER_OBJECT*) NULL))
    {
       if(DRV_I2S_BUFFER_OPERATION_TYPE_WRITEREAD == lQueueObj->bType)
        {
            drvObj->task = DRV_I2S_TASK_PROCESS_WRITE_READ;
        }
    }

    if (DRV_I2S_TASK_PROCESS_WRITE_READ == drvObj->task)
    {
        /* DMA mode of operation */
        if( SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleWrite &&
            SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleRead)
        {
            drvObj->task = DRV_I2S_TASK_PROCESS_QUEUE;

            /*Callback with I2S Buffer Complete Event*/
            clientObj->pEventCallBack(clientObj->bufferEvent,
                    (DRV_I2S_BUFFER_HANDLE)lQueueObj->indexHandle,
                    clientObj->hClientArg);
            
            lQueueObj->inUse = false;
            
            /* Update the driver object's head pointer */
            if(drvObj->queueHead != (DRV_I2S_BUFFER_OBJECT*) NULL)
            {
                drvObj->queueHead = drvObj->queueHead->next;
                
                /* Release the location of a queue size */
                drvObj->queueSizeTransmit++;
                drvObj->queueCurrentTransmitSize--;
            }
            
            if(drvObj->queueHead==NULL)
            {
               /* We don't have any buffers to process. We can disable 
                * the interrupt. */
                _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
                _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
            }
        }
    }
    else
    {
        /* Impossible case */
        ;
    }

    if(drvObj->queueHead!=NULL)
    {
        lQueueObj = drvObj->queueHead;

        if(DRV_I2S_BUFFER_OPERATION_TYPE_WRITEREAD == lQueueObj->bType)
        {
            /* Adding Read Request */
            SYS_DMA_ChannelTransferAdd(drvObj->dmaChannelHandleRead,
                            (const void *) &drvObj->moduleID->SSC_RHR, 
                            drvObj->srcDestSize, lQueueObj->rxbuffer,
                            lQueueObj->size,drvObj->cellSize);
            
            /* Adding Write Request */
            SYS_DMA_ChannelTransferAdd(drvObj->dmaChannelHandleWrite,
                                lQueueObj->txbuffer, lQueueObj->size,
                                (const void *) &drvObj->moduleID->SSC_THR, 
                                drvObj->srcDestSize,drvObj->cellSize);
        }
        else
        {
            /* Impossible case */
            ;
        }
        if(txChannelInterruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
        if(rxChannelInterruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
    }
    else
    {
        /* Nothing to do */
    }
   _DRV_I2S_isInInterruptContextClear(drvObj->isInInterruptContext);
   
   return;
}

// *****************************************************************************
/* Function:
    void DRV_I2S_ReadTasks(SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's receive state machine and implements its ISR,
    This function only works for READ mode.

  Description:
    This routine is used to maintain the driver's internal receive state machine
    and implement its transmit and receive ISR for interrupt-driven 
    implementations.
    In polling mode, this function should be called from SYS_Tasks() function.
    In interrupt mode, this function should be called from the interrupt
    service routine of the I2S that is associated with this I2S driver
    hardware instance.
    In DMA mode of operation, this function should be called from the interrupt
    service routine of the channel associated with the transmission/reception
    of the I2s driver hardware instance.

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This routine may execute in an ISR context & will never block or access any
    resources that may cause it to block.
*/

void DRV_I2S_SSC_ReadTasks
(
    SYS_MODULE_OBJ object
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    DRV_I2S_BUFFER_OBJECT *lQueueObj;
    bool rxChannelInterruptWasEnabled = false;
    
    drvObj = (DRV_I2S_SSC_OBJ *)object;
    ssc_registers_t volatile * ssc = ((ssc_registers_t *) (drvObj->sscID));
    
    if((false == drvObj->inUse) || (drvObj->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't do anything */
        return;
    }

    _DRV_I2S_isInInterruptContextSet(drvObj->isInInterruptContext);
    
    /* Setup the state of the I2S task based on the contents at the
     * head of queue and client IO intent */

    rxChannelInterruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
    lQueueObj = drvObj->readQueueHead;
    if(lQueueObj == (DRV_I2S_BUFFER_OBJECT*) NULL)
    {
        return;
    }    
    clientObj = lQueueObj->clientObject;
    
    /* DMA mode of operation */
    if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleRead)
    {
        drvObj->task = DRV_I2S_TASK_PROCESS_QUEUE;

        /* Callback with I2S Buffer Complete Event */
        clientObj->pEventCallBack(clientObj->bufferEvent,
                (DRV_I2S_BUFFER_HANDLE)lQueueObj->indexHandle,
                clientObj->hClientArg);

        lQueueObj->inUse = false;
        if(drvObj->readQueueHead != (DRV_I2S_BUFFER_OBJECT*) NULL)
        {
            /* Update the driver object's head pointer */
            drvObj->readQueueHead = drvObj->readQueueHead->next;
            /* Release the location of a queue size */
            drvObj->queueSizeReceive++;
            drvObj->queueCurrentReceiveSize--;
        }

        if(drvObj->readQueueHead==NULL)
        {
           /* We don't have any buffers to process.We can disable interrupt */
            _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
        }
    }

    if(drvObj->readQueueHead!=NULL)
    {
        lQueueObj = drvObj->readQueueHead;
        if(DRV_I2S_BUFFER_OPERATION_TYPE_READ == lQueueObj->bType)
        {
            SYS_DMA_ChannelTransferAdd(drvObj->dmaChannelHandleRead,
                        (const void *) &drvObj->moduleID->SSC_RHR, 
                        drvObj->srcDestSize, lQueueObj->rxbuffer,
                        lQueueObj->size,drvObj->cellSize);
        }
        
        if(rxChannelInterruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
    }
    else
    {
        /* Nothing to do */
    }
   _DRV_I2S_isInInterruptContextClear(drvObj->isInInterruptContext);
   
   return;
}

// *****************************************************************************
/* Function:
    void DRV_I2S_WriteTasks(SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's receive state machine and implements its ISR,
    This function only works for WRITE mode.

  Description:
    This routine is used to maintain the driver's internal receive state machine
    and implement its transmit and receive ISR for interrupt-driven 
    implementations.
    In polling mode,this function should be called from SYS_Tasks() function.
    In interrupt mode, this function should be called from the interrupt
    service routine of the I2S that is associated with this I2S driver
    hardware instance.
    In DMA mode of operation, this function should be called from the interrupt
    service routine of the channel associated with the transmission/reception
    of the I2s driver hardware instance.

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This routine may execute in an ISR context & will never block or access any
    resources that may cause it to block.
*/

 void DRV_I2S_SSC_WriteTasks
 (
    SYS_MODULE_OBJ object
 )
 {
        
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    DRV_I2S_BUFFER_OBJECT *lQueueObj;
    bool txChannelInterruptWasEnabled = false;
    
    drvObj = (DRV_I2S_SSC_OBJ *)object;
    ssc_registers_t * ssc = ((ssc_registers_t *)(drvObj->sscID));
    
    if((false == drvObj->inUse) || (drvObj->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't do anything */
        return;
    }

    _DRV_I2S_isInInterruptContextSet(drvObj->isInInterruptContext);
    
    /* Setup the state of the I2S task based on the contents at the
    head of queue and client IO intent */

    txChannelInterruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
    
    lQueueObj = drvObj->writeQueueHead;    
    if(lQueueObj == (DRV_I2S_BUFFER_OBJECT*) NULL){
        
        return;
    }
    clientObj = lQueueObj->clientObject;
    
    drvObj->task = DRV_I2S_TASK_PROCESS_WRITE_ONLY;
    
    /* DMA mode of operation */
    if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleWrite)
    {
        drvObj->task = DRV_I2S_TASK_PROCESS_QUEUE;

        /*Callback with I2S Buffer Complete Event*/
        clientObj->pEventCallBack(clientObj->bufferEvent,
                (DRV_I2S_BUFFER_HANDLE)lQueueObj->indexHandle,
                clientObj->hClientArg);

        lQueueObj->inUse = false;
        if(drvObj->writeQueueHead != (DRV_I2S_BUFFER_OBJECT*) NULL)
        {
            /* Update the driver object's head pointer */
            drvObj->writeQueueHead = drvObj->writeQueueHead->next;
            /* Release the location of a queue size */
            drvObj->queueSizeTransmit++;
            drvObj->queueCurrentTransmitSize--;
        }
            
        if(drvObj->writeQueueHead==NULL)
        {
            /* We don't have any buffers to process.We can disable interrupt. */
             _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
        }
    }
    
    if(drvObj->writeQueueHead!=NULL)
    {
        lQueueObj = drvObj->writeQueueHead;
        {
            SYS_DMA_ChannelTransferAdd(drvObj->dmaChannelHandleWrite,
                        lQueueObj->txbuffer, lQueueObj->size,
                        (const void *) &drvObj->moduleID->SSC_THR,
                        drvObj->srcDestSize,drvObj->cellSize);
        }
            
        if(txChannelInterruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
    }
    else
    {
    }
   _DRV_I2S_isInInterruptContextClear(drvObj->isInInterruptContext);
   
   return;
}
 
// *****************************************************************************
/* Function:
    void DRV_I2S_TasksError (SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's error state machine and implements its ISR

  Description:
    This routine is used to maintain the driver's internal error state machine
    and implement its error ISR for interrupt-driven implementations.  In
    polling mode, this function should be called from the SYS_Tasks() function.
    In interrupt mode, this function should be called in the error interrupt
    service routine of the I2S that is associated with this I2S driver
    hardware instance.
    In DMA mode of operation, this function should be called from the interrupt
    service routine of the channel associated with the transmission/reception
    of the I2s driver hardware instance.

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This routine may execute in an ISR context & will never block or access any
    resources that may cause it to block.
*/
 
void DRV_I2S_SSC_TasksError
(
    SYS_MODULE_OBJ object
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    DRV_I2S_BUFFER_OBJECT *lQueueObj;
    bool errorOccured;

    drvObj = (DRV_I2S_SSC_OBJ *)object;
    if((false == drvObj->inUse) || (drvObj->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't do anything */
        return;
    }

    errorOccured = false;
    lQueueObj = drvObj->queueHead;
    /* Check for a valid pointer */
    if (lQueueObj != (DRV_I2S_BUFFER_OBJECT *)NULL )
    {
        clientObj = lQueueObj->clientObject;
        
        /* DMA Mode: See if there is a DMA error */
        if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleWrite ||
            SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleRead )
        {
            if(DRV_I2S_DMA_TRANSFER_ERROR == lQueueObj->nPendingBytes)
            {
                if(DRV_I2S_BUFFER_EVENT_ERROR == clientObj->bufferEvent)
                {
                    /*Callback with I2S Buffer Complete Event*/
                    clientObj->errorInfo = DRV_I2S_ERROR_ADDRESS;
                    errorOccured = true;
                }
            }
        }
        if(true == errorOccured)
        {
            lQueueObj->inUse = false;
            
            /* Update the driver object's head pointer */
            if(DRV_I2S_TASK_PROCESS_WRITE_ONLY == drvObj->task)
            {
                drvObj->queueSizeTransmit++;
                drvObj->queueCurrentTransmitSize--;
            }
            else if(DRV_I2S_TASK_PROCESS_READ_ONLY == drvObj->task)
            {
                drvObj->queueSizeReceive++;
                drvObj->queueCurrentReceiveSize--;
            }
            else
            {
                /* Impossible case */
                ;
            }
            drvObj->task = DRV_I2S_TASK_PROCESS_QUEUE;
            drvObj->queueHead = drvObj->queueHead->next;
            
            /*Callback with I2S Buffer Error Event*/
            clientObj->pEventCallBack(DRV_I2S_BUFFER_EVENT_ERROR,
                (DRV_I2S_BUFFER_HANDLE)lQueueObj->indexHandle, clientObj->hClientArg);
        }
    }
    
    return;
}

// *****************************************************************************
// *****************************************************************************
// Section: I2S Driver Client Routines Implementation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_I2S_Open( const SYS_MODULE_INDEX drvIndex,
                             const DRV_IO_INTENT    ioIntent )

  Summary:
    Opens the specified I2S driver instance and returns a handle to it

  Description:
    This routine opens the specified I2S driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The DRV_IO_INTENT_BLOCKING and DRV_IO_INTENT_NONBLOCKING ioIntent
    options additionally affect the behavior of the DRV_I2S_Read() and
    DRV_I2S_Write() functions. If the ioIntent is DRV_IO_INTENT_NONBLOCKING,
    then these function will not block even if the required amount of
    data could not be processed. If the ioIntent is DRV_IO_INTENT_BLOCKING,
    these functions will block till the required amount of data is processed.

    If ioIntent is DRV_IO_INTENT_READ, the client will only be read from
    the driver. If ioIntent is DRV_IO_INTENT_WRITE, the client will only be
    able to write to the driver.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

   Remarks:
    The handle returned is valid until the DRV_I2S_Close routine is called.
    This routine will NEVER block waiting for hardware.If the requested intent
    flags are not supported, the routine will return DRV_HANDLE_INVALID.  This
    function is thread safe in a RTOS application. It should not be called in an
    ISR.
*/

DRV_HANDLE DRV_I2S_SSC_Open
(
    const SYS_MODULE_INDEX iDriver,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_I2S_CLIENT_OBJ *hClient;
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    uint32_t iClient;
    
    DRV_HANDLE returnValue = DRV_HANDLE_INVALID;

    drvObj = (DRV_I2S_SSC_OBJ *)&gDrvI2SSSCObj[iDriver];
    
    if ((drvObj->numClients > 0) && (true == drvObj->isExclusive))
    {
        /* Driver already opened in exclusive mode. Cannot open a new client. */
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Cannot open a new client in exclusive mode \r\n");
        returnValue = DRV_HANDLE_INVALID;
    }

    if ((drvObj->numClients > 0) &&
        (DRV_IO_INTENT_EXCLUSIVE == (ioIntent & DRV_IO_INTENT_EXCLUSIVE)))
    {
        /*  A client Instance of driver is open. Cannot open the new client in 
         *  exclusive mode */
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Cannot open a new client in exclusive mode \r\n");
        returnValue = DRV_HANDLE_INVALID;
    }

    iClient = 0;
    hClient = (DRV_I2S_CLIENT_OBJ *)&gDrvI2SClientObj[iClient];

    /* Grab client object mutex here */
    if(OSAL_MUTEX_Lock(&(gDrvI2SCommonDataObj.mutexClientObjects), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Setup client operations */
        /* Find available slot in array of client objects */
        for (iClient; iClient < DRV_I2S_CLIENTS_NUMBER; iClient++)
        {
            if (false == hClient->inUse)
            {
                /* Set the exclusive mode for the driver instance */
                if (DRV_IO_INTENT_EXCLUSIVE == (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
                {
                    drvObj->isExclusive = true;
                }
                if (DRV_IO_INTENT_READWRITE == (ioIntent & DRV_IO_INTENT_READWRITE))
                {
                    hClient->ioIntent = DRV_IO_INTENT_READWRITE;
                }
                else if (DRV_IO_INTENT_READ == (ioIntent & DRV_IO_INTENT_READ))
                {
                    hClient->ioIntent = DRV_IO_INTENT_READ;
                }
                else if (DRV_IO_INTENT_WRITE == (ioIntent & DRV_IO_INTENT_WRITE))
                {
                    hClient->ioIntent = DRV_IO_INTENT_WRITE;
                }
                else
                {
                    /* An operation mode is needed */
                    if((OSAL_MUTEX_Unlock(&(gDrvI2SCommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
                    {
                        SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to unlock open routine mutex \r\n");
                        returnValue = DRV_HANDLE_INVALID;
                    }
                }

                if (DRV_IO_INTENT_NONBLOCKING ==
                        (ioIntent & DRV_IO_INTENT_NONBLOCKING))
                {
                    hClient->ioIntent |= DRV_IO_INTENT_NONBLOCKING;
                }

                /* Remember which I2S driver instance owns me */
                hClient->inUse  = true;
                hClient->hDriver = drvObj;
                hClient->sscID = drvObj->sscID;
                hClient->pEventCallBack = NULL;
                drvObj->numClients++;
                drvObj->status = SYS_STATUS_READY;
                drvObj->inUse = true;
                
                /* We have found a client object, Release the mutex and 
                 * return with the driver handle. An operation mode is needed */
                
                if((OSAL_MUTEX_Unlock(&(gDrvI2SCommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
                {
                    SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to unlock open routine mutex \r\n");
                    returnValue = DRV_HANDLE_INVALID;
                }
                
                /* Return the client object */
                returnValue = (DRV_HANDLE) hClient;
            }
            hClient++;
        }
        /* Could not find a client object. Release the mutex and return 
         * with an invalid handle. */
        
        if((OSAL_MUTEX_Unlock(&(gDrvI2SCommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to unlock open routine mutex \r\n");
        }
    }
    
    return returnValue;
} 

// *****************************************************************************
/* Function:
    void DRV_I2S_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the I2S driver

  Description:
    This routine closes an opened-instance of the I2S driver, invalidating the
    handle. Any buffers in the driver queue that were submitted by this client
    will be removed.  After calling this routine, the handle passed in "handle"
    must not be used with any of the remaining driver routines.  
    A new handle must be obtained by calling DRV_I2S_Open before the caller 
    may use the driver again.

  Remarks:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.  The driver will abort any ongoing operations
    when this routine is called.
*/

void DRV_I2S_SSC_Close
(
    const DRV_HANDLE client
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    bool dmaWriteInterruptWasEnabled;
    bool dmaReadInterruptWasEnabled;
    DRV_I2S_BUFFER_OBJECT *iterator1;

    if(client == DRV_HANDLE_INVALID || (DRV_HANDLE)NULL == client)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid Driver Handle \r\n");
        return;
    }

    clientObj = (DRV_I2S_CLIENT_OBJ *) client;
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid Driver Handle \r\n");
        return;
    }

    drvObj = (DRV_I2S_SSC_OBJ *) clientObj->hDriver;
    
    /* Remove all buffer that this client owns from the driver queue */

    if((OSAL_MUTEX_Lock(&(drvObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE))
    {
        /* Disable the transmit interrupt */
        dmaWriteInterruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
        dmaReadInterruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);

        iterator1 = drvObj->queueHead;
        while(iterator1 != NULL)
        {
            if(clientObj == (DRV_I2S_CLIENT_OBJ *)iterator1->clientObject)
            {
                /* That means this buffer object is owned by this client. 
                 * This buffer object should be removed. */
                iterator1->inUse = false;
                iterator1 = iterator1->next;
                drvObj->queueSizeTransmit++;
                drvObj->queueCurrentTransmitSize--;
            }
        }
        
        iterator1 = drvObj->readQueueHead;
        while(iterator1 != NULL){
            if(clientObj == (DRV_I2S_CLIENT_OBJ *)iterator1->clientObject){
                iterator1->inUse = false;
                iterator1 = iterator1->next;
                drvObj->queueSizeReceive++;
                drvObj->queueCurrentReceiveSize--;
            }
        }
        
        iterator1 = drvObj->writeQueueHead;
        while(iterator1 != NULL){
            if(clientObj == (DRV_I2S_CLIENT_OBJ *)iterator1->clientObject){
                iterator1->inUse = false;
                iterator1 = iterator1->next;
                drvObj->queueSizeTransmit++;
                drvObj->queueCurrentTransmitSize--;
            }
        }

        /* After removing the closed clientobj, If there are no buffers 
         * in the queue, Make the head pointer point to NULL */
        
        if(0 == drvObj->queueCurrentReceiveSize &&
           0 == drvObj->queueCurrentTransmitSize)
        {
            drvObj->queueHead = (DRV_I2S_BUFFER_OBJECT *)0;
            drvObj->readQueueHead = (DRV_I2S_BUFFER_OBJECT *)0;
            drvObj->writeQueueHead = (DRV_I2S_BUFFER_OBJECT *)0;
            
            /* Once the queue becomes empty as a consequence to the calling of 
             * DRV_I2S_Close. Any interrupt which was enabled and pending and 
             * earlier to the call to 'DRV_I2S_Close' needs to be cleared as 
             * it might lead to a false handling of interrupt */
            
            if(dmaWriteInterruptWasEnabled && _DRV_I2S_InterruptSourceStatusGet(drvObj->interruptDMA))
            {            
                _DRV_I2S_InterruptSourceClear(drvObj->interruptDMA);                                
            }
            if(dmaReadInterruptWasEnabled && _DRV_I2S_InterruptSourceStatusGet(drvObj->interruptDMA))
            {
                _DRV_I2S_InterruptSourceClear(drvObj->interruptDMA);                
            }          
        }  
        else
        {
            /* Iterate to update the head pointer to point the first valid \
             * buffer object in the queue */
            
            iterator1 = drvObj->queueHead;
            while(iterator1 != (DRV_I2S_BUFFER_OBJECT *)0)
            {
                if(iterator1->inUse == true)
                {
                    drvObj->queueHead = iterator1;
                    break;
                }
                iterator1 = iterator1->next;
            }
            
            iterator1 = drvObj->readQueueHead;
            while(iterator1 != (DRV_I2S_BUFFER_OBJECT *)0)
            {
                if(iterator1->inUse == true)
                {
                    drvObj->readQueueHead = iterator1;
                    break;
                }
                iterator1 = iterator1->next;
            }
            
            iterator1 = drvObj->writeQueueHead;
            while(iterator1 != (DRV_I2S_BUFFER_OBJECT *)0)
            {
                if(iterator1->inUse == true)
                {
                    drvObj->writeQueueHead = iterator1;
                    break;
                }
                iterator1 = iterator1->next;
            }
            
        }

        /* Done with closing client now, Re-enable the interrupt if 
         * it was disabled */
        
        if(dmaWriteInterruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
        if(dmaReadInterruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
        
        /* Release the mutex */
        if((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to unlock write queue mutex in close routine \r\n");
        }
    }
    else
    {
        /* The case where the mutex lock timed out and the client buffer objects
         * could not be removed from the driver queue, 
         * the close function should fail. */
        
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Could not remove client buffer objects \r\n");
        
        return;
    }

    /* De-allocate the object */
    clientObj->inUse = false;
    
    /* Reduce the number of clients */
    drvObj->numClients--;
    
    return;

}

// *****************************************************************************
/*
Function:
    void DRV_I2S_BufferAddWrite( const DRV_HANDLE handle,
                                 DRV_I2S_BUFFER_HANDLE *bufferHandle,
                                 void * buffer, size_t size);

  Summary:
    Schedule a non-blocking driver write operation.

  Description:
    This function schedules a non-blocking write operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the write request
    was scheduled successfully. The function adds the request to the hardware
    instance transmit queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_I2S_BUFFER_HANDLE_INVALID
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only
    - if the buffer size is 0.
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_I2S_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_I2S_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the I2S Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    I2S driver instance. It should not otherwise be called directly in an ISR.

*/

void DRV_I2S_SSC_BufferAddWrite
(
    const DRV_HANDLE handle,
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *buffer, 
    size_t size
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_BUFFER_OBJECT *i2sBufObj;
    DRV_I2S_BUFFER_OBJECT_INDEX i2sBufIndex;
    DRV_I2S_SSC_OBJ *drvObj = NULL;
    bool interruptWasEnabled;
    uint8_t dmaDataWidth = 0;
    
    /* The Client and driver objects from the handle */
    interruptWasEnabled = false;
    clientObj = (DRV_I2S_CLIENT_OBJ *) handle;
    drvObj = (DRV_I2S_SSC_OBJ *) clientObj->hDriver;

    ssc_registers_t volatile * ssc = ((ssc_registers_t *)(drvObj->sscID));
    
    /* check if the transmit queue size is 0 */
    if (0 == drvObj->queueSizeTransmit)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Cannot add a write operation as the queueSizeTransmit is 0 \r\n");
        return;
    }        /* Bound checking for the buffer parameters */
    else if ((NULL == buffer) || (0 == size) || (bufferHandle == NULL))
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid parameters \r\n");
        return;
    }
    /* See if the handle is still valid */
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid Driver Handle \r\n");
        return;
    }
    /* If the driver was opened in read only mode */
    else if (DRV_IO_INTENT_READ == clientObj->ioIntent)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Driver was opened in Read mode \r\n");
        return;
    }
    else
    {
        ;
    }
    
    if (drvObj->audioCommWidth < 8)
    {
        dmaDataWidth = SYS_DMA_DATA_WIDTH_BYTE;
    }
    else if (drvObj->audioCommWidth < 16)
    {
         dmaDataWidth = SYS_DMA_DATA_WIDTH_HALF_WORD;
    }
    else if(drvObj->audioCommWidth < 32)
    {
         dmaDataWidth = SYS_DMA_DATA_WIDTH_WORD;
    }
    
    /* We will allow buffers to be added in the interrupt context of this 
     * I2S driver. But we must make sure that if we are in interrupt, then we 
     * should not modify mutexes. */
    
    if(!drvObj->isInInterruptContext)
    {
        /* Grab a mutex. This is okay because we are not in an interrupt 
         * context */
        if(OSAL_MUTEX_Lock(&(drvObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            /* We will disable interrupts so that the queue status does not 
             * get updated asynchronously. This code will always execute. */
            interruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
        }
        else
        {
            /* The mutex acquisition timed out. Return with an invalid handle. 
             * This code will not execute if there is no RTOS. */
            return;
        }
    }
    
    i2sBufIndex = _DRV_I2S_SSC_WriteQueueObjectIndexGet();
    if (i2sBufIndex != DRV_I2S_BUFFER_OBJECT_INDEX_INVALID)
    {
        i2sBufObj = &gDrvI2SWriteBufferQObj[i2sBufIndex];
        i2sBufObj->indexHandle = ((++gDrvI2SUniqueBufferHandle << 0x10) | i2sBufIndex);
        *bufferHandle = (DRV_I2S_BUFFER_HANDLE) i2sBufObj->indexHandle;
        i2sBufObj->txbuffer = (uint8_t *)buffer;
        i2sBufObj->rxbuffer = NULL;
        i2sBufObj->size = size;
        i2sBufObj->next = (DRV_I2S_BUFFER_OBJECT *) NULL;
        i2sBufObj->nPendingBytes = size;
        i2sBufObj->clientObject = clientObj;
        i2sBufObj->bType = DRV_I2S_BUFFER_OPERATION_TYPE_WRITE;

        /* If the queue is empty */
        if (drvObj->writeQueueHead == ((DRV_I2S_BUFFER_OBJECT *) NULL))
        {
            drvObj->writeQueueHead = i2sBufObj;
            
            /* Since this is the first buffer in the queue Add it immediately
             * to DMA for processing */
            SYS_DMA_ChannelTransferAdd(drvObj->dmaChannelHandleWrite,
                                i2sBufObj->txbuffer, i2sBufObj->size,
                                (const void *) &drvObj->moduleID->SSC_THR,
                                dmaDataWidth,drvObj->cellSize);
        }
        else
        {
            DRV_I2S_BUFFER_OBJECT *iterator;
            iterator = drvObj->writeQueueHead;
            
            /* Insert the object at the end of the queue */
            while (iterator->next != NULL)
            {
                iterator = iterator->next;
            }
            iterator->next = i2sBufObj;
            
            /* DMA Mode:There is already a buffer under processing in the queue.
             * This buffer will be added to DMA for processing immediately
             * after processing of the buffer prior to this buffer completes.
             * (This functionality is implemented in DRV_I2S_Tasks) */
        }
        
        /* A location of the queue size is being used */
        drvObj->queueSizeTransmit--;
        drvObj->queueCurrentTransmitSize++;
        
        /* We are done. Restore the interrupt enable status and return. */
        _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);

        /* Release mutex */
        if((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to DriverInstance mutex \r\n");
        }
        return;
    }
    else
    {
        /* This means we could not find a buffer. This will happen if 
         * the DRV_USART_QUEUE_DEPTH_COMBINED parameter is 
         * configured to be less */
        
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Insufficient Combined Queue Depth \r\n");
        
        /* Enable the interrupt if it was disabled */
        if(interruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
        
        /* Release mutex */
        if((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to uDriverInstance mutex \r\n");
        }
        return;
    }
}

// *****************************************************************************
/* Function:
	void DRV_I2S_BufferAddWriteRead(const DRV_HANDLE handle,
                                    DRV_I2S_BUFFER_HANDLE	*bufferHandle,
                                    void *transmitBuffer, void *receiveBuffer,
                                    size_t size)

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
    This function is thread safe in a RTOS application. It can be called from
    within the I2S Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    I2S driver instance. It should not otherwise be called directly in an ISR.

    This function is useful when there is valid read expected for every
    I2S write. The transmit and receive size must be same.

*/

void DRV_I2S_SSC_BufferAddWriteRead
(
    const DRV_HANDLE handle,
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *transmitBuffer, 
    void *receiveBuffer,
    size_t size
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_BUFFER_OBJECT *i2sBufObj;
    DRV_I2S_BUFFER_OBJECT_INDEX i2sBufIndex;
    DRV_I2S_SSC_OBJ *drvObj;
    bool interruptWasEnabled;
    uint8_t dmaDataWidth = 0;
    
    /* The Client and driver objects from the handle */
    interruptWasEnabled = false;
    clientObj = (DRV_I2S_CLIENT_OBJ *) handle;
    drvObj = (DRV_I2S_SSC_OBJ *) clientObj->hDriver;

    ssc_registers_t volatile * ssc = ((ssc_registers_t *)(drvObj->sscID));
    
    /* check if the transmit queue size is 0 */
    if (0 == drvObj->queueSizeTransmit)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Cannot add a write operation as the queueSizeTransmit is 0 \r\n");
        return;
    }        /* Bound checking for the buffer parameters */
    else if ((NULL == transmitBuffer) || (NULL == receiveBuffer) || (0 == size) || (bufferHandle == NULL))
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid parameters \r\n");
        return;
    }
    /* See if the handle is still valid */
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid Driver Handle \r\n");
        return;
    }
    /* If the driver was opened in read only or write only mode */
    else if (DRV_IO_INTENT_READ == (DRV_IO_INTENT_READWRITE & clientObj->ioIntent) ||
             DRV_IO_INTENT_WRITE == (DRV_IO_INTENT_READWRITE & clientObj->ioIntent))
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Driver was opened in Read/write only mode \r\n");
        return;
    }
    else
    {
        ;
    }
    
    if (drvObj->audioCommWidth < 8)
    {
        dmaDataWidth = SYS_DMA_DATA_WIDTH_BYTE;
    }
    else if (drvObj->audioCommWidth < 16)
    {
         dmaDataWidth = SYS_DMA_DATA_WIDTH_HALF_WORD;
    }
    else if(drvObj->audioCommWidth < 32)
    {
         dmaDataWidth = SYS_DMA_DATA_WIDTH_WORD;
    }

    /* We will allow buffers to be added in the interrupt context of this 
     * I2S driver. But we must make sure that if we are in interrupt, then we 
     * should not modify mutexes. */
    
    if(!drvObj->isInInterruptContext)
    {
        /* Grab a mutex. This is okay because we are not in an interrupt 
         * context */
        if(OSAL_MUTEX_Lock(&(drvObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            /* We will disable interrupts so that the queue status does not 
             * get updated asynchronously. This code will always execute. */
            interruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
        }
        else
        {
            /* The mutex acquisition timed out. Return with an invalid handle. 
             * This code will not execute if there is no RTOS. */
            return;
        }
    }
    
    i2sBufIndex = _DRV_I2S_SSC_QueueObjectIndexGet();
    if (i2sBufIndex != DRV_I2S_BUFFER_OBJECT_INDEX_INVALID)
    {
        i2sBufObj = &gDrvI2SBufferQObj[i2sBufIndex];
        i2sBufObj->indexHandle = ((++gDrvI2SUniqueBufferHandle << 0x10) | i2sBufIndex);
        *bufferHandle = (DRV_I2S_BUFFER_HANDLE) i2sBufObj->indexHandle;
        i2sBufObj->txbuffer = (uint8_t *)transmitBuffer;
        i2sBufObj->rxbuffer = (uint8_t *)receiveBuffer;
        i2sBufObj->size = size;
        i2sBufObj->next = (DRV_I2S_BUFFER_OBJECT *) NULL;
        i2sBufObj->nPendingBytes = size;
        i2sBufObj->clientObject = clientObj;
        i2sBufObj->bType = DRV_I2S_BUFFER_OPERATION_TYPE_WRITEREAD;

        /* If the queue is empty */
        if (drvObj->queueHead == ((DRV_I2S_BUFFER_OBJECT *) NULL))
        { 
            drvObj->queueHead = i2sBufObj;            
            /* Adding Read Request */
            SYS_DMA_ChannelTransferAdd(drvObj->dmaChannelHandleRead,
                            (const void *) &drvObj->moduleID->SSC_RHR,
                            i2sBufObj->size, i2sBufObj->rxbuffer,
                            dmaDataWidth,drvObj->cellSize);
            /* Since this is the first buffer in the queue Add it immediately
             * to DMA for processing */
            
            /* Adding Write Request */
            SYS_DMA_ChannelTransferAdd(drvObj->dmaChannelHandleWrite,
                                i2sBufObj->txbuffer, i2sBufObj->size,
                                (const void *) &drvObj->moduleID->SSC_THR,
                                dmaDataWidth,drvObj->cellSize);
        }
        else
        {
            DRV_I2S_BUFFER_OBJECT *iterator;
            iterator = drvObj->queueHead;
            
            if (iterator != NULL)
            {
                /* Insert the object at the end of the queue */
                while (iterator->next != NULL)
                {
                    iterator = iterator->next;
                }
                iterator->next = i2sBufObj;
                /* DMA Mode: There is already a buffer under processing 
                 * in the queue. This buffer will be added to DMA for 
                 * processing immediately after the processing of the buffer 
                 * prior to this buffer completes.
                 * (This functionality is implemented in DRV_I2S_SSC_Tasks) */
            }
        }
        
        /* A location of the queue size is being used */
        drvObj->queueSizeTransmit--;
        drvObj->queueCurrentTransmitSize++;
        
        /* We are done. Restore the interrupt enable status and return. */
        _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);

        /* Release mutex */
        if((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to DriverInstance mutex \r\n");
        }
        return;
    }
    else
    {
        /* This means we could not find a buffer. This
         * will happen if the the DRV_USART_QUEUE_DEPTH_COMBINED
         * parameter is configured to be less */
        
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Insufficient Combined Queue Depth \r\n");
        
        /* Enable the interrupt if it was disabled */
        if(interruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
        /* Release mutex */
        if((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to uDriverInstance mutex \r\n");
        }
        return;
    }
}

// *****************************************************************************
/*
Function:
    void DRV_I2S_BufferAddRead( const DRV_HANDLE handle,
                                DRV_I2S_BUFFER_HANDLE *bufferHandle,
                                void * buffer, size_t size)

  Summary:
    Schedule a non-blocking driver read operation.

  Description:
    This function schedules a non-blocking read operation. The function returns
    with a valid buffer handle  in the bufferHandle argument if the read request
    was scheduled successfully. The function adds the request to the hardware
    instance receive queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_I2S_BUFFER_HANDLE_INVALID
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for write only
    - if the buffer size is 0.
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_I2S_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_I2S_BUFFER_EVENT_ERROR event if the buffer
    was not processed successfully.

   Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the I2S Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    I2S driver instance. It should not otherwise be called directly in an ISR.

*/

void DRV_I2S_SSC_BufferAddRead
(
    const DRV_HANDLE handle,
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *buffer, 
    size_t size
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_BUFFER_OBJECT *i2sBufObj;
    DRV_I2S_BUFFER_OBJECT_INDEX i2sBufIndex;
    DRV_I2S_SSC_OBJ *drvObj;
    bool interruptWasEnabled;
    uint8_t dmaDataWidth = 0;

    /* The Client and driver objects from the handle */
    interruptWasEnabled = false;
    clientObj = (DRV_I2S_CLIENT_OBJ *) handle;
    drvObj = (DRV_I2S_SSC_OBJ *) clientObj->hDriver;

    ssc_registers_t volatile * ssc = ((ssc_registers_t *) (drvObj->sscID));
    
    /* check if the receive queue size is 0 */
    if (0 == drvObj->queueSizeReceive)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Cannot add a write operation as the queueSizeTransmit is 0 \r\n");
        return;
    }        /* Bound checking for the buffer parameters */
    else if ((NULL == buffer) || (0 == size) || (bufferHandle == NULL))
    {
        SYS_DEBUG(0,"\r\nDRV I2S SSC: Buffer Pointer is NULL or size is 0 \r\n");
        return;
    }
    /* See if the handle is still valid */
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0,"\r\nDRV I2S SSC: Invalid Driver Handle \r\n");
        return;
    }
    /* If the driver was not opened in read mode */
    else if (!(DRV_IO_INTENT_READ & clientObj->ioIntent))
    {
        SYS_DEBUG(0,"\r\nDRV I2S SSC: Driver was not opened for read \r\n");
        return;
    }
    else
    {
        ;
    }
    
    if (drvObj->audioCommWidth < 8)
    {
        dmaDataWidth = SYS_DMA_DATA_WIDTH_BYTE;
    }
    else if (drvObj->audioCommWidth < 16)
    {
         dmaDataWidth = SYS_DMA_DATA_WIDTH_HALF_WORD;
    }
    else if(drvObj->audioCommWidth < 32)
    {
         dmaDataWidth = SYS_DMA_DATA_WIDTH_WORD;
    }
    
    /* We will allow buffers to be added in the interrupt context of this 
     * I2S driver. But we must make sure that if we are in interrupt, then we 
     * should not modify mutexes. */
    
    if(!drvObj->isInInterruptContext)
    {
        /* Grab a mutex. This is okay because we are not in an interrupt 
         * context */
        if(OSAL_MUTEX_Lock(&(drvObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
        {
            /* We will disable interrupts so that the queue status does not 
             * get updated asynchronously. This code will always execute. */
            interruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
        }
        else
        {
            /* The mutex acquisition timed out. Return with an invalid handle. 
             * This code will not execute if there is no RTOS. */
            return;
        }
    }    
    i2sBufIndex = _DRV_I2S_SSC_ReadQueueObjectIndexGet();
    if (i2sBufIndex != DRV_I2S_BUFFER_OBJECT_INDEX_INVALID)
    {
        i2sBufObj = &gDrvI2SReadBufferQObj[i2sBufIndex];
        i2sBufObj->indexHandle = ((++gDrvI2SUniqueBufferHandle << 0x10) | i2sBufIndex);
        *bufferHandle = (DRV_I2S_BUFFER_HANDLE) i2sBufObj->indexHandle;
        i2sBufObj->rxbuffer = (uint8_t *)buffer;
        i2sBufObj->txbuffer = NULL;
        i2sBufObj->size = size;
        i2sBufObj->next = (DRV_I2S_BUFFER_OBJECT *) NULL;
        i2sBufObj->nPendingBytes = size;
        i2sBufObj->clientObject = clientObj;
        i2sBufObj->bType = DRV_I2S_BUFFER_OPERATION_TYPE_READ;

        /* If the queue is empty */
        if (drvObj->readQueueHead == ((DRV_I2S_BUFFER_OBJECT *) NULL))
        {
            drvObj->readQueueHead = i2sBufObj;
            
            /* Since this is the first buffer in the queue Add it immediately
             * to DMA for processing */
            
            SYS_DMA_ChannelTransferAdd(drvObj->dmaChannelHandleRead,
                            (const void *) &drvObj->moduleID->SSC_RHR,
                            i2sBufObj->size, i2sBufObj->rxbuffer,
                            dmaDataWidth,drvObj->cellSize); 
        }
        else
        {
            DRV_I2S_BUFFER_OBJECT *iterator;
            iterator = drvObj->readQueueHead;
            
            /* Insert the object at the end of the queue */
            while (iterator->next != NULL)
            {
                iterator = iterator->next;
            }
            iterator->next = i2sBufObj;
            /* DMA Mode: There is already a buffer under processing in 
             * the queue. This buffer will be added to DMA for processing 
             * immediately after the processing of the buffer prior to this 
             * buffer completes.
             * (This functionality is implemented in DRV_I2S_SSC_Tasks)*/
        }
        
        /* A location of the queue size is being used */
        drvObj->queueSizeReceive--;
        drvObj->queueCurrentReceiveSize++;

        /* We are done. Restore the interrupt enable status and return. */
        if(interruptWasEnabled)
        {
            /* DMA Mode of operation */
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
        
        /* Release mutex */
        if((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to unlock mutexDriverInstance mutex \r\n");
        }
        return;
    }
    else
    {
        /* This means we could not find a buffer. This will happen if \
         * the DRV_USART_QUEUE_DEPTH_COMBINED parameter is 
         * configured to be less */
        
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Insufficient Combined Queue Depth \r\n");
        
        /* Enable the interrupt if it was disabled */
        if(interruptWasEnabled)
        {
            _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
        }
        
        /* Release mutex */
        if((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "\r\nDRV I2S SSC: Unable to unlock mutexDriverInstance mutex \r\n");
        }
        return;
    }
}

// *****************************************************************************
/*
  Function:
    void DRV_I2S_BufferEventHandlerSet( const DRV_HANDLE handle,
                                DRV_I2S_BUFFER_EVENT_HANDLER eventHandler,
	                			uintptr_t contextHandle)

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
    If the client does not want to be notified when the queued buffer transfer
    has completed, it does not need to register a callback.
*/

void DRV_I2S_SSC_BufferEventHandlerSet
(
    DRV_HANDLE handle,
    const DRV_I2S_BUFFER_EVENT_HANDLER eventHandler,
    const uintptr_t contextHandle
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_SSC_OBJ *drvObj;

    /* Assign the event handler and the context */
    clientObj = (DRV_I2S_CLIENT_OBJ *) handle;
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid driver handle \r\n");
        return;
    }
    drvObj = clientObj->hDriver;
    
    /* Set the Event Handler and context */
    clientObj->pEventCallBack = eventHandler;
    clientObj->hClientArg = contextHandle;

    if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleRead 
        && SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleWrite)
    {
        if((DRV_IO_INTENT_READWRITE == (DRV_IO_INTENT_READWRITE & clientObj->ioIntent)))
        {
            SYS_DMA_ChannelTransferEventHandlerSet(drvObj->dmaChannelHandleRead,
                (SYS_DMA_CHANNEL_TRANSFER_EVENT_HANDLER)_DRV_I2S_SSC_DMA_EventHandler,
                (uintptr_t)drvObj);

            SYS_DMA_ChannelTransferEventHandlerSet(drvObj->dmaChannelHandleWrite,
                (SYS_DMA_CHANNEL_TRANSFER_EVENT_HANDLER)_DRV_I2S_SSC_DMA_EventHandler,
                (uintptr_t)drvObj);                                    
        }
        else
        {
            if((DRV_IO_INTENT_WRITE == (DRV_IO_INTENT_WRITE & clientObj->ioIntent))){
                SYS_DMA_ChannelTransferEventHandlerSet(drvObj->dmaChannelHandleWrite,
                    (SYS_DMA_CHANNEL_TRANSFER_EVENT_HANDLER)_DRV_I2S_SSC_DMA_WriteEventHandler,
                    (uintptr_t)drvObj);
            }
            else if((DRV_IO_INTENT_READ == (DRV_IO_INTENT_READ & clientObj->ioIntent)))
            {
                SYS_DMA_ChannelTransferEventHandlerSet(drvObj->dmaChannelHandleRead,
                    (SYS_DMA_CHANNEL_TRANSFER_EVENT_HANDLER)_DRV_I2S_SSC_DMA_ReadEventHandler,
                    (uintptr_t)drvObj);
            }
        } 
        
    }
    else
    {
        if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleWrite )
        {
            if((DRV_IO_INTENT_WRITE == (DRV_IO_INTENT_WRITE & clientObj->ioIntent)))
            {
                SYS_DMA_ChannelTransferEventHandlerSet(drvObj->dmaChannelHandleWrite,
                    (SYS_DMA_CHANNEL_TRANSFER_EVENT_HANDLER)_DRV_I2S_SSC_DMA_WriteEventHandler,
                    (uintptr_t)drvObj);
            }
        }
        else if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleRead)
        {
            if((DRV_IO_INTENT_READ == (DRV_IO_INTENT_READ & clientObj->ioIntent)))
            {
                SYS_DMA_ChannelTransferEventHandlerSet(drvObj->dmaChannelHandleRead,
                    (SYS_DMA_CHANNEL_TRANSFER_EVENT_HANDLER)_DRV_I2S_SSC_DMA_ReadEventHandler,
                    (uintptr_t)drvObj);
            }
        }
    }
    
    return;
}

// *****************************************************************************
/*
  Function:
    void DRV_I2S_BufferQueueFlush(DRV_HANDLE handle)

  Summary:
    This function flushes off the buffers associated with the client object.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function flushes off the buffers associated with the client object and
    disables the DMA channel used for transmission.

  Remarks:
     The Driver uses a specialized queue for the client's combined Read/Write, 
     Read Only, and Write Only operations.
*/

void DRV_I2S_SSC_BufferQueueFlush
(  
    DRV_HANDLE handle
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_SSC_OBJ *drvObj;
    DRV_I2S_BUFFER_OBJECT *iterator1, *readItr, *writeItr;    
    DRV_IO_INTENT ioIntent;
    bool dmaWriteInterruptWasEnabled;
    bool dmaReadInterruptWasEnabled;

    /* Assign the event handler and the context */
    clientObj = (DRV_I2S_CLIENT_OBJ *) handle;
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid driver handle \r\n");
        return;
    }
    ioIntent = clientObj->ioIntent;
    drvObj = (DRV_I2S_SSC_OBJ *) clientObj->hDriver;    
    iterator1 = drvObj->queueHead;       
    readItr = drvObj->readQueueHead;
    writeItr = drvObj->writeQueueHead;
    
    dmaWriteInterruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);
    dmaReadInterruptWasEnabled = _DRV_I2S_InterruptSourceDisable(drvObj->interruptDMA);    
        
    /* If the object being processed at the head of queue is owned 
     * by this client. (clientObj) */
    
    if (DRV_IO_INTENT_READWRITE == (ioIntent & DRV_IO_INTENT_READWRITE))
    {                        
        /* Clear the READWRITEQueue DMA */
        if(iterator1 != NULL && clientObj == (DRV_I2S_CLIENT_OBJ *)iterator1->clientObject )
        {
            SYS_DMA_ChannelForceAbort(drvObj->dmaChannelHandleWrite);
            SYS_DMA_ChannelForceAbort(drvObj->dmaChannelHandleRead);                     
        }
        while(iterator1 != NULL)
        {
            if(clientObj == (DRV_I2S_CLIENT_OBJ *)iterator1->clientObject )
            {
                iterator1->inUse = false;
                iterator1 = iterator1->next;
                drvObj->queueSizeTransmit++;
                drvObj->queueCurrentTransmitSize--;
            }
        }
    }   
    else if (DRV_IO_INTENT_READ == (ioIntent & DRV_IO_INTENT_READ))
    {
        /* Clear the READ Queue DMA */
        if(readItr != NULL && clientObj == (DRV_I2S_CLIENT_OBJ *)readItr->clientObject )
        {                
            SYS_DMA_ChannelForceAbort(drvObj->dmaChannelHandleRead);                
        }
        while(readItr != NULL)
        {
            if(clientObj == (DRV_I2S_CLIENT_OBJ *)readItr->clientObject )
            {
                readItr->inUse = false;
                readItr = readItr->next;
                drvObj->queueSizeReceive++;
                drvObj->queueCurrentReceiveSize--;
            } 
        }        
    }
    else if (DRV_IO_INTENT_WRITE == (ioIntent & DRV_IO_INTENT_WRITE))
    {
        /* Clear the WRITE Queue DMA */
        if(writeItr != NULL && clientObj == (DRV_I2S_CLIENT_OBJ *)writeItr->clientObject )
        {                 
            SYS_DMA_ChannelForceAbort(drvObj->dmaChannelHandleWrite);
        }
        while(writeItr != NULL)
        {
            if(clientObj == (DRV_I2S_CLIENT_OBJ *)writeItr->clientObject )
            {
                writeItr->inUse = false;
                writeItr = writeItr->next;
                drvObj->queueSizeTransmit++;
                drvObj->queueCurrentTransmitSize--;
            }
        }
    }        
    
    /* After removing the closed clientobj,If there are no buffers in the queue.
     * Make the head pointer point to NULL */
    
    if(0 == drvObj->queueCurrentReceiveSize &&
       0 == drvObj->queueCurrentTransmitSize)
    {
        drvObj->queueHead = (DRV_I2S_BUFFER_OBJECT *)0;
        drvObj->readQueueHead = (DRV_I2S_BUFFER_OBJECT *)0;
        drvObj->writeQueueHead = (DRV_I2S_BUFFER_OBJECT *)0;
        
        /* Once the queue becomes empty as a consequence to the calling of 
         * DRV_I2S_BufferQueueFlush, Any interrupt which was enabled and 
         * pending and earlier to the call to 'DRV_I2S_BufferQueueFlush' needs 
         * to be cleared as it might lead to a false handling of interrupt */
        
        if(dmaWriteInterruptWasEnabled && _DRV_I2S_InterruptSourceStatusGet(drvObj->interruptDMA))
        {            
            _DRV_I2S_InterruptSourceClear(drvObj->interruptDMA);                                
        }
        if(dmaReadInterruptWasEnabled && _DRV_I2S_InterruptSourceStatusGet(drvObj->interruptDMA))
        {
            _DRV_I2S_InterruptSourceClear(drvObj->interruptDMA);                
        }
        
    }
    else
    {
        /* READ/WRITE Queue */
        /* Iterate to update the head pointer to point the first valid buffer 
         * object in the queue */
        
        iterator1 = drvObj->queueHead;
        while(iterator1 != (DRV_I2S_BUFFER_OBJECT *)0)
        {
            if(iterator1->inUse == true)
            {
                drvObj->queueHead = iterator1;
                break;
            }
            iterator1 = iterator1->next;
        }

        /* READ Only Queue */
        readItr = drvObj->readQueueHead;
        while(readItr != (DRV_I2S_BUFFER_OBJECT *)0){
            if(readItr->inUse == true){
                drvObj->readQueueHead = readItr;
                break;
            }
            readItr = readItr->next;
        }

        /*WRITE Only Queue */
        writeItr = drvObj->writeQueueHead;
        while(writeItr != (DRV_I2S_BUFFER_OBJECT *)0){
            if(writeItr->inUse == true){
                drvObj->writeQueueHead = writeItr;
                break;
            }
            writeItr = writeItr->next;
        }
    }

    if(drvObj->queueHead != NULL){
        SYS_DMA_ChannelEnable(drvObj->dmaChannelHandleWrite);
        SYS_DMA_ChannelEnable(drvObj->dmaChannelHandleRead);   
    }
    else
    {
        if(drvObj->readQueueHead != NULL){
            SYS_DMA_ChannelEnable(drvObj->dmaChannelHandleRead);
        }
        if(drvObj->writeQueueHead != NULL){
            SYS_DMA_ChannelEnable(drvObj->dmaChannelHandleWrite);         
        }
    }
                        
    /* Re-enable the interrupt if it was disabled */
    if(dmaWriteInterruptWasEnabled)
    {
        _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
    }
    if(dmaReadInterruptWasEnabled)
    {
        _DRV_I2S_InterruptSourceEnable(drvObj->interruptDMA);
    }  
        
    return;
}

// *****************************************************************************
/* Function:
    DRV_I2S_ERROR DRV_I2S_ErrorGet(DRV_HANDLE handle)

  Summary:
    This function returns the error(if any) associated with the last client
    request.

  Description:
    This function returns the error(if any) associated with the last client
    request. The DRV_I2S_Read() and DRV_I2S_Write() will update the client
    error status when these functions return DRV_I2S_READ_ERROR and
    DRV_I2S_WRITE_ERROR respectively.  If the driver send a
    DRV_I2S_BUFFER_EVENT_ERROR to the client, the client can call this function
    to know the error cause. The error status will be updated on every operation
    and should be read frequently(ideally immediately after the driver operation
    has completed) to know the relevant error status.

  Remarks:
    It is the client's responsibility to make sure that the error status is
    obtained frequently. The driver will update the client error status
    irrespective of whether this has been examined by the client.
*/

DRV_I2S_ERROR DRV_I2S_SSC_ErrorGet
(
    DRV_HANDLE handle
)
{
    DRV_I2S_CLIENT_OBJ *clientObj;
    DRV_I2S_ERROR error;

    if(handle == (DRV_HANDLE)NULL || handle == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid Client handle \r\n");
        return (DRV_I2S_ERROR_NONE);
    }
    clientObj = (DRV_I2S_CLIENT_OBJ *) handle;

    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Invalid Driver Handle \r\n");
        return (DRV_I2S_ERROR_NONE);
    }

    /* Return the error. Clear the error before returning. */
    error = clientObj->errorInfo;
    clientObj->errorInfo = DRV_I2S_ERROR_NONE;
    
    return (error);
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    static void _DRV_I2S_HardwareSetup (DRV_I2S_OBJ *drvObj, 
                                        DRV_I2S_INIT * i2sInit )

  Summary:
    Initializes the hardware registers.

  Description:
    Take the initialization data from the application(through DRV_I2S_Initialize
    function) and initialize the hardware registers.

  Remarks:
    None
 */

static void _DRV_I2S_SSC_HardwareSetup
(
    DRV_I2S_SSC_OBJ *drvObj, 
    DRV_I2S_SSC_INIT *i2sInit
)
{
    ssc_registers_t volatile * ssc = ((ssc_registers_t *)(i2sInit->sscID));
    uint32_t volatile sscRxClockModeRegister = 0x00000000;
    uint32_t volatile sscRxFrameModeRegister = 0x00000000;
    uint32_t volatile sscTxClockModeRegister = 0x00000000;
    uint32_t volatile sscTxFrameModeRegister = 0x00000000;
           
    /* Receiver Configurations */
    sscRxClockModeRegister|= (i2sInit->rxClockSel << SSC_RCMR_CKS_Pos | i2sInit->rxClockOutputMode << SSC_RCMR_CKO_Pos |\
                              i2sInit->rxClockInverse << SSC_RCMR_CKI_Pos | i2sInit->rxClockGating << SSC_RCMR_CKG_Pos |\
                              i2sInit->txStartSel << SSC_RCMR_START_Pos | i2sInit->rxStop << SSC_RCMR_STOP_Pos |\
                              i2sInit->rxStartDelay << SSC_RCMR_STTDLY_Pos | i2sInit->rxPeriodDividerSel << SSC_RCMR_PERIOD_Pos);
  
    ssc->SSC_RCMR.w = sscRxClockModeRegister;
    
    sscRxFrameModeRegister |= (i2sInit->rxDataLen << SSC_RFMR_DATLEN_Pos | i2sInit->loopMode << SSC_RFMR_LOOP_Pos |\
                                i2sInit->rxMSBFirst << SSC_RFMR_MSBF_Pos | i2sInit->rxDataNumberperFrame << SSC_RFMR_DATNB_Pos |\
                                i2sInit->rxFrameSyncLen << SSC_RFMR_FSLEN_Pos | i2sInit->rxFSOutSel << SSC_RFMR_FSOS_Pos |\
                                i2sInit->rxFSEdgeDetection << SSC_RFMR_FSEDGE_Pos |i2sInit->rxFSLenExtn << SSC_RFMR_FSLEN_EXT_Pos );
       
    ssc->SSC_RFMR.w = sscRxFrameModeRegister;
    
    /* Transmitter Configurations */
    sscTxClockModeRegister|= ( i2sInit->txClockSel << SSC_TCMR_CKS_Pos | i2sInit->txClockOutputMode << SSC_TCMR_CKO_Pos |\
                               i2sInit->txClockInverse << SSC_TCMR_CKI_Pos | i2sInit->txClockGating << SSC_TCMR_CKG_Pos |\
                               i2sInit->txStartSel << SSC_TCMR_START_Pos | i2sInit->txStartDelay << SSC_TCMR_STTDLY_Pos |\
                               i2sInit->txPeriodDividerSel << SSC_TCMR_PERIOD_Pos );
    
    ssc->SSC_TCMR.w = sscTxClockModeRegister;
    
    sscTxFrameModeRegister |= (i2sInit->txDataLen << SSC_TFMR_DATLEN_Pos | i2sInit->txDefaultData << SSC_TFMR_DATDEF_Pos | \
                                i2sInit->txMSBFirst << SSC_TFMR_MSBF_Pos | i2sInit->txDataNumberperFrame << SSC_TFMR_DATNB_Pos |\
                                i2sInit->txFrameSyncLen << SSC_TFMR_FSLEN_Pos | i2sInit->txFSOutSel << SSC_TFMR_FSOS_Pos |\
                                i2sInit->txFSEdgeDetection << SSC_TFMR_FSEDGE_Pos | i2sInit->txFSLenExtn << SSC_TFMR_FSLEN_EXT_Pos );
    
    ssc->SSC_TFMR.w = sscTxFrameModeRegister;
    
    /* Common Clock Divider configuration */
    ssc->SSC_CMR.w = (SSC_CMR_DIV_Msk & ((i2sInit->clockDivider) << SSC_CMR_DIV_Pos));
    
    /* DMA mode of operation. Allocate a handle for the specified channel.
     * Setup the channel for transfer */
    
    /* Initializing the channel handles with invalid value */
    drvObj->dmaChannelHandleWrite = SYS_DMA_CHANNEL_HANDLE_INVALID;
    drvObj->dmaChannelHandleRead = SYS_DMA_CHANNEL_HANDLE_INVALID;

    /* Tx DMA channel setup */
    if(DMA_CHANNEL_NONE != i2sInit->dmaChannelSSCTransmit &&   
        DMA_CHANNELS_NUMBER != i2sInit->dmaChannelSSCTransmit)
    {
        drvObj->dmaChannelHandleWrite =
                SYS_DMA_ChannelAllocate(i2sInit->dmaChannelSSCTransmit);
        if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleWrite)
        {
            /* Setting the channel priority same as the channel number */
            SYS_DMA_ChannelSetup(drvObj->dmaChannelHandleWrite,
                SYS_DMA_CHANNEL_OP_MODE_BASIC, i2sInit->dmaChannelTransmitTrigger);
            SYS_DMA_ChannelDataWidthSet(drvObj->dmaChannelHandleWrite,SYS_DMA_DATA_WIDTH_HALF_WORD);
        }
    }

    /* Rx DMA channel setup */
    if(DMA_CHANNEL_NONE != i2sInit->dmaChannelSSCReceive &&
        DMA_CHANNELS_NUMBER != i2sInit->dmaChannelSSCReceive)
    {
        drvObj->dmaChannelHandleRead =
                SYS_DMA_ChannelAllocate(i2sInit->dmaChannelSSCReceive);
        if(SYS_DMA_CHANNEL_HANDLE_INVALID != drvObj->dmaChannelHandleRead)
        {
            /* Setting the channel priority same as the channel number */
            SYS_DMA_ChannelSetup(drvObj->dmaChannelHandleRead,
                SYS_DMA_CHANNEL_OP_MODE_BASIC, i2sInit->dmaChannelReceiveTrigger);
        }
    }
    
    return;
}

/*******************************************************************************
  Function:
    static DRV_I2S_BUFFER_OBJECT *_DRV_I2S_QueueObjectGet(void)

  Summary:
    Gets a buffer queue object from the available pool.

  Description:
    Take the initialization data from the application (through DRV_I2S_Initialize
    function) and initialize the hardware registers.

  Remarks:
    None
 */

static DRV_I2S_BUFFER_OBJECT_INDEX _DRV_I2S_SSC_QueueObjectIndexGet
(
    void
)
{
    DRV_I2S_BUFFER_OBJECT *queueObj;
    uint32_t index;
    SYS_INT_PROCESSOR_STATUS int_status;
    queueObj = &gDrvI2SBufferQObj[0];

    for (index = 0; index < DRV_I2S_QUEUE_DEPTH_COMBINED; index++)
    {
        int_status = SYS_INT_StatusGetAndDisable();
        if (false == queueObj->inUse)
        {
            queueObj->inUse = true;
            SYS_INT_StatusRestore(int_status);
            return index;
        }
        SYS_INT_StatusRestore(int_status);
        queueObj++;
    }

    return DRV_I2S_BUFFER_OBJECT_INDEX_INVALID;
}

static DRV_I2S_BUFFER_OBJECT_INDEX _DRV_I2S_SSC_ReadQueueObjectIndexGet
(
    void
)
{
    DRV_I2S_BUFFER_OBJECT *queueObj;
    uint32_t index;
    SYS_INT_PROCESSOR_STATUS int_status;
    queueObj = &gDrvI2SReadBufferQObj[0];

    for (index = 0; index < QUEUE_SIZE_RX_IDX0; index++)
    {
        int_status = SYS_INT_StatusGetAndDisable();
        if (false == queueObj->inUse)
        {
            queueObj->inUse = true;
            SYS_INT_StatusRestore(int_status);
            return index;
        }
        SYS_INT_StatusRestore(int_status);
        queueObj++;
    }
    return DRV_I2S_BUFFER_OBJECT_INDEX_INVALID;
}

static DRV_I2S_BUFFER_OBJECT_INDEX _DRV_I2S_SSC_WriteQueueObjectIndexGet(void)
{
    DRV_I2S_BUFFER_OBJECT *queueObj;
    uint32_t index;
    SYS_INT_PROCESSOR_STATUS int_status;
    queueObj = &gDrvI2SWriteBufferQObj[0];
    for (index = 0; index < QUEUE_SIZE_TX_IDX0; index++)
    {
        int_status = SYS_INT_StatusGetAndDisable();
        if (false == queueObj->inUse)
        {
            queueObj->inUse = true;
            SYS_INT_StatusRestore(int_status);
            return index;
        }
        SYS_INT_StatusRestore(int_status);
        queueObj++;
    }
    return DRV_I2S_BUFFER_OBJECT_INDEX_INVALID;
}

/*******************************************************************************
  Function:
    void _DRV_I2S_DMA_EventHandler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)

  Summary:
 Callback for the DMA channels

  Description:
 Callback for the DMA channels

  Remarks:
    None
 */

void _DRV_I2S_SSC_DMA_EventHandler
(
    SYS_DMA_TRANSFER_EVENT event,
    SYS_DMA_CHANNEL_HANDLE handle, 
    uintptr_t contextHandle
)
{
    DRV_I2S_SSC_OBJ *drvObj;

    if(SYS_DMA_CHANNEL_HANDLE_INVALID == handle || 0 == handle)
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Handle is invalid \r\n");
        return;
    }
    drvObj = (DRV_I2S_SSC_OBJ *)contextHandle;
    if(NULL != drvObj->queueHead)
    {
        if(drvObj->queueHead->bType == DRV_I2S_BUFFER_OPERATION_TYPE_WRITEREAD)
        {
            if(drvObj->dmaChannelHandleRead == handle)
            {
                DRV_I2S_Tasks((SYS_MODULE_OBJ)drvObj);
            }
            else
            {
                ;
            }
        }
        
    }
    return;
}

void _DRV_I2S_SSC_DMA_ReadEventHandler
(
    SYS_DMA_TRANSFER_EVENT event,
    SYS_DMA_CHANNEL_HANDLE handle, 
    uintptr_t contextHandle
)
{
    DRV_I2S_SSC_OBJ *drvObj;

    if(SYS_DMA_CHANNEL_HANDLE_INVALID == handle || 0 == handle)
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Handle is invalid \r\n");
        return;
    }
    drvObj = (DRV_I2S_SSC_OBJ *)contextHandle;
    DRV_I2S_SSC_ReadTasks((SYS_MODULE_OBJ)drvObj);
    
    return;
}

void _DRV_I2S_SSC_DMA_WriteEventHandler
(
    SYS_DMA_TRANSFER_EVENT event,
    SYS_DMA_CHANNEL_HANDLE handle, 
    uintptr_t contextHandle
)
{
    DRV_I2S_SSC_OBJ *drvObj;

    if(SYS_DMA_CHANNEL_HANDLE_INVALID == handle || 0 == handle)
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "\r\nDRV I2S SSC: Handle is invalid \r\n");
        return;
    }
 
    drvObj = (DRV_I2S_SSC_OBJ *)contextHandle;
    DRV_I2S_SSC_WriteTasks((SYS_MODULE_OBJ)drvObj);
    
    return;
}