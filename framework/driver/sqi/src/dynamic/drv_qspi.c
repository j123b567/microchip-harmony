/*******************************************************************************
  QSPI Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
   drv_spi.c

  Summary:
    QSPI Driver implementation

  Description:
    The QSPI Driver provides a interface to access the QSPI hardware on PIC32C
    microcontroller.  This file implements the QSPI Driver. This file
    should be included in the project if SPI driver functionality is needed.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 - 2017 released Microchip Technology Inc. All rights reserved.

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

#include "driver/sqi/drv_sqi.h"
#include "driver/sqi/src/drv_qspi_local.h"
#include "system/dma/src/sys_dma_local_pic32c.h"

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/
uint16_t gDrvSQIBufferToken = 0;

/*************************************************
 * OSAL Declarations
 *************************************************/
/* QSPI Client Object Mutex */
OSAL_MUTEX_DECLARE(qspiClientObjMutex);
/* QSPI Buffer Object Mutex*/
OSAL_MUTEX_DECLARE(qspiBufObjMutex);

/*************************************************
 * DMA Declarations
 *************************************************/
SYS_DMA_CHANNEL_TRANSFER_EVENT_HANDLER _DMA_QSPI_Channels_EventHandler (SYS_DMA_TRANSFER_EVENT event, SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);

/*************************************************
 * QSPI Objects Declarations
 *************************************************/
DRV_SQI_CLIENT_OBJECT gDrvSQIClientObj[DRV_SQI_CLIENTS_NUMBER];
DRV_SQI_OBJECT gDrvSQIObj[DRV_SQI_INSTANCES_NUMBER];
DRV_SQI_BUFFER_OBJECT gDrvSQIBufferObject[DRV_SQI_BUFFER_OBJECT_NUMBER];

/*************************************************
 * QSPI Internal functions
 *************************************************/
static void _QSPI_Transfer_data(DRV_SQI_OBJECT *dObj,DRV_SQI_BUFFER_OBJECT *bObj)
{
    SYS_DMA_CHANNEL_OBJECT *dmaObj ;
    uint32_t *pQspiBuffer = (uint32_t *)QSPI_RW_BUFFER_ADDR;
    uint32_t temp,dma_xfer_size,dma_sram_buffer_address;
    /* - Configure SQI mode for new transfer */
    temp = dObj->sqiId->QSPI_IFR.w;
    if ((bObj->xferData[bObj->xferCnt].flag & 3) == DRV_SQI_FLAG_MODE_SINGLE_LANE )
    {
        temp &=~(QSPI_IFR_WIDTH_Msk);
        temp |= QSPI_IFR_WIDTH_SINGLE_BIT_SPI;
    }
    if ((bObj->xferData[bObj->xferCnt].flag & 3) == DRV_SQI_FLAG_MODE_DUAL_LANE )
    {
        temp &=~(QSPI_IFR_WIDTH_Msk);
        temp |= QSPI_IFR_WIDTH_DUAL_CMD;
    }
    if ((bObj->xferData[bObj->xferCnt].flag & 3) == DRV_SQI_FLAG_MODE_QUAD_LANE )
    {
        temp &=~(QSPI_IFR_WIDTH_Msk);
        temp |= QSPI_IFR_WIDTH_QUAD_CMD;
    }
    /* - Check Transfer direction  */
    if (bObj->xferData[bObj->xferCnt].flag & DRV_SQI_FLAG_DIR_READ)
    {
        /* Prepare Read transaction */
        temp &=~(QSPI_IFR_TFRTYP_Msk);
        temp |= QSPI_IFR_TFRTYP(0);
        temp |= QSPI_IFR_DATAEN_Msk;
        temp &= ~(QSPI_IFR_INSTEN_Msk);
        dObj->sqiId->QSPI_IFR.w = temp;
        /* Check if transfer size is a multiple of 32Bit  */
        if(((bObj->xferData[bObj->xferCnt].length & 3) != 0 )&& (bObj->xferData[bObj->xferCnt].length < 4))
        {
            /* Read un-aligned as 8bit data */
            memcpy((bObj->xferData[bObj->xferCnt].data)++ , pQspiBuffer,1);
            dObj->sqiId->QSPI_CR.w |= QSPI_CR_LASTXFER_Msk;
            bObj->xferData[bObj->xferCnt].length--;
        }
        else
        {
            /* Prepare DMA transfer size and address */
            dma_xfer_size = (bObj->xferData[bObj->xferCnt].length - (bObj->xferData[bObj->xferCnt].length & 3));
            dma_sram_buffer_address = (uint32_t) bObj->xferData[bObj->xferCnt].data;
            /* Prepare next transfer */
            bObj->xferData[bObj->xferCnt].data = ((bObj->xferData[bObj->xferCnt].data) + dma_xfer_size);
            bObj->xferData[bObj->xferCnt].length = (bObj->xferData[bObj->xferCnt].length & 3);
            /* Read 32-bit aligned data frame using DMA */
            dmaObj = (SYS_DMA_CHANNEL_OBJECT *)dObj->DMA_Rx_Channel;
            SYS_DMA_ChannelSetup(dObj->DMA_Rx_Channel,SYS_DMA_CHANNEL_OP_MODE_BASIC,DMA_TRIGGER_SOURCE_NONE);
            // Configure Channel (not supported by current SYS_DMA service)
            CAST(xdmac_registers_t,XDMAC_ID_0)->XDMAC_CHID[dmaObj->channelID].XDMAC_CC.w = ( 
                    XDMAC_CC_MBSIZE(XDMAC_CC_MBSIZE_SINGLE_Val) |
                    XDMAC_CC_DWIDTH(XDMAC_CC_DWIDTH_WORD_Val) |
                    XDMAC_CC_SIF_Msk | 
                    XDMAC_CC_SAM(XDMAC_CC_SAM_FIXED_AM_Val) |
                    XDMAC_CC_DAM(XDMAC_CC_DAM_INCREMENTED_AM_Val) 
            );
            // Configure Mem to Mem transfer from QSPI reserved memory space to SRAM  
            SYS_DMA_ChannelTransferAdd( dObj->DMA_Rx_Channel, 
                    (void *)QSPI_RW_BUFFER_ADDR,
                    (dma_xfer_size >> 2),
                    (void *)dma_sram_buffer_address,
                    (dma_xfer_size >> 2),
                    1
            );
        }   
    }
    else
    {
        /* Prepare Write transfer */
        temp &=~(QSPI_IFR_TFRTYP_Msk);
        temp |= QSPI_IFR_TFRTYP(2);
        /* Check if remaining transfer size is a multiple of 32Bit  */
        if(((bObj->xferData[bObj->xferCnt].length & 3) != 0 )&& (bObj->xferData[bObj->xferCnt].length < 4))
        {
            /* Write 8-bit data */
            temp |= QSPI_IFR_INSTEN_Msk;
            temp &= ~(QSPI_IFR_DATAEN_Msk);
            dObj->sqiId->QSPI_ICR.w = *(bObj->xferData[bObj->xferCnt].data)++;
            dObj->sqiId->QSPI_IFR.w = temp; 
            bObj->xferData[bObj->xferCnt].length--;
        }
        else
        {
            /* Write @32-bit data frame using DMA */
            dmaObj = (SYS_DMA_CHANNEL_OBJECT *)dObj->DMA_Tx_Channel;
            temp &= ~(QSPI_IFR_INSTEN_Msk);
            temp |= QSPI_IFR_DATAEN_Msk;
            dObj->sqiId->QSPI_IFR.w = temp;
            /* Prepare DMA transfer size and address */
            dma_xfer_size = (bObj->xferData[bObj->xferCnt].length - (bObj->xferData[bObj->xferCnt].length & 3));
            dma_sram_buffer_address = (uint32_t)bObj->xferData[bObj->xferCnt].data;
            /* Prepare next transfer */
            bObj->xferData[bObj->xferCnt].data = ((bObj->xferData[bObj->xferCnt].data) + dma_xfer_size);
            bObj->xferData[bObj->xferCnt].length = (bObj->xferData[bObj->xferCnt].length & 3);
            SYS_DMA_ChannelSetup(dObj->DMA_Tx_Channel,SYS_DMA_CHANNEL_OP_MODE_BASIC,DMA_TRIGGER_SOURCE_NONE);
            // Configure Channel (not supported by current SYS_DMA service)
            CAST(xdmac_registers_t,XDMAC_ID_0)->XDMAC_CHID[dmaObj->channelID].XDMAC_CC.w = ( 
                    XDMAC_CC_MBSIZE(XDMAC_CC_MBSIZE_SINGLE_Val) |
                    XDMAC_CC_DWIDTH(XDMAC_CC_DWIDTH_WORD_Val) |
                    XDMAC_CC_DIF_Msk | 
                    XDMAC_CC_SAM(XDMAC_CC_SAM_INCREMENTED_AM_Val) |
                    XDMAC_CC_DAM(XDMAC_CC_DAM_FIXED_AM_Val) 
            );
            // Configure Mem to Mem transfer from SRAM to QSPI reserved memory space  
            SYS_DMA_ChannelTransferAdd( dObj->DMA_Tx_Channel, 
                    (void *)dma_sram_buffer_address,
                    (dma_xfer_size >> 2),
                    (void *)QSPI_RW_BUFFER_ADDR,
                    (dma_xfer_size >> 2),
                    1
            );
        }        
    }
}

SYS_DMA_CHANNEL_TRANSFER_EVENT_HANDLER _DMA_QSPI_Channels_EventHandler(SYS_DMA_TRANSFER_EVENT event, SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    volatile uint32_t flags,temp ;
    DRV_SQI_OBJECT *dObj = NULL;
    QUEUE_OBJECT *qObj = NULL;
    DRV_SQI_BUFFER_OBJECT *bObj;
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    
    /* Get Driver object from Handler context */
    dObj = (DRV_SQI_OBJECT *)contextHandle; 
    /* - Get queue Object from Driver object*/
    qObj = &dObj->queue;
    /* - Get current transfer Object from Driver queue*/
    bObj = qObj->head->data;
    /* Stop Current transfer */
    dObj->sqiId->QSPI_SR.w;
    dObj->sqiId->QSPI_CR.w |= QSPI_CR_LASTXFER_Msk;
}

/** This function finds and allocates a free client object. **/
static DRV_SQI_CLIENT_OBJECT* _DRV_SQI_AllocateClientObject ( void )
{
    uint8_t iClient = 0;
    DRV_SQI_CLIENT_OBJECT *object = &gDrvSQIClientObj[0];
    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_SQI_CLIENTS_NUMBER ; iClient++)
    {
        if (!object->inUse)
        {
            return object;
        }
        object ++;
    }
    return NULL;
}

/* This function validates the client handle. */
static DRV_SQI_CLIENT_OBJECT* _DRV_SQI_ValidateClientHandle ( DRV_HANDLE handle )
{
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;

    /* Validate the handle */
    if ((handle == (DRV_HANDLE)NULL) || (handle == DRV_HANDLE_INVALID))
    {
        return NULL;
    }
    /* See if the client has been opened */
    clientObj = (DRV_SQI_CLIENT_OBJECT *)handle;
    if (!clientObj->inUse)
    {
        return NULL;
    }
    /* Check if the driver is ready for operation */
    dObj = (DRV_SQI_OBJECT *)clientObj->driverObj;
    if (dObj->status != SYS_STATUS_READY)
    {
        return NULL;
    }
    return clientObj;
}

/*************************************************
 * QSPI External functions
 *************************************************/

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SQI_Initialize ( const SYS_MODULE_INDEX drvIndex,
                                        const SYS_MODULE_INIT *const init )

  Summary:
    Initializes the QSPI instance for the specified driver index

  Description:
    This routine initializes the QSPI driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Remarks:
    This routine must be called before any other QSPI routine is called.

    This routine should only be called once during system initialization unless
    DRV_SQI_Deinitialize is called to deinitialize the driver instance.

    This routine will NEVER block for hardware access. If the operation
    requires time to allow the hardware to initialize, it will be reported by
    the DRV_SQI_Status operation. The system must use DRV_SQI_Status to find
    out when the driver is in the ready state.
*/
SYS_MODULE_OBJ DRV_SQI_Initialize ( const SYS_MODULE_INDEX drvIndex,
                                    const SYS_MODULE_INIT *const init )
{
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_SQI_OBJECT *dObj = NULL;
    DRV_SQI_INIT *sqiInit;

    sqiInit = (DRV_SQI_INIT *)init;
    /* - Validate driver index */
    if (drvIndex > DRV_SQI_INSTANCES_NUMBER)
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    /* - Check if the instance has already been initialized */
    if (gDrvSQIObj[drvIndex].inUse)
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    /* Create Client OSAL Mutex */
    retVal = OSAL_MUTEX_Create(&qspiClientObjMutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    /* Create QSPI buffer object OSAL Mutex */
    retVal = OSAL_MUTEX_Create(&qspiBufObjMutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }  
    
    /* - Get Driver object according to driver index*/
    dObj = &gDrvSQIObj[drvIndex];
    /* - Indicate that this object is in use */
    dObj->inUse = true;
    /* - Check if clock divider parameter is valid*/
    if (dObj->clockDivider == -1)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Initialize(): Invalid Clock parameter.\n");
        return SYS_MODULE_OBJ_INVALID;
    }
    /* - Allocate QSPI DMA Tx channel */
    dObj->DMA_Tx_Channel = SYS_DMA_ChannelAllocate ( DMA_CHANNEL_ANY );
    if ( dObj->DMA_Tx_Channel == SYS_DMA_CHANNEL_HANDLE_INVALID)
    {
        return SYS_MODULE_OBJ_INVALID;
    }
    SYS_DMA_ChannelTransferEventHandlerSet(dObj->DMA_Tx_Channel,
                                (void *) _DMA_QSPI_Channels_EventHandler,
                                (uintptr_t) dObj);
    
    /* - Allocate QSPI DMA Rx channel */
    dObj->DMA_Rx_Channel = SYS_DMA_ChannelAllocate ( DMA_CHANNEL_ANY );
    if ( dObj->DMA_Rx_Channel == SYS_DMA_CHANNEL_HANDLE_INVALID)
    {
        return SYS_MODULE_OBJ_INVALID;
    }
        SYS_DMA_ChannelTransferEventHandlerSet(dObj->DMA_Rx_Channel,
                               (void *) _DMA_QSPI_Channels_EventHandler,
                               (uintptr_t) dObj);
    /* - Link QSPI instance to driver Object */
    dObj->sqiId = sqiInit->sqiId;
    /* - Drive SQI chip select high*/
    BSP_SQI_CSOn();
     /* - Set driver object Interrupt Source */
    dObj->interruptSource = sqiInit->interruptSource;
    /* - Disable QSPI */
    dObj->sqiId->QSPI_CR.w = QSPI_CR_QSPIDIS_Msk ;
    while( (bool)(dObj->sqiId->QSPI_SR.w & QSPI_SR_QSPIENS_Msk))
    {
        // Do Nothing
    }
    /* - Reset QSPI */
    dObj->sqiId->QSPI_CR.w = QSPI_CR_SWRST_Msk ;
    /* - Set in memory Mode */
    dObj->sqiId->QSPI_MR.w |= QSPI_MR_SMM_Msk;
    /* - Disable Frame Instruction sending */
    dObj->sqiId->QSPI_IFR.w &= ~(QSPI_IFR_INSTEN_Msk);
    /* - Disable Frame Option sending */
    dObj->sqiId->QSPI_IFR.w &= ~(QSPI_IFR_OPTEN_Msk);
    /* - Disable Frame Address sending */
    dObj->sqiId->QSPI_IFR.w &= ~(QSPI_IFR_ADDREN_Msk);
    /* - Disable QSPI loop back */
    dObj->sqiId->QSPI_MR.w &= (~QSPI_MR_LLB_Msk);
    /* - Disable wait data read before transfer */
    dObj->sqiId->QSPI_MR.w &= (~QSPI_MR_WDRBT_Msk);
    /* - Set QSPI minimum inactive QCS delay to 1 cycle of QSPI clock */
    dObj->sqiId->QSPI_MR.w |= QSPI_MR_DLYCS(0);
    /* - Set QSPI delay between consecutive transfers to 0 */
    dObj->sqiId->QSPI_MR.w |= QSPI_MR_DLYBCT(0);
    /* - Set QSPI mode (CPOL & CPHA) */
    dObj->sqiId->QSPI_SCR.w = sqiInit->spiMode ;
    /* - Set clock divider */
    dObj->sqiId->QSPI_SCR.w |=  QSPI_SCR_SCBR(sqiInit->clockDivider);
    /* - Disable scrambling mode */
    dObj->sqiId->QSPI_SMR.w &= (~QSPI_SMR_SCREN_Msk);
    dObj->sqiId->QSPI_SKR.w = QSPI_SKR_USRK(0);
    /* - Enable QSPI */
    dObj->sqiId->QSPI_CR.w |= QSPI_CR_QSPIEN_Msk;
    while(!(bool)(dObj->sqiId->QSPI_SR.w & QSPI_SR_QSPIENS_Msk))
    {
        // Do nothing
    }
    /* - Initialize Driver object Queue*/
    QUEUE_Initialize(&dObj->queue, DRV_SQI_BUFFER_OBJECT_NUMBER, dObj->queueElements);
    /* - Set Driver Object status as ready */
    dObj->status = SYS_STATUS_READY;
    /* - Return the driver index and the System Module Object */
    return drvIndex;
}

// ****************************************************************************
/* Function:
    void DRV_SQI_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the QSPI driver module

  Description:
    Deinitializes the specified instance of the QSPI driver module, disabling its
    operation (and any hardware). Invalidates all the internal data.

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.
*/
void DRV_SQI_Deinitialize ( SYS_MODULE_OBJ object )
{
    DRV_SQI_OBJECT * dObj = (DRV_SQI_OBJECT*)NULL;

    /* - Validate the object */
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_SQI_INSTANCES_NUMBER))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Deinitialize(): Invalid parameter.\n");
        return;
    }
    /* - Get Driver object according to object index*/
    dObj = (DRV_SQI_OBJECT*)&gDrvSQIObj[object];
    if(DRV_SQI_INTERRUPT_MODE == true)
    {
        /* - Disable the QSPI Interrupt */
        SYS_INT_SourceDisable(dObj->interruptSource);
    }
    /* - Reset the client count and the exclusive flag */
    dObj->numClients = 0;
    dObj->isExclusive = false;
    /* - Reset the queue */
    QUEUE_Reset(&dObj->queue);
    /* - Set the Hardware instance object status an un-initialized */
    dObj->status = SYS_STATUS_UNINITIALIZED;
    /* - Set Hardware instance object as not used */
    dObj->inUse = false;
    OSAL_MUTEX_Delete(&qspiClientObjMutex);
    OSAL_MUTEX_Delete(&qspiBufObjMutex);
}

// ****************************************************************************
/* Function:
    SYS_STATUS DRV_SQI_Status ( SYS_MODULE_OBJ object )
  Summary:
    Gets the current status of the QSPI driver module.
  Description:
    This routine provides the current status of the QSPI driver module.
  Remarks:
    Refer to drv_qspi.h for usage information.
*/
SYS_STATUS DRV_SQI_Status ( SYS_MODULE_OBJ object )
{
    /* - Validate the object */
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_SQI_INSTANCES_NUMBER))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Status(): Invalid parameter.\n");
        return SYS_STATUS_UNINITIALIZED;
    }
    /* Return the driver status */
    return (gDrvSQIObj[object].status);
}

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_SQI_Open ( const SYS_MODULE_INDEX drvIndex,
                              const DRV_IO_INTENT ioIntent )

  Summary:
    Opens the specified QSPI driver instance and returns a handle to it

  Description:
    This routine opens the specified QSPI driver instance and provides a handle.
    This handle must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Remarks:
    The handle returned is valid until the DRV_SQI_Close routine is called.
    This routine will NEVER block waiting for hardware. If the driver has
    has already been opened, it cannot be opened exclusively.
*/
DRV_HANDLE DRV_SQI_Open ( const SYS_MODULE_INDEX drvIndex,
                          const DRV_IO_INTENT ioIntent )
{
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    /* - Validate driver index */
    if (drvIndex >= DRV_SQI_INSTANCES_NUMBER)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Invalid driver index.\n");
        return DRV_HANDLE_INVALID;
    }
    /* - Get Driver object according to driver index*/
    dObj = &gDrvSQIObj[drvIndex];
    /* - Check if Driver is ready*/
    if (dObj->status != SYS_STATUS_READY)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Driver not ready.\n");
        return DRV_HANDLE_INVALID;
    }
    /* - Check if the driver has already been opened in exclusive mode */
    if ((dObj->isExclusive) || ((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE)))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Driver cannot be opened exclusively.\n");
        return DRV_HANDLE_INVALID;
    }
    /* - Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&qspiClientObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Failed to acquire the SQI Client Object Mutex.\n");
        return DRV_HANDLE_INVALID;
    }
    /* - Allocate Client object*/
    clientObj = _DRV_SQI_AllocateClientObject();
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Failed to allocate a Client Object.\n");
    }
    else
    {
        /* - Found a client object that can be used */
        clientObj->inUse = true;
        clientObj->driverObj =  dObj;
        clientObj->intent = ioIntent;
        clientObj->eventHandler = NULL;
        if (ioIntent & DRV_IO_INTENT_EXCLUSIVE)
        {
            /* Driver was opened in exclusive mode */
            dObj->isExclusive = true;
        }
        dObj->numClients ++;
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_Open(): Open successful.\n");
    }
    OSAL_MUTEX_Unlock(&qspiClientObjMutex);
    return clientObj ? ((DRV_HANDLE)clientObj) : DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    void DRV_SQI_Close( const DRV_HANDLE handle )

  Summary:
    Closes an opened-instance of the QSPI driver

  Description:
    This routine closes an opened-instance of the QSPI driver, invalidating the
    handle.

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines. A new handle must be obtained by
    calling DRV_SQI_Open before the caller may use the driver again. Usually
    there is no need for the driver client to verify that the Close operation
    has completed.
*/
void DRV_SQI_Close( const DRV_HANDLE handle )
{
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;
    DRV_SQI_BUFFER_OBJECT *bObj = NULL;
    QUEUE_ELEMENT_OBJECT *prev_qElement = NULL;
    QUEUE_ELEMENT_OBJECT *qElement;

    /* - Get the Client object from the handle passed */
    clientObj = _DRV_SQI_ValidateClientHandle(handle);
    /* - Check if the driver handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SQI_Close(): Invalid handle.\n");
        return;
    }
    /* - Get driver object from Client object */
    dObj = clientObj->driverObj;
    qElement = dObj->queue.head;
    if (DRV_SQI_INTERRUPT_MODE == true)
    {
        /* - Disable QSPI Interrupts at core level */
        SYS_INT_SourceDisable(dObj->interruptSource);
    }
    /* - Remove Client buffer object from the queue*/
    while (qElement != NULL)
    {
        bObj = qElement->data;
        if ( bObj->hClient == (DRV_HANDLE) clientObj)
        {
            if(prev_qElement==NULL)
            {
                QUEUE_Pop(&dObj->queue);
            }
            else
            {
                // remove object from queue
                prev_qElement->next = qElement->next;
                qElement->inUse = false;
                dObj->queue.numElements --;
            }
            /* Clear buffer object */
            bObj->inUse = false;
            bObj->sqiId = NULL;
            bObj->hClient = (DRV_HANDLE) NULL;
            bObj->commandHandle = (DRV_SQI_COMMAND_HANDLE) NULL;
            bObj->status = DRV_SQI_COMMAND_COMPLETED;
            bObj->xferData = NULL;
            bObj->xferNumber = 0;
            bObj->xferCnt = 0;
        }
        qElement = qElement->next;
    }
    /* - Decrease the client count */
    dObj->numClients --;
    if ((DRV_SQI_INTERRUPT_MODE == true)&&(dObj->numClients > 0))
    {
        /* - Disable QSPI Interrupts at core level */
        SYS_INT_SourceDisable(dObj->interruptSource);
    }
    /* - Remove Object exclusivity */
    dObj->isExclusive = false;
    /* - Free the Client Instance */
    clientObj->inUse = false;
    SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SQI_Close(): Close successful.\n");
    return;
}

// *****************************************************************************
/* Function:
    void DRV_SQI_TransferData ( DRV_HANDLE handle, DRV_SQI_COMMAND_HANDLE *commandHandle,
                                uint8_t qspiDevice, DRV_SQI_TransferElement *xferData,
                                uint8_t numElements )

  Summary:
    Queue a data transfer operation on the specified QSPI device.

  Description:
    This routine queues a data transfer operation on the specified QSPI device.
    The reads or writes of blocks of data generally involves sending down the
    read or a write command, the address on the device from/to which data is to
    be read/written. The client also has to specify the source or destination
    buffer and the number of bytes to be read or written. The client builds an
    array of transfer elements containing these information and passes the
    array and the number of elements of the array as part of this transfer
    operation. If an event handler is registered with the driver the event
    handler would be invoked with the status of the operation once the
    operation has been completed. The function returns
    DRV_SQI_COMMAND_HANDLE_INVALID in the commandHandle argument under the
    following circumstances:
    - if the driver handle is invalid
    - if the transfer element is NULL or number of transfer elements is zero
    - if a buffer object could not be allocated to the request

  Remarks:
    None.
*/
void DRV_SQI_TransferData ( DRV_HANDLE handle, DRV_SQI_COMMAND_HANDLE *commandHandle,
                            uint8_t qspiDevice, DRV_SQI_TransferElement *xferData,
                            uint8_t numElements )
{
    uint32_t i,temp;
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_OBJECT *dObj = NULL;
    DRV_SQI_BUFFER_OBJECT *bObj = NULL;
    SYS_DMA_CHANNEL_OBJECT *dmaObj ;
    uint8_t *pQspiBuffer = (uint8_t *)QSPI_RW_BUFFER_ADDR ;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    /* - Validate the client handle */
    clientObj = _DRV_SQI_ValidateClientHandle(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Invalid Client handle.\n");
        return;
    }
    /* - Validate the transfer parameters */
    if ((xferData == NULL) || (numElements == 0))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Invalid transfer parameters.\n");
        return;
    }
    /* - Get driver object from Client object*/
    dObj = clientObj->driverObj;
    /* - Obtain the buffer object mutex */
    retVal = OSAL_MUTEX_Lock(&qspiBufObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Failed to acquire the SQI Buffer Object Mutex.\n");
        return;
    }
    /* Disable the Interrupt */
    if (DRV_SQI_INTERRUPT_MODE == true)
    {
        /* - Disable SQI Interrupts at core level */
        SYS_INT_SourceDisable(dObj->interruptSource);
    }
    /* Search for available Transfer Buffer object*/
    for (i=0;1<DRV_SQI_BUFFER_OBJECT_NUMBER;i++)
    {
        if (gDrvSQIBufferObject[i].inUse == false)
        {
            bObj = &gDrvSQIBufferObject[i];
            /* configure Buffer element */
            bObj->commandHandle = _DRV_SQI_MAKE_HANDLE(gDrvSQIBufferToken, i);
            *commandHandle      =  bObj->commandHandle;
            _DRV_SQI_UPDATE_BUF_TOKEN(gDrvSQIBufferToken);
            bObj->hClient       = (DRV_HANDLE)clientObj;
            bObj->inUse         = true;
            bObj->sqiId         = dObj->sqiId;
            bObj->status        = DRV_SQI_COMMAND_QUEUED;
            bObj->xferData      = xferData;
            bObj->xferNumber    = numElements;
            bObj->xferCnt       = 0;
            /* Update the token number. */
            _DRV_SQI_UPDATE_BUF_TOKEN(gDrvSQIBufferToken);
            break;
        }
    }
    if (bObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): No Buffer Object available.\n");
        OSAL_MUTEX_Unlock(&qspiBufObjMutex);
        if (DRV_SQI_INTERRUPT_MODE == true)
        {
            /* - Enable QSPI interrupts at core level */
            SYS_INT_SourceEnable(dObj->interruptSource);
        }
        return;
    }
    /* - Add Transfer buffer element to transfer Queue */
    QUEUE_Push(&dObj->queue,bObj);
    /* - Check if newly added transfer buffer Object is the first element in the transfer queue */
    if (dObj->queue.head == dObj->queue.tail)
    {
        /* Drive SQI_chip Select Low*/
        BSP_SQI_CSOff();
        /* Start QSPI transfer */ 
        _QSPI_Transfer_data(dObj,bObj);
        if (DRV_SQI_INTERRUPT_MODE == true)
        {
            /* - Enable QSPI interrupts at core level */
            SYS_INT_SourceEnable(dObj->interruptSource);
            /* - Enable QSPI interrupts at peripheral level */
            dObj->sqiId->QSPI_IER.w = (QSPI_IER_INSTRE_Msk | QSPI_IER_CSR_Msk | QSPI_IER_OVRES_Msk);
        }
        OSAL_MUTEX_Unlock(&qspiBufObjMutex);
    }
    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_TransferData(): Request queued successfully.");
}

// *****************************************************************************
/* Function:
    void DRV_SQI_EventHandlerSet ( const DRV_HANDLE handle, const void* eventHandler,
                                   const uintptr_t context)

  Summary:
    Allows a client to identify an event handling function for the driver to
    call back when queued operation has completed.

  Description:
    This function allows a client to identify an event handling function for
    the driver to call back when queued operation has completed. When a client
    calls read or a write function, it is provided with a handle identifying
    the command that was added to the driver's buffer queue.  The driver will
    pass this handle back to the client by calling "eventHandler" function when
    the queued operation has completed.

    The event handler should be set before the client performs any read or
    write operations that could generate events. The event handler once set,
    persists until the client closes the driver or sets another event handler
    (which could be a "NULL" pointer to indicate no callback).

  Remarks:
    If the client does not want to be notified when the queued operation has
    completed, it does not need to register a callback.
*/
void DRV_SQI_EventHandlerSet ( const DRV_HANDLE handle, const void* eventHandler,
                               const uintptr_t context)
{
    DRV_SQI_CLIENT_OBJECT *clientObj;

    /* - Check if the client handle is valid */
    clientObj = _DRV_SQI_ValidateClientHandle(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_EventHandlerSet(): Invalid driver handle.\n");
        return;
    }
    /* Set the event handler */
    clientObj->eventHandler = eventHandler;
    clientObj->context = context;
}

// *****************************************************************************
/* Function:
    DRV_SQI_COMMAND_STATUS DRV_SQI_CommandStatus ( const DRV_HANDLE handle,
                                                   const DRV_SQI_COMMAND_HANDLE commandHandle)

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the command. The application must
    use this routine where the status of a scheduled command needs to polled
    on. The function may return DRV_SQI_COMMAND_COMPLETED in a case where the
    command handle has expired. A command handle expires when the internal
    buffer object is re-assigned to another request. It is recommended that
    this function be called regularly in order to track the command status
    correctly.

    The application can alternatively register an event handler to receive the
    operation completion events.

  Remarks:
    The upper 16 bits of the command handle are the token and the lower 16
    bits are index into the gDrvQSPIBufferObject array
    Refer to drv_qspi.h for usage information.
*/
DRV_SQI_COMMAND_STATUS DRV_SQI_CommandStatus ( const DRV_HANDLE handle,
                                               const DRV_SQI_COMMAND_HANDLE commandHandle)
{
    uint16_t iEntry = 0;

    /* - Validate the client handle */
    if (_DRV_SQI_ValidateClientHandle(handle) == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_CommandStatus(): Invalid driver handle.\n");
        return DRV_SQI_COMMAND_ERROR_UNKNOWN;
    }
    /* - Validate Command handle */
    if (commandHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_CommandStatus(): Invalid command handle.\n");
        return DRV_SQI_COMMAND_ERROR_UNKNOWN;
    }
    /* - Get the index bit */
    iEntry = commandHandle & 0xFFFF;
    if (gDrvSQIBufferObject[iEntry].commandHandle != commandHandle)
    {
        /* This means that object has been re-used by another request. Indicate
         * that the operation is completed.  */
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SQI_CommandStatus(): Command handle is expired.\n");
        return (DRV_SQI_COMMAND_COMPLETED);
    }

    /* - Return the last known buffer object status */
    return (gDrvSQIBufferObject[iEntry].status);
}

// ****************************************************************************
/* Function:
    void DRV_SQI_Tasks ( SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's task state machine.

  Description:
    This routine is used to maintain the driver's internal task state machine.

  Remarks:
    This routine is to be called by the system's task routine(SYS_Tasks) or
    from the ISR.
*/
void DRV_SQI_Tasks ( SYS_MODULE_OBJ object )
{
    volatile uint32_t flags,temp ;
    DRV_SQI_OBJECT *dObj = NULL;
    QUEUE_OBJECT *qObj = NULL;
    DRV_SQI_BUFFER_OBJECT *bObj;
    DRV_SQI_CLIENT_OBJECT *clientObj = NULL;
    DRV_SQI_EVENT event = DRV_SQI_EVENT_COMMAND_ERROR;
    SYS_DMA_CHANNEL_OBJECT *dmaObj ;
    uint32_t *pQspiBuffer = (uint32_t *)QSPI_RW_BUFFER_ADDR;

    /* - Validate the Object index */
    if (object == SYS_MODULE_OBJ_INVALID)
    {
        return;
    }
    /* - Get Driver object according to driver index*/
    dObj = &gDrvSQIObj[object];
    /* - Get queue Object from Driver object*/
    qObj = &dObj->queue;
    if (qObj->numElements == 0)
    {
        return;
    }
    /* - Get current transfer Object from Driver queue*/
    bObj = qObj->head->data;
    /* - Get QSPI Status register content */
    flags = dObj->sqiId->QSPI_SR.w;
    
    /* - Check if QSPI Overrun Error appears */
    if (flags & QSPI_SR_OVRES_Msk)
    {
        event = DRV_SQI_COMMAND_ERROR_UNKNOWN;
        bObj->status = DRV_SQI_COMMAND_ERROR_UNKNOWN;
        clientObj->eventHandler(event, bObj->commandHandle, (void*)clientObj->context);
    }
        
    /* - Check if QSPI is ready to transmit */
    if ((flags & QSPI_SR_INSTRE_Msk))
    {  
        /* - Check if current transfer is finished */
        if ((bObj->xferData[bObj->xferCnt].length != 0) && (bObj->xferCnt < bObj->xferNumber))
        {
            /* Continue current buffer transfer*/
            _QSPI_Transfer_data(dObj,bObj);
        }
        else
        {
            /* Stop previous buffer transfer */
            dObj->sqiId->QSPI_CR.w |= QSPI_CR_LASTXFER_Msk;
            bObj->xferCnt ++;
            /* Check if buffer transfer is pending  */
            if (bObj->xferCnt < bObj->xferNumber)
            {
                /* Start New buffer transfer */
                _QSPI_Transfer_data(dObj,bObj);
            }
            else
            {
                /* No transfer pending : finish the transfer */
                clientObj = (DRV_SQI_CLIENT_OBJECT *)bObj->hClient;
                /* - Drive SQI chip select high*/
                BSP_SQI_CSOn();
                /* - set QSPI command as completed */
                event = DRV_SQI_EVENT_COMMAND_COMPLETE;
                bObj->status = DRV_SQI_COMMAND_COMPLETED;
                /* - Invoke the Buffer Client callback */
                if (clientObj->eventHandler != NULL)
                {
                    clientObj->eventHandler(event, bObj->commandHandle, (void*)clientObj->context);
                }
                bObj->inUse = false;
                /* remove current buffer from the Queue */
                QUEUE_Pop(&dObj->queue);
                if (dObj->queue.numElements != 0)
                {
                    /* - Prepare next buffer transfer */
                    bObj = qObj->head->data;
                }
                else
                {
                    if (DRV_SQI_INTERRUPT_MODE == true)
                    {
                        /* - Stop QSPI interrupt */
                        SYS_INT_SourceDisable(dObj->interruptSource);
                    }
                }
            }
        }
    }
}

