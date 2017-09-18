/*******************************************************************************
  DAC Driver Dynamic implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_dac_pic32c.c

  Summary:
    Source code for the DAC driver dynamic implementation.

  Description:
    This file contains the source code for the dynamic implementation of the
    DAC driver.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
*******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "drv_dac_local_pic32c.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array */
DRV_DAC_OBJ gDrvDACObj[DRV_DAC_CHANNEL_INSTANCES_NUMBER] ;

/* This is the driver client object array */
DRV_DAC_CH_OBJ gDrvDACChObj[DRV_DAC_CHANNEL_INSTANCES_NUMBER] ;

// *****************************************************************************
// *****************************************************************************
// Section: DAC Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_DAC_Initialize
    (
        const SYS_MODULE_INDEX drvIndex,
        const SYS_MODULE_INIT * const init
    )

  Summary:
    Dynamic implementation of DRV_DAC_Initialize system interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_Initialize system interface
    function.

  Remarks:
    See drv_dac.h for usage information.
*/

SYS_MODULE_OBJ DRV_DAC_Initialize(const SYS_MODULE_INDEX drvIndex, const SYS_MODULE_INIT * const init)
{
	DRV_DAC_OBJ *dObj = (DRV_DAC_OBJ*)NULL;
    DRV_DAC_CH_OBJ *chObj = (DRV_DAC_CH_OBJ*)NULL;
    DRV_DAC_INIT *dacInit;
	
	if(drvIndex >= DRV_DAC_CHANNEL_INSTANCES_NUMBER)
    {   
        SYS_DEBUG(0, "Invalid driver index");
        return SYS_MODULE_OBJ_INVALID;
    }
	
	if(gDrvDACObj[drvIndex].inUse != false)
    {
        SYS_DEBUG(0, "Instance already in use");
        return SYS_MODULE_OBJ_INVALID;
    }
	
    dacInit = ( DRV_DAC_INIT *)init;

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = &gDrvDACObj[drvIndex];
    
    /* Object is valid, set it in use */
    dObj->inUse = true;
    
    /* Set status to Ready */
    dObj->status = SYS_STATUS_READY;
    
    /* Get DAC module Id */
	dObj->moduleId = (dacc_registers_t *)dacInit[0].dacID;
    
    /* Set DAC output to either Single-ended or Differential mode */
    dObj->outputMode = dacInit[0].outputMode;
    
    /* Set if Interrupt enabled or not */
	dObj->interruptMode = dacInit[0].interruptMode;
    
    /* Reset DAC registers */
	dObj->moduleId->DACC_CR.w = DACC_CR_SWRST_Msk;
    
    /* Set DAC Prescaler value */
	dObj->moduleId->DACC_MR.w |= ((dObj->moduleId->DACC_MR.w & (~DACC_MR_PRESCALER_Msk)) | ((DRV_DAC_PRESCALER_VALUE << DACC_MR_PRESCALER_Pos) & DACC_MR_PRESCALER_Msk));
    
    /* Half word transfer mode */
	dObj->moduleId->DACC_MR.w &= ~DACC_MR_WORD_Msk;
    
    if(DRV_DAC_OUTPUT_MODE_DIFFERENTIAL == dacInit[0].outputMode)
    {
	    dObj->moduleId->DACC_MR.w |= DACC_MR_DIFF_Msk;
    }
    else
    {
        dObj->moduleId->DACC_MR.w &= ~DACC_MR_DIFF_Msk;
    }
	
    if ((2 == DRV_DAC_CHANNEL_INSTANCES_NUMBER))
	{
		dObj->moduleId->DACC_CHER.w |= (DACC_CHER_CH0_Msk | DACC_CHER_CH1_Msk) ;
        /* Waits for the DAC start up time */
        while(!dObj->moduleId->DACC_CHSR.DACRDY0 &&  !dObj->moduleId->DACC_CHSR.DACRDY1);
    }

	for (uint8_t i = 0; i< DRV_DAC_CHANNEL_INSTANCES_NUMBER; i++)
	{
        chObj = &gDrvDACChObj[i];
        
        chObj->dObj = dObj;
        chObj->inUse = true;
        chObj->chId = (DRV_DAC_CHANNEL_INDEX)i;
        chObj->event = DRV_DAC_EVENT_NONE;
        chObj->eventHandler = NULL;
        
		_DRV_DAC_HardwareSetup(dObj->moduleId, &dacInit[i]);
	}
    
    if(true == dObj->interruptMode)
    {
        SYS_INT_SourceEnable(DACC_IRQn);
    }
    
	return ((SYS_MODULE_OBJ)drvIndex);
}

// *****************************************************************************
/* Function:
    void DRV_DAC_Deinitialize(SYS_MODULE_OBJ object)

  Summary:
    Dynamic implementation of DRV_DAC_Deinitialize system interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_Deinitialize system interface
    function.

  Remarks:
  
    See drv_dac.h for usage information.
*/

void  DRV_DAC_Deinitialize(SYS_MODULE_OBJ object)
{
    DRV_DAC_OBJ * dObj;
    DRV_DAC_CH_OBJ *chObj;

    /* Check that the object is valid */

    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "Invalid system object handle");
        return;
    }

    if(object >= DRV_DAC_CHANNEL_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "Invalid system object handle");
        return;
    }

    dObj = (DRV_DAC_OBJ*) &gDrvDACObj[object];

    if(!dObj->inUse)
    {
        SYS_DEBUG(0, "Invalid system object handle");
        return;
    }

    /* The driver will not have clients when it is being deinitialized. 
       So the order in which we do the following steps is not that important */

    /* Indicate that this object is not is use */
    dObj->inUse = false;

    /* Deinitialize the DAC status */
    dObj->status =  SYS_STATUS_UNINITIALIZED ;

    for (uint8_t i = 0; i< DRV_DAC_CHANNEL_INSTANCES_NUMBER; i++)
	{
        chObj = &gDrvDACChObj[i];
        
        chObj->inUse = false;
    }
    
    /* Resets the DACC simulating a hardware reset */
	dObj->moduleId->DACC_CR.w = DACC_CR_SWRST_Msk;
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_DAC_Status(SYS_MODULE_OBJ object)

  Summary:
    Dynamic implementation of DRV_DAC_Status system interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_Status system interface
    function.

  Remarks:
    See drv_dac.h for usage information.
*/

SYS_STATUS DRV_DAC_Status(SYS_MODULE_OBJ object)
{
    /* Check if we have a valid object */
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "Invalid system object handle");
        return(SYS_STATUS_UNINITIALIZED);
    }

    if(object > DRV_DAC_CHANNEL_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "Invalid system object handle");
        return(SYS_STATUS_UNINITIALIZED);
    }

    /* Return the system status of the hardware instance object */
    return (gDrvDACObj[object].status);
}

// *****************************************************************************
/* Function:
    void DRV_DAC_Tasks(SYS_MODULE_OBJ object)

  Summary:
    Dynamic implementation of DRV_DAC_Tasks system interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_Tasks system interface
    function.

  Remarks:
    See drv_dac.h for usage information.
*/

void DRV_DAC_Tasks(SYS_MODULE_OBJ object)
{
    DRV_DAC_OBJ * dObj = &gDrvDACObj[object];
    DRV_DAC_CH_OBJ *chObj = NULL;
    uint32_t channelStatus = false;
	
	if((!dObj->inUse) || (dObj->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't do anything */
        return;
    }

    channelStatus = dObj->moduleId->DACC_ISR.w;
    
	for (uint8_t i = 0; i< DRV_DAC_CHANNEL_INSTANCES_NUMBER; i++)
	{	
        dObj->moduleId->DACC_IDR.w = (DACC_IER_TXRDY0_Msk<<i);
        
        chObj = &gDrvDACChObj[i];
        
        if( chObj->inUse == true)
        {
            if (((channelStatus>>i) & 1) && (chObj->event == DRV_DAC_EVENT_PROCESSING))
            {
                chObj->event = DRV_DAC_EVENT_READY;
                
                if(NULL != chObj->eventHandler)
                {
                     chObj->eventHandler((DRV_HANDLE)chObj, chObj->chId, chObj->event, chObj->context);
                }
            }
        }
	}
    
    return;
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_DAC_Open(const SYS_MODULE_INDEX drvIndex, 
                            const DRV_IO_INTENT ioIntent)

  Summary:
    Dynamic implementation of DRV_DAC_Open client interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_Open client interface
    function.

  Remarks:
    See drv_dac.h for usage information.
*/

DRV_HANDLE DRV_DAC_Open(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT ioIntent)
{
    DRV_DAC_OBJ *dObj;
   
    dObj = &gDrvDACObj[drvIndex];
    
    if((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false))
    {
        /* The DAC module should be ready */
        SYS_DEBUG(0, "Was the driver initialized?");
        return DRV_HANDLE_INVALID;
    }
   
    return (DRV_HANDLE)drvIndex;
}

// *****************************************************************************
/* Function:
    void DRV_DAC_Close(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_DAC_Close client interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_Close client interface
    function.

  Remarks:
    See drv_dac.h for usage information.
*/
void DRV_DAC_Close(DRV_HANDLE handle)
{
    if(handle == DRV_HANDLE_INVALID)
    {
        /* The DAC module should be ready */
        SYS_DEBUG(0, "Was the driver opened?");
        return;
    }

    return;    
}

// *****************************************************************************
/* Function:
    DRV_CLIENT_STATUS DRV_DAC_ClientStatus(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of DRV_DAC_clinetStatus client interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_clientStatus client interface
    function.

  Remarks:
    See drv_dac.h for usage information.
*/

DRV_CLIENT_STATUS DRV_DAC_ClientStatus(DRV_HANDLE handle)
{
    if( handle == DRV_HANDLE_INVALID)
    {
        /* The DAC module should be ready */
        SYS_DEBUG(0, "Was the driver opened?");
        return DRV_CLIENT_STATUS_CLOSED;
    }

    return DRV_CLIENT_STATUS_READY;
}

// *****************************************************************************
/* Function:
    void DRV_DAC_DataWrite(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId, 
                           uint32_t data)

  Summary:
    Dynamic implementation of DRV_DAC_DataWrite client interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_DataWrite client interface
    function.

  Remarks:
    See drv_dac.h for usage information.
*/

void DRV_DAC_DataWrite(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId, uint32_t data)
{
    DRV_DAC_OBJ *dObj;
    DRV_DAC_CH_OBJ *chObj;

    if(handle == DRV_HANDLE_INVALID)
    {
        /* The DAC module should be ready */
        SYS_DEBUG(0, "Was the driver opened?");
        return;
    }
    
    if(dObj->outputMode == DRV_DAC_OUTPUT_MODE_DIFFERENTIAL && (chId == DRV_DAC_CHANNEL_INDEX_1))
    {
        /* In Differential mode, all operations are driven by Channel 0*/
        SYS_DEBUG(0, "In Differential mode all operations are driven by Channel 0");
        return;
    }
    
    if (gDrvDACChObj[chId].inUse != true)
    {
	    SYS_DEBUG(0, "Channel is not selected");
        return;
    }
    
    chObj = &gDrvDACChObj[chId];

    chObj->event = DRV_DAC_EVENT_PROCESSING;
    
    dObj = chObj->dObj;
  
    dObj->moduleId->DACC_CDR[chId].w = data;
    
    if (dObj->interruptMode)
	{
		dObj->moduleId->DACC_IER.w = DACC_IER_TXRDY0_Msk << chId;
	}
}

// *****************************************************************************
/* Function:
    void DRV_DAC_EventHandlerSet(DRV_HANDLE handle, 
                                 DRV_DAC_CHANNEL_INDEX chId, 
                                 const void* eventHandler, 
                                 const uintptr_t context)

  Summary:
    Dynamic implementation of DRV_DAC_EventHandlerSet client interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_EventHandlerSet client 
    interface function.

  Remarks:
    See drv_dac.h for usage information.
*/
void DRV_DAC_EventHandlerSet(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId, const void* eventHandler, const uintptr_t context)
{
    DRV_DAC_OBJ *dObj;
    DRV_DAC_CH_OBJ *chObj;

    if( handle == DRV_HANDLE_INVALID)
    {
        /* The DAC module should be ready */
        SYS_DEBUG(0, "Was the driver opened?");
        return;
    }
    
    if(dObj->outputMode == DRV_DAC_OUTPUT_MODE_DIFFERENTIAL && (chId == DRV_DAC_CHANNEL_INDEX_1))
    {
        /* In Differential mode, all operations are driven by Channel 0*/
        SYS_DEBUG(0, "In Differential mode all operations are driven by Channel 0");
        return;
    }
    
    if (gDrvDACChObj[chId].inUse != true)
    {
	    SYS_DEBUG(0, "Channel is not selected");
        return;
    }
    
    chObj = &gDrvDACChObj[chId];
    
    if(chObj->inUse == true)
    {
	    chObj->eventHandler = eventHandler; 
        chObj->context = context;
    }
    
    return;
}

// *****************************************************************************
/* Function:
    void DRV_DAC_EventStatusGet(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId)

  Summary:
    Dynamic implementation of DRV_DAC_EventStatusGet client interface function.

  Description:
    This is the dynamic implementation of DRV_DAC_EventStatusGet client 
    interface function.

  Remarks:
    See drv_dac.h for usage information.
*/
DRV_DAC_EVENT DRV_DAC_EventStatusGet(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId)
{
    DRV_DAC_CH_OBJ *chObj;

    if(handle == DRV_HANDLE_INVALID)
    {
        /* The DAC module should be ready */
        SYS_DEBUG(0, "Was the driver opened?");
        return DRV_DAC_EVENT_NONE;
    }
    
    chObj = &gDrvDACChObj[chId];
    
    return chObj->event;
}

// *****************************************************************************
// *****************************************************************************
// Section: Local functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    _DRV_DAC_HardwareSetup(dacc_registers_t *moduleId, DRV_DAC_INIT * dacInit)

  Summary:
    Dynamic implementation of _DRV_DAC_HardwareSetup local interface function.

  Description:
    This is the dynamic implementation of _DRV_DAC_HardwareSetup local interface
    function.

  Remarks:
    None
*/
void _DRV_DAC_HardwareSetup(dacc_registers_t *moduleId, DRV_DAC_INIT * dacInit)
{
    if (dacInit->dacIndex == DRV_DAC_CHANNEL_INDEX_0)
	{
        if (!moduleId->DACC_CHSR.DACRDY0)
        {
            moduleId->DACC_CHER.w |= DACC_CHER_CH0_Msk ;
            /* Waits for the DAC start up time */
            while(!moduleId->DACC_CHSR.DACRDY0);
        }
		
        /* Set Slew rate of the analog output */
        moduleId->DACC_ACR.w |= ((dacInit->dacSpeed << DACC_ACR_IBCTLCH0_Pos) & DACC_ACR_IBCTLCH0_Msk);
        
        /* Select DAC conversion mode */
        switch(dacInit->operationMode)
        {
            case DRV_DAC_OPERATION_TRIGGER_MODE:
            {
                moduleId->DACC_TRIGR.w |= DACC_TRIGR_TRGEN0_Msk;
                
                /* Set Trigger Source of DAC Channel */
                moduleId->DACC_TRIGR.w |= ((dacInit->triggerSource << DACC_TRIGR_TRGSEL0_Pos) & DACC_TRIGR_TRGSEL0_Msk);
                
                /* Set Oversampling Ratio of DAC Channel */
                if (dacInit->overSampleRatio != DRV_DAC_OSR_1)
                {
                    moduleId->DACC_TRIGR.w |= ((dacInit->overSampleRatio << DACC_TRIGR_OSR0_Pos) & DACC_TRIGR_OSR0_Msk);
                } 
                break;
            }
            case DRV_DAC_OPERATION_FREE_RUNNING_MODE:
            {
                moduleId->DACC_TRIGR.w &= (~(DACC_TRIGR_TRGEN0_Msk));
			    moduleId->DACC_MR.w &= (~(DACC_MR_MAXS0_Msk));
                break;
            }
            case DRV_DAC_OPERATION_MAX_SPEED_MODE:
            {
                moduleId->DACC_TRIGR.w &= (~(DACC_TRIGR_TRGEN0_Msk));
			    moduleId->DACC_MR.w |= DACC_MR_MAXS0_Msk;
                break;
            }
            
            default:
                break;
        }
	}
    
	else if ((dacInit->dacIndex == DRV_DAC_CHANNEL_INDEX_1) && (dacInit->outputMode == DRV_DAC_OUTPUT_MODE_SINGLE_ENDED))
	{
		if (!moduleId->DACC_CHSR.DACRDY1)
        {
            moduleId->DACC_CHER.w |= DACC_CHER_CH1_Msk ;
            /* Waits for the DAC start up time */
            while(!moduleId->DACC_CHSR.DACRDY1);
        }
        
        /* Set Slew rate of the analog output */
        moduleId->DACC_ACR.w |= ((dacInit->dacSpeed << DACC_ACR_IBCTLCH1_Pos) & DACC_ACR_IBCTLCH1_Msk);
        
        /* Select DAC conversion mode */
        switch(dacInit->operationMode)
        {
            case DRV_DAC_OPERATION_TRIGGER_MODE:
            {
                moduleId->DACC_TRIGR.w |= DACC_TRIGR_TRGEN1_Msk;
                
                /* Set Trigger Source of DAC Channel */
                moduleId->DACC_TRIGR.w |= ((dacInit->triggerSource << DACC_TRIGR_TRGSEL1_Pos) & DACC_TRIGR_TRGSEL1_Msk);
                
                /* Set Oversampling Ratio of DAC Channel */
                if (dacInit->overSampleRatio != DRV_DAC_OSR_1)
                {
                    moduleId->DACC_TRIGR.w |= ((dacInit->overSampleRatio << DACC_TRIGR_OSR1_Pos) & DACC_TRIGR_OSR1_Msk);
                } 
                break;
            }
            case DRV_DAC_OPERATION_FREE_RUNNING_MODE:
            {
                moduleId->DACC_TRIGR.w &= (~(DACC_TRIGR_TRGEN1_Msk));
			    moduleId->DACC_MR.w &= (~(DACC_MR_MAXS1_Msk));
                break;
            }
            case DRV_DAC_OPERATION_MAX_SPEED_MODE:
            {
                moduleId->DACC_TRIGR.w &= (~(DACC_TRIGR_TRGEN1_Msk));
			    moduleId->DACC_MR.w |= DACC_MR_MAXS1_Msk;
                break;
            }
            
            default:
                break;
        }
	}
    
}

/*******************************************************************************
 End of File
*/