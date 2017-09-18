/*******************************************************************************
  AFEC Device Driver dynamic implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_afec.c

  Summary:
    AFEC device driver dynamic implementation.

  Description:
    The AFEC device driver provides an interface to manage the AFEC 
    modules on Microchip PIC32C microcontrollers.  This file implements the core 
    interface routines for the AFEC driver in dynamic mode. 
    
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#include <stddef.h>
#include <string.h>
#include "driver/afec/src/drv_afec_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array. */
DRV_AFEC_OBJ gDrvAFECObj[DRV_AFEC_INSTANCES_NUMBER] ;

/* This is the client object array. */
DRV_AFEC_CLIENT_OBJ gDrvAFECClientObj[DRV_AFEC_CLIENTS_NUMBER];

extern DRV_AFEC_CHANNEL_SET gAFEC0ChannelSet[];
extern DRV_AFEC_CHANNEL_SET gAFEC1ChannelSet[];

/* void _DRV_AFEC_HardwareSetup
   (
        void *moduleId,
        DRV_AFEC_INIT * init
   )

  Summary:
    Initializes the AFEC module based on the init data structure.

  Description:
    This function initializes the AFEC module based on the init data structure.

  Remarks:
    This is a private function and should not be called directly by the
    application.
*/

void _DRV_AFEC_HardwareSetup(afec_registers_t * moduleId, DRV_AFEC_INIT * afecInit)
{
    uint32_t configData = 0;

    /* Reset AFEC registers */
    moduleId->AFEC_CR.SWRST = 1;
    
    /* If triggerMode is set to hardware, then enable hardware trigger */
    if (DRV_AFEC_TRIGGER_MODE_HARDWARE == afecInit->triggerMode)
    {
        configData |= AFEC_MR_TRGEN_EN;
    }
    
    if (DRV_AFEC_CONVERSION_SEQUENCE_TYPE_USER_SEQUENCE == afecInit->conversionSequenceType)
    {
        configData |= AFEC_MR_USEQ_Msk;
    }
    
    if (true == afecInit->sleepMode)
    {
        configData |= AFEC_MR_SLEEP_Msk;
    }
    
    if (true == afecInit->fastWake)
    {
        configData |= AFEC_MR_FWUP_Msk;
    }
    if (true == afecInit->conversionSequenceType)
    {
        configData |= AFEC_MR_USEQ_Msk;
        moduleId->AFEC_SEQ1R.w = afecInit->sequence1;
        moduleId->AFEC_SEQ2R.w = afecInit->sequence2;
    }

    /* Configuration data for AFEC_MR */
    configData |= AFEC_MR_PRESCAL(afecInit->clockPrescaler) |
                    AFEC_MR_STARTUP(afecInit->startupTime) |
                    AFEC_MR_TRGSEL(afecInit->trigSource) |
                    AFEC_MR_ONE_Msk |
                    AFEC_MR_TRACKTIM(15) | AFEC_MR_TRANSFER(2);
    moduleId->AFEC_MR.w = configData;
    
    configData = 0;
    
    if (DRV_AFEC_RESOLUTION_12_BIT != afecInit->resolution)
    {
        configData |= AFEC_EMR_STM_Msk;
    }
    
    if (DRV_AFEC_COMPARE_ALL_CHANNNELS == afecInit->compareType)
    {
        configData |= AFEC_EMR_CMPALL_Msk;
    }
    if (true == afecInit->tagControl)
    {
        configData |= AFEC_EMR_TAG_Pos;
    }
    configData |= AFEC_EMR_CMPFILTER(afecInit->compareFilter);
    /* Configuration for AFEC_EMR */
    configData |= AFEC_EMR_SIGNMODE(afecInit->signMode) |
                    AFEC_EMR_RES(afecInit->resolution) |
                    AFEC_EMR_CMPSEL(afecInit->compareAfecChannel) |
                    AFEC_EMR_CMPMODE(afecInit->compareMode);

    moduleId->AFEC_EMR.w = configData;
    
    configData = 0;
    /* Configuration for compare window */
    configData = AFEC_CWR_HIGHTHRES(afecInit->compareThresholdHigh) |
                    AFEC_CWR_LOWTHRES(afecInit->compareThresholdHigh);
    
    moduleId->AFEC_CWR.w = configData;
    
    configData = 0;
    /* Configure BIAS current - IBCTL
     * Enable PGA for both sample and hold units
     */
    configData = AFEC_ACR_PGA0EN_Msk | AFEC_ACR_PGA1EN_Msk |
                    AFEC_ACR_IBCTL(afecInit->biasCurrent);
    moduleId->AFEC_ACR.w = configData;
    
    configData = 0;
    /* Configuration for differential mode */
    configData = afecInit->differentialModeChannels;
    moduleId->AFEC_DIFFR.w = configData;
    
    configData = 0;
    /* Configuration for dual sampling */
    configData = afecInit->dualSamplingChannels;
    moduleId->AFEC_SHMR.w = configData;
}

SYS_MODULE_OBJ DRV_AFEC_Initialize
(
    const SYS_MODULE_INDEX drvIndex, 
    const SYS_MODULE_INIT * const init
)
{
    SYS_MODULE_OBJ sysObj = SYS_MODULE_OBJ_INVALID;
    DRV_AFEC_OBJ *dObj = NULL;
    DRV_AFEC_INIT *afecInit = NULL ;
    uint8_t channeSetCount = 0;
    uint8_t channelCount = 0;
    DRV_AFEC_CHANNEL_SET * channelSetArray = 0;
    uint8_t totalChannelsets = 0;
    
    /* Check if the specified driver index is in valid range */
    if(drvIndex >= DRV_AFEC_INSTANCES_NUMBER)
    {
        sysObj = SYS_MODULE_OBJ_INVALID;
    }
    /* Check if this hardware instance is already initialized */
    else if(gDrvAFECObj[drvIndex].inUse != false)
    {
        sysObj = SYS_MODULE_OBJ_INVALID;
    }
    /* If no error condition is detected, initialize the drive */    
    else
    {
        /* Assign the init data passed to a local pointer  */
        afecInit = ( DRV_AFEC_INIT * ) init ;

        /* Allocate the driver object and set the operation flag to be inUse */
        dObj = &gDrvAFECObj[drvIndex];
        dObj->inUse             = true;
        dObj->moduleId          = (afec_registers_t *)afecInit->afecID;
        dObj->interruptSource   =  afecInit->interruptSource;
        dObj->channelSetTable   = afecInit->channelSetTable;
        
        /* Copy channelset details to a locally */
        channelSetArray = afecInit->channelSetTable;
        totalChannelsets = afecInit->channelSetTableSize;

        /* Setup the Hardware */
        _DRV_AFEC_HardwareSetup(dObj->moduleId, afecInit ) ;

        /* Form the channelset map from gAFECChannelSet
         * Apply analog offset default value for all enabled channels
         */
        for(channelCount = 0; channelCount < 11; channelCount++)
        {
            /* Set all values to 0xff to indicate that the channel is not
             * part of any channelset.
             * This also means that the channel is not enabled via MHC
             */
            dObj->channelSetMap[channelCount] = 0xff;
        }
              
        for (channeSetCount=0; channeSetCount < totalChannelsets; channeSetCount++)
        {
            /* Value of a channel set will always be non zero */
            if(channelSetArray[channeSetCount])
            {
                for(channelCount = 0; channelCount < DRV_AFEC_NUMBER_OF_CHANNELS; channelCount++)
                {
                    if(channelSetArray[channeSetCount] & (1 << channelCount))
                    {
                        /* Set analog offset to mid point for all
                         * enabled channels
                         */
                        dObj->moduleId->AFEC_CSELR.w = AFEC_CSELR_CSEL(channelCount);
                        dObj->moduleId->AFEC_COCR.w = AFEC_COCR_AOFF(afecInit->channelOffset[channelCount]);
                        dObj->moduleId->AFEC_CGR.w |= ((afecInit->channelGain[channelCount]) << (channelCount*2));
                        dObj->channelSetMap[channelCount] = channeSetCount;
                    }
                    else
                    {
                        /* Do nothing */
                    }
                }
            }
        }
        /* Update the status */
        dObj->status = SYS_STATUS_READY;
        sysObj = (SYS_MODULE_OBJ)drvIndex;
    }

    /* Return the object structure */
    return (sysObj);
}

DRV_HANDLE DRV_AFEC_Open( const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT ioIntent)
{
    DRV_HANDLE returnValue = DRV_HANDLE_INVALID;
    DRV_AFEC_CLIENT_OBJ *clientObj;
    DRV_AFEC_OBJ *dObj;
    uint8_t idx = 0;
    
    if (drvIndex >= DRV_AFEC_INSTANCES_NUMBER)
    {
        returnValue = DRV_HANDLE_INVALID;
    }
    else
    {
        /* Take the driver object at the given index */
        dObj = &gDrvAFECObj[drvIndex];

        if((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false))
        {
            return DRV_HANDLE_INVALID;
        }

        /* Find a client object with .inUse=false */
        for(idx = 0; idx < DRV_AFEC_INSTANCES_NUMBER; idx++)
        {
            if(!(gDrvAFECClientObj[idx].inUse))
            {
                clientObj = &gDrvAFECClientObj[idx];
                clientObj->afecDrvIndex = drvIndex;
                returnValue = drvIndex;
            }
        }
    }
    
    return returnValue;
}

DRV_AFEC_RESULT DRV_AFEC_SamplesGet 
(
    DRV_HANDLE handle,
    int channelSetIdx,
    void * samples,
    size_t bufferSize
)
{
    DRV_AFEC_CLIENT_OBJ *clientObj;
    DRV_AFEC_OBJ *dObj;
    uint16_t channelStatus = 0;
    uint16_t tempData = 0;
    uint8_t loopCount = 0;
    uint8_t bufIdx = 0;
    uint8_t channelPos = 0;
    DRV_AFEC_RESULT afecResult = DRV_AFEC_RESULT_FAILURE;
    
    int16_t channelSet = 0;
    
    if(DRV_HANDLE_INVALID == handle)
    {
        afecResult = DRV_AFEC_RESULT_HANDLE_INVALID;
    }
    else if((NULL == samples) || (0 == bufferSize))
    {
        afecResult = DRV_AFEC_RESULT_PARAMETER_INVALID;
    }
    else
    {
        clientObj = &gDrvAFECClientObj[handle];

        /* Take the driver object for this client */
        dObj = &gDrvAFECObj[clientObj->afecDrvIndex];
        
        channelSet = dObj->channelSetTable[channelSetIdx];
        afecResult = DRV_AFEC_RESULT_SUCCESS;
        
        /* Clear all channel selection and enable only the requested channels */
        dObj->moduleId->AFEC_CHDR.w = 0x00000fff;
        dObj->moduleId->AFEC_CHER.w = channelSet;

        /* Each sample can be upto 16-bits. Thus total sample that can be stored is
         * (bufferSize/2)
         * If there is only one channel enabled, then the loop has to repeat
         * (bufferSize/2) times. If there are more number of channels enabled,
         *  then value of bufIdx decides when to exit the loop
         */
        for(loopCount = 0; loopCount < (bufferSize/2); loopCount++)
        {
            /* If the buffer is full, exit the loop
             * This check is not relevant in the first iteration of the loop
             * as bufIdx will be zero.
             * When there are more than one channel in the channelset, it is
             * possible that bufIdx is greater than loopCount. 
             */
            if( bufIdx >= (bufferSize/2))
            {
                break;
            }
            /* Start conversion on the given AFEC instance*/
            dObj->moduleId->AFEC_CR.START = 1;

            /* Wait till all channels are converted
             * This is to ensure that the following 'for()' loop reads all
             * the channels in the channel set, in each of its iterations.
             */
            while((dObj->moduleId->AFEC_ISR.w &  channelSet) != channelSet);

            for(channelPos = 0; channelPos <= 11; channelPos++)
            {
                channelStatus = dObj->moduleId->AFEC_ISR.w;
                /* If flag for a articular channel is set, read that into buffer */
                if((1<<channelPos) & channelStatus)
                {
                    /* If the buffer is not full, read data into buffer
                     * Else read to a temporary variable to clear the EOCx flag 
                     */
                    if( bufIdx < (bufferSize/2))
                    {
                        dObj->moduleId->AFEC_CSELR.w = AFEC_CSELR_CSEL(channelPos);
                        ((uint16_t *)samples)[bufIdx++] = (uint16_t) dObj->moduleId->AFEC_CDR.w;
                    }
                    else
                    {
                        /* Code comes here if the size of the 'samples' buffer
                         * is not enough to hold data from all enabled channels
                         */
                        dObj->moduleId->AFEC_CSELR.w = AFEC_CSELR_CSEL(channelPos);
                        tempData = (uint16_t) dObj->moduleId->AFEC_CDR.w;
                    }
                }
            }
        }
    }
    return afecResult;
}

