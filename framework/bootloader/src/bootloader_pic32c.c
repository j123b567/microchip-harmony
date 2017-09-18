/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    bootloader_pic32c.c
    
  Summary:
    Interface for the Bootloader library.

  Description:
    This file contains the interface definition for the Bootloader library.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#include "bootloader/src/bootloader.h"
#include "system/reset/sys_reset.h"

#if(SYS_FS_MAX_FILES > 0)
#define FILE_BUFFER_SIZE  512
uint8_t fileBuffer[FILE_BUFFER_SIZE] __attribute__((aligned(16)));
#endif

BOOTLOADER_DATA bootloaderData __attribute__((aligned(16)));
BOOTLOADER_BUFFER data_buff __attribute__((aligned(16)));

void Bootloader_BufferEventHandler(DATASTREAM_BUFFER_EVENT buffEvent,
                            DATASTREAM_BUFFER_HANDLE hBufferEvent,
                            uint16_t context );

/********************************************************************
* Function:     Enter_Application()
*
* Precondition:
*
* Input:        None.
*
* Output:
*
* Side Effects: No return from here.
*
* Overview:     The function will re-base the vector table to applications reset
*               address and jumps to reset handler of the application.
*
* Note:
********************************************************************/
static void Enter_Application(void)
{
    SYS_INT_Disable();

	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *) APP_RESET_ADDRESS);

	/* Rebase the vector table base address */
	SCB->VTOR = ((uint32_t) APP_RESET_ADDRESS & SCB_VTOR_TBLOFF_Msk);

    void (*fptr)(void);

    /* Set the Reset_Handler address of the application */
    fptr = (void (*)(void))(*(uint32_t *)(APP_RESET_ADDRESS + 4));
    fptr();
}

/**
 * Static table used for the table_driven implementation.
 *****************************************************************************/
static const uint16_t crc_table[16] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};

/********************************************************************
* Function:     CalculateCrc()
*
* Precondition:
*
* Input:        Data pointer and data length
*
* Output:       CRC.
*
* Side Effects: None.
*
* Overview:     Calculates CRC for the given data and len
*
*
* Note:         None.
********************************************************************/
uint32_t APP_CalculateCrc(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint16_t crc = 0;
    
    while(len--)
    {
        i = (crc >> 12) ^ (*data >> 4);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        i = (crc >> 12) ^ (*data >> 0);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        data++;
    }

    return (crc & 0xFFFF);
}
/******************************************************************************
  Function:
    SYS_MODULE_OBJ Bootloader_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                              const SYS_MODULE_INIT    * const moduleInit)

  Summary:
    Initializes primitive data structures for the general features
    of the primitive layer.

  Description:
    Initializes external and internal data structure for the general
    features of the primitive layer.

    This function must be called at system initialization.

  Remarks:
    None.
*/
void Bootloader_Initialize ( const BOOTLOADER_INIT *drvBootloaderInit )
{
    /* Place the App state machine in it's initial state. */
    bootloaderData.currentState = BOOTLOADER_CHECK_FOR_TRIGGER;
    bootloaderData.cmdBufferLength = 0;
    bootloaderData.streamHandle = DRV_HANDLE_INVALID;
    bootloaderData.datastreamStatus = DRV_CLIENT_STATUS_ERROR;
    bootloaderData.usrBufferEventComplete = false;

    bootloaderData.data = &data_buff;
    bootloaderData.type = drvBootloaderInit->drvType;
    
    bootloaderData.FlashEraseFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.StartAppFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.BlankCheckFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.ProgramCompleteFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.ForceBootloadFunc = (BOOTLOADER_CALLBACK)NULL;
    bootloaderData.softReset = (SYS_RESET_ReasonGet() & RESET_REASON_SOFTWARE) == RESET_REASON_SOFTWARE;
    SYS_RESET_ReasonClear(RESET_REASON_SOFTWARE);
}

void BOOTLOADER_FlashEraseRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.FlashEraseFunc = newFunc;
}

void BOOTLOADER_StartAppRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.StartAppFunc = newFunc;
}

void BOOTLOADER_BlankCheckRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.BlankCheckFunc = newFunc;
}

void BOOTLOADER_ProgramCompleteRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.ProgramCompleteFunc = newFunc;
}

void BOOTLOADER_ForceBootloadRegister(BOOTLOADER_CALLBACK newFunc)
{
    bootloaderData.ForceBootloadFunc = newFunc;
}

// *****************************************************************************
/* Function:
    void Bootloader_Tasks (SYS_MODULE_INDEX index);

  Summary:
    Maintains the Bootloader module state machine. It manages the Bootloader Module object list
    items and responds to Bootloader Module primitive events.

*/
void Bootloader_Tasks ()
{
    size_t BuffLen=0;
    uint16_t crc;
    unsigned int i;
    uint32_t status = 0;

    /* Check the application state*/
    switch ( bootloaderData.currentState )
    {
        case BOOTLOADER_CHECK_FOR_TRIGGER:
        {
            bool forceBootloadMode = false;

            if (bootloaderData.ForceBootloadFunc != NULL)
            {
                forceBootloadMode = (1 == bootloaderData.ForceBootloadFunc());
            }
            
            if(forceBootloadMode)
            {
                /* Override any soft reset from the bootloader, so we will do
                 one when bootloader mode is done. */
                bootloaderData.softReset = false;
                bootloaderData.currentState = BOOTLOADER_OPEN_DATASTREAM;
            }
            else
            {
                /* User reset address is not erased. Start program. */
                bootloaderData.currentState = BOOTLOADER_CLOSE_DATASTREAM;
            }
            break;
        }
       
        case BOOTLOADER_OPEN_DATASTREAM:
        {
            
            bootloaderData.streamHandle = DATASTREAM_Open(
                    DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);

            if (bootloaderData.streamHandle != DRV_HANDLE_INVALID )
            {
                
                if((bootloaderData.type != TYPE_USB_HOST) && (bootloaderData.type != TYPE_SD_CARD))
                {
                DATASTREAM_BufferEventHandlerSet(bootloaderData.streamHandle,
                        Bootloader_BufferEventHandler, APP_USR_CONTEXT);
                bootloaderData.currentState = BOOTLOADER_GET_COMMAND;
                }
                else
                {
                /* Host layer was opened successfully. Enable operation
                * and then wait for operation to be enabled  */
                bootloaderData.currentState = BOOTLOADER_WAIT_FOR_HOST_ENABLE;
                }
            }
            break;
        }

        case BOOTLOADER_PROCESS_COMMAND:
        {
            Bootloader_ProcessBuffer(&bootloaderData);
            break;
        }

        case BOOTLOADER_GET_COMMAND:
        {
            /* Get the datastream driver status */
            bootloaderData.datastreamStatus = DATASTREAM_ClientStatus( bootloaderData.streamHandle );
            /* Check if client is ready or not */
            if ( bootloaderData.datastreamStatus == DRV_CLIENT_STATUS_READY )
            {
                bootloaderData.bufferSize = BOOTLOADER_BUFFER_SIZE;
                
                 DATASTREAM_Data_Read( &(bootloaderData.datastreamBufferHandle),
                        bootloaderData.data->buffers.buff1, bootloaderData.bufferSize);

                if ( bootloaderData.datastreamBufferHandle == DRV_HANDLE_INVALID )
                {
                    /* Set the app state to invalid */
                    bootloaderData.currentState = BOOTLOADER_ERROR;
                }
                else
                {
                    /* Set the App. state to wait for done */
                    bootloaderData.prevState    = BOOTLOADER_GET_COMMAND;
                    bootloaderData.currentState = BOOTLOADER_WAIT_FOR_DONE;
                }
            }
            break;
        }

        case BOOTLOADER_WAIT_FOR_DONE:
        {
            /* check if the datastream buffer event is complete or not */
            if (bootloaderData.usrBufferEventComplete)
            {
                bootloaderData.usrBufferEventComplete = false;
                
                /* Get the next App. State */
                switch (bootloaderData.prevState)
                {
                    case BOOTLOADER_GET_COMMAND:
                        bootloaderData.currentState = BOOTLOADER_PROCESS_COMMAND;
                        break;
                    case BOOTLOADER_SEND_RESPONSE:
                    default:
                        bootloaderData.currentState = BOOTLOADER_GET_COMMAND;
                        break;
                }
            }
            break;
        }

    case BOOTLOADER_WAIT_FOR_NVM:
        do
        {
            status = _EFC_REGS->EEFC_FSR.w;
        } while ((status & EEFC_FSR_FRDY_Msk) != EEFC_FSR_FRDY_Msk);

        bootloaderData.currentState = BOOTLOADER_SEND_RESPONSE;
        break;

    case BOOTLOADER_SEND_RESPONSE:
        {
            if(bootloaderData.bufferSize)
            {
                /* Calculate the CRC of the response*/
                crc = APP_CalculateCrc(bootloaderData.data->buffers.buff1, bootloaderData.bufferSize);
                bootloaderData.data->buffers.buff1[bootloaderData.bufferSize++] = (uint8_t)crc;
                bootloaderData.data->buffers.buff1[bootloaderData.bufferSize++] = (crc>>8);

                bootloaderData.data->buffers.buff2[BuffLen++] = SOH;

                for (i = 0; i < bootloaderData.bufferSize; i++)
                {
                    if ((bootloaderData.data->buffers.buff1[i] == EOT) || (bootloaderData.data->buffers.buff1[i] == SOH)
                        || (bootloaderData.data->buffers.buff1[i] == DLE))
                    {
                        bootloaderData.data->buffers.buff2[BuffLen++] = DLE;
                    }
                    bootloaderData.data->buffers.buff2[BuffLen++] = bootloaderData.data->buffers.buff1[i];
                }

                bootloaderData.data->buffers.buff2[BuffLen++] = EOT;
                bootloaderData.bufferSize = 0;

                DATASTREAM_Data_Write( &(bootloaderData.datastreamBufferHandle),
                        bootloaderData.data->buffers.buff2, BuffLen);

                if ( bootloaderData.datastreamBufferHandle == DRV_HANDLE_INVALID )
                {
                    bootloaderData.currentState = BOOTLOADER_ERROR;
                }
                else
                {
                    bootloaderData.prevState = BOOTLOADER_SEND_RESPONSE;
                    bootloaderData.currentState = BOOTLOADER_WAIT_FOR_DONE;
                }
            }
            break;
        }

        case BOOTLOADER_WAIT_FOR_HOST_ENABLE:

            /* Check if the host operation has been enabled */
            if(DATASTREAM_ClientStatus(bootloaderData.streamHandle) == DRV_CLIENT_STATUS_READY)
            {
                /* This means host operation is enabled. We can
                 * move on to the next state */
                 DATASTREAM_BufferEventHandlerSet((DRV_HANDLE)bootloaderData.streamHandle, NULL, 0);
                 bootloaderData.currentState = BOOTLOADER_WAIT_FOR_DEVICE_ATTACH;
            }
            break;

        case BOOTLOADER_WAIT_FOR_DEVICE_ATTACH:
            /* Wait for device attach. The state machine will move
             * to the next state when the attach event
             * is received.  */
            break;

#if(SYS_FS_MAX_FILES > 0)
        case BOOTLOADER_DEVICE_CONNECTED:
            /* Device was connected. We can try opening the file */
            bootloaderData.currentState = BOOTLOADER_OPEN_FILE;
            break;

        case BOOTLOADER_OPEN_FILE:

            /* Try opening the file for reading */
            bootloaderData.fileHandle = SYS_FS_FileOpen(SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0 "/" BOOTLOADER_IMAGE_FILE_NAME, (SYS_FS_FILE_OPEN_READ));
            if(bootloaderData.fileHandle == SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the file. Error out*/
                bootloaderData.currentState = BOOTLOADER_ERROR;
            }
            else
            {
                /* File opened successfully. Read file */
                APP_FlashErase();
                bootloaderData.currentState = BOOTLOADER_READ_FILE;
            }
            break;

        case BOOTLOADER_READ_FILE:

            /* Try reading the file */
            BuffLen = DATASTREAM_Data_Read(NULL, (void*)fileBuffer, FILE_BUFFER_SIZE);
            if (BuffLen <= 0)
            {
                SYS_FS_FileClose(bootloaderData.fileHandle);
                bootloaderData.currentState = BOOTLOADER_CLOSE_DATASTREAM;
                SYS_RESET_SoftwareReset();
            }
            else
            {
                memcpy(&bootloaderData.data->buffer[bootloaderData.cmdBufferLength], fileBuffer, BuffLen);
                
                /* Process the buffer that we read. */
                bootloaderData.bufferSize = BuffLen;
                Bootloader_ProcessBuffer(&bootloaderData);
            }

            break;
#endif
        case BOOTLOADER_CLOSE_DATASTREAM:
            DATASTREAM_Close();
            bootloaderData.currentState = BOOTLOADER_ENTER_APPLICATION;
        case BOOTLOADER_ENTER_APPLICATION:
            Enter_Application();
            break;

        case BOOTLOADER_ERROR:
        default:
            bootloaderData.currentState = BOOTLOADER_ERROR;
            break;
    }

#if((DRV_USBFSV1_HOST_SUPPORT == false) && \
    (DRV_USBHSV1_HOST_SUPPORT == false) && \
    !defined(DRV_SDCARD_INSTANCES_NUMBER) && \
    !defined(DRV_SDHC_INSTANCES_NUMBER))
    /* Maintain Device Drivers */
    DATASTREAM_Tasks();
#endif

}
