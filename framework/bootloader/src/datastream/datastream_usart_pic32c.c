/*******************************************************************************
 Data Stream USART Source File

  File Name:
    datastream_usart_pic32c.c

  Summary:
 Data Stream USART source

  Description:
    This file contains source code necessary for the data stream interface.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END
#include "bootloader/src/datastream.h"
#include "bootloader/src/bootloader.h"
#include "driver/usart/drv_usart_static.h"

extern DATASTREAM_HandlerType* handler;
extern uintptr_t _context;

typedef struct {
    DRV_HANDLE usart_handle;
    uint16_t read_cntr;
    uint16_t write_cntr;
    uint8_t buff[BOOTLOADER_BUFFER_SIZE];
}T_USART_DATA;

static T_USART_DATA usart_data;

static void usart_receive_handler(const SYS_MODULE_INDEX index)
{
    if (usart_data.write_cntr == BOOTLOADER_BUFFER_SIZE)
        usart_data.write_cntr = 0;
    if (!DRV_USART_ReceiverBufferIsEmpty(usart_data.usart_handle))
    {
        usart_data.buff[usart_data.write_cntr++] = DRV_USART_ReadByte(usart_data.usart_handle);
    }
}

DRV_HANDLE DATASTREAM_Open(const DRV_IO_INTENT ioIntent)
{
    usart_data.usart_handle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE);
    DRV_USART_ByteReceiveCallbackSet(DRV_USART_INDEX_0, usart_receive_handler);
    return 0;
}

void DATASTREAM_Close(void)
{
    DRV_USART_Deinitialize((SYS_MODULE_OBJ)DRV_USART_INDEX_0);
}

void DATASTREAM_Tasks(void)
{
    if (handler == (DATASTREAM_HandlerType*)NULL)
    {
        return;
    }
    
    if (RX == currDir)
    {
        while (usart_data.read_cntr != usart_data.write_cntr)
        {
            // Copy the data to the buffer
            _rxBuffer[_rxCurSize] = usart_data.buff[usart_data.read_cntr++];

            if(usart_data.read_cntr == BOOTLOADER_BUFFER_SIZE)
                usart_data.read_cntr = 0;
            
            if((_rxBuffer[_rxCurSize++] == EOT) || (_rxCurSize >= _rxMaxSize))
            {
                /* All the Data has been read clear the buffer */
                if(usart_data.read_cntr == usart_data.write_cntr)
                {
                    memset(usart_data.buff, 0, BOOTLOADER_BUFFER_SIZE);
                    usart_data.read_cntr = usart_data.write_cntr = 0;
                }
                currDir = IDLE;
                handler(DATASTREAM_BUFFER_EVENT_COMPLETE, (DATASTREAM_BUFFER_HANDLE)_bufferHandle, _rxCurSize);
                _rxCurSize = 0;
                break;
            }
        }
    }
    else if (TX == currDir)
    {
        while (_txCurPos < _txMaxSize)
        {
            if (!DRV_USART0_TransmitBufferIsFull())
            {
                DRV_USART0_WriteByte(_txBuffer[_txCurPos++]);

                if (_txCurPos == _txMaxSize) // All data has been sent or is in the buffer
                {
                    currDir = IDLE;
                    handler(DATASTREAM_BUFFER_EVENT_COMPLETE, (DATASTREAM_BUFFER_HANDLE)_bufferHandle, _context);
                }
            }
        }
    }
}
