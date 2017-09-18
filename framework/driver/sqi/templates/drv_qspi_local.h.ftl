/*******************************************************************************
  SQI Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sqi_local.h

  Summary:
    SQI driver local declarations and definitions

  Description:
    This file contains the SQI driver's local declarations and definitions.
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

#ifndef _DRV_QSPI_LOCAL_H
#define _DRV_QSPI_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "arch/arm/devices_pic32c.h" /* PIC32C system header. */
#include "driver/sqi/drv_sqi.h"
#include "driver/sqi/drv_sqi_init.h"
#include "system/dma/sys_dma.h"

//#include "system/int/sys_int.h"
//#include "osal/osal.h"
#include "system/debug/sys_debug.h"
#include "system/common/sys_queue.h"


// *****************************************************************************
/* External SQI memory mapping base address */
#define QSPI_RW_BUFFER_ADDR 0x80000000


// *****************************************************************************
/* SQI Driver Buffer Handle Macros

  Summary:
    SQI driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. SQI driver buffer handle is a
    combination of buffer token and the buffer object index. The buffertoken
    is a 16 bit number that is incremented for every new write or erase request
    and is used along with the buffer object index to generate a new buffer
    handle for every request.

  Remarks:
    None
*/
#define _DRV_SQI_BUF_TOKEN_MAX         (0xFFFF)
#define _DRV_SQI_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define _DRV_SQI_UPDATE_BUF_TOKEN(token) {\
(token)++;\
(token) = ((token) == _DRV_SQI_BUF_TOKEN_MAX) ? 0: (token);\
}


// *****************************************************************************
/* QSPI Client Object
  Summary:
    Defines the content of QSPI Object
*/
typedef struct {

	/* Hardware instance object associate with the client */
	void * driverObj;

	/* Status of the client object */
    SYS_STATUS status;

	/* The intent with which the client was opened */
    DRV_IO_INTENT intent;

	/* Flag to indicate in use */
    bool inUse;

	/* Client specific event handler */
    DRV_SQI_EVENT_HANDLER eventHandler;

	/* Client specific context */
    uintptr_t context;

} DRV_SQI_CLIENT_OBJECT;


// *****************************************************************************
/* QSPI Buffer Object
  Summary:
    Defines the content of one QSPI transfer Buffer
*/
typedef struct DRV_SQI_BUFFER_OBJECT{

	/* True if transfer started */
	bool inUse;

	/* SQI device for which this buffer object is queued */
    qspi_registers_t *sqiId;

	/* Client that owns this buffer */
    DRV_HANDLE hClient;

	/* Current command handle of this buffer object */
    DRV_SQI_COMMAND_HANDLE commandHandle;

	/* Present status of this command */
    DRV_SQI_COMMAND_STATUS status;

	/* Pointer to current xfer element */
    DRV_SQI_TransferElement *xferData;

	/* Number of Xfer */
    uint32_t xferNumber;

	/* Xfer counter */
    uint32_t xferCnt;

} DRV_SQI_BUFFER_OBJECT;


// *****************************************************************************
/* QSPI Driver Hardware Instance Object */
typedef struct DRV_SQI_OBJECT{

	/* Module index associated with the object */
    qspi_registers_t *sqiId;

	/* Object Index in */
    SYS_MODULE_INDEX objIndex;

	/* Status of the driver */
    SYS_STATUS status;

	/* Object in use status */
    bool inUse;

	/* Object is exclusive status */
    bool isExclusive;

	/* Number of connected clients */
    uint8_t numClients;

	/* QSPI Hardware instance Clock Divider */
    uint8_t clockDivider;

	/* QSPI Operation Mode */
    DRV_SQI_SPI_OPERATION_MODE operationMode;

	/* Associated buffer object queue */
    QUEUE_OBJECT queue;
	
	/* Associated DMA Channel handlers */
    SYS_DMA_CHANNEL_HANDLE DMA_Tx_Channel;
    SYS_DMA_CHANNEL_HANDLE DMA_Rx_Channel;

	/* Current Buffer Object */
	QUEUE_ELEMENT_OBJECT queueElements[DRV_SQI_BUFFER_OBJECT_NUMBER];

    /* SQI Interrupt Source */
    INT_SOURCE interruptSource;

} DRV_SQI_OBJECT;


#endif //#ifndef _DRV_QSPI_LOCAL_H

/*******************************************************************************
 End of File
*/

