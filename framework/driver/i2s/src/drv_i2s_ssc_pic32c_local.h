/*******************************************************************************
  I2S Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2s_ssc_pic32c.h

  Summary:
    I2S driver local declarations and definitions

  Description:
    This file contains the I2S driver's local declarations and definitions
    for DMA and Non DMA mode of operations.
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
//DOM-IGNORE-END

#ifndef DRV_I2S_SSC_PIC32C_H_
#define DRV_I2S_SSC_PIC32C_H_


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system_config.h"
#include "system/debug/sys_debug.h"
#include "osal/osal.h"
#include "driver/i2s/drv_i2s_ssc_definitions_pic32c.h"
#include "driver/i2s/drv_i2s.h"
#ifndef DRV_I2S_INTERRUPT_MODE
#define DRV_I2S_INTERRUPT_MODE false
#endif
#include "driver/i2s/src/drv_i2s_variant_mapping.h"

// *****************************************************************************
/* I2S DMA transfer error

  Summary:
    I2S DMA transfer error

  Description:
    This constant indicates that the I2S DMA transfer has an address error.

  Remarks:
    None.
*/
#define DRV_I2S_DMA_TRANSFER_ERROR /*DOM-IGNORE-BEGIN*/((uint32_t)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/* I2S Buffer object Index

  Summary:
    I2S Buffer object Index

  Description:
    This constant indicates the I2S Buffer object Index.

  Remarks:
    None.
*/
#define DRV_I2S_BUFFER_OBJECT_INDEX_INVALID /*DOM-IGNORE-BEGIN*/((uint32_t)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* I2S Driver task states

  Summary
    Lists the different states that I2S task routine can have.

  Description
    This enumeration lists the different states that I2S task routine can have.

  Remarks:
    None.
*/

typedef enum
{
    /* Process queue */
    DRV_I2S_TASK_PROCESS_QUEUE,

    /* I2S task handle read only buffer request */
    DRV_I2S_TASK_PROCESS_READ_ONLY,

    /* I2S task handle write only buffer request */
    DRV_I2S_TASK_PROCESS_WRITE_ONLY,

    /* I2S task handle write READ buffer request */
    DRV_I2S_TASK_PROCESS_WRITE_READ


} DRV_I2S_TASK;

// *****************************************************************************
/* I2S Driver task buffer operations mode

  Summary
    Lists the different buffer operation modes that I2S driver can have.

  Description
    This enumeration lists the different modes that I2S task routine can have.

  Remarks:
    None.
*/

typedef enum
{
    /* Buffer is a Read only */
    DRV_I2S_BUFFER_OPERATION_TYPE_READ,

    /* Buffer is a Write only */
    DRV_I2S_BUFFER_OPERATION_TYPE_WRITE,

    /* Buffer is a ReadWrite only */
    DRV_I2S_BUFFER_OPERATION_TYPE_WRITEREAD,

} DRV_I2S_BUFFER_OPERATION_TYPE;

/**********************************************
 * Driver Client Obj
 **********************************************/
typedef struct _DRV_I2S_SSC_CLIENT_OBJ_STRUCT
{
    /* SSC Module ID */
    SSC_MODULE_ID sscID;

    /* Indicates that this object is in use */
    bool inUse;

    /* Indicate whether the client is open in read,write or read/write mode */
    DRV_IO_INTENT ioIntent;

    /* Indicates the error information for the last I2S operation (Read/Write)*/
    DRV_I2S_ERROR errorInfo;

    DRV_I2S_BUFFER_EVENT    bufferEvent;

    /* Call back function for this client */
    DRV_I2S_BUFFER_EVENT_HANDLER  pEventCallBack;

    /* Client data(Event Context) that will be returned at callback */
    uintptr_t hClientArg;

    /* pointer to the driver that own this object */
    void* hDriver;

} DRV_I2S_CLIENT_OBJ;

/***************************************************
 * This object is used by the driver as buffer place
 * holder along with queueing feature.
 ***************************************************/
typedef struct _DRV_I2S_BUFFER_OBJECT
{
    /* Indicates this object is in use */
    bool inUse;

	/* index handle to the buffer object */
    uint32_t indexHandle;

    /* tx Data buffer pointer */
    uint8_t *txbuffer;

    /* rx Data buffer pointer */
    uint8_t *rxbuffer;

    /* size in bytes */
    size_t size;

    /* Pending bytes in the object */
    size_t nPendingBytes;

    /* Pointer to the client object */
    DRV_I2S_CLIENT_OBJ	*clientObject;

    /* Buffer Operation Type */
    DRV_I2S_BUFFER_OPERATION_TYPE bType;

    /* This points to the next object in the queue */
    struct _DRV_I2S_BUFFER_OBJECT * next;

} DRV_I2S_BUFFER_OBJECT;

/***************************************************
 * This is a data type for the buffer object
   index which will be combined with the unique index id
   to produce a unique buffer handle.
 ***************************************************/
typedef uint32_t DRV_I2S_BUFFER_OBJECT_INDEX;

/***********************************************
 * Driver object structure. One object per
 * hardware instance
 **********************************************/

typedef struct _DRV_I2S_SSC_OBJ_STRUCT
{
    ssc_registers_t *moduleID;
            
    /* The peripheral Id associated with the object */
    SSC_MODULE_ID sscID;

    /* Status of this driver instance */
    SYS_STATUS status;

    /* Indicates this object is in use */
    bool inUse;

    /* Flag to indicate that the hardware instance is used
     *  in exclusive access mode */
    bool isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t numClients;

    /* I2S Initial Baud Rate Value */
    uint32_t baudRate;

    /* I2S Audio communication width */
    uint8_t audioCommWidth;

   /* Interrupt Source for Transmit Interrupt*/
    INT_SOURCE interruptSSC;

    /* Keeps track if the driver is in interrupt
     * context */
    bool isInInterruptContext;

    /* Hardware instance mutex */
    OSAL_MUTEX_DECLARE(mutexDriverInstance);

    /* Create a semaphore for read write function*/
    OSAL_SEM_DECLARE(semBufferQueueEmpty);

    /* This is the transmit buffer queue size. It
     * Indicates the number of available empty slots */
    uint32_t queueSizeTransmit;

    /* This is the receive buffer queue size.  It
     * Indicates the number of available empty slots */
    uint32_t queueSizeReceive;

   /* This is the transmit buffer current queue size. It
     * Indicates the number of occupied slots */
    uint32_t queueCurrentTransmitSize;

    /* This is the receive buffer current queue size.  It
     * Indicates the number of occupied slots  */
    uint32_t queueCurrentReceiveSize;

	/* DMA write channel handle */
    SYS_DMA_CHANNEL_HANDLE  dmaChannelHandleWrite;

	/* DMA read channel handle */
    SYS_DMA_CHANNEL_HANDLE  dmaChannelHandleRead;

    INT_SOURCE interruptDMA;
   
    /* This is the buffer queue object pointer */
    DRV_I2S_BUFFER_OBJECT *queueHead;
    
    DRV_I2S_BUFFER_OBJECT *readQueueHead;
    
    DRV_I2S_BUFFER_OBJECT *writeQueueHead;

    /* I2S hardware object task in process */
    DRV_I2S_TASK task;

    /* Flag to inhibit further transmission
    unless, a data is received */
    uint32_t rxInhibit;

    /* SSC module source/destination size */
    size_t srcDestSize;

    /* Cell Size */
    size_t cellSize;

} DRV_I2S_SSC_OBJ;

// *****************************************************************************
/* I2S Driver Global Instances Object

  Summary:
    Object used to keep track of data that is common to all instances of the
    I2S driver.

  Description:
    This object is used to keep track of any data that is common to all
    instances of the I2S driver.

  Remarks:
    None.
*/

typedef struct
{
    /* Set to true if all members of this structure
       have been initialized once */
    bool membersAreInitialized;

    /* Mutex to protect client object pool */
    OSAL_MUTEX_DECLARE(mutexClientObjects);

} DRV_I2S_COMMON_DATA_OBJ;

 /*
  Function:
    SYS_MODULE_OBJ  DRV_I2S_SSC_Initialize 
    (
        const SYS_MODULE_INDEX drvIndex, 
        const SYS_MODULE_INIT *const init
    )
  
  Summary:
    Initializes the SSC Module. This function will be called from Abstraction 
    layer's Initialize function.

  Description:
    This routine initializes the SSC driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the init parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized. The driver instance index is
    independent of the I2S module ID.
    
    This function will be called from I2S Abstraction layer.
    Refer drv_i2s.h for more information.  

  Remarks:
    None
*/

SYS_MODULE_OBJ  DRV_I2S_SSC_Initialize 
(
    const SYS_MODULE_INDEX drvIndex, 
    const SYS_MODULE_INIT *const init
);

 /*
  Function:
    DRV_HANDLE DRV_I2S_SSC_Open
    (
        const SYS_MODULE_INDEX iDriver, 
        const DRV_IO_INTENT ioIntent
    )

  Summary:
    Opens the specified ssc driver instance and returns a handle to it.
    Dynamic Implementation.

  Description:
    This routine initializes the SSC driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the init parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized. The driver instance index is
    independent of the I2S module ID.
    
    This function will be called from I2S Abstraction layer.
    Refer drv_i2s.h for more information. 

  Remarks:
    None
*/

DRV_HANDLE DRV_I2S_SSC_Open
(
    const SYS_MODULE_INDEX iDriver, 
    const DRV_IO_INTENT ioIntent
);

 /*
  Function:
    void DRV_I2S_SSC_BufferAddWrite
    (
        const DRV_HANDLE handle, 
        DRV_I2S_BUFFER_HANDLE *bufferHandle,
        void *buffer, 
        size_t size
    )

  Summary:
    Schedule a non-blocking driver write operation.
    Dynamic Implementation.

  Description:
    This function schedules a non-blocking write operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the write request
    was scheduled successfully. The function adds the request to the hardware
    instance transmit queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.
    
    This function will be called from I2S Abstraction layer.
    Refer drv_i2s.h for more information. 

  Remarks:
    None
*/

void DRV_I2S_SSC_BufferAddWrite
(
    const DRV_HANDLE handle, 
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *buffer, 
    size_t size
);

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
    instance receive queue and returns immediately. 
 
    This function will be called from I2S Abstraction layer.
    Refer drv_i2s.h for more information. 
   
  Remarks:
    None.

*/

void DRV_I2S_SSC_BufferAddRead
(
    const DRV_HANDLE handle,
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *buffer, 
    size_t size
);

 /*
  Function:
    void DRV_I2S_SSC_BufferEventHandlerSet
    (
        DRV_HANDLE handle,
        const DRV_I2S_BUFFER_EVENT_HANDLER eventHandler, 
        const uintptr_t contextHandle
    )

  Summary:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
    Dynamic Implementation.

  Description:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
    When a client calls either the DRV_I2S_BufferAddRead, DRV_I2S_BufferAddWrite
    or DRV_I2S_BufferAddWriteRead  function, it is provided with a handle
    identifying  the buffer that was added to the driver's buffer queue.  The
    driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.
    
    This function will be called from I2S Abstraction layer.
    Refer drv_i2s.h for more information. 

  Remarks:
    None
*/

void DRV_I2S_SSC_BufferEventHandlerSet
(
    DRV_HANDLE handle,
    const DRV_I2S_BUFFER_EVENT_HANDLER eventHandler, 
    const uintptr_t contextHandle
);

 /*
  Function:
    void DRV_I2S_SSC_BufferAddWriteRead
    (   
        const DRV_HANDLE handle,
        DRV_I2S_BUFFER_HANDLE *bufferHandle,
        void *transmitBuffer,
        void *receiveBuffer,
        size_t size
    )

  Summary:
    Schedule a non-blocking driver write-read operation.
    Dynamic Implementation.

  Description:
   This function schedules a non-blocking write-read operation. The function
    returns with a valid buffer handle in the bufferHandle argument if the
    write-read request was scheduled successfully. The function adds the request
    to the hardware instance queue and returns immediately. While the request is
    in the queue, the application buffer is owned by the driver and should not
    be modified.
    
    This function will be called from I2S Abstraction layer.
    Refer drv_i2s.h for more information. 

  Remarks:
    None
*/

void DRV_I2S_SSC_BufferAddWriteRead
(   
    const DRV_HANDLE handle,
    DRV_I2S_BUFFER_HANDLE *bufferHandle,
    void *transmitBuffer,
    void *receiveBuffer,
    size_t size
);

 /*
  Function:
    void DRV_I2S_SSC_WriteTasks
    (
        SYS_MODULE_OBJ object
    )

  Summary:
    Maintains the driver's receive state machine and implements its ISR,
    This function only works for WRITE mode.
    Dynamic Implementation.

  Description:
    In DMA mode of operation, this function should be called from the interrupt
    service routine of the channel associated with the transmission/reception
    of the I2s driver hardware instance.
    
    This function will be called from I2S Abstraction layer.
    Refer drv_i2s_ssc_pic32c.c file for more information. 

  Remarks:
    None
*/

void DRV_I2S_SSC_WriteTasks
(
    SYS_MODULE_OBJ object
);

/**************************************
 * Local functions.
 *************************************/
 /*
  Function:
    static void _DRV_I2S_SetupHardware 
    (
        DRV_I2S_OBJ *dObj, 
        DRV_I2S_INIT * i2sInit 
    )

  Summary:
    Initializes the hardware registers.

  Description:
    Take the initialization data from the application(through DRV_I2S_Initialize
    function) and initialize the hardware registers.

  Remarks:
    None
*/
static void   _DRV_I2S_SSC_HardwareSetup(DRV_I2S_SSC_OBJ *dObj, DRV_I2S_SSC_INIT *i2sSSCInit);

/*
  Function:
    static DRV_I2S_BUFFER_OBJECT_INDEX _DRV_I2S_QueueObjectIndexGet(void)

  Summary:
    Gets the index of buffer queue object from the available pool.

  Description:
    Returns a buffer object index from the available pool.

  Remarks:
    None
*/
static DRV_I2S_BUFFER_OBJECT_INDEX _DRV_I2S_SSC_QueueObjectIndexGet(void);
static DRV_I2S_BUFFER_OBJECT_INDEX _DRV_I2S_SSC_ReadQueueObjectIndexGet(void);
static DRV_I2S_BUFFER_OBJECT_INDEX _DRV_I2S_SSC_WriteQueueObjectIndexGet(void);

/*
  Function:
	static void _DRV_I2S_DMA_EventHandler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);

  Summary:
    This is the Event handler of I2S Driver from DMA system service.

  Description:
	The DMA system service gives calls this Event handler on finish
	of a DMA data transfer. This Event handler identifies the event
	(Transfer Complete, Transfer Abort or DMA address error)and calls
	the event handler of the I2S driver notifying the event.

  Remarks:
    None
*/
static void _DRV_I2S_SSC_DMA_EventHandler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);
static void _DRV_I2S_SSC_DMA_ReadEventHandler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);
static void _DRV_I2S_SSC_DMA_WriteEventHandler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);

#endif /* DRV_I2S_SSC_PIC32C_H_ */