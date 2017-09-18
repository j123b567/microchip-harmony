/******************************************************************************
  SQI Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sqi.h

  Summary:
    SQI Driver Interface Definition

  Description:
    The SQI driver provides data structues and interfaces to manage the SQI
    controller. This file contains the data structures and interface
    definitions of the SQI driver.
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
#ifndef _DRV_SQI_H
#define _DRV_SQI_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the bottom of
          this file.
*/

#include "system/common/sys_common.h"
#include "driver/driver_common.h"
#include "system/common/sys_module.h"
#include "system/int/sys_int.h"
#include "osal/osal.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SQI Driver Command Handle

  Summary:
    Identies a read or a write request queued to the SQI driver.

  Description:
    A command handle is returned by a call to the DRV_SQI_Read() or
    DRV_SQI_Write() functions. This handle allows the application to track the
    completion of the request. This command handle is also returned to the
    client along with the event that has occurred with respect to the request.
    This allows the application to connect the event to a specific command
    request in case where multiple commands are queued.

    The command handle associated with the command request expires when the
    client has been notified of the completion of the command (after event
    handler function that notifies the client returns) or after the command has
    been retired by the driver if no event handler callback was set. 

  Remarks:
    None.
*/

typedef uintptr_t DRV_SQI_COMMAND_HANDLE;


// *****************************************************************************
/* SQI Driver Invalid Command Handle

  Summary:
    Identifies an invalid command handle.

  Description:
    This is the definition of an invalid command handle. An invalid command
    handle is returned by DRV_SQI_Read() and DRV_SQI_Write() function if the
    read or write request was not queued.

  Remarks:
    None.
*/

#define DRV_SQI_COMMAND_HANDLE_INVALID ((DRV_SQI_COMMAND_HANDLE)(-1))

// *****************************************************************************
/* SQI Driver Module Index Numbers

  Summary:
    SQI driver index definitions.

  Description:
    This constant provides the SQI driver index definition.

  Remarks:
    This constant should be used in place of hard-coded numeric literal.

    This value should be passed into the DRV_SQI_Initialize and DRV_SQI_Open
    functions to identify the driver instance in use.
*/

#define DRV_SQI_INDEX_0         0


// *****************************************************************************
/* Flags associated with the SQI Driver Transfer element.

  Summary:
    Enumeration of the configuration options associated with a single transfer
    element.

  Description:
    This enumeration lists the various configuration options associated with a
    single transfer element(Refer to the data structure
    DRV_SQI_TransferElement). The client can specify one or more of these as
    configuration parameters of a single transfer element.

  Remarks:
    None
*/

typedef enum
{
    /* Bits 0-1: Indicates the Lane configuration to be used. */
    DRV_SQI_FLAG_MODE_SINGLE_LANE  = 0x00,
    DRV_SQI_FLAG_MODE_DUAL_LANE    = 0x01,
    DRV_SQI_FLAG_MODE_QUAD_LANE    = 0x02,

    /* Bit 2: This bit indicates if DDR or SDR mode of operation is to be used.
     * */
    DRV_SQI_FLAG_DDR_MODE          = 0x04,

    /* Bit 3: This bit indicates if CS is to be de-asserted at the end of this
     * transaction. */
    DRV_SQI_FLAG_DEASSERT_CS       = 0x08,

    /* Bit 7: This bit indicates if the operation is a read or a write. */
    DRV_SQI_FLAG_DIR_READ          = 0x80,

} DRV_SQI_TRANSFER_FLAGS;


// *****************************************************************************
/* SQI Driver Events

   Summary
    Identifies the possible events that can result from a request.

   Description
    This enumeration identifies the possible events that can result from a read
    or a write request issued by the client.

   Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that client registered with the driver by
    calling the DRV_SQI_EventHandlerSet function when a request is completed.
*/

typedef enum
{
    /* Operation has been completed successfully. */
    DRV_SQI_EVENT_COMMAND_COMPLETE = 0,

    /* There was an error during the operation */
    DRV_SQI_EVENT_COMMAND_ERROR

} DRV_SQI_EVENT;


// *****************************************************************************
/* SQI Driver Command Status

   Summary
    Specifies the status of the command for the read or write command requests.

   Description
    This enumeration identifies the possible status values associated with a
    read or a write command request. The client can retrieve the status by
    calling the DRV_SQI_CommandStatus () function and passing the command
    handle associated with the request.

   Remarks:
     None.
*/

typedef enum
{
    /* Command completed. */
    DRV_SQI_COMMAND_COMPLETED,

    /* Command is pending. */
    DRV_SQI_COMMAND_QUEUED,

    /* Command is being processed */
    DRV_SQI_COMMAND_IN_PROGRESS,

    /* There was an error while processing the command. */
    DRV_SQI_COMMAND_ERROR_UNKNOWN

} DRV_SQI_COMMAND_STATUS;



// *****************************************************************************
/* SQI SPI Mode of operation

  Summary:
    Enumeration of the SPI mode of operation supported by the SQI Controller.

  Description:
    This enumeration lists the SPI mode of operation supported by the SQI
    controller.
    In MODE 0 of operation:
    CPOL = 0 and CPHA = 0. SCK Idle state = LOW

    In MODE 3 of operation:
    CPOL = 1 and CPHA = 1. SCK Idle state = HIGH
        
    In both MODE 0 and MODE 3 of operation the:
    SQI Data Input is sampled on the rising edge of the SQI Clock
    SQI Data is Output on the falling edge of the SQI Clock

  Remarks:  
    None
*/

typedef enum
{
    /* CPOL = 0 and CPHA = 0. SCK Idle state = LOW */
    DRV_SQI_SPI_MODE_0 = 0,

    /* CPOL = 1 and CPHA = 1. SCK Idle state = HIGH */
    DRV_SQI_SPI_MODE_3 = 3

} DRV_SQI_SPI_OPERATION_MODE;




// *****************************************************************************
/* SQI Driver data transfer element.

  Summary:
    Defines the data transfer element of the SQI driver.

  Description:
    This data type defines the composition of a single transfer element. A
    single element will consist of the pointer to the source of destination
    buffer, length of the data to be transferred or received and the various
    configuration options to be used for the element. The configuration options
    also indicate if data is transferred to/from the device. A client builds an
    array of such transfer elements and passes the array and the number of
    elements of the array as part of the read or write operation.

  Remarks:
    None.
*/

typedef struct
{
    /* Pointer to the source or destination buffer */
    uint8_t *data;

    /* Length of the buffer in bytes. */
    uint32_t length;

    /* This is a bitmap used to indicate the configuration options to be used
     * for this transfer element. One or more values of the enumeration
     * DRV_SQI_TRANSFER_FLAGS can be passed as part of this flag. */
    uint8_t flag;

} DRV_SQI_TransferElement;


// *****************************************************************************
/* SQI Driver Command Event Handler Function Pointer

   Summary:
    Pointer to a SQI Driver Command Event handler function

   Description:
    This data type defines the required function signature for the SQI driver
    command event handling callback function. A client must register a pointer
    to a command event handling function the signature(parameter and return
    value types) of which should match the types specified by this function
    pointer in order to receive command related event callbacks from the
    driver.

    The parameters and return values and return value are described here and
    a partial example implementation is provided.

  Parameters:
    event           - Identifies the type of event

    commandHandle   - Handle identifying the command to which this event
                      relates

    context         - Value identifying the context of the application that
                      registered the event handling function.

  Returns:
    None.

  Example:
    <code>
    void MyAppCommandEventHandler
    (
        DRV_SQI_EVENT event,
        DRV_SQI_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT)context;

        switch(event)
        {
            case DRV_SQI_EVENT_COMMAND_COMPLETE:
                // Handle the completed command.
                break;

            case DRV_SQI_EVENT_COMMAND_ERROR:
            default:
                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_SQI_EVENT_COMMAND_COMPLETE, that the operation
    associated with the command was completed successfully.
    If the event is DRV_SQI_EVENT_COMMAND_ERROR, there was an error while
    executing the command request.

    The context parameter contains context details provided by the client as
    part of registering the event handler function.  This context value is
    passed back to the client as the "context" parameter. It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) of the client that made the request.
*/

typedef void (*DRV_SQI_EVENT_HANDLER)
(
    DRV_SQI_EVENT event,
    DRV_SQI_COMMAND_HANDLE commandHandle, 
    void *context
);

// *****************************************************************************
// *****************************************************************************
// Section: SQI Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SQI_Initialize
    ( 
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init 
    );
    
  Summary:
    Initializes the SQI instance for the specified driver index

  Description:
    This routine initializes the SQI driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Precondition:
    None.
  
  Parameters:
    index -  Identifier for the instance to be initialized
    init -   Pointer to a data structure containing any data necessary to
             initialize the driver.
  
  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise it returns SYS_MODULE_OBJ_INVALID.
  
  Example:
    <code>
    // This code snippet shows an example of initializing the SQI Driver.
    
    SYS_MODULE_OBJ  objectHandle;

    // SQI Driver Initialization Data
    const DRV_SQI_INIT drvSqiInit =
    {
        .sqiId = SQI_ID_0,
        .interruptSource = INT_SOURCE_SQI1,
        .enabledDevices = DRV_SQI_ENABLE_BOTH_DEVICES,
        .clockDivider = DRV_SQI_CLK_DIV_1,
        .devCfg[0].spiMode = DRV_SQI_SPI_MODE_0,
        .devCfg[0].lsbFirst = true,
        .devCfg[1].spiMode = DRV_SQI_SPI_MODE_3,
        .devCfg[1].lsbFirst = false,
    };

    objectHandle = DRV_SQI_Initialize(DRV_SQI_INDEX_0, (SYS_MODULE_INIT*)&drvSqiInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other SQI routine is called.
    
    This routine should only be called once during system initialization unless
    DRV_SQI_Deinitialize is called to deinitialize the driver instance.
    
    This routine will NEVER block for hardware access. If the operation
    requires time to allow the hardware to initialize, it will be reported by
    the DRV_SQI_Status operation. The system must use DRV_SQI_Status to find
    out when the driver is in the ready state.
*/

SYS_MODULE_OBJ DRV_SQI_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
);


// ****************************************************************************
/* Function:
    void DRV_SQI_Deinitialize
    (
        SYS_MODULE_OBJ object 
    );
    
  Summary:
    Deinitializes the specified instance of the SQI driver module

  Description:
    Deinitializes the specified instance of the SQI driver module, disabling its
    operation (and any hardware). Invalidates all the internal data.
  
  Preconditions:
    Function DRV_SQI_Initialize should have been called before calling
    this function.
  
  Parameter:
    object -  Driver object handle, returned from the DRV_SQI_Initialize
              routine

  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example of deinitializing the driver.
    
    SYS_MODULE_OBJ      object;     //  Returned from DRV_SQI_Initialize
    SYS_STATUS          status;
    
    DRV_SQI_Deinitialize(object);
    
    status = DRV_SQI_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know when the driver is
        // deinitialized.
    }
    </code>
  
  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.
*/

void DRV_SQI_Deinitialize
(
    SYS_MODULE_OBJ object
);

// *************************************************************************
/* Function:
    SYS_STATUS DRV_SQI_Status
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Gets the current status of the SQI driver module.
  
  Description:
    This routine provides the current status of the SQI driver module.
  
  Preconditions:
    Function DRV_SQI_Initialize should have been called before calling this
    function.
  
  Parameters:
    object -  Driver object handle, returned from the DRV_SQI_Initialize
              routine
  
  Returns:
    SYS_STATUS_READY - Indicates that the driver is ready and accept requests
                       for new operations.
    
    SYS_STATUS_UNINITIALIZED - Indicates the driver is not initialized.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SQI_Initialize
    SYS_STATUS          sqiStatus;
    
    sqiStatus = DRV_SQI_Status(object);
    else if (SYS_STATUS_ERROR >= sqiStatus)
    {
        // Handle error
    }
    </code>
  
  Remarks:
    This routine will NEVER block waiting for hardware.
*/

SYS_STATUS DRV_SQI_Status
(
    SYS_MODULE_OBJ object
);


// ****************************************************************************
/* Function:
    void DRV_SQI_Tasks 
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Maintains the driver's task state machine.
  
  Description:
    This routine is used to maintain the driver's internal task state machine.
  
  Preconditions:
    The DRV_SQI_Initialize routine must have been called for the specified
    SQI driver instance.
  
  Parameters:
    object -  Object handle for the specified driver instance (returned from
              DRV_SQI_Initialize)
  Returns:
    None.
  
  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SQI_Initialize
    
    while (true)
    {
        DRV_SQI_Tasks (object);
        // Do other tasks
    }
    </code>

  Remarks:
    This routine is to be called by the system's task routine(SYS_Tasks).
*/

void DRV_SQI_Tasks
(
    SYS_MODULE_OBJ object
);


// *****************************************************************************
// *****************************************************************************
// Section: SQI Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_SQI_Open
    ( 
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    );
    
  Summary:
    Opens the specified SQI driver instance and returns a handle to it
  
  Description:
    This routine opens the specified SQI driver instance and provides a handle.
    This handle must be provided to all other client-level operations to
    identify the caller and the instance of the driver.
  
  Preconditions:
    Function DRV_SQI_Initialize must have been called before calling this
    function.
  
  Parameters:
    index  - Identifier for the object instance to be opened
    intent - Zero or more of the values from the enumeration
             DRV_IO_INTENT "ORed" together to indicate the intended use
             of the driver
  
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).
    
    If an error occurs, DRV_HANDLE_INVALID is returned. Errors can occur
    under the following circumstances:
    	- if the number of client objects allocated via DRV_SQI_CLIENTS_NUMBER 
          is insufficient
    	- if the client is trying to open the driver but driver has been opened
    	  exclusively by another client
    	- if the client is trying to open the driver exclusively, but has already
          been opened in a non exclusive mode by another client.
    	- if the driver hardware instance being opened is not initialized or is
          invalid
  
  Example:
    <code>
    DRV_HANDLE handle;
    
    handle = DRV_SQI_Open(DRV_SQI_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>
  
  Remarks:
    The handle returned is valid until the DRV_SQI_Close routine is called.
    This routine will NEVER block waiting for hardware. If the driver has 
    has already been opened, it cannot be opened exclusively.
*/

DRV_HANDLE DRV_SQI_Open
(
    const SYS_MODULE_INDEX index, 
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_SQI_Close
    (
        const DRV_HANDLE handle
    );

  Summary:
    Closes an opened-instance of the SQI driver

  Description:
    This routine closes an opened-instance of the SQI driver, invalidating the
    handle.

  Precondition:
    The DRV_SQI_Initialize routine must have been called for the specified
    SQI driver instance.

    DRV_SQI_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_SQI_Open

    DRV_SQI_Close(handle);
    </code>

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines. A new handle must be obtained by
    calling DRV_SQI_Open before the caller may use the driver again. Usually
    there is no need for the driver client to verify that the Close operation
    has completed.
*/

void DRV_SQI_Close
(
    const DRV_HANDLE handle
);


// *****************************************************************************
/* Function:
    void DRV_SQI_TransferData
    (
        DRV_HANDLE handle,
        DRV_SQI_COMMAND_HANDLE *commandHandle,
        uint8_t sqiDevice,
        DRV_SQI_TransferElement *xferData,
        uint8_t numElements
    );

  Summary:
    Queue a data transfer operation on the specified SQI device.

  Description:
    This routine queues a data transfer operation on the specified SQI device.
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

  Precondition:
    The DRV_SQI_Initialize routine must have been called for the specified SQI 
    driver instance.

    DRV_SQI_Open must have been called with DRV_IO_INTENT_READ or
    DRV_IO_INTENT_READWRITE as the ioIntent to obtain a valid opened device
    handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle

    sqiDevice     - The SQI device index on which the operation is to be
                    performed.
                   
    xferData      - Pointer to the transfer elements array.

    numElements   - Number of elements in the transfer elements array.

  Returns:
    The handle to the command request is returned in the commandHandle
    argument. It will be DRV_SQI_COMMAND_HANDLE_INVALID if the request was not
    successful.

  Example:
    <code>
    #define READ_BUF_SIZE 512

    uint8_t readBuffer[READ_BUF_SIZE] __attribute__((coherent, aligned(16)));
    uint8_t command [5] __attribute__((coherent, aligned(16)) = {0x0B, 0x00, 0x00, 0x00, 0x0FF};
    uint8_t numElements = 0;
    DRV_SQI_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;
    DRV_SQI_TransferElement xferData[2];

    // mySQIHandle is the handle returned by the DRV_SQI_Open function.
    // Setup the transfer elements.
    
    xferData[0].data = &command[0];
    xferData[0].length = sizeof(command);
    xferData[0].flag = (DRV_SQI_FLAG_MODE_SINGLE_LANE);

    xferData[1].data = readBuffer;
    xferData[1].length = READ_BUF_SIZE;
    xferData[1].flag = (DRV_SQI_FLAG_MODE_QUAD_LANE | DRV_SQI_FLAG_DIR_READ | DRV_SQI_FLAG_DEASSERT_CS);

    DRV_SQI_TransferData(mySQIHandle, &commandHandle, 0, xferData, 2);

    if(DRV_SQI_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }
    else
    {
        // Transfer operation queued successfully. Wait for the completion event.
    }

    </code>

  Remarks:
    None.
*/

void DRV_SQI_TransferData
(
    DRV_HANDLE handle,
    DRV_SQI_COMMAND_HANDLE *commandHandle,
    uint8_t sqiDevice,
    DRV_SQI_TransferElement *xferData,
    uint8_t numElements
);


// *****************************************************************************
/* Function:
    DRV_SQI_COMMAND_STATUS DRV_SQI_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_SQI_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the command. The application must
    use this routine where the status of a scheduled command needs to polled
    on. The function may return DRV_SQI_COMMAND_COMPLETED in a case where
    the command handle has expired. A command handle expires when the internal
    buffer object is re-assigned to another read/write request. It is
    recommended that this function be called regularly in order to track the
    command status correctly.

    The application can alternatively register an event handler to receive the
    read/write operation completion events.

  Preconditions:
    The DRV_SQI_Initialize() routine must have been called.

    The DRV_SQI_Open() must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
             routine

  Returns:
    A DRV_SQI_COMMAND_STATUS value describing the current status of the
    command. Returns DRV_SQI_COMMAND_ERROR_UNKNOWN if the client handle or the
    command handle is not valid.

  Example:
    <code>
    DRV_HANDLE                  handle;         // Returned from DRV_SQI_Open
    DRV_SQI_COMMAND_HANDLE      commandHandle;
    DRV_SQI_COMMAND_STATUS      status;
 
    status = DRV_SQI_CommandStatus(handle, commandHandle);
    if(status == DRV_SQI_COMMAND_COMPLETED)
    {
        // Operation Done
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_SQI_COMMAND_STATUS DRV_SQI_CommandStatus
(
    const DRV_HANDLE handle, 
    const DRV_SQI_COMMAND_HANDLE commandHandle
);


// *****************************************************************************
/* Function:
    void DRV_SQI_EventHandlerSet
    (
        const DRV_HANDLE handle,
        const void *eventHandler,
        const uintptr_t context
    );

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

  Precondition:
    The DRV_SQI_Initialize() routine must have been called for the specified
    SQI driver instance.

    The DRV_SQI_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

    eventHandler - Pointer to the event handler function implemented by the
                   user
    
    context      - The value of parameter will be passed back to the client 
                   unchanged, when the eventHandler function is called. It can
                   be used to identify any client specific data object that 
                   identifies the instance of the client module (for example, 
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific state data object.
    MY_APP_OBJ myAppObj;

    DRV_SQI_TransferElement xferElements[2];
    DRV_SQI_COMMAND_HANDLE commandHandle;

    // drvSQIHandle is the handle returned by the DRV_SQI_Open function.
    // Client registers an event handler with driver. This is done once.
    DRV_SQI_EventHandlerSet(drvSQIHandle, APP_SQIEventHandler, (uintptr_t)&myAppObj);

    DRV_SQI_Read(drvSQIHandle, &commandHandle, &xferElements, 2);

    if(DRV_SQI_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event handler.
    void APP_SQIEventHandler
    (
        DRV_SQI_EVENT event, 
        DRV_SQI_COMMAND_HANDLE handle,
        uintptr_t context
    )
    {
        // The context handle was set to an application specific object. It is
        // now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_SQI_EVENT_COMMAND_COMPLETE:
                // This means the operation was completed successfully.
                break;
            
            case DRV_SQI_EVENT_COMMAND_ERROR:
                // Operation failed. Handle the error.
                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    If the client does not want to be notified when the queued operation has
    completed, it does not need to register a callback.
*/

void DRV_SQI_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _DRV_SQI_H
/*******************************************************************************
 End of File
*/

