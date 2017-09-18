/*******************************************************************************
  Digital-to-Analog Converter Driver Interface Declarations for Dynamic Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_dac.h

  Summary:
    Digital-to-Analog Converter driver interface declarations for the Dynamic 
    driver.

  Description:
    The Digital-to-Analog Converter device driver provides a simple interface to 
	manage the Digital-to-Analog Converter module on Microchip microcontrollers. 
	This file defines the interface Declarations for the Digital-to-Analog 
    Converter driver.
    
  Remarks:
    None
*******************************************************************************/

// DOM-IGNORE-BEGIN
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
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTOCULAR PURPOSE.
IN NO EVENT SHALL MOCROCHIP OR ITS LOCENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STROCT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVOCES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END
#ifndef _DRV_DAC_H
#define _DRV_DAC_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/driver_common.h"
#include "system/system.h"
#include "system/int/sys_int.h"
#include "system/debug/sys_debug.h"
#include "drv_dac_definitions_pic32c.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for the Dynamic driver
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_DAC_Initialize(const SYS_MODULE_INDEX index,
                                      const SYS_MODULE_INIT * const init);

  Summary:
    Initializes the DAC for the specified driver index.
    <p><b>Implementation:</b> Dynamic</p>	

  Description:
    This routine initializes the DAC driver instance for the specified driver
    instance, making it ready for clients to use it. The initialization routine
    is specified by the MHC parameters. 

  Precondition:
    None.

  Parameters:
    drvIndex        - Index for the driver instance to be initialized

    init            - Pointer to a data structure containing any data necessary
                      to initialize the driver. 

  Returns:
    If successful, returns a valid handle to a driver object. Otherwise, it
    returns SYS_MODULE_OBJ_INVALID. 

   Example:
    <code>
    // The following code snippet shows an example of DAC driver initialization.
    // The driver is initialized for Single Ended Analog output mode. 
        
    const DRV_DAC_INIT drvDACInitData[] =
    {
        //Channel 0 Initialization data
        {	
            .dacID = DACC_ID_0,
            .dacIndex = DRV_DAC_INST_IDX0, 
            .outputMode = DRV_DAC_OUTPUT_TYPE,
            .operationMode = DRV_DAC_MODE_OPTIONS_IDX0,
            .dacSpeed = DRV_DAC_SPEED_MODE_IDX0,
            .interruptMode = DRV_DAC_INTERRUPT_MODE
        },

        //Channel 1 Initialization data
        {	
            .dacID = DACC_ID_0,
            .dacIndex = DRV_DAC_INST_IDX1, 
            .outputMode = DRV_DAC_OUTPUT_TYPE,
            .operationMode = DRV_DAC_MODE_OPTIONS_IDX1,
            .dacSpeed = DRV_DAC_SPEED_MODE_IDX1,
            .interruptMode = DRV_DAC_INTERRUPT_MODE
        }
    }; 
    
    objectHandle = DRV_DAC_Initialize(DRV_DAC_INDEX_0, 
                                      (SYS_MODULE_INIT*)&drvDACInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>
 
    
  Remarks:
    This function must be called before any other DAC function is called.
    This function should only be called once during system initialization.
    Build configuration options may be used to statically override options in 
    the "init" structure and will take precedence over initialization data 
    passed using this function.
*/
SYS_MODULE_OBJ DRV_DAC_Initialize(const SYS_MODULE_INDEX drvIndex, const SYS_MODULE_INIT  * const init);

// *****************************************************************************
/* Function:
    void DRV_DAC_Deinitialize(SYS_MODULE_OBJ object);

  Summary:
    Deinitializes the specified instance of the DAC driver module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the DAC driver module, disabling
    its operation (and any hardware).  
    Invalidates all the DAC registers.
	
  Precondition:
    Function DRV_DAC_Initialize should have been called before calling this
    function.

  Parameters:
    object - Driver object handle, returned from the DRV_DAC_Initialize routine.

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_DAC_Initialize
    SYS_STATUS          status;

    DRV_DAC_Deinitialize(object);

    status = DRV_DAC_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again. This
    routine will NEVER block waiting for hardware.
*/
void  DRV_DAC_Deinitialize(SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_DAC_Status(SYS_MODULE_OBJ object)

  Summary:
    Gets the current status of the DAC driver module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine provides the current status of the DAC driver module.

  Precondition:
    Function DRV_DAC_Initialize should have been called before calling this
    function.

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_DAC_Initialize routine

  Returns:
    SYS_STATUS_READY          - Indicates that the driver is busy with a
                                previous system level operation and cannot start
                                another

    SYS_STATUS_DEINITIALIZED  - Indicates that the driver has been
                                deinitialized

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //Returned from DRV_DAC_Initialize
    SYS_STATUS          dacStatus;

    dacStatus = DRV_DAC_Status(object);
    if (SYS_STATUS_READY == dacStatus)
    {
        // This means the driver is initialised        
    }
    </code>

  Remarks:
    A driver can be opened only when its status is SYS_STATUS_READY.
*/
SYS_STATUS DRV_DAC_Status(SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    void DRV_DAC_Tasks(SYS_MODULE_OBJ object);

  Summary:
    Maintains the driver's state machine and implements its ISR.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to maintain the driver's internal state machine and 
    implement its transmit ISR for interrupt-driven implementations.
	In polling mode, this function should be called from the SYS_Tasks()
	function. 
    In interrupt mode, this function should be called in the interrupt service 
    routine of DAC that is associated with this DAC	driver hardware instance.

  Precondition:
    The DRV_DAC_Initialize routine must have been called for the specified
    DAC driver instance.

  Parameters:
    object - Object handle for the specified driver instance (returned from
             DRV_DAC_Initialize)
  
  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //Returned from DRV_DAC_Initialize

    while(true)
    {
        DRV_DAC_Tasks (object);

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This function may execute in an ISR context and will never block or access
    any resources that may cause it to block.
*/						
void DRV_DAC_Tasks(SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_DAC_Open(const SYS_MODULE_INDEX index, 
                            const DRV_IO_INTENT ioIntent);

  Summary:
    Opens the specified DAC driver instance.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine opens the specified DAC driver instance.

  Precondition:
    Function DRV_DAC_Initialize must have been called before calling this
    function.
    
    DRV_DAC_Status must return the status as SYS_STATUS_READY.

  Parameters:
    index        - Identifier for the object instance 
    
    ioIntent     - Zero or more of the values from the enumeration
                   DRV_IO_INTENT "ORed" together to indicate the intended use
                   of the driver. See function description for details.

  Returns:
    If successful, the routine returns a valid open-instance handle.

  Example:
     <code>
     DRV_HANDLE handle;
     //Opens an Driver handle
     handle = DRV_DAC_Open(DRV_DAC_INDEX_0, 
                           DRV_IO_INTENT_EXCLUSIVE);
                            
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
        // May be the driver is not initialized or the initialization
        // is not complete.
    }
    </code>                       

  Remarks:
    The handle returned is valid until the DRV_DAC_Close routine is called.
    This routine will NEVER block waiting for hardware.If the requested intent
    flags are not supported, the routine will return DRV_HANDLE_INVALID.  

*/
DRV_HANDLE DRV_DAC_Open(const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent);

// *****************************************************************************
/* Function:
    void DRV_DAC_Close(DRV_Handle handle)

  Summary:
    Closes an opened-instance of the DAC driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine closes an opened-instance of the DAC driver, invalidating the
    handle.
    
  Precondition:
    DRV_DAC_Initialize and DRV_DAC_Open functions must have been called.

  Parameters:
    handle - A valid open-instance handle, returned from driver's open routine

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  //Returned from DRV_DAC_Open

    DRV_DAC_Close(handle);

    </code>
   
  Remarks:
    Usually there is no need for the client to verify that the Close operation
    has completed.  The driver will abort any ongoing operations when this
    routine is called. 
*/
void DRV_DAC_Close(const DRV_HANDLE handle);

// *****************************************************************************
/*
  Function:
    DRV_CLIENT_STATUS DRV_DAC_ClientStatus(DRV_HANDLE handle)

  Summary:
    Gets the current client-specific status the DAC driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the client-specific status of the DAC driver associated
    with the given handle. This function can be used to check the status of
    client after the DRV_DAC_Close() function has been called.

  Preconditions:
    DRV_DAC_Initialize and DRV_DAC_Open functions must have been called.

  Parameters:
    handle -  Handle returned from the driver's open function.

  Returns:
    A DRV_CLIENT_STATUS value describing the current status of the driver.

  Example:
     <code>
    DRV_HANDLE          handle;  //Returned from DRV_DAC_Open
    DRV_CLIENT_STATUS   status;

    status = DRV_DAC_ClientStatus(handle);
    if(DRV_CLIENT_STATUS_CLOSED != status)
    {
        // The client had not closed.
    }
    </code>
  
  Remarks:
    This function will not block for hardware access and will immediately return
    the current status.  
*/
DRV_CLIENT_STATUS DRV_DAC_ClientStatus(DRV_HANDLE handle);

// *****************************************************************************
/* Function:
    void DRV_DAC_DataWrite(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId,
                           uint32_t data);

  Summary:
    Writes the Digital data to Conversion Data Register for Analog conversion.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine writes the Digital data to Conversion Data Register for Analog 
    conversion.

  Precondition:
    DRV_DAC_Initialize and DRV_DAC_Open functions must have been called.

  Parameters:
    handle - Handle returned from the driver's open function.
  
    chId   - Identifies the Channel Instance 
    
    data   - The data to be transferred to analog value. 
    
  Returns:
    None
   
  Example:
    <code>
    #define SAMPLES (20)
    DRV_HANDLE              handle; //Returned from DRV_DAC_Open
                 
    const uint16_t data[SAMPLES] = {0x0, 0x080, 0x100, 0x17f, 0x1fd, 0x278, 
                                    0x2f1, 0x367, 0x3da, 0x449,0x4b3, 0x519, 
                                    0x579, 0x5d4, 0x629, 0x678, 0x6c0, 0x702,
                                    0x73c, 0x76f};
      
    for (uint8_t sample = 0;  sample < SAMPLES; sample++) 
    {
       // Check for DAC ready status bit for True and DRV_DAC_EVENT status for
          DRV_DAC_EVENT_READY before writing data 	
       DRV_DAC_DataWrite (handle, DRV_DAC_CHANNEL_INDEX_0, data[sample]);
    }
    </code>
  Remarks:
    This routine writes the data to "Conversion Data Register".
    Writing the data to "Conversion Dara Register". 
    DAC starts the Data conversion with any of the DAC channel is Enabled.
    
*/
void DRV_DAC_DataWrite(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId, uint32_t data);

// *****************************************************************************
/* Function:
    void DRV_DAC_EventHandlerSet(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId, 
                                 const void* eventHandler, 
                                 const uintptr_t context);

  Summary:
    Allows a client to identify a event handling function for the driver to call
    back to know the DAC Data conversion status.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function allows a client to identify a event handling function for the 
    driver to callback to know the DAC Data conversion status.
    The driver will pass this handle back to client by calling "eventHandler" 
    function when the Data Conversion is completed.

  Precondition:
    DRV_DAC_Initialize and DRV_DAC_Open functions must have been called.

  Parameters:
    handle       - Handle returned from the driver's open function.
 
    chId         - Identifies the Channel Instance 

    eventHandler - Pointer to the event handler function.
    
    context      - The value of parameter will be passed back to the client
                   unchanged, when the eventHandler function is called.  It can
                   be used to identify any client specific data object that
                   identifies the instance of the client module (for example,
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Example:
    <code>
    MY_APP_OBJ myAppObj;
    DRV_Handle handle;
    
    void APP_DRV_DAC_EventHandler(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId, 
                                  DRV_DAC_EVENT event, 
                                  uintptr_t contextHandle)
    {   
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler. 
           MY_APP_OBJ myAppObj = (MY_APP_OBJ *) contextHandle; 
        //Application related tasks
    }

                           
    DRV_DAC_EventHandlerSet(handle, DRV_DAC_CHANNEL_INDEX_0,
                            APP_DRV_DAC_EventHandler, 
                            (uintptr_t)&myAppObj);                       
    </code>
    
  Remarks:
    None
*/
void DRV_DAC_EventHandlerSet(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId, const void* eventHandler, const uintptr_t context);

// *****************************************************************************
/* Function:
    DRV_DAC_EVENT DRV_DAC_EventStatusGet(DRV_HANDLE handle, 
                                         DRV_DAC_CHANNEL_INDEX chId);

  Summary:
    Allows a client to to get the status of the previous data conversion.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function allows get the status of the previous data conversion.
    This helps to keep track of the Status of the Event in Polled mode.

  Precondition:
    DRV_DAC_Initialize and DRV_DAC_Open functions must have been called.

  Parameters:
    handle - Handle returned from the driver's open function.

    chId   - Identifies the Channel Instance   
  
  Returns:
    Return the Event status of type DRV_DAC_EVENT .

  Example:
    <code>
    DRV_HANDLE handle;
    DRV_DAC_EVENT status;
    
    status = DRV_DAC_EventStatusGet(handle, DRV_DAC_CHANNEL_INDEX_0);
    
    while (status = DRV_DAC_EVENT_READY)
    {
        //DAC is ready to accept new conversion requests
    }
    </code>
    
  Remarks:
    None
*/
DRV_DAC_EVENT DRV_DAC_EventStatusGet(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId);

// *****************************************************************************
/* DAC Driver Event Handle Function pointer

  Summary:
    Pointer to a DAC Driver Event handler function
    <p><b>Implementation:</b> Dynamic</p>
    
  Description:
    This data type defines the required function signature for the DAC driver
    event handling callback function. 
    A client must register a pointer to a event handling function whose function
    signature (parameter and return value types) match the types specified by 
    this function pointer in order to receive event callback from the driver.

    The parameters and return values and are described here and a partial 
    example implementation is provided.

  Parameters:
    handle  - Handle returned from the driver's open function.
 
    chId   - Identifies the Channel Instance 
    
    event   - Identifies the type of event
    
    context - Value identifying the context of the application that registered
              the event handling function.

  Returns:
    None.
    
   Example:
    <code>
    void APP_DRV_DAC_EventHandler(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId,
                                  DRV_DAC_EVENT event,
                                  uintptr_t context)
    {   
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;
        switch(event)
        {
            case DRV_DAC_EVENT_DAC_READY:
                // DAC has completed the last conversion.
                break;

            case DRV_DAC_EVENT_PROCESSING:
            
            default:
               // Handle error.
                break;
        }
    }
    </code>
    
  Remarks:
    If the event is DRV_DAC_EVENT_DAC_READY, it means that DAC is ready 
    to accept new conversion request.

    If the event is DRV_DAC_EVENT_PROCESSING, it means that previous 
    conversion has not completed.
    
    The event handler function executes when driver is configured for 
    interrupt mode operation. It is recommended for the application to not 
    perform process intensive or blocking operations with in this function.

*/
typedef void (*DRV_DAC_EVENT_HANDLE)(DRV_HANDLE handle, DRV_DAC_CHANNEL_INDEX chId, DRV_DAC_EVENT event, uintptr_t context);
	
// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END
        
#endif // #ifndef _DRV_DAC_H
/*******************************************************************************
 End of File
*/
