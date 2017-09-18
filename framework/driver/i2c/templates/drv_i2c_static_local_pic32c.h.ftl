/*******************************************************************************
  PIC32C I2C Driver Local Data Structures.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_local_pic32c.h

  Summary:
    PIC32C I2C driver local declarations and definitions.

  Description:
    This file contains the PIC32C I2C driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_I2C_STATIC_LOCAL_PIC32C_H
#define _DRV_I2C_STATIC_LOCAL_PIC32C_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/i2c/drv_i2c.h"
#include "system/debug/sys_debug.h"
#include "driver/i2c/src/drv_i2c_variant_mapping_pic32c.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* I2C Master Maximum Baud rate

  Summary:
    Defines the maximum baud rate for Master mode of the I2C Driver.

  Description:
    This define gives the maximum baud rate for the Master mode of the 
    I2C driver.

  Remarks:
    Used by the _DRV_I2C_BaudRateSet function.
*/

#define DRV_I2C_MASTER_MAX_BAUDRATE            400000U

// *****************************************************************************
/* I2C low level of clock time limit

  Summary:
    Defines the I2C low level clock time limit.

  Description:
    This define gives the I2C low level clock time limit to 1.3 us.

  Remarks:
    Used by the _DRV_I2C_BaudRateSet function.
*/

#define DRV_I2C_LOW_LEVEL_TIME_LIMIT           384000U

// *****************************************************************************
/* I2C clock divider

  Summary:
    Defines the I2C clock divider value.

  Description:
    This define gives the I2C clock divider value.

  Remarks:
    Used by the _DRV_I2C_BaudRateSet function.
*/

#define DRV_I2C_CLK_DIVIDER                    2U

// *****************************************************************************
/* I2C clock calculation argument

  Summary:
    Defines the I2C clock calculation argument.

  Description:
    This define gives the I2C clock calculation argument value.

  Remarks:
    Used by the _DRV_I2C_BaudRateSet function.
*/

#define DRV_I2C_CLK_CALC_ARGU                  3U

// *****************************************************************************
/* I2C clock divider maximum value

  Summary:
    Defines the I2C clock divider maximum value.

  Description:
    This define gives the I2C clock divider maximum value.

  Remarks:
    Used by the _DRV_I2C_BaudRateSet function.
*/

#define DRV_I2C_CLK_DIV_MAX                    0xFFU

// *****************************************************************************
/* I2C clock divider minimum value

  Summary:
    Defines the I2C clock divider minimum value.

  Description:
    This define gives the I2C clock divider minimum value.

  Remarks:
    Used by the _DRV_I2C_BaudRateSet function.
*/

#define DRV_I2C_CLK_DIV_MIN                    7U

// *****************************************************************************
/* I2C 7 bit address upper limit

  Summary:
    Defines the I2C 7 bit address upper limit.

  Description:
    This define gives the I2C 7 bit address upper limit.

  Remarks:
    None.
*/

#define DRV_I2C_7BIT_ADDR_UPPER_LIMIT          0xFFU

// *****************************************************************************
/* I2C device internal address size

  Summary:
    Defines the I2C device internal address size.

  Description:
    This define gives the I2C device internal address size.

  Remarks:
    None.
*/

#define DRV_I2C_INTERNAL_ADDRESS_SIZE          0x03U

// *****************************************************************************
/* I2C Driver False Flag

  Summary:
    Defines the I2C Driver False Flag.

  Description:
    This define gives the I2C Driver False Flag.

  Remarks:
    None.
*/

#define DRV_I2C_FALSE                           0x0U

// *****************************************************************************
/* I2C Driver True Flag

  Summary:
    Defines the I2C Driver True Flag.

  Description:
    This define gives the I2C Driver True Flag.

  Remarks:
    None.
*/

#define DRV_I2C_TRUE                            0x1U

// *****************************************************************************
/* I2C Buffer Object Flags

  Summary:
    Defines the I2C Buffer Object Flags.

  Description:
    This enumeration defines the I2C Buffer Object flags.

  Remarks:
    None.
*/

typedef enum
{
    /* Indicates this buffer was submitted by a read function */
    DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_READ = 1 << 0,

    /* Indicates this buffer was submitted by a write function */            
    DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_WRITE = 1 << 1,
            
    /* Indicates this buffer operation is forced */
    DRV_I2C_BUFFER_OBJ_FLAG_BUFFER_FORCED = 1 << 2,
            
} DRV_I2C_BUFFER_OBJ_FLAGS;

// *****************************************************************************
/* I2C Buffer Object State

  Summary:
    Defines the I2C Buffer Object State.

  Description:
    This enumeration defines the I2C Buffer Object State.

  Remarks:
    None.
*/

typedef enum
{
    /* Indicates this buffer is free */
    DRV_I2C_BUFFER_IS_FREE = 0,
            
    /* Indicates this buffer is in queue */
    DRV_I2C_BUFFER_IS_IN_QUEUE,
            
} DRV_I2C_BUFFER_STATE;

// *****************************************************************************
/* I2C Data Object Task State

  Summary:
    Defines the I2C data object task state.

  Description:
    This enumeration defines the I2C data object task state.

  Remarks:
    None.
*/

typedef enum
{
    /* Indicates task state error */
    DRV_I2C_DATA_OBJ_TASK_ERROR = -1,
      
    /* Indicates task state address send (only master mode)*/
    DRV_I2C_DATA_OBJ_TASK_ADDRESS_SEND = 0,
            
    /* Indicates task state address read (only slave mode)*/
    DRV_I2C_DATA_OBJ_TASK_ADDRESS_RECEIVED,
            
    /* Indicates task state start slave operation (only slave mode)*/
    DRV_I2C_DATA_OBJ_TASK_SLAVE_OPERATION_START,
            
    /* Indicates task state write transfer (master/slave mode)*/
    DRV_I2C_DATA_OBJ_TASK_TRANSFER_WRITE,
            
    /* Indicates task state read transfer (master/slave mode)*/
    DRV_I2C_DATA_OBJ_TASK_TRANSFER_READ,
            
    /* Indicates task state write transfer (master/slave mode)*/
    DRV_I2C_DATA_OBJ_TASK_TRANSFER_DUMMY_WRITE,
            
    /* Indicates task state read transfer (master/slave mode)*/
    DRV_I2C_DATA_OBJ_TASK_TRANSFER_DUMMY_READ,
            
    /* Indicates task state transfer complete (only master mode)*/
    DRV_I2C_DATA_OBJ_TASK_TRANSFER_COMPLETE,
    
    /* Indicates task state done (master/slave mode)*/
    DRV_I2C_DATA_OBJ_TASK_DONE,
            
} DRV_I2C_DATA_OBJ_TASK_STATE;

// *****************************************************************************
/* I2C Client-Specific Driver Error

  Summary:
    Defines the client-specific error of the I2C driver.

  Description:
    This enumeration defines the client-specific error codes of the I2C
    driver.

  Remarks:
    Returned by the DRV_I2C_ClientStatus function.
*/

typedef enum
{
    /* Indicates no error */
    DRV_I2C_ERROR_NONE,
            
} DRV_I2C_ERROR;

// *****************************************************************************
/* I2C Driver Buffer Object

  Summary:
    Object used to keep track of a client's buffer.

  Description:
    This object is used to keep track of a client's buffer in the driver's 
    queue.

  Remarks:
    None.
*/

typedef struct _DRV_I2C_BUFFER_OBJ
{
    /* Driver instance to which the buffer object belongs to */
    uint8_t drvInstance;

    /* This flag tracks whether this object is in use */
    bool inUse;

    /* Pointer to the application read buffer */
    uint8_t * readBuffer;
    
    /* Pointer to the application write buffer */
    uint8_t * writeBuffer;
   
    /* Slave Address High Byte */
    uint8_t slave7BitAddress;

    /* Slave internal address */
    uint8_t internalAddress[3];    
    
    /* Slave internal address size */
    uint8_t internalAddrSize;
    
    /* Tracks how much data has been transferred */
    size_t nCurrentBytes;

    /* Number of bytes to be written */
    size_t writeSize;
    
    /* Number of bytes to be read */
    size_t readSize;

    /* Next buffer pointer */
    struct _DRV_I2C_BUFFER_OBJ * next;

    /* Previous buffer pointer */
    struct _DRV_I2C_BUFFER_OBJ * previous;

    /* Flags that indicate the type of buffer */
    DRV_I2C_BUFFER_OBJ_FLAGS flags;

    /* Current state of the buffer */
    DRV_I2C_BUFFER_STATE currentState;

    /* Buffer event */
    DRV_I2C_BUFFER_EVENT event;
    
    /* ignore any error events */
    DRV_I2C_BUS_ERROR_EVENT errorEvent;

} DRV_I2C_BUFFER_OBJ;

// *****************************************************************************
/* I2C Driver Instance Object

  Summary:
    Object used to keep any data required for an instance of the I2C driver.

  Description:
    This object is used to keep track of any data that must be maintained to 
    manage a single instance of the I2C driver.

  Remarks:
    None.
*/

typedef struct
{
    /* The module index associated with the object*/
    twi_registers_t *                        i2cModule;

    /* The status of the driver */
    SYS_STATUS                                 status;
   
    /* Flag to indicate this object is in use  */
    bool                                       inUse;

    /* Flag to indicate that driver has been opened exclusively. */
    bool                                       isExclusive;
    
    /* Operation Mode */
    DRV_I2C_MODE                               operationMode;
    
    /* Interrupt Source */
    INT_SOURCE                                 interruptSource;

    /* Keeps track if the driver is in interrupt context
       and if so the nesting levels. */
    uint32_t                                   interruptNestingCount;

    /* This callback is fired when an operation is about to start on the
       I2C bus.  This allows the user to set any pins that need to be set.
       This callback may be called from an ISR so should not include OSAL
       calls.  The context parameter is the same one passed into the
       BufferAddRead, BufferAddWrite, BufferAddWriteRead function.
    */
    
    DRV_I2C_CallBack                           operationStarting;
    
    void *                                     callbackContext;
<#if CONFIG_USE_3RDPARTY_RTOS>
    /* Hardware instance mutex */
    OSAL_MUTEX_DECLARE                       (mutexDriverInstance);
</#if>
    /* state of the Task */
    DRV_I2C_DATA_OBJ_TASK_STATE                taskState;
            
    /* Queue size */
    size_t                                     queueSize;
    
    /* Current read queue size */
    size_t                                     queueSizeCurrent;
    
    /* The buffer Q for the read write operations */
    DRV_I2C_BUFFER_OBJ *                       queue;

    /* Event handler for this function */
    DRV_I2C_BUFFER_EVENT_HANDLER               eventHandler;

    /* Application Context associated with this client */
    uintptr_t                                  context;

} DRV_I2C_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: Local functions.
// *****************************************************************************
// *****************************************************************************
<#macro DRV_I2C_STATIC_VALUES DRV_INSTANCE>
bool _DRV_I2C${DRV_INSTANCE}_HardwareInitialize(void);
bool _DRV_I2C${DRV_INSTANCE}_BaudRateSet(void);
void _DRV_I2C${DRV_INSTANCE}_Enable(void);
</#macro>
<#if CONFIG_DRV_I2C_INST_IDX0 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="0"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="1"/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="2"/>
</#if>
void _DRV_I2C_ResetAndDisable(twi_registers_t * i2cModule);
uint32_t _DRV_I2C_MakeInternalAddress(uint8_t *address, uint32_t length);

#endif //#ifndef _DRV_I2C_LOCAL_PIC32C_H

/*******************************************************************************
 End of File
*/

