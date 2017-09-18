/*******************************************************************************
  I2C Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_static.h

  Summary:
    I2C driver interface declarations for the static single instance driver.

  Description:
    The I2C device driver provides a simple interface to manage the I2C
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the I2C driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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

#ifndef _DRV_I2C_STATIC_H
#define _DRV_I2C_STATIC_H

#include "driver/i2c/src/drv_i2c_static_local_pic32c.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  

    extern "C" {

#endif
// DOM-IGNORE-END  

<#macro DRV_I2C_STATIC_API DRV_INSTANCE OPERATION_MODE>
// *********************************************************************************************
// *********************************************************************************************
// Section: System Interface Headers for the Instance ${DRV_INSTANCE} of I2C static driver
// *********************************************************************************************
// *********************************************************************************************
SYS_MODULE_OBJ DRV_I2C${DRV_INSTANCE}_Initialize(void);
void  DRV_I2C${DRV_INSTANCE}_Deinitialize(void);
SYS_STATUS DRV_I2C${DRV_INSTANCE}_Status(void);

// *********************************************************************************************
// *********************************************************************************************
// Section: General Client Interface Headers for the Instance ${DRV_INSTANCE} of I2C static driver
// *********************************************************************************************
// *********************************************************************************************
DRV_HANDLE DRV_I2C${DRV_INSTANCE}_Open(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT ioIntent);
void DRV_I2C${DRV_INSTANCE}_Close ( void );
void DRV_I2C${DRV_INSTANCE}_BufferEventHandlerSet ( const DRV_I2C_BUFFER_EVENT_HANDLER eventHandler,
                                      const uintptr_t context );
uint32_t DRV_I2C${DRV_INSTANCE}_BytesTransferred ( DRV_I2C_BUFFER_HANDLE bufferHandle );
DRV_I2C_BUFFER_EVENT DRV_I2C${DRV_INSTANCE}_TransferStatusGet ( DRV_I2C_BUFFER_HANDLE bufferHandle );
DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Receive ( uint16_t   slaveAddress,
                                         void *     buffer,
                                         size_t     size,
                                         void *     callbackContext );
DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_Transmit ( uint16_t   slaveAddress,
                                          void *     buffer,
                                          size_t     size,
                                          void *     context );
void DRV_I2C${DRV_INSTANCE}_Tasks ( void );
<#if OPERATION_MODE == "DRV_I2C_MODE_MASTER">
DRV_I2C_BUFFER_HANDLE DRV_I2C${DRV_INSTANCE}_TransmitThenReceive ( uint16_t   slaveAddress,
                                                     void *     writeBuffer,
                                                     size_t     writeSize,
                                                     void *     readBuffer,
                                                     size_t     readSize,
                                                     void *     callbackContext );
void DRV_I2C${DRV_INSTANCE}_QueueFlush (void);
<#elseif OPERATION_MODE == "DRV_I2C_MODE_SLAVE">
void DRV_I2C${DRV_INSTANCE}_SlaveCallbackSet (const DRV_I2C_CallBack callback, const uintptr_t context);
</#if>
</#macro>
<#if CONFIG_DRV_I2C_INST_IDX0 == true>
<@DRV_I2C_STATIC_API 
DRV_INSTANCE="0"
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX0 />
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
<@DRV_I2C_STATIC_API 
DRV_INSTANCE="1"
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX1 />
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
<@DRV_I2C_STATIC_API 
DRV_INSTANCE="2"
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX2 />
</#if>
// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif // #ifndef _DRV_I2C_STATIC_H

/*******************************************************************************
 End of File
*/
