/*******************************************************************************
  I2C Driver Interface Mapping Dynamic APIs to Static APIs           

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_mapping.h

  Summary:
    This file allows maintaining a single set of APIs for all I2C transactions  
    by making the type of implementation transparent to the application. In case
    where static implementation of I2C driver is selected, this file maps the 
    API functions to a particular driver instance-specific static implementation
    function, eliminating unnecessary dynamic parameters. 
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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

#include "system_config.h"
#include "system_definitions.h"

SYS_MODULE_OBJ DRV_I2C_Initialize(const SYS_MODULE_INDEX index,const SYS_MODULE_INIT * const init)
{
    SYS_MODULE_OBJ returnValue;

    switch(index)
    {
        case DRV_I2C_INDEX_0:
        {
            returnValue = DRV_I2C0_Initialize();
            break;
        }
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            returnValue = DRV_I2C1_Initialize();
            break;
        }
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            returnValue = DRV_I2C2_Initialize();
            break;
        }
</#if>
        default:
        {
            returnValue = SYS_MODULE_OBJ_INVALID;
            break;
        }
    }

    return returnValue;
}

void DRV_I2C_Deinitialize( SYS_MODULE_OBJ object)
{
    switch(object)
    {
        case DRV_I2C_INDEX_0:
        {
            DRV_I2C0_Deinitialize();
            break;
        }
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            DRV_I2C1_Deinitialize();
            break;
        }
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            DRV_I2C2_Deinitialize();
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
}

SYS_STATUS DRV_I2C_Status (SYS_MODULE_OBJ object)
{
    SYS_STATUS returnValue;

    switch(object)
    {
        case DRV_I2C_INDEX_0:
        {
            returnValue = DRV_I2C0_Status();
            break;
        }
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            returnValue = DRV_I2C1_Status();
            break;
        }
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            returnValue = DRV_I2C2_Status();
            break;
        }
</#if>
        default:
        {
            returnValue = SYS_STATUS_ERROR;
            break;
        }
    }

    return returnValue;
}

DRV_HANDLE DRV_I2C_Open ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent )
{
    switch (index)
    {
      <#if CONFIG_DRV_I2C_INST_IDX0 == true>
        case DRV_I2C_INDEX_0:
        {
            return (DRV_HANDLE)DRV_I2C_INDEX_0;
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            return (DRV_HANDLE)DRV_I2C_INDEX_1;
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            return (DRV_HANDLE)DRV_I2C_INDEX_2;
            break;
        }
      </#if>
        default:
        {
            return DRV_HANDLE_INVALID;
            break;
        }
    }
}

void DRV_I2C_Close( const DRV_HANDLE handle)
{
    uintptr_t instance;

    instance = handle & 0x00FF;

    switch(instance)
    {
        case DRV_I2C_INDEX_0:
        {
            DRV_I2C0_Close();
            break;
        }
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            DRV_I2C1_Close();
            break;
        }
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            DRV_I2C2_Close();
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
}

DRV_I2C_BUFFER_HANDLE DRV_I2C_Transmit( DRV_HANDLE drvhandle, uint16_t deviceaddress,
                                        void *txBuffer, size_t size, void * context)
{    
    switch (drvhandle)
    {
        case DRV_I2C_INDEX_0:
        {
            return ( DRV_I2C0_Transmit (deviceaddress, txBuffer, size, context) );
            break;
        }
      <#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            return ( DRV_I2C1_Transmit (deviceaddress, txBuffer, size, context) );
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            return ( DRV_I2C2_Transmit (deviceaddress, txBuffer, size, context) );
            break;
        }
      </#if>
        default:
        {
            return (DRV_I2C_BUFFER_HANDLE) NULL;
        }
    }   
}

DRV_I2C_BUFFER_HANDLE DRV_I2C_Receive ( DRV_HANDLE drvhandle, uint16_t deviceaddress, 
                                        void *rxBuffer, size_t size, void * context)
{
    switch (drvhandle)
    {
        case DRV_I2C_INDEX_0:
        {
            return ( DRV_I2C0_Receive(deviceaddress, rxBuffer, size, context) );
            break;
        }
      <#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            return ( DRV_I2C1_Receive(deviceaddress, rxBuffer, size, context) );
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            return ( DRV_I2C2_Receive(deviceaddress, rxBuffer, size, context) );
            break;
        }
      </#if>
        default:
        {
            return (DRV_I2C_BUFFER_HANDLE) NULL;
        }
    }
}

DRV_I2C_BUFFER_HANDLE DRV_I2C_TransmitThenReceive ( DRV_HANDLE drvhandle, uint16_t deviceaddress,
                                                    void *txBuffer, size_t wsize, 
                                                    void *rxBuffer, size_t rsize, void * context)
{
    switch (drvhandle)
    {
      <#if CONFIG_DRV_I2C_INST_IDX0 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX0 == "DRV_I2C_MODE_MASTER">
        case DRV_I2C_INDEX_0:
        {
            return ( DRV_I2C0_TransmitThenReceive(deviceaddress, txBuffer, wsize, rxBuffer, rsize, context) );
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX1 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX1 == "DRV_I2C_MODE_MASTER">
        case DRV_I2C_INDEX_1:
        {
            return ( DRV_I2C1_TransmitThenReceive(deviceaddress, txBuffer, wsize, rxBuffer, rsize, context) );
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX2 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX2 == "DRV_I2C_MODE_MASTER">
        case DRV_I2C_INDEX_2:
        {
            return ( DRV_I2C2_TransmitThenReceive(deviceaddress, txBuffer, wsize, rxBuffer, rsize, context) );
            break;
        }
      </#if>
        default:
        {
            return (DRV_I2C_BUFFER_HANDLE) NULL;
        }
    }
}

DRV_I2C_BUFFER_EVENT DRV_I2C_TransferStatusGet (  DRV_HANDLE drvhandle, DRV_I2C_BUFFER_HANDLE bufferHandle )
{
    switch (drvhandle)
    {
        case DRV_I2C_INDEX_0:
        {
            return ( DRV_I2C0_TransferStatusGet(bufferHandle) );
            break;
        }
        <#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            return ( DRV_I2C1_TransferStatusGet(bufferHandle) );
            break;
        }
        </#if>
        <#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            return ( DRV_I2C2_TransferStatusGet(bufferHandle) );
            break;
        }
        </#if>
        default:
        {
            return (DRV_I2C_BUFFER_EVENT) NULL;
        }
    }
}

uint32_t DRV_I2C_BytesTransferred (  DRV_HANDLE drvhandle,  DRV_I2C_BUFFER_HANDLE bufferHandle )
{
    switch (drvhandle)
    {
        case DRV_I2C_INDEX_0:
        {
            return ( DRV_I2C0_BytesTransferred(bufferHandle) );
            break;
        }
      <#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            return (DRV_I2C1_BytesTransferred(bufferHandle) );
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            return ( DRV_I2C2_BytesTransferred(bufferHandle) );
            break;
        }
      </#if>
        default:
        {
            return 0;
        }
    }
}

void DRV_I2C_BufferEventHandlerSet ( const DRV_HANDLE drvhandle,
                                     const DRV_I2C_BUFFER_EVENT_HANDLER eventHandler,
                                     const uintptr_t context )
{
    switch (drvhandle)
    {
        case DRV_I2C_INDEX_0:
        {
            DRV_I2C0_BufferEventHandlerSet (eventHandler,context);
            break;
        }
      <#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            DRV_I2C1_BufferEventHandlerSet (eventHandler,context);
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            DRV_I2C2_BufferEventHandlerSet (eventHandler,context);
            break;
        }
      </#if>
        default:
        {
            break;
        }
    }
}

void DRV_I2C_Tasks ( SYS_MODULE_OBJ object )
{
    switch (object)
    {
        case DRV_I2C_INDEX_0:
        {
            DRV_I2C0_Tasks();
            break;
        }
      <#if CONFIG_DRV_I2C_INST_IDX1 == true>
        case DRV_I2C_INDEX_1:
        {
            DRV_I2C1_Tasks();
            break;
        }
      </#if>
      <#if CONFIG_DRV_I2C_INST_IDX2 == true>
        case DRV_I2C_INDEX_2:
        {
            DRV_I2C2_Tasks();
            break;
        }
      </#if>
        default:
        {
            break;
        }
    }
}

void DRV_I2C_QueueFlush ( DRV_HANDLE handle )
{
    uintptr_t instance;

    instance = handle & 0x00FF;

    switch(instance)
    {
<#if CONFIG_DRV_I2C_INST_IDX0 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX0 == "DRV_I2C_MODE_MASTER">
        case DRV_I2C_INDEX_0:
        {
            DRV_I2C0_QueueFlush();
            break;
        }
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX1 == "DRV_I2C_MODE_MASTER">
        case DRV_I2C_INDEX_1:
        {
            DRV_I2C1_QueueFlush();
            break;
        }
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX2 == "DRV_I2C_MODE_MASTER">
        case DRV_I2C_INDEX_2:
        {
            DRV_I2C2_QueueFlush();
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
}

void DRV_I2C_SlaveCallbackSet ( const DRV_HANDLE handle, const DRV_I2C_CallBack callback,
                                const uintptr_t context )
{
    uintptr_t instance;

    instance = handle & 0x00FF;

    switch(instance)
    {
<#if CONFIG_DRV_I2C_INST_IDX0 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX0 == "DRV_I2C_MODE_SLAVE">
        case DRV_I2C_INDEX_0:
        {
            DRV_I2C0_SlaveCallbackSet(callback, context);
            break;
        }
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX1 == "DRV_I2C_MODE_SLAVE">
        case DRV_I2C_INDEX_1:
        {
            DRV_I2C1_SlaveCallbackSet(callback, context);
            break;
        }
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true && CONFIG_DRV_I2C_OPERATION_MODE_IDX2 == "DRV_I2C_MODE_SLAVE">
        case DRV_I2C_INDEX_2:
        {
            DRV_I2C2_SlaveCallbackSet(callback, context);
            break;
        }
</#if>
        default:
        {
            break;
        }
    }
}

/*******************************************************************************
 End of File
*/

