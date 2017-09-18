/*******************************************************************************
  I2C Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_i2c_static_pic32c.c
	
  Summary:
    I2C driver impementation for the static single instance driver.

  Description:
    The I2C device driver provides a simple interface to manage the I2C
    modules on Microchip microcontrollers. This file contains implemenation
    for the I2C driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
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
#include "system_config.h"
#include "system_definitions.h"

<#macro DRV_I2C_STATIC_VALUES DRV_INSTANCE HARDWARE_INSTANCE OPERATION_MODE 
I2C_INT_SRC BAUD_RATE SLAVE_ADDR SLAVE_ADDR_MASK>
<#if OPERATION_MODE == "DRV_I2C_MODE_SLAVE">
<#if DRV_INSTANCE == "0">
<#assign HIGH_SPEED = CONFIG_DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX0>
<#assign CLOCK_STRETCH = CONFIG_DRV_I2C_CLOCK_STRETCH_IDX0>
<#elseif DRV_INSTANCE == "1">
<#assign HIGH_SPEED = CONFIG_DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX1>
<#assign CLOCK_STRETCH = CONFIG_DRV_I2C_CLOCK_STRETCH_IDX1>
<#elseif DRV_INSTANCE == "2">
<#assign HIGH_SPEED = CONFIG_DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX2>
<#assign CLOCK_STRETCH = CONFIG_DRV_I2C_CLOCK_STRETCH_IDX2>
</#if>
</#if>
/* This is the driver static object . */
DRV_I2C_OBJ  gDrvI2C${DRV_INSTANCE}Obj ;

/* Global variable to access hardware instance */
<#if HARDWARE_INSTANCE == "TWI_ID_0">
    <#assign I2C_MODULE = "TWI0_Module">
static twi_registers_t *TWI0_Module = (twi_registers_t *)TWI_ID_0;
<#elseif HARDWARE_INSTANCE == "TWI_ID_1">
    <#assign I2C_MODULE = "TWI1_Module">
static twi_registers_t *TWI1_Module = (twi_registers_t *)TWI_ID_1;
<#elseif HARDWARE_INSTANCE == "TWI_ID_2">
    <#assign I2C_MODULE = "TWI2_Module">
static twi_registers_t *TWI2_Module = (twi_registers_t *)TWI_ID_2;
</#if>


// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_I2C${DRV_INSTANCE}_Initialize(void)

  Summary:
    Static implementation of DRV_I2C${DRV_INSTANCE}_Initialize system interface function.

  Description:
    This is the dynamic implementation of DRV_I2C${DRV_INSTANCE}_Initialize system interface
    function.

  Remarks:
    See drv_i2c_static.h for usage information.
*/

SYS_MODULE_OBJ DRV_I2C${DRV_INSTANCE}_Initialize(void)
{
    DRV_I2C_OBJ *dObj = &gDrvI2C${DRV_INSTANCE}Obj;
    
    /* Disable the I2C Module */
    _DRV_I2C_ResetAndDisable( ${I2C_MODULE} );

    dObj->isExclusive       = false;
<#if OPERATION_MODE == "DRV_I2C_MODE_SLAVE">
    dObj->operationStarting = NULL;
    dObj->taskState         = DRV_I2C_DATA_OBJ_TASK_ADDRESS_RECEIVED;
</#if>
<#if OPERATION_MODE == "DRV_I2C_MODE_MASTER">
    dObj->interruptNestingCount = 0;
    dObj->queueSize             = 0;
    dObj->queueSizeCurrent      = 0;
    dObj->queue                 = NULL;
    dObj->taskState             = DRV_I2C_DATA_OBJ_TASK_ADDRESS_SEND;
</#if>
    /* Initialize the I2C Module */
    _DRV_I2C${DRV_INSTANCE}_HardwareInitialize(  );
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
    /* Enable the interrupt the source in case of interrupt mode */
    SYS_INT_SourceEnable(${I2C_INT_SRC});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
    /* Create the hardware instance mutex. */
    if(OSAL_MUTEX_Create(&(drvObj->mutexDriverInstance)) != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }
</#if>
    /* Enable the I2c Module */
    _DRV_I2C${DRV_INSTANCE}_Enable( );
    
    /* Return the object structure */
    return ( (SYS_MODULE_OBJ)DRV_I2C_INDEX_${DRV_INSTANCE} );
}

// *****************************************************************************
/* Function:
    void DRV_I2C${DRV_INSTANCE}_Deinitialize(void)

  Summary:
    Dynamic implementation of DRV_I2C${DRV_INSTANCE}_Deinitialize system 
    interface function.

  Description:
    This is the dynamic implementation of DRV_I2C${DRV_INSTANCE}_Deinitialize 
    system interface function.

  Remarks:
    See drv_i2c_static.h for usage information.
*/

void  DRV_I2C${DRV_INSTANCE}_Deinitialize(void)
{
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
    /* Disable the interrupt */
    SYS_INT_SourceDisable(${I2C_INT_SRC}) ;
</#if>    

    /* Disable I2C module */
    _DRV_I2C_ResetAndDisable(${I2C_MODULE});

<#if CONFIG_USE_3RDPARTY_RTOS>
    /* Deallocate all mutexes */
    if(OSAL_MUTEX_Delete(&(dObj->mutexDriverInstance)) != OSAL_RESULT_TRUE)
    {
        return;
    }
</#if>
    
    return;
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_I2C${DRV_INSTANCE}_Status( void )

  Summary:
    Dynamic implementation of DRV_I2C${DRV_INSTANCE}_Status system interface 
    function.

  Description:
    This is the dynamic implementation of DRV_I2C${DRV_INSTANCE}_Status system 
    interface function.

  Remarks:
    See drv_i2c_static.h for usage information.
*/

SYS_STATUS DRV_I2C${DRV_INSTANCE}_Status( void )
{
    /* Return the status as ready always */
    return SYS_STATUS_READY;
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_I2C${DRV_INSTANCE}_Open( const SYS_MODULE_INDEX index,
                                            const DRV_IO_INTENT ioIntent )

  Summary:
    Dynamic implementation of DRV_I2C${DRV_INSTANCE}_Open client interface 
    function.

  Description:
    This is the dynamic implementation of DRV_I2C${DRV_INSTANCE}_Open client 
    interface function.

  Remarks:
    See drv_i2c_static.h for usage information.
*/

DRV_HANDLE DRV_I2C${DRV_INSTANCE}_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
   /* Return the driver instance value*/
    return ((DRV_HANDLE)DRV_I2C_INDEX_${DRV_INSTANCE});
}

// *****************************************************************************
/* Function:
    void DRV_I2C${DRV_INSTANCE}_Close ( void )

  Summary:
    Dynamic implementation of DRV_I2C${DRV_INSTANCE}_Close client interface function.

  Description:
    This is the dynamic implementation of DRV_I2C${DRV_INSTANCE}_Close client interface
    function.

  Remarks:
    See drv_i2c.h for usage information.
*/

void DRV_I2C${DRV_INSTANCE}_Close ( void )
{
    return;
}

// *****************************************************************************
/* Function:
    void DRV_I2C${DRV_INSTANCE}_BufferEventHandlerSet
    (
        const DRV_HANDLE hClient,
        const DRV_I2C_BUFFER_EVENT_HANDLER eventHandler,
        const uintptr_t context
    )

  Summary:
    Dynamic implementation of DRV_I2C_BufferEventHandlerSet client interface
    function.

  Description:
    This is the dynamic implementation of DRV_I2C_BufferEventHandlerSet
    client interface function.

  Remarks:
    See drv_i2c_static.h for usage information.
*/

void DRV_I2C${DRV_INSTANCE}_BufferEventHandlerSet
(
    const DRV_I2C_BUFFER_EVENT_HANDLER eventHandler,
    const uintptr_t context
)
{
    DRV_I2C_OBJ * dObj = (DRV_I2C_OBJ *) NULL;

    dObj = &gDrvI2C${DRV_INSTANCE}Obj;

    /* Register the event handler */
    dObj->eventHandler = eventHandler;
    dObj->context = context;
}

// *****************************************************************************
/* Function:
    void DRV_I2C${DRV_INSTANCE}_QueueFlush ( DRV_HANDLE handle )

  Summary:
    The existing transactions in the queue are voided and the queue   
    pointers are reset to their initial state. This renders the queue
    empty.

  Description:
    The existing transactions in the queue are voided and the queue   
    pointers are reset to their initial state. This renders the queue
    empty.

  Parameters:
    handle          -  A valid open-instance handle, returned from the driver's 
                       open routine 

  Returns:
    None
  
  Remarks:
    See drv_i2c_static.h for usage information.

*/
void DRV_I2C${DRV_INSTANCE}_QueueFlush ( void )
{
    DRV_I2C_OBJ * dObj = (DRV_I2C_OBJ *) NULL;
    DRV_I2C_BUFFER_OBJ * iterator = NULL;

    dObj = &gDrvI2C${DRV_INSTANCE}Obj;
<#if CONFIG_USE_3RDPARTY_RTOS>
    if(OSAL_MUTEX_Lock(&(dObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
</#if>
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        /* Disable the transmit interrupt */
        SYS_INT_SourceDisable(${I2C_INT_SRC});
</#if>
        iterator = dObj->queue;
        while(iterator != NULL)
        {
            /* The following code removes the object 
               from a doubly linked list queue. */
            iterator->inUse = false;
            if(iterator->previous != NULL)
            {
                iterator->previous->next = iterator->next;
            }
            if(iterator->next != NULL)
            {
                iterator->next->previous = iterator->previous;
            }

            /* Decrementing Current queue size */
            dObj->queueSizeCurrent --;

            iterator = iterator->next;
        }

        /* If there are no buffers in the queue.
         * Make the head pointer point to NULL */
        if(dObj->queueSizeCurrent == 0)
        {
            dObj->queue = NULL;
        }
        else
        {
            /* Iterate to update the head pointer to point
             * the first valid buffer object in the queue */
            iterator = dObj->queue;
            while(iterator != NULL)
            {
                if(iterator->inUse == true)
                {
                    dObj->queue = iterator;
                    break;
                }
                iterator = iterator->next;
            }
        }
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
        /* Re-enable the interrupt if it was enabled */
        SYS_INT_SourceEnable(${I2C_INT_SRC});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
        /* Unlock the mutex */
        OSAL_MUTEX_Unlock(&(dObj->mutexDriverInstance));
    }
</#if>        
    return;
}

// *****************************************************************************
/* Function:
    uint32_t DRV_I2C${DRV_INSTANCE}_BytesTransferred ( DRV_I2C_BUFFER_HANDLE bufferHandle )

  Summary:
    Returns the number of bytes transmitted or received in a particular I2C 
    transaction. The transaction is identified by the handle.

  Description:
    This returns the transmitter and receiver transfer status.

  Parameters:
    handle          -  A valid open-instance handle, returned from the driver's 
                       open routine 
    bufferHandle    -  A valid buffer handle obtained when calling   
                       Transmit/Receive/TransmitThenReceive/TransmitForced or
                       BufferAddRead/BufferAddWrite/BufferAddReadWrite function

  Returns:
    The number of bytes transferred in a particular I2C transaction. 
 
  Remarks:
    See drv_i2c.h for usage information.

*/

uint32_t DRV_I2C${DRV_INSTANCE}_BytesTransferred ( DRV_I2C_BUFFER_HANDLE bufferHandle )
{
    DRV_I2C_BUFFER_OBJ * bufferObj;
    
    if( 0 == bufferHandle || DRV_I2C_BUFFER_HANDLE_INVALID == bufferHandle )
    {
        return 0;
    }
    
    bufferObj = ( DRV_I2C_BUFFER_OBJ *) bufferHandle;
    
    return (bufferObj->nCurrentBytes);

}

/*****************************************************************************
  Function:
    DRV_I2C_BUFFER_EVENT DRV_I2C${DRV_INSTANCE}_TransferStatusGet ( DRV_I2C_BUFFER_HANDLE bufferHandle )

  Summary:
    Returns status of data transfer when Master or Slave acts either as a
    transmitter or a receiver.
    <p><b>Implementation:</b>static</p>

  Description:
    The bufferHandle parameter contains the buffer handle of the buffer that
    associated with the event.
    If the event is DRV_I2C_BUFFER_EVENT_COMPLETE, it means that the data was
    transferred successfully.
    If the event is DRV_I2C_BUFFER_EVENT_ERROR, it means that the data was not
    transferred successfully.

  Parameters:
    bufferHandle    -  A valid buffer handle obtained when calling   
                       Transmit/Receive/TransmitThenReceive/TransmitForced or
                       BufferAddRead/BufferAddWrite/BufferAddReadWrite function
  
  Returns:
    A DRV_I2C_TRANSFER_STATUS value describing the current status of the
    transfer.
   
   Remarks:
    See drv_i2c_static.h for usage information.

*/

DRV_I2C_BUFFER_EVENT DRV_I2C${DRV_INSTANCE}_TransferStatusGet ( DRV_I2C_BUFFER_HANDLE bufferHandle )
{
    DRV_I2C_BUFFER_OBJ * bufferObj;

    if( bufferHandle == 0 || bufferHandle == DRV_I2C_BUFFER_HANDLE_INVALID )
    {
        return DRV_I2C_BUFFER_EVENT_ERROR_INVALID_HANDLE;
    }
    
    bufferObj = ( DRV_I2C_BUFFER_OBJ * ) bufferHandle;
    
    return(bufferObj->event);
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

bool _DRV_I2C${DRV_INSTANCE}_HardwareInitialize( void )
{
<#if OPERATION_MODE == "DRV_I2C_MODE_MASTER">
    /* Calculate value of TWI_CWGR */
    if(!_DRV_I2C${DRV_INSTANCE}_BaudRateSet( ))
    {
        return (false);
    }

    /* Starts the transfer by clearing the transmit hold register  */
    ${I2C_MODULE}->TWI_CR.w = TWI_CR_THRCLR_Msk;
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
    /* Enables interrupt on nack and arbitration lost */
    ${I2C_MODULE}->TWI_IER.w = TWI_IER_NACK_Msk | TWI_IER_ARBLST_Msk;
</#if>            
</#if>
<#if OPERATION_MODE == "DRV_I2C_MODE_SLAVE"> 
    ${I2C_MODULE}->TWI_SMR.w   = 0;
    ${I2C_MODULE}->TWI_FILTR.w = 0;
<#if HIGH_SPEED == true>
    ${I2C_MODULE}->TWI_CR.w = TWI_CR_HSEN_Msk;

    /* Enable the Pad filter and 
       disable the input filtering */
    ${I2C_MODULE}->TWI_FILTR.w |= TWI_FILTR_PADFEN_Msk;
    ${I2C_MODULE}->TWI_FILTR.w &= ~TWI_FILTR_FILT_Msk;
<#else> 
    ${I2C_MODULE}->TWI_CR.w = TWI_CR_HSDIS_Msk;
</#if>
    /* Set slave address and address mask in slave mode */
    ${I2C_MODULE}->TWI_SMR.w = (~(TWI_SMR_SADR_Msk | TWI_SMR_MASK_Msk) & ${I2C_MODULE}->TWI_SMR.w) |
                                   ( TWI_SMR_SADR(${SLAVE_ADDR}  >> 1) |
                                     TWI_SMR_MASK(${SLAVE_ADDR_MASK}));
<#if CLOCK_STRETCH == false>
    ${I2C_MODULE}->TWI_SMR.w |= TWI_SMR_SCLWSDIS_Msk;
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
    /* Enable Underrun and overrun interrupts */
    ${I2C_MODULE}->TWI_IER.w = TWI_IER_UNRE_Msk | TWI_IER_OVRE_Msk;
</#if>
</#if>
<#if CONFIG_DRV_I2C_INTERRUPT_MODE == true>
    /* Enable Slave access, End of Slave access and NACK interrupts */
    ${I2C_MODULE}->TWI_IER.w = TWI_IER_SVACC_Msk | TWI_IER_EOSACC_Msk | TWI_IER_NACK_Msk;
</#if>
</#if>
    return (true);
}

<#if OPERATION_MODE == "DRV_I2C_MODE_MASTER">

bool _DRV_I2C${DRV_INSTANCE}_BaudRateSet( void )
{
    uint32_t ckdiv = 0;
	uint32_t c_lh_div;
	uint32_t cldiv, chdiv;
    uint32_t clkSpeed;

    /* Set Clock */
    if( DRV_I2C_MASTER_MAX_BAUDRATE < ${BAUD_RATE}  )
    {
        return (false);
    }
    
    /* Get Value of clock speed */
    clkSpeed = SYS_CLK_FrequencyGet(SYS_CLK_MASTER);
    
	/* Low level time not less than 1.3us of I2C Fast Mode. */
	if ( ${BAUD_RATE} > DRV_I2C_LOW_LEVEL_TIME_LIMIT ) 
    {
		/* Low level of time fixed for 1.3us. */
		cldiv = clkSpeed / ( DRV_I2C_LOW_LEVEL_TIME_LIMIT * 
                             DRV_I2C_CLK_DIVIDER ) - 
                             DRV_I2C_CLK_CALC_ARGU;
        
		chdiv = clkSpeed / (( ${BAUD_RATE} + 
                            ( ${BAUD_RATE} - DRV_I2C_LOW_LEVEL_TIME_LIMIT)) * 
                              DRV_I2C_CLK_DIVIDER ) - 
                              DRV_I2C_CLK_CALC_ARGU;
		
		/* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
		while (( cldiv > DRV_I2C_CLK_DIV_MAX ) && 
               ( ckdiv < DRV_I2C_CLK_DIV_MIN )) 
        {
			/* Increase clock divider */
			ckdiv++;
            
			/* Divide cldiv value */
			cldiv /= DRV_I2C_CLK_DIVIDER;
		}
        
		/* chdiv must fit in 8 bits, ckdiv must fit in 3 bits */
		while (( chdiv > DRV_I2C_CLK_DIV_MAX ) && 
               ( ckdiv < DRV_I2C_CLK_DIV_MIN )) 
        {
			/* Increase clock divider */
			ckdiv++;
            
			/* Divide cldiv value */
			chdiv /= DRV_I2C_CLK_DIVIDER;
		}

		/* set clock waveform generator register */
		${I2C_MODULE}->TWI_CWGR.w = ( TWI_CWGR_HOLD_Msk & ${I2C_MODULE}->TWI_CWGR.w ) |
                                      ( TWI_CWGR_CLDIV(cldiv) | 
                                        TWI_CWGR_CHDIV(chdiv) |
                                        TWI_CWGR_CKDIV(ckdiv) );
	} 
    else 
    {
		c_lh_div = clkSpeed / ( ${BAUD_RATE} * DRV_I2C_CLK_DIVIDER ) - 
                   DRV_I2C_CLK_CALC_ARGU;

		/* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
		while (( c_lh_div > DRV_I2C_CLK_DIV_MAX ) && 
               ( ckdiv < DRV_I2C_CLK_DIV_MIN )) 
        {
			/* Increase clock divider */
			ckdiv++;
            
			/* Divide cldiv value */
			c_lh_div /= DRV_I2C_CLK_DIVIDER;
		}

		/* set clock waveform generator register */
        ${I2C_MODULE}->TWI_CWGR.w = ( TWI_CWGR_HOLD_Msk & ${I2C_MODULE}->TWI_CWGR.w ) |
                                      ( TWI_CWGR_CLDIV(c_lh_div) | 
                                        TWI_CWGR_CHDIV(c_lh_div) |
                                        TWI_CWGR_CKDIV(ckdiv) )  ;
	}
    
    return (true);
}

</#if>

void _DRV_I2C${DRV_INSTANCE}_Enable( void )
{   
<#if OPERATION_MODE == "DRV_I2C_MODE_MASTER">        
    /* Enable Master Mode */
    ${I2C_MODULE}->TWI_CR.w = TWI_CR_MSEN_Msk;
</#if>
<#if OPERATION_MODE == "DRV_I2C_MODE_SLAVE">
    /* Enable slave mode */
    ${I2C_MODULE}->TWI_CR.w = TWI_CR_SVEN_Msk;
</#if>      
    return;
}

</#macro>

<#if CONFIG_DRV_I2C_INST_IDX0 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="0"
HARDWARE_INSTANCE=CONFIG_DRV_I2C_PERIPHERAL_ID_IDX0
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX0
I2C_INT_SRC=CONFIG_DRV_I2C_INT_SRC_IDX0
BAUD_RATE=CONFIG_DRV_I2C_BAUD_RATE_IDX0
SLAVE_ADDR=CONFIG_DRV_I2C_SLAVE_ADDRESS_VALUE_IDX0
SLAVE_ADDR_MASK=CONFIG_DRV_I2C_SLAVE_ADDRESS_MASK_IDX0
/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="1"
HARDWARE_INSTANCE=CONFIG_DRV_I2C_PERIPHERAL_ID_IDX1 
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX1
I2C_INT_SRC=CONFIG_DRV_I2C_INT_SRC_IDX1
BAUD_RATE=CONFIG_DRV_I2C_BAUD_RATE_IDX1
SLAVE_ADDR=CONFIG_DRV_I2C_SLAVE_ADDRESS_VALUE_IDX1
SLAVE_ADDR_MASK=CONFIG_DRV_I2C_SLAVE_ADDRESS_MASK_IDX1
/>
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
<@DRV_I2C_STATIC_VALUES
DRV_INSTANCE="2"
HARDWARE_INSTANCE=CONFIG_DRV_I2C_PERIPHERAL_ID_IDX2 
OPERATION_MODE=CONFIG_DRV_I2C_OPERATION_MODE_IDX2
I2C_INT_SRC=CONFIG_DRV_I2C_INT_SRC_IDX2
BAUD_RATE=CONFIG_DRV_I2C_BAUD_RATE_IDX2
SLAVE_ADDR=CONFIG_DRV_I2C_SLAVE_ADDRESS_VALUE_IDX2
SLAVE_ADDR_MASK=CONFIG_DRV_I2C_SLAVE_ADDRESS_MASK_IDX2
/>
</#if>

// *****************************************************************************
// *****************************************************************************
// Section: Common File scope functions
// *****************************************************************************
// *****************************************************************************

void _DRV_I2C_ResetAndDisable( twi_registers_t *i2cModule )
{    
    /* Disable the I2C Interrupts */
    i2cModule->TWI_IDR.w = ~0UL;
    
    /* Status register dummy read */
    i2cModule->TWI_SR.w;
    
    /* Reset the i2c Module */
    i2cModule->TWI_CR.w = TWI_CR_SWRST_Msk;
    i2cModule->TWI_RHR.w;
        
    /* Disable the I2C Master/Slave Mode */
    i2cModule->TWI_CR.w = TWI_CR_MSDIS_Msk | TWI_CR_SVDIS_Msk;
    
    return;
}

<#if CONFIG_DRV_I2C_MASTER_MODE_IDX0 = true || 
     CONFIG_DRV_I2C_MASTER_MODE_IDX1 = true ||
     CONFIG_DRV_I2C_MASTER_MODE_IDX2 = true >

uint32_t _DRV_I2C_MakeInternalAddress( uint8_t *address, uint32_t length )
{
    uint32_t result;
    
    if( length == 0 )
    {
        return 0;
    }
    
    result = address[0];
    
    if( length > 1 )
    {
        result <<= 8;
        result |= address[1];
    }
    
    if( length > 2 )
    {
        result <<= 8;
        result |= address[2];
    }
    
    return result;
}

</#if>

/*******************************************************************************
 End of File
*/
