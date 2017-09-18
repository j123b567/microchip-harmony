<#--
/*******************************************************************************
  CAN Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_int.c.ftl

  Summary:
    CAN driver interrupt handler templates.

  Description:
    The CAN device driver provides a simple interface to manage the CAN
    modules on Microchip microcontrollers and this module implements the
    interrupts.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
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
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTOCULAR PURPOSE.
IN NO EVENT SHALL MOCROCHIP OR ITS LOCENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STROCT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVOCES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END
-->
<#macro DRV_CAN_STATIC_FUNCTIONS
    DRV_INSTANCE
    CAN_INT_SRC
    CAN_INT_ISR
    CAN_INT_PRINUM>
<#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CAN_INT_ISR}, IPL${CAN_INT_PRINUM}SOFT) _IntHandlerDrvCANInstance${DRV_INSTANCE}(void)
  <#else>
    <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CAN_INT_PRINUM}AUTO), vector(${CAN_INT_ISR}))) IntHandlerDrvCANInstance${DRV_INSTANCE}_ISR( void );
    </#if>
void IntHandlerDrvCANInstance${DRV_INSTANCE}(void)
  </#if>
<#else>
void __ISR(${CAN_INT_ISR}, IPL${CAN_INT_PRINUM}AUTO) _IntHandlerDrvCANInstance${DRV_INSTANCE}(void)
</#if>
{
<#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
  </#if>
</#if>
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CAN_INT_SRC});
<#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">

    OS_LeaveNestableInterrupt();
  </#if>
</#if>
}
</#macro>
<#macro DRV_CAN_STATIC_FUNCTIONS_PIC32C
HANDLER_NAME
CAN_INSTANCE
INT_RXF0_NEW
INT_RXF0_WATERMARK
INT_RXF1_NEW
INT_RXF1_WATERMARK
INT_TX_DONE
INT_TX_EMPTY
INT_TX_TIMEOUT
>
/* One Interrupt handler for each CAN(MCAN) module */
void ${HANDLER_NAME}(void)
{
    __MCAN_IR_bits_t ir = _MCAN${CAN_INSTANCE}_REGS->MCAN_IR;

    <#if INT_TX_DONE>
    /* TX Completed */
    if (ir.TC)
    {
        /* service the TX Completed interrupt here */
    }

    </#if>
    <#if INT_TX_EMPTY>
    /* TX FIFO is empty */
    if (ir.TFE)
    {
        /* service the TX FIFO empty interrupt here */
    }

    </#if>
    <#if INT_TX_TIMEOUT>
    /* TIMEOUT occurred for TX */
    if (ir.TOO)
    {
        /* service the TX Timeout interrupt here */
    }

    </#if>
    <#if INT_RXF0_NEW>
    /* New Message in FIFO 0 */
    if (ir.RF0N)
    {
        /* service the New Entry for RX FIFO 0 here */
    }

    </#if>
    <#if INT_RXF0_WATERMARK>
    /* WATERMARK in FIFO 0 reached */
    if (ir.RF0W)
    {
        /* service the Watermark reached interrupt for RX FIFO 0 here */
    }

    </#if>
    <#if INT_RXF1_NEW>
    /* New Message in FIFO 1 */
    if (ir.RF1N)
    {
        /* service the New Entry for RX FIFO 1 here */
    }

    </#if>
    <#if INT_RXF1_WATERMARK>
    /* WATERMARK in FIFO 1 reached */
    if (ir.RF1W)
    {
        /* service the Watermark reached interrupt for RX FIFO 1 here */
    }

    </#if>
    /* Clear all of the interrupts */
    _MCAN${CAN_INSTANCE}_REGS->MCAN_IR = ir;
}
</#macro>

<#assign CAN_IDS = ["CAN_ID_0", "CAN_ID_1", "CAN_ID_2", "CAN_ID_3"]>

<#if CONFIG_DRV_CAN_INSTANCES_NUMBER?number gt 0>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID0 == true>
<#if CONFIG_ARCH_ARM == true>
<@DRV_CAN_STATIC_FUNCTIONS_PIC32C
HANDLER_NAME=CONFIG_DRV0_CAN_INTERRUPT_HANDLER_NAME
CAN_INSTANCE=CAN_IDS?seq_index_of(CONFIG_DRV_CAN_PERIPHERAL_ID_IDX0)
INT_RXF0_NEW=CONFIG_DRV0_CAN_INT_RX0_NEW
INT_RXF0_WATERMARK=CONFIG_DRV0_CAN_INT_RX0_WATERMARK
INT_RXF1_NEW=CONFIG_DRV0_CAN_INT_RX1_NEW
INT_RXF1_WATERMARK=CONFIG_DRV0_CAN_INT_RX1_WATERMARK
INT_TX_DONE=CONFIG_DRV_CAN_INT_TX_COMPLETED_IDX0
INT_TX_EMPTY=CONFIG_DRV_CAN_INT_TX_EMPTY_IDX0
INT_TX_TIMEOUT=CONFIG_DRV_CAN_INT_TIMEOUT_IDX0
/>
<#else>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="0"
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX0
CAN_INT_ISR=CONFIG_DRV_CAN_ISR_VECTOR_IDX0
CAN_INT_PRINUM=CONFIG_DRV_CAN_INT_PRIO_NUM_IDX0/>
</#if>
</#if>
</#if>

<#if CONFIG_DRV_CAN_INSTANCES_NUMBER?number gt 1>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID1 == true>
<#if CONFIG_ARCH_ARM == true>
<@DRV_CAN_STATIC_FUNCTIONS_PIC32C
HANDLER_NAME=CONFIG_DRV1_CAN_INTERRUPT_HANDLER_NAME
CAN_INSTANCE=CAN_IDS?seq_index_of(CONFIG_DRV_CAN_PERIPHERAL_ID_IDX1)
INT_RXF0_NEW=CONFIG_DRV1_CAN_INT_RX0_NEW
INT_RXF0_WATERMARK=CONFIG_DRV1_CAN_INT_RX0_WATERMARK
INT_RXF1_NEW=CONFIG_DRV1_CAN_INT_RX1_NEW
INT_RXF1_WATERMARK=CONFIG_DRV1_CAN_INT_RX1_WATERMARK
INT_TX_DONE=CONFIG_DRV_CAN_INT_TX_COMPLETED_IDX1
INT_TX_EMPTY=CONFIG_DRV_CAN_INT_TX_EMPTY_IDX1
INT_TX_TIMEOUT=CONFIG_DRV_CAN_INT_TIMEOUT_IDX1
/>
<#else>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="1"
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX1
CAN_INT_ISR=CONFIG_DRV_CAN_ISR_VECTOR_IDX1
CAN_INT_PRINUM=CONFIG_DRV_CAN_INT_PRIO_NUM_IDX1/>
</#if>
</#if>
</#if>

<#if CONFIG_DRV_CAN_INSTANCES_NUMBER?number gt 2>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID2 == true>
<#if CONFIG_ARCH_ARM == true>
<@DRV_CAN_STATIC_FUNCTIONS_PIC32C
HANDLER_NAME=CONFIG_DRV2_CAN_INTERRUPT_HANDLER_NAME
CAN_INSTANCE=CAN_IDS?seq_index_of(CONFIG_DRV_CAN_PERIPHERAL_ID_IDX2)
INT_RXF0_NEW=CONFIG_DRV2_CAN_INT_RX0_NEW
INT_RXF0_WATERMARK=CONFIG_DRV2_CAN_INT_RX0_WATERMARK
INT_RXF1_NEW=CONFIG_DRV2_CAN_INT_RX1_NEW
INT_RXF1_WATERMARK=CONFIG_DRV2_CAN_INT_RX1_WATERMARK
INT_TX_DONE=CONFIG_DRV_CAN_INT_TX_COMPLETED_IDX2
INT_TX_EMPTY=CONFIG_DRV_CAN_INT_TX_EMPTY_IDX2
INT_TX_TIMEOUT=CONFIG_DRV_CAN_INT_TIMEOUT_IDX2
/>
<#else>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="2"
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX2
CAN_INT_ISR=CONFIG_DRV_CAN_ISR_VECTOR_IDX2
CAN_INT_PRINUM=CONFIG_DRV_CAN_INT_PRIO_NUM_IDX2/>
</#if>
</#if>
</#if>

<#if CONFIG_DRV_CAN_INSTANCES_NUMBER?number gt 3>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID3 == true>
<#if CONFIG_ARCH_ARM == true>
<@DRV_CAN_STATIC_FUNCTIONS_PIC32C
HANDLER_NAME=CONFIG_DRV3_CAN_INTERRUPT_HANDLER_NAME
CAN_INSTANCE=CAN_IDS?seq_index_of(CONFIG_DRV_CAN_PERIPHERAL_ID_IDX3)
INT_RXF0_NEW=CONFIG_DRV3_CAN_INT_RX0_NEW
INT_RXF0_WATERMARK=CONFIG_DRV3_CAN_INT_RX0_WATERMARK
INT_RXF1_NEW=CONFIG_DRV3_CAN_INT_RX1_NEW
INT_RXF1_WATERMARK=CONFIG_DRV3_CAN_INT_RX1_WATERMARK
INT_TX_DONE=CONFIG_DRV_CAN_INT_TX_COMPLETED_IDX3
INT_TX_EMPTY=CONFIG_DRV_CAN_INT_TX_EMPTY_IDX3
INT_TX_TIMEOUT=CONFIG_DRV_CAN_INT_TIMEOUT_IDX3
/>
<#else>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="3"
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX3
CAN_INT_ISR=CONFIG_DRV_CAN_ISR_VECTOR_IDX3
CAN_INT_PRINUM=CONFIG_DRV_CAN_INT_PRIO_NUM_IDX3/>
</#if>
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
