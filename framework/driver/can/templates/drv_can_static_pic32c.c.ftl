/*******************************************************************************
  CAN Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_static_pic32c.c

  Summary:
    CAN driver implementation for the static single instance driver.

  Description:
    The CAN device driver provides a simple interface to manage the CAN
    modules on Microchip microcontrollers.

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
<#include "/utilities/mhc/templates/freemarker_functions.ftl">
// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "driver/can/drv_can.h"
#include "framework/driver/can/src/drv_can_local_pic32c.h"
#include "framework/driver/can/drv_can_static.h"

<#macro DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE
CAN_INSTANCE
TX_FIFO_ELEMENTS
TX_FIFO_WATERMARK
TX_FIFO_BYTES_CFG
TX_ELEMENT_BYTES
RX0_USE
RX0_ELEMENTS
RX0_FIFO_BYTES_CFG
RX0_ELEMENT_BYTES
RX0_WATERMARK
RX0_MODE
RX1_USE
RX1_ELEMENTS
RX1_FIFO_BYTES_CFG
RX1_ELEMENT_BYTES
RX1_WATERMARK
RX1_MODE
TIMESTAMP_USE
TIMESTAMP_MODE
TIME_PRESCALE
TIMEOUT_USE
TIMEOUT_SELECT
TIMEOUT_COUNT
FILTERS_STD
FILTERS_EXT
FILTERS_STD_REJECT
FILTERS_EXT_REJECT
FILTERS_STD_NOMATCH
FILTERS_EXT_NOMATCH
OPERATION_MODE
INT_TX_COMPLETED
INT_TX_FIFO_EMPTY
INT_TIMEOUT
INT_RX0_NEW_ENTRY
INT_RX0_WATERMARK
INT_RX1_NEW_ENTRY
INT_RX1_WATERMARK
INT_PRIORITY
BAUDRATE
>
<#assign INTERRUPTS_USED = INT_TX_COMPLETED || INT_TX_FIFO_EMPTY || INT_TIMEOUT || INT_RX0_NEW_ENTRY || INT_RX0_WATERMARK || INT_RX1_NEW_ENTRY || INT_RX1_WATERMARK >
/* define the constants for this CAN instance */
#define MCAN${CAN_INSTANCE} _MCAN${CAN_INSTANCE}_REGS
#define CONF_CAN${CAN_INSTANCE}_GFC (${FILTERS_STD_NOMATCH} | ${FILTERS_EXT_NOMATCH}<#if FILTERS_STD_REJECT> | MCAN_GFC_RRFS_REJECT</#if><#if FILTERS_EXT_REJECT> | MCAN_GFC_RRFE_REJECT)</#if>)

<#switch BAUDRATE>
    <#default>
    <#case "CAN_BAUD_1M">
#define CONF_CAN${CAN_INSTANCE}_NBTP 0x00011563
    <#break>
    <#case "CAN_BAUD_125K">
#define CONF_CAN${CAN_INSTANCE}_NBTP 0x000D1873
    <#break>
    <#case "CAN_BAUD_250K">
#define CONF_CAN${CAN_INSTANCE}_NBTP 0x00071563
    <#break>
    <#case "CAN_BAUD_500K">
#define CONF_CAN${CAN_INSTANCE}_NBTP 0x00021D93
    <#break>
</#switch>


#define CONF_CAN${CAN_INSTANCE}_IE 0\
<#if INT_TX_COMPLETED>
    | MCAN_IE_TCE_Msk\
</#if>
<#if INT_TX_FIFO_EMPTY>
    | MCAN_IE_TFEE_Msk\
</#if>
<#if INT_TIMEOUT>
    | MCAN_IE_TOOE_Msk\
</#if>
<#if INT_RX0_NEW_ENTRY>
    | MCAN_IE_RF0NE_Msk\
</#if>
<#if INT_RX0_WATERMARK>
    | MCAN_IE_RF0WE_Msk\
</#if>
<#if INT_RX1_NEW_ENTRY>
    | MCAN_IE_RF1NE_Msk\
</#if>
<#if INT_RX1_WATERMARK>
    | MCAN_IE_RF1WE_Msk\
</#if>
/* end of IE config */

<#if TIMESTAMP_USE || TIMEOUT_USE>
#define CONF_CAN${CAN_INSTANCE}_TSCC (${TIMESTAMP_MODE} | MCAN_TSCC_TCP(${TIME_PRESCALE}))
</#if>
<#if TIMEOUT_USE>
#define CONF_CAN${CAN_INSTANCE}_TOCC (${TIMEOUT_SELECT} | MCAN_TOCC_ETOC_TOS_CONTROLLED)
#define CONF_CAN${CAN_INSTANCE}_TOCV (MCAN_TOCV_TOC(${TIMEOUT_COUNT}))

</#if>
/* Configuration for the bytes in each element of RX FIFOs */
#define CONF_CAN${CAN_INSTANCE}_RXESC (0<#if RX0_USE> | MCAN_RXESC_F0DS(${RX0_FIFO_BYTES_CFG})</#if><#if RX1_USE> | MCAN_RXESC_F1DS(${RX1_FIFO_BYTES_CFG})</#if>)
<#if RX0_USE>
#define CONF_CAN${CAN_INSTANCE}_RXF0C (MCAN_RXF0C_F0S(${RX0_ELEMENTS}) | MCAN_RXF0C_F0WM(${RX0_WATERMARK})<#if RX0_MODE> | MCAN_RXF0C_F0OM_Msk</#if>)
static uint8_t can${CAN_INSTANCE}_rx0_fifo[${RX0_ELEMENT_BYTES} * ${RX0_ELEMENTS}]__attribute__((aligned (4)));

</#if>
<#if RX1_USE>
#define CONF_CAN${CAN_INSTANCE}_RXF1C (MCAN_RXF1C_F1S(${RX1_ELEMENTS}) | MCAN_RXF1C_F1WM(${RX1_WATERMARK})<#if RX1_MODE> | MCAN_RXF1C_F1OM_Msk</#if>)
static uint8_t can${CAN_INSTANCE}_rx1_fifo[${RX1_ELEMENT_BYTES} * ${RX1_ELEMENTS}]__attribute__((aligned (4)));

</#if>
#define CONF_CAN${CAN_INSTANCE}_TXESC MCAN_TXESC_TBDS(${TX_FIFO_BYTES_CFG})
#define CONF_CAN${CAN_INSTANCE}_TXBC  MCAN_TXBC_TFQS(${TX_FIFO_ELEMENTS})
#define CONF_CAN${CAN_INSTANCE}_TXEFC MCAN_TXEFC_EFWM(${TX_FIFO_WATERMARK}) | MCAN_TXEFC_EFS(${TX_FIFO_ELEMENTS})

static uint8_t can${CAN_INSTANCE}_tx_fifo[${TX_FIFO_ELEMENTS} * ${TX_ELEMENT_BYTES}]__attribute__((aligned (4)));
static struct _can_tx_event_entry can${CAN_INSTANCE}_tx_event_fifo[${TX_FIFO_ELEMENTS}]__attribute__((aligned (4)));

<#if FILTERS_STD?number gt 0>
#define CONF_CAN${CAN_INSTANCE}_SIDFC_LSS (${FILTERS_STD})
#define CONF_CAN${CAN_INSTANCE}_SIDF  (MCAN_SIDFC_LSS(CONF_CAN${CAN_INSTANCE}_SIDFC_LSS))
<#assign numInstance=FILTERS_STD?number>
__attribute__((aligned (4)))static struct _can_standard_message_filter_element
        can${CAN_INSTANCE}_std_filter[] =
{
    <#list 0..(numInstance-1) as idx>
        <#if .vars["CONFIG_DRV${DRV_INSTANCE}_CAN_STD_FILTER${idx}"]>
    {
        .S0.bit.SFID1 = ${.vars["CONFIG_DRV${DRV_INSTANCE}_CAN_STD_FILTER${idx}_SFID1"]},
        .S0.bit.SFID2 = ${.vars["CONFIG_DRV${DRV_INSTANCE}_CAN_STD_FILTER${idx}_SFID2"]},
        .S0.bit.SFEC  = ${.vars["CONFIG_DRV${DRV_INSTANCE}_CAN_STD_FILTER${idx}_CONFIG"]},
        .S0.bit.SFT   = ${.vars["CONFIG_DRV${DRV_INSTANCE}_CAN_STD_FILTER${idx}_TYPE"]},
    },
        </#if>
     </#list>
};


</#if>
<#if FILTERS_EXT?number gt 0>
#define CONF_CAN${CAN_INSTANCE}_XIDFC_LSE (${FILTERS_EXT})
#define CONF_CAN${CAN_INSTANCE}_XIDFC (MCAN_XIDFC_LSE(CONF_CAN${CAN_INSTANCE}_XIDFC_LSE))
#define CONF_CAN${CAN_INSTANCE}_XIDAM 0x1FFFFFFF

<#assign numInstance=FILTERS_EXT?number>
__attribute__((aligned (4)))static struct _can_extended_message_filter_element
    can${CAN_INSTANCE}_ext_filter[] =
{
    <#list 0..(numInstance-1) as idx>
        <#if .vars["CONFIG_DRV${DRV_INSTANCE}_CAN_EXT_FILTER${idx}"]>
    {
        .F0.bit.EFID1 = ${.vars["CONFIG_DRV${DRV_INSTANCE}_CAN_EXT_FILTER${idx}_EFID1"]},
        .F0.bit.EFEC  = ${.vars["CONFIG_DRV${DRV_INSTANCE}_CAN_EXT_FILTER${idx}_CONFIG"]},
        .F1.bit.EFT   = ${.vars["CONFIG_DRV${DRV_INSTANCE}_CAN_EXT_FILTER${idx}_TYPE"]},
        .F1.bit.EFID2 = ${.vars["CONFIG_DRV${DRV_INSTANCE}_CAN_EXT_FILTER${idx}_EFID2"]},
    },
        </#if>
     </#list>
};


</#if>




// *****************************************************************************
// *****************************************************************************
// Section: Instance ${DRV_INSTANCE} static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_CAN${DRV_INSTANCE}_Initialize(void)
{
    /* set INIT and CCE to get access to configuration registers */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.INIT = 1;
    while (!MCAN${CAN_INSTANCE}->MCAN_CCCR.INIT)  ;
    MCAN${CAN_INSTANCE}->MCAN_CCCR.CCE = 1;

    /*
    The following registers are cleared when MCAN_CCCR.CCE = 1
    > High Priority Message Status (MCAN_HPMS)
    > Receive FIFO 0 Status (MCAN_RXF0S)
    > Receive FIFO 1 Status (MCAN_RXF1S)
    > Transmit FIFO/Queue Status (MCAN_TXFQS)
    > Transmit Buffer Request Pending (MCAN_TXBRP)
    > Transmit Buffer Transmission Occurred (MCAN_TXBTO)
    > Transmit Buffer Cancellation Finished (MCAN_TXBCF)
    > Transmit Event FIFO Status (MCAN_TXEFS)
    */

<#if OPERATION_MODE == "CAN_FD_MODE">
    /* Fast Data Bit Timing and Prescaler Register */
    MCAN${CAN_INSTANCE}->MCAN_DBTP.w = CONF_CAN${CAN_INSTANCE}_DBTP;

<#else>
    /* Nominal Bit timing and Prescaler Register */
    MCAN${CAN_INSTANCE}->MCAN_NBTP.w = CONF_CAN${CAN_INSTANCE}_NBTP;

</#if>
<#if TIMESTAMP_USE || TIMEOUT_USE>
    /* Timestamp Counter Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_TSCC.w = CONF_CAN${CAN_INSTANCE}_TSCC;

</#if>
<#if TIMEOUT_USE>
    /* Timeout Counter Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_TOCC.w = CONF_CAN${CAN_INSTANCE}_TOCC;

    /* Timeout Counter Value Register */
    MCAN${CAN_INSTANCE}->MCAN_TOCV.w = CONF_CAN${CAN_INSTANCE}_TOCV;

</#if>
<#if RX0_USE>
    /* Receive FIFO 0 Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_RXF0C.w = CONF_CAN${CAN_INSTANCE}_RXF0C |
            (((uint32_t)can${CAN_INSTANCE}_rx0_fifo) & 0xFFFF);

</#if>
<#if RX1_USE>
    /* Receive FIFO 1 Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_RXF1C.w = CONF_CAN${CAN_INSTANCE}_RXF1C |
            (((uint32_t)can${CAN_INSTANCE}_rx1_fifo) & 0xFFFF);

</#if>
<#if RX0_USE || RX1_USE>
    /* Receive Buffer / FIFO Element Size Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_RXESC.w = CONF_CAN${CAN_INSTANCE}_RXESC;

</#if>
    /* Transmit Buffer Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_TXBC.w = CONF_CAN${CAN_INSTANCE}_TXBC |
            (((uint32_t)can${CAN_INSTANCE}_tx_fifo) & 0xFFFF);

    /* Transmit Buffer/FIFO Element Size Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_TXESC.w = CONF_CAN${CAN_INSTANCE}_TXESC;

    /* Transmit Event FIFO Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_TXEFC.w = CONF_CAN${CAN_INSTANCE}_TXEFC |
            (((uint32_t)can${CAN_INSTANCE}_tx_event_fifo) & 0xFFFF);

    /* Global Filter Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_GFC.w = CONF_CAN${CAN_INSTANCE}_GFC;

<#if FILTERS_STD?number gt 0>
    /* Standard ID Filter Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_SIDFC.w = CONF_CAN${CAN_INSTANCE}_SIDF |
            (((uint32_t)can${CAN_INSTANCE}_std_filter) & 0xFFFF);

</#if>
<#if FILTERS_EXT?number gt 0>
    /* Extended ID Filter Configuration Register */
    MCAN${CAN_INSTANCE}->MCAN_XIDFC.w = CONF_CAN${CAN_INSTANCE}_XIDFC |
            (((uint32_t)can${CAN_INSTANCE}_ext_filter) & 0xFFFF);

    /* Extended ID AND Mask Register */
    MCAN${CAN_INSTANCE}->MCAN_XIDAM.w = CONF_CAN${CAN_INSTANCE}_XIDAM;

</#if>
    <#switch OPERATION_MODE>
    <#default>
    <#case "CAN_NORMAL_MODE">
    /* CAN operation is set to normal mode */
    <#break>
    <#case "CAN_CONFIGURATION_MODE">
    /* CAN operation is set to configuration mode */
    return;  /* configuration mode set - leave now */
    <#break>
    <#case "CAN_FD_MODE">
    /* CAN operation is set to FD mode */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.FDOE = 1;
    <#break>
    <#case "CAN_RESTRICTED_MODE">
    /* CAN operation is set to restricted mode */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.ASM = 1;
    <#break>
    <#case "CAN_MONITOR_MODE">
    /* CAN operation is set to monitor mode */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.MON = 1;
    <#break>
    <#case "CAN_EXTERNAL_LOOPBACK_MODE">
    /* CAN operation is set to external loopback mode */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.TEST = 1;
    MCAN${CAN_INSTANCE}->MCAN_TEST.LBCK = 1;
    <#break>
    <#case "CAN_INTERNAL_LOOPBACK_MODE">
    /* CAN operation is set to internal loopback mode */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.TEST = 1;
    MCAN${CAN_INSTANCE}->MCAN_TEST.LBCK = 1;
    MCAN${CAN_INSTANCE}->MCAN_CCCR.MON = 1;
    <#break>
    </#switch>

    /* Disable INIT to prevent Configuration Change and leave initialize mode */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.TXP = 1;
    MCAN${CAN_INSTANCE}->MCAN_CCCR.INIT = 0;
    while (MCAN${CAN_INSTANCE}->MCAN_CCCR.INIT)  ;

<#if INTERRUPTS_USED>
    /* Enable selected interrupts */
    MCAN${CAN_INSTANCE}->MCAN_IE.w = CONF_CAN${CAN_INSTANCE}_IE;

    /* all interrupts are using line 0 - enable the interrupt */
    MCAN${CAN_INSTANCE}->MCAN_ILE.EINT0 = 1;

    /* set priority */
    SYS_INT_VectorPrioritySet(MCAN${CAN_INSTANCE}_IRQn, ${INT_PRIORITY});

    /* clear and register the MCAN with the NVIC */
    SYS_INT_SourceStatusClear(MCAN${CAN_INSTANCE}_IRQn);
    SYS_INT_SourceEnable(MCAN${CAN_INSTANCE}_IRQn);
</#if>
}



void DRV_CAN${DRV_INSTANCE}_Deinitialize(void)
{
    MCAN${CAN_INSTANCE}->MCAN_CCCR.INIT = 1;
<#if INTERRUPTS_USED>
    SYS_INT_SourceDisable(MCAN${CAN_INSTANCE}_IRQn);
</#if>
}



void DRV_CAN${DRV_INSTANCE}_Open(void)
{
    /* Switch the CAN module ON */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.INIT = 0;
}



void DRV_CAN${DRV_INSTANCE}_Close(void)
{
    /* Switch the CAN module OFF */
    MCAN${CAN_INSTANCE}->MCAN_CCCR.INIT = 1;
}



/* returns false on fail, true on success */
bool DRV_CAN${DRV_INSTANCE}_ChannelMessageTransmit(CAN_CHANNEL channelNum, int address,
        uint8_t DLC, uint8_t* message)
{
    (void)channelNum;

    uint32_t tfqpi = MCAN${CAN_INSTANCE}->MCAN_TXFQS.TFQPI;
    struct _can_tx_fifo_entry *f =
        (struct _can_tx_fifo_entry*) (can${CAN_INSTANCE}_tx_fifo + tfqpi * ${TX_ELEMENT_BYTES});

    if (MCAN${CAN_INSTANCE}->MCAN_TXFQS.TFQF)
    {
        /* The FIFO is full */
        return 0;
    }

    /* clear headers */
    f->T0.val = f->T1.val = 0;

    /* Note in this implementation all 11 bit will be sent as standard, and
       larger will be sent as extended. So an address of 0 can't be extended */
    /* extended (29 bit) or standard (11 bit) */
    address &= 0x1FFFFFFF;
    if (address & 0b00011111111111111111100000000000)
    {
        /* Extended */
        f->T0.bit.ID = address;
        f->T0.bit.XTD = 1;
    }
    else
    {
        /* A standard identifier is stored into ID[28:18] */
        f->T0.val = address << 18;
    }

    /* set data length */
<#if TX_FIFO_BYTES_CFG == 0>
    f->T1.bit.DLC = DLC;
<#else>
    if (DLC <= 8)
    {
        f->T1.bit.DLC = DLC;
    }
    else if (DLC <= 12)
    {
        f->T1.bit.DLC = 0x9;
    }
    else if (DLC <= 16)
    {
        f->T1.bit.DLC = 0xA;
    }
    else if (DLC <= 20)
    {
        f->T1.bit.DLC = 0xB;
    }
    else if (DLC <= 24)
    {
        f->T1.bit.DLC = 0xC;
    }
    else if (DLC <= 32)
    {
        f->T1.bit.DLC = 0xD;
    }
    else if (DLC <= 48)
    {
        f->T1.bit.DLC = 0xE;
    }
    else if (DLC <= 64)
    {
        f->T1.bit.DLC = 0xF;
    }
</#if>

    /* copy the message into the payload */
    memcpy(f->data, message, DLC);

    /* request the transmit */
    MCAN${CAN_INSTANCE}->MCAN_TXBAR.w = 1U << tfqpi;

    return 1;
}



/* Channel is which FIFO to use. Only 2 max available */
bool DRV_CAN${DRV_INSTANCE}_ChannelMessageReceive(CAN_CHANNEL channelNum, int address,
        uint8_t DLC, uint8_t* message)
{
<#if RX0_USE || RX1_USE>
    uint32_t rxgi;
    struct _can_rx_fifo_entry *f = NULL;

    /* Read a message off of the selected channel */
    switch (channelNum)
    {
        default:
<#if RX0_USE>
        case 0:
            if (!MCAN${CAN_INSTANCE}->MCAN_RXF0S.F0FL) return 0;

            /* read FIFO 0 */
            rxgi = MCAN${CAN_INSTANCE}->MCAN_RXF0S.F0GI;
            f = (struct _can_rx_fifo_entry*) (can${CAN_INSTANCE}_rx0_fifo + rxgi * ${RX0_ELEMENT_BYTES});

            /* ack the fifo position */
            MCAN${CAN_INSTANCE}->MCAN_RXF0A.F0AI = rxgi;
            break;
</#if>

<#if RX1_USE>
        case 1:
            if (!MCAN${CAN_INSTANCE}->MCAN_RXF1S.F1FL) return 0;

            /* Read FIFO 1 */
            rxgi = MCAN${CAN_INSTANCE}->MCAN_RXF1S.F1GI;
            f = (struct _can_rx_fifo_entry*) (can${CAN_INSTANCE}_rx1_fifo + rxgi * ${RX1_ELEMENT_BYTES});

            /* ack the fifo position */
            MCAN${CAN_INSTANCE}->MCAN_RXF1A.F1AI = rxgi;
            break;
</#if>
    }

    memcpy(message, f->data, DLC);
    return 1;
<#else>
    return 0;
</#if>
}

</#macro>




<#assign CAN_IDS = ["CAN_ID_0", "CAN_ID_1", "CAN_ID_2", "CAN_ID_3"]>

<#assign TIMESTAMP_MODES =
            {"CAN_TIMESTAMP_MODE_ZERO"    : "MCAN_TSCC_TSS_ALWAYS_0",
             "CAN_TIMESTAMP_MODE_TCP_INC" : "MCAN_TSCC_TSS_TCP_INC",
             "CAN_TIMESTAMP_MODE_EXT_INC" : "MCAN_TSCC_TSS_EXT_TIMESTAMP"}>


<#assign TIMEOUT_SELECT =
           {"CAN_TIMEOUT_MODE_CONTINUOUS" : "MCAN_TOCC_TOS_CONTINUOUS",
            "CAN_TIMEOUT_MODE_TX_EVENT"   : "MCAN_TOCC_TOS_TX_EV_TIMEOUT",
            "CAN_TIMEOUT_MODE_RX0_EVENT"  : "MCAN_TOCC_TOS_RX0_EV_TIMEOUT",
            "CAN_TIMEOUT_MODE_RX1_EVENT"  : "MCAN_TOCC_TOS_RX1_EV_TIMEOUT"}>

<#assign ELEMENT_BYTES =
    {"CAN_FIFO_8_BYTE"  : 16,
     "CAN_FIFO_12_BYTE" : 20,
     "CAN_FIFO_16_BYTE" : 24,
     "CAN_FIFO_20_BYTE" : 28,
     "CAN_FIFO_24_BYTE" : 32,
     "CAN_FIFO_32_BYTE" : 40,
     "CAN_FIFO_48_BYTE" : 56,
     "CAN_FIFO_64_BYTE" : 72}>

<#assign ELEMENT_BYTES_CFG =
    {"CAN_FIFO_8_BYTE"  : 0,
     "CAN_FIFO_12_BYTE" : 1,
     "CAN_FIFO_16_BYTE" : 2,
     "CAN_FIFO_20_BYTE" : 3,
     "CAN_FIFO_24_BYTE" : 4,
     "CAN_FIFO_32_BYTE" : 5,
     "CAN_FIFO_48_BYTE" : 6,
     "CAN_FIFO_64_BYTE" : 7}>

<#assign CAN_STD_NOMATCH =
      {"Accept in RX FIFO 0" : "MCAN_GFC_ANFS_RX_FIFO_0",
       "Accept in RX FIFO 1" : "MCAN_GFC_ANFS_RX_FIFO_1",
       "Message Rejected"    : "(2<<4)"}>

<#assign CAN_EXT_NOMATCH =
      {"Accept in RX FIFO 0" : "MCAN_GFC_ANFE_RX_FIFO_0",
       "Accept in RX FIFO 1" : "MCAN_GFC_ANFE_RX_FIFO_1",
       "Message Rejected"    : "(2<<2)"}>



<#if CONFIG_DRV_CAN_INST_IDX0!false>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="0"
CAN_INSTANCE=CAN_IDS?seq_index_of(CONFIG_DRV_CAN_PERIPHERAL_ID_IDX0)
TX_FIFO_ELEMENTS=CONFIG_DRV_CAN_TX_FIFO_ELEMENTS_IDX0
TX_FIFO_WATERMARK=CONFIG_DRV_CAN_TX_WATERMARK_LEVEL_IDX0
TX_FIFO_BYTES_CFG=ELEMENT_BYTES_CFG[CONFIG_DRV_CAN_TX_ELEMENT_BYTES_IDX0]!0
TX_ELEMENT_BYTES=ELEMENT_BYTES[CONFIG_DRV_CAN_TX_ELEMENT_BYTES_IDX0]!16
RX0_USE=CONFIG_DRV0_CAN_RX0_CHANNEL!n
RX0_ELEMENTS=CONFIG_DRV0_CAN_RX0_CHANNEL_ELEMENTS!0
RX0_FIFO_BYTES_CFG=ELEMENT_BYTES_CFG[CONFIG_DRV0_CAN_RX0_ELEMENT_BYTES]!0
RX0_ELEMENT_BYTES=ELEMENT_BYTES[CONFIG_DRV0_CAN_RX0_ELEMENT_BYTES]!8
RX0_WATERMARK=CONFIG_DRV0_CAN_RX0_FIFO_WATERMARK!0
RX0_MODE=CONFIG_DRV0_CAN_RX0_FIFO_OVERWR!n
RX1_USE=CONFIG_DRV0_CAN_RX1_CHANNEL!n
RX1_ELEMENTS=CONFIG_DRV0_CAN_RX1_CHANNEL_ELEMENTS!0
RX1_FIFO_BYTES_CFG=ELEMENT_BYTES_CFG[CONFIG_DRV0_CAN_RX1_ELEMENT_BYTES]!0
RX1_ELEMENT_BYTES=ELEMENT_BYTES[CONFIG_DRV0_CAN_RX1_ELEMENT_BYTES]!8
RX1_WATERMARK=CONFIG_DRV0_CAN_RX1_FIFO_WATERMARK!0
RX1_MODE=CONFIG_DRV0_CAN_RX1_FIFO_OVERWR!n
TIMESTAMP_USE=CONFIG_DRV_CAN_USE_TIMESTAMP_ID0
TIMESTAMP_MODE=TIMESTAMP_MODES[CONFIG_DRV_CAN_TIMESTAMP_MODE_IDX0]!"MCAN_TSCC_TSS_ALWAYS_0"
TIME_PRESCALE=CONFIG_DRV_CAN_TIME_DIVIDER_ID0!0
TIMEOUT_USE=CONFIG_DRV_CAN_USE_TIMEOUT_ID0
TIMEOUT_SELECT=TIMEOUT_SELECT[CONFIG_DRV_CAN_TIMEOUT_MODE_IDX0]!"MCAN_TOCC_TOS_CONTINUOUS"
TIMEOUT_COUNT=CONFIG_DRV_CAN_TIMEOUT_TOP_ID0!1000
FILTERS_STD=CONFIG_DRV_CAN_STD_FILTERS_IDX0
FILTERS_EXT=CONFIG_DRV_CAN_EXT_FILTERS_IDX0
FILTERS_STD_REJECT=CONFIG_DRV_CAN_STD_REJECT_IDX0!n
FILTERS_EXT_REJECT=CONFIG_DRV_CAN_EXT_REJECT_IDX0!n
FILTERS_STD_NOMATCH=CAN_STD_NOMATCH[CONFIG_DRV_CAN_STD_NOMATCH_IDX0]
FILTERS_EXT_NOMATCH=CAN_EXT_NOMATCH[CONFIG_DRV_CAN_EXT_NOMATCH_IDX0]
OPERATION_MODE=CONFIG_DRV_CAN_OPERATION_MODE_IDX0
INT_TX_COMPLETED=CONFIG_DRV_CAN_INT_TX_COMPLETED_IDX0
INT_TX_FIFO_EMPTY=CONFIG_DRV_CAN_INT_TX_EMPTY_IDX0
INT_TIMEOUT=CONFIG_DRV_CAN_INT_TIMEOUT_IDX0
INT_RX0_NEW_ENTRY=CONFIG_DRV0_CAN_INT_RX0_NEW
INT_RX0_WATERMARK=CONFIG_DRV0_CAN_INT_RX0_WATERMARK
INT_RX1_NEW_ENTRY=CONFIG_DRV0_CAN_INT_RX1_NEW
INT_RX1_WATERMARK=CONFIG_DRV0_CAN_INT_RX1_WATERMARK
INT_PRIORITY=CONFIG_DRV0_CAN_INTERRUPT_PRIORITY
BAUDRATE=CONFIG_DRV_CAN_BAUD_RATE_IDX0
/>
</#if>

<#if CONFIG_DRV_CAN_INST_IDX1!false>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="1"
CAN_INSTANCE=CAN_IDS?seq_index_of(CONFIG_DRV_CAN_PERIPHERAL_ID_IDX1)
TX_FIFO_ELEMENTS=CONFIG_DRV_CAN_TX_FIFO_ELEMENTS_IDX1
TX_FIFO_WATERMARK=CONFIG_DRV_CAN_TX_WATERMARK_LEVEL_IDX1
TX_FIFO_BYTES_CFG=ELEMENT_BYTES_CFG[CONFIG_DRV_CAN_TX_ELEMENT_BYTES_IDX1]!0
TX_ELEMENT_BYTES=ELEMENT_BYTES[CONFIG_DRV_CAN_TX_ELEMENT_BYTES_IDX1]!16
RX0_USE=CONFIG_DRV1_CAN_RX0_CHANNEL!n
RX0_ELEMENTS=CONFIG_DRV1_CAN_RX0_CHANNEL_ELEMENTS!0
RX0_FIFO_BYTES_CFG=ELEMENT_BYTES_CFG[CONFIG_DRV1_CAN_RX0_ELEMENT_BYTES]!0
RX0_ELEMENT_BYTES=ELEMENT_BYTES[CONFIG_DRV1_CAN_RX0_ELEMENT_BYTES]!8
RX0_WATERMARK=CONFIG_DRV1_CAN_RX0_FIFO_WATERMARK!0
RX0_MODE=CONFIG_DRV1_CAN_RX0_FIFO_OVERWR!n
RX1_USE=CONFIG_DRV1_CAN_RX1_CHANNEL!n
RX1_ELEMENTS=CONFIG_DRV1_CAN_RX1_CHANNEL_ELEMENTS!0
RX1_FIFO_BYTES_CFG=ELEMENT_BYTES_CFG[CONFIG_DRV1_CAN_RX1_ELEMENT_BYTES]!0
RX1_ELEMENT_BYTES=ELEMENT_BYTES[CONFIG_DRV1_CAN_RX1_ELEMENT_BYTES]!8
RX1_WATERMARK=CONFIG_DRV1_CAN_RX1_FIFO_WATERMARK!0
RX1_MODE=CONFIG_DRV1_CAN_RX1_FIFO_OVERWR!n
TIMESTAMP_USE=CONFIG_DRV_CAN_USE_TIMESTAMP_ID1
TIMESTAMP_MODE=TIMESTAMP_MODES[CONFIG_DRV_CAN_TIMESTAMP_MODE_IDX1]!"MCAN_TSCC_TSS_ALWAYS_0"
TIME_PRESCALE=CONFIG_DRV_CAN_TIME_DIVIDER_ID1!0
TIMEOUT_USE=CONFIG_DRV_CAN_USE_TIMEOUT_ID1
TIMEOUT_SELECT=TIMEOUT_SELECT[CONFIG_DRV_CAN_TIMEOUT_MODE_IDX1]!"MCAN_TOCC_TOS_CONTINUOUS"
TIMEOUT_COUNT=CONFIG_DRV_CAN_TIMEOUT_TOP_ID1!1000
FILTERS_STD=CONFIG_DRV_CAN_STD_FILTERS_IDX1
FILTERS_EXT=CONFIG_DRV_CAN_EXT_FILTERS_IDX1
FILTERS_STD_REJECT=CONFIG_DRV_CAN_STD_REJECT_IDX1!n
FILTERS_EXT_REJECT=CONFIG_DRV_CAN_EXT_REJECT_IDX1!n
FILTERS_STD_NOMATCH=CAN_STD_NOMATCH[CONFIG_DRV_CAN_STD_NOMATCH_IDX1]
FILTERS_EXT_NOMATCH=CAN_EXT_NOMATCH[CONFIG_DRV_CAN_EXT_NOMATCH_IDX1]
OPERATION_MODE=CONFIG_DRV_CAN_OPERATION_MODE_IDX1
INT_TX_COMPLETED=CONFIG_DRV_CAN_INT_TX_COMPLETED_IDX1
INT_TX_FIFO_EMPTY=CONFIG_DRV_CAN_INT_TX_EMPTY_IDX1
INT_TIMEOUT=CONFIG_DRV_CAN_INT_TIMEOUT_IDX1
INT_RX0_NEW_ENTRY=CONFIG_DRV1_CAN_INT_RX0_NEW
INT_RX0_WATERMARK=CONFIG_DRV1_CAN_INT_RX0_WATERMARK
INT_RX1_NEW_ENTRY=CONFIG_DRV1_CAN_INT_RX1_NEW
INT_RX1_WATERMARK=CONFIG_DRV1_CAN_INT_RX1_WATERMARK
INT_PRIORITY=CONFIG_DRV1_CAN_INTERRUPT_PRIORITY
BAUDRATE=CONFIG_DRV_CAN_BAUD_RATE_IDX1
/>
</#if>

/*******************************************************************************
 End of File
*/

