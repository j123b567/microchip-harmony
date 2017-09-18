/*******************************************************************************
  CAN Device Library Interface Definition for PIC32C

  Company:
    Microchip Technology Inc.

  File Name:
    dev_can_definitions_pic32c.h

  Summary:
    CAN device data type definitions header.

  Description:
    This file contains the data type definitions header.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_CAN_DEFS_PIC32C_
#define _DRV_CAN_DEFS_PIC32C_

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include "system_config.h"
#include "system/common/sys_common.h"
#include "system/common/sys_module.h"
#include "system/int/sys_int.h"
#include "arch/arm/devices_pic32c.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************


typedef enum {
    CAN_CHANNEL0 = 0x00,
    CAN_CHANNEL1 = 0x01,
    CAN_CHANNEL2 = 0x02,
    CAN_CHANNEL3 = 0x03,
} CAN_CHANNEL;


typedef enum {
    CAN_NORMAL_MODE,
    CAN_FD_MODE,
    CAN_DISABLE_MODE,
    CAN_RESTRICTED_MODE,
    CAN_MONITOR_MODE,
    CAN_CONFIGURATION_MODE,
    CAN_INTERNAL_LOOPBACK_MODE,
} CAN_OPERATION_MODES;


typedef enum {
    CAN_RX_CHANNEL_NOT_EMPTY = 0x01,
    CAN_RX_CHANNEL_HALF_FULL = 0x02,
    CAN_RX_CHANNEL_FULL = 0x04,
    CAN_RX_CHANNEL_OVERFLOW = 0x08,
    CAN_RX_CHANNEL_ANY_EVENT = 0x0F,
    CAN_TX_CHANNEL_EMPTY = 0x100,
    CAN_TX_CHANNEL_HALF_EMPTY = 0x200,
    CAN_TX_CHANNEL_NOT_FULL = 0x400,
    CAN_TX_CHANNEL_ANY_EVENT = 0x700
} CAN_CHANNEL_EVENT;


typedef enum {
    CAN_BAUD_RATE_PRESCALE_1 = 0,
    CAN_BAUD_RATE_PRESCALE_2 = 1,
    CAN_BAUD_RATE_PRESCALE_3 = 2,
    CAN_BAUD_RATE_PRESCALE_4 = 3,
    CAN_BAUD_RATE_PRESCALE_5 = 4,
    CAN_BAUD_RATE_PRESCALE_6 = 5,
    CAN_BAUD_RATE_PRESCALE_7 = 6,
    CAN_BAUD_RATE_PRESCALE_8 = 7,
    CAN_BAUD_RATE_PRESCALE_9 = 8,
    CAN_BAUD_RATE_PRESCALE_10 = 9,
    CAN_BAUD_RATE_PRESCALE_11 = 10,
    CAN_BAUD_RATE_PRESCALE_12 = 11,
    CAN_BAUD_RATE_PRESCALE_13 = 12,
    CAN_BAUD_RATE_PRESCALE_14 = 13,
    CAN_BAUD_RATE_PRESCALE_15 = 14,
    CAN_BAUD_RATE_PRESCALE_16 = 15,
    CAN_BAUD_RATE_PRESCALE_17 = 16,
    CAN_BAUD_RATE_PRESCALE_18 = 17,
    CAN_BAUD_RATE_PRESCALE_19 = 18,
    CAN_BAUD_RATE_PRESCALE_20 = 19,
    CAN_BAUD_RATE_PRESCALE_21 = 20,
    CAN_BAUD_RATE_PRESCALE_22 = 21,
    CAN_BAUD_RATE_PRESCALE_23 = 22,
    CAN_BAUD_RATE_PRESCALE_24 = 23,
    CAN_BAUD_RATE_PRESCALE_25 = 24,
    CAN_BAUD_RATE_PRESCALE_26 = 25,
    CAN_BAUD_RATE_PRESCALE_27 = 26,
    CAN_BAUD_RATE_PRESCALE_28 = 27,
    CAN_BAUD_RATE_PRESCALE_29 = 28,
    CAN_BAUD_RATE_PRESCALE_30 = 29,
    CAN_BAUD_RATE_PRESCALE_31 = 30,
    CAN_BAUD_RATE_PRESCALE_32 = 31,
} CAN_BAUD_RATE_PRESCALE;


typedef enum {
    CAN_TIME_SEGMENT_LEN_1TQ = 0x00,
    CAN_TIME_SEGMENT_LEN_2TQ = 0x01,
    CAN_TIME_SEGMENT_LEN_3TQ = 0x02,
    CAN_TIME_SEGMENT_LEN_4TQ = 0x03,
    CAN_TIME_SEGMENT_LEN_5TQ = 0x04,
    CAN_TIME_SEGMENT_LEN_6TQ = 0x05,
    CAN_TIME_SEGMENT_LEN_7TQ = 0x06,
    CAN_TIME_SEGMENT_LEN_8TQ = 0x07
} CAN_TIME_SEGMENT_LENGTH;


typedef enum {
    CAN_CHANNEL0_EVENT = 0x00,
    CAN_CHANNEL1_EVENT = 0x01,
    CAN_NO_EVENT = 0x40,
    CAN_ERROR_EVENT = 0x41,
    CAN_WAKEUP_EVENT = 0x42,
    CAN_RX_CHANNEL_OVERFLOW_EVENT = 0x43,
    CAN_ADDRESS_ERROR_EVENT = 0x44,
    CAN_BUS_BANDWIDTH_ERROR = 0x45,
    CAN_TIMESTAMP_TIMER_EVENT = 0x46,
    CAN_MODE_CHANGE_EVENT = 0x47,
    CAN_INVALID_MESSAGE_RECEIVED_EVENT = 0x48
} CAN_EVENT_CODE;


typedef enum {
    CAN_FILTER0 = 0x00,
    CAN_FILTER1 = 0x01,
    CAN_FILTER2 = 0x02,
    CAN_FILTER3 = 0x03,
    CAN_FILTER4 = 0x04,
    CAN_FILTER5 = 0x05,
    CAN_FILTER6 = 0x06,
    CAN_FILTER7 = 0x07,
    CAN_FILTER8 = 0x08,
    CAN_FILTER9 = 0x09,
    CAN_FILTER10 = 0x0A,
    CAN_FILTER11 = 0x0B,
    CAN_FILTER12 = 0x0C,
    CAN_FILTER13 = 0x0D,
    CAN_FILTER14 = 0x0E,
    CAN_FILTER15 = 0x0F,
    CAN_FILTER16 = 0x10,
    CAN_FILTER17 = 0x11,
    CAN_FILTER18 = 0x12,
    CAN_FILTER19 = 0x13,
    CAN_FILTER20 = 0x14,
    CAN_FILTER21 = 0x15,
    CAN_FILTER22 = 0x16,
    CAN_FILTER23 = 0x17,
    CAN_FILTER24 = 0x18,
    CAN_FILTER25 = 0x19,
    CAN_FILTER26 = 0x1A,
    CAN_FILTER27 = 0x1B,
    CAN_FILTER28 = 0x1C,
    CAN_FILTER29 = 0x1D,
    CAN_FILTER30 = 0x1E,
    CAN_FILTER31 = 0x1F,
    CAN_TOTAL_FILTERS = 0x20

} CAN_FILTER;


typedef enum {
    CAN_FILTER_MASK0 = 0x00,
    CAN_FILTER_MASK1 = 0x01,
    CAN_FILTER_MASK2 = 0x02,
    CAN_FILTER_MASK3 = 0x03,
    CAN_NUMBER_OF_FILTER_MASKS = 0x04
} CAN_FILTER_MASK;


typedef enum {
    CAN_TX_RTR_ENABLED = 0x01,
    CAN_TX_RTR_DISABLED = 0x00
} CAN_TX_RTR;


typedef enum {
    CAN_RX_DATA_ONLY = 0x00,
    CAN_RX_FULL_RECEIVE = 0x01
} CAN_RX_DATA_MODE;


typedef enum {
    CAN_FILTER_MASK_IDE_TYPE = 0x00,
    CAN_FILTER_MASK_ANY_TYPE = 0x01
} CAN_FILTER_MASK_TYPE;


typedef enum {
    CAN_LOWEST_PRIORITY = 0x00,
    CAN_LOW_MEDIUM_PRIORITY = 0x01,
    CAN_HIGH_MEDIUM_PRIORITY = 0x02,
    CAN_HIGHEST_PRIORITY = 0x03
} CAN_TXCHANNEL_PRIORITY;


typedef enum {
    CAN_DNET_FILTER_SIZE_18_BIT = 0x12,
    CAN_DNET_FILTER_SIZE_17_BIT = 0x11,
    CAN_DNET_FILTER_SIZE_16_BIT = 0x10,
    CAN_DNET_FILTER_SIZE_15_BIT = 0x0F,
    CAN_DNET_FILTER_SIZE_14_BIT = 0x0E,
    CAN_DNET_FILTER_SIZE_13_BIT = 0x0D,
    CAN_DNET_FILTER_SIZE_12_BIT = 0x0C,
    CAN_DNET_FILTER_SIZE_11_BIT = 0x0B,
    CAN_DNET_FILTER_SIZE_10_BIT = 0x0A,
    CAN_DNET_FILTER_SIZE_9_BIT = 0x09,
    CAN_DNET_FILTER_SIZE_8_BIT = 0x08,
    CAN_DNET_FILTER_SIZE_7_BIT = 0x07,
    CAN_DNET_FILTER_SIZE_6_BIT = 0x06,
    CAN_DNET_FILTER_SIZE_5_BIT = 0x05,
    CAN_DNET_FILTER_SIZE_4_BIT = 0x04,
    CAN_DNET_FILTER_SIZE_3_BIT = 0x03,
    CAN_DNET_FILTER_SIZE_2_BIT = 0x02,
    CAN_DNET_FILTER_SIZE_1_BIT = 0x01,
    CAN_DNET_FILTER_DISABLE = 0x00
} CAN_DNET_FILTER_SIZE;


typedef enum {

    CAN_TX_EVENT = 0x01,
    CAN_RX_EVENT = 0x02,
    CAN_TIMESTAMP_TIMER_OVERFLOW_EVENT = 0x04,
    CAN_OPERATION_MODE_CHANGE_EVENT = 0x08,
    CAN_RX_OVERFLOW_EVENT = 0x800,
    CAN_SYSTEM_ERROR_EVENT = 0x1000,
    CAN_BUS_ERROR_EVENT = 0x2000,
    CAN_BUS_ACTIVITY_WAKEUP_EVENT = 0x4000,
    CAN_INVALID_RX_MESSAGE_EVENT = 0x8000,
    CAN_All_EVENTS = 0xF80F
} CAN_MODULE_EVENT;


typedef enum {
    CAN_TX_RX_WARNING_STATE = 0x10000,
    CAN_RX_WARNING_STATE = 0x20000,
    CAN_TX_WARNING_STATE = 0x40000,
    CAN_RX_BUS_PASSIVE_STATE = 0x80000,
    CAN_TX_BUS_PASSIVE_STATE = 0x100000,
    CAN_TX_BUS_OFF_STATE = 0x200000
} CAN_ERROR_STATE;


typedef enum {
    CAN_EID = 0x00,
    CAN_SID = 0x01
} CAN_ID_TYPE;


typedef enum {
    CAN_CHANNEL0_MASK = 0x00000001,
    CAN_CHANNEL1_MASK = 0x00000002,
    CAN_ANYCHANNEL_MASK = 0xFFFFFFFF
} CAN_CHANNEL_MASK;


typedef enum {
    CAN_TX_CHANNEL_TRANSMITTING = 0x01,
    CAN_TX_CHANNEL_ERROR = 0x02,
    CAN_TX_CHANNEL_ARBITRATION_LOST = 0x04
} CAN_TX_CHANNEL_STATUS;

#endif /* _DRV_CAN_DEFS_PIC32C_ */
