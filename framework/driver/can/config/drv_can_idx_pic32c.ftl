config DRV_CAN_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
    default n if DRV_CAN_INSTANCES_NUMBER_GT_${INSTANCE} = n
</#if>
    default n if DRV_CAN_INSTANCES_NUMBER = ${INSTANCE+1}
    default y

config DRV_CAN_INST_IDX${INSTANCE}
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
                 && DRV_CAN_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver Instance ${INSTANCE}"
    default y

ifblock DRV_CAN_INST_IDX${INSTANCE}

config DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE}
    string "CAN module ID"
    range CAN_MODULE_ID
    default "CAN_ID_${INSTANCE}"
    set SYS_CLK_PMC_ID_MCAN0 optionally to y if DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_0"
    set SYS_CLK_PMC_ID_MCAN1 optionally to y if DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_1"
    set SYS_CLK_PMC_ID_MCAN2 optionally to y if DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_2"
    set SYS_CLK_PMC_ID_MCAN3 optionally to y if DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_3"
    ---help---
    IDH_HTML_CAN_MODULE_ID
    ---endhelp---

config DRV_CAN_USE_TIMEOUT_ID${INSTANCE}
    bool "Use Timeout Counter?"
    default n
    ---help---
    Checking this option enables timeout.
    ---endhelp---

config DRV_CAN_TIMEOUT_TOP_ID${INSTANCE}
    int "Timeout Top Countdown Value"
    depends on DRV_CAN_USE_TIMEOUT_ID${INSTANCE}
    default 20000
    range 1 65535
    ---help---
    Value for timeout to start with.
    ---endhelp---

config DRV_CAN_TIMEOUT_MODE_IDX${INSTANCE}
    string "Timeout Mode"
    depends on DRV_CAN_USE_TIMEOUT_ID${INSTANCE}
    range CAN_TIMEOUT_MODES
    default "CAN_TIMEOUT_MODE_TX_EVENT"
    ---help---
    Which timeout mode to use
    ---endhelp---

config DRV_CAN_USE_TIMESTAMP_ID${INSTANCE}
    bool "Use Timestamps?"
    default n
    ---help---
    Checking this option enables timestamps.
    ---endhelp---

config DRV_CAN_TIMESTAMP_MODE_IDX${INSTANCE}
    string "Timestamp Mode"
    depends on DRV_CAN_USE_TIMESTAMP_ID${INSTANCE}
    range CAN_TIMESTAMP_MODES
    default "CAN_TIMESTAMP_MODE_TCP_INC"
    ---help---
    Which timestamp mode to use
    ---endhelp---

config DRV_CAN_TIME_DIVIDER_ID${INSTANCE}
    int "Timeout/Timestamp divider"
    depends on DRV_CAN_USE_TIMEOUT_ID${INSTANCE} || DRV_CAN_USE_TIMESTAMP_ID${INSTANCE}
    default 2
    range 1 16
    ---help---
    Divider for clock for both Timeout and Timestamp.
    ---endhelp---


config DRV_CAN_INTERRUPT_MODE_ID${INSTANCE}
    bool "Add interrupts to support CAN ${INSTANCE}?"
    depends on USE_DRV_CAN
    select USE_SYS_INT_NEEDED
    default n
    ---help---
    Checking this option would add supporting interrupts.
    ---endhelp---

    config DRV${INSTANCE}_CAN_INTERRUPT_PRIORITY
        string "CAN Interrupt Priority (0 is highest)"
        range INT_PRIORITY_LEVEL_PIC32C
        default "INT_PRIORITY_LEVEL0"
        depends on DRV_CAN_INTERRUPT_MODE_ID${INSTANCE}
        ---help---
        IDH_HTML_INT_PRIORITY_LEVEL
        ---endhelp---

    config DRV${INSTANCE}_CAN_INTERRUPT_HANDLER_NAME
        string "Interrupt handler name"
        default "CAN${INSTANCE}_Handler"
        depends on DRV_CAN_INTERRUPT_MODE_ID${INSTANCE}

    config DRV_CAN_INT_TX_COMPLETED_IDX${INSTANCE}
        bool "TX Completed - TCE"
        default n
        depends on DRV_CAN_INTERRUPT_MODE_ID${INSTANCE}

    config DRV_CAN_INT_TX_EMPTY_IDX${INSTANCE}
        bool "TX FIFO Empty - TFEE"
        default n
        depends on DRV_CAN_INTERRUPT_MODE_ID${INSTANCE}

    config DRV_CAN_INT_TIMEOUT_IDX${INSTANCE}
        bool "Timeout Occurred"
        default n
        depends on DRV_CAN_INTERRUPT_MODE_ID${INSTANCE}


config DRV_CAN_BAUD_RATE_IDX${INSTANCE}
    string "Desired Baud Rate"
    range CAN_BAUDRATES
    default "CAN_BAUD_1M"
    ---help---
    Select BAUD rate. Adjust PCLK 5 divider if needed
    ---endhelp---


config DRV_CAN_OPERATION_MODE_IDX${INSTANCE}
    string "Operation Mode"
    range CAN_OPERATION_MODES
    default "CAN_NORMAL_MODE"
    ---help---
    IDH_HTML_CAN_OPERATION_MODES
    ---endhelp---

config DRV_CAN_RX_CHANNELS_IDX${INSTANCE}
    int "Number of RX Channels"
    range 0 2
    default 1

config DRV_CAN_TX_FIFO_ELEMENTS_IDX${INSTANCE}
    int "Number of TX FIFO Elements"
    range 1 32
    default 1

config DRV_CAN_TX_ELEMENT_BYTES_IDX${INSTANCE}
    string "Number of bytes in TX FIFO Element"
    range CAN_FIFO_ELEMENT_BYTES
    default "CAN_FIFO_8_BYTE"
    depends on DRV_CAN_OPERATION_MODE_IDX${INSTANCE} = "CAN_FD_MODE"

config DRV_CAN_TX_WATERMARK_LEVEL_IDX${INSTANCE}
    int "TX FIFO Watermark Level"
    range 0 DRV_CAN_TX_FIFO_ELEMENTS_IDX${INSTANCE}
    default 0

config DRV_CAN_STD_REJECT_IDX${INSTANCE}
    bool "Reject All Standard IDs?"
    default n

config DRV_CAN_STD_FILTERS_IDX${INSTANCE}
    int "Number of Standard ID Filters"
    range 0 128
    default 0

config DRV_CAN_STD_NOMATCH_IDX${INSTANCE}
    string "Where to put non-matching Standard frames:"
    range CAN_NOMATCH
    default "Message Rejected"

config DRV_CAN_EXT_REJECT_IDX${INSTANCE}
    bool "Reject All Extended IDs?"
    default n

config DRV_CAN_EXT_FILTERS_IDX${INSTANCE}
    int "Number of Extended ID Filters"
    range 0 64
    default 0

config DRV_CAN_EXT_NOMATCH_IDX${INSTANCE}
    string "Where to put non-matching Extended frames:"
    range CAN_NOMATCH
    default "Message Rejected"

<#if INSTANCE == 0>
source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv0_can_std_filter_pic32c.ftl" 128 instances
source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv0_can_ext_filter_pic32c.ftl" 64 instances
source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv0_can_channel_pic32c.ftl" 2 instances
</#if>

<#if INSTANCE == 1>
source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv1_can_std_filter_pic32c.ftl" 128 instances
source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv1_can_ext_filter_pic32c.ftl" 64 instances
source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv1_can_channel_pic32c.ftl" 2 instances
</#if>

endif
