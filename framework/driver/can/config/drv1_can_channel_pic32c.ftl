config DRV1_CAN_CHANNEL_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
	default n if DRV1_CAN_CHANNEL_NUMBER_GT_${INSTANCE} = n
</#if>
	default n if DRV_CAN_RX_CHANNELS_IDX1 = ${INSTANCE+1}
	default y

config DRV1_CAN_RX${INSTANCE}_CHANNEL
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
	             && DRV1_CAN_CHANNEL_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver 1 RX Channel ${INSTANCE}"
    default y

ifblock DRV1_CAN_RX${INSTANCE}_CHANNEL

config DRV1_CAN_RX${INSTANCE}_FIFO_OVERWR
    bool "Use Overwrite mode instead of Blocking mode for RX FIFO"
    default n

config DRV1_CAN_RX${INSTANCE}_CHANNEL_ELEMENTS
    int "Number of RX FIFO Elements"
    range 1 64
    default 4

config DRV1_CAN_RX${INSTANCE}_ELEMENT_BYTES
    string "Number of bytes in RX FIFO Element"
    range CAN_FIFO_ELEMENT_BYTES
    default "CAN_FIFO_8_BYTE"

config DRV1_CAN_RX${INSTANCE}_FIFO_WATERMARK
    int "RX FIFO Watermark Level"
    range 0 DRV1_CAN_RX${INSTANCE}_CHANNEL_ELEMENTS
    default 0

config DRV1_CAN_INT_RX${INSTANCE}_NEW
    bool "Use RX FIFO New Message Interrupt"
    depends on DRV_CAN_INTERRUPT_MODE_ID1
    default n

config DRV1_CAN_INT_RX${INSTANCE}_WATERMARK
    bool "Use RX FIFO Watermark Reached Interrupt"
    depends on DRV_CAN_INTERRUPT_MODE_ID1
    default n


endif
