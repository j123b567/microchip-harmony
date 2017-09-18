config DRV1_CAN_STD_FILTER_NUMBER_GT_${INSTANCE+1}
    bool
<#if INSTANCE != 0>
    default n if DRV1_CAN_STD_FILTER_NUMBER_GT_${INSTANCE} = n
</#if>
    default n if DRV_CAN_STD_FILTERS_IDX1 = ${INSTANCE+1}
    default y

config DRV1_CAN_STD_FILTER${INSTANCE}
    depends on DRV_CAN_STD_FILTERS_IDX1 != 0
<#if INSTANCE != 0>
             && DRV1_CAN_STD_FILTER_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver 1 Standard Filter ${INSTANCE}"
    default n


ifblock DRV1_CAN_STD_FILTER${INSTANCE}


config DRV1_CAN_STD_FILTER${INSTANCE}_TYPE
    string "CAN Filter Identifier Mode"
    range CAN_STD_FILTER_TYPE
    default "CAN_STD_FILTER_RANGE"
    ---help---
    ---endhelp---

config DRV1_CAN_STD_FILTER${INSTANCE}_SFID1
    int "Standard Filter ID 1"
    range 0 2047
    default 0

config DRV1_CAN_STD_FILTER${INSTANCE}_SFID2
    int "Standard Filter ID 2"
    range 0 2047
    default 0

config DRV1_CAN_STD_FILTER${INSTANCE}_CONFIG
    string "CAN Filter Element Configuration"
    range CAN_FILTER_CONFIG
    default "CAN_FILTER_CONFIG_FIFO_0"
    ---help---
    ---endhelp---


endif

