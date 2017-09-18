config DRV_CAN_EXT_FILTER_NUMBER_GT_${INSTANCE+1}
    bool
<#if INSTANCE != 0>
    default n if DRV_CAN_EXT_FILTER_NUMBER_GT_${INSTANCE} = n
</#if>
    default n if DRV_CAN_EXT_FILTERS_IDX0 = ${INSTANCE+1}
    default y

config DRV0_CAN_EXT_FILTER${INSTANCE}
    depends on DRV_CAN_EXT_FILTERS_IDX0 != 0
<#if INSTANCE != 0>
             && DRV_CAN_EXT_FILTER_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver 0 Extended Filter ${INSTANCE}"
    default n


ifblock DRV0_CAN_EXT_FILTER${INSTANCE}


config DRV0_CAN_EXT_FILTER${INSTANCE}_TYPE
    string "Filter Identifier Mode"
    range CAN_EXT_FILTER_TYPE
    default "CAN_EXT_FILTER_RANGE"
    ---help---
    ---endhelp---

config DRV0_CAN_EXT_FILTER${INSTANCE}_EFID1
    int "Extended Filter ID 1"
    default 0

config DRV0_CAN_EXT_FILTER${INSTANCE}_EFID2
    int "Extended Filter ID 2"
    default 0

config DRV0_CAN_EXT_FILTER${INSTANCE}_CONFIG
    string "Filter Element Configuration"
    range CAN_FILTER_CONFIG
    default "CAN_FILTER_CONFIG_FIFO_0"
    ---help---
    ---endhelp---

endif
