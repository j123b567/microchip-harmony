config DRV_AFEC1_CHANNEL_SET_NUMBER_GT_${INSTANCE+1}
    depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC 
	bool	
<#if INSTANCE != 0>
    default n if DRV_AFEC1_CHANNEL_SET_NUMBER_GT_${INSTANCE} = n 
</#if>
    default n if DRV_AFEC1_CHANNEL_SET_NUMBER = ${INSTANCE+1}
    default y
   
config DRV_AFEC1_CHANNELSET_IDX${INSTANCE}
    depends on USE_DRV_AFEC
    depends on DRV_ADC_TYPE_AFEC
<#if INSTANCE != 0>
                 && DRV_AFEC1_CHANNEL_SET_NUMBER_GT_${INSTANCE}
</#if>
    string "Driver Instance 1 Channelset ${INSTANCE}"
    default 0x001
    ---help---
    IDH_HTML_DRV_AFEC_CHANNELSET_IDX
    ---endhelp---
    