config DRV_AFEC0_CHANNELS_CONF_${INSTANCE+1}
	depends on DRV_ADC_TYPE_AFEC
    bool
<#if INSTANCE != 0>
	default n if DRV_AFEC0_CHANNELS_CONF_${INSTANCE} = n     
</#if>	
	default y
	
config DRV_AFEC0_CHANNEL_CONF_IDX${INSTANCE}
	depends on DRV_ADC_TYPE_AFEC
<#if INSTANCE != 0>
	             && DRV_AFEC0_CHANNELS_CONF_${INSTANCE}
</#if>
    bool "Configure Channel ${INSTANCE}"
    default n
    ---help---
    IDH_HTML_DRV_AFEC_CHANNELS_CONF_IDX
    ---endhelp---

config DRV_AFEC0_CHANNEL_GAIN${INSTANCE}
	depends on DRV_ADC_TYPE_AFEC
    depends on DRV_AFEC0_CHANNEL_CONF_IDX${INSTANCE}
    string "Channel ${INSTANCE} Gain"
    default "DRV_AFEC_CHANNEL_GAIN1"
    range DRV_AFEC_CHANNEL_GAIN
    ---help---
    IDH_HTML_DRV_AFEC_CHANNEL_GAIN
    ---endhelp---
    
config DRV_AFEC0_CHANNEL_OFFSET${INSTANCE}
	depends on DRV_ADC_TYPE_AFEC
    depends on DRV_AFEC0_CHANNEL_CONF_IDX${INSTANCE}
    string "Channel ${INSTANCE} Analog Offset"
    default 512
    ---help---
    IDH_HTML_DRV_AFEC_CHANNEL_OFFSET
    ---endhelp---
    