config DRV_AFEC_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
	bool	
<#if INSTANCE != 0>
    default n if DRV_AFEC_INSTANCES_NUMBER_GT_${INSTANCE} = n 
</#if>
    default n if DRV_AFEC_INSTANCE_NUMBER = ${INSTANCE+1}
    default y

config DRV_AFEC_INST_IDX${INSTANCE}
    depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
<#if INSTANCE != 0>
                 && DRV_AFEC_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "AFEC Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_AFEC_INSTANCES_NUMBER
    ---endhelp---
    
ifblock DRV_AFEC_INST_IDX${INSTANCE}

config DRV_AFEC_MODULE_ID${INSTANCE}
    string "AFEC module ID"
    depends on USE_DRV_AFEC
    depends on DRV_ADC_TYPE_AFEC
    range AFEC_MODULE_ID
    <#if INSTANCE = 0>
    default "AFEC_ID_0"
    </#if>
    <#if INSTANCE = 1>
    default "AFEC_ID_1"
    </#if>
    set SYS_CLK_PMC_ID_AFEC0 optionally to y if DRV_AFEC_MODULE_ID${INSTANCE} = "AFEC_ID_0"
    set SYS_CLK_PMC_ID_AFEC1 optionally to y if DRV_AFEC_MODULE_ID${INSTANCE} = "AFEC_ID_1"

menu "Global Setup"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC

config DRV_AFEC_INTERRUPT_SOURCE${INSTANCE}
    string "Interrupt Source"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default "AFEC0_IRQn" if DRV_AFEC_MODULE_ID${INSTANCE}="AFEC_ID_0"
    default "AFEC1_IRQn" if DRV_AFEC_MODULE_ID${INSTANCE}="AFEC_ID_1"
	persistent
    ---help---
    IDH_HTML_DRV_AFEC_INTERRUPT_SOURCE
    ---endhelp---

config DRV_AFEC_RESOLUTION${INSTANCE}
    string "AFEC Resolution"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
	range AFEC_ENUM_RESOLUTION 
    default "12 Bit"
	---help---
    IDH_HTML_DRV_AFEC_RESOLUTION
    ---endhelp---

config DRV_AFEC_RESOLUTION_VALUE${INSTANCE}
    string
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default "DRV_AFEC_RESOLUTION_12_BIT" if DRV_AFEC_RESOLUTION${INSTANCE}="12 Bit"
    default "DRV_AFEC_RESOLUTION_13_BIT" if DRV_AFEC_RESOLUTION${INSTANCE}="13 Bit"
    default "DRV_AFEC_RESOLUTION_14_BIT" if DRV_AFEC_RESOLUTION${INSTANCE}="14 Bit"
    default "DRV_AFEC_RESOLUTION_15_BIT" if DRV_AFEC_RESOLUTION${INSTANCE}="15 Bit"
    default "DRV_AFEC_RESOLUTION_16_BIT" if DRV_AFEC_RESOLUTION${INSTANCE}="16 Bit"
    
config DRV_AFEC_SIGN_MODE${INSTANCE}
    string "AFEC Sign Mode"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
	range AFEC_ENUM_SIGN_MODE 
    default "All Unsigned"
	---help---
    IDH_HTML_DRV_AFEC_SIGN_MODE
    ---endhelp---

config DRV_AFEC_SIGN_MODE_VALUE${INSTANCE}
    string
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default "DRV_AFEC_SIGN_MODE_ALL_UNSIGN" if DRV_AFEC_SIGN_MODE${INSTANCE}="All Unsigned"
    default "DRV_AFEC_SIGN_MODE_ALL_SIGN" if DRV_AFEC_SIGN_MODE${INSTANCE}="All Signed"
    default "DRV_AFEC_SIGN_MODE_SE_UNSIGN_DF_SIGN" if DRV_AFEC_SIGN_MODE${INSTANCE}="SingleEnded-Unsigned, Differential-Signed"
    default "DRV_AFEC_SIGN_MODE_SE_SIGN_DF_UNSIGN" if DRV_AFEC_SIGN_MODE${INSTANCE}="SingleEnded-Signed, Differential-Unsigned"

config DRV_AFEC_DUAL_CHANNELS${INSTANCE}
    string "Dual Sampling Enabled Channels"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default 0x000
    comment "**** This value indicates multiple dual-sampled channel pairs ****"
    comment "**** Eg: Value of 0x041 indicates that AN0 and AN6 will be sampled at the same time ****"
    ---help---
    IDH_HTML_DRV_AFEC_DUAL_CHANNELS
    ---endhelp---
	
config DRV_AFEC_DIFFERENTIAL_CHANNELS${INSTANCE}
    string "Differential Channels"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default 0x000
    comment "**** Eg: Value of 0x003 indicates that AN0 and AN1 forms a differntial pair ****"
    ---help---
    IDH_HTML_DRV_AFEC_DIFFERENTIAL_CHANNELS
    ---endhelp---
endmenu

<#if INSTANCE == 0>
menu "Channelset Configuration"
config DRV_AFEC0_CHANNEL_SET_NUMBER
    int "Number of channel sets for driver instance 0"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default 2
    range 0 4
    comment "**** Channel set defines a group of AFEC inputs the clients can use ****"
    comment "**** Each bit in a channelset corresponds to a channel ****"
    comment "**** Example: Value 0x003 for a channel set means, AD0 and AD1 are the only channels in the channel set ****"
    ---help---
    IDH_HTML_DRV_AFEC0_CHANNEL_SET_NUMBER
    ---endhelp---
source "$HARMONY_VERSION_PATH/framework/driver/afec/config/drv_afec0_channelset_idx.ftl" 4 instances
endmenu
</#if>

<#if INSTANCE == 1>
menu "Channelset Configuration"
config DRV_AFEC1_CHANNEL_SET_NUMBER
    int "Number of channel sets for driver instance 1"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default 2
    range 0 4
    comment "**** Channel set defines a group of AFEC inputs the clients can use ****"
    comment "**** Each bit in a channelset corresponds to a channel ****"
    comment "**** Example: Value 0x003 for a channel set means, AD0 and AD1 are the only channels in the channel set ****"
    ---help---
    IDH_HTML_DRV_AFEC_CHANNEL_SET_NUMBER
    ---endhelp---
source "$HARMONY_VERSION_PATH/framework/driver/afec/config/drv_afec1_channelset_idx.ftl" 4 instances
endmenu
</#if>

menu "Timing Configuration"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    
config DRV_AFEC_CLOCK_DIV${INSTANCE}
    int "Clock Division Factor for AFEC"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default 14
    comment "**** (Master Clock frequency / (Clock Division Factor for AFEC + 1)) should be between 4MHz to 40MHz ****"
	---help---
	IDH_HTML_AFEC_CLOCK_DIV
	---endhelp---
    
config DRV_AFEC_MASTER_CLOCK${INSTANCE}
    string "Master Clock frequency"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default SYS_CLK_MASTERCLK_FREQ
	persistent
    ---help---
    IDH_HTML_DRV_AFEC_MASTER_CLOCK
    ---endhelp---

config DRV_AFEC_BIAS_CONTROL${INSTANCE}
    string "AFEC Bias Current Control"
 	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default "Configure for AFEC sampling frequency higher than 1MHz"
    range DRV_AFEC_BIAS_CURRENT
	---help---
    IDH_HTML_DRV_AFEC_BIAS_CONTROL
    ---endhelp---
    
config DRV_AFEC_BIAS_CONTROL_VALUE${INSTANCE}
    string
 	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default "DRV_AFEC_BIAS_CURRENT_01" if DRV_AFEC_BIAS_CONTROL${INSTANCE}="Configure for AFEC sampling frequency less than 500kHz"
    default "DRV_AFEC_BIAS_CURRENT_10" if DRV_AFEC_BIAS_CONTROL${INSTANCE}="Configure for AFEC sampling frequency upto 1MHz"
    default "DRV_AFEC_BIAS_CURRENT_11" if DRV_AFEC_BIAS_CONTROL${INSTANCE}="Configure for AFEC sampling frequency higher than 1MHz"
    
config DRV_AFEC_STARTUP_TIME${INSTANCE}
    string "Start-up time for AFEC"
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default "64 AFEC Clock Cycles"
    range DRV_AFEC_STARTUP
    comment "**** Select startup time which meets the specification given in the datasheet ****"
    ---help---
	IDH_HTML_DRV_AFEC_STARTUP_TIME
	---endhelp---

config DRV_AFEC_STARTUP_TIME_VALUE${INSTANCE}
    string
	depends on USE_DRV_AFEC
	depends on DRV_ADC_TYPE_AFEC
    default "DRV_AFEC_STARTUP_SUT0" if DRV_AFEC_STARTUP_TIME${INSTANCE}="0 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT8" if DRV_AFEC_STARTUP_TIME${INSTANCE}="8 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT16" if DRV_AFEC_STARTUP_TIME${INSTANCE}="16 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT24" if DRV_AFEC_STARTUP_TIME${INSTANCE}="24 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT64" if DRV_AFEC_STARTUP_TIME${INSTANCE}="64 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT80" if DRV_AFEC_STARTUP_TIME${INSTANCE}="80 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT96" if DRV_AFEC_STARTUP_TIME${INSTANCE}="96 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT112" if DRV_AFEC_STARTUP_TIME${INSTANCE}="112 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT512" if DRV_AFEC_STARTUP_TIME${INSTANCE}="512 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT576" if DRV_AFEC_STARTUP_TIME${INSTANCE}="576 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT640" if DRV_AFEC_STARTUP_TIME${INSTANCE}="640 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT704" if DRV_AFEC_STARTUP_TIME${INSTANCE}="704 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT768" if DRV_AFEC_STARTUP_TIME${INSTANCE}="768 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT832" if DRV_AFEC_STARTUP_TIME${INSTANCE}="832 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT896" if DRV_AFEC_STARTUP_TIME${INSTANCE}="896 AFEC Clock Cycles"
    default "DRV_AFEC_STARTUP_SUT960" if DRV_AFEC_STARTUP_TIME${INSTANCE}="960 AFEC Clock Cycles"
   
endmenu

<#if INSTANCE == 0>
menu "Channel Specific Configuration"
source "$HARMONY_VERSION_PATH/framework/driver/afec/config/drv_afec0_channels_conf.ftl" 12 instances
endmenu
</#if>

<#if INSTANCE == 1>
menu "Channel Specific Configuration"
source "$HARMONY_VERSION_PATH/framework/driver/afec/config/drv_afec1_channels_conf.ftl" 12 instances
endmenu
</#if>

endif
