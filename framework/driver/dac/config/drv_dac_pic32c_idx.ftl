config DRV_DAC_CHANNEL_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_DAC
<#if INSTANCE != 0>
    default n if ( (DRV_DAC_CHANNEL_NUMBER_GT_${INSTANCE} = n)&&(DRV_DAC_DRIVER_MODE = "SINGLE_ENDED") )
    default n if ( (DRV_DAC_CHANNEL_NUMBER_GT_${INSTANCE} = n)&&(DRV_DAC_DRIVER_MODE = "DIFFERENTIAL") )    
</#if>  
    default n if ( (DRV_DAC_CHANNEL_NUMBER = ${INSTANCE+1}) && (DRV_DAC_DRIVER_MODE = "SINGLE_ENDED") )
    default n if ( (DRV_DAC_CHANNEL_NUMBER_SINGLE = ${INSTANCE+1}) && (DRV_DAC_DRIVER_MODE = "DIFFERENTIAL") )
    default y
    
config DRV_DAC_INST_IDX${INSTANCE}
    depends on USE_DRV_DAC
<#if INSTANCE != 0>
                 && DRV_DAC_CHANNEL_NUMBER_GT_${INSTANCE}
</#if>
	bool "Channel ${INSTANCE}" if DRV_DAC_DRIVER_MODE = "SINGLE_ENDED"
    bool "Channel Configuration" if DRV_DAC_DRIVER_MODE = "DIFFERENTIAL"
    default y


ifblock DRV_DAC_INST_IDX${INSTANCE}
    config DRV_DAC_MODE_OPTIONS_IDX${INSTANCE}
        string "Mode Options"
        depends on USE_DRV_DAC
        range DRV_DAC_MODE_OPTIONS
        default "TRIGGER_MODE"	

    ifblock DRV_DAC_MODE_OPTIONS_IDX${INSTANCE} = "MAX_SPEED_MODE" 
        config DRV_DAC_SPEED_MODE1_IDX${INSTANCE}
            string "Set Bias Current for Max. Samples"
            range DRV_DAC_SPEED_MAX_MODE 
            depends on USE_DRV_DAC
            persistent if DRV_DAC_MODE_OPTIONS_IDX${INSTANCE} = "MAX_SPEED_MODE"
            default "1M"
    endif 

    ifblock DRV_DAC_MODE_OPTIONS_IDX${INSTANCE} = "TRIGGER_MODE"
    config DRV_DAC_SPEED_MODE2_IDX${INSTANCE}
        string "Set Bias Current for Max. Samples"
        range DRV_DAC_SPEED_TRIGGER_MODE 
        depends on USE_DRV_DAC
        default "1M"
    endif 
	
    ifblock DRV_DAC_MODE_OPTIONS_IDX${INSTANCE} = "FREE_RUNNING_MODE" 
    config DRV_DAC_SPEED_MODE3_IDX${INSTANCE}
        string "Set Bias Current for Max. Samples"
        range DRV_DAC_SPEED_FREE_MODE 
        depends on USE_DRV_DAC
        default "1M"
    endif 	
	
    config DRV_DAC_TRIGGER_EVENT_SELECT_IDX${INSTANCE}
        string "Select Trigger Source"
        depends on USE_DRV_DAC
        depends on DRV_DAC_MODE_OPTIONS_IDX${INSTANCE} = "TRIGGER_MODE"
        range DRV_DAC_TRIGGER_SOURCE
        default "EXTERNAL_INPUT_DATRG"
        
    config DRV_DAC_INTERPOLATE_SELECT_IDX${INSTANCE}
        string "Select Oversampling Ratio"
        depends on USE_DRV_DAC
        depends on DRV_DAC_MODE_OPTIONS_IDX${INSTANCE} = "TRIGGER_MODE"
        range DRV_DAC_OVERSAMPLE_RATIO
        default "OSR_1"
endif