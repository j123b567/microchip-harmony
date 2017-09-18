config DRV_I2S_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on USE_DRV_I2S
    bool
<#if INSTANCE != 0>
    default n if DRV_I2S_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
    default n if DRV_I2S_INSTANCES_NUMBER = ${INSTANCE+1}
    default y
    
config DRV_I2S_INST_IDX${INSTANCE}
    depends on USE_DRV_I2S 
<#if INSTANCE != 0>
                && DRV_I2S_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "I2S Driver Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_I2S_INSTANCES_NUMBER
    ---endhelp---
    
ifblock DRV_I2S_INST_IDX${INSTANCE}

config DRV_I2S_SSC_INSTANCES_NUMBER${INSTANCE}
    int "SSC Instance number"
    range 0 1
    default 1
    persistent
    ---help---
    IDH_HTML_SPI_MODULE_ID
    ---endhelp---

config DRV_I2S_SSC_CLIENTS_NUMBER${INSTANCE}
    int "SSC Client number"
    range 0 6
    default 1
    ---help---
    IDH_HTML_SPI_MODULE_ID
    ---endhelp---
        
config DRV_I2S_PERIPHERAL_ID_IDX${INSTANCE}
    string "I2S Interface Module ID"
    range DRV_I2S_MODULE_ID
    default "SSC_ID_0"
    persistent
    ---help---
    IDH_HTML_SPI_MODULE_ID
    ---endhelp---

config DRV_I2S_SSC_CLOCK_DIVIDER${INSTANCE}
    int "Clock divider for TX and RX Block"
    depends on USE_DRV_I2S
    range 0 4095
    default 0
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_CLOCK_SELECTION${INSTANCE}
    string "Receiver Clock Selection"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_CLK_SEL
    default "DRV_I2S_SSC_RX_TK_SIGNAL"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_CLOCK_OUTPUT_MODE${INSTANCE}
    string "Receiver Clock Output Mode"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_CLK_OUTPUT
    default "I2S_SSC_NONE_RK_PIN_AS_INPUT"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_CLOCK_INVERSE${INSTANCE}
    string "Receiver Clock Inverse"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_CLK_INV
    default "I2S_SSC_RX_RISING_FALLING_EDGE"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_CLOCK_GATING${INSTANCE}
    string "Receiver Clock Gating"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_CLK_GATE
    default "I2S_SSC_RX_CLK_CONTINUOUS"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_START_SELECTION${INSTANCE}
    string "Receiver Start Selection"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_START_SEL
    default "I2S_SSC_RX_CONTINUOUS"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_LOOPMODE${INSTANCE}
    string "Receiver Loop Mode or Normal Mode"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_LOOP_MODE
    default "I2S_SSC_NORMAL_MODE"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_START_DELAY${INSTANCE}
    int "Receiver Start Delay"
    depends on USE_DRV_I2S
    range 0 255
    default "0"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_PERIOD_DIVIDER_SELECTION${INSTANCE}
    int "Receiver Period Divider Selection"
    depends on USE_DRV_I2S
    range 0 255
    default "0"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_DATA_LENGTH${INSTANCE}
    int "Receiver Data Length(n+1 is actual data size)"
    depends on USE_DRV_I2S
    range 0 31
    default "15"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_MSBFIRST${INSTANCE}
    string "Receiver MSB or LSB First"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_MSB_FIRST
    default "I2S_SSC_RX_MSB_SAMPLED_FIRST"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_FS_LENGTH${INSTANCE}
    int "Receiver Frame Sync Length"
    depends on USE_DRV_I2S
    range 0 15
    default "0"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_DATA_NUM_PER_FRAME${INSTANCE}
    int "Receiver Data Number Per Frame"
    depends on USE_DRV_I2S
    range 0 15
    default 0
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_FS_OUTPUT_SELECTION${INSTANCE}
    string "Receiver Frame Sync Output Selection"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_FS_OUT_SEL
    default "I2S_SSC_RX_FS_OUTPUT_NONE"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_RX_FS_EDGEDETECTION${INSTANCE}
    string "Receiver Frame Sync Edge Detection"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_RX_FS_EDGE_DETECTION
    default "I2S_SSC_RX_FS_EDGE_POSITIVE"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_CLOCK_SELECTION${INSTANCE}
    string "Transmitter Clock Selection"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_CLK_SEL
    default "DRV_I2S_SSC_TX_DIVIDED_CLOCK"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_CLOCK_OUTPUT_MODE${INSTANCE}
    string "Transmitter Clock Output Mode"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_CLK_OUTPUT
    default "I2S_SSC_TX_CONTINUOUS_CLK_OUTPUT"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_CLOCK_INVERSE${INSTANCE}
    string "Transmitter Clock Inverse"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_CLK_INV
    default "I2S_SSC_TX_FALLING_RISING_EDGE"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_CLOCK_GATING${INSTANCE}
    string "Transmitter Clock Gating"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_CLK_GATE
    default "I2S_SSC_TX_CLK_CONTINUOUS"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_START_SELECTION${INSTANCE}
    string "Transmitter Start Selection"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_START_SEL
    default "I2S_SSC_TX_CONTINUOUS"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_START_DELAY${INSTANCE}
    int "Transmitter Start Delay"
    depends on USE_DRV_I2S
    range 0 255
    default "0"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_PERIOD_DIVIDER_SELECTION${INSTANCE}
    int "Transmitter Period Divider Selection"
    depends on USE_DRV_I2S
    range 0 255
    default "0"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_DATA_LENGTH${INSTANCE}
    int "Transmitter Data Length(n+1 is actual data size)"
    depends on USE_DRV_I2S
    range 0 31
    default "15"
    ---help---
    ---endhelp---
    
config DRV_I2S_SSC_TX_DEFAULTDATA${INSTANCE}
    string "Transmitter Default Data to be Transmitted"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_DEFAULT_DATA
    default "I2S_SSC_TX_TD_OUTPUT_ZEROS"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_MSBFIRST${INSTANCE}
    string "Transmitter MSB or LSB First"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_MSB_FIRST
    default "I2S_SSC_TX_MSB_SAMPLED_FIRST"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_DATA_NUM_PER_FRAME${INSTANCE}
    int "Transmitter Data Number Per Frame"
    depends on USE_DRV_I2S
    range 0 15
    default "0"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_FS_LENGTH${INSTANCE}
    int "Transmitter Frame Sync Length"
    depends on USE_DRV_I2S
    range 0 15
    default "0"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_FS_OUTPUT_SELECTION${INSTANCE}
    string "Transmitter Frame Sync Output Selection"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_FS_OUT_SEL
    default "I2S_SSC_TX_FS_OUTPUT_TOGGLING"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_FS_DATAEN${INSTANCE}
    string "Transmitter Frame Sync Data Enable"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_FS_DATA_ENABLE
    default "I2S_SSC_TX_OUT_DEF_VALUE"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_FS_EDGEDETECTION${INSTANCE}
    string "Transmitter Frame Sync Edge Detection"
    depends on USE_DRV_I2S
    range DRV_I2S_SSC_TX_FS_EDGE_DETECTION
    default "I2S_SSC_TX_FS_EDGE_POSITIVE"
    ---help---
    ---endhelp---

config DRV_I2S_SSC_TX_FSLENGTH_EXTENSION${INSTANCE}
    int "Transmitter Frame Sync Length Extension"
    depends on USE_DRV_I2S
    range 0 15
    default "0"
    ---help---
    ---endhelp---

menu "RTOS Configuration"
    depends on USE_DRV_I2S
    depends on USE_3RDPARTY_RTOS
    depends on DRV_I2S_IMPL = "DYNAMIC"

config DRV_I2S_RTOS_IDX${INSTANCE}
    string "Run Library Tasks As"
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config DRV_I2S_IDX${INSTANCE}_RTOS_TASK_SIZE
    int "Task Size"
    depends on DRV_I2S_RTOS_IDX${INSTANCE} = "Standalone"
    default 1024

config DRV_I2S_IDX${INSTANCE}_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on DRV_I2S_RTOS_IDX${INSTANCE} = "Standalone"
    default 1

config DRV_I2S_IDX${INSTANCE}_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on DRV_I2S_RTOS_IDX${INSTANCE} = "Standalone"
    default y

config DRV_I2S_IDX${INSTANCE}_RTOS_DELAY
    int "Task Delay"
    depends on DRV_I2S_RTOS_IDX${INSTANCE} = "Standalone"
    depends on DRV_I2S_IDX${INSTANCE}_RTOS_USE_DELAY
    default 1000
endmenu

config QUEUE_SIZE_TX_IDX${INSTANCE}
    int "Queue Size Transmit"
    depends on USE_DRV_I2S
    default 3
    range 1 65535
    ---help---
    IDH_HTML_DRV_I2S_INIT
    ---endhelp---

config QUEUE_SIZE_RX_IDX${INSTANCE}
    int "Queue Size Receive"
    depends on USE_DRV_I2S
    default 2
    range 1 65535	
    ---help---
    IDH_HTML_DRV_I2S_INIT
    ---endhelp---
    
config 	DRV_I2S_TX_DMA_CHANNEL_IDX${INSTANCE}
    int "Transmit DMA Channel Instance"
    depends on DRV_I2S_DMA_INTERRUPTS_ENABLE
    range 0	3
	default 0
    ---help---
    IDH_HTML_DRV_I2S_TRANSMIT_DMA_CHANNEL
    ---endhelp---

config 	DRV_I2S_RX_DMA_CHANNEL_IDX${INSTANCE}
    int "Receive DMA Channel Instance"
    depends on DRV_I2S_DMA_INTERRUPTS_ENABLE
    range 0	3
	default 1
    ---help---
    IDH_HTML_DRV_I2S_TRANSMIT_DMA_CHANNEL
    ---endhelp---
    
config DRV_I2S_POWER_STATE_IDX${INSTANCE}
    string "Power State" 
    range SYS_MODULE_POWER_STATE
    default "SYS_MODULE_POWER_RUN_FULL"
    ---help---
    IDH_HTML_SYS_MODULE_INIT
    ---endhelp---

endif

