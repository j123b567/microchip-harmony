config DRV_I2C_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_I2C
<#if INSTANCE != 0>
	default n if DRV_I2C_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_I2C_INSTANCES_NUMBER = ${INSTANCE+1}
	default y

config DRV_I2C_INST_IDX${INSTANCE}
    depends on USE_DRV_I2C 
<#if INSTANCE != 0>
	             && DRV_I2C_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "I2C Driver Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_I2C_INSTANCES_NUMBER
    ---endhelp---

ifblock DRV_I2C_INST_IDX${INSTANCE}

menu "RTOS Configuration (Instance ${INSTANCE})"
    depends on USE_DRV_I2C
    depends on USE_3RDPARTY_RTOS
    depends on DRV_I2C_DRIVER_MODE = "DYNAMIC"

config DRV_I2C_RTOS_IDX${INSTANCE}
    string "Run This Driver Instance As"
    depends on DRV_I2C_DRIVER_MODE = "DYNAMIC"
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Combined with System Tasks"

config DRV_I2C_IDX${INSTANCE}_RTOS_TASK_SIZE
    int "Task Size"
    depends on DRV_I2C_RTOS_IDX${INSTANCE} = "Standalone"
    default 1024

config DRV_I2C_IDX${INSTANCE}_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on DRV_I2C_RTOS_IDX${INSTANCE} = "Standalone"
    default 1

config DRV_I2C_IDX${INSTANCE}_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on DRV_I2C_RTOS_IDX${INSTANCE} = "Standalone"
    default y

config DRV_I2C_IDX${INSTANCE}_RTOS_DELAY
    int "Task Delay"
    depends on DRV_I2C_RTOS_IDX${INSTANCE} = "Standalone"
    depends on DRV_I2C_IDX${INSTANCE}_RTOS_USE_DELAY
    default 1000
endmenu

config DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE}
    string "I2C Module ID"
    depends on USE_DRV_I2C
    range TWI_MODULE_ID
    default "TWI_ID_0"
	set SYS_CLK_PMC_ID_TWI0 optionally to y if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE}="TWI_ID_0"
	set SYS_CLK_PMC_ID_TWI1 optionally to y if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE}="TWI_ID_1"
	set SYS_CLK_PMC_ID_TWI2 optionally to y if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE}="TWI_ID_2"	
    ---help---
    IDH_HTML_I2C_MODULE_ID
    ---endhelp---

config DRV_I2C_OPERATION_MODE_IDX${INSTANCE}
    string "Operation Mode"
	depends on USE_DRV_I2C
    range DRV_I2C_OP_MODE
    default "DRV_I2C_MODE_MASTER"
    ---help---
    IDH_HTML_PLIB_I2C_Initializing_the_I2C
    ---endhelp---

config DRV_I2C_MASTER_MODE_IDX${INSTANCE}
    bool
    depends on USE_DRV_I2C
    depends on DRV_I2C_OPERATION_MODE_IDX${INSTANCE} = "DRV_I2C_MODE_MASTER"
    default y

config DRV_I2C_SLAVE_MODE_IDX${INSTANCE}
    bool
    depends on USE_DRV_I2C
    depends on DRV_I2C_OPERATION_MODE_IDX${INSTANCE} = "DRV_I2C_MODE_SLAVE"
    default y

config DRV_I2C_INT_SRC_IDX${INSTANCE}
    string
    depends on USE_DRV_I2C
    depends on DRV_I2C_INTERRUPT_MODE
    default "TWI0_IRQn" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_0"
    default "TWI1_IRQn" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_1"
    default "TWI2_IRQn" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_2"

config DRV_I2C_INT_VECTOR_IDX${INSTANCE}
    string
    depends on USE_DRV_I2C
    depends on DRV_I2C_INTERRUPT_MODE
    default "TWI0_IRQn" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_0"
    default "TWI1_IRQn" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_1"
    default "TWI2_IRQn" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_2"



config DRV_I2C_INT_PRIORITY_IDX${INSTANCE}
    string "I2C Interrupt Priority"
    depends on USE_DRV_I2C
    depends on DRV_I2C_INTERRUPT_MODE
    range INT_PRIORITY_LEVEL_PIC32C
    default "INT_PRIORITY_LEVEL0"
    ---help---
    IDH_HTML_INT_PRIORITY_LEVEL
    ---endhelp---

config DRV_I2C_INT_HANDLER_IDX${INSTANCE}
    string "I2C Interrupt Handler"
    depends on USE_DRV_I2C
    depends on DRV_I2C_INTERRUPT_MODE
    default "TWI0_Handler" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_0"
    default "TWI1_Handler" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_1"
    default "TWI2_Handler" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_2"

config DRV_I2C_INT_HANDLER_TABLE_ENTRY_IDX${INSTANCE}
    string
    depends on USE_DRV_I2C
    depends on DRV_I2C_INTERRUPT_MODE
    default "pfnTWI0_Handler" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_0"
    default "pfnTWI1_Handler" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_1"
    default "pfnTWI2_Handler" if DRV_I2C_PERIPHERAL_ID_IDX${INSTANCE} = "TWI_ID_2"

config DRV_I2C_INT_PRIO_NUM_IDX${INSTANCE}
    string
    depends on USE_DRV_I2C
    depends on DRV_I2C_INTERRUPT_MODE
    default "0" if DRV_I2C_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL0"
    default "1" if DRV_I2C_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL1"
    default "2" if DRV_I2C_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL2"
    default "3" if DRV_I2C_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL3"
    default "4" if DRV_I2C_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL4"
    default "5" if DRV_I2C_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL5"
    default "6" if DRV_I2C_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL6"
    default "7" if DRV_I2C_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL7"

config DRV_I2C_BAUD_RATE_IDX${INSTANCE}
    int "I2C Bus Speed (bps)"
    depends on USE_DRV_I2C
    depends on DRV_I2C_OPERATION_MODE_IDX${INSTANCE} = "DRV_I2C_MODE_MASTER"
    default 400000
    ---help---
    IDH_HTML_PLIB_I2C_BaudRateSet_I2C_MODULE_ID_uint32_t_I2C_BAUD_RATE
    ---endhelp---

config DRV_I2C_QUEUE_SIZE_IDX${INSTANCE}
    int "Queue Size"
    depends on USE_DRV_I2C
    depends on DRV_I2C_OPERATION_MODE_IDX${INSTANCE} = "DRV_I2C_MODE_MASTER"
    default 10

config DRV_I2C_SLAVE_ADDRESS_MASK_IDX${INSTANCE}
    hex "Slave Address Mask (hexadecimal)"
	depends on USE_DRV_I2C
	depends on  DRV_I2C_OPERATION_MODE_IDX${INSTANCE} = "DRV_I2C_MODE_SLAVE"
	default 0x00

config DRV_I2C_SLAVE_ADDRESS_VALUE_IDX${INSTANCE}
    hex "Slave Address Value (hexadecimal)"
	depends on USE_DRV_I2C
	depends on  DRV_I2C_OPERATION_MODE_IDX${INSTANCE} = "DRV_I2C_MODE_SLAVE"
	default 0x00

config DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX${INSTANCE}
    bool "Enable High-Speed(3.4 Mbps) Mode"
    depends on USE_DRV_I2C
    depends on  DRV_I2C_OPERATION_MODE_IDX${INSTANCE} = "DRV_I2C_MODE_SLAVE"
    default n

config DRV_I2C_CLOCK_STRETCH_IDX${INSTANCE}
    bool "Clock Stretch Enable"
	depends on USE_DRV_I2C
	depends on DRV_I2C_OPERATION_MODE_IDX${INSTANCE} = "DRV_I2C_MODE_SLAVE"
    depends on DRV_I2C_SLAVE_HIGH_SPEED_MODE_IDX${INSTANCE} = n
	default y

endif
