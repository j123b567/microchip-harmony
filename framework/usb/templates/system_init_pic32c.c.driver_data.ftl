// <editor-fold defaultstate="collapsed" desc="DRV_USB Initialization Data">
/******************************************************
 * USB Driver Initialization
 ******************************************************/
static DRV_USB_VBUS_LEVEL DRV_USBHSV1_VBUS_Comparator(void)
{
    DRV_USB_VBUS_LEVEL retVal = DRV_USB_VBUS_LEVEL_INVALID;

    if(true == BSP_USB_VBUS_INStateGet())
    {
        retVal = DRV_USB_VBUS_LEVEL_VALID;
    }

    return (retVal);

}

const DRV_USBHSV1_INIT drvUSBInit =
{
<#if CONFIG_DRV_USB_INTERRUPT_MODE == true>
    /* Interrupt Source for USB module */
    .interruptSource = ${CONFIG_DRV_USB_INTERRUPT_SOURCE_IDX0},
</#if>

<#if CONFIG_USB_DEVICE_POWER_STATE_IDX0?has_content>
    /* System module initialization */
    .moduleInit = {${CONFIG_USB_DEVICE_POWER_STATE_IDX0}},
</#if>

    /* USB Controller to operate as USB Device */
<#if CONFIG_DRV_USB_DEVICE_SUPPORT == true>   
    .operationMode = DRV_USBHSV1_OPMODE_DEVICE,
<#elseif CONFIG_DRV_USB_HOST_SUPPORT == true>
    .operationMode = DRV_USBHSV1_OPMODE_HOST,
</#if>

    /* To operate in USB Normal Mode */
<#if CONFIG_USB_DEVICE_FUNCTION_1_SPEED_HSV1_NORMAL_IDX0>
    .operationSpeed = DRV_USBHSV1_DEVICE_SPEEDCONF_NORMAL,
<#else >
    .operationSpeed = DRV_USBHSV1_DEVICE_SPEEDCONF_LOW_POWER,
</#if>
    
    /* Identifies peripheral (PLIB-level) ID */
    .usbID = _USBHS_REGS,
    
    /* Function to check for VBus */
    .vbusComparator = DRV_USBHSV1_VBUS_Comparator
};

// </editor-fold>
