<#if CONFIG_DRV_USB_DEVICE_SUPPORT == true >
    /* Initialize USB Driver */ 
    sysObj.drvUSBObject = DRV_USBHSV1_Initialize(DRV_USBHSV1_INDEX_0, (SYS_MODULE_INIT *) &drvUSBInit);
</#if>
