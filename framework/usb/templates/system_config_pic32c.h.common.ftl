<#--
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
 -->
/*** USB Driver Configuration ***/


<#if CONFIG_DRV_USB_DEVICE_SUPPORT == true >
/* Enables Device Support */
#define DRV_USBHSV1_DEVICE_SUPPORT    true
<#elseif CONFIG_DRV_USB_DEVICE_SUPPORT == false >
/* Disable Device Support */
#define DRV_USBHSV1_DEVICE_SUPPORT    false
</#if>

<#if CONFIG_DRV_USB_HOST_SUPPORT == true >
/* Enables Host Support */
#define DRV_USBHSV1_HOST_SUPPORT    true
<#elseif CONFIG_DRV_USB_HOST_SUPPORT == false >
/* Disable Host Support */
#define DRV_USBHSV1_HOST_SUPPORT      false
</#if>

<#if CONFIG_DRV_USB_DEVICE_SUPPORT == true || CONFIG_DRV_USB_HOST_SUPPORT == true>
/* Maximum USB driver instances */
#define DRV_USBHSV1_INSTANCES_NUMBER  1
</#if>

<#if CONFIG_DRV_USB_INTERRUPT_MODE == true>
/* Interrupt mode enabled */
#define DRV_USBHSV1_INTERRUPT_MODE    true
<#elseif CONFIG_DRV_USB_INTERRUPT_MODE == false>
/* Interrupt mode Disabled */
#define DRV_USBHSV1_INTERRUPT_MODE      false
</#if>


/* Number of Endpoints used */
<#if CONFIG_DRV_USB_DEVICE_SUPPORT == true>
#define DRV_USBHSV1_ENDPOINTS_NUMBER  ${CONFIG_DRV_USB_ENDPOINTS_NUMBER}
<#elseif CONFIG_DRV_USB_HOST_SUPPORT == true>
#define DRV_USBHSV1_ENDPOINTS_NUMBER    1
</#if>


<#if CONFIG_DRV_USB_DEVICE_SUPPORT == true>
<#include "/framework/usb/templates/system_config_pic32c.h.device.ftl">
</#if>

<#if CONFIG_DRV_USB_HOST_SUPPORT == true>
<#include "/framework/usb/templates/system_config_pic32c.h.host.ftl">
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
