<#--
/*******************************************************************************
  Timer Driver Initialization File

  File Name:
    drv_tmr_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
 *******************************************************************************/

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
<#if CONFIG_USE_SYS_INT == true &&  CONFIG_DRV_USB_INTERRUPT_MODE == true>

<#if CONFIG_DRV_USB_HOST_SUPPORT == true >

<#if CONFIG_BSP_BT_AUDIO_DK_BOARD == true>

    /* Select USB Host Mode */
   BSP_USBSwitchSelect(BSP_USB_HOST);
   
</#if>
</#if>

<#if CONFIG_DRV_USB_HOST_SUPPORT == true>
    sysObj.usbHostObject0 = USB_HOST_Initialize ((SYS_MODULE_INIT *)& usbHostInitData );
    sysObj.drvUSBObject = DRV_USBHSV1_Initialize (DRV_USBHSV1_INDEX_0, (SYS_MODULE_INIT *) &drvUSBHSInit);
    SYS_INT_VectorPrioritySet(USBHS_IRQn, INT_PRIORITY_LEVEL5);
</#if>							

</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
