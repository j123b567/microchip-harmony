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
// *****************************************************************************
// Section: Bootloader Configuration
// *****************************************************************************

#define BTL_TRIGGER_SWITCH    BSP_SWITCH_0
#define BTL_LED               BSP_LED_0

/* APP_FLASH_BASE_ADDRESS and APP_FLASH_END_ADDRESS reserves program Flash for the application*/
/* Rule:
    1)The memory regions of the application linker script must fall with in APP_FLASH_BASE_ADDRESS
    and APP_FLASH_END_ADDRESS

    2)The base address and end address must align on NVM_ERASE_PAGE_SIZE boundaries */

#define APP_FLASH_BASE_ADDRESS    (${CONFIG_FLASH_ADDR_START} + ${CONFIG_BOOTLOADER_SIZE})

#define APP_FLASH_END_ADDRESS     (${CONFIG_FLASH_ADDR_START} + ${CONFIG_FLASH_ADDR_SIZE} - 1)

/* Address of  the Flash from where the application starts executing */
#define APP_RESET_ADDRESS         (APP_FLASH_BASE_ADDRESS)

<#if CONFIG_BOOTLOADER_TYPE == "USB_HOST" || CONFIG_BOOTLOADER_TYPE == "SD_CARD">
#define BOOTLOADER_IMAGE_FILE_NAME      "${CONFIG_BOOTLOADER_IMAGE_FILENAME}"
</#if>

// *****************************************************************************
// Section: Bootloader NVM Driver Configuration
// *****************************************************************************
#define NVM_ERASE_PAGE_SIZE       0x4000

<#--
/*******************************************************************************
 End of File
*/
-->

