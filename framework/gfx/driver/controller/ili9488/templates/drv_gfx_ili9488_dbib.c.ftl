/*******************************************************************************
  MPLAB Harmony Generated Driver Implementation File

  File Name:
    drv_gfx_ili9488_intf.c

  Summary:
    Implements DBIB parallel interface for the ILI9488

  Description:
    Implements DBIB parallel interface for the ILI9488. This driver uses the SMC
    peripheral port to drive the parallel interface.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
 *******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#include "system_config.h"
#include "system_definitions.h"

#include "gfx/hal/inc/gfx_common.h"
#include "gfx/hal/inc/gfx_context.h"

#include "drv_gfx_ili9488_cmd_defs.h"
#include "drv_gfx_ili9488_common.h"

// This is the base address of the SMC0 peripheral on V71.
// Update to appropriate base address for other MCUs.
#define ILI9488_DBIB_BASE_ADDR  0x60000000
#define ILI9488_DBIB_CMD_ADDR  ILI9488_DBIB_BASE_ADDR

// Use Address bit ${CONFIG_DRV_GFX_ILI9488_DBIB_DCX_ADDR_BIT} as DCX
// This lets set set DCX = 1 by writing the data/params to ILI9488_DBIB_DATA_ADDR
#define ILI9488_DBIB_DATA_ADDR  (ILI9488_DBIB_BASE_ADDR | (1 << ${CONFIG_DRV_GFX_ILI9488_DBIB_DCX_ADDR_BIT}))

<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (16-bit Parallel/SMC)">
// Data width for 16-bit SMC
typedef uint16_t DBUS_WIDTH_T;
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (8-bit Parallel/SMC)">
// Data width for 8-bit SMC
typedef uint8_t DBUS_WIDTH_T;
</#if>

/** ILI9488_DBIB_PRIV

  Summary:
    Structure contains status and handles for DBI-B interface
    
 */
typedef struct 
{
    /* Address to write commands */
    volatile DBUS_WIDTH_T * cmdAddr;
    
    /* Address to write data/parameters */
    volatile DBUS_WIDTH_T * dataAddr;
} ILI9488_DBIB_PRIV;

/** ILI9488_DBIB_PRIV

  Function:
    static GFX_Result ILI9488_Intf_Sync(void)

  Summary:
    Add synchronization for core writes to the SMC

 */
static inline void ILI9488_Intf_Sync(void)
{
    __ASM volatile ("dsb");
    __ASM volatile ("dmb");
}

/** 
  Function:
    static GFX_Result ILI9488_Intf_Read(struct ILI9488_DRV *drv, 
                                       uint8_t cmd, 
                                       uint8_t *data,
                                       int bytes)

  Summary:
    Sends read command and returns response from the ILI9488 device.

  Description:
    This function will do an SMC write operation to send the read command 
    to the ILI9488, and then do a SMC read operation to read the response.

  Parameters:
    drv         - ILI9488 driver handle
    cmd         - Read command
    data        - Buffer to store read data
    bytes       - Number of bytes to read
 
Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed


 */
static GFX_Result ILI9488_Intf_Read(struct ILI9488_DRV *drv,
                                    uint8_t cmd,
                                    uint8_t *data,
                                    int bytes) 
{
    ILI9488_DBIB_PRIV *dbiPriv = NULL;
    unsigned int i;

    if ((!drv) || (!data) || (bytes <= 0))
        return GFX_FAILURE;

    ILI9488_NCSAssert();

    dbiPriv = (ILI9488_DBIB_PRIV *) drv->port_priv;

    ILI9488_Intf_Sync();  
    *dbiPriv->cmdAddr = cmd;
    ILI9488_Intf_Sync();  
    
    // Read data 1 byte a time
    for (i = 0; i < bytes; i++)
    {
        *(data + i) = *(dbiPriv->dataAddr);
    }
    
    ILI9488_NCSDeassert();

    return GFX_SUCCESS;;
}

/** 
  Function:
    GFX_Result ILI9488_Intf_WriteCmd(struct ILI9488_DRV *drv,
                                    uint8_t cmd,
                                    uint8_t *parms,
                                    int num_parms)

  Summary:
    Sends write command and parameters to the ILI9488 device.

  Description:

    This function will do a SMC write operation to send the write command 
    and its parameters to the ILI9488.


  Parameters:
    drv         - ILI9488 driver handle
    cmd         - Read command
    parms       - Pointer to array of 8-bit parameters
    bytes       - Number of command parameters
 
Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

 
 */
GFX_Result ILI9488_Intf_WriteCmd(struct ILI9488_DRV *drv,
                                uint8_t cmd,
                                uint8_t *parms,
                                int num_parms) 
{
    ILI9488_DBIB_PRIV *dbiPriv = NULL;
    unsigned int i;

    if (!drv)
        return GFX_FAILURE;

    ILI9488_NCSAssert();

    dbiPriv = (ILI9488_DBIB_PRIV *) drv->port_priv;

    // Write the command
    ILI9488_Intf_Sync();  
    *(dbiPriv->cmdAddr) = cmd ;
    ILI9488_Intf_Sync();  
   
    // Write data one byte at a time
    for (i = 0; i < num_parms; i++)
    {
        *(dbiPriv->dataAddr) = *(parms);
        parms++;
        ILI9488_Intf_Sync();  
    }
    
    ILI9488_NCSDeassert();

    return GFX_SUCCESS;
}

/** 
  Function:
    GFX_Result ILI9488_Intf_WritePixels(struct ILI9488_DRV *drv,
                                              uint32_t start_x,
                                              uint32_t start_y,
                                              uint8_t *data,
                                              unsigned int num_pixels)

  Summary:
    Writes pixel data to ILI9488 GRAM from specified position.

  Description:
    This function will first write the start column, page information, then 
    write the pixel data to the ILI9488 GRAM.

  Parameters:
    drv             - ILI9488 driver handle
    start_x         - Start column position
    start_y         - Start page position
    data            - Array of 8-bit pixel data (8-bit/pixel RGB)
    num_pixels      - Number of pixels
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

 */
GFX_Result ILI9488_Intf_WritePixels(struct ILI9488_DRV *drv,
                                   uint32_t start_x,
                                   uint32_t start_y,
                                   uint8_t *data,
                                   unsigned int num_pixels)
{
    GFX_Result returnValue = GFX_FAILURE;
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (16-bit Parallel/SMC)">
    ILI9488_DBIB_PRIV *dbiPriv = NULL;
    unsigned int i;
</#if>
    uint8_t buf[4];
    
    if (!drv)
        return GFX_FAILURE;

    //Set column
    buf[0] = (start_x >> 8);
    buf[1] = (start_x & 0xff);
    buf[2] = (((drv->gfx->display_info->rect.width - 1) & 0xff00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.width - 1) & 0xff);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_COLUMN_ADDRESS_SET,
                                       buf,
                                       4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    //Set page
    buf[0] = (start_y >> 8);
    buf[1] = (start_y & 0xff);
    buf[2] = (((drv->gfx->display_info->rect.height - 1) & 0xff00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.height - 1) & 0xff);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_PAGE_ADDRESS_SET,
                                       buf,
                                       4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (16-bit Parallel/SMC)">
    dbiPriv = (ILI9488_DBIB_PRIV *) drv->port_priv;
    
    ILI9488_NCSAssert();

    // Write the command
    ILI9488_Intf_Sync();  
    *(dbiPriv->cmdAddr) = ILI9488_CMD_MEMORY_WRITE ;
    ILI9488_Intf_Sync();  

    // Write data 2 bytes at a time
    for (i = 0; i < num_pixels; i++)
    {
        *(dbiPriv->dataAddr) = *((uint16_t *) data); //(data[0] << 8) | data[1];
        ILI9488_Intf_Sync();  
        data += 2;
    }
    
    ILI9488_NCSDeassert();
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (8-bit Parallel/SMC)">
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_MEMORY_WRITE,
                                       data,
                                       num_pixels * 2);
</#if>

    return returnValue;
}

/** 
  Function:
    GFX_Result ILI9488_Intf_ReadPixels(struct ILI9488_DRV *drv,
                                      uint32_t x,
                                      uint32_t y,
                                      uint16_t *value,
                                      unsigned int num_pixels)

  Summary:
    Read pixel data from specified position in ILI9488 GRAM.

  Description:
    This function will first write the start column, page information, then
    read the pixel data from the ILI9488 GRAM.

  Parameters:
    drv             - ILI9488 driver handle
    x               - Column position
    y               - Page position
    value           - Value to store the read pixel color (8-bit/pixel RGB)
    num_pixels      - Number of pixels to read
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

 */
GFX_Result ILI9488_Intf_ReadPixels(struct ILI9488_DRV *drv,
                                  uint32_t x,
                                  uint32_t y,
                                  uint8_t *value,
                                  unsigned int num_pixels)
{
    GFX_Result returnValue = GFX_FAILURE;
    ILI9488_DBIB_PRIV *dbiPriv = NULL;
    uint8_t buf[4];
    unsigned int i;
    volatile DBUS_WIDTH_T pixel[3] = {0};
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (16-bit Parallel/SMC)">
    uint16_t * pixelPtr;
</#if>

    if (!drv)
        return GFX_FAILURE;

    dbiPriv = (ILI9488_DBIB_PRIV *) drv->port_priv;

    //Set column
    buf[0] = ((x & 0xff00) >> 8);
    buf[1] = (x & 0xff);
    buf[2] = (((drv->gfx->display_info->rect.width - 1) & 0xff00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.width - 1) & 0xff);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_COLUMN_ADDRESS_SET,
                                       buf,
                                       4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    //Set page
    buf[0] = ((y & 0xff00) >> 8);
    buf[1] = (y & 0xff);
    buf[2] = (((drv->gfx->display_info->rect.height - 1) & 0xff00) >> 8);
    buf[3] = ((drv->gfx->display_info->rect.height - 1) & 0xff);
    returnValue = ILI9488_Intf_WriteCmd(drv,
                                       ILI9488_CMD_PAGE_ADDRESS_SET,
                                       buf,
                                       4);
    if (GFX_SUCCESS != returnValue)
        return GFX_FAILURE;

    ILI9488_NCSAssert();

    ILI9488_Intf_Sync();  
    *dbiPriv->cmdAddr = ILI9488_CMD_MEMORY_READ;
    ILI9488_Intf_Sync();  

    // Read the dummy byte
    pixel[0] = *(dbiPriv->dataAddr);

    // Read the pixel data
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (16-bit Parallel/SMC)">
    for (i = 0, pixelPtr = (uint16_t *) value; i < num_pixels; i++, pixelPtr++)
    {
        //In 16-bit mode, each 16-bit read contains 2 bytes of color data, with 
        //each byte containing one color in RGB565 mode. So 3 16-bit reads will 
        //read 2 pixels
        pixel[0] = *(dbiPriv->dataAddr); //pixel[0] bits [15:11] -> R(n), bits [7:2] -> G(n)
        pixel[1] = *(dbiPriv->dataAddr); //pixel[1] bits [15:11] -> B(n), bits [7:3] -> R(n+1)
        pixel[2] = *(dbiPriv->dataAddr); //pixel[2] bits [15:10] -> G(n+1), bits [7:3] -> B(n+1)
        
        *pixelPtr = (pixel[0] & 0xf800)             //R
                    | ((pixel[0] & 0x00fc) << 3)    //G 
                    | ((pixel[1] & 0xf800) >> 11);  //B
        
        // If reading more than 1 pixel, set next pixel
        if (i < num_pixels)
        {
            pixelPtr++;
            *pixelPtr = ((pixel[1] & 0x00f8) << 8)  //R
                    | ((pixel[2] & 0xfc00) >> 5)    //G
                    | ((pixel[2] & 0x00f8) >> 3);   //B

            i++;
        }
    }
</#if>
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "ILI9488 (8-bit Parallel/SMC)">
    for (i = 0; i < num_pixels; i++)
    {
        // In 8-bit mode, each 8-bit read gets one color in RGB565 color mode.
        // Read 3 bytes to get R, G, and B.
        pixel[0] = *(dbiPriv->dataAddr); //R
        pixel[1] = *(dbiPriv->dataAddr); //G
        pixel[2] = *(dbiPriv->dataAddr); //B
        
        value[i * drv->bytesPerPixelBuffer] = (pixel[0] | (pixel[1] >> 5));
        value[(i * drv->bytesPerPixelBuffer) + 1] = 
                                    ((pixel[1] & 0x1c) << 3) | (pixel[2] >> 3);
    }
</#if>

    ILI9488_NCSDeassert();

    return GFX_SUCCESS;
}

/** 
  Function:
    GFX_Result ILI9488_Intf_ReadCmd(struct ILI9488_DRV *drv, 
                                          uint8_t cmd, 
                                          uint8_t *data,
                                          int bytes);

  Summary:
    Sends read command and reads response from ILI9488.

  Description:
    This function will fist write the the read command and then read back the 
    response from the ILI9488 GRAM.

  Parameters:
    drv             - ILI9488 driver handle
    cmd             - Read command
    data            - Buffer to store the read data to
    bytes           - Number of bytes to read
 
  Returns:
    * GFX_SUCCESS       Operation successful
    * GFX_FAILURE       Operation failed
 
  Remarks:
    This function only supports 8-, 24- or 32-bit reads.

 */
GFX_Result ILI9488_Intf_ReadCmd(struct ILI9488_DRV *drv,
                               uint8_t cmd,
                               uint8_t *data,
                               int bytes)
{
    GFX_Result returnValue = GFX_FAILURE;
    uint8_t buff[5];

    //API supports only 8-, 24-, or 32-bit reads
    if ((!drv) || (!data) ||
        ((bytes != 1) && (bytes != 3) && (bytes != 4)))
        return GFX_FAILURE;

    returnValue = ILI9488_Intf_Read(drv, cmd, buff, bytes + 1);

    return returnValue;
}

// *****************************************************************************

/** 
  Function:
    GFX_Result ILI9488_Intf_Open(ILI9488_DRV *drv, unsigned int index)

  Summary:
    Opens the specified port to the ILI9488 device.

  Description:
    In SPI mode, this function will open the SPI port, allocate the port-specific
    data structures and set the port operation handler functions. When done 
    using the port, ILI9488_Intf_Close must be called to free up the data 
    structures and close the port.

  Parameters:
    drv         - ILI9488 driver handle
    index       - Port index
 
  Returns:
    * GFX_SUCCESS       - Operation successful
    * GFX_FAILURE       - Operation failed

 */
GFX_Result ILI9488_Intf_Open(ILI9488_DRV *drv, unsigned int index)
{
    ILI9488_DBIB_PRIV *dbiPriv = NULL;

    if (!drv)
        return GFX_FAILURE;

    dbiPriv = (ILI9488_DBIB_PRIV *) 
                drv->gfx->memory.calloc(1, sizeof (ILI9488_DBIB_PRIV));

    dbiPriv->dataAddr = (DBUS_WIDTH_T *) ILI9488_DBIB_DATA_ADDR;
    dbiPriv->cmdAddr = (DBUS_WIDTH_T *) ILI9488_DBIB_CMD_ADDR;
    
    drv->port_priv = (void *) dbiPriv;

    ILI9488_NCSDeassert();

    return GFX_SUCCESS;
}

/** 
  Function:
    void ILI9488_Intf_Close(ILI9488_DRV *drv)

  Summary:
    Closes the HW interface to the ILI9488 device.

  Description:
    This function will close the specified interface, free the port-specific
    data structures and unset the port operation handler functions.

  Parameters:
    drv         - ILI9488 driver handle
 
  Returns:
    None.

 */
void ILI9488_Intf_Close(ILI9488_DRV *drv) 
{
    ILI9488_DBIB_PRIV *dbiPriv = NULL;

    if (!drv)
        return;

    dbiPriv = (ILI9488_DBIB_PRIV *) drv->port_priv;

    drv->gfx->memory.free(dbiPriv);

    drv->port_priv = NULL;

    ILI9488_NCSDeassert();
}
/* *****************************************************************************
 End of File
 */
