<#macro MACRO_HSYNC_OFF><#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>BSP_LCD_HSYNCOn();<#else>BSP_LCD_HSYNCOff();</#if></#macro>
<#macro MACRO_HSYNC_ON><#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>BSP_LCD_HSYNCOff();<#else>BSP_LCD_HSYNCOn();</#if></#macro>
<#macro MACRO_VSYNC_OFF><#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>BSP_LCD_VSYNCOn();<#else>BSP_LCD_VSYNCOff();</#if></#macro>
<#macro MACRO_VSYNC_ON><#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>BSP_LCD_VSYNCOff();<#else>BSP_LCD_VSYNCOn();</#if></#macro>
<#macro MACRO_DE_OFF><#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>BSP_LCD_DEOn();<#else>BSP_LCD_DEOff();</#if></#macro>
<#macro MACRO_DE_ON><#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>BSP_LCD_DEOff();<#else>BSP_LCD_DEOn();</#if></#macro>
<#macro MACRO_RESET_OFF><#if CONFIG_DRV_GFX_DISPLAY_RESET_POSITIVE_POLARITY == true>BSP_LCD_RESETOff();<#else>BSP_LCD_RESETOn();</#if></#macro>
<#macro MACRO_RESET_ON><#if CONFIG_DRV_GFX_DISPLAY_RESET_POSITIVE_POLARITY == true>BSP_LCD_RESETOn();<#else>BSP_LCD_RESETOff();</#if></#macro>
<#macro MACRO_CS_OFF><#if CONFIG_DRV_GFX_DISPLAY_CHIP_SELECT_POSITIVE_POLARITY == true>BSP_LCD_CSOff();<#else>BSP_LCD_CSOn();</#if></#macro>
<#macro MACRO_CS_ON><#if CONFIG_DRV_GFX_DISPLAY_CHIP_SELECT_POSITIVE_POLARITY == true>BSP_LCD_CSOn();<#else>BSP_LCD_CSOff();</#if></#macro>
/*******************************************************************************
  MPLAB Harmony LCC Generated Driver Implementation File

  File Name:
    drv_gfx_lcc_generic.c

  Summary:
    Build-time generated implementation for the LCC Driver for PIC32C MCUs.

  Description:
    Build-time generated implementation for the LCC Driver for PIC32C MCUs.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2016 released Microchip Technology Inc.  All rights reserved.

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

#include "framework/gfx/driver/controller/lcc/drv_gfx_lcc_generic.h"

#define MAX_LAYER_COUNT 1
#define BUFFER_COUNT    1
#define DISPLAY_WIDTH   ${CONFIG_DRV_GFX_DISPLAY_WIDTH}
#define DISPLAY_HEIGHT  ${CONFIG_DRV_GFX_DISPLAY_HEIGHT}

#define SMC0_BASE_ADDR  0x60000000

static SYS_DMA_CHANNEL_HANDLE dmaHandle = SYS_DMA_CHANNEL_HANDLE_INVALID;

const char* DRIVER_NAME = "LCC PIC32CZ";
static uint32_t supported_color_formats = GFX_COLOR_MASK_RGB_565;

uint32_t state;

uint16_t __attribute__((aligned(16))) frameBuffer[BUFFER_COUNT][DISPLAY_WIDTH * DISPLAY_HEIGHT];

#define DRV_GFX_LCC_DMA_CHANNEL_INDEX DMA_CHANNEL_${CONFIG_DRV_GFX_LCC_DMA_INSTANCE_CHANNEL}

/**** Hardware Abstraction Interfaces ****/
enum
{
    INIT = 0,
    RUN
};

static int DRV_GFX_LCC_Start();
static void DRV_GFX_LCC_DisplayRefresh(void);
void dmaIntHandler (SYS_DMA_TRANSFER_EVENT event,
                    SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);

GFX_Context* cntxt;

uint16_t HBackPorch;
uint32_t VER_BLANK;

uint32_t DISP_HOR_FRONT_PORCH;
uint32_t DISP_HOR_RESOLUTION;
uint32_t DISP_HOR_BACK_PORCH;
uint32_t DISP_HOR_PULSE_WIDTH;

uint32_t DISP_VER_FRONT_PORCH;
uint32_t DISP_VER_RESOLUTION;
uint32_t DISP_VER_BACK_PORCH;
uint32_t DISP_VER_PULSE_WIDTH;

int16_t line = 0;
uint32_t offset = 0;
uint16_t pixels = 0;
uint32_t hSyncs = 0;
    
uint32_t vsyncPeriod = 0;
uint32_t vsyncPulseDown = 0;
uint32_t vsyncPulseUp = 0;
uint32_t vsyncEnd = 0;

volatile bool allowFrameUpdate = false;
volatile bool allowLineUpdate = false;

// function that returns the information for this driver
GFX_Result driverLCCInfoGet(GFX_DriverInfo* info)
{
	if(info == NULL)
        return GFX_FAILURE;

	// populate info struct
    strcpy(info->name, DRIVER_NAME);
    info->color_formats = supported_color_formats;
    info->layer_count = MAX_LAYER_COUNT;
    
    return GFX_SUCCESS;
}

static GFX_Result lccUpdate()
{
    GFX_Context* context = GFX_ActiveContext();
   
    if(context == NULL)
        return GFX_FAILURE;
    
    if(state == INIT)
    {
        if(DRV_GFX_LCC_Start() != 0)
            return GFX_FAILURE;
        
        state = RUN;
    }
    
    return GFX_SUCCESS;
}

static void lccDestroy(GFX_Context* context)
{	
	// driver specific shutdown tasks
	if(context->driver_data != GFX_NULL)
	{
		context->memory.free(context->driver_data);
		context->driver_data = GFX_NULL;
	}
	
	// general default shutdown
	defDestroy(context);
}

static GFX_Result layerBufferCountSet(uint32_t count)
{
    count = count;
        
    return GFX_FAILURE;
}

static GFX_Result layerBufferAddressSet(uint32_t idx, GFX_Buffer address)
{
    idx = 0;
    address = address;
    
    return GFX_FAILURE;
}

static GFX_Result layerBufferAllocate(uint32_t idx)
{
    idx = 0;
    
    return GFX_FAILURE;
}

static GFX_Result lccInitialize(GFX_Context* context)
{
	uint32_t i, j;
	
	cntxt = context;
	
	// general default initialization
	if(defInitialize(context) == GFX_FAILURE)
		return GFX_FAILURE;	
		
	// override default HAL functions with LCC specific implementations
    context->hal.update = &lccUpdate;
    context->hal.destroy = &lccDestroy;
	context->hal.layerBufferCountSet = &layerBufferCountSet;
    context->hal.layerBufferAddressSet = &layerBufferAddressSet;
    context->hal.layerBufferAllocate = &layerBufferAllocate;
	
	// driver specific initialization tasks	
	// initialize all layer color modes
    for(i = 0; i < MAX_LAYER_COUNT; i++)
    {        
        for(j = 0; j < BUFFER_COUNT; j++)
        {
            GFX_PixelBufferCreate(DISPLAY_WIDTH,
                                  DISPLAY_HEIGHT,
                                  GFX_COLOR_MODE_RGB_565,
                                  frameBuffer[j],
                                  &context->layer.layers[i].buffers[j].pb);
            
            context->layer.layers[i].buffers[j].state = GFX_BS_MANAGED;
        }
	}
	
	VER_BLANK = context->display_info->attributes.vert.pulse_width +
	            context->display_info->attributes.vert.back_porch +
				context->display_info->attributes.vert.front_porch - 1;
	
	HBackPorch = context->display_info->attributes.horz.pulse_width +
	             context->display_info->attributes.horz.back_porch;
	
	DISP_HOR_FRONT_PORCH = context->display_info->attributes.horz.front_porch;
	DISP_HOR_RESOLUTION = DISPLAY_WIDTH;
	DISP_HOR_BACK_PORCH = context->display_info->attributes.horz.back_porch;
	DISP_HOR_PULSE_WIDTH = context->display_info->attributes.horz.pulse_width;
	
	DISP_VER_FRONT_PORCH = context->display_info->attributes.vert.front_porch;
	DISP_VER_RESOLUTION = DISPLAY_HEIGHT;
	DISP_VER_BACK_PORCH = context->display_info->attributes.vert.back_porch;
	DISP_VER_PULSE_WIDTH = context->display_info->attributes.vert.pulse_width;

	vsyncPeriod = DISP_VER_FRONT_PORCH + DISP_VER_RESOLUTION + DISP_VER_BACK_PORCH;

<#if CONFIG_DRV_GFX_DISPLAY_USE_RESET == true>
<#if CONFIG_DRV_GFX_DISPLAY_RESET_POSITIVE_POLARITY == true>
    BSP_LCD_RESETOn();
<#else>
    BSP_LCD_RESETOff();

</#if>
</#if>
<#if CONFIG_DRV_GFX_DISPLAY_USE_CHIP_SELECT == true>
<#if CONFIG_DRV_GFX_DISPLAY_CHIP_SELECT_POSITIVE_POLARITY == true>
    BSP_LCD_CSOff();
<#else>
    BSP_LCD_CSOn();
</#if>
</#if>

    /*Turn Backlight on*/
<#if CONFIG_DRV_GFX_DISPLAY_BACKLIGHT_ENABLE_LEVEL == "1">
    BSP_LCD_BACKLIGHTOn();
<#else>
    BSP_LCD_BACKLIGHTOff();
</#if>
	
	return GFX_SUCCESS;
}

// function that initialized the driver context
GFX_Result driverLCCContextInitialize(GFX_Context* context)
{
	// set driver-specific data initialization function address
	context->hal.initialize = &lccInitialize; 
	
	// set driver-specific destroy function address
    context->hal.destroy = &lccDestroy;
	
	return GFX_SUCCESS;
}

/**** End Hardware Abstraction Interfaces ****/

static inline void lccDMAStartTransfer(SYS_DMA_CHANNEL_HANDLE handle,
                  const void *srcAddr, size_t srcSize,
                  const void *destAddr, size_t destSize)
{
    // The DMA APIs currently do not provide access to configure the DMA registers
    // to the desired settings, so we need to do a couple of direct register writes
    // outside the DMA API.
    CAST(xdmac_registers_t, XDMAC_ID_0)->XDMAC_CHID[DRV_GFX_LCC_DMA_CHANNEL_INDEX].XDMAC_CC.DAM = XDMAC_CC_DAM_FIXED_AM_Val;
    CAST(xdmac_registers_t, XDMAC_ID_0)->XDMAC_CHID[DRV_GFX_LCC_DMA_CHANNEL_INDEX].XDMAC_CBC.w = (srcSize >> 1) - 1;
    
    // Start transfer using DMA API
    SYS_DMA_ChannelTransferAdd(handle,
							   srcAddr,
                               1,
                               (uint32_t*) destAddr,
                               SYS_DMA_DATA_WIDTH_HALF_WORD,
                               1);
}

static int DRV_GFX_LCC_Start()
{
	/*Suspend DMA Module*/
    SYS_DMA_Suspend();
	
    /* Allocate DMA channel */
    dmaHandle = SYS_DMA_ChannelAllocate(DRV_GFX_LCC_DMA_CHANNEL_INDEX);
    
	if (SYS_DMA_CHANNEL_HANDLE_INVALID == dmaHandle)
        return -1;
		
	SYS_INT_VectorPrioritySet(XDMAC_IRQn, ${CONFIG_DRV_GFX_LCC_INTERRUPT_PRIORITY});
    SYS_INT_VectorSubprioritySet(XDMAC_IRQn, INT_SUBPRIORITY_LEVEL0);

    SYS_INT_SourceEnable(XDMAC_IRQn);
	
    // set the transfer event control: what event is to start the DMA transfer
    SYS_DMA_ChannelSetup(dmaHandle,
                         SYS_DMA_CHANNEL_OP_MODE_BASIC,
                         DMA_TRIGGER_SOURCE_NONE);

    SYS_DMA_ChannelTransferEventHandlerSet(dmaHandle,
                                           dmaIntHandler,
                                           (const uintptr_t) NULL);

    SYS_DMA_ChannelDataWidthSet (dmaHandle,
                                 SYS_DMA_DATA_WIDTH_HALF_WORD);
    
    lccDMAStartTransfer(dmaHandle, 
                        frameBuffer, 
                        2,
                        (const void *) SMC0_BASE_ADDR, 
                        SYS_DMA_DATA_WIDTH_HALF_WORD);
    
    /*Unsuspend DMA Module*/
    SYS_DMA_Resume();
	
	return 0;
}

static void DRV_GFX_LCC_DisplayRefresh(void)
{
	GFX_Point drawPoint;
	GFX_PixelBuffer* buffer;
    GFX_PixelBuffer* buffer_to_tx = (GFX_PixelBuffer *) frameBuffer;

    typedef enum
	{
        HSYNC_FRONT_PORCH,
        HSYNC_PULSE,
        HSYNC_BACK_PORCH,
        HSYNC_DATA_ENABLE,
        HSYNC_DATA_ENABLE_OVERFLOW        
    } HSYNC_STATES;

    typedef enum
    {
        VSYNC_FRONT_PORCH,
        VSYNC_PULSE,
        VSYNC_BACK_PORCH,
        VSYNC_BLANK        
    } VSYNC_STATES;

    static HSYNC_STATES hsyncState = HSYNC_FRONT_PORCH;
    static VSYNC_STATES vsyncState = VSYNC_BLANK;

    switch(vsyncState)
    {
        case VSYNC_FRONT_PORCH:
		{
            if (hSyncs > vsyncPulseDown)
            {
<#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
                BSP_LCD_VSYNCOn();
<#else>
                BSP_LCD_VSYNCOff();
</#if>

                vsyncPulseUp = hSyncs + DISP_VER_PULSE_WIDTH;
                vsyncState = VSYNC_PULSE;

				if(cntxt->layer.active->vsync == GFX_TRUE
					&& cntxt->layer.active->swap == GFX_TRUE)
					GFX_LayerSwap(cntxt->layer.active);

                allowFrameUpdate = false;
                line = 0;
            }
			
            break;
		}
        case VSYNC_PULSE:
		{
            if (hSyncs >= vsyncPulseUp)
            {
<#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
                BSP_LCD_VSYNCOff();
<#else>
                BSP_LCD_VSYNCOn();
</#if>
                vsyncEnd = hSyncs + DISP_VER_BACK_PORCH;
                vsyncState = VSYNC_BACK_PORCH;
            }
			
            break;
		}
        case VSYNC_BACK_PORCH:
		{
            if (hSyncs >= vsyncEnd)
                vsyncState = VSYNC_BLANK;
            
            break;
		}
        case VSYNC_BLANK:
		{
            break;
		}
    }

    switch (hsyncState)
    {
        case HSYNC_FRONT_PORCH:
		{
<#if CONFIG_DRV_GFX_DISPLAY_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>
            BSP_LCD_DEOff();			
<#else>
            BSP_LCD_DEOn();			
</#if>
</#if>		

            hsyncState = HSYNC_PULSE;
            allowLineUpdate = true;
			
			if (DISP_HOR_FRONT_PORCH > 0)
			{
	            pixels = DISP_HOR_FRONT_PORCH;
	            break;
			}
		}
        case HSYNC_PULSE:
		{
<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOn();
<#else>
            BSP_LCD_HSYNCOff();
</#if>

            if (hSyncs >= vsyncPeriod)
            {
				vsyncPeriod = hSyncs + DISP_VER_PULSE_WIDTH + DISP_VER_FRONT_PORCH + DISP_VER_RESOLUTION + DISP_VER_BACK_PORCH;
                vsyncPulseDown = hSyncs + DISP_VER_FRONT_PORCH;
                vsyncState = VSYNC_FRONT_PORCH;
                allowFrameUpdate = true;
            }
			
            hSyncs++; 
            
			pixels = DISP_HOR_PULSE_WIDTH;
            hsyncState = HSYNC_BACK_PORCH;  
            
			break;
		}
        case HSYNC_BACK_PORCH:
		{
<#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
            BSP_LCD_HSYNCOff();
<#else>
            BSP_LCD_HSYNCOn();
</#if>

            hsyncState = HSYNC_DATA_ENABLE; 
            allowLineUpdate = false;
			
			if (DISP_HOR_BACK_PORCH > 0)
			{
				pixels = DISP_HOR_BACK_PORCH;
				break;
			}
		}
        case HSYNC_DATA_ENABLE:
		{
            if (vsyncState == VSYNC_BLANK)
            {
<#if CONFIG_DRV_GFX_DISPLAY_USE_DATA_ENABLE == true>
<#if CONFIG_DRV_GFX_DISPLAY_DATA_ENABLE_POSITIVE_POLARITY == true>
                BSP_LCD_DEOn();
<#else>
                BSP_LCD_DEOff();
</#if>
</#if>
				drawPoint.x = 0;
				drawPoint.y = line++;
				
				buffer = &cntxt->layer.active->buffers[cntxt->layer.active->buffer_read_idx].pb;

                buffer_to_tx = GFX_PixelBufferOffsetGet_Unsafe(buffer, &drawPoint);
                   
			}
            
			pixels = DISP_HOR_RESOLUTION;
            hsyncState = HSYNC_FRONT_PORCH;
			
			break;
		}
        case HSYNC_DATA_ENABLE_OVERFLOW:
		{
            hsyncState = HSYNC_FRONT_PORCH;
			
            break;
		}
    }

    lccDMAStartTransfer(dmaHandle,
                        buffer_to_tx,
                        (pixels << 1), //2 bytes per pixel
                        (uint32_t*) SMC0_BASE_ADDR,
                        SYS_DMA_DATA_WIDTH_HALF_WORD);
}

void dmaIntHandler (SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    DRV_GFX_LCC_DisplayRefresh();
}
