/********************************************************************************
  GFX GLCD Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_glcd.c

  Summary:
    Source code for the GFX GLCD driver static implementation.

  Description:
    This file contains the source code for the static implementation of the
    GFX GLCD driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "framework/gfx/driver/controller/glcd/drv_gfx_glcd_static.h"
#include "system/int/sys_int.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

#define BUFFER_PER_LAYER	2

const char* DRIVER_NAME = "GLCD";
static uint32_t supported_color_formats = GFX_COLOR_MODE_RGBA_8888;
uint32_t state;

//Layer Parameters
//--------------------------------------------------------------------------
typedef struct __display_layer {
    uint32_t  *baseaddr[BUFFER_PER_LAYER];
    int        draw;
    int        frame;
    uint32_t   resx;
    uint32_t   resy;
    uint32_t   buscfg;
    uint32_t   format;
    uint32_t   stride;
    uint32_t   startx;
    uint32_t   starty;
    uint32_t   sizex;
    uint32_t   sizey;
    uint32_t   alpha;
    uint32_t   dblend;
    uint32_t   sblend;
    uint32_t   colorspace;
    uint16_t   color;
} DISPLAY_LAYER;
static DISPLAY_LAYER drvLayer[GFX_GLCD_LAYERS];


/**** Hardware Abstraction Interfaces ****/
enum
{
    INIT = 0,
    RUN
};

static int DRV_GFX_GLCD_Start();

GFX_Context* cntxt;

// function that returns the information for this driver
GFX_Result driverGLCDInfoGet(GFX_DriverInfo* info)
{
    if(info == NULL)
    return GFX_FAILURE;

    // populate info struct
    strcpy(info->name, DRIVER_NAME);
    info->color_formats = supported_color_formats;
    info->layer_count = GFX_GLCD_LAYERS;

    return GFX_SUCCESS;
}

static GFX_Result glcdUpdate()
{
    GFX_Context* context = GFX_ActiveContext();

    if(context == NULL)
        return GFX_FAILURE;

    if(state == INIT)
    {
        if(DRV_GFX_GLCD_Start() != 0)
            return GFX_FAILURE;

        state = RUN;
    }

    return GFX_SUCCESS;
}

static void glcdDestroy(GFX_Context* context)
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

static GLCD_LAYER_COLOR_MODE convertColorModeGfxToGLCD(GFX_ColorMode mode)
{
    switch(mode)
    {
        case GFX_COLOR_MODE_GS_8:
            return GLCD_LAYER_COLOR_MODE_LUT8;
            break;
        case GFX_COLOR_MODE_RGB_332:
            return GLCD_LAYER_COLOR_MODE_RGB332;
            break;
        case GFX_COLOR_MODE_RGB_565:
            return GLCD_LAYER_COLOR_MODE_RGB565;
            break;
        case GFX_COLOR_MODE_RGB_888:
            return GLCD_LAYER_COLOR_MODE_RGB888;
            break;
        case GFX_COLOR_MODE_RGBA_8888:
            return GLCD_LAYER_COLOR_MODE_RGBA8888;
            break;
        case GFX_COLOR_MODE_ARGB_8888:
            return GLCD_LAYER_COLOR_MODE_ARGB8888;
            break;
        case GFX_COLOR_MODE_INDEX_1:
            return GLCD_LAYER_COLOR_MODE_L1;
            break;
        case GFX_COLOR_MODE_INDEX_4:
            return GLCD_LAYER_COLOR_MODE_L4;
            break;
        case GFX_COLOR_MODE_INDEX_8:
            return GLCD_LAYER_COLOR_MODE_L8;
            break;
        default:
            return GLCD_LAYER_COLOR_MODE_RGBA8888;
            break;
                        
    }
}

static GFX_Result colorModeSet(GFX_ColorMode mode)
{
    GFX_Layer* layer;
    GFX_Context* context = GFX_ActiveContext();
    uint32_t i;
    GLCD_LAYER_COLOR_MODE glcdMode;
    
    layer = context->layer.active;
		
   
    // use default implementation to initialize buffer struct
	defColorModeSet(mode);
    
    // ensure all buffers are marked as managed by the driver so application
    // can't delete or modify them
    
    //Translate
    //GFX->GLCD
    glcdMode = convertColorModeGfxToGLCD(mode);
    
    //
    for(i = 0; i < layer->buffer_count; i++)
    {
        drvLayer[i].colorspace = glcdMode;
        PLIB_GLCD_LayerColorModeSet(GLCD_ID_0, 0, drvLayer[i].colorspace );
    }
    
	return GFX_SUCCESS;
}

static GFX_Result layerBufferCountSet(uint32_t count)
{
    GFX_Layer* layer;
    GFX_Context* context = GFX_ActiveContext();
    uint32_t i;
    
    layer = context->layer.active;
		
    if(count > BUFFER_PER_LAYER)
        return GFX_FAILURE;
    
    // use default implementation to initialize buffer struct
	defLayerBufferCountSet(count);
    
    // ensure all buffers are marked as managed by the driver so application
    // can't delete or modify them
    for(i = 0; i < layer->buffer_count; i++)
    {
        GFX_PixelBufferCreate(layer->rect.display.width,
                  layer->rect.display.height,
                  context->colorMode,
                  drvLayer[layer->id].baseaddr[i],
                  &layer->buffers[i].pb);

        layer->buffers[i].state = GFX_BS_MANAGED;
	}
	
	// ensure GLCD buffers are in-sync with the library
	PLIB_GLCD_LayerBaseAddressSet(GLCD_ID_0, layer->id, (uint32_t)drvLayer[layer->id].baseaddr[layer->buffer_read_idx]);

	return GFX_SUCCESS;
}

static GFX_Result layerBufferAddressSet(uint32_t idx, GFX_Buffer address)
{
	return GFX_UNSUPPORTED;
}

static GFX_Result layerBufferAllocate(uint32_t idx)
{
    GFX_Layer* layer;
    GFX_Context* context = GFX_ActiveContext();
	uint32_t  i,j;
    uint32_t  color = 0;
    
    layer = context->layer.active;

    if (layer->id == 0)
    {
        color = GFX_GLCD_BACKGROUND_COLOR;
    }
    
    for(i = 0; i < layer->rect.display.height + 1; i++)
	{
        for(j = 0; j < layer->rect.display.width + 1; j++)
		{
            *(uint32_t*)(drvLayer[layer->id].baseaddr[idx] + i*layer->rect.display.width + j) = color;
		}
	}

	return GFX_SUCCESS;
}

static GFX_Result layerBufferFree(uint32_t idx)
{
	return GFX_UNSUPPORTED;
}

static GFX_Result layerPositionSet(int32_t x, int32_t y)
{
    uint32_t idx;
    
    idx = GFX_ActiveContext()->layer.active->id;

    defLayerPositionSet(x, y);    
	
	PLIB_GLCD_LayerStartXYSet(GLCD_ID_0,
                              idx,
                              GFX_ActiveContext()->layer.active->rect.display.x,
                              GFX_ActiveContext()->layer.active->rect.display.y);

    return GFX_SUCCESS;
}

static GFX_Result layerSizeSet(int32_t width, int32_t height)
{
    uint32_t idx;
    
    idx = GFX_ActiveContext()->layer.active->id;

    defLayerSizeSet(width, height);
    	
	PLIB_GLCD_LayerSizeXYSet(GLCD_ID_0,
                             idx,
                             GFX_ActiveContext()->layer.active->rect.display.width,
                             GFX_ActiveContext()->layer.active->rect.display.height);

    return GFX_SUCCESS;    
}

static GFX_Result layerAlphaAmountSet(uint32_t alpha)
{
    uint32_t idx;
    
    idx = GFX_ActiveContext()->layer.active->id;

    defLayerAlphaAmountSet(alpha);

    PLIB_GLCD_LayerGlobalAlphaSet( GLCD_ID_0, idx, alpha);
	return GFX_SUCCESS;
}
    
static uint32_t layerAlphaAmountGet(void)
{
    uint32_t idx;
    
    idx = GFX_ActiveContext()->layer.active->id;

    return PLIB_GLCD_LayerGlobalAlphaGet( GLCD_ID_0, idx);
}

void layerSwapped(GFX_Layer* layer)
{
<#list 0..CONFIG_DRV_GFX_GLCD_LAYERS_NUMBER?number-1 as i>
    if(layer->id == GFX_ActiveContext()->layer.layers[${i}].id)
    {
        if (layer->buffer_count > BUFFER_PER_LAYER)
			return;

        PLIB_GLCD_LayerBaseAddressSet(GLCD_ID_0, ${i}, (uint32_t)drvLayer[${i}].baseaddr[layer->buffer_read_idx]);
    }
</#list>
}

static GFX_Result layerEnabledSet(GFX_Bool val)
{
	GFX_ActiveContext()->layer.active->enabled = val;
	
    if(val == GFX_TRUE)
        PLIB_GLCD_LayerEnable(GLCD_ID_0, GFX_ActiveContext()->layer.active->id);
	else
        PLIB_GLCD_LayerDisable(GLCD_ID_0, GFX_ActiveContext()->layer.active->id);
		
	return GFX_SUCCESS;
}

static GFX_Result glcdInitialize(GFX_Context* context)
{
    uint32_t      xResolution;
    uint32_t      yResolution;
    uint32_t      rightMargin;
    uint32_t      lowerMargin;
    uint32_t      hsyncLength;
    uint32_t      vsyncLength;
    uint32_t      leftMargin;
    uint32_t      upperMargin;
	uint32_t      i,j;

    cntxt = context;

    // general default initialization
    if(defInitialize(context) == GFX_FAILURE)
            return GFX_FAILURE;

    // override default HAL functions with GLCD specific implementations
    context->hal.update = &glcdUpdate;
    context->hal.destroy = &glcdDestroy;
    context->hal.layerBufferCountSet = &layerBufferCountSet;
    context->hal.layerBufferAddressSet = &layerBufferAddressSet;
    context->hal.layerBufferAllocate = &layerBufferAllocate;
	context->hal.layerBufferFree = &layerBufferFree;
	context->hal.layerSwapped = &layerSwapped;
    context->hal.layerPositionSet = &layerPositionSet;
    context->hal.layerSizeSet = &layerSizeSet;
	context->hal.layerEnabledSet = &layerEnabledSet;

	context->hal.layerAlphaAmountSet = &layerAlphaAmountSet;
	context->hal.layerAlphaAmountGet = &layerAlphaAmountGet;

	context->hal.colorModeSet = &colorModeSet;

    <#if CONFIG_DRV_GFX_DISPLAY_SYS_INIT_SCRIPT?has_content>
    ${CONFIG_DRV_GFX_DISPLAY_SYS_INIT_SCRIPT}
    </#if>

    /* set temporary information */
    xResolution     = context->display_info->rect.width;
    yResolution     = context->display_info->rect.height;
    rightMargin     = context->display_info->attributes.horz.front_porch;
    leftMargin      = context->display_info->attributes.horz.back_porch;
    hsyncLength     = context->display_info->attributes.horz.pulse_width;
    vsyncLength     = context->display_info->attributes.vert.pulse_width;
    upperMargin     = context->display_info->attributes.vert.back_porch;
    lowerMargin     = context->display_info->attributes.vert.front_porch;

    /* glcd initialization */
    PLIB_GLCD_Disable(GLCD_ID_0);
    PLIB_GLCD_BackgroundColorSet(GLCD_ID_0, GFX_GLCD_BACKGROUND_COLOR);
    PLIB_GLCD_VSyncInterruptDisable(GLCD_ID_0);
    PLIB_GLCD_HSyncInterruptDisable(GLCD_ID_0);
    PLIB_GLCD_RGBSequentialModeSet(GLCD_ID_0, 1<<31);
 
    PLIB_GLCD_FrontPorchXYSet(GLCD_ID_0, xResolution + rightMargin, yResolution + lowerMargin);
    PLIB_GLCD_BlankingXYSet(GLCD_ID_0, xResolution + rightMargin + hsyncLength, yResolution + lowerMargin + vsyncLength);
    PLIB_GLCD_BackPorchXYSet(GLCD_ID_0, xResolution + rightMargin + hsyncLength + leftMargin, yResolution + lowerMargin + vsyncLength + upperMargin);

    PLIB_GLCD_ClockDividerSet(GLCD_ID_0, ${CONFIG_DRV_GFX_GLCD_CLK_DIVIDER});
    PLIB_GLCD_ResolutionXYSet(GLCD_ID_0, xResolution, yResolution);

    <#if CONFIG_DRV_GFX_DISPLAY_VSYNC_NEGATIVE_POLARITY == true>
    PLIB_GLCD_SignalPolaritySet(  GLCD_ID_0, GLCD_VSYNC_POLARITY_NEGATIVE );
    </#if>
    <#if CONFIG_DRV_GFX_DISPLAY_HSYNC_NEGATIVE_POLARITY == true>
    PLIB_GLCD_SignalPolaritySet(  GLCD_ID_0, GLCD_HSYNC_POLARITY_NEGATIVE );
    </#if>

    PLIB_GLCD_Enable(GLCD_ID_0);   

    <#list 0..CONFIG_DRV_GFX_GLCD_LAYERS_NUMBER?number-1 as i>
	drvLayer[${i}].baseaddr[0] = (uint32_t*)GFX_GLCD_LAYER${i}_BASEADDR;
	drvLayer[${i}].baseaddr[1] = (uint32_t*)GFX_GLCD_LAYER${i}_DBL_BASEADDR;

    for(i = 0; i < ${CONFIG_DRV_GFX_DISPLAY_HEIGHT} + 1; i++)
	{
        for(j = 0; j < ${CONFIG_DRV_GFX_DISPLAY_WIDTH} + 1; j++)
		{
            *(uint32_t*)(drvLayer[${i}].baseaddr[0] + i*${CONFIG_DRV_GFX_DISPLAY_WIDTH} + j) = 0;
            *(uint32_t*)(drvLayer[${i}].baseaddr[1] + i*${CONFIG_DRV_GFX_DISPLAY_WIDTH} + j) = 0;
		}
	}

    drvLayer[${i}].resx       = context->layer.layers[${i}].rect.display.width;
    drvLayer[${i}].resy       = context->layer.layers[${i}].rect.display.height;
    drvLayer[${i}].startx     = context->layer.layers[${i}].rect.display.x;
    drvLayer[${i}].starty     = context->layer.layers[${i}].rect.display.y;
    drvLayer[${i}].sizex      = drvLayer[${i}].resx;
    drvLayer[${i}].sizey      = drvLayer[${i}].resy;
    drvLayer[${i}].alpha      = context->layer.layers[${i}].alphaAmount;
    drvLayer[${i}].dblend     = GLCD_LAYER_DEST_BLEND_INV_SRCGBL;
    drvLayer[${i}].sblend     = GLCD_LAYER_SRC_BLEND_ALPHA_SRCGBL;
    drvLayer[${i}].colorspace = GLCD_LAYER_COLOR_MODE_RGBA8888;

    PLIB_GLCD_LayerBaseAddressSet(GLCD_ID_0, ${i}, (uint32_t)drvLayer[${i}].baseaddr[0]);
    PLIB_GLCD_LayerStrideSet(GLCD_ID_0, ${i}, drvLayer[${i}].resx * sizeof(uint32_t) );
    PLIB_GLCD_LayerResXYSet( GLCD_ID_0, ${i}, drvLayer[${i}].resx, drvLayer[${i}].resy );
    PLIB_GLCD_LayerStartXYSet( GLCD_ID_0, ${i}, drvLayer[${i}].startx, drvLayer[${i}].starty );
    PLIB_GLCD_LayerSizeXYSet( GLCD_ID_0, ${i}, drvLayer[${i}].sizex, drvLayer[${i}].sizey);
    PLIB_GLCD_LayerGlobalAlphaSet( GLCD_ID_0, ${i}, drvLayer[${i}].alpha);
    PLIB_GLCD_LayerDestBlendFuncSet(GLCD_ID_0, ${i}, drvLayer[${i}].dblend );
    PLIB_GLCD_LayerSrcBlendFuncSet(GLCD_ID_0, ${i}, drvLayer[${i}].sblend );
    PLIB_GLCD_LayerColorModeSet(GLCD_ID_0, ${i}, drvLayer[${i}].colorspace );
    PLIB_GLCD_LayerEnable(GLCD_ID_0, ${i});
    </#list>
	
	SYS_INT_VectorPrioritySet(INT_VECTOR_GLCD, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_GLCD, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_SourceStatusClear(INT_SOURCE_GLCD);
    SYS_INT_SourceEnable(INT_SOURCE_GLCD);

    return GFX_SUCCESS;
}

static void layerSwapPending(GFX_Layer* layer)
{
    PLIB_GLCD_VSyncInterruptEnable(GLCD_ID_0); // enable vsync interrupt
}

// function that initialized the driver context
GFX_Result driverGLCDContextInitialize(GFX_Context* context)
{
        // set driver-specific data initialization function address
        context->hal.initialize = &glcdInitialize;

        // set driver-specific destroy function address
        context->hal.destroy = &glcdDestroy;
		
		// vsync support
		context->hal.layerSwapPending = &layerSwapPending; 
		
        return GFX_SUCCESS;
}

void __ISR(_GLCD_VECTOR, ipl1AUTO) _IntHandlerVSync(void)
{
    uint32_t i;
    GFX_Context* context = GFX_ActiveContext();
    
	// disable vsync interrupt
    PLIB_GLCD_VSyncInterruptDisable(GLCD_ID_0); 
	
	// clear interrupt flag
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_GLCD);
    
	// swap all pending layers
    for(i = 0; i < context->layer.count; i++)
    {
        if(context->layer.layers[i].swap == GFX_TRUE)
            GFX_LayerSwap(&context->layer.layers[i]);
    }
}

/**** End Hardware Abstraction Interfaces ****/


static int DRV_GFX_GLCD_Start()
{
    return 0;
}


/*

void  DRV_GFX_GLCD_BackgroundColorSet(uint32_t bgColor)
{
    PLIB_GLCD_BackgroundColorSet(GLCD_ID_0, bgColor);
}

void DRV_GFX_GLCD_LayerDestBlendSet(GLCD_LAYER_DEST_BLEND_FUNC blend)
{
    PLIB_GLCD_LayerDestBlendFuncSet(GLCD_ID_0, 0, blend );
}

void DRV_GFX_GLCD_LayerSrcBlendSet(GLCD_LAYER_SRC_BLEND_FUNC blend)
{
    PLIB_GLCD_LayerSrcBlendFuncSet(GLCD_ID_0, 0, blend );
}

void DRV_GFX_GLCD_LayerColorSpaceSet(GLCD_LAYER_COLOR_MODE colorSpace)
{
    PLIB_GLCD_LayerColorModeSet(GLCD_ID_0, 0, colorSpace );
}



void DRV_GFX_GLCD_LayerModeSet(uint32_t layerMode)
{
    if ( layerMode & GLCD_LAYER_ENABLE )
    {
        PLIB_GLCD_LayerEnable(GLCD_ID_0, 0 );
    } else {
        PLIB_GLCD_LayerDisable(GLCD_ID_0, 0 );
    }
}

void DRV_GFX_GLCD_LayerFrameBufferSet(uint32_t * frame)
{
    PLIB_GLCD_LayerBaseAddressSet(GLCD_ID_0, 0, (uint32_t)drvGLCDObj.layer[0].baseaddr[0]);
}

void  DRV_GFX_GLCD_CursorSetPosition(uint32_t x, uint32_t y, bool enable)
{
    if ( enable )
    {
        PLIB_GLCD_CursorEnable(GLCD_ID_0);
        PLIB_GLCD_CursorXYSet(GLCD_ID_0, x, y);
    } else 
	{
        PLIB_GLCD_CursorDisable(GLCD_ID_0);
    }
}

void  DRV_GFX_GLCD_CursorImageSet(uint32_t * cursorImage)
{
    int addr;
    int mask;
    int shift;
    int x, y;
    int color;
    uint32_t cursorData[128];

    for(x=0;x<32;x++)
        for(y=0;y<32;y++)
        {
            color = cursorImage[y*32+x] & 0xf;
            shift = (x*4);
            addr  = (y*4)+(x>>3);
            mask  = (0xf << shift);
            cursorData[addr] = (cursorData[addr] & ~mask) | (mask & (color << (shift) ));
        }

    PLIB_GLCD_CursorDataSet(GLCD_ID_0, cursorData );
}

void  DRV_GFX_GLCD_CursorPaletteSet(uint32_t * cursorPalette)
{
    PLIB_GLCD_CursorLUTSet( 0, cursorPalette );
}

void  DRV_GFX_GLCD_GammaPaletteSet(uint32_t * gammaPalette)
{
    PLIB_GLCD_GlobalColorLUTSet( 0, gammaPalette );
}
*/

/*******************************************************************************
 End of File
*/
