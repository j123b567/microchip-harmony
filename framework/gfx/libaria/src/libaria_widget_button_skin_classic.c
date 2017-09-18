#include "gfx/libaria/inc/libaria_widget_button.h"

#if LA_BUTTON_WIDGET_ENABLED

#include "gfx/libaria/inc/libaria_context.h"
#include "gfx/libaria/inc/libaria_draw.h"
#include "gfx/libaria/inc/libaria_layer.h"
#include "gfx/libaria/inc/libaria_string.h"
#include "gfx/libaria/inc/libaria_utils.h"
#include "gfx/libaria/inc/libaria_widget.h"

#include "gfx/libaria/inc/libaria_widget_skin_classic_common.h"

enum
{
    NOT_STARTED = LA_WIDGET_DRAW_STATE_READY,
    DONE = LA_WIDGET_DRAW_STATE_DONE,
    DRAW_BACKGROUND,
    DRAW_IMAGE,
    WAIT_IMAGE,
    DRAW_STRING,
    WAIT_STRING,
    DRAW_BORDER,
};

void _laButtonWidget_GetImageRect(laButtonWidget* btn,
                                  GFX_Rect* imgRect,
								  GFX_Rect* imgSrcRect)
{
    GFX_Rect textRect;
    GFX_Rect bounds = laUtils_WidgetLocalRect((laWidget*)btn);
    
    imgRect->x = 0;
    imgRect->y = 0;
    
    laString_GetRect(&btn->text, &textRect);
    
    if(btn->state != LA_BUTTON_STATE_UP)
    {
        if(btn->pressedImage != NULL)
        {
            imgRect->width = btn->pressedImage->width;
            imgRect->height = btn->pressedImage->height;
        }
    }
    else
    {
        if(btn->releasedImage != NULL)
        {
            imgRect->width = btn->releasedImage->width;
            imgRect->height = btn->releasedImage->height;
        }
    }
    
    *imgSrcRect = *imgRect;
    
    laUtils_ArrangeRectangle(imgRect,
                             textRect,
                             bounds,
                             btn->halign,
                             btn->valign,
                             btn->imagePosition,
                             btn->widget.margin.left,
                             btn->widget.margin.top,
                             btn->widget.margin.right,
                             btn->widget.margin.bottom,
                             btn->imageMargin);
                             
    if(btn->state != LA_BUTTON_STATE_UP)
    {
        imgRect->x += btn->pressedOffset;
        imgRect->y += btn->pressedOffset;
    }                             
                             
    *imgRect = GFX_RectClipAdj(imgRect, &bounds, imgSrcRect); 
    
    laUtils_RectToLayerSpace((laWidget*)btn, imgRect);                         
}

void _laButtonWidget_GetTextRect(laButtonWidget* btn,
                                 GFX_Rect* textRect,
								 GFX_Rect* drawRect)
{
    GFX_Rect bounds;
    
    GFX_Rect imgRect = {0};
    
    bounds = laUtils_WidgetLocalRect((laWidget*)btn);
    
    laString_GetRect(&btn->text, textRect);
    
    if(btn->state != LA_BUTTON_STATE_UP)
    {
        if(btn->pressedImage != NULL)
        {
            imgRect.width = btn->pressedImage->width;
            imgRect.height = btn->pressedImage->height;
        }
    }
    else
    {
        if(btn->releasedImage != NULL)
        {
            imgRect.width = btn->releasedImage->width;
            imgRect.height = btn->releasedImage->height;
        }
    }
    
    // arrange relative to image rect
    laUtils_ArrangeRectangleRelative(textRect,
                                     imgRect,
                                     bounds,
                                     btn->halign,
                                     btn->valign,
                                     btn->imagePosition,
                                     btn->widget.margin.left,
                                     btn->widget.margin.top,
                                     btn->widget.margin.right,
                                     btn->widget.margin.bottom,
                                     btn->imageMargin);

    if(btn->state != LA_BUTTON_STATE_UP)
    {
        textRect->x += btn->pressedOffset;
        textRect->y += btn->pressedOffset;
    }
                                     
    GFX_RectClip(textRect, &bounds, drawRect);

	// move the rects to layer space
	laUtils_RectToLayerSpace((laWidget*)btn, textRect);
    laUtils_RectToLayerSpace((laWidget*)btn, drawRect);
}

void _laButtonWidget_InvalidateBorderAreas(laButtonWidget* btn)
{
    GFX_Rect rect, dmgRect;
	int32_t left, top, right, bottom;
	laLayer* layer;
	
	if(btn->widget.borderType == LA_WIDGET_BORDER_NONE)
	    return;
	
	rect = laUtils_WidgetLayerRect((laWidget*)btn);
	layer = laUtils_GetLayer((laWidget*)btn);
	
	if(btn->widget.borderType == LA_WIDGET_BORDER_LINE)
	{
	    if(rect.width == 0 || rect.height == 0)
			return;

		if(rect.width <= 1 || rect.height <= 1)
		{
			laLayer_AddDamageRect(layer, &rect, LA_TRUE);

			return;
		}
	
	    left = 1;
	    top = 1;
	    right = 1;
	    bottom = 1;
	}
	else
	{
	    if(rect.width == 0 || rect.height == 0)
			return;

		if(rect.width <= 3 || rect.height <= 3)
		{
			laLayer_AddDamageRect(layer, &rect, LA_TRUE);
			
			return;
        }
        
        right = 2;
        bottom = 2;
        
        if(btn->state == LA_BUTTON_STATE_UP)
        {
            left = 1;
            top = 1;
        }
        else
        {
            left = 2;
	        top = 2;
	    }
	}
	
	// left line
	dmgRect.x = rect.x;
	dmgRect.y = rect.y;
	dmgRect.width = left;
	dmgRect.height = rect.height;

	laLayer_AddDamageRect(layer, &dmgRect, LA_TRUE);

	// top line
	dmgRect.width = rect.width;
	dmgRect.height = top;

	laLayer_AddDamageRect(layer, &dmgRect, LA_TRUE);

	// right line
	dmgRect.x = rect.x + rect.width - right;
	dmgRect.y = rect.y;
	dmgRect.width = right;
	dmgRect.height = rect.height;

	laLayer_AddDamageRect(layer, &dmgRect, LA_TRUE);

	// bottom line
	dmgRect.x = rect.x;
	dmgRect.y = rect.y + rect.height - bottom;
	dmgRect.width = rect.width;
	dmgRect.height = bottom;

	laLayer_AddDamageRect(layer, &dmgRect, LA_TRUE);
}

static void drawBackground(laButtonWidget* btn);
static void drawString(laButtonWidget* btn);
static void waitString(laButtonWidget* btn);
static void drawImage(laButtonWidget* btn);
static void waitImage(laButtonWidget* btn);
static void drawBorder(laButtonWidget* btn);

static void nextState(laButtonWidget* btn)
{
    switch(btn->widget.drawState)
    {
        case NOT_STARTED:
        {
            if(btn->widget.backgroundType != LA_WIDGET_BACKGROUND_NONE) 
            {
                btn->widget.drawState = DRAW_BACKGROUND;
                btn->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawBackground;

                return;
            }
        }
        case DRAW_BACKGROUND:
        {
            if((btn->state != LA_BUTTON_STATE_UP && btn->pressedImage != NULL) ||
               (btn->state == LA_BUTTON_STATE_UP && btn->releasedImage != NULL))
            {
                btn->widget.drawState = DRAW_IMAGE;
                btn->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawImage;

                return;
            }
        }
        case DRAW_IMAGE:
        {            
            if(laString_IsEmpty(&btn->text) == LA_FALSE)
            {
                btn->widget.drawState = DRAW_STRING;
                btn->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawString;

                return;
            }
        }
        case DRAW_STRING:
        {
            if(btn->widget.borderType != LA_WIDGET_BORDER_NONE)
            {
                btn->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawBorder;
                btn->widget.drawState = DRAW_BORDER;
                
                return;
            }
        }
        case DRAW_BORDER:
        {
            btn->widget.drawState = DONE;
            btn->widget.drawFunc = NULL;
        }
    }
}

static void drawBackground(laButtonWidget* btn)
{
    if(btn->widget.backgroundType == LA_WIDGET_BACKGROUND_FILL)
    {
        if(btn->state != LA_BUTTON_STATE_UP)
            laWidget_SkinClassic_DrawBackground((laWidget*)btn, btn->widget.scheme->background);
        else
            laWidget_SkinClassic_DrawBackground((laWidget*)btn, btn->widget.scheme->base);
    }
    else if(btn->widget.backgroundType == LA_WIDGET_BACKGROUND_CACHE)
    {
        laWidget_SkinClassic_DrawBlit((laWidget*)btn, btn->widget.cache);
    }

    nextState(btn);
}

static void drawImage(laButtonWidget* btn)
{
    GFX_Rect imgRect, imgSrcRect, clipRect;
    GFXU_ImageAsset* img = NULL;
    laLayer* layer = laUtils_GetLayer((laWidget*)btn);
    
    if(btn->state != LA_BUTTON_STATE_UP)
    {
        if(btn->pressedImage != NULL)
            img = btn->pressedImage;
    }
    else
    {
        if(btn->releasedImage != NULL)
            img = btn->releasedImage;
    }
    
    _laButtonWidget_GetImageRect(btn, &imgRect, &imgSrcRect);
    
    if(GFX_RectIntersects(&layer->clippedDrawingRect, &imgRect) == GFX_TRUE)
    {        
        clipRect = GFX_RectClipAdj(&imgRect, &layer->clippedDrawingRect, &imgSrcRect);
        
        GFXU_DrawImage(img,
                       imgSrcRect.x,
                       imgSrcRect.y,
                       imgSrcRect.width,
                       imgSrcRect.height,
                       clipRect.x,
                       clipRect.y,
                       &laContext_GetActive()->memIntf,
                       &btn->reader);
                
        if(btn->reader != NULL)
        {  
            btn->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&waitImage;
            btn->widget.drawState = WAIT_IMAGE;
            
            return;
        }
    }
    
    nextState(btn);
}

static void waitImage(laButtonWidget* btn)
{
    if(btn->reader->status != GFXU_READER_STATUS_FINISHED)
    {
        btn->reader->run(btn->reader);

        return;
    }
    else
    {
        // free the reader
        btn->reader->memIntf->heap.free(btn->reader);
        btn->reader = NULL;
    }
            
    btn->widget.drawState = DRAW_IMAGE;
    
    nextState(btn);
}

static void drawString(laButtonWidget* btn)
{
    GFX_Rect textRect, drawRect, clipRect;

    laLayer* layer = laUtils_GetLayer((laWidget*)btn);
   
    _laButtonWidget_GetTextRect(btn, &textRect, &drawRect);
    
    if(GFX_RectIntersects(&layer->clippedDrawingRect, &drawRect) == GFX_TRUE)
    {
        GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
        
        GFX_Set(GFXF_DRAW_MASK_ENABLE, GFX_FALSE);
        GFX_Set(GFXF_DRAW_COLOR, btn->widget.scheme->text);
        
        laString_DrawClipped(&btn->text,
                             clipRect.x,
                             clipRect.y,
                             clipRect.width,
                             clipRect.height,
                             textRect.x,
                             textRect.y,
                             &btn->reader);
        
        if(btn->reader != NULL)
        {
            btn->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&waitString;
            btn->widget.drawState = WAIT_STRING;
            
            return;
        }
    }
    
    nextState(btn);
}

static void waitString(laButtonWidget* btn)
{
    if(btn->reader->status != GFXU_READER_STATUS_FINISHED)
    {
        btn->reader->run(btn->reader);
        
        return;
    }
    
    // free the reader
    btn->reader->memIntf->heap.free(btn->reader);
    btn->reader = NULL;
    
    btn->widget.drawState = DRAW_STRING;
    
    nextState(btn);
}

static void drawBorder(laButtonWidget* btn)
{
    if(btn->widget.borderType == LA_WIDGET_BORDER_LINE)
    {
        laWidget_SkinClassic_DrawStandardLineBorder((laWidget*)btn);
    }
    else if(btn->widget.borderType == LA_WIDGET_BORDER_BEVEL)
    {
        if(btn->state != LA_BUTTON_STATE_UP)
        {
            laWidget_SkinClassic_DrawStandardLoweredBorder((laWidget*)btn);
        }
        else
        {
            laWidget_SkinClassic_DrawStandardHybridBorder((laWidget*)btn);
        }
    }
    
    nextState(btn);
}

void _laButtonWidget_Paint(laButtonWidget* btn)
{
    laContext* context = laContext_GetActive();
    
    if(btn->widget.scheme == NULL)
    {
        btn->widget.drawState = DONE;
        
        return;
    }
    
    if(btn->widget.drawState == NOT_STARTED)
        nextState(btn);
    
    while(btn->widget.drawState != DONE)
    {
        btn->widget.drawFunc((laWidget*)btn);
        
        if(context->preemptLevel == LA_PREEMPTION_LEVEL_2 ||
           btn->widget.drawState == WAIT_IMAGE ||
           btn->widget.drawState == WAIT_STRING)
            break;
    }
}

#endif // LA_BUTTON_WIDGET_ENABLED