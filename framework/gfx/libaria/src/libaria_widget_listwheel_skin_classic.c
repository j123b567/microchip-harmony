#include "gfx/libaria/inc/libaria_widget_listwheel.h"
#include "gfx/hal/inc/gfx_math.h"

#if LA_LISTWHEEL_WIDGET_ENABLED

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
    DRAW_STRING,
    WAIT_STRING,
    DRAW_ICON,
    WAIT_ICON,
    DRAW_INDICATORS,
    DRAW_BORDER,
};

static int32_t getItemY(laListWheelWidget* whl,
                        int32_t pos,
                        int32_t itemHeight)
{
    int32_t y = pos * whl->paintState.per;
    
    return (y - (itemHeight / 2)) + whl->rotation;
}

void _laListWheelWidget_GetItemTextRect(laListWheelWidget* whl,
                                        uint32_t idx,
                                        uint32_t pos,
                                        GFX_Rect* textRect,
                                        GFX_Rect* drawRect)
{
    GFX_Rect bounds, imageRect;
    laListWheelItem* item;
    int32_t y;
        
    // get rectangles
    item = whl->items.values[idx];
    
    laString_GetRect(&item->string, textRect);
    
    bounds.x = 0;
    bounds.y = 0;
    bounds.width = whl->widget.rect.width;
    bounds.height = textRect->height;
    
    imageRect = GFX_Rect_Zero;
    
    if(item->icon != NULL)
    {
        if(bounds.height < (int32_t)item->icon->height)
            bounds.height = (int32_t)item->icon->height;
            
        imageRect.width = item->icon->width;
        imageRect.height = item->icon->height;
    }
    
    // arrange relative to image rect
    laUtils_ArrangeRectangleRelative(textRect,
                                     imageRect,
                                     bounds,
                                     whl->halign,
                                     LA_VALIGN_MIDDLE,
                                     whl->iconPos,
                                     whl->widget.margin.left,
                                     whl->widget.margin.top,
                                     whl->widget.margin.right,
                                     whl->widget.margin.bottom,
                                     whl->iconMargin);
                                     
    GFX_RectClip(textRect, &bounds, drawRect);

	// move the rects to layer space
	laUtils_RectToLayerSpace((laWidget*)whl, textRect);
    laUtils_RectToLayerSpace((laWidget*)whl, drawRect);   
    
    y = getItemY(whl, pos, bounds.height);
    
    textRect->y += y;
    drawRect->y += y;  
}

void _laListWheelWidget_GetItemIconRect(laListWheelWidget* whl,
                                        uint32_t idx,
                                        uint32_t pos,
                                        GFX_Rect* imgRect,
                                        GFX_Rect* imgSrcRect)
{
    GFX_Rect bounds = {0};
    GFX_Rect textRect = {0};
    laListWheelItem* item = whl->items.values[idx];
        
    GFXU_ImageAsset* img = item->icon;
    
    imgRect->x = 0;
    imgRect->y = 0;
    imgRect->width = img->width;
    imgRect->height = img->height;
    
    laString_GetRect(&item->string, &textRect);
    
    bounds.x = 0;
    bounds.y = 0;
    bounds.width = whl->widget.rect.width;
    bounds.height = textRect.height;
    
    if(item->icon != NULL)
    {
        if(bounds.height < (int32_t)item->icon->height)
            bounds.height = (int32_t)item->icon->height;
            
        imgRect->width = item->icon->width;
        imgRect->height = item->icon->height;
    }
    
    *imgSrcRect = *imgRect;
    
    // arrange image rect
    laUtils_ArrangeRectangle(imgRect,
                             textRect,
                             bounds,
                             whl->halign,
                             LA_VALIGN_MIDDLE,
                             whl->iconPos,
                             whl->widget.margin.left,
                             whl->widget.margin.top,
                             whl->widget.margin.right,
                             whl->widget.margin.bottom,
                             whl->iconMargin);   
                             
    *imgRect = GFX_RectClipAdj(imgRect, &bounds, imgSrcRect); 
    
    laUtils_RectToLayerSpace((laWidget*)whl, imgRect); 
    
    imgRect->y += getItemY(whl, pos, bounds.height);
    
    //if(pos == 1)
    //    printf("y=%i\n", getItemY(whl, pos, bounds.height));
}

static void drawBackground(laListWheelWidget* whl);
static void drawString(laListWheelWidget* whl);
static void waitString(laListWheelWidget* whl);
static void drawIcon(laListWheelWidget* whl);
static void waitIcon(laListWheelWidget* whl);
static void drawIndicators(laListWheelWidget* whl);
static void drawBorder(laListWheelWidget* whl);

static void nextState(laListWheelWidget* whl)
{
    switch(whl->widget.drawState)
    {
        case NOT_STARTED:
        {
            
            whl->paintState.per = GFX_DivideRounding(whl->widget.rect.height, whl->visibleItems - 1);
            
            whl->widget.drawState = DRAW_BACKGROUND;
            whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawBackground;

            return;
        }
        case DRAW_BACKGROUND:
        {
            if(whl->items.size > 0)
            {
                whl->paintState.nextItem = whl->topItem;
                whl->paintState.y = -1;
                
                whl->widget.drawState = DRAW_STRING;
                whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawString;
            
                return;
            }
        }
        case DRAW_STRING:
        {            
            //printf("\n");
            
            if(whl->items.size > 0)
            {
                whl->paintState.nextItem = whl->topItem;
                whl->paintState.y = -1;
                
                whl->widget.drawState = DRAW_ICON;
                whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawIcon;
            
                return;
            }
        }
        case DRAW_ICON:
        {
            if(whl->showIndicators == LA_TRUE)
            {                
                whl->widget.drawState = DRAW_INDICATORS;
                whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawIndicators;
            
                return;
            }
        }
        case DRAW_INDICATORS:
        {
            if(whl->widget.borderType != LA_WIDGET_BORDER_NONE)
            {
                whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawBorder;
                whl->widget.drawState = DRAW_BORDER;
                
                return;
            }
        }
        case DRAW_BORDER:
        {
            whl->widget.drawState = DONE;
            whl->widget.drawFunc = NULL;
        }
    }
}

static void nextItem(laListWheelWidget* whl)
{
    whl->paintState.nextItem++;
    whl->paintState.y++;
    
    if(whl->paintState.nextItem >= whl->items.size)
        whl->paintState.nextItem = 0;
}



static void drawBackground(laListWheelWidget* whl)
{
    GFX_Rect widgetRect, drawRect, clipRect;
    laLayer* layer;
    
    //printf("drawbackground\n");

    if(whl->widget.backgroundType == LA_WIDGET_BACKGROUND_FILL)
    {
        if(whl->shaded == LA_TRUE)
        {
            widgetRect = laUtils_WidgetLocalRect((laWidget*)whl);
            laUtils_RectToLayerSpace((laWidget*)whl, &widgetRect);
            
            layer = laUtils_GetLayer((laWidget*)whl);
        
            GFX_Set(GFXF_DRAW_MASK_ENABLE, GFX_FALSE);
            GFX_Set(GFXF_DRAW_MODE, GFX_DRAW_FILL);
            GFX_Set(GFXF_DRAW_CLIP_ENABLE, GFX_TRUE);
        
            // upper rectangle
            GFX_Set(GFXF_DRAW_GRADIENT_COLOR,
                whl->widget.scheme->backgroundDisabled,
                whl->widget.scheme->backgroundInactive,
                NULL,
                NULL);
            GFX_Set(GFXF_DRAW_MODE, GFX_DRAW_GRADIENT_TOP_BOTTOM);
            
            drawRect.x = widgetRect.x;
            drawRect.y = widgetRect.y;
            drawRect.width = widgetRect.width;
            drawRect.height = (widgetRect.height / 2) - whl->indicatorArea;
            
            if(GFX_RectIntersects(&drawRect, &layer->clippedDrawingRect) == GFX_TRUE)
            {
                GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
                
                GFX_Set(GFXF_DRAW_CLIP_RECT, &clipRect);
                
                GFX_DrawRect(drawRect.x,
                             drawRect.y,
                             drawRect.width,
                             drawRect.height);
            }
                         
            // middle rectangle
            drawRect.y = (widgetRect.y + widgetRect.height / 2) - whl->indicatorArea;
            drawRect.height = whl->indicatorArea * 2;
            
            GFX_Set(GFXF_DRAW_COLOR, whl->widget.scheme->background);
            GFX_Set(GFXF_DRAW_MODE, GFX_DRAW_FILL);
            
            if(GFX_RectIntersects(&drawRect, &layer->clippedDrawingRect) == GFX_TRUE)
            {
                GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
                
                GFX_Set(GFXF_DRAW_CLIP_RECT, &clipRect);
                
                GFX_DrawRect(drawRect.x,
                             drawRect.y,
                             drawRect.width,
                             drawRect.height);
            }
                         
            // lower rectangle
            GFX_Set(GFXF_DRAW_GRADIENT_COLOR,
                whl->widget.scheme->backgroundInactive,
                whl->widget.scheme->backgroundDisabled,
                NULL,
                NULL);
            GFX_Set(GFXF_DRAW_MODE, GFX_DRAW_GRADIENT_TOP_BOTTOM);
            
            drawRect.y = (widgetRect.y + widgetRect.height / 2) + whl->indicatorArea;
            drawRect.height = (widgetRect.height / 2) - whl->indicatorArea;
            
            if(GFX_RectIntersects(&drawRect, &layer->clippedDrawingRect) == GFX_TRUE)
            {
                GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
                
                GFX_Set(GFXF_DRAW_CLIP_RECT, &clipRect);
                
                GFX_DrawRect(drawRect.x,
                             drawRect.y,
                             drawRect.width,
                             drawRect.height);
            }
            
            //GFX_Set(GFXF_DRAW_CLIP_RECT, &widgetRect);
            GFX_Set(GFXF_DRAW_CLIP_ENABLE, GFX_FALSE);
        }
        else
        {
            laWidget_SkinClassic_DrawBackground((laWidget*)whl, whl->widget.scheme->background);
        }
    }
    else if(whl->widget.backgroundType == LA_WIDGET_BACKGROUND_CACHE)
    {
        laWidget_SkinClassic_DrawBlit((laWidget*)whl, whl->widget.cache);
    }
   
    nextState(whl);    
}


static void drawString(laListWheelWidget* whl)
{
    GFX_Rect textRect, drawRect, widgetRect, clipRect;
    laListWheelItem* item;
    laLayer* layer = laUtils_GetLayer((laWidget*)whl);
 
    //if(whl->paintState.nextItem < whl->items.size)
    //    nextItem(whl);
    
    if(!(whl->paintState.y < whl->visibleItems + 1))
    {
        nextState(whl);

        return;
    }
    
    item = whl->items.values[whl->paintState.nextItem];
    
    //printf("drawing item: %i, rot=%i\n", whl->paintState.nextItem, whl->rotation);
    
    if(laString_IsEmpty(&item->string) == LA_TRUE)
    {
        nextItem(whl);
        
        return;
    }
    
    _laListWheelWidget_GetItemTextRect(whl,
                                       whl->paintState.nextItem,
                                       whl->paintState.y,
                                       &textRect,
                                       &drawRect);
    
    widgetRect = laUtils_WidgetLayerRect((laWidget*)whl);
    
    if(GFX_RectIntersects(&drawRect, &widgetRect) == GFX_FALSE)
    {
        nextItem(whl);
        
        return;
    }
    
    // clip draw rect to widget rect
    GFX_RectClip(&drawRect, &widgetRect, &clipRect);
    
    drawRect = clipRect;
    
    item = whl->items.values[whl->paintState.nextItem];

    if(GFX_RectIntersects(&layer->clippedDrawingRect, &drawRect) == GFX_TRUE)
    {        
        GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
            
        GFX_Set(GFXF_DRAW_MASK_ENABLE, GFX_FALSE);
        GFX_Set(GFXF_DRAW_COLOR, whl->widget.scheme->text);
        
        laString_DrawClipped(&item->string,
                             clipRect.x,
                             clipRect.y,
                             clipRect.width,
                             clipRect.height,
                             textRect.x,
                             textRect.y,
                             &whl->reader);
    
        if(whl->reader != NULL)
        {
            whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&waitString;
            whl->widget.drawState = WAIT_STRING;
            
            return;
        }
    }
    
    nextItem(whl);
}

static void waitString(laListWheelWidget* whl)
{
    if(whl->reader->status != GFXU_READER_STATUS_FINISHED)
    {
        whl->reader->run(whl->reader);
        
        return;
    }
    
    // free the reader
    whl->reader->memIntf->heap.free(whl->reader);
    whl->reader = NULL;
    
    nextItem(whl);
    
    whl->widget.drawState = DRAW_STRING;
    whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawString;
}

static void drawIcon(laListWheelWidget* whl)
{
    GFX_Rect widgetRect, imgRect, imgSrcRect, clipRect;
    //int32_t y;
    laLayer* layer = laUtils_GetLayer((laWidget*)whl);
    laListWheelItem* item = whl->items.values[whl->paintState.nextItem];
    
    if(!(whl->paintState.y < whl->visibleItems + 1))
    {
        nextState(whl);

        return;
    }
    
    // skip if no icon
    if(item->icon == NULL)
    {
        nextItem(whl);
        
        return;
    }
    
    _laListWheelWidget_GetItemIconRect(whl,
                                       whl->paintState.nextItem,
                                       whl->paintState.y,
                                       &imgRect,
                                       &imgSrcRect);
    
    widgetRect = laUtils_WidgetLayerRect((laWidget*)whl);
    
    if(GFX_RectIntersects(&imgRect, &widgetRect) == GFX_TRUE)
    {
        // clip imgrect to widget rect
        imgRect = GFX_RectClipAdj(&imgRect, &widgetRect, &imgSrcRect);
     
        if(GFX_RectIntersects(&imgRect, &layer->clippedDrawingRect) == GFX_TRUE)
        {
            clipRect = GFX_RectClipAdj(&imgRect, &layer->clippedDrawingRect, &imgSrcRect);
            
            GFXU_DrawImage(item->icon,
                           imgSrcRect.x,
                           imgSrcRect.y,
                           imgSrcRect.width,
                           imgSrcRect.height,
                           clipRect.x,
                           clipRect.y,
                           &laContext_GetActive()->memIntf,
                           &whl->reader);
                    
            if(whl->reader != NULL)
            {
                whl->widget.drawState = WAIT_ICON;
                whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&waitIcon;
                
                return;
            }
        }
    }
    
    nextItem(whl);
}


static void waitIcon(laListWheelWidget* whl)
{
    if(whl->reader->status != GFXU_READER_STATUS_FINISHED)
    {
        whl->reader->run(whl->reader);
        
        return;
    }
    
    // free the reader
    whl->reader->memIntf->heap.free(whl->reader);
    whl->reader = NULL;
    
    nextItem(whl);
    
    whl->widget.drawState = DRAW_ICON;
    whl->widget.drawFunc = (laWidget_DrawFunction_FnPtr)&drawIcon;
}

static void drawIndicators(laListWheelWidget* whl)
{
    GFX_Rect rect, drawRect, clipRect;
    uint32_t halfHeight;
    uint32_t topLine;
    uint32_t bottomLine;
    laLayer* layer;
    
    layer = laUtils_GetLayer((laWidget*)whl);
    
    rect = laUtils_WidgetLocalRect((laWidget*)whl);
    halfHeight = rect.height / 2;
    
    laUtils_RectToLayerSpace((laWidget*)whl, &rect);
    
    topLine = (rect.y + halfHeight) - whl->indicatorArea;
    bottomLine = (rect.y + halfHeight) + whl->indicatorArea;
    
    // upper lines
    GFX_Set(GFXF_DRAW_COLOR, whl->widget.scheme->foreground);
    GFX_Set(GFXF_DRAW_MODE, GFX_DRAW_LINE);
    
    drawRect.x = rect.x;
    drawRect.y = topLine;
    drawRect.width = rect.width;
    drawRect.height = 1;
    
    // top outer line
    if(GFX_RectIntersects(&drawRect, &layer->clippedDrawingRect) == GFX_TRUE)
    {
        GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
        
        GFX_DrawRect(clipRect.x,
                     clipRect.y,
                     clipRect.width,
                     clipRect.height);
    }
    
    drawRect.y = bottomLine;
    
    // bottom inner line
    if(GFX_RectIntersects(&drawRect, &layer->clippedDrawingRect) == GFX_TRUE)
    {
        GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
        
        GFX_DrawRect(clipRect.x,
                     clipRect.y,
                     clipRect.width,
                     clipRect.height);
    }
    
    // top inner line
    GFX_Set(GFXF_DRAW_COLOR, whl->widget.scheme->foregroundDisabled);
    
    drawRect.y = topLine + 1;
    
    // bottom inner line
    if(GFX_RectIntersects(&drawRect, &layer->clippedDrawingRect) == GFX_TRUE)
    {
        GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
        
        GFX_DrawRect(clipRect.x,
                     clipRect.y,
                     clipRect.width,
                     clipRect.height);
    }
    
    // bottom outer line
    drawRect.y = bottomLine + 1;
    
    if(GFX_RectIntersects(&drawRect, &layer->clippedDrawingRect) == GFX_TRUE)
    {
        GFX_RectClip(&drawRect, &layer->clippedDrawingRect, &clipRect);
        
        GFX_DrawRect(clipRect.x,
                     clipRect.y,
                     clipRect.width,
                     clipRect.height);
    }
    
    nextState(whl);
}

static void drawBorder(laListWheelWidget* whl)
{
    if(whl->widget.borderType == LA_WIDGET_BORDER_LINE)
        laWidget_SkinClassic_DrawStandardLineBorder((laWidget*)whl);
    else if(whl->widget.borderType == LA_WIDGET_BORDER_BEVEL)
        laWidget_SkinClassic_DrawStandardLoweredBorder((laWidget*)whl);
    
    nextState(whl);
}

void _laListWheelWidget_Paint(laListWheelWidget* whl)
{
    laContext* context = laContext_GetActive();
    
    if(whl->widget.scheme == NULL)
    {
        whl->widget.drawState = DONE;
        
        return;
    }
    
    if(whl->widget.drawState == NOT_STARTED)
        nextState(whl);
    
    while(whl->widget.drawState != DONE)
    {
        whl->widget.drawFunc((laWidget*)whl);
        
        if(context->preemptLevel == LA_PREEMPTION_LEVEL_2 ||
           whl->widget.drawState == WAIT_STRING ||
           whl->widget.drawState == WAIT_ICON)
            break;
    }
}

#endif // LA_LISTWHEEL_WIDGET_ENABLED