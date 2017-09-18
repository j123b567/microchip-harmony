#include "gfx/libaria/inc/libaria_layer.h"

#include "gfx/libaria/inc/libaria_context.h"
#include "gfx/libaria/inc/libaria_screen.h"
#include "gfx/libaria/inc/libaria_utils.h"

#include "gfx/hal/inc/gfx_rect.h"

#define DEFAULT_WIDTH   100
#define DEFAULT_HEIGHT  100

laLayer* laLayer_New()
{
    laLayer* lyr = NULL;

    if(laContext_GetActive() == NULL)
        return NULL;

    lyr = laContext_GetActive()->memIntf.heap.calloc(1, sizeof(laLayer));
    
    _laWidget_Constructor((laWidget*)lyr);

    lyr->widget.type = LA_WIDGET_LAYER;
    lyr->widget.root = LA_TRUE;
    
    lyr->bufferCount = 1;

    lyr->widget.rect.x = 0;
    lyr->widget.rect.y = 0;
    lyr->widget.rect.width = DEFAULT_WIDTH;
    lyr->widget.rect.height = DEFAULT_HEIGHT;
    
    lyr->alphaEnable = LA_FALSE;

    lyr->widget.moved = (laWidget_Moved_FnPtr)&_laLayer_Moved;
    lyr->widget.resized = (laWidget_Resized_FnPtr)&_laLayer_Resized;
    lyr->widget.paint = (laWidget_Paint_FnPtr)&_laLayer_Paint;
    
    laRectArray_Create(&lyr->prevDamageRects);
    laRectArray_Create(&lyr->frameDamageRects);
    laRectArray_Create(&lyr->pendingDamageRects);
    
    lyr->vsync = LA_TRUE;
    lyr->allowInputPassThrough = LA_TRUE;
    
    lyr->frameState = LA_LAYER_FRAME_READY;

    laLayer_AddDamageRect(lyr, &lyr->widget.rect, LA_FALSE);

    return lyr;
}

void laLayer_Delete(laLayer* layer)
{
    if(layer == NULL)
        return;

    laRectArray_Destroy(&layer->prevDamageRects);
    laRectArray_Destroy(&layer->frameDamageRects);
    laRectArray_Destroy(&layer->pendingDamageRects);
    
    layer->deleting = LA_TRUE;
    
    layer->widget.destructor((laWidget*)layer);    
}

laBool laLayer_GetEnabled(const laLayer* layer)
{
    if(layer == NULL)
        return LA_FAILURE;
        
    return layer->widget.enabled;
}

laResult laLayer_SetEnabled(laLayer* layer, laBool enable)
{
    if(layer == NULL)
        return LA_FAILURE;
    
    layer->widget.enabled = enable;
    
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return GFX_SUCCESS;
    
    GFX_Set(GFXF_LAYER_ACTIVE, laScreen_GetLayerIndex(layer->screen, layer));
    GFX_Set(GFXF_LAYER_ENABLED, layer->widget.enabled);
    
    return GFX_SUCCESS;
}

laBool laLayer_GetAlphaEnable(const laLayer* layer)
{
    if(layer == NULL)
        return LA_FAILURE;
        
    return layer->alphaEnable;
}

laResult laLayer_SetAlphaEnable(laLayer* layer, laBool enable)
{
    if(layer == NULL)
        return LA_FAILURE;
    
    layer->alphaEnable = enable;
    
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return GFX_SUCCESS;
    
    GFX_Set(GFXF_LAYER_ACTIVE, laScreen_GetLayerIndex(layer->screen, layer));
    GFX_Set(GFXF_LAYER_ALPHA_ENABLE, layer->alphaEnable);
    
    return GFX_SUCCESS;
}

laResult laLayer_SetAlphaAmount(laLayer* layer, uint32_t amount)
{
    if(laWidget_SetAlphaAmount((laWidget*)layer, amount) == GFX_FAILURE)
        return GFX_FAILURE;
        
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return GFX_SUCCESS;
    
    GFX_Set(GFXF_LAYER_ACTIVE, laScreen_GetLayerIndex(layer->screen, layer));
    GFX_Set(GFXF_LAYER_ALPHA_AMOUNT, layer->widget.alphaAmount);
    
    return GFX_SUCCESS;
}

laBool laLayer_GetMaskEnable(const laLayer* layer)
{
    if(layer == NULL)
        return 0;
        
    return layer->maskEnable;
}

laResult laLayer_SetMaskEnable(laLayer* layer, laBool enable)
{
    if(layer == NULL || layer->maskEnable == enable)
        return LA_FAILURE;
        
    layer->maskEnable = enable;
        
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return LA_SUCCESS;
    
    GFX_Set(GFXF_LAYER_ACTIVE, laScreen_GetLayerIndex(layer->screen, layer));
    GFX_Set(GFXF_LAYER_MASK_ENABLE, layer->maskEnable);

    return LA_SUCCESS;
}

GFX_Color laLayer_GetMaskColor(const laLayer* layer)
{
    if(layer == NULL)
        return 0;
        
    return layer->maskColor;
}

laResult laLayer_SetMaskColor(laLayer* layer, GFX_Color color)
{
    if(layer == NULL || layer->maskColor == color)
        return LA_FAILURE;
        
    layer->maskColor = color;
        
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return LA_SUCCESS;
    
    GFX_Set(GFXF_LAYER_ACTIVE, laScreen_GetLayerIndex(layer->screen, layer));
    GFX_Set(GFXF_LAYER_MASK_COLOR, layer->maskColor);
    
    return LA_SUCCESS;
}

uint32_t laLayer_GetBufferCount(const laLayer* layer)
{
    if(layer == NULL)
        return 0;

    return layer->bufferCount;
}

laResult laLayer_SetBufferCount(laLayer* layer, uint32_t count)
{
    if(layer == NULL)
        return LA_FAILURE;

    layer->bufferCount = count;
    
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return LA_SUCCESS;
    
    GFX_Set(GFXF_LAYER_BUFFER_COUNT, count);
    
    return LA_SUCCESS;
}

laBool laLayer_GetVSync(const laLayer* layer)
{
    if(layer == NULL)
        return 0;

    return layer->vsync;
}

laResult laLayer_SetVSync(laLayer* layer, laBool enable)
{
    if(layer == NULL)
        return LA_FAILURE;
    
    layer->vsync = enable;
    
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return LA_SUCCESS;
    
    GFX_Set(GFXF_LAYER_VSYNC, enable);
    
    return LA_SUCCESS;
}

laBool laLayer_GetAllowInputPassThrough(const laLayer* layer)
{
    if(layer == NULL)
        return 0;

    return layer->allowInputPassThrough;
}

laResult laLayer_SetAllowInputPassthrough(laLayer* layer, laBool enable)
{
    if(layer == NULL)
        return LA_FAILURE;
    
    layer->allowInputPassThrough = enable;
    
    return LA_SUCCESS;
}

laResult laLayer_AddDamageRect(laLayer* layer,
                               const GFX_Rect* rect,
							   laBool noCombine)
{
    GFX_Rect clipRect;

    if(layer == NULL || layer->deleting == LA_TRUE)
        return LA_FAILURE;

    // make sure rect is inside the layer
    GFX_RectClip(&layer->widget.rect, rect, &clipRect);


    if(layer->frameState == LA_LAYER_FRAME_READY)
    {
        //printf("toggling preframe\n");
        layer->frameState = LA_LAYER_FRAME_PREFRAME;
        
        // get ready for a new frame
        _laWidget_IncreaseDirtyState((laWidget*)layer, LA_WIDGET_DIRTY_STATE_DIRTY);
    }

    if(layer->frameState < LA_LAYER_FRAME_IN_PROGRESS)
        _laLayer_AddDamageRectToList(&layer->frameDamageRects, &clipRect, noCombine);
    else
    {
        //printf("added pending rect\n");
        _laLayer_AddDamageRectToList(&layer->pendingDamageRects, &clipRect, noCombine);
    }
    
    return LA_SUCCESS;
}

laBool laLayer_IsDrawing(laLayer* layer)
{
    if(layer == NULL)
        return LA_FALSE;
        
    return layer->frameState >= LA_LAYER_FRAME_IN_PROGRESS;
}

//int32_t dump = 0;

void _laLayer_Preframe(laLayer* layer)
{
    layer->frameState = LA_LAYER_FRAME_IN_PROGRESS;
    
    //printf("dump: %i\n", dump++);
    
    if(layer->bufferCount > 1 && layer->prevDamageRects.size > 0)
    {
        layer->currentDrawingRect = layer->prevDamageRects.rects[0];
        
        laRectArray_PopFront(&layer->prevDamageRects);
        
        layer->drawingPrev = LA_TRUE;
        
        /*printf("prev: %i, %i, %i, %i\n",
               layer->dmgRectPtr->x,
               layer->dmgRectPtr->y,
               layer->dmgRectPtr->width, 
               layer->dmgRectPtr->height);*/
    }
    else
    {
        layer->currentDrawingRect = layer->frameDamageRects.rects[0];
        
        laRectArray_PopFront(&layer->frameDamageRects);
        
        layer->drawingPrev = LA_FALSE;
        
        /*printf("curr: %i, %i, %i, %i\n",
               layer->dmgRectPtr->x,
               layer->dmgRectPtr->y,
               layer->dmgRectPtr->width, 
               layer->dmgRectPtr->height);*/
    }
    
    _laLayer_InvalidateChildren(layer);
    
    layer->frameDrawCount = 0;
    
    /*GFX_DisableDebugRects();
    
    GFX_SetDebugRect(layer->currentDrawingRect.x,
                     layer->currentDrawingRect.y,
                     layer->currentDrawingRect.width,
                     layer->currentDrawingRect.height);
                     
    GFX_SetDebugRectColor(0x0000FF);
    GFX_ShowDebugRect(GFX_TRUE);
    GFX_NextDebugRect();*/
}

void _laLayer_Postframe(laLayer* layer)
{
    layer->frameDrawCount = 0;
    
    // manage the draw history
    if(layer->bufferCount > 1)
    {
        // rendered a previous rectangle
        if(layer->drawingPrev == LA_TRUE)
        {
            // ready to move on to current array
            if(layer->prevDamageRects.size == 0)
            {
                layer->currentDrawingRect = layer->frameDamageRects.rects[0];
                
                laRectArray_PopFront(&layer->frameDamageRects);
                
                layer->drawingPrev = LA_FALSE;
                
                /*printf("curr: %i, %i, %i, %i\n",
                       layer->dmgRectPtr->x,
                       layer->dmgRectPtr->y,
                       layer->dmgRectPtr->width, 
                       layer->dmgRectPtr->height);*/
            }
            else
            {
                layer->currentDrawingRect = layer->prevDamageRects.rects[0];
                
                laRectArray_PopFront(&layer->prevDamageRects);
        
                /*printf("prev: %i, %i, %i, %i\n",
                       layer->dmgRectPtr->x,
                       layer->dmgRectPtr->y,
                       layer->dmgRectPtr->width, 
                       layer->dmgRectPtr->height);*/
            }
        }
        // rendered a rectangle from this frame
        else
        {
            // add rectangle to previous list for next frame
            laRectArray_PushBack(&layer->prevDamageRects,
                                 &layer->currentDrawingRect);
            
            // more to render?
            if(layer->frameDamageRects.size > 0)
            {
                layer->currentDrawingRect = layer->frameDamageRects.rects[0];
                
                laRectArray_PopFront(&layer->frameDamageRects);
                
                /*printf("curr: %i, %i, %i, %i\n",
                       layer->dmgRectPtr->x,
                       layer->dmgRectPtr->y,
                       layer->dmgRectPtr->width, 
                       layer->dmgRectPtr->height);*/
            }
            else
            {
                layer->currentDrawingRect = GFX_Rect_Zero;
                layer->clippedDrawingRect = GFX_Rect_Zero;
                
                layer->frameState = LA_LAYER_FRAME_COMPLETE;
                
                return;
            }
        }
    }
    else
    {
        if(layer->frameDamageRects.size > 0)
        {
            layer->currentDrawingRect = layer->frameDamageRects.rects[0];
        
            laRectArray_PopFront(&layer->frameDamageRects);
            
            /*printf("curr: %i, %i, %i, %i\n",
                       layer->dmgRectPtr->x,
                       layer->dmgRectPtr->y,
                       layer->dmgRectPtr->width, 
                       layer->dmgRectPtr->height);*/
        }
        else
        {
            layer->currentDrawingRect = GFX_Rect_Zero;
            layer->clippedDrawingRect = GFX_Rect_Zero;
        
            layer->frameState = LA_LAYER_FRAME_COMPLETE;
            
            return;
        }
    }
    
    _laLayer_InvalidateChildren(layer);
    
    /*GFX_SetDebugRect(layer->currentDrawingRect.x,
                     layer->currentDrawingRect.y,
                     layer->currentDrawingRect.width,
                     layer->currentDrawingRect.height);
                     
    GFX_SetDebugRectColor(0x0000FF);
    GFX_ShowDebugRect(GFX_TRUE);
    GFX_NextDebugRect();*/
}

void _laLayer_Moved(laLayer* layer)
{
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return;
    
    GFX_Set(GFXF_LAYER_ACTIVE, laScreen_GetLayerIndex(layer->screen, layer));
    GFX_Set(GFXF_LAYER_POSITION, layer->widget.rect.x, layer->widget.rect.y); 
}

void _laLayer_Resized(laLayer* layer)
{
    if(layer->screen == NULL || laContext_GetActive()->activeScreen != layer->screen)
        return;

    GFX_Set(GFXF_LAYER_ACTIVE, laScreen_GetLayerIndex(layer->screen, layer));
    GFX_Set(GFXF_LAYER_SIZE, layer->widget.rect.width, layer->widget.rect.height); 
}

void _laLayer_Paint(laLayer* layer)
{
    if(layer->widget.backgroundType == LA_WIDGET_BACKGROUND_FILL)
    {
        if(laUtils_WidgetIsOccluded((laWidget*)layer, 
                                    &layer->clippedDrawingRect) == LA_FALSE)
        {
            GFX_Set(GFXF_DRAW_MASK_ENABLE, GFX_FALSE);
            GFX_Set(GFXF_DRAW_COLOR, layer->widget.scheme->background);
            GFX_Set(GFXF_DRAW_MODE, GFX_DRAW_FILL);

            GFX_DrawRect(layer->clippedDrawingRect.x,
                         layer->clippedDrawingRect.y,
                         layer->clippedDrawingRect.width,
                         layer->clippedDrawingRect.height);
                         
            /*printf("layer drew %i, %i, %i, %i\n", layer->dmgRectPtr->x,
                         layer->dmgRectPtr->y,
                         layer->dmgRectPtr->width,
                         layer->dmgRectPtr->height);*/
        }
    }
    
    layer->widget.drawState = LA_WIDGET_DRAW_STATE_DONE;
}

void _laLayer_AddDamageRectToList(laRectArray* arr,
								  const GFX_Rect* rect,
								  laBool noCombine)
{
    uint32_t i;
    
    for(i = 0; i < arr->size; i++)
    {
        // nothing to do, damaged area is already covered by an existing rect
        if(GFX_RectContainsRect(&arr->rects[i], rect) == GFX_TRUE)
        {
            return;
        }
        // new rect completely envelopes old rect, just replace
        else if(GFX_RectContainsRect(rect, &arr->rects[i]) == GFX_TRUE)
        {
            arr->rects[i] = *rect;

            return;
        }
        // two rectangles are touching, combine the areas
        else if(noCombine == LA_FALSE && GFX_RectIntersects(rect, &arr->rects[i]) == GFX_TRUE)
        {
            arr->rects[i] = GFX_RectCombine(rect, &arr->rects[i]);

            return;
        }
    }

    laRectArray_PushBack(arr, rect);
}

void _laLayer_InvalidateChildren(laLayer* layer)
{
    uint32_t i;
    
    _laWidget_ValidateChildren((laWidget*)layer);
    
    if(laUtils_WidgetIsOccluded((laWidget*)layer,
       &layer->currentDrawingRect) == GFX_FALSE)
    {
        _laWidget_SetDirtyState((laWidget*)layer, LA_WIDGET_DIRTY_STATE_DIRTY);
    }
    else
    {
        _laWidget_IncreaseDirtyState((laWidget*)layer, LA_WIDGET_DIRTY_STATE_CHILD);
    }
        
    for(i = 0; i < layer->widget.children.size; i++)
        _laWidget_Invalidate(layer->widget.children.values[i],
                             &layer->currentDrawingRect);
}