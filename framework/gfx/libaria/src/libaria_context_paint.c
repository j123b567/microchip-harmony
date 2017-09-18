#include "gfx/libaria/inc/libaria_context.h"

#include "gfx/hal/gfx.h"

#include "gfx/libaria/inc/libaria_layer.h"
#include "gfx/libaria/inc/libaria_screen.h"
#include "gfx/libaria/inc/libaria_utils.h"
#include "gfx/libaria/inc/libaria_widget.h"

static void updateDirtyFlags(laWidget* widget)
{
    laWidget* child;
    GFX_Rect childRect;
    uint32_t i;

    _laWidget_ClearDirtyState(widget);
    
    for(i = 0; i < widget->children.size; i++)
    {
        child = widget->children.values[i];
        
        childRect = child->rect;
        childRect.x += widget->rect.x;
        childRect.y += widget->rect.y;

        // pre-cull based on visibility and rectangle intersection
        // anything not visible or completely outside parent can be ignored.
        if(child->visible == LA_TRUE &&
           child->dirtyState != LA_WIDGET_DIRTY_STATE_CLEAN &&
           GFX_RectIntersects(&widget->rect, &childRect) == LA_TRUE)
        {
            _laWidget_IncreaseDirtyState(widget, LA_WIDGET_DIRTY_STATE_CHILD);
        }
        else
        {            
            // clear all dirty flags for this child and its descendents
            _laWidget_ValidateChildren(child);
        }
    }
}

GFX_Result _laContext_PaintWidget(laWidget* widget)
{
    laWidget* child;
    GFX_Rect widgetRect, parentRect, clipRect, cacheRect;
    laBool alphaEnable;
    laBool shouldPaint = LA_FALSE;
    laBool painted = LA_FALSE;
    laLayer* layer;
    uint32_t i;
    
    layer = laUtils_GetLayer(widget);
    
    // fill widget cache if needed
    if(widget->backgroundType == LA_WIDGET_BACKGROUND_CACHE)
    {
        if(widget->cache == NULL)
        {
            // allocate struct
            widget->cache = laContext_GetActive()->memIntf.heap.malloc(sizeof(GFX_PixelBuffer));
            
            laContext_GetActive()->memIntf.heap.memset(widget->cache, 0, sizeof(GFX_PixelBuffer));
            
            widget->cacheInvalid = LA_TRUE;
        } 
        else if(layer->frameDrawCount > 0)
        {
            widget->cacheInvalid = LA_TRUE;
        }
    
        if(widget->cacheInvalid == GFX_TRUE)
        {
            cacheRect = laUtils_WidgetLocalRect((laWidget*)widget);
            
            laUtils_RectToLayerSpace(widget, &cacheRect);
            
            // fill the cache from the in-progress write buffer
            GFX_ReadPixelArray(GFX_BUFFER_WRITE,
                               &cacheRect,
                               widget->cache);
            
            widget->cacheInvalid = LA_FALSE;
        }
    }

    // skip any child that isn't dirty or that does not have a dirty descendent
    if(widget->dirtyState == LA_WIDGET_DIRTY_STATE_CLEAN)
        return GFX_SUCCESS;
        
    // if widget is completely transparent just mark clean and return
    if(widget->alphaEnabled == GFX_TRUE && widget->alphaAmount == 0)
    {
        _laWidget_ValidateChildren(widget);
        
        return GFX_SUCCESS;
    }
    
    if(widget->dirtyState == LA_WIDGET_DIRTY_STATE_DIRTY)
        shouldPaint = LA_TRUE;
    
    if((widget->optimizationFlags & LA_WIDGET_OPT_DRAW_ONCE) > 0 &&
       widget->drawCount >= laUtils_GetLayer(widget)->bufferCount)
        shouldPaint = LA_FALSE;
    
    // paint this widget and clear dirty flag
    if(shouldPaint == LA_TRUE)
    {
        // manage alpha blending
        alphaEnable = laWidget_GetCumulativeAlphaEnable(widget);
        
        if(alphaEnable == GFX_TRUE)
        {
            GFX_Set(GFXF_DRAW_BLEND_MODE, GFX_BLEND_GLOBAL);
            GFX_Set(GFXF_DRAW_ALPHA_VALUE, laWidget_GetCumulativeAlphaAmount(widget));
        }
        else
        {
            GFX_Set(GFXF_DRAW_BLEND_MODE, GFX_BLEND_NONE);
        }
        
        // clip the damage rectangle to the child's parent
        if(widget->parent != NULL)
        {
            widgetRect = laUtils_WidgetLayerRect(widget);
            parentRect = laUtils_WidgetLayerRect(widget->parent);
            
            // child does not intersect parent at all, do not draw
            if(GFX_RectIntersects(&widgetRect, &parentRect) == GFX_FALSE)
            {
                _laWidget_ValidateChildren(widget);
        
                return GFX_SUCCESS;
            }
            
            // get the delta area between the parent and child
            GFX_RectClip(&widgetRect, &parentRect, &clipRect);
            
            // widget visible area does not intersect dirty area at all
            // do not draw
            if(GFX_RectIntersects(&clipRect,
                                  &layer->currentDrawingRect) == GFX_FALSE)
            {
                _laWidget_ValidateChildren(widget);
        
                return GFX_SUCCESS;
            }
            
            // get the delta area between the dirty area and the child/parent
            // delta area
            GFX_RectClip(&layer->currentDrawingRect,
                         &clipRect,
                         &layer->clippedDrawingRect);
        }
        else
        {
            //layer->dmgRectPtr = &layer->currentRect;
            layer->clippedDrawingRect = layer->currentDrawingRect;
        }
        
        // turn off clipping by default
        GFX_Set(GFXF_DRAW_CLIP_ENABLE, GFX_FALSE);
        
        widget->paint(widget);
        widget->drawCount++;
        
        painted = LA_TRUE;
        
        //printf("painting widget - %i, %i\n", widget->type, widget->id);
        
        if(widget->drawState != LA_WIDGET_DRAW_STATE_DONE)
            return GFX_FAILURE;
            
        layer->frameDrawCount++;
    }
    
    _laWidget_ClearDirtyState(widget);
    
    // update widget dirty flags
    updateDirtyFlags(widget);
    
    // preempt if necessary
    if(laContext_GetActive()->preemptLevel >= LA_PREEMPTION_LEVEL_1 &&
       painted == LA_TRUE)
        return GFX_FAILURE;
        
    if(widget->children.size == 0)
        return GFX_SUCCESS;
    
    // draw children
    for(i = 0; i < widget->children.size; i++)
    {
        child = widget->children.values[i];

        if(child->dirtyState != LA_WIDGET_DIRTY_STATE_CLEAN)
        {
            if(_laContext_PaintWidget(child) == GFX_FAILURE)
                return GFX_FAILURE;
        }
    }
    
    // update widget dirty flags (should come out clean)
    updateDirtyFlags(widget);
    
    return GFX_SUCCESS;
}

void _laContext_Paint()
{
    laScreen* activeScreen;
    laLayer* layer;
    GFX_Rect screenRect;
    uint32_t i;

    activeScreen = laContext_GetActive()->activeScreen;
    
    if(activeScreen == NULL)
        return;

    screenRect = laContext_GetScreenRect();
            
    //GFX_Set(GFXF_DRAW_CLIP_ENABLE, GFX_TRUE);
    GFX_Set(GFXF_DRAW_CLIP_ENABLE, GFX_FALSE);

    // iterate over all qualifying layers and paint
    for(i = 0; i < LA_MAX_LAYERS; i++)
    {
        layer = activeScreen->layers[i];

        if(layer == NULL ||
           layer->widget.enabled == LA_FALSE ||
           GFX_RectIntersects(&screenRect, &layer->widget.rect) == LA_FALSE)
            continue;
        
        if(layer->widget.dirtyState == LA_WIDGET_DIRTY_STATE_CLEAN)
            continue;
            
        if(layer->frameState == LA_LAYER_FRAME_PREFRAME)
        {
            //printf("preframe\n");
            
            _laLayer_Preframe(layer);
        }
        
        // set active driver layer
        GFX_Set(GFXF_LAYER_ACTIVE, i);
        
        // notify the draw layer that the library wants to start drawing to the active layer
        if(GFX_Begin() != GFX_SUCCESS)
            return;
            
        // paint root node
        _laContext_PaintWidget((laWidget*)layer);
        
         // notify the draw layer that drawing is over
        GFX_End();
        
        // indicate swap if layer is clean
        if(layer->widget.dirtyState == LA_WIDGET_DIRTY_STATE_CLEAN)
            _laLayer_Postframe(layer);
        
        if(layer->frameState == LA_LAYER_FRAME_COMPLETE)
        {
            GFX_Set(GFXF_LAYER_SWAP, GFX_TRUE);
            
            //printf("swap\n");
            
            if(layer->pendingDamageRects.size > 0)
            {
                laRectArray_Copy(&layer->pendingDamageRects, &layer->frameDamageRects);
                laRectArray_Clear(&layer->pendingDamageRects);
                
                _laWidget_IncreaseDirtyState((laWidget*)layer, LA_WIDGET_DIRTY_STATE_DIRTY);
                layer->frameState = LA_LAYER_FRAME_PREFRAME;
            }
            else
            {
                layer->frameState = LA_LAYER_FRAME_READY;
            }
        }
    }
    
    GFX_Set(GFXF_DRAW_CLIP_ENABLE, GFX_FALSE);
}