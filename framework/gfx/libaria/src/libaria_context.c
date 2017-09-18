/*******************************************************************************
 Module for Microchip Graphics Library - Graphic Object Layer

  Company:
    Microchip Technology Inc.

  File Name:
    libaria.c

  Summary:
    Graphics Object Layer of the Microchip Graphics Library.

  Description:
    This module implements the common routines for the Graphics Object Layer
    of the Microchip Graphics Library. The routines are independent of the
    Display Driver Layer and should be compatible with any Display Driver
    that is compliant with the requirements of the Display Driver
    Layer of the Graphics Library.
    The module utilizes the Graphics Primitive Layer to render the objects.
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
#include "gfx/libaria/inc/libaria_context.h"

#include "gfx/hal/gfx.h"
#include "gfx/utils/gfx_utils.h"

#include "gfx/libaria/inc/libaria_draw.h"
#include "gfx/libaria/inc/libaria_layer.h"
#include "gfx/libaria/inc/libaria_list.h"
#include "gfx/libaria/inc/libaria_screen.h"
#include "gfx/libaria/inc/libaria_utils.h"

laContext* _activeContext = NULL;

typedef struct laContext_ScreenChangeEvent_t
{
    laEvent event;
    
    uint32_t index;
} laContext_ScreenChangeEvent;

laContext* laContext_Create(GFX_Driver driver,
                            GFX_Display display,
                            GFX_Processor processor,
                            GFX_ColorMode mode,
                            GFXU_MemoryIntf* memoryIntf)
{
    laContext* context;

    if(memoryIntf == NULL)
    {
        context = calloc(1, sizeof(laContext));

        context->memIntf.heap.malloc = (GFX_Malloc_FnPtr) &malloc;
        context->memIntf.heap.calloc = (GFX_Calloc_FnPtr) &calloc;
        context->memIntf.heap.realloc = (GFX_Realloc_FnPtr) &realloc;
        context->memIntf.heap.free = (GFX_Free_FnPtr) &free;
        context->memIntf.heap.memset = (GFX_Memset_FnPtr) &memset;
        context->memIntf.heap.memcpy = (GFX_Memcpy_FnPtr) &memcpy;
    }
    else
    {
        context = memoryIntf->heap.calloc(1, sizeof(laContext));
        
        if(context == NULL)
            return NULL;

        context->memIntf = *memoryIntf;
    }

    context->gfxContext = GFX_Open(driver,
                                   processor,
                                   display,
                                   &memoryIntf->heap);

    if(context->gfxContext == NULL)
        return NULL;

    context->displayIndex = display;

    laArray_Create(&context->screenList);
    
    context->colorMode = mode;

    if(_activeContext == NULL)
        _activeContext = context;
        
    laScheme_Initialize(&_activeContext->defaultScheme, mode);
    laInput_SetEnabled(LA_TRUE);

    return context;
}

laResult laContext_Destroy(laContext* context)
{
    uint32_t i;
    
    if(context == NULL)
        return LA_FAILURE;

    for(i = 0; i < context->screenList.size; i++)
        laScreen_Delete(context->screenList.values[i]);
    
    laArray_Destroy(&context->screenList);
    
    context->activeScreen = NULL;
    
    laList_Destroy(&context->event.events);
    
    GFX_Close();
    
    if(_activeContext == context)
        _activeContext = NULL;
    
    context->memIntf.heap.free(context);

    return LA_SUCCESS;
}

void laContext_Update(uint32_t dt)
{
    if(_activeContext == NULL)
        return;

    laEvent_ProcessEvents();
    
    _laContext_Update(dt);
    _laContext_Paint();
}

laContext* laContext_GetActive()
{
    return _activeContext;
}

laResult laContext_SetActive(laContext* context)
{
    _activeContext = context;

    return LA_SUCCESS;
}

GFX_ColorMode laContext_GetColorMode()
{
    if(_activeContext == NULL)
        return GFX_COLOR_MODE_GS_8;

    return _activeContext->colorMode;
}

laResult laContext_AddScreen(laScreen* screen)
{
    if(_activeContext == NULL || screen == NULL)
        return LA_FAILURE;
        
    screen->id = _activeContext->screenList.size;

    laArray_PushBack(&_activeContext->screenList, screen);

    return LA_SUCCESS;
}

laResult laContext_RemoveScreen(laScreen* screen)
{
    if(_activeContext == NULL || screen == NULL)
        return LA_FAILURE;

    if(laArray_Remove(&_activeContext->screenList, screen) == LA_FAILURE)
        return LA_FAILURE;
        
    return LA_SUCCESS;
}

GFX_Rect laContext_GetScreenRect()
{
    GFX_Rect rect = {0};

    if(_activeContext == NULL)
        return rect;

    rect = GFX_DisplayInfoPtrGet(_activeContext->displayIndex)->rect;

    return rect;
}

laScreen* laContext_GetActiveScreen()
{
    if(_activeContext == NULL)
        return NULL;
        
    return _activeContext->activeScreen;
}

int32_t laContext_GetActiveScreenIndex()
{
    if(_activeContext == NULL || _activeContext->activeScreen == NULL)
        return -1;
    
    return laArray_Find(&_activeContext->screenList, _activeContext->activeScreen);
}

laResult laContext_SetActiveScreen(uint32_t idx)
{
    laContext_ScreenChangeEvent* evt;
    
    if(_activeContext == NULL || idx < 0 || idx >= _activeContext->screenList.size)
        return LA_FAILURE;

    if(laContext_GetActiveScreenIndex() == idx)
        return LA_SUCCESS;
    
    // if no active screen then change immediately    
    if(_activeContext->activeScreen == NULL)
    {
        laScreen_Show(_activeContext->screenList.values[idx]);
    }
    // else execute later to avoid race conditions
    else
    {
        evt = _activeContext->memIntf.heap.malloc(sizeof(laContext_ScreenChangeEvent));
        
        evt->event.id = LA_EVENT_SCREEN_CHANGE;
        evt->index = idx;
        
        laEvent_AddEvent((laEvent*)evt);
    }

    return LA_SUCCESS;
}

laResult laContext_HideActiveScreen()
{
    if(_activeContext == NULL)
        return LA_FAILURE;

    if(_activeContext->activeScreen != NULL)
        return laScreen_Hide(_activeContext->activeScreen);

    return LA_FAILURE;
}

laPreemptionLevel laContext_GetPreemptionLevel()
{
    if(_activeContext == NULL)
        return 0;
        
    return _activeContext->preemptLevel;
}

laResult laContext_SetPreemptionLevel(laPreemptionLevel level)
{
    if(_activeContext == NULL)
        return LA_FAILURE;

    _activeContext->preemptLevel = level;

    return LA_SUCCESS;
}

// rework this to be non-recursive if necessary
void _laContext_UpdateWidget(laWidget* widget, uint32_t dt)
{
    laWidget* child;
    uint32_t i;

    widget->update(widget, dt);

    // iterate over all existing children for update
    for(i = 0; i < widget->children.size; i++)
    {
        child = widget->children.values[i];

        _laContext_UpdateWidget(child, dt);
    }
}

void _laContext_Update(uint32_t dt)
{
    laScreen* activeScreen;
    laLayer* layer;
    uint32_t modDelta;
    int32_t i;

    activeScreen = laContext_GetActive()->activeScreen;
    
    if(activeScreen == NULL)
        return;

    // iterate over all existing layers for update
    for(i = 0; i < LA_MAX_LAYERS; i++)
    {
        layer = activeScreen->layers[i];

        if(layer == NULL)
            continue;
            
        if(layer->frameState <= LA_LAYER_FRAME_PREFRAME)
        {
            modDelta = dt;
            
            if(layer->deltaTime != 0)
            {
                modDelta += layer->deltaTime;
                layer->deltaTime = 0;
            }
            
            _laContext_UpdateWidget((laWidget*)layer, modDelta);
        }
        else
        {
            layer->deltaTime += dt;
        }
    }
}


GFXU_StringTableAsset* laContext_GetStringTable()
{
    if(_activeContext == NULL)
        return NULL;
        
    return _activeContext->stringTable;
}

void laContext_SetStringTable(GFXU_StringTableAsset* table)
{
    if(_activeContext == NULL)
        return;
        
    _activeContext->stringTable = table;    
    
    laContext_RedrawAll();
}

uint32_t laContext_GetStringLanguage()
{
    if(_activeContext == NULL)
        return 0;
        
    return _activeContext->languageID;
}

static void signalLanguageChanging(laWidget* wgt)
{
    uint32_t i;
    laWidget* child;
    
    // signal event
    wgt->languageChangeEvent(wgt);
    
    for(i = 0; i < wgt->children.size; i++)
    {
        child = wgt->children.values[i];

        signalLanguageChanging(child);
    }
}

void laContext_SetStringLanguage(uint32_t id)
{   
    laScreen* activeScreen;
    GFX_Rect screenRect;
    laLayer* layer;
    uint32_t i;
    
    if(_activeContext == NULL)
        return;
        
    activeScreen = laContext_GetActive()->activeScreen;
    
    screenRect = laContext_GetScreenRect();
    
    if(activeScreen == NULL)
        return;
        
    // iterate over all qualifying layers and signal event
    for(i = 0; i < LA_MAX_LAYERS; i++)
    {
        layer = activeScreen->layers[i];

        if(layer == NULL ||
           layer->widget.enabled == LA_FALSE ||
           GFX_RectIntersects(&screenRect, &layer->widget.rect) == LA_FALSE)
            continue;
            
        signalLanguageChanging((laWidget*)layer);
    }
    
    _activeContext->languageID = id;

	// do it again after the change
    for(i = 0; i < LA_MAX_LAYERS; i++)
    {
        layer = activeScreen->layers[i];

        if(layer == NULL ||
           layer->widget.enabled == LA_FALSE ||
           GFX_RectIntersects(&screenRect, &layer->widget.rect) == LA_FALSE)
            continue;
            
        signalLanguageChanging((laWidget*)layer);
    }
}

void laContext_RedrawAll()
{
    laScreen* activeScreen;
    laLayer* layer;
    GFX_Rect screenRect;
    uint32_t i;
    
    if(_activeContext == NULL)
        return;
        
    activeScreen = laContext_GetActive()->activeScreen;
    
    if(activeScreen == NULL)
        return;
        
    screenRect = laContext_GetScreenRect();
        
    // iterate over all qualifying layers and invalidate
    for(i = 0; i < LA_MAX_LAYERS; i++)
    {
        layer = activeScreen->layers[i];

        if(layer == NULL ||
           layer->widget.enabled == LA_FALSE ||
           GFX_RectIntersects(&screenRect, &layer->widget.rect) == LA_FALSE)
            continue;

        laWidget_Invalidate((laWidget*)layer);
    }
}

laScheme* laContext_GetDefaultScheme()
{
    if(_activeContext == NULL)
        return NULL;
        
    return &_activeContext->defaultScheme;
}

laWidget* laContext_GetFocusWidget()
{
    if(_activeContext == NULL)
        return NULL;

    return _activeContext->focus;
}

laResult laContext_SetFocusWidget(laWidget* widget)
{
    if(_activeContext == NULL)
        return LA_FAILURE;

    if(_activeContext->focus == widget)
        return LA_SUCCESS;

    if(_activeContext->focus != NULL)
        _activeContext->focus->focusLost(_activeContext->focus);

    _activeContext->focus = widget;

    if(_activeContext->focus != NULL)
        _activeContext->focus->focusGained(_activeContext->focus);

    return LA_SUCCESS;
}

laEditWidget* laContext_GetEditWidget()
{
    if(_activeContext == NULL)
        return NULL;

    return _activeContext->edit;
}

laResult laContext_SetEditWidget(laWidget* widget)
{
    laEditWidget* edit;
    
    if(_activeContext == NULL)
        return LA_FAILURE;

    if(_activeContext->edit == (laEditWidget*)widget)
        return LA_SUCCESS;

    if(_activeContext->edit != NULL)
        laEditWidget_EndEdit(_activeContext->edit);

    if(widget != NULL && widget->editable == LA_FALSE)
        return LA_FAILURE;
        
    edit = (laEditWidget*)widget;

    _activeContext->edit = (laEditWidget*)widget;

    if(laEditWidget_StartEdit(edit) == LA_FAILURE)
    {
        _activeContext->edit = NULL;
        
        return LA_FAILURE;
    }

    return LA_SUCCESS;
}

laEventResult _laContext_HandleScreenChangeEvent(laEvent* evt)
{
    laContext_ScreenChangeEvent scrEvt = *(laContext_ScreenChangeEvent*)evt;
    
    // don't process any more events for the active screen
    laList_Destroy(&_activeContext->event.events);
    
    if(_activeContext->activeScreen != NULL)
    {
        laContext_SetFocusWidget(NULL);
        laContext_SetEditWidget(NULL);
        
        laScreen_Hide(_activeContext->activeScreen);
    }
    
    laScreen_Show(_activeContext->screenList.values[scrEvt.index]);
    
    // failure here means the event handler won't try to delete the event
    // the event has already been deleted
    return LA_EVENT_RESET_QUEUE; 
}

laBool laContext_IsDrawing()
{
    uint32_t idx;
    
    if(_activeContext == NULL || _activeContext->activeScreen == NULL)
        return LA_FALSE;
        
    for(idx = 0; idx < LA_MAX_LAYERS; idx++)
    {
        if(laLayer_IsDrawing(_activeContext->activeScreen->layers[idx]))
            return LA_TRUE;
    }
    
    return LA_FALSE;
}

laBool laContext_IsLayerDrawing(uint32_t idx)
{
    if(_activeContext == NULL ||
       _activeContext->activeScreen == NULL ||
       idx >= LA_MAX_LAYERS)
        return LA_FALSE;
        
    return laLayer_IsDrawing(_activeContext->activeScreen->layers[idx]);
}