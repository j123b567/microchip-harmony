#include "gfx/libaria/inc/libaria_rectarray.h"

#include "gfx/libaria/inc/libaria_context.h"

#define STD_RESIZE_AMT   5

static void _shuffleRight(laRectArray* arr, uint32_t idx)
{
    uint32_t i;

    if(arr->size == 0 || idx == arr->capacity)
        return;

    if(arr->size < arr->capacity)
        arr->size++;

    for(i = arr->size - 1; i >= idx; i++)
        arr->rects[i+1] = arr->rects[i];

    arr->rects[i].x = 0;
    arr->rects[i].y = 0;
    arr->rects[i].width = 0;
    arr->rects[i].height = 0;
}

static void _shuffleLeft(laRectArray* arr, uint32_t idx)
{
    uint32_t i;

    if(arr->size == 0 || idx == arr->capacity)
        return;

    for(i = idx; i < arr->size; i++)
        arr->rects[i] = arr->rects[i+1];

    i--;

    arr->rects[i].x = 0;
    arr->rects[i].y = 0;
    arr->rects[i].width = 0;
    arr->rects[i].height = 0;

    arr->size--;
}

laResult laRectArray_Create(laRectArray* arr)
{
    if(arr == NULL)
        return LA_FAILURE;

    arr->rects = NULL;
    arr->size = 0;
    arr->capacity = 0;

    return LA_SUCCESS;
}

laResult laRectArray_Resize(laRectArray* arr, uint32_t sz)
{
    if(laContext_GetActive() == NULL || arr == NULL || arr->capacity == sz)
        return LA_FAILURE;

    arr->rects = laContext_GetActive()->memIntf.heap.realloc(arr->rects, 
                                                             sizeof(GFX_Rect) * sz);

    if(arr->rects == NULL)
    {
        arr->size = 0;
        arr->capacity = 0;

        return LA_FAILURE;
    }

    arr->capacity = sz;

    if(arr->size >= arr->capacity)
        arr->size = arr->capacity;

    return LA_SUCCESS;
}

laResult laRectArray_PushFront(laRectArray* arr, const GFX_Rect* rect)
{
    if(laContext_GetActive() == NULL ||
       arr == NULL ||
       arr->size == 0 ||
       rect == NULL)
        return LA_FAILURE;

    if(arr->size == arr->capacity)
    {
        if(laRectArray_Resize(arr, arr->capacity + STD_RESIZE_AMT) == LA_FAILURE)
            return LA_FAILURE;
    }

    _shuffleRight(arr, 0);

    arr->rects[0] = *rect;

    return LA_SUCCESS;
}

laResult laRectArray_PopFront(laRectArray* arr)
{
    if(arr == NULL || arr->size == 0)
        return LA_FAILURE;

    _shuffleLeft(arr, 0);

    return LA_SUCCESS;
}

laResult laRectArray_PushBack(laRectArray* arr, const GFX_Rect* rect)
{
    if(laContext_GetActive() == NULL ||
       arr == NULL ||
       rect == NULL)
        return LA_FAILURE;

    if(arr->size == arr->capacity)
    {
        if(laRectArray_Resize(arr, arr->capacity + STD_RESIZE_AMT) == LA_FAILURE)
            return LA_FAILURE;
    }

    arr->rects[arr->size] = *rect;
    arr->size++;

    return LA_SUCCESS;
}

laResult laRectArray_PopBack(laRectArray* arr)
{
    if(arr == NULL || arr->size == 0)
        return LA_FAILURE;

    arr->size--;

    return LA_SUCCESS;
}

laResult laRectArray_InsertAt(laRectArray* arr,
                              uint32_t idx,
                              const GFX_Rect* rect)
{
    if(laContext_GetActive() == NULL ||
       arr == NULL ||
       idx > arr->size ||
       rect == NULL)
        return LA_FAILURE;

    if(idx == arr->size)
        return laRectArray_PushBack(arr, rect);

    if(arr->size == arr->capacity)
    {
        if(laRectArray_Resize(arr, arr->capacity + STD_RESIZE_AMT) == LA_FAILURE)
            return LA_FAILURE;
    }

    _shuffleRight(arr, idx);

    arr->rects[idx] = *rect;

    return LA_SUCCESS;
}

laResult laRectArray_RemoveAt(laRectArray* arr, uint32_t idx)
{
    if(laContext_GetActive() == NULL ||
       arr == NULL ||
       arr->size == 0 ||
       idx >= arr->size)
        return LA_FAILURE;

    _shuffleLeft(arr, idx);

    return LA_SUCCESS;
}

laResult laRectArray_Copy(laRectArray* src, laRectArray* dest)
{
    if(laContext_GetActive() == NULL || src == NULL || dest == NULL)
        return LA_FAILURE;

    if(dest->capacity < src->size)
    {
        if(laRectArray_Resize(dest, src->size) == LA_FAILURE)
            return LA_FAILURE;
    }

    dest->size = src->size;

    if(dest->size == 0)
        return LA_SUCCESS;

    laContext_GetActive()->memIntf.heap.memcpy(dest->rects,
                                               src->rects,
                                               src->size * sizeof(GFX_Rect));
    
    return LA_SUCCESS;
}

laResult laRectArray_Clear(laRectArray* arr)
{
    if(arr == NULL)
        return LA_FAILURE;

    arr->size = 0;

    return LA_SUCCESS;
}

laResult laRectArray_Destroy(laRectArray* arr)
{
    if(laContext_GetActive() == NULL || arr == NULL)
        return LA_FAILURE;
        
    if(arr->rects != NULL)
    {
        laContext_GetActive()->memIntf.heap.free(arr->rects);
        arr->rects = NULL;
    }
    
    arr->capacity = 0;
    arr->size = 0;

    return LA_SUCCESS;
}

