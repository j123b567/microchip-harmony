/*******************************************************************************
 Module for Microchip Graphics Library - Aria User Interface Library

  Company:
    Microchip Technology Inc.

  File Name:
    libaria_layer.h

  Summary:
    Aria layers map directly to layers provided by the Graphics Hardware
    Abstraction layer.  HAL layers map directly to hardware layers provided
    by graphics hardware.  UI layers are logical containers for widgets
    and provide many of the same features.
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

#ifndef LIBARIA_LAYER_H
#define LIBARIA_LAYER_H
//DOM-IGNORE-END

#include "gfx/libaria/inc/libaria_common.h"
#include "gfx/libaria/inc/libaria_widget.h"
#include "gfx/libaria/inc/libaria_rectarray.h"

typedef struct laScreen_t laScreen;

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Enumeration:
    laLayerBufferType_t

  Summary:
    Defines the type of a layer.  If the layer has an explicit address then
    Aria tries to set that through the HAL when the layer is being set up.

  Remarks:
    None.
*/
typedef enum laLayerBufferType_t
{
    LA_BUFFER_TYPE_AUTO,
    LA_BUFFER_TYPE_ADDRESS
} laLayerBufferType;

// *****************************************************************************
/* Structure:
    laLayerBuffer_t

  Summary:
    Structure to maintain the buffer type and track the buffer location for each layer

  Description:
    Structure to maintain the buffer type and track the buffer location for each layer

  Remarks:
    None.
*/
typedef struct laLayerBuffer_t
{
    laLayerBufferType type;
    void* address;
} laLayerBuffer;

// *****************************************************************************
/* Enumeration:
    laLayerFrameState

  Summary:
    Defines the frame state of a layer.  Certain actions must only be performed
    at the start of a new frame and other actions must wait until the end of
    the current frame.

  Remarks:
    None.
*/
typedef enum laLayerFrameState_t
{
    LA_LAYER_FRAME_READY,
    LA_LAYER_FRAME_PREFRAME,
    LA_LAYER_FRAME_IN_PROGRESS,
    LA_LAYER_FRAME_COMPLETE
} laLayerFrameState;

// *****************************************************************************
/* Structure:
    laLayer_t

  Summary:
    Primary definition of a layer.  Builds on base functions of a standard
    widget.  Should never have a direct parent.

  Remarks:
    None.
*/
typedef struct laLayer_t
{
    laWidget widget; // base widget
    laScreen* screen; // owning screen pointer
    
    laBool deleting; // flag indicating that no changes should be made
                     // to the layer because it is in the process of
                     // being deleted

    uint32_t bufferCount; // number of buffers in the layer
    laLayerBuffer buffers[GFX_MAX_BUFFER_COUNT]; // buffer array
    
    laBool alphaEnable; // layer-based alpha blending enable flag
  
    laBool maskEnable;  // layer-based color masking enable flag
    GFX_Color maskColor; // layer-based masking color value

    laBool vsync; // layer vsync flag
    
    laRectArray prevDamageRects; // previous damaged rectangle list
    laRectArray frameDamageRects; // queued damaged rectangle list
    laRectArray pendingDamageRects; // pending damaged rectangle list
                                    // these are rectangles added during
                                    // a frame in progress
    
    GFX_Rect currentDrawingRect; // the current damage rectangle
    GFX_Rect clippedDrawingRect; // the current damage rectangle clipped
                                 // to the currently rendering widget
                                 
    laBool drawingPrev;          // indicates if the layer is currently
                                 // drawing from its previous rectangle
                                 // array

    laLayerFrameState frameState; // the current frame render state of the
                                  // layer
                                  
    uint32_t frameDrawCount;      // the number of widgets that have rendered
                                  // on this layer this frame
    
    laBool allowInputPassThrough; // indicates that input events should
                                  // be propagated through the layer
                                  // node to left siblings
    
    uint32_t deltaTime;           // stores delta time for updates that happen
                                  // during rendering

} laLayer;

// *****************************************************************************
// *****************************************************************************
// Section: Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    laLayer* laLayer_New()

  Summary:
    Constructor for a new layer    

  Description:
    Constructor for a new layer, returns the layer object

  Parameters:
    void
    
  Returns:
    laLayer*
    
  Remarks:
    Allocates memory for a layer using the active context memory interface.
    Once added to a screen the it becomes the responsibility of the framework
    to free the memory.
*/
LIB_EXPORT laLayer* laLayer_New();

// *****************************************************************************
/* Function:
    void laLayer_Delete(laLayer* layer)

  Summary:
    Destructor for the layer object    

  Description:
    Destructor for the layer object

  Parameters:
    laLayer*
    
  Returns:
    void
    
  Remarks:
    
*/
LIB_EXPORT void laLayer_Delete(laLayer* layer);

// internal functions
LIB_EXPORT void _laLayer_Moved(laLayer* layer);
LIB_EXPORT void _laLayer_Resized(laLayer* layer);

// *****************************************************************************
/* Function:
    laBool laLayer_GetEnabled(const laLayer* layer)
    
  Summary:
    Returns the boolean value of the layer enabled property

  Description:
    Returns the boolean value of the layer enabled property

  Parameters:
    laLayer* - the layer
    
    
  Returns:
    laBool - the flag value
    
  Remarks:
    
*/
LIB_EXPORT laBool laLayer_GetEnabled(const laLayer* layer);

// *****************************************************************************
/* Function:
    laResult laLayer_SetEnabled(laLayer* widget, laBool enable)
    
  Summary:
    Sets the boolean value of the layer enabled property

  Description:
    Sets the boolean value of the layer enabled property

  Parameters:
    laLayer* - the layer
    laBool - the desired enabled value
    
  Returns:
    laResult - the result of the operation
    
  Remarks:
    The enabled flag for a layer will often control the hardware setting for
    layer usage, depending on the display driver
*/
LIB_EXPORT laResult laLayer_SetEnabled(laLayer* widget, laBool enable);

// *****************************************************************************
/* Function:
    laBool laLayer_GetAlphaEnable(const laLayer* layer)

  Summary:
    Gets the layer alpha enable flag

  Parameters:
    const laLayer* - the layer
    
  Returns:
    laBool - the value of the alpha enable flag
*/
LIB_EXPORT laBool laLayer_GetAlphaEnable(const laLayer* layer);

// *****************************************************************************
/* Function:
    laResult laLayer_SetAlphaEnable(laLayer* layer, laBool enable)

  Summary:
    Sets the layer alpha enable flag to the specified value

  Parameters:
    laLayer* layer - the layer
    laBool enable - the desired value of the flag
    
  Returns:
    laResult - the result of the operation
    
  Remarks:
    
*/
LIB_EXPORT laResult laLayer_SetAlphaEnable(laLayer* layer, laBool enable);

// *****************************************************************************
/* Function:
    uint32_t laLayer_GetAlphaAmount(const laLayer* layer)

  Summary:
    Get's the amount of alpha blending for a given layer

  Parameters:
    laLayer* layer - the layer

  Returns:
    uint32_t - an alpha channel value from 0 - 255
*/
LIB_EXPORT uint32_t laLayer_GetAlphaAmount(const laLayer* layer);

// *****************************************************************************
/* Function:
    laResult laLayer_SetAlphaAmount(laLayer* layer, uint32_t amount)

  Summary:
    Set's the amount of alpha blending for a given layer

  Description:
    Set's the amount of alpha blending for a given layer

  Parameters:
    laLayer* layer - the layer
    uint32_t amount - an alpha amount from 0 - 255
    
  Returns:
    laResult - success if the operation succeeded
    
  Remarks:
    
*/
LIB_EXPORT laResult laLayer_SetAlphaAmount(laLayer* layer, uint32_t amount);

// *****************************************************************************
/* Function:
    laBool laLayer_GetMaskEnable(const laLayer* layer)

  Summary:
    Gets the layer mask enable flag

  Description:
    Gets the layer mask enable flag

  Parameters:
    laLayer* layer - the layer
    
  Returns:
    laBool - the value of the mask enable flag
    
  Remarks:
    
*/
LIB_EXPORT laBool laLayer_GetMaskEnable(const laLayer* layer);

// *****************************************************************************
/* Function:
    laResult laLayer_SetMaskEnable(laLayer* layer, laBool enable)

  Summary:
    Sets the layer mask enable flag to the specified value

  Parameters:
    laLayer* layer - the layer
    laBool enable - the desired value of the flag
    
  Returns:
    laResult - the result of the operation
    
  Remarks:
    
*/
LIB_EXPORT laResult laLayer_SetMaskEnable(laLayer* layer, laBool enable);

// *****************************************************************************
/* Function:
    GFX_Color laLayer_GetMaskColor(const laLayer* layer)

  Summary:
    Returns the mask color value for the current layer 

  Description:
    Returns the mask color value for the current layer 

  Parameters:
    laLayer* layer - the layer
    
  Returns:
    GFX_Color - the layer mask color value
    
  Remarks:
    
*/
LIB_EXPORT GFX_Color laLayer_GetMaskColor(const laLayer* layer);

// *****************************************************************************
/* Function:
    void laLayer_SetMaskColor(laLayer* layer, GFX_Color color)

  Summary:
    Set the mask color value for the current layer to the specified value

  Description:
    Set the mask color value for the current layer to the specified value 

  Parameters:
    laLayer* layer - the layer
    GFX_color color - the desired mask color value
    
  Returns:
    laResult - the result of the operation
    
  Remarks:
    
*/
LIB_EXPORT laResult laLayer_SetMaskColor(laLayer* layer, GFX_Color color);

// *****************************************************************************
/* Function:
    uint32_t laLayer_GetBufferCount(const laLayer* layer)

  Summary:
    Return the buffer count for the current layer

  Description:
    Return the buffer count for the current layer

  Parameters:
    laLayer* layer - the layer

  Returns:
    uint32_t - the current number of buffers for the layer
    
  Remarks:
    
*/
LIB_EXPORT uint32_t laLayer_GetBufferCount(const laLayer* layer);

// *****************************************************************************
/* Function:
    laResult laLayer_SetBufferCount(laLayer* layer, uint32_t count)

  Summary:
    Set the buffer count for the current layer to the specified value

  Description:
    Set the buffer count for the current layer to the specified value

  Parameters:
    laLayer* layer - the layer
    uint32_t count - the desired number of buffers

  Returns:
    laResult - the result of the operation
    
  Remarks:
    
*/
LIB_EXPORT laResult laLayer_SetBufferCount(laLayer* layer, uint32_t count);

// *****************************************************************************
/* Function:
    laBool laLayer_GetVSync(const laLayer* layer)

  Summary:
    Gets the layer's vsync flag setting

  Description:
    

  Parameters:
    const laLayer* layer - the layer
    
  Returns:
    laBool - the state of the layer's vsync flag
    
  Remarks:
    
*/                                             
LIB_EXPORT laBool laLayer_GetVSync(const laLayer* layer);

// *****************************************************************************
/* Function:
    void laLayer_SetVSync(laLayer* layer, laBool enable)

  Summary:
    Sets the layer's vsync flag.

  Description:
    Sets the layer's vsync flag.

  Parameters:
    const laLayer* layer - the layer
    
  Returns:
    laResult - the result of the operation
    
  Remarks:
    
*/    
LIB_EXPORT laResult laLayer_SetVSync(laLayer* layer, laBool enable);

// *****************************************************************************
/* Function:
    laBool laLayer_GetAllowInputPassThrough(const laLayer* layer)

  Summary:
    Gets the layer's input passthrough setting

  Description:
    The input passthrough setting is used to prohibit or allow input events
    to pass through a layer.  If a layer is opaque or semi-opaque input events
    should probably not be allowed to pass through.  If the layer is completely
    transparent then input events may be allowed to pass through to interact
    with widgets on layers further back in the hierarchy.

  Parameters:
    const laLayer* layer - the layer
    
  Returns:
    laBool - the state of the layer's passthrough flag
    
  Remarks:
    
*/                                             
LIB_EXPORT laBool laLayer_GetAllowInputPassThrough(const laLayer* layer);

// *****************************************************************************
/* Function:
    void laLayer_SetAllowInputPassthrough(laLayer* layer, laBool enable)

  Summary:
    Sets the layer's input passthrough flag.

  Description:
    Sets the layer's input passthrough flag.  

  Parameters:
    const laLayer* layer - the layer
    
  Returns:
    laResult - the result of the operation
    
  Remarks:
    
*/    
LIB_EXPORT laResult laLayer_SetAllowInputPassthrough(laLayer* layer, laBool enable);

// *****************************************************************************
/* Function:
    laResult laLayer_AddDamageRect(laLayer* layer, const GFX_Rect* rect)

   Summary:
    Adds a damaged rectangle to the list.  Damage rectangles are used in minimal
    redraw algorithms.

   Precondition:

   Parameters:
     laLayer* layer - the layer
     const GFX_Rect* rect - the rectangle
    
  Returns:
    laResult - the result of the operation

  Remarks:    
    
*/
LIB_EXPORT laResult laLayer_AddDamageRect(laLayer* layer,
                                          const GFX_Rect* rect,
										  laBool noCombine);
										  
// *****************************************************************************
/* Function:
    laBool laLayer_IsDrawing(laLayer* layer)

   Summary:
    Queries a layer to find out if it is currently drawing a frame.

   Precondition:

   Parameters:
     laLayer* layer - the layer
         
  Returns:
    laBool - the result of the operation

  Remarks:    
    
*/
LIB_EXPORT laBool laLayer_IsDrawing(laLayer* layer);										  
										  

// these are for internal use only
void _laLayer_Preframe(laLayer* layer);
void _laLayer_Postframe(laLayer* layer);

void _laLayer_InvalidateChildren(laLayer* layer);

void _laLayer_Paint(laLayer* layer);

void _laLayer_RedrawPartial(laLayer* layer);
void _laLayer_SwapLists(laLayer* layer);

void _laLayer_AddDamageRectToList(laRectArray* arr,
								  const GFX_Rect* rect,
								  laBool noCombine);

void _laLayer_MergeRectLists(laLayer* layer);

#endif /* LIBARIA_LAYER_H */