/*******************************************************************************
 Module for Microchip Graphics Library - Aria User Interface Library

  Company:
    Microchip Technology Inc.

  File Name:
    libaria_widget_slider.h

  Summary:
    

  Description:
    This module implements slider control widget functions.
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
#ifndef LIBARIA_SLIDER_H
#define LIBARIA_SLIDER_H
//DOM-IGNORE-END

#include "gfx/libaria/inc/libaria_common.h"

#if LA_SLIDER_WIDGET_ENABLED

#include "gfx/libaria/inc/libaria_widget.h"
#include "gfx/libaria/inc/libaria_string.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Enumeration:
    laSliderState_t

  Summary:
    Describes various slider states

  Description:
    

  Remarks:
    None.
*/
typedef enum laSliderState_t
{
    LA_SLIDER_STATE_NONE,
    LA_SLIDER_STATE_HANDLE_DOWN,
    LA_SLIDER_STATE_AREA_DOWN
} laSliderState;

// *****************************************************************************
/* Enumeration:
    laSliderOrientation_t

  Summary:
    Slider orientations

  Description:
    

  Remarks:
    None.
*/
typedef enum laSliderOrientation_t
{
    LA_SLIDER_ORIENT_VERTICAL,
    LA_SLIDER_ORIENT_HORIZONTAL
} laSliderOrientation;

typedef struct laSliderWidget_t laSliderWidget;

typedef void (*laSliderWidget_ValueChangedEvent)(laSliderWidget*);

// *****************************************************************************
/* Structure:
    laSliderWidget_t

  Summary:
    Implementation of a slider widget struct

  Description:
    A slider bar is a widget that is capable of displaying a range and a slider
    handle.  The slider can be moved between two discreet values and can have
    a variable min and max range.

  Remarks:
    None.
*/
typedef struct laSliderWidget_t
{
    laWidget widget; // widget base class

    laSliderState state; // slider state
    laSliderOrientation alignment; // slider alignment
    
    int32_t min; // slider min value
    int32_t max; // slider max value
    int32_t value; // slider current value
    uint32_t grip; // slider grip size
    
    laSliderWidget_ValueChangedEvent valueChangedEvent; // value changed event
    
    GFX_Point handleDownOffset;
} laSliderWidget;

void _laSliderWidget_Constructor(laSliderWidget* sld);
void _laSliderWidget_Destructor(laSliderWidget* sld);

void _laSliderWidget_Update(laSliderWidget* sld);
void _laSliderWidget_Paint(laSliderWidget* sld);

// *****************************************************************************
// *****************************************************************************
// Section: Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    laSliderWidget* laSliderWidget_New()

  Summary:
    Allocates memory for a new widget of this type.  The application is
    responsible for the managment of this memory until the widget is added to
    a widget tree.

  Description:
    

  Parameters:
    
  Returns:
    laSliderWidget*
    
  Remarks:
    
*/
LIB_EXPORT laSliderWidget* laSliderWidget_New();

// *****************************************************************************
/* Function:
    laSliderOrientation laSliderWidget_GetOrientation(laSliderWidget* sld)

  Summary:
    Gets the orientation value for the slider

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    
  Returns:
    laSliderOrientation
    
  Remarks:
    
*/
LIB_EXPORT laSliderOrientation laSliderWidget_GetOrientation(laSliderWidget* sld);

// *****************************************************************************
/* Function:
    laResult laSliderWidget_SetOrientation(laSliderWidget* sld,
                                           laSliderOrientation align,
                                           laBool swapDimensions)

  Summary:
    

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    laSliderOrientation - the desired slider orientation
    laBool - indicates if the width and height of the slider should be swapped
    
  Returns:
    laResult - the operation result
    
  Remarks:
    
*/
LIB_EXPORT laResult laSliderWidget_SetOrientation(laSliderWidget* sld,
                                                  laSliderOrientation align,
                                                  laBool swapDimensions);

// *****************************************************************************
/* Function:
    uint32_t laSliderWidget_GetGripSize(laSliderWidget* sld)

  Summary:
    Gets the current grip size of the slider

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    
  Returns:
    uint32_t - the current grip size
    
  Remarks:
    
*/                                                  
LIB_EXPORT uint32_t laSliderWidget_GetGripSize(laSliderWidget* sld);

// *****************************************************************************
/* Function:
    laResult laSliderWidget_SetGripSize(laSliderWidget* sld,
                                        uint32_t size)

  Summary:
    Sets the grip size of the slider

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    uint32_t size - the desired grip size
    
  Returns:
    laResult - the operation result
    
  Remarks:
    
*/
LIB_EXPORT laResult laSliderWidget_SetGripSize(laSliderWidget* sld,
                                               uint32_t size);

// *****************************************************************************
/* Function:
    uint32_t laSliderWidget_GetMininumValue(laSliderWidget* sld)

  Summary:
    Gets the minimum value for the slider

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
        
  Returns:
    uint32_t - the minimum slider value
    
  Remarks:
    
*/                                               
LIB_EXPORT uint32_t laSliderWidget_GetMininumValue(laSliderWidget* sld);

// *****************************************************************************
/* Function:
    laResult laSliderWidget_SetMinimumValue(laSliderWidget* sld,
                                            uint32_t val)

  Summary:
    Sets the minimum value for the slider

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    uint32_t val - the desired minimum value
    
  Returns:
    laResult - the operation result
    
  Remarks:
    
*/
LIB_EXPORT laResult laSliderWidget_SetMinimumValue(laSliderWidget* sld,
                                                      uint32_t val);

// *****************************************************************************
/* Function:
    uint32_t laSliderWidget_GetMaxinumValue(laSliderWidget* sld)

  Summary:
    Gets the maximum value for the slider

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
        
  Returns:
    uint32_t - the maximum value for the slider
    
  Remarks:
    
*/                                                      
LIB_EXPORT uint32_t laSliderWidget_GetMaxinumValue(laSliderWidget* sld);

// *****************************************************************************
/* Function:
    laResult laSliderWidget_SetMaximumValue(laSliderWidget* sld,
                                            uint32_t val)

  Summary:
    Sets the maximum value for the slider

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    uint32_t val - the desired maximum value for the slider
    
  Returns:
    laResult - the operation result
    
  Remarks:
    
*/
LIB_EXPORT laResult laSliderWidget_SetMaximumValue(laSliderWidget* sld,
                                                      uint32_t val);                                                     

// *****************************************************************************
/* Function:
    uint32_t laSliderWidget_GetSliderValue(laSliderWidget* sld)

  Summary:
    Gets the current slider value

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
        
  Returns:
    uint32_t - the current slider value
    
  Remarks:
    
*/                                                      
LIB_EXPORT int32_t laSliderWidget_GetSliderValue(laSliderWidget* sld);

// *****************************************************************************
/* Function:
    laResult laSliderWidget_SetSliderValue(laSliderWidget* sld,
                                           int32_t val)

  Summary:
    Sets the current slider value

  Description:
    Must be between slider min and max

  Parameters:
    laSliderWidget* sld - the widget
    uint32_t val - the desired slider value
    
  Returns:
    laResult - the operation result
    
  Remarks:
    
*/
LIB_EXPORT laResult laSliderWidget_SetSliderValue(laSliderWidget* sld,
                                                  int32_t val);         

// *****************************************************************************
/* Function:
    uint32_t laSliderWidget_GetSliderPercentage(laSliderWidget* sld)

  Summary:
    Gets the slider value as a percentage

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
        
  Returns:
    uint32_t - the slider value as a percentage
    
  Remarks:
    
*/                                                  
LIB_EXPORT uint32_t laSliderWidget_GetSliderPercentage(laSliderWidget* sld);

// *****************************************************************************
/* Function:
    laResult laSliderWidget_SetSliderPercentage(laSliderWidget* sld,
                                                uint32_t val)

  Summary:
    Sets the slider value using a percentage.  Value must be from 0 - 100.

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    uint32_t val - a percentage value from 0 - 100
    
  Returns:
    laResult - the operation result
    
  Remarks:
    
*/
LIB_EXPORT laResult laSliderWidget_SetSliderPercentage(laSliderWidget* sld,
                                                       uint32_t val);                                                                                                 

// *****************************************************************************
/* Function:
    laResult laSliderWidget_Step(laSliderWidget* sld, int32_t amount)

  Summary:
    Moves the slider by a given amount

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    int32_t amount - the amount by which to adjust the current slider value
    
  Returns:
    laResult - the operation result
    
  Remarks:
    
*/                                                       
LIB_EXPORT laResult laSliderWidget_Step(laSliderWidget* sld, int32_t amount);

// *****************************************************************************
/* Function:
    laSliderWidget_ValueChangedEvent laSliderWidget_GetValueChangedEventCallback(laSliderWidget* sld)

  Summary:
    Gets the current value changed event callback pointer

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
        
  Returns:
    laSliderWidget_ValueChangedEvent - a valid callback or NULL
    
  Remarks:
    
*/                                              
LIB_EXPORT laSliderWidget_ValueChangedEvent laSliderWidget_GetValueChangedEventCallback(laSliderWidget* sld);

// *****************************************************************************
/* Function:
    laResult laSliderWidget_SetValueChangedEventCallback(laSliderWidget* sld,
                                                         laSliderWidget_ValueChangedEvent cb)

  Summary:
    Sets the value changed event callback pointer

  Description:
    

  Parameters:
    laSliderWidget* sld - the widget
    laSliderWidget_ValueChangedEvent - a valid pointer or NULL
    
  Returns:
    laResult - the operation result
    
  Remarks:
    
*/
LIB_EXPORT laResult laSliderWidget_SetValueChangedEventCallback(laSliderWidget* sld,
                                                                laSliderWidget_ValueChangedEvent cb);

// internal use only
void _laSliderWidget_TouchDownEvent(laSliderWidget* sld, laInput_TouchDownEvent* evt);
void _laSliderWidget_TouchUpEvent(laSliderWidget* sld, laInput_TouchUpEvent* evt);
void _laSliderWidget_TouchMovedEvent(laSliderWidget* sld, laInput_TouchMovedEvent* evt);

void _laSliderWidget_GetSlideAreaRect(laSliderWidget* sld, GFX_Rect* rect);
void _laSliderWidget_GetHandleRect(laSliderWidget* sld, GFX_Rect* rect);

uint32_t _laSliderWidget_GetPercentFromPoint(laSliderWidget* sld, GFX_Point* pnt);
uint32_t _laSliderWidget_GetValueFromPercent(laSliderWidget* sld, uint32_t per);

void _laSliderWidget_InvalidateBorderAreas(laSliderWidget* sld);

#endif // LA_SLIDER_WIDGET_ENABLED
#endif /* LIBARIA_SLIDER_H */