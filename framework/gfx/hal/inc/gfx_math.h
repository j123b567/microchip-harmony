/*******************************************************************************
 Module for Microchip Graphics Library - Hardware Abstraction Layer

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_layer.h

  Summary:
    Contains some general purpose math functions

  Description:
    Math support functions.
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

#ifndef GFX_MATH_H
#define GFX_MATH_H
//DOM-IGNORE-END

#include "gfx/hal/inc/gfx_common.h"

#if GFX_DRAW_PIPELINE_ENABLED

// *****************************************************************************
/* Function:
    int32_t GFX_Mini(int32_t l, int32_t r);

  Summary:
    Returns the smaller of two integers.
    
  Parameters:
    l - the first number to test
    r - the second number to test
  
  Returns:
    int32_t - the smaller of the two numbers
*/
LIB_EXPORT int32_t GFX_Mini(int32_t l, int32_t r);

// *****************************************************************************
/* Function:
    int32_t GFX_Maxi(int32_t l, int32_t r);

  Summary:
    Returns the larger of two integers.
    
  Parameters:
    l - the first number to test
    r - the second number to test
  
  Returns:
    int32_t - the larger of the two numbers
*/
LIB_EXPORT int32_t GFX_Maxi(int32_t l, int32_t r);

// *****************************************************************************
/* Function:
    float GFX_Minf(float l, float r);

  Summary:
    Returns the smaller of two floats.
    
  Parameters:
    l - the first float to test
    r - the second float to test
  
  Returns:
    float - the smaller of the two floats
*/
LIB_EXPORT float GFX_Minf(float l, float r);

// *****************************************************************************
/* Function:
    float GFX_Maxf(float l, float r);

  Summary:
    Returns the larger of two floats.
    
  Parameters:
    l - the first float to test
    r - the second float to test
  
  Returns:
    float - the larger of the two floats
*/
LIB_EXPORT float GFX_Maxf(float l, float r);

// *****************************************************************************
/* Function:
    int32_t GFX_Clampi(int32_t min, int32_t max, int32_t i);

  Summary:
    Clamps an integer between a min and max
    
  Parameters:
    min - the minimum value
    max - the maximum value
    i - the number to clamp
  
  Returns:
    int32_t - the clamped value
*/
LIB_EXPORT int32_t GFX_Clampi(int32_t min, int32_t max, int32_t i);

// *****************************************************************************
/* Function:
    float GFX_Clampi(float min, float max, float i);

  Summary:
    Clamps a float between a min and max
    
  Parameters:
    min - the minimum value
    max - the maximum value
    i - the float to clamp
  
  Returns:
    float - the clamped value
*/
LIB_EXPORT float GFX_Clampf(float min, float max, float f);

// *****************************************************************************
/* Function:
    uint32_t GFX_Percent(uint32_t l, uint32_t r);

  Summary:
    Calculates the percentage of one number when applied to another.  Integer
    based.  Accuracy for higher numbers is not guaranteed.   
    
    The result is the decimal percentage multiplied by 100.
        
  Parameters:
    l - the first number of the equation
    r - the second number of the equation
  
  Returns:
    uint32_t - the percentage represented as a whole number
*/
LIB_EXPORT uint32_t GFX_Percent(uint32_t l, uint32_t r);

// *****************************************************************************
/* Function:
    uint32_t GFX_PercentWholeRounded(uint32_t l, uint32_t r);

  Summary:
    Calculates the percentage of one number when applied to another.  Integer
    based.  Accuracy for higher numbers is not guaranteed.   
    
    The difference between this and GFX_Percent is that the decimal portion
    of the whole number is rounded off.
        
  Parameters:
    l - the first number of the equation
    r - the second number of the equation
  
  Returns:
    uint32_t - the percentage represented as a whole number
*/
LIB_EXPORT uint32_t GFX_PercentWholeRounded(uint32_t l, uint32_t r);

// *****************************************************************************
/* Function:
    uint32_t GFX_PercentOf(uint32_t l, uint32_t percent);

  Summary:
    Calculates the percentage of a number.  Returns a whole number with no
    decimal component.
        
  Parameters:
    num - the number to consider
    percent - the percentage to apply
  
  Returns:
    uint32_t - the resultant percentage of the number 
*/
LIB_EXPORT uint32_t GFX_PercentOf(uint32_t num, uint32_t percent);

LIB_EXPORT void GFX_PercentOfDec(uint32_t num, uint32_t percent, uint32_t* whl, uint32_t* dec);

// *****************************************************************************
/* Function:
    uint32_t GFX_ScaleInteger(uint32_t num, uint32_t oldMax, uint32_t newMax);

  Summary:
    Scales an integer from one number range of 0 -> n0 to another range 0 -> n1 
    based on percentages.
        
  Parameters:
    num - the number to consider
    oldMax - the old range maximum
    newMax - the new range maximum
  
  Returns:
    uint32_t - the number as defined in the new number range
*/
LIB_EXPORT uint32_t GFX_ScaleInteger(uint32_t num, uint32_t oldMax, uint32_t newMax);

// *****************************************************************************
/* Function:
    uint32_t GFX_AbsoluteValue(int32_t val);

  Summary:
    Calculates the absolute value of a signed integer.
        
  Parameters:
    val - the number to consider
  
  Returns:
    uint32_t - the absolute value
*/
LIB_EXPORT uint32_t GFX_AbsoluteValue(int32_t val);

// *****************************************************************************
/* Function:
    int32_t GFX_Lerp(int32_t x, int32_t y, uint32_t per);

  Summary:
    Performs a linear interpolation of an integer based on a percentage between
    two signed points.
        
  Parameters:
    x - the first point to consider
    y - the second point to consider
    per - the percentage of interpolation
  
  Returns:
    int32_t - the interpolated value
*/
LIB_EXPORT int32_t GFX_Lerp(int32_t x, int32_t y, uint32_t per);

LIB_EXPORT int32_t GFX_DivideRounding(int32_t num, int32_t denom);

#endif // GFX_DRAW_PIPELINE_ENABLED
#endif /* GFX_MATH_H */