#include "gfx/hal/inc/gfx_math.h"

#if GFX_DRAW_PIPELINE_ENABLED

int32_t GFX_Mini(int32_t l, int32_t r)
{
    return l < r ? l : r;
}

int32_t GFX_Maxi(int32_t l, int32_t r)
{
    return l > r ? l : r;
}

float GFX_Minf(float l, float r)
{
    return l < r ? l : r;
}

float GFX_Maxf(float l, float r)
{
    return l > r ? l : r;
}

int32_t GFX_Clampi(int32_t min, int32_t max, int32_t i)
{
    if(i < min)
        return min;
    else if(i > max)
        return max;
        
    return i;
}

float GFX_Clampf(float min, float max, float f)
{
    if(f < min)
        return min;
    else if(f > max)
        return max;
        
    return f;
}

uint32_t GFX_Percent(uint32_t l, uint32_t r)
{
    return (l * 10000) / r;
}

uint32_t GFX_PercentWholeRounded(uint32_t l, uint32_t r)
{
    return (((l * 10000) / r) + 50) / 100;
}

uint32_t GFX_PercentOf(uint32_t num, uint32_t percent)
{
    int whl = ((num * 100) * percent) / 10000;
    int dec = (((num * 1000) * percent) / 10000) % 10;
    
    if(dec >= 5)
        whl++;
    
    return whl;
}

void GFX_PercentOfDec(uint32_t num, uint32_t percent, uint32_t* whl, uint32_t* dec)
{
    *whl = ((num * 100) * percent) / 10000;
    *dec = (((num * 1000) * percent) / 1000) % 100;
}

/*int32_t GFX_ScaleInteger(int num, int oldMax, int newMax)
{
    num = GFX_Percent(num, oldMax);
    num = GFX_PercentOf(newMax, num);
    
    return num;
}*/

float roundFloat(float flt);

uint32_t GFX_ScaleInteger(uint32_t num, uint32_t oldMax, uint32_t newMax)
{
    float percent;
    
    percent = (float)num / (float)oldMax;
    percent *= newMax;
    //percent = roundFloat(percent);
    
    return (uint32_t)percent;
}

/*void GFX_RebaseValue(int32_t imin, int32_t ival, uint32_t* uval)
{
    int32_t min, max;
    
    if(imin == ival)
    {
        *uval = 0;
        
        return;
    }
    
    *umax = imax - min;
}*/

uint32_t GFX_AbsoluteValue(int32_t val)
{
    uint32_t temp = val >> 31;
    val ^= temp;
    val += temp & 1;
    
    return (uint32_t)val;
}

int32_t GFX_Lerp(int32_t l, int32_t r, uint32_t per)
{
    int32_t imin, imax;
    uint32_t umax, val;
    
    if(l == r)
        return l;
        
    if(per <= 0)
        return l;
    
    if(per >= 100)
        return r;
        
    imin = l;
    imax = r;
    
    if(l > r)
    {
        imin = r;
        imax = l;
        
        per = 100 - per;
    }
        
    umax = imax - imin;
    
    val = GFX_PercentOf(umax, per);
    
    return imin + val;    
}

int32_t GFX_DivideRounding(int32_t num, int32_t denom)
{
    int32_t lnum, ldenom, lquo;
    
    if(denom == 0)
        return 0;
        
    lnum = num * 100;
    ldenom = denom;// * 100;
    
    lquo = lnum / ldenom;
    
    if(lquo % 100 >= 50)
        return (num / denom) + 1;
    else
        return num / denom;
}

#endif // GFX_DRAW_PIPELINE_ENABLED