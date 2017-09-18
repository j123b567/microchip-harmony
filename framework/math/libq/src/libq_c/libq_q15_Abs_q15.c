//*****************************************************************************
//
// File: libq_q15_Abs_q15.c
//
// Description: Creates a saturated Absolute value.
//
//*****************************************************************************/

#include "libq_c.h"


/******************************************************************************
 *
 * libq_q15_Abs_q15()
 *
 * Description:
 *   Creates a saturated Absolute value.  It takes the absolute value of the
 *   16-bit 2s-complement fractional input with saturation. The saturation is
 *   for handling the case where taking the absolute value of MINFRACT16 is
 *   greater than MAXFRACT16, or the allowable range of 16-bit values.
 *   This function relates to the ETSI abs function.
 *
 * Arguments:
 *   q15 a
 *     [in] input argument
 *
 * Return Value:
 *   q15 result
 *     [return]  abs(input) <= MAXFRACT16
 *
 ******************************************************************************/
q15 libq_q15_Abs_q15(q15 a)
{
  q15 result;                   /* Value returned */

  /* If input is a positive 2's complement value, then 
   * simply take the input 2's complement value */
  if (a >= (q15)0)
  {
    result = (q15)(a);
  }
  /* Else the Input is a Negative value, so check for min value (-1) */
  else
  {
    /* If not the min value (-1), just negate the input for absolute value */ 
    if (a != (q15)MINFRACT16)
    {
      result = (q15)(-a);
    }
    /* Else it was the min value (-1), so saturate to max value (1) */
    else
    {
      result = (q15)MAXFRACT16;
    }
  }
  return (result);
}
