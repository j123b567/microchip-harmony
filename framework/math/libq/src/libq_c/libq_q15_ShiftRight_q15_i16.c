//*****************************************************************************
//
// File: libq_q15_ShiftRight_q15_i16.c
//
// Description: Performs an 'Arithmetic' Right Shift of the 16-bit input 
//   argument 'a' by the input argument 'b' bit positions.
//
//*****************************************************************************

#include "libq_C.h"


/******************************************************************************
 *
 * libq_q15_ShiftRight_q15_i16()
 *
 * Description:
 *   Performs an 'Arithmetic' RIGHT Shift on a 16-bit input by 'b' bit
 *   positions.  For positive shift directions (b>0), 'b' Lsb-bits are shifted
 *   out to the right and 'b' sign-extended Msb-bits fill in from the left.  
 *   For negative shift directions (b<0), 'b' Lsb's are shifted to the
 *   LEFT with 0's filling in the empty lsb position. The left shifting
 *   causes 'b' Msb-bits to fall off to the left, saturation is applied to
 *   any shift left value that overflows. This function calls the
 *   '..Shl()' function to perform any 16-bit left shifts.  This function 
 *   does not provide any status-type information to indicate when overflows 
 *   occur.
 *
 *   This function relates to the ETSI shr function.
 *
 * Arguments:
 *   q15 a
 *     [in]  16-bit signed input value to shift
 *   i16 b
 *     [in]  16-bit signed integer shift index 
 *          positive value: # of bits to right shift (sign extend)
 *                          { To get all sign bits, b>=15 }  
 *          negative value: # of bits to left shift (zeros inserted at LSB's)
 *
 * Return Value:
 *   q15 result
 *     [return]  16-bit signed shifted output
 *
 ******************************************************************************/
q15 libq_q15_ShiftRight_q15_i16(q15 a, i16 b)
{
  q15 result;                    /* Value returned */

  /* "Right shift" or "Left shift"*/
  if (b >= 0)
  {   
    /* Right shift, first check whether 'Msb' will shift past 'Lsb'.
     * If yes, every bit in result is a copy of original sign bit. */
    if (b >= (NUMBITSFRACT16-1)) 
    {
      result = (a < 0) ? (q15)BITMASKFRACT16: (q15)0;
    }
    /* Else, Right shift input by 'b' bits with sign extention. If 'a'
     * is positive, the sign-extenting is automatic. If 'a' is negative,
     * the sign bit must be held while shifting.  Hold the sign bit 
     * by complementing the value 'before' and 'after' the shift, ex:
     * (a=0x8000,b=8): before:~(0x8000)=>0x7fff, after:~(0x07fff)=>0xf800 */ 
    else
    {
      result = (a < 0) ? (q15)(~ ((~a) >> b)): (q15)(a >> b);
    }
  }
  /* Else, Left shift by 'b' bits with zero-fill. Any overflow caused by 
   * the shift is saturated.  */
  else
  {
    if (b != MININT16)
    {
      result = libq_q15_ShiftLeft_q15_i16(a,(int16_t)(-b));
    }
    else
    {
      result = libq_q15_ShiftLeft_q15_i16(a,(int16_t)(MAXINT16));
    }
  }
  return (result);
}
