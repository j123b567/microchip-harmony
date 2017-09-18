//*****************************************************************************
//
// File: libq_q15_Add_q15_q15.c
//
// Description: Add (op1 + op2) or subtract (op1 - op2) two 16-bit 
//   2s-complement fractional operands to produce a 16-bit 2s-complement 
//   fractional result with saturation.
//
//*****************************************************************************/

#include "libq_c.h"


//*****************************************************************************
//
// libq_Saturate()
//
// Description:
//   Private function used only by this fractional library to  
//   saturate a 32-bit value to 16-bits and returns the result.
//   The lower 16-bits of the 32-bit input are limited to within the
//   range of MINFRACT16 <= In <= MAXFRACT16.  It is assumed that 
//   the binary point in the 16-bit output is aligned with the LOWER 16-bits
//   of the 32 bit input. The function is used for overflow control to limit
//   values before they exceed their range.
//   This function does not produce an overflow status flag.
//
// Arguments:
//   q31 a
//     [in]        Input value to be saturated
//
// Return Value:
//   q15 result
//     [return]    Saturated value
//
//*****************************************************************************/
static q15 libq_Saturate(q31 a)
{
  q15 result;                   /* Value returned */

  /* Take input's bottom 16-bits for output.  Modify this result next
   * only if input exceeds the output range */  
  result = (q15)a;
  /* Check for overflow with the 'Sign-Extended' 16-bit range limits. If it 
   * overflows in the positive direction, force to Max positive value.*/
  if (a > (q31)(MAXFRACT16)) 
  {
    result = (q15)MAXFRACT16;
  }
  /* If it overflows in the negative direction, force to MIN negative value.*/
  if (a < (q31)(MINFRACT16))
  {
    result = (q15)MINFRACT16;
  } 
  return (result);
}


//*****************************************************************************
//
// libq_q15_Add_q15_q15()
//
// Description:
//   Add two 16-bit 2s-complement fractional (op1 + op2) to produce a 16-bit
//   2s-complement fractional result with saturation. The saturation is for
//   handling the overflow/underflow cases, where the result is set to
//   MAX16 when an overflow occurs and the result is set to
//   MIN16 when an underflow occurs.  This function does not produce
//   any status flag to indicate when an overflow or underflow has occured.
//   It is assumed that the binary point is in exactly the same bit position
//   for both 16-bit inputs and the resulting 16-bit output.  
//
//   This function calls the libq_Sat_q15()
//
// Arguments:
//   q15 a
//     [in] Addition operand op1.
//   q15 b;
//     [in] Addition operand op2.
//
// Return Value:
//   q15 result
//     [return]  Range: MINFRACT16 <= result <= MAXFRACT16
//
//*****************************************************************************
q15 libq_q15_Add_q15_q15(q15 a, q15 b)
{
  q31  sumExtP;                   /* Value returned */

  /* Sign-Extend inputs to 32-bits before 'adding' so that the addition is
   * done with extended-precision to accomodate for any overflow bits. */
  sumExtP = (q31)a + (q31)b;
  /* Saturate the 32-bit sum to eliminate any overflow bits so that the 
   * output sum will be within the allowable 16-bit range. */
  return (libq_Saturate(sumExtP));
}


/******************************************************************************
 *
 * libq_q15_Sub_q15_q15()
 *
 * Description:
 *   Subtract two 16-bit 2s-complement fractional (op1 - op2) to produce a
 *   16-bit 2s-complement fractional difference result with saturation. The
 *   saturation is for handling the overflow/underflow cases, where the result
 *   is set to MAX16 when an overflow occurs and the result is set to
 *   MIN16 when an underflow occurs.  This function does not produce
 *   any status flag to indicate when an overflow or underflow has occured.
 *   It is assumed that the binary point is in exactly the same bit position
 *   for both 16-bit inputs and the resulting 16-bit output.  
 *   This function relates to the ETSI sub function.
 *
 *   This function calls the ..Sat() function.
 *
 * Arguments:
 *   q15 a
 *     [in] Subtraction operand op1.
 *   q15 b;
 *     [in] Subtraction operand op2.
 *
 * Return Value:
 *   q15 result
 *     [return]  Range: MINFRACT16 <= result <= MAXFRACT16
 *
 ******************************************************************************/
q15 libq_q15_Sub_q15_q15(q15 a, q15 b)
{
  q31  diffExtP;                   /* Value returned */

  /* Sign-Extend inputs to 32-bits before 'subtracting' so that the subtraction
   * is done with extended-precision to accomodate for any overflow bits. */
  diffExtP = (q31)a - (q31)b;
  /* Saturate the 32-bit difference to eliminate any overflow bits so that 
   * the output difference will be within the allowable 16-bit range. */
  return (libq_Saturate(diffExtP));
}
