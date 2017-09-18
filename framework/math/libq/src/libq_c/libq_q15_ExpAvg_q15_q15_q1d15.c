//*****************************************************************************
// File: libq_q15_ExpAvg_q15_q15_q1d15.c
//*****************************************************************************/

#include "libq_C.h"

//*****************************************************************************
//
// libq_q15_ExpAvg_q15_q15_q1d15()
//
// Description:
//   Exponential averaging implements a smoothing function based on the form:
//       avg[i+1] = avg[i] * lamda + new * (1-lamda)
//   In this implementation, is has been optimized as follows.
//       avg[i+1] = (avg[i] - new) * lamda + new
//
//   The optimization precludes accurate processing of new numbers that differ
//   from the current average by more than unity. If the difference is greater
//   than unity or less than negative unity, the difference is saturated.
//
//   The effect is akin to a smaller lambda, e.g., the new value will have a
//   greater weight than expected. If the smoothing is of data that is entirely
//   positive or entirely negative, then the saturation will not be an issue.
//
// Arguments:
//   FxPnt16 prevAvg16
//     [in] Previous exponential average
//   FxPnt16 newMeas16
//     [in] New value to be averaged in
//   q15 lamdaQ1d15
//     [in] exponential averaging constant
//
// Return Value:
//   FxPnt16 newAvg16
//
//*****************************************************************************/
q15 libq_q15_ExpAvg_q15_q15_q1d15(q15 prevAvg16,
                     q15 newMeas16,
                     q15 lamdaQ1d15)
{
  q15 newAvg16;   /* The new average to be returned. */

  /* Do exponential average */
  /* avg[i+1] = (avg[i] - new) * lamda + new */
  newAvg16 = libq_q15_Sub_q15_q15(prevAvg16, newMeas16);
  newAvg16 = libq_q15_MultipyR2_q15_q15(lamdaQ1d15, newAvg16);
  newAvg16 = libq_q15_Add_q15_q15(newAvg16, newMeas16);

  /* Return the product */
  return(newAvg16);

}
