/**
	@file fixp.c
	@brief Fixed point value operation
 */

#include "fixpawd.h"

/**
	@brief Multiple fixed point values (fast var)
	@param a [in] Input value A
	@param b [in] Input value B
	@return A*B
 */
fixp4 fp4mulf(fixp4 a, fixp4 b)
{
  return (fixp4)(((long long)a * b) / FP4_ONE);
}

/**
	@brief Multiple fixed point values
	@param a [in] Input value A
	@param b [in] Input value B
	@return A*B
 */
fixp4 fp4mul(fixp4 a, fixp4 b)
{
  long long y;

  y = (long long)a * b;
  y = y >> FP4_POINTBIT;

  if (y > 0x7FFFFFFF)
    return 0x7FFFFFFF;
  if (y < -0x7FFFFFFF)
    return -0x7FFFFFFF;
  return (fixp4)y;
}

/**
	@brief Divide Fixed point value
	@param a [in] Input value A
	@param b [in] Input value B
	@return A/B
 */
fixp4 fp4div(fixp4 a, fixp4 b)
{
  return (fixp4)(((long long)a * FP4_ONE) / b);
}

/**
	@brief Convert double value to fixed point value
	@param a [in] Input value A
	@return A expressed in fixed point
 */
fixp4 double2fp4(double a)
{
  return (fixp4)(a * FP4_MUL + 0.5);
}

/**
	@brief Convert int value to fixed point value
	@param a [in] Input value A
	@return A expressed in fixed point
 */
fixp4 int2fp4(int a)
{
  return (fixp4)(a * FP4_ONE);
}

/**
	@brief Convert fixed point value to double value
	@param a [in] Input value A
	@return A expressed in double
 */
double fp42double(fixp4 a)
{
  return (double)a * FP4_DIV;
}
