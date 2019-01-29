/* ----------------------------------------------------------------------------
 * Copyright 2011-2019 T-frog Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ----------------------------------------------------------------------------
 */

/**
	@file fixpmath.c
	@brief Fixed point value mathmatical functions
 */

#include "fixpawd_math.h"

fixp4 abs(fixp4 x)
{
  if (x < 0)
    return -x;
  return x;
}

/**
	@brief Calculate sin function (fast ver, 0 <= x <= PI/4)
	@param x [in] Input value
	@return sin(x)
 */
fixp4 fp4sinf(fixp4 x)
{
  fixp4 x2;   // x^2
  fixp4 xn;   // x^3, x^5, ...
  fixp4 res;  // Output

  xn = x;
  x2 = fp4mulf(x, x);

  xn = fp4mulf(xn, x2);
  res = x - fp4mulf(xn, DOUBLE2FP4(1.0 / 6.0));
  xn = fp4mulf(xn, x2);
  res += fp4mulf(xn, DOUBLE2FP4(1.0 / 120.0));
  xn = fp4mulf(xn, x2);
  res -= fp4mulf(xn, DOUBLE2FP4(1.0 / 5040.0));

  return res;
}

/**
	@brief Calculate sin function
	@param x [in] Input value
	@return sin(x)
 */
fixp4 fp4sin(fixp4 x)
{
  char minus;  // Is output value < 0
  fixp4 y;     // Output

  minus = 0;
  if (x < 0)
  {
    if (x < -FP4_PI2)
      x = x % (FP4_PI2);
    x = FP4_PI2 - x;
  }
  else
  {
    if (x > FP4_PI2)
      x = x % (FP4_PI2);
  }

  if (x > FP4_PI)
  {
    minus = 1;
    x -= FP4_PI;
  }
  if (x > FP4_PI_2)
    y = fp4cos(x - FP4_PI_2);
  else if (x > FP4_PI_4)
    y = 2 * fp4mulf(fp4cosf(x / 2), fp4sinf(x / 2));
  else
    y = fp4sinf(x);

  if (minus)
    return -y;
  return y;
}

/**
	@brief Calculate cos function (fast ver, 0 <= x <= PI/4)
	@param x [in] Input value
	@return cos(x)
 */
fixp4 fp4cosf(fixp4 x)
{
  fixp4 x2;   // x^2
  fixp4 xn;   // x^2, x^4, ...
  fixp4 res;  // Output

  x2 = fp4mulf(x, x);
  xn = x2;
  res = FP4_ONE - fp4mulf(xn, DOUBLE2FP4(1.0 / 2.0));
  xn = fp4mulf(xn, x2);
  res += fp4mulf(xn, DOUBLE2FP4(1.0 / 24.0));
  xn = fp4mulf(xn, x2);
  res -= fp4mulf(xn, DOUBLE2FP4(1.0 / 720.0));

  return res;
}

/**
	@brief Calculate cos function
	@param x [in] Input value
	@return cos(x)
 */
fixp4 fp4cos(fixp4 x)
{
  char minus;  // Is output value < 0
  fixp4 res;   // Output

  minus = 0;

  if (x < 0)
  {
    if (x < -FP4_PI2)
      x = x % (FP4_PI2);
    x = FP4_PI2 - x;
  }
  else
  {
    if (x > FP4_PI2)
      x = x % (FP4_PI2);
  }
  if (x > FP4_PI)
  {
    minus = 1;
    x -= FP4_PI;
  }

  if (x > FP4_PI_2)
    res = -fp4sin(x - FP4_PI_2);
  else if (x > FP4_PI_4)
  {
    res = fp4sinf(x / 2);
    res = FP4_ONE - 2 * fp4mulf(res, res);
  }
  else
    res = fp4cosf(x);

  if (minus)
    return -res;
  return res;
}

/**
	@brief Calculate arctan [5 digit .17bit]
	@param x [in] Input value
	@return atan(x)
 */
fixp4 fp4atan(fixp4 x)
{
  fixp4 xn;    // x^2, x^3, ...
  fixp4 th;    // Output
  char minus;  // Is output value < 0
  char inv;    // Is output value > 45deg

  minus = 0;
  inv = 0;

  if (x < 0)
  {
    x = -x;
    minus = 1;
  }
  if (x > FP4_ONE)
  {
    x = fp4div(FP4_ONE, x);
    inv = 1;
  }
  xn = x;

  // 5 digit
  if (x <= DOUBLE2FP4(0.005))
  {
    th = x;
  }
  else if (x <= DOUBLE2FP4(0.45))
  {
    // 0.0947x^5 + 0.0631x^4 - 0.3489x^3 + 0.0018x^2 + 0.9999x + 0.000003
    th = DOUBLE2FP4(0.000009);
    th += fp4mul(xn, DOUBLE2FP4(0.99991));
    xn = fp4mul(xn, x);
    th += fp4mul(xn, DOUBLE2FP4(0.0018));
    xn = fp4mul(xn, x);
    th -= fp4mul(xn, DOUBLE2FP4(0.3489));
    xn = fp4mul(xn, x);
    th += fp4mul(xn, DOUBLE2FP4(0.0631));
    xn = fp4mul(xn, x);
    th += fp4mul(xn, DOUBLE2FP4(0.0947));
  }
  else if (x <= DOUBLE2FP4(0.995))
  {
    // -0.0687x^5 + 0.3126x^4 - 0.4914x^3 + 0.0332x^2 + 1.0006x + 0.00092
    th = -DOUBLE2FP4(0.000919);
    th += fp4mul(xn, DOUBLE2FP4(1.0006));
    xn = fp4mul(xn, x);
    th += fp4mul(xn, DOUBLE2FP4(0.0332));
    xn = fp4mul(xn, x);
    th -= fp4mul(xn, DOUBLE2FP4(0.4914));
    xn = fp4mul(xn, x);
    th += fp4mul(xn, DOUBLE2FP4(0.3126));
    xn = fp4mul(xn, x);
    th -= fp4mul(xn, DOUBLE2FP4(0.0687));
  }
  else
  {
    th = FP4_PI_4 - FP4_ONE / 2 + x / 2;
  }

  if (inv)
    th = FP4_PI_2 - th;
  if (minus)
    return -th;
  return th;
}

/**
	@brief Calculate arctan with quadrant info [5 digit .17bit]
	@param y [in] Input value y
	@param x [in] Input value x
	@return atan2(y/x)
 */
fixp4 fp4atan2(fixp4 y, fixp4 x)
{
  fixp4 th;  // Output

  if (x > y)
  {
    th = fp4atan(abs(fp4div(y, x)));
  }
  else
  {
    th = FP4_PI_2 - fp4atan(abs(fp4div(x, y)));
  }
  if (y >= 0)
  {
    if (x >= 0)
    {
      return th;
    }
    else
    {
      return FP4_PI - th;
    }
  }
  else
  {
    if (x >= 0)
    {
      return -th;
    }
    else
    {
      return -FP4_PI + th;
    }
  }
  return 0;
}

/**
	@brief Calculate sqrt function
	@param x [in] Input value
	@return sqrt(x)
 */
fixp4 fp4sqrt(fixp4 x)
{
  fixp4 res;  // Output

  res = fp4mulf(x, fp4sqrtinv(x));
  res = (fp4div(x, res) + res) >> 1;

  return res;
  /* 
	 * // Slow: fixp4 s, res; long long x1;
	 * 
	 * x1 = ( (long long) x ) << FP4_POINTBIT; s = 1 << FP4_POINTBIT; res = x; while( s < res ){ s = s << 1; res = res
	 * >> 1; } do{ res = s; s = ( x1 / s + s ) >> 1; }while( s < res );
	 * 
	 * return s; */
}

/**
	@brief Calculate sqrt function (fast ver)
	@param x [in] Input value
	@return sqrt(x)
 */
fixp4 fp4sqrtf(fixp4 x)
{
  return fp4mulf(x, fp4sqrtinv(x));
}

/**
	@brief Calculate 1/sqrt function
	@param x [in] Input value
	@return 1/sqrt(x)
 */
fixp4 fp4sqrtinv(fixp4 x)
{
  fixp4 res;   // Output
  fixp4 h, t;  // Temporary
  // char i; // Loop

  if (x & 0x40000000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 30];
  else if (x & 0x20000000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 29];
  else if (x & 0x10000000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 28];
  else if (x & 0x08000000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 27];
  else if (x & 0x04000000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 26];
  else if (x & 0x02000000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 25];
  else if (x & 0x01000000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 24];
  else if (x & 0x00800000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 23];
  else if (x & 0x00400000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 22];
  else if (x & 0x00200000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 21];
  else if (x & 0x00100000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 20];
  else if (x & 0x00080000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 19];
  else if (x & 0x00040000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 18];
  else if (x & 0x00020000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 17];
  else if (x & 0x00010000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 16];
  else if (x & 0x00008000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 15];
  else if (x & 0x00004000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 14];
  else if (x & 0x00002000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 13];
  else if (x & 0x00001000)
    res = fp4_sqrt[32 - FP4_POINTBIT + 12];
  else if (x & 0x00000800)
    res = fp4_sqrt[32 - FP4_POINTBIT + 11];
  else if (x & 0x00000400)
    res = fp4_sqrt[32 - FP4_POINTBIT + 10];
  else if (x & 0x00000200)
    res = fp4_sqrt[32 - FP4_POINTBIT + 9];
  else if (x & 0x00000100)
    res = fp4_sqrt[32 - FP4_POINTBIT + 8];
  else if (x & 0x00000080)
    res = fp4_sqrt[32 - FP4_POINTBIT + 7];
  else if (x & 0x00000040)
    res = fp4_sqrt[32 - FP4_POINTBIT + 6];
  else if (x & 0x00000020)
    res = fp4_sqrt[32 - FP4_POINTBIT + 5];
  else if (x & 0x00000010)
    res = fp4_sqrt[32 - FP4_POINTBIT + 4];
  else if (x & 0x00000008)
    res = fp4_sqrt[32 - FP4_POINTBIT + 3];
  else if (x & 0x00000004)
    res = fp4_sqrt[32 - FP4_POINTBIT + 2];
  else if (x & 0x00000002)
    res = fp4_sqrt[32 - FP4_POINTBIT + 1];
  else if (x & 0x00000001)
    res = fp4_sqrt[32 - FP4_POINTBIT];
  else
    return 0;
  /* 
	 * // 'if' lines mean: for( i = 30; i >= 0; i -- ){ if( x & ( 1 << i ) ){ res = fp4_sqrt[ 32 - FP4_POINTBIT + i ];
	 * break; } } */
  do
  {
    h = FP4_ONE - (((((long long)x * res) >> FP4_POINTBIT) * res) >> FP4_POINTBIT);

    t = h * 3 + FP4_ONE * 4;
    t = ((long long)h * t) >> (FP4_POINTBIT + 3);
    t += FP4_ONE;

    res = ((long long)res * t) >> FP4_POINTBIT;
  } while (abs(h) > res);
  res++;  // magic

#if FP4_POINTBIT > 17
  h = ((((((long long)x * res) >> FP4_POINTBIT) * res) >> FP4_POINTBIT) * res) >> FP4_POINTBIT;
  res = (3 * res - h) >> 1;
#endif

  return res;
}

/**
	@brief Calculate log2 function [4 digit .17bit]
	@param x [in] Input value
	@return log2(x)
 */
fixp4 fp4log2(fixp4 x)
{
  fixp4 res;  // Output
  fixp4 fp;   // Fraction

  res = 0;
  while (x < FP4_ONE)
  {
    res -= FP4_ONE;
    x *= 2;
  }
  while (x >= FP4_ONE * 2)
  {
    res += FP4_ONE;
    x /= 2;
  }

  x <<= (32 - FP4_POINTBIT - 5);
  fp = FP4_ONE << (32 - FP4_POINTBIT - 5);
  res <<= (32 - FP4_POINTBIT - 5);

  while (fp >= (1 << (33 - FP4_POINTBIT - 5)))
  {
    fp /= 2;
    x = ((long long)x * x) >> (32 - 5);

    if (x >= (FP4_ONE << (32 - FP4_POINTBIT - 5 + 1)))
    {
      x /= 2;
      res += fp;
    }
  }
  return res >> (32 - FP4_POINTBIT - 5);
}

/**
	@brief Calculate log2 function (fast ver) [2 digit .17bit]
	@param x [in] Input value
	@return log2(x)
 */
fixp4 fp4log2f(fixp4 x)
{
  fixp4 res;  // Output
  fixp4 fp;   // Fraction

  res = 0;
  while (x < FP4_ONE)
  {
    res -= FP4_ONE;
    x *= 2;
  }
  while (x >= FP4_ONE * 2)
  {
    res += FP4_ONE;
    x /= 2;
  }

  x <<= (32 - FP4_POINTBIT - 5);
  fp = FP4_ONE << (32 - FP4_POINTBIT - 5);
  res <<= (32 - FP4_POINTBIT - 5);

  while (fp >= (1 << (33 - FP4_POINTBIT - 5)))
  {
    fp /= 2;
    x = (x >> 13) * (x >> 14);

    if (x >= (FP4_ONE << (32 - FP4_POINTBIT - 5 + 1)))
    {
      x /= 2;
      res += fp;
    }
  }
  return res >> (32 - FP4_POINTBIT - 5);
}

/**
	@brief Calculate ln function
	@param x [in] Input value
	@return ln(x)
 */
fixp4 fp4ln(fixp4 x)
{
  return fp4mulf(fp4log2(x), FP4_LOG2E);
}

/**
	@brief Calculate log function
	@param x [in] Input value
	@return log(x)
 */
fixp4 fp4log(fixp4 x)
{
  return fp4mulf(fp4log2(x), FP4_LOG2T);
}

/**
	@brief Calculate logn function
	@param x [in] Input value
	@param n [in] Base value
	@return log(x)
 */
fixp4 fp4logn(fixp4 x, fixp4 n)
{
  return fp4div(fp4log2(x), fp4log2(n));
}

/**
	@brief Calculate ln function (fast ver)
	@param x [in] Input value
	@return ln(x)
 */
fixp4 fp4lnf(fixp4 x)
{
  return fp4mulf(fp4log2f(x), FP4_LOG2E);
}

/**
	@brief Calculate log function (fast ver)
	@param x [in] Input value
	@return log(x)
 */
fixp4 fp4logf(fixp4 x)
{
  return fp4mulf(fp4log2f(x), FP4_LOG2T);
}

/**
	@brief Calculate logn function (fast ver)
	@param x [in] Input value
	@param n [in] Base value
	@return log(x)
 */
fixp4 fp4lognf(fixp4 x, fixp4 n)
{
  return fp4div(fp4log2f(x), fp4log2f(n));
}

/**
	@brief Calculate exp function
	@param x [in] Input value
	@return exp(x)
 */
fixp4 fp4exp(fixp4 x)
{
  fixp4 res;  // Output
  fixp4 mask;
  int i;
  char inv;

  if (x < 0)
  {
    x = -x;
    inv = 1;
  }
  else
    inv = 0;

  mask = 1;
  res = FP4_ONE << (32 - FP4_POINTBIT - 6);
  for (i = 31 - FP4_POINTBIT; i < 33; i++)
  {
    if (x & mask)
      res = fp4mulf(res, fp4_exp[i]);
    mask <<= 1;
  }
  res >>= (32 - FP4_POINTBIT - 6);

  if (x & (1 << (33 - 31 + FP4_POINTBIT + 0)))
    res = fp4mul(res, fp4_exp[33]);
  if (x & (1 << (33 - 31 + FP4_POINTBIT + 1)))
    res = fp4mul(res, fp4_exp[34]);
  if (x & (1 << (33 - 31 + FP4_POINTBIT + 2)))
    res = fp4mul(res, fp4_exp[35]);

  /* 
	 * // 'if' lines mean: for( i = 33; i < 36; i ++ ){ if( x & mask ) res = fp4mul( res, fp4_exp[i] ); mask <<= 1; } */

  if (inv)
    return fp4div(FP4_ONE, res);
  return res;
}
