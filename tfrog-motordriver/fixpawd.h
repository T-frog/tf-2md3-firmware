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
	@file fixp.h
	@brief Fixed point value operation
 */

#ifndef __FIX_POINT_AWD__
#define __FIX_POINT_AWD__

#define INLINE inline

typedef int fixp4;

#ifndef FP4_POINTBIT
#define FP4_POINTBIT 17
#endif

#define FP4_POINTBIT2 (FP4_POINTBIT * 2)
#define FP4_DIV (1.0 / (double)(1 << FP4_POINTBIT))
#define FP4_MUL ((double)(1 << FP4_POINTBIT))
#define FP4_ONE (1 << FP4_POINTBIT)

#define DOUBLE2FP4(a) (fixp4)(a * FP4_MUL + 0.5)
#define INT2FP4(a) (fixp4)(a << FP4_POINTBIT)

fixp4 fp4mul(fixp4 a, fixp4 b);
fixp4 fp4mulf(fixp4 a, fixp4 b);
fixp4 fp4div(fixp4 a, fixp4 b);
fixp4 double2fp4(double a);
fixp4 int2fp4(int a);
double fp42double(fixp4 a);

#endif
