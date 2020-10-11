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

#include <stdint.h>

#include "filter.h"
#include "utils.h"

#define FIXED_POINT 4096

int32_t Filter1st_Filter(Filter1st* filter, int32_t input)
{
  if (filter->init == 0)
  {
    filter->init = 1;
    filter->x = input * (FIXED_POINT - filter->k[2]) / filter->k[3];
  }
  filter->x = (filter->k[0] * input + filter->k[1] * filter->x) / FIXED_POINT;
  return (filter->k[2] * input + filter->k[3] * filter->x) / FIXED_POINT;
}

int32_t Filter1st_CreateLPF(Filter1st* filter, float timeconst)
{
  filter->init = 0;
  filter->k[3] = (int32_t)((-1.0 / (1.0 + 2.0 * timeconst)) * FIXED_POINT);
  filter->k[2] = -filter->k[3];
  filter->k[1] = (int32_t)(((1.0 - 2.0 * timeconst) * (-1.0 / (1.0 + 2.0 * timeconst))) * FIXED_POINT);
  filter->k[0] = -filter->k[1] - FIXED_POINT;
  filter->x = 0;
  return 1;
}

int32_t FilterExp_Filter(FilterExp* filter, const int32_t input)
{
  if (filter->init == 0)
  {
    filter->init = 1;
    filter->x = input * FIXED_POINT;
  }
  filter->x = filter->x * filter->alpha_complement / FIXED_POINT + input * filter->alpha;
  return filter->x / FIXED_POINT;
}

int32_t FilterExp_FilterAngle(FilterExp* filter, int32_t input, const int32_t pi2, const int32_t ang_max)
{
  if (filter->init == 0)
  {
    filter->init = 1;
    filter->x = input * FIXED_POINT;
  }

  const int32_t x0 = filter->x / FIXED_POINT;
  normalize(&input, x0 - pi2 / 2, pi2);

  filter->x =
      (int32_t)((int64_t)(filter->x) * filter->alpha_complement / FIXED_POINT) +
      input * filter->alpha;

  normalize(&filter->x, 0, ang_max * FIXED_POINT);
  return filter->x / FIXED_POINT;
}

int32_t FilterExp_CreateLPF(FilterExp* filter, const int32_t timeconst)
{
  filter->init = 0;
  filter->alpha = FIXED_POINT / timeconst;
  filter->alpha_complement = FIXED_POINT - filter->alpha;
  return 1;
}
