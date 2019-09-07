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

int Filter1st_Filter(Filter1st* filter, int input)
{
  if (filter->init == 0)
  {
    filter->init = 1;
    filter->x = input * (FIXED_POINT - filter->k[2]) / filter->k[3];
  }
  filter->x = (filter->k[0] * input + filter->k[1] * filter->x) / FIXED_POINT;
  return (filter->k[2] * input + filter->k[3] * filter->x) / FIXED_POINT;
}

int Filter1st_CreateLPF(Filter1st* filter, float timeconst)
{
  filter->init = 0;
  filter->k[3] = (int)((-1.0 / (1.0 + 2.0 * timeconst)) * FIXED_POINT);
  filter->k[2] = -filter->k[3];
  filter->k[1] = (int)(((1.0 - 2.0 * timeconst) * (-1.0 / (1.0 + 2.0 * timeconst))) * FIXED_POINT);
  filter->k[0] = -filter->k[1] - FIXED_POINT;
  filter->x = 0;
  return 1;
}

int FilterExp_Filter(FilterExp* filter, const int input)
{
  if (filter->init == 0)
  {
    filter->init = 1;
    filter->x = input * FIXED_POINT;
  }
  filter->x = filter->x * filter->alpha_complement / FIXED_POINT + input * filter->alpha;
  return filter->x / FIXED_POINT;
}

int FilterExp_FilterAngle(FilterExp* filter, int input, const int pi2, const int ang_max)
{
  if (filter->init == 0)
  {
    filter->init = 1;
    filter->x = input * FIXED_POINT;
  }

  const int x0 = filter->x / FIXED_POINT;
  normalize(&input, x0 - pi2 / 2, pi2);

  filter->x =
      (int)((int64_t)(filter->x) * filter->alpha_complement / FIXED_POINT) +
      input * filter->alpha;

  normalize(&filter->x, 0, ang_max * FIXED_POINT);
  return filter->x / FIXED_POINT;
}

int FilterExp_CreateLPF(FilterExp* filter, const int timeconst)
{
  filter->init = 0;
  filter->alpha = FIXED_POINT / timeconst;
  filter->alpha_complement = FIXED_POINT - filter->alpha;
  return 1;
}
