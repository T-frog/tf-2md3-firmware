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

#ifndef __FILTER_H__
#define __FILTER_H__

typedef struct
{
  int32_t k[4];
  int32_t x;
  char init;
} Filter1st;

int32_t Filter1st_Filter(Filter1st* filter, int32_t input);
int32_t Filter1st_CreateLPF(Filter1st* filter, float timeconst);

typedef struct
{
  int32_t x;
  int32_t alpha;
  int32_t alpha_complement;
  char init;
} FilterExp;

int32_t FilterExp_Filter(FilterExp* filter, const int32_t input);
int32_t FilterExp_FilterAngle(FilterExp* filter, int32_t input, const int32_t pi2, const int32_t ang_max);
int32_t FilterExp_CreateLPF(FilterExp* filter, const int32_t timeconst);

#endif
