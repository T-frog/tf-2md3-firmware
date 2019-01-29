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

#ifndef __ERRORS_H__
#define __ERRORS_H__

const unsigned char error_pat[ERROR_NUM + 1] =
    {
      0xCC,  // 11001100 low voltage
      0xBF,  // 10111111 hall1
      0xAF,  // 10101111 hall2
      0xAA,  // 10101010 watchdog
      0x00,  // 00000000 none
    };

#endif
