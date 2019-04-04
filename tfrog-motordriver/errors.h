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

const unsigned char error_pat[2][ERROR_NUM + 1] =
    {
      {
          0xCC,  // 11001100 low voltage
          0xBF,  // 10111111 hall sequence
          0xB3,  // 10110011 hall inconsistent with encoder
          0xAA,  // 10101010 watchdog
          0xA0,  // 10100000 eeprom
          0xA8,  // 10101000 internal
          0x00,  // 00000000 none
      },
      {
          0x00,  // 00000000 enabled only on motor 0
          0xAF,  // 10101111 hall sequence
          0xA3,  // 10100011 hall inconsistent with encoder
          0x00,  // 00000000 enabled only on motor 0
          0x00,  // 00000000 enabled only on motor 0
          0x00,  // 00000000 enabled only on motor 0
          0x00,  // 00000000 none
      }
    };

#endif
