/* ----------------------------------------------------------------------------
 * Copyright 2011-2020 T-frog Project
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

#include "communication.h"
#include "controlVelocity.h"
#include "debug.h"
#include "eeprom.h"
#include "registerFPGA.h"

extern uint8_t send_buf[SEND_BUF_LEN];
extern volatile uint32_t send_buf_pos;

static int32_t dump_id = -1;
static volatile void* dump_ptr = 0;
static int32_t dump_size = -1;
static int32_t dump_cnt = 0;

void start_dump(const int32_t imotor, const int32_t arg)
{
  if (dump_ptr != 0)
    return;

  dump_id = imotor << 4 | arg;
  dump_cnt = 0;
  switch (arg)
  {
    case 0:
      dump_ptr = &motor[imotor];
      dump_size = sizeof(MotorState);
      break;
    case 1:
      dump_ptr = &motor_param[imotor];
      dump_size = sizeof(MotorParam);
      break;
    case 2:
      dump_ptr = &driver_state;
      dump_size = sizeof(DriverState);
      break;
    case 3:
      dump_ptr = &driver_param;
      dump_size = sizeof(DriverParam);
      break;
    case 4:
      dump_ptr = &saved_param;
      dump_size = sizeof(Tfrog_EEPROM_data);
      break;
    case 5:
      dump_ptr = &THEVA;
      dump_size = sizeof(THEVA_REG);
      break;
    default:
      dump_ptr = 0;
      break;
  }
}

void dump_send()
{
  if (dump_ptr == 0)
    return;

  int32_t size = dump_size;
  if (size > 32)
    size = 32;

  send_buf_pos = 0;
  send_buf[0] = COMMUNICATION_INT_BYTE;

  const uint8_t hdr[3] = {
      INT_debug_dump,
      dump_id,
      dump_cnt,
  };
  int32_t encode_len = encode(
      hdr, 3, send_buf + 1, SEND_BUF_LEN - 2);
  if (encode_len < 0)
  {
    dump_ptr = 0;
    return;
  }

  const int32_t encode_len2 = encode(
      (uint8_t*)dump_ptr, size,
      send_buf + 1 + encode_len, SEND_BUF_LEN - 2 - encode_len);
  if (encode_len2 < 0)
  {
    dump_ptr = 0;
    return;
  }
  encode_len += encode_len2;

  send_buf[encode_len + 1] = COMMUNICATION_END_BYTE;
  send_buf_pos = encode_len + 2;

  flush();
  dump_cnt++;
  dump_size -= size;

  if (dump_size <= 0)
    dump_ptr = 0;
  else
    dump_ptr += size;
}
