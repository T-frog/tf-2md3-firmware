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

#include <board.h>
#include "io.h"

#define REV4_BITSC(a) ((((a)&0x1)) |                \
                       ((((a) >> 8) & 0x7) << 1) |  \
                       ((((a) >> 14) & 0x7) << 4) | \
                       ((((a) >> 21) & 0x1) << 7))
#define REV4_BITSD(a) ((((a)&0x1)) |        \
                       (((a)&0xE) << 7) |   \
                       (((a)&0x70) << 10) | \
                       (((a)&0x80) << 14))
#define REV4_MASK 0x0021C701

void set_io_dir(unsigned char io_dir)
{
#if defined(tfrog_rev5)
  AT91C_BASE_PIOB->PIO_ODR = 0xFF000000;
  AT91C_BASE_PIOB->PIO_OER = io_dir << 24;
#elif defined(tfrog_rev4)
  // 0000 0000 0010 0001 1100 0111 0000 0001
  AT91C_BASE_PIOB->PIO_ODR = REV4_MASK;
  AT91C_BASE_PIOB->PIO_OER = REV4_BITSD(io_dir);
#endif
}

void set_io_data(unsigned char io_data)
{
#if defined(tfrog_rev5)
  AT91C_BASE_PIOB->PIO_CODR = ((~io_data) & 0xFF) << 24;
  AT91C_BASE_PIOB->PIO_SODR = ((io_data)&0xFF) << 24;
#elif defined(tfrog_rev4)
  AT91C_BASE_PIOB->PIO_CODR = REV4_BITSD(~io_data);
  AT91C_BASE_PIOB->PIO_SODR = REV4_BITSD(io_data);
#endif
}

unsigned char get_io_data()
{
#if defined(tfrog_rev5)
  return AT91C_BASE_PIOB->PIO_PDSR >> 24;
#elif defined(tfrog_rev4)
  return REV4_BITSC(AT91C_BASE_PIOB->PIO_PDSR);
#endif
  return 0;
}
