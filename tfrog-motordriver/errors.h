#ifndef __ERRORS_H__
#define __ERRORS_H__

const unsigned char error_pat[2][ERROR_NUM + 1] =
    {
      {
          0xCC,  // 11001100 low voltage
          0xBF,  // 10111111 hall sequence
          0xB3,  // 10110011 hall inconsistent with encoder
          0xAA,  // 10101010 watchdog
          0xDD,  // 11011101 heartbeat
          0x00,  // 00000000 none
      },
      {
          0x00,  // 00000000 enabled only on motor 0
          0xAF,  // 10101111 hall sequence
          0xA3,  // 10100011 hall inconsistent with encoder
          0x00,  // 00000000 enabled only on motor 0
          0x00,  // 00000000 enabled only on motor 0
          0x00,  // 00000000 none
      }
    };

#endif
