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
#include <string.h>
#include <utility/trace.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <flash/flashd.h>
#include <usart/usart.h>

#include "communication.h"
#include "registerFPGA.h"
#include "controlVelocity.h"
#include "controlPWM.h"
#include "eeprom.h"
#include "io.h"

#include "crc16.h"

unsigned char send_buf[SEND_BUF_LEN];
volatile unsigned long send_buf_pos = 0;
unsigned char send_buf485_[2][SEND_BUF_LEN];
unsigned char* send_buf485 = (unsigned char*)&send_buf485_[0][0];
volatile unsigned long send_buf_pos485 = 0;
unsigned char receive_buf[RECV_BUF_LEN];
volatile int w_receive_buf = 0;
volatile int r_receive_buf = 0;
unsigned char receive_buf485[RECV_BUF_LEN];
volatile int w_receive_buf485 = 0;
volatile int r_receive_buf485 = 0;
extern const Pin pinPWMEnable;
extern Tfrog_EEPROM_data saved_param;
extern volatile char rs485_timeout;
extern volatile short tic;
extern volatile unsigned char usb_read_pause;
volatile int usb_timeout_cnt = 0;

unsigned short crc16(unsigned char* buf, int len) RAMFUNC;
unsigned char crc8(unsigned char* buf, int len) RAMFUNC;
int add_crc_485(unsigned char* buf, int len) RAMFUNC;
char verify_crc_485(unsigned char* buf, int len) RAMFUNC;
int send(char* buf) RAMFUNC;
int nsend(char* buf, int len) RAMFUNC;
void flush(void) RAMFUNC;
void flush485(void) RAMFUNC;
int encode(unsigned char* src, int len, unsigned char* dst, int buf_max) RAMFUNC;
int decord(unsigned char* src, int len, unsigned char* dst, int buf_max) RAMFUNC;
short rs485_timeout_wait(unsigned char t, unsigned short timeout) RAMFUNC;
int data_send(short* cnt, short* pwm, char* en, short* analog, unsigned short analog_mask) RAMFUNC;
int data_send485(short* cnt, short* pwm, char* en, short* analog, unsigned short analog_mask) RAMFUNC;
int data_fetch_(unsigned char* receive_buf,
                volatile int* w_receive_buf, volatile int* r_receive_buf,
                unsigned char* data, int len) RAMFUNC;
int command_analyze(unsigned char* data, int len) RAMFUNC;

unsigned short crc16(unsigned char* buf, int len)
{
  unsigned short ret = 0;
  unsigned char* pos;

  for (pos = buf; len; len--)
  {
    ret = (ret >> 8) ^ crc16_tb[(ret ^ (*pos)) & 0x00FF];
    pos++;
  }
  return ret;
}

int add_crc_485(unsigned char* buf, int len)
{
  unsigned short crc = crc16(buf, len);
  buf[len] = crc & 0xFF;
  len++;
  buf[len] = crc >> 8;
  len++;
  buf[len] = 0xAA;
  len++;
  return len;
}

char verify_crc_485(unsigned char* buf, int len)
{
  if (crc16(buf, len - 2) == (buf[len - 2] | buf[len - 1] << 8))
    return 1;
  return 0;
}

int hextoi(char* buf)
{
  int ret;
  ret = 0;
  for (; *buf;)
  {
    if ('0' <= *buf && *buf <= '9')
    {
      ret *= 16;
      ret += *buf - '0';
    }
    else if ('A' <= *buf && *buf <= 'F')
    {
      ret *= 16;
      ret += *buf - 'A' + 0xA;
    }
    buf++;
  }
  return ret;
}

int atoi(char* buf)
{
  int ret;
  ret = 0;
  for (; *buf;)
  {
    if ('0' <= *buf && *buf <= '9')
    {
      ret *= 10;
      ret += *buf - '0';
    }
    buf++;
  }
  return ret;
}

int nhex(char* buf, int data, int len)
{
  int i;
  for (i = 0; i < len; i++)
  {
    *buf = (((unsigned int)data >> ((len - i - 1) * 4)) & 0xF) + '0';
    if (*buf > '9')
    {
      *buf = *buf - '9' - 1 + 'A';
    }
    buf++;
  }
  *buf = 0;
  return len;
}

int itoa10(char* buf, int data)
{
  int i;
  int len;
  char txt[16];
  int sign = 0;

  if (data < 0)
  {
    sign = 1;
    data = -data;
    buf[0] = '-';
  }
  for (i = 0; data; i++)
  {
    txt[i] = data % 10 + '0';
    data = data / 10;
  }
  if (i == 0)
  {
    txt[i] = '0';
    i++;
  }
  len = i;
  for (i = 0; i < len; i++)
  {
    buf[sign + len - i - 1] = txt[i];
  }
  buf[sign + len] = 0;
  return len;
}

int send(char* buf)
{
  int i = 0;
  for (; *buf; buf++)
  {
    send_buf[send_buf_pos] = (unsigned char)(*buf);
    send_buf_pos++;
    if (send_buf_pos >= SEND_BUF_LEN || *buf == '\n')
      flush();
    i++;
  }
  return i;
}

int nsend(char* buf, int len)
{
  int i;
  for (i = 0; i < len && *buf; i++, buf++)
  {
    send_buf[send_buf_pos] = (unsigned char)(*buf);
    send_buf_pos++;
    if (send_buf_pos >= SEND_BUF_LEN - 1 || *buf == '\n')
      flush();
  }
  return i;
}

void flush(void)
{
  const int len = send_buf_pos;
  char ret;

  if (len == 0)
    return;

  const unsigned short tic_start = tic;
  while ((unsigned short)(tic - tic_start) < 46)
  {
    ret = CDCDSerialDriver_Write(send_buf, len, 0, 0);
    if (ret == USBD_STATUS_LOCKED)
    {
      continue;
    }
    else if (ret != USBD_STATUS_SUCCESS)
    {
      printf("USB:w err (%d)\n\r", ret);
      send_buf_pos = 0;
      return;
    }
    else
    {
      send_buf_pos -= len;
      return;
    }
  }
  usb_timeout_cnt++;
  send_buf_pos = 0;
}
void flush485(void)
{
  if (send_buf_pos485 == 0)
    return;

  const unsigned short tic_start = tic;
  while ((unsigned short)(tic - tic_start) < 10)
  {
    if (USART_WriteBuffer(AT91C_BASE_US0, send_buf485, send_buf_pos485))
    {
      if (send_buf485 == (unsigned char*)&send_buf485_[0][0])
        send_buf485 = (unsigned char*)&send_buf485_[1][0];
      else
        send_buf485 = (unsigned char*)&send_buf485_[0][0];

      send_buf_pos485 = 0;
      rs485_timeout = 0;
      return;
    }
  }
  printf("485:w timeout\n\r");
  send_buf_pos485 = 0;
  rs485_timeout = 0;
}

/**
 * @brief エンコード
 */
int encode(unsigned char* src, int len, unsigned char* dst, int buf_max)
{
  static int pos, s_pos, w_pos;
  static unsigned short b;
  pos = 0;    // read position
  w_pos = 0;  // write_position
  s_pos = 0;
  b = 0;

  while (pos < len || s_pos >= 6)
  {
    if (s_pos >= 6)
    {
      dst[w_pos] = ((b >> 10) & 0x3f) + 0x40;
      w_pos++;
      if (w_pos >= buf_max)
        return (-1);
      b = b << 6;
      s_pos -= 6;
    }
    else
    {
      b |= src[pos] << (8 - s_pos);
      s_pos += 8;
      pos++;
      if (pos >= len)
        s_pos += 4;  // 最後
    }
  }

  if (w_pos >= buf_max)
    return (-1);

  return w_pos;
}

/**
 * @brief デコード
 * @param src[in] デコードする文字列
 * @param len[in] デコードする文字列の長さ
 * @param dst[out] デコード後のデータ
 * @param buf_max[in] デコード後のデータバッファのサイズ
 * @return デコード後のバイト数
 */
int decord(unsigned char* src, int len, unsigned char* dst, int buf_max)
{
  static unsigned short dat, b;
  static int s_pos, w_pos;
  static int rerr;

  w_pos = 0;  // write_position
  s_pos = 0;  // shift position
  rerr = 0;
  dat = 0;
  b = 0;
  while (len)
  {
    if (*src >= 0x40)
      b = *src - 0x40;
    else
      rerr++;

    dat |= (b << (10 - s_pos));
    s_pos += 6;
    if (s_pos >= 8)
    {
      dst[w_pos] = (dat >> 8);
      w_pos++;
      if (w_pos >= buf_max)
        return 0;
      s_pos -= 8;
      dat = dat << 8;
    }
    src++;
    len--;
  }

  if (rerr)
    return -rerr;
  return w_pos;
}

short rs485_timeout_wait(unsigned char t, unsigned short timeout)
{
  const unsigned short tic_start = tic;
  while (rs485_timeout < t)
  {
    if ((unsigned short)(tic - tic_start) > timeout)
      return 0;
  }
  return 1;
}

int data_send(short* cnt, short* pwm, char* en, short* analog, unsigned short analog_mask)
{
  unsigned char data[34];
  int len, encode_len;
  len = data_pack(cnt, pwm, en, analog, analog_mask, data);

  send_buf_pos = 0;
  send_buf[0] = COMMUNICATION_START_BYTE;
  encode_len = encode((unsigned char*)data, len, send_buf + 1, SEND_BUF_LEN - 2);
  if (encode_len < 0)
    return encode_len;
  send_buf[encode_len + 1] = COMMUNICATION_END_BYTE;
  send_buf_pos = encode_len + 2;

  flush();
  return encode_len;
}
int data_send485(short* cnt, short* pwm, char* en, short* analog, unsigned short analog_mask)
{
  unsigned char data[34];
  int len, encode_len;
  len = data_pack(cnt, pwm, en, analog, analog_mask, data);

  send_buf485[0] = 0xAA;
  send_buf_pos485 = 1;

  unsigned char* buf;
  int buf_len;
  buf = &send_buf485[send_buf_pos485];
  buf_len = 0;

  buf[0] = COMMUNICATION_START_BYTE;
  buf[1] = saved_param.id485 + 0x40;
  if (driver_state.ifmode == 0)
    buf[1] = 0x40;
  buf[2] = 0x40 - 1;
  buf_len = 3;

  encode_len = encode((unsigned char*)data, len, buf + buf_len,
                      SEND_BUF_LEN - send_buf_pos485 - buf_len - 3);
  if (encode_len < 0)
    return encode_len;

  buf_len += encode_len;
  buf[buf_len] = COMMUNICATION_END_BYTE;
  buf_len++;

  buf_len = add_crc_485(buf, buf_len);
  send_buf_pos485 += buf_len;

  //printf("send485\n\r");
  if (rs485_timeout_wait(saved_param.id485 * 4 + 4, 32))
  {
    flush485();
    return encode_len;
  }
  else
  {
    send_buf_pos485 = 0;
    rs485_timeout = 0;
    printf("485:skip\n\r");
    return -1;
  }
}

int int_send(const char param, const char id, const int value)
{
  unsigned char data[8];
  int len, encode_len;
  data[0] = param;
  data[1] = id;
  Integer4 v;
  v.integer = value;
  data[2] = v.byte[3];
  data[3] = v.byte[2];
  data[4] = v.byte[1];
  data[5] = v.byte[0];
  len = 6;

  send_buf_pos = 0;
  send_buf[0] = COMMUNICATION_INT_BYTE;
  encode_len = encode((unsigned char*)data, len, send_buf + 1, SEND_BUF_LEN - 2);
  if (encode_len < 0)
    return encode_len;
  send_buf[encode_len + 1] = COMMUNICATION_END_BYTE;
  send_buf_pos = encode_len + 2;

  flush();
  return encode_len;
}
int int_send485(const char param, const char id, const int value)
{
  unsigned char data[8];
  int len, encode_len;
  data[0] = param;
  data[1] = id;
  Integer4 v;
  v.integer = value;
  data[2] = v.byte[3];
  data[3] = v.byte[2];
  data[4] = v.byte[1];
  data[5] = v.byte[0];
  len = 6;

  send_buf485[0] = 0xAA;
  send_buf_pos485 = 1;

  unsigned char* buf;
  int buf_len;
  buf = &send_buf485[send_buf_pos485];
  buf_len = 0;

  buf[0] = COMMUNICATION_INT_BYTE;
  buf[1] = saved_param.id485 + 0x40;
  if (driver_state.ifmode == 0)
    buf[1] = 0x40;
  buf[2] = 0x40 - 1;
  buf_len = 3;

  encode_len = encode((unsigned char*)data, len, buf + buf_len,
                      SEND_BUF_LEN - send_buf_pos485 - buf_len - 3);
  if (encode_len < 0)
    return encode_len;

  buf_len += encode_len;
  buf[buf_len] = COMMUNICATION_END_BYTE;
  buf_len++;

  buf_len = add_crc_485(buf, buf_len);
  send_buf_pos485 += buf_len;

  //printf("send485\n\r");
  if (rs485_timeout_wait(saved_param.id485 * 4 + 4, 32))
  {
    flush485();
    return encode_len;
  }
  else
  {
    send_buf_pos485 = 0;
    rs485_timeout = 0;
    printf("485:skip\n\r");
    return -1;
  }
}

/* オドメトリデータの送信 */
inline int data_pack(short* cnt, short* pwm, char* en, short* analog, unsigned short analog_mask, unsigned char* data)
{
  static int i;
  int len = 0;

  for (i = 0; i < COM_MOTORS; i++)
  {
    if (!en[i])
      continue;
    data[len++] = ((Integer2)cnt[i]).byte[1];
    data[len++] = ((Integer2)cnt[i]).byte[0];
  }
  for (i = 0; i < COM_MOTORS; i++)
  {
    if (!en[i])
      continue;
    data[len++] = ((Integer2)pwm[i]).byte[1];
    data[len++] = ((Integer2)pwm[i]).byte[0];
  }

  for (i = 0; analog_mask != 0; analog_mask = analog_mask >> 1, i++)
  {
    if (analog_mask & 1)
    {
      data[len] = ((Integer2)analog[i]).byte[1];
      data[len + 1] = ((Integer2)analog[i]).byte[0];
      len += 2;
    }
  }
  return len;
}

int data_fetch(unsigned char* data, int len)
{
  return data_fetch_(receive_buf,
                     &w_receive_buf, &r_receive_buf,
                     data, len);
}
int data_fetch485(unsigned char* data, int len)
{
  return data_fetch_(receive_buf485,
                     &w_receive_buf485, &r_receive_buf485,
                     data, len);
}
int buf_left()
{
  int buf_left;

  buf_left = r_receive_buf - w_receive_buf;
  if (buf_left <= 0)
    buf_left += RECV_BUF_LEN;
  buf_left--;

  return buf_left;
}
int data_fetch_(unsigned char* receive_buf,
                volatile int* w_receive_buf, volatile int* r_receive_buf,
                unsigned char* data, int len)
{
  int buf_left;

  buf_left = *r_receive_buf - *w_receive_buf;
  if (buf_left <= 0)
    buf_left += RECV_BUF_LEN;
  buf_left--;

  if (buf_left > len)
  {
    for (; len; len--)
    {
      receive_buf[*w_receive_buf] = *data;
      (*w_receive_buf)++;
      data++;
      if (*w_receive_buf >= RECV_BUF_LEN)
      {
        *w_receive_buf = 0;
      }
    }
    return 0;
  }
  return len;
}

static inline int data_analyze_(
    unsigned char* receive_buf,
    volatile int* w_receive_buf, volatile int* r_receive_buf, int fromto)
{
  static unsigned char line_full[64 + 3];
  static unsigned char* line = line_full + 3;
  unsigned char* data;
  int r_buf, len, w_buf;
  short from = -1, to = -1;
  short id = saved_param.id485;
  enum
  {
    STATE_IDLE,
    STATE_FROM,
    STATE_TO,
    STATE_RECIEVING,
    STATE_RECIEVED,
    STATE_CRC16_1,
    STATE_CRC16_2,
  } state = STATE_IDLE;
  enum
  {
    ISOCHRONOUS,
    INTERRUPT
  } mode = ISOCHRONOUS;

  if (!fromto)
  {
    id = 0;
  }

  r_buf = *r_receive_buf;
  w_buf = *w_receive_buf;
  data = &receive_buf[r_buf];
  len = 0;
  for (;;)
  {
    char receive_period = 0;
    if (r_buf == w_buf)
      break;

    line[len] = *data;
    len++;

    if (len > 63)
    {
      printf("COM:rbuf ovf %d\n\r", len);
      len = 0;
      receive_period = 1;
      state = STATE_IDLE;
    }

    switch (state)
    {
      case STATE_IDLE:
        if (*data == COMMUNICATION_START_BYTE ||
            *data == COMMUNICATION_INT_BYTE)
        {
          len = 0;
          if (fromto)
          {
            state = STATE_FROM;
            line_full[0] = *data;
          }
          else
          {
            state = STATE_RECIEVING;
            from = -1;
            to = id;
          }
          if (*data == COMMUNICATION_START_BYTE)
            mode = ISOCHRONOUS;
          else
            mode = INTERRUPT;
        }
        else if (*data == COMMUNICATION_END_BYTE)
        {
          line[len - 1] = 0;

          extended_command_analyze((char*)line);
          len = 0;
          receive_period = 1;
          state = STATE_IDLE;
        }
        break;
      case STATE_FROM:
        from = (*data) - 0x40;
        line_full[1] = *data;
        state = STATE_TO;
        break;
      case STATE_TO:
        to = (*data) - 0x40;
        line_full[2] = *data;
        state = STATE_RECIEVING;
        len = 0;
        break;
      case STATE_RECIEVING:
        if (*data == COMMUNICATION_START_BYTE ||
            *data == COMMUNICATION_INT_BYTE)
        {
          len = 0;
          if (fromto)
          {
            state = STATE_FROM;
            line_full[0] = *data;
          }
          else
          {
            state = STATE_RECIEVING;
            from = -1;
            to = id;
          }
          if (*data == COMMUNICATION_START_BYTE)
            mode = ISOCHRONOUS;
          else
            mode = INTERRUPT;
        }
        if (*data == COMMUNICATION_END_BYTE)
        {
          if (fromto)
            state = STATE_CRC16_1;
          else
            state = STATE_RECIEVED;
        }
        break;
      case STATE_RECIEVED:
        break;
      case STATE_CRC16_1:
        state = STATE_CRC16_2;
        break;
      case STATE_CRC16_2:
        if (!(to == id || (id == 0 && to == -1)))
        {
          state = STATE_IDLE;
          receive_period = 1;
          //printf( "not for me\n\r" );
        }
        else if (verify_crc_485(line_full, len + 3))
        {
          state = STATE_RECIEVED;
          len -= 2;
        }
        else
        {
          state = STATE_IDLE;
          receive_period = 1;
          printf("COM:CRC err (%d)\n\r", len);
        }
        break;
    }
    if (state == STATE_RECIEVED)
    {
      static unsigned char rawdata[16];
      int data_len;

      if (to == id)
      {
        data_len = decord(line, len - 1, rawdata, 16);
        if (data_len < 6)
        {
          printf("COM:decode err (%d)\n\r", data_len);
        }
        else
        {
          unsigned char imotor = rawdata[1];
          if (id * 2 <= imotor && imotor <= id * 2 + 1)
          {
            rawdata[1] = rawdata[1] & 1;

            command_analyze(rawdata, data_len);
            driver_state.ifmode = fromto;
            //printf("for me\n\r");
          }
          else if (from == -1)
          {
            if (rawdata[0] == PARAM_servo)
              com_en[imotor] = 1;
            // Forward from USB(id: 0) to RS485(id: imotor/2)
            if (send_buf_pos485 == 0)
            {
              send_buf485[send_buf_pos485] = 0xAA;
              send_buf_pos485++;
            }
            unsigned char* buf;
            int buf_len;
            buf = &send_buf485[send_buf_pos485];
            buf_len = 0;
            buf[0] = COMMUNICATION_START_BYTE;
            buf[1] = 0 + 0x40;
            buf[2] = imotor / 2 + 0x40;
            int i;
            for (i = 0; i < len; i++)
            {
              buf[3 + i] = line[i];
            }
            buf_len = len + 3;
            buf_len = add_crc_485(buf, buf_len);
            send_buf_pos485 += buf_len;

            if (send_buf_pos485 > SEND_BUF_LEN - 16)
            {
              if (rs485_timeout_wait(4, 32))
              {
                flush485();
              }
              else
              {
                send_buf_pos485 = 0;
                rs485_timeout = 0;
                printf("485:skip\n\r");
              }
            }
            else
            {
              send_buf_pos485--;
            }
          }
        }
      }
      else if (id == 0 && to == -1 &&
               0 < from && from < COM_MOTORS / 2)
      {
        // Forward packet from RS485 to USB
        data_len = decord(line, len - 1, rawdata, 16);
        if (mode == ISOCHRONOUS)
        {
          Integer2 tmp;
          int i = 0, j;
          for (j = 0; j < 2; j++)
          {
            tmp.byte[1] = rawdata[i++];
            tmp.byte[0] = rawdata[i++];
            com_cnts[from * 2 + j] = tmp.integer;
          }
          for (j = 0; j < 2; j++)
          {
            tmp.byte[1] = rawdata[i++];
            tmp.byte[0] = rawdata[i++];
            com_pwms[from * 2 + j] = tmp.integer;
          }
        }
        else
        {
          Integer4 value;
          value.byte[0] = rawdata[5];
          value.byte[1] = rawdata[4];
          value.byte[2] = rawdata[3];
          value.byte[3] = rawdata[2];
          int_send(rawdata[0], from * 2 + rawdata[1], value.integer);
        }
      }
      len = 0;
      receive_period = 1;
      state = STATE_IDLE;
    }

    if (len == 0 && !receive_period)
    {
      // Line buffer is cleared. Move receive pointer to the current position.
      *r_receive_buf = r_buf;
    }
    data++;
    r_buf++;
    if (r_buf >= RECV_BUF_LEN)
    {
      r_buf = 0;
      data = receive_buf;
    }
    if (receive_period)
    {
      // Line is end. Move receive pointer to the next position.
      *r_receive_buf = r_buf;
    }
  }
  if (send_buf_pos485 > 0)
  {
    if (rs485_timeout_wait(4, 32))
    {
      flush485();
    }
    else
    {
      send_buf_pos485 = 0;
      rs485_timeout = 0;
      printf("485:skip\n\r");
    }
  }
  return 0;
}

int data_analyze()
{
  return data_analyze_(receive_buf, &w_receive_buf, &r_receive_buf, 0);
}
int data_analyze485()
{
  return data_analyze_(receive_buf485, &w_receive_buf485, &r_receive_buf485, 1);
}

int ext_continue = -1;
// //////////////////////////////////////////////////
/* 受信したYPSpur拡張コマンドの解析 */
int extended_command_analyze(char* data)
{
  static int i;

  if (motor[0].servo_level != SERVO_LEVEL_STOP ||
      motor[1].servo_level != SERVO_LEVEL_STOP)
    return 0;

  if (ext_continue >= 0)
  {
    char val[10];
    int len;
    int wrote;

    if (data[0] == 0)
    {
      char zero = 0;
      msleep(5);
      EEPROM_Write(TFROG_EEPROM_ROBOTPARAM_ADDR + ext_continue,
                   &zero, 1);
      send(data);
      send("\n00P\n");
      itoa10(val, ext_continue);
      send(val);
      send(" bytes saved\n\n");
      flush();
      ext_continue = -1;
      saved_param.stored_data = TFROG_EEPROM_DATA_TEXT;
      EEPROM_Write(0, &saved_param, sizeof(saved_param));
      return 1;
    }

    len = strlen(data);
    data[len] = '\n';
    msleep(5);
    wrote = EEPROM_Write(TFROG_EEPROM_ROBOTPARAM_ADDR + ext_continue,
                         data, len + 1);
    data[len] = 0;
    if (wrote < 0)
    {
      char zero = 0;
      msleep(5);
      EEPROM_Write(TFROG_EEPROM_ROBOTPARAM_ADDR + ext_continue,
                   &zero, 1);
      send(data);
      send("\n01Q\nFailed (");
      itoa10(val, ext_continue);
      send(val);
      send(" bytes saved)\n\n");
      flush();
      ext_continue = -1;
      return 0;
    }

    ext_continue += len + 1;
    return 1;
  }

  if (strstr(data, "VV") == data)
  {
    char val[10];
    send(data);
    send("\n00P\nVEND:");
    send(YP_VENDOR_NAME);
    send("; \nPROD:");
    send(BOARD_NAME);
    send("; \nFIRM:");
    send(YP_FIRMWARE_NAME);
    send("; \nPROT:");
    send(YP_PROTOCOL_NAME);
    send("; \nSERI:");
    nhex(val, saved_param.serial_no, 8);
    send(val);
    send("; \nID:");
    itoa10(val, saved_param.id485);
    send(val);
    send("; \n\n");
  }
  else if (strstr(data, "PP") == data)
  {
    char val[10];
    char sep[2] = { 0 };
    send(data);
    send("\n00P\nNAME:");
    send(saved_param.robot_name);
    send("; \nPWMRES:");
    itoa10(val, saved_param.PWM_resolution);
    send(val);
    send("; \nMOTORNUM:");
    send(YP_DRIVERPARAM_MOTORNUM);
    send("; \nDEADTIME:");
    itoa10(val, saved_param.PWM_deadtime);
    send(val);
    send("; \nPARAM:");
    if (saved_param.stored_data == TFROG_EEPROM_DATA_TEXT)
      send("text");
    else if (saved_param.stored_data == TFROG_EEPROM_DATA_BIN)
      send("binary");
    else if (saved_param.stored_data == TFROG_EEPROM_DATA_BIN_LOCKED)
      send("binary locked");
    else if (saved_param.stored_data == TFROG_EEPROM_DATA_BIN_SAVING)
      send("binary saving");
    send("; \nOPTIONS:");
    if (saved_param.rely_hall > 0)
    {
      send("RELYHALL");
      sep[0] = ',';
      sep[1] = 0;
    }
    if (saved_param.high_frequency_encoder > 0)
    {
      send(sep);
      send("HFREQENC");
      sep[0] = ',';
      sep[1] = 0;
    }
    send(sep);
    send("BUZ_LVL");
    itoa10(val, saved_param.buz_lvl);
    send(val);
    send("; \n\n");
  }
  else if (strstr(data, "$LOCKPARAM") == data)
  {
    if (saved_param.stored_data == TFROG_EEPROM_DATA_BIN)
    {
      saved_param.stored_data = TFROG_EEPROM_DATA_BIN_LOCKED;
      if (EEPROM_Write(0, &saved_param, sizeof(saved_param)) < 0)
      {
        send(data);
        send("\n01Q\n\n");
      }
      else
      {
        send(data);
        send("\n00P\n\n");
      }
    }
    else
    {
      send(data);
      send("\n02R\n\n");
    }
  }
  else if (strstr(data, "$KEEPPARAM") == data)
  {
    saved_param.stored_data = TFROG_EEPROM_DATA_BIN_SAVING;
    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "GETEMBEDDEDPARAM") == data)
  {
    char epval[256];
    int len;
    int i;

    send(data);
    send("\n");
    len = EEPROM_Read(TFROG_EEPROM_ROBOTPARAM_ADDR, epval, 256);
    if (len < 0)
    {
      send("01Q\n\n");
    }
    else
    {
      send("00P\n");
      flush();
      if (saved_param.stored_data == TFROG_EEPROM_DATA_TEXT)
      {
        for (i = 0; i < 1792; i += 256)
        {
          len = EEPROM_Read(TFROG_EEPROM_ROBOTPARAM_ADDR + i, epval, 256);
          if (nsend(epval, 256) < 256)
            break;
          AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
          flush();
        }
      }
      send("\n\n");
    }
  }
  else if (strstr(data, "SETEMBEDDEDPARAM") == data)
  {
    ext_continue = 0;
    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$EEPROMDUMP") == data)
  {
    char val[3];
    char epval[256];
    int i, j;

    send(data);
    send("\n00P\n");

    for (j = 0; j < 8; j++)
    {
      EEPROM_Read(256 * j, epval, 256);
      for (i = 0; i < 256; i++)
      {
        nhex(val, epval[i], 2);
        send(val);
      }
      send("\n");
      if (j == 7)
        break;
      flush();
      AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
    }
    send("\n");
  }
  else if (strstr(data, "$FLASHERACE") == data)
  {
    static int erace_flag = 0;
    if (erace_flag == 0 && data[11] == 'A')
    {
      erace_flag = 1;
      send(data);
      send("\n00P\n\n");
    }
    else if (erace_flag == 1 && data[11] == 'B')
    {
      erace_flag = 0;
      send(data);
      send("\n00P\n\n");
      flush();
      FLASHD_ClearGPNVM(2);
      AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST | AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST;
      while (1)
        ;
    }
    else
    {
      erace_flag = 0;
      send(data);
      send("\n01Q\n\n");
    }
  }
  else if (strstr(data, "$EEPROMERACE") == data)
  {
    int i;
    char clear[16];

    for (i = 0; i < 16; i++)
      clear[i] = 0xFF;

    for (i = 0; i < 2048; i += 16)
    {
      msleep(5);
      EEPROM_Write(i, &clear, 16);
      AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
    }

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$SETID") == data)
  {
    saved_param.id485 = atoi(data + 6);

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$SETSERIALNO") == data)
  {
    saved_param.serial_no = hextoi(data + 12);

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$HFREQENC") == data)
  {
    saved_param.high_frequency_encoder = atoi(data + 9);

    if (saved_param.high_frequency_encoder)
    {
      THEVA.GENERAL.ENCODER.HFREQ = 1;
    }
    else
    {
      THEVA.GENERAL.ENCODER.HFREQ = 0;
    }

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$RELYHALL") == data)
  {
    saved_param.rely_hall = hextoi(data + 9);

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$SETNAME") == data)
  {
    strcpy(saved_param.robot_name, data + 8);

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$SETPWMRESOLUTION") == data)
  {
    saved_param.PWM_resolution = atoi(data + 17);

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$SETPWMDEADTIME") == data)
  {
    saved_param.PWM_deadtime = atoi(data + 15);

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$SETBUZZERLEVEL") == data)
  {
    saved_param.buz_lvl = atoi(data + 15);
    if (saved_param.buz_lvl > 4)
      saved_param.buz_lvl = 4;

    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "$EEPROMSAVE") == data)
  {
    if (EEPROM_Write(0, &saved_param, sizeof(saved_param)) < 0)
    {
      send(data);
      send("\n01Q\n\n");
    }
    else
    {
      send(data);
      send("\n00P\n\n");
    }
  }
  else if (strstr(data, "$ENC0") == data)
  {
    char num[16];

    send(data);
    send("\nANG0  ");
    nhex(num, motor_param[0].enc0tran, 4);
    num[4] = 0;
    send(num);
    send(",");
    nhex(num, motor_param[1].enc0tran, 4);
    num[4] = 0;
    send(num);
    send("\n");
    send("ANG0T ");
    nhex(num, motor_param[0].enc0, 4);
    num[4] = 0;
    send(num);
    send(",");
    nhex(num, motor_param[1].enc0, 4);
    num[4] = 0;
    send(num);
    send("\n\n");
  }
  else if (strstr(data, "$TESTENC") == data)
  {
    unsigned short tmp;
    char num[16];

    send(data);
    send("\nHALL");
    tmp = THEVA.MOTOR[0].ROT_DETECTER.HALL;
    if (tmp & HALL_U)
      num[0] = 'U';
    else
      num[0] = '-';
    if (tmp & HALL_V)
      num[1] = 'V';
    else
      num[1] = '-';
    if (tmp & HALL_W)
      num[2] = 'W';
    else
      num[2] = '-';
    if (tmp & HALL_Z)
      num[3] = 'Z';
    else
      num[3] = '-';
    num[4] = 0;
    send(num);
    send(",");
    tmp = THEVA.MOTOR[1].ROT_DETECTER.HALL;
    if (tmp & HALL_U)
      num[0] = 'U';
    else
      num[0] = '-';
    if (tmp & HALL_V)
      num[1] = 'V';
    else
      num[1] = '-';
    if (tmp & HALL_W)
      num[2] = 'W';
    else
      num[2] = '-';
    if (tmp & HALL_Z)
      num[3] = 'Z';
    else
      num[3] = '-';
    num[4] = 0;
    send(num);
    send("\n");

    send("ANG ");
    nhex(num, THEVA.MOTOR[0].ENCODER, 4);
    num[4] = 0;
    send(num);
    send(",");
    nhex(num, THEVA.MOTOR[1].ENCODER, 4);
    num[4] = 0;
    send(num);
    send("\n");

    send("SPD ");
    nhex(num, THEVA.MOTOR[0].SPEED, 4);
    num[4] = 0;
    send(num);
    send(",");
    nhex(num, THEVA.MOTOR[1].SPEED, 4);
    num[4] = 0;
    send(num);
    send("\n");

    send("O_R ");
    nhex(num, THEVA.MOTOR[0].INDEX_RISE_ANGLE, 4);
    num[4] = 0;
    send(num);
    send(",");
    nhex(num, THEVA.MOTOR[1].INDEX_RISE_ANGLE, 4);
    num[4] = 0;
    send(num);
    send("\n");

    send("O_F ");
    nhex(num, THEVA.MOTOR[0].INDEX_FALL_ANGLE, 4);
    num[4] = 0;
    send(num);
    send(",");
    nhex(num, THEVA.MOTOR[1].INDEX_FALL_ANGLE, 4);
    num[4] = 0;
    send(num);
    send("\n");

    send("\n\n");
  }
  else if (strstr(data, "ADMASK") == data)
  {
    unsigned char tmp;

    tmp = 0;
    for (i = 6; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++)
    {
      tmp = tmp << 1;
      if (data[i] == '1')
        tmp |= 1;
    }
    driver_state.admask = tmp;
    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "SETIODIR") == data)
  {
    unsigned char tmp;

    tmp = 0;
    for (i = 8; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++)
    {
      tmp = tmp << 1;
      if (data[i] == '1')
        tmp |= 1;
    }
    set_io_dir(tmp);
    driver_state.io_dir = tmp;
    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "GETIOVAL") == data)
  {
    unsigned short tmp;
    char num[3];
    tmp = get_io_data();
    send(data);
    send("\n");
    if ((tmp >> 4) > 9)
    {
      num[0] = (tmp >> 4) - 10 + 'A';
    }
    else
    {
      num[0] = (tmp >> 4) + '0';
    }
    if ((tmp & 0xF) > 9)
    {
      num[1] = (tmp & 0xF) - 10 + 'A';
    }
    else
    {
      num[1] = (tmp & 0xF) + '0';
    }
    num[2] = 0;
    send(num);
    send(" \n\n");
  }
  else if (strstr(data, "GETIO") == data)
  {
    switch (data[5])
    {
      case '1':
        driver_state.io_mask[0] = 0xFF;
        driver_state.io_mask[1] = 0;
        break;
      case '2':
        driver_state.io_mask[1] = 0xFF;
        driver_state.io_mask[0] = 0;
        break;
      case '3':
        driver_state.io_mask[0] = 0xFF;
        driver_state.io_mask[1] = 0xFF;
        break;
      default:
        driver_state.io_mask[0] = 0;
        driver_state.io_mask[1] = 0;
        break;
    }
    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "OUTPUT") == data)
  {
    unsigned char tmp;

    tmp = 0;
    for (i = 6; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++)
    {
      tmp = tmp << 1;
      if (data[i] == '1')
        tmp |= 1;
    }
    set_io_data(tmp);
    send(data);
    send("\n00P\n\n");
  }
  else if (strstr(data, "SS") == data)
  {
    int tmp;

    tmp = 0;
    for (i = 2; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++)
    {
      tmp *= 10;
      tmp += data[i] - '0';
    }
    send(data);
    send("\n04T\n\n");
  }
  else
  {
    if (data[0] == 0 || data[0] == '\n' || data[0] == '\r')
    {
      flush();
    }
    else
    {
      send(data);
      send("\n0Ee\n\n");

      char* d;
      for (d = data; *d != 0; ++d)
      {
        if (*d < 0x20 || 0x7e < *d)
          *d = '?';
      }
      printf("unknown command \"%s\"\n\r", data);
    }
  }
  flush();

  return 1;
}

// //////////////////////////////////////////////////
/* 受信したコマンドの解析 */
int command_analyze(unsigned char* data, int len)
{
  static int imotor;

  static Integer4 i;

  i.byte[3] = data[2];
  i.byte[2] = data[3];
  i.byte[1] = data[4];
  i.byte[0] = data[5];

  imotor = data[1];
  if (imotor < 0 || imotor >= 2)
  {
    return 0;
  }

  //if(data[0] != PARAM_w_ref &&
  //		data[0] != PARAM_w_ref_highprec)
  //	printf("get %d %d %d\n\r",data[0],data[1],i.integer);
  char param_set = 0;
  switch (data[0])
  {
    case PARAM_w_ref:
      i.integer *= 16;
    case PARAM_w_ref_highprec:
      if (motor[imotor].ref.vel != i.integer)
        motor[imotor].ref.vel_changed = 1;
      motor[imotor].ref.vel = i.integer;
      break;
    case PARAM_p_toq_offset:
      motor[imotor].ref.torque_offset = i.integer;
      break;
    case PARAM_servo:
      if (motor[imotor].servo_level < SERVO_LEVEL_TORQUE &&
          i.integer >= SERVO_LEVEL_TORQUE)
      {
        controlPWM_config(imotor);
      }
      if (motor[imotor].servo_level != SERVO_LEVEL_VELOCITY &&
          i.integer == SERVO_LEVEL_VELOCITY)
      {
        // servo levelが速度制御に推移した
        motor[imotor].control_init = 1;
      }
      motor[imotor].servo_level = i.integer;
      break;
    case PARAM_io_dir:
      driver_state.io_dir = i.integer;
      set_io_dir(driver_state.io_dir);
      break;
    case PARAM_io_data:
      set_io_data(i.integer);
      break;
    case PARAM_ad_mask:
      driver_state.admask = i.integer;
      break;
    case PARAM_protocol_version:
      driver_state.protocol_version = i.integer;
      break;
    default:
      param_set = 1;
  }
  if (param_set == 1 && saved_param.stored_data != TFROG_EEPROM_DATA_BIN_LOCKED)
  {
    switch (data[0])
    {
      case PARAM_p_ki:
        motor_param[imotor].Kcurrent = i.integer;
        break;
      case PARAM_p_kv:
        motor_param[imotor].Kvolt = i.integer;
        break;
      case PARAM_p_fr_plus:
        motor_param[imotor].fr_plus = i.integer;
        break;
      case PARAM_p_fr_wplus:
        motor_param[imotor].fr_wplus = i.integer;
        break;
      case PARAM_p_fr_minus:
        motor_param[imotor].fr_minus = i.integer;
        break;
      case PARAM_p_fr_wminus:
        motor_param[imotor].fr_wminus = i.integer;
        break;
      case PARAM_p_inertia_self:
        motor_param[imotor].inertia_self = i.integer;
        break;
      case PARAM_p_inertia_cross:
        motor_param[imotor].inertia_cross = i.integer;
        break;
      case PARAM_p_A:
        motor_param[0].inertia_self = i.integer;
        break;
      case PARAM_p_B:
        motor_param[1].inertia_self = i.integer;
        break;
      case PARAM_p_C:
        motor_param[0].inertia_cross = i.integer;
        break;
      case PARAM_p_D:
        motor_param[1].inertia_cross = i.integer;
        break;
      case PARAM_p_E:
        break;
      case PARAM_p_F:
        break;
      case PARAM_p_pi_kp:
        motor_param[imotor].Kp = i.integer;
        break;
      case PARAM_p_pi_ki:
        motor_param[imotor].Ki = i.integer;
        break;
      case PARAM_pwm_max:
        driver_param.PWM_max = i.integer;
        break;
      case PARAM_pwm_min:
        driver_param.PWM_min = i.integer;
        break;
      case PARAM_toq_max:
        motor_param[imotor].torque_max = i.integer;
        break;
      case PARAM_toq_min:
        motor_param[imotor].torque_min = i.integer;
        break;
      case PARAM_toq_limit:
        motor_param[imotor].torque_limit = i.integer;
        break;
      case PARAM_int_max:
        motor_param[imotor].integ_max = i.integer * 16;
        break;
      case PARAM_int_min:
        motor_param[imotor].integ_min = i.integer * 16;
        break;
      case PARAM_motor_phase:
        switch (i.integer)
        {
          case 0:
            motor_param[imotor].motor_type = MOTOR_TYPE_DC;
            break;
          case -3:
          case 3:
            motor_param[imotor].motor_type = MOTOR_TYPE_AC3;
            break;
        }
        if (i.integer < 0)
          motor_param[imotor].rotation_dir = -1;
        else
          motor_param[imotor].rotation_dir = 1;
        break;
      case PARAM_watch_dog_limit:
        driver_param.watchdog_limit = i.integer;
        break;
      case PARAM_phase_offset:
        motor_param[imotor].phase_offset = i.integer;
        break;
      case PARAM_enc_rev:
        motor_param[imotor].enc_rev_raw = i.integer;
        break;
      case PARAM_enc_div:
        motor_param[imotor].enc_div = i.integer;
        break;
      case PARAM_enc_denominator:
        motor_param[imotor].enc_denominator = i.integer;
        break;
      case PARAM_enc_type:
        motor_param[imotor].enc_type = i.integer;
        break;
      case PARAM_control_cycle:
        driver_param.control_cycle = i.integer;
        controlVelocity_config();
        controlPWM_init();
        break;
      case PARAM_vsrc:
        // ad = 1024 * ( vsrc * VSRC_DIV ) / 3.3
        driver_param.vsrc_rated = 310 * ((int)i.integer * VSRC_DIV) / 256;
        break;
      default:
        return 0;
    }
  }
  driver_state.watchdog = 0;
  return 0;
}
