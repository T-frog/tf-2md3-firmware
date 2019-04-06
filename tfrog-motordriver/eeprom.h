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

#ifndef __EEPROM_H__
#define __EEPROM_H__

void msleep(int ms);
void EEPROM_Init();
int EEPROM_Read(int addr, void* data, int len);
int EEPROM_Write(int addr, void* data, int len);

typedef struct _Tfrog_EEPROM_data
{
  unsigned int key;
  unsigned short size;
  unsigned short stored_param_version;
  unsigned int serial_no;
  char robot_name[32];
  unsigned short PWM_resolution;
  unsigned short PWM_deadtime;
  unsigned char id485;
  unsigned char stored_data;
  unsigned char buz_lvl;
  unsigned char high_frequency_encoder;
  unsigned char rely_hall;
  char __endbyte;  // must be at the end of the struct to detect actual struct size
} Tfrog_EEPROM_data;

#define TFROG_EEPROM_DATA_SIZE ((int)(&(((Tfrog_EEPROM_data*)NULL)->__endbyte)))

#define TFROG_EEPROM_ROBOTPARAM_ADDR 0x100

// Increment if Tfrog_EEPROM_data struct is destructively changed.
// Basically, don't change it destructively, but add new field at the end.
#define TFROG_EEPROM_KEY 0x00AA7701
// Increment if MotorParam, DriverParam struct is changed.
#define TFROG_EEPROM_PARAM_VERSION 0x0002

#define TFROG_EEPROM_DEFAULT        \
  {                                 \
    TFROG_EEPROM_KEY,               \
        TFROG_EEPROM_DATA_SIZE,     \
        TFROG_EEPROM_PARAM_VERSION, \
        0x01300000,                 \
        { "unknown" },              \
        1200,                       \
        20,                         \
        0,                          \
        0,                          \
        0,                          \
        0,                          \
        0,                          \
  }

#define TFROG_EEPROM_DATA_TEXT 0
#define TFROG_EEPROM_DATA_BIN 1
#define TFROG_EEPROM_DATA_BIN_SAVING 2
#define TFROG_EEPROM_DATA_BIN_LOCKED 3

#endif
