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

#include <stdint.h>

void msleep(int32_t ms);
void EEPROM_Init();
int32_t EEPROM_Read(int32_t addr, void* data, int32_t len);
int32_t EEPROM_Write(int32_t addr, void* data, int32_t len);

typedef struct _Tfrog_EEPROM_data
{
  uint32_t key;
  uint16_t size;
  uint16_t stored_param_version;
  uint32_t serial_no;
  char robot_name[32];
  uint16_t PWM_resolution;
  uint16_t PWM_deadtime;
  uint8_t id485;
  uint8_t stored_data;
  uint8_t buz_lvl;
  uint8_t high_frequency_encoder;
  uint8_t rely_hall;
  uint8_t io_dir;
  uint8_t io_data;
  char __endbyte;  // must be at the end of the struct to detect actual struct size
} Tfrog_EEPROM_data;

#define TFROG_EEPROM_DATA_SIZE ((int32_t)(&(((Tfrog_EEPROM_data*)NULL)->__endbyte)))

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
        {"unknown"},                \
        1200,                       \
        20,                         \
        0,                          \
        0,                          \
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
