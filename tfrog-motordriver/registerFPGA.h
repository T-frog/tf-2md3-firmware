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

#ifndef __REGISTER_FPGA_H__
#define __REGISTER_FPGA_H__

typedef volatile unsigned short TVREG;

// Mapping(0x00-0x7F)
// 0x00-0x0F General Information
// 0x10-0x1F Motor1 Control Registers
// 0x20-0x2F Motor2 Control Registers

typedef volatile struct _REG_GENERAL
{
  TVREG ID;             // 0x0 FPGA識別用
  TVREG OUTPUT_ENABLE;  // 0x1 モータ出力有効/無効
  TVREG Reserved0[6];   // 0x2-0x7 予約
  struct
  {
    TVREG COUNT_ENABLE;  // 0x8 三角波カウント有効/無効
    TVREG HALF_PERIOD;   // 0x9 PWM半周期クロック数(48MHz)
    TVREG COUNT;         // 0xA 三角波カウント値
    TVREG DEADTIME;      // 0xB デッドタイム生成長
  } PWM;
  struct
  {
    unsigned HFREQ : 1;               // - High frequency count
    unsigned Reserved : 15;           // - 予約
  } __attribute__((packed)) ENCODER;  // 0xC エンコーダカウント設定
  TVREG Reserved1[3];                 // 0xD-0xF 予約
} REG_GENERAL;

typedef volatile struct _REG_MOTOR
{
  TVREG ENCODER;       // 0x0 エンコーダ4逓倍アップダウンカウント値
  TVREG SPEED;         // 0x1 エンコーダスピード
  TVREG Reserved0[1];  // 0x2 予約
  struct
  {
    unsigned HALL : 3;                     // - ホール素子状態
    unsigned Reserved : 12;                // - 予約
    unsigned Z : 1;                        // - Z相信号
  } __attribute__((packed)) ROT_DETECTER;  // 0x3 絶対角信号
  struct
  {
    TVREG H;               // ハイサイドコンパレートレベル
    TVREG L;               // ハイサイドコンパレートレベル
  } PWM[3];                // 0x4-0x9
  TVREG INDEX_RISE_ANGLE;  // 0xA
  TVREG INDEX_FALL_ANGLE;  // 0xB
  TVREG Reserved1[3];      // 0xC-0xE 予約
  TVREG INVERT;            // 0xF 反転
} REG_MOTOR;

typedef volatile struct _THEVA_REG
{
  REG_GENERAL GENERAL;  // 0x00
  TVREG Reserved0[16];  // 0x10-0x1F
  REG_MOTOR MOTOR[2];   // 0x20-0x3F
  TVREG Reserved1[48];  // 0x40-0x6F
  TVREG PORT[16];       // 0x70-0x7F
} THEVA_REG;

#define THEVA (*((THEVA_REG*)0x10000000))

#define HALL_U (0x01)
#define HALL_V (0x02)
#define HALL_W (0x04)
#define HALL_Z (0x80)

#endif
