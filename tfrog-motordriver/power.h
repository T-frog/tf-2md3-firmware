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

#ifndef __POWER_H__
#define __POWER_H__

void LowPowerMode(void);
void NormalPowerMode(void);

static const Pin pinsLED[] = {PIN_LED_0, PIN_LED_1, PIN_LED_2};

#define LED_on(num)  PIO_Clear(&pinsLED[num]);
#define LED_off(num) PIO_Set(&pinsLED[num]);

#endif
