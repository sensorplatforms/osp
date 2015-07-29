/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef US_TICKER_API_H
#define US_TICKER_API_H

#include "device.h"

#if DEVICE_SPI

#ifdef __cplusplus
extern "C" {
#endif

void us_ticker_init(void);
uint32_t us_ticker_read(void);
void us_ticker_set_interrupt(timestamp_t timestamp);
void us_ticker_disable_interrupt(void);
void us_ticker_clear_interrupt(void);

#ifdef __cplusplus

}
#endif

#endif

#endif
