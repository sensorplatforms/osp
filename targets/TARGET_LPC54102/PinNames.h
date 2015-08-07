/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2014, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#ifndef PINNAMES_H
#define PINNAMES_H

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

// MODE (see GPIOMode_TypeDef structure)
// AFNUM (see AF_mapping constant table)
#define ENCODE_PIN_DATA(MODE, AFNUM)  (((MODE) << 8) | (AFNUM))
#define DECODE_PIN_MODE(X)            ((X) >> 8)
#define DECODE_PIN_AFNUM(X)           ((X) & 0xFF)

// High nibble = port number (0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, 7=H)
// Low nibble  = pin number
#define ENCODE_PORT_PIN(port,pin) (PinName)(((uint32_t)port << 16) + (uint16_t)pin)
#define DECODE_PORT(X) (((uint32_t)(X) >> 16) & 0xF)
#define DECODE_PIN(X)  ((uint32_t)(X) & 0xFFFF)

typedef enum 
{
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

typedef enum 
{
    Pin_0  = 0x00,
    Pin_1  = 0x01,
    Pin_2  = 0x02,
    Pin_3  = 0x03,
    Pin_4  = 0x04,
    Pin_5  = 0x05,
    Pin_6  = 0x06,
    Pin_7  = 0x07,
    Pin_8  = 0x08,
    Pin_9  = 0x09,
    Pin_10 = 0x0A,
    Pin_11 = 0x0B,
    Pin_12 = 0x0C,
    Pin_13 = 0x0D,
    Pin_14 = 0x0E,
    Pin_15 = 0x0F,
    Pin_16 = 0x10,
    Pin_17 = 0x11,
    Pin_18 = 0x12,
    Pin_19 = 0x13,
    Pin_20 = 0x14,
    Pin_21 = 0x15,
    Pin_22 = 0x16,
    Pin_23 = 0x17,
    Pin_24 = 0x18,
    Pin_25 = 0x19,
    Pin_26 = 0x1A,
    Pin_27 = 0x1B,
    Pin_28 = 0x1C,
    Pin_29 = 0x1D,
    Pin_30 = 0x1E,
    Pin_31 = 0x1F,
    Pin_All= 0xFFFF
} PinNumber;

typedef enum 
{
    Port_0  = 0x00,
    Port_1  = 0x01,
    Port_2  = 0x02,
    Port_3  = 0x03,
    Port_4  = 0x04,
    Port_5  = 0x05,
    Port_6  = 0x06,
    Port_7  = 0x07,
    Port_All= 0xFFFF
} PortNumber;

#define NC (uint32_t)0xFFFFFFFF
typedef uint32_t PinName;

/**
 * PIN function and mode selection definitions
 * Might need to be changed while porting to different platform
 */
typedef int32_t PinMode;

typedef enum 
{
    PINMAP_FUNC0 = 0x0,				/*!< Selects pin function 0 */
    PINMAP_FUNC1 = 0x1,				/*!< Selects pin function 1 */
    PINMAP_FUNC2 = 0x2,				/*!< Selects pin function 2 */
    PINMAP_FUNC3 = 0x3,				/*!< Selects pin function 3 */
    PINMAP_FUNC4 = 0x4,				/*!< Selects pin function 4 */
    PINMAP_FUNC5 = 0x5,				/*!< Selects pin function 5 */
    PINMAP_FUNC6 = 0x6,				/*!< Selects pin function 6 */
    PINMAP_FUNC7 = 0x7				/*!< Selects pin function 7 */
} PINMAP_FUNC;

typedef enum
{
    PINMAP_MODE_INACT    = (0x0 << 3),		/*!< No addition pin function */
    PINMAP_MODE_PULLDOWN = (0x1 << 3),		/*!< Selects pull-down function */
    PINMAP_MODE_PULLUP   = (0x2 << 3),		/*!< Selects pull-up function */
    PINMAP_MODE_REPEATER = (0x3 << 3)		/*!< Selects pin repeater function */
} PINMAP_MODE;

#define PINMAP_HYS_EN            (0x1 << 5)		/*!< Enables hysteresis */
#define PINMAP_GPIO_MODE         (0x1 << 5)		/*!< GPIO Mode */
#define PINMAP_I2C_SLEW          (0x1 << 5)		/*!< I2C Slew Rate Control */
#define PINMAP_INV_EN            (0x1 << 6)		/*!< Enables invert function on input */
#define PINMAP_ANALOG_EN         (0x0 << 7)		/*!< Enables analog function by setting 0 to bit 7 */
#define PINMAP_DIGITAL_EN        (0x1 << 7)		/*!< Enables digital function by setting 1 to bit 7(default) */
#define PINMAP_STDI2C_EN         (0x1 << 8)		/*!< I2C standard mode/fast-mode */
#define PINMAP_FASTI2C_EN        (0x3 << 8)		/*!< I2C Fast-mode Plus and high-speed slave */
#define PINMAP_INPFILT_OFF       (0x1 << 8)		/*!< Input filter Off for GPIO pins */
#define PINMAP_INPFILT_ON        (0x0 << 8)		/*!< Input filter On for GPIO pins */
#define PINMAP_OPENDRAIN_EN      (0x1 << 10)		/*!< Enables open-drain function */

#ifdef __cplusplus
}
#endif

#endif
