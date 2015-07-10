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
/* #include "mbed_assert.h" */
#include "device.h"
#include "pinmap.h"

// Alternate-function mapping
#if 0
static const uint32_t AF_mapping[] = {
  0,                      // 0 = No AF
  GPIO_Remap_SPI1,        // 1
  GPIO_Remap_I2C1,        // 2
  GPIO_Remap_USART1,      // 3
  GPIO_Remap_USART2,      // 4
  GPIO_FullRemap_TIM2,    // 5
  GPIO_FullRemap_TIM3,    // 6
  GPIO_PartialRemap_TIM3, // 7
  GPIO_Remap_I2C1         // 8
};
#endif

// Enable GPIO clock and return GPIO base address
uint32_t Set_GPIO_Clock(uint32_t port_idx) {
#if 0
    uint32_t gpio_add = 0;
    switch (port_idx) {
        case PortA:
            gpio_add = GPIOA_BASE;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            break;
        case PortB:
            gpio_add = GPIOB_BASE;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
            break;
        case PortC:
            gpio_add = GPIOC_BASE;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
            break;
        case PortD:
            gpio_add = GPIOD_BASE;
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
            break;
        default:
            D1_printf("Port number is not correct.");
            break;
    }
    return gpio_add;
#endif
    return 0;
}

/**
 * Configure pin (input, output, alternate function or analog) + output speed + AF
 */
void pin_function(PinName pin, uint32_t data) {

    uint32_t port_index = DECODE_PORT(pin);
    uint32_t pin_index  = DECODE_PIN(pin);

    ASF_assert(pin != (PinName)NC);

    // Configure GPIO
    Chip_IOCON_PinMuxSet(LPC_IOCON, port_index, pin_index, data);
}

/**
 * Configure pin pull-up/pull-down
 */
void pin_mode(PinName pin, PinMode mode) {
#if 0
    ASF_assert(pin != (PinName)NC);
    GPIO_InitTypeDef GPIO_InitStructure;
    
    uint32_t port_index = STM_PORT(pin);
    uint32_t pin_index  = STM_PIN(pin);

    // Enable GPIO clock
    uint32_t gpio_add = Set_GPIO_Clock(port_index);
    GPIO_TypeDef *gpio = (GPIO_TypeDef *)gpio_add;
  
    // Configure open-drain and pull-up/down
    switch (mode) {
        case AnalogInput:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
            break;
        case Floating:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            break;
        case PullUp:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            break;
        case PullDown:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
            break;
        case OpenDrain:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            break;
        case PushPullOutput:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            break;
        case OpenDrainOutputAF:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
            break;
        case PushPullOutputAF:
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            break;
        default:
            D1_printf("PINMAP: Bad GPIO Mode: %d\r\n", mode);
            break;
    }
    
    // Configure GPIO
    GPIO_InitStructure.GPIO_Pin   = (uint16_t)pin_index;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &GPIO_InitStructure);
#endif
}

