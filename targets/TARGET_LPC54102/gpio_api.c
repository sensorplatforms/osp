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
#include "gpio_api.h"
#include "pinmap.h"
#include "Gpio_object.h"

void gpio_init(gpio_t __attribute__((unused)) *obj, PinName __attribute__((unused)) pin) {

    /* Moving peripheral clock init and LED init here. */
    /* INMUX and IOCON are used by many apps, enable both INMUX and IOCON clock bits here. */
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);
    
    /* Initialize GPIO */
    Chip_GPIO_Init(LPC_GPIO);
    
    Chip_PININT_Init(LPC_PININT);
}

void gpio_dir(gpio_t *obj, PinDirection direction) {
    ASF_assert(obj != NULL);
    ASF_assert(obj->pin != (PinName)NC);
    if (direction == PIN_OUTPUT) {
        Chip_GPIO_SetPinDIROutput(LPC_GPIO, DECODE_PORT(obj->pin), DECODE_PIN(obj->pin));
    }
    else { // PIN_INPUT
        Chip_GPIO_SetPinDIRInput(LPC_GPIO, DECODE_PORT(obj->pin), DECODE_PIN(obj->pin));
    }
}
