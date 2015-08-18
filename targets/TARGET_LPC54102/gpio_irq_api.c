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
#include <stddef.h>
#include "common.h"

#include "gpio_irq_api.h"
#include "pinmap.h"


int gpio_irq_init(gpio_irq_t *obj, PinName pin, gpio_irq_handler handler, uint32_t id) {

    ASF_assert(pin != NC);
    Chip_INMUX_PinIntSel(id, DECODE_PORT(pin), DECODE_PIN(pin));
    gpio_irq_enable(obj);
    return 0;
}

void gpio_irq_enable(gpio_irq_t *obj) {
    switch(obj->event){
        case IRQ_EDGE_RISE:
            Chip_PININT_SetPinModeEdge(LPC_PININT, obj->irq_index); /* edge sensitive */
            Chip_PININT_EnableIntHigh(LPC_PININT, obj->irq_index);  /* Rising edge interrupt */
            break;
        case IRQ_EDGE_FALL:
            Chip_PININT_SetPinModeEdge(LPC_PININT, obj->irq_index); /* Edge sensitive */
            Chip_PININT_EnableIntLow(LPC_PININT, obj->irq_index);   /* Falling edge interrupt */
            break;
        case IRQ_LEVEL_HIGH:
            Chip_PININT_SetPinModeLevel(LPC_PININT, obj->irq_index); /* Level sensitive */
            Chip_PININT_EnableIntHigh(LPC_PININT, obj->irq_index);   /* High level interrupt */
            break;
        case IRQ_LEVEL_LOW:
            Chip_PININT_SetPinModeLevel(LPC_PININT, obj->irq_index); /* Level sensitive */
            Chip_PININT_EnableIntLow(LPC_PININT, obj->irq_index);    /* Low level interrupt */
            break;
        default:
            D1_printf("GPIO_IRQ_ENABLE: Bad IRQ Mode: %d\r\n", obj->irq_index);
            while(1){};
    }        
}

void gpio_irq_disable(gpio_irq_t *obj) {
	Chip_PININT_ClearIntStatus(LPC_PININT, obj->irq_index);
}
