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

#define EDGE_NONE (0)
#define EDGE_RISE (1)
#define EDGE_FALL (2)
#define EDGE_BOTH (3)

#define CHANNEL_NUM (16)

#if 0
static uint32_t channel_ids[CHANNEL_NUM]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint32_t channel_gpio[CHANNEL_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint32_t channel_pin[CHANNEL_NUM]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static gpio_irq_handler irq_handler;

typedef struct {
    EXTITrigger_TypeDef EXTI_Trigger;
    uint8_t NVIC_IRQChannel;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
}Gpio_irq_init_t;

static Gpio_irq_init_t tGpio_irq_init[EXTI_MAX_ID] = {
        //EXTI_Trigger              ,NVIC_IRQChannel         ,NVIC_IRQChannelPreemptionPriority,    NVIC_IRQChannelSubPriority
        {EXTI_Trigger_Rising ,ACCEL_INT_IRQChannel,ACCEL_A_INT_PREEMPT_PRIO      ,ACCEL_A_INT_SUB_PRIORITY   }, //Accel
        {EXTI_Trigger_Rising ,GYRO_RDY_IRQCHANNEL ,GYRO_DRDY_INT_PREEMPT_PRIORITY,GYRO_DRDY_INT_SUB_PRIORITY }, //Gyro
        {EXTI_Trigger_Falling,MAG_RDY_IRQCHANNEL  ,MAG_DRDY_INT_PREEMPT_PRIORITY ,MAG_DRDY_INT_SUB_PRIORITY  }  //Mag
};
#endif

static void __attribute__((unused)) handle_interrupt_in(uint32_t irq_index) {
#if 0
    //if irq_index==5 loop exti 5 to 9
    //if irq_index==10 loop exti 10 to 15
    //else exti loop one irq_index
    uint32_t to_irq_index=(irq_index==5)?9:((irq_index==10)?15:irq_index);
    GPIO_TypeDef *gpio;
    uint32_t pin;
    do{
      // Retrieve the gpio and pin that generate the irq
      gpio = (GPIO_TypeDef *)(channel_gpio[irq_index]);
      pin = (uint32_t)(1 << channel_pin[irq_index]);
      // Clear interrupt flag
      if (EXTI_GetITStatus(pin) != RESET)
      {
          EXTI_ClearITPendingBit(pin);
          if (channel_ids[irq_index] == 0) return;
          // Check which edge has generated the irq
          if ((gpio->IDR & pin) == 0) {
              irq_handler(channel_ids[irq_index], IRQ_FALL);
          } else {
              irq_handler(channel_ids[irq_index], IRQ_RISE);
          }
      }
    }while(irq_index++ < to_irq_index);
#endif
}

// The irq_index is passed to the function
//static void gpio_irq0(void) {handle_interrupt_in(0);} // EXTI line 0
//static void gpio_irq1(void) {handle_interrupt_in(1);} // EXTI line 1
//static void gpio_irq2(void) {handle_interrupt_in(2);} // EXTI line 2
//static void gpio_irq3(void) {handle_interrupt_in(3);} // EXTI line 3
//static void gpio_irq4(void) {handle_interrupt_in(4);} // EXTI line 4
//static void gpio_irq5(void) {handle_interrupt_in(5);} // EXTI lines 5 to 9
//static void gpio_irq6(void) {handle_interrupt_in(10);} // EXTI lines 10 to 15

int gpio_irq_init(gpio_irq_t *obj, PinName pin, gpio_irq_handler handler, uint32_t id) {

#if 0
    uint32_t port_index = DECODE_PORT(pin);
    uint32_t pin_index  = DECODE_PIN(pin);

    if (pin == NC) return -1;

    // Connect EXTI line to pin
    GPIO_EXTILineConfig(port_index, pin_index);

    EXTI_ClearFlag(MAG_RDY_INT_EXTI_LINE);
    
    // Configure EXTI line
    EXTI_InitTypeDef EXTI_InitStructure;    
    EXTI_InitStructure.EXTI_Line = (uint32_t)(1 << pin_index);
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = tGpio_irq_init[id].EXTI_Trigger;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // Enable and set EXTI interrupt to the lowest priority
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = tGpio_irq_init[id].NVIC_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = tGpio_irq_init[id].NVIC_IRQChannelPreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = tGpio_irq_init[id].NVIC_IRQChannelSubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
#endif
    return 0;
}

void gpio_irq_free(gpio_irq_t *obj) {
#if 0
    channel_ids[obj->irq_index] = 0;
    channel_gpio[obj->irq_index] = 0;
    channel_pin[obj->irq_index] = 0;
    // Disable EXTI line
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_Init(&EXTI_InitStructure);  
    obj->event = EDGE_NONE;
#endif
}

void gpio_irq_set(gpio_irq_t *obj, gpio_irq_event event, uint32_t enable) {
#if 0
    EXTI_InitTypeDef EXTI_InitStructure;
    
    uint32_t pin_index = channel_pin[obj->irq_index];

    EXTI_InitStructure.EXTI_Line = (uint32_t)(1 << pin_index);
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    
    if (event == IRQ_RISE) {
        if ((obj->event == EDGE_FALL) || (obj->event == EDGE_BOTH)) {
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
            obj->event = EDGE_BOTH;
        }
        else { // NONE or RISE
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
            obj->event = EDGE_RISE;
        }
    }
    
    if (event == IRQ_FALL) {
        if ((obj->event == EDGE_RISE) || (obj->event == EDGE_BOTH)) {
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
            obj->event = EDGE_BOTH;
        }
        else { // NONE or FALL
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
            obj->event = EDGE_FALL;
        }
    }
    
    if (enable) {
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    }
    else {
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    }
    
    EXTI_Init(&EXTI_InitStructure);
#endif
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
