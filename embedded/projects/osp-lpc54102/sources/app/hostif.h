/*
 * @brief I2C based host interface module
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef _HOSTIF_H_
#define _HOSTIF_H_

#include "spi-sensor-hub-priv.h"
#include "gpio_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_HOSTIF : Sensor hub host interface
 * @ingroup SENSOR_HUB
 * @{
 */

#define SLAVE_MAX_BUFFER_SIZE 256			// note: Android limitation is 1K

/**
 * @brief	Initialize host interface subsystem
 * @return	none
 */
void Hostif_Init(void);

/**
 * @brief	Start Host interface transfer
 * @param	pBuf	: Pointer to buffer to be transmitted
 * @param	size	: Size of the buffer
 * @return	none
 */
void Hostif_StartTx(uint8_t *pBuf, uint16_t size, int magic);

/**
 * @brief	Assert interrupt line to Host/AP
 * @return	none
 */
static INLINE void Hostif_AssertIRQ(void) 
{
	//Chip_GPIO_SetPinState(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN, 0);
    gpio_t hostifIrq;
    hostifIrq.pin = ENCODE_PORT_PIN(HOSTIF_IRQ_PORT,HOSTIF_IRQ_PIN);
    gpio_write(&hostifIrq,0);
}

/**
 * @brief	Deassert interrupt line to Host/AP
 * @return	none
 */
static INLINE void Hostif_DeassertIRQ(void) 
{
	//Chip_GPIO_SetPinState(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN, 1);
    gpio_t hostifIrq;
    hostifIrq.pin = ENCODE_PORT_PIN(HOSTIF_IRQ_PORT,HOSTIF_IRQ_PIN);
    gpio_write(&hostifIrq,1);
}

/**
 * @brief	Deassert interrupt line to Host/AP
 * @return	none
 */
static INLINE bool Hostif_IRQActive(void) 
{
	//return (Chip_GPIO_GetPinState(LPC_GPIO, HOSTIF_IRQ_PORT, HOSTIF_IRQ_PIN) == false);
    gpio_t hostifIrq;
    hostifIrq.pin = ENCODE_PORT_PIN(HOSTIF_IRQ_PORT,HOSTIF_IRQ_PIN);
    return (gpio_read(&hostifIrq) == false);
}

/**
 * @brief	Process host I/F tasks
 * @return	If 0 the caller can sleep else there are pending tasks.
 */
uint32_t Hostif_Process(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif	/* _HOSTIF_H_ */
