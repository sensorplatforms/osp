/*
 * @brief Resource manager interface
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
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
#include <stdint.h>
#include "board.h"

#ifndef __SENSORHUB_H_
#define __SENSORHUB_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_RESMGR CHIP: Resource manager interface
 * @ingroup SENSOR_HUB
 * @{
 */

/** Sensor Hub framework IRQ priorities */
#define SENSOR_IRQ_PRIORITY			0
#define SENSOR_I2C_PRIORITY			0
#define HOSTIF_IRQ_PRIORITY			6
#define DMA_IRQ_PRIORITY			0

#define MAX_DURATION				0xFFFFFF
/** Inter activity gap threshold in usecs to trigger back ground processing */
#define BACKGROUND_THRESHOLD		(10000)	
/** Inter activity gap threshold in usecs to enter power down mode */
#define PWRDOWN_THRESHOLD			(400)
/** Inter activity gap threshold in usecs to enter sleep mode */
#define SLEEP_THRESHOLD				(8)	

/** Sensor bus interface defines */
#define I2C_SENSOR_BUS_IRQn          I2C0_IRQn
#define I2C_SENSOR_BUS_IRQHandler    I2C0_IRQHandler
#define I2C_SENSOR_BUS_DMAID         DMAREQ_I2C0_MASTER
#define I2C_SENSOR_BUS_DMABYTELIM    (1)	/* Use DMA when sending or receiving more bytes than this define,
                                             I2C ADDR byte is never sent using DMA. Use 0 to always use DMA. */

/** Host interface (I2C slave) defines */  
#define I2C_HOSTIF					LPC_I2C2
#define I2C_HOSTIF_IRQn				I2C2_IRQn
#define I2C_HOSTIF_IRQHandler		I2C2_IRQHandler
#define I2C_HOSTIF_WAKE             SYSCON_STARTER_I2C2
#define I2C_HOSTIF_CLOCK_DIV 		2	/* recommended by Noah */
#define I2C_HOSTIF_ADDR				(0x18)
#define I2C_HOSTIF_CLK              (SYSCON_CLOCK_I2C2)
#define I2C_HOSTIF_RST              RESET_I2C2
/** Host interface IRQ defines */
#define HOSTIF_IRQ_PORT				0
#define HOSTIF_IRQ_PIN				19
#define I2C_HOSTID_DMAID            DMAREQ_I2C2_SLAVE

/** General code build defines */
//#define I2CM_IRQ_BASED               /* Enable for I2C interrupt support */
//#define I2CM_DMA_BASED               /* Enable for I2C master DMA support, requires I2CM_IRQ_BASED */
//#define I2CS_DMA_BASED               /* Enable for I2C slave DMA support */
/**
 * @}
 */
uint8_t process_command(uint8_t *, uint16_t);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORHUB_H_ */
