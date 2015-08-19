/*
 * @brief Adaptation layer for Bosch Drivers
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

#ifndef __SENSACQ_I2C_H
#define __SENSACQ_I2C_H
#include "osp-types.h"

#define INTERRUPT_BASED_SAMPLING    /* Sensor sampling is interrupt driver */
#undef  TRIGGERED_MAG_SAMPLING      /* Magnetometer sampling is software triggered */
/* Sensor acquisition related definitions */

#if !defined INTERRUPT_BASED_SAMPLING
# define SENSOR_SAMPLE_PERIOD           (20)   //time in ms
# define MAG_DECIMATE_FACTOR            (1)
# define ACCEL_SAMPLE_DECIMATE          (1)
# define GYRO_SAMPLE_DECIMATE           (1)
#else
# define MAG_DECIMATE_FACTOR            (1)
# define ACCEL_SAMPLE_DECIMATE          (1)
# define GYRO_SAMPLE_DECIMATE           (1)
#endif

#define	PRESSURE_SAMPLE_PERIOD		(40)

#ifdef TRIGGERED_MAG_SAMPLING
# define MAG_TRIGGER_RATE_DECIMATE      (1) //1/2 of Accel ODR
#endif

#define ALGORITHM_TASK                  ALGORITHM_TASK_ID

#define TIMER_REF_SENSOR_READ               (0x55B0)
#define TIMER_REF_PRESSURE_READ             (0x55ca)

osp_bool_t dev_i2c_init(void);

void dev_i2c_delay(unsigned int msec);

char dev_i2c_write(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt);

char dev_i2c_read(unsigned char dev_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt);

#endif /* __SENSACQ_I2C_H */
