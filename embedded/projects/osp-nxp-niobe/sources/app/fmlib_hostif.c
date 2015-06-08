/****************************************************************************************************
 *                                                                                                  *
 *                                    Sensor Platforms Inc.                                         *
 *                                    2860 Zanker Road, Suite 210                                   *
 *                                    San Jose, CA 95134                                            *
 *                                                                                                  *
 ****************************************************************************************************
 *                                                                                                  *
 *                                Copyright (c) 2012 Sensor Platforms Inc.                          *
 *                                        All Rights Reserved                                       *
 *                                                                                                  *
 ***************************************************************************************************/
/*
 * @brief Glue logic between Free Motion Library and NXP Sensor Hub framework
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 */

#include <stdint.h>
#include "string.h"
#include "sensorhub.h"
#include "fmlib_hostif.h"
#include "hw_setup.h"


#define SPI_SH_WHO_AM_I                 0x54
#define SPI_SH_VERSION0                 0x01
#define SPI_SH_VERSION1                 0x22


static uint64_t sensorEnable = 0;
//#if 0
static void controlSensorEnable(uint8_t sensorId, uint8_t enable) {
	uint8_t update = 0;
	if ((sensorId < sizeof(sensorEnable) * 8) && (sensorId < SPI_SH_SENSOR_ID_COUNT)) {
		if (enable) {
			if (!(sensorEnable & (1L << sensorId))) {
				sensorEnable |= (1L << sensorId);
				update = 1;
			}
		} else {
			if (sensorEnable & (1L << sensorId)) {
				sensorEnable &= ~(1L << sensorId);
				update = 1;
			}
		}
		/* Need to add to OSP */
		if (update) {
			do {
				// Algorithm_EnableSensor( sensorId,  enable ? 1 : 0);
			} while(0);
		}
	}
}

uint8_t isSensorEnable(uint8_t sensorId) {
    if ((sensorId < sizeof(sensorEnable) * 8) && (sensorId < SPI_SH_SENSOR_ID_COUNT)) {
        return (sensorEnable & (1L << sensorId)) ? 1 : 0;
    }
    return 0;
}

uint16_t sensorDelay[SPI_SH_SENSOR_ID_COUNT] = {0};

static void controlSensorDelay(uint8_t sensorId, uint16_t miliSecondsDelay) {
    if ((sensorId < sizeof(sensorEnable) * 8) && (sensorId < SPI_SH_SENSOR_ID_COUNT)) {
        if (miliSecondsDelay != sensorDelay[sensorId]) {
            sensorDelay[sensorId] = miliSecondsDelay;
            //SensorAcqSendSensorDelayIndication( sensorId,  miliSecondsDelay);
        }
    }
}

uint16_t getSensorDelay(uint8_t sensorId) {
    if ((sensorId < sizeof(sensorEnable) * 8) && (sensorId < SPI_SH_SENSOR_ID_COUNT)) {
        return sensorDelay[sensorId] ;
    }
    return 0;
}

uint16_t timeStampExpansion;
uint16_t broadcast_buf_wr = 0, broadcast_buf_rd = 0;

/**
* @brief <b>Description:</b> Processes any pending read requests
*
**/ 

static uint8_t currentOpCode;
void Hostif_StartTx(uint8_t *pBuf, uint16_t size);
