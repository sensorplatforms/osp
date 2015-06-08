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
/**
 * @file CalibrationTypes.h
 * This file defines application specific configuration values that cannot go in Common.h
 * This file should only be included by Common.h file
 *
 ***************************************************************************************************/

#ifndef _CALIBRATIONTYPES_H_
#define _CALIBRATIONTYPES_H_

#include "embedded/FM_Types.h"
#include "FM_DataTypes.h"

typedef struct {
    NTPRECISE skr[3][3];
    NTPRECISE quality[3];
    int32_t offset[3]; //could be NTPRECISE or NTEXTENDED, and it doesn't matter which one it is
    EnumCalSource_t calsource;
} FMAlg_CalStruct_t ;

typedef struct {
    int32_t scale[3];
    int32_t skew[3];
    int32_t offset[3];
    int32_t rotation[3];
    int32_t quality[12];
    EnumCalType_t sensortype;
    EnumCalSource_t calsource;
    uint16_t crc;
} FM_CalStorageStruct;

#endif
