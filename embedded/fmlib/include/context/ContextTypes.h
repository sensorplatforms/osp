/*****************************************************************************
 *                                                                           *
 *                       Sensor Platforms Inc.                               *
 *                   2860 Zanker Road, Suite 210                             *
 *                        San Jose, CA 95134                                 *
 *                                                                           *
 *****************************************************************************
 *                                                                           *
 *               Copyright (c) 2013 Sensor Platforms Inc.                    *
 *                       All Rights Reserved                                 *
 *                                                                           *
 *                   Proprietary and Confidential                            *
 *             Use only under license described in EULA.txt                  *
 *                                                                           *
 ****************************************************************************/
/*! \file                                                                    *
 *                                                                           *
 *  @author Sensor Platforms Inc: http://www.sensorplatforms.com             *
 *  @author        Support Email: support@sensorplatforms.com                *
 *                                                                           *
 ****************************************************************************/
#ifndef CONTEXTTYPES_H_
#define CONTEXTTYPES_H_

#include "embedded/FM_Types.h"
#include "FM_DataTypes.h"



// This file contains structure and enum type definitions for context objects
// 
// These types are used for context data passing on both embedded and non-embedded

typedef enum {
    transition = 0,
    steadyState,
    transitionStep,
    steadyStateStep,
    event,
    ESEGMENT_COUNT
}ESegmentType;

typedef struct Segment_t {
  NTTIME startTime;
  NTTIME endTime;
  NTDELTATIME duration;
  ESegmentType type;
  uint32_t flags;
  uint32_t sequenceNumber;
} Segment_t;

//struct for defining a step
typedef struct StepData_emb_t{
    NTTIME startTime;
    NTTIME stopTime;
    NT stepLength;
    NT stepFrequency;
    uint16_t numStepsTotal;
    uint16_t numStepsConsecutive;
    uint16_t numStepsUp;
    uint16_t numStepsDown;
    uint16_t numStepsLevel;

} StepData_emb_t;

//struct for defining a step
typedef struct StepSensitiveData_emb_t{
    NTTIME startTime;
    NTTIME stopTime;
    NT stepLength;
    NT stepFrequency;
    uint16_t numStepsTotal;
    uint16_t numStepsConsecutive;
} StepSensitiveData_emb_t;



#ifndef __cplusplus
    typedef StepData_emb_t StepData_t;
    typedef StepSensitiveData_emb_t StepSensitiveData_t;
#endif


#endif /*CONTEXTTYPES_H_*/
