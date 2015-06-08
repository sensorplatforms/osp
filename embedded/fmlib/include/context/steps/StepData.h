/*****************************************************************************
 *                                                                           *
 *                       Sensor Platforms Inc.                               *
 *                   2860 Zanker Road, Suite 210                             *
 *                        San Jose, CA 95134                                 *
 *                                                                           *
 *****************************************************************************
 *                                                                           *
 *               Copyright (c) 2012 Sensor Platforms Inc.                    *
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
#ifndef _STEPDATA_H_
#define _STEPDATA_H_

#include "FM_Platform.h"
//#include "core/FixedPointTypes.h"
#include "osp-fixedpoint-types.h"
//if C++, include serialized data type definitions
#ifdef __cplusplus
#include "data/Messages.h"
#endif

#include "context/ContextTypes.h"

// Note: StepData_emb_t is defined in context/ContextTypes.h

#ifdef __cplusplus
typedef struct StepData_t : public StepData_emb_t {

    typedef SPI::data::StepData SerializedDataType;
    inline operator const SPI::data::StepData() const{
        SPI::data::StepData data;
        data.timestamp = TOFLT_TIME(startTime);
        data.lastStepStartTime = TOFLT_TIME(startTime);
        data.lastStepStopTime = TOFLT_TIME(stopTime);
        data.lastStepLength = TOFLT(stepLength);
        data.stepFrequency = TOFLT(stepFrequency);
        data.numStepsTotal = numStepsTotal;
        data.numStepsUp = numStepsUp;
        data.numStepsDown = numStepsDown;
        data.numStepsLevel = numStepsLevel;
        data.numStepsConsecutive = numStepsConsecutive;

        return data;
    }
} StepData_t;

#endif

#ifdef __cplusplus
extern "C" {
#endif

void ResetStep(StepData_t *step);
  
#ifdef __cplusplus
}
#endif


#endif //_STEPDATA_H_
