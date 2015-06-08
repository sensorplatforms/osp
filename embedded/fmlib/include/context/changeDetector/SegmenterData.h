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
#ifndef SEGMENTERDATA_H
#define SEGMENTERDATA_H

#include "FM_DataTypes.h"
//#include "core/FixedPointTypes.h"
#include "osp-fixedpoint-types.h"
#include "context/ContextTypes.h"
//result flags for change detector
#define FPCD_FLAG_SEGMENT_START                          (0x00000001)
#define FPCD_FLAG_SEGMENT_END                            (0x00000002)
#define FPCD_FLAG_SEGMENT_POTENTIALLY_A_GESTURE          (0x00000004)
#define FPCD_FLAG_SEGMENT_CANCEL                         (0x00000008)

/* NOTE:  ESegmentType is defined on context/ContextTypes.h */
/* NOTE:  Segment_t is defined on context/ContextTypes.h */

typedef void (*ChangeDetectorCallback_t)(void * objPtr,const Segment_t * pSegment);

#define FM_CPP_TO_C_CHANGE_DETECTOR_CALLBACK(myname,myfunc,myclass)\
    extern "C" void myname(void * ptr,const Segment_t * pSegment);\
    void myname(void * ptr,const Segment_t * pSegment) {\
        if (ptr) {\
            myclass * p = static_cast<myclass *>(ptr);\
            (*p).myfunc(pSegment);                             \
        }\
    }



#ifdef __cplusplus
extern "C" {
#endif

// \brief Resets segment struct
void Segment_Reset(Segment_t * pSegment);

// \brief Logs segment struct
void Segment_Log(const fm_char_t * name, const Segment_t * pSegment);

// \brief Update segment start and end times and update duration accordingly
void Segment_SetTimes(Segment_t * pSegment, NTTIME startTime, NTTIME endTime);

// \brief Update segment start time and update duration accordingly
void Segment_SetStartTime(Segment_t * pSegment, NTTIME startTime);

// \brief Update segment end time and update duration accordingly
void Segment_SetEndTime(Segment_t * pSegment, NTTIME endTime);

// \brief Computes fraction of overlap of segment_1 with segment_2
NT Segment_ComputeFractionOverlap(Segment_t segment_1, Segment_t segment_2);

#ifdef __cplusplus
}
#endif

#endif // SEGMENTERDATA_H
