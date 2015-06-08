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

#ifndef CONTEXTDATATYPES_H_
#define CONTEXTDATATYPES_H_

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
 * DEVICE MOTION CONTEXT
 *****************************************************************************/
typedef enum  {unknownDeviceMotion=-1,
               isStill=0,
               isAccelerating=1,
               isRotating=2,
               isTranslating=3,
               isFreefalling=4,
               numEContextDeviceMotionTypes=5}
              EContextDeviceMotionType;

typedef enum {deviceMotionContextInsensitive = 0,
              deviceMotionContextRegular = 1,
              deviceMotionContextSensitive = 2,
              numEDeviceMotionContextSensitivty = 3}
             EContextDeviceMotionSensitivty;

/******************************************************************************
 * POSTURE CONTEXT
 *****************************************************************************/
typedef enum   {unknownPosture=-1, isWalking=0, isStanding=1,
                           isSitting=2, isJogging=3, isRunning=4, numEContextPostureTypes=5} EContextPostureType;

/******************************************************************************
 * CARRY CONTEXT
 *****************************************************************************/
typedef enum   {unknownCarry=-1, inPocket=0, inHand=1, notOnPerson=2,
                         inHandAtWaist=3, inHandAtSide=4,
                         numEContextCarryTypes=5} EContextCarryType;

/******************************************************************************
 * TRANSPORT CONTEXT
 *****************************************************************************/
    typedef enum  {unknownTransport=-1, isInCar=0, RESERVED = 1, isOnEscalator=2, isOnMovingWalkway=3,
                            isOnTrain=4, isOnAirplane=5, isInMagAnomaly=6,
                            isClimbingUpstairs=7, isClimbingDownstairs=8,
                            isInElevatorUp=9, isInElevatorDown=10,
                            numEContextTransportTypes=11} EContextTransportType;

#ifdef __cplusplus
}
#endif


#endif /* CONTEXTDATATYPES_H_ */

