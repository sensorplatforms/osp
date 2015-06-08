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
#ifndef FM_CONSTANTS_H_
#define FM_CONSTANTS_H_

/*--------------------------------------------------------------------------*\
 |    M A T H   C O N S T A N T S   &   M A C R O S
\*--------------------------------------------------------------------------*/
#ifndef M_SI_EARTH_GRAVITY
#define M_SI_EARTH_GRAVITY           (9.805f)
#endif

#ifndef M_PI
#define M_PI                         (3.141592653589793f)
#endif

#ifndef M_2PI
#define M_2PI                        (6.28318530717959f)
#endif

#ifndef M_RAD2DEG
#define M_RAD2DEG(x)                 (x*(180.0f/M_PI))
#endif

#ifndef M_DEG2RAD
#define M_DEG2RAD(x)                 (x*(M_PI/180.0f))
#endif


/*--------------------------------------------------------------------------*\
 |   I N T E R N A L   C O N S T A N T S   &   M A C R O S
\*--------------------------------------------------------------------------*/
#ifndef FM_GYRO_SCALE_FACTOR
#define FM_GYRO_SCALE_FACTOR         (3)
#endif

#ifndef FM_DATA_STORE_FILE_NAME
#define FM_DATA_STORE_FILE_NAME       "spi_embedded_datastore.dat"
#endif

#endif /* FM_CONSTANTS_H_ */
