#ifndef __PRESSURE_COMMON_H__
#define __PRESSURE_COMMON_H__
#include "common.h"

void Pressure_HardwareSetup(osp_bool_t);
void Pressure_Initialize(void);
void Pressure_ReadData(MsgPressData *);
#endif
