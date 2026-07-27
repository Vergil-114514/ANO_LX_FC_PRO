#ifndef _DRVADC_H_
#define _DRVADC_H_
#include "SysConfig.h"

extern float AdcVal_BatVot;
extern float AdcVal_BatCur;

void DrvAdcInit(void);
void DrvAdcCal(void);
#endif
