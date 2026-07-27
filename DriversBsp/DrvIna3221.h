#ifndef _DRVINA3221_H_
#define _DRVINA3221_H_
#include "SysConfig.h"

extern float Ina3221Cur[3];

void DrvIna3221Init(void);
void DrvIna3221GetVals(void);
#endif
