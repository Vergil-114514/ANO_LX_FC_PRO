#ifndef DRV_ANOOF_PTV7_H
#define DRV_ANOOF_PTV7_H

//==引用
#include "Drv_AnoOf.h"
//==定义/声明
void DrvAnoOFCheckState_ptv7(float dT_s);
void DrvAnoOFGetOneByte_ptv7(const uint8_t linktype, const uint8_t data);
#endif
