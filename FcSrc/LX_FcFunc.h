#ifndef __LX_FC_FUN_H
#define __LX_FC_FUN_H

//==引用
#include "SysConfig.h"

//==定义/声明

//==数据声明

//==函数声明
//static

//public
uint8_t FC_Unlock(void);
uint8_t FC_Lock(void);
uint8_t LX_Change_Mode(uint8_t new_mode);
uint8_t OneKey_Takeoff(uint16_t  height_cm);
uint8_t OneKey_Land(void);
uint8_t OneKey_Flip(void);
uint8_t OneKey_Return_Home(void);
uint8_t Horizontal_Calibrate(void);
uint8_t Horizontal_Move(uint16_t  distance_cm, uint16_t  velocity_cmps, uint16_t  dir_angle_0_360);
uint8_t Mag_Calibrate(void);
uint8_t ACC_Calibrate(void);
uint8_t GYR_Calibrate(void);
#endif
