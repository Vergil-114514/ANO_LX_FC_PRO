#ifndef _SYSCONFIG_H_
#define _SYSCONFIG_H_
#include "McuConfig.h"
#include "Drv_BSP.h"
//================define===================
typedef float vec3_f[3];
typedef float vec2_f[2];
typedef int32_t vec3_s32[3];
typedef int32_t vec2_s32[2];
typedef int16_t vec3_s16[3];
typedef int16_t vec2_s16[2];

#define TICK_PER_SECOND	1000
#define TICK_US	(1000000/TICK_PER_SECOND)

#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

#define ESC_TYPE	(0)				//0:pwm信号输出;	1:DSHOT600信号输出

//PT_NULL：飞控直接才接电压电流
//PT_ANOPMU：使用匿名PMU
//PT_FCS200：使用FCS-200霍尔电流计
enum e_pmutype{PT_NULL, PT_ANOPMU, PT_FCS200, PT_COUNT};
#define PMU_TYPE	(PT_FCS200)		//PMU种类

//#define GPS_USE_RTK
#define GPS_USE_UBLOX_M8
//================RCCNANNELDEF===================
//飞控模式切换通道
#define RCCNANNELDEF_FLIGHTMODE		ch_6_aux2	
//飞控紧急停机通道
#define RCCNANNELDEF_EMERGENCYSTOPESC	ch_5_aux1

//=========================================
#endif
