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

// UART4/UC navigation input. Exactly one source may own this port.
#define NAV_INPUT_UWB_MINI5 1
#define NAV_INPUT_GPS 0

#if ((NAV_INPUT_UWB_MINI5 + NAV_INPUT_GPS) != 1)
#error "Select exactly one UART4 navigation input"
#endif

#if NAV_INPUT_GPS
#define GPS_USE_UBLOX_M8
#endif

// Mini5 anchor antenna phase-center coordinates in the car UWB frame, mm.
#define UWB_A0_X_MM (-300)
#define UWB_A0_Y_MM (-300)
#define UWB_A1_X_MM (300)
#define UWB_A1_Y_MM (-300)
#define UWB_A2_X_MM (-300)
#define UWB_A2_Y_MM (300)
#define UWB_A3_X_MM (300)
#define UWB_A3_Y_MM (300)
#define UWB_TAG_SHORT_ID 0x07U
#define UWB_DATA_TIMEOUT_MS 500U
#define UWB_RANGE_MIN_MM 100U
#define UWB_RANGE_MAX_MM 50000U

// UWB alpha-beta filter settings. Raw positions remain available for frame 0x32.
#define UWB_FILTER_ALPHA 0.35f
#define UWB_FILTER_BETA 0.05f
#define UWB_FILTER_MAX_RESIDUAL_MM 300U
#define UWB_FILTER_MAX_SPEED_MMPS 3000U
#define UWB_FILTER_WARMUP_SAMPLES 3U

// Car heading input selection.
#define CAR_LINK_PORT_NONE 0U
#define CAR_LINK_PORT_UART8 8U
#define CAR_LINK_PORT CAR_LINK_PORT_UART8
#define CAR_LINK_UART8_BAUDRATE 115200U
#define CAR_LINK_TIMEOUT_MS 200U

// Automatic payload and dynamic-landing task settings.
#define AUTO_CRUISE_ALT_CM             150U
#define DROP_ALT_CM                    100U
#define UWB_TARGET_OFFSET_X_MM         0
#define UWB_TARGET_OFFSET_Y_MM         0
#define PAYLOAD_RELEASE_DELAY_MS       500U
#define AUTO_ALIGN_TOL_MM              80U
#define AUTO_ALIGN_STABLE_MS           500U
#define AUTO_LAND_CONTACT_ALT_CM       15U
#define AUTO_CAR_DWELL_MS              5000U
#define AUTO_TASK_TIMEOUT_MS           90000U
#define AUTO_ALTITUDE_TOL_CM           10U
#define AUTO_MAX_VERTICAL_SPEED_CMPS   20.0f
#define AUTO_VERTICAL_SPEED_KP          0.5f
#define AUTO_TOF_DATA_TIMEOUT_MS        500U
#define AUTO_IMU_VEL_DATA_TIMEOUT_MS    500U
//================RCCNANNELDEF===================
//飞控模式切换通道
#define RCCNANNELDEF_FLIGHTMODE		ch_6_aux2	
//飞控紧急停机通道
#define RCCNANNELDEF_EMERGENCYSTOPESC	ch_5_aux1

//=========================================
#endif
