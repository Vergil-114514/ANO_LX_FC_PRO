#ifndef __DRV_ANO_OF_H
#define __DRV_ANO_OF_H

//==引用
#include "SysConfig.h"
#include "AnoPTv8.h"
//==定义/声明

typedef struct
{
	//
	uint8_t of_update_cnt;  //光流数据更新计数。
	uint8_t alt_update_cnt; //高度数据更新计数。
	uint8_t of_display_update_cnt; //供上位机显示的光流数据更新计数。
	//
	uint8_t link_sta; //连接状态：0，未连接。1，已连接。
	uint8_t work_sta; //工作状态：0，异常。1，正常
	//
	uint8_t of_quality;
	uint8_t of_display_type;
	int16_t of_display_dx;
	int16_t of_display_dy;
	//
	uint8_t of0_sta;
	int8_t of0_dx;
	int8_t of0_dy;
	//
	uint8_t of1_sta;
	int16_t of1_dx;
	int16_t of1_dy;
	//
	uint8_t of2_sta;
	int16_t of2_dx;
	int16_t of2_dy;
	int16_t of2_dx_fix;
	int16_t of2_dy_fix;
	int16_t intergral_x;
	int16_t intergral_y;
	//
	uint32_t of_alt_cm;
	//
	float quaternion[4];
	//
	int16_t acc_data_x;
	int16_t acc_data_y;
	int16_t acc_data_z;
	int16_t gyr_data_x;
	int16_t gyr_data_y;
	int16_t gyr_data_z;

} _ano_of_st;

//飞控状态

//==数据声明
extern _ano_of_st ano_of;
//==函数声明
//public
void AnoOF_Check_State(float dT_s);
void AnoOFFrameAnl(const uint8_t linktype, const _un_frame_v8 *p);
#endif
