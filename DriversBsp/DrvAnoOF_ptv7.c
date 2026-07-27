#include "DrvAnoOF_ptv7.h"
#define OF_TEMP_LEN  50

static float check_time_ms[3];
void DrvAnoOFCheckState_ptv7(float dT_s)
{
	uint8_t tmp[2];
	//连接检查
	if (check_time_ms[0] < 500)
	{
		check_time_ms[0]++;
		ano_of.link_sta = 1;
	}
	else
	{
		ano_of.link_sta = 0;
	}
	//数据检查1
	if (check_time_ms[1] < 500)
	{
		check_time_ms[1]++;
		tmp[0] = 1;
	}
	else
	{
		tmp[0] = 0;
	}
	//数据检查2
	if (check_time_ms[2] < 500)
	{
		check_time_ms[2]++;
		tmp[1] = 1;
	}
	else
	{
		tmp[1] = 0;
	}
	//设置工作状态
	if (tmp[0] && tmp[1])
	{
		ano_of.work_sta = 1;
	}
	else
	{
		ano_of.work_sta = 0;
	}
}

static void DrvAnoOFDataAnl_ptv7(uint8_t *data, uint8_t len)
{
	u8 check_sum1 = 0, check_sum2 = 0;
	if (*(data + 3) != (len - 6)) //判断数据长度是否正确
		return;
	for (u8 i = 0; i < len - 2; i++)
	{
		check_sum1 += *(data + i);
		check_sum2 += check_sum1;
	}
	if ((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) //判断sum校验
		return;
	//================================================================================

	if (*(data + 2) == 0X51) //光流信息
	{
		if (*(data + 4) == 0) //原始光流信息
		{
			ano_of.of0_sta = *(data + 5);
			ano_of.of0_dx = *(data + 6);
			ano_of.of0_dy = *(data + 7);
			ano_of.of_quality = *(data + 8);
		}
		else if (*(data + 4) == 1) //高度融合后光流信息
		{
			ano_of.of1_sta = *(data + 5);
			ano_of.of1_dx = *((s16 *)(data + 6));
			ano_of.of1_dy = *((s16 *)(data + 8));
			ano_of.of_quality = *(data + 10);
			//
			check_time_ms[1] = 0;
			ano_of.of_update_cnt++;
		}
		else if (*(data + 4) == 2) //惯导融合后光流信息
		{
			ano_of.of2_sta = *(data + 5);
			ano_of.of2_dx = *((s16 *)(data + 6));
			ano_of.of2_dy = *((s16 *)(data + 8));
			ano_of.of2_dx_fix = *((s16 *)(data + 10));
			ano_of.of2_dy_fix = *((s16 *)(data + 12));
			ano_of.intergral_x = *((s16 *)(data + 14));
			ano_of.intergral_y = *((s16 *)(data + 16));
			ano_of.of_quality = *(data + 18);
			//
		}
	}
	else if (*(data + 2) == 0X34) //高度信息
	{
		ano_of.of_alt_cm = *((u32 *)(data + 7));
		//
		check_time_ms[2] = 0;
		ano_of.alt_update_cnt++;
	}
	else if (*(data + 2) == 0X01) //惯性数据
	{
		ano_of.acc_data_x = *((s16 *)(data + 4));
		ano_of.acc_data_y = *((s16 *)(data + 6));
		ano_of.acc_data_z = *((s16 *)(data + 8));
		ano_of.gyr_data_x = *((s16 *)(data + 10));
		ano_of.gyr_data_y = *((s16 *)(data + 12));
		ano_of.gyr_data_z = *((s16 *)(data + 14));
		//shock_sta+16
	}
	else if (*(data + 2) == 0X04) //姿态信息
	{
		//四元数格式
		ano_of.quaternion[0] = (*((s16 *)(data + 4))) * 0.0001f;
		ano_of.quaternion[1] = (*((s16 *)(data + 6))) * 0.0001f;
		ano_of.quaternion[2] = (*((s16 *)(data + 8))) * 0.0001f;
		ano_of.quaternion[3] = (*((s16 *)(data + 10))) * 0.0001f;
	}
}


void DrvAnoOFGetOneByte_ptv7(const uint8_t linktype, const uint8_t data)
{
	static uint8_t _datatemp[OF_TEMP_LEN];
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		_datatemp[0] = data;
	}
	else if (rxstate == 1 && (data == ANOPTV8DEVID_MY || data == ANOPTV8DEVID_ALL))
	{
		rxstate = 2;
		_datatemp[1] = data;
	}
	else if (rxstate == 2)
	{
		rxstate = 3;
		_datatemp[2] = data;
	}
	else if (rxstate == 3 && data < OF_TEMP_LEN-5)
	{
		rxstate = 4;
		_datatemp[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (rxstate == 4 && _data_len > 0)
	{
		_data_len--;
		_datatemp[4 + _data_cnt++] = data;
		if (_data_len == 0)
			rxstate = 5;
	}
	else if (rxstate == 5)
	{
		rxstate = 6;
		_datatemp[4 + _data_cnt++] = data;
	}
	else if (rxstate == 6)
	{
		rxstate = 0;
		_datatemp[4 + _data_cnt] = data;
		DrvAnoOFDataAnl_ptv7(_datatemp, _data_cnt + 5); //
	}
	else
	{
		rxstate = 0;
	}
}
