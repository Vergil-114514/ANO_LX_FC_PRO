#include "Drv_AnoOf.h"

_ano_of_st ano_of;
static float check_time_ms[3];
void AnoOF_Check_State(float dT_s)
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

#include "AnoPTv8.h"
#include "AnoPTv8Run.h"
void AnoOFFrameAnl(const uint8_t linktype, const _un_frame_v8 *p)
{
	uint8_t _fid = p->frame.frameid;
    switch(_fid)
    {
		case 0x51:
		{
			//光流信息
			if (p->frame.data[0] == 0) //原始光流信息
			{
				ano_of.of0_sta = *(p->frame.data + 1);
				ano_of.of0_dx = *(p->frame.data + 2);
				ano_of.of0_dy = *(p->frame.data + 3);
				ano_of.of_quality = *(p->frame.data + 4);
			}
			else if (p->frame.data[0] == 1) //高度融合后光流信息
			{
				ano_of.of1_sta = *(p->frame.data + 1);
				ano_of.of1_dx = *((int16_t *)(p->frame.data + 2));
				ano_of.of1_dy = *((int16_t *)(p->frame.data + 4));
				ano_of.of_quality = *(p->frame.data + 6);
				//
				check_time_ms[1] = 0;
				ano_of.of_update_cnt++;
			}
			else if (p->frame.data[0] == 2) //惯导融合后光流信息
			{
				ano_of.of2_sta = *(p->frame.data + 1);
				ano_of.of2_dx = *((int16_t *)(p->frame.data + 2));
				ano_of.of2_dy = *((int16_t *)(p->frame.data + 4));
				ano_of.of2_dx_fix = *((int16_t *)(p->frame.data + 6));
				ano_of.of2_dy_fix = *((int16_t *)(p->frame.data + 8));
				ano_of.intergral_x = *((int16_t *)(p->frame.data + 10));
				ano_of.intergral_y = *((int16_t *)(p->frame.data + 12));
				ano_of.of_quality = *(p->frame.data + 14);
				//
			}
			break;
		}
		case 0X34:
		{
			//高度信息
			ano_of.of_alt_cm = *((int32_t *)(p->frame.data + 3));
			//
			check_time_ms[2] = 0;
			ano_of.alt_update_cnt++;
			break;
		}
		case 0X01:
		{
			//惯性数据
			ano_of.acc_data_x = *((int16_t *)(p->frame.data + 0));
			ano_of.acc_data_y = *((int16_t *)(p->frame.data + 2));
			ano_of.acc_data_z = *((int16_t *)(p->frame.data + 4));
			ano_of.gyr_data_x = *((int16_t *)(p->frame.data + 6));
			ano_of.gyr_data_y = *((int16_t *)(p->frame.data + 8));
			ano_of.gyr_data_z = *((int16_t *)(p->frame.data + 10));
			break;
		}
		case 0X04:
		{
			//姿态信息
			ano_of.quaternion[0] = (*((int16_t *)(p->frame.data + 0))) * 0.0001f;
			ano_of.quaternion[1] = (*((int16_t *)(p->frame.data + 2))) * 0.0001f;
			ano_of.quaternion[2] = (*((int16_t *)(p->frame.data + 4))) * 0.0001f;
			ano_of.quaternion[3] = (*((int16_t *)(p->frame.data + 6))) * 0.0001f;
			break;
		}
	}
}


