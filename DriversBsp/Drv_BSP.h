#ifndef _DRVBSP_H_
#define _DRVBSP_H_
#include "SysConfig.h"
#include "Drv_Sys.h"
#include "LX_LowLevelFunc.h"

typedef struct
{
	uint8_t sig_mode; //0==null,1==ppm,2==sbus
	//
	int16_t ppm_ch[9];
	//
	int16_t sbus_ch[16];
	uint8_t sbus_flag;
	//
	int16_t crsf_ch[16];
	uint8_t crsf_flag;
	//
	uint16_t  signal_fre;
	uint8_t no_signal;
	uint8_t fail_safe;
	_rc_ch_un rc_ch;
	uint16_t  signal_cnt_tmp;
	uint8_t rc_in_mode_tmp;
} _rc_input_st;

//==Êý¾ÝÉùÃ÷
extern _rc_input_st rc_in;

typedef union {
    uint8_t rawdata[25];
    struct{
			uint32_t head : 8;
			uint32_t ch0 : 11;
			uint32_t ch1 : 11;
			uint32_t ch2 : 11;
			uint32_t ch3 : 11;
			uint32_t ch4 : 11;
			uint32_t ch5 : 11;
			uint32_t ch6 : 11;
			uint32_t ch7 : 11;
			uint32_t ch8 : 11;
			uint32_t ch9 : 11;
			uint32_t ch10 : 11;
			uint32_t ch11 : 11;
			uint32_t ch12 : 11;
			uint32_t ch13 : 11;
			uint32_t ch14 : 11;
			uint32_t ch15 : 11;
			uint32_t flag : 8;
			uint32_t end : 8;
		}__attribute__((__packed__)) stdata;
}  _rc_sbus_un;
extern _rc_sbus_un sbus_in;

extern uint8_t EmergencyStopESC;

uint8_t All_Init(void);

void DrvRcInputInit(void);
void DrvPpmGetOneCh(uint16_t  data);
void DrvSbusGetOneByte(uint8_t data);
void DrvRcInputTask(float dT_s);

#endif
