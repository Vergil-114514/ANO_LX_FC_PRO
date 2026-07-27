#ifndef __ANO_LX_H
#define __ANO_LX_H
//==引用
#include "McuConfig.h"

//==定义/声明


enum
{
    ch_1_rol=0,
    ch_2_pit,
    ch_3_thr,
    ch_4_yaw,
    ch_5_aux1,
    ch_6_aux2,
    ch_7_aux3,
    ch_8_aux4,
    ch_9_aux5,
    ch_10_aux6,
};

//0x40
typedef struct
{
    int16_t ch_[10]; //

} __attribute__ ((__packed__)) _rc_ch_st;

typedef union
{
    uint8_t byte_data[20];
    _rc_ch_st st_data;
} _rc_ch_un;

//0x41
typedef struct
{
    int16_t rol;
    int16_t pit;
    int16_t thr;
    int16_t yaw_dps;
    int16_t vel_x;
    int16_t vel_y;
    int16_t vel_z;

} __attribute__ ((__packed__)) _rt_tar_st;

typedef union
{
    uint8_t byte_data[14];
    _rt_tar_st st_data;
} _rt_tar_un;

//0x0D
typedef struct
{
    uint16_t  voltage_100;
    uint16_t  current_100;

} __attribute__ ((__packed__)) _fc_bat_st;

typedef union
{
    uint8_t byte_data[4];
    _fc_bat_st st_data;
} _fc_bat_un;

//0x03
typedef struct
{
    int16_t rol_x100;
    int16_t pit_x100;
    int16_t yaw_x100;
    uint8_t state;
} __attribute__ ((__packed__)) _fc_att_st;

typedef union
{
    uint8_t byte_data[7];
    _fc_att_st st_data;
} _fc_att_un;

//0x04
typedef struct
{
    int16_t w_x10000;
    int16_t x_x10000;
    int16_t y_x10000;
    int16_t z_x10000;
    uint8_t state;
} __attribute__ ((__packed__)) _fc_att_qua_st;

typedef union
{
    uint8_t byte_data[9];
    _fc_att_qua_st st_data;
} _fc_att_qua_un;

//0x05
typedef struct
{
    int32_t fus;
    int32_t add;
    uint8_t state;
} __attribute__ ((__packed__)) _fc_alt_st;

typedef union
{
    uint8_t byte_data[9];
    _fc_alt_st st_data;
} _fc_alt_un;

//0x07
typedef struct
{
    int16_t vel_x;
    int16_t vel_y;
    int16_t vel_z;

} __attribute__ ((__packed__)) _fc_vel_st;

typedef union
{
    uint8_t byte_data[6];
    _fc_vel_st st_data;
} _fc_vel_un;
//
typedef struct
{
    uint16_t  pwm_m1;
    uint16_t  pwm_m2;
    uint16_t  pwm_m3;
    uint16_t  pwm_m4;
    uint16_t  pwm_m5;
    uint16_t  pwm_m6;
    uint16_t  pwm_m7;
    uint16_t  pwm_m8;
} _pwm_st;

//==数据声明
extern _fc_att_un fc_att;
extern _fc_att_qua_un fc_att_qua;
extern _fc_vel_un fc_vel;
extern _fc_alt_un fc_alt;
extern _rt_tar_un rt_tar;
extern _fc_bat_un fc_bat;
extern _pwm_st pwm_to_esc;
//==函数声明
//static


//public
void ANO_LX_Task(void);

#endif

