#ifndef __ANO_DRV_UBLOX_GPS_H__
#define __ANO_DRV_UBLOX_GPS_H__

#include "SysConfig.h"

typedef struct
{
  uint32_t ITOW;      // ms   GPS Millisecond Time of week   U4
  int32_t Longitude; // deg  Longitude                      i4 1e-7
  int32_t Latitude;  // deg  Latitude                       i4 1e-7
  int32_t Height;    // mm   Height above ellipsoid                      地球高度
  int32_t HMSL;      // mm   Height above mean sea level                 海拔高度
  uint32_t HAcc;      // mm   Horizontal accuracy estimate
  uint32_t VAcc;      // mm   Vertical accuracy estimate
  uint16_t  Statuds;
} UBXPOSLLH_t;

typedef struct
{
  uint32_t ITOW;      // ms   GPS Millisecond Time of week
  int32_t FTOW;      // ns   remainder of rounded ms above
  int16_t week;      // week GPS week num
  uint8_t GPSfix;     //      GPSfix Type ,range 0...6
  uint8_t Flags;      //      fix status flags
  int32_t ECEF_X;    // cm   ECEF X coordinate
  int32_t ECEF_Y;    // cm   ECEF Y coordinate
  int32_t ECEF_Z;    // cm   ECEF Z coordinate
  uint32_t PAcc;      // cm   3D Position Accuracy Estimate
  int32_t ECEFVX;    // cm/s ECEF X velocity
  int32_t ECEFVY;    // cm/s ECEF Y velocity
  int32_t ECEFVZ;    // cm/s ECEF Z velocity
  uint32_t SAcc;      // cm/s speed accuracy estimate
  uint16_t  PDOP;      //      position DOP                     0.01
  uint8_t reserved1;  //      Reserved
  uint8_t numSV;      //      Number of SVs used in Nav Solution
  uint32_t reserved2; //      Reserved
  uint16_t  Statuds;   //    len
} UBXSOL_t;

typedef struct
{
  uint32_t ITOW;        // ms   GPS time of week of the navigation epoch.
  int32_t VEL_N;       // cm/s North velocity component
  int32_t VEL_E;       // cm/s East velocity component
  int32_t VEL_D;       // cm/s Down velocity component
  uint32_t Speed;       // cm/s Speed(3-D)
  uint32_t GroundSpeed; // cm/s Ground speed (2-D)
  int32_t Heading;     // deg  Heading of motion 2-D
  uint32_t SAcc;        // cm/s Speed accuracy Estimate
  uint32_t CAcc;        // deg  Course/Heading accuracy estimate
  uint16_t  Statuds;

} UBXVELNED_t;

typedef struct
{

  uint32_t ITOW;      // ms
  uint16_t gDOP; //Geometric  DOP  0.01
  uint16_t pDOP; //Position   DOP  0.01
  uint16_t tDOP; //Time       DOP  0.01
  uint16_t vDOP; //Vertical   DOP  0.01
  uint16_t hDOP; //Horizontal DOP  0.01
  uint16_t nDOP; //Northing   DOP  0.01
  uint16_t eDOP; //Easting    DOP  0.01
  uint16_t  Statuds;   //
} UBXDOP_t;

typedef struct
{

  uint32_t ITOW;    // ms
  uint32_t TAcc;    // ns
  int32_t NANO;    // ns
  uint16_t  Year;    // y
  uint8_t Month;    // month
  uint8_t Day;      // d
  uint8_t Hour;     // h
  uint8_t Min;      // min
  uint8_t Sec;      // s
  uint8_t Valid;    //
  uint16_t  Statuds; //
} UBXTIMEUTC_t;

typedef struct
{
  uint32_t iTOW; //ms
  uint16_t  year; //y
  uint8_t month; //m
  uint8_t day;   //d
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;   //Validity Flags (see graphic below)
  uint32_t tAcc;   // ns Time accuracy estimate (UTC)
  int32_t nano;   // ns Fraction of second, range -1e9 .. 1e9 (UTC)
  uint8_t fixType; //
  /*
	GNSSfix Type, range 0..5
	0x00 = No Fix
	0x01 = Dead Reckoning only
	0x02 = 2D-Fix
	0x03 = 3D-Fix
	0x04 = GNSS + dead reckoning combined
	0x05 = Time only fix
	0x06..0xff: reserved
	*/
  uint8_t flags;     // - Fix Status Flags (see graphic below)
  uint8_t reserved1; // - Reserved
  uint8_t numSV;     // - Number of satellites used in Nav Solution
  int32_t lon;      //1e-7 deg Longitude
  int32_t lat;      //1e-7 deg Latitude
  int32_t height;   // mm Height above ellipsoid
  int32_t hMSL;     // mm Height above mean sea level
  uint32_t hAcc;     // mm Horizontal accuracy estimate
  uint32_t vAcc;     // mm Vertical accuracy estimate
  int32_t velN;     // mm/s NED north velocity
  int32_t velE;     // mm/s NED east velocity
  int32_t velD;     // mm/s NED down velocity
  int32_t gSpeed;   // mm/s Ground Speed (2-D)
  int32_t headMot;  // 1e-5 deg Heading of motion (2-D)
  uint32_t sAcc;     // mm/s Speed accuracy estimate
  uint32_t headAcc;  // 1e-5 deg Heading accuracy estimate (both motion and vehicle)
  uint16_t  pDOP;     // 0.01 - Position DOP
  //	uint16_t  reserved2;// - Reserved
  //	uint32_t reserved3;// - Reserved
  //	int32_t headVeh;// deg Heading of vehicle (2-D)1e-5
  //	uint32_t reserved4;// - Reserved

} __attribute__((packed)) _UBXPVT_st;

typedef struct
{
	uint8_t gnssid;
	uint8_t svid;
	uint8_t cno;
	int8_t elev;
	int16_t azim;
	int16_t prRes;
	uint32_t flag;
}__attribute__((packed)) _UBXSAT_st;
//typedef struct
//{
//	uint8_t updata_cnt;
//	uint8_t state;//0:offline 1:online but unavailable 2: online and availabal 3:online and works in a good state
//
//	uint8_t fixType;//
//	uint8_t numSV;// - Number of satellites used in Nav Solution
//	int32_t lon;//1e-7 deg Longitude
//	int32_t lat;//1e-7 deg Latitude
////	int32_t height;// mm Height above ellipsoid
//	int32_t hMSL;// mm Height above mean sea level
//	uint32_t hAcc;// mm Horizontal accuracy estimate
//	uint32_t vAcc;// mm Vertical accuracy estimate
//	int32_t velN;// mm/s NWU north velocity
//	int32_t velW;// mm/s NWU west velocity
//	int32_t velU;// mm/s NWU up velocity
////	int32_t gSpeed;// mm/s Ground Speed (2-D)
////	int32_t headMot;// 1e-5 deg Heading of motion (2-D)
//	uint32_t sAcc;// mm/s Speed accuracy estimate
////	uint32_t headAcc;// 1e-5 deg Heading accuracy estimate (both motion and vehicle)
//	uint16_t  pDOP;// 0.01 - Position DOP
//

//}_ubx_user_data_st;
//extern _ubx_user_data_st ubx_user_data;

//
void GPS_Rate_H(void);
void GPS_Rate_L(void);
//public
void Init_GPS(void);
void GPS_Data_Prepare_Task(uint8_t dT_ms);
void UBLOX_M8_GPS_Data_Receive(const uint8_t linktype, const uint8_t Data);

#endif
