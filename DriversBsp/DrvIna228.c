#include "DrvIna228.h"
#include "Drv_SI2c.h"

#define INA228_I2C_ADDR 0x45  // 默认I2C地址

// INA228寄存器地址
#define INA228_CONFIG      0x00
#define INA228_ADC_CONFIG  0x01
#define INA228_SHUNT_CAL   0x02
#define INA228_SHUNT_TEMPCO 0x03
#define INA228_VSHUNT      0x04
#define INA228_VBUS       0x05
#define INA228_DIETEMP     0x06
#define INA228_CURRENT     0x07
#define INA228_POWER       0x08
#define INA228_ENERGY      0x09
#define INA228_CHARGE      0x0A
#define INA228_DIAG_ALRT   0x0B
#define INA228_SOVL        0x0C
#define INA228_SUVL        0x0D
#define INA228_BOVL        0x0E
#define INA228_BUVL        0x0F
#define INA228_TEMP_LIMIT  0x10
#define INA228_PWR_LIMIT   0x11
#define INA228_MANUFACTURER_ID 0x3E
#define INA228_DEVICE_ID    0x3F

// ADC平均次数配置（0=1次，1=4次，2=16次，3=64次，4=128次，5=256次，6=512次，7=1024次）
#define INA228_AVG_MODE    7         // 选择1024次平均（最高精度）
#define INA228_MODE_CONTINUOUS 0x7   // 连续转换模式（Shunt+Bus+Temp）

uint8_t Ina228InitOk = 0;
float Ina228Cur;

uint16_t ina228ReadReg16(uint8_t reg)
{
	uint16_t value;
	uint8_t _v[2] = {0};
	DrvSI2c_ReadNByte(INA228_I2C_ADDR, reg, 2, _v);
	value = ((uint16_t)_v[0] << 8) + _v[1];
	return value;
}

void ina228WriteReg16(uint8_t reg, uint16_t val)
{
	uint8_t _v[2];
	_v[0] = (val >> 8) & 0xFF;
	_v[1] = val & 0xFF;
	DrvSI2c_WriteNByte(INA228_I2C_ADDR, reg, 2, _v);
}

// 重置INA228
void DrvIna228_Reset(void) {
    ina228WriteReg16(INA228_CONFIG, 0x8000);
    // 等待复位完成
    // 这里可以添加适当的延时
	MyDelayMs(20);
}

// 读取分流电压（V）
float ina228_ReadShuntVoltage(void) {
	uint8_t _v[3] = {0};
	DrvSI2c_ReadNByte(INA228_I2C_ADDR, INA228_VSHUNT, 3, _v);
	int32_t _i32 = ((int32_t)_v[0]<<16) | ((int32_t)_v[1]<<8) | ((int32_t)_v[2]);
	_i32 &= 0x000FFFFF; // 保留低20位
	int32_t value = (int32_t)(_i32 << 12) >> 16;
    return (float)value * 312.5e-9;  // 312.5nV/LSB
}

// 读取总线电压（V）
float ina228_ReadBusVoltage(void) {
    uint16_t raw = ina228ReadReg16(INA228_VBUS);
    return raw * 195.3125e-6;  // 195.3125μV/LSB (50V范围)
}

// 读取电流（A）
float ina228_ReadCurrent(void) {
	uint8_t _v[3] = {0};
	DrvSI2c_ReadNByte(INA228_I2C_ADDR, INA228_CURRENT, 3, _v);
	int32_t _i32 = ((int32_t)_v[0]<<16) | ((int32_t)_v[1]<<8) | ((int32_t)_v[2]);
	_i32 &= 0x000FFFFF; // 保留低20位
	int32_t value = (int32_t)(_i32 << 12) >> 16;
    return (float)value * 204.8f/524288.0f;  
}

// 读取功率（W）
float ina228_ReadPower(void) {
    uint32_t raw = ina228ReadReg16(INA228_POWER);
    return raw * 0.001 * 3.2;  // POWER_LSB = CURRENT_LSB * 3.2
}

void DrvIna228Init(void)
{
	DrvSI2cInit();
	uint16_t _id = ina228ReadReg16(INA228_DEVICE_ID);
	if(_id < 0x2280 || _id >0x228F)
	{
		return;
	}
	
	DrvIna228_Reset();
	// ADC Configuration (0x01)
	// 二进制: 1111 1011 0110 1011 (0xFB6B)
	// bit[15:12] = 1111 (连续测量 VBUS, VSHUNT, TEMP)
	// bit[11:9]  = 101 (VBUS 转换时间 1.1ms)
	// bit[8:6]   = 101 (VSHUNT 转换时间 1.1ms)
	// bit[5:3]   = 101 (温度转换时间 1.1ms)
	// bit[2:0]   = 011 (64次平均)
	uint16_t adc_config_r = ina228ReadReg16(INA228_ADC_CONFIG);
    ina228WriteReg16(INA228_ADC_CONFIG, 0xFB6B);
	adc_config_r = ina228ReadReg16(INA228_ADC_CONFIG);

    // 计算SHUNT_CAL（0.2mΩ采样电阻）
    float current_lsb = 204.8f/524288.0f;
    float r_shunt = 0.0002f;          // 0.2mΩ
    uint16_t shunt_cal = (float)((float)13107.2e6 * current_lsb * r_shunt);
    ina228WriteReg16(INA228_SHUNT_CAL, shunt_cal);
	
	Ina228InitOk = 1;
}

void DrvIna228GetVals(void)
{
	if(!Ina228InitOk)
		return;
	Ina228Cur =ina228_ReadCurrent();
}
