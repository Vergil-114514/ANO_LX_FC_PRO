#include "DrvIna3221.h"
#include "Drv_SI2c.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define INA3221_ADDRESS                         (0x40<<1)    // 1000000 (A0+A1=GND)
#define INA3221_READ                            (0x01)
/*=========================================================================*/
#define INA3221_REG_CONFIG                      (0x00)
#define INA3221_REG_SHUNTVOLTAGE_1              (0x01)
#define INA3221_REG_BUSVOLTAGE_1                (0x02)
#define INA3221_REG_CRITICAL_ALERT_1            (0x07)
#define INA3221_REG_WARNING_ALERT_1             (0x08)
#define INA3221_REG_SHUNT_VOLTAGE_SUM           (0x0D)
#define INA3221_REG_SHUNT_VOLTAGE_SUM_LIMIT     (0x0E)
#define INA3221_REG_MASK                        (0x0F)
#define INA3221_REG_VALID_POWER_UPPER_LIMIT     (0x10)
#define INA3221_REG_VALID_POWER_LOWER_LIMIT     (0x11)
/*=========================================================================
    CONFIG REGISTER (R/W)
    -----------------------------------------------------------------------*/
#define INA3221_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA3221_CONFIG_ENABLE_CHAN1             (0x4000)  // Enable Channel 1
#define INA3221_CONFIG_ENABLE_CHAN2             (0x2000)  // Enable Channel 2
#define INA3221_CONFIG_ENABLE_CHAN3             (0x1000)  // Enable Channel 3

#define INA3221_CONFIG_AVG2                     (0x0800)  // AVG Samples Bit 2 - See table 3 spec
#define INA3221_CONFIG_AVG1                     (0x0400)  // AVG Samples Bit 1 - See table 3 spec
#define INA3221_CONFIG_AVG0                     (0x0200)  // AVG Samples Bit 0 - See table 3 spec

#define INA3221_CONFIG_VBUS_CT2                 (0x0100)  // VBUS bit 2 Conversion time - See table 4 spec
#define INA3221_CONFIG_VBUS_CT1                 (0x0080)  // VBUS bit 1 Conversion time - See table 4 spec
#define INA3221_CONFIG_VBUS_CT0                 (0x0040)  // VBUS bit 0 Conversion time - See table 4 spec

#define INA3221_CONFIG_VSH_CT2                  (0x0020)  // Vshunt bit 2 Conversion time - See table 5 spec
#define INA3221_CONFIG_VSH_CT1                  (0x0010)  // Vshunt bit 1 Conversion time - See table 5 spec
#define INA3221_CONFIG_VSH_CT0                  (0x0008)  // Vshunt bit 0 Conversion time - See table 5 spec

#define INA3221_CONFIG_MODE_2                   (0x0004)  // Operating Mode bit 2 - See table 6 spec
#define INA3221_CONFIG_MODE_1                   (0x0002)  // Operating Mode bit 1 - See table 6 spec
#define INA3221_CONFIG_MODE_0                   (0x0001)  // Operating Mode bit 0 - See table 6 spec
/*=========================================================================*/

/*=========================================================================
    SHUNT VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA3221_REG_SHUNTVOLTAGE_1                (0x01)
/*=========================================================================*/

/*=========================================================================
    BUS VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA3221_REG_BUSVOLTAGE_1                  (0x02)
/*=========================================================================*/
#define SHUNT_100m  (0.1)   // default shunt resistor value of 0.1 Ohm
#define SHUNT_10m  (0.01)   // default shunt resistor value of 0.01 Ohm
#define DC                            (0x00)
#define AC                            (0x01)


float Ina3221Cur[3];

void ina3221WriteRegister16(uint8_t reg, uint16_t val) {
	uint8_t _v[2];
	_v[0] = (val >> 8) & 0xFF;
	_v[1] = val & 0xFF;
	DrvSI2c_WriteNByte(INA3221_ADDRESS, reg, 2, _v);
}

uint16_t ina3221ReadRegister16(uint8_t reg) {
	uint16_t value;
	uint8_t _v[2] = {0};
	DrvSI2c_ReadNByte(INA3221_ADDRESS, reg, 2, _v);
	value = ((uint16_t)_v[0] << 8) + _v[1];
	return value;
}

void DrvIna3221Init(void)
{
	DrvSI2cInit();
	uint16_t _id = ina3221ReadRegister16(0xFF);
	if(_id != 0x3220)
	{
		return;
	}
	uint16_t config = INA3221_CONFIG_ENABLE_CHAN1 |
                      INA3221_CONFIG_ENABLE_CHAN2 |
                      INA3221_CONFIG_ENABLE_CHAN3 |
                      INA3221_CONFIG_AVG1 |
                      INA3221_CONFIG_VBUS_CT2 |
                      INA3221_CONFIG_VSH_CT2 |
                      INA3221_CONFIG_MODE_2 |
                      INA3221_CONFIG_MODE_1 |
                      INA3221_CONFIG_MODE_0;
    ina3221WriteRegister16(INA3221_REG_CONFIG, config);
	
}
void DrvIna3221GetVals(void)
{
	
}
