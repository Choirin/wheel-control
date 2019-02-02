#include "stm32f4xx_hal.h"

#define VL53L0X_Address    0x52
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS        0x8A

void Vl53L0X_SetDeviceAddress(uint8_t addr_);
uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg);
uint16_t convuint16(int lsb, int msb);
void write_byte_data_at(uint8_t reg, uint8_t data);
uint8_t read_byte_data_at(uint8_t reg);
void read_block_data_at(uint8_t reg, int sz);
void Vl53L0X_Test(void);
void Vl53L0X_Set(void);
uint16_t Vl53L0X_Read(uint16_t *pdist);
uint8_t VL53L0X_Address_Test(void);
