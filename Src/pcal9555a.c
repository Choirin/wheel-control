#include "pcal9555a.h"

I2C_HandleTypeDef hi2c1;
uint8_t buf_ = 0x00;

void PCAL9555A_write_byte_data_at(uint8_t reg, uint8_t data) {
  uint8_t val[1];
  val[0]=data;
  HAL_I2C_Mem_Write(&hi2c1, PCAL9555A_Address, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 1000);
}

void PCAL9555A_init(void) {
    PCAL9555A_write_byte_data_at(PCAL9555A_REG_OUTPUT_PORT_CONFIG, 0x03);
    PCAL9555A_write_byte_data_at(PCAL9555A_REG_OUTPUT_PORT_1, 0x00);
    PCAL9555A_write_byte_data_at(PCAL9555A_REG_CONFIGURATION_PORT_1, 0x00);
}

void PCAL9555A_enable(uint8_t port, uint8_t output) {
    if (7 < port) {
        return;
    }

    if (1 == output) {
        buf_ |= (0x01 << port);
    } else {
        buf_ &= ~(0x01 << port);
    }

    PCAL9555A_write_byte_data_at(PCAL9555A_REG_OUTPUT_PORT_1, buf_);
}
