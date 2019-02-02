#include "stm32f4xx_hal.h"

#define PCAL9555A_Address    0x40
#define PCAL9555A_REG_INPUT_PORT_0         0x00
#define PCAL9555A_REG_INPUT_PORT_1         0x01
#define PCAL9555A_REG_OUTPUT_PORT_0        0x02
#define PCAL9555A_REG_OUTPUT_PORT_1        0x03
#define PCAL9555A_REG_POLARITY_INVERSION_0 0x04
#define PCAL9555A_REG_POLARITY_INVERSION_1 0x05
#define PCAL9555A_REG_CONFIGURATION_PORT_0 0x06
#define PCAL9555A_REG_CONFIGURATION_PORT_1 0x07

#define PCAL9555A_REG_PUPD_ENABLE_0        0x46
#define PCAL9555A_REG_PUPD_ENABLE_1        0x47
#define PCAL9555A_REG_OUTPUT_PORT_CONFIG   0x4F

void PCAL9555A_write_byte_data_at(uint8_t reg, uint8_t data) ;
void PCAL9555A_init(void);
void PCAL9555A_enable(uint8_t port, uint8_t output);
