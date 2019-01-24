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

I2C_HandleTypeDef hi2c1;
uint8_t buf[16];
uint8_t addr = VL53L0X_Address;

void Vl53L0X_SetDeviceAddress(uint8_t addr_)
{
  addr = addr_;
}

uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}

uint16_t convuint16(int lsb, int msb) {
  return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void write_byte_data_at(uint8_t reg, uint8_t data) {
  uint8_t val[1];
  val[0]=data;
  HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 1000);
}

uint8_t read_byte_data_at(uint8_t reg) {
  uint8_t value;
  HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
  return value;
}

void read_block_data_at(uint8_t reg, int sz) {
  HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, sz, 1000);
}

void Vl53L0X_Test(void){
  uint8_t val1 ;
  //CHK Param
  val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
  printf("Revision ID: %d, ",val1);

  val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
  printf("Device ID:  %d \n",val1);

  val1 = read_byte_data_at(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
  printf("PRE_RANGE_CONFIG_VCSEL_PERIOD: %d \n",val1);
  printf(" decode:   %d \n",VL53L0X_decode_vcsel_period(val1));

  val1 = read_byte_data_at(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
  printf("FINAL_RANGE_CONFIG_VCSEL_PERIOD: %d \n",val1);
  printf(" decode:   %d \n",VL53L0X_decode_vcsel_period(val1));

  //Init Start
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  uint8_t val = 0;
  int cnt = 0;
  while (cnt < 100) { // 1 second waiting time max
    HAL_Delay(10);
    val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    if (val & 0x01) break;
      cnt++;
  }
  if (val & 0x01) printf("Ready!! \n"); else printf("Not Ready!!");

  read_block_data_at(0x14, 12);
  uint16_t acnt = convuint16(buf[7], buf[6]);
  uint16_t scnt = convuint16(buf[9], buf[8]);
  uint16_t dist = convuint16(buf[11], buf[10]);
  uint8_t DeviceRangeStatusInternal = ((buf[0] & 0x78) >> 3);

  //printf("ambient count: %d, signal count: %d, distance: %d, status: %d  \n",
  //        acnt,scnt,dist,DeviceRangeStatusInternal);
  printf("RES: %d mm  \n",dist);
}

void Vl53L0X_Set(void){
  uint8_t val1 ;
  //Init Start
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);
}

uint16_t Vl53L0X_Read(uint16_t *pdist){
  uint8_t val = 0;
  val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
  if (!(val & 0x01)) return 0;

  read_block_data_at(0x14, 12);
  uint16_t acnt = convuint16(buf[7], buf[6]);
  uint16_t scnt = convuint16(buf[9], buf[8]);
  uint16_t dist = convuint16(buf[11], buf[10]);
  uint8_t DeviceRangeStatusInternal = ((buf[0] & 0x78) >> 3);

  *pdist = dist;

  return 1;
}

uint8_t VL53L0X_Address_Test(void){
  uint8_t tmp[2];
  HAL_I2C_Mem_Read(&hi2c1, addr, 0xC1, I2C_MEMADD_SIZE_8BIT, tmp, 1, 100);
   
  if(tmp[0]==0xAA){
    printf("VL53L0X is Found! \n");
    return 1;
  }
  else{
    printf("VL53L0X is NOT Found! ERROR! \n");
    return 0;
  }
}