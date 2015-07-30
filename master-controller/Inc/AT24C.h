#include "stm32f1xx_hal.h"
#include <stdbool.h>

bool AT24C_Read(I2C_HandleTypeDef * i2c, uint16_t adress, uint8_t * buff, uint16_t len);

bool AT24C_Write(I2C_HandleTypeDef * i2c, uint16_t adress, uint8_t * buff, uint16_t len);
