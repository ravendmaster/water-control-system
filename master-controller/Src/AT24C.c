#include "stm32f1xx_hal.h"
#include "AT24C.h"

bool AT24C_Read(I2C_HandleTypeDef * i2c, uint16_t adress, uint8_t * buff, uint16_t len)
{
	if(HAL_I2C_Mem_Read(i2c, 174, adress, I2C_MEMADD_SIZE_16BIT, buff, len, 100)!= HAL_OK)
	{
		return true;
	}
	return false;
}

bool AT24C_Write(I2C_HandleTypeDef * i2c, uint16_t adress, uint8_t * buff, uint16_t len)
{
	if(HAL_I2C_Mem_Write(i2c, 174, adress, I2C_MEMADD_SIZE_16BIT, buff, len, 100)!= HAL_OK)
	{
		return true;
	}
	return false;
}
