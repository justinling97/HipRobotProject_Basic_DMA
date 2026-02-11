#include "main.h"
#include "MB85RC1M.h"
#include <string.h>



uint8_t MB85RC16_Default_I2C_Addr  =  0xA0; //Pin A2=A1=0

 

extern uint32_t MB85RC1M_Access_Addr;   //FRAM MB85RC1M access address (17-bit)


extern I2C_HandleTypeDef hi2c1;


void MB85RC1M_Write(uint32_t addr, uint8_t * data, uint32_t len)
{
	uint8_t MB85RC16_I2C_Addr;

	MB85RC16_I2C_Addr = MB85RC16_Default_I2C_Addr | ((addr>>8)<<1); //high 3-bit access address placed into I2C address

	uint8_t TD[len+1];
	TD[0] = addr & 0x00FF;  //low 8-bit access address placed into I2C first data

	memcpy(TD+1, data, len);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // 将引脚5设置为低电平

	HAL_I2C_Master_Transmit(&hi2c1, MB85RC16_I2C_Addr, TD, len+1, 2700);  //Write data
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);


}


void MB85RC1M_Read(uint32_t addr, uint8_t * data, uint32_t len)
{
uint8_t MB85RC16_I2C_Addr;

	MB85RC16_I2C_Addr = MB85RC16_Default_I2C_Addr | ((addr>>8)<<1); //high 3-bit access address placed into I2C address

	uint8_t RA[1];
	RA[0] = addr & 0x00FF;  //low 8-bit access address placed into I2C first data

	HAL_I2C_Master_Transmit(&hi2c1, MB85RC16_I2C_Addr, &RA[0], 1, 2700); //Write address for read
	HAL_I2C_Master_Receive(&hi2c1, MB85RC16_I2C_Addr, data, len, 2700); //Read data


}
