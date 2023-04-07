/*
 * LPS331AP.c
 *
 *  Created on: Apr 3, 2023
 *      Author: barte
 */
#include "main.h"
#include"LPS331AP.h"


uint8_t Read8(LPS331AP_t *lps, uint8_t Register)
{
	uint8_t Value;



	HAL_I2C_Mem_Read(lps->lps_i2c, ((lps->Address)<<1), Register, 1, &Value, 1, LPS331AP_I2C_TIMEOUT);
	return Value;

}

void Write8(LPS331AP_t *lps, uint8_t Register, uint8_t Value)
{
	HAL_I2C_Mem_Write(lps->lps_i2c, ((lps->Address)<<1), Register, 1, &Value, 1, LPS331AP_I2C_TIMEOUT);
}

void BMP280_SetREG_CTRL1_InitConfiguration(LPS331AP_t *lps, uint8_t Mode)
{
	/*Power down device before changing ODR register*/
	Write8(lps, LPS331AP_CTRL_REG1, LPS331AP_MODE_OFF);
	Write8(lps, LPS331AP_CTRL_REG1, Mode);
}

void BMP280_SetTemperatureOversampling(LPS331AP_t *lps, uint8_t TOversampling)
{
	uint8_t Tmp;
	//if(TOversampling > 1) Mode = 1;
	Tmp = Read8(lps, LPS331AP_CTRL_REG1);
	Tmp = Tmp & 0x8F;
	Tmp |= TOversampling;
	Write8(lps, LPS331AP_CTRL_REG1, Tmp);
}

uint8_t LPS331AP_Init(LPS331AP_t *lps, I2C_HandleTypeDef *i2c, uint8_t address)
{
	uint8_t ChipID;
	lps->lps_i2c = i2c;
	lps->Address = address;

	ChipID = Read8(lps, LPS331AP_WHO_AM_I);
	if(ChipID != 0xBB)
	{
		return 1;
	}
	/*set ctrl reggister*/
	BMP280_SetREG_CTRL1(lps, LPS331AP_SET_REGCTRL1);
	return 0;
}

