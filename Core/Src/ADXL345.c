/*
 * ADXL345.c
 *
 *  Created on: Mar 31, 2022
 *      Author: MFatih
 *		e-mail: mfatihto@gmail.com
 *
 *      Sleep and Standby functions should be used after Initializing the ADXL345_Init_X (X --> SPI or I2C) function.
 *      Desired configurations can be made after the init functions.
 *
 */

#include "ADXL345.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

ADXL345_CorStruc Coordinates;
ADXL345_CorStruc CoordiantesCalib;

ADXL345_CorStruc Coordinates_SPI;
ADXL345_CorStruc CoordinatesCalib_SPI;

/* SPI Functions */

void ADXL345_WriteAddress8_SPI(uint8_t wRegister, uint8_t value)
{
	//wRegister |= 0x40;
	wRegister &= ~(0x80);
	//uint8_t data[2] = {};
	//data[0] = wRegister ;
	//data[1] = value;
	HAL_GPIO_WritePin(GPIOD, ADXL345_CSPIN, GPIO_PIN_RESET);   //cs pin is disabled to data transfer.
	HAL_SPI_Transmit(&hspi1, &wRegister, 1, 100);
	HAL_SPI_Transmit(&hspi1, &value, 1, 100);
	HAL_GPIO_WritePin(GPIOD, ADXL345_CSPIN, GPIO_PIN_SET);     //cs pin is enabled since the transition finished.
}

void ADXL345_ReadAddress16_SPI(uint8_t addr, int bytes)
{
	addr |= 0x80;        // To be able to read
	addr |= 0x40;        // To be able to do multi-byte reading
	uint8_t tmp8[6] = {};
	HAL_GPIO_WritePin(GPIOD, ADXL345_CSPIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
	HAL_SPI_Receive(&hspi1, &tmp8[0], bytes, 100);
	HAL_GPIO_WritePin(GPIOD, ADXL345_CSPIN, GPIO_PIN_SET);
	int i = 0;
	for(i=0;i<=5;i++)
	{
		Coordinates_SPI.xyz[i] = tmp8[i];
	}
}

uint8_t ADXL345_ReadAddress8_SPI(uint8_t addr)
{
	addr |= 0x80;
	addr |= 0x40;
	uint8_t tmp;
	HAL_GPIO_WritePin(GPIOD, ADXL345_CSPIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
	HAL_SPI_Receive(&hspi1, &tmp, 1, 100);
	HAL_GPIO_WritePin(GPIOD, ADXL345_CSPIN, GPIO_PIN_SET);
	return tmp;
}

void ADXL345_RegConfig_SPI()
{
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_POWER_CTL, 0);
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_POWER_CTL,((ADXL345_LINK << 5) | (ADXL345_AUTO_SLEEP << 4) | (1 << 3) | (1 << 2) | 1));
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_DATA_FORMAT, ((0 << 7) | (ADXL345_SPI << 6) | (ADXL345_INT_INVERT << 5) | (0 << 4) | (ADXL345_FULL_RES << 3) | (1 << 2) | ADXL345_RANGE));
}

void ADXL345_RegTapConfig_SPI()
{
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_TAP_AXES, ((0 << 3) | (ADXL345_TAP_X << 2) | (ADXL345_TAP_Y << 1) | (ADXL345_TAP_Z)));
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_THRESH_TAP, (uint8_t)ADXL345_THRESH_VAL);
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_DUR, (uint8_t)ADXL345_DUR_VAL);
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_LATENT, (uint8_t)ADXL345_LATENT_VAL);
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_INT_MAP, ADXL345_INT1);
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_INT_ENABLE, 0x00);
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_INT_ENABLE, ((ADXL345_SINGLE_TAP << 6) | (ADXL345_DOUBLE_TAP << 5)));
}

void ADXL345_Init_SPI()
{
	HAL_GPIO_WritePin(GPIOD, ADXL345_CSPIN, GPIO_PIN_SET);
	HAL_Delay(5);
	if(ADXL345_ReadAddress8_SPI(ADXL345_REGISTER_DEVID) == 0xE5)
	{
		ADXL345_RegConfig_SPI();
		ADXL345_RegTapConfig_SPI();
	}

}

void ADXL345_ReadAccValues_SPI()
{
	if(ADXL345_ReadAddress8_SPI(ADXL345_REGISTER_DEVID) == 0xE5)
	{
		ADXL345_ReadAddress16_SPI(ADXL345_REGISTER_DATAXYZ, 6);
		Coordinates_SPI.x = (( Coordinates_SPI.xyz[1] << 8) | Coordinates_SPI.xyz[0]);    // X-axis
		Coordinates_SPI.y = (( Coordinates_SPI.xyz[3]<< 8)	| Coordinates_SPI.xyz[2]);    // Y-axis
		Coordinates_SPI.z = (( Coordinates_SPI.xyz[5] << 8) | Coordinates_SPI.xyz[4]);    // Z-axis
		double sens_cnst = 0.0;
		if(ADXL345_RANGE == 0) sens_cnst = 3.9;
		else if(ADXL345_RANGE == 1) sens_cnst = 7.8;
		else if(ADXL345_RANGE == 2) sens_cnst = 15.6;
		else if(ADXL345_RANGE == 3) sens_cnst = 31.2;
		CoordinatesCalib_SPI.x = ((float)Coordinates_SPI.x * pow(10,-3) * sens_cnst);
		CoordinatesCalib_SPI.y = ((float)Coordinates_SPI.y * pow(10,-3) * sens_cnst);
		CoordinatesCalib_SPI.z = ((float)Coordinates_SPI.z * pow(10,-3) * sens_cnst);
	}
}
/* for ON_OFF param. take 1 to ON, 0  to OFF
 * sleepRate is the param. takes values (0 to 3):    ADXL345_SLEEP_RATE_1HZ
 * 													 ADXL345_SLEEP_RATE_2HZ
 * 													 ADXL345_SLEEP_RATE_4HZ
 * 													 ADXL345_SLEEP_RATE_8HZ
 */
void ADXL345_Sleep_SPI(int ON_OFF, uint8_t sleepRate)   // sleep rate for pow_ctl wake up bits, sleepRate = ADXL345_SLEEP_RATE_1HZ up to 8HZ in the same format.
{
	uint8_t tmp = 0;
	tmp = ADXL345_ReadAddress8_SPI(ADXL345_REGISTER_POWER_CTL);
	switch(ON_OFF)
	{
	case 1:                // ON
		tmp = (tmp | (1 << 2));
		tmp &= ~(1 << 3);
		tmp = tmp + sleepRate;
		ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_POWER_CTL, tmp);
		break;
	case 0:                // OFF
		tmp &= ~(1 << 2);
		ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_POWER_CTL, tmp);
		break;
	}
}

/* for ON_OFF param. take 1 to ON, 0 to OFF
 */
void ADXL345_Standby_SPI(int ON_OFF)
{
	uint8_t tmp = 0;
	tmp = ADXL345_ReadAddress8_SPI(ADXL345_REGISTER_POWER_CTL);
	switch(ON_OFF)
	{
	case 1:                // ON
		tmp &= ~(1 << 2);
		tmp &= ~(1 << 3);
		ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_POWER_CTL, tmp);
		break;
	case 0:                // OFF
		tmp |= (1 << 2);
		ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_POWER_CTL, tmp);
		break;
	}
}

void ADXL345_Measure_SPI(int ON_OFF)
{
	uint8_t tmp = 0;
	tmp = ADXL345_ReadAddress8_SPI(ADXL345_REGISTER_POWER_CTL);
	switch(ON_OFF)
	{
	case 1:           // Measurement ON
		tmp &= ~(1 << 2);
		tmp |= (1 << 3);
		break;
	case 0:           // Measurement OFF
		tmp &= ~(1 << 3);
		break;
	}
}

void ADXL345_TapFunc_SPI()
{
	uint8_t tmp, singl, doub;
	tmp = ADXL345_ReadAddress8_SPI(ADXL345_REGISTER_INT_SOURCE);
	singl = 0;
	doub = 0;
	singl = (tmp >> 5);
	doub = (tmp >> 6);
	if(singl == 1)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_14, GPIO_PIN_SET);
	}
	else if(doub == 1)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);
	}
}

void ADXL345_OffSetXYZ_SPI(int8_t OffsetX, int8_t OffsetY, int8_t OffsetZ)
{
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_OFSX, OffsetX);
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_OFSY, OffsetY);
	ADXL345_WriteAddress8_SPI(ADXL345_REGISTER_OFSZ, OffsetZ);
}

/* I2C Functions */

uint8_t ReadAddress8_I2C(uint8_t rRegister){
	uint8_t tmp;
	HAL_I2C_Mem_Read(&hi2c1, ADXL345_READ_ADDRESS, rRegister, 1, &tmp, 1, 100);
	return tmp;
}

void ADXL345_WriteAddress8_I2C(uint8_t wRegister, uint8_t value)
{
	uint8_t data[2];
	data[0] = wRegister;
	data[1] = value;
	HAL_I2C_Master_Transmit(&hi2c1, ADXL345_WRITE_ADDRESS, data, 2, 100);
}

void ADXL345_ReadAddress16_I2C(uint8_t addr, int bytes)
{
	uint8_t tmp8[6] = {};
	int i = 0;
	HAL_I2C_Mem_Read(&hi2c1, ADXL345_READ_ADDRESS, addr, 1, &tmp8[0], bytes, 100);
	for(i=0;i<=5;i++)
	{
		Coordinates.xyz[i] = tmp8[i];
	}
}

void ADXL345_RegConfig_I2C()
{
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_POWER_CTL, 0);
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_POWER_CTL, ((ADXL345_LINK << 5) | (ADXL345_AUTO_SLEEP << 4) | (1 << 3) | (1 << 2) | 1));   //POWER_CTR
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_DATA_FORMAT, ((0 << 7) | (ADXL345_SPI << 6) | (ADXL345_INT_INVERT << 5) | (0 << 4) | (ADXL345_FULL_RES << 3) | (1 << 2) | ADXL345_RANGE));	 //DATA_FORMAT
}

void ADXL345_Init_I2C()
{
	if(ADXL345_ReadAddress8_I2C(ADXL345_REGISTER_DEVID) == 0xE5)
	{
		ADXL345_RegConfig_I2C();
		ADXL345_RegTapConfig_I2C();
	}
}

void ADXL345_ReadAccValues_I2C()
{
	ADXL345_ReadAddress16_I2C(ADXL345_REGISTER_DATAXYZ, 6);
	Coordinates.x = (( Coordinates.xyz[1] << 8) | Coordinates.xyz[0]);    // X-axis
	Coordinates.y = (( Coordinates.xyz[3]<< 8)	 | Coordinates.xyz[2]);   // Y-axis
	Coordinates.z = (( Coordinates.xyz[5] << 8) | Coordinates.xyz[4]);    // Z-axis
	CoordiantesCalib.x = Coordinates.x * pow(10,-3) * 8.7;
	CoordiantesCalib.y = Coordinates.y * pow(10,-3) * 8.7;
	CoordiantesCalib.z = Coordinates.z * pow(10,-3) * 8.7;
}

void ADXL345_TapFunc_I2C()
{
	uint8_t tmp, singl, doub;
	tmp = ADXL345_ReadAddress8_I2C(ADXL345_REGISTER_INT_SOURCE);
	singl = 0;
	doub = 0;
	singl = (tmp >> 5);
	doub = (tmp >> 6);
	if(singl == 1)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_14, GPIO_PIN_SET);
	}
	else if(doub == 1)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);
	}

}

void ADXL345_OffSetXYZ_I2C(int8_t OffsetX, int8_t OffsetY, int8_t OffsetZ)
{
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_OFSX, OffsetX);
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_OFSY, OffsetY);
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_OFSZ, OffsetZ);
}

void ADXL345_RegTapConfig_I2C()
{
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_TAP_AXES, ((0 << 3) | (ADXL345_TAP_X << 2) | (ADXL345_TAP_Y << 1) | (ADXL345_TAP_Z)));
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_THRESH_TAP, (uint8_t)ADXL345_THRESH_VAL);
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_DUR, (uint8_t)ADXL345_DUR_VAL);
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_LATENT, (uint8_t)ADXL345_LATENT_VAL);
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_INT_MAP, ADXL345_INT1);
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_INT_ENABLE, 0x00);
	ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_INT_ENABLE, ((ADXL345_SINGLE_TAP << 6) | (ADXL345_DOUBLE_TAP << 5)));
}

void ADXL345_Standby_I2C(int ON_OFF)
{
	uint8_t tmp = 0;
	tmp = ADXL345_ReadAddress8_I2C(ADXL345_REGISTER_POWER_CTL);
	switch(ON_OFF)
	{
	case 1:                // ON
		tmp &= ~(1 << 2);
		tmp &= ~(1 << 3);
		ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_POWER_CTL, tmp);
		break;
	case 0:                // OFF
		tmp |= (1 << 2);
		ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_POWER_CTL, tmp);
		break;
	}
}

void ADXL345_Sleep_I2C(int ON_OFF, uint8_t sleepRate)
{
	uint8_t tmp = 0;
	tmp = ADXL345_ReadAddress8_I2C(ADXL345_REGISTER_POWER_CTL);
	switch(ON_OFF)
	{
	case 1:                // ON
		tmp = (tmp | (1 << 2));
		tmp &= ~(1 << 3);
		tmp = tmp + sleepRate;
		ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_POWER_CTL, tmp);
		break;
	case 0:                // OFF
		tmp &= ~(1 << 2);
		ADXL345_WriteAddress8_I2C(ADXL345_REGISTER_POWER_CTL, tmp);
		break;
	}
}

void ADXL345_Measure_SPI(int ON_OFF)
{
	uint8_t tmp = 0;
	tmp = ADXL345_ReadAddress8_I2C(ADXL345_REGISTER_POWER_CTL);
	switch(ON_OFF)
	{
	case 1:           // Measurement ON
		tmp &= ~(1 << 2);
		tmp |= (1 << 3);
		break;
	case 0:           // Measurement OFF
		tmp &= ~(1 << 3);
		break;
	}
}




