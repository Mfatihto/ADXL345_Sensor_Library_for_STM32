/*
 * ADXL345.h
 *
 *  Created on: Mar 31, 2022
 *      Author: MFatih
 *
 *      For some of the register configurations, you will have to use functions for it. (For instance Standby and sleep mode).
 *
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Change stm library and I2C/SPI According to your Setup */
#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

#define ADXL345_WRITE_ADDRESS 0xA6
#define ADXL345_READ_ADDRESS  0xA7

#define ADXL345_CSPIN GPIO_PIN_10

/* Register Configurations */

/* SPI Wire Mode */
#define ADXL345_SPI 0    //4 wire SPI Mode
//#define ADXL345_SPI 1    //3 wire SPI Mode

/* Interrupt Invert */
#define ADXL345_INT_INVERT 0
//#define ADXL345_INT_INVERT 1

/* Link Bit */
#define ADXL345_LINK 0    //
//#define ADXL345_LINK 1    //

/* Range Setting */
//#define ADXL345_RANGE 0      //+-2g
#define ADXL345_RANGE 1		//+-4g
//#define ADXL345_RANGE 2		//+-8g
//#define ADXL345_RANGE 3		//+-16g

/* Full Resolution Bit */
#define ADXL345_FULL_RES 0    //10-bit mode, the range bits determine the maximum g range and scale factor.
//#define ADXL345_FULL_RES 1    //Full res mode, resolution increases with the g range set by the range bits to maintain a 4 mg/LSB scale factor.

/* Justify bit */
#define ADXL345_JUSTFIY 0     //Left-justified (MSB) mode.
//#define ADXL345_JUSTFIY 1     //Right-justified (LSB) mode.

/* Auto-Sleep Bit */
#define ADXL345_AUTO_SLEEP 0    //
//#define ADXL345_AUTO_SLEEP 1    //

/* Rate bit (at BW_RATE register)
*
*	NORMAL MODE:
*
*				BW value             Output Data Rate (Hz)
*					6 		  				6.25     // Default
*					7 		  				12.5
*					8 		  				25
*					9 		  				50
*					10 		  				100
*					11 		 				200
*					12 	      				400
*					13 		  				800
*					14 		  				1600
*					15 		  				3200
*
*	LOWPOWER MODE:
*
*				BW value 	        Output Data Rate (Hz)
*					7 		  				12.5	  // Default
*					8 		  				25
*					9 		  				50
*					10 		  				100
*					11 		  				200
*					12 		  				400
 */
#define ADXL345_BW_RATE 100    //4 bit register value.

/* Low Power mode bit */
#define ADXL345_LOW_POWER 0    //Normal operation.
//#define ADXL345_LOW_POWER 1    //Reduced power operation, somewhat higher noise.

/* Self Test */
#define ADXL345_SELF_TEST 0
//#define ADXL345_SELF_TEST 1

/* Defining the TAP mode */
//#define ADXL345_TAP_MODE 0     // Disabling both double and single tap.
//#define ADXL345_TAP_MODE 1     // Enabling single tap and disabling double tap.
//#define ADXL345_TAP_MODE 2     // Enabling double tap and disabling single tap.
#define ADXL345_TAP_MODE 3     // Enabling both double and single tap.

/* TAP_Z bit */
//#define ADXL345_REGISTER_TAP_Z 0
#define ADXL345_TAP_Z 1

/* TAP_Y bit */
#define ADXL345_TAP_Y 0
//#define ADXL345_REGISTER_TAP_Y 1

/* TAP_X bit */
#define ADXL345_TAP_X 0
//#define ADXL345_REGISTER_TAP_X 1

/* Enabling or Disabling or both Single Tap and Double Tap */
//#define ADXL345_REGISTER_SINGLE_TAP 0
#define ADXL345_SINGLE_TAP 1
//#define ADXL345_REGISTER_DOUBLE_TAP 0
#define ADXL345_DOUBLE_TAP 1

/* Defining Thresh Tap Value (data is in 8 bits unsigned format, in dec. (0-255)) */
#define ADXL345_THRESH_VAL 40    // Scale factor is 62.5 mg/LSB, meaning that taking 40 results --> 40 * 62.5 * pow(10, -6) = 2.5 g

/* Defining Duration Value (data is in 8 bits unsigned time value, in dec. (0-255)) */
#define ADXL345_DUR_VAL 100      // Scale factor is 625 us/LSB, meaning that taking 32 results --> 32 * 625 * pow(10, -6) = 0.02 s

/* Defining Latent Value (data is in 8 bits unsigned time value, in dec. (0-255)) */
#define ADXL345_LATENT_VAL 80    // Scale factor is 1.25 ms/LSB, meaning that taking 80 results --> 80 * 1.25 * pow(10, -3) = 0.01 s

/* Sleep Rate */                 // The frequency of readings in sleep mode
//#define ADXL345_SLEEP_RATE 0   // 8 Hz
#define ADXL345_SLEEP_RATE 1     // 4 Hz
//#define ADXL345_SLEEP_RATE 2   // 2 Hz
//#define ADXL345_SLEEP_RATE 3   // 1 Hz

/*
 *
 *
 *  Do register configurations till here. Leave the rest of the register bits as default.
 *
 *
 */

/* Measure Bit */
#define ADXL345_MEASURE 1

/* Sleep Bit */
#define ADXL345_SLEEP 0

/* Wake Up */            //control the frequency of readings
#define ADXL345_WAKE_UP	0

/* INT1 or INT2 */
#define ADXL345_INT1 0x00
#define ADXL345_INT2 0x01

/*            --- Register definitions ---            */
#define ADXL345_REGISTER_DEVID          0x00
#define ADXL345_REGISTER_THRESH_TAP     0x1D
#define ADXL345_REGISTER_OFSX           0x1E
#define ADXL345_REGISTER_OFSY           0x1F
#define ADXL345_REGISTER_OFSZ           0x20
#define ADXL345_REGISTER_DUR            0x21
#define ADXL345_REGISTER_LATENT         0x22
#define ADXL345_REGISTER_WINDOW         0x23
#define ADXL345_REGISTER_THRESH_ACT     0x24
#define ADXL345_REGISTER_THRESH_INACT   0x25
#define ADXL345_REGISTER_TIME_INACT     0x26
#define ADXL345_REGISTER_ACT_INACT_CTL  0x27
#define ADXL345_REGISTER_THRESH_FF      0x28
#define ADXL345_REGISTER_TIME_FF        0x29
#define ADXL345_REGISTER_TAP_AXES       0x2A
#define ADXL345_REGISTER_ACT_TAP_STATUS 0x2B
#define ADXL345_REGISTER_BW_RATE        0x2C
#define ADXL345_REGISTER_POWER_CTL      0x2D
#define ADXL345_REGISTER_INT_ENABLE     0x2E
#define ADXL345_REGISTER_INT_MAP        0x2F
#define ADXL345_REGISTER_INT_SOURCE     0x30
#define ADXL345_REGISTER_DATA_FORMAT    0x31
#define ADXL345_REGISTER_DATAXYZ        0x32
#define ADXL345_REGISTER_FIFO_CTL       0x38
#define ADXL345_REGISTER_FIFO_STATUS    0x39

/* Functions for I2C */
void ADXL345_Init_I2C(void);
void ADXL345_ReadAddress16_I2C(uint8_t addr,int bytes);
uint8_t ADXL345_ReadAddress8_I2C(uint8_t addr);
void ADXL345_WriteAddress8_I2C(uint8_t wRegister, uint8_t value);
void ADXL345_RegConfig_I2C(void);
void ADXL345_ReadAccValues_I2C(void);
void ADXL345_TapFunc_I2C(void);
void ADXL345_Sleep_I2C(int ON_OFF, uint8_t sleepRate);
void ADXL345_Standby_I2C(int ON_OFF);
void ADXL345_RegTapConfig_I2C(void);
void ADXL345_OffSetXYZ_I2C(int8_t OffsetX, int8_t OffsetY, int8_t OffsetZ);
void ADXL345_Measure_I2C(int ON_OFF);

/* Functions for SPI */
void ADXL345_WriteAddress8_SPI(uint8_t wRegister, uint8_t value);
void ADXL345_ReadAddress16_SPI(uint8_t addr, int bytes);
void ADXL345_RegConfig_SPI(void);
void ADXL345_Init_SPI(void);
void ADXL345_ReadAccValues_SPI(void);
uint8_t ADXL345_ReadAddress8_SPI(uint8_t addr);
void ADXL345_TapFunc_SPI(void);
void ADXL345_RegTapConfig_SPI(void);
void ADXL345_Sleep_SPI(int ON_OFF, uint8_t sleepRate);
void ADXL345_Standby_SPI(int ON_OFF);
void ADXL345_OffSetXYZ_SPI(int8_t OffsetX, int8_t OffsetY, int8_t OffsetZ);
void ADXL345_Measure_SPI(int ON_OFF);

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	int8_t xyz[6];
}ADXL345_CorStruc;

#ifdef __cplusplus
}
#endif

#endif /* INC_ADXL345_H_ */
