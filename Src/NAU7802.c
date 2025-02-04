/*
 * NAU7802.c
 *
 *  Created on: Jan 23, 2025
 *      Author: Ziya
 */

#include "NAU7802.h"

// WEIGHT = (RAW - TARE) / CALIBRATION FACTORY
uint32_t tare_value = 642;
float calibration_factory = 22.6;

uint8_t I2C_ERROR = 0;
uint8_t tempReg;

/*
 * @brief write register NAU7802 using I2C
 * @param uint8_t reg, write register adress
 * @param uint8_t value, write value
 * @retval void
 * @author ziya
 */
void 	writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t data[2] = {reg, value};
	data[0] = reg;
	data[1] = value;
	if (HAL_I2C_Master_Transmit_IT(&hi2c1, NAU7802_WRITE_ADRESS, data, 2) != HAL_OK)
    {
    	I2C_ERROR = 1;
        Error_Handler();
    }
    HAL_Delay(10);

}

/*
 * @brief read register NAU7802 using I2C
 * @param uint8_t reg, read register
 * @param uint8_t value, read value
 * @retval void
 * @author ziya
 */
void		readRegister(uint8_t reg, uint8_t *value)
{
    if (HAL_I2C_Master_Transmit_IT(&hi2c1, NAU7802_READ_ADRESS, &reg, 1) != HAL_OK)
    {
    	I2C_ERROR = 2;
        Error_Handler();
    }
    HAL_Delay(10);


    if (HAL_I2C_Master_Receive_IT(&hi2c1, NAU7802_READ_ADRESS, value, 1) != HAL_OK)
    {
    	I2C_ERROR = 4;
    	//Error_Handler();
    }
    HAL_Delay(10);
}

/*
 * @brief Initilalizing NAU7802 gain set = 128, sampling rate set = 320Hz
 * @param none
 * @return void
 * @author ziya
 */
void 	NAU7802_Init(void)
{

	readRegister(PU_CTRL_REG, &tempReg);
	tempReg |= PU_CTRL_RR_MASK;
	writeRegister(PU_CTRL_REG, tempReg);
	writeRegister(PU_CTRL_REG, 0x00000000);

	HAL_Delay(1);


	readRegister(PU_CTRL_REG, &tempReg);
	tempReg |= PU_CTRL_PUD_MASK;
	writeRegister(PU_CTRL_REG, tempReg);


	HAL_Delay(1);

	readRegister(PU_CTRL_REG, &tempReg);
	tempReg |= PU_CTRL_PUA_MASK;
	writeRegister(PU_CTRL_REG, tempReg);


	HAL_Delay(1);

	readRegister(PU_CTRL_REG, &tempReg);
	tempReg |= PU_CTRL_AVDDS_MASK;
	writeRegister(PU_CTRL_REG, tempReg);

	readRegister(CTRL1_REG, &tempReg);
	tempReg &= ~ CTRL1_GAINS_MASK;
	tempReg |= (GAIN_SET_16 & CTRL1_GAINS_MASK);
	writeRegister(CTRL1_REG, tempReg);

	readRegister(CTRL2_REG, &tempReg);
	tempReg &= ~ CTRL2_CRS_MASK;
	tempReg |= (CRS_SET_320 & CTRL2_CRS_MASK);
	writeRegister(CTRL2_REG, tempReg);

	readRegister(PU_CTRL_REG, &tempReg);
	tempReg |= PU_CTRL_CS_MASK;
	writeRegister(PU_CTRL_REG, tempReg);

	setCalibration();

}

/*
 * @brief
 * @param void
 * @retval
 * @author ziya
 */
void setCalibration(void)
{
	readRegister(CTRL2_REG, &tempReg);
	tempReg |= CTRL2_CALS_MASK ;
	writeRegister(CTRL2_REG, tempReg);

	while(checkCALS());

	readRegister(CTRL2_REG, &tempReg);
	tempReg = tempReg | (CTRL2_CALS_MASK | CTRL2_CALMOD_OFFSET_MASK) ;
	writeRegister(CTRL2_REG, tempReg);

	while(checkCALS());

	//readRegister(CTRL2_REG, &tempReg);
	//tempReg = tempReg | (CTRL2_CALS_MASK | CTRL2_CALMOD_GAIN_MASK) ;
	//writeRegister(CTRL2_REG, tempReg);

	while(checkCALS());
}

/*
 * @brief Check the REG0X00 (PU_CTRL) Cycle Ready bit
 * @param  void
 * @retval uint8_t tempreg
 * @author ziya
 */
uint8_t checkCR(void)
{
	readRegister(PU_CTRL_REG, &tempReg);
	tempReg &= PU_CTRL_CR_MASK;
	tempReg = tempReg >> 5;
	return tempReg;
}

/*
 * @brief
 * @param
 * @retval
 * @author
 */
uint8_t checkCALS(void)
{
	readRegister(CTRL2_REG, &tempReg);
	tempReg = (tempReg & CTRL2_CALS_MASK) >> 2;
	while(tempReg)
	{
		readRegister(CTRL2_REG, &tempReg);
		tempReg = (tempReg & CTRL2_CALS_MASK) >> 2;	}
	//if(tempReg)
	//{
	//	I2C_ERROR = 255;
	//	Error_Handler();
	//}

	return tempReg;
}

/*
 * @brief Read adc out register and weight = (rawValue - tare_value) / calibration_factory
 * @param void
 * @retval uint32_t weight
 * @author ziya
 */
uint32_t getRaw(void)
{
	uint32_t rawValue;
	//uint32_t weight;

	uint8_t b2;
	uint8_t b1;
	uint8_t b0;

    readRegister(ADCO_B0_REG, &b0);
    readRegister(ADCO_B1_REG, &b1);
    readRegister(ADCO_B2_REG, &b2);

    rawValue = (b2 << 16) | (b1 << 8) | b0;
    //weight = (rawValue - tare_value) / calibration_factory ;

    if (rawValue > 8388608) {
      rawValue &= ~ 0xFF80000;
    }
    rawValue = rawValue + 5000;

    return rawValue;
}
