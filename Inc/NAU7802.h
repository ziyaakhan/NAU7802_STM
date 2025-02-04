/*
 * NAU7802.h
 *
 *  Created on: Jan 23, 2025
 *      Author: Ziya
 */

#ifndef INC_NAU7802_H_
#define INC_NAU7802_H_

#include "main.h"

#define NAU7802_WRITE_ADRESS 0x54
#define NAU7802_READ_ADRESS  0x55

#define CALIBRATION_FACTORY 22.6F
#define TARE_VALUE 52300

// Datasheet page 28
#define PU_CTRL_REG          0x00
#define CTRL1_REG            0x01
#define CTRL2_REG            0x02
#define OCAL1_B2_REG         0x03
#define OCAL1_B1_REG         0x04
#define OCAL1_B0_REG         0x05
#define GCAL1_B3_REG         0x06
#define GCAL1_B2_REG         0x07
#define GCAL1_B1_REG         0x08
#define GCAL1_B0_REG         0x09
#define OCAL2_B2_REG         0x0A
#define OCAL2_B1_REG         0x0B
#define OCAL2_B0_REG         0x0C
#define GCAL2_B3_REG         0x0D
#define GCAL2_B2_REG         0x0E
#define GCAL2_B1_REG         0x0F
#define GCAL2_B0_REG         0x10
#define I2C_CTRL_REG         0x11
#define ADCO_B2_REG          0x12
#define ADCO_B1_REG          0x13
#define ADCO_B0_REG          0x14
#define OTP_B1_REG           0x15
#define OTP_B0_REG           0x16
#define DEVICE_REV_REG       0x1F

// PU_CTRL Register Bits
#define PU_CTRL_AVDDS_MASK   0x80
#define PU_CTRL_OSCS_MASK    0x40
#define PU_CTRL_CR_MASK      (1 << 5) // 0x20
#define PU_CTRL_CS_MASK      (1 << 4) // 0x10
#define PU_CTRL_PUR_MASK     (1 << 3) // 0x8
#define PU_CTRL_PUA_MASK     (1 << 2) // 0x4
#define PU_CTRL_PUD_MASK     0x02
#define PU_CTRL_RR_MASK      0x01

// CTRL1 Register Bits
#define CTRL1_CRP_MASK       0x80
#define CTRL1_VLDO_MASK      0x70
#define CTRL1_GAINS_MASK     0x07

#define GAIN_SET_128 0b00000111
#define GAIN_SET_64  0b110
#define GAIN_SET_32  0b101
#define GAIN_SET_16  0b011
#define GAIN_SET_8	 0b011
#define GAIN_SET_4   0b010
#define GAIN_SET_2   0b001
#define GAIN_SET_1   0b000

// CTRL2 Register Bits
#define CTRL2_CHS_MASK      0x80
#define CTRL2_CRS_MASK      0x60
#define CTRL2_CAL_ERR_MASK  (1 << 3)
#define CTRL2_CALS_MASK     (1 << 2)


#define CTRL2_CALMOD_GAIN_MASK    			0x03
#define CTRL2_CALMOD_OFFSET_MASK    		0x02
#define CTRL2_CALMOD_RESERVED				0x01
#define CTRL2_CALMOD_INTERNAL_OFFSET_MASK   0x00

#define CRS_SET_320	0b111
#define CRS_SET_80	0b011
#define CRS_SET_40	0b010
#define CRS_SET_20	0b001
#define CRS_SET_10	0b000

// I2C Control Register Bits
#define I2C_CTRL_CRSD_MASK   0x80
#define I2C_CTRL_FDR_MASK    0x40
#define I2C_CTRL_SPE_MASK    0x20
#define I2C_CTRL_SI_MASK     0x10
#define I2C_CTRL_BOPGA_MASK  0x08
#define I2C_CTRL_TS_MASK     0x04
#define I2C_CTRL_BGPCP_MASK  0x02

// ADC Output Registers (Read-Only)
#define ADCO_B2_MASK         0xFF
#define ADCO_B1_MASK         0xFF
#define ADCO_B0_MASK         0xFF

// OTP Registers (Read-Only)
#define OTP_B1_MASK          0xFF
#define OTP_B0_MASK          0xFF

// Device Revision Code (Read-Only)
#define DEVICE_REV_MASK      0xFF

extern uint32_t getRaw();
extern void 	NAU7802_Init();
extern void		readRegister(uint8_t reg, uint8_t *value);
extern void 	writeRegister(uint8_t reg, uint8_t value);
extern uint8_t  checkCR(void);
extern uint8_t checkCALS(void);
extern void setCalibration(void);
extern void readAllReg(void);

extern uint32_t tare_value;
extern float calibration_factory;
extern uint8_t I2C_ERROR;


extern I2C_HandleTypeDef hi2c1;

#endif /* INC_NAU7802_H_ */
