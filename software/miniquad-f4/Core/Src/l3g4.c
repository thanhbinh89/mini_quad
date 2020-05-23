/*
 * l3g4200d.c
 *
 *  Created on: Apr 17, 2020
 *      Author: FPT-BinhIoT
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "l3g4.h"

#define FS_2000_SENSI 	0.07
#define FS_500_SENSI	0.0175
#define FS_250_SENSI	0.00875

static L3G4_Status_t readRegister8(L3G4_t *l3g4, uint8_t reg, uint8_t *result);
static L3G4_Status_t readRegister16(L3G4_t *l3g4, uint8_t reg, uint16_t *result);
static L3G4_Status_t writeRegister8(L3G4_t *l3g4, uint8_t reg, uint8_t data);
static L3G4_Status_t writeAndVerifyRegister8(L3G4_t *l3g4, uint8_t reg,
		uint8_t data);

void L3G4_Init(L3G4_t *l3g4) {
	memset(l3g4, 0, sizeof(*l3g4));
	l3g4->config.ODR = L3G4_ODR_400;
	l3g4->config.Bandwidth = L3G4_BW_0;
	l3g4->config.Mode = L3G4_NOR_SLE_MOD;
	l3g4->config.Axis_Enable = L3G4_XYZEN;
	l3g4->config.HPF_Enable = L3G4_HPF_ENA;
	l3g4->config.HPF_Mode = L3G4_HPM_NOR_RES;
	l3g4->config.HPCF = L3G4_HPCF_9;
	l3g4->config.INT1_Enable = L3G4_INT1_DIS;
	l3g4->config.DRRY_Enable = L3G4_DRDY_INT2_ENA;
	l3g4->config.BDU = L3G4_BDU_CONTINOUS;
	l3g4->config.FS = L3G4_FS_2000;
	l3g4->config.FIFO_Enable = L3G4_FIFO_DIS;
	l3g4->config.FIFO_Mode = L3G4_BYBASS_MOD;
	l3g4->config.INT1_SEL = L3G4_INT1_SEL_3;
	l3g4->config.OUT_SEL = L3G4_OUT_SEL_3;
	l3g4->spi = (void*) 0;
	l3g4->drdyPin = 0xFFFF;
	l3g4->outX = 0;
	l3g4->outY = 0;
	l3g4->outZ = 0;
	l3g4->calibX = 0;
	l3g4->calibY = 0;
	l3g4->calibZ = 0;
}

void L3G4_InstallDriver(L3G4_t *l3g4, SPI_HandleTypeDef *spi) {
	l3g4->spi = spi;
}

void L3G4_InstallGPIO(L3G4_t *l3g4, GPIO_TypeDef *csPort, uint16_t csPin,
		uint16_t drdyPin) {
	l3g4->csPort = csPort;
	l3g4->csPin = csPin;
	l3g4->drdyPin = drdyPin;
	HAL_GPIO_WritePin(l3g4->csPort, l3g4->csPin, GPIO_PIN_SET);
}

L3G4_Status_t L3G4_CheckConnect(L3G4_t *l3g4) {
	uint8_t result = 0;
	if (L3G4_OK != readRegister8(l3g4, L3G4_WHO_AM_I, &result)) {
		return L3G4_ERROR;
	}

	if (result != WHO_AM_I_VALID) {
		if (L3G4_OK != readRegister8(l3g4, L3G4_WHO_AM_I, &result)) {
			return L3G4_ERROR;
		}
		if (result != WHO_AM_I_VALID) {
			return L3G4_ERROR;
		}
		return L3G4_OK;
	}
	return L3G4_OK;
}

L3G4_Status_t L3G4_CheckDataAvailable(L3G4_t *l3g4) {
	uint8_t result = 0;
	if (L3G4_OK == readRegister8(l3g4, L3G4_STATUS_REG, &result)) {
		if (result & 0x08) {
			return L3G4_OK;
		}
	}
	return L3G4_BUSY;
}

L3G4_Status_t L3G4_Config(L3G4_t *l3g4) {
	uint8_t ctrl1 = 0;
	uint8_t ctrl2 = 0;
	uint8_t ctrl3 = 0;
	uint8_t ctrl4 = 0;
	uint8_t ctrl5 = 0;

	ctrl1 |= l3g4->config.ODR | l3g4->config.Bandwidth | l3g4->config.Mode
			| l3g4->config.Axis_Enable;
	ctrl2 |= l3g4->config.HPF_Mode | l3g4->config.HPCF;
	ctrl3 |= l3g4->config.INT1_Enable | L3G4_BOOT_INT1_DIS | L3G4_INT1_ACT_H
			| L3G4_INT1_PP | l3g4->config.DRRY_Enable | L3G4_WTM_INT2_DIS
			| L3G4_ORUN_INT2_DIS | L3G4_EMP_INT2_DIS;
	ctrl4 |= l3g4->config.BDU | L3G4_BLE_LSB | l3g4->config.FS | L3G4_ST_NOR
			| L3G4_SPI_4_WIRE;
	ctrl5 |= L3G4_BOOT_NOR | l3g4->config.FIFO_Enable | l3g4->config.HPF_Enable
			| l3g4->config.INT1_SEL | l3g4->config.OUT_SEL;

	if (L3G4_OK != writeAndVerifyRegister8(l3g4, L3G4_CTRL_REG1, ctrl1)) {
		return L3G4_ERROR;
	}
	if (L3G4_OK != writeAndVerifyRegister8(l3g4, L3G4_CTRL_REG2, ctrl2)) {
		return L3G4_ERROR;
	}
	if (L3G4_OK != writeAndVerifyRegister8(l3g4, L3G4_CTRL_REG3, ctrl3)) {
		return L3G4_ERROR;
	}
	if (L3G4_OK != writeAndVerifyRegister8(l3g4, L3G4_CTRL_REG4, ctrl4)) {
		return L3G4_ERROR;
	}
	if (L3G4_OK != writeAndVerifyRegister8(l3g4, L3G4_CTRL_REG5, ctrl5)) {
		return L3G4_ERROR;
	}

	return L3G4_OK;
}

void L3G4_Calib(L3G4_t *l3g4, int32_t sample) {
	uint16_t outX = 0, outY = 0, outZ = 0;
	int32_t calibX = 0, calibY = 0, calibZ = 0;
	int32_t counter = 0;
	while (counter < sample / 2) {
		if (L3G4_OK == L3G4_CheckDataAvailable(l3g4)) {
			++counter;
			readRegister16(l3g4, L3G4_OUT_X_L, &outX);
			readRegister16(l3g4, L3G4_OUT_Y_L, &outY);
			readRegister16(l3g4, L3G4_OUT_Z_L, &outZ);
		}
	}

	outX = 0, outY = 0, outZ = 0;
	calibX = 0, calibY = 0, calibZ = 0;
	counter = 0;
	while (counter < sample) {
		if (L3G4_OK == L3G4_CheckDataAvailable(l3g4)) {
			++counter;
			readRegister16(l3g4, L3G4_OUT_X_L, &outX);
			readRegister16(l3g4, L3G4_OUT_Y_L, &outY);
			readRegister16(l3g4, L3G4_OUT_Z_L, &outZ);
			calibX += ((int16_t) outX);
			calibY += ((int16_t) outY);
			calibZ += ((int16_t) outZ);
		}
	}
	l3g4->calibX = calibX / sample;
	l3g4->calibY = calibY / sample;
	l3g4->calibZ = calibZ / sample;
}

float L3G4_GetLastX(L3G4_t *l3g4) {
	float x;
	switch (l3g4->config.FS) {
	case L3G4_FS_2000:
		x = (float) l3g4->outX * FS_2000_SENSI;
		break;
	case L3G4_FS_500:
		x = (float) l3g4->outX * FS_500_SENSI;
		break;
	case L3G4_FS_250:
		x = (float) l3g4->outX * FS_250_SENSI;
		break;
	default:
		break;
	}
	return x;
}

float L3G4_GetLastY(L3G4_t *l3g4) {
	float y;
	switch (l3g4->config.FS) {
	case L3G4_FS_2000:
		y = (float) l3g4->outY * FS_2000_SENSI;
		break;
	case L3G4_FS_500:
		y = (float) l3g4->outY * FS_500_SENSI;
		break;
	case L3G4_FS_250:
		y = (float) l3g4->outY * FS_250_SENSI;
		break;
	default:
		break;
	}
	return y;
}

float L3G4_GetLastZ(L3G4_t *l3g4) {
	float z;
	switch (l3g4->config.FS) {
	case L3G4_FS_2000:
		z = (float) l3g4->outZ * FS_2000_SENSI;
		break;
	case L3G4_FS_500:
		z = (float) l3g4->outZ * FS_500_SENSI;
		break;
	case L3G4_FS_250:
		z = (float) l3g4->outZ * FS_250_SENSI;
		break;
	default:
		break;
	}
	return z;
}

float L3G4_GetCalibX(L3G4_t *l3g4) {
	float x;
	switch (l3g4->config.FS) {
	case L3G4_FS_2000:
		x = (float) l3g4->calibX * FS_2000_SENSI;
		break;
	case L3G4_FS_500:
		x = (float) l3g4->calibX * FS_500_SENSI;
		break;
	case L3G4_FS_250:
		x = (float) l3g4->calibX * FS_250_SENSI;
		break;
	default:
		break;
	}
	return x;
}

float L3G4_GetCalibY(L3G4_t *l3g4) {
	float y;
	switch (l3g4->config.FS) {
	case L3G4_FS_2000:
		y = (float) l3g4->calibY * FS_2000_SENSI;
		break;
	case L3G4_FS_500:
		y = (float) l3g4->calibY * FS_500_SENSI;
		break;
	case L3G4_FS_250:
		y = (float) l3g4->calibY * FS_250_SENSI;
		break;
	default:
		break;
	}
	return y;
}

float L3G4_GetCalibZ(L3G4_t *l3g4) {
	float z;
	switch (l3g4->config.FS) {
	case L3G4_FS_2000:
		z = (float) l3g4->calibZ * FS_2000_SENSI;
		break;
	case L3G4_FS_500:
		z = (float) l3g4->calibZ * FS_500_SENSI;
		break;
	case L3G4_FS_250:
		z = (float) l3g4->calibZ * FS_250_SENSI;
		break;
	default:
		break;
	}
	return z;
}

void L3G4_Loop(L3G4_t *l3g4) {
	if (L3G4_OK == L3G4_CheckDataAvailable(l3g4)) {
		L3G4_ReadAll(l3g4);
	} else {
		HAL_Delay(1);
	}
}

void L3G4_ReadAll(L3G4_t *l3g4) {
	uint16_t outX = 0, outY = 0, outZ = 0;

	readRegister16(l3g4, L3G4_OUT_X_L, &outX);
	readRegister16(l3g4, L3G4_OUT_Y_L, &outY);
	readRegister16(l3g4, L3G4_OUT_Z_L, &outZ);

	l3g4->outX = ((int16_t) outX) - l3g4->calibX;
	l3g4->outY = ((int16_t) outY) - l3g4->calibY;
	l3g4->outZ = ((int16_t) outZ) - l3g4->calibZ;
}

void L3G4_InterruptCallback(L3G4_t *l3g4, uint16_t pin) {
	if (pin == l3g4->drdyPin) {
		l3g4->dataAvai = 1U;
	}
}

L3G4_Status_t readRegister8(L3G4_t *l3g4, uint8_t reg, uint8_t *result) {
	uint8_t outByte[2] = { 0x80 | reg, 0 };
	uint8_t inByte[2] = { 0, 0 };
	HAL_StatusTypeDef status = HAL_ERROR;

	HAL_GPIO_WritePin(l3g4->csPort, l3g4->csPin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(l3g4->spi, outByte, inByte, 2, 1000);
	HAL_GPIO_WritePin(l3g4->csPort, l3g4->csPin, GPIO_PIN_SET);

	if (HAL_OK != status) {
		return L3G4_ERROR;
	}
	*result = inByte[1];
	return L3G4_OK;
}

L3G4_Status_t readRegister16(L3G4_t *l3g4, uint8_t reg, uint16_t *result) {
	uint8_t outByte[] = { 0xC0 | reg, 0, 0 };
	uint8_t inByte[] = { 0, 0, 0 };
	HAL_StatusTypeDef status = HAL_ERROR;

	HAL_GPIO_WritePin(l3g4->csPort, l3g4->csPin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(l3g4->spi, outByte, inByte, 3, 1000);
	HAL_GPIO_WritePin(l3g4->csPort, l3g4->csPin, GPIO_PIN_SET);

	if (HAL_OK != status) {
		return L3G4_ERROR;
	}
	*result = inByte[2] << 8 | inByte[1];
	return L3G4_OK;
}

L3G4_Status_t writeRegister8(L3G4_t *l3g4, uint8_t reg, uint8_t data) {
	uint8_t outByte[2] = { 0x7F & reg, data };
	uint8_t inByte[2] = { 0, 0 };
	HAL_StatusTypeDef status = HAL_ERROR;

	HAL_GPIO_WritePin(l3g4->csPort, l3g4->csPin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(l3g4->spi, outByte, inByte, 2, 1000);
	HAL_GPIO_WritePin(l3g4->csPort, l3g4->csPin, GPIO_PIN_SET);

	if (HAL_OK != status) {
		return L3G4_ERROR;
	}
	return L3G4_OK;
}

L3G4_Status_t writeAndVerifyRegister8(L3G4_t *l3g4, uint8_t reg, uint8_t data) {
	uint8_t result = 0;
	writeRegister8(l3g4, reg, data);
	readRegister8(l3g4, reg, &result);
	if (data != result) {
		return L3G4_ERROR;
	}
	return L3G4_OK;
}
