/*
 * ppm.c
 *
 *  Created on: Apr 25, 2020
 *      Author: thanhbinh89
 */
#include <string.h>
#include "ppm.h"

#define PPM_BLANK_TIME 2100

void PPM_Init(ppm_t *ppm, uint32_t maxChannel) {
	memset(ppm, 0, sizeof(*ppm));
	ppm->maxChannel = maxChannel;
	ppm->ic2LastValue = 0;
	ppm->channel = 0;
}

void PPM_InstallDriver(ppm_t *ppm, TIM_HandleTypeDef *tim, uint32_t timChannel) {
	ppm->tim = tim;
	ppm->timChannel = timChannel;
}

void PPM_Start(ppm_t *ppm) {
	HAL_TIM_IC_Start_IT(ppm->tim, ppm->timChannel);
}

uint16_t PPM_GetValue(ppm_t *ppm, uint32_t channel) {
	return (uint16_t) ppm->values[channel];
}

void PPM_InterruptCallback(ppm_t *ppm) {
	uint32_t ic2Value;
	uint32_t ic2DiffValue;
	ic2Value = HAL_TIM_ReadCapturedValue(ppm->tim, ppm->timChannel);
	if (ic2Value > ppm->ic2LastValue) {
		ic2DiffValue = ic2Value - ppm->ic2LastValue;
	} else if (ic2Value < ppm->ic2LastValue) {
		ic2DiffValue = 0xFFFF - ppm->ic2LastValue + ic2Value + 1;
	}
	ppm->ic2LastValue = ic2Value;

	if (ic2DiffValue > PPM_BLANK_TIME) {
		ppm->channel = 0;
	} else {
		if (ppm->channel < ppm->maxChannel) {
			ppm->values[ppm->channel] = ic2DiffValue;
		}
		ppm->channel++;
	}

}
