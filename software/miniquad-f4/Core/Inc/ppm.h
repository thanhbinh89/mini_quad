/*
 * ppm.h
 *
 *  Created on: Apr 25, 2020
 *      Author: thanhbinh89
 */

#ifndef __PPM_H__
#define __PPM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#define PPM_MAX_CHANNEL_STATIC 8

typedef struct {
	TIM_HandleTypeDef *tim;
	volatile uint32_t timChannel;

	volatile uint32_t ic2LastValue;
	volatile uint32_t channel;
	volatile uint32_t values[PPM_MAX_CHANNEL_STATIC];
	volatile uint32_t maxChannel;
} ppm_t;

void PPM_Init(ppm_t *ppm, uint32_t maxChannel);
void PPM_InstallDriver(ppm_t *ppm, TIM_HandleTypeDef *tim, uint32_t timChannel);
void PPM_Start(ppm_t *ppm);
uint16_t PPM_GetValue(ppm_t *ppm, uint32_t channel);

void PPM_InterruptCallback(ppm_t *ppm);

#ifdef __cplusplus
}
#endif

#endif /* __PPM_H__ */
