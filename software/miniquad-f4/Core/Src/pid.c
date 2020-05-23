/*
 * pid.c
 *
 *  Created on: Apr 24, 2020
 *      Author: thanhbinh89
 */
#include <stdio.h>
#include <stdint.h>
#include "pid.h"

void PID_Setup(pidHelper_t *pid, float p, float i, float d, float max) {
	pid->pGain = p;
	pid->iGain = i;
	pid->dGain = d;
	pid->maxOutput = max;
	pid->pMem = 0;
	pid->iMem = 0;
	pid->dMem = 0;
	pid->preError = 0;
	pid->prePreError = 0;
	pid->preOutput = 0;
}

float PID_Calculate(pidHelper_t *pid, float input, float setpoint, float dT) {
	float error = input - setpoint;
	float out;
#if 1
	//P
	pid->pMem = pid->pGain * error;
	//I
	pid->iMem = pid->iMem + pid->iGain * error * dT;
	//D
	pid->dMem = pid->dGain * (error - pid->preError) / dT;

	out = pid->preOutput + pid->pMem + pid->iMem + pid->dMem;
	pid->preError = error;
//	pid->preOutput = out;
#else
	//P
	pid->pMem = pid->pGain * (error - pid->preError);
	//I
	pid->iMem = 0.5 * pid->iGain * (error + pid->preError) * dT;
//	if (pid->iMem > 100) pid->iMem = 100;
//	else if (pid->iMem < (-100)) pid->iMem = (-100);
	//D
	pid->dMem = pid->dGain * (error - 2.0 * pid->preError + pid->prePreError) / dT;
//	if (pid->dMem > 100) pid->dMem = 100;
//	else if (pid->dMem < (-100)) pid->dMem = (-100);

	out = pid->preOutput + pid->pMem + pid->iMem + pid->dMem;
	pid->prePreError = pid->preError;
	pid->preError = error;
	pid->preOutput = out;
#endif
	return out;

}

