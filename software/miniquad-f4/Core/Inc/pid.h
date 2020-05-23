#ifndef __PID_HELPER_H__
#define __PID_HELPER_H__

typedef struct {
	float pGain;
	float iGain;
	float dGain;
	int maxOutput;

	float pMem;
	float iMem;
	float dMem;
	float preError;
	float prePreError;
	float preOutput;
} pidHelper_t;

extern void PID_Setup(pidHelper_t *pid, float p, float i, float d, float max);
extern float PID_Calculate(pidHelper_t *pid, float input, float setpoint,
		float dT);

#endif // __PID_H__
