#ifndef __bsp_pid
#define __bsp_pid
//#include "type.h"
#include "stm32f4xx_hal.h"
#include "main.h"



void pid_init(PidTypeDef *pid);
float pid_caculate(PidTypeDef *pid, const float ref, const float set);
void PID_clear(PidTypeDef *pid);
void pid_reset(PidTypeDef	*pid, float PID[3]);
#endif
