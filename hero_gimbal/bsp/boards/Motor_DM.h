#ifndef _MOTOR_DM_H
#define _MOTOR_DM_H


#include "main.h"

#define Motar_mode 0	//设置模式为何种模式，为0为IMT模式，为1为位置速度模式，为2为速度模式

#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -30			//速度最小值
#define V_MAX 30			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -10			//转矩最大值
#define T_MAX 10			//转矩最小值


uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
extern void Motor_enable(void);
extern void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);
extern void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel);
extern void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);

#endif
