#ifndef _MOTOR_DM_H
#define _MOTOR_DM_H


#include "main.h"

#define Motar_mode 0	//����ģʽΪ����ģʽ��Ϊ0ΪIMTģʽ��Ϊ1Ϊλ���ٶ�ģʽ��Ϊ2Ϊ�ٶ�ģʽ

#define P_MIN -12.5		//λ����Сֵ
#define P_MAX 12.5		//λ�����ֵ
#define V_MIN -30			//�ٶ���Сֵ
#define V_MAX 30			//�ٶ����ֵ
#define KP_MIN 0.0		//Kp��Сֵ
#define KP_MAX 500.0	//Kp���ֵ
#define KD_MIN 0.0		//Kd��Сֵ
#define KD_MAX 5.0		//Kd���ֵ
#define T_MIN -10			//ת�����ֵ
#define T_MAX 10			//ת����Сֵ


uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
extern void Motor_enable(void);
extern void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);
extern void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel);
extern void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);

#endif
