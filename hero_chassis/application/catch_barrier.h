//#ifndef __CATCH_BARRIER_H
//#define __CATCH_BARRIER_H

//#include "main.h"
//#include "mysystem.h"

//extern float servo_v[2]; //���ÿ���ж���ת�Ƕ� 
//extern uint16_t servo_count[2];  //�����תʱ���жϷ�--10msΪһ����λ ��תĿ��ʱ��Ϊdt*n
//extern float servoDeg[2];  //ǰһ���Ƕ� ��ʼ��Ϊ��ʼ�̶��Ƕ� �Ƕȷ�Χ��500-2500 *100  ��Ӧ0-180��

//typedef struct{
//enum
//{
//	PREPARE,
//	WORK_UP,
//	WORK_DOWN,
//}servo_status;	

//}ServoState;

//extern ServoState servo_state;

//extern void setAngle(uint8_t channel,uint16_t val,uint16_t t);
//extern void servo_control(void);
//extern void servo_mode_choose(void);
////typedef struct{
////enum
////{
////	PREPARE,
////	WORK,
////}servo_status;	

////float servo_pre_left;
////float servo_pre_right;
////float servo_set_left;
////float servo_set_right;

////}ServoState;
////extern ServoState servo_state;

////typedef struct{
////union
////	{
////		uint8_t buff[4];
////		float value;
////	}servo_angle;
////	union																																																					\
////    {
////        uint8_t buff[4];
////		float value;
////    }servo_speed;
////uint8_t servo_id;

////}ServoType;

////extern ServoType ServoValue_L;
////extern ServoType ServoValue_R;

////extern void set_servo_angle(CAN_HandleTypeDef* hcan, ServoType * servo_value);
////extern void set_servo_angle1(CAN_HandleTypeDef* hcan, ServoType * servo_value);
////extern void set_servo_speed(CAN_HandleTypeDef* hcan, ServoType * servo_value);
////extern void servo_send(void);
////extern void servo_init(void);
//#endif

