//#ifndef __CATCH_BARRIER_H
//#define __CATCH_BARRIER_H

//#include "main.h"
//#include "mysystem.h"

//extern float servo_v[2]; //舵机每次中断旋转角度 
//extern uint16_t servo_count[2];  //舵机旋转时间判断符--10ms为一个单位 旋转目标时间为dt*n
//extern float servoDeg[2];  //前一个角度 初始化为初始固定角度 角度范围：500-2500 *100  对应0-180°

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

