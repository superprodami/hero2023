#ifndef __MYDEF
#define __MYDEF

#include "mydef.h"

int16_t see,targetp,ml,mr,targetm,out,outmax;

uint8_t state_judge = 0;
volatile SYSTEMVALUE SystemValue = Starting;	////程序状态
volatile EnvironmentModeType EnvironmentMode = NOMAL;	//所处环境
volatile ControlModeType ControlMode = KEYBOARD; //REMOTE;//KEYBOARD;			//控制方式
uint32_t control_judge_flag = 0;
//
eShootState ShootState = UNSTART;
Mocalun_Status mocalun_status;
heat_measure_t heat_judge = {0};
shoot_measure_t shoot_judge = {0};
//
volatile eGimbalAction actGimbal = GIMBAL_NORMAL;       ////云台操作模式///
Gimbal_Hanging_Status gimbal_hanging;
float Cloud_Angle_Target[2][2];
//
volatile bool imu_init_finish_flag = 0;
//
volatile eChassisAction actChassis = CHASSIS_NORMAL; //默认底盘不跟随云台行走
eChassisAction actChassis_last = CHASSIS_NORMAL;
//
flag_t Flag_status;//各种flag 状态位

#endif
