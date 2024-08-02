/**
  ******************************************************************************
  * @file    shoot_task.c
  * @brief   机器人的射击task，包括遥控器和键盘两种控制模式，以及热量限制、防卡弹等
  *
  ******************************************************************************
  * @attention
  *
  * 2021.4 新写的防卡弹函数效果比较明显，不过个人认为仍然是权宜之计
  * 上届留下来的热量限制函数也还可以用。具体限制方式为卡弹或即将超热量时进入relax模式，拨弹盘放松
  * 同时跳过模式选择，防止放松被打断
  *
  *
  ******************************************************************************
  */
#include "shoot_task.h"
//#include "param.h"
//#include "type.h"
#include "judge.h"
//#include "main.h"
#include "bspcan.h"
eShootAction actShoot = SHOOT_NORMAL;
eShootAction actShoot_last = SHOOT_NORMAL;
//eShootState ShootState = UNSTART;
//Mocalun_Status mocalun_status;

fp32 Ammunition_Motor_Position_pid[3] = {6, 0, 10};
//fp32 Ammunition_Motor_Speed_pid_pos[3] = {20, 0.5, 0};
fp32 Ammunition_Motor_Speed_pid[3] = {10, 1, 0};// {15, 1, 0}
const static fp32 Mocalun_l_speed[3] = {8,0.1,0.2};//{8, 0.4, 0};
const static fp32 Mocalun_r_speed[3] = {8,0.1,0.2};//{8, 0.4, 0};
const static fp32 nothing[3] = {0, 0, 0};

int32_t main_angle=0;
void ShootFun(void const *argument)
{
    portTickType currentTime;

    REVOLVER_InitArgument();
  
    while (1)
    {
        currentTime = xTaskGetTickCount(); //当前系统时间
			ml=mocalun_l.motor_value->speed_rpm;
			mr=mocalun_r.motor_value->speed_rpm;
			targetm=mocalun_r.motor_value->target_speed_rpm ;
		Flag_status.protect_flag_heat = false;  //临时使用 记得热量控制启动后注释掉

        switch(ControlMode)
        {
			case REMOTE:
				RemoteShoot();
				REVOLVER_Key_Ctrl();
				break;
			case KEYBOARD:
				SHOOT_Mode_Choose();
				REVOLVER_Key_Ctrl();
				break;
			case UNUSUAL:
				Unusual_Mode_Ctrl();
				break;
			default:
				break;
        }
//		vision_send();
        vTaskDelayUntil(&currentTime, TIME_STAMP_2MS); //绝对延时
    }
}

/**
  * @brief  发射机构各参数初始化
  * @param  void
  * @retval void
  * @attention void
  */
static void REVOLVER_InitArgument(void)
{

	  Flag_status.shoot_single_flag =1;
		Flag_status.shoot_triple_flag =1;
	  Flag_status.shoot_normal_flag =1;
    Flag_status.protect_flag_heat = false;
    Flag_status.protect_flag_sutck = false;
    Flag_status.shoot_cnt = 0;
    Flag_status.shoot_left_time = 0;//计算左键连按时间,时间过长切换成连发

    //(Motortype*motor,int ID,float pid1[3], outmax1, imax1,       float pid2[3], outmax2, imax2
    Motor_Init(&mocalun_l, 1, nothing, PID_DELTA, 0, 0, 3e38, 0, 0, 8192, 0, Mocalun_l_speed, PID_DELTA, 30000, 4000, 3e38, 0, 0.1,  0, 0);
    Motor_Init(&mocalun_r, 2, nothing, PID_DELTA, 0, 0, 3e38, 0, 0, 8192, 0, Mocalun_r_speed, PID_DELTA, 30000, 4000, 3e38, 0, 0.1,  0, 0);
    Motor_Init(&Ammunition_Motor, 6, Ammunition_Motor_Position_pid, PID_POSITION, 8000, 5000, 3000, 0, 0, 8192, 0,
               Ammunition_Motor_Speed_pid, PID_DELTA, 8000, 5000, 3e38, 0, 0.1, 0, 0);
}

/**
  * @brief  键盘模式下发弹模式选择
  * @param  void
  * @retval void
  * @attention  普通模式清零计算,且不发弹
  */
bool single_shoot_flag = 1;
static void SHOOT_Mode_Choose(void)
{
	if(rc.sw1 == 1 && rc.sw2 == 2)
	{
		ControlMode = REMOTE;
		SystemValue = Starting;
	}
    if(ShootState == UNSTART)
    {
        mocalun_l.motor_value->target_speed_rpm = 0;
        mocalun_r.motor_value->target_speed_rpm = 0;

        actShoot = SHOOT_NORMAL;
        ShootState = START;
    }
    else if(ShootState != UNSTART)
    {
		mocalun_l.motor_value->target_speed_rpm = -(int16_t)Judge_Speed();
		mocalun_r.motor_value->target_speed_rpm = (int16_t)Judge_Speed();
        if(IF_MOUSE_PRESSED_LEFT && Flag_status.shoot_left_time <= TIME_STAMP_200MS)//左键发弹
        {
						Flag_status.shoot_left_time++;//判断长按,切换
//						if(Flag_status.flag & (1 << 1))
//						{
//							actShoot_last = actShoot;
//							Clear_Bit(Flag_status.flag,1);
//						}
//						if(!(Flag_status.flag & (1 << 2))) Set_Bit(Flag_status.flag,2);
//						if(!(Flag_status.flag & (1 << 3))) Set_Bit(Flag_status.flag,3);
//						if(!(Flag_status.flag & (1 << 4))) Set_Bit(Flag_status.flag,4);
//						if(!(Flag_status.flag & (1 << 5))) Set_Bit(Flag_status.flag,5);

					  if(Flag_status.shoot_single_flag)
						{
					    actShoot_last = actShoot;
							Flag_status.shoot_single_flag =0;
						}
						if(!(Flag_status.shoot_triple_flag )) Flag_status.shoot_triple_flag =1;
  					if(!(Flag_status.shoot_normal_flag )) Flag_status.shoot_normal_flag =1;
						
						actShoot = SHOOT_SINGLE;			

            mocalun_l.motor_value->target_speed_rpm = -(int16_t)Judge_Speed();
            mocalun_r.motor_value->target_speed_rpm = (int16_t)Judge_Speed();
        }
        else if(IF_MOUSE_PRESSED_LEFT && Flag_status.shoot_left_time > TIME_STAMP_200MS)	//连按大于200ms
        {
            Flag_status.shoot_left_time++;
            Flag_status.shoot_cnt++;
            Flag_status.shoot_cnt = Flag_status.shoot_cnt > 65530? 10 : Flag_status.shoot_cnt;
//            if(Flag_status.flag & (1 << 2))
//            {
//              actShoot_last = actShoot;
//              Clear_Bit(Flag_status.flag,2);
//            }
//            if(!(Flag_status.flag & (1 << 1))) Set_Bit(Flag_status.flag,1);
//            if(!(Flag_status.flag & (1 << 3))) Set_Bit(Flag_status.flag,3);
//            if(!(Flag_status.flag & (1 << 4))) Set_Bit(Flag_status.flag,4);
//            if(!(Flag_status.flag & (1 << 5))) Set_Bit(Flag_status.flag,5);

					  if(Flag_status.shoot_triple_flag)
						{
					    actShoot_last = actShoot;
							Flag_status.shoot_triple_flag =0;
						}
						if(!(Flag_status.shoot_single_flag )) Flag_status.shoot_single_flag =1;
  					if(!(Flag_status.shoot_normal_flag )) Flag_status.shoot_normal_flag =1;
						
            actShoot = SHOOT_TRIPLE;//连发模式

            mocalun_l.motor_value->target_speed_rpm = -(int16_t)Judge_Speed();
            mocalun_r.motor_value->target_speed_rpm = (int16_t)Judge_Speed();
        }
//        else if(IF_KEY_PRESSED_B)
//        {
//            if(Flag_status.flag & (1 << 3))
//            {
//              actShoot_last = actShoot;
//              Clear_Bit(Flag_status.flag,3);
//            }
//            if(!(Flag_status.flag & (1 << 1))) Set_Bit(Flag_status.flag,1);
//            if(!(Flag_status.flag & (1 << 2))) Set_Bit(Flag_status.flag,2);
//            if(!(Flag_status.flag & (1 << 4))) Set_Bit(Flag_status.flag,4);
//            if(!(Flag_status.flag & (1 << 5))) Set_Bit(Flag_status.flag,5);

//            actShoot = SHOOT_NORMAL;//普通模式

//            Flag_status.shoot_cnt = 0;
//            if(actGimbal != GIMBAL_AUTO)
//            {
//              mocalun_l.motor_value->target_speed_rpm = 0;
//              mocalun_r.motor_value->target_speed_rpm = 0;
//            }
//        }
//        else if(IF_KEY_PRESSED_V) //需要按住才有用
//        {
//            mocalun_l.motor_value->target_speed_rpm = -(int16_t)Judge_Speed();
//            mocalun_r.motor_value->target_speed_rpm = (int16_t)Judge_Speed();
//        }
        else if(!IF_MOUSE_PRESSED_LEFT)
        {

//						if(Flag_status.flag & (1 << 4))
//						{
//							actShoot_last = actShoot;
//							Clear_Bit(Flag_status.flag,4);
//						}
//						if(!(Flag_status.flag & (1 << 1))) Set_Bit(Flag_status.flag,1);
//						if(!(Flag_status.flag & (1 << 2))) Set_Bit(Flag_status.flag,2);
//						if(!(Flag_status.flag & (1 << 3))) Set_Bit(Flag_status.flag,3);
//						if(!(Flag_status.flag & (1 << 5))) Set_Bit(Flag_status.flag,5);

//					  if(Flag_status.shoot_normal_flag)
//						{
//					    actShoot_last = actShoot;
//							Flag_status.shoot_normal_flag =0;
//						}
//						if(!(Flag_status.shoot_triple_flag )) Flag_status.shoot_triple_flag =1;
//  					if(!(Flag_status.shoot_single_flag )) Flag_status.shoot_single_flag =1;
						
						actShoot = SHOOT_NORMAL;

						single_shoot_flag=1;
						Flag_status.shoot_left_time = 0;
        }
    }
}

/************************底盘键盘模式各类模式小函数****************************/

/*******键盘模式************/
/**
  * @brief  拨盘的键盘模式
  * @param  void
  * @retval void
  * @attention 键盘用位置环控制
  */
static void REVOLVER_Key_Ctrl(void)
{
    mocalun_l.Motor_PID_Speed.f_cal_pid(&mocalun_l.Motor_PID_Speed, mocalun_l.motor_value->speed_rpm, mocalun_l.motor_value->target_speed_rpm);
    mocalun_r.Motor_PID_Speed.f_cal_pid(&mocalun_r.Motor_PID_Speed, mocalun_r.motor_value->speed_rpm, mocalun_r.motor_value->target_speed_rpm);
	if(abs(mocalun_l.motor_value->speed_rpm) > 200 && abs(mocalun_r.motor_value->speed_rpm) > 200)
		mocalun_status = mNORMAL;
	else
		mocalun_status = mUNUSUAL;
    set_moto1234_current(&hcan1, mocalun_l.Motor_PID_Speed.out, mocalun_r.Motor_PID_Speed.out, 0, 0);
//	    set_moto1234_current(&hcan1, 0, 0, 0, 0);

}

static void Unusual_Mode_Ctrl(void)
{
	mocalun_l.motor_value->target_speed_rpm = -(int16_t)Judge_Speed();
	mocalun_r.motor_value->target_speed_rpm = (int16_t)Judge_Speed();

	mocalun_l.Motor_PID_Speed.f_cal_pid(&mocalun_l.Motor_PID_Speed, mocalun_l.motor_value->speed_rpm, mocalun_l.motor_value->target_speed_rpm);
	mocalun_r.Motor_PID_Speed.f_cal_pid(&mocalun_r.Motor_PID_Speed, mocalun_r.motor_value->speed_rpm, mocalun_r.motor_value->target_speed_rpm);
	set_moto1234_current(&hcan1, mocalun_l.Motor_PID_Speed.out, mocalun_r.Motor_PID_Speed.out, 0, 0);
}

/**
  * @brief  根据裁判系统返回值确定当前射速
  * @param  void
  * @retval void
  * @attention  void
10:5500
  */
int shoot_speed_adjust = 0;
int def = 0;
uint16_t rate=0;

static uint16_t Judge_Speed(void)
{
//    uint16_t rate=0;
    uint8_t Speed_limit=0;
//	  float Speed_42mm;
    Speed_limit = shoot_judge.shoot_limit;
//	  Speed_42mm = shoot_judge.shoot_42mm;
    switch(Speed_limit)
    {
      case 10:
          rate = 3700;//3800
        break;
      case 16:
          rate = 5800;//5400
        break;
      default:
		 rate = 5800;//5400
        break;
    }

	if(!IF_KEY_PRESSED_Q)
	{
		Flag_status.Chassis_Switch_Q = 1;
	}
	if(IF_KEY_PRESSED_Q && Flag_status.Chassis_Switch_Q == 1)
	{
		Flag_status.Chassis_Switch_Q = 0;
		def += 50;
	}	
	if(!IF_KEY_PRESSED_E)
	{
		Flag_status.Chassis_Switch_E = 1;
	}
	if(IF_KEY_PRESSED_E && Flag_status.Chassis_Switch_E == 1)
	{
		Flag_status.Chassis_Switch_E = 0;
		def -= 50;
	}
	shoot_speed_adjust = rate + def;

//	if(Speed_42mm > Speed_limit && Speed_42mm > 0)
//	{
//		def += 2;
//	}
//	else if (Speed_42mm < (Speed_limit - 4)){
//		
//		def -= 5;
//	}
//	rate -= def;
	
    return rate+def;
}

/**
  * @brief  遥控器控制射弹
  * @param  void
  * @retval void
  * @attention void
  */
static void RemoteShoot(void)
{
    if(!(Flag_status.protect_flag_sutck || Flag_status.protect_flag_heat))
    {
        if((rc.sw1 == 3 && rc.sw2 == 2) || Flag_status.moca_flag == 1 || (Flag_status.moca_flag == 0 && Flag_status.Ammunition_flag == 0))
        {
            Flag_status.moca_flag = 1;
            mocalun_l.motor_value->target_speed_rpm = 0;
            mocalun_r.motor_value->target_speed_rpm = 0;
        }
        if((rc.sw1 == 3 && rc.sw2 == 3) || Flag_status.moca_flag == 2)
        {
            Flag_status.moca_flag = 2;
            mocalun_l.motor_value->target_speed_rpm = -6000;
            mocalun_r.motor_value->target_speed_rpm = 6000;
        }
        if((rc.sw1 == 3 && rc.sw2 == 1) || Flag_status.moca_flag == 3)
        {
            Flag_status.moca_flag = 3;
            mocalun_l.motor_value->target_speed_rpm = -7000;
            mocalun_r.motor_value->target_speed_rpm = 7000;
        }
        if((rc.sw1 == 2 && rc.sw2 == 2) || Flag_status.Ammunition_flag == 4)
        {
            Flag_status.Ammunition_flag = 4;
            actShoot_last = actShoot;
            actShoot = SHOOT_NORMAL;//普通模式
        }
        if((rc.sw1 == 2 && rc.sw2 == 3) || Flag_status.Ammunition_flag == 5)
        {
            Flag_status.Ammunition_flag = 5;
            Flag_status.shoot_cnt = 10;
        }
        if((rc.sw1 == 2 && rc.sw2 == 1))
        {
		 	      actShoot_last = actShoot;
            actShoot = SHOOT_NORMAL;//普通模式
            Flag_status.Ammunition_flag = 0;
			      Flag_status.moca_flag = 0;
            ControlMode = KEYBOARD;
			      SystemValue = Starting;
        }
    }

}
