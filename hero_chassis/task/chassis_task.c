/**
  ******************************************************************************
  * @file    chassis_task.c
  * @brief   底盘控制任务，包括遥控器和键盘两种控制模式。以及底盘跟随、小陀螺等等运动模式
  * 的实现，底盘限功率，麦克纳姆轮解算算法，底盘（云台）开机复位。        
  ******************************************************************************
  * @attention
  *
  * 2021.4 考虑到云台复位时只转云台较易受攻击，更改原来只转云台的云台复位方式为转底盘，云台不动
  * 考虑到pid控制的因素，将复位时云台和底盘的动作分开，分别在各自的task中实现
  *
  ******************************************************************************
  */
#include "chassis_task.h"
#include "arm_math.h"
#include "SQ_judge.h"
#include "cap.h"
#include "math.h"

Chassis_Speed absolute_chassis_speed;
int16_t chassis_setspeed[4];    //四个轮子目标转速

PidTypeDef Chassis_Follow_PID;
 fp32 chassisnothing[3] = {0, 0, 0};
 
 

 
 
const static fp32 motorid1_speed_pid[3] ={10,0.39,0};  // {20,0.8,7}; //20, 0.8, 2  25.5, 1, 0
const static fp32 motorid2_speed_pid[3] = {10,0.39,0};
const static fp32 motorid3_speed_pid[3] = {10,0.39,0};
const static fp32 motorid4_speed_pid[3] = {10,0.39,0};
 fp32 Chassis_Follow_pid[3] = {0.00001, 0.00000000001, 0.00005};
                                 
/**
  * @brief  底盘各参数初始化
  * @param  void
  * @retval void
  * @attention void
  */
static void Chassis_InitArgument(void)
{
	//底盘模式切换：普通/跟随
    Flag_status.Chassis_Switch_F = 1;
    Flag_status.Chassis_Key_F_Change = 0;

    //转头180度
    Flag_status.Chassis_Switch_X = 1;
    Flag_status.Chassis_Key_X_Change = 0;

    //小陀螺
	  Flag_status.Chassis_Switch_Shift = 1;
    Flag_status.Chassis_Key_Shift_Change = 0;

    //摩擦轮转速增加
    Flag_status.Chassis_Switch_Q = 1;
    Flag_status.Chassis_Key_Q_Change = 0;

    //摩擦轮转速减小
    Flag_status.Chassis_Switch_E = 1;
    Flag_status.Chassis_Key_E_Change = 0;


    //复位
    Flag_status.Chassis_Switch_Z = 1;
    Flag_status.Chassis_Key_Z_Change = 0;
	
	//电容启动键 短按切换
    Flag_status.Chassis_Switch_C = 1;
    Flag_status.Chassis_Key_C_Change = 0;
	
	//视觉模式切换
    Flag_status.Chassis_Switch_G = 1;
    Flag_status.Chassis_Key_G_Change = 0;
    
    //底盘跟随专用
    pid_init(&Chassis_Follow_PID);//PidTypeDef *pid,           mode,     fp32 PID[3],  max_out,   max_iout, I_Separation, Dead_Zone,    gama, angle_max,  angle_min
    Chassis_Follow_PID.f_param_init(&Chassis_Follow_PID, PID_POSITION, Chassis_Follow_pid, 3.0,     0.003,     1e30,          10,       0.2,    0,         0);

    /******************底盘电机PID*****************************************/
    //Motortype*motor,int ID,float pid1[3],                                  float pid2[3],                outmax2,imax2  I_Separation, Dead_Zone, gama, angle_max,  angle_min
    Motor_Init(&Chassis_Motor[0], 1, chassisnothing, PID_DELTA, 0, 0, 0, 0, 0, 0, 0, motorid1_speed_pid, PID_DELTA, 20000, 5000, 3e38,         0,         0.1,  0, 0);
    Motor_Init(&Chassis_Motor[1], 2, chassisnothing, PID_DELTA, 0, 0, 0, 0, 0, 0, 0, motorid2_speed_pid, PID_DELTA, 20000, 5000, 3e38,         0,         0.1,  0, 0);
    Motor_Init(&Chassis_Motor[2], 3, chassisnothing, PID_DELTA, 0, 0, 0, 0, 0, 0, 0, motorid3_speed_pid, PID_DELTA, 20000, 5000, 3e38,         0,         0.1,  0, 0);
    Motor_Init(&Chassis_Motor[3], 4, chassisnothing, PID_DELTA, 0, 0, 0, 0, 0, 0, 0, motorid4_speed_pid, PID_DELTA, 20000, 5000, 3e38,         0,         0.1,  0, 0);

}
//extern uint8_t cap_switch;
void ChassisFun(void const *argument)
{
    portTickType currentTime;

    Chassis_InitArgument();

    while(1)
    {
        currentTime = xTaskGetTickCount();//当前系统时间
				fellowfdb=absolute_chassis_speed.vw;
				fellowset=0;
				switch(ControlMode)
				{
					case KEYBOARD:
					{
						if(SystemValue == Starting)
						{
		//                    Chassis_open_init();
		//                    SystemValue = Running;      //系统初始化结束--初始化交给云台
						}
						else
						{
							Chassis_Mode_Choose();     //设置车辆所处环境
							KeyboardControlChassis();
							CHASSIS_Single_Loop_Out();
						}
						break;
					}
					case REMOTE:
					{
						if(SystemValue == Starting)
						{
		//					  Chassis_open_init();
		//                    SystemValue = Running;      //系统初始化结束
						}
						else
						{
							RemoteModeChoose();
							RemoteControlChassis();
							CHASSIS_Single_Loop_Out();
						}
						break;
					}
					case UNUSUAL:
					{
						Chassis_Mode_Choose();     //设置车辆所处环境
						KeyboardControlChassis();	
						
						break;				
					}
					default:
						break;
				}
//			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);  //控制频率测试

        vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
    }
}

/**
  * @brief  底盘电机输出
  * @param  void
  * @retval void
  * @attention
  */
static void CHASSIS_Single_Loop_Out()
{
    if(actChassis == CHASSIS_GYROSCOPE || actChassis == CHASSIS_NORMAL)
    {
        Absolute_Cal(&absolute_chassis_speed, (float)(GIMBAL_YAW_ENCODER_MIDDLE1 - Gimbal_MotorImuYaw.motor_value->angle) * 0.043945f);
    }
    else
    {
        Absolute_Cal(&absolute_chassis_speed, 0);
    }
    Mecanum_Set_Motor_Speed(chassis_setspeed, Chassis_Motor); //设置各个电机的目标速度
    /************************************底盘电机速度环计算*********************************************/
   
		
		Chassis_Motor[0].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[0].Motor_PID_Speed,
            Chassis_Motor[0].motor_value->speed_rpm,
            -Chassis_Motor[0].motor_value->target_speed_rpm);

    Chassis_Motor[1].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[1].Motor_PID_Speed,
            Chassis_Motor[1].motor_value->speed_rpm,
            +Chassis_Motor[1].motor_value->target_speed_rpm);

    Chassis_Motor[2].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[2].Motor_PID_Speed,
            Chassis_Motor[2].motor_value->speed_rpm,
            -Chassis_Motor[2].motor_value->target_speed_rpm);

    Chassis_Motor[3].Motor_PID_Speed.f_cal_pid(&Chassis_Motor[3].Motor_PID_Speed,
            Chassis_Motor[3].motor_value->speed_rpm,
            +Chassis_Motor[3].motor_value->target_speed_rpm);
    /************************************将电流参数发送给电机*********************************************/
    Send_cap_msg();
    Chassis_Power_Limit();//功率限制,电流重新分配
    set_moto1234_current(&hcan1, Chassis_Motor[0].Motor_PID_Speed.out,
                        Chassis_Motor[1].Motor_PID_Speed.out,
                        Chassis_Motor[2].Motor_PID_Speed.out,
                        Chassis_Motor[3].Motor_PID_Speed.out);

}

/***********************************************************************************下面的部分是遥控器模式**********************************************************************************/
/**
  * @brief  遥控器模式控制
  * @param  void
  * @retval void
  * @attention
  */
static void RemoteModeChoose(void)
{
	if(rc.sw1 == 1 && rc.sw2 == 3)
		actChassis = CHASSIS_FOLLOW_GIMBAL;  //底盘跟随云台
	else if(rc.sw1 == 1 && rc.sw2 == 2)
		actChassis = CHASSIS_NORMAL;  //底盘不跟随云台
	else if(rc.sw1 == 1 && rc.sw2 == 1)
		actChassis = CHASSIS_GYROSCOPE;    //小陀螺模式
}

/**
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention
  */

float ttt= 0.0008;
static void RemoteControlChassis(void)
{
    /***********************************确定底盘四个电机的目标速度*****************************************/
    switch(actChassis)
    {
			case CHASSIS_FOLLOW_GIMBAL://跟随云台
				
				if(((Gimbal_MotorImuYaw.motor_value->angle > GIMBAL_YAW_ENCODER_NINETY1)&&(Gimbal_MotorImuYaw.motor_value->angle < GIMBAL_YAW_ENCODER_MIDDLE1)) ||
						((Gimbal_MotorImuYaw.motor_value->angle > GIMBAL_YAW_ENCODER_MIDDLE1)&&(Gimbal_MotorImuYaw.motor_value->angle < GIMBAL_YAW_ENCODER_NINETY2)))
				{
					gimbal_follow = GIMBAL_HEAD;
				absolute_chassis_speed.vx = RAMP_float((float)rc.ch4 / 200 , absolute_chassis_speed.vx  , 0.1);
				absolute_chassis_speed.vy = RAMP_float((float)rc.ch3 / 200 , absolute_chassis_speed.vy  , 0.1);
				}
				else
				{
					gimbal_follow = GIMBAL_TAIL;
				absolute_chassis_speed.vx = RAMP_float(-(float)rc.ch4 / 200 , absolute_chassis_speed.vx  , 0.1);
				absolute_chassis_speed.vy = RAMP_float(-(float)rc.ch3 / 200 , absolute_chassis_speed.vy  , 0.1);
				}
				if(!(Flag_status.chassis_follow_flag))
					absolute_chassis_speed.vw = RAMP_float(Chassis_Follow_PID.f_cal_pid(&Chassis_Follow_PID, Find_Y_AnglePNY(), 0),
						absolute_chassis_speed.vw, 0.0008); //PID使底盘跟随云台速度
				else 
					absolute_chassis_speed.vw = 0;
				break;
			case CHASSIS_NORMAL://不跟随云台
				absolute_chassis_speed.vx = RAMP_float((float)rc.ch4 / 200 , absolute_chassis_speed.vx  , 0.1);
				absolute_chassis_speed.vy = RAMP_float((float)rc.ch3 / 200 , absolute_chassis_speed.vy  , 0.1);
				absolute_chassis_speed.vw = 0;
				break;
			case CHASSIS_GYROSCOPE:    //小陀螺模式
				absolute_chassis_speed.vx = (float)rc.ch4 / 200;
				absolute_chassis_speed.vy = (float)rc.ch3 / 200;
				absolute_chassis_speed.vw = 0.0032;
				break;
			default:
					break;
    }
}

/***********************************************************************************下面的部分是键盘模式**********************************************************************************/

/**
  * @brief  键盘选择底盘模式
  * @param  void
  * @retval void
  * @attention 模式选择,进入某模式后记得写退出到普通模式的判断
  * 无按键按下会一直处于自动闪避模式,除了模式切换外的按键按下则处于模式切换选择模式
  */
static void Chassis_Mode_Choose()  //键盘控制下
{
    //////////////////F键切换底盘模式：普通模式/跟随模式/////////////////////	
		if(!IF_KEY_PRESSED_F)
		{
			Flag_status.Chassis_Switch_F = 1;
		}
		if(IF_KEY_PRESSED_F && Flag_status.Chassis_Switch_F == 1)
		{
			Flag_status.Chassis_Switch_F = 0;
			Flag_status.Chassis_Key_F_Change ++;
			Flag_status.Chassis_Key_F_Change %= 2;
			
			if(Flag_status.Chassis_Key_F_Change)
			{
				actChassis_last = actChassis;
				actChassis = CHASSIS_FOLLOW_GIMBAL;
			}else
			{
				actChassis_last = actChassis;
				actChassis = CHASSIS_NORMAL;
				Flag_status.chassis_follow_flag = 0;
			}
		}
	/////////////////Shift键选择小陀螺--长按保持模式/////////////////////
    if(IF_KEY_PRESSED_SHIFT)
    {
		if(!(ControlMode == UNUSUAL))
		{
			actChassis_last = actChassis;
			actChassis = CHASSIS_GYROSCOPE;
		}

    }
    if(!IF_KEY_PRESSED_SHIFT)
    {
		if(actChassis == CHASSIS_GYROSCOPE)
		{
		   actChassis_last = actChassis;
			 actChassis = CHASSIS_FOLLOW_GIMBAL;
		}

    }

    /////////////////Z键按下触发软件复位///////////////////
    if(!IF_KEY_PRESSED_Z)
    {
        Flag_status.Chassis_Switch_Z = 1;
    }
    if(IF_KEY_PRESSED_Z && Flag_status.Chassis_Switch_Z == 1)
    {
		    set_cap1(&hcan2, CAP_STATUS_UNUSUAL, CAP_SWITCH_CLOSE, 0);
        Flag_status.Chassis_Switch_Z = 0;
        Stop_All();//电机停转
        soft_rest(); //软件复位
    }
}


/**
  * @brief  键盘控制底盘模式
  * @param  void
  * @retval void
  * @attention
  */
static void KeyboardControlChassis(void)
{
    switch (actChassis)
    {
				case CHASSIS_NORMAL:
				{
					if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
					{
						if(cap_info.cap_mode == CAP_NOMAL)
							Chassis_Keyboard_Move_Calculate(15000, 10, 2000);
						else if(cap_info.cap_mode == CAP_SLOPE)
							Chassis_Keyboard_Move_Calculate(15000, 10, 2000);
						
					}else{	
						Chassis_Keyboard_Move_Calculate(10000, 10, 2000);
					}
				}
				break;
				case CHASSIS_FOLLOW_GIMBAL:
				{
					if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
					{
						if(cap_info.cap_mode == CAP_NOMAL)
							Chassis_Keyboard_Move_Calculate(15000, 10, 2000);

						else if(cap_info.cap_mode == CAP_SLOPE)
							Chassis_Keyboard_Move_Calculate(15000, 10, 2000);
						
					}else{	
						Chassis_Keyboard_Move_Calculate(10000, 10, 2000);
					}
					
					Chassis_Mouse_Move_Calculate();
				}
				break;
				case CHASSIS_GYROSCOPE:
				{
					if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
					{
						if(cap_info.cap_mode == CAP_NOMAL)
							Chassis_Keyboard_Move_Calculate(15000, 8, 30);
						else if(cap_info.cap_mode == CAP_SLOPE)
							Chassis_Keyboard_Move_Calculate(15000, 8, 30);
					}else{
						Chassis_Keyboard_Move_Calculate(10000, 8, 30);
					}
					
					if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN){
						absolute_chassis_speed.vw = RAMP_float(0.006, absolute_chassis_speed.vw, 0.00005);
						
					}else{
						if(JUDGE_usGetPowerLimit() <= 60)
							absolute_chassis_speed.vw = RAMP_float(0.0025, absolute_chassis_speed.vw, 0.00005);//0.025
						else if(JUDGE_usGetPowerLimit() >60 && JUDGE_usGetPowerLimit() <= 80)
							absolute_chassis_speed.vw = RAMP_float(0.0035, absolute_chassis_speed.vw, 0.00005);//0.035
						else
							absolute_chassis_speed.vw = RAMP_float(0.005, absolute_chassis_speed.vw, 0.00005);
					}
				}
				break;
				default:
					break;
    }
}

/**
  * @brief  键盘模式下底盘运动计算
  * @param  速度最大输出量    增加速度(最大293)
  * @retval void
  * @attention  键盘控制前后左右平移,平移无机械和陀螺仪模式之分
  *             需要获取时间来进行斜坡函数计算
  */
/************底盘各类模式的一些辅助定义*************/
float    Chassis_Standard_Move_Max;                 //底盘前后左右平移限速
int16_t  timeXFron, timeXBack, timeYLeft, timeYRigh;//键盘  s  w  d  a

//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back;
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

static void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp_inc, int16_t sMoveRamp_dec )
{
    static portTickType  ulCurrentTime = 0;
    static uint32_t  ulDelay = 0;

	  static uint16_t w_cnt = 0;
	  static bool W = 0;
		static uint16_t s_cnt = 0;
		static bool S = 0;
    static uint16_t a_cnt = 0;
    static bool A = 0;
    static uint16_t d_cnt = 0;
    static bool D = 0;
	
	  static uint32_t Mouse_Stop  = 0;
    static float Mouse_w;  //键盘陀螺仪模式下鼠标统计w增量,此值会自己缓慢减小,防止甩头过快

    Chassis_Standard_Move_Max = sMoveMax;//调整速度限幅,水平移动
    ulCurrentTime = xTaskGetTickCount();//当前系统时间

    if (ulCurrentTime >= ulDelay)//每10ms变化一次斜坡量
    {
        ulDelay = ulCurrentTime + 10;

        if (IF_KEY_PRESSED_W)
        {
            w_cnt = 0;
            W = 1;
            timeXBack = 0;//按下前进则后退斜坡归零,方便下次计算后退斜坡
        }
        else
        {
            w_cnt++;
        }
        if(w_cnt > 10)
        {
            w_cnt = 0;
            W = 0;
        }

        if (IF_KEY_PRESSED_S)
        {
            s_cnt = 0;
            S = 1;
            timeXFron = 0;//同理
        }
        else
        {
            s_cnt++;
        }
        if(s_cnt > 10)
        {
            s_cnt = 0;
            S = 0;
        }

        if (IF_KEY_PRESSED_D)
        {
            d_cnt = 0;
            D=1;
            timeYRigh = 0;
        }
        else
        {
            d_cnt++;
        }
        if(d_cnt > 10)
        {
            d_cnt = 0;
            D = 0;
        }

        if (IF_KEY_PRESSED_A)
        {
            a_cnt = 0;
            A=1;
            timeYLeft = 0;
        }
        else
        {
            a_cnt++;
        }
        if(a_cnt > 10)
        {
            a_cnt = 0;
            A = 0;
        }
        if(actChassis == CHASSIS_NORMAL)
        {

					if(ControlMode == UNUSUAL)
					{
						if(MOUSE_X_MOVE_SPEED != 0)
						{
							Mouse_w = MOUSE_X_MOVE_SPEED * 0.00005f;	
						}
						else if(MOUSE_X_MOVE_SPEED == 0)
						{
							Mouse_Stop ++ ;
							if(Mouse_Stop > 5) //鼠标长时间停留，停止移动
							{
								Mouse_w = RAMP_float(0, Mouse_w, 50);
							}
						}
						else
						{
							Mouse_w = RAMP_float(0, Mouse_w, 100);
						}
				
						absolute_chassis_speed.vw = RAMP_float(Mouse_w, absolute_chassis_speed.vw, 0.0002);
			
					}
					else
					{
						absolute_chassis_speed.vw = RAMP_float(0, absolute_chassis_speed.vw, 0.0001);				
					}
        }


        Slope_Chassis_Move_Fron = (int16_t)( Chassis_Standard_Move_Max *
                                             Chassis_Key_MoveRamp( W, &timeXFron, sMoveRamp_inc, sMoveRamp_dec ) );
        
				Slope_Chassis_Move_Back = (int16_t)( -Chassis_Standard_Move_Max *
                                             Chassis_Key_MoveRamp( S, &timeXBack, sMoveRamp_inc, sMoveRamp_dec ) );

        Slope_Chassis_Move_Left = (int16_t)( -Chassis_Standard_Move_Max *
                                             Chassis_Key_MoveRamp( A, &timeYRigh, sMoveRamp_inc / 2.0f, sMoveRamp_dec ) );

        Slope_Chassis_Move_Righ = (int16_t)( Chassis_Standard_Move_Max *
                                             Chassis_Key_MoveRamp( D, &timeYLeft, sMoveRamp_inc / 2.0f, sMoveRamp_dec ) );

        if(actChassis == CHASSIS_FOLLOW_GIMBAL)
        {
            if(((Gimbal_MotorImuYaw.motor_value->angle > GIMBAL_YAW_ENCODER_NINETY1)&&(Gimbal_MotorImuYaw.motor_value->angle < GIMBAL_YAW_ENCODER_MIDDLE1)) ||
						((Gimbal_MotorImuYaw.motor_value->angle > GIMBAL_YAW_ENCODER_MIDDLE1)&&(Gimbal_MotorImuYaw.motor_value->angle < GIMBAL_YAW_ENCODER_NINETY2)))
						{
							gimbal_follow = GIMBAL_HEAD;
							if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
							{
								absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //前后计算
								absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //左右计算
								
							}else{
								absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //前后计算
								absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //左右计算
							}

            }
            else
            {
							gimbal_follow = GIMBAL_TAIL;
							if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
							{
								absolute_chassis_speed.vx  = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //前后计算
								absolute_chassis_speed.vy  = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //左右计算
								
							}else{
								absolute_chassis_speed.vx  = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //前后计算
								absolute_chassis_speed.vy  = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //左右计算
							}
            }
        }
        else
        {
					if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
					{
						absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //前后计算
						absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //左右计算
						
					}else{
						absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //前后计算
						absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //左右计算
					}
        }
    }
}
/**
  * @brief  底盘键盘斜坡函数
  * @param  判断按键是否被按下, 时间量, 每次增加的量, 一共要减小的量
  * @retval 斜坡比例系数
  * @attention  0~1
  */
static float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
    float  factor = 0;
    factor = 0.15 * sqrt( 0.15 * (*time) );  //计算速度斜坡,time累加到296.3斜坡就完成

    if (status == 1){  //按键被按下
		
        if (factor < 1)//防止time太大
			*time += inc;
		
    }else{  //按键松开
        if (factor > 0)
        {
            *time -= dec;
            if (*time < 0)
				*time = 0;
        }
    }
    LimitValue_f(&factor,1,0);//注意一定是float类型限幅
	
    return factor;  //注意方向
}


/**
  * @brief  鼠标控制底盘旋转,键盘QEC控制快速转圈
  * @param  速度最大输出量
  * @retval void
  * @attention  鼠标控制左右旋转
  */
static void Chassis_Mouse_Move_Calculate(void)
{
	if(!Flag_status.chassis_follow_flag){
		
		absolute_chassis_speed.vw = RAMP_float(Chassis_Follow_PID.f_cal_pid(&Chassis_Follow_PID, Find_Y_AnglePNY(), 0), absolute_chassis_speed.vw, 0.0008);
	}
	else{
		absolute_chassis_speed.vw = 0;
	}
}

/***********************************************************************************对底盘电机进行的各种处理**********************************************************************************/

/*****************底盘功率*************************/
//不同模式下的最高速度
#define    CHAS_CURRENT_LIMIT_Lv0        27500
#define    CHAS_CURRENT_LIMIT_Lv1        31000    //四个轮子的速度总和最大值,单个输出*4,限功率调比例可用
#define    CHAS_CURRENT_LIMIT_Lv2        32000
#define    CHAS_CURRENT_LIMIT_Lv3        33000
#define    CHAS_CURRENT_LIMIT_Lv4        33000
#define    CHAS_CURRENT_LIMIT_Lv5        33000
#define    CHAS_CURRENT_LIMIT_Lv6        33000
#define    CHAS_CURRENT_LIMIT_Lv7        33000
#define    CHAS_CURRENT_LIMIT_Lv8        33000
#define    CHAS_CURRENT_LIMIT_Lv9        33000
#define    CHAS_CURRENT_LIMIT_Lv10        33000


float WARNING_REMAIN_POWER = 40;//裁判系统剩余焦耳能量低于这个数值则开始限功率,40扭屁股会超功率,平地开不会超
float fChasCurrentLimit = CHAS_CURRENT_LIMIT_Lv3;//限制4个轮子的速度总和
float fTotalCurrentLimit;//电流分配,平地模式下分配是均匀的
/**
  * @brief  底盘功率限制
  * @param  void
  * @retval void
  * @attention  在底盘输出计算后调用,主要是比例的算法,ICRA
  */
	
	
PidTypeDef buffer_pid; 
fp32 buffer[3]={2,0.2,0};
void get_chassis_power_and_buffer(float *chassis_power,float *chassis_power_buffer)
{
    *chassis_power=JUDGE_fGetChassisPower();//读取实时功率
    *chassis_power_buffer=JUDGE_fGetRemainEnergy();//读取剩余焦耳能量
}

static void Chassis_Power_Limit()
{
    uint8_t cap_state=0;
    uint16_t max_power_limit=40;
    fp32 chassis_max_power=0;
    float input_power=0;
    float initial_give_power[4];
    float initial_total_power=0;
    fp32 scaled_give_power[4];
    fp32 chassis_power=0.0f;
    fp32 chassis_power_buffer=0.0f;
    fp32 toque_coefficient=1.99688994e-6f;
    fp32 k1=1.453e-07;
    //实际上是k2   1.453e-07;  1.255e-07;           1.653e-07
    fp32 k2=1.23e-07;
    //实际上是k1   1.23e-07;    1.44e-07;           1.43e-07
    fp32 constant=2.081f;
    //             4.081f;       3.343;小陀螺不稳   2.081f  这个也是，但是不开小陀螺巨稳
    get_chassis_power_and_buffer(&chassis_power,&chassis_power_buffer);
    pid_init(&buffer_pid);
	  buffer_pid.f_param_init(&buffer_pid , PID_POSITION,buffer,100,1000,0,0,0,0,0);
    buffer_pid.f_cal_pid(&buffer_pid,chassis_power_buffer,30);//pid控制缓冲能量，保留30
    max_power_limit=JUDGE_usGetPowerLimit();
    input_power=max_power_limit-buffer_pid.out;                   
//        input_power=max_power_limit;
    if(actChassis==CHASSIS_GYROSCOPE)
    {
    switch (max_power_limit)
   {
       case 50:absolute_chassis_speed.vw=0.0046; break;
       case 60:absolute_chassis_speed.vw=0.0058; break;
       case 70:absolute_chassis_speed.vw=0.0065; break;
       case 80:absolute_chassis_speed.vw=0.0072; break;
       case 100:absolute_chassis_speed.vw=0.0076; break;
       case 120:absolute_chassis_speed.vw=0.008; break;
       default:absolute_chassis_speed.vw=0.0046;
        break;
   }
    }
        
    if (IF_KEY_PRESSED_SHIFT)
	{
		cap_state = 0;

	}
	if (IF_KEY_PRESSED_C)
	{
		cap_state = 1;
	}

//	if (Power.cap_percent.value>5)
//	{
//		if (cap_state == 0)
//		{
//			chassis_max_power = input_power + 5; // Slightly greater than the maximum power, avoiding the capacitor being full all the time and improving energy utilization
//		}
//		else
//		{
//			chassis_max_power = input_power + 200;
//		}
//	}
//	else
//	{
		chassis_max_power = input_power;
//	}
    
    for(uint8_t i=0;i<4;i++)
    {
        initial_give_power[i]=Chassis_Motor[i].Motor_PID_Speed.out*toque_coefficient*Chassis_Motor[i].motor_value->speed_rpm
        +k2*Chassis_Motor[i].motor_value->speed_rpm*Chassis_Motor[i].motor_value->speed_rpm
        +k1*Chassis_Motor[i].Motor_PID_Speed.out*Chassis_Motor[i].Motor_PID_Speed.out
        +constant;
    
    
    if(initial_give_power[i]<0)
        continue;
    initial_total_power+=initial_give_power[i];
    }
    
    if(initial_total_power>chassis_max_power)
    {
    fp32 power_scale =chassis_max_power/initial_total_power;
    for(uint8_t i=0;i<4;i++)
        {
            scaled_give_power[i]=initial_give_power[i]*power_scale;
           if(scaled_give_power[i]<0)
              continue;
              
               fp32 b=toque_coefficient * Chassis_Motor[i].motor_value->speed_rpm;
               fp32 c=k2*Chassis_Motor[i].motor_value->speed_rpm*Chassis_Motor[i].motor_value->speed_rpm-scaled_give_power[i]+constant;
        
        if(Chassis_Motor[i].Motor_PID_Speed.out>0)
            {
                fp32 temp=(-b+sqrt(b*b-4*k1*c))/(2*k1);
                if(temp>16000)

                {
                  Chassis_Motor[i].Motor_PID_Speed.out=16000;  
                }
                else Chassis_Motor[i].Motor_PID_Speed.out=temp;
                
            }
                else 
                {
            fp32 temp=(-b-sqrt(b*b-4*k1*c))/(2*k1);
                if(temp<-16000)
                {
                Chassis_Motor[i].Motor_PID_Speed.out=-16000;
                }
                else Chassis_Motor[i].Motor_PID_Speed.out=temp;
                }
            }
    }
}


//float    kLimit = 1;//功率限制系数
//float    chassis_totaloutput = 0;//统计总输出电流
//float    Joule_Residue = 0;//剩余焦耳缓冲能量
//float    Power_Limit_Out = 0;
//uint16_t Power_realtime = 0;
//uint16_t Power_limit = 0;
//int16_t  judgDataCorrect = 0;//裁判系统数据是否可用
//static int32_t judgDataError_Time = 0;

//static void Chassis_Power_Limit()
//{
//    /*********************祖传算法*************************/
//    Power_realtime = JUDGE_fGetChassisPower(); //获取实时底盘功率
//    Power_limit = JUDGE_usGetPowerLimit();  //获取功率限制
//    judgDataCorrect = JUDGE_sGetDataState();//裁判系统数据是否可用
//    Joule_Residue = JUDGE_fGetRemainEnergy();//剩余焦耳能量
//    
//		uint16_t fChasCurrentLimit_Min = 28000;
//		switch(Power_limit)
//			{	
//				case 55:
//						fChasCurrentLimit  =  fChasCurrentLimit_Min ;
//						break;
//				case 60:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min+1000 ; 
//						break;
//				case 65:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min+2000 ; 
//						break;
//				case 70:
//						fChasCurrentLimit  =  fChasCurrentLimit_Min+3000 ; 
//						break;
//				case 75:
//						fChasCurrentLimit  =  fChasCurrentLimit_Min+4000 ; 
//						break;
//				case 80:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min+5000 ; 
//						break;
//				case 85:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min+6000 ; 
//						break;
//				case 90:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min+7000 ; 
//						break;
//				case 95:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min+9000 ; 
//						break;
//				case 100:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min+10000 ; 
//						break;
//				case 105:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min+11000 ; 
//						break;
//				case 110:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min + 12000 ; 
//						break;
//				case 120:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min + 1300 ; 				
//						break;
//				default	:
//					  fChasCurrentLimit  =  fChasCurrentLimit_Min  ; 				
//						break;					
//			}
//				

//	
//    //统计底盘总输出
//    chassis_totaloutput = abs_float(Chassis_Motor[0].Motor_PID_Speed.out) + abs_float(Chassis_Motor[1].Motor_PID_Speed.out)
//                          + abs_float(Chassis_Motor[2].Motor_PID_Speed.out) + abs_float(Chassis_Motor[3].Motor_PID_Speed.out);

//    if(judgDataCorrect == JUDGE_DATA_ERROR)//裁判系统无效时强制限速
//    {
//        judgDataError_Time++;
//        if(judgDataError_Time > 100)
//        {
//            fTotalCurrentLimit = fChasCurrentLimit/2;//降为最大的1/4
//        }
//    }
//    else
//    {
//      judgDataError_Time = 0;   
//        //剩余焦耳量过小,开始限制输出,限制系数为平方关系
//        if(Joule_Residue < WARNING_REMAIN_POWER)
//        {
//			if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN){
//				kLimit = RAMP_float((float)(Joule_Residue / WARNING_REMAIN_POWER), kLimit, 0.04);
////				kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER);
//			}else{
//				kLimit = (float)(Joule_Residue / WARNING_REMAIN_POWER)
//						 * (float)(Joule_Residue / WARNING_REMAIN_POWER);
//			}

//            fTotalCurrentLimit = kLimit * fChasCurrentLimit;
//        }
//        else   //焦耳能量恢复到一定数值
//        {
//            fTotalCurrentLimit = fChasCurrentLimit;
//        }
//    }

//    //底盘各电机电流重新分配
//    if (chassis_totaloutput > fTotalCurrentLimit)
//    {
//        Chassis_Motor[0].Motor_PID_Speed.out = (float)(Chassis_Motor[0].Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
//        Chassis_Motor[1].Motor_PID_Speed.out = (float)(Chassis_Motor[1].Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
//        Chassis_Motor[2].Motor_PID_Speed.out = (float)(Chassis_Motor[2].Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
//        Chassis_Motor[3].Motor_PID_Speed.out = (float)(Chassis_Motor[3].Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
//    }

//}

///**
//  * @brief  设置底盘电机输出电流最大值
//  * @param  电机1最大值，电机2最大值，电机3最大值，电机4最大值
//  * @retval void
//  * @attention
//  */
//static void SetChassisMotorMaxCurrent(const int16_t max1, const int16_t max2, const int16_t max3, const int16_t max4)
//{
//    Chassis_Motor[0].Motor_PID_Speed.max_out = max1;
//    Chassis_Motor[1].Motor_PID_Speed.max_out = max2;
//    Chassis_Motor[2].Motor_PID_Speed.max_out = max3;
//    Chassis_Motor[3].Motor_PID_Speed.max_out = max4;
//}

///**
//  * @brief  根据不同情况进行限流
//  * @param  void
//  * @retval void
//  * @attention void
//  */
//void LimitChassisMotorCurrent(void)
//{
//    switch(actChassis)
//    {
//    case CHASSIS_NORMAL:    //底盘不跟随云台
//        switch(EnvironmentMode)
//        {
//			case NOMAL:      //普通地形
//				SetChassisMotorMaxCurrent(NOMOAL_CHASSIS_MAX1, NOMOAL_CHASSIS_MAX2, NOMOAL_CHASSIS_MAX3, NOMOAL_CHASSIS_MAX4);
//				break;
//			case CLIMBING:    //爬坡地形
//				SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_MAX1, CLIMBING_CHASSIS_MAX2, CLIMBING_CHASSIS_MAX3, CLIMBING_CHASSIS_MAX4);
//				break;
//        }
//        break;
//    case CHASSIS_FOLLOW_GIMBAL:      //底盘跟随云台
//    case CHASSIS_GYROSCOPE:      //小陀螺模式
//    default:
//        break;
//    }
//}

/*
 * @param absolute_speed 绝对坐标需要的速度
 * @param angle 云台相对于底盘的角度
 */
static void Absolute_Cal(Chassis_Speed *absolute_speed, float angle )
{
    float angle_hd = angle * PI / 180;
    Chassis_Speed temp_speed;
    temp_speed.vw = absolute_speed->vw;
    temp_speed.vx = absolute_speed->vx * cos(angle_hd) - absolute_speed->vy * sin(angle_hd);
    temp_speed.vy = absolute_speed->vx * sin(angle_hd) + absolute_speed->vy * cos(angle_hd);
    mecanum_calc(&temp_speed, chassis_setspeed);
}

static void mecanum_calc(Chassis_Speed *speed, int16_t *out_speed)
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;

    wheel_rpm[0] = ( speed->vx - speed->vy - speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[1] = ( speed->vx + speed->vy + speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[2] = (-speed->vx + speed->vy - speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;
    wheel_rpm[3] = (-speed->vx - speed->vy + speed->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;

    memcpy(out_speed, wheel_rpm, 4 * sizeof(int16_t));
}

static void Mecanum_Set_Motor_Speed(int16_t *out_speed, Motortype *Motor )
{
    Motor[0].motor_value->target_speed_rpm = out_speed[0];
    Motor[1].motor_value->target_speed_rpm = out_speed[1];
    Motor[2].motor_value->target_speed_rpm = out_speed[2];
    Motor[3].motor_value->target_speed_rpm = out_speed[3];

}

/**
  * @brief  找出与+y轴最小偏差角
  * @param  void
  * @retval 偏差角，角度制
  * @attention 通过遥控器/键盘
  */
static float Find_Y_AnglePNY(void)
{
    float temp1 = Gimbal_MotorImuYaw.motor_value->angle - GIMBAL_YAW_ENCODER_MIDDLE1;
    float temp2 = Gimbal_MotorImuYaw.motor_value->angle - GIMBAL_YAW_ENCODER_MIDDLE2;
    float mintemp1;
    if(temp1 > 4096)
        temp1 -= 8192;
    else if(temp1 < -4096)
        temp1 += 8192;
    if(temp2 > 4096)
        temp2 -= 8192;
    else if(temp2 < -4096)
        temp2 += 8192;

    mintemp1 = (abs((int32_t)temp1) < abs((int32_t)temp2) ? temp1 : temp2);
    return mintemp1;
}
/**
  * @brief  找出与45°轴最小偏差角
  * @param  void
  * @retval 偏差角，角度制
  * @attention 通过遥控器/键盘
  */
static float FindMinAngleFortyFive(void)
{
    float temp1 = Gimbal_MotorImuYaw.motor_value->angle - GIMBAL_YAW_ENCODER_FORTYFIVE1;
    float temp2 = Gimbal_MotorImuYaw.motor_value->angle - GIMBAL_YAW_ENCODER_FORTYFIVE2;
    float temp3 = Gimbal_MotorImuYaw.motor_value->angle - GIMBAL_YAW_ENCODER_FORTYFIVE3;
    float temp4 = Gimbal_MotorImuYaw.motor_value->angle - GIMBAL_YAW_ENCODER_FORTYFIVE4;
    float mintemp1, mintemp2;
    if(temp1 > 4096)
        temp1 -= 8192;
    else if(temp1 < -4096)
        temp1 += 8192;
    if(temp2 > 4096)
        temp2 -= 8192;
    else if(temp2 < -4096)
        temp2 += 8192;
    if(temp3 > 4096)
        temp3 -= 8192;
    else if(temp3 < -4096)
        temp3 += 8192;
    if(temp4 > 4096)
        temp4 -= 8192;
    else if(temp4 < -4096)
        temp4 += 8192;
    mintemp1 = (abs((int32_t)temp1) < abs((int32_t)temp2) ? temp1 : temp2);
    mintemp2 = (abs((int32_t)temp3) < abs((int32_t)temp4) ? temp3 : temp4);
    return (abs((int32_t)mintemp1) < abs((int32_t)mintemp2) ? mintemp1 : mintemp2);
}
