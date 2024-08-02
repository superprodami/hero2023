#include "DMgimbal_task.h"
#include "judge.h"
#include "usbd_cdc_if.h"
#include "Motor_DM.h"
#include "motor.h"
#include "myfun.h" 


/*************卡尔曼滤波**************/
/*二阶卡尔曼*/  //自瞄
//云台角度误差卡尔曼
kalman_filter_init_t yaw_kalman_filter_para =
{
    .xhat_data = {0, 0},                                  //状态矩阵 位置（角度）、速度 (会被快速迭代，不是很重要)
    .P_data = {2, 0, 0, 2},                               //预测状态协方差矩阵(会被快速迭代，不是很重要)
    .A_data = {1, 0.002, 0, 1},//采样时间间隔             //状态转移矩阵
    .B_data = {0.002 * 0.002 * 0.5, 0.002},               //控制矩阵
    .Q_data = {1, 0, 0, 1},                               //状态转移矩阵的协方差矩阵 “预测模型”本身的噪声
    .H_data = {1, 0, 0, 1},                               //观测矩阵
    .R_data = {200, 0, 0, 400}//500 1000                  //观测矩阵的协方差(噪声方差)
};//初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para =
{
    .xhat_data = {0, 0},
    .P_data = {2, 0, 0, 2},
    .A_data = {1, 0.002, 0, 1},//采样时间间隔
    .B_data = {0.002 * 0.002 * 0.5, 0.002},
    .Q_data = {1, 0, 0, 1},
    .H_data = {1, 0, 0, 1},
    .R_data = {200, 0, 0, 400}
};//初始化pitch的部分kalman参数

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
extKalman_t follow_kalman_yaw;
extKalman_t follow_kalman_pitch;
float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度
PidTypeDef VISION_PITCH;
#define pi 3.1415926536f
fp32 visionpitch;


	static void GIMBAL_InitArgument(void)
{
		fp32 Gimbal_DM_Motor_Position_pid[3] =   {18,0,2}; //编码器模式
		fp32 Gimbal_DM_Motor_Speed_pid[3] ={0.5,0.009,0.4};
		fp32 Gimbal_DM_Pitch_imu_pid[3]={6,0,38};  

		fp32 VISION_PITCH_PID[3] = {0.02,0.00001,0.02};

    PID_clear(&Gimbal_MotorPitch.Motor_PID_Position);
    PID_clear(&Gimbal_MotorPitch.Motor_PID_Speed);

    Flag_status.follow_flag_remote=1;
    Flag_status.FLAG_Key=1;
    Flag_status.follow_flag_key=1;
 	
		Motor_Init_DM(&Gimbal_DM_MotorPitch, 2,Gimbal_DM_Pitch_imu_pid, PID_POSITION, 10, 5, 1e30, 0, 0.2, 0, 0 , 
											Gimbal_DM_Motor_Speed_pid, PID_POSITION, 10, 3, 3e38,0,0.1,0, 0);
    Gimbal_DM_MotorPitch.motor_value->hall = 0;  //用来判断是否收到达秒电机信号
		Cloud_Angle_Target[PITCH][GYRO] = 0;
		
		pid_init(&VISION_PITCH);
		VISION_PITCH.f_param_init(&VISION_PITCH, PID_DELTA, VISION_PITCH_PID, 45,20,20,0,0.75,0,0);// 45,45,2,0,0,0,0
		
		Small_gyro_init(&Small_Gyro, 500 ,      8  ,       2  ,      4    ,    2  ,    0.1, 0.5);
															//timemax,distance_max,near_max,count_max,quit_value, Q,   R
    //二阶自瞄滤波
    kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
    kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);

    //一阶自瞄滤波
    KalmanCreate(&follow_kalman_yaw,   1, 100);
    KalmanCreate(&follow_kalman_pitch, 1, 100);
	

}


void DMGimbalFun(void const *argument)
{
    
	GIMBAL_InitArgument();
    while(1)
    {
			if(Gimbal_DM_MotorPitch.motor_value->hall ==0)
						Motor_enable();					//电机使能
				
			control_judge_flag++;//在can2接收到底盘dbus按键函数get_dkey_info中清零
			if(control_judge_flag > 3000)
				control_judge_flag = 3000;
			if(control_judge_flag >= 2500){
				ControlMode = UNUSUAL;
			}
			else{
				if(ControlMode == UNUSUAL) 
					ControlMode = KEYBOARD;
			}

			if(imu_init_finish_flag)
			{
					switch(ControlMode)
					{
						case KEYBOARD:
						{
								if(SystemValue == Starting)
								{
										SystemValue = Running;      //系统初始化结束
								}
								else
								{
										GIMBAL_Mode_Choose();
										GIMBAL_Key_Ctrl();
										GIMBAL_Double_Loop_Out();
								}
								break;
						}
						case REMOTE:
						{
								if(SystemValue == Starting)
								{
										SystemValue = Running;      //系统初始化结束
								}
								else
								{
									GIMBAL_Key_Ctrl();
									RemoteControlGimbal();
								}

								break;
						}
						case UNUSUAL:
							Cloud_Angle_Target[PITCH][GYRO] = 0;
							Gimbal_DM_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_DM_MotorPitch.Motor_PID_Position,
																																IMU_angle[1],
																																Cloud_Angle_Target[PITCH][GYRO]);
							Gimbal_DM_MotorPitch.motor_value->target_velocity = - Gimbal_DM_MotorPitch.Motor_PID_Position.out;
							Gimbal_DM_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_DM_MotorPitch.Motor_PID_Speed,
																																Gimbal_DM_MotorPitch.motor_value->velocity ,
																																Gimbal_DM_MotorPitch.motor_value->target_velocity);		

							MIT_CtrlMotor(&hcan1,0x103,  0,  0, 0, 0, Gimbal_DM_MotorPitch.Motor_PID_Speed.out);

							break;
						default:
								break;
					}
			if(actGimbal == GIMBAL_NORMAL)
            Cloud_Angle_Target[PITCH][GYRO] -= (float)rc.ch2 / (6 *1000);    //目标陀螺仪角度
        LimitValue_f(&Cloud_Angle_Target[PITCH][GYRO], 15, -35);    
			
			Gimbal_DM_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_DM_MotorPitch.Motor_PID_Position,
																												IMU_angle[1],
																												Cloud_Angle_Target[PITCH][GYRO]);
			Gimbal_DM_MotorPitch.motor_value->target_velocity = - Gimbal_DM_MotorPitch.Motor_PID_Position.out;
			Gimbal_DM_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_DM_MotorPitch.Motor_PID_Speed,
																												Gimbal_DM_MotorPitch.motor_value->velocity ,
																												Gimbal_DM_MotorPitch.motor_value->target_velocity);		

			MIT_CtrlMotor(&hcan1,0x103,  0,  0, 0, 0, Gimbal_DM_MotorPitch.Motor_PID_Speed.out);
			osDelay(2);
		}
	} 
}


/**
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention
  */
uint16_t  test_flag=0;

static void RemoteControlGimbal(void)
{

        //目标值累加
			if(actGimbal == GIMBAL_NORMAL)
            Cloud_Angle_Target[PITCH][GYRO] -= (float)rc.ch2 / 2000;    //目标陀螺仪角度

        //板载陀螺仪PITCH轴位置环+电机速度环
//			LimitValue_f(&Cloud_Angle_Target[PITCH][GYRO], 15, -35);    
//			Gimbal_DM_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_DM_MotorPitch.Motor_PID_Position,
//																												IMU_angle[1],
//																												Cloud_Angle_Target[PITCH][GYRO]);
//			Gimbal_DM_MotorPitch.motor_value->target_velocity = - Gimbal_DM_MotorPitch.Motor_PID_Position.out;
//			Gimbal_DM_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_DM_MotorPitch.Motor_PID_Speed,
//																												Gimbal_DM_MotorPitch.motor_value->velocity ,
//																												Gimbal_DM_MotorPitch.motor_value->target_velocity);		

//			MIT_CtrlMotor(&hcan1,0x103,  0,  0, 0, 0, Gimbal_DM_MotorPitch.Motor_PID_Speed.out);
}
/**
  * @brief  云台电机输出
  * @param  void
  * @retval void
  * @attention
  */


static void GIMBAL_Double_Loop_Out()
{


//			Gimbal_DM_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_DM_MotorPitch.Motor_PID_Position,
//																												IMU_angle[1],
//																												Cloud_Angle_Target[PITCH][GYRO]);
//			Gimbal_DM_MotorPitch.motor_value->target_velocity = - Gimbal_DM_MotorPitch.Motor_PID_Position.out;
//			Gimbal_DM_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_DM_MotorPitch.Motor_PID_Speed,
//																												Gimbal_DM_MotorPitch.motor_value->velocity ,
//																												Gimbal_DM_MotorPitch.motor_value->target_velocity);		

//			MIT_CtrlMotor(&hcan1,0x103,  0,  0, 0, 0, Gimbal_DM_MotorPitch.Motor_PID_Speed.out);

}

/**
  * @brief  键盘控制云台模式
  * @param  void
  * @retval void
  * @attention
  */
portTickType ztime;
static void GIMBAL_Key_Ctrl()
{
    switch(actGimbal)
    {
    /*--------------云台模式选择----------------*/
    case GIMBAL_NORMAL:
    {
        ztime = 0;
        kalman_filter_change_realtime();
        if(ControlMode == KEYBOARD)
            GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl();
        break;
    }
    /*--------------右键自瞄----------------*/
    case GIMBAL_AUTO:
    {
      if(VisionValue.identify_target == '0')
      {  
          if(ControlMode == KEYBOARD)
              GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl();
      }
      else if(VisionValue.identify_target == '1')
      {
          ztime++;//:是普通自瞄
          
          GIMBAL_AUTO_Mode_Ctrl();//自瞄控制函数
      }
      break;
    }
    default:
        break;
    }
}

/**
  * @brief  卡尔曼滤波初始化，提前开始拟合
  * @param  void
  * @retval void
  * @attention void
  */
static void kalman_filter_change_realtime(void)
{
    kalman_filter_calc(&yaw_kalman_filter, IMU_angle[0], Gimbal_MotorYaw.motor_value->speed_rpm, 0.01);
    kalman_filter_calc(&pitch_kalman_filter, IMU_angle[1], Gimbal_MotorPitch.motor_value->speed_rpm, 0.01);
    KalmanFilter(&follow_kalman_yaw, VisionValue.vision_yaw_value.value);
    KalmanFilter(&follow_kalman_pitch, VisionValue.vision_pitch_value.value);
}
/**********************************************************************************/

/**
  * @brief  自瞄控制函数
  * @param  void
  * @retval void
  * @attention 中间坐标(0,0),左正右负,上负下正
  *            yaw为陀螺仪模式,pitch为机械模式(其实pitch全程都在用机械模式)
  *            只有当新数据来了才在当前实时角度上累加误差当成目标角度
  *            新数据一来,目标角度就实时更新
  */

static void GIMBAL_AUTO_Mode_Ctrl(void)
{
	static uint32_t last_vision_time = 0;
	static uint16_t stime = 0;
	
	if(isnan(VisionValue.vision_yaw_value.value)||isnan(VisionValue.vision_pitch_value.value)||
	isinf(VisionValue.vision_yaw_value.value)||isinf(VisionValue.vision_pitch_value.value))
	{
		memset(&VisionValue,0,sizeof(VisionValue)); 
		VISION_PITCH.Iout=0;
	}
	if(++stime > 80)
	{
		if(vision_time == last_vision_time)
		{
			memset(&VisionValue,0,sizeof(VisionValue)); 
			vision_time = 0;
		}
		stime = 0;
		last_vision_time = vision_time;
	}
		visionpitch=180.0f*VisionValue.vision_pitch_value.value/pi;
    VISION_PITCH.f_cal_pid(&VISION_PITCH, visionpitch, 0); 
		
	  Cloud_Angle_Target[PITCH][GYRO] -= VISION_PITCH.out;
}

/**
  * @brief  云台键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 云台键盘控制状态下的所有模式切换都在这
  * 无模式切换时一直处于此模式
  */
bool Gimbal_Switch_Z = 1;
static void GIMBAL_Mode_Choose(void)
{
    ////////////长按右键自瞄////////////////
    if(IF_MOUSE_PRESSED_RIGH)
    {
		if(!(ControlMode == UNUSUAL))
			actGimbal = GIMBAL_AUTO;
    }
	else
	{
		if(actGimbal == GIMBAL_AUTO)
			actGimbal = GIMBAL_NORMAL;
	}
	
    //////////////////ctrl键切换吊射模式：vb键位更改pitch还是yaw/////////////////////	
		if(!IF_KEY_PRESSED_CTRL)
		{
			Flag_status.Gimbal_Switch_Ctrl = 1;
		}
		if(IF_KEY_PRESSED_CTRL && Flag_status.Gimbal_Switch_Ctrl == 1)
		{
			Flag_status.Gimbal_Switch_Ctrl = 0;
			Flag_status.Gimbal_Key_Ctrl_Change ++;
			Flag_status.Gimbal_Key_Ctrl_Change %= 2;
			
			if(Flag_status.Gimbal_Key_Ctrl_Change == 0)
			{
				gimbal_hanging = GIMBAL_VERTICAL;
			
			}
			else
			{
				gimbal_hanging = GIMBAL_HORIZON;
			}
		}
		
    ////////////Z键软件复位////////////////
    if(!IF_KEY_PRESSED_Z)
    {
        Gimbal_Switch_Z = 1;
    }
    if(IF_KEY_PRESSED_Z && Gimbal_Switch_Z == 1)
    {
        Gimbal_Switch_Z = 0;
        Stop_All();//电机停转
        soft_rest(); //软件复位
    }
}

float PITCH_TEST = 0.006f;

/**
  * @brief  底盘跟随云台函数
  * @param  void
  * @retval void
  * @attention
  */
static void GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl()
{
    static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响?
    static float Mouse_Gyro_Pitch;  //Mouse_Gyro_Yaw, 键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快
    {
        if(MOUSE_Y_MOVE_SPEED != 0)
        {
           Mouse_Gyro_Pitch -= MOUSE_Y_MOVE_SPEED * -PITCH_TEST;//pitch仍旧使用机械模式
        }
        else if(MOUSE_Y_MOVE_SPEED == 0)
        {
            Mouse_Pitch_Stop ++ ;
            if(Mouse_Pitch_Stop > 5) //鼠标长时间停留，停止移动
            {
                Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 50);
            }
        }
        else
        {
            Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 50);
        }
        Cloud_Angle_Target[PITCH][GYRO] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][GYRO], 10);
				if(gimbal_hanging == GIMBAL_VERTICAL)
				{
					if(IF_KEY_PRESSED_V)
						Cloud_Angle_Target[PITCH][GYRO] -= 0.005f;
					if(IF_KEY_PRESSED_B)
						Cloud_Angle_Target[PITCH][GYRO] += 0.005f;		
				}


    }
}
