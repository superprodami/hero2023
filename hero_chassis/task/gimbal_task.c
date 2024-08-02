/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @brief   云台控制，包括遥控器和键盘两种控制模式。以及自瞄控制函数、云台（底盘）开机复位
  *
  ******************************************************************************
  * @attention
  *
  * 2021.4 考虑到云台复位时只转云台较易受攻击，更改原来只转云台的云台复位方式为转底盘，云台不动
  * 考虑到pid控制的因素，将复位时云台和底盘的动作分开，分别在各自的task中实现
  *
  ******************************************************************************
  */
#include "gimbal_task.h"
#include "SQ_judge.h"
#include "kalman_filter.h"
#include "Motor_DM.h"
#include "math.h"
GimbalModeType YawGimbalMode   = USEIMU;
GimbalModeType PitchGimbalMode = USEIMU;
//followStruct followData = {0};
PidTypeDef VISION_YAW;

//float IMU_angle[3] = {0.0f, 0.0f, 0.0f};
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

/**
  * @brief  云台参数初始化
  * @param  void
  * @retval void
  * @attention 没有加I会有误差,只在系统启动时调用一次
  **/
//
FFC gimbal_yaw_ffc ={0,0,0,0,0};
FFC vision_yaw_ffc ={0,0,0,0,0};
float ffc_out;
//
float Cloud_Angle_Target[2][2];//  pitch/yaw    mech/gyro

fp32 Gimbal_Yaw_speed_pid[3]            = {300,5,0};//{350,5,70};//{300,5,0};//{200,5, 44};   //200 1.6  //100 0.8 0    {18,  0.5, 0}; //300,5, 0
fp32 Gimbal_Yaw_imu_pid[3]              = {10,0,300};//{15,0,350};//{10,0,300};//{33, 0, 555};  //IMU位置环pid //30 0.05 60  //30 0.2 0 {160, 0.8, 80}//10, 0, 280

fp32 Gimbal_Yaw_encoder_speed_pid[3]    = {300,10,0};//{12,0.45, 0.45};   
fp32 Gimbal_Yaw_encoder_pid[3]          = {0.5,0,15};//{0.45,0,12};//{227.6, 0, 6372.8};//编码器位置环pid//

//fp32 Gimbal_Yaw_imu_limit[7]            ={5000,   1000,   50,          0,        0.85,   360,      0};
//fp32 Gimbal_Yaw_encoder_limit[7]        ={5000,   1000,   50,          0,        0.85,   8192,      0};

fp32 Gimbal_Yaw_speed_start_pid[3]            = {300,5, 0};   //200 1.6  //100 0.8 0    {18,  0.5, 0}; 
fp32 Gimbal_Yaw_imu_start_pid[3]              = {8, 0, 100};  //IMU位置环pid //30 0.05 60  //30 0.2 0 {160, 0.8, 80}

fp32 Gimbal_Yaw_speed_autoaim_pid[3]            = {150,9, 10};   //200 1.6  //100 0.8 0    {18,  0.5, 0}; {310,10, 0}; {300,9, 55}; 
fp32 Gimbal_Yaw_imu_autoaim_pid[3]              = {3, 0, 100};  //IMU位置环pid //30 0.05 60  //30 0.2 0 {160, 0.8, 80} {3, 0, 200}; 
//const static fp32 Gimbal_Yaw_Encoder_Position_pid[3] = {18, 0, 400};//{10, 0, 0};    //encoder位置环pid

//fp32 gimbalnothing[3]                     = {0, 0, 0};
fp32 VISION_YAW_PID[3] = {0.012,0.00005,0.03};//{0.09,0.00002,0.3};//0.06 
#define pi 3.1415926536f
fp32 visionyaw;

/**
  * @brief  云台电机pid清零
  * @param  void
  * @retval void
  * @attention void
  */
static void gimbal_PID_clear(PidTypeDef *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->error[0] = gimbal_pid_clear->error[1] = gimbal_pid_clear->error[2] = 0.0f;
    gimbal_pid_clear->Dbuf[0] = gimbal_pid_clear->Dbuf[1] = gimbal_pid_clear->Dbuf[2] = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

static void FFC_clear(FFC *gimbal_ffc_clear)
{
    if (gimbal_ffc_clear == NULL)
    {
        return;
    }
    gimbal_ffc_clear->lastRin = 0.0f;
    gimbal_ffc_clear->perrRin= 0.0f;
    gimbal_ffc_clear->rin= 0.0f;
}



void GimbalFun(void const * argument)
{
    portTickType currentTime;

    GIMBAL_InitArgument();
	  static uint8_t unusual_flag = 0;
	
    while(1)
    {
			testfbd =Gimbal_MotorEncoderYaw.motor_value->speed_rpm;
			testset =Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm;
//			see=IMU_angle[0];
//			set=Cloud_Angle_Target[YAW][GYRO];
			set=Gimbal_MotorEncoderYaw.motor_value->target_angle;
			  see=Gimbal_MotorEncoderYaw.motor_value->angle;
        currentTime = xTaskGetTickCount();//当前系统时间
				control_judge_flag++;
				if(control_judge_flag > 3000) control_judge_flag = 3000;
				
				if(control_judge_flag >= 2500)
				{
					ControlMode = UNUSUAL;
				}else
				{
					if(unusual_flag == 1)
					{	
						ControlMode = KEYBOARD;
//						GIMBAL_State_Change(USEENCODER);
						GIMBAL_State_Change(USEIMU);

						Gimbal_MotorImuYaw.Motor_PID_Position.angle_max = 360;
					}
					unusual_flag = 0;
				}

        if(imu_init_finish_flag || ControlMode == UNUSUAL)
        {

            switch(ControlMode)
            {
							case KEYBOARD:
							{
									if(SystemValue == Starting)
									{
										Gimbal_Open_Init();  
	//									SystemValue = Running;
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
										PID_clear(&Gimbal_MotorImuYaw.Motor_PID_Position);
										PID_clear(&Gimbal_MotorImuYaw.Motor_PID_Speed);
					//					Gimbal_Open_Init();           //刚开启，将云台缓慢的移动到正向
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
								if(!unusual_flag)
								{
									GIMBAL_State_Change(USEENCODER);
									actChassis = CHASSIS_NORMAL;
									Gimbal_MotorEncoderYaw.Motor_PID_Position.angle_max = 8192;
									unusual_flag = 1;
								}
								GIMBAL_Mode_Choose();
								GIMBAL_Key_Ctrl();
								GIMBAL_Double_Loop_Out();
								break;
							default:
									break;
            }
        }
        vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);
    }
}




/**
  * @brief  云台各参数初始化
  * @param  void
  * @retval void
  * @attention void
  */
static void GIMBAL_InitArgument(void)
{

    PID_clear(&Gimbal_MotorImuYaw.Motor_PID_Position);
    PID_clear(&Gimbal_MotorImuYaw.Motor_PID_Speed);
	
	  PID_clear(&Gimbal_MotorEncoderYaw.Motor_PID_Position);
    PID_clear(&Gimbal_MotorEncoderYaw.Motor_PID_Speed);
	
    Flag_status.FLAG_Remote=1;
    Flag_status.follow_flag_remote=1;
    Flag_status.FLAG_Key=1;
    Flag_status.follow_flag_key=1;

    //Motortype*motor            ID
    Motor_Init(&Gimbal_MotorImuYaw, 5,
               //   float pid1[3],                outmax1 imax1 I_Separation Dead_Zone  gama angle_max angle_min
               Gimbal_Yaw_imu_pid, PID_POSITION,   5000,   1000,   50,          0,        0.85,   360,      0,
               //   float pid2[3],                outmax2 imax2 I_Separation Dead_Zone  gama angle_max angle_min
               Gimbal_Yaw_speed_pid, PID_DELTA,    30000,   8000,   2000,        0,         0, 0,        0);
	  
	 Motor_Init(&Gimbal_MotorEncoderYaw, 5,
               //   float pid1[3],                outmax1 imax1 I_Separation Dead_Zone  gama angle_max angle_min
               Gimbal_Yaw_encoder_pid, PID_POSITION,   50000,   10000,   50,          0,        0.85,   8192,      0,
               //   float pid2[3],                				outmax2  imax2 I_Separation Dead_Zone   gama angle_max angle_min
               Gimbal_Yaw_encoder_speed_pid, PID_DELTA,    30000,   8000,   2000,        0,         0,    0,        0);
		pid_init(&VISION_YAW);
		VISION_YAW.f_param_init(&VISION_YAW, PID_DELTA, VISION_YAW_PID, 40,20,20,0,0.75,0,0);

		Small_gyro_init(&Small_Gyro, 500 ,      8  ,       2  ,      4    ,    2  ,    0.1, 0.5);
															//timemax,distance_max,near_max,count_max,quit_value, Q,   R
    //二阶自瞄滤波
    kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);

    //一阶自瞄滤波
    KalmanCreate(&follow_kalman_yaw,   1, 100);
	
}

/**
  * @brief  遥控器控制方式
  * @param  void
  * @retval void
  * @attention
  */
static void RemoteControlGimbal(void)
{
	static uint32_t time;
    static uint32_t lasttime;
	
    if(rc.ch1 != 0)
        Flag_status.FLAG_Remote = 1;
    else                                            //如果静止
    {
        if(Flag_status.FLAG_Remote)                                     //如果是第一次
        {
            lasttime = xTaskGetTickCount();              //更新时间
            time = xTaskGetTickCount();
            Flag_status.FLAG_Remote = 0;
        }
        else
        {
            time = xTaskGetTickCount();
        }
    }
    if(YawGimbalMode == USEENCODER) //纯编码器调YAW轴
    {
        Gimbal_MotorEncoderYaw.motor_value->target_angle -= (float)rc.ch1 / SENSITIVITY_REMOTE_GIMBAL_YAW; //目标编码器角度
        AngleLoop_f(&(Gimbal_MotorEncoderYaw.motor_value->target_angle), 8192);

        Gimbal_MotorEncoderYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorEncoderYaw.Motor_PID_Position,
                Gimbal_MotorEncoderYaw.motor_value->angle,
                Gimbal_MotorEncoderYaw.motor_value->target_angle);

        Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm =  Gimbal_MotorEncoderYaw.Motor_PID_Position.out; //位置环

        Gimbal_MotorEncoderYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorEncoderYaw.Motor_PID_Speed,
                Gimbal_MotorEncoderYaw.motor_value->speed_rpm, //速度环
                Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm);
				set_moto5678_current(&hcan1, Gimbal_MotorEncoderYaw.Motor_PID_Speed.out, 0, 0, 0);
//				MIT_CtrlMotor(&hcan1,0x102,  0,  0, 0, 0, Ammunition_DM_Motor.Motor_PID_Speed.out);
    }
    else if(YawGimbalMode == USEIMU) //IMU调YAW轴
    {
        //目标值累加
        if(actGimbal == GIMBAL_NORMAL)
            Cloud_Angle_Target[YAW][GYRO] -= (float)rc.ch1 /2000;    //目标陀螺仪角度
		
		//零漂状态机 消除因陀螺仪漂移而导致的云台漂移
        if((actGimbal == GIMBAL_NORMAL) && rc.ch1 == 0&&rc.ch2 == 0&&rc.ch3 == 0&&rc.ch4 == 0 && actChassis != CHASSIS_GYROSCOPE && actGimbal != GIMBAL_AUTO) //如果不在自瞄模式  VisionValue.identify_target == '0' ||  && actChassis == CHASSIS_FOLLOW_GIMBAL 
        {
					if((time - lasttime) > 3000) //鼠标静止超过3秒 认为应当停止抖动
					{
						if(!Flag_status.chassis_follow_flag)
						{
							gimbal_PID_clear(&Gimbal_MotorEncoderYaw.Motor_PID_Position);
							Gimbal_MotorEncoderYaw.motor_value->target_angle = Gimbal_MotorEncoderYaw.motor_value->angle;
						}
						Gimbal_MotorEncoderYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorEncoderYaw.Motor_PID_Position, 
									Gimbal_MotorEncoderYaw.motor_value->angle, 
									Gimbal_MotorEncoderYaw.motor_value->target_angle);
						Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm = Gimbal_MotorEncoderYaw.Motor_PID_Position.out;
						
						Gimbal_MotorEncoderYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorEncoderYaw.Motor_PID_Speed,
                Gimbal_MotorEncoderYaw.motor_value->speed_rpm,
                Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm);
						set_moto5678_current(&hcan1, Gimbal_MotorEncoderYaw.Motor_PID_Speed.out, 0, 0, 0);
//						MIT_CtrlMotor(&hcan1,0x102,  0,  0, 0, 0, Ammunition_DM_Motor.Motor_PID_Speed.out);

						Flag_status.chassis_follow_flag = 1;
						Cloud_Angle_Target[YAW][GYRO] = IMU_angle[0];
					}else{
						Flag_status.chassis_follow_flag = 0;
						AngleLoop_f(&Cloud_Angle_Target[YAW][GYRO], 360);
						Gimbal_MotorImuYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Position, IMU_angle[0], Cloud_Angle_Target[YAW][GYRO]);
						Gimbal_MotorImuYaw.motor_value->target_speed_rpm = Gimbal_MotorImuYaw.Motor_PID_Position.out;
						
						Gimbal_MotorImuYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Speed,
                Gimbal_MotorImuYaw.motor_value->speed_rpm,
                Gimbal_MotorImuYaw.motor_value->target_speed_rpm);
						set_moto5678_current(&hcan1, Gimbal_MotorImuYaw.Motor_PID_Speed.out, 0, 0, 0);

					}
        }
				else
				{
					Flag_status.chassis_follow_flag = 0;
					AngleLoop_f(&Cloud_Angle_Target[YAW][GYRO], 360);
					Gimbal_MotorImuYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Position, IMU_angle[0], Cloud_Angle_Target[YAW][GYRO]);
					Gimbal_MotorImuYaw.motor_value->target_speed_rpm = Gimbal_MotorImuYaw.Motor_PID_Position.out;
					
					Gimbal_MotorImuYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Speed,
                Gimbal_MotorImuYaw.motor_value->speed_rpm,
                Gimbal_MotorImuYaw.motor_value->target_speed_rpm);
					set_moto5678_current(&hcan1, Gimbal_MotorImuYaw.Motor_PID_Speed.out, 0, 0, 0);

				}
    }
	
}
/**
  * @brief  云台电机输出
  * @param  void
  * @retval void
  * @attention
  */
int aaa0 = 200;
int16_t current_add;
uint16_t add_value=1500;
static void GIMBAL_Double_Loop_Out()
{
	static uint32_t time;
	static uint32_t lasttime;

	if(MOUSE_X_MOVE_SPEED != 0){
		
		Flag_status.FLAG_Key = 1;
		Flag_status.chassis_follow_flag = 0;
		lasttime = time;
	}else{  //如果静止
	
		if(Flag_status.FLAG_Key){  //如果是第一次
			
			lasttime = xTaskGetTickCount();              //更新时间
			time = xTaskGetTickCount();
			Flag_status.FLAG_Key = 0;
		}else{
			
			time = xTaskGetTickCount();
		}
	}
	
    if(YawGimbalMode == USEIMU )
    {
		//零漂状态机 消除因陀螺仪漂移而导致的云台漂移--normal和follow模式都要解决
        if((actGimbal == GIMBAL_NORMAL) && MOUSE_X_MOVE_SPEED == 0 && actChassis != CHASSIS_GYROSCOPE  && actGimbal != GIMBAL_AUTO) 
        {
					if(IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D || IF_KEY_PRESSED_X)
					{
						//更新lasttime
						lasttime = time;
					}
					if((time - lasttime) > 3000) //鼠标静止超过3秒 认为应当停止抖动
					{
						if(!Flag_status.chassis_follow_flag)
						{
							gimbal_PID_clear(&Gimbal_MotorEncoderYaw.Motor_PID_Position);
							Gimbal_MotorEncoderYaw.motor_value->target_angle = Gimbal_MotorEncoderYaw.motor_value->angle;
							PID_Change(Gimbal_Yaw_imu_start_pid,Gimbal_Yaw_speed_start_pid);
						}
						Flag_status.chassis_follow_flag = 1;
						
						if(gimbal_hanging == GIMBAL_HORIZON)
						{
							PID_Change(Gimbal_Yaw_imu_start_pid,Gimbal_Yaw_speed_start_pid);
							if(IF_KEY_PRESSED_V)
								Gimbal_MotorEncoderYaw.motor_value->target_angle += 0.05f;
							if(IF_KEY_PRESSED_B)
								Gimbal_MotorEncoderYaw.motor_value->target_angle -= 0.05f;				
						}			

						Gimbal_MotorEncoderYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorEncoderYaw.Motor_PID_Position, 
						    Gimbal_MotorEncoderYaw.motor_value->angle, Gimbal_MotorEncoderYaw.motor_value->target_angle);
						Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm = Gimbal_MotorEncoderYaw.Motor_PID_Position.out;
						Gimbal_MotorEncoderYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorEncoderYaw.Motor_PID_Speed,
						Gimbal_MotorEncoderYaw.motor_value->speed_rpm,
						Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm);
						set_moto5678_current(&hcan1, Gimbal_MotorEncoderYaw.Motor_PID_Speed.out+current_add, 0, 0, 0);
//						MIT_CtrlMotor(&hcan1,0x102,  0,  0, 0, 0, Ammunition_DM_Motor.Motor_PID_Speed.out);
						Cloud_Angle_Target[YAW][GYRO] = IMU_angle[0];

					}else{	
						PID_Change(Gimbal_Yaw_imu_pid,Gimbal_Yaw_speed_pid);
						Flag_status.chassis_follow_flag = 0;
						AngleLoop_f(&Cloud_Angle_Target[YAW][GYRO], 360);
						Gimbal_MotorImuYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Position, IMU_angle[0], Cloud_Angle_Target[YAW][GYRO]);
						Gimbal_MotorImuYaw.motor_value->target_speed_rpm = Gimbal_MotorImuYaw.Motor_PID_Position.out;
						Gimbal_MotorImuYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Speed,
						Gimbal_MotorImuYaw.motor_value->speed_rpm,
						Gimbal_MotorImuYaw.motor_value->target_speed_rpm);
						set_moto5678_current(&hcan1, Gimbal_MotorImuYaw.Motor_PID_Speed.out+current_add, 0, 0, 0);
//						MIT_CtrlMotor(&hcan1,0x102,  0,  0, 0, 0, Ammunition_DM_Motor.Motor_PID_Speed.out);

					}
        }
			else
			{
				if(Flag_status.chassis_follow_flag)
					PID_Change(Gimbal_Yaw_imu_pid,Gimbal_Yaw_speed_pid);
				lasttime = time;
				Flag_status.chassis_follow_flag = 0;
				AngleLoop_f(&Cloud_Angle_Target[YAW][GYRO], 360);
				Gimbal_MotorImuYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Position, IMU_angle[0], Cloud_Angle_Target[YAW][GYRO]);
				Gimbal_MotorImuYaw.motor_value->target_speed_rpm = Gimbal_MotorImuYaw.Motor_PID_Position.out;
				Gimbal_MotorImuYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Speed,
						Gimbal_MotorImuYaw.motor_value->speed_rpm,
						Gimbal_MotorImuYaw.motor_value->target_speed_rpm);
			  set_moto5678_current(&hcan1, Gimbal_MotorImuYaw.Motor_PID_Speed.out+current_add, 0, 0, 0);
//				MIT_CtrlMotor(&hcan1,0x102,  0,  0, 0, 0, Ammunition_DM_Motor.Motor_PID_Speed.out);

			}
			
			
			if(actChassis == CHASSIS_GYROSCOPE) Flag_status.follow_flag_key = 1; //小陀螺模式单独置follow_flag
//				Gimbal_MotorImuYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Speed,
//						Gimbal_MotorImuYaw.motor_value->speed_rpm,
//						Gimbal_MotorImuYaw.motor_value->target_speed_rpm);
    }
    else if(YawGimbalMode == USEENCODER)
    {
        Gimbal_MotorEncoderYaw.motor_value->target_angle = Cloud_Angle_Target[YAW][MECH];

        AngleLoop_f(&(Gimbal_MotorEncoderYaw.motor_value->target_angle), 8192);

        Gimbal_MotorEncoderYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorEncoderYaw.Motor_PID_Position,
                (Gimbal_MotorEncoderYaw.motor_value->angle),
                (Gimbal_MotorEncoderYaw.motor_value->target_angle));

        Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm = Gimbal_MotorEncoderYaw.Motor_PID_Position.out; //位置环

        Gimbal_MotorEncoderYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorEncoderYaw.Motor_PID_Speed,
                Gimbal_MotorEncoderYaw.motor_value->speed_rpm, //速度环
                Gimbal_MotorEncoderYaw.motor_value->target_speed_rpm);
				set_moto5678_current(&hcan1, Gimbal_MotorEncoderYaw.Motor_PID_Speed.out+current_add,0, 0, 0);
//				MIT_CtrlMotor(&hcan1,0x102,  0,  0, 0, 0, Ammunition_DM_Motor.Motor_PID_Speed.out);

    }

	if(IF_MOUSE_PRESSED_RIGH && VisionValue.center_flag)
	{
		current_add = Gimbal_MotorImuYaw.Motor_PID_Speed.out/fabs(Gimbal_MotorImuYaw.Motor_PID_Speed.out)*add_value;
		if(fabs(Gimbal_MotorImuYaw.Motor_PID_Speed.out+current_add)>30000)
			current_add=0;
	}
	else 
		current_add = 0;
//	set_moto5678_current(&hcan1, Gimbal_MotorImuYaw.Motor_PID_Speed.out+current_add, Ammunition_Motor.Motor_PID_Speed.out, 0, 0);
//	set_moto5678_current(&hcan1, 0, 0, 0, 0);

}

/**
  * @brief  键盘控制云台模式
  * @param  void
  * @retval void
  * @attention
  */
//uint8_t auto_mode=0;			//发送给视觉的字符
//float cs_yaw_after=0;  		//均值滤波后的值  
float cs_yaw_total=0;  		//均值滤波总和
float yaw_smooth[10]={0};	//均值滤波数组
float yaw_smooth2[10]={0};	//均值滤波数组
//int16_t smoothf=0;       	//均值滤波移位标志量

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
        if(ControlMode == KEYBOARD  || ControlMode == UNUSUAL )
            GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl();
        break;
    }
    /*--------------右键自瞄----------------*/
		case GIMBAL_AUTO:
    {
				if(VisionValue.identify_target == '0')
				{			
					PID_clear(&VISION_YAW);
					
					if(ControlMode == KEYBOARD)
						GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl();
				}
				else if(VisionValue.identify_target == '1')
				{     
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
    kalman_filter_calc(&yaw_kalman_filter, IMU_angle[0], Gimbal_MotorImuYaw.motor_value->speed_rpm, 0.01);
    KalmanFilter(&follow_kalman_yaw, VisionValue.vision_yaw_value.value);
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
float yaw_value;
float yaw_follow_kp = -0.007;
float yaw_speed0 = 0;
static void GIMBAL_AUTO_Mode_Ctrl(void)
{
	static uint32_t last_vision_time = 0;
	static uint16_t stime = 0;
	
	if(isnan(VisionValue.vision_yaw_value.value)||isnan(VisionValue.vision_pitch_value.value)||
		isinf(VisionValue.vision_yaw_value.value)||isinf(VisionValue.vision_pitch_value.value))
	{
		memset(&VisionValue,0,sizeof(VisionValue)); 
		VISION_YAW.Iout=0;
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
	
	visionyaw=180.0f*VisionValue.vision_yaw_value.value/pi;
	VISION_YAW.f_cal_pid(&VISION_YAW, visionyaw, 0);//    AngleLoop_f(&yaw_target_angle, 360);
	yaw_speed0 = VisionValue.normal_yaw_value.value;

	Cloud_Angle_Target[YAW][GYRO] += VISION_YAW.out ;
	
}

/***********************************************************************************下面的部分是键盘模式**********************************************************************************/
/**
  * @brief  机械 陀螺仪模式切换函数
  * @param  void
  * @retval void
  * @attention 云台键盘控制状态下的所有模式切换都在这
  * 无模式切换时一直处于此模式
  */
void GIMBAL_State_Change(GimbalModeType Type)
{
    if(Type == USEIMU)
    {
        if(YawGimbalMode == USEENCODER) //YAW轴必须采用陀螺仪模式的情况
        {
            Cloud_Angle_Target[YAW][GYRO] = IMU_angle[0];  //避免模式切换时云台乱转
            Cloud_Angle_Target[PITCH][GYRO] = IMU_angle[1];
            YawGimbalMode = USEIMU;
        }
    }
    else if(Type == USEENCODER)
    {
        if(YawGimbalMode == USEIMU)
        {
            Cloud_Angle_Target[YAW][MECH]   = GIMBAL_YAW_ENCODER_MIDDLE1;//Gimbal_MotorImuYaw.motor_value  -> angle;   //得到当前角度//
            Cloud_Angle_Target[PITCH][MECH] = Gimbal_MotorPitch.motor_value-> angle;
            YawGimbalMode = USEENCODER;
        }
    }
}

/**
  * @brief  云台键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 云台键盘控制状态下的所有模式切换都在这
  * 无模式切换时一直处于此模式
  */
void PID_Change(fp32 *Kpid_Angle ,fp32 *Kpid_speed)
{
	pid_reset(&Gimbal_MotorImuYaw.Motor_PID_Position,Kpid_Angle);
	pid_reset(&Gimbal_MotorImuYaw.Motor_PID_Speed,Kpid_speed);
}

static void GIMBAL_Mode_Choose(void)
{
    ////////////长按右键自瞄////////////////
    if(IF_MOUSE_PRESSED_RIGH)
    {
			if(!(ControlMode == UNUSUAL))
			actGimbal = GIMBAL_AUTO;
//			PID_Change(Gimbal_Yaw_imu_autoaim_pid,Gimbal_Yaw_speed_autoaim_pid);
			Gimbal_MotorImuYaw.Motor_PID_Speed.mode=PID_POSITION;
    }
		else if(!IF_MOUSE_PRESSED_RIGH)
		{
			actGimbal = GIMBAL_NORMAL;
			PID_Change(Gimbal_Yaw_imu_pid,Gimbal_Yaw_speed_pid);
			Gimbal_MotorImuYaw.Motor_PID_Speed.mode=PID_DELTA;
			PID_clear(&VISION_YAW);
		}
	
    //////////////////G键切换自瞄模式：普通模式/吊射模式/////////////////////	
//		if(!IF_KEY_PRESSED_G)
//		{
//			Flag_status.Chassis_Switch_G = 1;
//		}
//		if(IF_KEY_PRESSED_G && Flag_status.Chassis_Switch_G == 1)
//		{
//			Flag_status.Chassis_Switch_G = 0;
//			Flag_status.Chassis_Key_G_Change ++;
//			Flag_status.Chassis_Key_G_Change %= 2;
//			
//			if(Flag_status.Chassis_Key_G_Change)
//			{
//				vision_mode = aHANGING;
//			
//			}else{
//				vision_mode = aNORMAL; 
//			}
//		}
	
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
}

/**
  * @brief  底盘跟随云台函数
  * @param  void
  * @retval void
  * @attention
  */
float TURN_180_Gimbal_Thistime = 0;
static void GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl()
{
	
	static portTickType  Key_Ctrl_CurrentTime = 0;
	static TickType_t Turn_Time[2] = {0};
	static uint32_t Mouse_Yaw_Stop  = 0;
	static uint32_t XTurn180_ms  = 0;//90°,250ms延时响应,1秒最多按4下
	static u8 Xturn180_f = 0;

  static float Mouse_Gyro_Yaw; //键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快
	static float Mouse_Gyro_Yaw_Mech;
	if(!(ControlMode == UNUSUAL))
	{
		Key_Ctrl_CurrentTime = xTaskGetTickCount();//获取实时时间,用来做按键延时判断

		if(actChassis == CHASSIS_FOLLOW_GIMBAL && IF_KEY_PRESSED_X && Key_Ctrl_CurrentTime > XTurn180_ms)
		{
			Turn_Time[NOW] = xTaskGetTickCount();//当前系统时间

			XTurn180_ms = Key_Ctrl_CurrentTime + TIME_STAMP_600MS;//600ms延时防手贱狂按
			TURN_180_Gimbal_Thistime = Cloud_Angle_Target[YAW][GYRO];
			TURN_180_Gimbal_Thistime -= 180;
			Xturn180_f = 1;
			actChassis = CHASSIS_NORMAL;
			Flag_status.chassis_follow_flag = 0;
		}

		if(Key_Ctrl_CurrentTime - Turn_Time[NOW]  > TIME_STAMP_600MS)
		{
			if(Xturn180_f == 1)
			{
				actChassis = CHASSIS_FOLLOW_GIMBAL;
				Xturn180_f = 0;
			}
			else{
				TURN_180_Gimbal_Thistime = Cloud_Angle_Target[YAW][GYRO];
			}
		}
		else if(Xturn180_f)
		{
			if((TURN_180_Gimbal_Thistime - Cloud_Angle_Target[YAW][GYRO]) <= -180){
				
				Cloud_Angle_Target[YAW][GYRO] = RAMP_float( TURN_180_Gimbal_Thistime, Cloud_Angle_Target[YAW][GYRO], -0.6 );
			}else{
				
				Cloud_Angle_Target[YAW][GYRO] = RAMP_float( TURN_180_Gimbal_Thistime, Cloud_Angle_Target[YAW][GYRO], 0.6 );
			}
		}
		
		/////普通控制部分
		if(Xturn180_f == 0)
		{
			if(MOUSE_X_MOVE_SPEED != 0)
			{
				Mouse_Gyro_Yaw -= MOUSE_X_MOVE_SPEED * 0.005f;//yaw仍旧使用机械模式
				
				Mouse_Gyro_Yaw_Mech -= MOUSE_X_MOVE_SPEED * 0.02f * 22.76f;		
			}
			else if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 5) //鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = RAMP_float(0, Mouse_Gyro_Yaw, 100);
					Mouse_Gyro_Yaw_Mech = RAMP_float(0, Mouse_Gyro_Yaw_Mech, 2000);
				}
			}
			else
			{
				Mouse_Gyro_Yaw = RAMP_float(0, Mouse_Gyro_Yaw, 100);
				Mouse_Gyro_Yaw_Mech = RAMP_float(0, Mouse_Gyro_Yaw_Mech, 500);
			}
			Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], 10);

			Cloud_Angle_Target[YAW][MECH] = RampInc_float( &Mouse_Gyro_Yaw_Mech, Cloud_Angle_Target[YAW][MECH], 200);
		}	
	}

}

/**
  * @brief  刚开机，使云台缓慢的移动到＋y平整位置
  * @param  void
  * @retval void
  * @attention
  */
static void Gimbal_Open_Init(void)
{
	portTickType  StartTime = xTaskGetTickCount();
	portTickType  SteadyTime = 0;
	portTickType  SteadyTimeUntil = 0;
	uint8_t IS_Steady = 0;
	uint8_t Steady_Cnt = 0;
	PID_Change(Gimbal_Yaw_imu_start_pid,Gimbal_Yaw_speed_start_pid);
	
	while(1)	//让电机缓慢的转到初始状态
	{
		SteadyTime = xTaskGetTickCount();
		
		Gimbal_MotorImuYaw.Motor_PID_Position.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Position,
		        (Gimbal_MotorImuYaw.motor_value->angle)/22.76f, 
		        GIMBAL_YAW_ENCODER_MIDDLE1/22.76f);

		Gimbal_MotorImuYaw.motor_value->target_speed_rpm = Gimbal_MotorImuYaw.Motor_PID_Position.out;

		Gimbal_MotorImuYaw.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorImuYaw.Motor_PID_Speed,
						Gimbal_MotorImuYaw.motor_value->speed_rpm,
						Gimbal_MotorImuYaw.motor_value->target_speed_rpm);
		
		LimitValue_f(&Gimbal_MotorImuYaw.Motor_PID_Speed.out,15000,-15000);
		
	  set_moto5678_current(&hcan1, Gimbal_MotorImuYaw.Motor_PID_Speed.out,0, 0, 0);
		if((SteadyTime > SteadyTimeUntil)) {
			SteadyTimeUntil = SteadyTime + 50;
			Steady_Cnt++;
		}
		if(abs(Gimbal_MotorImuYaw.motor_value->angle - GIMBAL_YAW_ENCODER_MIDDLE1) < 50 )
			IS_Steady++;
		if(IS_Steady >= 30 || Steady_Cnt >= 30)
		{
			set_moto5678_current(&hcan1, 0, 0, 0, 0);
			PID_clear(&Gimbal_MotorImuYaw.Motor_PID_Position);
			PID_clear(&Gimbal_MotorImuYaw.Motor_PID_Speed);
			Cloud_Angle_Target[YAW][GYRO] = IMU_angle[0];
			Gimbal_MotorEncoderYaw.motor_value->target_angle = Gimbal_MotorEncoderYaw.motor_value->angle;
			GIMBAL_InitArgument();
			SystemValue = Running;			//系统初始化结束
			break;
		}
        vTaskDelayUntil(&StartTime, TIME_STAMP_2MS);
	}
	
}
