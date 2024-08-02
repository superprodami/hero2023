///**
//  ******************************************************************************
//  * @file    gimbal_task.c
//  * @brief   云台控制，包括遥控器和键盘两种控制模式。以及自瞄控制函数、云台（底盘）开机复位
//  *
//  ******************************************************************************
//  * @attention
//  *
//  * 2021.4 考虑到云台复位时只转云台较易受攻击，更改原来只转云台的云台复位方式为转底盘，云台不动
//  * 考虑到pid控制的因素，将复位时云台和底盘的动作分开，分别在各自的task中实现
//  *
//  ******************************************************************************
//  */
//#include "gimbal_task.h"
//#include "judge.h"
//#include "usbd_cdc_if.h"


///*************卡尔曼滤波**************/
///*二阶卡尔曼*/  //自瞄
////云台角度误差卡尔曼
//kalman_filter_init_t yaw_kalman_filter_para =
//{
//    .xhat_data = {0, 0},                                  //状态矩阵 位置（角度）、速度 (会被快速迭代，不是很重要)
//    .P_data = {2, 0, 0, 2},                               //预测状态协方差矩阵(会被快速迭代，不是很重要)
//    .A_data = {1, 0.002, 0, 1},//采样时间间隔             //状态转移矩阵
//    .B_data = {0.002 * 0.002 * 0.5, 0.002},               //控制矩阵
//    .Q_data = {1, 0, 0, 1},                               //状态转移矩阵的协方差矩阵 “预测模型”本身的噪声
//    .H_data = {1, 0, 0, 1},                               //观测矩阵
//    .R_data = {200, 0, 0, 400}//500 1000                  //观测矩阵的协方差(噪声方差)
//};//初始化yaw的部分kalman参数

//kalman_filter_init_t pitch_kalman_filter_para =
//{
//    .xhat_data = {0, 0},
//    .P_data = {2, 0, 0, 2},
//    .A_data = {1, 0.002, 0, 1},//采样时间间隔
//    .B_data = {0.002 * 0.002 * 0.5, 0.002},
//    .Q_data = {1, 0, 0, 1},
//    .H_data = {1, 0, 0, 1},
//    .R_data = {200, 0, 0, 400}
//};//初始化pitch的部分kalman参数

//kalman_filter_t yaw_kalman_filter;
//kalman_filter_t pitch_kalman_filter;
//extKalman_t follow_kalman_yaw;
//extKalman_t follow_kalman_pitch;
//float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度
//PidTypeDef VISION_PITCH;
//#define pi 3.1415926536f
//fp32 visionpitch;
//void GimbalFun(void const *argument)
//{
//	
//    portTickType currentTime;

//    GIMBAL_InitArgument();
//    while(1)
//    {
//			currentTime = xTaskGetTickCount();//当前系统时间
//			control_judge_flag++;//在can2接收到底盘dbus按键函数get_dkey_info中清零
//			if(control_judge_flag > 3000)
//				control_judge_flag = 3000;
//			if(control_judge_flag >= 2500){
//				ControlMode = UNUSUAL;
//			}
//			else{
//				if(ControlMode == UNUSUAL) 
//					ControlMode = KEYBOARD;
//			}

//			if(imu_init_finish_flag)
//			{
//					switch(ControlMode)
//					{
//						case KEYBOARD:
//						{
//								if(SystemValue == Starting)
//								{
//										SystemValue = Running;      //系统初始化结束
//								}
//								else
//								{
//										GIMBAL_Mode_Choose();
//										GIMBAL_Key_Ctrl();
//										GIMBAL_Double_Loop_Out();
//								}
//								break;
//						}
//						case REMOTE:
//						{
//								if(SystemValue == Starting)
//								{
//										SystemValue = Running;      //系统初始化结束
//								}
//								else
//								{
//										GIMBAL_Key_Ctrl();
//									RemoteControlGimbal();
//								}

//								break;
//						}
//						case UNUSUAL:
//							Cloud_Angle_Target[PITCH][GYRO] = -10;
//							Gimbal_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Position, 
//						      IMU_angle[1], Cloud_Angle_Target[PITCH][GYRO]);

//							Gimbal_MotorPitch.motor_value->target_speed_rpm = - Gimbal_MotorPitch.Motor_PID_Position.out;      //位置环输出

//							Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//									Gimbal_MotorPitch.motor_value->speed_rpm,
//									Gimbal_MotorPitch.motor_value->target_speed_rpm);

//							set_moto5678_current(&hcan1, -Gimbal_MotorPitch.Motor_PID_Speed.out,0, 0, 0);  //加入CAN发送队列中

//							break;
//						default:
//								break;
//					}
//			}
//			vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);
//    }
//}


///**
//  * @brief  云台参数初始化
//  * @param  void
//  * @retval void
//  * @attention 没有加I会有误差,只在系统启动时调用一次
//  **/
//followStruct followData = {0};
//fp32 Gimbal_Pitch_Speed_pid[3] = {250,7,50};
//fp32 Gimbal_Pitch_imu_pid[3]   = {20,0,888};
//fp32 VISION_PITCH_PID[3] = {0.02,0.00001,0.02};

///**
//  * @brief  云台各参数初始化
//  * @param  void
//  * @retval void
//  * @attention void
//  */
//static void GIMBAL_InitArgument(void)
//{
//    PID_clear(&Gimbal_MotorPitch.Motor_PID_Position);
//    PID_clear(&Gimbal_MotorPitch.Motor_PID_Speed);

//    Flag_status.follow_flag_remote=1;
//    Flag_status.FLAG_Key=1;
//    Flag_status.follow_flag_key=1;

//    //Motortype*motor            ID
//    Motor_Init(&Gimbal_MotorPitch, 5,
//                //   float pid1[3],                  outmax1 imax1 I_Separation Dead_Zone  gama angle_max angle_min
//                Gimbal_Pitch_imu_pid, PID_POSITION,   1000,  300,  100,        0,        0.8,   360,      0,
//                //   float pid2[3],                  outmax2 imax2 I_Separation Dead_Zone  gama angle_max angle_min
//                Gimbal_Pitch_Speed_pid, PID_DELTA, 30000,  7000,  3e38,        0,        0.8, 0,        0);

//	pid_init(&VISION_PITCH);
//	VISION_PITCH.f_param_init(&VISION_PITCH, PID_DELTA, VISION_PITCH_PID, 45,20,20,0,0.75,0,0);// 45,45,2,0,0,0,0
//	
//	Small_gyro_init(&Small_Gyro, 500 ,      8  ,       2  ,      4    ,    2  ,    0.1, 0.5);
//															//timemax,distance_max,near_max,count_max,quit_value, Q,   R
//    //二阶自瞄滤波
//    kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
//    kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);

//    //一阶自瞄滤波
//    KalmanCreate(&follow_kalman_yaw,   1, 100);
//    KalmanCreate(&follow_kalman_pitch, 1, 100);
//	
//	Cloud_Angle_Target[PITCH][GYRO] = 0;
//}

///**
//  * @brief  遥控器控制方式
//  * @param  void
//  * @retval void
//  * @attention
//  */
//uint16_t  test_flag=0;

//static void RemoteControlGimbal(void)
//{

//    if(PitchGimbalMode == USEENCODER) //纯编码器调PITCH轴
//    {
//        Gimbal_MotorPitch.motor_value->target_angle += (float)rc.ch2 / SENSITIVITY_REMOTE_GIMBAL_PITCH; //目标编码器角度
//        LimitValue_f(&(Gimbal_MotorPitch.motor_value->target_angle), GIMBAL_PITCH_ENCODER_MAX, GIMBAL_PITCH_ENCODER_MIN);

//        Gimbal_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Position,
//                Gimbal_MotorPitch.motor_value->angle,
//                Gimbal_MotorPitch.motor_value->target_angle);

//        Gimbal_MotorPitch.motor_value->target_speed_rpm = -Gimbal_MotorPitch.Motor_PID_Position.out; //位置环

//        Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//                Gimbal_MotorPitch.motor_value->speed_rpm, //速度环
//                Gimbal_MotorPitch.motor_value->target_speed_rpm);
//    }
//    else if(PitchGimbalMode == USEIMU) //纯imu调PITCH轴
//    {
//        //目标值累加
//        if(actGimbal == GIMBAL_NORMAL)
//            Cloud_Angle_Target[PITCH][GYRO] -= (float)rc.ch2 / 2000;    //目标陀螺仪角度

//        //板载陀螺仪PITCH轴位置环+电机速度环
//        LimitValue_f(&Cloud_Angle_Target[PITCH][GYRO], 15, -40);    //15  -40
//        Gimbal_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Position, IMU_angle[1], Cloud_Angle_Target[PITCH][GYRO]);

//        Gimbal_MotorPitch.motor_value->target_speed_rpm = Gimbal_MotorPitch.Motor_PID_Position.out;      //位置环输出

//        Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//                Gimbal_MotorPitch.motor_value->speed_rpm,
//                Gimbal_MotorPitch.motor_value->target_speed_rpm);
//    }

//    set_moto5678_current(&hcan1, Gimbal_MotorPitch.Motor_PID_Speed.out,0, 0, 0);  //加入CAN发送队列中
//}
///**
//  * @brief  云台电机输出
//  * @param  void
//  * @retval void
//  * @attention
//  */


//static void GIMBAL_Double_Loop_Out()
//{

//    if(PitchGimbalMode == USEIMU)
//    {
//        LimitValue_f(&Cloud_Angle_Target[PITCH][GYRO], 15, -40);
//		
//		
//        Gimbal_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Position, 
//								IMU_angle[1], 
//								Cloud_Angle_Target[PITCH][GYRO]);
//        
//				Gimbal_MotorPitch.motor_value->target_speed_rpm = -Gimbal_MotorPitch.Motor_PID_Position.out ; //位置环
//	
//			
//        Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//                Gimbal_MotorPitch.motor_value->speed_rpm, //速度环
//                Gimbal_MotorPitch.motor_value->target_speed_rpm);
//    }
//    else if(PitchGimbalMode == USEENCODER)
//    {
//        Gimbal_MotorPitch.motor_value->target_angle = Cloud_Angle_Target[PITCH][MECH];

//        LimitValue_f(&(Gimbal_MotorPitch.motor_value->target_angle), GIMBAL_PITCH_ENCODER_MAX, GIMBAL_PITCH_ENCODER_MIN);

//        Gimbal_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Position,
//                Gimbal_MotorPitch.motor_value->angle,
//                Gimbal_MotorPitch.motor_value->target_angle);

//        Gimbal_MotorPitch.motor_value->target_speed_rpm = -Gimbal_MotorPitch.Motor_PID_Position.out; //位置环

//        Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//                Gimbal_MotorPitch.motor_value->speed_rpm, //速度环
//                Gimbal_MotorPitch.motor_value->target_speed_rpm);
//    }
//    set_moto5678_current(&hcan1, Gimbal_MotorPitch.Motor_PID_Speed.out,0, 0, 0);  //加入CAN发送队列中
//    //    set_moto5678_current(&hcan1, 0, 0, 0, 0);

//}

///**
//  * @brief  键盘控制云台模式
//  * @param  void
//  * @retval void
//  * @attention
//  */
//portTickType ztime;
//static void GIMBAL_Key_Ctrl()
//{
//    switch(actGimbal)
//    {
//    /*--------------云台模式选择----------------*/
//    case GIMBAL_NORMAL:
//    {
//        ztime = 0;
//        kalman_filter_change_realtime();
//        if(ControlMode == KEYBOARD)
//            GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl();
//        break;
//    }
//    /*--------------右键自瞄----------------*/
//    case GIMBAL_AUTO:
//    {
//      if(VisionValue.identify_target == '0')
//      {  
//          if(ControlMode == KEYBOARD)
//              GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl();
//      }
//      else if(VisionValue.identify_target == '1')
//      {
//          ztime++;//:是普通自瞄
//          
//          GIMBAL_AUTO_Mode_Ctrl();//自瞄控制函数
//      }
//      break;
//    }
//    default:
//        break;
//    }
//}

///**
//  * @brief  卡尔曼滤波初始化，提前开始拟合
//  * @param  void
//  * @retval void
//  * @attention void
//  */
//static void kalman_filter_change_realtime(void)
//{
//    kalman_filter_calc(&yaw_kalman_filter, IMU_angle[0], Gimbal_MotorYaw.motor_value->speed_rpm, 0.01);
//    kalman_filter_calc(&pitch_kalman_filter, IMU_angle[1], Gimbal_MotorPitch.motor_value->speed_rpm, 0.01);
//    KalmanFilter(&follow_kalman_yaw, VisionValue.vision_yaw_value.value);
//    KalmanFilter(&follow_kalman_pitch, VisionValue.vision_pitch_value.value);
//}
///**********************************************************************************/

///**
//  * @brief  自瞄控制函数
//  * @param  void
//  * @retval void
//  * @attention 中间坐标(0,0),左正右负,上负下正
//  *            yaw为陀螺仪模式,pitch为机械模式(其实pitch全程都在用机械模式)
//  *            只有当新数据来了才在当前实时角度上累加误差当成目标角度
//  *            新数据一来,目标角度就实时更新
//  */

//static void GIMBAL_AUTO_Mode_Ctrl(void)
//{
//	static uint32_t last_vision_time = 0;
//	static uint16_t stime = 0;
//	
//	if(isnan(VisionValue.vision_yaw_value.value)||isnan(VisionValue.vision_pitch_value.value)||
//	isinf(VisionValue.vision_yaw_value.value)||isinf(VisionValue.vision_pitch_value.value))
//	{
//		memset(&VisionValue,0,sizeof(VisionValue)); 
//		VISION_PITCH.Iout=0;
//	}
//	if(++stime > 80)
//	{
//		if(vision_time == last_vision_time)
//		{
//			memset(&VisionValue,0,sizeof(VisionValue)); 
//			vision_time = 0;
//		}
//		stime = 0;
//		last_vision_time = vision_time;
//	}
//		visionpitch=180.0f*VisionValue.vision_pitch_value.value/pi;
//    VISION_PITCH.f_cal_pid(&VISION_PITCH, visionpitch, 0); 
//		
//	  Cloud_Angle_Target[PITCH][GYRO] -= VISION_PITCH.out;

////	Cloud_Angle_Target[PITCH][GYRO]= (VisionValue.vision_pitch_value.value);

//}


///***********************************************************************************下面的部分是键盘模式**********************************************************************************/
///**
//  * @brief  机械 陀螺仪模式切换函数
//  * @param  void
//  * @retval void
//  * @attention 云台键盘控制状态下的所有模式切换都在这
//  * 无模式切换时一直处于此模式
//  */
////void GIMBAL_State_Change(GimbalModeType Type)
////{
////    if(Type == USEIMU)
////    {
////        if(YawGimbalMode == USEENCODER) //YAW轴必须采用陀螺仪模式的情况
////        {
////            Cloud_Angle_Target[YAW][GYRO] = IMU_angle[0];  //避免模式切换时云台乱转
////            Cloud_Angle_Target[PITCH][GYRO] = IMU_angle[1];
////            YawGimbalMode = USEIMU;
////        }
////    }
////    else if(Type == USEENCODER)
////    {
////        if(YawGimbalMode == USEIMU)
////        {
////            Cloud_Angle_Target[YAW][MECH]   = Gimbal_MotorYaw.motor_value  -> main_angle;   //得到当前角度//
////            Cloud_Angle_Target[PITCH][MECH] = Gimbal_MotorPitch.motor_value-> angle;
////            YawGimbalMode = USEENCODER;
////        }
////    }
////}

////void GIMBAL_State_Change_key(void)
////{
////    if(YawGimbalMode == USEENCODER && ((actGimbal == GIMBAL_AUTO) || (actGimbal == GIMBAL_NORMAL) ||
////                                       (actGimbal == GIMBAL_GYROSCOPE) || (actGimbal == GIMBAL_CHASSIS_FOLLOW))) //YAW轴必须采用陀螺仪模式的情况
////    {
////        Cloud_Angle_Target[YAW][GYRO] = IMU_angle[0];	//避免模式切换时云台乱转
////        YawGimbalMode = USEIMU;
////    }
////    else if(YawGimbalMode == USEIMU && ((actGimbal != GIMBAL_AUTO) && (actGimbal != GIMBAL_GYROSCOPE)
////                                        && (actGimbal != GIMBAL_NORMAL)))
////    {
////        Cloud_Angle_Target[YAW][MECH]   = Gimbal_MotorYaw.motor_value  -> main_angle;   //得到当前角度//
////        Cloud_Angle_Target[PITCH][MECH] = Gimbal_MotorPitch.motor_value-> angle;
////        YawGimbalMode = USEENCODER;
////    }
////}
///**
//  * @brief  云台键盘模式选择,按键响应
//  * @param  void
//  * @retval void
//  * @attention 云台键盘控制状态下的所有模式切换都在这
//  * 无模式切换时一直处于此模式
//  */
//bool Gimbal_Switch_Z = 1;
//static void GIMBAL_Mode_Choose(void)
//{
//    ////////////长按右键自瞄////////////////
//    if(IF_MOUSE_PRESSED_RIGH)
//    {
//		if(!(ControlMode == UNUSUAL))
//			actGimbal = GIMBAL_AUTO;
//    }
//	else
//	{
//		if(actGimbal == GIMBAL_AUTO)
//			actGimbal = GIMBAL_NORMAL;
//	}
//	
//    //////////////////G键切换自瞄模式：普通模式/吊射模式/////////////////////	
////	if(!IF_KEY_PRESSED_G)
////	{
////		Flag_status.Chassis_Switch_G = 1;
////	}
////	if(IF_KEY_PRESSED_G && Flag_status.Chassis_Switch_G == 1)
////	{
////		Flag_status.Chassis_Switch_G = 0;
////		Flag_status.Chassis_Key_G_Change ++;
////		Flag_status.Chassis_Key_G_Change %= 2;
////		
////		if(Flag_status.Chassis_Key_G_Change)
////		{
////			vision_mode = aHANGING;
////		
////		}else{
////			vision_mode = aNORMAL;
////		}
////	}
//	
//    //////////////////ctrl键切换吊射模式：vb键位更改pitch还是yaw/////////////////////	
//		if(!IF_KEY_PRESSED_CTRL)
//		{
//			Flag_status.Gimbal_Switch_Ctrl = 1;
//		}
//		if(IF_KEY_PRESSED_CTRL && Flag_status.Gimbal_Switch_Ctrl == 1)
//		{
//			Flag_status.Gimbal_Switch_Ctrl = 0;
//			Flag_status.Gimbal_Key_Ctrl_Change ++;
//			Flag_status.Gimbal_Key_Ctrl_Change %= 2;
//			
//			if(Flag_status.Gimbal_Key_Ctrl_Change == 0)
//			{
//				gimbal_hanging = GIMBAL_VERTICAL;
//			
//			}
//			else
//			{
//				gimbal_hanging = GIMBAL_HORIZON;
//			}
//		}
//		
//    ////////////Z键软件复位////////////////
//    if(!IF_KEY_PRESSED_Z)
//    {
//        Gimbal_Switch_Z = 1;
//    }
//    if(IF_KEY_PRESSED_Z && Gimbal_Switch_Z == 1)
//    {
//        Gimbal_Switch_Z = 0;
//        Stop_All();//电机停转
//        soft_rest(); //软件复位
//    }
//}

///**
//  * @brief  底盘跟随云台函数
//  * @param  void
//  * @retval void
//  * @attention
//  */
//static void GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl()
//{
////    static portTickType  Key_Ctrl_CurrentTime = 0;
////    static TickType_t Turn_Time[2] = {0};
//    static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响?

////    static uint32_t QTurn90_ms  = 0;//90°,250ms延时响应,1秒最多按4下
////    static uint32_t ETurn90_ms  = 0;//90°,250ms延时响应,1秒最多按4下
////    static u8 Qturn90_f = 0;
////    static u8 Etrun90_f = 0;
//    static float Mouse_Gyro_Pitch;  //Mouse_Gyro_Yaw, 键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快

////    Key_Ctrl_CurrentTime = xTaskGetTickCount();//获取实时时间,用来做按键延时判断
//    //左右旋转时云台补偿
//    //底盘跟随云台时转90度部分
////    if(actChassis == CHASSIS_FOLLOW_GIMBAL && IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > QTurn90_ms)
////    {
////        Turn_Time[NOW] = xTaskGetTickCount();//当前系统时间

////        QTurn90_ms = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms延时防手贱狂按
////        Qturn90_f = 1;
////    }
////    else if(actChassis == CHASSIS_FOLLOW_GIMBAL && IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > ETurn90_ms)
////    {
////        Turn_Time[NOW] = xTaskGetTickCount();//当前系统时间
////        ETurn90_ms = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms延时防手贱狂按
////        Etrun90_f = 1;
////    }

////    if(Key_Ctrl_CurrentTime - Turn_Time[NOW]  > TIME_STAMP_500MS)
////    {
////        Etrun90_f = 0;
////        Qturn90_f = 0;
////    }
////    else if(Qturn90_f)
////    {
////        Cloud_Angle_Target[PITCH][MECH] = GIMBAL_PITCH_ENCODER_MIDDLE;
////    }
////    else if(Etrun90_f)
////    {
////        Cloud_Angle_Target[PITCH][MECH] = GIMBAL_PITCH_ENCODER_MIDDLE;
////    }
//    //普通控制部分
////    if(Etrun90_f == 0 && Qturn90_f == 0)
//    {
//        if(MOUSE_Y_MOVE_SPEED != 0)
//        {
//            Mouse_Gyro_Pitch -= MOUSE_Y_MOVE_SPEED * -0.01f;//pitch仍旧使用机械模式
//        }
//        else if(MOUSE_Y_MOVE_SPEED == 0)
//        {
//            Mouse_Pitch_Stop ++ ;
//            if(Mouse_Pitch_Stop > 5) //鼠标长时间停留，停止移动
//            {
//                Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 50);
//            }
//        }
//        else
//        {
//            Mouse_Gyro_Pitch = RAMP_float(0, Mouse_Gyro_Pitch, 50);
//        }
//        Cloud_Angle_Target[PITCH][GYRO] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][GYRO], 10);
//				if(gimbal_hanging == GIMBAL_VERTICAL)
//				{
//					if(IF_KEY_PRESSED_V)
//						Cloud_Angle_Target[PITCH][GYRO] -= 0.005f;
//					if(IF_KEY_PRESSED_B)
//						Cloud_Angle_Target[PITCH][GYRO] += 0.005f;		
//				}
//				if(IF_KEY_PRESSED_G)
//				{
//					Cloud_Angle_Target[PITCH][GYRO]=-35.00f;
//				}

//    }
//}
