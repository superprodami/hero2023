///**
//  ******************************************************************************
//  * @file    gimbal_task.c
//  * @brief   ��̨���ƣ�����ң�����ͼ������ֿ���ģʽ���Լ�������ƺ�������̨�����̣�������λ
//  *
//  ******************************************************************************
//  * @attention
//  *
//  * 2021.4 ���ǵ���̨��λʱֻת��̨�����ܹ���������ԭ��ֻת��̨����̨��λ��ʽΪת���̣���̨����
//  * ���ǵ�pid���Ƶ����أ�����λʱ��̨�͵��̵Ķ����ֿ����ֱ��ڸ��Ե�task��ʵ��
//  *
//  ******************************************************************************
//  */
//#include "gimbal_task.h"
//#include "judge.h"
//#include "usbd_cdc_if.h"


///*************�������˲�**************/
///*���׿�����*/  //����
////��̨�Ƕ�������
//kalman_filter_init_t yaw_kalman_filter_para =
//{
//    .xhat_data = {0, 0},                                  //״̬���� λ�ã��Ƕȣ����ٶ� (�ᱻ���ٵ��������Ǻ���Ҫ)
//    .P_data = {2, 0, 0, 2},                               //Ԥ��״̬Э�������(�ᱻ���ٵ��������Ǻ���Ҫ)
//    .A_data = {1, 0.002, 0, 1},//����ʱ����             //״̬ת�ƾ���
//    .B_data = {0.002 * 0.002 * 0.5, 0.002},               //���ƾ���
//    .Q_data = {1, 0, 0, 1},                               //״̬ת�ƾ����Э������� ��Ԥ��ģ�͡����������
//    .H_data = {1, 0, 0, 1},                               //�۲����
//    .R_data = {200, 0, 0, 400}//500 1000                  //�۲�����Э����(��������)
//};//��ʼ��yaw�Ĳ���kalman����

//kalman_filter_init_t pitch_kalman_filter_para =
//{
//    .xhat_data = {0, 0},
//    .P_data = {2, 0, 0, 2},
//    .A_data = {1, 0.002, 0, 1},//����ʱ����
//    .B_data = {0.002 * 0.002 * 0.5, 0.002},
//    .Q_data = {1, 0, 0, 1},
//    .H_data = {1, 0, 0, 1},
//    .R_data = {200, 0, 0, 400}
//};//��ʼ��pitch�Ĳ���kalman����

//kalman_filter_t yaw_kalman_filter;
//kalman_filter_t pitch_kalman_filter;
//extKalman_t follow_kalman_yaw;
//extKalman_t follow_kalman_pitch;
//float *yaw_kf_result, *pitch_kf_result;//���׿������˲����,0�Ƕ� 1�ٶ�
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
//			currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
//			control_judge_flag++;//��can2���յ�����dbus��������get_dkey_info������
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
//										SystemValue = Running;      //ϵͳ��ʼ������
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
//										SystemValue = Running;      //ϵͳ��ʼ������
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

//							Gimbal_MotorPitch.motor_value->target_speed_rpm = - Gimbal_MotorPitch.Motor_PID_Position.out;      //λ�û����

//							Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//									Gimbal_MotorPitch.motor_value->speed_rpm,
//									Gimbal_MotorPitch.motor_value->target_speed_rpm);

//							set_moto5678_current(&hcan1, -Gimbal_MotorPitch.Motor_PID_Speed.out,0, 0, 0);  //����CAN���Ͷ�����

//							break;
//						default:
//								break;
//					}
//			}
//			vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);
//    }
//}


///**
//  * @brief  ��̨������ʼ��
//  * @param  void
//  * @retval void
//  * @attention û�м�I�������,ֻ��ϵͳ����ʱ����һ��
//  **/
//followStruct followData = {0};
//fp32 Gimbal_Pitch_Speed_pid[3] = {250,7,50};
//fp32 Gimbal_Pitch_imu_pid[3]   = {20,0,888};
//fp32 VISION_PITCH_PID[3] = {0.02,0.00001,0.02};

///**
//  * @brief  ��̨��������ʼ��
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
//    //���������˲�
//    kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
//    kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);

//    //һ�������˲�
//    KalmanCreate(&follow_kalman_yaw,   1, 100);
//    KalmanCreate(&follow_kalman_pitch, 1, 100);
//	
//	Cloud_Angle_Target[PITCH][GYRO] = 0;
//}

///**
//  * @brief  ң�������Ʒ�ʽ
//  * @param  void
//  * @retval void
//  * @attention
//  */
//uint16_t  test_flag=0;

//static void RemoteControlGimbal(void)
//{

//    if(PitchGimbalMode == USEENCODER) //����������PITCH��
//    {
//        Gimbal_MotorPitch.motor_value->target_angle += (float)rc.ch2 / SENSITIVITY_REMOTE_GIMBAL_PITCH; //Ŀ��������Ƕ�
//        LimitValue_f(&(Gimbal_MotorPitch.motor_value->target_angle), GIMBAL_PITCH_ENCODER_MAX, GIMBAL_PITCH_ENCODER_MIN);

//        Gimbal_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Position,
//                Gimbal_MotorPitch.motor_value->angle,
//                Gimbal_MotorPitch.motor_value->target_angle);

//        Gimbal_MotorPitch.motor_value->target_speed_rpm = -Gimbal_MotorPitch.Motor_PID_Position.out; //λ�û�

//        Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//                Gimbal_MotorPitch.motor_value->speed_rpm, //�ٶȻ�
//                Gimbal_MotorPitch.motor_value->target_speed_rpm);
//    }
//    else if(PitchGimbalMode == USEIMU) //��imu��PITCH��
//    {
//        //Ŀ��ֵ�ۼ�
//        if(actGimbal == GIMBAL_NORMAL)
//            Cloud_Angle_Target[PITCH][GYRO] -= (float)rc.ch2 / 2000;    //Ŀ�������ǽǶ�

//        //����������PITCH��λ�û�+����ٶȻ�
//        LimitValue_f(&Cloud_Angle_Target[PITCH][GYRO], 15, -40);    //15  -40
//        Gimbal_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Position, IMU_angle[1], Cloud_Angle_Target[PITCH][GYRO]);

//        Gimbal_MotorPitch.motor_value->target_speed_rpm = Gimbal_MotorPitch.Motor_PID_Position.out;      //λ�û����

//        Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//                Gimbal_MotorPitch.motor_value->speed_rpm,
//                Gimbal_MotorPitch.motor_value->target_speed_rpm);
//    }

//    set_moto5678_current(&hcan1, Gimbal_MotorPitch.Motor_PID_Speed.out,0, 0, 0);  //����CAN���Ͷ�����
//}
///**
//  * @brief  ��̨������
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
//				Gimbal_MotorPitch.motor_value->target_speed_rpm = -Gimbal_MotorPitch.Motor_PID_Position.out ; //λ�û�
//	
//			
//        Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//                Gimbal_MotorPitch.motor_value->speed_rpm, //�ٶȻ�
//                Gimbal_MotorPitch.motor_value->target_speed_rpm);
//    }
//    else if(PitchGimbalMode == USEENCODER)
//    {
//        Gimbal_MotorPitch.motor_value->target_angle = Cloud_Angle_Target[PITCH][MECH];

//        LimitValue_f(&(Gimbal_MotorPitch.motor_value->target_angle), GIMBAL_PITCH_ENCODER_MAX, GIMBAL_PITCH_ENCODER_MIN);

//        Gimbal_MotorPitch.Motor_PID_Position.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Position,
//                Gimbal_MotorPitch.motor_value->angle,
//                Gimbal_MotorPitch.motor_value->target_angle);

//        Gimbal_MotorPitch.motor_value->target_speed_rpm = -Gimbal_MotorPitch.Motor_PID_Position.out; //λ�û�

//        Gimbal_MotorPitch.Motor_PID_Speed.f_cal_pid(&Gimbal_MotorPitch.Motor_PID_Speed,
//                Gimbal_MotorPitch.motor_value->speed_rpm, //�ٶȻ�
//                Gimbal_MotorPitch.motor_value->target_speed_rpm);
//    }
//    set_moto5678_current(&hcan1, Gimbal_MotorPitch.Motor_PID_Speed.out,0, 0, 0);  //����CAN���Ͷ�����
//    //    set_moto5678_current(&hcan1, 0, 0, 0, 0);

//}

///**
//  * @brief  ���̿�����̨ģʽ
//  * @param  void
//  * @retval void
//  * @attention
//  */
//portTickType ztime;
//static void GIMBAL_Key_Ctrl()
//{
//    switch(actGimbal)
//    {
//    /*--------------��̨ģʽѡ��----------------*/
//    case GIMBAL_NORMAL:
//    {
//        ztime = 0;
//        kalman_filter_change_realtime();
//        if(ControlMode == KEYBOARD)
//            GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl();
//        break;
//    }
//    /*--------------�Ҽ�����----------------*/
//    case GIMBAL_AUTO:
//    {
//      if(VisionValue.identify_target == '0')
//      {  
//          if(ControlMode == KEYBOARD)
//              GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl();
//      }
//      else if(VisionValue.identify_target == '1')
//      {
//          ztime++;//:����ͨ����
//          
//          GIMBAL_AUTO_Mode_Ctrl();//������ƺ���
//      }
//      break;
//    }
//    default:
//        break;
//    }
//}

///**
//  * @brief  �������˲���ʼ������ǰ��ʼ���
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
//  * @brief  ������ƺ���
//  * @param  void
//  * @retval void
//  * @attention �м�����(0,0),�����Ҹ�,�ϸ�����
//  *            yawΪ������ģʽ,pitchΪ��еģʽ(��ʵpitchȫ�̶����û�еģʽ)
//  *            ֻ�е����������˲��ڵ�ǰʵʱ�Ƕ����ۼ�����Ŀ��Ƕ�
//  *            ������һ��,Ŀ��ǶȾ�ʵʱ����
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


///***********************************************************************************����Ĳ����Ǽ���ģʽ**********************************************************************************/
///**
//  * @brief  ��е ������ģʽ�л�����
//  * @param  void
//  * @retval void
//  * @attention ��̨���̿���״̬�µ�����ģʽ�л�������
//  * ��ģʽ�л�ʱһֱ���ڴ�ģʽ
//  */
////void GIMBAL_State_Change(GimbalModeType Type)
////{
////    if(Type == USEIMU)
////    {
////        if(YawGimbalMode == USEENCODER) //YAW��������������ģʽ�����
////        {
////            Cloud_Angle_Target[YAW][GYRO] = IMU_angle[0];  //����ģʽ�л�ʱ��̨��ת
////            Cloud_Angle_Target[PITCH][GYRO] = IMU_angle[1];
////            YawGimbalMode = USEIMU;
////        }
////    }
////    else if(Type == USEENCODER)
////    {
////        if(YawGimbalMode == USEIMU)
////        {
////            Cloud_Angle_Target[YAW][MECH]   = Gimbal_MotorYaw.motor_value  -> main_angle;   //�õ���ǰ�Ƕ�//
////            Cloud_Angle_Target[PITCH][MECH] = Gimbal_MotorPitch.motor_value-> angle;
////            YawGimbalMode = USEENCODER;
////        }
////    }
////}

////void GIMBAL_State_Change_key(void)
////{
////    if(YawGimbalMode == USEENCODER && ((actGimbal == GIMBAL_AUTO) || (actGimbal == GIMBAL_NORMAL) ||
////                                       (actGimbal == GIMBAL_GYROSCOPE) || (actGimbal == GIMBAL_CHASSIS_FOLLOW))) //YAW��������������ģʽ�����
////    {
////        Cloud_Angle_Target[YAW][GYRO] = IMU_angle[0];	//����ģʽ�л�ʱ��̨��ת
////        YawGimbalMode = USEIMU;
////    }
////    else if(YawGimbalMode == USEIMU && ((actGimbal != GIMBAL_AUTO) && (actGimbal != GIMBAL_GYROSCOPE)
////                                        && (actGimbal != GIMBAL_NORMAL)))
////    {
////        Cloud_Angle_Target[YAW][MECH]   = Gimbal_MotorYaw.motor_value  -> main_angle;   //�õ���ǰ�Ƕ�//
////        Cloud_Angle_Target[PITCH][MECH] = Gimbal_MotorPitch.motor_value-> angle;
////        YawGimbalMode = USEENCODER;
////    }
////}
///**
//  * @brief  ��̨����ģʽѡ��,������Ӧ
//  * @param  void
//  * @retval void
//  * @attention ��̨���̿���״̬�µ�����ģʽ�л�������
//  * ��ģʽ�л�ʱһֱ���ڴ�ģʽ
//  */
//bool Gimbal_Switch_Z = 1;
//static void GIMBAL_Mode_Choose(void)
//{
//    ////////////�����Ҽ�����////////////////
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
//    //////////////////G���л�����ģʽ����ͨģʽ/����ģʽ/////////////////////	
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
//    //////////////////ctrl���л�����ģʽ��vb��λ����pitch����yaw/////////////////////	
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
//    ////////////Z�������λ////////////////
//    if(!IF_KEY_PRESSED_Z)
//    {
//        Gimbal_Switch_Z = 1;
//    }
//    if(IF_KEY_PRESSED_Z && Gimbal_Switch_Z == 1)
//    {
//        Gimbal_Switch_Z = 0;
//        Stop_All();//���ͣת
//        soft_rest(); //�����λ
//    }
//}

///**
//  * @brief  ���̸�����̨����
//  * @param  void
//  * @retval void
//  * @attention
//  */
//static void GIMBAL_CHASSIS_FOLLOW_Mode_Ctrl()
//{
////    static portTickType  Key_Ctrl_CurrentTime = 0;
////    static TickType_t Turn_Time[2] = {0};
//    static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������?

////    static uint32_t QTurn90_ms  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
////    static uint32_t ETurn90_ms  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
////    static u8 Qturn90_f = 0;
////    static u8 Etrun90_f = 0;
//    static float Mouse_Gyro_Pitch;  //Mouse_Gyro_Yaw, ����������ģʽ�����ͳ��yawƫ����,��ֵ���Լ�������С,��ֹ˦ͷ����

////    Key_Ctrl_CurrentTime = xTaskGetTickCount();//��ȡʵʱʱ��,������������ʱ�ж�
//    //������תʱ��̨����
//    //���̸�����̨ʱת90�Ȳ���
////    if(actChassis == CHASSIS_FOLLOW_GIMBAL && IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > QTurn90_ms)
////    {
////        Turn_Time[NOW] = xTaskGetTickCount();//��ǰϵͳʱ��

////        QTurn90_ms = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms��ʱ���ּ���
////        Qturn90_f = 1;
////    }
////    else if(actChassis == CHASSIS_FOLLOW_GIMBAL && IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > ETurn90_ms)
////    {
////        Turn_Time[NOW] = xTaskGetTickCount();//��ǰϵͳʱ��
////        ETurn90_ms = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms��ʱ���ּ���
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
//    //��ͨ���Ʋ���
////    if(Etrun90_f == 0 && Qturn90_f == 0)
//    {
//        if(MOUSE_Y_MOVE_SPEED != 0)
//        {
//            Mouse_Gyro_Pitch -= MOUSE_Y_MOVE_SPEED * -0.01f;//pitch�Ծ�ʹ�û�еģʽ
//        }
//        else if(MOUSE_Y_MOVE_SPEED == 0)
//        {
//            Mouse_Pitch_Stop ++ ;
//            if(Mouse_Pitch_Stop > 5) //��곤ʱ��ͣ����ֹͣ�ƶ�
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
