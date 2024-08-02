/**
  ******************************************************************************
  * @file    chassis_task.c
  * @brief   ���̿������񣬰���ң�����ͼ������ֿ���ģʽ���Լ����̸��桢С���ݵȵ��˶�ģʽ
  * ��ʵ�֣������޹��ʣ������ķ�ֽ����㷨�����̣���̨��������λ��        
  ******************************************************************************
  * @attention
  *
  * 2021.4 ���ǵ���̨��λʱֻת��̨�����ܹ���������ԭ��ֻת��̨����̨��λ��ʽΪת���̣���̨����
  * ���ǵ�pid���Ƶ����أ�����λʱ��̨�͵��̵Ķ����ֿ����ֱ��ڸ��Ե�task��ʵ��
  *
  ******************************************************************************
  */
#include "chassis_task.h"
#include "arm_math.h"
#include "SQ_judge.h"
#include "cap.h"
#include "math.h"

Chassis_Speed absolute_chassis_speed;
int16_t chassis_setspeed[4];    //�ĸ�����Ŀ��ת��

PidTypeDef Chassis_Follow_PID;
 fp32 chassisnothing[3] = {0, 0, 0};
 
 

 
 
const static fp32 motorid1_speed_pid[3] ={10,0.39,0};  // {20,0.8,7}; //20, 0.8, 2  25.5, 1, 0
const static fp32 motorid2_speed_pid[3] = {10,0.39,0};
const static fp32 motorid3_speed_pid[3] = {10,0.39,0};
const static fp32 motorid4_speed_pid[3] = {10,0.39,0};
 fp32 Chassis_Follow_pid[3] = {0.00001, 0.00000000001, 0.00005};
                                 
/**
  * @brief  ���̸�������ʼ��
  * @param  void
  * @retval void
  * @attention void
  */
static void Chassis_InitArgument(void)
{
	//����ģʽ�л�����ͨ/����
    Flag_status.Chassis_Switch_F = 1;
    Flag_status.Chassis_Key_F_Change = 0;

    //תͷ180��
    Flag_status.Chassis_Switch_X = 1;
    Flag_status.Chassis_Key_X_Change = 0;

    //С����
	  Flag_status.Chassis_Switch_Shift = 1;
    Flag_status.Chassis_Key_Shift_Change = 0;

    //Ħ����ת������
    Flag_status.Chassis_Switch_Q = 1;
    Flag_status.Chassis_Key_Q_Change = 0;

    //Ħ����ת�ټ�С
    Flag_status.Chassis_Switch_E = 1;
    Flag_status.Chassis_Key_E_Change = 0;


    //��λ
    Flag_status.Chassis_Switch_Z = 1;
    Flag_status.Chassis_Key_Z_Change = 0;
	
	//���������� �̰��л�
    Flag_status.Chassis_Switch_C = 1;
    Flag_status.Chassis_Key_C_Change = 0;
	
	//�Ӿ�ģʽ�л�
    Flag_status.Chassis_Switch_G = 1;
    Flag_status.Chassis_Key_G_Change = 0;
    
    //���̸���ר��
    pid_init(&Chassis_Follow_PID);//PidTypeDef *pid,           mode,     fp32 PID[3],  max_out,   max_iout, I_Separation, Dead_Zone,    gama, angle_max,  angle_min
    Chassis_Follow_PID.f_param_init(&Chassis_Follow_PID, PID_POSITION, Chassis_Follow_pid, 3.0,     0.003,     1e30,          10,       0.2,    0,         0);

    /******************���̵��PID*****************************************/
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
        currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
				fellowfdb=absolute_chassis_speed.vw;
				fellowset=0;
				switch(ControlMode)
				{
					case KEYBOARD:
					{
						if(SystemValue == Starting)
						{
		//                    Chassis_open_init();
		//                    SystemValue = Running;      //ϵͳ��ʼ������--��ʼ��������̨
						}
						else
						{
							Chassis_Mode_Choose();     //���ó�����������
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
		//                    SystemValue = Running;      //ϵͳ��ʼ������
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
						Chassis_Mode_Choose();     //���ó�����������
						KeyboardControlChassis();	
						
						break;				
					}
					default:
						break;
				}
//			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);  //����Ƶ�ʲ���

        vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
    }
}

/**
  * @brief  ���̵�����
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
    Mecanum_Set_Motor_Speed(chassis_setspeed, Chassis_Motor); //���ø��������Ŀ���ٶ�
    /************************************���̵���ٶȻ�����*********************************************/
   
		
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
    /************************************�������������͸����*********************************************/
    Send_cap_msg();
    Chassis_Power_Limit();//��������,�������·���
    set_moto1234_current(&hcan1, Chassis_Motor[0].Motor_PID_Speed.out,
                        Chassis_Motor[1].Motor_PID_Speed.out,
                        Chassis_Motor[2].Motor_PID_Speed.out,
                        Chassis_Motor[3].Motor_PID_Speed.out);

}

/***********************************************************************************����Ĳ�����ң����ģʽ**********************************************************************************/
/**
  * @brief  ң����ģʽ����
  * @param  void
  * @retval void
  * @attention
  */
static void RemoteModeChoose(void)
{
	if(rc.sw1 == 1 && rc.sw2 == 3)
		actChassis = CHASSIS_FOLLOW_GIMBAL;  //���̸�����̨
	else if(rc.sw1 == 1 && rc.sw2 == 2)
		actChassis = CHASSIS_NORMAL;  //���̲�������̨
	else if(rc.sw1 == 1 && rc.sw2 == 1)
		actChassis = CHASSIS_GYROSCOPE;    //С����ģʽ
}

/**
  * @brief  ң�������Ʒ�ʽ
  * @param  void
  * @retval void
  * @attention
  */

float ttt= 0.0008;
static void RemoteControlChassis(void)
{
    /***********************************ȷ�������ĸ������Ŀ���ٶ�*****************************************/
    switch(actChassis)
    {
			case CHASSIS_FOLLOW_GIMBAL://������̨
				
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
						absolute_chassis_speed.vw, 0.0008); //PIDʹ���̸�����̨�ٶ�
				else 
					absolute_chassis_speed.vw = 0;
				break;
			case CHASSIS_NORMAL://��������̨
				absolute_chassis_speed.vx = RAMP_float((float)rc.ch4 / 200 , absolute_chassis_speed.vx  , 0.1);
				absolute_chassis_speed.vy = RAMP_float((float)rc.ch3 / 200 , absolute_chassis_speed.vy  , 0.1);
				absolute_chassis_speed.vw = 0;
				break;
			case CHASSIS_GYROSCOPE:    //С����ģʽ
				absolute_chassis_speed.vx = (float)rc.ch4 / 200;
				absolute_chassis_speed.vy = (float)rc.ch3 / 200;
				absolute_chassis_speed.vw = 0.0032;
				break;
			default:
					break;
    }
}

/***********************************************************************************����Ĳ����Ǽ���ģʽ**********************************************************************************/

/**
  * @brief  ����ѡ�����ģʽ
  * @param  void
  * @retval void
  * @attention ģʽѡ��,����ĳģʽ��ǵ�д�˳�����ͨģʽ���ж�
  * �ް������»�һֱ�����Զ�����ģʽ,����ģʽ�л���İ�����������ģʽ�л�ѡ��ģʽ
  */
static void Chassis_Mode_Choose()  //���̿�����
{
    //////////////////F���л�����ģʽ����ͨģʽ/����ģʽ/////////////////////	
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
	/////////////////Shift��ѡ��С����--��������ģʽ/////////////////////
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

    /////////////////Z�����´��������λ///////////////////
    if(!IF_KEY_PRESSED_Z)
    {
        Flag_status.Chassis_Switch_Z = 1;
    }
    if(IF_KEY_PRESSED_Z && Flag_status.Chassis_Switch_Z == 1)
    {
		    set_cap1(&hcan2, CAP_STATUS_UNUSUAL, CAP_SWITCH_CLOSE, 0);
        Flag_status.Chassis_Switch_Z = 0;
        Stop_All();//���ͣת
        soft_rest(); //�����λ
    }
}


/**
  * @brief  ���̿��Ƶ���ģʽ
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
  * @brief  ����ģʽ�µ����˶�����
  * @param  �ٶ���������    �����ٶ�(���293)
  * @retval void
  * @attention  ���̿���ǰ������ƽ��,ƽ���޻�е��������ģʽ֮��
  *             ��Ҫ��ȡʱ��������б�º�������
  */
/************���̸���ģʽ��һЩ��������*************/
float    Chassis_Standard_Move_Max;                 //����ǰ������ƽ������
int16_t  timeXFron, timeXBack, timeYLeft, timeYRigh;//����  s  w  d  a

//����ģʽ��ȫ���ƶ�����,б����
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
    static float Mouse_w;  //����������ģʽ�����ͳ��w����,��ֵ���Լ�������С,��ֹ˦ͷ����

    Chassis_Standard_Move_Max = sMoveMax;//�����ٶ��޷�,ˮƽ�ƶ�
    ulCurrentTime = xTaskGetTickCount();//��ǰϵͳʱ��

    if (ulCurrentTime >= ulDelay)//ÿ10ms�仯һ��б����
    {
        ulDelay = ulCurrentTime + 10;

        if (IF_KEY_PRESSED_W)
        {
            w_cnt = 0;
            W = 1;
            timeXBack = 0;//����ǰ�������б�¹���,�����´μ������б��
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
            timeXFron = 0;//ͬ��
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
							if(Mouse_Stop > 5) //��곤ʱ��ͣ����ֹͣ�ƶ�
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
								absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //ǰ�����
								absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //���Ҽ���
								
							}else{
								absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
								absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
							}

            }
            else
            {
							gimbal_follow = GIMBAL_TAIL;
							if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
							{
								absolute_chassis_speed.vx  = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //ǰ�����
								absolute_chassis_speed.vy  = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //���Ҽ���
								
							}else{
								absolute_chassis_speed.vx  = -(Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
								absolute_chassis_speed.vy  = -(Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
							}
            }
        }
        else
        {
					if(cap_info.cap_status == CAP_STATUS_FLAG && cap_info.switch_status == CAP_SWITCH_OPEN)
					{
						absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 4000.0f; //ǰ�����
						absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 4000.0f; //���Ҽ���
						
					}else{
						absolute_chassis_speed.vx  = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) / 7000.0f; //ǰ�����
						absolute_chassis_speed.vy  = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) / 7000.0f; //���Ҽ���
					}
        }
    }
}
/**
  * @brief  ���̼���б�º���
  * @param  �жϰ����Ƿ񱻰���, ʱ����, ÿ�����ӵ���, һ��Ҫ��С����
  * @retval б�±���ϵ��
  * @attention  0~1
  */
static float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
    float  factor = 0;
    factor = 0.15 * sqrt( 0.15 * (*time) );  //�����ٶ�б��,time�ۼӵ�296.3б�¾����

    if (status == 1){  //����������
		
        if (factor < 1)//��ֹtime̫��
			*time += inc;
		
    }else{  //�����ɿ�
        if (factor > 0)
        {
            *time -= dec;
            if (*time < 0)
				*time = 0;
        }
    }
    LimitValue_f(&factor,1,0);//ע��һ����float�����޷�
	
    return factor;  //ע�ⷽ��
}


/**
  * @brief  �����Ƶ�����ת,����QEC���ƿ���תȦ
  * @param  �ٶ���������
  * @retval void
  * @attention  ������������ת
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

/***********************************************************************************�Ե��̵�����еĸ��ִ���**********************************************************************************/

/*****************���̹���*************************/
//��ͬģʽ�µ�����ٶ�
#define    CHAS_CURRENT_LIMIT_Lv0        27500
#define    CHAS_CURRENT_LIMIT_Lv1        31000    //�ĸ����ӵ��ٶ��ܺ����ֵ,�������*4,�޹��ʵ���������
#define    CHAS_CURRENT_LIMIT_Lv2        32000
#define    CHAS_CURRENT_LIMIT_Lv3        33000
#define    CHAS_CURRENT_LIMIT_Lv4        33000
#define    CHAS_CURRENT_LIMIT_Lv5        33000
#define    CHAS_CURRENT_LIMIT_Lv6        33000
#define    CHAS_CURRENT_LIMIT_Lv7        33000
#define    CHAS_CURRENT_LIMIT_Lv8        33000
#define    CHAS_CURRENT_LIMIT_Lv9        33000
#define    CHAS_CURRENT_LIMIT_Lv10        33000


float WARNING_REMAIN_POWER = 40;//����ϵͳʣ�ཹ���������������ֵ��ʼ�޹���,40Ťƨ�ɻᳬ����,ƽ�ؿ����ᳬ
float fChasCurrentLimit = CHAS_CURRENT_LIMIT_Lv3;//����4�����ӵ��ٶ��ܺ�
float fTotalCurrentLimit;//��������,ƽ��ģʽ�·����Ǿ��ȵ�
/**
  * @brief  ���̹�������
  * @param  void
  * @retval void
  * @attention  �ڵ��������������,��Ҫ�Ǳ������㷨,ICRA
  */
	
	
PidTypeDef buffer_pid; 
fp32 buffer[3]={2,0.2,0};
void get_chassis_power_and_buffer(float *chassis_power,float *chassis_power_buffer)
{
    *chassis_power=JUDGE_fGetChassisPower();//��ȡʵʱ����
    *chassis_power_buffer=JUDGE_fGetRemainEnergy();//��ȡʣ�ཹ������
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
    //ʵ������k2   1.453e-07;  1.255e-07;           1.653e-07
    fp32 k2=1.23e-07;
    //ʵ������k1   1.23e-07;    1.44e-07;           1.43e-07
    fp32 constant=2.081f;
    //             4.081f;       3.343;С���ݲ���   2.081f  ���Ҳ�ǣ����ǲ���С���ݾ���
    get_chassis_power_and_buffer(&chassis_power,&chassis_power_buffer);
    pid_init(&buffer_pid);
	  buffer_pid.f_param_init(&buffer_pid , PID_POSITION,buffer,100,1000,0,0,0,0,0);
    buffer_pid.f_cal_pid(&buffer_pid,chassis_power_buffer,30);//pid���ƻ�������������30
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


//float    kLimit = 1;//��������ϵ��
//float    chassis_totaloutput = 0;//ͳ�����������
//float    Joule_Residue = 0;//ʣ�ཹ����������
//float    Power_Limit_Out = 0;
//uint16_t Power_realtime = 0;
//uint16_t Power_limit = 0;
//int16_t  judgDataCorrect = 0;//����ϵͳ�����Ƿ����
//static int32_t judgDataError_Time = 0;

//static void Chassis_Power_Limit()
//{
//    /*********************�洫�㷨*************************/
//    Power_realtime = JUDGE_fGetChassisPower(); //��ȡʵʱ���̹���
//    Power_limit = JUDGE_usGetPowerLimit();  //��ȡ��������
//    judgDataCorrect = JUDGE_sGetDataState();//����ϵͳ�����Ƿ����
//    Joule_Residue = JUDGE_fGetRemainEnergy();//ʣ�ཹ������
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
//    //ͳ�Ƶ��������
//    chassis_totaloutput = abs_float(Chassis_Motor[0].Motor_PID_Speed.out) + abs_float(Chassis_Motor[1].Motor_PID_Speed.out)
//                          + abs_float(Chassis_Motor[2].Motor_PID_Speed.out) + abs_float(Chassis_Motor[3].Motor_PID_Speed.out);

//    if(judgDataCorrect == JUDGE_DATA_ERROR)//����ϵͳ��Чʱǿ������
//    {
//        judgDataError_Time++;
//        if(judgDataError_Time > 100)
//        {
//            fTotalCurrentLimit = fChasCurrentLimit/2;//��Ϊ����1/4
//        }
//    }
//    else
//    {
//      judgDataError_Time = 0;   
//        //ʣ�ཹ������С,��ʼ�������,����ϵ��Ϊƽ����ϵ
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
//        else   //���������ָ���һ����ֵ
//        {
//            fTotalCurrentLimit = fChasCurrentLimit;
//        }
//    }

//    //���̸�����������·���
//    if (chassis_totaloutput > fTotalCurrentLimit)
//    {
//        Chassis_Motor[0].Motor_PID_Speed.out = (float)(Chassis_Motor[0].Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
//        Chassis_Motor[1].Motor_PID_Speed.out = (float)(Chassis_Motor[1].Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
//        Chassis_Motor[2].Motor_PID_Speed.out = (float)(Chassis_Motor[2].Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
//        Chassis_Motor[3].Motor_PID_Speed.out = (float)(Chassis_Motor[3].Motor_PID_Speed.out / chassis_totaloutput * fTotalCurrentLimit);
//    }

//}

///**
//  * @brief  ���õ��̵������������ֵ
//  * @param  ���1���ֵ�����2���ֵ�����3���ֵ�����4���ֵ
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
//  * @brief  ���ݲ�ͬ�����������
//  * @param  void
//  * @retval void
//  * @attention void
//  */
//void LimitChassisMotorCurrent(void)
//{
//    switch(actChassis)
//    {
//    case CHASSIS_NORMAL:    //���̲�������̨
//        switch(EnvironmentMode)
//        {
//			case NOMAL:      //��ͨ����
//				SetChassisMotorMaxCurrent(NOMOAL_CHASSIS_MAX1, NOMOAL_CHASSIS_MAX2, NOMOAL_CHASSIS_MAX3, NOMOAL_CHASSIS_MAX4);
//				break;
//			case CLIMBING:    //���µ���
//				SetChassisMotorMaxCurrent(CLIMBING_CHASSIS_MAX1, CLIMBING_CHASSIS_MAX2, CLIMBING_CHASSIS_MAX3, CLIMBING_CHASSIS_MAX4);
//				break;
//        }
//        break;
//    case CHASSIS_FOLLOW_GIMBAL:      //���̸�����̨
//    case CHASSIS_GYROSCOPE:      //С����ģʽ
//    default:
//        break;
//    }
//}

/*
 * @param absolute_speed ����������Ҫ���ٶ�
 * @param angle ��̨����ڵ��̵ĽǶ�
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
  * @brief  �ҳ���+y����Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
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
  * @brief  �ҳ���45������Сƫ���
  * @param  void
  * @retval ƫ��ǣ��Ƕ���
  * @attention ͨ��ң����/����
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
