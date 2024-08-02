/**
  ******************************************************************************
  * @file    shoot_task.c
  * @brief   �����˵����task������ң�����ͼ������ֿ���ģʽ���Լ��������ơ���������
  *
  ******************************************************************************
  * @attention
  *
  * 2021.4 ��д�ķ���������Ч���Ƚ����ԣ�����������Ϊ��Ȼ��Ȩ��֮��
  * �Ͻ����������������ƺ���Ҳ�������á��������Ʒ�ʽΪ�����򼴽�������ʱ����relaxģʽ�������̷���
  * ͬʱ����ģʽѡ�񣬷�ֹ���ɱ����
  * ����ɵ��
  *
  ******************************************************************************
  */
#include "shoot_task.h"
//#include "param.h"
#include "SQ_judge.h"
#include "Motor_DM.h"
fp32 abstotalangle=0;  //�ϵ��ľ��ԽǶ�
fp32 angleture=1;
float shoot_tp ,shoot_p,closest;
//float Ammunition_DM_Angle[6] = {0.835f,1.88219f,2.92938f,3.97657f,5.02376f,6.07095f};
uint16_t DM_Angle_choose;

uint8_t DM_enable_flag = 0;
fp32 Ammunition_Motor_Position_pid[3] =   {0.4, 0.00001, 2};  
fp32 Ammunition_Motor_Position_pid_pos[3] = {0.07,0.00001,0.5};
fp32 Ammunition_Motor_Speed_pid_pos[3] = {12,1.5 ,20};
fp32 Ammunition_Motor_Speed_pid[3] = {0.4,0.1,2.0};//{12,1.5,20};//{18, 1, 0};

fp32 Ammunition_DM_Motor_Position_pid[3] =  {18,0.005,3};  //d=2
fp32 Ammunition_DM_Motor_Position_pid_pos[3] = {18,0.005,3};
fp32 Ammunition_DM_Motor_Speed_pid_pos[3] ={0.4,0.1,1.5}; //{0.4,0.1,2.0};
fp32 Ammunition_DM_Motor_Speed_pid[3] = {0.4,0.1,1.5};

void ShootFun(void const *argument)
{
    portTickType currentTime;
		
    REVOLVER_InitArgument();
	
    while (1)
    {
        currentTime = xTaskGetTickCount(); //��ǰϵͳʱ��
			
			if(Ammunition_DM_Motor.motor_value->hall ==0)
			{	
				Motor_enable();					//���ʹ��	
				DM_enable_flag	= 1;			
//				Ammunition_DM_Motor.motor_value->target_position = Ammunition_DM_Motor.motor_value->position;
			}

	
        Limit_something(); //�������� ��������

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
						SHOOT_Mode_Choose();
						REVOLVER_Key_Ctrl();
						break;
        }
		
		    SendJudgeMsg();
        vTaskDelayUntil(&currentTime, TIME_STAMP_2MS); //������ʱ
    }
}

/**
  * @brief  ���������������ʼ��
  * @param  void
  * @retval void
  * @attention void
  */
static void REVOLVER_InitArgument(void)
{
    Flag_status.protect_flag_heat = false;
    Flag_status.protect_flag_sutck = false;
    Flag_status.shoot_cnt = 0;
    Flag_status.shoot_left_time = 0;//�����������ʱ��,ʱ������л�������
		Flag_status.shoot_single_finish_flag = 1;
		Flag_status.shoot_single_time = 0;



				Motor_Init_DM(&Ammunition_DM_Motor, 1,Ammunition_DM_Motor_Position_pid, PID_POSITION, 30, 20, 1e30, 0, 0.2, 0, 0 , //40,20,20,0,0.75,0,0,//  
											Ammunition_DM_Motor_Speed_pid, PID_POSITION, 10, 5, 3e38,0,0.1,0, 0);
				Ammunition_DM_Motor.Motor_PID_Position.flag_Slop =1;     //  �޷�б��
				Ammunition_DM_Motor.Motor_PID_Speed.flag_Slop =1;
				Ammunition_DM_Motor.Motor_PID_Position.Slop = 0.5;
				Ammunition_DM_Motor.Motor_PID_Speed.Slop = 3;
		    Ammunition_DM_Motor.motor_value->hall = 0;
	if(abstotalangleinit==1)
		{
			abstotalangle=Ammunition_DM_Motor.motor_value->total_angle;
			Ammunition_DM_Motor.motor_value->target_angle =abstotalangle ;
			
		}
}

/**
  * @brief  ����ģʽ�·���ģʽѡ��
  * @param  void
  * @retval void
  * @attention  ��ͨģʽ�������,�Ҳ�����
  */
static void SHOOT_Mode_Choose(void)
{
  main_angle = Ammunition_DM_Motor.motor_value->total_angle / 19.f;
	if(rc.sw1 == 1 && rc.sw2 == 2)
	{
		ControlMode = REMOTE;
		SystemValue = Starting;
	}
	if(ShootState == UNSTART)
	{
			Ammunition_DM_Motor.motor_value->target_velocity = 0;
			Ammunition_DM_Motor.motor_value->target_angle = Ammunition_DM_Motor.motor_value->total_angle;  //19.f;
			actShoot = SHOOT_NORMAL;
			ShootState = START;
	}
	else if(ShootState != UNSTART && (!(Flag_status.protect_flag_sutck || (Flag_status.protect_flag_heat&&Flag_status.shoot_single_finish_flag))))
	{
			if((IF_MOUSE_PRESSED_LEFT && Flag_status.shoot_left_time <= TIME_STAMP_500MS && actGimbal != GIMBAL_AUTO) //��ͨģʽ����
		|| (actGimbal == GIMBAL_AUTO && (IF_MOUSE_PRESSED_LEFT ) && Flag_status.shoot_left_time <= TIME_STAMP_500MS)  //����ģʽ�������ĵ�󵥻� && VisionValue.center_flag
		|| ((!Flag_status.shoot_single_finish_flag) && (Flag_status.shoot_single_time > 0))) //������δ��ɵ���ȫ������
			{
				if((!(Flag_status.protect_flag_sutck || Flag_status.protect_flag_heat)))
				{
					Flag_status.shoot_left_time++;//�жϳ���,�л�
				}
					if(Flag_status.shoot_single_flag)
					{
						actShoot_last = actShoot;
						Flag_status.shoot_single_flag =0;
					}
					if(!(Flag_status.shoot_triple_flag )) Flag_status.shoot_triple_flag =1;
					if(!(Flag_status.shoot_normal_flag )) Flag_status.shoot_normal_flag =1;

					actShoot = SHOOT_SINGLE;

			}
			
			else if(((!IF_MOUSE_PRESSED_LEFT && actGimbal != GIMBAL_AUTO) || (!IF_MOUSE_PRESSED_LEFT && actGimbal == GIMBAL_AUTO)) && single_shoot_flag == 0)// && !VisionValue.center_flag
			{				
				Flag_status.shoot_left_time = 0;
				if((Flag_status.shoot_single_finish_flag && actShoot == SHOOT_SINGLE) || actShoot == SHOOT_TRIPLE)
				{
					if(Flag_status.shoot_normal_flag)
					{
						actShoot_last = actShoot;
						Flag_status.shoot_normal_flag =0;
					}
					if(!(Flag_status.shoot_triple_flag )) Flag_status.shoot_triple_flag =1;
					if(!(Flag_status.shoot_single_flag )) Flag_status.shoot_single_flag =1;

					actShoot = SHOOT_NORMAL;

					single_shoot_flag=1;

				}
			}
			
			else if(IF_MOUSE_PRESSED_LEFT && Flag_status.shoot_left_time > TIME_STAMP_500MS&&Flag_status.shoot_single_finish_flag)//��������200ms && actGimbal != GIMBAL_AUTO
			{
					Flag_status.shoot_left_time++;
					if(Flag_status.shoot_triple_flag)
					{
						actShoot_last = actShoot;
						Flag_status.shoot_triple_flag =0;
					}
					if(!(Flag_status.shoot_single_flag )) Flag_status.shoot_single_flag =1;
					if(!(Flag_status.shoot_normal_flag )) Flag_status.shoot_normal_flag =1;

					actShoot = SHOOT_TRIPLE;//����ģʽ

			}
	}
}

/************************���̼���ģʽ����ģʽС����****************************/

/*******����ģʽ************/
/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention 
  */

uint32_t countsingle=0;
static void SHOOT_SINGLE_Ctrl(void)
{
	static uint8_t single_step = 0;
	static uint8_t single_cnt = 0;
	static uint16_t last_shoot_num = 0;
	static uint16_t shoot_num = 0;
	static uint8_t judge_shoot_flag = 0;//����ϵͳ�����Ƿ�����
    if(single_shoot_flag)  //��֤��갴��˲����δ���ִֻ��һ��
    {

			while(abstotalangle -Ammunition_DM_Motor.motor_value->total_angle<0.0f)//26165.3f
			{
				abstotalangle +=26218.78f;  //8192*3591/187 *60/360
			}
			while(abstotalangle-Ammunition_DM_Motor.motor_value->total_angle >26218.78f)//26165.3f
			{
				abstotalangle -=26218.78f;
				
			}
			if(abstotalangle -Ammunition_DM_Motor.motor_value->total_angle>=0&&abstotalangle -Ammunition_DM_Motor.motor_value->total_angle<300)
			{
				Ammunition_DM_Motor.motor_value->target_angle=abstotalangle;
				angleture =1;
			}
			else if(Ammunition_DM_Motor.motor_value->total_angle-(abstotalangle-26218.78f)<300)
			{
				Ammunition_DM_Motor.motor_value->target_angle=abstotalangle-26218.78f;
				angleture =1;
			}
			else
			{
				angleture =0;
				Ammunition_DM_Motor.motor_value->target_angle=abstotalangle;
			}
			if(Ammunition_DM_Motor.motor_value->total_angle-(abstotalangle+26207.3f)<0&&Ammunition_DM_Motor.motor_value->total_angle-(abstotalangle+26207.3f)>-9000.0f )
			{
				abstotalangle +=26207.3f;
			}			

			if(angleture==1)
			{
				Ammunition_DM_Motor.motor_value->target_angle += 18218.00f;//��֤���ÿ����һ�β����Ĳ�ֵ��Ϊ60��
			}
			pid_reset(&Ammunition_DM_Motor.Motor_PID_Position, Ammunition_DM_Motor_Position_pid);
			pid_reset(&(Ammunition_DM_Motor.Motor_PID_Speed), Ammunition_DM_Motor_Speed_pid_pos);
			Flag_status.shoot_single_finish_flag = 0;
			Flag_status.shoot_single_time = 0;
			single_step = 0;
			single_cnt = 0;
			single_shoot_flag = 0;
			judge_shoot_flag = 0;
    }
	
		if(JUDGE_sGetDataState() && !Flag_status.shoot_single_finish_flag)
		{
			last_shoot_num = shoot_num;
			shoot_num = JUDGE_usGetShootNum();
			if(shoot_num> last_shoot_num)
			{
				judge_shoot_flag = 1;
			}
		}
		judge_shoot_flag = 0;
		switch(single_step)
		{
			case 0:
				if(fabs(Ammunition_DM_Motor.motor_value->target_angle - Ammunition_DM_Motor.motor_value->total_angle) <= 100 && !Flag_status.shoot_single_finish_flag)
				{
						countsingle++;
						if(countsingle ==200)
						{
							countsingle=0;
							single_step = 1;
							single_cnt = 0;
							if(angleture==1)
							{								
								Ammunition_DM_Motor.motor_value->target_angle += 8000.78f ;//12955.16f - compensate_flag * 9079.4f; //6485.4f
							}
								pid_reset(&Ammunition_DM_Motor.Motor_PID_Position,Ammunition_DM_Motor_Position_pid_pos);
							
						}
					
				}
				break;
			case 1:
				if(fabs(Ammunition_DM_Motor.motor_value->target_angle - Ammunition_DM_Motor.motor_value->total_angle) <= 100 && (!Flag_status.shoot_single_finish_flag))
				{
					single_cnt ++;
					if(single_cnt >= 30)
					{
						single_step++;
						Flag_status.shoot_single_finish_flag = 1;
					}
				}			
				break;
				
		}
		if(!Flag_status.shoot_single_finish_flag)
		{
			Flag_status.shoot_single_time++;
			if(Flag_status.shoot_single_time >= 1000)
			{
				Flag_status.shoot_single_finish_flag = 1;
				Flag_status.shoot_single_time = 0;
			}	
		}
		if(!(Flag_status.shoot_single_finish_flag)){
			
			Ammunition_DM_Motor.Motor_PID_Position.f_cal_pid(&Ammunition_DM_Motor.Motor_PID_Position,
				Ammunition_DM_Motor.motor_value->position,
				Ammunition_DM_Motor.motor_value->target_position);

			Ammunition_DM_Motor.motor_value->target_velocity = Ammunition_DM_Motor.Motor_PID_Position.out;
			
		}else{
			PID_clear(&Ammunition_DM_Motor.Motor_PID_Position);
			Ammunition_DM_Motor.motor_value->target_velocity = 0;
		}
}


uint16_t shoot_flag_dm=0,shoot_flag_dm_time=0;   //���Խ��뺯������ 


static void DM_SHOOT_SINGLE_Ctrl(void)
	{
		static uint8_t single_cnt = 0;
  
		shoot_flag_dm_time++;
    shoot_p= Ammunition_DM_Motor.motor_value->position;;
		if(single_shoot_flag)
		{		
			AngleLoop(&shoot_p,_PI);
			Ammunition_DM_Motor.motor_value->target_position += 1.04719f; 
			Flag_status.shoot_single_finish_flag = 0;
			Flag_status.shoot_single_time = 0;
			single_shoot_flag = 0;
		  shoot_flag_dm++;
		}			


		if(fabs(Ammunition_DM_Motor.motor_value->target_position - Ammunition_DM_Motor.motor_value->position) <= 0.02 && (!Flag_status.shoot_single_finish_flag))
				{
					single_cnt ++;
					if(single_cnt >= 200)
					{
						Flag_status.shoot_single_finish_flag = 1;
					}
				}			
		
		if(!Flag_status.shoot_single_finish_flag)
			{
				Flag_status.shoot_single_time++;
					if(Flag_status.shoot_single_time >= 2500)
					{
						Flag_status.shoot_single_finish_flag = 1;
						Flag_status.shoot_single_time = 0;
					}	
		 	}

}



static void SHOOT_SINGLE_Ctrl_DM(void)
{
	if(Flag_status.rc_shoot_flag)
		{

				Ammunition_DM_Motor.motor_value->target_position += 1.04719f;
				Flag_status.rc_shoot_flag=0;
				shoot_flag_dm++;
		}
}


static void SHOOT_TRIPLE_Ctrl(void)
{
	pid_reset(&(Ammunition_DM_Motor.Motor_PID_Speed), Ammunition_DM_Motor_Speed_pid);
	
	if(IF_KEY_PRESSED_R)//�������� һ����Ƶ
	{
		Ammunition_DM_Motor.motor_value->target_velocity = 3;    //5000;
	}	
	else
		Ammunition_DM_Motor.motor_value->target_velocity = 0.7;     //100;
}

/**
  * @brief  ���̵ļ���ģʽ
  * @param  void
  * @retval void
  * @attention ������λ�û�����
  */
static void REVOLVER_Key_Ctrl(void)
{
    /*------ ���̧�����ܴ���һ�� -------*/
    switch(actShoot)
    {
			case SHOOT_NORMAL:
			{
//				Ammunition_DM_Motor.motor_value->target_position =shoot_tp; 
				break;
			}
			case SHOOT_SINGLE:
			{
					DM_SHOOT_SINGLE_Ctrl();
					break;
			}
			case SHOOT_TRIPLE:
			{
					SHOOT_TRIPLE_Ctrl();
					break;
			}
			case RELAX:
			{
					Ammunition_DM_Motor.motor_value->target_velocity = -2;     //-60;         
					break;
			}
			case STOP:
			{
					Ammunition_DM_Motor.motor_value->target_velocity = 0;
					break;
			}
			case SHOOT_SINGLE_DM:
			{
					SHOOT_SINGLE_Ctrl_DM();
					break;
			}
			default:
				break;
    }
		
		

		

		if(DM_enable_flag == 0){
		  shoot_p = Ammunition_DM_Motor.motor_value->position;
			AngleLoop_DM(&(Ammunition_DM_Motor.motor_value->target_position),&shoot_p);
      Ammunition_DM_Motor.Motor_PID_Position.f_cal_pid(&Ammunition_DM_Motor.Motor_PID_Position,shoot_p,Ammunition_DM_Motor.motor_value->target_position);
			Ammunition_DM_Motor.motor_value->target_velocity=Ammunition_DM_Motor.Motor_PID_Position.out;
			Ammunition_DM_Motor.Motor_PID_Speed.f_cal_pid(&Ammunition_DM_Motor.Motor_PID_Speed,Ammunition_DM_Motor.motor_value->velocity ,Ammunition_DM_Motor.motor_value->target_velocity);			
		}
			else if (DM_enable_flag > 0 )
			{
			Ammunition_DM_Motor.motor_value->target_position = moto_CAN_DM[0].position;
			Ammunition_DM_Motor.Motor_PID_Speed.out = 0;
				
			DM_enable_flag++;
			if(DM_enable_flag > 2)
			 {
				DM_enable_flag = 0;
			}			
			}
			//			if( (Game_Robot_State.power_management_shooter_output == 0)&& (JUDGE_sGetDataState() == TRUE ))
			if( Game_Robot_State.power_management_shooter_output == 0)		
			{
			   PID_clear(&Ammunition_DM_Motor.Motor_PID_Position);
			   PID_clear(&Ammunition_DM_Motor.Motor_PID_Speed);
				 Ammunition_DM_Motor.motor_value->target_position = moto_CAN_DM[0].position; 
				 Ammunition_DM_Motor.Motor_PID_Speed.out = 0;
			}
			MIT_CtrlMotor(&hcan1,0x102,  0,  0, 0, 0, Ammunition_DM_Motor.Motor_PID_Speed.out);

}

/*********************��Ƶ��������****************************/

uint16_t judge_test_buff[5];




/**
  * @brief  ǹ����������
  * @param  void
  * @retval �����Ƿ���
  * @attention  ����Ҫ����һ�²���,����ʣ��ɷ����������ջ�
  *             �����˫ǹ����˺���������
  */
static bool Revolver_Heat_Limit(void)
{
    static uint16_t  usHeatBuffer = 0;
    static bool  IfShootAllow  =  TRUE;

    static  uint16_t  usShootNumBuffer = 0;
    static  portTickType  ulShootTimeRecordReal = 0;
    static  portTickType  msShootTimeRecord = 0;
    static  uint16_t  usShootNumPrev = 0;
	
    uint16_t  usHeatReal = 0;
    uint16_t  usShootNumReal = 0;
    uint16_t  usHeatOneShoot = 100;  //����ϵͳ��⵽��һ�ţ�ǹ��������80��������ٶ��޹� ������һ��100 �������ǵ��͵��ӳٿ����ʵ�ЩСһ��
    uint16_t  usHeatLimit;

    /* ��ȡ���� */
    usHeatReal = JUDGE_usGetRemoteHeat42();
    /* ��ȡ��������*/
    usHeatLimit = JUDGE_usGetHeatLimit();

    /* ��ȡ������� */
    usShootNumReal = JUDGE_usGetShootNum();

    /* ��¼��ǰʱ�� */
    ulShootTimeRecordReal = xTaskGetTickCount();

		judge_test_buff[0] = usHeatReal;
		judge_test_buff[1] = usHeatLimit;
		judge_test_buff[2] = usShootNumReal;
		judge_test_buff[3] = ulShootTimeRecordReal;



    /* ֻҪ��100MSǹ����ȴֵ�� */
    if(ulShootTimeRecordReal > msShootTimeRecord)
    {
        msShootTimeRecord = ulShootTimeRecordReal + 100;
        usShootNumBuffer -= JUDGE_usGetShootCold() / 10;
    }

    /* ֻҪ���˵������� */
    if (usShootNumReal > usShootNumPrev)
    {
        usShootNumBuffer += 10;
    }

    /* ʣ������ */
    if (usHeatReal <= usHeatLimit){
        usHeatBuffer = usHeatLimit - usHeatReal;
    } else{
        usHeatBuffer = 0;
    }

    if (usHeatBuffer >= usHeatOneShoot){  //ʣ���������ڴ�һ����������
        /* ���ܴ�����ӵ���Ŀ */
        IfShootAllow = TRUE;
    }else{
        IfShootAllow = FALSE;
    }

    usShootNumPrev = usShootNumReal;
    if(IF_KEY_PRESSED_R)//�������� һ����Ƶ
	{
		IfShootAllow = TRUE;
	}
        return IfShootAllow;	
	

}

/*****************************��������**************************************/
///************����************/
/**
  * @brief  �ٶȻ�ʽ��������
  * @param  void
  * @retval void
  * @attention  ��ס����������һ������������
  */
static void REVOL_SpeedStuck(void)
{
    static uint16_t  stuck_time    = 0;//������ʱ
    static uint16_t  relax_time = 0;//���ɼ�ʱ
    static bool Revol_Speed_ifStuck = FALSE;//�����ж�

    if (Revol_Speed_ifStuck == TRUE)//��ȷ�Ͽ���,��ʼ���ɼ�ʱ
    {
			  if(Flag_status.stuck_flag)
				{
					actShoot_last = actShoot;
					Flag_status.stuck_flag =0;
				}
//        actShoot = RELAX;
        Flag_status.protect_flag_sutck = 1;   //������ 1
        relax_time++;//����һ��ʱ��

        if (relax_time > Stuck_Relax_Time )//�������
        {
            relax_time = 0;
            Flag_status.protect_flag_sutck = 0;
            Revol_Speed_ifStuck = FALSE;//������ת					
        }
    }
    else//Stuck_Revol_PIDTerm
    {
        if ( fabs(Ammunition_DM_Motor.Motor_PID_Speed.out) >= Stuck_Revol_PIDTerm //PID�������
                && abs(Ammunition_DM_Motor.motor_value->velocity) <= Stuck_Speed_Low  &&
		            Ammunition_DM_Motor.motor_value->target_velocity > 0)//�ٶȹ���
        {
            stuck_time++;//������ʱ
					if(stuck_time>=2000)
					{
						stuck_time =2000;
					}
        }
        else
        {
            stuck_time = 0;//û�г�ʱ�俨��,��ʱ����
        }

        if (stuck_time > Stuck_SpeedPID_Time)//���˳���60ms
        {
            stuck_time = 0;
            Revol_Speed_ifStuck = TRUE;//��ǿ��Խ�����ɼ�ʱ
        }
				Flag_status.stuck_flag =1;
        Flag_status.protect_flag_sutck = 0;
   } 
}
/**
  * @brief  ���������������������ַ�
  * @param  void
  * @retval void
  * @attention  void
  */
static void Limit_something(void)
{
    if(Revolver_Heat_Limit() == FALSE)
    {
			  if(Flag_status.heat_flag)
				{
					actShoot_last = actShoot;
					Flag_status.heat_flag=0;
				}
        actShoot = STOP;
        Flag_status.protect_flag_heat = 1;  ///��������  ����Ӧ����1
    }
    else
    {
        Flag_status.protect_flag_heat = 0;
			  Flag_status.heat_flag=1;
    }
    REVOL_SpeedStuck();
}


/**
  * @brief  ң���������䵯
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
            Ammunition_DM_Motor.motor_value->target_velocity = 0;
        }
        if((rc.sw1 == 3 && rc.sw2 == 3) || Flag_status.moca_flag == 2)
        {
            Flag_status.moca_flag = 2;
        }
        if((rc.sw1 == 3 && rc.sw2 == 1) || Flag_status.moca_flag == 3)
        {
            Flag_status.moca_flag = 3;
        }
        if((rc.sw1 == 2 && rc.sw2 == 2) || Flag_status.Ammunition_flag == 4)
        {
            Flag_status.Ammunition_flag = 4;
            actShoot_last = actShoot;
            actShoot = SHOOT_NORMAL;//��ͨģʽ
				}
        if((rc.sw1 == 2 && rc.sw2 == 3) || Flag_status.Ammunition_flag == 5)
        {
            Flag_status.Ammunition_flag = 5;
            actShoot_last = actShoot;
					
					
					
					if(rc.wheel>500&&Flag_status.rc_wheel_flag==1)
					{
						Flag_status.rc_shoot_flag = 1;
						Flag_status.rc_wheel_flag = 0;
						actShoot=SHOOT_SINGLE_DM;
					}
					else if(rc.wheel<100&&rc.wheel>-100)
					{
						actShoot = SHOOT_NORMAL;
						Flag_status.rc_wheel_flag = 1;
					}
					
					

            Flag_status.shoot_cnt = 10;
        }
        if((rc.sw1 == 2 && rc.sw2 == 1))  //�л��ɼ���ģʽ
        {
			      actShoot_last = actShoot;
            actShoot = SHOOT_NORMAL;//��ͨģʽ
			      Flag_status.Ammunition_flag = 0;
			      Flag_status.moca_flag = 0;
            ControlMode = KEYBOARD;
			      SystemValue = Starting;
        }
       
		
		}

}

float speed_42mm = 0;

static void SendJudgeMsg(void)
{
	static uint8_t judge_step = 0;
	int16_t  judge_status = 0;//����ϵͳ�����Ƿ����
	float speed_42mm = 0;
		
	if(++judge_step == 8)
	{	
		/*��ȡ���ٵȼ�*/
//		speed_limit = JUDGE_usGetSpeedLimit();
		/*��ȡ��ǰ����*/
		speed_42mm = JUDGE_usGetSpeed42();
		/*����ϵͳ�����Ƿ����*/
		judge_status = JUDGE_sGetDataState();

		set_shoot_speed(&hcan2, judge_status, Speed_limit,speed_42mm);
		judge_step = 0;
	}

}

/**
  * @brief  ���ݲ���ϵͳ����ֵȷ����ǰ����
  * @param  void
  * @retval void
  * @attention  void
10:5500
  */
//int shoot_speed_adjust = 0;
int def = 0;
uint16_t rate=0;

void  Judge_Speed(void)
{


		 rate = 5800;

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
}

