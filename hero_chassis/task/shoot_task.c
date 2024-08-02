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
  * 邵钦傻比
  *
  ******************************************************************************
  */
#include "shoot_task.h"
//#include "param.h"
#include "SQ_judge.h"
#include "Motor_DM.h"
fp32 abstotalangle=0;  //上电后的绝对角度
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
        currentTime = xTaskGetTickCount(); //当前系统时间
			
			if(Ammunition_DM_Motor.motor_value->hall ==0)
			{	
				Motor_enable();					//电机使能	
				DM_enable_flag	= 1;			
//				Ammunition_DM_Motor.motor_value->target_position = Ammunition_DM_Motor.motor_value->position;
			}

	
        Limit_something(); //热量控制 卡弹控制

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
    Flag_status.protect_flag_heat = false;
    Flag_status.protect_flag_sutck = false;
    Flag_status.shoot_cnt = 0;
    Flag_status.shoot_left_time = 0;//计算左键连按时间,时间过长切换成连发
		Flag_status.shoot_single_finish_flag = 1;
		Flag_status.shoot_single_time = 0;



				Motor_Init_DM(&Ammunition_DM_Motor, 1,Ammunition_DM_Motor_Position_pid, PID_POSITION, 30, 20, 1e30, 0, 0.2, 0, 0 , //40,20,20,0,0.75,0,0,//  
											Ammunition_DM_Motor_Speed_pid, PID_POSITION, 10, 5, 3e38,0,0.1,0, 0);
				Ammunition_DM_Motor.Motor_PID_Position.flag_Slop =1;     //  限幅斜坡
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
  * @brief  键盘模式下发弹模式选择
  * @param  void
  * @retval void
  * @attention  普通模式清零计算,且不发弹
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
			if((IF_MOUSE_PRESSED_LEFT && Flag_status.shoot_left_time <= TIME_STAMP_500MS && actGimbal != GIMBAL_AUTO) //普通模式单击
		|| (actGimbal == GIMBAL_AUTO && (IF_MOUSE_PRESSED_LEFT ) && Flag_status.shoot_left_time <= TIME_STAMP_500MS)  //自瞄模式瞄中中心点后单击 && VisionValue.center_flag
		|| ((!Flag_status.shoot_single_finish_flag) && (Flag_status.shoot_single_time > 0))) //单击后未完成单发全部流程
			{
				if((!(Flag_status.protect_flag_sutck || Flag_status.protect_flag_heat)))
				{
					Flag_status.shoot_left_time++;//判断长按,切换
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
			
			else if(IF_MOUSE_PRESSED_LEFT && Flag_status.shoot_left_time > TIME_STAMP_500MS&&Flag_status.shoot_single_finish_flag)//连按大于200ms && actGimbal != GIMBAL_AUTO
			{
					Flag_status.shoot_left_time++;
					if(Flag_status.shoot_triple_flag)
					{
						actShoot_last = actShoot;
						Flag_status.shoot_triple_flag =0;
					}
					if(!(Flag_status.shoot_single_flag )) Flag_status.shoot_single_flag =1;
					if(!(Flag_status.shoot_normal_flag )) Flag_status.shoot_normal_flag =1;

					actShoot = SHOOT_TRIPLE;//连发模式

			}
	}
}

/************************底盘键盘模式各类模式小函数****************************/

/*******键盘模式************/
/**
  * @brief  单发控制
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
	static uint8_t judge_shoot_flag = 0;//裁判系统反馈是否打出弹
    if(single_shoot_flag)  //保证鼠标按下瞬间这段代码只执行一次
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
				Ammunition_DM_Motor.motor_value->target_angle += 18218.00f;//保证鼠标每按下一次产生的差值均为60度
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


uint16_t shoot_flag_dm=0,shoot_flag_dm_time=0;   //测试进入函数次数 


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
	
	if(IF_KEY_PRESSED_R)//无视热量 一键超频
	{
		Ammunition_DM_Motor.motor_value->target_velocity = 3;    //5000;
	}	
	else
		Ammunition_DM_Motor.motor_value->target_velocity = 0.7;     //100;
}

/**
  * @brief  拨盘的键盘模式
  * @param  void
  * @retval void
  * @attention 键盘用位置环控制
  */
static void REVOLVER_Key_Ctrl(void)
{
    /*------ 左键抬起后才能打下一颗 -------*/
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

/*********************射频热量限制****************************/

uint16_t judge_test_buff[5];




/**
  * @brief  枪管热量限制
  * @param  void
  * @retval 热量是否超限
  * @attention  超限要重置一下拨盘,根据剩余可发弹量来做闭环
  *             如果做双枪管则此函数不适用
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
    uint16_t  usHeatOneShoot = 100;  //裁判系统检测到打一颗，枪口热量加80，与其初速度无关 规则是一发100 不过考虑到送弹延迟可以适当些小一点
    uint16_t  usHeatLimit;

    /* 读取热量 */
    usHeatReal = JUDGE_usGetRemoteHeat42();
    /* 读取热量上限*/
    usHeatLimit = JUDGE_usGetHeatLimit();

    /* 读取射击次数 */
    usShootNumReal = JUDGE_usGetShootNum();

    /* 记录当前时间 */
    ulShootTimeRecordReal = xTaskGetTickCount();

		judge_test_buff[0] = usHeatReal;
		judge_test_buff[1] = usHeatLimit;
		judge_test_buff[2] = usShootNumReal;
		judge_test_buff[3] = ulShootTimeRecordReal;



    /* 只要过100MS枪口冷却值减 */
    if(ulShootTimeRecordReal > msShootTimeRecord)
    {
        msShootTimeRecord = ulShootTimeRecordReal + 100;
        usShootNumBuffer -= JUDGE_usGetShootCold() / 10;
    }

    /* 只要打了弹就增加 */
    if (usShootNumReal > usShootNumPrev)
    {
        usShootNumBuffer += 10;
    }

    /* 剩余热量 */
    if (usHeatReal <= usHeatLimit){
        usHeatBuffer = usHeatLimit - usHeatReal;
    } else{
        usHeatBuffer = 0;
    }

    if (usHeatBuffer >= usHeatOneShoot){  //剩余热量大于打一发所需热量
        /* 还能打出的子弹数目 */
        IfShootAllow = TRUE;
    }else{
        IfShootAllow = FALSE;
    }

    usShootNumPrev = usShootNumReal;
    if(IF_KEY_PRESSED_R)//无视热量 一键超频
	{
		IfShootAllow = TRUE;
	}
        return IfShootAllow;	
	

}

/*****************************卡弹处理**************************************/
///************卡弹************/
/**
  * @brief  速度环式卡弹处理
  * @param  void
  * @retval void
  * @attention  卡住就松力，过一会再重新上力
  */
static void REVOL_SpeedStuck(void)
{
    static uint16_t  stuck_time    = 0;//卡弹计时
    static uint16_t  relax_time = 0;//放松计时
    static bool Revol_Speed_ifStuck = FALSE;//卡弹判断

    if (Revol_Speed_ifStuck == TRUE)//已确认卡弹,开始放松计时
    {
			  if(Flag_status.stuck_flag)
				{
					actShoot_last = actShoot;
					Flag_status.stuck_flag =0;
				}
//        actShoot = RELAX;
        Flag_status.protect_flag_sutck = 1;   //本来是 1
        relax_time++;//放松一定时间

        if (relax_time > Stuck_Relax_Time )//放松完成
        {
            relax_time = 0;
            Flag_status.protect_flag_sutck = 0;
            Revol_Speed_ifStuck = FALSE;//可以正转					
        }
    }
    else//Stuck_Revol_PIDTerm
    {
        if ( fabs(Ammunition_DM_Motor.Motor_PID_Speed.out) >= Stuck_Revol_PIDTerm //PID输出过大
                && abs(Ammunition_DM_Motor.motor_value->velocity) <= Stuck_Speed_Low  &&
		            Ammunition_DM_Motor.motor_value->target_velocity > 0)//速度过低
        {
            stuck_time++;//卡弹计时
					if(stuck_time>=2000)
					{
						stuck_time =2000;
					}
        }
        else
        {
            stuck_time = 0;//没有长时间卡弹,及时清零
        }

        if (stuck_time > Stuck_SpeedPID_Time)//卡了超过60ms
        {
            stuck_time = 0;
            Revol_Speed_ifStuck = TRUE;//标记可以进入放松计时
        }
				Flag_status.stuck_flag =1;
        Flag_status.protect_flag_sutck = 0;
   } 
}
/**
  * @brief  防卡弹，防超热量，各种防
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
        Flag_status.protect_flag_heat = 1;  ///热量限制  本来应该是1
    }
    else
    {
        Flag_status.protect_flag_heat = 0;
			  Flag_status.heat_flag=1;
    }
    REVOL_SpeedStuck();
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
            actShoot = SHOOT_NORMAL;//普通模式
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
        if((rc.sw1 == 2 && rc.sw2 == 1))  //切换成键盘模式
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

float speed_42mm = 0;

static void SendJudgeMsg(void)
{
	static uint8_t judge_step = 0;
	int16_t  judge_status = 0;//裁判系统数据是否可用
	float speed_42mm = 0;
		
	if(++judge_step == 8)
	{	
		/*读取射速等级*/
//		speed_limit = JUDGE_usGetSpeedLimit();
		/*读取当前射速*/
		speed_42mm = JUDGE_usGetSpeed42();
		/*裁判系统数据是否可用*/
		judge_status = JUDGE_sGetDataState();

		set_shoot_speed(&hcan2, judge_status, Speed_limit,speed_42mm);
		judge_step = 0;
	}

}

/**
  * @brief  根据裁判系统返回值确定当前射速
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

