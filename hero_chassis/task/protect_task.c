#include "protect_task.h"
#include "Motor_DM.h"

/**
  * @brief  检测程序正常运行
  * @param  void
  * @retval void
  * @attention
  */
bool flag=1;
void ShowRunningFun(void const *argument)
{
    portTickType currentTime;
    for(;;)
    {
        currentTime = xTaskGetTickCount();//当前系统时间

#if (SAFE_MODE==1)
        if(currentTime > REMOTE_ulGetLostTime() || REMOTE_IfDataError() == true)	//遥控器失联时间过长,或者数据出错
        {
			    state_judge = 0;

          vTaskSuspend(ChassisTaskHandle);
			    vTaskSuspend(GimbalTaskHandle);
			    vTaskSuspend(ShootTaskHandle);
          vTaskResume(OutControlTaskHandle);//解挂失控保护控制任务

					PID_clear(&Gimbal_MotorImuYaw.Motor_PID_Position);
					PID_clear(&Gimbal_MotorImuYaw.Motor_PID_Speed);	
					PID_clear(&Ammunition_DM_Motor.Motor_PID_Position);
					PID_clear(&Ammunition_DM_Motor.Motor_PID_Speed);		


					set_moto1234_current(&hcan1, 0, 0, 0, 0);
					set_moto5678_current(&hcan1, 0, 0, 0, 0);
					
					MIT_CtrlMotor(&hcan1,0x102,  0,  0, 0, 0, 0);
					set_cap0(&hcan2, 0xbb, 0x01, 0);
					set_cap1(&hcan2, 0xbb, 0x01, 0);
					memset(&VisionValue, 0, sizeof(VisionValue));
//					SystemValue = Starting; //系统恢复至重启状态
					ShootState = UNSTART;
					actGimbal = GIMBAL_NORMAL;
					vision_mode = aNORMAL;
					
//					ControlMode=KEYBOARD ;
					
					Flag_status.Chassis_Key_G_Change = 0;
					Flag_status.Gimbal_Key_Ctrl_Change = 0;
					gimbal_hanging = GIMBAL_VERTICAL;
        }
        else
        {
					 flag=1;
					 state_judge = 1;
					 vTaskResume(ChassisTaskHandle);
					 vTaskResume(GimbalTaskHandle);
					 vTaskResume(ShootTaskHandle);
					 vTaskSuspend(OutControlTaskHandle);//挂起失控保护控制任务
        }
#endif
        vTaskDelayUntil(&currentTime, TIME_STAMP_20MS);//绝对延时
    }
}

/**
  * @brief  失控控制任务若解挂则每4ms执行一次，相对延迟
  * @param  void
  * @retval void
  * @attention void
  */
void OutControlFun(void *pvParameters)
{
    for(;;)
    {
        vTaskDelay(TIME_STAMP_4MS);				//4ms

        SystemValue = Starting; //系统恢复至重启状态
		    ShootState = UNSTART;
        REMOTE_vResetData();//遥控数据恢;复至默认状态
		    vision_mode = aNORMAL;
			
//								ControlMode=KEYBOARD ;

			
//        Flag_status.Chassis_Key_G_Change = 0;

        //关闭所有电机输出
        set_moto1234_current(&hcan1, 0, 0, 0, 0);
//        set_moto1234_current(&hcan2, 0, 0, 0, 0);
        set_moto5678_current(&hcan1, 0, 0, 0, 0);
//        set_moto5678_current(&hcan2, 0, 0, 0, 0);
    }
}

///**
//  * @brief  系统软件复位，相当于按reset
//  * @param  void
//  * @retval void
//  * @attention void
//  */
//void soft_rest(void)
//{
//    __set_FAULTMASK(1); //关闭所有中断
//    NVIC_SystemReset(); //复位
//}

///**
//  * @brief  在软件复位前，为了安全先停止所有电机
//  * @param  void
//  * @retval void
//  * @attention void
//  */
//void Stop_All(void)
//{
//    int i = 30;
//    while(i--)
//    {
//			set_moto5678_current(&hcan1,0,0,0,0);
//			set_moto1234_current(&hcan1,0,0,0,0);
//			osDelay(1);
//    }
//}
