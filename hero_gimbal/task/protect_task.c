#include "protect_task.h"
//#include "iwdg.h"
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
        if(!state_judge)	//遥控器失联时间过长,或者数据出错
        {
								vTaskSuspend(DMGimbalTaskHandle);		//将任务挂起
//								vTaskSuspend(GimbalTaskHandle);						
								vTaskSuspend(ShootTaskHandle);
								vTaskResume(OutControlTaskHandle);//解挂失控保护控制任务
				
					Cloud_Angle_Target[PITCH][GYRO] = 0;
					mocalun_l.motor_value->target_speed_rpm=0;
					mocalun_r.motor_value->target_speed_rpm=0;
					PID_clear(&Gimbal_MotorPitch.Motor_PID_Position);
					PID_clear(&Gimbal_MotorPitch.Motor_PID_Speed);
					PID_clear(&Gimbal_DM_MotorPitch.Motor_PID_Position);
					PID_clear(&Gimbal_DM_MotorPitch.Motor_PID_Speed);
					set_moto1234_current(&hcan1, 0, 0, 0, 0);
					set_moto5678_current(&hcan1, 0, 0, 0, 0);
					MIT_CtrlMotor(&hcan1,0x103,  0,  0, 0, 0, 0);
					Gimbal_DM_MotorPitch.motor_value->hall = 0;
					memset(&VisionValue, 0, sizeof(VisionValue));
					SystemValue = Starting; //系统恢复至重启状态
					ShootState = UNSTART;
					actGimbal = GIMBAL_NORMAL;
					vision_mode = aNORMAL;
					Flag_status.Chassis_Key_G_Change = 0;
					Flag_status.Gimbal_Key_Ctrl_Change = 0;
					gimbal_hanging = GIMBAL_VERTICAL;

        }
        else
        {
            flag=1;
//            vTaskResume(GimbalTaskHandle);					
            vTaskResume(DMGimbalTaskHandle);
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
		    actGimbal = GIMBAL_NORMAL;
        REMOTE_vResetData();//遥控数据恢复至默认状态
		    vision_mode = aNORMAL;
        Flag_status.Chassis_Key_G_Change = 0;
        //关闭所有电机输出
			mocalun_l.motor_value->target_speed_rpm=0;
					mocalun_r.motor_value->target_speed_rpm=0;
        set_moto1234_current(&hcan1, 0, 0, 0, 0);
        set_moto5678_current(&hcan1, 0, 0, 0, 0);
				MIT_CtrlMotor(&hcan1,0x103,  0,  0, 0, 0, 0);

    }
}

