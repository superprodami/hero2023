#include "protect_task.h"
//#include "iwdg.h"
#include "Motor_DM.h"

/**
  * @brief  ��������������
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
        currentTime = xTaskGetTickCount();//��ǰϵͳʱ��

#if (SAFE_MODE==1)
        if(!state_judge)	//ң����ʧ��ʱ�����,�������ݳ���
        {
								vTaskSuspend(DMGimbalTaskHandle);		//���������
//								vTaskSuspend(GimbalTaskHandle);						
								vTaskSuspend(ShootTaskHandle);
								vTaskResume(OutControlTaskHandle);//���ʧ�ر�����������
				
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
					SystemValue = Starting; //ϵͳ�ָ�������״̬
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
            vTaskSuspend(OutControlTaskHandle);//����ʧ�ر�����������
        }
#endif
        vTaskDelayUntil(&currentTime, TIME_STAMP_20MS);//������ʱ
    }
}

/**
  * @brief  ʧ�ؿ��������������ÿ4msִ��һ�Σ�����ӳ�
  * @param  void
  * @retval void
  * @attention void
  */
void OutControlFun(void *pvParameters)
{
    for(;;)
    {
        vTaskDelay(TIME_STAMP_4MS);				//4ms

        SystemValue = Starting; //ϵͳ�ָ�������״̬
	     	ShootState = UNSTART;
		    actGimbal = GIMBAL_NORMAL;
        REMOTE_vResetData();//ң�����ݻָ���Ĭ��״̬
		    vision_mode = aNORMAL;
        Flag_status.Chassis_Key_G_Change = 0;
        //�ر����е�����
			mocalun_l.motor_value->target_speed_rpm=0;
					mocalun_r.motor_value->target_speed_rpm=0;
        set_moto1234_current(&hcan1, 0, 0, 0, 0);
        set_moto5678_current(&hcan1, 0, 0, 0, 0);
				MIT_CtrlMotor(&hcan1,0x103,  0,  0, 0, 0, 0);

    }
}

