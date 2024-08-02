#include "protect_task.h"
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
        if(currentTime > REMOTE_ulGetLostTime() || REMOTE_IfDataError() == true)	//ң����ʧ��ʱ�����,�������ݳ���
        {
			    state_judge = 0;

          vTaskSuspend(ChassisTaskHandle);
			    vTaskSuspend(GimbalTaskHandle);
			    vTaskSuspend(ShootTaskHandle);
          vTaskResume(OutControlTaskHandle);//���ʧ�ر�����������

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
//					SystemValue = Starting; //ϵͳ�ָ�������״̬
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
        REMOTE_vResetData();//ң�����ݻ�;����Ĭ��״̬
		    vision_mode = aNORMAL;
			
//								ControlMode=KEYBOARD ;

			
//        Flag_status.Chassis_Key_G_Change = 0;

        //�ر����е�����
        set_moto1234_current(&hcan1, 0, 0, 0, 0);
//        set_moto1234_current(&hcan2, 0, 0, 0, 0);
        set_moto5678_current(&hcan1, 0, 0, 0, 0);
//        set_moto5678_current(&hcan2, 0, 0, 0, 0);
    }
}

///**
//  * @brief  ϵͳ�����λ���൱�ڰ�reset
//  * @param  void
//  * @retval void
//  * @attention void
//  */
//void soft_rest(void)
//{
//    __set_FAULTMASK(1); //�ر������ж�
//    NVIC_SystemReset(); //��λ
//}

///**
//  * @brief  �������λǰ��Ϊ�˰�ȫ��ֹͣ���е��
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
