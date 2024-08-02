#include "vision.h"
#include "string.h"
#include "usbd_cdc_if.h"

VisionValueType VisionValue;
Vision_Mode vision_mode;
uint8_t vision_buf[VISION_BUFLEN];
uint8_t Vision_Send_Buff[VISION_BUFLEN];

uint16_t identify[30],identify_sum;
uint16_t identify_flag;
float yaw_value_last=0;
volatile int8_t yaw_vision_flag=1;
volatile int8_t yaw_vision_value=0;
uint32_t vision_time = 0;

fp32 imuvision[3]= {0.0f, 0.0f, 0.0f};
uint32_t  *imuvisionaddr[3];

SmallGyroType Small_Gyro;

void Small_gyro_init(SmallGyroType *small_gyro,uint16_t small_gyro_timemax,float distance_max,float near_max,uint8_t is_count_max,float quit_value,float T_Q,float T_R)
{
	small_gyro->SmallGyro_init.small_gyro_timemax=small_gyro_timemax;
	small_gyro->SmallGyro_init.distance_max=distance_max;
	small_gyro->SmallGyro_init.near_max=near_max;
	small_gyro->SmallGyro_init.is_count_max=is_count_max;
	small_gyro->SmallGyro_init.quit_value=quit_value;
	small_gyro->SmallGyro_init.frequency_gyro_kalman.X_last=0.f;
	
    small_gyro->SmallGyro_init.frequency_gyro_kalman.P_last = 0;
    small_gyro->SmallGyro_init.frequency_gyro_kalman.Q = T_Q;
    small_gyro->SmallGyro_init.frequency_gyro_kalman.R = T_R;
    small_gyro->SmallGyro_init.frequency_gyro_kalman.A = 1;
    small_gyro->SmallGyro_init.frequency_gyro_kalman.B = 0;
    small_gyro->SmallGyro_init.frequency_gyro_kalman.H = 1;
    small_gyro->SmallGyro_init.frequency_gyro_kalman.X_mid =0.f;
	
	small_gyro->SmallGyro_out.is_small_gyro = 0;
    small_gyro->SmallGyro_out.last_dir = -1;
	small_gyro->SmallGyro_out.time1 = 0;
	small_gyro->SmallGyro_out.Small_gyro_near = 1;
	small_gyro->SmallGyro_out.Frequency_gyro = 0;
	small_gyro->SmallGyro_out.small_gyro_aimed = 0;
}
void SMALL_Gyro_Judge(SmallGyroType *small_gyro,VisionValueType *visionvalue)
{
		//根据距离判断倍率
		if(visionvalue->vision_distance.value < small_gyro->SmallGyro_init.distance_max && visionvalue->vision_distance.value > 0.1f)
		{
			small_gyro->SmallGyro_out.Small_gyro_near=(small_gyro->SmallGyro_init.distance_max-visionvalue->vision_distance.value)
																								*(small_gyro->SmallGyro_init.near_max-1) / small_gyro->SmallGyro_init.distance_max + 1;
		}
		else 
			small_gyro->SmallGyro_out.Small_gyro_near=1;
		
		
		if( small_gyro->SmallGyro_out.last_dir*visionvalue->vision_yaw_value.value< 0
			&& fabs(visionvalue->vision_yaw_value.value)> 0.1f *small_gyro->SmallGyro_out.Small_gyro_near)
		{
			if(small_gyro->SmallGyro_out.time1 > 0)
				small_gyro->SmallGyro_out.Frequency_gyro = KalmanFilter(&small_gyro->SmallGyro_init.frequency_gyro_kalman,500.f/small_gyro->SmallGyro_out.time1);
			if(small_gyro->SmallGyro_out.Frequency_gyro > 9)
				small_gyro->SmallGyro_out.Frequency_gyro = 9;
			small_gyro->SmallGyro_out.time1 = 0;
			small_gyro->SmallGyro_out.last_dir *= -1;
			small_gyro->SmallGyro_out.is_small_gyro++;
			small_gyro->SmallGyro_out.time2 = xTaskGetTickCount();
		}
		else
		{
			small_gyro->SmallGyro_out.time1 = xTaskGetTickCount() - small_gyro->SmallGyro_out.time2;//通过系统时钟计算频率
		}

		if(fabs(visionvalue->normal_yaw_value.value) > small_gyro->SmallGyro_init.quit_value / small_gyro->SmallGyro_out.Small_gyro_near)
		{//normal_yaw_value为无预测的视觉偏移量，此处是为了防止预测值的突增
			small_gyro->SmallGyro_out.time1 = 0;
			small_gyro->SmallGyro_out.is_small_gyro = 0;
			small_gyro->SmallGyro_debug.timeout_flag = 0;
			small_gyro->SmallGyro_debug.valueout_flag = 1;
		}
		if(small_gyro->SmallGyro_out.time1 > small_gyro->SmallGyro_init.small_gyro_timemax * small_gyro->SmallGyro_out.Small_gyro_near)
		{
			small_gyro->SmallGyro_out.time1 = 0;
			small_gyro->SmallGyro_out.is_small_gyro = 0;
			small_gyro->SmallGyro_debug.timeout_flag = 1;
			small_gyro->SmallGyro_debug.valueout_flag = 0;
		}

		if(small_gyro->SmallGyro_out.is_small_gyro > small_gyro->SmallGyro_init.is_count_max)
		{
			small_gyro->SmallGyro_out.small_gyro_aimed = 1;
		}
		else
		{
			small_gyro->SmallGyro_out.small_gyro_aimed = 0;
		}
}

void vision_callback_handler(uint8_t *buff)//0-31
{ 
    if(buff[0]== 's' &&buff[31]=='e')
	{
		vision_time++;
		identify[identify_flag] = (int)buff[1];
		if(identify_flag == 14)
			 identify_flag = 0;
		else
			 identify_flag ++;
		for(int j = 0;j < 15;j++){
			   identify_sum += identify[j];
		}
		if(identify_sum == 720)
		   VisionValue.identify_target  = '0';
		else
			 VisionValue.identify_target  = '1';
		identify_sum = 0;
		VisionValue.center_flag = buff[2]; //是否在中心
		memcpy(&(VisionValue.vision_yaw_value.buff[0]),&buff[3],4);  //yaw轴偏移量
		memcpy(&(VisionValue.vision_pitch_value.buff[0]),&buff[7],4);  //pitch轴偏移量
		memcpy(&(VisionValue.normal_yaw_value.buff[0]),&buff[11],4);//无预测yaw目标值
		memcpy(&(VisionValue.normal_pitch_value.buff[0]),&buff[15],4);//无预测yaw目标值
		memcpy(&(VisionValue.vision_distance.buff[0]),&buff[19],4);  //距离
	}
		vision_message_send0(&hcan2);
		vision_message_send1(&hcan2);
		vision_message_send2(&hcan2);

}

void vision_message_send0(CAN_HandleTypeDef* hcan)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x332;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	
	Txtemp[0] = 0xa0;
	Txtemp[1] = VisionValue.vision_yaw_value.buff[0];
	Txtemp[2] = VisionValue.vision_yaw_value.buff[1];
	Txtemp[3] = VisionValue.vision_yaw_value.buff[2];
	Txtemp[4] = VisionValue.vision_yaw_value.buff[3];

	
	Txtemp[5] = VisionValue.center_flag;
	Txtemp[6] = VisionValue.identify_target;

	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{

	}

}

void vision_message_send1(CAN_HandleTypeDef* hcan)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x332;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	
	Txtemp[0] = 0xa1;
	Txtemp[1] = VisionValue.vision_distance.buff[0];
	Txtemp[2] = VisionValue.vision_distance.buff[1];
	Txtemp[3] = VisionValue.vision_distance.buff[2];
	Txtemp[4] = VisionValue.vision_distance.buff[3];

	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{

	}

}

void vision_message_send2(CAN_HandleTypeDef* hcan)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x332;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	
	Txtemp[0] = 0xa2;
	Txtemp[1] = VisionValue.normal_yaw_value.buff[0];
	Txtemp[2] = VisionValue.normal_yaw_value.buff[1];
	Txtemp[3] = VisionValue.normal_yaw_value.buff[2];
	Txtemp[4] = VisionValue.normal_yaw_value.buff[3];

	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{

	}

}
float a45f0 = 15;
void vision_send(void)
{
	Vision_Send_Buff[0] = 's';
	if(JudgeValue.color_judge == 0)
		Vision_Send_Buff[1] = 'b';
	else
		Vision_Send_Buff[1] = 'r';
	Vision_Send_Buff[2] = 'a';
	if(JudgeValue.speed_42mm.value > 0)
	{
		Vision_Send_Buff[3] = JudgeValue.speed_42mm.buff[0];
		Vision_Send_Buff[4] = JudgeValue.speed_42mm.buff[1];
		Vision_Send_Buff[5] = JudgeValue.speed_42mm.buff[2];
		Vision_Send_Buff[6] = JudgeValue.speed_42mm.buff[3];
	}
	else
	{
		JudgeValue.speed_42mm_vision.value = a45f0;
		Vision_Send_Buff[3] = JudgeValue.speed_42mm_vision.buff[0];
		Vision_Send_Buff[4] = JudgeValue.speed_42mm_vision.buff[1];
		Vision_Send_Buff[5] = JudgeValue.speed_42mm_vision.buff[2];
		Vision_Send_Buff[6] = JudgeValue.speed_42mm_vision.buff[3];
	}

//	if(vision_mode == 0)
//		Vision_Send_Buff[7] = '0';
//	else
//		Vision_Send_Buff[7] = 'h';
	VisionValue.imuvisionyaw.value=INS_angle[0];
	VisionValue.imuvisionpitch.value=INS_angle[1];
	VisionValue.imuvisionroll.value=INS_angle[2];

	Vision_Send_Buff[7]=VisionValue.imuvisionyaw.buff[0];
	Vision_Send_Buff[8]=VisionValue.imuvisionyaw.buff[1];
	Vision_Send_Buff[9]=VisionValue.imuvisionyaw.buff[2];
	Vision_Send_Buff[10]=VisionValue.imuvisionyaw.buff[3];
	Vision_Send_Buff[15]=VisionValue.imuvisionpitch.buff[0];
	Vision_Send_Buff[16]=VisionValue.imuvisionpitch.buff[1];
	Vision_Send_Buff[17]=VisionValue.imuvisionpitch.buff[2];
	Vision_Send_Buff[18]=VisionValue.imuvisionpitch.buff[3];
	Vision_Send_Buff[11]=VisionValue.imuvisionroll.buff[0];
	Vision_Send_Buff[12]=VisionValue.imuvisionroll.buff[1];
	Vision_Send_Buff[13]=VisionValue.imuvisionroll.buff[2];
	Vision_Send_Buff[14]=VisionValue.imuvisionroll.buff[3];
//	imuvision [0]=IMU_angle[0];
//	imuvision [1]=IMU_angle[1];
//	imuvision [2]=IMU_angle[2];
//  imuvisionaddr[0]=(uint32_t *)&imuvision[0];
//	imuvisionaddr[1]=(uint32_t *)&imuvision[1];
//	imuvisionaddr[2]=(uint32_t *)&imuvision[2];
//	
//	Vision_Send_Buff [7]=GET_BIT0(*imuvisionaddr[0]);
//	Vision_Send_Buff [8]=GET_BIT1(*imuvisionaddr[0]);
//	Vision_Send_Buff [9]=GET_BIT2(*imuvisionaddr[0]);
//	Vision_Send_Buff [10]=GET_BIT3(*imuvisionaddr[0]);

//	Vision_Send_Buff [11]=GET_BIT0(*imuvisionaddr[2]);
//	Vision_Send_Buff [12]=GET_BIT1(*imuvisionaddr[2]);
//	Vision_Send_Buff [13]=GET_BIT2(*imuvisionaddr[2]);
//	Vision_Send_Buff [14]=GET_BIT3(*imuvisionaddr[2]);

//	Vision_Send_Buff [15]=GET_BIT0(*imuvisionaddr[1]);
//	Vision_Send_Buff [16]=GET_BIT1(*imuvisionaddr[1]);
//	Vision_Send_Buff [17]=GET_BIT2(*imuvisionaddr[1]);
//	Vision_Send_Buff [18]=GET_BIT3(*imuvisionaddr[1]);

//	for(int i = 8; i < 31; i++)
//		Vision_Send_Buff[i] = 0;
	Vision_Send_Buff[31] = 'e';
	
	CDC_Transmit_FS(Vision_Send_Buff,32);
}
