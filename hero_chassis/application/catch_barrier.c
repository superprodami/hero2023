//#include "catch_barrier.h"


//float servo_v[2] = {0}; //���ÿ���ж���ת�Ƕ� 
//uint16_t servo_count[2]={0};  //�����תʱ���жϷ�--10msΪһ����λ ��תĿ��ʱ��Ϊdt*n
//float servoDeg[2]={50000,250000};  //ǰһ���Ƕ� ��ʼ��Ϊ��ʼ�̶��Ƕ� �Ƕȷ�Χ��500-2500 *100  ��Ӧ0-180��
//ServoState servo_state;

//void setAngle(uint8_t channel,uint16_t val,uint16_t t)
//{
//	servo_count[channel]=t;  //�����ת����
//	servo_v[channel]=(val*100-servoDeg[channel])/t;  //ÿ��Ҫ����ĽǶ�
//}

//void servo_control(void)
//{
//	for(int i=0;i<2;i++){
//	  if(servo_count[i]>=1){	
//		servoDeg[i]+=servo_v[i];
//		servo_count[i]--;
//	  }
//	}
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,(uint16_t)(servoDeg[0]/100));//���Ƕ�ֵд��ȽϼĴ���
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,(uint16_t)(servoDeg[1]/100));//���Ƕ�ֵд��ȽϼĴ���
//}

//void servo_mode_choose(void)
//{
//    //////////////////R�л������Ƿ��� ctrl+R����צ��/////////////////////	
//	if(IF_KEY_PRESSED_CTRL)
//	{
//		if(IF_KEY_PRESSED_R)
//		{
//			servo_state.servo_status = PREPARE;
//			Flag_status.Chassis_Key_R_Change = 0;
//			setAngle(0,500,50);
//			setAngle(1,2500,50);
//		}
//	}
//	else
//	{
//		if(!IF_KEY_PRESSED_R)
//		{
//			Flag_status.Chassis_Switch_R = 1;
//		}
//		if(IF_KEY_PRESSED_R && Flag_status.Chassis_Switch_R == 1)
//		{
//			Flag_status.Chassis_Switch_R = 0;
//			Flag_status.Chassis_Key_R_Change ++;
//			Flag_status.Chassis_Key_R_Change %= 2;
//			
//			if(Flag_status.Chassis_Key_R_Change)
//			{
//				servo_state.servo_status = WORK_DOWN;
//				setAngle(0,1500,50);
//				setAngle(1,1500,50);
//			}else{
//				servo_state.servo_status = WORK_UP;
//				setAngle(0,800,35);
//				setAngle(1,2200,35);
//			}
//		}	
//	}

//}












////ServoType ServoValue_L;
////ServoType ServoValue_R;
////ServoState servo_state;

/////*
//// *---------------CANЭ��-----------------
//// *�Ƽ�����Ƶ���Ƽ�0.5hz 
//// *���ձ�ʶ��0x230
//// * ���Ƕ�ģʽ
//// * �Ƕȷ�Χ��0<=angle<=360
//// *[0]0xAA   -----֡��Ч��
//// *[1]ID     -----Ŀ����ID����0XFFΪ���������ж��
//// *[2] �Ƕ�������float[0]
//// *[3] �Ƕ�������float[1]
//// *[4] �Ƕ�������float[2]
//// *[5] �Ƕ�������float[3]
//// *[6]0xFA   -----���Ƕȱ�
//// *[7]0xBB   -----֡��Ч��
//// */
////void set_servo_angle(CAN_HandleTypeDef* hcan, ServoType * servo_value)
////{
////	CAN_TxHeaderTypeDef _TxHeader;
////	uint8_t Txtemp[8];
////	_TxHeader.StdId = 0x231;
////	_TxHeader.IDE = CAN_ID_STD;
////	_TxHeader.RTR = CAN_RTR_DATA;
////	_TxHeader.DLC = 0x08;
////	Txtemp[0] = 0xAA;
////	Txtemp[1] = servo_value->servo_id;
////	Txtemp[2] = servo_value->servo_angle.buff[0];
////	Txtemp[3] = servo_value->servo_angle.buff[1];
////	Txtemp[4] = servo_value->servo_angle.buff[2];
////	Txtemp[5] = servo_value->servo_angle.buff[3];
////	Txtemp[6] = 0xFA;
////	Txtemp[7] = 0xBB;
////	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
////	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
////	{
////		
////	} 
////}

////void set_servo_angle1(CAN_HandleTypeDef* hcan, ServoType * servo_value)
////{
////	CAN_TxHeaderTypeDef _TxHeader;
////	uint8_t Txtemp[8];
////	_TxHeader.StdId = 0x231;
////	_TxHeader.IDE = CAN_ID_STD;
////	_TxHeader.RTR = CAN_RTR_DATA;
////	_TxHeader.DLC = 0x08;
////	Txtemp[0] = 0xAA;
////	Txtemp[1] = servo_value->servo_id;
////	Txtemp[2] = servo_value->servo_angle.buff[0];
////	Txtemp[3] = servo_value->servo_angle.buff[1];
////	Txtemp[4] = servo_value->servo_angle.buff[2];
////	Txtemp[5] = servo_value->servo_angle.buff[3];
////	Txtemp[6] = 0xFA;
////	Txtemp[7] = 0xBB;
////	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
////	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
////	{
////		
////	} 
////}

/////*
//// *�Ƽ�����Ƶ���Ƽ�0.5hz 
//// *���ձ�ʶ��0x230
//// * ���ٶ�ģʽ
//// * �ٶȷ�Χ��-100<=angle<=100
//// *[0]0xAA   -----֡��Ч��
//// *[1]ID     -----Ŀ����ID����0XFFΪ���������ж��
//// *[2] �ٶ�������float[0]
//// *[3] �ٶ�������float[1]
//// *[4] �ٶ�������float[2]
//// *[5] �ٶ�������float[3]
//// *[6]0xFB   -----���ٶȱ�
//// *[7]0xBB   -----֡��Ч��
////*/
////void set_servo_speed(CAN_HandleTypeDef* hcan, ServoType * servo_value)
////{
////	CAN_TxHeaderTypeDef _TxHeader;
////	uint8_t Txtemp[8];
////	_TxHeader.StdId = 0x231;
////	_TxHeader.IDE = CAN_ID_STD;
////	_TxHeader.RTR = CAN_RTR_DATA;
////	_TxHeader.DLC = 0x08;
////	Txtemp[0] = 0xAA;
////	Txtemp[1] = servo_value->servo_id;
////	Txtemp[2] = servo_value->servo_speed.buff[0];
////	Txtemp[3] = servo_value->servo_speed.buff[1];
////	Txtemp[4] = servo_value->servo_speed.buff[2];
////	Txtemp[5] = servo_value->servo_speed.buff[3];
////	Txtemp[6] = 0xFB;
////	Txtemp[7] = 0xBB;
////	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
////	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
////	{
////		
////	} 
////}

////float aaspeed = 0;
////void servo_send(void)
////{
////	static uint32_t cnt;
////	cnt++;
////	ServoValue_L.servo_speed.value = aaspeed;
////	ServoValue_R.servo_speed.value = aaspeed;
////	if(cnt % 40 == 0)
////	{
////		set_servo_speed(&hcan2,&ServoValue_R);
//////		set_servo_speed(&hcan2,&ServoValue_L);	//��ʱ�����ӽǶ�--̧��Ƕ�����
////		cnt = 0;
////	}

////}

////void servo_init(void)
////{
////	servo_state.servo_status = PREPARE;
////	servo_state.servo_pre_left = 0;
////	servo_state.servo_pre_right = 0;
////	servo_state.servo_set_left = 0;
////	servo_state.servo_set_right = 0;
////	ServoValue_L.servo_id = 0x04;
////	ServoValue_R.servo_id = 0x01;
////	ServoValue_L.servo_angle.value = 220;
////	ServoValue_R.servo_angle.value = 0;
////}
