//#include "catch_barrier.h"


//float servo_v[2] = {0}; //舵机每次中断旋转角度 
//uint16_t servo_count[2]={0};  //舵机旋转时间判断符--10ms为一个单位 旋转目标时间为dt*n
//float servoDeg[2]={50000,250000};  //前一个角度 初始化为初始固定角度 角度范围：500-2500 *100  对应0-180°
//ServoState servo_state;

//void setAngle(uint8_t channel,uint16_t val,uint16_t t)
//{
//	servo_count[channel]=t;  //舵机旋转配速
//	servo_v[channel]=(val*100-servoDeg[channel])/t;  //每次要到达的角度
//}

//void servo_control(void)
//{
//	for(int i=0;i<2;i++){
//	  if(servo_count[i]>=1){	
//		servoDeg[i]+=servo_v[i];
//		servo_count[i]--;
//	  }
//	}
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,(uint16_t)(servoDeg[0]/100));//将角度值写入比较寄存器
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,(uint16_t)(servoDeg[1]/100));//将角度值写入比较寄存器
//}

//void servo_mode_choose(void)
//{
//    //////////////////R切换拿起还是放下 ctrl+R收起爪子/////////////////////	
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
//// *---------------CAN协议-----------------
//// *推荐发送频率推荐0.5hz 
//// *接收标识符0x230
//// * 定角度模式
//// * 角度范围：0<=angle<=360
//// *[0]0xAA   -----帧有效性
//// *[1]ID     -----目标舵机ID，发0XFF为总线上所有舵机
//// *[2] 角度联合体float[0]
//// *[3] 角度联合体float[1]
//// *[4] 角度联合体float[2]
//// *[5] 角度联合体float[3]
//// *[6]0xFA   -----定角度标
//// *[7]0xBB   -----帧有效性
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
//// *推荐发送频率推荐0.5hz 
//// *接收标识符0x230
//// * 定速度模式
//// * 速度范围：-100<=angle<=100
//// *[0]0xAA   -----帧有效性
//// *[1]ID     -----目标舵机ID，发0XFF为总线上所有舵机
//// *[2] 速度联合体float[0]
//// *[3] 速度联合体float[1]
//// *[4] 速度联合体float[2]
//// *[5] 速度联合体float[3]
//// *[6]0xFB   -----定速度标
//// *[7]0xBB   -----帧有效性
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
//////		set_servo_speed(&hcan2,&ServoValue_L);	//逆时针增加角度--抬起角度增加
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
