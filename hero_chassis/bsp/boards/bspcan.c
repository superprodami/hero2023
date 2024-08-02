/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "can.h"
#include "bspcan.h"
#include "stm32f4xx_hal_can.h"
//#include "shoot_task.h"
#include "SQ_judge.h"
#include "Info_Transmit_task.h"
#include  "Motor_DM.h"
moto_measure_t moto_CAN[11] ={0};
moto_measure_t moto_CAN2[11] = {0};
moto_measure_t moto_CAN_DM[3] ={0};

GimbalValueType gimbal_value;


void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);
//void get_judge_info(uint8_t temp[]);



JudgeValueType JudgeValue;

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		sFilterConfig;

   sFilterConfig.FilterIdHigh         = 0x0000;
   sFilterConfig.FilterIdLow          = 0x0000;
   sFilterConfig.FilterMaskIdHigh     = 0x0000;
   sFilterConfig.FilterMaskIdLow      = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterBank=27;
   sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 0;
	if(HAL_CAN_ConfigFilter(_hcan, &sFilterConfig) != HAL_OK)
	{

	}

}

void my_can_filter1_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		sFilterConfig;

   sFilterConfig.FilterIdHigh         = 0x0000;
   sFilterConfig.FilterIdLow          = 0x0000;
   sFilterConfig.FilterMaskIdHigh     = 0x0000;
   sFilterConfig.FilterMaskIdLow      = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
   sFilterConfig.FilterBank=27;
   sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 0;
	if(HAL_CAN_ConfigFilter(_hcan, &sFilterConfig) != HAL_OK)
	{

	}

}


/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
uint8_t text;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{
	CAN_RxHeaderTypeDef _RxHeader;
	uint8_t temp[8];

	HAL_CAN_GetRxMessage(_hcan,CAN_RX_FIFO0,&_RxHeader,temp);
	if(_RxHeader.StdId == CAN_Moto6_ID)
	{
				if(_hcan==&hcan1)
					get_moto_measure(&moto_CAN[5],temp);
				else
					get_moto_measure(&moto_CAN2[5], temp);
				
				return;
	}

	//ignore can1 or can2.
	switch(_RxHeader.StdId){
		case CAN_Moto1_ID:
		case CAN_Moto2_ID:
		case CAN_Moto3_ID:
		case CAN_Moto4_ID:
		case CAN_Moto5_ID:
		case CAN_Moto6_ID:
		case CAN_Moto7_ID:
		case CAN_Moto8_ID:
		case CAN_Moto9_ID:
		case CAN_Moto10_ID:
		case CAN_Moto11_ID:
			{
				u8 i;
				i = _RxHeader.StdId - CAN_Moto1_ID;
				if(_hcan==&hcan1)
					get_moto_measure(&moto_CAN[i],temp);
				else
					get_moto_measure(&moto_CAN2[i], temp);
			}
			break;
		case CAN_DM_Moto1_ID:
					get_dm_moto_measure(&moto_CAN_DM[0],temp);		
			break;		
		case yaw_id:
			get_yaw_info(temp);
			break;
		case cap_id:
			get_cap_info(temp);
			break;
		case vision_id:
			get_vision_info(temp);
			break;
	}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/

	  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}




bool angle_dm = 0, hall_dm = 0;

void get_dm_moto_measure(moto_measure_t *ptr, uint8_t temp[])
{
	float  	real_current_t;
	uint8_t  	hall_t;
	int 	angle_t,speed_rpm_t,t_int;
	
	angle_t=(temp[1]<<8)|temp[2];
	speed_rpm_t=(temp[3]<<4)|(temp[4]>>4);
	t_int=((temp[4]&0xF)<<8)|temp[5];
	hall_t=(float)(temp[7]);
	ptr ->id =temp[0] & 0xF;
  ptr->dm_error = temp[0] >> 4;

	ptr->position = uint_to_float(angle_t, P_MIN, P_MAX, 16); // (-12.5,12.5)
	ptr->velocity = uint_to_float(speed_rpm_t, V_MIN, V_MAX, 12); // (-45.0,45.0)
	ptr->torque = uint_to_float(t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
	ptr->hall = hall_t;
	
		if(ptr->position == 0)
			angle_dm = 1;
  if(ptr->hall == 0)
		hall_dm = 1;
	
	get_total_angle(ptr);
	ptr->RX_MSG.RX_add++;	
	get_total_angle_dm(ptr);

	abstotalangleinit=1;   //接收到拨盘电机的信号，可以开始赋值
	
	ptr->RX_MSG.RX_add++;
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param
  * @Retval		None
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr,uint8_t temp[])
{
	int16_t	 	speed_rpm_t;
	float  	real_current_t;
	uint8_t  	hall_t;
	uint16_t 	angle_t;
	angle_t = (uint16_t)(temp[0]<<8 | temp[1]);
	speed_rpm_t  = (int16_t)(temp[2]<<8 | temp[3]);
	real_current_t = (temp[4]<<8 | temp[5])*5.f/16384.f;
	hall_t = temp[6];
	
	if(hall_t==0 && real_current_t==0 && speed_rpm_t==0 && angle_t==0)
		return ;
//	ptr->last_angle = ptr->angle;
	ptr->angle = angle_t;
	ptr->speed_rpm  = speed_rpm_t;
	ptr->real_current = real_current_t;
	ptr->hall = hall_t;
	
	get_total_angle(ptr);
	
	ptr->RX_MSG.RX_add++;
	
}


/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = 0;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){
	
	if(p->last_angle < 2000 && p->angle > 6200)
		p->round_cnt--;
	else if(p->last_angle > 6200 && p->angle < 2000)
		p->round_cnt++;
	
	p->total_angle=p->angle + p->round_cnt*8192;
	p->last_angle = p->angle;
}

void get_total_angle_dm(moto_measure_t *p){
	
//	while (p->position < _2PI) 
//		{
//			p->position += _2PI;
//		}
//    while (p->position >= _2PI)
//		{
//			p->position -= _2PI;
//		}
		
	if(p->last_angle < _PI_2 && p->position > (_PI+_PI_2))
		p->round_cnt--;
	else if(p->last_angle > (_PI+_PI_2) && p->position < _PI_2)
		p->round_cnt++;
	
	p->total_angle=p->position + p->round_cnt*_2PI;
	p->last_angle = p->position;
}


//底盘四个电机
void set_moto1234_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x200;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (iq1 >> 8);
	Txtemp[1] = iq1;
	Txtemp[2] = (iq2 >> 8);
	Txtemp[3] = iq2;
	Txtemp[4] = iq3 >> 8;
	Txtemp[5] = iq3;
	Txtemp[6] = iq4 >> 8;
	Txtemp[7] = iq4;
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	  {
		
	  }

}
//5yaw轴；6拨盘；
void set_moto5678_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x1FF;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (iq1 >> 8);
	Txtemp[1] = iq1;
	Txtemp[2] = (iq2 >> 8);
	Txtemp[3] = iq2;
	Txtemp[4] = iq3 >> 8;
	Txtemp[5] = iq3;
	Txtemp[6] = iq4 >> 8;
	Txtemp[7] = iq4;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0);
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	  {

	  }

}



/*******************************************************************************************
  * @Func	  set_heat(CAN_HandleTypeDef* hcan, s8 judge_status, s16 heat_real,s16 usHeatLimit, s16 shoot_num_real )
  * @Brief    发送数据给云台
  * @Param    裁判系统状态  枪口热量  枪口热量上限  射击次数
  * @Retval		None
  * @Date     2021/10/30
 *******************************************************************************************/
void set_heat(CAN_HandleTypeDef* hcan, s8 judge_status, s16 heat_real,s16 heat_limit, s16 shoot_num_real )
{ 
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x320;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = judge_status;
	Txtemp[1] = heat_real;
	Txtemp[2] = heat_real >> 8;
	Txtemp[3] = heat_limit;
	Txtemp[4] = heat_limit >> 8;
	Txtemp[5] = shoot_num_real;
	Txtemp[6] = shoot_num_real >> 8;
//	Txtemp[7] = iq5;
	while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{

	}

}

/*******************************************************************************************
  * @Func	  set_shoot_speed(CAN_HandleTypeDef* hcan, s8 judge_status, s8 shoot_speed)
  * @Brief    发送数据给云台
  * @Param    裁判系统状态  摩擦轮速度
  * @Retval		None
  * @Date     2021/10/30
 *******************************************************************************************/
void set_shoot_speed(CAN_HandleTypeDef* hcan, s8 judge_status, s8 speed_limit, float speed_42mm)
{ 
	CAN_TxHeaderTypeDef _TxHeader;
	JudgeValue.speed_42mm.value = speed_42mm;
	JudgeValue.color_judge = is_red_or_blue();  //0是蓝色 1是红色
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x324;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = judge_status;
	Txtemp[1] = speed_limit;
	Txtemp[2] = JudgeValue.speed_42mm.buff[0];
	Txtemp[3] = JudgeValue.speed_42mm.buff[1];
	Txtemp[4] = JudgeValue.speed_42mm.buff[2];
	Txtemp[5] = JudgeValue.speed_42mm.buff[3];
	Txtemp[6] = JudgeValue.color_judge;

	while( HAL_CAN_GetTxMailboxesFreeLevel( hcan ) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{

	}

}

void set_rc_ch(CAN_HandleTypeDef* hcan, s16 ch1, s16 ch2, s16 ch3, s16 ch4)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x321;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (ch1 >> 8);
	Txtemp[1] = ch1;
	Txtemp[2] = (ch2 >> 8);
	Txtemp[3] = ch2;
	Txtemp[4] = ch3 >> 8;
	Txtemp[5] = ch3;
	Txtemp[6] = ch4 >> 8;
	Txtemp[7] = ch4;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
	
}
void set_rc_mouse(CAN_HandleTypeDef* hcan, s16 x, s16 y, s16 z, s8 l, s8 r)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x322;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = (x >> 8);
	Txtemp[1] = x;
	Txtemp[2] = (y >> 8);
	Txtemp[3] = y;
	Txtemp[4] = z >> 8;
	Txtemp[5] = z;
	Txtemp[6] = l;
	Txtemp[7] = r;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}
void set_rc_key(CAN_HandleTypeDef* hcan, s8 sw1, s8 sw2, s16 v, s16 wheel,u8 state)
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x323;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = sw1;
	Txtemp[1] = sw2;
	Txtemp[2] = (v >> 8);
	Txtemp[3] = v;
	Txtemp[4] = wheel >> 8;
	Txtemp[5] = wheel;
	Txtemp[6] = state;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}



void get_yaw_info(uint8_t temp[])
{
//	memcpy(&(gimbal_value.gimbal_yaw_angle.buff[0]),&temp[0],4);
	CAN2_RXTEST.RX_add++;
	control_judge_flag = 0;
	gimbal_value.gimbal_yaw_angle.buff[0] = temp[0];
	gimbal_value.gimbal_yaw_angle.buff[1] = temp[1];
	gimbal_value.gimbal_yaw_angle.buff[2] = temp[2];
	gimbal_value.gimbal_yaw_angle.buff[3] = temp[3];
	imu_init_finish_flag = temp[4];
	if(temp[4] == 1) 
		IMU_angle[0] = gimbal_value.gimbal_yaw_angle.value;
		pitch_show = (int16_t)(temp[7] << 8 | temp[6]);
	if(temp[5] == 0)
		mocalun_status = mNORMAL;
	else
		mocalun_status = mUNUSUAL;
	
}

//uint8_t temp_judge[8];
//void get_judge_info(uint8_t temp[])
//{
//	for(int i = 0; i<8; i++)
//	{
//		temp_judge[i] = temp[i];
//	}

//}
