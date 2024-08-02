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
#include "dbus.h"
#include  "Motor_DM.h"

extern rc_info_t rc;

moto_measure_t moto_CAN[11] ={0};
moto_measure_t moto_CAN2[11] = {0};
moto_measure_t moto_CAN_DM[3] ={0};

//heat_measure_t heat_judge = {0};
//shoot_measure_t shoot_judge = {0};

angle_measure_t yaw_angle_send;
GimbalValueType gimbal_value;
JudgeValueType JudgeValue;


void get_total_angle(moto_measure_t *p);
//void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

void get_judge_info(heat_measure_t* _heat, uint8_t temp[]);

void get_dch_info(rc_info_t* _rc, uint8_t temp[]);
void get_dmouse_info(rc_info_t* _rc, uint8_t temp[]);
void get_dkey_info(rc_info_t* _rc, uint8_t temp[]);

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
				static u8 i;
				i = _RxHeader.StdId - CAN_Moto1_ID;
				if(_hcan==&hcan1)
					get_moto_measure(&moto_CAN[i],temp);
				else
					get_moto_measure(&moto_CAN2[i], temp);
			}
			break;
		case CAN_DM_Moto2_ID:
					get_dm_moto_measure(&moto_CAN_DM[1],temp);	
		  break;
		case heat_id:
				get_judge_info(&heat_judge, temp);
				break;
		case dbus_ch_id:
			get_dch_info(&rc, temp);
			break;
		case dbus_mouse_id:
			get_dmouse_info(&rc, temp);
			break;
		case dbus_key_id:
			get_dkey_info(&rc, temp);
			break;
		case shoot_speed_id:
			get_shoot_info(&shoot_judge, temp);
			break;
	}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/

	  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param
  * @Retval		None
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr,uint8_t temp[])
{

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(temp[0]<<8 | temp[1]) ;
	ptr->speed_rpm  = (int16_t)(temp[2]<<8 |temp[3]);
	ptr->real_current = (temp[4]<<8 | temp[5])*5.f/16384.f;

	ptr->hall = temp[6];

	get_total_angle(ptr);

  ptr->round_cnt = (int16_t)((ptr->total_angle / RADIO) / 8192);

	ptr->main_angle = (ptr->total_angle / RADIO) - ptr->round_cnt * 8192;

	if(ptr->main_angle < 0) ptr->main_angle = 8192 + ptr->main_angle;
}

void get_dm_moto_measure(moto_measure_t *ptr, uint8_t temp[])
{
	float  	real_current_t;
	uint8_t  	hall_t;
	int 	angle_t,speed_rpm_t,t_int;

	
	angle_t=(temp[1]<<8)|temp[2];
	speed_rpm_t=(temp[3]<<4)|(temp[4]>>4);
	t_int=((temp[4]&0xF)<<8)|temp[5];
	hall_t=(float)(temp[7]);
	
	ptr->position = uint_to_float(angle_t, P_MIN, P_MAX, 16); // (-12.5,12.5)
	ptr->velocity = uint_to_float(speed_rpm_t, V_MIN, V_MAX, 12); // (-45.0,45.0)
	ptr->torque = uint_to_float(t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
	ptr->hall = hall_t;
	get_total_angle(ptr);
	ptr->RX_MSG.RX_add++;	

	
	ptr->RX_MSG.RX_add++;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){

	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}
//摩擦轮电机;1l;2r
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
//5pitch轴电机
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


angle_measure_t _yaw_angle;
int pitch_show  = 0;
void set_angle(CAN_HandleTypeDef* hcan, float yaw_angle, bool flag)
{  //当前yaw轴角度 
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x330;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	
	pitch_show = (int)(IMU_angle[1] * 100);
	_yaw_angle.chassis_yaw_value.value = yaw_angle;
	
	Txtemp[0] = _yaw_angle.chassis_yaw_value.buff[0];
	Txtemp[1] = _yaw_angle.chassis_yaw_value.buff[1];
	Txtemp[2] = _yaw_angle.chassis_yaw_value.buff[2];
	Txtemp[3] = _yaw_angle.chassis_yaw_value.buff[3];
	
	Txtemp[4] = flag;
	Txtemp[5] = mocalun_status;
	Txtemp[6] = pitch_show;
	Txtemp[7] = pitch_show >> 8;
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan ) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	  {

	  }

}



void get_judge_info(heat_measure_t* _heat, uint8_t temp[])
{
	_heat->judge_status = temp[0];
	_heat->heat_real = (uint16_t)(temp[2]<<8 | temp[1]);
	_heat->heat_limit = (uint16_t)(temp[4]<<8 | temp[3]) ;
	_heat->shoot_num = (uint16_t)(temp[6]<<8 | temp[5]) ;

}

void get_shoot_info(shoot_measure_t* _shoot,uint8_t temp[])
{
	_shoot->judge_status = temp[0];
	if(temp[0])
	{
		_shoot->shoot_limit = temp[1];
		JudgeValue.speed_42mm.buff[0] = temp[2];
		JudgeValue.speed_42mm.buff[1] = temp[3];
		JudgeValue.speed_42mm.buff[2] = temp[4];
		JudgeValue.speed_42mm.buff[3] = temp[5];
		JudgeValue.color_judge = temp[6];
		_shoot->shoot_42mm = JudgeValue.speed_42mm.value;
	}
		
}

void get_dch_info(rc_info_t* _rc, uint8_t temp[])
{
	_rc->ch1 = (int16_t)(temp[0]<<8 | temp[1]);
	_rc->ch2 = (int16_t)(temp[2]<<8 | temp[3]);
	_rc->ch3 = (int16_t)(temp[4]<<8 | temp[5]);
	_rc->ch4 = (int16_t)(temp[6]<<8 | temp[7]);
}

void get_dmouse_info(rc_info_t* _rc, uint8_t temp[])
{
	_rc->mouse.x = (int16_t)(temp[0]<<8 | temp[1]);
	_rc->mouse.y = (int16_t)(temp[2]<<8 | temp[3]);
	_rc->mouse.z = (int16_t)(temp[4]<<8 | temp[5]);
	_rc->mouse.press_l = temp[6];
	_rc->mouse.press_r = temp[7];
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);

}

void get_dkey_info(rc_info_t* _rc, uint8_t temp[])
{
	_rc->sw1 = temp[0];
	_rc->sw2 = temp[1];
	_rc->key.v = (int16_t)(temp[2]<<8 | temp[3]);
	_rc->wheel = (int16_t)(temp[4]<<8 | temp[5]);
	state_judge = temp[6];
	control_judge_flag = 0;
}
