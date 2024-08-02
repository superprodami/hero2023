#include "Motor_DM.h"


uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//电机失能命令
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	//电机保存零点命令

void Motor_enable(void)
{
	#if Motar_mode==0
	CANx_SendStdData(&hcan1,0x102,Data_Enable,8);	
	#elif Motar_mode==1
	CANx_SendStdData(&hcan1,0x202,Data_Enable,8);	
	#elif Motar_mode==2
	CANx_SendStdData(&hcan1,0x302,Data_Enable,8);	
	#endif
}


uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
	
		while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,pData,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	  {
		
	  }
    return 0;
//        /*找到空的发送邮箱，把数据发送出去*/
//	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
//	{
//		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
//		{
//			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
//        }
//    }
}


//达妙电机
/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 * @param  _KP    位置比例系数
 * @param  _KD    位置微分系数
 * @param  _torq  转矩给定值
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq)
 { 
	static CAN_TxHeaderTypeDef   Tx_Header;
	uint8_t Txtemp[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	Tx_Header.StdId=id;
	Tx_Header.IDE=CAN_ID_STD;
	Tx_Header.RTR=CAN_RTR_DATA;
	Tx_Header.DLC=0x08;
	
	Txtemp[0] = (pos_tmp >> 8);
	Txtemp[1] = pos_tmp;
	Txtemp[2] = (vel_tmp >> 4);
	Txtemp[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	Txtemp[4] = kp_tmp;
	Txtemp[5] = (kd_tmp >> 4);
	Txtemp[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	Txtemp[7] = tor_tmp;
	 
	 //寻空邮箱发送数据
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,Txtemp,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK)
	  {
		
	  }

 }
 
 
/* *
 * @brief  位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel)
{
    static CAN_TxHeaderTypeDef Tx_Header;
		uint8_t Txtemp[8];
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

		Tx_Header.StdId=id+0x100;
		Tx_Header.IDE=CAN_ID_STD;
		Tx_Header.RTR=CAN_RTR_DATA;
		Tx_Header.DLC=0x08;

    Txtemp[0] = *pbuf;;
    Txtemp[1] = *(pbuf+1);
    Txtemp[2] = *(pbuf+2);
    Txtemp[3] = *(pbuf+3);
    Txtemp[4] = *vbuf;
    Txtemp[5] = *(vbuf+1);
    Txtemp[6] = *(vbuf+2);
    Txtemp[7] = *(vbuf+3);
//	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
//	{
//		HAL_CAN_AddTxMessage(hcan,&Tx_Header,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
//	}
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&Tx_Header,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	  {
		
	  }
}

/**
 * @brief  速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _vel   速度给定
 */
void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel)
{
		static CAN_TxHeaderTypeDef   Tx_Header;
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;
		uint8_t Txtemp[8];

    Tx_Header.StdId = ID+0x200;
    Tx_Header.IDE = CAN_ID_STD;
    Tx_Header.RTR = CAN_RTR_DATA;
    Tx_Header.DLC = 0x04;

    Txtemp[0] = *vbuf;
    Txtemp[1] = *(vbuf+1);
    Txtemp[2] = *(vbuf+2);
    Txtemp[3] = *(vbuf+3);

    //找到空的发送邮箱，把数据发送出去
	  if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, Txtemp, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, Txtemp, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, Txtemp, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}



