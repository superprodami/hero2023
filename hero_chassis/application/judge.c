///**
//******************************************************************************
//* @file    judge.c
//* @brief   ��������ϵͳ���������ݣ��Ի�û����˵�ǰ��״̬���ݡ�
//*
//******************************************************************************
//* @attention
//*
//* 2021.3 �Ѿ�����˺�tuxin.c�����䡣
//*
//*
//*
//******************************************************************************
//*/

//#include "judge.h"
//#include "SQ_judge.h"
//#include "crc.h"
//#include "tuxin.h"

//frame_t judge_frame_rx = {0};
//frame_t judge_frame_tx;
//uint8_t receive_student_data[113] = {0};
//uint8_t judge_rx_buff[JUDGE_MAX_LENGTH];

//judge_show_data_t    Show_data = {0};//�ͻ�����Ϣ
//send_to_teammate     CommuData;//����ͨ����Ϣ
//robot_type_t robot_type;

//bool Judge_Data_TF = false;//���������Ƿ���ã�������������
//uint8_t Judge_Self_ID;//��ǰ�����˵�ID
//uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID
///**************����ϵͳ���ݸ���****************/
//uint16_t ShootNum=0;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
//bool Hurt_Data_Update = false;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������
//uint16_t Hurt_num = 0;
//#define BLUE  0
//#define RED   1

//uint16_t choose_client(uint8_t robot_id)
//{
//	uint16_t client_id;
//	switch (robot_id)
//	{
//	case red_hero:
//			client_id = red_hero_client;
//			break;
//	case red_engineer:
//			client_id = red_engineer_client;
//			break;
//	case red_infantry_3:
//			client_id = red_infantry_3_client;
//			break;
//	case red_infantry_4:
//			client_id = red_infantry_4_client;
//			break;
//	case red_infantry_5:
//			client_id = red_infantry_5_client;
//			break;
//	case red_aerial:
//			client_id = red_aerial_client;
//			break;

//	case blue_hero:
//			client_id = blue_hero_client;
//			break;
//	case blue_engineer:
//			client_id = blue_engineer_client;
//			break;
//	case blue_infantry_3:
//			client_id = blue_infantry_3_client;
//			break;
//	case blue_infantry_4:
//			client_id = blue_infantry_4_client;
//			break;
//	case blue_infantry_5:
//			client_id = blue_infantry_5_client;
//			break;
//	case blue_aerial:
//			client_id = blue_aerial_client;
//			break;
//	default:
//			break;
//	}
//	return client_id;
//}

////�Ӳ���ϵͳ��ȡ����
//bool Judge_Read_Data(uint8_t *ReadFromUsart)
//{
//	bool retval_tf = false;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����

//	//�����ݰ��������κδ���
//	if (ReadFromUsart == NULL)
//	{
//			return -1;
//	}
//	//��֡ͷ��Ϣ����
//	memcpy(&judge_frame_rx.frame_header, ReadFromUsart, LEN_HEADER);

//	judge_frame_rx.frame_header.Data_Length = (ReadFromUsart[DATA_LENGTH + 1] << 8 ) | ReadFromUsart[DATA_LENGTH];
//	//�����ж�֡ͷ
//	if(judge_frame_rx.frame_header.SOF == Judge_Data_SOF)
//	{
//			//�ж�֡ͷCRCУ���Ƿ���ȷ  
//			if(Verify_CRC8_Check_Sum(ReadFromUsart, LEN_HEADER) == true)
//			{
//					//�ж�����CRCУ���Ƿ���ȷ
//					if(Verify_CRC16_Check_Sum(ReadFromUsart, LEN_HEADER + LEN_CMDID + judge_frame_rx.frame_header.Data_Length + LEN_TAIL) == true)
//					{
//							retval_tf = true;//��У�������˵�����ݿ���

//							judge_frame_rx.cmd_id = (uint16_t)ReadFromUsart[6] << 8 | ReadFromUsart[5];
//							//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
//							switch(judge_frame_rx.cmd_id)
//							{

//							case game_status_t:
//									memcpy(&judge_frame_rx.data.game_status, ReadFromUsart + DATA, len_game_status);
//									break ;

//							case game_result_t:
//									memcpy(&judge_frame_rx.data.game_result, ReadFromUsart + DATA, len_game_result);
//									break;

//							case game_robot_HP_t:
//									memcpy(&judge_frame_rx.data.game_robot_HP, ReadFromUsart + DATA, len_game_robot_HP);
//									break;

//							case dart_status_t:
//									memcpy(&judge_frame_rx.data.dart_status, ReadFromUsart + DATA, len_dart_status);
//									break;

//							case ICRA_buff_debuff_zone_status_t:
//									memcpy(&judge_frame_rx.data.ICRA_buff_debuff_zone_status, ReadFromUsart + DATA, len_ICRA_buff_debuff_zone_status);
//									break;

//							case event_data_t:
//									memcpy(&judge_frame_rx.data.event_data, ReadFromUsart + DATA, len_event_data);
//									break;

//							case supply_projectile_action_t:
//									memcpy(&judge_frame_rx.data.supply_projectile_action, ReadFromUsart + DATA, len_supply_projectile_action);
//									break;

//							case referee_warning_t:
//									memcpy(&judge_frame_rx.data.referee_warning, ReadFromUsart + DATA, len_referee_warning);
//									break;

//							case dart_remaining_time_t:
//									memcpy(&judge_frame_rx.data.dart_remaining_time, ReadFromUsart + DATA, len_dart_remaining_time);
//									break;

//							case game_robot_status_t:
//									memcpy(&judge_frame_rx.data.game_robot_status, ReadFromUsart + DATA, len_game_robot_status);
//									Judge_Self_ID = judge_frame_rx.data.game_robot_status.robot_id ;
//									if(Judge_Self_ID < 100)
//									{
//											robot_type = red_robot;
//									}
//									else
//									{
//											robot_type = blue_robot;
//									}
//									break;

//							case power_heat_data_t:
//									memcpy(&judge_frame_rx.data.power_heat_data, ReadFromUsart + DATA, len_power_heat_data);
//									break;

//							case game_robot_pos_t:
//									memcpy(&judge_frame_rx.data.game_robot_pos, ReadFromUsart + DATA, len_game_robot_pos);
//									break;

//							case buff_t:
//									memcpy(&judge_frame_rx.data.buff, ReadFromUsart + DATA, len_buff);
//									break;

//							case aerial_robot_energy_t:
//									memcpy(&judge_frame_rx.data.aerial_robot_energy, ReadFromUsart + DATA, len_aerial_robot_energy);
//									break;

//							case robot_hurt_t:
//									memcpy(&judge_frame_rx.data.robot_hurt, ReadFromUsart + DATA, len_robot_hurt);
//									if(judge_frame_rx.data.robot_hurt.hurt_type == 0)//װ���˺���Ѫ
//									{
//											Hurt_Data_Update = true;	//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
//											Hurt_num++;
//									}
//									else
//									{
//											Hurt_Data_Update = false;
//									}
//									break;

//							case shoot_data_t:
//									memcpy(&judge_frame_rx.data.shoot_data, ReadFromUsart + DATA, len_shoot_data);
//									JUDGE_ShootNumCount();//������ͳ��
//									break;

//							case bullet_remaining_t:
//									memcpy(&judge_frame_rx.data.bullet_remaining, ReadFromUsart + DATA, len_bullet_remaining);
//									break;

//							case rfid_status_t:
//									memcpy(&judge_frame_rx.data.rfid_status, ReadFromUsart + DATA, len_rfid_status);
//									break;

//							case dart_client_cmd_t:
//									memcpy(&judge_frame_rx.data.dart_client_cmd, ReadFromUsart + DATA, len_dart_client_cmd);
//									break;

//							case student_interactive_header_data_t:
//									memcpy(&receive_student_data, ReadFromUsart + STU_DATA, len_student_interactive_header_data);
//									break;

//							case robot_interactive_data_t:
//									memcpy(&judge_frame_rx.data.robot_interactive_data, ReadFromUsart + DATA, len_robot_interactive_data);
//									break;

//							case smallmap_communicate_t:
//									memcpy(&judge_frame_rx.data.smallmap_communicate, ReadFromUsart + STU_DATA, len_smallmap_communicate);
//									break;
//							case robot_command_t:
//									memcpy(&judge_frame_rx.data.robot_command, ReadFromUsart + STU_DATA, len_robot_command);
//									break;
//							case client_map_command_t:
//									memcpy(&judge_frame_rx.data.client_map_command, ReadFromUsart + STU_DATA, len_client_map_command);
//									break;

//							default:
//									break;
//							}
//					}
//			}
//			//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
//			if(*(ReadFromUsart + sizeof(frame_header_t) + LEN_CMDID + judge_frame_rx.frame_header.Data_Length + LEN_TAIL) == 0xA5)
//			{
//					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
//					Judge_Read_Data(ReadFromUsart + sizeof(frame_header_t) + LEN_CMDID + judge_frame_rx.frame_header.Data_Length + LEN_TAIL);
//			}
//	}
//	if (retval_tf == true)
//	{
//			Judge_Data_TF = true;//����������
//	}
//	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
//	{
//			Judge_Data_TF = false;//����������
//	}

//	return retval_tf;//����������������

//}

//#define send_max_len     200
//unsigned char CliendTxBuffer[send_max_len];
//void JUDGE_Show_Data(void)
//{
//	//	static uint8_t datalength,i;
//	//	uint8_t judge_led = 0xff;//��ʼ��ledΪȫ��
//	//	static uint8_t auto_led_time = 0;
//	//	static uint8_t buff_led_time = 0;

//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID

//	Show_data.frame_header.SOF = 0xA5;
//	Show_data.frame_header.Data_Length = sizeof(ext_student_interactive_header_data_t) + sizeof(operate_data_t);
//	Show_data.frame_header.Seq = 0;
//	memcpy(CliendTxBuffer, &Show_data.frame_header, sizeof(frame_header_t));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(frame_header_t));//д��֡ͷCRC8У����

//	Show_data.cmd_id = 0x0301;

//	Show_data.student_interactive_header.data_cmd_id = 0xD180;//���͸��ͻ��˵�cmd���ٷ��̶�
//	//ID�Ѿ����Զ���ȡ��
//	Show_data.student_interactive_header.sender_ID 	 = Judge_Self_ID;//�����ߵ�ID
//	Show_data.student_interactive_header.receiver_ID = Judge_SelfClient_ID;//�����ߵ�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//}

//#define Teammate_max_len     200
//unsigned char TeammateTxBuffer[Teammate_max_len];
//bool Send_Color = 0;
//bool First_Time_Send_Commu = false;
//uint16_t send_time = 0;
//void Send_to_Teammate(void)
//{
//	//	static uint8_t datalength,i;

//	Send_Color = is_red_or_blue();//�жϷ��͸��ڱ�����ɫ��17����7��?

//	memset(TeammateTxBuffer,0,200);

//	CommuData.frame_header.SOF = 0xA5;
//	CommuData.frame_header.Data_Length = sizeof(ext_student_interactive_header_data_t) + sizeof(CommuData.student_interactive_data);
//	CommuData.frame_header.Seq = 0;
//	memcpy(TeammateTxBuffer, &CommuData.frame_header, sizeof(frame_header_t));
//	Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(frame_header_t));

//	CommuData.cmd_id = 0x0301;

//	CommuData.student_interactive_header_data.sender_ID = Judge_Self_ID;//�����ߵ�ID

//	if( First_Time_Send_Commu == true )
//	{
//			CommuData.student_interactive_header_data.data_cmd_id = 0x0292;//��0x0200-0x02ff֮��ѡ?
//			CommuData.student_interactive_data[0] = 0x01;
//			CommuData.student_interactive_data[1] = 0x03;
//			CommuData.student_interactive_data[2] = 0x03;
//			CommuData.student_interactive_data[3] = 0x04;
//			CommuData.student_interactive_data[4] = 0x05;
//			CommuData.student_interactive_data[5] = 0x06;
//			CommuData.student_interactive_data[6] = 'W';
//			CommuData.student_interactive_data[19] = 0x99;

//			if(Send_Color == BLUE)//�Լ����������͸����ڱ�
//			{
//					CommuData.student_interactive_header_data.receiver_ID = 107;//������ID
//			}
//			else if(Send_Color == RED)//�Լ��Ǻ죬���͸����ڱ�
//			{
//					CommuData.student_interactive_header_data.receiver_ID = 7;//������ID
//			}
//	}
//	else
//	{
//			CommuData.student_interactive_header_data.data_cmd_id = 0x0255;
//			send_time = 0;
//			CommuData.student_interactive_header_data.receiver_ID = 88;//������ID��������
//	}

//	CommuData.student_interactive_data[0] = 0;//���͵����ݣ���С��Ҫ���������ı�������

//	memcpy(TeammateTxBuffer + 5, (uint8_t *)&CommuData.cmd_id, (sizeof(CommuData.cmd_id) + CommuData.frame_header.Data_Length));
//	Append_CRC16_Check_Sum(TeammateTxBuffer, sizeof(CommuData.cmd_id) + sizeof(CommuData.frame_header) + sizeof(CommuData.student_interactive_header_data) + 20 + sizeof(CommuData.frame_tail));

//	//datalength = sizeof(CommuData);
//	//	if( First_Time_Send_Commu == true )
//	//	{
//	//     HAL_UART_Transmit_DMA(&huart6, TeammateTxBuffer, JUDGE_MAX_LENGTH);
//	//	}
//}



//bool Color;
//bool is_red_or_blue(void)//�ж��Լ�������
//{
//	Judge_Self_ID = judge_frame_rx.data.game_robot_status.robot_id;//��ȡ��ǰ������ID

//	if(judge_frame_rx.data.game_robot_status.robot_id > 10)
//	{
//			return BLUE;
//	}
//	else
//	{
//			return RED;
//	}
//}

//void determine_ID(void)
//{
//	Color = is_red_or_blue();
//	if(Color == BLUE)
//	{
//			Judge_SelfClient_ID = 0x0164 + (Judge_Self_ID - 100); //����ͻ���id
//	}
//	else if(Color == RED)
//	{
//			Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
//	}
//}

///********************�������ݸ����жϺ���***************************/

///**
//* @brief  �����Ƿ����
//* @param  void
//* @retval  TRUE����   FALSE������
//* @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
//*/
//bool JUDGE_sGetDataState(void)
//{
//	return Judge_Data_TF;
//}

//uint8_t JUDGE_GameState(void)
//{
//return judge_frame_rx.data.game_status.game_progress;
//}
///**
//* @brief  ��ȡ˲ʱ����
//* @param  void
//* @retval ʵʱ����ֵ
//* @attention
//*/
//float JUDGE_fGetChassisPower(void)
//{
//	return (judge_frame_rx.data.power_heat_data.chassis_power);
//}

///**
//* @brief  ��ȡ���̹�������
//* @param  void
//* @retval ʵʱ���̹�������
//* @attention
//*/

//uint8_t JUDGE_usGetPowerLimit(void)
//{

//	return (judge_frame_rx.data.game_robot_status.chassis_power_limit );
//}



////Power_Heat_Data

///**
//* @brief  ��ȡʣ�ཹ������
//* @param  void
//* @retval ʣ�໺�役������(���60)
//* @attention
//*/
//uint16_t JUDGE_fGetRemainEnergy(void)
//{
//	return (judge_frame_rx.data.power_heat_data.chassis_power_buffer);
//}

///**
//* @brief  ��ȡ��ǰ�ȼ�
//* @param  void
//* @retval ��ǰ�ȼ�
//* @attention
//*/
//uint8_t JUDGE_ucGetRobotLevel(void)
//{
//	return	judge_frame_rx.data.game_robot_status.robot_level;
//}

///**
//* @brief  ��ȡǹ������
//* @param  void
//* @retval 42mm
//* @attention  ʵʱ����
//*/
//uint16_t JUDGE_usGetRemoteHeat42(void)
//{
//	return judge_frame_rx.data.power_heat_data.shooter_id1_42mm_cooling_heat;
//}

///**
//* @brief  ��ȡ����
//* @param  void
//* @retval 42mm
//* @attention  ʵʱ����
//*/
//float JUDGE_usGetSpeed42(void)
//{
//	return judge_frame_rx.data.shoot_data.bullet_speed;
//}

///**
//* @brief  ͳ�Ʒ�����
//* @param  void
//* @retval void
//* @attention
//*/
//float Shoot_Speed_Now = 0;
//float Shoot_Speed_Last = 0;
//void JUDGE_ShootNumCount(void)
//{
//	Shoot_Speed_Now = judge_frame_rx.data.shoot_data.bullet_speed;
//	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
//	{
//			ShootNum++;
//			Shoot_Speed_Last = Shoot_Speed_Now;
//	}
//}

///**
//* @brief  ��ȡ������
//* @param  void
//* @retval ������
//* @attention ��������˫ǹ��
//*/
//uint16_t JUDGE_usGetShootNum(void)
//{
//	return ShootNum;
//}

///**
//* @brief  ����������
//* @param  void
//* @retval void
//* @attention
//*/
//void JUDGE_ShootNum_Clear(void)
//{
//	ShootNum = 0;
//}

///**
//* @brief  ��ȡǹ������
//* @param  void
//* @retval ��ǰ�ȼ�42mm��������
//* @attention
//*/
//uint16_t JUDGE_usGetHeatLimit(void)
//{
//	return judge_frame_rx.data.game_robot_status.shooter_id1_42mm_cooling_limit;
//}
///**
//* @brief  ��ȡǹ����������
//* @param  void
//* @retval ��ǰ�ȼ�42mm��������
//* @attention
//*/
//uint8_t JUDGE_usGetSpeedLimit(void)
//{
//	return judge_frame_rx.data.game_robot_status.shooter_id1_42mm_speed_limit;
//}
///**
//* @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
//* @param  void
//* @retval ��ǰ�ȼ�42mm��ȴ�ٶ�
//* @attention
//*/
//uint16_t JUDGE_usGetShootCold(void)
//{
//	return judge_frame_rx.data.game_robot_status.shooter_id1_42mm_cooling_rate;
//}

//bool Judge_If_Death(void)
//{
//	if(judge_frame_rx.data.game_robot_status.remain_HP == 0 && JUDGE_sGetDataState() == true)
//	{
//			return true;
//	}
//	else
//	{
//			return false;
//	}
//}

//bool Judge_If_Near_Death(void)
//{
//	if(judge_frame_rx.data.game_robot_status.remain_HP <= 20 && JUDGE_sGetDataState() == true)
//	{
//			return true;
//	}
//	else
//	{
//			return false;
//	}
//}





