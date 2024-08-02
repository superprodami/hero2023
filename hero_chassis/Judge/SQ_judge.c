/**
  ******************************************************************************
  * @file    judge.c
  * @brief   ��������ϵͳ���������ݣ��Ի�û����˵�ǰ��״̬���ݡ�
  *
  ******************************************************************************
  * @attention
  *
  * 2021.3 �Ѿ�����˺�tuxin.c�����䡣
  *
  *
  *
  ******************************************************************************
  */

#include "SQ_judge.h"
#include "usart.h"
#include "CRCs.h"
#include "string.h"
#include "dma.h"


bool Judge_Data_TF = false;//���������Ƿ���ã�������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID
/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum=0;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = false;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������
uint16_t Hurt_num = 0;
#define BLUE  0
#define RED   1


/* protocol��ͷ�ṹ�� */
frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
ext_game_status_t   Game_Status;
ext_game_result_t   Game_Result;
ext_game_robot_HP_t Game_Robot_HP;

/* 0x010X */
ext_event_data_t                Event_Data;
ext_supply_projectile_action_t  Supply_Projectile_Action;
ext_supply_projectile_booking_t Supply_Projectile_Booking;
ext_referee_warning_t           Referee_Warning;
ext_dart_remaining_time_t       Dart_Remaining_Time;

/* 0x020X */
ext_game_robot_state_t Game_Robot_State;
ext_power_heat_data_t  Power_Heat_Data;
ext_game_robot_pos_t   Game_Robot_Pos;
ext_buff_musk_t        Buff_Musk;
aerial_robot_energy_t  Aerial_Robot_Energy;
ext_robot_hurt_t       Robot_Hurt;
ext_shoot_data_t       Shoot_Data;
ext_bullet_remaining_t Bullet_Remaining;
ext_rfid_status_t      RFID_Status;
ext_dart_client_cmd_t  Dart_Client_Cmd;

/* 0x030X */
ext_student_interactive_header_data_t Student_Interactive_Header_Data;
robot_interactive_data_t              Robot_Interactive_Data;
ext_robot_command_t                   Robot_Command;
ext_client_map_command_t              Client_Map_Command;


/* Private variables ---------------------------------------------------------*/
/* ����ϵͳ����˫������ */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* ����ϵͳ�������ݶ��� */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol�������ṹ�� */
unpack_data_t Referee_Unpack_OBJ;


extern DMA_HandleTypeDef hdma_usart6_rx;

extern DMA_HandleTypeDef hdma_usart6_rx;

void USART6_IRQHandler_1(void)
{
		if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if(USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)//�����ǻ�����1,��ȡ2�ڴ�
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
        }
        else//�����ǻ�����2,��ȡ1�ڴ�
        {
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[0], this_time_rx_len);
        }
    }
}

/*==============================================================================
              ##### ����ϵͳ��ʼ������ #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ϵͳ�ṹ���ʼ������ Referee_StructInit
			(+) ����ϵͳ���ڳ�ʼ������ Referee_UARTInit
*/
void Referee_StructInit(void)
{
	memset(&Referee_Receive_Header,          0, sizeof(Referee_Receive_Header));
	
	memset(&Game_Status,                     0, sizeof(Game_Status));
	memset(&Game_Result,                     0, sizeof(Game_Result));
	memset(&Game_Robot_HP,                   0, sizeof(Game_Robot_HP));
	
	memset(&Event_Data,                      0, sizeof(Event_Data));
	memset(&Supply_Projectile_Action,        0, sizeof(Supply_Projectile_Action));
	memset(&Supply_Projectile_Booking,       0, sizeof(Supply_Projectile_Booking));
	memset(&Referee_Warning,                 0, sizeof(Referee_Warning));
	memset(&Dart_Remaining_Time,             0, sizeof(Dart_Remaining_Time));
	
	memset(&Game_Robot_State,                0, sizeof(Game_Robot_State));
	memset(&Power_Heat_Data,                 0, sizeof(Power_Heat_Data));
	memset(&Game_Robot_Pos,                  0, sizeof(Game_Robot_Pos));
	memset(&Buff_Musk,                       0, sizeof(Buff_Musk));
	memset(&Aerial_Robot_Energy,             0, sizeof(Aerial_Robot_Energy));
	memset(&Robot_Hurt,                      0, sizeof(Robot_Hurt));
	memset(&Shoot_Data,                      0, sizeof(Shoot_Data));
	memset(&Bullet_Remaining,                0, sizeof(Bullet_Remaining));
	memset(&RFID_Status,                     0, sizeof(RFID_Status));
	memset(&Dart_Client_Cmd,                 0, sizeof(Dart_Client_Cmd));
	
	memset(&Student_Interactive_Header_Data, 0, sizeof(Student_Interactive_Header_Data));
	memset(&Robot_Interactive_Data,          0, sizeof(Robot_Interactive_Data));
	memset(&Robot_Command,                   0, sizeof(Robot_Command));
	memset(&Client_Map_Command,              0, sizeof(Client_Map_Command));
}

//buff0 1Ϊ���������� 
void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength)
{
	/* ʹ�ܴ���DMA */
	SET_BIT(Referee_UART.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(Referee_UART.Instance->CR3, USART_CR3_DMAT);
	
	/* ʹ�ܴ��ڿ����ж� */
	__HAL_UART_ENABLE_IT(&Referee_UART, UART_IT_IDLE);
	
	/* ȷ��DMA RXʧ�� */
	while(Referee_UART.hdmarx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(Referee_UART.hdmarx);
	}
	
	/* ��ձ�־λ */
	__HAL_DMA_CLEAR_FLAG(Referee_UART.hdmarx, DMA_LISR_TCIF1);

	/* ���ý���˫������ */
	Referee_UART.hdmarx->Instance->PAR  = (uint32_t) & (Referee_UART.Instance->DR);
	Referee_UART.hdmarx->Instance->M0AR = (uint32_t)(Buffer0);
	Referee_UART.hdmarx->Instance->M1AR = (uint32_t)(Buffer1);
	
	/* �������ݳ��� */
	__HAL_DMA_SET_COUNTER(Referee_UART.hdmarx, BufferLength);
	
	/* ʹ��˫������ */
	SET_BIT(Referee_UART.hdmarx->Instance->CR, DMA_SxCR_DBM);
	
	/* ʹ��DMA RX */
	__HAL_DMA_ENABLE(Referee_UART.hdmarx);
	
	/* ȷ��DMA TXʧ�� */
	while(Referee_UART.hdmatx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(Referee_UART.hdmatx);
	}
	
	Referee_UART.hdmatx->Instance->PAR  = (uint32_t) & (Referee_UART.Instance->DR);
}

/*==============================================================================
              ##### ����ϵͳ���ݽ������� #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ϵͳ�������ݽ�ѹ���� Referee_UnpackFifoData
      (+) ����ϵͳ�������ݴ����� Referee_SolveFifoData
*/
void Referee_UnpackFifoData(unpack_data_t *referee_unpack_obj, fifo_s_t *referee_fifo)
{
  uint8_t byte = 0;
  uint8_t sof  = HEADER_SOF;
	
  while(fifo_s_used(referee_fifo))
  {
    byte = fifo_s_get(referee_fifo);
    switch(referee_unpack_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          referee_unpack_obj->unpack_step = STEP_LENGTH_LOW;
          referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        }
        else
        {
          referee_unpack_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        referee_unpack_obj->data_len = byte;
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        referee_unpack_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        referee_unpack_obj->data_len |= (byte << 8);
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        if(referee_unpack_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          referee_unpack_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
          referee_unpack_obj->index = 0;
        }
      }break;
			
      case STEP_FRAME_SEQ:
      {
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        referee_unpack_obj->unpack_step = STEP_HEADER_CRC8;
      }break;
			
      case STEP_HEADER_CRC8:
      {
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        if(referee_unpack_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if(CRC08_Verify(referee_unpack_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
          {
            referee_unpack_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
            referee_unpack_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if(referee_unpack_obj->index <  (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
        {
           referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;  
        }
        if(referee_unpack_obj->index >= (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
        {
          referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
          referee_unpack_obj->index = 0;
          if(CRC16_Verify(referee_unpack_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
          {
            Referee_SolveFifoData(referee_unpack_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
        referee_unpack_obj->index = 0;
      }break;
    }
  }
}

referee_cmd_id_e cmd_id = 0;
void Referee_SolveFifoData(uint8_t *frame)
{

	uint8_t  index  = 0;
	
	memcpy(&Referee_Receive_Header, frame, sizeof(frame_header_struct_t));
	index += sizeof(frame_header_struct_t);
	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);
	
	switch(cmd_id)
	{
		case GAME_STATE_CMD_ID:      	         memcpy(&Game_Status,               frame + index, sizeof(ext_game_status_t));               break;
		case GAME_RESULT_CMD_ID:               memcpy(&Game_Result,               frame + index, sizeof(ext_game_result_t));               break;
		case GAME_ROBOT_HP_CMD_ID:             memcpy(&Game_Robot_HP,             frame + index, sizeof(ext_game_robot_HP_t));             break;
		case DART_FLYING_STATE_CMD_ID:                                                                                                     break;
		
    case FIELD_EVENTS_CMD_ID:              memcpy(&Event_Data,                frame + index, sizeof(ext_event_data_t));                break;
		case SUPPLY_PROJECTILE_ACTION_CMD_ID:  memcpy(&Supply_Projectile_Action,  frame + index, sizeof(ext_supply_projectile_action_t));  break;
		case SUPPLY_PROJECTILE_BOOKING_CMD_ID: memcpy(&Supply_Projectile_Booking, frame + index, sizeof(ext_supply_projectile_booking_t)); break;
		case REFEREE_WARNING_CMD_ID:           memcpy(&Referee_Warning,           frame + index, sizeof(ext_referee_warning_t));           break;
		case DART_REMAINING_TIME_CMD_ID:       memcpy(&Dart_Remaining_Time,       frame + index, sizeof(ext_dart_remaining_time_t));       break;
		
		case ROBOT_STATE_CMD_ID:
		{
		  memcpy(&Game_Robot_State, frame + index, sizeof(ext_game_robot_state_t));
		  break;
		}
		case POWER_HEAT_DATA_CMD_ID:
		{
//			Power_Heat_Data.RX_MSG.RX_add++;   
			memcpy(&Power_Heat_Data,           frame + index, sizeof(ext_power_heat_data_t));           
			break;
		}
		case ROBOT_POS_CMD_ID:
		{
			memcpy(&Game_Robot_Pos,            frame + index, sizeof(ext_game_robot_pos_t));            
			break;
		}
		case BUFF_MUSK_CMD_ID:                 memcpy(&Buff_Musk,                 frame + index, sizeof(ext_buff_musk_t));                 break;
		case AERIAL_ROBOT_ENERGY_CMD_ID:       memcpy(&Aerial_Robot_Energy,       frame + index, sizeof(aerial_robot_energy_t));           break;
		case ROBOT_HURT_CMD_ID:                memcpy(&Robot_Hurt,                frame + index, sizeof(ext_robot_hurt_t));                break;
		case SHOOT_DATA_CMD_ID:                Shoot_Data.RX_MSG.RX_add++;        memcpy(&Shoot_Data,                frame + index, sizeof(ext_shoot_data_t));                 break;
		case BULLET_REMAINING_CMD_ID:          memcpy(&Bullet_Remaining,          frame + index, sizeof(ext_bullet_remaining_t));          break;
		case ROBOT_RFID_STATE_CMD_ID:          memcpy(&RFID_Status,               frame + index, sizeof(ext_rfid_status_t));               break;
		case DART_CLIENT_CMD_ID:               memcpy(&Dart_Client_Cmd,           frame + index, sizeof(ext_dart_client_cmd_t));           break;
		
		case STUDENT_INTERACTIVE_DATA_CMD_ID:  memcpy(&Robot_Interactive_Data,    frame + index, sizeof(robot_interactive_data_t));        break;
		case ROBOT_COMMAND_CMD_ID:             memcpy(&Robot_Command,             frame + index, sizeof(ext_robot_command_t));             break;
		case CLIENT_MAP_COMMAND_CMD_ID:        memcpy(&Client_Map_Command,        frame + index, sizeof(ext_client_map_command_t));        break;
		
		default:                                                                                                                           break;
	}
}



bool Color;
bool is_red_or_blue(void)//�ж��Լ�������
{
    Judge_Self_ID = Game_Robot_State.robot_id;//��ȡ��ǰ������ID

    if(Game_Robot_State.robot_id > 10)
    {
        return BLUE;
    }
    else
    {
        return RED;
    }
}

uint16_t determine_ID(void)
{
    Color = is_red_or_blue();
    if(Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0164 + (Judge_Self_ID - 100); //����ͻ���id
    }
    else if(Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
    }
		return Judge_SelfClient_ID;
}

/********************�������ݸ����жϺ���***************************/

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
bool JUDGE_sGetDataState(void)
{
    return Judge_Data_TF;
}

uint8_t JUDGE_GameState(void)
{
  return Game_Status.game_progress;
}
/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention
  */
float JUDGE_fGetChassisPower(void)
{
    return (Power_Heat_Data.chassis_power);
}

/**
  * @brief  ��ȡ���̹�������
  * @param  void
  * @retval ʵʱ���̹�������
  * @attention
  */

uint8_t JUDGE_usGetPowerLimit(void)
{
	
    return (Game_Robot_State.chassis_power_limit );
}



//Power_Heat_Data

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
    return (Power_Heat_Data.chassis_power_buffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	Game_Robot_State.robot_level;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 42mm
  * @attention  ʵʱ����
  */
uint16_t JUDGE_usGetRemoteHeat42(void)
{
    return Power_Heat_Data.shooter_id1_42mm_cooling_heat;
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 42mm
  * @attention  ʵʱ����
  */
float JUDGE_usGetSpeed42(void)
{
    return Shoot_Data.bullet_speed;
}

/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention
  */
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
    Shoot_Speed_Now = Shoot_Data.bullet_speed;
    if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
    {
        ShootNum++;
        Shoot_Speed_Last = Shoot_Speed_Now;
    }
}

/**
  * @brief  ��ȡ������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
  */
uint16_t JUDGE_usGetShootNum(void)
{
    return ShootNum;
}

/**
  * @brief  ����������
  * @param  void
  * @retval void
  * @attention
  */
void JUDGE_ShootNum_Clear(void)
{
    ShootNum = 0;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ�42mm��������
  * @attention
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
    return Game_Robot_State.shooter_barrel_heat_limit;
}
/**
  * @brief  ��ȡǹ����������
  * @param  void
  * @retval ��ǰ�ȼ�42mm��������
  * @attention
  */
uint8_t JUDGE_usGetSpeedLimit(void)
{
    return 16 ;
}
/**
  * @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�42mm��ȴ�ٶ�
  * @attention
  */
uint16_t JUDGE_usGetShootCold(void)
{
    return Game_Robot_State.shooter_barrel_cooling_value;
}

bool Judge_If_Death(void)
{
    if(Game_Robot_State.current_HP == 0 && JUDGE_sGetDataState() == true)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Judge_If_Near_Death(void)
{
    if(Game_Robot_State.current_HP <= 20 && JUDGE_sGetDataState() == true)
    {
        return true;
    }
    else
    {
        return false;
    }
}

