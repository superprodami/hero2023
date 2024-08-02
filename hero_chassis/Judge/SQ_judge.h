#ifndef __judge_h
#define __judge_h

#include "main.h"
#include "stdint.h"
#include "fifo.h"



/* Defines -------------------------------------------------------------------*/

extern uint8_t Judge_Self_ID;//��ǰ�����˵�ID
extern uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID
//����֡ͷ
//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define   Judge_Data_SOF 0xA5


#define   JUDGE_DATA_ERROR      0
#define   JUDGE_DATA_CORRECT    1

#define 	LEN_HEADER 	  5				//֡ͷ��
#define   LEN_CMDID     2       //�����볤��
#define   LEN_TAIL      2	      //֡βCRC16

typedef __packed struct
{
    uint8_t SOF;
    uint16_t Data_Length;
    uint8_t Seq;
    uint8_t CRC8;

} frame_header_t;

typedef enum
{
    FRAME_HEADER         = 0,
    CMD_ID               = 5,
    DATA                 = 7,
    STU_HEADER					 = 7,
    STU_DATA             = 13
} JudgeFrameOffset;

//5�ֽ�֡ͷ,ƫ��λ��
typedef enum
{
    SOF          = 0,//��ʼλ
    DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
    SEQ          = 3,//�����
    CRC8         = 4 //CRC8
} frame_header_OFFSET;






#define Referee_UART huart6
#define Referee_IRQHandler USART6_IRQHandler

#define REFEREE_USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH     1024




#define HEADER_SOF                  0xA5

#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE       2
#define REF_PROTOCOL_CRC16_SIZE     2

#define REF_HEADER_CRC_LEN          (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN    (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN        (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
	GAME_STATE_CMD_ID                 = 0x0001,  //����״̬����
	GAME_RESULT_CMD_ID                = 0x0002,  //�����������
	GAME_ROBOT_HP_CMD_ID              = 0x0003,  //������Ѫ������
	DART_FLYING_STATE_CMD_ID          = 0x0004,  //���ڷ����״̬
	
	FIELD_EVENTS_CMD_ID               = 0x0101,  //�����¼�����
	SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,  //����վ������ʶ
	SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,  //���󲹸�վ�������ݣ��ɲ����ӷ��ͣ�RM �Կ�����δ���ţ�
	REFEREE_WARNING_CMD_ID            = 0x0104,  //���о�����Ϣ
	DART_REMAINING_TIME_CMD_ID        = 0x0105,  //���ڷ���ڵ���ʱ
	
	ROBOT_STATE_CMD_ID                = 0x0201,  //����������״̬
	POWER_HEAT_DATA_CMD_ID            = 0x0202,  //ʵʱ������������
	ROBOT_POS_CMD_ID                  = 0x0203,  //������λ��
	BUFF_MUSK_CMD_ID                  = 0x0204,  //����������
	AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,  //���л���������״̬
	ROBOT_HURT_CMD_ID                 = 0x0206,  //�˺�״̬
	SHOOT_DATA_CMD_ID                 = 0x0207,  //ʵʱ�����Ϣ
	BULLET_REMAINING_CMD_ID           = 0x0208,  //�ӵ�ʣ�෢����
	ROBOT_RFID_STATE_CMD_ID           = 0x0209,  //������RFID״̬
	DART_CLIENT_CMD_ID                = 0x020A,  //���ڻ����˿ͻ���ָ������
	ROBOT_POSITION_DM_ID              = 0x020B,  //�ڱ����������λ������  
	
	STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,  //�����˼�ͨ��
  ROBOT_COMMAND_CMD_ID              = 0x0303,  //С��ͼ�·���Ϣ��ʶ
  CLIENT_MAP_COMMAND_CMD_ID         = 0x0305,  //С��ͼ������Ϣ��ʶ
	
	IDCustomData,
}referee_cmd_id_e;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef __packed struct
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header_struct_t;

typedef __packed struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)


/* Referee Defines -----------------------------------------------------------*/
/* �������� */
#define Game_Type_RMUC     1 //�����Կ���
#define Game_Type_RMUT     2 //������
#define Game_Type_RMUA     3 //�˹�������ս��
#define Game_Type_RMUL_3V3 4 //��У������3V3
#define Game_Type_RMUL_1V1 5 //��У������1V1

/* �����׶� */
#define Game_Progress_Unstart   0 //δ��ʼ����
#define Game_Progress_Prepare   1 //׼���׶�
#define Game_Progress_SelfCheck 2 //�Լ�׶�
#define Game_Progress_5sCount   3 //5s����ʱ
#define Game_Progress_Battle    4 //��ս��
#define Game_Progress_Calculate 5 //����������

/* ������� */
#define Game_Result_Draw    0 //ƽ��
#define Game_Result_RedWin  1 //�췽ʤ��
#define Game_Result_BlueWin 2 //����ʤ��

/* ������Ϣ */
#define Warning_Yellow  1 //���ƾ���
#define Warning_Red     2 //���ƾ���
#define Warning_Failure 3 //�и�

/* ������ID */
#define Robot_ID_Red_Hero         1 //�췽Ӣ��
#define Robot_ID_Red_Engineer     2 //�췽����
#define Robot_ID_Red_Infantry3    3 //�췽����3
#define Robot_ID_Red_Infantry4    4 //�췽����4
#define Robot_ID_Red_Infantry5    5 //�췽����5
#define Robot_ID_Red_Aerial       6 //�췽���˻�
#define Robot_ID_Red_Sentry       7 //�췽�ڱ�
#define Robot_ID_Red_Darts        8 //�췽����
#define Robot_ID_Red_Radar        9 //�췽�״�
#define Robot_ID_Blue_Hero      101 //����Ӣ��
#define Robot_ID_Blue_Engineer  102 //��������
#define Robot_ID_Blue_Infantry3 103 //��������3
#define Robot_ID_Blue_Infantry4 104 //��������4
#define Robot_ID_Blue_Infantry5 105 //��������5
#define Robot_ID_Blue_Aerial    106 //�������˻�
#define Robot_ID_Blue_Sentry    107 //�����ڱ�
#define Robot_ID_Blue_Darts     108 //��������
#define Robot_ID_Blue_Radar     109 //�����״�

/* �����˵ȼ� */
#define Robot_Level_1 1 //1��
#define Robot_Level_2 2 //2��
#define Robot_Level_3 3 //3��

/* ��Ѫ���� */
#define Hurt_Type_ArmoredPlate     0 //װ�װ��˺�
#define Hurt_Type_ModuleOffline    1 //ģ������
#define Hurt_Type_OverShootSpeed   2 //ǹ�ڳ�����
#define Hurt_Type_OverShootHeat    3 //ǹ�ܳ�����
#define Hurt_Type_OverChassisPower 4 //���̳�����
#define Hurt_Type_Collision        5 //װ��ײ��

/* ���������� */
#define Shooter_ID1_17mm 1 //1��17mm�������
#define Shooter_ID2_17mm 2 //2��17mm�������
#define Shooter_ID1_42mm 3 //1��42mm�������

/* ������Ϣ */
#define Dart_State_Open     0 //����բ�ſ���
#define Dart_State_Close    1 //����բ�Źر�
#define Dart_State_Changing 2 //���ڿ������߹ر���
#define Dart_Target_Outpost 0 //����Ŀ��Ϊǰ��վ
#define Dart_Target_Base    1 //����Ŀ��Ϊ����

/* ������ID */
#define Cilent_ID_Red_Hero       0x0101 //�췽Ӣ�۲�����
#define Cilent_ID_Red_Engineer   0x0102 //�췽���̲�����
#define Cilent_ID_Red_Infantry3  0x0103 //�췽����3������
#define Cilent_ID_Red_Infantry4  0x0104 //�췽����4������
#define Cilent_ID_Red_Infantry5  0x0105 //�췽����5������
#define Cilent_ID_Red_Aerial     0x0106 //�췽����
#define Cilent_ID_Blue_Hero      0x0165 //����Ӣ�۲�����
#define Cilent_ID_Blue_Engineer  0x0166 //�������̲�����
#define Cilent_ID_Blue_Infantry3 0x0167 //��������3������
#define Cilent_ID_Blue_Infantry4 0x0168 //��������4������
#define Cilent_ID_Blue_Infantry5 0x0169 //��������5������
#define Cilent_ID_Blue_Aerial    0x016A //��������

typedef __packed struct  //����Ƶ��
{
	uint16_t RX_Frequent;
	uint16_t RX_add;
} rx_msg;


/* 0x000X --------------------------------------------------------------------*/
typedef __packed struct  //0x0001 ����״̬����
{
	uint8_t  game_type : 4;
	uint8_t  game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct  //0x0002 �����������
{
	uint8_t winner;
} ext_game_result_t;

typedef __packed struct  //0x0003 ������Ѫ������
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;	
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* 0x010X --------------------------------------------------------------------*/
typedef __packed struct  //0x0101 �����¼�����
{
  uint32_t event_type;
} ext_event_data_t;

typedef __packed struct  //0x0102 ����վ������ʶ
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct //0x0103 ���󲹸�վ�������ݣ��ɲ����ӷ��ͣ�RM �Կ�����δ���ţ�
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct  //0x0104 ���о�����Ϣ
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;

typedef __packed struct  //0x0105 ���ڷ���ڵ���ʱ
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* 0x020X --------------------------------------------------------------------*/
typedef __packed struct  //0x0201 ����������״̬
{
	uint8_t  robot_id;
	uint8_t  robot_level;
	uint16_t current_HP; 
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t  power_management_gimbal_output : 1;
	uint8_t  power_management_chassis_output : 1;
	uint8_t  power_management_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct  //0x0202 ʵʱ������������
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float    chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct  //0x0203 ������λ��
{
	float x;
	float y;
	float z;
	float angle;
} ext_game_robot_pos_t;

typedef __packed struct  //0x0204 ����������
{

 uint8_t recovery_buff;
 uint8_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;
 uint16_t attack_buff;
} ext_buff_musk_t;

typedef __packed struct  //0x0205 ���л���������״̬
{
	uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct  //0x0206 �˺�״̬
{
	uint8_t armor_type : 4;
	uint8_t hurt_type  : 4;
} ext_robot_hurt_t;

typedef __packed struct  //0x0207 ʵʱ�����Ϣ
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float   bullet_speed;
	rx_msg RX_MSG;
} ext_shoot_data_t;

typedef __packed struct  //0x0208 �ӵ�ʣ�෢����
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct  //0x0209 ������RFID״̬
{
	uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct  //0x020A ���ڻ����˿ͻ���ָ������
{
	uint8_t  dart_launch_opening_status;
	uint8_t  dart_attack_target;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 0x030X --------------------------------------------------------------------*/
typedef __packed struct  //0x0301 �����˼�ͨ�� ͷ�ṹ��
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef __packed struct  //0x0301 �����˼�ͨ�� ���ݽṹ��
{
	uint8_t *data;
} robot_interactive_data_t;

typedef __packed struct  //0x0303 С��ͼ�·���Ϣ��ʶ
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
} ext_robot_command_t;

typedef __packed struct  //0x0305 С��ͼ������Ϣ��ʶ
{
	uint16_t target_robot_ID;
	float    target_position_x;
	float    target_position_y;
} ext_client_map_command_t;


/* Structs -------------------------------------------------------------------*/
/* protocol��ͷ�ṹ�� */
extern frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
extern ext_game_status_t   Game_Status;
extern ext_game_result_t   Game_Result;
extern ext_game_robot_HP_t Game_Robot_HP;

/* 0x010X */
extern ext_event_data_t                Event_Data;
extern ext_supply_projectile_action_t  Supply_Projectile_Action;
extern ext_supply_projectile_booking_t Supply_Projectile_Booking;
extern ext_referee_warning_t           Referee_Warning;
extern ext_dart_remaining_time_t       Dart_Remaining_Time;

/* 0x020X */
extern ext_game_robot_state_t Game_Robot_State;
extern ext_power_heat_data_t  Power_Heat_Data;
extern ext_game_robot_pos_t   Game_Robot_Pos;
extern ext_buff_musk_t        Buff_Musk;
extern aerial_robot_energy_t  Aerial_Robot_Energy;
extern ext_robot_hurt_t       Robot_Hurt;
extern ext_shoot_data_t       Shoot_Data;
extern ext_bullet_remaining_t Bullet_Remaining;
extern ext_rfid_status_t      RFID_Status;
extern ext_dart_client_cmd_t  Dart_Client_Cmd;

/* 0x030X */
extern ext_student_interactive_header_data_t Student_Interactive_Header_Data;
extern robot_interactive_data_t              Robot_Interactive_Data;
extern ext_robot_command_t                   Robot_Command;
extern ext_client_map_command_t              Client_Map_Command;

/* ����ϵͳ����˫������ */
extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* ����ϵͳ�������ݶ��� */
extern fifo_s_t Referee_FIFO;
extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol�������ṹ�� */
extern unpack_data_t Referee_Unpack_OBJ;

/* Functions -----------------------------------------------------------------*/
void Referee_StructInit(void);
void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength);
void Referee_SolveFifoData(uint8_t *frame);
void Referee_UnpackFifoData(unpack_data_t *p_obj, fifo_s_t *referee_fifo);




//����֡
typedef __packed struct
{
    frame_header_t frame_header;
    uint16_t cmd_id;
    referee_cmd_id_e data;
    uint16_t  frame_tail;	//crc16����E�сE
}frame_t;
/**********************************ѧ��������֮���ͨ��******************************************/
typedef enum
{
    LEN_SEVEN_GRAPH = 105,
    LEN_STU_HEAD = 6,
    LEN_SINGLE_GRAPH = 15
}length_stu_to_judge;



typedef enum
{
    absolute_position_id						 					= 0x0200,
    radar_enermies_position_id										= 0x0201,
    aerial_enermies_position_id										= 0x0202,

    client_custom_graphic_delete_id									= 0x0100,
    client_custom_graphic_single_id									= 0x0101,
    client_custom_graphic_double_id									= 0x0102,
    client_custom_graphic_five_id									= 0x0103,
    client_custom_graphic_seven_id									= 0x0104,
    client_custom_character_id										= 0x0110
}data_cmd_id_t;



typedef struct
{
    frame_header_t frame_header;																					//֡ͷ
    uint16_t cmd_id;																											//������
    ext_student_interactive_header_data_t student_interactive_header_data;//���ݶ�ͷ�ṹ
    uint8_t student_interactive_data[20];																//��������
    uint16_t frame_tail;																									//֡β��16λcrcУ�飩
}send_to_teammate;


bool Judge_Read_Data(uint8_t *ReadFromUsart);
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
uint16_t determine_ID(void);
bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
uint8_t JUDGE_usGetPowerLimit(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat42(void);
float JUDGE_usGetSpeed42(void);
void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);
uint16_t JUDGE_usGetHeatLimit(void);
uint8_t JUDGE_usGetSpeedLimit(void);
uint16_t JUDGE_usGetShootCold(void);
bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);
bool Judge_If_Near_Death(void);
uint16_t choose_client(uint8_t robot_id);
uint8_t JUDGE_GameState(void);


#endif
