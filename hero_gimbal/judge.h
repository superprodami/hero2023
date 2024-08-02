#ifndef __judge_h
#define __judge_h

//#include "param.h"
//#include "type.h"
//#include "mysystem.h"
#include "stdbool.h"

#include "stdint.h"
#include "stm32f4xx_hal.h"
//#define JUDGE_BUFFER_LEN           200
//#define IT_MASK                   ((uint16_t)0x001F)
#define JUDGE_MAX_LENGTH           2000

extern uint8_t judge_rx_buff[JUDGE_MAX_LENGTH];
extern uint8_t Judge_Self_ID;//当前机器人的ID
extern uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID
//extern send_to_teammate  CommuData;
//定义帧头
//起始字节,协议固定为0xA5
#define   Judge_Data_SOF 0xA5

//#define		JUDGE_18		18
//#define		JUDGE_19		19
//#define		JUDGE_VERSION	JUDGE_19

#define   JUDGE_DATA_ERROR      0
#define   JUDGE_DATA_CORRECT    1

#define 	LEN_HEADER 	  5				//帧头长
#define   LEN_CMDID     2       //命令码长度
#define   LEN_TAIL      2	      //帧尾CRC16

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

//5字节帧头,偏移位置
typedef enum
{
    SOF          = 0,//起始位
    DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
    SEQ          = 3,//包序号
    CRC8         = 4 //CRC8
} frame_header_OFFSET;

/***********************************************************
 以下数值找2021裁判系统串口协议附录
***********************************************************/

//命令码ID,用来判断接收的是什么数据
typedef enum
{
    game_status_t						 					  = 0x0001,//比赛状态数据
    game_result_t						 					  = 0x0002,//比赛结果数据
    game_robot_HP_t 										= 0x0003,//比赛机器人血量数据
    dart_status_t 											= 0x0004,//飞镖发射状态
    ICRA_buff_debuff_zone_status_t 		  = 0x0005,//人工智能挑战赛加成与惩罚状态
    event_data_t  											= 0x0101,//场地事件数据
    supply_projectile_action_t  				= 0x0102,//场地补给站动作标识数据
//    ID_supply_projectile_booking      	= 0x0103,//场地补给站预约子弹数据(RM对抗赛尚未开放)
    referee_warning_t  									= 0x0104,//裁判警告数据
    dart_remaining_time_t  							= 0x0105,//飞镖发射口倒计时
    game_robot_status_t  								= 0x0201,//机器人状态数据
    power_heat_data_t 									= 0x0202,//实时功率热量数据
    game_robot_pos_t										= 0x0203,//机器人位置数据
    buff_t  														= 0x0204,//机器人增益数据
    aerial_robot_energy_t 							= 0x0205,//空中机器人能量状态数据
    robot_hurt_t  											= 0x0206,//伤害状态数据
    shoot_data_t  											= 0x0207,//实时射击数据
    bullet_remaining_t  								= 0x0208,//子弹剩余发送数
    rfid_status_t  											= 0x0209,//机器人RFID状态
    dart_client_cmd_t  									= 0x020A,//飞镖机器人客户端指令书
    student_interactive_header_data_t  	= 0x0301,//机器人之间交互数据
    robot_interactive_data_t            = 0x0302,//自定义控制器交互数据接口
    smallmap_communicate_t              = 0x0303,//客户端小地图交互数据
    robot_command_t                     = 0x0304,//键盘，鼠标信息
		client_map_command_t                = 0x0305//小地图接收信息标识
} frame_cmd_id_t;

//命令码数据段长，根据官方协议来定义长度
typedef enum
{
    len_game_status = 11,
    len_game_result = 1,
    len_game_robot_HP = 32,//len_game_robot_HP = 28,
    len_dart_status = 3,
    len_ICRA_buff_debuff_zone_status = 11,
    len_event_data = 4,
    len_supply_projectile_action = 4,//3
//    len_ID_supply_projectile_booking = 2,
    len_referee_warning = 2,
    len_dart_remaining_time = 1,
    len_game_robot_status = 27,//15
    len_power_heat_data = 16,//14
    len_game_robot_pos = 16,
    len_buff = 1,
    len_aerial_robot_energy = 1,//3
    len_robot_hurt = 1,
    len_shoot_data = 7,//6
    len_bullet_remaining = 6,//2
    len_rfid_status = 4,
    len_dart_client_cmd = 6,
    len_student_interactive_header_data = 20,//自定义数值
    len_robot_interactive_data = 20,//自定义数值
    len_smallmap_communicate = 15,
    len_robot_command = 12,
		len_client_map_command = 10
} cmd_id_len_t;

typedef enum
{
    red_robot				= 0,
    red_hero 				= 1,
    red_engineer 		= 2,
    red_infantry_3 	= 3,
    red_infantry_4 	= 4,
    red_infantry_5 	= 5,
    red_aerial     	= 6,
    red_sentry		 	= 7,
    red_radar			 	= 9,
		red_outpost     = 10,
		red_base        = 11,

    blue_robot				= 100,
    blue_hero			 		= 101,
    blue_engineer 		= 102,
    blue_infantry_3 	= 103,
    blue_infantry_4 	= 104,
    blue_infantry_5 	= 105,
    blue_aerial     	= 106,
    blue_sentry		 		= 107,
    blue_radar			 	= 109,
		blue_outpost      = 110,
		blue_base         = 111,

} robot_type_t;

typedef enum
{
    red_hero_client 				= 0x0101,
    red_engineer_client 		= 0x0102,
    red_infantry_3_client 	= 0x0103,
    red_infantry_4_client 	= 0x0104,
    red_infantry_5_client 	= 0x0105,
    red_aerial_client     	= 0x0106,

    blue_hero_client			 		= 0x0165,
    blue_engineer_client 			= 0x0166,
    blue_infantry_3_client 		= 0x0167,
    blue_infantry_4_client 		= 0x0168,
    blue_infantry_5_client 		= 0x0169,
    blue_aerial_client    	 	= 0x016A,
} robot_client_type_t;

/********************************************************裁判系统发送信息***********************************************************************/
//ID：0x0001 Byte： 11  比赛状态数据
typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;  //0未开始 1准备 2自检 3 5s倒计时 4对战 5比赛结算中
    uint16_t stage_remain_time; //当前状态剩余时间
    uint64_t SyncTimeStamp;
} ext_game_status_t;

//ID：0x0002 Byte：1 比赛结果数据
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;
// 0x0003
typedef __packed struct
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
//0x0004
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;
//0x0005
typedef __packed struct
{
    uint8_t F1_zone_status: 1;
    uint8_t F1_zone_buff_debuff_status: 3;
    uint8_t F2_zone_status: 1;
    uint8_t F2_zone_buff_debuff_status: 3;
    uint8_t F3_zone_status: 1;
    uint8_t F3_zone_buff_debuff_status: 3;
    uint8_t F4_zone_status: 1;
    uint8_t F4_zone_buff_debuff_status: 3;
    uint8_t F5_zone_status: 1;
    uint8_t F5_zone_buff_debuff_status: 3;
    uint8_t F6_zone_status: 1;
    uint8_t F6_zone_buff_debuff_status: 3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;
//ID：0x0101 Byte：4  场地事件数据
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;
//ID:0x0102  Byte:3  场地补给站动作
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;
//ID:0x0104
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;
//0x0105
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;
//0x0201
typedef __packed struct
{
    uint8_t robot_id;                         //机器人ID，可用来校验发送
    uint8_t robot_level;                      //1一级，2二级，3三级
    uint16_t remain_HP;                       //机器人剩余血量
    uint16_t max_HP;                          //机器人满血量
    uint16_t shooter_id1_17mm_cooling_rate;   //机器人 17mm 子弹热量冷却速度 单位 /s
    uint16_t shooter_id1_17mm_cooling_limit;  //机器人 17mm 子弹热量上限
    uint8_t shooter_id1_17mm_speed_limit;
    uint8_t shooter_id1_17mm_speed_limit_nothing;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint8_t shooter_id2_17mm_speed_limit;//低八位都是0
    uint8_t shooter_id2_17mm_speed_limit_nothing;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint8_t shooter_id1_42mm_speed_limit;//低八位都是0
    uint8_t shooter_id1_42mm_speed_limit_nothing;
    uint8_t chassis_power_limit_nothing;
    uint8_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

//typedef __packed struct
//{
//	uint8_t robot_id;
//	uint8_t robot_level;
//	uint16_t remain_HP;
//	uint16_t max_HP;
//	uint16_t shooter_id1_17mm_cooling_rate;
//	uint16_t shooter_id1_17mm_cooling_limit;
//	uint16_t shooter_id1_17mm_speed_limit
//	uint16_t shooter_id2_17mm_cooling_rate;
//	uint16_t shooter_id2_17mm_cooling_limit;
//	uint16_t shooter_id2_17mm_speed_limit;
//	uint16_t shooter_id1_42mm_cooling_rate;
//	uint16_t shooter_id1_42mm_cooling_limit;
//	uint16_t shooter_id1_42mm_speed_limit;
//	uint16_t chassis_power_limit;
//	uint8_t mains_power_gimbal_output : 1;
//	uint8_t mains_power_chassis_output : 1;
//	uint8_t mains_power_shooter_output : 1;
//} ext_game_robot_status_t;
//0x0202 实时功率热量数据
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;//瞬时功率
    uint16_t chassis_power_buffer;//60焦耳缓冲能量
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;
//ID: 0x0203 Byte:16  机器人位置数据
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;
//ID:0x0204 Byte:1  机器人增益数据
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;
//0x0205    空中机器人能量状态数据
typedef __packed struct
{
    uint8_t attack_time;
} ext_aerial_robot_energy_t;
//0x0206    伤害状态数据
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;
//0x0207    实时射击数据
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;
//0x0208
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;
//0x0209
typedef __packed struct
{
    uint32_t rfid_status;
}ext_rfid_status_t;
//0x020A
//typedef __packed struct
//{
//    uint8_t dart_launch_opening_status;
//    uint8_t dart_attack_target;
//    uint16_t target_change_time;
//    uint8_t first_dart_speed;
//    uint8_t second_dart_speed;
//    uint8_t third_dart_speed;
//    uint8_t fourth_dart_speed;
//    uint16_t last_dart_launch_time;
//    uint16_t operate_launch_cmd_time;
//}ext_dart_client_cmd_t;
typedef __packed struct
{
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;
//0x0302
typedef __packed struct
{
    uint8_t	data[30];
}ext_robot_interactive_data_t;
//0x0303
typedef __packed struct
{
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;
}ext_smallmap_communicate_t;
//0x0304
typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
}ext_robot_command_t;
//0x305
typedef __packed struct
{
	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
}ext_client_map_command_t;
//定义数据
typedef __packed struct
{
    ext_game_status_t 													game_status;									//比赛状态数据0x0001
    ext_game_result_t 													game_result;									//比赛结果数据0x0002
    ext_game_robot_HP_t 												game_robot_HP;								//机器人血量数据： 0x0003
    ext_dart_status_t  													dart_status;									//飞镖发射状态： 0x0004
    ext_ICRA_buff_debuff_zone_status_t 					ICRA_buff_debuff_zone_status;	//人工智能挑战赛加成与惩罚状态 0x0005
    ext_event_data_t 														event_data;										//场地事件数据： 0x0101
    ext_supply_projectile_action_t 							supply_projectile_action;			//场地补给站动作标识数据0x0102
    ext_referee_warning_t 											referee_warning;							//裁判警告信息： 0x0104
    ext_dart_remaining_time_t 									dart_remaining_time;					//飞镖发射口倒计时：0x0105
    ext_game_robot_status_t 										game_robot_status;						//机器人状态数据 0x0201
    ext_power_heat_data_t 											power_heat_data;							//实时功率热量数据： 0x0202
    ext_game_robot_pos_t 												game_robot_pos;								//机器人位置： 0x0203
    ext_buff_t 																	buff;													//机器人增益： 0x0204
    ext_aerial_robot_energy_t 									aerial_robot_energy;					//空中机器人能量状态： 0x0205
    ext_robot_hurt_t 														robot_hurt;										//伤害状态： 0x0206
    ext_shoot_data_t 														shoot_data;										//实时射击信息： 0x0207
    ext_bullet_remaining_t 											bullet_remaining;							//子弹剩余发射数： 0x0208
    ext_rfid_status_t 													rfid_status;									//机器人 RFID 状态： 0x0209
    ext_dart_client_cmd_t 											dart_client_cmd;							//飞镖机器人客户端指令书： 0x020A
    ext_robot_interactive_data_t                robot_interactive_data;       //0x0302
    ext_smallmap_communicate_t                  smallmap_communicate;         //0x0303
    ext_robot_command_t 												robot_command;                //0x0304
		ext_client_map_command_t										client_map_command;						//0x0305
    uint8_t 																		student_data[113];     				//学生数据
}frame_data_t;

//定义帧
typedef __packed struct
{
    frame_header_t frame_header;
    uint16_t cmd_id;
    frame_data_t data;
    uint16_t  frame_tail;	//crc16整皝EＱ丒
}frame_t;
/**********************************学生机器人之间的通信******************************************/
typedef enum
{
    LEN_SEVEN_GRAPH = 105,
    LEN_STU_HEAD = 6,
    LEN_SINGLE_GRAPH = 15
}length_stu_to_judge;

typedef __packed struct
{
    uint16_t data_cmd_id;			//数据内容id
    uint16_t sender_ID;				//发送者id
    uint16_t receiver_ID;			//接收者id
}ext_student_interactive_header_data_t;

typedef enum
{
    //?????????
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
    frame_header_t frame_header;																					//帧头
    uint16_t cmd_id;																											//命令码
    ext_student_interactive_header_data_t student_interactive_header_data;//数据段头结构
    uint8_t student_interactive_data[20];																//数据内容
    uint16_t frame_tail;																									//帧尾（16位crc校验）
}send_to_teammate;


bool Judge_Read_Data(uint8_t *ReadFromUsart);
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
void determine_ID(void);
bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
uint8_t JUDGE_usGetPowerLimit(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat42(void);
float JUDGE_usGetSpeedHeat42(void);
//void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);
uint16_t JUDGE_usGetHeatLimit(void);
uint8_t JUDGE_usGetSpeedLimit(void);
uint16_t JUDGE_usGetShootCold(void);
bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);
uint16_t choose_client(uint8_t robot_id);
#endif
