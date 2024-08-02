//#ifndef MYSYSTEM_H
//#define MYSYSTEM_H
//#include "main.h"
////#include "type.h"
////void LimitValue_16(int16_t* VALUE,int16_t MAX,int16_t MIN);
////void LimitValue_u16(uint16_t* VALUE,uint16_t MAX,uint16_t MIN);
////float constrain_float(float amt, float low, float high);
////float RampInc_float( float *buffer, float now, float ramp );
////void AngleLoop (float* angle ,float max);
////void AngleLoop_f (float* angle ,float max);
////float RAMP_float( float final, float now, float ramp );
////void LimitValue_f(float* VALUE,float MAX,float MIN);
////float abs_float(float a);
////float queue_sum(int n,float*yaw_smooth,float*yaw_smooth1,float item1);

////typedef struct
////{
//////  uint8_t flag;
////	bool shoot_single_flag;
////  bool shoot_triple_flag;
////	bool shoot_normal_flag;
////	
////	bool stuck_flag;
////	bool heat_flag;

////  uint16_t shoot_cnt;
////  uint32_t shoot_left_time;
////  uint8_t shoot_single_finish_flag;
////  uint32_t shoot_single_time;
////  uint8_t Ammunition_flag; //ң����ģʽʹ�ã��ڿ��ز�����Ӧλ����ʱ���濪�ط�������
////  uint8_t moca_flag;       //����ͬ��
////  bool protect_flag_sutck; //������ģʽ����������ģʽѡ�� ��ֹ���������ɱ����
////  bool protect_flag_heat;  //����ͬ��
////  
////  bool FLAG_Remote;
////  bool follow_flag_remote;
////  bool FLAG_Key;
////  bool follow_flag_key;
////  bool chassis_follow_flag;
////	
////  bool Chassis_Switch_C;
////  uint8_t Chassis_Key_C_Change;  
////  bool Chassis_Switch_F;
////  uint8_t Chassis_Key_F_Change;
////  bool Chassis_Switch_X;
////  uint8_t Chassis_Key_X_Change;
////  bool Chassis_Switch_G;
////  uint8_t Chassis_Key_G_Change;
////  bool Chassis_Switch_Q;
////  uint8_t Chassis_Key_Q_Change;
////  bool Chassis_Switch_E;
////  uint8_t Chassis_Key_E_Change;
//////  bool Chassis_Switch_B;
//////  uint8_t Chassis_Key_B_Change;
////  bool Chassis_Switch_R;
////  uint8_t Chassis_Key_R_Change;
////  bool Chassis_Switch_Z;
////  uint8_t Chassis_Key_Z_Change;
////  bool Chassis_Switch_Shift;
////  uint8_t Chassis_Key_Shift_Change;
//////  
////  bool Gimbal_Switch_Ctrl;
////  uint8_t Gimbal_Key_Ctrl_Change;
//////  
//////  bool Gimbal_Switch_V;
//////  uint8_t Gimbal_Key_V_Change;
////  
////} flag_t;//����flag ״̬λ

////extern flag_t Flag_status;//����flag ״̬λ

////#define	GET_LOW_BYTE0(x)	((x >>  0) & 0x000000ff)	/* ��ȡ��0���ֽ� */
////#define	GET_LOW_BYTE1(x)	((x >>  8) & 0x000000ff)	/* ��ȡ��1���ֽ� */
////#define	GET_LOW_BYTE2(x)	((x >> 16) & 0x000000ff)	/* ��ȡ��2���ֽ� */
////#define	GET_LOW_BYTE3(x)	((x >> 24) & 0x000000ff)	/* ��ȡ��3���ֽ� */

////#define	GET_BIT(x, bit)	((x & (1 << bit)) >> bit)	/* ��ȡ��bitλ */

////#define	CLEAR_LOW_BYTE0(x)	(x &= 0xffffff00)	/* �����0���ֽ� */
////#define	CLEAR_LOW_BYTE1(x)	(x &= 0xffff00ff)	/* �����1���ֽ� */
////#define	CLEAR_LOW_BYTE2(x)	(x &= 0xff00ffff)	/* �����2���ֽ� */
////#define	CLEAR_LOW_BYTE3(x)	(x &= 0x00ffffff)	/* �����3���ֽ� */

////#define	Clear_Bit(x, bit)	(x &= ~(1 << bit))	/* �����bitλ */
////#define	Set_Bit(x, bit)	(x |= (1 << bit))	/* ��λ��bitλ */

////#define	SET_LOW_BYTE0(x)  (x |= 0x000000ff)	/* ��0���ֽ���1 */	
////#define	SET_LOW_BYTE1(x)  (x |= 0x0000ff00)	/* ��1���ֽ���1 */	
////#define	SET_LOW_BYTE2(x)  (x |= 0x00ff0000)	/* ��2���ֽ���1 */	
////#define	SET_LOW_BYTE3(x)  (x |= 0xff000000)	/* ��3���ֽ���1 */



/////* ��ȡ��[n:m]λ��ֵ */
////#define BIT_M_TO_N(x, m, n)  ((unsigned int)(x << (31-(n))) >> ((31 - (n)) + (m)))
//#endif
