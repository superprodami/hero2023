/*
 * �������ݽ��ܱ��ĸ�ʽ
 * ��ʶ��:0x4FF
 *�����ӡ�  [2]0x03����ģʽ  [3]0xcc�����λ���ڲ����ְ��¸�λ����ʱ������ȷ���0xcc���������ٸ�λ
 *
 * [0]0xAE                                    �� �̶����ӱ�־
 * [1]0x1a                                    �� �����ź�1
 * [2]0x01�ػ� 0x02���� 0x03����ģʽ�����ŵ�  �������ַ���
 * [3]0xaa���� 0xbb���� oxcc�����λ          �� ����ϵͳѪ��0����ң�����رշ����ˣ���0�����ţ����ù��Ƿ��Դ�ϵ�
 * [4]��������0                               �� ���������������ڴ�[0]   
 * [5]��������1                               �� ���������������ڴ�[1]  
 * [6]��������2                               �� ���������������ڴ�[2]  
 * [7]��������3                               �� ���������������ڴ�[3]  
 
 * [0]0xAE                                    �� �̶����ӱ�־
 * [1]0x2b                                    �� �����ź�2
 * [2]0x01�ػ� 0x02���� 0x03����ģʽ�����ŵ�  �������ַ���
 * [3]0xaa���� 0xbb���� oxcc�����λ          �� ����ϵͳѪ��0����ң�����رշ����ˣ���0�����ţ����ù��Ƿ��Դ�ϵ�
 * [4]���й���0                               �� ���й����������ڴ�[0]  
 * [5]���й���1                               �� ���й����������ڴ�[1]  
 * [6]���й���2                               �� ���й����������ڴ�[2]  
 * [7]���й���3                               �� ���й����������ڴ�[3] 
 */
 
 
/*
 * �������ݷ��䱨�ĸ�ʽ
 * ��ʶ��:0x210 
 *
 *[1]�ֽڷ�1ʱ UI��ʾ ��Waiting��
 *һ��ʱ��δ�յ����������ź�ʱ�Զ��л�������dischargeģʽ����[2]�ֽڷ���0x01��������UI��ʾ��Error��
 *[2]�ֽڷ�������ͨ����ģʽ�ͷ���ģʽ����ڡ�UI���������֡�
 *
 * [0]0x1a                �� �����ź�1
 * [1]0x01                ,  �������� �������ݱ�ʾ�������ϻ����ڿ���,��ʱ����������ֵ��Ч������Ҫ���ڳ��ģʽ
 * [2]0x01�ػ� 0x02���� 0x03����ģʽ  ,  ʵʱ���ػ�״̬
 * [3]0x00                ,  û�����λ��ʲô
 * [4]ʣ������0           �� ʣ�������������ڴ�[0]
 * [5]ʣ������1           �� ʣ�������������ڴ�[1]
 * [6]ʣ������2           �� ʣ�������������ڴ�[2]
 * [7]ʣ������3           �� ʣ�������������ڴ�[3]
                          
 */
#include "cap.h"
#include "SQ_judge.h"
#include "bspcan.h"
//#include "mysystem.h"
#include "main.h"
extern eChassisAction actChassis_last;
float cap_percent;

CapInfo cap_info;

uint8_t robot_status = 0xbb; 
uint8_t cap_switch = 0x01;

uint16_t power_limit = 0;  //��������

/**
* @brief  ���ݸ���ģʽ�������ͺ���
  * @param  void
  * @retval void
  * @attention ���ݵı��� ģʽ�л����
  * CΪ������ GΪģʽ�л��� 
  */
void Send_cap_msg(void)
{
	static uint8_t cap_step = 0;
	float joule_residue = 0;  //ʣ�ཹ����������
	
	cap_info.cap_step++;
	if(cap_info.cap_step > 1000)
		cap_info.cap_step = 1000;
	
	if(cap_info.cap_step > 800)
	{
		cap_info.cap_status = 0x00;
	}
	
	if(ControlMode == KEYBOARD)
	{
	    /////////////////C�����ݿ���-�̰��л�����///////////////////
		if(!IF_KEY_PRESSED_C)
		{
			Flag_status.Chassis_Switch_C = 1;
		}
		if(IF_KEY_PRESSED_C && Flag_status.Chassis_Switch_C == 1)
		{
			Flag_status.Chassis_Switch_C = 0;
			Flag_status.Chassis_Key_C_Change ++;
			Flag_status.Chassis_Key_C_Change %= 2;
			
			if(Flag_status.Chassis_Key_C_Change && (cap_info.cap_status == CAP_STATUS_FLAG))
			{
				cap_switch = CAP_SWITCH_OPEN;
				cap_info.cap_mode = CAP_NOMAL;
			
			}else{
				cap_switch = CAP_SWITCH_CLOSE;
			}
		}
		
//	    /////////////////G���л�����ģʽ-��ͨģʽ�ͷ���ģʽ///////////////////
//		if(!IF_KEY_PRESSED_G)
//		{
//			Flag_status.Chassis_Switch_G = 1;
//		}
//		if(IF_KEY_PRESSED_G && Flag_status.Chassis_Switch_G == 1)
//		{
//			Flag_status.Chassis_Switch_G = 0;
//			if(cap_info.cap_mode == CAP_NOMAL){
//				cap_info.cap_mode = CAP_SLOPE;
//				cap_switch = CAP_SWITCH_SLOPE;
//				
//			}else if(cap_info.cap_mode == CAP_SLOPE){
//				cap_info.cap_mode = CAP_NOMAL;
//				cap_switch = CAP_SWITCH_OPEN;
//			}
//		}
		///////////����������ʣ��С��1%ʱ�Զ��ر�--��ֹ��������ʹ�õ��ݳ�����/////////////
		if(cap_info.cap_joule_residue <= 1) 
			cap_switch = CAP_SWITCH_CLOSE;
			
	}
	
	if(!Judge_If_Death())
	{
		robot_status = CAP_STATUS_WORK;
	}else{
		
		robot_status = CAP_STATUS_OFFWORK;
	}
	
	if(ControlMode == REMOTE)
	{
//		if(rc.sw1 == 1 && rc.sw2 == 2)
//		{
//			robot_status = CAP_STATUS_OFFWORK;
//		}
//		else
//		{
//			robot_status = CAP_STATUS_WORK;
//		}
		robot_status = CAP_STATUS_OFFWORK;
	}
	
	if(++cap_step == 7)  //��������
	{
		/*��ȡ��������*/			
		power_limit = JUDGE_usGetPowerLimit();
		/*ʣ�ཹ������*/
		joule_residue = JUDGE_fGetRemainEnergy();
		
		set_cap0(&hcan2, robot_status, cap_switch, joule_residue);
		set_cap1(&hcan2, robot_status, cap_switch, power_limit);
		
		cap_step = 0;
	
	}
}

/*
 * �������ݽ��ܱ��ĸ�ʽ
 * ��ʶ��:0x4FF
 *
 * [0]0xAE                �� �̶����ӱ�־
 * [1]0x1a                �� �����ź�1
 * [2]0x01�ػ� 0x02����   �� �����ַ���
 * [3]0xaa���� 0xbb���� 0xcc����   �� ����ϵͳѪ��0�����ˣ���0�����ţ����ù��Ƿ��Դ�ϵ�
 * [4]��������0           �� ���������������ڴ�[0]   
 * [5]��������1           �� ���������������ڴ�[1]  
 * [6]��������2           �� ���������������ڴ�[2]  
 * [7]��������3           �� ���������������ڴ�[3]  
 
 * [0]0xAE                �� �̶����ӱ�־
 * [1]0x2b                �� �����ź�2
 * [2]0x01�ػ� 0x02����   �� �����ַ���
 * [3]0xaa���� 0xbb����   �� ����ϵͳѪ��0�����ˣ���0�����ţ����ù��Ƿ��Դ�ϵ�
 * [4]���й���0           �� ���й����������ڴ�[0]  
 * [5]���й���1           �� ���й����������ڴ�[1]  
 * [6]���й���2           �� ���й����������ڴ�[2]  
 * [7]���й���3           �� ���й����������ڴ�[3]   
 */
void set_cap0(CAN_HandleTypeDef* hcan, s8 robot_status, s8 cap_switch, float joule_residue)
{
//	float    Joule_Residue = 0;//ʣ�ཹ����������
	JudgeValue.joule_residue.value = joule_residue;
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x4FF;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xAE;
	Txtemp[1] = 0x1a;
	Txtemp[2] =	cap_switch;
	Txtemp[3] =	robot_status;
	Txtemp[4] =	JudgeValue.joule_residue.buff[0];
	Txtemp[5] = JudgeValue.joule_residue.buff[1];
	Txtemp[6] = JudgeValue.joule_residue.buff[2];
	Txtemp[7] = JudgeValue.joule_residue.buff[3];
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}

void set_cap1(CAN_HandleTypeDef* hcan, s8 robot_status, s8 cap_switch, float power_limit)
{
	JudgeValue.power_limit.value = power_limit;
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x4FF;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xAE;
	Txtemp[1] = 0x2b;
	Txtemp[2] =	cap_switch;
	Txtemp[3] =	robot_status;
	Txtemp[4] =	JudgeValue.power_limit.buff[0];
	Txtemp[5] = JudgeValue.power_limit.buff[1];
	Txtemp[6] = JudgeValue.power_limit.buff[2];
	Txtemp[7] = JudgeValue.power_limit.buff[3];
	while( HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 );
	if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}

/*
 * �������ݷ��䱨�ĸ�ʽ
 * ��ʶ��:0x210 
 *
 * [0]0x1a                �� �����ź�1
 * [1]0x01                ,  �������� �������ݱ�ʾ�������ϻ����ڿ���
 * [2]0x01�ػ� 0x02���� 0x03����   ,  ʵʱ���ػ�״̬
 * [3]0x00                ,  û�����λ��ʲô
 * [4]ʣ������0           �� ʣ�������������ڴ�[0]
 * [5]ʣ������1           �� ʣ�������������ڴ�[1]
 * [6]ʣ������2           �� ʣ�������������ڴ�[2]
 * [7]ʣ������3           �� ʣ�������������ڴ�[3]
 
 * [0]0x2b                �� �����ź�2
 * [1]0x01                ,  �������� �������ݱ�ʾ�������ϻ����ڿ���
 * [2]0x01�ػ� 0x02����   ,  ʵʱ���ػ�״̬
 * [3]0x00                ,  û�����λ��ʲô
 * [4]�������0           �� ��������������ڴ�[0]
 * [5]�������1           �� ��������������ڴ�[1]
 * [6]�������2           �� ��������������ڴ�[2]
 * [7]�������3           �� ��������������ڴ�[3] 
 
 * [0]0x3c                �� �����ź�3
 * [1]0x01                ,  �������� �������ݱ�ʾ�������ϻ����ڿ���
 * [2]0x01�ػ� 0x02����   ,  ʵʱ���ػ�״̬
 * [3]0x00                ,  û�����λ��ʲô
 * [4]���빦��0           �� ���빦���������ڴ�[0]
 * [5]���빦��1           �� ���빦���������ڴ�[1]
 * [6]���빦��2           �� ���빦���������ڴ�[2]
 * [7]���빦��3           �� ���빦���������ڴ�[3]
 */
void get_cap_info(uint8_t temp[])
{
	if(temp[0] == 0x1a)
	{
		cap_info.cap_status = temp[1];
		cap_info.switch_status = temp[2];
		if(cap_info.cap_status == CAP_STATUS_FLAG)
		{
			JudgeValue.cap_joule_residue.buff[0] = temp[4];
			JudgeValue.cap_joule_residue.buff[1] = temp[5];
			JudgeValue.cap_joule_residue.buff[2] = temp[6];
			JudgeValue.cap_joule_residue.buff[3] = temp[7];
			cap_info.cap_joule_residue = JudgeValue.cap_joule_residue.value;
		}
		
	}
	cap_info.cap_step = 0;
    cap_percent = cap_info.cap_joule_residue;
}

void cap_init(void)
{
	cap_info.cap_mode = CAP_NOMAL;
	cap_info.cap_step = 0;
}
