#include "cap.h"
#include "judge.h"
uint8_t cap_buf[CAP_BUFLEN];
uint8_t Cap_txbuffer[3]={0};
uint8_t Capuse;

frames Frames;//����֡ 
power  Power;
uint8_t cap_mode;
void Frames_init(frames *vframes)
{
	vframes->Frame_Header1=0xAA ;
	vframes->Frame_Header2=0x0A ;
	Power.power_set = 60;
//	vframes->Frame_Len =0;
//	vframes->Frame_Verify =0x01;  
//	//����λ
//	vframes->Voltage_Input = 0x11;//Ϊ������У��λ���д�0x11��ʼ
//	vframes->Current_Input = 0x12;
//	vframes->Voltage_Output = 0x13;
//	vframes->Current_Output = 0x14;
//	vframes->Voltage_Cap_Input = 0x15;
//	vframes->Current_Cap_Input = 0x16;
//	vframes->power_set = 0x17;
//	vframes->power_input = 0x18;//���빦��
//	vframes->power_cap = 0x19;//�������ݳ�繦��
//	vframes->power_output = 0x1A;//�������
//	vframes->cap_flag = 0x20;
//	vframes->cap_full_flag = 0x21;
}


void Cap_callback_handler(uint8_t *buff)
{
	if(buff[0]==0XAA &&buff[13]==0X0A)
	{
		Cap_buffer_analysis(buff);
	}
}

void Cap_buffer_analysis(uint8_t *buff)
{
		Power.power_input=(buff[1]|buff[2]<<8)/100.00;
		Power.power_output=(buff[3]|buff[4]<<8)/100.00;
		Power.power_cap=(buff[5]|buff[6]<<8)/100.00;
		Power.power_cap_percentage=(buff[7]|buff[8]<<8)/100.00;
		Power.power_energy=(buff[9]|buff[10]<<8)/10.00;
		Power.cap_flag=buff[11]>>1;
		Power.cap_full_flag=buff[11]&0x01;
}

void Package_Frame(uint8_t _cap_mode)
{
  if(JUDGE_GameState() < 4)
    Power.power_set = 40;
  else
    Power.power_set = JUDGE_usGetPowerLimit();
	Cap_txbuffer[0] = 0XAA;
	Cap_txbuffer[1] = (uint8_t)(Power.power_set|((_cap_mode)<<7));
	Cap_txbuffer[2] = 0X0A;
}

void Send_cap_msg(uint8_t _cap_mode)
{
	Package_Frame(_cap_mode);
	for(int i = 4; i > 0;i--)
	{
//		HAL_UART_Transmit(&huart6,Cap_txbuffer, sizeof(Cap_txbuffer),10);
	}
}


//���ݵ�ѹ���ȶ� �� �޸ĵ����ͷŵ�ѹ
//���ݷŵ�����ֹͣ�ŵ�����ݳ�磬���ݱ��������������˶�   ��   ���ݺ��û�ͨ�ţ����û�֪��ʲôʱ���������
