//#include "mysystem.h"

//flag_t Flag_status;//����flag ״̬λ

//void LimitValue_f(float*VALUE, float MAX, float MIN)
//{
//    if(*VALUE > MAX)
//        *VALUE = MAX;
//    else if(*VALUE < MIN)
//        *VALUE = MIN;
//}

//void LimitValue_u16(uint16_t *VALUE, uint16_t MAX, uint16_t MIN)
//{
//    if(*VALUE > MAX)
//        *VALUE = MAX;
//    else if(*VALUE < MIN)
//        *VALUE = MIN;
//}

//void LimitValue_16(int16_t *VALUE, int16_t MAX, int16_t MIN)
//{
//    if(*VALUE > MAX)
//        *VALUE = MAX;
//    else if(*VALUE < MIN)
//        *VALUE = MIN;
//}

//float constrain_float(float amt, float low, float high)
//{
//    if (amt < low)
//        return low;
//    else if (amt > high)
//        return high;
//    else
//        return amt;
//}

///**
//  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
//  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
//  * @retval Ŀ�������
//  * @attention
//  *
//*/
//float RampInc_float( float *buffer, float now, float ramp )
//{

//    if (*buffer > 0)
//    {
//        if (*buffer > ramp)
//        {
//            now     += ramp;
//            *buffer -= ramp;
//        }
//        else
//        {
//            now     += *buffer;
//            *buffer  = 0;
//        }
//    }
//    else
//    {
//        if (*buffer < -ramp)
//        {
//            now     += -ramp;
//            *buffer -= -ramp;
//        }
//        else
//        {
//            now     += *buffer;
//            *buffer  = 0;
//        }
//    }

//    return now;
//}

///**
//  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
//  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
//  * @retval ��ǰ���
//  * @attention
//  */
//float RAMP_float( float final, float now, float ramp )
//{
//    float buffer = 0;


//    buffer = final - now;

//    if (buffer > 0)
//    {
//        if (buffer > ramp)
//        {
//            now += ramp;
//        }
//        else
//        {
//            now += buffer;
//        }
//    }
//    else
//    {
//        if (buffer < -ramp)
//        {
//            now += -ramp;
//        }
//        else
//        {
//            now += buffer;
//        }
//    }

//    return now;
//}
///**
//  * @brief  �ǶȻػ� ����
//  * @param  void
//  * @retval �Ƕ�ֵ�����Ƕ�ֵ
//  * @attention
//  */
//void AngleLoop_f (float *angle , float max)
//{
//    if(*angle < -(max / 2))
//    {
//        *angle += max;
//    }
//    else if(*angle > (max / 2))
//    {
//        *angle -= max;
//    }
//}
///**
//  * @brief  �ǶȻػ�
//  * @param  void
//  * @retval �Ƕ�ֵ�����Ƕ�ֵ
//  * @attention
//  */
//void AngleLoop (float *angle , float max)
//{
//    if(*angle < -(max / 2))
//    {
//        *angle += max;
//    }
//    else if(*angle > (max / 2))
//    {
//        *angle -= max;
//    }
//}

///**
//  * @brief  float���;���ֵ
//  * @param  void
//  * @retval
//  * @attention
//  */
//float abs_float(float a){
//	if(a<0)
//		return -a;
//	else
//		return a;
//}

//float queue_sum(int n,float*yaw_smooth,float*yaw_smooth1,float item1)
//{
//	//��ֵ�˲�
//	float cs_yaw_tot=0;
////	int cs_yaw_after=0;
//	static int smoothf=0;       	//��ֵ�˲���λ��־��
//  /********�ǶȻػ�����*********/
//  if(item1<80 && yaw_smooth1[(smoothf-3+n)%n]>280 )
//  {
//    yaw_smooth[smoothf]=360+item1-yaw_smooth1[(smoothf-3+n)%n];
//  }
//  else if(item1>280 && yaw_smooth1[(smoothf-3+n)%n]<80)
//  {
//    yaw_smooth[smoothf]=item1-yaw_smooth1[(smoothf-3+n)%n]-360;
//  }
//  else
//  {
//    yaw_smooth[smoothf]=item1-yaw_smooth1[(smoothf-3+n)%n];
//  }
//  yaw_smooth1[smoothf]=item1;
//  if(smoothf==n-1)
//  {
//    smoothf=0;
//  }
//  else
//  {
//    smoothf++;
//  }

//  for(int j=0;j<n;j++)
//	{
//		cs_yaw_tot+=yaw_smooth[j];
//	}     
//	return cs_yaw_tot;
//}
