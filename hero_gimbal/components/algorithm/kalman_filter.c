#include "kalman_filter.h"
//float matrix_value1;
//float matrix_value2;

//void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
//{
//  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
//  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
//  mat_init(&F->z,2,1,(float *)I->z_data);
//  mat_init(&F->A,2,2,(float *)I->A_data);
//  mat_init(&F->H,2,2,(float *)I->H_data);
//  mat_init(&F->Q,2,2,(float *)I->Q_data);
//  mat_init(&F->R,2,2,(float *)I->R_data);
//  mat_init(&F->P,2,2,(float *)I->P_data);
//  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
//  mat_init(&F->K,2,2,(float *)I->K_data);
//  mat_init(&F->AT,2,2,(float *)I->AT_data);
//  mat_trans(&F->A, &F->AT);
//  mat_init(&F->HT,2,2,(float *)I->HT_data);
//  mat_trans(&F->H, &F->HT);
////  matrix_value2 = F->A.pData[1];
//}
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
    mat_init(&F->xhat, 2, 1, (float *)I->xhat_data); //����ָ�� �� �� ��������ָ��
    mat_init(&F->xhatminus, 2, 1, (float *)I->xhatminus_data);
    mat_init(&F->z, 2, 1, (float *)I->z_data);
    mat_init(&F->A, 2, 2, (float *)I->A_data);
    mat_init(&F->B, 2, 1, (float *)I->B_data);
    mat_init(&F->H, 2, 2, (float *)I->H_data);
    mat_init(&F->Q, 2, 2, (float *)I->Q_data);
    mat_init(&F->R, 2, 2, (float *)I->R_data);
    mat_init(&F->P, 2, 2, (float *)I->P_data);
    mat_init(&F->Pminus, 2, 2, (float *)I->Pminus_data);
    mat_init(&F->K, 2, 2, (float *)I->K_data);
    mat_init(&F->KT, 2, 2, (float *)I->KT_data);

    mat_init(&F->AT, 2, 2, (float *)I->AT_data);
    mat_trans(&F->A, &F->AT);
    mat_init(&F->HT, 2, 2, (float *)I->HT_data);
    mat_trans(&F->H, &F->HT);
}


// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param �����������ṹ��
  *@param �Ƕ�
  *@param �ٶ�
*/
//float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
//{
//  float TEMP_data[4] = {0, 0, 0, 0};
//  float TEMP_data21[2] = {0, 0};
//  mat TEMP,TEMP21;

//  mat TEMP_Input;

//  mat_init(&TEMP,2,2,(float *)TEMP_data);//
//  mat_init(&TEMP21,2,1,(float *)TEMP_data21);//
//  mat_init(&TEMP_Input,2,1,(float *)TEMP_data21);//
//  TEMP_Input.pData[0] = 0;
//  TEMP_Input.pData[1] = signal2;

//  F->z.pData[0] = signal1;//z(k)
//  F->z.pData[1] = signal2;//z(k)

//  //1. xhat'(k)= A xhat(k-1)
//  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

//  mat_add(&F->xhatminus,&TEMP_Input,&F->xhatminus);
//
//  //2. P'(k) = A P(k-1) AT + Q
//  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
//  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
//  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

//  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
//  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
//  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
//  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

//  mat_inv(&F->K, &F->P);//
//  mat_mult(&F->Pminus, &F->HT, &TEMP);//
//  mat_mult(&TEMP, &F->P, &F->K);//

//  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
//  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
//  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
//  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
//  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

//  //5. P(k) = (1-K(k)H)P'(k)
//  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
//  mat_sub(&F->Q, &F->P, &TEMP);//
//  mat_mult(&TEMP, &F->Pminus, &F->P);

//  matrix_value1 = F->xhat.pData[0];
//  matrix_value2 = F->xhat.pData[1];

//  F->filtered_value[0] = F->xhat.pData[0];
//  F->filtered_value[1] = F->xhat.pData[1];
//  return F->filtered_value;
//}
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2, float signal3)
{
    float TEMP_data[4] = {0, 0, 0, 0};
    float TEMP_data21[2] = {0, 0};
    float TEMP_data22[4] = {0, 0, 0, 0};
    float I_data[4] = {1, 0, 0, 1};

    mat TEMP, TEMP21, TEMP22, I;

    float CONTROL;

    mat_init(&I, 2, 2, (float *)I_data);
    mat_init(&TEMP, 2, 2, (float *)TEMP_data);
    mat_init(&TEMP21, 2, 1, (float *)TEMP_data21);
    mat_init(&TEMP22, 2, 2, (float *)TEMP_data22);

    CONTROL = signal3; //���ٶ�

    F->z.pData[0] = signal1;//z(k) ����ֵ λ��
    F->z.pData[1] = signal2;//z(k) �ٶ�


    /////******ʱ�����*******/////
    //1. xhat'(k)= A xhat(k-1) ״̬Ԥ��
    mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    mat_scale(&F->B, CONTROL, &TEMP21);
    mat_add(&F->xhatminus, &TEMP21, &F->xhatminus);

    //2. P'(k) = A P(k-1) AT + Q Э����Ԥ��
    mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q


    /////******�۲����*******/////
    //3. K(k) = P'(k) HT / (H P'(k) HT + R) ����������
    mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

    mat_inv(&F->K, &F->P);                 //��������
    mat_mult(&F->Pminus, &F->HT, &TEMP);
    mat_mult(&TEMP, &F->P, &F->K);

    //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))  ״̬����
    mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

    //5. P(k) = (1-K(k)H)P'(k)  Э�������
    /*��ʽ��1����ʽ�򵥣�������С�������ۻ��������ʹЭ�������ʧȥ�Ǹ������������Գ��ԣ�
    ����ʵ���г�ʹ�ù�ʽ��2��*/

    //  mat_mult(&F->K, &F->H, &F->P);//            (1)p(k|k) = (I-kg(k)*H)*P(k|k-1)
    //  //mat_sub(&F->Q, &F->P, &TEMP);
    //  mat_sub(&I, &F->P, &TEMP);
    //  mat_mult(&TEMP, &F->Pminus, &F->P);

    mat_mult(&F->K, &F->H, &F->P);               //(2)p(k|k) = (I-kg(k)*H)*P(k|k-1)*(I-kg(k)*H) + kg(k)*R*kgT(k)
    mat_sub(&I, &F->P, &TEMP);
    mat_mult(&TEMP, &F->Pminus, &F->P);
    mat_mult(&F->P, &TEMP, &F->Pminus);

    mat_mult(&F->K, &F->R, &TEMP);
    mat_trans(&F->K, &F->KT);
    mat_mult(&TEMP, &F->KT, &TEMP22);
    mat_add(&F->Pminus, &TEMP22, &F->P);

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];
    return F->filtered_value;
}

//float dt = 0.001f;
//float Q_angle = 1.0f;             //����������Э����
//float Q_gyro = 0.6f;              //Ư������Э����
//float R_angle =0.05f;                 //���ٶȲ�������Э����
//float Angle = 0;
//float Gyro = 0;
//float Q_bias = 0;
//float K_0, K_1;
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };
//float IMU_value[2];
//float *Kalman_Filter_IMU(float newAngle,float newGyro)
//{
//

//  Angle += (newGyro - Q_bias) * dt; //�������
//
//  PP[0][0] += Q_angle - PP[0][1] * dt - PP[1][0] * dt;
//  PP[0][1] -= PP[1][1] * dt;   //����������Э����
//  PP[1][0] -= PP[1][1] * dt;
//  PP[1][1] += Q_gyro;
//
//  K_0 = PP[0][0] / (R_angle + PP[0][0]);
//  K_1 = PP[1][0] / (R_angle + PP[0][0]);
//
//  Angle += K_0 * (newAngle - Angle);
//  Q_bias += K_1 * (newAngle - Angle); //�������
//  Gyro = newGyro - Q_bias;
//  //����������Э����
//  PP[0][0] -= K_0 * PP[0][0];
//  PP[0][1] -= K_0 * PP[0][1];
//  PP[1][0] -= K_1 * PP[0][0];
//  PP[1][1] -= K_1 * PP[0][1];
//
//  IMU_value[0] = Angle;
//  IMU_value[1] = Gyro;
//  return  IMU_value;
//}
#define dt              0.001f//�������˲��������� s
float R_angle   =       0.5f; //����������Э������ǲ���ƫ�
float Q_angle   =       0.0001f;//����������Э����
float Q_gyro    =       0.0003f; //����������Э����  ��������Э����Ϊһ��һ�����о���
static float angle = 0;            //��ʱ�����Ź���ֵ�Ƕ�
float q_bias = 0;        //�����ǵ�ƫ��
float P[2][2] = {{ 1, 0 }, { 0, 1 }};//����Э�������
float Kalman_Filter_IMU(const float newGyro, const float newAngle)
{

    float K_0;//���п��������������һ�����������ڼ������Ź���ֵ
    float K_1;//���п���������ĺ��������ڼ������Ź���ֵ��ƫ��

    float Y_0, Y_1;

    float Rate;//ȥ��ƫ���Ľ��ٶ�

    float Pdot[4];//����Э��������΢�־���

    float angle_err;//�Ƕ�ƫ��

    float E;//����Ĺ�����

    Rate = newGyro - q_bias;

    //�������Э��������΢�־���
    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = - P[1][1];
    Pdot[2] = - P[1][1];
    Pdot[3] = Q_gyro;

    angle += Rate * dt; //���ٶȻ��ֵó��Ƕ�

    P[0][0] += Pdot[0] * dt; //����Э�������
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    angle_err = newAngle - angle; //����Ƕ�ƫ��

    E = R_angle + P[0][0];
    K_0 = P[0][0] / E; //���㿨��������
    K_1 = P[1][0] / E;

    Y_0 = P[0][0];
    Y_1 = P[0][1];

    P[0][0] -= K_0 * Y_0; //����Э�������
    P[0][1] -= K_0 * Y_1;
    P[1][0] -= K_1 * Y_0;
    P[1][1] -= K_1 * Y_1;

    angle += K_0 * angle_err; //�������Ź���ֵ

    q_bias += K_1 * angle_err;//�������Ź���ֵƫ��

    return angle;
}

//
/**
  * @author  Liu heng
  * һ�׿������˲�������RoboMaster��̳
  *   һά�������˲���
  *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲���
  *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
  *          ʹ��ʾ��
  *          extKalman p;                  //����һ���������˲����ṹ��
  *          float SersorData;             //��Ҫ�����˲�������
  *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����Q=20 R=200����
  *          while(1)
  *          {
  *             SersorData = sersor();                     //��ȡ����
  *             SersorData = KalmanFilter(&p,SersorData);  //�����ݽ����˲�
  *          }
  */

/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *
  * @retval none
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *             ��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid = p->A * p->X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A * p->P_last + p->Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid / (p->P_mid + p->R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1 - p->kg) * p->P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;                //���Ԥ����x(k|k)
}
//