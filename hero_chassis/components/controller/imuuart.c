#include "imuuart.h"

imudata imu_data;
uint8_t imubuff[14];

void imu_send() {
    imu_data.imuyaw.value = IMU_angle[0];
    imu_data.imupitch.value = IMU_angle[1];
    imu_data.imuroll.value = IMU_angle[2];
    imubuff[0] = 'f';
    imubuff[1]=imu_data.imuyaw.buff[0];
    imubuff[2]=imu_data.imuyaw.buff[1];
    imubuff[3]=imu_data.imuyaw.buff[2];
    imubuff[4]=imu_data.imuyaw.buff[3];
    imubuff[5]=imu_data.imupitch.buff[0];
    imubuff[6]=imu_data.imupitch.buff[1];
    imubuff[7]=imu_data.imupitch.buff[2];
    imubuff[8]=imu_data.imupitch.buff[3];
    imubuff[9]=imu_data.imuroll.buff[0];
    imubuff[10]=imu_data.imuroll.buff[1];
    imubuff[11]=imu_data.imuroll.buff[2];
    imubuff[12]=imu_data.imuroll.buff[3];
    imubuff[13]='u';
}
