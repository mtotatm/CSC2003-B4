//possible kalman filter for accelerometer
#include <stdint.h>
#include <stdbool.h>

int16_t rawAccX, rawAccY, rawAccZ,rawGyroX, rawGyroY, rawGyroZ;
float gyroXoffset, gyroYoffset, gyroZoffset;
float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleGyroX, angleGyroY, angleGyroZ,angleAccX, angleAccY, angleAccZ;
float angleX, angleY, angleZ;
uint32_t timer;

double P[2][2] = {{ 1, 0 },{ 0, 1 }};
double Pdot[4] ={ 0,0,0,0};
static const double Q_angle=0.001, Q_gyro=0.003, R_angle=0.5,dtt=0.005,C_0 = 1;
double q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

void Kalman_Filter(double angle_m,double gyro_m)
{
    angleX+=(gyro_m-q_bias) * dtt;
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]=- P[1][1];
    Pdot[2]=- P[1][1];
    Pdot[3]=Q_gyro;
    P[0][0] += Pdot[0] * dtt;
    P[0][1] += Pdot[1] * dtt;
    P[1][0] += Pdot[2] * dtt;
    P[1][1] += Pdot[3] * dtt;
    angle_err = angle_m - angleX;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angleX+= K_0 * angle_err;
    q_bias += K_1 * angle_err;
}
