#include "imu.h"

Imu::Imu(const dm_imu_measure_t* imu_ptr):
imu_measure(imu_ptr)
{}
    

    



void Imu::update()
{
    euler[0]=imu_measure->pitch*2*PI/360;
	euler[1]=imu_measure->yaw*2*PI/360;
	euler[2]=imu_measure->roll*2*PI/360;

    gyro[0]=imu_measure->x_gyro;
	gyro[1]=imu_measure->y_gyro;
	gyro[2]=imu_measure->z_gyro;

}




