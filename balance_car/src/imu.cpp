#include "imu.h"

Imu::Imu(const dm_imu_measure_t* imu_ptr):
imu_measure(imu_ptr)
{}
    

    

void Imu::update()
{
    euler[0]=imu_measure->pitch;
	euler[1]=imu_measure->yaw;
	euler[2]=imu_measure->roll;

    gyro[0]=imu_measure->gyro_x;
	gyro[1]=imu_measure->gyro_y;
	gyro[2]=imu_measure->gyro_z;

}




