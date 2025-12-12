#ifndef IMU_H
#define IMU_H


#include "uart_receive.h"

class Imu
{
public:

    float euler[3];
    float gyro[3];

    const dm_imu_measure_t *imu_measure;
    Imu();
    Imu(const dm_imu_measure_t* imu_ptr);


    void update();


};

#endif
