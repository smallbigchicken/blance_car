#ifndef IMU_H
#define IMU_H

#include <cstdint>
#include "main.h"
#include "fdcan.h"
#include "can_receive.h"




class Imu
{
public:

    fp32 euler[3];
    fp32 gyro[3];

    const dm_imu_measure_t *imu_measure;
    Imu();
    Imu(const dm_imu_measure_t* imu_ptr);


    void update();


};

#endif

