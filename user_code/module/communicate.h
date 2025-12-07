#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"
#include "can_receive.h"
#include "imu.h"


class Communicate
{
public:
    void init();

    void run();

    
};



extern Imu imu;
extern Communicate communicate;
extern Can_receive can_receive;

#endif

