#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"
#include "can_receive.h"
#include "imu.h"
#include "ws2812.h"

class Communicate
{
public:
    
    void init();

    void run();

    
};



extern Communicate communicate;


#endif

