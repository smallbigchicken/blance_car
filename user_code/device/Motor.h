#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"       
#include "Can_receive.h" 


class Motor {
public:
   
    float speed_rpm;       
    float speed_rads;      
    
    float angle_single_round; 
    float total_angle;        

    
    int16_t current_give;  

   
    Motor();
    void update();
};

class DJI_Motor : public Motor
{
public:
    
    const dji_motor_measure_t *motor_measure;

    
    uint16_t offset_ecd;  
    uint16_t max_ecd;      

   
    int32_t round_count;  
    uint16_t last_ecd;    

    
    DJI_Motor(); 
    DJI_Motor(const dji_motor_measure_t* measure_ptr,
              uint16_t offset_ecd = 0,
              uint16_t max_ecd = 8191);

        
    void update();
};

#endif