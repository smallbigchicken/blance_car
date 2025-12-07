#ifndef MOTOR_H
#define MOTOR_H


#include "Pid.h"
#include "Can_receive.h"

#define SPEED 0
#define ENCODE_ANGLE 1
#define GYRO_ANGLE 2


#define DJI_RPM_TO_RAD 0.000415809748903494517209f //2*PI/60


class Motor{
public:

    //speed & speed_set rad/s
    float speed;
    float speed_set;

    //encode_angle & encode_angle_set  rad
    float encode_angle;
    float encode_angle_set;
    
    //gyro_angle & gyro_angle_set rad
    float gyro_angle;
    float gyro_angle_set;

    //speed_PID
    Pid speed_pid;
    //encode_angle_PID
    Pid encode_angle_pid;
    //gyro_angle_PID
    Pid gyro_angle_pid;
    
    

    float current_give;

    Motor(const fp32* speed_parm = NULL,
          const fp32* encode_parm = NULL, 
          const fp32* gyro_parm  = NULL 
          );
                                                                                                                    
    void update();
    void set(float set,uint8_t mode);
    void solve(uint8_t mode);

};

class DJI_Motor : public Motor
{
public:

    const dji_motor_measure_t *motor_measure;

    //the zero point ecd
    uint16_t offset_ecd;

    //the whole ecd
    uint16_t max_ecd;
    

    DJI_Motor();

    DJI_Motor(const dji_motor_measure_t* motor_ptr,
              uint16_t offset_ecd,
              uint16_t max_ecd,
              const fp32* speed_parm = NULL,
              const fp32* encode_parm = NULL, 
              const fp32* gyro_parm  = NULL
              );


    fp32 ecd_to_angle(uint16_t ecd);

    void update();

};







#endif

