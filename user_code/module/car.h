#include "motor.h"
#include "can_receive.h"



typedef enum
{
    CAR_STOP = 0,//无力
    CAR_RUN        //归中
} car_mode_e;




class Car {
public:


    DJI_Motor left_leg;
    DJI_Motor right_leg;
    uint16_t car_mode;
    Car(const dji_motor_measure_t* left_motor_ptr,
        const dji_motor_measure_t* right_motor_ptr,
        uint16_t offset_ecd,
        uint16_t max_ecd,
        const fp32* speed_parm = NULL,
        const fp32* encode_parm = NULL, 
        const fp32* gyro_parm  = NULL);

    void feedback_update();
    void set_control();
    void solve();  // 核心计算
    void output(); // 发送电流
};

extern Car car;
