#include "car.h"



float LEG_SPEED_PID[6] = {6000.0f,0.0f,2.0f,0.0f,2000.0f,6000.0f};
Car car(can_receive.get_dji_motor_measure_point(0),can_receive.get_dji_motor_measure_point(1),0,8192,LEG_SPEED_PID);


Car::Car(const dji_motor_measure_t* left_motor_ptr,
        const dji_motor_measure_t* right_motor_ptr,
        uint16_t offset_ecd,
        uint16_t max_ecd,
        const fp32* speed_parm,
        const fp32* encode_parm, 
        const fp32* gyro_parm):
left_leg(left_motor_ptr,offset_ecd,max_ecd,speed_parm,encode_parm,gyro_parm),
right_leg(right_motor_ptr,offset_ecd,max_ecd,speed_parm,encode_parm,gyro_parm),
car_mode(0)
{

}

void Car::feedback_update()
{
    left_leg.update();
    right_leg.update();
}

void Car::set_control()
{

    left_leg.set(3,SPEED);
    right_leg.set(3,SPEED);

}

void Car::solve()
{
    left_leg.solve(SPEED);
    right_leg.solve(SPEED);
}

void Car::output()
{
    if (car_mode == CAR_STOP)
    {
        left_leg.current_give=0;
        right_leg.current_give=0;
    }
    // can_receive.can_cmd_dji_motor(left_leg.current_give, right_leg.current_give,
    //                             0, 0,CAN_CHASSIS_MOTIVE_ALL_ID);
    can_receive.can_cmd_leg_motor(800, 100,CAN_LEGS_ALL_ID);
}
