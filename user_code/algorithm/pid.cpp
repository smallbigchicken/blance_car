#include "Pid.h"

Pid::Pid(PidMode mode_, const PidParam &param_) 
    : mode(mode_), param(param_) 
{
    Reset();
}

void Pid::Reset()
{
    error = 0;
    last_error = 0;
    i_out = 0;
}

fp32 Pid::Calc(fp32 current, fp32 target)
{
    // 1. 计算基础误差
    error = target - current;

    // 2. 角度模式特殊处理 (只有转向环可能用到)
    if (mode == PID_ANGLE) {
        error = AngleFormat(error);
    }


    i_out += param.ki * error;
    Limit(i_out, param.max_iout);


    fp32 d_out = param.kd * (error - last_error);



    fp32 p_out = param.kp * error;

    
    fp32 total_out = p_out + i_out + d_out;
    Limit(total_out, param.max_out);
    param.out=total_out;
    // 8. 记录历史
    last_error = error;

    return total_out;
}

fp32 Pid::AngleFormat(fp32 angle) {
    const fp32 PI = 3.14159265f;
    while (angle > PI)  angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}