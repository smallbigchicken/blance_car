#ifndef PID_H
#define PID_H

#include <cmath>

typedef float fp32;

enum PidMode {
    PID_POSITION = 0, // 位置/速度模式 (直立环、速度环用这个)
    PID_ANGLE    = 1  // 角度模式 (只有转向环可能用到，处理 -PI 到 PI)
};

struct PidParam {
    fp32 kp;
    fp32 ki;
    fp32 kd;
    fp32 max_iout;   // 积分上限
    fp32 max_out;    // 总输出上限
};

class Pid {
public:
    Pid();
    
    // 构造函数
    Pid(PidMode mode, const PidParam &param);

    // 核心计算
    fp32 Calc(fp32 current, fp32 target);

    // 清除状态 (倒地扶起后需调用)
    void Reset();

private:
    PidMode mode;
    PidParam param;

    fp32 error;
    fp32 last_error;
    fp32 i_out; 

    // 简单的角度归一化 (-PI ~ PI)
    fp32 AngleFormat(fp32 angle);

    // 限幅函数
    static void Limit(fp32 &val, fp32 max) {
        if (val > max) val = max;
        else if (val < -max) val = -max;
    }
};

#endif