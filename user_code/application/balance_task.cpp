#include "balance_task.h"

void balance_task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    car.init();

    while (1)
    {
      //设置云台状态机
      car.set_mode();
      //云台数据反馈
      car.feedback_update();
      //设置云台控制量
      car.set_control();
      //设置PID计算
      car.solve();
      //输出电流
      car.output();
      //系统延时
      vTaskDelay(GIMBAL_CONTROL_TIME_MS);
    }

}
