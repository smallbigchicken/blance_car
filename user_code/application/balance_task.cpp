#include "balance_task.h"

void balance_Task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    while (1)
    {

      car.feedback_update();

      car.output();
      
      vTaskDelay(GIMBAL_CONTROL_TIME_MS);
    }

}
