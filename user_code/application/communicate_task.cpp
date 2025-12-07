#include "communicate_task.h"
#include "communicate.h"



void communicate_Task(void const * pvParameters)
{
  vTaskDelay(COMMUNICATE_TASK_INIT_TIME);

  communicate.init();

  while (1)
  {
    communicate.run();
    vTaskDelay(COMMUNICATE_CONTROL_TIME_MS);
  }
}
