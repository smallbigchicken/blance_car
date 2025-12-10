#include "balance_task.h"
#include "Car.h"
#include "cmsis_os.h"  // 包含 FreeRTOS/CMSIS 定义
#include "main.h"      // 包含 LED_GPIO_Port 等定义



void balance_Task(void *pvParameters)
{
   
    vTaskDelay(GIMBAL_TASK_INIT_TIME);



    while (1)
    {
 
        car.feedback_update();


        car.set_control();

     
        car.solve();


        car.output();


        vTaskDelay(GIMBAL_CONTROL_TIME_MS);
    }
}