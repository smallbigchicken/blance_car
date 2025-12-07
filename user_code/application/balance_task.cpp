#include "balance_task.h"
#include "Car.h"
#include "cmsis_os.h"  // 包含 FreeRTOS/CMSIS 定义
#include "main.h"      // 包含 LED_GPIO_Port 等定义



void balance_Task(void *pvParameters)
{
   
    vTaskDelay(500); 

   
    car.init(&motor_left, &motor_right, &imu_data);
    car.pid_init(PID_UPRIGHT, PID_SPEED, PID_TURN);


    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(GIMBAL_CONTROL_TIME_MS);

    // 4. 进入死循环控制
    while (1)
    {
 
        car.feedback_update();


        car.set_control();

     
        car.solve();


        car.output();


        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}