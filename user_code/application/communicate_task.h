#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H


#include "cmsis_os.h"
#include "main.h"



#define COMMUNICATE_TASK_INIT_TIME 30


#define COMMUNICATE_CONTROL_TIME_MS 2

#ifdef __cplusplus
extern "C" {

extern void communicate_Task(void const * pvParameters);

}
#endif


#endif
