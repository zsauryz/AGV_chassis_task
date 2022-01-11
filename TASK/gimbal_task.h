#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
#include "main.h"

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//任务绝对延迟间隔
#define GIMBAL_CONTROL_TIME_MS 2
void gimbal_task(void const *pvParameters);
#endif


