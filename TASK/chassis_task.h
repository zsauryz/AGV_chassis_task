#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "main.h"

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//����������Ƽ�� 0.002s

void chassis_task(void const *pvParameters);

extern uint16_t RC_FLAG;
#endif


