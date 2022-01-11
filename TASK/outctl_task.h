/**
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             µ×ÅÌ¿ØÖÆÈÎÎñ
  * @note       
  * @history
  *  Version    Date            Author          Modification

  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 DIODE****************************
  */
#ifndef __OUTCTL_TASK_H
#define __OUTCTL_TASK_H
#include "main.h"

#define OUTCTL_TASK_INIT_TIME   500
#define OUTCTL_TASK_TIME_MS     100

void outctl_task(void const *pvParameters);

#endif


