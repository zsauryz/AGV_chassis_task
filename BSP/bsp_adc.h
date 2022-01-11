#ifndef __BSP_ADC_H
#define __BSP_ADC_H
#include "main.h"

#define MUC_TEMPERATE ADC_GetMcuTemperate()

uint16_t ADC_Get(uint32_t ch);

float ADC_GetMcuTemperate(void);

#endif

