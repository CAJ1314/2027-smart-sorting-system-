#ifndef __SENSOR_H
#define __SENSOR_H
#include "stm32f4xx.h"

void Sensor_Init(void);
uint8_t Sensor_Get(uint8_t index);   // index: 1~6, 对应1~6号光电传感器

#endif
