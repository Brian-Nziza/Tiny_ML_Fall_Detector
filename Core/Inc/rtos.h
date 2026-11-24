#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

// call after osKernelInitialize(), before osKernelStart()
void     Rtos_Init(void);
void     Rtos_OnFall(uint32_t t, float score);
uint32_t Rtos_Dropped(void);

#endif
