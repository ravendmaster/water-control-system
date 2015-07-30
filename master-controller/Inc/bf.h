#include "stm32f1xx_hal.h"

#ifndef __bf_H
#define __bf_H

typedef struct 
{
		uint32_t nextStateChangeTick;
		uint32_t minTimeInterval;
		GPIO_PinState curState;
		uint32_t counter;
} BF_Instance;

void BF_Init(BF_Instance * instance, uint32_t timeInterval);

void BF_SetState(BF_Instance * instance, GPIO_PinState state);

GPIO_PinState BF_GetState(BF_Instance * instance);

#endif
