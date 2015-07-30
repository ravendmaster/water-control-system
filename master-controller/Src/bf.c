#include "stm32f1xx_hal.h"
#include "bf.h"

void BF_Init(BF_Instance * instance, uint32_t timeInterval)
{
	instance->nextStateChangeTick=HAL_GetTick()+timeInterval;
	instance->minTimeInterval=timeInterval;
	instance->counter=0;
	instance->curState=GPIO_PIN_SET;
}

void BF_SetState(BF_Instance * instance, GPIO_PinState state)
{
	uint32_t curTick=HAL_GetTick();
	if(curTick<instance->nextStateChangeTick)return;
	if(instance->curState==state)return;
	if(state==GPIO_PIN_SET)instance->counter++;
	
	instance->curState=state;
	instance->nextStateChangeTick=curTick+instance->minTimeInterval;
}

GPIO_PinState BF_GetState(BF_Instance * instance)
{
	return instance->curState;
}
