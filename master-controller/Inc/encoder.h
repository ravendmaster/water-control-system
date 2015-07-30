#include "stm32f1xx_hal.h"

typedef struct
{
	int8_t disp;
	GPIO_PinState prev_a_state;
	GPIO_PinState button_state;
} Encoder;

void UserActivity(void);

void Encoder_Init(Encoder * encoder);

void Encoder_SetState(Encoder * encoder, GPIO_PinState encoder_a, GPIO_PinState encoder_b, GPIO_PinState encoder_button);

int8_t Encoder_GetDisp(Encoder * encoder);
 
GPIO_PinState Encoder_GetButtonState(Encoder * encoder);
