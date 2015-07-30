#include "stm32f1xx_hal.h"
#include "encoder.h"

void Encoder_Init(Encoder * encoder)
{
	encoder->disp=0;
	encoder->prev_a_state=GPIO_PIN_SET;
}

void Encoder_SetState(Encoder * encoder, GPIO_PinState encoder_a, GPIO_PinState encoder_b, GPIO_PinState encoder_button)
{
	encoder->button_state = encoder_button;
	
	GPIO_PinState encoder_a_state=encoder_a;
	GPIO_PinState encoder_b_state=encoder_b;
	
	if((encoder->prev_a_state != encoder_a_state) && (encoder_a_state==GPIO_PIN_RESET))
	{
		
		if(encoder_b_state==GPIO_PIN_SET)
		{
			encoder->disp+=1;
		}
		else
		{
			encoder->disp-=1;
		}
		
	}
	encoder->prev_a_state = encoder_a_state;
	
 }

 int8_t Encoder_GetDisp(Encoder * encoder)
 {
	 uint8_t res=encoder->disp;
	 encoder->disp=0;
	 return res;
 }
 
 GPIO_PinState Encoder_GetButtonState(Encoder * encoder)
 {
	 return encoder->button_state;
 }
