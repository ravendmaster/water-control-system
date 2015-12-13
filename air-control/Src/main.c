/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 12/12/2015 22:20:39
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "nrf24.h"
#include "aes.h"
#include <stdio.h>
#include <stdlib.h>
#include "encrypted_block.h"
#include "nrf24_address_cfg.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void deinit()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
												|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                        |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                        |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

aes128_ctx_t AES_ctx;

uint8_t phrase[16];

uint32_t isAccidient()
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==GPIO_PIN_SET)
	{
		return 1;
	}
	
	return 0;
}

uint8_t LightIsOn()
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET)
	{
		return 0;
	}
	return 1;
}

uint32_t getPostKey()
{
	return BKP->DR1 | BKP->DR2<<16;
}

void setPostKey(uint32_t key)
{
		BKP->DR1=key&0xffff;
		BKP->DR2=key>>16;	
}

void xprintf(void * ptr, ...)
{
}

typedef struct QMTT_Message
{
	char * topic;
	char * value;
} QMTT_Message;

QMTT_Message QMTT_Get(EncryptedBlock * encryptedBlock)
{
	int topic_len=strlen((char*)encryptedBlock->data);
	char * value=(char*)encryptedBlock->data+topic_len+1;

	QMTT_Message message;
	message.topic=(char*)encryptedBlock->data;
	message.value=value;
	
	return message;
}

const static uint8_t deciceId=1;
static uint8_t UD1_address[5] =  {deciceId,0xF7,0xF7,0xF7,0xF7};
static uint32_t switches_states;
static uint32_t switches_count=4;

static uint16_t dimmer_values[1] = {0};
static uint32_t dimmers_count=1;

static uint8_t sun_mode=0; //0 - off, 1 - sunrise, 2 - sunset


static char in_topic[]="/myhome/in";
static char out_topic[]="/myhome/out";

typedef struct
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} SwitchPinAdress;

static SwitchPinAdress switchesMap[32]=
{
	{GPIOB,GPIO_PIN_1},
	{GPIOA,GPIO_PIN_3},
	{GPIOA,GPIO_PIN_4},
	{GPIOA,GPIO_PIN_5}
	
};

void actualizeHardwareStatus()
{
	for(int i=0;i<switches_count;i++)
	{
		GPIO_PinState pinstate;
		if(1&(switches_states>>i)){
			pinstate=GPIO_PIN_SET;
		}else{
			pinstate=GPIO_PIN_RESET;
		};
				
		HAL_GPIO_WritePin(switchesMap[i].GPIOx, switchesMap[i].GPIO_Pin, pinstate);
	}
	
	for(int i=0;i<dimmers_count;i++)
	{
		TIM3->CCR1=dimmer_values[i];
	}
		
}

void sendSwitchStateToQMTTServer(uint32_t switch_no)
{
		char topic[32];
		char message_on[]="ON";
		char message_off[]="OFF";
		char * message;
		
		sprintf(topic, "UD%d_%d", deciceId, switch_no);
		if((switches_states>>switch_no)&1){
			message=message_on;
		}else{
			message=message_off;
		};
		
		QMTT_SendTextMessage(topic, message, UD1_address, &AES_ctx);
}

void sendDimerStateToQMTTServer(uint32_t item_no)
{
		char topic[32];
		char message[32];
		sprintf(topic, "DM%d_%d", deciceId, item_no);
		sprintf(message, "%d", dimmer_values[item_no]);
		QMTT_SendTextMessage(topic, message, UD1_address, &AES_ctx);
}


void sendFullStatesToQMTTServer()
{
	QMTT_SendOutTopic(out_topic, UD1_address, &AES_ctx);
	QMTT_SendInTopic(in_topic, UD1_address, &AES_ctx);
	
	for(int i=0;i<switches_count;i++)
	{
		sendSwitchStateToQMTTServer(i);
	}
	
	for(int i=0;i<dimmers_count;i++)
	{
		sendDimerStateToQMTTServer(i);
	}
	
}

void interpretateQMTTMessage(char * topic, char * value)
{
	char temp[32];
	
	for(int i=0;i<switches_count;i++)
	{
		sprintf(temp, "UD%d_%d", deciceId, i);
		if(strcmp(topic, temp)==0)
		{
			if(strcmp(value, "ON")==0)
			{
				switches_states|=(1<<i);
			}
			else if(strcmp(value, "OFF")==0)
			{
				switches_states&=~(1<<i);
			}
		}
	}
	
		
	for(int i=0;i<dimmers_count;i++)
	{
		sprintf(temp, "DM%d_%d", deciceId, i);
		
		if(strcmp(topic, temp)==0)
		{		
			uint16_t val=atoi(value);
			dimmer_values[i]=65535*val/100;
		}
	}

	char temp2[32];
	sprintf(temp2, "SN%d_0", deciceId);

	if(strcmp(topic, temp2)==0)
	{
		if(strcmp(value, "ON")==0)
		{
			sun_mode=1;
		}
		else if(strcmp(value, "OFF")==0)
		{
			sun_mode=2;
		}

	}
	
	
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

	aes128_init(AES_key, &AES_ctx);
	
	nrf24_init(&hspi2);
	nrf24_config(99,32);

	nrf24_rx_address(RX_ADDR_P1, UD1_address);
	nrf24_tx_address(openhab_gate_address_pipe1);
	
	uint32_t last_t=0xffff;
	//sendFullStatesToQMTTServer();
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
	nrf24_powerUpRx();
	uint32_t last_op=HAL_GetTick();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t t=HAL_GetTick();

		if(t-last_op>30) //1 - 65 sec
		{
			last_op=t;
			
			if(sun_mode==1)
			{
				if(dimmer_values[0]==0xffff){
					sun_mode=0;
				}
				else{
					dimmer_values[0]+=1;
					TIM3->CCR1=dimmer_values[0];
				}
			}
			if(sun_mode==2)
			{
				if(dimmer_values[0]==0){
					sun_mode=0;
				}else{
					dimmer_values[0]-=1;
					TIM3->CCR1=dimmer_values[0];
				}
			}
		}

		
		//uint32_t t=HAL_GetTick();
		if(t-last_t>60000*10)
		{
			last_t=t;
			sendFullStatesToQMTTServer();
			nrf24_powerUpRx();
		}
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)==GPIO_PIN_SET) //onboard button pressed
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_SET)
			{
				QMTT_SendTextMessage("UD1_0", "OFF", UD1_address, &AES_ctx);
				interpretateQMTTMessage("UD1_0", "OFF");
			}
			else
			{
				QMTT_SendTextMessage("UD1_0", "ON", UD1_address, &AES_ctx);
				interpretateQMTTMessage("UD1_0", "ON");
			}
			actualizeHardwareStatus();

			while((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)==GPIO_PIN_SET)){};
		}
			
		
		if(nrf24_dataReady())
    {	
			EncryptedBlock encryptedBlock;
			uint8_t pipeNo=nrf24_dataReadyPipeNo();
			nrf24_getData((uint8_t*)&encryptedBlock);
			aes128_dec((uint8_t*)&encryptedBlock, &AES_ctx);
			aes128_dec((uint8_t*)&encryptedBlock+16, &AES_ctx);
			
			QMTT_Message qmtt_message=QMTT_Get(&encryptedBlock);

			interpretateQMTTMessage(qmtt_message.topic, qmtt_message.value);
			
			actualizeHardwareStatus();
		}
			
			
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  hrtc.DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  hrtc.DateToUpdate.Month = RTC_MONTH_JANUARY;
  hrtc.DateToUpdate.Date = 1;
  hrtc.DateToUpdate.Year = 0;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, FORMAT_BCD);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  HAL_SPI_Init(&hspi2);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA4 PA5 
                           PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA7 PA10 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12 
                           PB3 PB4 PB5 PB6 
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
