/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 28/07/2015 20:51:52
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
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "LCD.h"
#include "bf.h"
#include "menu.h"
#include <stdio.h>
#include <stdlib.h>
#include "AT24C.h"
#include "nrf24.h"
#include "aes.h"
#include "encrypted_block.h"
#include "nrf24_address_cfg.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

osThreadId LCDTaskHandle;
osThreadId UserManagementHandle;
osThreadId I2CTimeTaskHandle;
osThreadId NRFf24TaskHandle;
osSemaphoreId SPI1BinarySemHandle;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartLCDTask(void const * argument);
void StartUserManagementTask(void const * argument);
void StartI2CTimeTask(void const * argument);
void StartNrf24Task(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t nrf_data_buff[32];

aes128_ctx_t AES_ctx;

Encoder encoder1;

BF_Instance count_cw;
BF_Instance count_hw;

uint8_t time_data_buff[19];
uint8_t time_data_buff_need_save=0;

bool counter_settings_data_buff_inizialized=false;
uint8_t counter_settings_data_buff[64];
bool counter_settings_data_buff_need_save=false;

bool userAlreadyNotified=false;

uint32_t sensor_resistance_value=0;


uint8_t HexToDec(uint8_t hex)
{
	return (hex&0xf) + (hex>>4&0xf)*10;
}

uint8_t DecToHex(uint8_t dec)
{
	int val=((dec/10)<<4) | (dec%10);
	return val;
}


//начало память от батарейки
typedef enum
{
	ACCIDENT_MODE=0,
	REGULAR_MODE
} AccidentStatus;

typedef struct
{
	uint8_t Sec;
	uint8_t Min;
	uint8_t Hour;
}Time;

typedef struct
{
	uint8_t Day;
	uint8_t Mont;
	uint8_t Year;
}Date;

typedef struct
{
	Time Time;
	uint8_t WeekDay;
	Date Date;
} DateTime;

uint32_t radio_sensor_resistance_value=0xffff;
//DateTime radio_sensor_dateTime;

typedef enum ValveStatus
{
	VALVE_STATUS_CLOSED=0,
	VALVE_STATUS_OPENED
} ValveStatus;

typedef struct
{
	uint32_t ccw; //0-3
	uint32_t chw; //4-7
	AccidentStatus accident_status; //8
	uint16_t accident_level_value; //9,10
	DateTime valveCloseDate; //11,12,13, 14-день недели, 15,16,17
	DateTime valveOpenDate; //18 - 24
	ValveStatus valveStatus; //25
	uint16_t radio_onair_interval; //26,27
	uint32_t key; //28-31
	DateTime radio_sensor_dateTime; //32 - 38
} SettingsAndStatus;

SettingsAndStatus * getSettingsStruct()
{
		return (SettingsAndStatus*)counter_settings_data_buff;
}

enum Event
{
	Event_Nop=0,
	Event_FullDate,
	Event_DiffDate,
	Event_Start,
	Event_ValveOpen,
	Event_ValveClose,
	Event_AccidentButtonPush,
	Event_MainSwitchOn,
	Event_MainSwitchOff,
	Event_AccidentSensorActivated,
	Event_AccidentSensorDisactivated,
};

void saveCCW(uint32_t data)
{
	getSettingsStruct()->ccw=data;
	counter_settings_data_buff_need_save=true;
}

void saveCHW(uint32_t data)
{
	getSettingsStruct()->chw=data;
	counter_settings_data_buff_need_save=true;
}

uint32_t loadCCW()
{
	return getSettingsStruct()->ccw;
}

uint32_t loadCHW()
{
	return getSettingsStruct()->chw;
}

DateTime * getCurrentDateTime()
{
	return (DateTime *)time_data_buff;
}

void saveAccidentStatus(AccidentStatus mode)
{
	getSettingsStruct()->accident_status=mode;
	counter_settings_data_buff_need_save=true;
}

AccidentStatus loadAccidentStatus()
{
	return getSettingsStruct()->accident_status;
}

void printDataTime(char * buff, DateTime * dateTime)
{
	sprintf(buff, "%02d/%02d/%02d %02d:%02d:%02d", HexToDec(dateTime->Date.Day), HexToDec(dateTime->Date.Mont), HexToDec(dateTime->Date.Year), HexToDec(dateTime->Time.Hour), HexToDec(dateTime->Time.Min), HexToDec(dateTime->Time.Sec));
}

Widget * getWidgetAccidentLevel()
{
	Widget * tempWidget=menu_getWidget();
	sprintf(tempWidget->menuViewTempBuff, "%d", getSettingsStruct()->accident_level_value);
	return tempWidget;
}

Widget * getWidgetSensorResistance()
{
	Widget * tempWidget=menu_getWidget();
	sprintf(tempWidget->menuViewTempBuff, "%d", sensor_resistance_value);
	return tempWidget;
}

Widget * getWidgetRadioSensorResistance()
{
	Widget * tempWidget=menu_getWidget();
	if(radio_sensor_resistance_value==0xffff)
	{
		sprintf(tempWidget->menuViewTempBuff, "n/a");
	}
	else
	{
		sprintf(tempWidget->menuViewTempBuff, "%d", radio_sensor_resistance_value);
	}
	return tempWidget;
}

Widget * getWidgetRadioOnAirInterval()
{
	Widget * tempWidget=menu_getWidget();
	sprintf(tempWidget->menuViewTempBuff, "%d", getSettingsStruct()->radio_onair_interval);
	return tempWidget;
}

Widget * getWidgetRadioProbeOnAirDateTime()
{
	Widget * tempWidget=menu_getWidget();
	printDataTime(tempWidget->menuViewTempBuff, &getSettingsStruct()->radio_sensor_dateTime);
	return tempWidget;
}


Widget * getWidgetCurrentDateTime()
{
	Widget * tempWidget=menu_getWidget();
	printDataTime(tempWidget->menuViewTempBuff, getCurrentDateTime());
	tempWidget->backgroundColor=LCD_RGB(0.5, 0.5, 0.5);
	tempWidget->foregroundColor=LCD_WHITE;
	return tempWidget;
}

Widget * getWidgetCounters()
{
	Widget * tempWidget=menu_getWidget();
	sprintf(tempWidget->menuViewTempBuff, "ХВС: %u ГВС: %u", count_cw.counter/1000, count_hw.counter/1000);
	tempWidget->backgroundColor=LCD_RGB(0.6, 0.6, 0.6);
	tempWidget->foregroundColor=LCD_WHITE;
	return tempWidget;
}

Widget * getWidgetValveLastOpen()
{
	Widget * tempWidget=menu_getWidget();
	DateTime * dateTime=&getSettingsStruct()->valveOpenDate;
	printDataTime(tempWidget->menuViewTempBuff, dateTime);
	return tempWidget;
}

Widget * getWidgetValveLastClose()
{
	Widget * tempWidget=menu_getWidget();
	DateTime * dateTime=&getSettingsStruct()->valveCloseDate;
	printDataTime(tempWidget->menuViewTempBuff, dateTime);
	return tempWidget;
}

Widget * getWidgetWorkMode()
{
	Widget * tempWidget=menu_getWidget();
	
	if(getSettingsStruct()->accident_status==ACCIDENT_MODE)
	{
		sprintf(tempWidget->menuViewTempBuff, "%s", "АВАРИЯ!!!");
		if(HAL_GetTick()&1024)
		{
			tempWidget->backgroundColor=LCD_RED;
			tempWidget->foregroundColor=LCD_WHITE;
		}
	}
	else
	{
		sprintf(tempWidget->menuViewTempBuff, "%s", "Штатный");
	}
	return tempWidget;
}


Size OnDrawWidgetNRFData(uint16_t x, uint16_t y, bool selected)
{
	Size size;
	size.width=320;
	size.height=200;
	nrf_data_buff[31]=0;
	LCD_DrawString(0,30, (char*)nrf_data_buff, false);
	
//	uint16_t pos=(HAL_GetTick()/10)&255;
////	
//	LCD_setBackgroundColor(LCD_GREEN);
//	LCD_FillRect(x,y,pos,size.height);
//	LCD_setBackgroundColor(LCD_RED);
//	LCD_FillRect(pos,y,10,size.height);
//	LCD_setBackgroundColor(LCD_BLUE);
//	LCD_FillRect(pos+10,y,320-pos-10,size.height);
	
	return size;
}

Widget * getWidgetNRFData()
{
	Widget * tempWidget=menu_getWidget();
	tempWidget->onDrawFunc=OnDrawWidgetNRFData;
	
	//sprintf(tempWidget->menuViewTempBuff, "%d", sensor_resistance_value);
	return tempWidget;
}


Size OnDrawWidgetValveStatus(uint16_t x, uint16_t y, bool selected)
{
	Size size;
	size.width=320;
	size.height=50;
	
	uint16_t pos=(HAL_GetTick()/10)&255;
	
	LCD_setBackgroundColor(LCD_GREEN);
	LCD_FillRect(x,y,pos,size.height);
	LCD_setBackgroundColor(LCD_RED);
	LCD_FillRect(pos,y,10,size.height);
	LCD_setBackgroundColor(LCD_BLUE);
	LCD_FillRect(pos+10,y,320-pos-10,size.height);
	
	return size;
}

Widget * getWidgetValveStatus()
{
	Widget * tempWidget=menu_getWidget();
	//tempWidget->onDrawFunc=OnDrawWidgetValveStatus;
	
	if(getSettingsStruct()->valveStatus==VALVE_STATUS_CLOSED)
	{
		sprintf(tempWidget->menuViewTempBuff, "%s", "Закрыты");
	}
	else
	{
		sprintf(tempWidget->menuViewTempBuff, "%s", "Открыты");
	}
	return tempWidget;
}

void setADCAccidentValue(uint16_t value)
{
	getSettingsStruct()->accident_level_value=value;
	counter_settings_data_buff_need_save=true;
}

void setRadioOnAirIntervalValue(uint16_t value)
{
	getSettingsStruct()->radio_onair_interval=value;
	counter_settings_data_buff_need_save=true;
}


//начало работа с реле
void Relay1On()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}
void Relay1Off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void Relay2On()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
}
void Relay2Off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}
//конец работа с реле
	
//начало настройка счетчиков воды 

void ccw_disp(void * param)
{
	count_cw.counter+=(*(int16_t*)param);
}

void chw_disp(void * param)
{
	count_hw.counter+=(*(int16_t*)param);
}
//конец настройка счетчиков воды 


void WaterRelePowerOff()
{
	Relay1Off();
	Relay2Off();
}


uint32_t water_rele_power_timer_off;
uint8_t water_rele_power_off_enabled=0;
void water_startRelePowerOffTimer()
{
	water_rele_power_timer_off=HAL_GetTick()+6000; //6 секунды
	water_rele_power_off_enabled=1;
}

void valve_relay_tick()
{
	if(!water_rele_power_off_enabled)return;
	if(HAL_GetTick()>=water_rele_power_timer_off)
	{
		WaterRelePowerOff();
		water_rele_power_off_enabled=0;
	}
}

void * memcpy(void * dest, const void * src, unsigned int len)
{
	for(int i=0;i<len;i++)
	{
		((char*)dest)[i]=((char*)src)[i];
	}
	return dest;
}

void saveValveStatus(ValveStatus status)
{
	if(getSettingsStruct()->valveStatus==status)
	{
		return;
	}
	void * ptr;
	switch(status)
	{
		case VALVE_STATUS_CLOSED:
		ptr=&getSettingsStruct()->valveCloseDate;
		break;
		case VALVE_STATUS_OPENED:
		ptr=&getSettingsStruct()->valveOpenDate;
		break;
	}
	memcpy(ptr, getCurrentDateTime(), sizeof(DateTime));
	getSettingsStruct()->valveStatus=status;
	counter_settings_data_buff_need_save=true;
}

//Открытие кранов
void ValveOn(void * param)
{
	if(loadAccidentStatus()==ACCIDENT_MODE)
	{
		return;//последнее состояние - авания, не даем открыть краны
	}
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==GPIO_PIN_SET)
	{
		return;//главный выключатель в положении "закрыт"
	}

	Relay1On();
	Relay2Off();
	water_startRelePowerOffTimer();

	saveValveStatus(VALVE_STATUS_OPENED);
}


//Закрытие кранов
void ValveOff(void * param)
{
	Relay1Off();
	Relay2On();
	water_startRelePowerOffTimer();
	
	saveValveStatus(VALVE_STATUS_CLOSED);
}


//сброс признака протечки
void resetWaterAlert(void * param)
{
	userAlreadyNotified=false;
	saveAccidentStatus(REGULAR_MODE);
	ValveOn(0);
}

//установка значений RTC
void time_h_plus(void * param)
{
	DateTime * dateTime=(DateTime*)time_data_buff;
	uint8_t h=HexToDec(dateTime->Time.Hour);
	h++;
	if(h==25)
	{
		h=0;
	}
	dateTime->Time.Hour=DecToHex(h);
	time_data_buff_need_save=1;
}

void time_m_plus(void * param)
{
	DateTime * dateTime=(DateTime*)time_data_buff;
	uint8_t m=HexToDec(dateTime->Time.Min);
	m++;
	if(m==60)
	{
		m=0;
	}
	dateTime->Time.Min=DecToHex(m);
	time_data_buff_need_save=1;
}

void time_sec_to_zero(void * param)
{
	DateTime * dateTime=(DateTime*)time_data_buff;
	uint8_t s=HexToDec(dateTime->Time.Sec);
	if(s>30)
	{
		time_m_plus(0);
	}
	dateTime->Time.Sec=0;
	time_data_buff_need_save=1;

}

void time_day_plus(void * param)
{
	DateTime * dateTime=(DateTime*)time_data_buff;
	uint8_t value=HexToDec(dateTime->Date.Day);
	value++;
	if(value==32)
	{
		value=1;
	}
	dateTime->Date.Day=DecToHex(value);
	time_data_buff_need_save=1;
}

void time_month_plus(void * param)
{
	DateTime * dateTime=(DateTime*)time_data_buff;
	uint8_t value=HexToDec(dateTime->Date.Mont);
	value++;
	if(value==13)
	{
		value=1;
	}
	dateTime->Date.Mont=DecToHex(value);
	time_data_buff_need_save=1;
}

void time_year_plus(void * param)
{
	DateTime * dateTime=(DateTime*)time_data_buff;
	uint8_t value=HexToDec(dateTime->Date.Year);
	value++;
	if(value==100)
	{
		value=0;
	}
	dateTime->Date.Year=DecToHex(value);
	time_data_buff_need_save=1;
}

uint32_t getKey(){
	return getSettingsStruct()->key;
}

void setKey(uint32_t key){
	getSettingsStruct()->key=key;
	counter_settings_data_buff_need_save=true;
}

//настройка уровня срабатывания датчика протечки
void accident_level_tune(void * param)
{
	setADCAccidentValue(getSettingsStruct()->accident_level_value + (*(int8_t*)param));
}

//настройка интервала выхода в эфир радиодатчика протечки
void radio_probe_onair_interval_tune(void * param)
{
	setRadioOnAirIntervalValue(getSettingsStruct()->radio_onair_interval + (*(int8_t*)param));
}

Widget * getWidgetCCW()
{
	Widget * tempWidget=menu_getWidget();
	sprintf(tempWidget->menuViewTempBuff, "%d", count_cw.counter);
	return tempWidget;
}
	
Widget * getWidgetCHW()
{
	Widget * tempWidget=menu_getWidget();
	sprintf(tempWidget->menuViewTempBuff, "%d", count_hw.counter);
	return tempWidget;
}

Widget * getWidgetRadioKey()
{
	Widget * tempWidget=menu_getWidget();
	sprintf(tempWidget->menuViewTempBuff, "%u", getKey());
	return tempWidget;
}



void resetKey(void * val)
{
	setKey(0);
}

const int16_t one=1;
const int16_t minus_one=-1;
const int16_t thousand=1000;
const int16_t minus_thousand=-1000;

Menu mainMenu;

MenuItem manual_water_control_menu_items[]={ {"Главное меню",menu_goToMenu,(void*)&mainMenu},
																						{"Открыть краны",ValveOn,0,0,X_PLUS_20},
																						{"Закрыть",ValveOff},
																						{"Закрыты: %s", 0, 0, getWidgetValveLastClose},
																						{"Открыты: %s", 0, 0, getWidgetValveLastOpen},
																						{"Краны: %s", 0, 0, getWidgetValveStatus},
																					};
Menu manualWaterControlMenu={(MenuItem*)manual_water_control_menu_items, 6};

MenuItem time_setup_menu_items[]={
																	{"%s", 0, 0, getWidgetCurrentDateTime},
																	{"Главное меню",menu_goToMenu,(void*)&mainMenu},
																	{"День",time_day_plus,0,0,X_PLUS_20},
																	{"Месяц",time_month_plus,0,0,X_PLUS_20},
																	{"Год",time_year_plus,0,0,NEW_LINE},
																	{"Часы",time_h_plus,0,0,X_PLUS_20},
																	{"Минуты",time_m_plus,0,0,X_PLUS_20},
																	{"Секунды",time_sec_to_zero,0,0,X_PLUS_20},
																	};
Menu timeSetupMenu={(MenuItem*)time_setup_menu_items, 8};

MenuItem counters_setup_menu_items[]={ {"Главное меню",menu_goToMenu,(void*)&mainMenu},
																						{"ХВС: %s",0,0,getWidgetCCW},
																						{"+1000",ccw_disp,(void*)&thousand,0,X_PLUS_20},
																						{"-1000",ccw_disp,(void*)&minus_thousand,0,X_PLUS_20},
																						{"+1",ccw_disp,(void*)&one,0,X_PLUS_20},
																						{"-1",ccw_disp,(void*)&minus_one},
																						
																						{"ГВС: %s",0,0,getWidgetCHW},
																						{"+1000",chw_disp,(void*)&thousand,0,X_PLUS_20},
																						{"-1000",chw_disp,(void*)&minus_thousand,0,X_PLUS_20},
																						{"+1",chw_disp,(void*)&one,0,X_PLUS_20},
																						{"-1",chw_disp,(void*)&minus_one}
																					};

																					

Menu countersSetupMenu={(MenuItem*)counters_setup_menu_items, 11};

MenuItem accident_settings_menu_items[]={ 	{"%s", 0, 0, getWidgetCurrentDateTime},
																						{"Главное меню",menu_goToMenu,(void*)&mainMenu},
																						{"Текущее: ",0, 0, 0, X_PLUS_0},
																						{"%s", 0, 0, getWidgetSensorResistance,X_PLUS_0},
																						{"/%s", 0, 0, getWidgetRadioSensorResistance},
																						
																						{"Граница: ", 0, 0, 0, X_PLUS_0},
																						{"%s", 0, 0, getWidgetAccidentLevel, X_160},
																						{"[+]",accident_level_tune,(void*)&one,0, X_PLUS_20},
																						{"[-]",accident_level_tune,(void*)&minus_one},

																						{"Ключ/сброс: %s",resetKey, 0, getWidgetRadioKey},
																						
																						{"Радиоэфир: ", 0, 0, 0, X_PLUS_0},
																						{"%s", 0, 0, getWidgetRadioOnAirInterval, X_PLUS_20},
																						{"[+]",radio_probe_onair_interval_tune,(void*)&one,0, X_PLUS_20},
																						{"[-]",radio_probe_onair_interval_tune,(void*)&minus_one},
																						{"Последний рапорт:"},
																						{"%s", 0, 0, getWidgetRadioProbeOnAirDateTime},

																					};
Menu accidentSettingsMenu={(MenuItem*)accident_settings_menu_items, 16};

MenuItem other_menu_items[]={ {"Главное меню",menu_goToMenu,(void*)&mainMenu},
																						{"%s", 0, 0, getWidgetNRFData},
																					};
Menu otherMenu={(MenuItem*)other_menu_items, 2};


MenuItem main_menu_itens[]={	{"%s", 0, 0, getWidgetCurrentDateTime},
															{"%s", 0, 0, getWidgetCounters},
															{"Текущий режим: %s", 0, 0, getWidgetWorkMode},
															{"Настройка тек.показаний...",menu_goToMenu,(void*)&countersSetupMenu},
															{"Настройка часов...",menu_goToMenu,(void*)&timeSetupMenu},
															{"Сброс признака протечки",resetWaterAlert},
															{"Состояние кранов...",menu_goToMenu,(void*)&manualWaterControlMenu},
															{"Датчики...",menu_goToMenu,(void*)&accidentSettingsMenu},
															//{"Прочее...",menu_goToMenu,(void*)&otherMenu},
															};
Menu mainMenu={(MenuItem*)main_menu_itens, 8};

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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //buzzer off
	
	nrf24_init(&hspi1);
	

	
	
	menu_Init(&mainMenu);
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of SPI1BinarySem */
  osSemaphoreDef(SPI1BinarySem);
  SPI1BinarySemHandle = osSemaphoreCreate(osSemaphore(SPI1BinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LCDTask */
  osThreadDef(LCDTask, StartLCDTask, osPriorityNormal, 0, 128);
  LCDTaskHandle = osThreadCreate(osThread(LCDTask), NULL);

  /* definition and creation of UserManagement */
  osThreadDef(UserManagement, StartUserManagementTask, osPriorityNormal, 0, 128);
  UserManagementHandle = osThreadCreate(osThread(UserManagement), NULL);

  /* definition and creation of I2CTimeTask */
  osThreadDef(I2CTimeTask, StartI2CTimeTask, osPriorityNormal, 0, 128);
  I2CTimeTaskHandle = osThreadCreate(osThread(I2CTimeTask), NULL);

  /* definition and creation of NRFf24Task */
  osThreadDef(NRFf24Task, StartNrf24Task, osPriorityIdle, 0, 128);
  NRFf24TaskHandle = osThreadCreate(osThread(NRFf24Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
	
	while (1)
	{
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc2);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  HAL_SPI_Init(&hspi1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13 
                           PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB14 
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



uint32_t waterDetectLastTick;
uint8_t waterDetectCounter=0;


DMA_HandleTypeDef rxtemp;

void setWaterAccidentMode()
{
		//Протечка!!! 0 - протечка
		if(loadAccidentStatus()==REGULAR_MODE)
		{
			saveAccidentStatus(ACCIDENT_MODE);
			ValveOff(0);
		}
}

void readSensorResistanceValue()
{
	HAL_ADC_Start(&hadc2);
	 HAL_ADC_PollForConversion(&hadc2, 100);
	if(HAL_ADC_GetState(&hadc2) == HAL_ADC_STATE_EOC_REG)
	{
		sensor_resistance_value=HAL_ADC_GetValue(&hadc2);
	}
	HAL_ADC_Stop(&hadc2);
}
	
void MainLED(GPIO_PinState r, GPIO_PinState g, GPIO_PinState b){
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, r);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, g);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, b);
}

#define MainLED_OFF MainLED(GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_RESET)
#define MainLED_RED MainLED(GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_RESET)
#define MainLED_GREEN MainLED(GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET)
#define MainLED_BLUE MainLED(GPIO_PIN_RESET,GPIO_PIN_RESET,GPIO_PIN_SET)

/* USER CODE END 4 */

/* StartLCDTask function */
void StartLCDTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	osSemaphoreWait(SPI1BinarySemHandle, osWaitForever);
	LCD_Init();
	osSemaphoreRelease(SPI1BinarySemHandle);
		
	hspi1.hdmarx=&rxtemp;
	menu_setNeedCLS(1);
	
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreWait(SPI1BinarySemHandle, osWaitForever);
		Menu_Draw();
		LCD_WaitAndReleaseSPI();
		osSemaphoreRelease(SPI1BinarySemHandle);
		
		osDelay(1);
	}
  /* USER CODE END 5 */ 
}

/* StartUserManagementTask function */
void StartUserManagementTask(void const * argument)
{
  /* USER CODE BEGIN StartUserManagementTask */
	while(!counter_settings_data_buff_inizialized)
	{
		osDelay(1);
	}
	
	BF_Init(&count_cw, 50);
	count_cw.counter=loadCCW();// BKP->DR1;
	BF_Init(&count_hw, 50);
	count_hw.counter=loadCHW();// BKP->DR2;

	Encoder_Init(&encoder1);

	LCD_UserActivity();

	GPIO_PinState main_switch_state=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	
	uint16_t coldStartCounter=10;
	
	/* Infinite loop */
	for(;;)
	{

		valve_relay_tick(); //следит, отключает питание реле воды после отработки

		//главный выключатель
		GPIO_PinState main_switch_new_state=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
		if(main_switch_new_state!=main_switch_state)
		{
			LCD_UserActivity();
			
			if(main_switch_state==GPIO_PIN_SET)
			{
				ValveOn(0);
			}
			else
			{
				ValveOff(0);
			}
				
			main_switch_state=main_switch_new_state;
		}
		
		
		//красная кнопка
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_RESET)
		{
			radio_sensor_resistance_value=0xffff; // до следующего эфира значение будет "неопределено"
			
			userAlreadyNotified=true;
			if(loadAccidentStatus()==REGULAR_MODE)
			{
				setWaterAccidentMode();
				LCD_UserActivity();
			}
		}
		
		//датчик протечки цифровой сигнал
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)==GPIO_PIN_RESET) 
		{
			if(HAL_GetTick()>waterDetectLastTick+100)
			{
				waterDetectCounter+=1;
				waterDetectLastTick=HAL_GetTick();
			}
			
			if(waterDetectCounter>=3) //три раза подрят, каждые 100 мсек датчик был активен
			{
				setWaterAccidentMode();
				LCD_UserActivity();
			}
		}
		else
		{
			waterDetectCounter=0;
		}
		
		//датчик протечки аналоговый сигнал
		if((coldStartCounter==0)&&(sensor_resistance_value<=getSettingsStruct()->accident_level_value))
		{
			setWaterAccidentMode();
			LCD_UserActivity();
		}
		
		//радио датчик
		if((radio_sensor_resistance_value!=0xffff)&&(radio_sensor_resistance_value<=getSettingsStruct()->accident_level_value))
		{
			setWaterAccidentMode();
			LCD_UserActivity();
		}

		Encoder_SetState(&encoder1, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10), HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11), HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2));
		
		Menu_DoUserAction(&encoder1);
		
		//аппаратное чтение контактов счетчиков
		BF_SetState(&count_cw, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12));
		BF_SetState(&count_hw, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11));


		//сохраняем в регистры
		if(count_cw.counter!=loadCCW())
		{
			saveCCW(count_cw.counter);
		}
		if(count_hw.counter!=loadCHW())
		{
			saveCHW(count_hw.counter);
		}

		//buzzer
		if((loadAccidentStatus()==ACCIDENT_MODE)&&(!userAlreadyNotified)) //ПРОТЕЧКА
		{
			switch(HAL_GetTick()&512)
			{
				case 0:
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);		
					break;
				default:
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);		
					break;
			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);		
		}
		
			
		uint16_t flashRate;
		if(loadAccidentStatus()==ACCIDENT_MODE) //ПРОТЕЧКА
		{
			flashRate=128;
		}
		else
		{
			flashRate=1024;
		}
			
		if((!LCD_IsActive())&&(HAL_GetTick()&flashRate))
		{
			if(loadAccidentStatus()==REGULAR_MODE)
			{
				if(main_switch_new_state==GPIO_PIN_RESET)
				{
					MainLED_GREEN;
				}
				else{
					MainLED_BLUE;
				}
			}
			else
			{
				MainLED_RED;
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		}
		else
		{
			MainLED_OFF;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		}
		
			

		
			
		osDelay(1);
		if(coldStartCounter){coldStartCounter--;}
	} 
	
  /* USER CODE END StartUserManagementTask */
}

/* StartI2CTimeTask function */
void StartI2CTimeTask(void const * argument)
{
  /* USER CODE BEGIN StartI2CTimeTask */
	
	//if(HAL_I2C_Mem_Read(&hi2c1, 174, 0, I2C_MEMADD_SIZE_16BIT, (uint8_t*)counter_settings_data_buff, 32, 100)!= HAL_OK)
	if(AT24C_Read(&hi2c1, 0,	counter_settings_data_buff, 32))
	{
	}
	if(AT24C_Read(&hi2c1, 32,	&counter_settings_data_buff[32], 32))
	{
	}
	counter_settings_data_buff_inizialized=true;

	
	/* Infinite loop */
	for(;;)
	{
		if(time_data_buff_need_save)
		{
			time_data_buff_need_save=0;
			if(HAL_I2C_Mem_Write(&hi2c1, 208, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t*)time_data_buff, 19, 100)!= HAL_OK)
			{
			}
		}
		else
		{
			if(HAL_I2C_Mem_Read(&hi2c1, 208, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t*)time_data_buff, 19, 100)!= HAL_OK)
			{
			}
		}
		
		if(counter_settings_data_buff_need_save)
		{
			counter_settings_data_buff_need_save=false;
			if(AT24C_Write(&hi2c1, 0, counter_settings_data_buff, 32))
			{
			}

			if(AT24C_Write(&hi2c1, 32, &counter_settings_data_buff[32], 32))
			{
			}
			
		}
		
		readSensorResistanceValue();

		osDelay(1000);
	}
  /* USER CODE END StartI2CTimeTask */
}

/* StartNrf24Task function */
void StartNrf24Task(void const * argument)
{
  /* USER CODE BEGIN StartNrf24Task */
	osSemaphoreWait(SPI1BinarySemHandle, osWaitForever);
	nrf24_config(2,32);
	nrf24_powerUpRx();
	
	
  
	
	nrf24_rx_address(master_controller_address);	
	nrf24_tx_address(zond_address);

	osSemaphoreRelease(SPI1BinarySemHandle);

	uint8_t AES_key[16]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	aes128_init(AES_key, &AES_ctx);
	
	//uint32_t actualToken;
	//uint32_t preparedToken;
	
//	for(int i=0;i<sizeof(actualToken);i++)
//	{
//		actualToken<<=8;
//		actualToken|=getRND8();
//	}

	//uint32_t lastOnAirCenter=0;

	
  /* Infinite loop */
  for(;;)
  {
		osDelay(10);
		
		osSemaphoreWait(SPI1BinarySemHandle, osWaitForever);
		if(nrf24_dataReady())
    {	
				EncryptedBlock probeBlock;
				nrf24_getData((uint8_t*)&probeBlock);
				osSemaphoreRelease(SPI1BinarySemHandle);
			
				aes128_dec((uint8_t*)&probeBlock, &AES_ctx);
				aes128_dec((uint8_t*)&probeBlock+16, &AES_ctx);

				if(probeBlock.token<=getKey()){
					continue;
				}
				setKey(probeBlock.token+1);
				
				srand(probeBlock.token);

				ProbeData * probeData=(ProbeData *)probeBlock.data;
				radio_sensor_resistance_value=probeData->sensor_val;
				getSettingsStruct()->radio_sensor_dateTime=*getCurrentDateTime();
				counter_settings_data_buff_need_save=true;

				//данные в зонд
//				EncryptedBlock mainBlock;
//				ControllerData * controllerData=(ControllerData *)mainBlock.data;
//				controllerData->air_interval=getSettingsStruct()->radio_onair_interval;
//				for(int i=sizeof(ControllerData);i<sizeof(EncryptedBlock);i++)
//				{
//					probeBlock.data[i]=rand()%255;
//				}					
//				aes128_enc(&mainBlock, &AES_ctx);
//				aes128_enc(&mainBlock+16, &AES_ctx);
//				
//				osSemaphoreWait(SPI1BinarySemHandle, osWaitForever);
//				nrf24_send(&mainBlock);
//				while(nrf24_isSending()){};
				
		}
		
//		if(HAL_GetTick()-lastOnAirCenter>1000)
//		{
//			lastOnAirCenter=HAL_GetTick();
//			
//			//данные в центр
//			
//			
//			nrf24_rx_address(master_controller_address);	
//			nrf24_tx_address(openhab_gate_address);
//			SettingsAndStatus * data=getSettingsStruct();	
//			nrf24_send(data);
//			while(nrf24_isSending()){};
//		}
//		
//		nrf24_rx_address(master_controller_address);	
//		nrf24_tx_address(zond_address);
//		nrf24_powerUpRx();
					
		osSemaphoreRelease(SPI1BinarySemHandle);
	
    
  }
  /* USER CODE END StartNrf24Task */
}

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
	//LCD_DrawString(0,0,"!!!",true);
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
