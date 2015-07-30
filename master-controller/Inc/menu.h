#include "stm32f1xx_hal.h"
#include "encoder.h"
#include "LCD.h"
#include <stdbool.h>

enum MenuItemTerminator
 {
	 NEW_LINE=0,
	 X_PLUS_0,
	 X_PLUS_20,
	 X_160
 };
 
 typedef struct
{
	char menuViewTempBuff[32];
	uint16_t foregroundColor;
	uint16_t backgroundColor;
	Size (*onDrawFunc)(uint16_t x, uint16_t y, bool selected);
}Widget;

 typedef struct 
 {
	 char text[40];
	 void (*actionFunc)(void * param);
	 void * param;
	 Widget * (*widget)();
	 enum MenuItemTerminator terminator;
 } MenuItem;
 
typedef struct 
{
	MenuItem * items;
	uint8_t size;
} Menu; 

Widget * menu_getWidget(void);
	
void menu_setNeedCLS(uint8_t value);

void menu_Init(Menu * rootMenu);

void menu_goToMenu(void * param);

void Menu_Draw(void);

void Menu_DoUserAction(Encoder * encoder);
