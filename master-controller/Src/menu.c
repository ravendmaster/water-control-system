#include "cmsis_os.h"
#include "menu.h"
#include <stdio.h>
#include "LCD.h"
#include <stdbool.h>

typedef struct
{
	Menu * cur_menu;
	uint8_t drawing;
	int8_t cur_position;
	GPIO_PinState cur_button_state;
	uint8_t needCLS;
	uint32_t repeatNextModeTick;
	uint8_t repeatSpeed;
} MenuInstance;

MenuInstance menuInstance;

Widget widgetInstance;

Widget * menu_getWidget()
{
	widgetInstance.foregroundColor=1;// 1 - def LCD_BLACK;
	widgetInstance.backgroundColor=1;// 1 - def LCD_WHITE;
	widgetInstance.onDrawFunc=0;
	return &widgetInstance;
}

void menu_Init(Menu * rootMenu)
{
	menuInstance.repeatSpeed=1;
	menuInstance.repeatNextModeTick=0;
	menuInstance.needCLS=0;
	menuInstance.drawing=0;
	menuInstance.cur_position=0;
	menuInstance.cur_menu=rootMenu;
}

void menu_setNeedCLS(uint8_t value)
{
	menuInstance.needCLS=value;
}

	
void menu_goToMenu(void * param)
{
	if(param==0)return;
	
	while(menuInstance.drawing)osDelay(1);
	menuInstance.cur_menu=(Menu*)param;
	menuInstance.cur_position=0;
	while(menuInstance.cur_menu->items[menuInstance.cur_position].actionFunc==0)
	{
		menuInstance.cur_position++;
	}
	
	menuInstance.needCLS=1;
}

uint16_t max(uint16_t arg1, uint16_t arg2)
{
	if(arg1>arg2)return arg1;
	return arg2;
}

void Menu_Draw()
{
	LCD_LightControlTick();

	if(!LCD_IsActive())return;

	
	if(menuInstance.needCLS)
	{
		menu_setNeedCLS(0);
		LCD_setForegroundColor(LCD_BLACK);
		LCD_setBackgroundColor(LCD_WHITE);
		LCD_FillRect(0,0,320,240);
	}	
	
	menuInstance.drawing=1;

	uint16_t current_xpos=0;
	uint16_t current_ypos=0;

	uint16_t max_height=0;
	
	for(int i=0;i<menuInstance.cur_menu->size;i++)
	{
		char * text;

		Widget * widget=0;
		if(menuInstance.cur_menu->items[i].widget!=0)
		{
			widget=menuInstance.cur_menu->items[i].widget();
		}
		
		if((widget)&&(widget->onDrawFunc))
		{
			Size size=widget->onDrawFunc(current_xpos,current_ypos, i==menuInstance.cur_position);
			max_height=max(max_height, size.height);
			current_xpos=current_xpos+size.width;
		}
		else
		{
			if(i==menuInstance.cur_position)
			{
				LCD_setForegroundColor(LCD_WHITE);
				LCD_setBackgroundColor(LCD_BLUE);
			}
			else
			{
				if(menuInstance.cur_menu->items[i].actionFunc==0)
				{
					LCD_setForegroundColor(LCD_RGB(0.5,0.5,1));
				}else{
					LCD_setForegroundColor(LCD_BLACK);
				}
				
				LCD_setBackgroundColor(LCD_WHITE);
			}
			
			if(widget)
			{
				char textbuff[32];
				//Widget * widget=menuInstance.cur_menu->items[i].widget();
				if(i!=menuInstance.cur_position){
					
					if(widget->backgroundColor!=1){
						LCD_setBackgroundColor(widget->backgroundColor);
					}
					
					if(widget->foregroundColor!=1){
						LCD_setForegroundColor(widget->foregroundColor);
					}
				}
				sprintf(textbuff, (char*)menuInstance.cur_menu->items[i].text, widget->menuViewTempBuff);
				text=textbuff;
			}
			else
			{
				text=(char*)menuInstance.cur_menu->items[i].text;
			}
			LCD_FillRect(current_xpos++,current_ypos,1,CHAR_H_PXL); //отступ слева, перед первым символом
			current_xpos=LCD_DrawString(current_xpos,current_ypos, text, menuInstance.cur_menu->items[i].terminator==NEW_LINE);
			max_height=max(max_height,CHAR_H_PXL);
		}

		uint8_t space=0;
		switch(menuInstance.cur_menu->items[i].terminator)
		{
			case X_PLUS_0:
				break;
			case X_160:
				space=169-current_xpos;
			break;
			case X_PLUS_20:
				space=20;
				break;
			case NEW_LINE:
				break;
		}
		//int ypos=current_ypos;//(CHAR_H_PXL)*(current_line);
		
		if(space)
		{
			LCD_FillRect(current_xpos, current_ypos, space, CHAR_H_PXL);
			current_xpos+=space;
		}
		
		if(menuInstance.cur_menu->items[i].terminator==NEW_LINE)
		{
			current_ypos+=max_height;
			current_xpos=0;
		}
		
	}
	menuInstance.drawing=0;
}

void Menu_DoUserAction(Encoder * encoder)
{
	uint8_t button_on=(Encoder_GetButtonState(encoder)==GPIO_PIN_RESET);
	
	int8_t disp=Encoder_GetDisp(encoder);
	if((disp!=0)&&(!button_on))
	{
		LCD_UserActivity();
		
		//menuInstance.cur_position+=disp;
		int8_t newPos=menuInstance.cur_position+disp;
		while(1)
		{
			if(menuInstance.cur_menu->items[newPos].actionFunc!=0)
			{
				menuInstance.cur_position=newPos;
				break;
			}
			newPos+=disp;
			if((newPos<0)||(newPos>menuInstance.cur_menu->size-1))
			{
				break;
			}
		}
		
		
		if(menuInstance.cur_position>menuInstance.cur_menu->size-1)
		{
			menuInstance.cur_position=menuInstance.cur_menu->size-1;
		}
		if(menuInstance.cur_position<0)
		{
		menuInstance.cur_position=0;
		}
	}
	
	if((menuInstance.cur_button_state!=encoder->button_state) || (HAL_GetTick()>menuInstance.repeatNextModeTick))
	{
			menuInstance.repeatNextModeTick=HAL_GetTick()+1000/(menuInstance.repeatSpeed*4);
			if(menuInstance.repeatSpeed<250){menuInstance.repeatSpeed+=1;}
		
		if(button_on)
		{
			LCD_UserActivity();
			if(menuInstance.cur_menu->items[menuInstance.cur_position].actionFunc!=0)
			{
				menuInstance.cur_menu->items[menuInstance.cur_position].actionFunc( menuInstance.cur_menu->items[menuInstance.cur_position].param);
				if(menuInstance.cur_menu->items[menuInstance.cur_position].actionFunc == menu_goToMenu)
				{
					menuInstance.repeatSpeed=1;
				}
			}
		}
		else
		{
			menuInstance.repeatSpeed=1;
		}
		menuInstance.cur_button_state=encoder->button_state;
	}
}
