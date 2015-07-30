#include "stdint.h"
#include <stdbool.h>

#ifndef _LCD_H
#define _LCD_H

#define CHAR_H_PXL 27
#define CHAR_W_BYTES 3
#define CHAR_MAX_BUFF_SIZE (CHAR_H_PXL*CHAR_W_BYTES*8)

#define LCD_BLACK 0x0000
#define LCD_WHITE 0xFFFF

#define LCD_BLUE 0x1f00
#define LCD_RED 0xf0
#define LCD_GREEN 0xf

struct
{
	uint16_t width;
	uint16_t height;
} typedef Size;

void LCD_WaitAndReleaseSPI(void);

void LCD_Init(void);

bool LCD_IsActive(void);
	
uint16_t LCD_RGB(float r, float g, float b);

void LCD_setBackgroundColor(uint16_t color);

void LCD_setForegroundColor(uint16_t color);

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);

uint16_t LCD_DrawString(uint16_t x, uint16_t y, char *str, bool fillLine);
	
void LCD_SendCommand(uint8_t com);

void LCD_SendData(uint8_t data);

void LCD_UserActivity(void);

void LCD_LightControlTick(void);

#endif
