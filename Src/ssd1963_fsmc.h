#ifndef _SSD1963_H_
#define _SSD1963_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>//из этой библиотеки берём ф-цию abs()


#define HDP 799	//количество пикселей по горизонтали дисплея
#define VDP 479	//количество пикселей по вертикали дисплея


// Определяем адреса, по которым будем писать
// Для записи данных
#define LCD_DATA    			    ((uint32_t)0x60020000)    
// Для записи команд
#define LCD_REG   		  	    ((uint32_t)0x60000000)	  
 


#define MAX_X 799

#define GREY	  			0x8410
#define YELLOW   			0x07FF
#define	RED	  				0X001F
#define	GREEN	  			0X07E0
#define	BLUE      		0XF800
#define BLACK     		0X0000
#define WHITE     		0XFF18
#define DARK_BLUE 		0X3900


extern const unsigned char font8x8[][8];

//////////extern void delay_us(uint32_t us);
//////////extern void delay_ms(uint32_t ms);

void Init_Periph(void);
void Init_SSD1963(void);
void TFT_Set_XY(uint16_t x, uint16_t y);
void TFT_Set_Work_Area(uint16_t x, uint16_t y, uint16_t length, uint16_t width);


void TFT_Clear_Screen(uint16_t color);
void TFT_Draw_Char(uint16_t x, uint16_t y, uint16_t color, uint16_t phone, const uint8_t *table, uint8_t ascii, uint8_t size);
void TFT_Draw_String(uint16_t x, uint16_t y, uint16_t color, uint16_t phone, const uint8_t *table, char *string, uint8_t size);

void TFT_Draw_Line (uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint8_t size,uint16_t color);
void TFT_Draw_HLine(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t color);
void TFT_Draw_VLine(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t color);

void TFT_Draw_Rectangle(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint8_t size, uint16_t color);
void TFT_Draw_Fill_Rectangle(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t color);

void TFT_Draw_Round_Rect(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t r, uint8_t size, uint16_t color);
void TFT_Draw_Fill_Round_Rect(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t r, uint16_t color);

void TFT_Draw_Triangle( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t size, uint16_t color);
void TFT_Draw_Circle(uint16_t x, uint16_t y, uint8_t radius, uint8_t fill, uint8_t size, uint16_t color);

#endif
