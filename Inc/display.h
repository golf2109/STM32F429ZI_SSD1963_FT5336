#ifndef DISPLAY_SSD1963
#define DISPLAY_SSD1963

#include "stm32f4xx_hal.h"

#define DISP_WIDTH  800
#define DISP_HEIGHT 480

#define HDP		(DISP_WIDTH - 1)
#define HT 		928
#define HPS		46
#define LPS		15
#define HPW		48

#define VDP		(DISP_HEIGHT - 1)
#define VT		525
#define VPS		16
#define FPS		8
#define VPW		16

// Определяем адреса
// для записи данных
#define LCD_DATA 0x60020000
// для записи команд
#define LCD_REG 0x60000000

#define yellow 	  0x07FF
#define magneta   0xF81F
#define cyan      0xFFE0
#define	red		  	0X001F
#define	green	  	0X07E0
#define	blue      0XF800
#define white     0XFFFF
#define black     0X3185

void Lcd_Write_Index(uint16_t index);
void Lcd_Write_Data(uint16_t data);
uint16_t Lcd_Read_Data(void);
uint16_t Lcd_Read_Reg(uint16_t reg_addr);
void Lcd_Write_Reg(uint16_t reg,uint16_t value);
void Set_Cursor(uint16_t x_kur, uint16_t y_kur);
void Initial_SSD1963(void);

#endif
