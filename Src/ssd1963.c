//=============================================================================
// SSD1963 driver for STM32 microcontrollers
// (c) Radoslaw Kwiecien, radek@dxp.pl	
// http://en.radzio.dxp.pl/ssd1963/
//=============================================================================
#include "ssd1963.h"
//=============================================================================
// Write command to SSD1963
//=============================================================================
////////void SSD1963_WriteCommand(unsigned int commandToWrite)
////////{
////////SSD1963_DATAPORT->ODR  = commandToWrite;
////////SSD1963_CTRLPORT->BSRR = SSD1963_PIN_RD;
////////SSD1963_CTRLPORT->BRR  = SSD1963_PIN_A0 | SSD1963_PIN_CS | SSD1963_PIN_WR;
////////asm("nop");
////////SSD1963_CTRLPORT->BSRR = SSD1963_PIN_A0 | SSD1963_PIN_CS | SSD1963_PIN_WR;
////////}
//////////=============================================================================
////////// Write data to SSD1963
//////////=============================================================================
////////void SSD1963_WriteData(unsigned int dataToWrite)
////////{
////////SSD1963_DATAPORT->ODR  = dataToWrite;
////////SSD1963_CTRLPORT->BSRR = SSD1963_PIN_RD | SSD1963_PIN_A0;
////////SSD1963_CTRLPORT->BRR  = SSD1963_PIN_CS | SSD1963_PIN_WR;
////////asm("nop");
////////SSD1963_CTRLPORT->BSRR = SSD1963_PIN_CS | SSD1963_PIN_WR;
////////}
//=============================================================================
// 
//=============================================================================
void SSD1963_Init (void)
{
volatile unsigned int dly;
////SSD1963_CTRLPORT->BRR = SSD1963_PIN_RST;
////for(dly = 0; dly < 1000; dly++);
////SSD1963_CTRLPORT->BSRR = SSD1963_PIN_RST;
////for(dly = 0; dly < 1000; dly++);

Lcd_Write_Index(SSD1963_SOFT_RESET);

Lcd_Write_Index(SSD1963_SET_PLL_MN);
Lcd_Write_Data(49);	// PLLclk = REFclk * 50 (500MHz)
Lcd_Write_Data(4);	// SYSclk = PLLclk / 5  (100MHz)
Lcd_Write_Data(4);  // dummy

Lcd_Write_Index(SSD1963_SET_PLL);
Lcd_Write_Data(0x01);
 
for(dly = 0; dly < 100000; dly++);

Lcd_Write_Index(SSD1963_SET_PLL);
Lcd_Write_Data(0x03);

Lcd_Write_Index(SSD1963_SET_LCD_MODE); 
Lcd_Write_Data(0x0C);			
Lcd_Write_Data(0x00);			
Lcd_Write_Data(mHIGH((TFT_WIDTH-1)));
Lcd_Write_Data(mLOW((TFT_WIDTH-1)));
Lcd_Write_Data(mHIGH((TFT_HEIGHT-1)));
Lcd_Write_Data(mLOW((TFT_HEIGHT-1)));
Lcd_Write_Data(0x00);

Lcd_Write_Index(SSD1963_SET_PIXEL_DATA_INTERFACE);
Lcd_Write_Data(SSD1963_PDI_16BIT565);

Lcd_Write_Index(SSD1963_SET_LSHIFT_FREQ); 
Lcd_Write_Data((LCD_FPR >> 16) & 0xFF);
Lcd_Write_Data((LCD_FPR >> 8) & 0xFF);
Lcd_Write_Data(LCD_FPR & 0xFF);

Lcd_Write_Index(SSD1963_SET_HORI_PERIOD);
Lcd_Write_Data(mHIGH(TFT_HSYNC_PERIOD));
Lcd_Write_Data(mLOW(TFT_HSYNC_PERIOD));
Lcd_Write_Data(mHIGH((TFT_HSYNC_PULSE + TFT_HSYNC_BACK_PORCH)));
Lcd_Write_Data(mLOW((TFT_HSYNC_PULSE + TFT_HSYNC_BACK_PORCH)));
Lcd_Write_Data(TFT_HSYNC_PULSE);
Lcd_Write_Data(0x00);			
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x00);			

Lcd_Write_Index(SSD1963_SET_VERT_PERIOD); 		
Lcd_Write_Data(mHIGH(TFT_VSYNC_PERIOD));
Lcd_Write_Data(mLOW(TFT_VSYNC_PERIOD));
Lcd_Write_Data(mHIGH((TFT_VSYNC_PULSE + TFT_VSYNC_BACK_PORCH)));
Lcd_Write_Data(mLOW((TFT_VSYNC_PULSE + TFT_VSYNC_BACK_PORCH)));
Lcd_Write_Data(TFT_VSYNC_PULSE);
Lcd_Write_Data(0x00);			
Lcd_Write_Data(0x00);

Lcd_Write_Index(SSD1963_SET_DISPLAY_ON);		//SET display on
}
//=============================================================================
//
//=============================================================================
void SSD1963_SetArea(unsigned int sx, unsigned int ex, unsigned int sy, unsigned int ey)
{
Lcd_Write_Index(SSD1963_SET_COLUMN_ADDRESS);	
Lcd_Write_Data((sx >> 8) & 0xFF);
Lcd_Write_Data((sx >> 0) & 0xFF);
Lcd_Write_Data((ex >> 8) & 0xFF);
Lcd_Write_Data((ex >> 0) & 0xFF);

Lcd_Write_Index(SSD1963_SET_PAGE_ADDRESS);	
Lcd_Write_Data((sy >> 8) & 0xFF);
Lcd_Write_Data((sy >> 0) & 0xFF);
Lcd_Write_Data((ey >> 8) & 0xFF);
Lcd_Write_Data((ey >> 0) & 0xFF);
}
//=============================================================================
// Fill area of specified color
//=============================================================================
void SSD1963_FillArea(unsigned int sx, unsigned int ex, unsigned int sy, unsigned int ey, int color)
{
int i;
SSD1963_SetArea(sx, ex, sy, ey);
Lcd_Write_Index(SSD1963_WRITE_MEMORY_START);
for(i = 0; i < ((ex-sx+1)*(ey-sy+1)); i++)
	{
	Lcd_Write_Data(color);
	}
}
//=============================================================================
// Fills whole screen specified color
//=============================================================================
void SSD1963_ClearScreen(unsigned long color)
{
unsigned int x,y;
SSD1963_SetArea(0, TFT_WIDTH-1 , 0, TFT_HEIGHT-1);
Lcd_Write_Index(0x2c);
for(x=0;x<TFT_WIDTH;x++)
	{
        for(y= 0;y<TFT_HEIGHT;y++)
                {
                Lcd_Write_Data(color);
                }
	}
}
//=============================================================================
//
//=============================================================================
void GLCD_SetPixel(int x, int y, int color)
{
SSD1963_SetArea(x, x, y, y);
Lcd_Write_Index(0x2c);
Lcd_Write_Data(color);
Lcd_Write_Index(0x0);
}
//=============================================================================
//
//=============================================================================
