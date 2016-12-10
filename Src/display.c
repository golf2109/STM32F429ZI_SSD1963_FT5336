#include "display.h"

void Lcd_Write_Index(uint16_t index)
{
   *(uint16_t *) (LCD_REG) = index;
}
////////////////////////
void Lcd_Write_Data(uint16_t data)
{
   *(uint16_t *) (LCD_DATA)= data;
}
///////////////////
uint16_t Lcd_Read_Data(void)
{
   uint16_t data = * (uint16_t *)(LCD_DATA);
   return data;
}
////////////////////////
uint16_t Lcd_Read_Reg(uint16_t reg_addr)
{
   volatile uint16_t data = 0;
   Lcd_Write_Index(reg_addr);
   data = Lcd_Read_Data();
   return data;
}
///////////////////////
void Lcd_Write_Reg(uint16_t reg,uint16_t value)
{
   *(uint16_t *) (LCD_REG) = reg;
   *(uint16_t *) (LCD_DATA) = value;
}

//////////////////
//ф-ция выбирает ячейку в видеоОЗУ
void Set_Cursor(uint16_t x_kur, uint16_t y_kur)
{	
	Lcd_Write_Reg(0x004e,x_kur);
	Lcd_Write_Reg(0x004f,y_kur);
	Lcd_Write_Index(0x0022);

}
////////////////////////
//ф-ция закрашивает экран выбранным цветом
void Lcd_Clear(uint16_t color)
{
	uint32_t index = 0;
	
	Set_Cursor(0,0);	
	
	  for(index=0;index < 76800;index++)
	  { 			
		  Lcd_Write_Data(color);
	  }
}

void Initial_SSD1963(void)
{
////WRITE COMMAND У0xE0Ф
////WRITE DATA У0x01Ф
////Wait 100us to let the PLL stable
////WRITE COMMAND У0xE0Ф
////WRITE DATA У0x03Ф
////WRITE COMMAND У0x01Ф
  
Lcd_Write_Index(0xe0); //START PLL
Lcd_Write_Data(0x01);  //Set bit Enable PLL
HAL_Delay(50);  //Wait  to let the PLL stable
  
Lcd_Write_Index(0xe0); //START PLL
Lcd_Write_Data(0x03);  //(0 - Use reference clock as system clock   1 -  Use PLL output as system clock)
HAL_Delay(5);

Lcd_Write_Index(0x01); // Software reset
HAL_Delay(10);  
  
Lcd_Write_Index(0xe2);//Set the PLL
Lcd_Write_Data(0x1d);
Lcd_Write_Data(0x02);
Lcd_Write_Data(0x54);
  
Lcd_Write_Index(0xe6);//Set the LSHIFT (pixel clock) frequency
Lcd_Write_Data(0x04);
Lcd_Write_Data(0x6f);
Lcd_Write_Data(0x47);  

Lcd_Write_Index(0xf0);//SET pixel data I/F format=8bit
Lcd_Write_Data(0x03); //pixel data format, 0x03 is 16bit(565 format);0x00 is for 8-bit
//Set the MN of PLL

//////cput(0xe2);   //SET PLL freq=113.33MHz ;
//////dput(0x1d); // values set are diff in app note (1D)
//////dput(0x02);   // values set are diff in app note (02)
//////dput(0x54); // values set are diff in app note (54)
////// 
//////cput(0xe6);   //SET PCLK freq=33.26MHz ; pixel clock frequency
//////dput(0x04); //01 values set are diff in app note 0x04
//////dput(0x6f); //30 values set are diff in app note 0x6f
//////dput(0x47); //16 values set are diff in app note 0x47

Lcd_Write_Index(0xb0);	//LCD SPECIFICATION   SET LCD MODE SET TFT 18Bits MODE
Lcd_Write_Data(0x20);  //dput(0x08); //SET TFT MODE & hsync+Vsync+DEN MODE
Lcd_Write_Data(0x80);  //
Lcd_Write_Data(0x03);//SET horizontal size=800-1 HightByte
Lcd_Write_Data(0x1f);//1f   //SET horizontal size=800-1 LowByte
Lcd_Write_Data(0x01);//SET vertical size=480-1 HightByte
Lcd_Write_Data(0xdf);//df   //SET vertical size=480-1 LowByte
Lcd_Write_Data(0x00);//   dput(0x2d); //SET even/odd line RGB seq.=RGB

//Set front porch and back porch
Lcd_Write_Index(0xb4);// SET HBP
Lcd_Write_Data(0x04); // SET HSYNC Total = // 0x04 SET HSYNC Total = 1056  // hsync of  value in app note is 8367 & not 1056
Lcd_Write_Data(0x20);  
Lcd_Write_Data(0x00); // SET HBP = //SET HBP = 256 // HBP of value in app note is 163 & not 256
Lcd_Write_Data(0x2e);  //SET VBP 0 //  earlier value of VBP = 8 , write data ( 0x07)
Lcd_Write_Data(0xd2);// SET VBP
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x00);// SET Hsync pulse start position
Lcd_Write_Data(0x00);// SET Hsync pulse subpixel start position

Lcd_Write_Index(0xb6);    //SET VBP,
Lcd_Write_Data(0x02); //0x02 SET Vsync total 526 = 525 + 1  // vsync of  value in app note is 496  & not 525
Lcd_Write_Data(0x0d);
Lcd_Write_Data(0x00); //SET VBP = 45                // HBP of value in app note is 4 & not 45
Lcd_Write_Data(0x17);// 0x2d
Lcd_Write_Data(0x16);// 0x00 SET Vsync pulse 0     // SET Vsync pulse in app note is 2 
Lcd_Write_Data(0x00); //SET Vsync pulse start position
Lcd_Write_Data(0x00);

Lcd_Write_Index(0x2a);    //SET column address
Lcd_Write_Data(0x00); //SET start column address=0
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x03); //SET end column address=799
Lcd_Write_Data(0x1f);

Lcd_Write_Index(0x2b);;     //SET page address
Lcd_Write_Data(0x00);//SET start page address=0
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x01);//SET end page address=479
Lcd_Write_Data(0xdf);  //1f

//Lcd_Write_Index(0x36);;     //SET address mode to rotate mode
//Lcd_Write_Data(0x60);
//Lcd_Write_Index(0x3a);  //Reserved command ???????????
//Lcd_Write_Data(0x60);


////////===========================================
//////Lcd_Write_Index(0x33);
//////Lcd_Write_Data(0x00); //SET top fixed area=0
//////Lcd_Write_Data(0xB4);
//////Lcd_Write_Data(0x00); //SET vertical scrolling area=479
//////Lcd_Write_Data(0x94);
//////Lcd_Write_Data(0x00); //SET bottom fixed area=0
//////Lcd_Write_Data(0x98);
////////===============================================

Lcd_Write_Index(0xb8);
Lcd_Write_Data(0x0f);
Lcd_Write_Data(0x01);

Lcd_Write_Index(0xba);
Lcd_Write_Data(0x01);

Lcd_Write_Index(0x29);// SET display on

Lcd_Write_Index(0x2c);
}

