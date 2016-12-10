#include "ssd1963_fsmc.h"
void delay_us(uint32_t us)
{
////	int32_t us_count_tick =  us * (SystemCoreClock/1000000);
////	//разрешаем использовать счётчик
////	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
////        //обнуляем значение счётного регистра
////	DWT_CYCCNT  = 0;
////        //запускаем счётчик
////	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk; 
////	while(DWT_CYCCNT < us_count_tick);
////        //останавливаем счётчик
////	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
	
}
//////////////////////////////
void delay_ms(uint32_t ms)
{
////	int32_t ms_count_tick =  ms * (SystemCoreClock/1000);
////	//разрешаем использовать счётчик
////	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
////         //обнуляем значение счётного регистра
////	DWT_CYCCNT  = 0;
////        //запускаем счётчик
////	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
////	while(DWT_CYCCNT < ms_count_tick);
////        //останавливаем счётчик
////	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
	
}
//Ф-ция инициализирует выводы и задает настройки FSMC
void Init_Periph()
{
//////////		FSMC_NORSRAMInitTypeDef  fsmc;
//////////    FSMC_NORSRAMTimingInitTypeDef fsmcTiming;

//////////		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN| RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN;
//////////		RCC->AHBENR |= RCC_AHBENR_FSMCEN;
//////////		
//////////		GPIOD->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1 | GPIO_CRL_CNF4 | GPIO_CRL_CNF5 | GPIO_CRL_CNF7);
//////////		GPIOD->CRL |= GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1 | GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1 | GPIO_CRL_CNF7_1;
//////////		
//////////		GPIOD->CRL	|= GPIO_CRL_MODE0 | GPIO_CRL_MODE1 | GPIO_CRL_MODE4 | GPIO_CRL_MODE5 | GPIO_CRL_MODE7;
//////////		
//////////		
//////////		GPIOD->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_CNF9 | GPIO_CRH_CNF10 | GPIO_CRH_CNF11 | GPIO_CRH_CNF14 | GPIO_CRH_CNF15);
//////////		GPIOD->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_1 | GPIO_CRH_CNF10_1 | GPIO_CRH_CNF11_1 | GPIO_CRH_CNF14_1 | GPIO_CRH_CNF15_1;
//////////		
//////////		GPIOD->CRH |= GPIO_CRH_MODE8 | GPIO_CRH_MODE9 | GPIO_CRH_MODE10 | GPIO_CRH_MODE11 | GPIO_CRH_MODE14 | GPIO_CRH_MODE15;
//////////		
//////////		GPIOE->CRL &= ~ (GPIO_CRL_CNF7 | GPIO_CRL_CNF1);
//////////		GPIOE->CRL |=   GPIO_CRL_CNF7_1;
//////////		
//////////		GPIOE->CRL |=   GPIO_CRL_MODE7 | GPIO_CRL_MODE1;
//////////		
//////////		GPIOE->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_CNF9 | GPIO_CRH_CNF10 | GPIO_CRH_CNF11 | GPIO_CRH_CNF12 | GPIO_CRH_CNF13 | GPIO_CRH_CNF14 | GPIO_CRH_CNF15);
//////////		GPIOE->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_1 | GPIO_CRH_CNF10_1 | GPIO_CRH_CNF11_1 | GPIO_CRH_CNF12_1 | GPIO_CRH_CNF13_1 | GPIO_CRH_CNF14_1 | GPIO_CRH_CNF15_1;
//////////		
//////////		GPIOE->CRH |= GPIO_CRH_MODE8 | GPIO_CRH_MODE9 | GPIO_CRH_MODE10 | GPIO_CRH_MODE11 | GPIO_CRH_MODE12 | GPIO_CRH_MODE13 | GPIO_CRH_MODE14 | GPIO_CRH_MODE15;
//////////	
//////////		GPIOE->BSRR |= GPIO_BSRR_BR1;
//////////		GPIOD->BSRR |= GPIO_BSRR_BS4 | GPIO_BSRR_BS5 | GPIO_BSRR_BS7;
//////////		
//////////    // Настройка FSMC
//////////    fsmcTiming.FSMC_AddressSetupTime = 0x02;
//////////    fsmcTiming.FSMC_AddressHoldTime = 0x00;
//////////    fsmcTiming.FSMC_DataSetupTime = 0x05;
//////////    fsmcTiming.FSMC_BusTurnAroundDuration = 0x00;
//////////    fsmcTiming.FSMC_CLKDivision = 0x00;
//////////    fsmcTiming.FSMC_DataLatency = 0x00;
//////////    fsmcTiming.FSMC_AccessMode = FSMC_AccessMode_B;
////////// 
//////////    fsmc.FSMC_Bank = FSMC_Bank1_NORSRAM1;
//////////    fsmc.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
//////////    fsmc.FSMC_MemoryType = FSMC_MemoryType_NOR;
//////////    fsmc.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
//////////    fsmc.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
//////////    fsmc.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
//////////    fsmc.FSMC_WrapMode = FSMC_WrapMode_Disable;
//////////    fsmc.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
//////////    fsmc.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
//////////    fsmc.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
//////////    fsmc.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
//////////    fsmc.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
//////////    fsmc.FSMC_ReadWriteTimingStruct = &fsmcTiming;
//////////    fsmc.FSMC_WriteTimingStruct = &fsmcTiming;	   
////////// 
//////////    FSMC_NORSRAMInit(&fsmc); 
//////////    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);	

}

//ф-ция для отправки команд
static __inline void 	TFT_Send_Cmd(uint8_t index)
{
	*(uint8_t *) (LCD_REG) = index;	
}

//ф-ция для отправки данных
static __inline void TFT_Write_Data(uint16_t data)
{   
    *(uint16_t *) (LCD_DATA) = data; 	
}

//ф-ция инициализации дисплея
void Init_SSD1963(void)
{
	//Reset дисплея
	GPIOE->BSRR |= GPIO_BSRR_BR1;
	delay_ms(1);					   
	GPIOE->BSRR |= GPIO_BSRR_BS1;	

	
	TFT_Send_Cmd(0x00E2);	//PLL multiplier, set PLL clock to 120M
	TFT_Write_Data(0x0023);	    //N=0x36 for 6.5M, 0x23 for 10M crystal
	TFT_Write_Data(0x0002);
	TFT_Write_Data(0x0054);
	TFT_Send_Cmd(0x00E0);  // PLL enable
	TFT_Write_Data(0x0001);
	delay_ms(10);
	TFT_Send_Cmd(0x00E0);
	TFT_Write_Data(0x0003);		// now, use PLL output as system clock
	delay_ms(10);
	TFT_Send_Cmd(0x0001);  // software reset
	delay_ms(20);
	TFT_Send_Cmd(0x00E6);	//PLL setting for PCLK, depends on resolution
	TFT_Write_Data(0x0003);
	TFT_Write_Data(0x0033);
	TFT_Write_Data(0x0033);

	TFT_Send_Cmd(0x00B0);	//LCD SPECIFICATION
	TFT_Write_Data(0x0020); //24 bit TFT panel
	TFT_Write_Data(0x0000); //Hsync+Vsync +DE mode  TFT mode
	TFT_Write_Data((HDP>>8));  //Set HDP
	TFT_Write_Data(HDP);
    TFT_Write_Data(VDP>>8);  //Set VDP
	TFT_Write_Data(VDP);
    TFT_Write_Data(0x0000);

	TFT_Send_Cmd(0x00B4);	//HSYNC
	TFT_Write_Data(0x04);  //Set HT
	TFT_Write_Data(0x1f);
	TFT_Write_Data(0x00);  //Set HPS
	TFT_Write_Data(0xd2);
	TFT_Write_Data(0x00);			   //Set HPW
	TFT_Write_Data(0x00);  //Set HPS
	TFT_Write_Data(0x00);
	TFT_Write_Data(0x00);

	TFT_Send_Cmd(0x00B6);	//VSYNC
	TFT_Write_Data(0x02);   //Set VT
	TFT_Write_Data(0x0c);
	TFT_Write_Data(0x00);  //Set VPS
	TFT_Write_Data(0x22);
	TFT_Write_Data(0x00);		//Set VPW
	TFT_Write_Data(0x00);  //Set FPS
	TFT_Write_Data(0x00);


	TFT_Send_Cmd(0x00B8);
	TFT_Write_Data(0x000f);    //GPIO is controlled by host GPIO[3:0]=output   GPIO[0]=1  LCD ON  GPIO[0]=1  LCD OFF 
	TFT_Write_Data(0x0001);    //GPIO0 normal

	TFT_Send_Cmd(0x00BA);
	TFT_Write_Data(0x0001);    //GPIO[0] out 1 --- LCD display on/off control PIN

	TFT_Send_Cmd(0x0036); //rotation
	TFT_Write_Data(0x00C0);//RGB=BGR

	TFT_Send_Cmd(0x003A); //Set the current pixel format for RGB image data
	TFT_Write_Data(0x0050);//16-bit/pixel

	TFT_Send_Cmd(0x00F0); //Pixel Data Interface Format
	TFT_Write_Data(0x0003);//16-bit(565 format) data 

	TFT_Send_Cmd(0x00BC); 
	TFT_Write_Data(0x0040);//contrast value
	TFT_Write_Data(0x0080);//brightness value
	TFT_Write_Data(0x0040);//saturation value
	TFT_Write_Data(0x0001);//Post Processor Enable

	delay_ms(5);

	TFT_Send_Cmd(0x0029); //display on


	TFT_Send_Cmd(0x00BE); //set PWM for B/L
	TFT_Write_Data(0x0006);
	TFT_Write_Data(0x0080);
	TFT_Write_Data(0x0001);
	TFT_Write_Data(0x00f0);
	TFT_Write_Data(0x0000);
	TFT_Write_Data(0x0000);

	TFT_Send_Cmd(0x00d0); 
	TFT_Write_Data(0x000d);
}

//ф-ция устанавливает рабочую область по X
void TFT_Set_X(uint16_t start_x,uint16_t end_x)
{
	TFT_Send_Cmd(0x002A);
	TFT_Write_Data(start_x>>8);
	TFT_Write_Data(start_x&0x00ff);
	
	TFT_Write_Data(end_x>>8);
	TFT_Write_Data(end_x&0x00ff);
}

//ф-ция устанавливает рабочую область по Y
void TFT_Set_Y(uint16_t start_y,uint16_t end_y)
{
	TFT_Send_Cmd(0x002B);
	TFT_Write_Data(start_y>>8);
	TFT_Write_Data(start_y&0x00ff);
	
	TFT_Write_Data(end_y>>8);
	TFT_Write_Data(end_y&0x00ff);
}

//ф-ция задает координаты точки на дисплее
void TFT_Set_XY(uint16_t x, uint16_t y)
{
	TFT_Set_X(x, x);
	TFT_Set_Y(y, y);
}


//ф-ция  задает координаты рабочей области
void TFT_Set_Work_Area(uint16_t x, uint16_t y, uint16_t length, uint16_t width)
{
	TFT_Set_X(x, x+length-1);
	TFT_Set_Y(y, y+width-1);
	TFT_Send_Cmd(0x2C);
}

//ф-ция закрашивает дисплей выбранным цветом
void TFT_Clear_Screen(uint16_t color)
{
	uint32_t i=0;
	TFT_Set_Work_Area(0,0,800,480);
	for(i=0; i < 384000; i++)
	{
		TFT_Write_Data(color);	//передаём кодировку цвета
	}
}


//ф-ция рисует символ нужного размера, цвета, на выбранном фоне, из указанной таблицы(это на случай если хочется использовать разные шрифты но размером 8х8)
void TFT_Draw_Char(uint16_t x, uint16_t y, uint16_t color, uint16_t phone,const uint8_t *table, uint8_t ascii, uint8_t size)
{
	uint8_t i,f = 0;
	
	
	for (i = 0; i < 8; i++)
	{
		for(f = 0; f < 8; f++)
		{
			if((*(table + 8*(ascii-0x20)+i)>>(7-f))&0x01)
			{
				 TFT_Draw_Fill_Rectangle(x+f*size, y+i*size, size, size, color);
			}
			else
			{	
				 TFT_Draw_Fill_Rectangle(x+f*size, y+i*size, size, size, phone);
			}
		}
	}
}

//ф-ция рисует строку, символами из указанной таблицы
void TFT_Draw_String(uint16_t x, uint16_t y, uint16_t color,uint16_t phone, const uint8_t *table, char *string, uint8_t size)
{
   //определить конец строки очень просто если знать, что она ВСЕГДА заканчивается нулём
	while(*string)
	{      
    //проверяем не вылезем ли мы за пределы экрана при отрисовке следующего символа,
    // если да, то переходим на следующую строчку
		if((x + 8) > MAX_X)
		{
			x = 1;
			y = y + 8*size;
		}
		TFT_Draw_Char(x, y, color, phone, table, *string, size);//отрисовываем символ
		x += 8*size;     //изменяем координату для отрисовки следующего символа
		*string++;           //увеличиваем значение указателя, чтобы он ссылался на следующий символ
	}
}

//ф-ция рисует линию заданного цвета и размера
void TFT_Draw_Line (uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2, uint8_t size,uint16_t color)
{
	int deltaX = abs(x2 - x1);
	int deltaY = abs(y2 - y1);
	int signX = x1 < x2 ? 1 : -1;
	int signY = y1 < y2 ? 1 : -1;
	int error = deltaX - deltaY;
	int error2 = 0;
	
	for (;;)
	{
		TFT_Draw_Fill_Rectangle(x1,y1,size,size,color);
		
		if(x1 == x2 && y1 == y2)
		break;
		
		error2 = error * 2;
		
		if(error2 > -deltaY)
		{
			error -= deltaY;
			x1 += signX;
		}
		
		if(error2 < deltaX)
		{
			error += deltaX;
			y1 += signY;
		}
	}
}

//ф-ция рисует горизонтальную линию, указанной длины, толщины и цвета
void TFT_Draw_HLine(uint16_t x, uint16_t y, uint16_t length, uint16_t size, uint16_t color)
{
	uint16_t i=0;
	
	TFT_Set_Work_Area(x,y,length,size);
	for(i=0; i<(length*size); i++)
	TFT_Write_Data(color);
}

//ф-ция рисует вертикальную линию, указанной длины, толщины и цвета
void TFT_Draw_VLine(uint16_t x, uint16_t y, uint16_t length, uint16_t size, uint16_t color)
{
	uint16_t i=0;
	
	TFT_Set_Work_Area(x,y,size,length);
	for(i=0; i<(length*size); i++)
	TFT_Write_Data(color);
}

//ф-ция рисует прямоугольник, указанной длины, ширины, толщины линий и цвета
void TFT_Draw_Rectangle(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint8_t size, uint16_t color)
{
	TFT_Draw_HLine(x, y, length, size, color);
	TFT_Draw_HLine(x, y + width, length, size, color);
	TFT_Draw_VLine(x, y, width, size, color);
	TFT_Draw_VLine(x + length - size, y, width, size, color);
}

//ф-ция рисует закрашенный прямоугольник, указанной длины, ширины, цвета
void TFT_Draw_Fill_Rectangle(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t color)
{
	uint32_t i=0;
	
	TFT_Set_Work_Area(x,y,length, width);
	for(i=0; i < length*width; i++)
	{
		TFT_Write_Data(color);	//передаём кодировку цвета
	}
}

//ф-ция рисует треугольник по точкам с указанной толщиной линий и выбранным цветом
void TFT_Draw_Triangle( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t size, uint16_t color)
{
	
	TFT_Draw_Line( x1, y1, x2, y2, color, size);
	TFT_Draw_Line( x2, y2, x3, y3, color, size);
	TFT_Draw_Line( x3, y3, x1, y1, color, size);
}

//ф-ция рисует окружность нужного радиуса, линией задданой толщины и выбранным цветом, также возможно 
//залить окружность нужным цветом для этого установить аргумент fill равным единице, иначе ноль
void TFT_Draw_Circle(uint16_t x, uint16_t y, uint8_t radius, uint8_t fill, uint8_t size, uint16_t color)
{
	int a_,b_,P;
	a_ = 0;
	b_ = radius;
	P = 1 - radius;
	while (a_ <= b_)
	{
		if(fill == 1)
		{
			TFT_Draw_Fill_Rectangle(x-a_,y-b_,2*a_+1,2*b_+1,color);
			TFT_Draw_Fill_Rectangle(x-b_,y-a_,2*b_+1,2*a_+1,color);
		}
		else
		{
			TFT_Draw_Fill_Rectangle(a_+x, b_+y, size, size, color);
			TFT_Draw_Fill_Rectangle(b_+x, a_+y, size, size, color);
			TFT_Draw_Fill_Rectangle(x-a_, b_+y, size, size, color);
			TFT_Draw_Fill_Rectangle(x-b_, a_+y, size, size, color);
			TFT_Draw_Fill_Rectangle(b_+x, y-a_, size, size, color);
			TFT_Draw_Fill_Rectangle(a_+x, y-b_, size, size, color);
			TFT_Draw_Fill_Rectangle(x-a_, y-b_, size, size, color);
			TFT_Draw_Fill_Rectangle(x-b_, y-a_, size, size, color);
		}
		if (P < 0 )
		{
			P = (P + 3) + (2* a_);
			a_ ++;
		}
		else
		{
			P = (P + 5) + (2* (a_ - b_));
			a_ ++;
			b_ --;
		}
	}
}

//вспомогательная ф-ция для закругления краёв прямоугольника
void TFT_Draw_Circle_Helper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint8_t size, uint16_t color)
{
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      TFT_Draw_Fill_Rectangle(x0 + x, y0 + y, size, size, color);
      TFT_Draw_Fill_Rectangle(x0 + y, y0 + x, size, size, color);
    }
    if (cornername & 0x2) {
      TFT_Draw_Fill_Rectangle(x0 + x, y0 - y, size, size, color);
      TFT_Draw_Fill_Rectangle(x0 + y, y0 - x, size, size, color);
    }
    if (cornername & 0x8) {
      TFT_Draw_Fill_Rectangle(x0 - y, y0 + x, size, size, color);
      TFT_Draw_Fill_Rectangle(x0 - x, y0 + y, size, size, color);
    }
    if (cornername & 0x1) {
      TFT_Draw_Fill_Rectangle(x0 - y, y0 - x, size, size, color);
      TFT_Draw_Fill_Rectangle(x0 - x, y0 - y, size, size, color);
    }
  }
}

//ф-ция рисует прямоугольник заданной длины, ширины, радиусом закругления краёв, толщины линий и выбранным цветом
void TFT_Draw_Round_Rect(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t r, uint8_t size, uint16_t color)
{

  TFT_Draw_HLine(x+r  , y    , length-2*r, size, color); // Top
  TFT_Draw_HLine(x+r  , y+width-1, length-2*r, size, color); // Bottom
  TFT_Draw_VLine(x    , y+r  , width-2*r, size, color); // Left
  TFT_Draw_VLine(x+length-1, y+r  , width-2*r, size, color); // Right

  TFT_Draw_Circle_Helper(x+r    , y+r    , r, 		 			1, size, color);
  TFT_Draw_Circle_Helper(x+length-r-1, y+r    , r, 			2, size, color);
  TFT_Draw_Circle_Helper(x+length-r-1, y+width-r-1, r,  4, size, color);
  TFT_Draw_Circle_Helper(x+r    , y+width-r-1, r, 			8, size, color);
}

//вспомогательная ф-ция для закругления краёв закрашенного прямоугольника
void TFT_Draw_Fill_Circle_Helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color) 
{

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      TFT_Draw_VLine(x0+x, y0-y, 2*y+1+delta, 1, color);
      TFT_Draw_VLine(x0+y, y0-x, 2*x+1+delta, 1, color);
    }
    if (cornername & 0x2) {
     TFT_Draw_VLine(x0-x, y0-y, 2*y+1+delta, 1, color);
     TFT_Draw_VLine(x0-y, y0-x, 2*x+1+delta, 1, color);
    }
  }
}

//ф-ция рисует закрашенный прямоугольник заданной длины, ширины, радиусом закругления краев и выбранным цветом
void TFT_Draw_Fill_Round_Rect(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t r, uint16_t color)
{
  TFT_Draw_Fill_Rectangle(x+r, y, length-2*r, width, color);

  TFT_Draw_Fill_Circle_Helper(x+length-r-1, y+r, r, 1, width-2*r-1, color);
  TFT_Draw_Fill_Circle_Helper(x+r    , y+r, r, 2, width-2*r-1, color);
}


