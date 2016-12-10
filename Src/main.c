/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
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

////typedef struct
////{
////  volatile uint16_t LCD_REG;
////  volatile uint16_t LCD_RAM;
////} LCD_TypeDef;

//////I have connected FSMC_A17 to the SSD1963 RS, and FSMC_NE1 to SSD1963 CS.
//////After reading up a bit, I calculated the FSMC address as:
//////#define LCD_BASE           ((uint32_t)(0x60000000 | ((0UL << 27) | (0UL << 26))) | (1UL << 17))

//////// Note: LCD /CS is NE1 - Bank 1 of NOR/SRAM Bank 1~4
//////#define LCD_BASE           ((uint32_t)(0x60000000 | 0x0001fffE))
//#define LCD                ((LCD_TypeDef *) LCD_BASE)


//////Мы помним, что с точки зрения кода, обращение к FSMC будет простой записью/чтением из памяти, 
//////поэтому нам нужно определить по каким именно адресам обращаться. Смотрим в референс мануал на STM32,
//////раздел FSMC, и видим, что для NOR/SRAM выделены адреса, начинающиеся с 0x60000000.
//////Под банками в широком смысле в мануале подразумеваются большие регионы, выделенные для устройств разного типа, 
//////так, банк #1 – это NOR/SRAM, банки #2 и #3 – NAND, банк #4- PC Card.
//////В свою очередь банк #1 может быть использован для доступа к целым 4 чипам памяти, 
//////каждый из которых может независимо от других быть NOR либо SRAM. Так как дисплей 
//////подключен как NE1, нас интересует банк, объявленный как FSMC_Bank1_NORSRAM1. 
//////Исходя из базового адреса, можно сразу же записать определение 
//////#define LCDMemory		(*((volatile uint16_t*) 0x60020000))

////     #define LCD_BASE        (0x60000000UL | 0x0C000000UL)
////     #define LCD_REG16  (*((volatile U16 *)(LCD_BASE  ))) 
////     #define LCD_DAT16  (*((volatile U16 *)(LCD_BASE+2)))


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


//extern const unsigned char simbols [];
void Lcd_Write_Index(uint16_t index);
void Lcd_Write_Data(uint16_t data);
void Lcd_Write_Reg(uint16_t lcd_reg, uint16_t lcd_data);
uint16_t Lcd_Read_Reg(uint16_t reg_addr);

uint16_t read_reg[10];
uint16_t lcd_id;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//////// Запись в регистр LCD
//////void FSMC_LcdWriteReg ( uint8_t LCD_Reg, uint16_t LCD_RegValue )
//////{
//////	// Write 16-bit Index, then Write Reg
//////	LCD->LCD_REG = LCD_Reg;
//////	// Write 16-bit Reg
//////	LCD->LCD_RAM = LCD_RegValue;
//////} // FSMC_LcdWriteReg

//////// Чтение регистра LCD
//////uint16_t FSMC_LcdReadReg ( uint8_t LCD_Reg )
//////{
//////	// Write 16-bit Index (then Read Reg)
//////	LCD->LCD_REG = LCD_Reg;
//////	// Read 16-bit Reg
//////	return (LCD->LCD_RAM);
//////} // FSMC_LcdReadReg

//////// Запись команды в LCD
//////void FSMC_LcdWriteCmd ( uint16_t val )
//////{
//////	// Write 16-bit Index (then Read Reg)
//////	LCD->LCD_REG = val;
//////} // FSMC_LcdWriteCmd

//////// чтение команды из LCD
//////uint16_t FSMC_LcdReadCmd ( void )
//////{
//////	return (LCD->LCD_REG);
//////} // FSMC_LcdWriteCmd

//////// чтение данных из LCD
//////uint16_t FSMC_LcdReadData ( void )
//////{
//////	return (LCD->LCD_RAM);
//////} // FSMC_LcdWriteCmd

//////// Запись данных в LCD
//////void FSMC_LcdWriteData ( uint16_t val )
//////{
//////	// Write 16-bit Reg
//////	LCD->LCD_RAM = val;
//////} // FSMC_LcdWriteData


__inline void Lcd_Write_Index(uint16_t index)
{
   *(uint16_t *) (LCD_REG) = index;
}
////////////////////////
__inline void Lcd_Write_Data(uint16_t data)
{
   *(uint16_t *) (LCD_DATA)= data;
}
///////////////////
uint16_t Lcd_Read_Data()
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
//ф-циЯ выбирает Ячейку в видеоЋ‡“
void Set_Cursor(uint16_t x_kur, uint16_t y_kur)
{	
	Lcd_Write_Reg(0x004e,x_kur);
	Lcd_Write_Reg(0x004f,y_kur);
	Lcd_Write_Index(0x0022);

}
////////////////////////
//ф-циЯ закрашивает экран выбранным цветом
void Lcd_Clear(uint16_t color)
{
	uint32_t index = 0;
	
	Set_Cursor(0,0);	
	
	  for(index=0;index < 76800;index++)
	  { 			
		  Lcd_Write_Data(color);
	  }
}
/////////////////////////////////////
//ф-циЯ рисует id дисплеЯ заданным цветом на выбранном фоне, в указанную позицию
/*void Get_Lcd_Id(uint16_t x, uint16_t y,uint16_t color, uint16_t phone, uint8_t size)
{
	 uint16_t data = 0;
	 uint8_t id [4] = {0};
	
	data = Lcd_Read_Reg(0x00);
	itoa( data, (char*) id, 16); //преобразуем число в строку длЯ вывода на дисплей
	for (uint8_t k = 0; k < 4; k++)
	{
		Draw_Simbol(x ,y + 8*k*size,color,phone, &simbols[(id[k]-0x20)*8],size);
	}
}*/
///////////////////////////////////


void Initial_SSD1963()
{
Lcd_Write_Index(0x01);
HAL_Delay(10);
Lcd_Write_Index(0xe0); //START PLL
Lcd_Write_Data(0x01);
HAL_Delay(50);
Lcd_Write_Index(0xe0); //START PLL
Lcd_Write_Data(0x03);
HAL_Delay(5);
Lcd_Write_Index(0xb0);	//LCD SPECIFICATION
Lcd_Write_Data(0x20);
Lcd_Write_Data(0x80);
Lcd_Write_Data(0x03);
Lcd_Write_Data(0x1f);
Lcd_Write_Data(0x01);
Lcd_Write_Data(0xdf);
Lcd_Write_Data(0x00);
Lcd_Write_Index(0xf0);
Lcd_Write_Data(0x03); //pixel data format, 0x03 is 16bit(565 format);0x00 is for 8-bit
//Set the MN of PLL
Lcd_Write_Index(0xe2);
Lcd_Write_Data(0x1d);
Lcd_Write_Data(0x02);
Lcd_Write_Data(0x54);
Lcd_Write_Index(0xe6);
Lcd_Write_Data(0x04);
Lcd_Write_Data(0x6f);
Lcd_Write_Data(0x47);
//Set front porch and back porch
Lcd_Write_Index(0xb4);
Lcd_Write_Data(0x04);
Lcd_Write_Data(0x20);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x2e);
Lcd_Write_Data(0xd2);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x00);
Lcd_Write_Index(0xb6);

Lcd_Write_Data(0x02);
Lcd_Write_Data(0x0d);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x17);
Lcd_Write_Data(0x16);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x00);
Lcd_Write_Index(0x2a);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x03);
Lcd_Write_Data(0x1f);
Lcd_Write_Index(0x2b);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x00);
Lcd_Write_Data(0x01);
Lcd_Write_Data(0x1f);
Lcd_Write_Index(0xb8);
Lcd_Write_Data(0x0f);
Lcd_Write_Data(0x01);
Lcd_Write_Index(0xba);
Lcd_Write_Data(0x01);
Lcd_Write_Index(0x29);
Lcd_Write_Index(0x2c);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_FMC_Init();

  /* USER CODE BEGIN 2 */
Initial_SSD1963();  
  
//read_reg[0] = Lcd_Read_Reg(0xb1);    
//read_reg[1] = Lcd_Read_Data();   
//read_reg[2] = Lcd_Read_Data();
//read_reg[3] = Lcd_Read_Data();
//read_reg[4] = Lcd_Read_Data();
//read_reg[5] = Lcd_Read_Data();
//read_reg[6] = Lcd_Read_Data();    
//read_reg[7] = Lcd_Read_Data();      
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//Initial_SSD1963();  
//for(uint16_t i=0; i <= 0xfffc; i++){
//Lcd_Clear(i);
  
	uint32_t i;
//	Lcd_Write_Index(0x002C);
	for (i = 0; i < (800 * 480); i++)
		{
			Lcd_Write_Data(0x001f);
      
		}  
 HAL_Delay(100);  
////	for (i = 0; i < (800 * 480); i++)
////		{
////			Lcd_Write_Data(0xf81f);
////		}   
////HAL_Delay(100); 
}    
   
//  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Reset_LCD_GPIO_Port, Reset_LCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Reset_LCD_Pin */
  GPIO_InitStruct.Pin = Reset_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Reset_LCD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
