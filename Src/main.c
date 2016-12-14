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
#include "display_ssd1963.h"
#include "touch.h"
#include "GUI.h"
#include "WindowDLG.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t MasterTX[5] = {1,2,3,4,5};
uint8_t SlaveRX[5];
uint8_t str[200];
uint8_t touch_receive[0x32];
uint16_t touch_adr;
uint16_t x_pos[5],y_pos[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);


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
  MX_I2C1_Init();
  
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  
//  HAL_I2C_Slave_Receive(&hi2c3, SlaveRX,SIZE,100);

  HAL_GPIO_WritePin(CTP_WAKE_GPIO_Port, CTP_WAKE_Pin,GPIO_PIN_SET);  
  HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin,GPIO_PIN_RESET);  
  HAL_GPIO_WritePin(Reset_LCD_GPIO_Port, Reset_LCD_Pin,GPIO_PIN_RESET);
  HAL_Delay(500); 
  HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin,GPIO_PIN_SET);  
  HAL_GPIO_WritePin(Reset_LCD_GPIO_Port, Reset_LCD_Pin,GPIO_PIN_SET);
  HAL_Delay(100);  
  __HAL_RCC_CRC_CLK_ENABLE();  
  Init_SSD1963();  
  GUI_Init();

  for(uint32_t index_clr=0;index_clr < 800*480;index_clr++){
    Lcd_Write_Data(BLACK); 	//setbuf color pixel
 	}
  Lcd_ClearScreen(BLUE);
//  for(uint32_t index_set=0;index_set < 100;index_set++){
//    Lcd_SetPixel(index_set, index_set, green);
//  }  
//  Lcd_FillArea(100, 150, 100, 150, white);
//	TFT_Draw_Char(500,100,RED,BLACK,(const uint8_t*) font8x8,'5',3);
//	TFT_Draw_Char(500,200,GREEN,BLACK,(const uint8_t*) font8x8,'b',3);
//	TFT_Draw_Char(200,300,GREEN,BLACK,(const uint8_t*) font8x8,'5',3);  

//	TFT_Draw_String(230, 220, YELLOW, BLUE,(const uint8_t*) font8x8, "SLAVA UKRAINI",4);  

//ф-ция рисует окружность нужного радиуса, линией задданой толщины и выбранным цветом, также возможно 
//залить окружность нужным цветом для этого установить аргумент fill равным единице, иначе ноль
//void TFT_Draw_Circle(uint16_t x, uint16_t y, uint8_t radius, uint8_t fill, uint8_t size, uint16_t color);
//TFT_Draw_Circle(200, 200, 50, 0, 0 , green); 
  /* USER CODE BEGIN WHILE */
HAL_Delay(1000);   
y_pos[0] = 0;
x_pos[0] = 0;
  CreateWindow();  
    GUI_Delay(5000);  
  while (1)
  {

y_pos[0]=800*(256*(0x0F & (touch_receive[3])) + touch_receive[4])/1791;
x_pos[0]=480*(256*(0x0F & (touch_receive[5])) + touch_receive[6])/1024;    
////y_pos[1]=256*(0x0F & (touch_receive[9]))+touch_receive[0xa];
////x_pos[1]=256*(0x0F & (touch_receive[0xb]))+touch_receive[0xc];
////    
////y_pos[2]=256*(0x0F & (touch_receive[0x0d]))+touch_receive[0x0e];
////x_pos[2]=256*(0x0F & (touch_receive[0x0f]))+touch_receive[0x11];    
////    
////y_pos[3]=256*(0x0F & (touch_receive[0x12]))+touch_receive[0x13];
////x_pos[3]=256*(0x0F & (touch_receive[0x14]))+touch_receive[0x15];

////y_pos[4]=256*(0x0F & (touch_receive[0x16]))+touch_receive[0x17];
////x_pos[4]=256*(0x0F & (touch_receive[0x18]))+touch_receive[0x19];   
   if((x_pos[0] > 8 ) && (y_pos[0] >8)){
TFT_Draw_Circle(y_pos[0], x_pos[0], 8, 1, 1 , GREEN);    
  }
   else{
  Lcd_ClearScreen(BLUE);     
   }
////////MasterTX[0]=0x00;
////////MasterTX[1]=0x01;
////////MasterTX[2]=0x02;
////////MasterTX[3]=0x03;    
//////////touch_adr = HAL_I2C_Master_Receive(&hi2c1, 0x6e, touch_receive, 0x03, 1);   
//////////HAL_Delay(1);    
////////touch_adr = HAL_I2C_Master_Transmit(&hi2c1, 0x70, MasterTX, 0x01, 1);   
//////////HAL_Delay(1);   
//////////for(uint8_t ttt=0; ttt<0x1F;ttt++){
////////touch_adr = HAL_I2C_Master_Receive(&hi2c1, 0x71, touch_receive, 0x30, 1);   
////////HAL_Delay(10);      

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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 222;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

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
  Timing.AddressSetupTime = 10;
  Timing.AddressHoldTime = 10;
  Timing.DataSetupTime = 10;
  Timing.BusTurnAroundDuration = 10;
  Timing.CLKDivision = 5;
  Timing.DataLatency = 10;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CTP_WAKE_Pin|CTP_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Reset_LCD_GPIO_Port, Reset_LCD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CTP_WAKE_Pin CTP_RST_Pin */
  GPIO_InitStruct.Pin = CTP_WAKE_Pin|CTP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_INT_Pin */
  GPIO_InitStruct.Pin = CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Reset_LCD_Pin */
  GPIO_InitStruct.Pin = Reset_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Reset_LCD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
