#include "stm32f4xx_hal.h"


/**
  ******************************************************************************
  * @file    ft5336.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file provides a set of functions needed to manage the FT5336
  *          touch screen devices.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "touch.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup FT5336
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/** @defgroup FT5336_Private_Types_Definitions
  * @{
  */

/* Private define ------------------------------------------------------------*/

/** @defgroup FT5336_Private_Defines
  * @{
  */

/* Private macro -------------------------------------------------------------*/

/** @defgroup FT5336_Private_Macros
  * @{
  */

/* Private variables ---------------------------------------------------------*/

/** @defgroup FT5336_Private_Variables
  * @{
  */

/* Touch screen driver structure initialization */
TS_DrvTypeDef ft5336_ts_drv =
{
  ft5336_Init,
  ft5336_ReadID,
  ft5336_Reset,

  ft5336_TS_Start,
  ft5336_TS_DetectTouch,
  ft5336_TS_GetXY,

  ft5336_TS_EnableIT,
  ft5336_TS_ClearIT,
  ft5336_TS_ITStatus,
  ft5336_TS_DisableIT

};

/* Global ft5336 handle */
ft5336_handle_TypeDef ft5336_handle = { FT5336_I2C_NOT_INITIALIZED, 0, 0};

/**
  * @}
  */

/** @defgroup ft5336_Private_Function_Prototypes
  * @{
  */

/* Private functions prototypes-----------------------------------------------*/

/**
  * @brief  Return the status of I2C was initialized or not.
  * @param  None.
  * @retval : I2C initialization status.
  */
uint8_t ft5336_Get_I2C_InitializedStatus(void);

/**
  * @brief  I2C initialize if needed.
  * @param  None.
  * @retval : None.
  */
void ft5336_I2C_InitializeIfRequired(void);

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT5336 Device address for communication on I2C Bus.
  * @retval Status FT5336_STATUS_OK or FT5336_STATUS_NOT_OK.
  */
uint32_t ft5336_TS_Configure(uint16_t DeviceAddr);

/** @defgroup ft5336_Private_Functions
  * @{
  */

/** @defgroup ft5336_Public_Function_Body
  * @{
  */

/* Public functions bodies-----------------------------------------------*/




/******************************* I2C Routines *********************************/
/**
  * @brief  Initializes I2C MSP.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
void I2Cx_MspInit(I2C_HandleTypeDef *i2c_handler)
{
  GPIO_InitTypeDef  gpio_init_structure;
  
  if (i2c_handler == (I2C_HandleTypeDef*)(&hI2cAudioHandler))
  {
    /* AUDIO and LCD I2C MSP init */

    /*** Configure the GPIOs ***/
    /* Enable GPIO clock */
    DISCOVERY_AUDIO_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

    /* Configure I2C Tx as alternate function */
    gpio_init_structure.Pin = DISCOVERY_AUDIO_I2Cx_SCL_PIN;
    gpio_init_structure.Mode = GPIO_MODE_AF_OD;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    gpio_init_structure.Alternate = DISCOVERY_AUDIO_I2Cx_SCL_SDA_AF;
    HAL_GPIO_Init(DISCOVERY_AUDIO_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

    /* Configure I2C Rx as alternate function */
    gpio_init_structure.Pin = DISCOVERY_AUDIO_I2Cx_SDA_PIN;
    HAL_GPIO_Init(DISCOVERY_AUDIO_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

    /*** Configure the I2C peripheral ***/
    /* Enable I2C clock */
    DISCOVERY_AUDIO_I2Cx_CLK_ENABLE();

    /* Force the I2C peripheral clock reset */
    DISCOVERY_AUDIO_I2Cx_FORCE_RESET();

    /* Release the I2C peripheral clock reset */
    DISCOVERY_AUDIO_I2Cx_RELEASE_RESET();

    /* Enable and set I2Cx Interrupt to a lower priority */
    HAL_NVIC_SetPriority(DISCOVERY_AUDIO_I2Cx_EV_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_AUDIO_I2Cx_EV_IRQn);

    /* Enable and set I2Cx Interrupt to a lower priority */
    HAL_NVIC_SetPriority(DISCOVERY_AUDIO_I2Cx_ER_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_AUDIO_I2Cx_ER_IRQn);
  }
  else
  {
    /* External, camera and Arduino connector I2C MSP init */

    /*** Configure the GPIOs ***/
    /* Enable GPIO clock */
//////    DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

//////    /* Configure I2C Tx as alternate function */
//////    gpio_init_structure.Pin = DISCOVERY_EXT_I2Cx_SCL_PIN;
//////    gpio_init_structure.Mode = GPIO_MODE_AF_OD;
//////    gpio_init_structure.Pull = GPIO_NOPULL;
//////    gpio_init_structure.Speed = GPIO_SPEED_FAST;
//////    gpio_init_structure.Alternate = DISCOVERY_EXT_I2Cx_SCL_SDA_AF;
//////    HAL_GPIO_Init(DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

//////    /* Configure I2C Rx as alternate function */
//////    gpio_init_structure.Pin = DISCOVERY_EXT_I2Cx_SDA_PIN;
//////    HAL_GPIO_Init(DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

//////    /*** Configure the I2C peripheral ***/
//////    /* Enable I2C clock */
//////    DISCOVERY_EXT_I2Cx_CLK_ENABLE();

//////    /* Force the I2C peripheral clock reset */
//////    DISCOVERY_EXT_I2Cx_FORCE_RESET();

//////    /* Release the I2C peripheral clock reset */
//////    DISCOVERY_EXT_I2Cx_RELEASE_RESET();

//////    /* Enable and set I2Cx Interrupt to a lower priority */
//////    HAL_NVIC_SetPriority(DISCOVERY_EXT_I2Cx_EV_IRQn, 0x0F, 0);
//////    HAL_NVIC_EnableIRQ(DISCOVERY_EXT_I2Cx_EV_IRQn);

//////    /* Enable and set I2Cx Interrupt to a lower priority */
//////    HAL_NVIC_SetPriority(DISCOVERY_EXT_I2Cx_ER_IRQn, 0x0F, 0);
//////    HAL_NVIC_EnableIRQ(DISCOVERY_EXT_I2Cx_ER_IRQn);
  }
}

/**
  * @brief  Initializes I2C HAL.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
void I2Cx_Init(I2C_HandleTypeDef *i2c_handler)
{
  if(HAL_I2C_GetState(i2c_handler) == HAL_I2C_STATE_RESET)
  {
    if (i2c_handler == (I2C_HandleTypeDef*)(&hI2cAudioHandler))
    {
      /* Audio and LCD I2C configuration */
      i2c_handler->Instance = DISCOVERY_AUDIO_I2Cx;
    }
    else
    {
      /* External, camera and Arduino connector  I2C configuration */
//////      i2c_handler->Instance = DISCOVERY_EXT_I2Cx;
    }
//////    i2c_handler->Init.Timing           = DISCOVERY_I2Cx_TIMING;
    i2c_handler->Init.OwnAddress1      = 0;
    i2c_handler->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    i2c_handler->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    i2c_handler->Init.OwnAddress2      = 0;
    i2c_handler->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    i2c_handler->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
    I2Cx_MspInit(i2c_handler);
    HAL_I2C_Init(i2c_handler);
  }
}

void I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr)
{
  /* De-initialize the I2C communication bus */
  HAL_I2C_DeInit(i2c_handler);
  
  /* Re-Initialize the I2C communication bus */
  I2Cx_Init(i2c_handler);
}

/**
  * @brief  Reads multiple data.
  * @param  i2c_handler : I2C handler
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  MemAddress: Memory address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *i2c_handler,
                                           uint8_t Addr,
                                           uint16_t Reg,
                                           uint16_t MemAddress,
                                           uint8_t *Buffer,
                                           uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* I2C error occurred */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;    
}

/**
  * @brief  Writes a value in a register of the device through BUS in using DMA mode.
  * @param  i2c_handler : I2C handler
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  MemAddress: Memory address 
  * @param  Buffer: The target register value to be written 
  * @param  Length: buffer size to be written
  * @retval HAL status
  */
HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef *i2c_handler,
                                            uint8_t Addr,
                                            uint16_t Reg,
                                            uint16_t MemAddress,
                                            uint8_t *Buffer,
                                            uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  i2c_handler : I2C handler
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
HAL_StatusTypeDef I2Cx_IsDeviceReady(I2C_HandleTypeDef *i2c_handler, uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(i2c_handler, DevAddress, Trials, 1000));
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  i2c_handler : I2C handler
  * @param  Addr: I2C Address
  * @retval None
  */


/********************************* LINK TOUCHSCREEN *********************************/

/**
  * @brief  Initializes Touchscreen low level.
  * @retval None
  */
void TS_IO_Init(void)
{
  I2Cx_Init(&hI2cAudioHandler);
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteMultiple(&hI2cAudioHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Value, 1);
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  I2Cx_ReadMultiple(&hI2cAudioHandler, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1);

  return read_value;
}

/**
  * @brief  TS delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void TS_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/**
  * @brief  Initialize the ft5336 communication bus
  *         from MCU to FT5336 : ie I2C channel initialization (if required).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @retval None
  */
void ft5336_Init(uint16_t DeviceAddr)
{
  /* Wait at least 200ms after power up before accessing registers
   * Trsi timing (Time of starting to report point after resetting) from FT5336GQQ datasheet */
  TS_IO_Delay(200);

  /* Initialize I2C link if needed */
  ft5336_I2C_InitializeIfRequired();
}

/**
  * @brief  Software Reset the ft5336.
  *         @note : Not applicable to FT5336.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @retval None
  */
void ft5336_Reset(uint16_t DeviceAddr)
{
  /* Do nothing */
  /* No software reset sequence available in FT5336 IC */
}

/**
  * @brief  Read the ft5336 device ID, pre initialize I2C in case of need to be
  *         able to read the FT5336 device ID, and verify this is a FT5336.
  * @param  DeviceAddr: I2C FT5336 Slave address.
  * @retval The Device ID (two bytes).
  */
uint16_t ft5336_ReadID(uint16_t DeviceAddr)
{
  volatile uint8_t ucReadId = 0;
  uint8_t nbReadAttempts = 0;
  uint8_t bFoundDevice = 0; /* Device not found by default */

  /* Initialize I2C link if needed */
  ft5336_I2C_InitializeIfRequired();

  /* At maximum 4 attempts to read ID : exit at first finding of the searched device ID */
  for(nbReadAttempts = 0; ((nbReadAttempts < 3) && !(bFoundDevice)); nbReadAttempts++)
  {
    /* Read register FT5336_CHIP_ID_REG as DeviceID detection */
    ucReadId = TS_IO_Read(DeviceAddr, FT5336_CHIP_ID_REG);

    /* Found the searched device ID ? */
    if(ucReadId == FT5336_ID_VALUE)
    {
      /* Set device as found */
      bFoundDevice = 1;
    }
  }

  /* Return the device ID value */
  return (ucReadId);
}

/**
  * @brief  Configures the touch Screen IC device to start detecting touches
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
  * @retval None.
  */
void ft5336_TS_Start(uint16_t DeviceAddr)
{
  /* Minimum static configuration of FT5336 */
  FT5336_ASSERT(ft5336_TS_Configure(DeviceAddr));

  /* By default set FT5336 IC in Polling mode : no INT generation on FT5336 for new touch available */
  /* Note TS_INT is active low                                                                      */
  ft5336_TS_DisableIT(DeviceAddr);
}

/**
  * @brief  Return if there is touches detected or not.
  *         Try to detect new touches and forget the old ones (reset internal global
  *         variables).
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval : Number of active touches detected (can be 0, 1 or 2).
  */
uint8_t ft5336_TS_DetectTouch(uint16_t DeviceAddr)
{
  volatile uint8_t nbTouch = 0;

  /* Read register FT5336_TD_STAT_REG to check number of touches detection */
  nbTouch = TS_IO_Read(DeviceAddr, FT5336_TD_STAT_REG);
  nbTouch &= FT5336_TD_STAT_MASK;

  if(nbTouch > FT5336_MAX_DETECTABLE_TOUCH)
  {
    /* If invalid number of touch detected, set it to zero */
    nbTouch = 0;
  }

  /* Update ft5336 driver internal global : current number of active touches */
  ft5336_handle.currActiveTouchNb = nbTouch;

  /* Reset current active touch index on which to work on */
  ft5336_handle.currActiveTouchIdx = 0;

  return(nbTouch);
}

/**
  * @brief  Get the touch screen X and Y positions values
  *         Manage multi touch thanks to touch Index global
  *         variable 'ft5336_handle.currActiveTouchIdx'.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
void ft5336_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
  volatile uint8_t ucReadData = 0;
  static uint16_t coord;
  uint8_t regAddressXLow = 0;
  uint8_t regAddressXHigh = 0;
  uint8_t regAddressYLow = 0;
  uint8_t regAddressYHigh = 0;

  if(ft5336_handle.currActiveTouchIdx < ft5336_handle.currActiveTouchNb)
  {
    switch(ft5336_handle.currActiveTouchIdx)
    {
    case 0 :
      regAddressXLow  = FT5336_P1_XL_REG;
      regAddressXHigh = FT5336_P1_XH_REG;
      regAddressYLow  = FT5336_P1_YL_REG;
      regAddressYHigh = FT5336_P1_YH_REG;
      break;

    case 1 :
      regAddressXLow  = FT5336_P2_XL_REG;
      regAddressXHigh = FT5336_P2_XH_REG;
      regAddressYLow  = FT5336_P2_YL_REG;
      regAddressYHigh = FT5336_P2_YH_REG;
      break;

    case 2 :
      regAddressXLow  = FT5336_P3_XL_REG;
      regAddressXHigh = FT5336_P3_XH_REG;
      regAddressYLow  = FT5336_P3_YL_REG;
      regAddressYHigh = FT5336_P3_YH_REG;
      break;

    case 3 :
      regAddressXLow  = FT5336_P4_XL_REG;
      regAddressXHigh = FT5336_P4_XH_REG;
      regAddressYLow  = FT5336_P4_YL_REG;
      regAddressYHigh = FT5336_P4_YH_REG;
      break;

    case 4 :
      regAddressXLow  = FT5336_P5_XL_REG;
      regAddressXHigh = FT5336_P5_XH_REG;
      regAddressYLow  = FT5336_P5_YL_REG;
      regAddressYHigh = FT5336_P5_YH_REG;
      break;

    case 5 :
      regAddressXLow  = FT5336_P6_XL_REG;
      regAddressXHigh = FT5336_P6_XH_REG;
      regAddressYLow  = FT5336_P6_YL_REG;
      regAddressYHigh = FT5336_P6_YH_REG;
      break;

    case 6 :
      regAddressXLow  = FT5336_P7_XL_REG;
      regAddressXHigh = FT5336_P7_XH_REG;
      regAddressYLow  = FT5336_P7_YL_REG;
      regAddressYHigh = FT5336_P7_YH_REG;
      break;

    case 7 :
      regAddressXLow  = FT5336_P8_XL_REG;
      regAddressXHigh = FT5336_P8_XH_REG;
      regAddressYLow  = FT5336_P8_YL_REG;
      regAddressYHigh = FT5336_P8_YH_REG;
      break;

    case 8 :
      regAddressXLow  = FT5336_P9_XL_REG;
      regAddressXHigh = FT5336_P9_XH_REG;
      regAddressYLow  = FT5336_P9_YL_REG;
      regAddressYHigh = FT5336_P9_YH_REG;
      break;

    case 9 :
      regAddressXLow  = FT5336_P10_XL_REG;
      regAddressXHigh = FT5336_P10_XH_REG;
      regAddressYLow  = FT5336_P10_YL_REG;
      regAddressYHigh = FT5336_P10_YH_REG;
      break;

    default :
      break;

    } /* end switch(ft5336_handle.currActiveTouchIdx) */

    /* Read low part of X position */
    ucReadData = TS_IO_Read(DeviceAddr, regAddressXLow);
    coord = (ucReadData & FT5336_TOUCH_POS_LSB_MASK) >> FT5336_TOUCH_POS_LSB_SHIFT;

    /* Read high part of X position */
    ucReadData = TS_IO_Read(DeviceAddr, regAddressXHigh);
    coord |= ((ucReadData & FT5336_TOUCH_POS_MSB_MASK) >> FT5336_TOUCH_POS_MSB_SHIFT) << 8;

    /* Send back ready X position to caller */
    *X = coord;

    /* Read low part of Y position */
    ucReadData = TS_IO_Read(DeviceAddr, regAddressYLow);
    coord = (ucReadData & FT5336_TOUCH_POS_LSB_MASK) >> FT5336_TOUCH_POS_LSB_SHIFT;

    /* Read high part of Y position */
    ucReadData = TS_IO_Read(DeviceAddr, regAddressYHigh);
    coord |= ((ucReadData & FT5336_TOUCH_POS_MSB_MASK) >> FT5336_TOUCH_POS_MSB_SHIFT) << 8;

    /* Send back ready Y position to caller */
    *Y = coord;

    ft5336_handle.currActiveTouchIdx++; /* next call will work on next touch */

  } /* of if(ft5336_handle.currActiveTouchIdx < ft5336_handle.currActiveTouchNb) */
}

/**
  * @brief  Configure the FT5336 device to generate IT on given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5336).
  * @retval None
  */
void ft5336_TS_EnableIT(uint16_t DeviceAddr)
{
   uint8_t regValue = 0;
   regValue = (FT5336_G_MODE_INTERRUPT_TRIGGER & (FT5336_G_MODE_INTERRUPT_MASK >> FT5336_G_MODE_INTERRUPT_SHIFT)) << FT5336_G_MODE_INTERRUPT_SHIFT;

   /* Set interrupt trigger mode in FT5336_GMODE_REG */
   TS_IO_Write(DeviceAddr, FT5336_GMODE_REG, regValue);
}

/**
  * @brief  Configure the FT5336 device to stop generating IT on the given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5336).
  * @retval None
  */
void ft5336_TS_DisableIT(uint16_t DeviceAddr)
{
  uint8_t regValue = 0;
  regValue = (FT5336_G_MODE_INTERRUPT_POLLING & (FT5336_G_MODE_INTERRUPT_MASK >> FT5336_G_MODE_INTERRUPT_SHIFT)) << FT5336_G_MODE_INTERRUPT_SHIFT;

  /* Set interrupt polling mode in FT5336_GMODE_REG */
  TS_IO_Write(DeviceAddr, FT5336_GMODE_REG, regValue);
}

/**
  * @brief  Get IT status from FT5336 interrupt status registers
  *         Should be called Following an EXTI coming to the MCU to know the detailed
  *         reason of the interrupt.
  *         @note : This feature is not applicable to FT5336.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @retval TS interrupts status : always return 0 here
  */
uint8_t ft5336_TS_ITStatus(uint16_t DeviceAddr)
{
  /* Always return 0 as feature not applicable to FT5336 */
  return 0;
}

/**
  * @brief  Clear IT status in FT5336 interrupt status clear registers
  *         Should be called Following an EXTI coming to the MCU.
  *         @note : This feature is not applicable to FT5336.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @retval None
  */
void ft5336_TS_ClearIT(uint16_t DeviceAddr)
{
  /* Nothing to be done here for FT5336 */
}

/**** NEW FEATURES enabled when Multi-touch support is enabled ****/

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

/**
  * @brief  Get the last touch gesture identification (zoom, move up/down...).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @param  pGestureId : Pointer to get last touch gesture Identification.
  * @retval None.
  */
void ft5336_TS_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId)
{
  volatile uint8_t ucReadData = 0;

  ucReadData = TS_IO_Read(DeviceAddr, FT5336_GEST_ID_REG);

  * pGestureId = ucReadData;
}

/**
  * @brief  Get the touch detailed informations on touch number 'touchIdx' (0..1)
  *         This touch detailed information contains :
  *         - weight that was applied to this touch
  *         - sub-area of the touch in the touch panel
  *         - event of linked to the touch (press down, lift up, ...)
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5336).
  * @param  touchIdx : Passed index of the touch (0..1) on which we want to get the
  *                    detailed information.
  * @param  pWeight : Pointer to to get the weight information of 'touchIdx'.
  * @param  pArea   : Pointer to to get the sub-area information of 'touchIdx'.
  * @param  pEvent  : Pointer to to get the event information of 'touchIdx'.

  * @retval None.
  */
void ft5336_TS_GetTouchInfo(uint16_t   DeviceAddr,
                            uint32_t   touchIdx,
                            uint32_t * pWeight,
                            uint32_t * pArea,
                            uint32_t * pEvent)
{
  volatile uint8_t ucReadData = 0;
  uint8_t regAddressXHigh = 0;
  uint8_t regAddressPWeight = 0;
  uint8_t regAddressPMisc = 0;

  if(touchIdx < ft5336_handle.currActiveTouchNb)
  {
    switch(touchIdx)
    {
    case 0 :
      regAddressXHigh   = FT5336_P1_XH_REG;
      regAddressPWeight = FT5336_P1_WEIGHT_REG;
      regAddressPMisc   = FT5336_P1_MISC_REG;
      break;

    case 1 :
      regAddressXHigh   = FT5336_P2_XH_REG;
      regAddressPWeight = FT5336_P2_WEIGHT_REG;
      regAddressPMisc   = FT5336_P2_MISC_REG;
      break;

    case 2 :
      regAddressXHigh   = FT5336_P3_XH_REG;
      regAddressPWeight = FT5336_P3_WEIGHT_REG;
      regAddressPMisc   = FT5336_P3_MISC_REG;
      break;

    case 3 :
      regAddressXHigh   = FT5336_P4_XH_REG;
      regAddressPWeight = FT5336_P4_WEIGHT_REG;
      regAddressPMisc   = FT5336_P4_MISC_REG;
      break;

    case 4 :
      regAddressXHigh   = FT5336_P5_XH_REG;
      regAddressPWeight = FT5336_P5_WEIGHT_REG;
      regAddressPMisc   = FT5336_P5_MISC_REG;
      break;

    case 5 :
      regAddressXHigh   = FT5336_P6_XH_REG;
      regAddressPWeight = FT5336_P6_WEIGHT_REG;
      regAddressPMisc   = FT5336_P6_MISC_REG;
      break;

    case 6 :
      regAddressXHigh   = FT5336_P7_XH_REG;
      regAddressPWeight = FT5336_P7_WEIGHT_REG;
      regAddressPMisc   = FT5336_P7_MISC_REG;
      break;

    case 7 :
      regAddressXHigh   = FT5336_P8_XH_REG;
      regAddressPWeight = FT5336_P8_WEIGHT_REG;
      regAddressPMisc   = FT5336_P8_MISC_REG;
      break;

    case 8 :
      regAddressXHigh   = FT5336_P9_XH_REG;
      regAddressPWeight = FT5336_P9_WEIGHT_REG;
      regAddressPMisc   = FT5336_P9_MISC_REG;
      break;

    case 9 :
      regAddressXHigh   = FT5336_P10_XH_REG;
      regAddressPWeight = FT5336_P10_WEIGHT_REG;
      regAddressPMisc   = FT5336_P10_MISC_REG;
      break;

    default :
      break;

    } /* end switch(touchIdx) */

    /* Read Event Id of touch index */
    ucReadData = TS_IO_Read(DeviceAddr, regAddressXHigh);
    * pEvent = (ucReadData & FT5336_TOUCH_EVT_FLAG_MASK) >> FT5336_TOUCH_EVT_FLAG_SHIFT;

    /* Read weight of touch index */
    ucReadData = TS_IO_Read(DeviceAddr, regAddressPWeight);
    * pWeight = (ucReadData & FT5336_TOUCH_WEIGHT_MASK) >> FT5336_TOUCH_WEIGHT_SHIFT;

    /* Read area of touch index */
    ucReadData = TS_IO_Read(DeviceAddr, regAddressPMisc);
    * pArea = (ucReadData & FT5336_TOUCH_AREA_MASK) >> FT5336_TOUCH_AREA_SHIFT;

  } /* of if(touchIdx < ft5336_handle.currActiveTouchNb) */
}

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

/** @defgroup ft5336_Static_Function_Body
  * @{
  */

/* Static functions bodies-----------------------------------------------*/


/**
  * @brief  Return the status of I2C was initialized or not.
  * @param  None.
  * @retval : I2C initialization status.
  */
uint8_t ft5336_Get_I2C_InitializedStatus(void)
{
  return(ft5336_handle.i2cInitialized);
}

/**
  * @brief  I2C initialize if needed.
  * @param  None.
  * @retval : None.
  */
void ft5336_I2C_InitializeIfRequired(void)
{
  if(ft5336_Get_I2C_InitializedStatus() == FT5336_I2C_NOT_INITIALIZED)
  {
    /* Initialize TS IO BUS layer (I2C) */
    TS_IO_Init();

    /* Set state to initialized */
    ft5336_handle.i2cInitialized = FT5336_I2C_INITIALIZED;
  }
}

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT5336 Device address for communication on I2C Bus.
  * @retval Status FT5336_STATUS_OK or FT5336_STATUS_NOT_OK.
  */
uint32_t ft5336_TS_Configure(uint16_t DeviceAddr)
{
  uint32_t status = FT5336_STATUS_OK;

  /* Nothing special to be done for FT5336 */

  return(status);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
