//=============================================================================
// SSD1963 driver for STM32 microcontrollers
// (c) Radoslaw Kwiecien, radek@dxp.pl	
// http://en.radzio.dxp.pl/ssd1963/
//=============================================================================
#include "stm32f4xx_hal.h"
#include "display.h"

void SSD1963_Init (void);
void SSD1963_SetArea(unsigned int sx, unsigned int ex, unsigned int sy, unsigned int ey);
void SSD1963_FillArea(unsigned int sx, unsigned int ex, unsigned int sy, unsigned int ey, int color);
void SSD1963_ClearScreen(unsigned long color);
void GLCD_SetPixel(int x, int y, int color);

//=============================================================================
// Configuration
//=============================================================================
#define SSD1963_DATAPORT	GPIOE
#define SSD1963_CTRLPORT	GPIOD

#define SSD1963_PIN_RD	(1 << 0)
#define SSD1963_PIN_WR	(1 << 1)
#define SSD1963_PIN_CS	(1 << 2)
#define SSD1963_PIN_A0	(1 << 3)
#define SSD1963_PIN_RST	(1 << 4)

//#define TFT320240
#define TFT800480
//#define TFT640480
//#define TFTCUSTOM

//=============================================================================
// All numeric constants with "ULL" suffix !
//=============================================================================
#define TFT_FPS 60ULL

#ifdef TFTCUSTOM
	#define TFT_WIDTH				0ULL
	#define TFT_HSYNC_BACK_PORCH	0ULL
	#define TFT_HSYNC_FRONT_PORCH	0ULL
	#define TFT_HSYNC_PULSE			0ULL
	
	#define TFT_HEIGHT				0ULL
	#define TFT_VSYNC_BACK_PORCH	0ULL
	#define TFT_VSYNC_FRONT_PORCH	0ULL
	#define TFT_VSYNC_PULSE			0ULL
#endif

#ifdef TFT320240
	#define TFT_WIDTH				320ULL
	#define TFT_HSYNC_BACK_PORCH	40ULL
	#define TFT_HSYNC_FRONT_PORCH	20ULL
	#define TFT_HSYNC_PULSE			28ULL
	
	#define TFT_HEIGHT				240ULL
	#define TFT_VSYNC_BACK_PORCH	17ULL
	#define TFT_VSYNC_FRONT_PORCH	4ULL
	#define TFT_VSYNC_PULSE			3ULL
#endif

#ifdef TFT800480
	#define TFT_WIDTH				800ULL
	#define TFT_HSYNC_BACK_PORCH	30LL
	#define TFT_HSYNC_FRONT_PORCH	0ULL
	#define TFT_HSYNC_PULSE			0ULL
	
	#define TFT_HEIGHT				480ULL
	#define TFT_VSYNC_BACK_PORCH	10ULL
	#define TFT_VSYNC_FRONT_PORCH	0ULL
	#define TFT_VSYNC_PULSE			0ULL
#endif

#ifdef TFT640480

	#define TFT_WIDTH				640ULL
	#define TFT_HSYNC_BACK_PORCH	56ULL
	#define TFT_HSYNC_FRONT_PORCH	8ULL
	#define TFT_HSYNC_PULSE			96ULL

	#define TFT_HEIGHT				480ULL
	#define TFT_VSYNC_BACK_PORCH	41ULL
	#define TFT_VSYNC_FRONT_PORCH	2ULL
	#define TFT_VSYNC_PULSE			2ULL
#endif



#define	TFT_HSYNC_PERIOD	(TFT_HSYNC_PULSE + TFT_HSYNC_BACK_PORCH + TFT_WIDTH  + TFT_HSYNC_FRONT_PORCH)
#define	TFT_VSYNC_PERIOD	(TFT_VSYNC_PULSE + TFT_VSYNC_BACK_PORCH + TFT_HEIGHT + TFT_VSYNC_FRONT_PORCH)

#define TFT_PCLK	(TFT_HSYNC_PERIOD * TFT_VSYNC_PERIOD * TFT_FPS)
//#define TFT_PCLK (928 * 525 * 60)
#define LCD_FPR		((TFT_PCLK * 1048576)/100000000)

#define mHIGH(x) (x >> 8)
#define mLOW(x) (x & 0xFF)
//=============================================================================
// SSD1963 commands
//=============================================================================
#define SSD1963_NOP						0x00
#define SSD1963_SOFT_RESET  			0x01
#define SSD1963_GET_POWER_MODE 			0x0A
#define SSD1963_GET_ADDRESS_MODE		0x0B
#define SSD1963_GET_DISPLAY_MODE		0x0D
#define SSD1963_GET_TEAR_EFFECT_STATUS 	0x0E
#define SSD1963_ENTER_SLEEP_MODE		0x10
#define SSD1963_EXIT_SLEEP_MODE			0x11
#define SSD1963_ENTER_PARTIAL_MODE		0x12
#define SSD1963_ENTER_NORMAL_MODE		0x13
#define SSD1963_EXIT_INVERT_MODE		0x20
#define SSD1963_ENTER_INVERT_MODE		0x21
#define SSD1963_SET_GAMMA_CURVE			0x26
#define SSD1963_SET_DISPLAY_OFF			0x28
#define SSD1963_SET_DISPLAY_ON			0x29
#define SSD1963_SET_COLUMN_ADDRESS		0x2A
#define SSD1963_SET_PAGE_ADDRESS		0x2B
#define SSD1963_WRITE_MEMORY_START		0x2C
#define SSD1963_READ_MEMORY_START		0x2E
#define SSD1963_SET_PARTIAL_AREA		0x30
#define SSD1963_SET_SCROLL_AREA			0x33
#define SSD1963_SET_TEAR_OFF			0x34
#define SSD1963_SET_REAR_ON				0x35
#define SSD1963_SET_ADDRESS_MODE		0x36
#define SSD1963_SET_SCROLL_START		0x37
#define SSD1963_EXIT_IDLE_MODE			0x38
#define SSD1963_ENTER_IDLE_MODE			0x39
#define SSD1963_WRITE_MEMORY_CONTINUE	0x3C
#define SSD1963_READ_MEMORY_CONTINUE	0x3E
#define SSD1963_SET_TEAR_SCANLINE		0x44
#define SSD1963_GET_SCANLINE			0x45
#define SSD1963_READ_DDB				0xA1
#define SSD1963_SET_LCD_MODE			0xB0
#define SSD1963_GET_LCD_MODE			0xB1
#define SSD1963_SET_HORI_PERIOD			0xB4
#define SSD1963_GET_HORI_PERIOD			0xB5
#define SSD1963_SET_VERT_PERIOD			0xB6
#define SSD1963_GET_VERT_PERIOD			0xB7
#define SSD1963_SET_GPIO_CONF			0xB8
#define SSD1963_GET_GPIO_CONF			0xB9
#define SSD1963_SET_GPIO_VALUE			0xBA
#define SSD1963_GET_GPIO_STATUS			0xBB
#define SSD1963_SET_POST_PROC			0xBC
#define SSD1963_GET_POST_PROC			0xBD
#define SSD1963_SET_PWM_CONF			0xBE
#define SSD1963_GET_PWM_CONF			0xBF
#define SSD1963_GET_LCD_GEN0			0xC0
#define SSD1963_SET_LCD_GEN0			0xC1
#define SSD1963_GET_LCD_GEN1			0xC2
#define SSD1963_SET_LCD_GEN1			0xC3
#define SSD1963_GET_LCD_GEN2			0xC4
#define SSD1963_SET_LCD_GEN2			0xC5
#define SSD1963_GET_LCD_GEN3			0xC6
#define SSD1963_SET_LCD_GEN3			0xC7
#define SSD1963_SET_GPIO0_ROP			0xC8
#define SSD1963_GET_GPIO0_ROP			0xC9
#define SSD1963_SET_GPIO1_ROP			0xCA
#define SSD1963_GET_GPIO1_ROP			0xCB
#define SSD1963_SET_GPIO2_ROP			0xCC
#define SSD1963_GET_GPIO2_ROP			0xCD
#define SSD1963_SET_GPIO3_ROP			0xCE
#define SSD1963_GET_GPIO3_ROP			0xCF
#define SSD1963_SET_DBC_CONF			0xD0
#define SSD1963_GET_DBC_CONF			0xD1
#define SSD1963_SET_DBC_TH				0xD4
#define SSD1963_GET_DBC_TH				0xD5
#define SSD1963_SET_PLL					0xE0
#define SSD1963_SET_PLL_MN				0xE2
#define SSD1963_GET_PLL_MN				0xE3
#define SSD1963_GET_PLL_STATUS			0xE4
#define SSD1963_SET_DEEP_SLEEP			0xE5
#define SSD1963_SET_LSHIFT_FREQ			0xE6
#define SSD1963_GET_LSHIFT_FREQ			0xE7
#define SSD1963_SET_PIXEL_DATA_INTERFACE 0xF0
	#define SSD1963_PDI_8BIT			0
	#define SSD1963_PDI_12BIT			1
	#define SSD1963_PDI_16BIT			2
	#define SSD1963_PDI_16BIT565		3
	#define SSD1963_PDI_18BIT			4
	#define SSD1963_PDI_24BIT			5
	#define SSD1963_PDI_9BIT			6
#define SSD1963_GET_PIXEL_DATA_INTERFACE 0xF1
