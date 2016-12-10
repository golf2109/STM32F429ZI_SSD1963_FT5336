#ifndef DISPLAY_SSD1963
#define DISPLAY_SSD1963

void Send_WF43_Byte(uint8_t data);
void Write_WF43_C(uint8_t command, uint8_t args);
void Write_WF43_D(uint8_t command);
void SSD1963_init(void);
void FULL_ON(unsigned long dat);
void WindowSet(uint32_t s_x,uint32_t e_x,uint32_t s_y,uint32_t e_y);
void Write_Command(uint8_t command);
void SendData(uint32_t color);
void FillWin(unsigned long dat,unsigned short x, unsigned short y, unsigned short w, unsigned short h);
void ShowTest(void);
void TFT_putuint8_t(unsigned short x, unsigned short y, uint8_t c);
void TFT_printf(unsigned short x, unsigned short y, uint8_t *arg_list, ...);
void SSD1963_lowlevelinit(void);
void SSD1963_reset_init(void);

#endif
