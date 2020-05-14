#ifndef TM1637_FOR_STM32
#define TM1637_FOR_STM32

// if you will configure pins manually, comment this define
#define TM1637_FOR_STM32_CONFIGURE

void tm1637Init(void);
void tm1637DisplayDecimal(int v, int displaySeparator);
void tm1637SetBrightness(unsigned char brightness);

#endif
