#include "stm32f1xx_hal.h"
// #include "stm32l1xx_hal.h"

#include "stm32_tm1637.h"

//===================================================
// -------   CONFIG   -------------------------------
//---------------------------------------------------
#define CLK_PORT GPIOB
#define DIO_PORT GPIOB
#define CLK_PIN GPIO_PIN_11
#define DIO_PIN GPIO_PIN_10
#define CLK_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
#define DIO_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
//====================================================

// private functions
void _tm1637Start(void);
void _tm1637Stop(void);
void _tm1637ReadResult(void);
void _tm1637WriteByte(unsigned char b);
void _tm1637DelayUsec(uint16_t mksec);
void _tm1637ClkHigh(void);
void _tm1637ClkLow(void);
void _tm1637DioHigh(void);
void _tm1637DioLow(void);
//-------------------------------------------

const char segmentMap[] = {
    0x3f, // 0
    0x06, // 1
    0x5b, // 2
    0x4f, // 3
    0x66, // 4
    0x6d, // 5
    0x7d, // 6
    0x07, // 7
    0x7f, // 8
    0x6f, // 9
    0x77, // A
    0x7c, // b
    0x39, // c
    0x5e, // d
    0x79, // E
    0x71, // F
    0x00  // clear
};


void tm1637Init(void)
{

#ifdef TM1637_FOR_STM32_CONFIGURE
    CLK_PORT_CLK_ENABLE();
    DIO_PORT_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pull = GPIO_PULLUP;
    g.Mode = GPIO_MODE_OUTPUT_OD; // OD = open drain
//    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;      // for stm32l1xx_hal.h
    g.Speed = GPIO_SPEED_FREQ_HIGH;             // for stm32f1xx_hal.h
    g.Pin = CLK_PIN;
    HAL_GPIO_Init(CLK_PORT, &g);
    g.Pin = DIO_PIN;
    HAL_GPIO_Init(DIO_PORT, &g);
#endif

    tm1637SetBrightness(8); // default
}

void tm1637DisplayDecimal(int num, int displaySeparator)
{
    unsigned char digitArr[4];
    for (int i = 0; i < 4; ++i) {
        digitArr[i] = segmentMap[num % 10];
        if (i == 2 && displaySeparator) {
            digitArr[i] |= 1 << 7;
        }
        num /= 10;
    }

    _tm1637Start();
    _tm1637WriteByte(0x40);
    _tm1637ReadResult();
    _tm1637Stop();

    _tm1637Start();
    _tm1637WriteByte(0xc0);
    _tm1637ReadResult();

    for (int i = 0; i < 4; ++i) {
        _tm1637WriteByte(digitArr[3 - i]);
        _tm1637ReadResult();
    }

    _tm1637Stop();
}

// Valid brightness values: 0 - 8.
// 0 = display off.
void tm1637SetBrightness(unsigned char brightness)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness
    _tm1637Start();
    _tm1637WriteByte((uint8_t)(brightness + 0x87));
    _tm1637ReadResult();
    _tm1637Stop();
}

void _tm1637Start(void)
{
    _tm1637ClkHigh();
    _tm1637DioHigh();
    _tm1637DelayUsec(2);
    _tm1637DioLow();
}

void _tm1637Stop(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(2);
    _tm1637DioLow();
    _tm1637DelayUsec(2);
    _tm1637ClkHigh();
    _tm1637DelayUsec(2);
    _tm1637DioHigh();
}

void _tm1637ReadResult(void)
{
    _tm1637ClkLow();
    _tm1637DelayUsec(5);
    // while (dio); // We're cheating here and not actually reading back the response.
    _tm1637ClkHigh();
    _tm1637DelayUsec(2);
    _tm1637ClkLow();
}

void _tm1637WriteByte(unsigned char b)
{
    for (int i = 0; i < 8; ++i) {
        _tm1637ClkLow();
        if (b & 0x01) {
            _tm1637DioHigh();
        }
        else {
            _tm1637DioLow();
        }
        _tm1637DelayUsec(3);
        b >>= 1;
        _tm1637ClkHigh();
        _tm1637DelayUsec(3);
    }
}

//void _tm1637DelayUsec(unsigned int i)
//{
//    for (; i>0; i--) {
//        for (int j = 0; j < 10; ++j) {
//            __asm__ __volatile__("nop\n\t":::"memory");
//        }
//    }
//}

void _tm1637DelayUsec(uint16_t sometime)
{
	for (uint32_t j = 0; j < 40UL * sometime; ++j) {
		__NOP();
	}
}

void _tm1637ClkHigh(void)
{
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
}

void _tm1637ClkLow(void)
{
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
}

void _tm1637DioHigh(void)
{
    HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_SET);
}

void _tm1637DioLow(void)
{
    HAL_GPIO_WritePin(DIO_PORT, DIO_PIN, GPIO_PIN_RESET);
}
