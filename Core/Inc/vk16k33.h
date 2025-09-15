#ifndef __VK16K33_H
#define __VK16K33_H

#include "main.h"

/* VK16K33 I2C地址 (7位地址) */
#define VK16K33_I2C_ADDR        0x70    // 默认地址，可能需要根据硬件调整

/* VK16K33 命令定义 */
#define VK16K33_CMD_OSCILLATOR  0x21    // 开启振荡器
#define VK16K33_CMD_DISPLAY_ON  0x81    // 开启显示，无闪烁
#define VK16K33_CMD_BRIGHTNESS  0xE0    // 亮度控制基地址 (0xE0-0xEF)

/* 数码管段码定义 (共阴极) */
#define DIGIT_0     0x3F    // 0
#define DIGIT_1     0x06    // 1
#define DIGIT_2     0x5B    // 2
#define DIGIT_3     0x4F    // 3
#define DIGIT_4     0x66    // 4
#define DIGIT_5     0x6D    // 5
#define DIGIT_6     0x7D    // 6
#define DIGIT_7     0x07    // 7
#define DIGIT_8     0x7F    // 8
#define DIGIT_9     0x6F    // 9
#define DIGIT_A     0x77    // A
#define DIGIT_B     0x7C    // b
#define DIGIT_C     0x39    // C
#define DIGIT_D     0x5E    // d
#define DIGIT_E     0x79    // E
#define DIGIT_F     0x71    // F
#define DIGIT_OFF   0x00    // 关闭

/* 函数声明 */
uint8_t VK16K33_Init(void);
uint8_t VK16K33_SetBrightness(uint8_t brightness);
uint8_t VK16K33_Clear(void);
uint8_t VK16K33_SetDigit(uint8_t position, uint8_t segment_data);
uint8_t VK16K33_DisplayNumber(uint16_t number);
uint8_t VK16K33_Test_AllDigits(void);
uint8_t VK16K33_Animation_Running(void);
uint8_t VK16K33_Animation_Blink(void);
uint8_t VK16K33_SelfTest(void);

#endif