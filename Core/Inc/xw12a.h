#ifndef __XW12A_H
#define __XW12A_H

#include "main.h"

/* XW12A 按键位定义 - 12个按键 */
#define XW12A_KEY_1             0x0001    // 按键1
#define XW12A_KEY_2             0x0002    // 按键2
#define XW12A_KEY_3             0x0004    // 按键3
#define XW12A_KEY_4             0x0008    // 按键4
#define XW12A_KEY_5             0x0010    // 按键5
#define XW12A_KEY_6             0x0020    // 按键6
#define XW12A_KEY_7             0x0040    // 按键7
#define XW12A_KEY_8             0x0080    // 按键8
#define XW12A_KEY_9             0x0100    // 按键9
#define XW12A_KEY_10            0x0200    // 按键10
#define XW12A_KEY_11            0x0400    // 按键11
#define XW12A_KEY_12            0x0800    // 按键12

/* 触摸事件结构体 */
typedef struct {
    uint16_t key_pressed;       // 当前按下的按键 (12位)
    uint16_t key_released;      // 刚释放的按键 (12位)
    uint16_t key_state;         // 当前按键状态 (12位)
    uint8_t touch_detected;     // 触摸检测标志
    uint32_t timestamp;         // 时间戳
} XW12A_TouchEvent_t;

/* 函数声明 */
uint8_t XW12A_Init(void);
uint16_t XW12A_ReadKeys(void);
uint16_t XW12A_ReadRawData(void);  // 调试用原始数据读取
uint8_t XW12A_GetTouchEvent(XW12A_TouchEvent_t *event);
uint8_t XW12A_SelfTest(void);
uint8_t XW12A_DiagnoseHardware(void);

#endif /* __XW12A_H */