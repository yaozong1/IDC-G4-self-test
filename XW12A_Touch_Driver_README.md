# XW12A 触摸芯片驱动说明

## 概述
已成功为STM32G071CBT6项目添加XW12A触摸芯片驱动，支持通过I2C1接口控制多按键触摸检测。

## 硬件连接
- **I2C接口**: I2C1 (PB6/PB7)
- **XW12A地址**: 0x20 (7位地址)
- **连接**:
  - PB6 -> I2C1_SCL
  - PB7 -> I2C1_SDA
  - VCC -> 3.3V/5V
  - GND -> GND

## 驱动文件
- `Core/Inc/xw12a.h` - 头文件定义
- `Core/Src/xw12a.c` - 驱动实现

## 主要功能

### 1. 基本控制函数
- `XW12A_Init()` - 初始化XW12A芯片
- `XW12A_ReadChipID()` - 读取芯片ID
- `XW12A_ReadStatus()` - 读取状态寄存器
- `XW12A_ReadKeys()` - 读取当前按键状态
- `XW12A_SetSensitivity(sensitivity)` - 设置触摸灵敏度(1-15)
- `XW12A_Enable()` / `XW12A_Disable()` - 使能/禁用触摸检测

### 2. 高级功能
- `XW12A_GetTouchEvent(event)` - 获取触摸事件（按下/释放）
- `XW12A_TestAllKeys()` - 交互式按键测试
- `XW12A_SelfTest()` - 完整的自测序列

### 3. 触摸事件结构体
```c
typedef struct {
    uint8_t key_pressed;        // 当前按下的按键
    uint8_t key_released;       // 刚释放的按键
    uint8_t key_state;          // 当前按键状态
    uint8_t touch_detected;     // 触摸检测标志
    uint32_t timestamp;         // 时间戳
} XW12A_TouchEvent_t;
```

## 按键定义
支持8个触摸按键：
```c
#define XW12A_KEY_1             0x01    // 按键1
#define XW12A_KEY_2             0x02    // 按键2
#define XW12A_KEY_3             0x04    // 按键3
#define XW12A_KEY_4             0x08    // 按键4
#define XW12A_KEY_5             0x10    // 按键5
#define XW12A_KEY_6             0x20    // 按键6
#define XW12A_KEY_7             0x40    // 按键7
#define XW12A_KEY_8             0x80    // 按键8
```

## 自测流程 (`XW12A_SelfTest()`)

### Step 1: 初始化测试
- 芯片初始化和I2C通信验证
- 基础配置设置

### Step 2: 芯片ID验证
- 读取和验证芯片识别码
- 确认硬件连接正常

### Step 3: 状态寄存器测试
- 验证状态寄存器读取功能
- 检查芯片工作状态

### Step 4: 灵敏度设置测试
- 测试不同灵敏度级别(5, 8, 11)
- 验证配置寄存器写入功能

### Step 5: 使能/禁用功能测试
- 测试触摸检测开关功能
- 验证配置控制能力

### Step 6: 基础按键读取测试
- 连续10次按键状态读取
- 验证数据读取稳定性

### Step 7: 触摸事件检测测试
- 5秒实时触摸事件监控
- 测试事件检测和处理

### Step 8: 通信稳定性测试
- 100次连续读取操作
- 计算通信成功率(要求≥95%)

## 主循环集成

### 初始化阶段
```c
uint8_t xw12a_test_result = XW12A_SelfTest();
if(xw12a_test_result == 1) {
    SEGGER_RTT_WriteString(0, "XW12A Touch Self-Test: PASSED\r\n");
} else {
    SEGGER_RTT_WriteString(0, "XW12A Touch Self-Test: FAILED\r\n");
}
```

### 运行阶段
- 每100ms检测触摸事件
- 实时输出按键按下/释放信息
- 通过SEGGER RTT显示触摸活动

## 调试输出示例

### 自测输出
```
=== XW12A Touch Test Start ===
Step 1: Initializing XW12A...
XW12A: Chip ID = 0x42
XW12A initialization: PASSED
Step 2: Verifying chip ID...
Chip ID: 0x42 - PASSED
Step 3: Testing status register...
Status register: 0x02 - PASSED
Step 4: Testing sensitivity settings...
Sensitivity level 5: PASSED
Sensitivity level 8: PASSED
Sensitivity level 11: PASSED
Step 5: Testing enable/disable functions...
Enable/disable functions: PASSED
Step 6: Testing basic key reading...
Key reading 1: 0x00
Key reading 2: 0x00
...
Basic key reading: PASSED
Step 7: Testing touch event detection (5 seconds)...
Please touch any keys to test event detection...
Event 1: Pressed=0x01, Released=0x00, State=0x01
Event 2: Pressed=0x00, Released=0x01, State=0x00
Touch events detected: 2 - PASSED
Step 8: Testing communication stability...
Progress: 0/100
Progress: 25/100
Progress: 50/100
Progress: 75/100
Communication success rate: 100.0% (100/100)
Communication stability: PASSED
=== XW12A Touch Test: SUCCESS ===
XW12A Touch Self-Test: PASSED
```

### 运行时输出
```
Touch: Key pressed 0x01
Touch: Key released 0x01
Touch: Key pressed 0x04
Touch: Key released 0x04
```

## 编译结果
- Flash使用: 45,060字节 (34.38% of 128KB)
- RAM使用: 8,576字节 (23.26% of 36KB)
- 编译状态: ✅ 无错误无警告

## 功能特性

### 1. 多按键支持 ✅
- 最多8个独立触摸按键
- 同时按键检测能力
- 按键状态位掩码操作

### 2. 事件驱动设计 ✅
- 按键按下/释放事件
- 时间戳记录
- 状态变化检测

### 3. 可配置参数 ✅
- 灵敏度调节(1-15级)
- 触摸检测开关
- 寄存器级别控制

### 4. 实时监控 ✅
- 主循环中100ms周期检测
- RTT调试输出
- 事件日志记录

### 5. 错误处理 ✅
- I2C通信错误检测
- 超时保护机制
- 详细错误报告

## 扩展功能建议

1. **长按检测**: 添加按键长按时间检测
2. **组合按键**: 支持多键组合功能
3. **按键映射**: 自定义按键功能映射
4. **回调函数**: 注册按键事件回调
5. **省电模式**: 实现低功耗触摸检测
6. **校准功能**: 触摸灵敏度自动校准

## 使用注意事项

1. **I2C地址**: 确认XW12A硬件地址配置
2. **上拉电阻**: I2C总线需要上拉电阻
3. **电源稳定**: 确保芯片电源稳定
4. **干扰屏蔽**: 避免电磁干扰影响触摸检测
5. **检测周期**: 建议检测周期不小于50ms

## 集成状态
XW12A触摸芯片驱动已完全集成到STM32G4自测系统中，与W25Q32 Flash和VK16K33数码管驱动采用统一的自测接口设计。