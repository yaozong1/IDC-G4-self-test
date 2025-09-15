# VK16K33 数码管驱动说明

## 概述
已成功为STM32G071CBT6项目添加VK16K33数码管驱动，支持通过I2C接口控制7段数码管显示。

## 硬件连接
- **I2C接口**: I2C2 (PA11/PA12)
- **VK16K33地址**: 0x70 (7位地址)
- **连接**:
  - PA11 -> I2C2_SCL
  - PA12 -> I2C2_SDA
  - VCC -> 3.3V/5V
  - GND -> GND

## 驱动文件
- `Core/Inc/vk16k33.h` - 头文件定义
- `Core/Src/vk16k33.c` - 驱动实现

## 主要功能

### 1. 基本控制函数
- `VK16K33_Init()` - 初始化VK16K33芯片
- `VK16K33_SetBrightness(brightness)` - 设置亮度(0-15)
- `VK16K33_Clear()` - 清除所有显示
- `VK16K33_SetDigit(position, segment_data)` - 在指定位置显示段码
- `VK16K33_DisplayNumber(number)` - 显示4位数字(0-9999)

### 2. 测试和动画函数
- `VK16K33_Test_AllDigits()` - 完整的数码管测试序列
- `VK16K33_Animation_Running()` - 流水灯动画效果
- `VK16K33_Animation_Blink()` - 闪烁动画效果

## 段码定义
支持0-9数字和A-F字母显示：
```c
#define DIGIT_0     0x3F    // 0
#define DIGIT_1     0x06    // 1
#define DIGIT_2     0x5B    // 2
// ... 等等
#define DIGIT_A     0x77    // A
#define DIGIT_B     0x7C    // b
#define DIGIT_C     0x39    // C
// ... 等等
```

## 运行流程

### 初始化阶段
1. 系统启动后自动初始化VK16K33
2. 执行完整的数码管测试序列：
   - 逐个点亮数码管位置0-7
   - 显示"8888"
   - 计数显示0-15
3. 测试完成后清除显示

### 运行阶段
- 主循环中每1秒更新一次显示
- 显示计数器从0000到9999循环
- 通过SEGGER RTT输出当前显示值用于调试

## 调试输出
所有操作都通过SEGGER RTT输出调试信息：
```
VK16K33: Initializing...
VK16K33: Initialization successful
VK16K33: Starting digit test...
VK16K33: Lighting digit 0
VK16K33: Lighting digit 1
...
Display: 0001
Display: 0002
...
```

## 错误处理
- 所有I2C通信都有超时保护(100ms)
- 函数返回值: 1=成功, 0=失败
- 详细的错误信息通过RTT输出

## 扩展功能
驱动支持8位数码管，当前使用前4位显示数字。可以轻松扩展为：
- 显示更多位数
- 显示时间格式(HH:MM)
- 自定义字符显示
- 不同的动画效果

## 编译结果
- Flash使用: 35,920字节 (27.40%)
- RAM使用: 8,480字节 (23.00%)
- 编译无错误无警告

## 注意事项
1. VK16K33地址可能需要根据硬件设置调整
2. 确保I2C时序配置正确
3. 数码管显示格式为共阴极
4. 支持的数码管数量最多8位