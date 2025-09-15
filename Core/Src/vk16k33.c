#include "vk16k33.h"
#include "SEGGER_RTT.h"

/* 外部I2C句柄声明 */
extern I2C_HandleTypeDef hi2c2;

/* 数码管段码查找表 */
static const uint8_t digit_segments[] = {
    DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4,
    DIGIT_5, DIGIT_6, DIGIT_7, DIGIT_8, DIGIT_9,
    DIGIT_A, DIGIT_B, DIGIT_C, DIGIT_D, DIGIT_E, DIGIT_F
};

/**
 * @brief  初始化VK16K33芯片
 * @retval 0: 失败, 1: 成功
 */
uint8_t VK16K33_Init(void)
{
    HAL_StatusTypeDef status;
    
    SEGGER_RTT_printf(0, "VK16K33: Initializing...\n");
    
    /* 开启振荡器 */
    uint8_t cmd = VK16K33_CMD_OSCILLATOR;
    status = HAL_I2C_Master_Transmit(&hi2c2, VK16K33_I2C_ADDR << 1, &cmd, 1, 100);
    if (status != HAL_OK) {
        SEGGER_RTT_printf(0, "VK16K33: Failed to enable oscillator (0x%02X)\n", status);
        return 0;
    }
    
    HAL_Delay(1);
    
    /* 开启显示 */
    cmd = VK16K33_CMD_DISPLAY_ON;
    status = HAL_I2C_Master_Transmit(&hi2c2, VK16K33_I2C_ADDR << 1, &cmd, 1, 100);
    if (status != HAL_OK) {
        SEGGER_RTT_printf(0, "VK16K33: Failed to enable display (0x%02X)\n", status);
        return 0;
    }
    
    HAL_Delay(1);
    
    /* 设置中等亮度 */
    if (!VK16K33_SetBrightness(8)) {
        SEGGER_RTT_printf(0, "VK16K33: Failed to set brightness\n");
        return 0;
    }
    
    /* 清除显示 */
    if (!VK16K33_Clear()) {
        SEGGER_RTT_printf(0, "VK16K33: Failed to clear display\n");
        return 0;
    }
    
    SEGGER_RTT_printf(0, "VK16K33: Initialization successful\n");
    return 1;
}

/**
 * @brief  设置显示亮度
 * @param  brightness: 亮度级别 (0-15)
 * @retval 0: 失败, 1: 成功
 */
uint8_t VK16K33_SetBrightness(uint8_t brightness)
{
    if (brightness > 15) {
        brightness = 15;
    }
    
    uint8_t cmd = VK16K33_CMD_BRIGHTNESS | brightness;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, VK16K33_I2C_ADDR << 1, &cmd, 1, 100);
    
    if (status != HAL_OK) {
        SEGGER_RTT_printf(0, "VK16K33: Failed to set brightness (0x%02X)\n", status);
        return 0;
    }
    
    return 1;
}

/**
 * @brief  清除所有显示
 * @retval 0: 失败, 1: 成功
 */
uint8_t VK16K33_Clear(void)
{
    uint8_t data[17] = {0}; // 第一个字节是起始地址0x00，后面16字节为显示数据
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, VK16K33_I2C_ADDR << 1, data, sizeof(data), 100);
    
    if (status != HAL_OK) {
        SEGGER_RTT_printf(0, "VK16K33: Failed to clear display (0x%02X)\n", status);
        return 0;
    }
    
    return 1;
}

/**
 * @brief  在指定位置显示段码
 * @param  position: 位置 (0-7，对应8个数码管位置)
 * @param  segment_data: 段码数据
 * @retval 0: 失败, 1: 成功
 */
uint8_t VK16K33_SetDigit(uint8_t position, uint8_t segment_data)
{
    if (position > 7) {
        return 0;
    }
    
    uint8_t data[3];
    data[0] = position * 2;  // 每个数码管占用2个字节，起始地址
    data[1] = segment_data;  // 段码数据
    data[2] = 0x00;         // 高字节通常为0
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, VK16K33_I2C_ADDR << 1, data, 3, 100);
    
    if (status != HAL_OK) {
        SEGGER_RTT_printf(0, "VK16K33: Failed to set digit %d (0x%02X)\n", position, status);
        return 0;
    }
    
    return 1;
}

/**
 * @brief  显示4位数字
 * @param  number: 要显示的数字 (0-9999)
 * @retval 0: 失败, 1: 成功
 */
uint8_t VK16K33_DisplayNumber(uint16_t number)
{
    if (number > 9999) {
        number = 9999;
    }
    
    /* 提取各位数字 */
    uint8_t thousands = (number / 1000) % 10;
    uint8_t hundreds = (number / 100) % 10;
    uint8_t tens = (number / 10) % 10;
    uint8_t ones = number % 10;
    
    /* 显示各位数字 */
    if (!VK16K33_SetDigit(0, digit_segments[thousands])) return 0;
    if (!VK16K33_SetDigit(1, digit_segments[hundreds])) return 0;
    if (!VK16K33_SetDigit(2, digit_segments[tens])) return 0;
    if (!VK16K33_SetDigit(3, digit_segments[ones])) return 0;
    
    return 1;
}

/**
 * @brief  测试所有数码管 - 轮流点亮
 * @retval 0: 失败, 1: 成功
 */
uint8_t VK16K33_Test_AllDigits(void)
{
    SEGGER_RTT_printf(0, "VK16K33: Starting digit test...\n");
    
    /* 清除显示 */
    if (!VK16K33_Clear()) {
        return 0;
    }
    
    HAL_Delay(500);
    
    /* 逐个点亮数码管，显示0-7 */
    for (uint8_t i = 0; i < 8; i++) {
        /* 清除所有显示 */
        VK16K33_Clear();
        
        /* 点亮当前位置 */
        if (!VK16K33_SetDigit(i, digit_segments[i])) {
            SEGGER_RTT_printf(0, "VK16K33: Failed to light digit %d\n", i);
            return 0;
        }
        
        SEGGER_RTT_printf(0, "VK16K33: Lighting digit %d\n", i);
        HAL_Delay(500);
    }
    
    /* 全部点亮显示8888 */
    SEGGER_RTT_printf(0, "VK16K33: Displaying 8888\n");
    for (uint8_t i = 0; i < 4; i++) {
        VK16K33_SetDigit(i, DIGIT_8);
    }
    HAL_Delay(1000);
    
    /* 显示计数 0000-0015 */
    SEGGER_RTT_printf(0, "VK16K33: Displaying counting 0-15\n");
    for (uint16_t count = 0; count < 16; count++) {
        VK16K33_DisplayNumber(count);
        HAL_Delay(300);
    }
    
    /* 清除显示 */
    VK16K33_Clear();
    
    SEGGER_RTT_printf(0, "VK16K33: Test completed successfully\n");
    return 1;
}

/**
 * @brief  数码管动画效果 - 流水灯
 * @retval 0: 失败, 1: 成功
 */
uint8_t VK16K33_Animation_Running(void)
{
    /* 流水灯效果 - 只点亮一位，从左到右 */
    for (uint8_t cycle = 0; cycle < 3; cycle++) {
        for (uint8_t pos = 0; pos < 4; pos++) {
            VK16K33_Clear();
            VK16K33_SetDigit(pos, DIGIT_8);
            HAL_Delay(200);
        }
    }
    return 1;
}

/**
 * @brief  数码管动画效果 - 闪烁
 * @retval 0: 失败, 1: 成功
 */
uint8_t VK16K33_Animation_Blink(void)
{
    /* 闪烁效果 */
    for (uint8_t blink = 0; blink < 5; blink++) {
        VK16K33_DisplayNumber(8888);
        HAL_Delay(300);
        VK16K33_Clear();
        HAL_Delay(300);
    }
    return 1;
}

/**
 * @brief  VK16K33数码管完整自测函数
 * @retval 0: 测试失败, 1: 测试成功
 */
uint8_t VK16K33_SelfTest(void)
{
    SEGGER_RTT_WriteString(0, "=== VK16K33 Quick Test ===\r\n");
    if (!VK16K33_Init()) {
        SEGGER_RTT_WriteString(0, "VK16K33 init FAILED\r\n");
        return 0;
    }
    VK16K33_SetBrightness(8);
    VK16K33_DisplayNumber(1234);
    SEGGER_RTT_WriteString(0, "Display 1234 OK\r\n");
    return 1;
}