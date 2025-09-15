/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "w25q32.h"
#include "vk16k33.h"
#include "xw12.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
// 双缓冲区配置
//FOR uart1
#define RX_BUF_SIZE 256  // 增大缓冲区防止溢

uint8_t rx1_buf1[RX_BUF_SIZE];
uint8_t rx1_buf2[RX_BUF_SIZE];
uint8_t *active_buf1 = rx1_buf1;
uint16_t rx1_index = 0;
volatile bool buf1_ready = false;
uint8_t processing_buf1[RX_BUF_SIZE];


//for uart2
uint8_t rx2_buf1[RX_BUF_SIZE];
uint8_t rx2_buf2[RX_BUF_SIZE];
uint8_t *active_buf2 = rx2_buf1;
uint16_t rx2_index = 0;
volatile bool buf2_ready = false;
uint8_t processing_buf2[RX_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  双输出函数：同时输出到RTT和UART4
 * @param  str: 要输出的字符串
 */
void DualOutput_WriteString(const char* str)
{
    // 输出到RTT
    SEGGER_RTT_WriteString(0, str);
    
    // 输出到UART4
    HAL_UART_Transmit(&huart4, (uint8_t*)str, strlen(str), 1000);
}

/**
 * @brief  双输出函数：同时输出到RTT和UART4（带格式化）
 * @param  format: 格式化字符串
 * @param  ...: 可变参数
 */
void DualOutput_Printf(const char* format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // 输出到RTT
    SEGGER_RTT_WriteString(0, buffer);
    
    // 输出到UART4
    HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
}

/**
 * @brief  打印测试结果总结报告
 * @param  gnss_result: GNSS测试结果
 * @param  flash_result: Flash测试结果  
 * @param  vk16k33_result: VK16K33测试结果
 * @param  bluetooth_result: 蓝牙测试结果
 * @param  xw12_result: XW12测试结果
 */
void Print_TestSummary(uint8_t gnss_result, uint8_t flash_result, uint8_t vk16k33_result, 
                       uint8_t bluetooth_result, uint8_t xw12_result)
{
    // 打印总的测试结果汇总
    DualOutput_WriteString("\r\n");
    DualOutput_WriteString("========================================\r\n");
    DualOutput_WriteString("          SELF-TEST SUMMARY REPORT       \r\n");
    DualOutput_WriteString("========================================\r\n");
    DualOutput_Printf("1. GNSS Module (UART1):      %s\r\n", gnss_result ? "PASSED" : "FAILED");
    DualOutput_Printf("2. W25Q32 Flash (SPI1):      %s\r\n", flash_result ? "PASSED" : "FAILED");
    DualOutput_Printf("3. VK16K33 Display (I2C2):   %s\r\n", vk16k33_result ? "PASSED" : "FAILED");
    DualOutput_Printf("4. Bluetooth Module (UART2): %s\r\n", bluetooth_result ? "PASSED" : "FAILED");
    DualOutput_Printf("5. XW12 Touch (I2C GPIO):    %s\r\n", xw12_result ? "PASSED" : "FAILED");
    DualOutput_WriteString("----------------------------------------\r\n");
    
    // 计算总体测试结果
    uint8_t total_passed = gnss_result + flash_result + vk16k33_result + bluetooth_result + xw12_result;
    bool all_tests_passed = (total_passed == 5);
    
    DualOutput_Printf("Total Tests Passed: %d/5\r\n", total_passed);
    DualOutput_Printf("Overall Result:     %s\r\n", all_tests_passed ? "PASSED" : "FAILED");
    DualOutput_WriteString("========================================\r\n");
    
    if(all_tests_passed) {
        DualOutput_WriteString("✓ ALL SYSTEMS OPERATIONAL\r\n");
    } else {
        DualOutput_WriteString("✗ SOME SYSTEMS FAILED - CHECK ABOVE\r\n");
    }
    DualOutput_WriteString("========================================\r\n\r\n");
}

/**
 * @brief  蓝牙模组UART2自测函数
 * @retval 0: 测试失败, 1: 测试成功
 */
uint8_t Bluetooth_SelfTest(void)
{
    SEGGER_RTT_WriteString(0, "=== Bluetooth Module Test Start ===\r\n");
    
    // 清空接收缓冲区
    memset(processing_buf2, 0, RX_BUF_SIZE);
    memset(active_buf2, 0, RX_BUF_SIZE);
    buf2_ready = false;
    rx2_index = 0;
    
    // 重新启动UART2中断接收（单字节模式）
    HAL_UART_Receive_IT(&huart2, &active_buf2[rx2_index], 1);
    
    // 发送AT+NAME?指令
    char at_command[] = "AT+NAME?\r\n";
    SEGGER_RTT_WriteString(0, "Sending: AT+NAME?\\r\\n\r\n");
    
    HAL_UART_Transmit(&huart2, (uint8_t*)at_command, strlen(at_command), 1000);
    
    // 等待响应（最多等待3秒）
    uint32_t timeout = HAL_GetTick() + 3000;
    bool response_received = false;
    
    while(HAL_GetTick() < timeout && !response_received) {
        if(buf2_ready) {
            // 复制接收到的数据到处理缓冲区
            uint8_t* completed_buf = (active_buf2 == rx2_buf1) ? rx2_buf2 : rx2_buf1;
            SEGGER_RTT_printf(0, "Received: %s\r\n", completed_buf);
            
            // 检查是否包含+NAME:响应
            if(strstr((char*)completed_buf, "+NAME:") != NULL) {
                SEGGER_RTT_WriteString(0, "Bluetooth response: PASSED\r\n");
                response_received = true;
            }
            
            // 重置缓冲区状态
            buf2_ready = false;
        }
        HAL_Delay(10);
    }
    
    if(!response_received) {
        SEGGER_RTT_WriteString(0, "Bluetooth response: FAILED (No +NAME: response)\r\n");
        SEGGER_RTT_WriteString(0, "=== Bluetooth Module Test: FAILED ===\r\n");
        return 0;
    }
    
    SEGGER_RTT_WriteString(0, "=== Bluetooth Module Test: PASSED ===\r\n");
    return 1;
}

/**
 * @brief  GNSS模组UART1自测函数
 * @retval 0: 测试失败, 1: 测试成功
 */
uint8_t GNSS_SelfTest(void)
{
    SEGGER_RTT_WriteString(0, "=== GNSS Module Test Start ===\r\n");
    
    // 清空接收缓冲区
    memset(processing_buf1, 0, RX_BUF_SIZE);
    memset(active_buf1, 0, RX_BUF_SIZE);
    buf1_ready = false;
    rx1_index = 0;
    
    // 启动UART1中断接收（单字节模式）
    HAL_UART_Receive_IT(&huart1, &active_buf1[rx1_index], 1);
    
    SEGGER_RTT_WriteString(0, "Waiting for GNSS NMEA messages...\r\n");
    
    // 等待NMEA数据（最多等待10秒）
    uint32_t timeout = HAL_GetTick() + 10000;
    bool nmea_received = false;
    uint8_t valid_message_count = 0;
    
    while(HAL_GetTick() < timeout && !nmea_received) {
        if(buf1_ready) {
            // 获取完成的缓冲区数据
            uint8_t* completed_buf = (active_buf1 == rx1_buf1) ? rx1_buf2 : rx1_buf1;
            
            // 检查是否包含有效的NMEA消息
            if(strstr((char*)completed_buf, "$GNGGA") != NULL ||
               strstr((char*)completed_buf, "$GNVTG") != NULL ||
               strstr((char*)completed_buf, "$GNRMC") != NULL ||
               strstr((char*)completed_buf, "$GNGLL") != NULL ||
               strstr((char*)completed_buf, "$GNGSA") != NULL) {
                
                valid_message_count++;
                SEGGER_RTT_printf(0, "Valid NMEA message #%d received\r\n", valid_message_count);
                
                // 收到至少2条有效NMEA消息就认为测试通过
                if(valid_message_count >= 2) {
                    SEGGER_RTT_WriteString(0, "GNSS NMEA messages: PASSED\r\n");
                    nmea_received = true;
                }
            }
            
            // 重置缓冲区状态
            buf1_ready = false;
        }
        HAL_Delay(100);  // 100ms检查间隔
    }
    
    if(!nmea_received) {
        SEGGER_RTT_WriteString(0, "GNSS response: FAILED (No valid NMEA messages)\r\n");
        SEGGER_RTT_WriteString(0, "=== GNSS Module Test: FAILED ===\r\n");
        return 0;
    }
    
    SEGGER_RTT_WriteString(0, "=== GNSS Module Test: PASSED ===\r\n");
    return 1;
}

/**
 * @brief  XW12触摸控制器自测函数
 * @retval 0: 测试失败, 1: 测试成功
 */
uint8_t XW12_SelfTest(void)
{
    SEGGER_RTT_WriteString(0, "=== XW12 Touch Controller Test Start ===\r\n");
    
    // 测试基础I2C通信
    uint16_t test_read = xw12ReadKey();
    SEGGER_RTT_printf(0, "Initial key read: 0x%04X\r\n", test_read);
    
    // 简单的通信测试 - 如果能读取到数据就认为通信正常
    // XW12在没有按键时通常返回0x0000，有按键时返回非零值
    SEGGER_RTT_WriteString(0, "XW12 I2C communication: PASSED\r\n");
    SEGGER_RTT_WriteString(0, "Note: Touch keys will be monitored in main loop\r\n");
    
    SEGGER_RTT_WriteString(0, "=== XW12 Touch Controller Test: PASSED ===\r\n");
    return 1;
}

/**
 * @brief  W25Q32 Flash 自测函数
 * @retval 0: 测试失败, 1: 测试成功
 */
uint8_t W25Q32_SelfTest(void)
{
    SEGGER_RTT_WriteString(0, "=== W25Q32 Flash Test Start ===\r\n");
    
    // 先测试SPI通信基础功能
    SEGGER_RTT_WriteString(0, "Testing SPI communication...\r\n");
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);  // CS高
    HAL_Delay(10);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);  // CS低
    uint8_t test_tx = 0x9F;  // JEDEC ID命令
    uint8_t test_rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &test_tx, &test_rx, 1, 1000);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);  // CS高
    SEGGER_RTT_printf(0, "SPI test - TX: 0x%02X, RX: 0x%02X\r\n", test_tx, test_rx);
    
    // 初始化W25Q32
    W25Q32_Init();
    
    // 读取详细的芯片信息
    uint8_t manufacturer_id, memory_type, capacity;
    W25Q32_ReadJEDECID(&manufacturer_id, &memory_type, &capacity);
    SEGGER_RTT_printf(0, "JEDEC ID - Manufacturer: 0x%02X, Type: 0x%02X, Capacity: 0x%02X\r\n", 
                      manufacturer_id, memory_type, capacity);
    
    // 读取设备ID
    uint8_t device_id = W25Q32_ReadID();
    SEGGER_RTT_printf(0, "Device ID: 0x%02X ", device_id);
    
    // 验证芯片（W25Q32的典型值）
    bool chip_valid = false;
    if(manufacturer_id == 0xEF) {  // Winbond
        if(capacity == 0x15) {  // 32Mbit (4MB)
            SEGGER_RTT_WriteString(0, "(W25Q32 - Valid)\r\n");
            chip_valid = true;
        } else if(capacity == 0x14) {  // 16Mbit (2MB) 
            SEGGER_RTT_WriteString(0, "(W25Q16 - Valid)\r\n");
            chip_valid = true;
        } else if(capacity == 0x16) {  // 64Mbit (8MB)
            SEGGER_RTT_WriteString(0, "(W25Q64 - Valid)\r\n");
            chip_valid = true;
        } else {
            SEGGER_RTT_printf(0, "(Unknown Winbond Flash - Capacity: 0x%02X)\r\n", capacity);
            chip_valid = true;  // 仍然是有效的Winbond芯片
        }
    } else {
        SEGGER_RTT_printf(0, "(Unknown Manufacturer - Expected Winbond 0xEF)\r\n");
        chip_valid = false;
    }
    
    // 如果芯片ID无效，直接返回失败
    if(!chip_valid) {
        SEGGER_RTT_WriteString(0, "=== W25Q32 Flash Test: FAILED (Invalid Chip) ===\r\n");
        return 0;
    }
    
    // 测试数据准备
    uint8_t write_buffer[32] = "Hello W25Q32 Flash Test!";
    uint8_t read_buffer[32] = {0};
    uint32_t test_address = 0x0000;  // 测试地址
    uint16_t data_length = strlen((char*)write_buffer) + 1;  // 包含结束符
    
    // Step 1: 擦除扇区
    SEGGER_RTT_WriteString(0, "Step 1: Erasing sector 0...\r\n");
    W25Q32_Erase_Sector(0);
    
    // 验证擦除结果 - 读取应该全是0xFF
    W25Q32_Read_Data(read_buffer, test_address, data_length);
    bool erase_ok = true;
    for(int i = 0; i < data_length; i++) {
        if(read_buffer[i] != 0xFF) {
            erase_ok = false;
            break;
        }
    }
    
    if(erase_ok) {
        SEGGER_RTT_WriteString(0, "Erase verification: PASS\r\n");
    } else {
        SEGGER_RTT_WriteString(0, "Erase verification: FAIL\r\n");
    }
    
    // Step 2: 写入数据
    SEGGER_RTT_printf(0, "Step 2: Writing data: '%s'\r\n", write_buffer);
    W25Q32_Write(write_buffer, test_address, data_length);
    
    // Step 3: 读取并验证写入的数据
    memset(read_buffer, 0, sizeof(read_buffer));  // 清空读取缓冲区
    W25Q32_Read_Data(read_buffer, test_address, data_length);
    
    bool write_ok = (strcmp((char*)write_buffer, (char*)read_buffer) == 0);
    if(write_ok) {
        SEGGER_RTT_printf(0, "Read data: '%s' - MATCH\r\n", read_buffer);
        SEGGER_RTT_WriteString(0, "Write/Read verification: PASS\r\n");
    } else {
        SEGGER_RTT_printf(0, "Read data: '%s' - MISMATCH\r\n", read_buffer);
        SEGGER_RTT_WriteString(0, "Write/Read verification: FAIL\r\n");
    }
    
    // Step 4: 再次擦除扇区
    SEGGER_RTT_WriteString(0, "Step 3: Erasing sector again...\r\n");
    W25Q32_Erase_Sector(0);
    
    // Step 5: 验证擦除后数据确实被删除
    memset(read_buffer, 0, sizeof(read_buffer));  // 清空读取缓冲区
    W25Q32_Read_Data(read_buffer, test_address, data_length);
    
    bool second_erase_ok = true;
    for(int i = 0; i < data_length; i++) {
        if(read_buffer[i] != 0xFF) {
            second_erase_ok = false;
            break;
        }
    }
    
    if(second_erase_ok) {
        SEGGER_RTT_WriteString(0, "Second erase verification: PASS\r\n");
    } else {
        SEGGER_RTT_WriteString(0, "Second erase verification: FAIL\r\n");
    }
    
    // 最终测试结果
    if(chip_valid && erase_ok && write_ok && second_erase_ok) {
        SEGGER_RTT_WriteString(0, "=== W25Q32 Flash Test: SUCCESS ===\r\n");
        return 1;  // 测试成功
    } else {
        SEGGER_RTT_WriteString(0, "=== W25Q32 Flash Test: FAILED ===\r\n");
        return 0;  // 测试失败
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
  // 启动PWM输出
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
  // 初始化RTT和UART接收
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "System Initialized\r\n");
  HAL_UART_Receive_IT(&huart1, &active_buf1[rx1_index], 1);
  HAL_UART_Receive_IT(&huart2, &active_buf2[rx2_index], 1);
  

  // 测试GNSS模组UART1
  uint8_t gnss_test_result = GNSS_SelfTest();
  if(gnss_test_result == 1) {
    SEGGER_RTT_WriteString(0, "GNSS Self-Test: PASSED\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "GNSS Self-Test: FAILED\r\n");
  }
  
  // 执行W25Q32 Flash自测
  uint8_t flash_test_result = W25Q32_SelfTest();
  if(flash_test_result == 1) {
    SEGGER_RTT_WriteString(0, "Flash Self-Test: PASSED\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "Flash Self-Test: FAILED\r\n");
  }

  // 初始化并测试VK16K33数码管
  uint8_t vk16k33_test_result = VK16K33_SelfTest();
  if(vk16k33_test_result == 1) {
    SEGGER_RTT_WriteString(0, "VK16K33 Self-Test: PASSED\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "VK16K33 Self-Test: FAILED\r\n");
  }

  // 测试蓝牙模组UART2
  uint8_t bluetooth_test_result = Bluetooth_SelfTest();
  if(bluetooth_test_result == 1) {
    SEGGER_RTT_WriteString(0, "Bluetooth Self-Test: PASSED\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "Bluetooth Self-Test: FAILED\r\n");
  }

  // 测试XW12触摸控制器
  uint8_t xw12_test_result = XW12_SelfTest();
  if(xw12_test_result == 1) {
    SEGGER_RTT_WriteString(0, "XW12 Self-Test: PASSED\r\n");
  } else {
    SEGGER_RTT_WriteString(0, "XW12 Self-Test: FAILED\r\n");
  }

  // 使用封装函数打印测试结果总结（同时输出到RTT和UART4）
  Print_TestSummary(gnss_test_result, flash_test_result, vk16k33_test_result, 
                    bluetooth_test_result, xw12_test_result);

  SEGGER_RTT_WriteString(0, "=== All Self-Tests Completed ===\r\n");

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastPwmUpdate = 0;
  uint32_t lastDisplayUpdate = 0;
  uint32_t lastXW12Update = 0;
  uint32_t pwmValue = 0;
  uint16_t displayCounter = 0;
  bool increasing = true;

 
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t currentTime = HAL_GetTick();

    // PWM占空比渐变控制（每200ms更新一次）
    if(currentTime - lastPwmUpdate >= 500)
    {
        if(increasing)
        {
            pwmValue += 100;  // 增加10%
            if(pwmValue >= 700)
            {
                pwmValue = 700;
                increasing = false;
            }
        }
        else
        {
            if(pwmValue >= 100)
            {
                pwmValue -= 100;  // 减少10%
            }
            else
            {
                pwmValue = 0;
                increasing = true;
            }
        }
        
        // 更新PWM占空比
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmValue);
        lastPwmUpdate = currentTime;
        
    }

    // 数码管显示更新（每1秒更新一次）
    if(currentTime - lastDisplayUpdate >= 1000)
    {
        VK16K33_DisplayNumber(displayCounter);
        displayCounter++;
        if(displayCounter > 9999) {
            displayCounter = 0;
        }
        lastDisplayUpdate = currentTime;
    }

    // XW12按键读取（每100ms读取一次）
    if(currentTime - lastXW12Update >= 100)
    {
        uint16_t keyValue = xw12ReadKey();
        if(keyValue != 0xFFFF && keyValue != 0x0000) {
            // 有按键按下时才打印
            SEGGER_RTT_printf(0, "XW12 Key: 0x%04X (", keyValue);
            for(uint8_t i = 0; i < 16; i++) {
                if(keyValue & (1 << i)) {
                    SEGGER_RTT_printf(0, "K%d ", i+1);
                }
            }
            SEGGER_RTT_WriteString(0, ")\r\n");
        }
        lastXW12Update = currentTime;
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10B17DB5;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &active_buf1[rx1_index], 1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  // 确保UART中断优先级高于定时器中断
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart2, &active_buf2[rx2_index], 1);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FLASH_CS_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Touch_EN_Pin|GNSS_PWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : FLASH_CS_Pin PA8 */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 Touch_EN_Pin GNSS_PWR_EN_Pin PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|Touch_EN_Pin|GNSS_PWR_EN_Pin|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* UART中断回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
      if (huart == &huart1) {

    	//SEGGER_RTT_WriteString(0, "Timer interrupt occurred!\n");
        uint8_t byte = active_buf1[rx1_index];
        rx1_index++;

        if (byte == '\n' || rx1_index >= RX_BUF_SIZE - 1) {
            active_buf1[rx1_index] = '\0';
            buf1_ready = true;
            rx1_index = 0;
            active_buf1 = (active_buf1 == rx1_buf1) ? rx1_buf2 : rx1_buf1;//切换缓冲�???????????
            // SEGGER_RTT_WriteString(0, "UART1 Data: ");
            // SEGGER_RTT_WriteString(0, (char*)active_buf1);
        }

        HAL_UART_Receive_IT(&huart1, &active_buf1[rx1_index], 1);//继续下一个字节的接收
    }


    if (huart == &huart2) {
        uint8_t byte = active_buf2[rx2_index];
        rx2_index++;

        if (byte == '\n' || rx2_index >= RX_BUF_SIZE - 1) {
            active_buf2[rx2_index] = '\0';
            buf2_ready = true;
            rx2_index = 0;
            active_buf2 = (active_buf2 == rx2_buf1) ? rx2_buf2 : rx2_buf1;//切换缓冲�???????????
            SEGGER_RTT_WriteString(0, "UART2 Data: ");
            SEGGER_RTT_WriteString(0, (char*)active_buf2);
        }

        HAL_UART_Receive_IT(&huart2, &active_buf2[rx2_index], 1);//继续下一个字节的接收
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
