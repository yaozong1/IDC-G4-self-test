/* Private includes ----------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stdio.h"
#include "main.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define XW12_SDA(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (GPIO_PinState)x) //需根据实际管脚修改
#define XW12_SCL(x) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (GPIO_PinState)x) //需根据实际管脚修改

#define XW12DELAY 3
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void configureXW12SDAOutput(void);
static void configureXW12SDAInput(void);
/* Private user code ---------------------------------------------------------*/

/*
 *    此程序为读取I2C数据的子程序，默认SDA是准双向口。即可直接对SDA赋值或者读取SDA数据�?
 *    根据不同的单片机，可能需要先设置SDA为输出，才能对SDA赋�?设置SDA为输入，方可读取SDA的值�?
 *
 */
#define DEV_ADDR 0x81 //根据spec定义相应的器件地址，XW12=0x81;

static void RCCdelay_us(uint32_t udelay)
{
    __IO uint32_t Delay = udelay * 8; //(SystemCoreClock / 8U / 1000000U)
                                      //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
    do
    {

        __NOP();
    } while (Delay--);
}

uint16_t xw12ReadKey(void)
{
    unsigned char bitnum, temp, addr;
    uint16_t key2byte;
    addr = DEV_ADDR;   //芯片的器件地址，需根据不同芯片改成对应值�?
    key2byte = 0xffff; //用来存储读到的数据。默认全1.(9通道�?2通道的数据为16bit)
    XW12_SDA(1);
    XW12_SCL(1);
    RCCdelay_us(XW12DELAY); //延迟
    XW12_SDA(0);            // START 信号，因为默认SCL和SDA上拉�?，在SCL=1时SDA下降沿表示START信号
    RCCdelay_us(XW12DELAY); //延迟
    XW12_SCL(0);            // START 信号，因为默认SCL和SDA上拉�?，在SCL=1时SDA下降沿表示START信号

    __disable_irq(); //为演示方便，关闭其它中断。实际使用中需要合理安排相应的中断优先级�?
    __NOP();
    XW12_SCL(0);                           // SCL=0时SDA方可改变
    for (bitnum = 0; bitnum < 8; bitnum++) //发送器件地址
    {
        temp = addr & 0x80; //取地址的最高位
        __NOP();
        if (temp == 0x80) //根据temp的最高位值决定SDA发�?还是0
            XW12_SDA(1);
        else
            XW12_SDA(0);
        addr <<= 1;             //地址移位，下一次循环依然发送的是地址最高位
        RCCdelay_us(XW12DELAY); //延迟
        XW12_SCL(1);            // SCL 时序，参考I2C协议�?
        RCCdelay_us(XW12DELAY);
        XW12_SCL(0); // SCL=0时SDA方可改变
    }

    configureXW12SDAInput();
    XW12_SDA(1); //发送完地址后释放SDA总线，等待芯片的ACK回应�?
    __NOP();
    RCCdelay_us(XW12DELAY); // delay
    XW12_SCL(1);            // SCL拉高，读入SDA数据�?
    RCCdelay_us(XW12DELAY); // delay
    temp = 0;
    while ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) && (temp < 250)) //读到的SDA数据�?，则芯片正常ACK;�?则表示出错。需要根�?<常见问题.pdf>>做检查�?
    {
        temp++;
        __NOP();
    }
    if (temp >= 250)
    { //芯片没有ACK，出错�?
      // return 0XFFFF;
      // printf("ack err\r\n");
    }

    // read key
    for (bitnum = 0; bitnum < 16; bitnum++) //模拟I2C通讯时序，读取按键顺序，连续读取16bit，中间无需ACK判断�?
    {
        XW12_SCL(0);
        RCCdelay_us(XW12DELAY); // delay
        XW12_SCL(1);
        key2byte = key2byte << 1; //移位并在最低位保存读到的数�?
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
        {
            key2byte++;
        }
        RCCdelay_us(XW12DELAY);
    }
    XW12_SCL(0);
    configureXW12SDAOutput();
    XW12_SDA(1); // SDA=1,准备发送NACK信号
    RCCdelay_us(XW12DELAY);
    XW12_SCL(1);
    RCCdelay_us(XW12DELAY); //发送NACK信号

    XW12_SCL(0);
    XW12_SDA(0);
    __NOP();
    RCCdelay_us(XW12DELAY); //
    XW12_SCL(1);
    __NOP();
    XW12_SDA(1);            //发送STOP信号
    RCCdelay_us(XW12DELAY); // delay
    __enable_irq();         //恢复中断使能

    key2byte = key2byte ^ 0xffff; //对读到的数据做取反操作，则有按键按下的通道读到的为1;如数据为0x5000,表示通道1和通道3被按�?这一步根据实际需要，可以不要.
    return (key2byte);
}
static void configureXW12SDAOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
static void configureXW12SDAInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
