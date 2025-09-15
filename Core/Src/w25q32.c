#include "w25q32.h"
#include "SEGGER_RTT.h"

extern SPI_HandleTypeDef hspi1;
#define W25Q32_CS_LOW()       HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET)
#define W25Q32_CS_HIGH()      HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET)

static uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);       
    return Rxdata;
}

void W25Q32_Init(void)
{
    // 确保CS引脚为高电平（空闲状态）
    W25Q32_CS_HIGH();
    HAL_Delay(100);
    
    // 释放可能的掉电模式
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25Q32_RELEASE_POWER_DOWN);
    W25Q32_CS_HIGH();
    HAL_Delay(10);
    
    // 解除写保护
    W25Q32_Write_Enable();
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25Q32_WRITE_STATUS_REG);
    SPI1_ReadWriteByte(0x00);  // 设置状态寄存器1的值为0，解除所有保护
    W25Q32_CS_HIGH();
    W25Q32_Wait_Busy();
}

uint8_t W25Q32_ReadID(void)
{
    uint8_t device_id = 0;
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(0x90);  // Read Manufacturer/Device ID
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0xFF);  // 跳过制造商ID
    device_id = SPI1_ReadWriteByte(0xFF);  // 读取设备ID
    W25Q32_CS_HIGH();
    return device_id;
}

// 添加JEDEC ID读取函数
void W25Q32_ReadJEDECID(uint8_t *manufacturer_id, uint8_t *memory_type, uint8_t *capacity)
{
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(0x9F);  // JEDEC ID命令
    *manufacturer_id = SPI1_ReadWriteByte(0xFF);
    *memory_type = SPI1_ReadWriteByte(0xFF);
    *capacity = SPI1_ReadWriteByte(0xFF);
    W25Q32_CS_HIGH();
}

void W25Q32_Write_Enable(void)
{
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25Q32_WRITE_ENABLE);
    W25Q32_CS_HIGH();
}

void W25Q32_Wait_Busy(void)
{
    uint8_t status;
    do
    {
        W25Q32_CS_LOW();
        SPI1_ReadWriteByte(W25Q32_READ_STATUS_REG1);
        status = SPI1_ReadWriteByte(0xFF);
        W25Q32_CS_HIGH();
    } while ((status & 0x01) == 0x01);
}

void W25Q32_Erase_Sector(uint32_t sector_addr)
{
    sector_addr *= 4096;
    W25Q32_Write_Enable();
    W25Q32_Wait_Busy();
    
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25Q32_SECTOR_ERASE);
    SPI1_ReadWriteByte((uint8_t)((sector_addr) >> 16));
    SPI1_ReadWriteByte((uint8_t)((sector_addr) >> 8));
    SPI1_ReadWriteByte((uint8_t)sector_addr);
    W25Q32_CS_HIGH();
    
    W25Q32_Wait_Busy();
}

void W25Q32_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    W25Q32_Write_Enable();
    
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25Q32_PAGE_PROGRAM);
    SPI1_ReadWriteByte((uint8_t)((WriteAddr) >> 16));
    SPI1_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
    SPI1_ReadWriteByte((uint8_t)WriteAddr);
    
    for(uint16_t i = 0; i < NumByteToWrite; i++)
    {
        SPI1_ReadWriteByte(pBuffer[i]);
    }
    
    W25Q32_CS_HIGH();
    W25Q32_Wait_Busy();
}

void W25Q32_Read_Data(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    W25Q32_CS_LOW();
    SPI1_ReadWriteByte(W25Q32_READ_DATA);
    SPI1_ReadWriteByte((uint8_t)((ReadAddr) >> 16));
    SPI1_ReadWriteByte((uint8_t)((ReadAddr) >> 8));
    SPI1_ReadWriteByte((uint8_t)ReadAddr);
    
    for(uint16_t i = 0; i < NumByteToRead; i++)
    {
        pBuffer[i] = SPI1_ReadWriteByte(0xFF);
    }
    
    W25Q32_CS_HIGH();
}

void W25Q32_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;
    pageremain = 256 - WriteAddr % 256;
    
    if(NumByteToWrite <= pageremain)
    {
        pageremain = NumByteToWrite;
    }
    
    while(1)
    {
        W25Q32_Write_Page(pBuffer, WriteAddr, pageremain);
        
        if(NumByteToWrite == pageremain)
        {
            break;
        }
        else
        {
            pBuffer += pageremain;
            WriteAddr += pageremain;
            NumByteToWrite -= pageremain;
            
            if(NumByteToWrite > 256)
            {
                pageremain = 256;
            }
            else
            {
                pageremain = NumByteToWrite;
            }
        }
    }
}

uint8_t W25Q32_BUFFER[4096];
void W25Q32_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    
    secpos = WriteAddr / 4096;
    secoff = WriteAddr % 4096;
    secremain = 4096 - secoff;
    
    if(NumByteToWrite <= secremain)
    {
        secremain = NumByteToWrite;
    }
    
    while(1)
    {
        W25Q32_Read_Data(W25Q32_BUFFER, secpos*4096, 4096);
        
        for(i = 0; i < secremain; i++)
        {
            if(W25Q32_BUFFER[secoff + i] != 0xFF)
                break;
        }
        
        if(i < secremain)
        {
            W25Q32_Erase_Sector(secpos);
            
            for(i = 0; i < secremain; i++)
            {
                W25Q32_BUFFER[i + secoff] = pBuffer[i];
            }
            W25Q32_Write_NoCheck(W25Q32_BUFFER, secpos*4096, 4096);
        }
        else
        {
            W25Q32_Write_NoCheck(pBuffer, WriteAddr, secremain);
        }
        
        if(NumByteToWrite == secremain)
        {
            break;
        }
        else
        {
            secpos++;
            secoff = 0;
            pBuffer += secremain;
            WriteAddr += secremain;
            NumByteToWrite -= secremain;
            
            if(NumByteToWrite > 4096)
            {
                secremain = 4096;
            }
            else
            {
                secremain = NumByteToWrite;
            }
        }
    }
}