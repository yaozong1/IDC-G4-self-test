#ifndef __W25Q32_H
#define __W25Q32_H

#include "main.h"

/* GPIO definitions */
#define FLASH_CS_Pin       GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA

/* W25Q32JVSS 指令集 */
#define W25Q32_WRITE_ENABLE      0x06
#define W25Q32_WRITE_DISABLE     0x04
#define W25Q32_READ_STATUS_REG1  0x05
#define W25Q32_READ_STATUS_REG2  0x35
#define W25Q32_WRITE_STATUS_REG  0x01
#define W25Q32_PAGE_PROGRAM      0x02
#define W25Q32_SECTOR_ERASE     0x20
#define W25Q32_BLOCK_ERASE      0xD8
#define W25Q32_CHIP_ERASE       0xC7
#define W25Q32_POWER_DOWN       0xB9
#define W25Q32_RELEASE_POWER_DOWN 0xAB
#define W25Q32_DEVICE_ID        0xAB
#define W25Q32_MANUF_ID         0x90
#define W25Q32_JEDEC_ID         0x9F
#define W25Q32_READ_DATA        0x03
#define W25Q32_FAST_READ        0x0B

/* 函数声明 */
void W25Q32_Init(void);
uint8_t W25Q32_ReadID(void);
void W25Q32_ReadJEDECID(uint8_t *manufacturer_id, uint8_t *memory_type, uint8_t *capacity);
void W25Q32_Write_Enable(void);
void W25Q32_Wait_Busy(void);
void W25Q32_Erase_Sector(uint32_t sector_addr);
void W25Q32_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25Q32_Read_Data(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void W25Q32_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25Q32_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

#endif