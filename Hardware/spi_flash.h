#ifndef SPI_FLASH_H
#define SPI_FLASH_H
#include "main.h"

/* 天空星 W25Q128：SPI1 PA5/6/7，PA4 软件片选。
 * 仅默认任务调用；擦除/编程只提交命令，随后由调用者轮询忙位。
 * 不提供整片擦除，不自动解除写保护。 */
#define SPI_FLASH_SIZE       0x1000000UL
#define SPI_FLASH_SECTOR     4096U
#define SPI_FLASH_PAGE       256U
HAL_StatusTypeDef SPIFlash_Init(void);
HAL_StatusTypeDef SPIFlash_Ready(void); /* HAL_BUSY 表示芯片正在擦写 */
HAL_StatusTypeDef SPIFlash_Read(uint32_t address, void *data, uint16_t length);
HAL_StatusTypeDef SPIFlash_Program(uint32_t address, const void *data, uint16_t length);
HAL_StatusTypeDef SPIFlash_EraseSector(uint32_t address);
extern uint32_t spi_flash_jedec_id;
#endif
