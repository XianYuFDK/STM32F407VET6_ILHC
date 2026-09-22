#include "spi_flash.h"
#include <string.h>

static SPI_HandleTypeDef s_spi;
static uint8_t s_tx[260], s_rx[260];
static uint8_t s_online;
uint32_t spi_flash_jedec_id;

/* 短 SPI 事务有界等待；绝不关中断等待 Flash 擦写完成。 */
static HAL_StatusTypeDef Transfer(uint16_t size)
{
  HAL_StatusTypeDef result;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  result = HAL_SPI_TransmitReceive(&s_spi, s_tx, s_rx, size, 2U);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  return result;
}

static HAL_StatusTypeDef Status(uint8_t *status)
{
  s_tx[0] = 0x05U; s_tx[1] = 0xFFU;
  if (Transfer(2U) != HAL_OK) return HAL_ERROR;
  *status = s_rx[1];
  /* 全 1 不作为正常就绪，防止断线时继续写。 */
  return (*status == 0xFFU) ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef SPIFlash_Ready(void)
{
  uint8_t status;
  if (!s_online || Status(&status) != HAL_OK) return HAL_ERROR;
  return (status & 1U) ? HAL_BUSY : HAL_OK;
}

HAL_StatusTypeDef SPIFlash_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t start;
  HAL_StatusTypeDef result;
  s_online = 0U;
  spi_flash_jedec_id = 0U;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &gpio);
  s_spi.Instance = SPI1;
  s_spi.Init.Mode = SPI_MODE_MASTER;
  s_spi.Init.Direction = SPI_DIRECTION_2LINES;
  s_spi.Init.DataSize = SPI_DATASIZE_8BIT;
  s_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
  s_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
  s_spi.Init.NSS = SPI_NSS_SOFT;
  s_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 84MHz/16=5.25MHz */
  s_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  s_spi.Init.TIMode = SPI_TIMODE_DISABLE;
  s_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  s_spi.Init.CRCPolynomial = 7U;
  if (HAL_SPI_Init(&s_spi) != HAL_OK) return HAL_ERROR;
  /* 退出深度休眠；不发复位命令，保留 MCU 单独复位前正在执行的擦写。 */
  s_tx[0] = 0xABU;
  if (Transfer(1U) != HAL_OK) return HAL_ERROR;
  HAL_Delay(1U);
  s_online = 1U;
  start = HAL_GetTick();
  do {
    result = SPIFlash_Ready();
    if (result == HAL_ERROR) { s_online = 0U; return HAL_ERROR; }
    if (result == HAL_OK) break;
    HAL_Delay(1U);
  } while ((uint32_t)(HAL_GetTick() - start) < 1000U);
  if (result != HAL_OK) { s_online = 0U; return HAL_TIMEOUT; }
  memset(s_tx, 0xFF, 4U); s_tx[0] = 0x9FU;
  if (Transfer(4U) != HAL_OK) { s_online = 0U; return HAL_ERROR; }
  spi_flash_jedec_id = ((uint32_t)s_rx[1] << 16) | ((uint32_t)s_rx[2] << 8) | s_rx[3];
  /* 仅接受已知的 3.3V W25Q128 JEDEC ID，未知型号绝不擦写。 */
  if (spi_flash_jedec_id != 0xEF4018UL && spi_flash_jedec_id != 0xEF7018UL)
  { s_online = 0U; return HAL_ERROR; }
  return HAL_OK;
}

static void Address(uint8_t command, uint32_t address)
{
  s_tx[0] = command;
  s_tx[1] = (uint8_t)(address >> 16);
  s_tx[2] = (uint8_t)(address >> 8);
  s_tx[3] = (uint8_t)address;
}

HAL_StatusTypeDef SPIFlash_Read(uint32_t address, void *data, uint16_t length)
{
  HAL_StatusTypeDef result;
  if (!data || !length || length > SPI_FLASH_PAGE || address > SPI_FLASH_SIZE - length)
    return HAL_ERROR;
  result = SPIFlash_Ready();
  if (result != HAL_OK) return result;
  Address(0x03U, address);
  memset(s_tx + 4, 0xFF, length);
  if (Transfer((uint16_t)(length + 4U)) != HAL_OK) return HAL_ERROR;
  memcpy(data, s_rx + 4, length);
  return HAL_OK;
}

static HAL_StatusTypeDef WriteEnable(void)
{
  uint8_t status;
  HAL_StatusTypeDef result = SPIFlash_Ready();
  if (result != HAL_OK) return result;
  s_tx[0] = 0x06U;
  if (Transfer(1U) != HAL_OK || Status(&status) != HAL_OK) return HAL_ERROR;
  return ((status & 3U) == 2U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef SPIFlash_Program(uint32_t address, const void *data, uint16_t length)
{
  HAL_StatusTypeDef result;
  if (!data || !length || length > SPI_FLASH_PAGE || address > SPI_FLASH_SIZE - length ||
      (address % SPI_FLASH_PAGE) + length > SPI_FLASH_PAGE) return HAL_ERROR;
  result = WriteEnable();
  if (result != HAL_OK) return result;
  Address(0x02U, address);
  memcpy(s_tx + 4, data, length);
  return Transfer((uint16_t)(length + 4U));
}

HAL_StatusTypeDef SPIFlash_EraseSector(uint32_t address)
{
  HAL_StatusTypeDef result;
  if (address >= SPI_FLASH_SIZE || address % SPI_FLASH_SECTOR) return HAL_ERROR;
  result = WriteEnable();
  if (result != HAL_OK) return result;
  Address(0x20U, address);
  return Transfer(4U);
}
