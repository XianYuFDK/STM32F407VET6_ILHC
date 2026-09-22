"""编译真实SPI驱动，模拟HAL事务校验JEDEC、命令、页边界和故障传播。"""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
HEADER = r'''
#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
typedef struct {void *Instance;struct {uint32_t Mode,Direction,DataSize,CLKPolarity,
 CLKPhase,NSS,BaudRatePrescaler,FirstBit,TIMode,CRCCalculation,CRCPolynomial;} Init;} SPI_HandleTypeDef;
#define GPIOA ((void*)1)
#define SPI1 ((void*)2)
#define GPIO_PIN_4 16
#define GPIO_PIN_5 32
#define GPIO_PIN_6 64
#define GPIO_PIN_7 128
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_AF_PP 2
#define GPIO_PULLUP 1
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_HIGH 2
#define GPIO_AF5_SPI1 5
#define SPI_MODE_MASTER 1
#define SPI_DIRECTION_2LINES 0
#define SPI_DATASIZE_8BIT 0
#define SPI_POLARITY_LOW 0
#define SPI_PHASE_1EDGE 0
#define SPI_NSS_SOFT 1
#define SPI_BAUDRATEPRESCALER_16 16
#define SPI_FIRSTBIT_MSB 0
#define SPI_TIMODE_DISABLE 0
#define SPI_CRCCALCULATION_DISABLE 0
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI1_CLK_ENABLE() ((void)0)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t n);
void HAL_GPIO_WritePin(void*,uint32_t,uint32_t);
void HAL_GPIO_Init(void*,GPIO_InitTypeDef*);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef*);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t*,uint8_t*,uint16_t,uint32_t);
#endif
'''
TEST = r'''
#include "spi_flash.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static uint32_t tick, id=0xEF4018, address, calls, programs, erases;
static uint8_t status, memory[4096];
static int cs=1, fail, no_wel;
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t n){tick+=n;}
void HAL_GPIO_WritePin(void *port,uint32_t pin,uint32_t level){assert(port==GPIOA && pin==16);cs=(int)level;}
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *p){assert(port==GPIOA);assert(p->Pin==16 || p->Pin==224);}
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *p){assert(p->Instance==SPI1 && p->Init.BaudRatePrescaler==16);return HAL_OK;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *p,uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t timeout){
 (void)p;assert(!cs && timeout==2 && n<=260);calls++;memset(rx,0,n);
 if(fail)return HAL_TIMEOUT;
 switch(tx[0]){
 case 0xAB:break;
 case 0x9F:assert(n==4);rx[1]=(uint8_t)(id>>16);rx[2]=(uint8_t)(id>>8);rx[3]=(uint8_t)id;break;
 case 0x05:assert(n==2);rx[1]=status;break;
 case 0x06:assert(n==1);if(!no_wel)status|=2;break;
 case 0x03:case 0x02:case 0x20:
   address=((uint32_t)tx[1]<<16)|((uint32_t)tx[2]<<8)|tx[3];
   if(tx[0]==0x03)memcpy(rx+4,memory+(address%4096),n-4);
   if(tx[0]==0x02){assert(status&2);programs++;memcpy(memory+(address%4096),tx+4,n-4);status=1;}
   if(tx[0]==0x20){assert(status&2);erases++;memset(memory,255,4096);status=1;}
   break;
 default:assert(0);
 }return HAL_OK;
}
int main(void){
 uint8_t data[256],out[256];unsigned before;
 memset(data,0xAC,sizeof(data));assert(SPIFlash_Init()==HAL_OK && spi_flash_jedec_id==id && cs);
 assert(SPIFlash_Program(0xFFE000,data,256)==HAL_OK && programs==1 && address==0xFFE000 && cs);
 assert(SPIFlash_Ready()==HAL_BUSY);before=programs;
 assert(SPIFlash_Program(0xFFE100,data,1)==HAL_BUSY && programs==before);
 status=0;assert(SPIFlash_Read(0xFFE000,out,256)==HAL_OK && !memcmp(data,out,256));
 before=calls;assert(SPIFlash_Program(255,data,2)==HAL_ERROR && calls==before);
 assert(SPIFlash_Program(SPI_FLASH_SIZE,data,1)==HAL_ERROR);
 assert(SPIFlash_Read(SPI_FLASH_SIZE-1,out,2)==HAL_ERROR);
 assert(SPIFlash_Read(0,out,257)==HAL_ERROR);
 assert(SPIFlash_EraseSector(1)==HAL_ERROR);
 assert(SPIFlash_EraseSector(0xFFE000)==HAL_OK && erases==1 && cs);
 status=0;no_wel=1;assert(SPIFlash_Program(0,data,1)==HAL_ERROR && programs==1);no_wel=0;
 fail=1;assert(SPIFlash_Read(0,out,1)==HAL_ERROR && cs);fail=0;
 status=255;assert(SPIFlash_Ready()==HAL_ERROR);status=0;
 id=0xFFFFFF;assert(SPIFlash_Init()==HAL_ERROR);before=calls;
 assert(SPIFlash_EraseSector(0)==HAL_ERROR && calls==before);
 id=0xEF7018;assert(SPIFlash_Init()==HAL_OK);
 status=1;before=tick;assert(SPIFlash_Init()==HAL_TIMEOUT && tick-before<=1001);
 puts("PASS: SPI1 configuration, JEDEC, commands, bounds, busy, WEL, timeout, CS release");return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='ilhc_spi_flash_') as directory:
    p=Path(directory)
    (p/'main.h').write_text(HEADER,encoding='utf-8')
    (p/'test.c').write_text(TEST,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(p),
                    '-I',str(ROOT/'Hardware'),str(ROOT/'Hardware/spi_flash.c'),
                    str(p/'test.c'),'-o',str(p/'test.exe')],check=True)
    subprocess.run([str(p/'test.exe')],check=True)
