"""编译真实参数状态机，注入逐字节掉电、擦除中断及芯片故障。"""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
HEADER = r'''
#include <stdint.h>
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT} HAL_StatusTypeDef;
uint32_t HAL_GetTick(void);
'''
TEST = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "debug_param_store.h"
#include "spi_flash.h"
#define BASE (SPI_FLASH_SIZE-8192U)
static uint8_t flash[8192], backup[8192];
static uint32_t tick, busy_until, programs, erases;
static int offline, stuck, io_error, torn_body=-1, torn_commit=-1, torn_erase=-1;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef SPIFlash_Init(void) { return offline ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef SPIFlash_Ready(void) {
 if(io_error) return HAL_ERROR;
 return stuck || tick < busy_until ? HAL_BUSY : HAL_OK;
}
HAL_StatusTypeDef SPIFlash_Read(uint32_t a,void *p,uint16_t n) {
 assert(a>=BASE && a+n<=SPI_FLASH_SIZE);
 if(io_error) return HAL_ERROR;
 memcpy(p,flash+a-BASE,n); return HAL_OK;
}
HAL_StatusTypeDef SPIFlash_Program(uint32_t a,const void *p,uint16_t n) {
 const uint8_t *bytes=p; uint32_t count=n;
 assert(a>=BASE && a+n<=SPI_FLASH_SIZE && a/256==(a+n-1)/256);
 if(io_error) return HAL_ERROR;
 programs++;
 if(n==84 && torn_body>=0) count=(uint32_t)torn_body;
 if(n==4 && torn_commit>=0) count=(uint32_t)torn_commit;
 for(uint32_t i=0;i<count;i++) flash[a-BASE+i]&=bytes[i];
 busy_until=tick+3;return HAL_OK;
}
HAL_StatusTypeDef SPIFlash_EraseSector(uint32_t a) {
 assert(a>=BASE && a+4096<=SPI_FLASH_SIZE && !(a%4096));
 if(io_error) return HAL_ERROR;
 erases++;memset(flash+a-BASE,255,torn_erase>=0?(unsigned)torn_erase:4096U);
 busy_until=tick+100;return HAL_OK;
}
static DebugParamValues defaults(void) {
 DebugParamValues v={{2.3f,2.3f,9,1600,750,0,0,60,-50,1,1,0,0,2,.5f,0}};
 return v;
}
static ParamStoreResult boot(DebugParamValues *v) {
 *v=defaults();tick+=1100;stuck=0;io_error=0;torn_body=torn_commit=torn_erase=-1;
 return DebugParamStore_Init(v);
}
static ParamStoreResult advance(DebugParamValues *v, unsigned n) {
 ParamStoreResult result=PARAM_STORE_PENDING;
 while(n--) {tick+=20;result=DebugParamStore_Service(v,tick);
   if(result==PARAM_STORE_ERROR || result==PARAM_STORE_SAVED) return result;}
 return result;
}
int main(void) {
 DebugParamValues v,loaded;ParamStoreResult result;unsigned before;
 memset(flash,255,sizeof(flash));assert(boot(&v)==PARAM_STORE_DEFAULTS);
 assert(advance(&v,200)==PARAM_STORE_DEFAULTS && !programs && !erases);
 v.value[0]=3;assert(advance(&v,100)==PARAM_STORE_PENDING && !programs);
 assert(advance(&v,20)==PARAM_STORE_SAVED);
 assert(boot(&loaded)==PARAM_STORE_LOADED && loaded.value[0]==3);
 before=programs;assert(advance(&loaded,200)==PARAM_STORE_DEFAULTS && programs==before);
 memcpy(backup,flash,sizeof(flash));
 for(int cut=0;cut<84;cut++) {
   memcpy(flash,backup,sizeof(flash));assert(boot(&v)==PARAM_STORE_LOADED);
   v.value[0]=4;torn_body=cut;assert(advance(&v,140)==PARAM_STORE_ERROR);
   assert(boot(&loaded)==PARAM_STORE_LOADED && loaded.value[0]==3);
 }
 for(int cut=0;cut<=4;cut++) {
   memcpy(flash,backup,sizeof(flash));boot(&v);v.value[0]=4;torn_commit=cut;
   result=advance(&v,140);
   assert(result==(cut==4?PARAM_STORE_SAVED:PARAM_STORE_ERROR));
   assert(boot(&loaded)==PARAM_STORE_LOADED && loaded.value[0]==(cut==4?4:3));
 }
 for(unsigned cut=0;cut<110;cut++) {
   memcpy(flash,backup,sizeof(flash));boot(&v);v.value[0]=4;
   advance(&v,cut);assert(boot(&loaded)==PARAM_STORE_LOADED);
   assert(loaded.value[0]==3 || loaded.value[0]==4);
 }
 memcpy(flash,backup,sizeof(flash));boot(&v);before=erases;
 for(unsigned i=0;i<40;i++) {v.value[0]=(float)(i+5);assert(advance(&v,160)==PARAM_STORE_SAVED);}
 assert(erases-before==2);
 v.value[7]=-100;v.value[8]=90;v.value[9]=7;v.value[10]=2;
 v.value[11]=1.2f;v.value[12]=-4;v.value[13]=5;v.value[14]=1;v.value[15]=2;
 assert(advance(&v,160)==PARAM_STORE_SAVED);
 boot(&loaded);assert(!memcmp(&v,&loaded,sizeof(v)));
 for(unsigned i=0;i<32;i++) {
   uint32_t seq;memcpy(&seq,flash+i*256+8,4);
   if(seq==42) flash[i*256+16]^=1;
 }
 boot(&loaded);assert(loaded.value[7]==60);
 memset(flash,255,sizeof(flash));boot(&v);
 for(unsigned i=1;i<=32;i++) {v.value[0]=(float)i;assert(advance(&v,160)==PARAM_STORE_SAVED);}
 memcpy(backup,flash,sizeof(flash));
 for(int cut=0;cut<4096;cut+=127) {
   memcpy(flash,backup,sizeof(flash));boot(&v);v.value[0]=33;torn_erase=cut;
   advance(&v,102); /* 擦除已经开始，控制任务尚未开始编程，直接掉电。 */
   assert(boot(&loaded)==PARAM_STORE_LOADED && loaded.value[0]==32);
 }
 memcpy(flash,backup,sizeof(flash));boot(&v);v.value[0]=33;
 advance(&v,102);stuck=1;assert(advance(&v,60)==PARAM_STORE_ERROR);
 assert(boot(&loaded)==PARAM_STORE_LOADED && loaded.value[0]==32);
 offline=1;loaded=defaults();assert(DebugParamStore_Init(&loaded)==PARAM_STORE_ERROR);
 assert(loaded.value[0]==2.3f);offline=0;
 v=defaults();v.value[0]=NAN;assert(!DebugParamStore_Valid(&v));
 v=defaults();v.value[0]=INFINITY;assert(!DebugParamStore_Valid(&v));
 v=defaults();v.value[9]=1.5f;assert(!DebugParamStore_Valid(&v));
 v=defaults();v.value[10]=3;assert(!DebugParamStore_Valid(&v));
 /* 合成有效v1记录，确认仅DM地址迁移为3，且落盘后不再覆盖新的DMID。 */
 memset(flash,255,sizeof(flash));boot(&v);v.value[0]=7;v.value[7]=88;v.value[9]=1;
 assert(advance(&v,160)==PARAM_STORE_SAVED);
 { uint32_t version=1,crc=0xFFFFFFFFU;
   memcpy(flash+4,&version,4);
   for(unsigned j=0;j<80;j++){crc^=flash[j];for(unsigned k=0;k<8;k++)crc=(crc>>1)^((crc&1)?0xEDB88320U:0);}
   crc=~crc;memcpy(flash+80,&crc,4);
 }
 assert(boot(&loaded)==PARAM_STORE_LOADED);
 assert(loaded.value[9]==3 && loaded.value[0]==7 && loaded.value[7]==88);
 assert(advance(&loaded,160)==PARAM_STORE_SAVED);
 assert(boot(&v)==PARAM_STORE_LOADED && v.value[9]==3);
 v.value[9]=5;assert(advance(&v,160)==PARAM_STORE_SAVED);
 assert(boot(&loaded)==PARAM_STORE_LOADED && loaded.value[9]==5);
 puts("PASS: debounce, fields, rollover, power cuts, CRC, faults, v1 address migration");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='ilhc_param_store_') as directory:
    p = Path(directory)
    (p/'main.h').write_text(HEADER, encoding='utf-8')
    (p/'test.c').write_text(TEST, encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(p),
                    '-I',str(ROOT/'Hardware'),str(ROOT/'Hardware/debug_param_store.c'),
                    str(p/'test.c'),'-o',str(p/'test.exe')],check=True)
    subprocess.run([str(p/'test.exe')],check=True)
