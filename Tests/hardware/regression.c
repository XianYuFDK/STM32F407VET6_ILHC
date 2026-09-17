/* 直接编译实际驱动，以 HAL 替身检查报文和显存，不访问硬件。 */
#include "stepper_2835.h"
#include "OLED_SoftSPI.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan2 = {CAN2, HAL_CAN_STATE_LISTENING};
uint32_t test_primask;
static uint32_t free_slots=3, sent;
static uint8_t bytes[3][8];
static CAN_TxHeaderTypeDef headers[3];
static uint32_t rx_calls;
static HAL_StatusTypeDef rx_status = HAL_ERROR;
static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
uint32_t HAL_GetTick(void){return 1234;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h){(void)h;return free_slots;}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,CAN_TxHeaderTypeDef *t,uint8_t *d,uint32_t *m)
{
    (void)h; assert(test_primask==1); assert(free_slots && sent<3);
    headers[sent]=*t; memcpy(bytes[sent],d,8); *m=sent++; --free_slots; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h,CAN_FilterTypeDef *f){(void)h;(void)f;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h,uint32_t n){(void)h;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h){h->State=HAL_CAN_STATE_LISTENING;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h,uint32_t f,CAN_RxHeaderTypeDef *r,uint8_t *d)
{
    (void)h;(void)f;
    rx_calls++;
    if (rx_status == HAL_OK)
    {
        *r = rx_header;
        memcpy(d, rx_data, sizeof(rx_data));
    }
    return rx_status;
}
static void reset_bus(void){free_slots=3;sent=0;test_primask=0;hcan2.State=HAL_CAN_STATE_LISTENING;}
static unsigned pixel(unsigned x,unsigned y){return (SoftSPI_OLED_GRAM[x][y/8]>>(y%8))&1;}
int main(void)
{
    const uint8_t expected[16]={0xFD,0,0xAF,0xFF,0xAF,0xFF,3,0xE8,0xFD,0,0,3,0xE8,1,0,0x6B};
    uint8_t short_data[4]={0x9A,2,0,0x6B}, picture=0xFF;
    Stepper2835Reply_t reply;
    CAN_HandleTypeDef not_can2 = {(void *)1, HAL_CAN_STATE_LISTENING};

    memset(&rx_header, 0, sizeof(rx_header));
    memset(rx_data, 0x5A, sizeof(rx_data));
    rx_header.StdId = 0x123U;
    rx_header.IDE = CAN_ID_STD;
    rx_header.RTR = CAN_RTR_DATA;
    rx_header.DLC = 8U;
    HCan_ClearRxFlag();
    rx_calls = 0U;
    rx_status = HAL_OK;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    assert(rx_calls == 1U && hcanRxFlag == 1U);
    assert(hcanRxFrame.StdId == 0x123U && hcanRxFrame.Data[0] == 0x5A);
    HCan_ClearRxFlag();
    rx_calls = 0U;
    HAL_CAN_RxFifo0MsgPendingCallback(&not_can2);
    assert(rx_calls == 0U && hcanRxFlag == 0U);
    rx_status = HAL_ERROR;

    reset_bus();
    assert(Motor_AbsPosition(0,MOTOR35_CAN_ID,1000,1000)==HAL_OK);
    assert(sent==2 && headers[0].ExtId==0x300 && headers[1].ExtId==0x301);
    assert(!memcmp(bytes[0],expected,8) && !memcmp(bytes[1],expected+8,8));
    assert(headers[0].DLC==8 && test_primask==0);
    reset_bus();free_slots=1;
    assert(Motor_AbsPosition(0,MOTOR35_CAN_ID,1,1)==HAL_BUSY && sent==0);
    reset_bus();hcan2.State=0;
    assert(Motor_AbsPosition(0,MOTOR35_CAN_ID,1,1)==HAL_ERROR && sent==0);
    reset_bus();test_primask=1;
    assert(Motor_Homing(MOTOR28_CAN_ID)==HAL_OK && test_primask==1);
    assert(headers[0].DLC==4 && !memcmp(bytes[0],short_data,4));
    assert(bytes[0][4]==0 && bytes[0][7]==0);
    reset_bus();
    assert(Can_SendCmd(0x400,short_data,4)==HAL_OK && headers[0].DLC==4);
    reset_bus();
    assert(Can_SendCmd(0x1FFFFFFF,expected,16)==HAL_ERROR && sent==0);
    assert(Motor35_AbsPosition(0,2185)==HAL_ERROR && sent==0);
    assert(Motor_AbsPosition(2,0x300,0,1)==HAL_ERROR);
    assert(Stepper2835_OnRx(0x400,short_data,4)==1);
    assert(Stepper2835_GetReply(0x400,&reply)==1 && reply.last_tick==1234 && reply.count==1);
    assert(Stepper2835_OnRx(0x400,short_data,9)==0);

    memset(SoftSPI_OLED_GRAM,0,sizeof(SoftSPI_OLED_GRAM));
    SoftSPI_OLED_ShowPicture(0,0,1,1,&picture,1);
    assert(pixel(0,0)==1 && pixel(0,1)==0 && pixel(0,7)==0);
    memset(SoftSPI_OLED_GRAM,0xFF,sizeof(SoftSPI_OLED_GRAM));
    SoftSPI_OLED_ShowChar(0,0,' ',12,1);
    assert(pixel(0,11)==0 && pixel(0,12)==1 && pixel(0,15)==1);
    SoftSPI_OLED_Clear();
    SoftSPI_OLED_DrawCircle(0,0,0);assert(pixel(0,0));
    SoftSPI_OLED_DrawLine(2,2,4,2,1);assert(pixel(2,2)&&pixel(3,2)&&pixel(4,2));
    SoftSPI_OLED_DrawPoint(255,255,1);
    SoftSPI_OLED_ShowChinese(0,0,255,64,1);
    SoftSPI_OLED_ScrollDisplay(2,1,1);
    SoftSPI_OLED_Clear();
    SoftSPI_OLED_ScrollDisplay(2,1,1);
    puts("CAN / stepper / OLED regression passed");
    return 0;
}
