#ifndef ILHC_APP_RX_H
#define ILHC_APP_RX_H

#include <stdint.h>

/* 队列保存接收时刻，不能用稍后解析的时刻刷新主机/OPS在线状态。 */
#define APP_RX_PACKET_SIZE 256U
enum { APP_RX_HOST, APP_RX_OPS, APP_RX_PORTS };
typedef struct {
  uint32_t tick;
  uint32_t epoch;
  uint16_t size;
  uint8_t data[APP_RX_PACKET_SIZE];
} AppRxPacket;
extern volatile uint32_t app_rx_overflows[APP_RX_PORTS];
uint8_t APP_RX_Push(uint8_t port, const uint8_t *data, uint16_t size, uint32_t tick);
uint8_t APP_RX_Take(uint8_t port, AppRxPacket *packet);
void APP_RX_Reset(uint8_t port);
uint8_t APP_RX_Fault(uint8_t port);
uint8_t APP_RX_Pending(void);
uint32_t APP_RX_Epoch(uint8_t port);

#endif
