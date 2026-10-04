#include "app_rx.h"
#include "main.h"
#include <string.h>

/* 固定内存、不在ISR分配；7个可用槽，每端口最多1792字节积压。 */
#define RX_SLOTS 8U
static AppRxPacket packets[APP_RX_PORTS][RX_SLOTS];
static volatile uint8_t reads[APP_RX_PORTS], writes[APP_RX_PORTS], faults[APP_RX_PORTS];
static volatile uint32_t epochs[APP_RX_PORTS];
volatile uint32_t app_rx_overflows[APP_RX_PORTS];

uint8_t APP_RX_Push(uint8_t port, const uint8_t *data, uint16_t size, uint32_t tick)
{
  uint8_t next;
  if (port >= APP_RX_PORTS || !data || !size) return 0U;
  next = (uint8_t)((writes[port] + 1U) % RX_SLOTS);
  if (size > APP_RX_PACKET_SIZE || faults[port] || next == reads[port]) {
    if (!faults[port]) ++app_rx_overflows[port];
    faults[port] = 1U;
    return 0U;
  }
  packets[port][writes[port]].tick = tick;
  packets[port][writes[port]].epoch = epochs[port];
  packets[port][writes[port]].size = size;
  memcpy(packets[port][writes[port]].data, data, size);
  __DMB();
  writes[port] = next;
  return 1U;
}

uint8_t APP_RX_Take(uint8_t port, AppRxPacket *packet)
{
  uint32_t mask;
  uint8_t available;
  if (port >= APP_RX_PORTS || !packet) return 0U;
  mask = __get_PRIMASK(); __disable_irq();
  available = (uint8_t)(!faults[port] && reads[port] != writes[port]);
  if (available) {
    *packet = packets[port][reads[port]];
    reads[port] = (uint8_t)((reads[port] + 1U) % RX_SLOTS);
  }
  if (!mask) __enable_irq();
  return available;
}

void APP_RX_Reset(uint8_t port)
{
  uint32_t mask;
  if (port >= APP_RX_PORTS) return;
  mask = __get_PRIMASK(); __disable_irq();
  reads[port] = writes[port]; faults[port] = 0U; ++epochs[port];
  if (!mask) __enable_irq();
}

uint8_t APP_RX_Fault(uint8_t port)
{
  return port < APP_RX_PORTS ? faults[port] : 1U;
}

uint8_t APP_RX_Pending(void)
{
  uint8_t i;
  for (i = 0U; i < APP_RX_PORTS; ++i)
    if (faults[i] || reads[i] != writes[i]) return 1U;
  return 0U;
}

uint32_t APP_RX_Epoch(uint8_t port)
{
  return port < APP_RX_PORTS ? epochs[port] : 0U;
}
