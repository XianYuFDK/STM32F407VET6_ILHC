/* 主机回归测试专用 HAL 替身，不加入固件包含路径。 */
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
#define __weak __attribute__((weak))
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {void *Instance; int State;} CAN_HandleTypeDef;
typedef struct {uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime;} CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
typedef struct {uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow,
FilterFIFOAssignment, FilterBank, FilterMode, FilterScale, FilterActivation, SlaveStartFilterBank;} CAN_FilterTypeDef;
#define CAN1 ((void *)1)
#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_FILTER_FIFO0 0U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define ENABLE 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_RX_FIFO0 0U
#define HAL_CAN_STATE_LISTENING 2
extern CAN_HandleTypeDef hcan1;
extern uint32_t test_primask;
static inline uint32_t __get_PRIMASK(void) {return test_primask;}
static inline void __disable_irq(void) {test_primask=1;}
static inline void __enable_irq(void) {test_primask=0;}
uint32_t HAL_GetTick(void);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *t, uint8_t *d, uint32_t *m);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t n);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t f, CAN_RxHeaderTypeDef *r, uint8_t *d);
typedef struct {uint32_t Pin, Mode, Pull, Speed;} GPIO_InitTypeDef;
#define GPIOB ((void *)2)
#define GPIOC ((void *)3)
#define GPIOE ((void *)4)
#define GPIO_PIN_13 (1U<<13)
#define GPIO_PIN_3 (1U<<3)
#define GPIO_PIN_2 (1U<<2)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIO_MODE_OUTPUT_PP 0
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_HIGH 0
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
static inline void HAL_GPIO_WritePin(void *p,uint32_t n,int s){(void)p;(void)n;(void)s;}
static inline void HAL_GPIO_Init(void *p,GPIO_InitTypeDef *s){(void)p;(void)s;}
static inline void HAL_Delay(uint32_t t){(void)t;}
#endif
