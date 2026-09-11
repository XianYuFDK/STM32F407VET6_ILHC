#ifndef __SOFTSPI_OLED_H
#define __SOFTSPI_OLED_H

#include "main.h"
#include "stdint.h"
#include "stdlib.h"

// 引脚定义（需与CubeMX配置一致）
#define SOFTSPI_OLED_SCL_PORT    GPIOB
#define SOFTSPI_OLED_SCL_PIN     GPIO_PIN_13
#define SOFTSPI_OLED_SDA_PORT    GPIOC
#define SOFTSPI_OLED_SDA_PIN     GPIO_PIN_3
#define SOFTSPI_OLED_RES_PORT    GPIOE
#define SOFTSPI_OLED_RES_PIN     GPIO_PIN_2
#define SOFTSPI_OLED_DC_PORT     GPIOE
#define SOFTSPI_OLED_DC_PIN      GPIO_PIN_3
#define SOFTSPI_OLED_CS_PORT     GPIOE
#define SOFTSPI_OLED_CS_PIN      GPIO_PIN_4

// 引脚操作宏定义（HAL库风格）
#define SoftSPI_OLED_SCL_Clr()   HAL_GPIO_WritePin(SOFTSPI_OLED_SCL_PORT, SOFTSPI_OLED_SCL_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_SCL_Set()   HAL_GPIO_WritePin(SOFTSPI_OLED_SCL_PORT, SOFTSPI_OLED_SCL_PIN, GPIO_PIN_SET)
#define SoftSPI_OLED_SDA_Clr()   HAL_GPIO_WritePin(SOFTSPI_OLED_SDA_PORT, SOFTSPI_OLED_SDA_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_SDA_Set()   HAL_GPIO_WritePin(SOFTSPI_OLED_SDA_PORT, SOFTSPI_OLED_SDA_PIN, GPIO_PIN_SET)
#define SoftSPI_OLED_RES_Clr()   HAL_GPIO_WritePin(SOFTSPI_OLED_RES_PORT, SOFTSPI_OLED_RES_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_RES_Set()   HAL_GPIO_WritePin(SOFTSPI_OLED_RES_PORT, SOFTSPI_OLED_RES_PIN, GPIO_PIN_SET)
#define SoftSPI_OLED_DC_Clr()    HAL_GPIO_WritePin(SOFTSPI_OLED_DC_PORT, SOFTSPI_OLED_DC_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_DC_Set()    HAL_GPIO_WritePin(SOFTSPI_OLED_DC_PORT, SOFTSPI_OLED_DC_PIN, GPIO_PIN_SET)
#define SoftSPI_OLED_CS_Clr()    HAL_GPIO_WritePin(SOFTSPI_OLED_CS_PORT, SOFTSPI_OLED_CS_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_CS_Set()    HAL_GPIO_WritePin(SOFTSPI_OLED_CS_PORT, SOFTSPI_OLED_CS_PIN, GPIO_PIN_SET)

// 命令/数据定义
#define SOFTSPI_OLED_CMD  0
#define SOFTSPI_OLED_DATA 1

// 全局显存数组
extern uint8_t SoftSPI_OLED_GRAM[144][8];

// 函数声明
/**
 * @brief 设置整屏反显：i=0 正常、i=1 反色；其他值不发送。立即写控制器，不改变显存。
 */
void SoftSPI_OLED_ColorTurn(uint8_t i);
/**
 * @brief 设置屏幕方向：i=0 原方向、i=1 旋转180度；其他值忽略。立即生效，不变更逻辑坐标。
 */
void SoftSPI_OLED_DisplayTurn(uint8_t i);
/**
 * @brief 发送一个字节。dat 为原始数据，cmd=0 命令、非0 数据；高位先发。
 * 同步软件 SPI，无错误应答；只能在初始化 GPIO 后由单一任务串行调用。
 */
void SoftSPI_OLED_WR_Byte(uint8_t dat, uint8_t cmd);
/**
 * @brief 向控制器写电荷泵与显示开启命令，不修改显存。控制序列沿用原工程，需匹配屏幕。
 */
void SoftSPI_OLED_DisPlay_On(void);
/**
 * @brief 关闭显示和电荷泵，不清除 RAM 显存；再次开启后可 Refresh 恢复图像。
 */
void SoftSPI_OLED_DisPlay_Off(void);
/**
 * @brief 将显存前128列、8页发送至屏幕，后16列是滚动暂存区。
 * 同步发送1024字节像素及页命令，无 DMA、无互斥；同一任务低频刷新。
 */
void SoftSPI_OLED_Refresh(void);
/**
 * @brief 清零全部144×8字节显存，重置滚动位置并立即刷新。不要在中断中调用。
 */
void SoftSPI_OLED_Clear(void);
/**
 * @brief 修改显存像素。x=0..143（128..143为暂存区），y=0..63，t非0置位、0清零。
 * 越界忽略；只修改显存，不自动刷新。
 */
void SoftSPI_OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
/**
 * @brief 从(x1,y1)到(x2,y2)绘制含端点直线，mode非0置位、0清零。
 * 坐标为像素，越出显存范围的点忽略；同点起终点绘制单点，不自动刷新。
 */
void SoftSPI_OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode);
/**
 * @brief 以(x,y)为中心绘制半径r的圆轮廓，r=0绘制单点。
 * 仅绘制可见128×64区域，边缘使用有符号坐标裁剪；不自动刷新。
 */
void SoftSPI_OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);
/**
 * @brief 显示一个ASCII可打印字符（空格至~）。x/y为像素，size1=8/12/16/24。
 * 对应字框6×8、6×12、8×16、12×24；mode=1正常、0反色。
 * 不支持字符或字号时忽略，超出显存部分裁剪，不自动刷新。
 */
void SoftSPI_OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode);
/**
 * @brief 显示以零结尾的ASCII字符串，遇非可打印字符或右边界停止，不换行。
 * chr不可为空；x/y为像素，size1与mode同ShowChar；不解析UTF-8，不自动刷新。
 */
void SoftSPI_OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode);
/**
 * @brief 计算m的n次方，n=0返回1。结果为uint32_t，超范围按无符号整数回绕。
 * 数字显示内部仅使用10的0..9次方，调用方不要将其当作任意精度运算。
 */
uint32_t SoftSPI_OLED_Pow(uint8_t m, uint8_t n);
/**
 * @brief 显示无符号整数。len=1..10，位数不足补前导零，超过len仅显示低len位。
 * x/y为像素，字号与模式同ShowChar；超出可见右边界停止，不自动刷新。
 */
void SoftSPI_OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode);
/**
 * @brief 按字模索引显示汉字，非UTF-8接口。size1=16/24/32/64，num为对应Hzk1..4索引。
 * mode=1正常、0反色；索引越界忽略；x可到143以供滚动预绘制，不自动刷新。
 */
void SoftSPI_OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode);
/* 单次移动一列，任务中周期调用；与原版无限循环行为不同。 */
/**
 * @brief 将前num个16×16字模向左滚动一列，并立即刷新。space为间隔的16列块数量。
 * 由任务定期调用控制速度；num必须大于0且不超过Hzk1字数。
 * 单实例状态；清屏或改变num/space/mode后从首个字模重新开始。
 */
void SoftSPI_OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode);
/**
 * @brief 按页优先、每字节竖向8像素、低位在上的格式绘制位图。
 * x/y为像素；sizex/sizey为实际宽高，调用方保证BMP至少有sizex*ceil(sizey/8)字节。
 * 空指针或矩形超出128×64时忽略；末页填充位不绘制。mode=1正常、0反色，不自动刷新。
 */
void SoftSPI_OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode);
/**
 * @brief 配置PB13/PC3/PE2/PE3/PE4输出、硬件复位、发送原控制器初始化序列并清屏。
 * 依赖HAL时基可用，含约220ms阻塞等待；只在启动或显式重新初始化时调用。
 */
void SoftSPI_OLED_Init(void);

/**
 * @brief 显示原字库的16×32符号（加号与数字），num为每四行Hzk组成的符号索引。
 * x/y为像素，mode=0采用原阴码反转、mode=1显示原字模位；索引越界忽略，不自动刷新。
 */
void SoftSPI_OLED_ShowBN(uint8_t x, uint8_t y, uint8_t num, uint8_t mode);

#endif
