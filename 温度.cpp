/*****************************************************************************
 * 文件名：main.c
 * 描述：DHT11 温湿度传感器读取实验，基于 STM32F407 标准外设库
 * 说明：使用 GPIOI_Pin_6 作为单总线，读取 DHT11 温湿度数据并通过串口打印
 * 版本：V1.0
 *****************************************************************************/

#include "stm32f4xx.h"
#include <stdio.h>

/* =====================================================
 * 1. 调试串口初始化（USART2, PA2-TX, PA3-RX）
 * ===================================================== */

static void UART2_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    USART_Cmd(USART2, ENABLE);
}

/* printf 重定向到 USART2 */
int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    return ch;
}

/* =====================================================
 * 2. 软件延时函数（微秒 / 毫秒）
 * ===================================================== */

static void delay_us(uint32_t nus)
{
    uint32_t i;
    while (nus--) {
        i = 31;
        while (i--);
    }
}

static void delay_ms(uint32_t nms)
{
    uint32_t i;
    while (nms--) {
        i = 33800;
        while (i--);
    }
}

/* =====================================================
 * 3. DHT11 温湿度传感器驱动（GPIOI_Pin_6）
 * ===================================================== */

#define DHT11_PORT      GPIOI
#define DHT11_PIN       GPIO_Pin_6

/* 温湿度数据结构体 */
typedef struct {
    uint8_t humi_int;      // 湿度整数部分
    uint8_t humi_deci;     // 湿度小数部分
    uint8_t temp_int;      // 温度整数部分
    uint8_t temp_deci;     // 温度小数部分
    uint8_t check_sum;     // 校验和
} DHT11_Data;

/* 3.1 GPIO 方向切换函数 */
static void DHT11_Mode_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin = DHT11_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

static void DHT11_Mode_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin = DHT11_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/* 3.2 等待指定电平（带超时） */
static int8_t DHT11_WaitFor(BitAction bitValue, uint32_t max_us)
{
    while (GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN) != bitValue) {
        if (max_us-- == 0) {
            return -1;
        }
        delay_us(1);
    }
    return 0;
}

/* 3.3 读取一个数据位（返回 0 或 1） */
static int8_t DHT11_ReadBit(void)
{
    /* 等待电平变低（数据位开始传输） */
    if (DHT11_WaitFor(Bit_RESET, 100) < 0) {
        return -1;
    }

    /* 等待电平变高（进入数据位区分时段） */
    if (DHT11_WaitFor(Bit_SET, 100) < 0) {
        return -1;
    }

    /* 延时 40us，区分数据 0 和 1 */
    delay_us(40);

    /* 40us 后仍为高电平 → 数据 1；否则为数据 0 */
    if (GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN) == Bit_SET) {
        return 1;
    } else {
        return 0;
    }
}

/* 3.4 读取一个字节（8 个数据位） */
static uint8_t DHT11_ReadByte(void)
{
    uint8_t i, byte = 0;

    for (i = 0; i < 8; i++) {
        int8_t bit = DHT11_ReadBit();
        if (bit < 0) {
            return 0;
        }
        byte <<= 1;
        byte |= bit;
    }
    return byte;
}

/* 3.5 DHT11 初始化 */
static void DHT11_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

    DHT11_Mode_OUT();
    GPIO_SetBits(DHT11_PORT, DHT11_PIN);
}

/* 3.6 读取完整的温湿度数据（一次完整传输共 40 bit） */
static int8_t DHT11_ReadData(DHT11_Data *pData)
{
    if (pData == NULL) {
        return -1;
    }

    /* ----- 主机发送起始信号 ----- */
    DHT11_Mode_OUT();
    GPIO_SetBits(DHT11_PORT, DHT11_PIN);
    GPIO_ResetBits(DHT11_PORT, DHT11_PIN);
    delay_ms(18);                    // 拉低保持 18ms
    GPIO_SetBits(DHT11_PORT, DHT11_PIN);
    delay_us(30);                    // 拉高保持 30us

    /* ----- 等待 DHT11 响应 ----- */
    DHT11_Mode_IN();

    /* 等待 DHT11 拉低（响应信号开始） */
    if (DHT11_WaitFor(Bit_RESET, 200) < 0) {
        return -1;
    }

    /* 等待 DHT11 拉高（响应信号结束） */
    if (DHT11_WaitFor(Bit_SET, 200) < 0) {
        return -1;
    }

    /* 延时 80us，跳过响应信号的剩余时间 */
    delay_us(80);

    /* ----- 读取 5 个字节数据 ----- */
    pData->humi_int = DHT11_ReadByte();
    pData->humi_deci = DHT11_ReadByte();
    pData->temp_int = DHT11_ReadByte();
    pData->temp_deci = DHT11_ReadByte();
    pData->check_sum = DHT11_ReadByte();

    /* ----- 校验和验证 ----- */
    if (pData->check_sum !=
        pData->humi_int + pData->humi_deci +
        pData->temp_int + pData->temp_deci) {
        return -1;
    }

    return 0;
}

/* =====================================================
 * 4. 主函数
 * ===================================================== */

int main(void)
{
    DHT11_Data data;

    UART2_Init(115200);
    printf("DHT11 Test Start\r\n");

    DHT11_Init();
    printf("DHT11 Init OK\r\n");

    delay_ms(1000);

    while (1) {
        if (DHT11_ReadData(&data) == 0) {
            printf("HUMI:%d.%d_TEMP:%d.%d\r\n",
                   data.humi_int, data.humi_deci,
                   data.temp_int, data.temp_deci);
        } else {
            printf("Read DHT11 ERROR!\r\n");
        }
        delay_ms(2000);
    }
}