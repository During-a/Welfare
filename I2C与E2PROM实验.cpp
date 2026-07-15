/*****************************************************************************
 * 文件名：main.c
 * 描述：E2PROM (AT24C02) 读写实验，基于 STM32F407 标准外设库
 * 说明：使用 I2C1 接口读写 AT24C02，测试数据写入并读出校验
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
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
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
 * 2. 简单软件延时（微秒/毫秒）
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
    while (nms--) {
        delay_us(1000);
    }
}

/* =====================================================
 * 3. AT24C02 E2PROM 驱动（I2C1, PB6-SCL, PB7-SDA）
 * ===================================================== */

#define AT24C02_I2C          I2C1
#define AT24C02_ADDRESS      0xA0          // 设备地址（写）
#define PAGE_SIZE            8             // 每页字节数
#define PAGE_COUNT           32            // 总页数
#define TOTAL_SIZE           256           // 总字节数
#define I2C_SPEED            100000        // 100KHz

/* 3.1 I2C GPIO 初始化 */
static void I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
}

/* 3.2 I2C 工作模式配置 */
static void I2C_Mode_Config(void)
{
    I2C_InitTypeDef I2C_InitStruct;

    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = I2C_SPEED;
    I2C_Init(AT24C02_I2C, &I2C_InitStruct);

    I2C_Cmd(AT24C02_I2C, ENABLE);
    I2C_AcknowledgeConfig(AT24C02_I2C, ENABLE);
}

/* 3.3 AT24C02 总初始化 */
static void AT24C02_Init(void)
{
    I2C_GPIO_Config();
    I2C_Mode_Config();
}

/* 3.4 等待指定 I2C 事件（超时返回 -1） */
static int8_t AT24C02_WaitI2cEvent(uint32_t I2C_EVENT)
{
    uint32_t timeout = 0x10000;
    while (!I2C_CheckEvent(AT24C02_I2C, I2C_EVENT)) {
        if ((timeout--) == 0) {
            return -1;
        }
    }
    return 0;
}

/* 3.5 等待指定 I2C 标志位（超时返回 -1） */
static int8_t AT24C02_WaitI2cFlag(uint32_t I2C_FLAG, FlagStatus Status)
{
    uint32_t timeout = 0x10000;
    while (I2C_GetFlagStatus(AT24C02_I2C, I2C_FLAG) != Status) {
        if ((timeout--) == 0) {
            return -1;
        }
    }
    return 0;
}

/* 3.6 等待 E2PROM 内部写操作完成 */
static void AT24C02_WaitE2promOK(void)
{
    uint32_t timeout = 100;   // 最多 100ms

    do {
        if ((timeout--) == 0) {
            return;
        }
        delay_ms(1);

        I2C_GenerateSTART(AT24C02_I2C, ENABLE);
        if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_MODE_SELECT) != 0) {
            continue;
        }

        I2C_Send7bitAddress(AT24C02_I2C, AT24C02_ADDRESS, I2C_Direction_Transmitter);
        if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0) {
            break;
        }
    } while (1);

    I2C_ClearFlag(AT24C02_I2C, I2C_FLAG_AF);
    I2C_GenerateSTOP(AT24C02_I2C, ENABLE);
}

/* 3.7 页写入（单页，最多 8 字节） */
static int8_t AT24C02_PageWrite(uint8_t *pBuffer, uint8_t WriteAddr, uint8_t Number)
{
    I2C_GenerateSTART(AT24C02_I2C, ENABLE);
    if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_MODE_SELECT) != 0) {
        return -1;
    }

    I2C_Send7bitAddress(AT24C02_I2C, AT24C02_ADDRESS, I2C_Direction_Transmitter);
    if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != 0) {
        return -1;
    }

    I2C_SendData(AT24C02_I2C, WriteAddr);
    if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != 0) {
        return -1;
    }

    while (Number--) {
        I2C_SendData(AT24C02_I2C, *pBuffer++);
        if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != 0) {
            return -1;
        }
    }

    I2C_GenerateSTOP(AT24C02_I2C, ENABLE);
    AT24C02_WaitE2promOK();
    return 0;
}

/* 3.8 任意长度数据写入（支持跨页） */
static void AT24C02_BufferWrite(uint8_t *pBuffer, uint8_t Addr, uint16_t Number)
{
    uint8_t PageIndex, FirstCount;
    uint8_t PageNum, ByteNum;

    if (Number > (TOTAL_SIZE - Addr)) {
        Number = TOTAL_SIZE - Addr;
    }

    PageIndex = Addr % PAGE_SIZE;
    FirstCount = PAGE_SIZE - PageIndex;
    if (FirstCount > Number) {
        FirstCount = Number;
    }

    if (PageIndex != 0) {
        AT24C02_PageWrite(pBuffer, Addr, FirstCount);
        Addr += FirstCount;
        pBuffer += FirstCount;
        Number -= FirstCount;
    }

    PageNum = Number / PAGE_SIZE;
    ByteNum = Number % PAGE_SIZE;

    while (PageNum > 0) {
        AT24C02_PageWrite(pBuffer, Addr, PAGE_SIZE);
        Addr += PAGE_SIZE;
        pBuffer += PAGE_SIZE;
        PageNum--;
    }

    if (ByteNum > 0) {
        AT24C02_PageWrite(pBuffer, Addr, ByteNum);
    }
}

/* 3.9 任意长度数据读取 */
static int8_t AT24C02_BufferRead(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t Number)
{
    if (Number > (TOTAL_SIZE - ReadAddr)) {
        Number = TOTAL_SIZE - ReadAddr;
    }

    I2C_GenerateSTART(AT24C02_I2C, ENABLE);
    if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_MODE_SELECT) != 0) {
        return -1;
    }

    I2C_Send7bitAddress(AT24C02_I2C, AT24C02_ADDRESS, I2C_Direction_Transmitter);
    if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != 0) {
        return -1;
    }

    I2C_SendData(AT24C02_I2C, ReadAddr);
    if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != 0) {
        return -1;
    }

    I2C_GenerateSTART(AT24C02_I2C, ENABLE);
    if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_MODE_SELECT) != 0) {
        return -1;
    }

    I2C_Send7bitAddress(AT24C02_I2C, AT24C02_ADDRESS | 0x01, I2C_Direction_Receiver);
    if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != 0) {
        return -1;
    }

    while (Number) {
        if (Number == 1) {
            I2C_AcknowledgeConfig(AT24C02_I2C, DISABLE);
            I2C_GenerateSTOP(AT24C02_I2C, ENABLE);
        }

        if (AT24C02_WaitI2cEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) != 0) {
            return -1;
        }
        *pBuffer++ = I2C_ReceiveData(AT24C02_I2C);
        Number--;
    }

    I2C_AcknowledgeConfig(AT24C02_I2C, ENABLE);
    return 0;
}

/* =====================================================
 * 4. 测试数据生成与校验
 * ===================================================== */

#define TEST_SIZE  256
static uint8_t BufRead[TEST_SIZE];
static uint8_t BufWrite[TEST_SIZE];

static void MakeTestData(void)
{
    uint32_t i;
    for (i = 0; i < TEST_SIZE; i++) {
        BufWrite[i] = (uint8_t)i;
        printf("0x%02X ", BufWrite[i]);
        if ((i % 16) == 15) {
            printf("\r\n");
        }
    }
    printf("\r\n");
}

static void CheckTestData(void)
{
    uint32_t i;
    for (i = 0; i < TEST_SIZE; i++) {
        if (BufRead[i] != BufWrite[i]) {
            printf("0x%02X ", BufRead[i]);
            printf("not correct at addr %d\r\n", (int)i);
            return;
        }
        printf("0x%02X ", BufRead[i]);
        if ((i % 16) == 15) {
            printf("\r\n");
        }
    }
    printf("\r\n");
    if (i >= TEST_SIZE) {
        printf("TEST OK\r\n");
    }
}

/* =====================================================
 * 5. 主函数
 * ===================================================== */

int main(void)
{
    UART2_Init(115200);
    printf("E2PROM AT24C02 Test Start\r\n");

    AT24C02_Init();
    printf("AT24C02 Init OK\r\n");

    /* 生成测试数据 */
    printf("Test Data:\r\n");
    MakeTestData();

    /* 写入 E2PROM */
    printf("Writing to E2PROM ...\r\n");
    AT24C02_BufferWrite(BufWrite, 0, TEST_SIZE);
    printf("Write OK\r\n");

    /* 从 E2PROM 读出 */
    printf("Reading from E2PROM ...\r\n");
    AT24C02_BufferRead(BufRead, 0, TEST_SIZE);

    /* 校验数据 */
    printf("Check Data:\r\n");
    CheckTestData();

    printf("E2PROM Test Complete\r\n");

    while (1) {
        /* 保持运行 */
    }
}
