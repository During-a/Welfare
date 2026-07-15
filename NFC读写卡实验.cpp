/*****************************************************************************
 * 文件名：main.c
 * 说明：整合 PN532 NFC 模块驱动、命令、读写流程及主程序于一体
 *       适用于 STM32F407 + AU100 开发板，UART1 连接 PN532，UART2 调试输出
 *       支持 S50 卡读写（块2），自动寻卡、验证、读写，带重试机制
 *       修正：删除 NFC_Init 中重复的 NVIC_InitStructure 声明
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"

/* ========================== 调试串口 UART2 ================================ */
void UART2_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    return ch;
}

int fgetc(FILE *f)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
    return (int)USART_ReceiveData(USART2);
}

void USART2_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) {
        uint8_t ch = USART_ReceiveData(USART2);
        USART_SendData(USART2, ch);
    }
}

/* ========================== 延时函数 ====================================== */
static void delay_init(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

static void delay_ms(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }
}

static void delay_us(uint32_t us)
{
    uint32_t ticks = (SystemCoreClock / 1000000) * us;
    uint32_t start = SysTick->VAL;
    uint32_t elapsed;
    do {
        elapsed = (start - SysTick->VAL) & 0x00FFFFFF;
    } while (elapsed < ticks);
}

#define PN532_DELAY_MS(x)  delay_ms(x)

/* ========================== PN532 NFC 驱动 =============================== */

// 使用 USART1
#define PN532_USART  USART1

// 接收缓冲区
#define PN532_BUF_LEN  500
uint8_t PN532_RxBuf[PN532_BUF_LEN];
uint16_t PN532_RxBufLen = 0;

// 重试次数（0 表示无限重试）
uint32_t MAX_TRY = 0;

// 卡片 UID（4字节）
uint8_t UID[4] = {0};
uint8_t UID_backup[4] = {0};

// 默认密钥（KeyA 和 KeyB）
uint8_t KEY_A[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t KEY_B[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 帧解析偏移量
#define LEN_ACK       6
#define INDEX_LEN     3
#define INDEX_TFI     5

// 命令码
#define CMD_SAMConfiguration  0x14
#define CMD_InListPassiveTarget 0x4A
#define CMD_InDataExchange     0x40

// 函数声明
static void PN532_SendData(uint8_t *data, uint8_t length);
static void PN532_ClearRxBuf(void);
static int8_t PN532_CheckRxBuf(void);
static uint8_t PN532_CheckSum(uint8_t *data);
static int8_t PN532_SAMConfiguration(uint8_t mode);
static int8_t PN532_InListPassiveTarget(void);
static int8_t PN532_PsdVerifyKeyA(uint8_t *pKeyA);
static int8_t PN532_ReadBlock(uint8_t block, uint8_t *buf);
static int8_t PN532_WriteBlock(uint8_t block, uint8_t *buf);

// 对外接口
void NFC_Init(uint32_t baud);
int8_t NFC_WakeUp(void);
int8_t NFC_Read(uint8_t block, uint8_t *buf);
int8_t NFC_Write(uint8_t block, uint8_t *buf);

/* ======================== 驱动函数实现 ==================================== */

// UART1 初始化（修正：删除重复的 NVIC_InitStructure 声明）
void NFC_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;   // 此处声明一次

    // 使能 GPIOA 和 USART1 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 复用功能映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(PN532_USART, &USART_InitStructure);

    USART_ITConfig(PN532_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(PN532_USART, ENABLE);

    // 配置 NVIC 中断（使用上面声明的 NVIC_InitStructure）
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// 串口发送数据
static void PN532_SendData(uint8_t *data, uint8_t length)
{
    uint8_t i;
    for (i = 0; i < length; i++) {
        USART_SendData(PN532_USART, data[i]);
        while (USART_GetFlagStatus(PN532_USART, USART_FLAG_TXE) == RESET);
    }
}

// 清空接收缓冲区
static void PN532_ClearRxBuf(void)
{
    memset(PN532_RxBuf, 0, PN532_RxBufLen);
    PN532_RxBufLen = 0;
}

// 校验接收帧
static int8_t PN532_CheckRxBuf(void)
{
    uint8_t i, len, temp = 0;

    if (PN532_RxBufLen < (LEN_ACK + INDEX_TFI + 4))
        return -1;

    len = PN532_RxBuf[LEN_ACK + INDEX_LEN];
    temp = 0x100 - len;
    if (temp != PN532_RxBuf[LEN_ACK + INDEX_LEN + 1])
        return -1;

    temp = 0;
    for (i = 0; i < len; i++) {
        temp += PN532_RxBuf[LEN_ACK + INDEX_TFI + i];
    }
    temp = 0x100 - temp;
    if (temp != PN532_RxBuf[LEN_ACK + INDEX_TFI + len])
        return -1;

    return 0;
}

// 计算校验和（针对 data 数组，从 INDEX_TFI 开始，长度为 data[INDEX_LEN]）
static uint8_t PN532_CheckSum(uint8_t *data)
{
    uint8_t i;
    uint8_t temp = 0;
    uint8_t len = data[INDEX_LEN];

    for (i = 0; i < len; i++) {
        temp += data[INDEX_TFI + i];
    }
    return 0x100 - temp;
}

// USART1 中断服务
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        if (PN532_RxBufLen < PN532_BUF_LEN - 1) {
            PN532_RxBuf[PN532_RxBufLen++] = USART_ReceiveData(USART1);
        } else {
            (void)USART_ReceiveData(USART1);
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// SAM 配置
static int8_t PN532_SAMConfiguration(uint8_t mode)
{
    uint8_t data[10] = {0};
    int8_t ret = 0;

    data[0] = 0x00;   // 前导码
    data[1] = 0x00;   // 开始码
    data[2] = 0xFF;
    data[3] = 0x03;   // 包长度
    data[4] = 0x100 - data[3];  // LCS
    data[5] = 0xD4;   // TFI
    data[6] = CMD_SAMConfiguration;
    data[7] = mode;
    data[8] = PN532_CheckSum(data);
    data[9] = 0x00;   // 后序码

    PN532_SendData(data, sizeof(data));
    PN532_DELAY_MS(200);

    if (PN532_CheckRxBuf() < 0) {
        ret = -1;
    }
    PN532_ClearRxBuf();
    return ret;
}

// 唤醒模块（发送唤醒帧 + SAM 配置测试）
int8_t NFC_WakeUp(void)
{
    uint32_t i = 0;
    uint8_t wakeup_data[14] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    while (1) {
        PN532_SendData(wakeup_data, sizeof(wakeup_data));
        if (PN532_SAMConfiguration(1) == 0) {
            printf("NFC WakeUP OK\r\n");
            return 0;
        }
        PN532_DELAY_MS(100);
        i++;
        if (MAX_TRY != 0 && i > MAX_TRY) {
            printf("NFC WakeUP TimeOut\r\n");
            return -1;
        }
    }
}

// 寻卡（InListPassiveTarget）
static int8_t PN532_InListPassiveTarget(void)
{
    uint8_t data[11] = {0};
    int8_t ret = -1;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0xFF;
    data[3] = 0x04;
    data[4] = 0x100 - data[3];
    data[5] = 0xD4;
    data[6] = CMD_InListPassiveTarget;
    data[7] = 0x01;      // 只选1张卡
    data[8] = 0x00;      // 106K TypeA
    data[9] = PN532_CheckSum(data);
    data[10] = 0x00;

    PN532_SendData(data, sizeof(data));
    PN532_DELAY_MS(200);

    if (PN532_CheckRxBuf() < 0) {
        ret = -1;
        goto _END;
    }

    // 检查卡数量（Tg）
    if (PN532_RxBuf[LEN_ACK + INDEX_TFI + 2] != 1) {
        ret = 0;   // 无卡
        goto _END;
    }

    // 保存 UID（4字节）
    UID[0] = PN532_RxBuf[LEN_ACK + INDEX_TFI + 8];
    UID[1] = PN532_RxBuf[LEN_ACK + INDEX_TFI + 9];
    UID[2] = PN532_RxBuf[LEN_ACK + INDEX_TFI + 10];
    UID[3] = PN532_RxBuf[LEN_ACK + INDEX_TFI + 11];
    ret = 1;   // 有卡

_END:
    PN532_ClearRxBuf();
    return ret;
}

// 验证 KeyA
static int8_t PN532_PsdVerifyKeyA(uint8_t *pKeyA)
{
    uint8_t data[22] = {0};
    int8_t ret = -1;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0xFF;
    data[3] = 0x0F;
    data[4] = 0x100 - data[3];
    data[5] = 0xD4;
    data[6] = CMD_InDataExchange;
    data[7] = 0x01;          // 卡号
    data[8] = 0x60;          // 验证 KeyA
    data[9] = 0x03;          // 块地址（用于验证）
    data[10] = pKeyA[0];
    data[11] = pKeyA[1];
    data[12] = pKeyA[2];
    data[13] = pKeyA[3];
    data[14] = pKeyA[4];
    data[15] = pKeyA[5];
    data[16] = UID[0];
    data[17] = UID[1];
    data[18] = UID[2];
    data[19] = UID[3];
    data[20] = PN532_CheckSum(data);
    data[21] = 0x00;

    PN532_SendData(data, sizeof(data));
    PN532_DELAY_MS(200);

    if (PN532_CheckRxBuf() < 0) {
        ret = -1;
        goto _END;
    }
    if (PN532_RxBuf[LEN_ACK + INDEX_TFI + 2] != 0) {
        ret = -1;
        goto _END;
    }
    ret = 0;

_END:
    PN532_ClearRxBuf();
    return ret;
}

// 读块
static int8_t PN532_ReadBlock(uint8_t block, uint8_t *buf)
{
    uint8_t data[12] = {0};
    int8_t ret = -1;

    if (buf == NULL) return -1;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0xFF;
    data[3] = 0x05;
    data[4] = 0x100 - data[3];
    data[5] = 0xD4;
    data[6] = CMD_InDataExchange;
    data[7] = 0x01;          // 卡号
    data[8] = 0x30;          // 读命令
    data[9] = block;
    data[10] = PN532_CheckSum(data);
    data[11] = 0x00;

    PN532_SendData(data, sizeof(data));
    PN532_DELAY_MS(200);

    if (PN532_CheckRxBuf() < 0) {
        ret = -1;
        goto _END;
    }
    if (PN532_RxBuf[LEN_ACK + INDEX_TFI + 2] != 0) {
        ret = -1;
        goto _END;
    }
    memcpy(buf, PN532_RxBuf + LEN_ACK + INDEX_TFI + 3, 16);
    ret = 0;

_END:
    PN532_ClearRxBuf();
    return ret;
}

// 写块
static int8_t PN532_WriteBlock(uint8_t block, uint8_t *buf)
{
    uint8_t data[28] = {0};
    int8_t ret = -1;

    if (buf == NULL) return -1;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0xFF;
    data[3] = 0x15;
    data[4] = 0x100 - data[3];
    data[5] = 0xD4;
    data[6] = CMD_InDataExchange;
    data[7] = 0x01;          // 卡号
    data[8] = 0xA0;          // 写命令
    data[9] = block;
    memcpy(data + 10, buf, 16);
    data[26] = PN532_CheckSum(data);
    data[27] = 0x00;

    PN532_SendData(data, sizeof(data));
    PN532_DELAY_MS(200);

    if (PN532_CheckRxBuf() < 0) {
        ret = -1;
        goto _END;
    }
    if (PN532_RxBuf[LEN_ACK + INDEX_TFI + 2] != 0) {
        ret = -1;
        goto _END;
    }
    ret = 0;

_END:
    PN532_ClearRxBuf();
    return ret;
}

// ======================== 对外读写接口（带重试） ============================

// 写卡（带重试）
int8_t NFC_Write(uint8_t block, uint8_t *buf)
{
    uint32_t i = 0;

    while (1) {
        PN532_DELAY_MS(100);
        i++;
        if (MAX_TRY != 0 && i > MAX_TRY) {
            printf("NFC Write TimeOut\r\n");
            return -1;
        }

        if (PN532_InListPassiveTarget() <= 0) {
            continue;
        }
        if (PN532_PsdVerifyKeyA(KEY_A) < 0) {
            continue;
        }
        if (PN532_WriteBlock(block, buf) < 0) {
            continue;
        }
        break;
    }
    return 0;
}

// 读卡（带重试）
int8_t NFC_Read(uint8_t block, uint8_t *buf)
{
    uint32_t i = 0;

    while (1) {
        PN532_DELAY_MS(100);
        i++;
        if (MAX_TRY != 0 && i > MAX_TRY) {
            printf("NFC Read TimeOut\r\n");
            return -1;
        }

        if (PN532_InListPassiveTarget() <= 0) {
            continue;
        }
        if (PN532_PsdVerifyKeyA(KEY_A) < 0) {
            continue;
        }
        if (PN532_ReadBlock(block, buf) < 0) {
            continue;
        }
        break;
    }
    return 0;
}

/* ========================== 测试辅助函数 ================================== */

#define TEST_SIZE  16
uint8_t BufRead[TEST_SIZE] = {0};
uint8_t BufWrite[TEST_SIZE] = {0};

// 生成测试数据（0x00 ~ 0x0F）
void make_test_data(void)
{
    uint32_t i;
    printf("Test Data: ");
    for (i = 0; i < TEST_SIZE; i++) {
        BufWrite[i] = (uint8_t)i;
        printf("0x%02X ", BufWrite[i]);
    }
    printf("\r\n");
}

// 校验读取的数据
void check_test_data(void)
{
    uint32_t i;
    printf("Read Data : ");
    for (i = 0; i < TEST_SIZE; i++) {
        printf("0x%02X ", BufRead[i]);
        if (BufRead[i] != BufWrite[i]) {
            printf(" [ERROR at %d]\r\n", i);
            return;
        }
    }
    printf("\r\n");
    printf("TEST OK\r\n");
}

/* ========================== 主程序 ======================================== */

int main(void)
{
    delay_init();
    UART2_Init(115200);
    printf("\r\nNFC Test Start...\r\n");

    NFC_Init(115200);
    printf("NFC Init OK\r\n");

    if (NFC_WakeUp() != 0) {
        printf("NFC WakeUp Failed, check hardware!\r\n");
        while (1);
    }

    make_test_data();

    while (1) {
        printf("\r\n--- Write Block 2 ---\r\n");
        if (NFC_Write(2, BufWrite) == 0) {
            printf("Write OK\r\n");
        } else {
            printf("Write Failed\r\n");
            continue;
        }

        PN532_DELAY_MS(5000);   // 等待5秒，便于观察

        printf("--- Read Block 2 ---\r\n");
        if (NFC_Read(2, BufRead) == 0) {
            printf("Read OK\r\n");
        } else {
            printf("Read Failed\r\n");
            continue;
        }

        check_test_data();
        PN532_DELAY_MS(30000);  // 等待30秒后循环
    }
}
