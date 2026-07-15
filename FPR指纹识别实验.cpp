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

    // 本实验调试串口不使用中断（仅发送），但保留接收中断关闭
    // USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

// 重定向 printf 到 USART2
int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    return ch;
}

// 重定向 getchar 从 USART2
int fgetc(FILE *f)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
    return (int)USART_ReceiveData(USART2);
}

// USART2 中断（保留但不使用）
void USART2_IRQHandler(void)
{
    // 空，不处理
}

/* ========================== 延时函数（简单轮询） ========================== */
static void delay_init(void)
{
    // 使用 SysTick 产生1ms时基，但不开启中断，仅用于轮询
    SysTick_Config(SystemCoreClock / 1000);
}

static void delay_ms(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }
}

// 微秒延时（粗略）
static void delay_us(uint32_t us)
{
    uint32_t ticks = (SystemCoreClock / 1000000) * us;
    uint32_t start = SysTick->VAL;
    uint32_t elapsed;
    do {
        elapsed = (start - SysTick->VAL) & 0x00FFFFFF;
    } while (elapsed < ticks);
}

#define ZN632_DELAY_MS(x) delay_ms(x)

/* ========================== ZN632 指纹模块驱动 ============================ */

// 定义使用的串口为 USART1
#define ZN632_USART  USART1

// 接收缓冲区
#define ZN632_BUF_LEN  128
uint32_t ZN632_RxBufLen = 0;
uint8_t ZN632_RxBuf[ZN632_BUF_LEN];

// 全局退出标志（用于在等待手指时退出）
uint8_t g_uExitToChange = 0;

// 模块地址（默认 0xFFFFFFFF）
uint8_t ZN632_ADDR[4] = {0xFF, 0xFF, 0xFF, 0xFF};

// 最大指纹模板数
#define ZN632_INDEX_MAX  240
// 索引表（32字节，每bit代表一个ID是否已录入）
uint8_t ZN632_INDEX[32] = {0};

// 命令码定义
#define CMD_GetImage         0x01
#define CMD_GetEnrollImage   0x29
#define CMD_GenChar          0x02
#define CMD_Match            0x03
#define CMD_Search           0x04
#define CMD_RegModel         0x05
#define CMD_StoreChar        0x06
#define CMD_LoadChar         0x07
#define CMD_UpChar           0x08
#define CMD_DownChar         0x09
#define CMD_UpImage          0x0A
#define CMD_DownImage        0x0B
#define CMD_DeleteChar       0x0C
#define CMD_Empty            0x0D
#define CMD_WriteReg         0x0E
#define CMD_ReadSysPara      0x0F
#define CMD_VryPwd           0x13
#define CMD_GetRandomCode    0x14
#define CMD_SetChipAddr      0x15
#define CMD_ReadINPage       0x16
#define CMD_Port_Control     0x17
#define CMD_HighSpeedSearch  0x1B
#define CMD_ReadIndexTable   0x1F

// 函数声明
static void ZN632_GPIO_Init(void);
static void ZN632_UART_Init(uint32_t baud);
static void ZN632_SendData(uint8_t *data, uint8_t length);
static void ZN632_SendHead(void);
static void ZN632_ClearRxBuf(void);
static int16_t ZN632_CheckAck(void);
static void ZN632_ShowError(uint16_t code);

// 对外接口函数
void FPR_Init(uint32_t baud);
int16_t ZN632_PowerOn(void);
int16_t ZN632_VryPwd(void);
int16_t ZN632_ReadIndexTable(void);
uint16_t ZN632_GetIndexEmpty(void);
void ZN632_SetIndex(uint16_t uIndex);
void ZN632_UnsetIndex(uint16_t uIndex);
int16_t ZN632_GetImage(void);
int16_t ZN632_GenChar(uint8_t BufferID);
int16_t ZN632_RegModel(void);
int16_t ZN632_StoreChar(uint8_t BufferID, uint16_t PageID);
int16_t ZN632_HighSpeedSearch(uint8_t BufferID, uint16_t *pID, uint16_t *pScore);
int8_t FPR_GetImage(void);
void FPR_AddFinger(void);
void FPR_MatchFinger(void);

/* ======================== 驱动函数实现 ==================================== */

// GPIO 初始化（PC9 电源控制）
static void ZN632_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// UART1 初始化（PA9, PA10）
static void ZN632_UART_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

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
    USART_Init(ZN632_USART, &USART_InitStructure);

    // 使能接收中断
    USART_ITConfig(ZN632_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(ZN632_USART, ENABLE);

    // NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// 串口发送数据
static void ZN632_SendData(uint8_t *data, uint8_t length)
{
    uint8_t i;
    for (i = 0; i < length; i++) {
        USART_SendData(ZN632_USART, data[i]);
        while (USART_GetFlagStatus(ZN632_USART, USART_FLAG_TXE) == RESET);
    }
}

// 发送命令头（6字节：包头+地址）
static void ZN632_SendHead(void)
{
    uint8_t head[6] = {0xEF, 0x01, ZN632_ADDR[0], ZN632_ADDR[1], ZN632_ADDR[2], ZN632_ADDR[3]};
    ZN632_SendData(head, 6);
}

// 清空接收缓冲区
static void ZN632_ClearRxBuf(void)
{
    memset(ZN632_RxBuf, 0, ZN632_RxBufLen);
    ZN632_RxBufLen = 0;
}

// 检查应答包是否合法，返回确认码（0表示成功，其他为错误码）
static int16_t ZN632_CheckAck(void)
{
    uint16_t i, len, temp, sum = 0;

    // 包头检查
    if (ZN632_RxBuf[0] != 0xEF || ZN632_RxBuf[1] != 0x01)
        return -1;

    // 地址检查
    if (ZN632_RxBuf[2] != ZN632_ADDR[0] || ZN632_RxBuf[3] != ZN632_ADDR[1] ||
        ZN632_RxBuf[4] != ZN632_ADDR[2] || ZN632_RxBuf[5] != ZN632_ADDR[3])
        return -1;

    // 包长度
    len = (ZN632_RxBuf[7] << 8) | ZN632_RxBuf[8];

    // 计算校验和（从第6字节开始，长度为 len 字节）
    for (i = 0; i <= len; i++) {
        sum += ZN632_RxBuf[6 + i];
    }

    // 校验和位于 (9+len-2) 和 (9+len-1)
    temp = (ZN632_RxBuf[9 + len - 2] << 8) | ZN632_RxBuf[9 + len - 1];
    if (sum != temp)
        return -1;

    // 返回确认码（第10字节，即 ZN632_RxBuf[9]）
    return ZN632_RxBuf[9];
}

// 显示错误信息
static void ZN632_ShowError(uint16_t code)
{
    switch (code) {
        case 0x00: printf("OK\r\n"); break;
        case 0x01: printf("数据包接收错误\r\n"); break;
        case 0x02: printf("指纹模块没有检测到指纹!\r\n"); break;
        case 0x03: printf("录入指纹图像失败\r\n"); break;
        case 0x04: printf("指纹图像太干、太淡而生不成特征\r\n"); break;
        case 0x05: printf("指纹图像太湿、太糊而生不成特征\r\n"); break;
        case 0x06: printf("指纹图像太乱而生不成特征\r\n"); break;
        case 0x07: printf("指纹图像正常，但特征点太少（或面积太小）而生不成特征\r\n"); break;
        case 0x08: printf("指纹不匹配\r\n"); break;
        case 0x09: printf("对比指纹失败，指纹库不存在此指纹！\r\n"); break;
        case 0x0A: printf("特征合并失败\r\n"); break;
        case 0x0B: printf("访问指纹库时地址序号超出指纹库范围\r\n"); break;
        case 0x10: printf("删除模板失败\r\n"); break;
        case 0x11: printf("清空指纹库失败\r\n"); break;
        case 0x15: printf("缓冲区内没有有效原始图而生不成图像\r\n"); break;
        case 0x18: printf("读写FLASH出错\r\n"); break;
        case 0x19: printf("未定义错误\r\n"); break;
        case 0x1A: printf("无效寄存器号\r\n"); break;
        case 0x1B: printf("寄存器设定内容错误\r\n"); break;
        case 0x1C: printf("记事本页码指定错误\r\n"); break;
        case 0x1F: printf("指纹库满\r\n"); break;
        case 0x20: printf("地址错误\r\n"); break;
        default:   printf("模块返回确认码有误 (0x%02X)\r\n", code); break;
    }
}

// UART1 中断服务函数（接收数据）
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        if (ZN632_RxBufLen < ZN632_BUF_LEN - 1) {
            ZN632_RxBuf[ZN632_RxBufLen++] = USART_ReceiveData(USART1);
        } else {
            (void)USART_ReceiveData(USART1); // 丢弃溢出数据
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// 上电初始化：拉低PC9供电，等待0x55握手信号
int16_t ZN632_PowerOn(void)
{
    uint32_t i = 0;
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);  // 拉低PC9，给模块供电

    while (1) {
        if (ZN632_RxBufLen > 0 && ZN632_RxBuf[0] == 0x55) {
            ZN632_ClearRxBuf();
            return 0;   // 握手成功
        }
        i++;
        if (i > 1000) {  // 超时约1秒
            return -1;
        }
        ZN632_DELAY_MS(1);
    }
}

// 总初始化
void FPR_Init(uint32_t baud)
{
    ZN632_GPIO_Init();
    ZN632_UART_Init(baud);
    ZN632_PowerOn();
}

// 验证密码（默认密码为0，此处发送全0密码）
int16_t ZN632_VryPwd(void)
{
    uint16_t temp;
    int16_t ret;
    uint8_t cmd[10] = {0};
    cmd[0] = 0x01;      // 包标识
    cmd[1] = 0x00;      // 包长度高8位
    cmd[2] = 0x07;      // 包长度低8位
    cmd[3] = CMD_VryPwd;// 命令码
    cmd[4] = 0;         // 密码（4字节，默认0）
    cmd[5] = 0;
    cmd[6] = 0;
    cmd[7] = 0;
    temp = cmd[0] + cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5] + cmd[6] + cmd[7];
    cmd[8] = temp >> 8;
    cmd[9] = temp;

    ZN632_SendHead();
    ZN632_SendData(cmd, 10);

    ZN632_DELAY_MS(300);
    ret = ZN632_CheckAck();
    if (ret != 0) {
        ZN632_ShowError(ret);
    }
    ZN632_ClearRxBuf();
    return ret;
}

// 读取索引表（只读页码0）
int16_t ZN632_ReadIndexTable(void)
{
    uint16_t temp;
    int16_t ret;
    uint8_t cmd[7] = {0};
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x04;
    cmd[3] = CMD_ReadIndexTable;
    cmd[4] = 0;   // 页码0
    temp = cmd[0] + cmd[1] + cmd[2] + cmd[3] + cmd[4];
    cmd[5] = temp >> 8;
    cmd[6] = temp;

    ZN632_SendHead();
    ZN632_SendData(cmd, 7);

    ZN632_DELAY_MS(300);
    ret = ZN632_CheckAck();
    if (ret != 0) {
        ZN632_ShowError(ret);
        ZN632_ClearRxBuf();
        return ret;
    }
    // 保存索引数据（从第10字节开始，32字节）
    memcpy(ZN632_INDEX, ZN632_RxBuf + 10, 32);
    ZN632_ClearRxBuf();
    return 0;
}

// 获取第一个空索引
uint16_t ZN632_GetIndexEmpty(void)
{
    uint16_t index = 0;
    uint8_t i, j;
    for (i = 0; i < sizeof(ZN632_INDEX); i++) {
        for (j = 0; j < 8; j++) {
            if (ZN632_INDEX[i] & (0x01 << j)) {
                index++;
            } else {
                return index;
            }
        }
    }
    return ZN632_INDEX_MAX;  // 库满
}

// 设置索引位（标记已录入）
void ZN632_SetIndex(uint16_t uIndex)
{
    if (uIndex >= ZN632_INDEX_MAX) return;
    uint8_t uByte = uIndex / 8;
    uint8_t uBit = uIndex % 8;
    ZN632_INDEX[uByte] |= (0x01 << uBit);
}

// 清除索引位
void ZN632_UnsetIndex(uint16_t uIndex)
{
    if (uIndex >= ZN632_INDEX_MAX) return;
    uint8_t uByte = uIndex / 8;
    uint8_t uBit = uIndex % 8;
    ZN632_INDEX[uByte] &= ~(0x01 << uBit);
}

// 获取图像
int16_t ZN632_GetImage(void)
{
    uint16_t temp;
    int16_t ret;
    uint8_t cmd[6] = {0};
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x03;
    cmd[3] = CMD_GetImage;
    temp = cmd[0] + cmd[1] + cmd[2] + cmd[3];
    cmd[4] = temp >> 8;
    cmd[5] = temp;

    ZN632_SendHead();
    ZN632_SendData(cmd, 6);

    ZN632_DELAY_MS(500);
    ret = ZN632_CheckAck();
    if (ret != 0) {
        ZN632_ShowError(ret);
    }
    ZN632_ClearRxBuf();
    return ret;
}

// 生成特征（BufferID: 1或2）
int16_t ZN632_GenChar(uint8_t BufferID)
{
    uint16_t temp;
    int16_t ret;
    uint8_t cmd[7] = {0};
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x04;
    cmd[3] = CMD_GenChar;
    cmd[4] = BufferID;
    temp = cmd[0] + cmd[1] + cmd[2] + cmd[3] + cmd[4];
    cmd[5] = temp >> 8;
    cmd[6] = temp;

    ZN632_SendHead();
    ZN632_SendData(cmd, 7);

    ZN632_DELAY_MS(500);
    ret = ZN632_CheckAck();
    if (ret != 0) {
        ZN632_ShowError(ret);
    }
    ZN632_ClearRxBuf();
    return ret;
}

// 合成模板（将CharBuffer1和CharBuffer2合并）
int16_t ZN632_RegModel(void)
{
    uint16_t temp;
    int16_t ret;
    uint8_t cmd[6] = {0};
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x03;
    cmd[3] = CMD_RegModel;
    temp = cmd[0] + cmd[1] + cmd[2] + cmd[3];
    cmd[4] = temp >> 8;
    cmd[5] = temp;

    ZN632_SendHead();
    ZN632_SendData(cmd, 6);

    ZN632_DELAY_MS(500);
    ret = ZN632_CheckAck();
    if (ret != 0) {
        ZN632_ShowError(ret);
    }
    ZN632_ClearRxBuf();
    return ret;
}

// 储存模板
int16_t ZN632_StoreChar(uint8_t BufferID, uint16_t PageID)
{
    uint16_t temp;
    int16_t ret;
    uint8_t cmd[9] = {0};
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x06;
    cmd[3] = CMD_StoreChar;
    cmd[4] = BufferID;
    cmd[5] = PageID >> 8;
    cmd[6] = PageID & 0xFF;
    temp = cmd[0] + cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5] + cmd[6];
    cmd[7] = temp >> 8;
    cmd[8] = temp;

    ZN632_SendHead();
    ZN632_SendData(cmd, 9);

    ZN632_DELAY_MS(300);
    ret = ZN632_CheckAck();
    if (ret != 0) {
        ZN632_ShowError(ret);
    }
    ZN632_ClearRxBuf();
    return ret;
}

// 高速搜索
int16_t ZN632_HighSpeedSearch(uint8_t BufferID, uint16_t *pID, uint16_t *pScore)
{
    uint16_t temp;
    int16_t ret;
    uint8_t cmd[11] = {0};
    cmd[0] = 0x01;
    cmd[1] = 0x00;
    cmd[2] = 0x08;
    cmd[3] = CMD_HighSpeedSearch;
    cmd[4] = BufferID;
    cmd[5] = 0;          // StartPage 高8位
    cmd[6] = 0;          // StartPage 低8位
    cmd[7] = ZN632_INDEX_MAX >> 8;
    cmd[8] = ZN632_INDEX_MAX & 0xFF;
    temp = cmd[0] + cmd[1] + cmd[2] + cmd[3] + cmd[4] + cmd[5] + cmd[6] + cmd[7] + cmd[8];
    cmd[9] = temp >> 8;
    cmd[10] = temp;

    ZN632_SendHead();
    ZN632_SendData(cmd, 11);

    ZN632_DELAY_MS(300);
    ret = ZN632_CheckAck();
    if (ret != 0) {
        ZN632_ShowError(ret);
        ZN632_ClearRxBuf();
        return ret;
    }
    // 读取返回的ID和分数（从第10字节开始）
    *pID = (ZN632_RxBuf[10] << 8) | ZN632_RxBuf[11];
    *pScore = (ZN632_RxBuf[12] << 8) | ZN632_RxBuf[13];
    ZN632_ClearRxBuf();
    return 0;
}

// 阻塞获取图像（带退出标志）
int8_t FPR_GetImage(void)
{
    while (1) {
        if (ZN632_GetImage() == 0) {
            printf("Get One Finger\r\n");
            return 0;
        }
        if (g_uExitToChange == 1) {
            g_uExitToChange = 0;
            return -1;
        }
        ZN632_DELAY_MS(100);
    }
}

// 录入指纹流程
void FPR_AddFinger(void)
{
    int16_t ret;
    uint16_t uID = 0, uScore = 0;
    uint8_t uStep = 1;

    while (uStep <= 2) {
        printf("Put Finger On Sensor: %d\r\n", uStep);
        ret = FPR_GetImage();
        if (ret != 0) return;

        ret = ZN632_GenChar(uStep);
        if (ret != 0) {
            printf("GenChar failed, retry\r\n");
            continue;
        }

        // 搜索库中是否已存在该指纹
        ret = ZN632_HighSpeedSearch(uStep, &uID, &uScore);
        if (ret == 0) {
            printf("Finger is Already registered, ID=%d\r\n", uID);
            continue;
        }
        uStep++;
    }

    ret = ZN632_RegModel();
    if (ret != 0) {
        printf("RegModel failed\r\n");
        return;
    }

    uID = ZN632_GetIndexEmpty();
    if (uID >= ZN632_INDEX_MAX) {
        printf("Fingerprint database is full!\r\n");
        return;
    }
    ret = ZN632_StoreChar(2, uID);
    if (ret != 0) {
        printf("StoreChar failed\r\n");
        return;
    }
    ZN632_SetIndex(uID);
    printf("Add Finger OK, ID: %d\r\n", uID);
}

// 匹配指纹流程
void FPR_MatchFinger(void)
{
    uint16_t uID = 0, uScore = 0;
    int16_t ret;

    printf("Put Finger On Sensor:\r\n");
    ret = FPR_GetImage();
    if (ret != 0) return;

    ret = ZN632_GenChar(1);
    if (ret != 0) {
        printf("GenChar failed\r\n");
        return;
    }

    ret = ZN632_HighSpeedSearch(1, &uID, &uScore);
    if (ret != 0) {
        printf("Finger NOT Match\r\n");
        return;
    }
    printf("Finger Match, ID: %d, Score: %d\r\n", uID, uScore);
}

// 菜单显示
void FPR_Usage(void)
{
    printf("\r\nSelect one:\r\n");
    printf("1  Add New Finger\r\n");
    printf("2  Match Finger\r\n");
    printf("3  (预留) Delete Finger (未实现)\r\n");
}

/* ========================== 主程序 ======================================== */
int main(void)
{
    char ch = 0;

    delay_init();
    UART2_Init(115200);
    printf("FPR Test Start...\r\n");

    FPR_Init(56700);   // 使用手册中的波特率，若通信异常可改为57600
    printf("FPR Init OK\r\n");

    if (ZN632_VryPwd() == 0) {
        printf("Password OK\r\n");
    } else {
        printf("Password Verify Failed\r\n");
    }

    if (ZN632_ReadIndexTable() == 0) {
        printf("ReadIndexTable OK\r\n");
    } else {
        printf("ReadIndexTable Failed\r\n");
    }

    while (1) {
        FPR_Usage();
        ch = getchar();
        while (getchar() != '\n'); // 清除多余字符
        switch (ch) {
            case '1':
                FPR_AddFinger();
                break;
            case '2':
                FPR_MatchFinger();
                break;
            default:
                printf("No this number\r\n");
                break;
        }
    }
}
