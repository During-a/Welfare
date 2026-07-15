/*****************************************************************************
 * 文件名：main.c
 * 描述：红外接收实验（NEC 协议解码），基于 STM32F407 标准外设库
 * 说明：使用 PE5（TIM9_CH1）输入捕获，解析红外遥控信号并通过串口打印
 * 版本：V1.0
 *****************************************************************************/

#include "stm32f4xx.h"
#include <stdio.h>

/* =====================================================
 * 1. 串口初始化（USART2, PA2-TX, PA3-RX）
 * ===================================================== */

static void USART2_Init(uint32_t baudrate)
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

    USART_InitStruct.USART_BaudRate = baudrate;
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
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    return ch;
}

/* =====================================================
 * 2. 红外接收模块（PE5, TIM9_CH1, 输入捕获）
 * ===================================================== */

#define IR_RECV_TIM      TIM9
#define IR_RECV_PORT     GPIOE
#define IR_RECV_PIN      GPIO_Pin_5

/* 接收状态标志 */
static uint8_t IRState = 0;
#define IR_STATE_LEAD_OK    0x80    // 已接收到引导码
#define IR_STATE_DATA_OK    0x40    // 已接收完一帧数据
#define IR_STATE_UP_OK      0x10    // 已收到第一个上升沿

static uint32_t CapValue = 0;       // 捕获时长
static uint32_t IRCount = 0;        // 按键重复计数
static uint32_t IRData = 0;         // 32 位接收数据

/* 2.1 初始化 GPIO */
static void IR_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStruct.GPIO_Pin = IR_RECV_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(IR_RECV_PORT, &GPIO_InitStruct);

    GPIO_PinAFConfig(IR_RECV_PORT, GPIO_PinSource5, GPIO_AF_TIM9);
}

/* 2.2 初始化 TIM9 输入捕获 */
static void IR_TIM_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    /* 定时器基础配置：周期 10ms，计数频率 1MHz（1us 加 1） */
    TIM_TimeBaseStruct.TIM_Period = 10000;
    TIM_TimeBaseStruct.TIM_Prescaler = 167;         // 168M / (167+1) = 1MHz
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(IR_RECV_TIM, &TIM_TimeBaseStruct);

    /* 输入捕获通道 1 配置：上升沿触发 */
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter = 0x03;            // 8 个时钟周期滤波
    TIM_ICInit(IR_RECV_TIM, &TIM_ICInitStruct);

    /* 使能中断：更新溢出 + 捕获 */
    TIM_ITConfig(IR_RECV_TIM, TIM_IT_Update | TIM_IT_CC1, ENABLE);

    /* 使能定时器 */
    TIM_Cmd(IR_RECV_TIM, ENABLE);

    /* NVIC 配置：TIM9 中断 */
    NVIC_InitStruct.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/* 2.3 IR 接收总初始化 */
static void IR_Recv_Init(void)
{
    IR_GPIO_Init();
    IR_TIM_Init();
}

/* 2.4 默认键码处理函数（用户可在此修改） */
static void IR_Rec_Proc(uint16_t addr, uint8_t code)
{
    printf("IRRecv: addr=%d, code=%d\r\n", addr, code);
}

/* 2.5 中断服务函数 */
void TIM1_BRK_TIM9_IRQHandler(void)
{
    /* ----- 输入捕获中断（上升沿）----- */
    if (TIM_GetITStatus(IR_RECV_TIM, TIM_IT_CC1) != RESET) {
        TIM_ClearITPendingBit(IR_RECV_TIM, TIM_IT_CC1);
        TIM_SetCounter(IR_RECV_TIM, 0);      // 清空计数器

        /* 第一个上升沿只表示新数据开始，不做时长计算 */
        if (!(IRState & IR_STATE_UP_OK)) {
            IRState |= IR_STATE_UP_OK;
            return;
        }

        /* 两次上升沿间隔 - 560us 脉冲 = 高电平时长 */
        CapValue = TIM_GetCapture1(IR_RECV_TIM) - 560;

        /* 根据高电平时长判断数据位 */
        if (CapValue > 300 && CapValue < 800) {
            /* 数据位 0：标准值 560us */
            IRData >>= 1;
        } else if (CapValue > 1400 && CapValue < 1800) {
            /* 数据位 1：标准值 1680us */
            IRData >>= 1;
            IRData |= 0x80000000;
        } else if (CapValue > 2200 && CapValue < 2600) {
            /* 重复码：标准值 2500us */
            IRCount++;
        } else if (CapValue > 4200 && CapValue < 5000) {
            /* 引导码：标准值 4500us */
            IRData = 0;
            IRState |= IR_STATE_LEAD_OK;
            IRCount = 0;
        }
    }

    /* ----- 定时器溢出中断（数据接收完成）----- */
    if (TIM_GetITStatus(IR_RECV_TIM, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(IR_RECV_TIM, TIM_IT_Update);

        if (IRState & IR_STATE_LEAD_OK) {
            IRState &= ~IR_STATE_UP_OK;
            IRState &= ~IR_STATE_LEAD_OK;
            IRState |= IR_STATE_DATA_OK;     // 标记一帧数据接收完成
        }
    }
}

/* 2.6 红外接收任务处理（由主循环调用） */
static void IR_Recv(void)
{
    uint16_t addr = 0;
    uint8_t byte1, byte2, byte3, byte4;

    if (!(IRState & IR_STATE_DATA_OK)) {
        return;                              // 数据未就绪
    }

    IRState &= ~IR_STATE_DATA_OK;            // 清除就绪标志

    /* 提取 4 个字节 */
    byte1 = (uint8_t)(IRData);               // 地址码
    byte2 = (uint8_t)(IRData >> 8);          // 地址反码
    byte3 = (uint8_t)(IRData >> 16);         // 键码
    byte4 = (uint8_t)(IRData >> 24);         // 键码反码
    IRData = 0;

    /* 校验键码反码 */
    if (byte3 != (uint8_t)(~byte4)) {
        return;
    }

    /* 判断地址模式：标准 NEC 或扩展 NEC */
    if (byte1 != (uint8_t)(~byte2)) {
        /* 扩展 NEC：16 位地址 */
        addr = ((uint16_t)byte2 << 8) | byte1;
    } else {
        /* 标准 NEC：8 位地址 */
        addr = byte1;
    }

    /* 调用回调函数处理键码 */
    IR_Rec_Proc(addr, byte3);
}

/* =====================================================
 * 3. 主函数
 * ===================================================== */

int main(void)
{
    USART2_Init(115200);
    printf("IR Recv Init OK\r\n");

    IR_Recv_Init();

    while (1) {
        IR_Recv();
    }
}
