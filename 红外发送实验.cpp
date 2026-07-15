/*****************************************************************************
 * 文件名：main.c
 * 描述：红外发送实验（NEC 协议）完整代码，基于 STM32F407 标准外设库
 * 版本：V1.0
 * 说明：将本文件作为工程中的 main.c 使用，并添加标准外设库源文件即可编译。
 *****************************************************************************/

#include "stm32f4xx.h"
#include <stdio.h>

// ----------------------------------------------------------------------------
// 1. SysTick 微秒延时函数（使用 SysTick 计数器）
// ----------------------------------------------------------------------------
static void systick_delay_us(uint32_t us)
{
    uint32_t ticks = 0;
    uint32_t told, tnow, reload, tcnt = 0;

    reload = SysTick->LOAD;                          // 获取重装载值
    ticks = us * (SystemCoreClock / 1000000);        // 需要的计数次数
    told = SysTick->VAL;                             // 当前计数值

    while (1) {
        tnow = SysTick->VAL;
        if (tnow == told) {
            continue;
        }
        if (tnow < told) {
            tcnt += told - tnow;
        } else {
            tcnt += reload - tnow + told;
        }
        told = tnow;
        if (tcnt >= ticks) {
            break;
        }
    }
}

// ----------------------------------------------------------------------------
// 2. USART2 初始化（PA2-TX, PA3-RX，用于 printf 调试）
// ----------------------------------------------------------------------------
static void USART2_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    // 时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // GPIO 复用配置
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 映射到 USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    // USART 参数：115200, 8N1
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);

    USART_Cmd(USART2, ENABLE);
}

// printf 重定向到 USART2
int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    return ch;
}

// ----------------------------------------------------------------------------
// 3. 红外发送模块（PB8，TIM10_CH1，38kHz PWM）
// ----------------------------------------------------------------------------
#define IR_SEND_TIM      TIM10
#define IR_SEND_PORT     GPIOB
#define IR_SEND_PIN      GPIO_Pin_8

// 3.1 初始化 GPIO
static void IR_Send_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = IR_SEND_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(IR_SEND_PORT, &GPIO_InitStruct);

    GPIO_PinAFConfig(IR_SEND_PORT, GPIO_PinSource8, GPIO_AF_TIM10);
}

// 3.2 初始化 TIM10 为 PWM 输出（38kHz, 50% 占空比）
static void IR_Send_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

    // 频率 = 168MHz / (Prescaler+1) / (Period+1) ≈ 38kHz
    // 这里 Prescaler = 1, Period = 4421，实际约 38k
    TIM_TimeBaseStruct.TIM_Period = 4421;
    TIM_TimeBaseStruct.TIM_Prescaler = 1;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(IR_SEND_TIM, &TIM_TimeBaseStruct);

    // PWM1 模式，50% 占空比 (Pulse = 2210)
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 2210;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(IR_SEND_TIM, &TIM_OCInitStruct);

    TIM_CCxCmd(IR_SEND_TIM, TIM_Channel_1, TIM_CCx_Enable);
}

// 3.3 开启 PWM 输出
static void PwmEnable(void)
{
    TIM_ForcedOC1Config(IR_SEND_TIM, TIM_OCMode_PWM1);
    TIM_Cmd(IR_SEND_TIM, ENABLE);
}

// 3.4 关闭 PWM 输出（强制低电平）
static void PwmDisable(void)
{
    TIM_ForcedOC1Config(IR_SEND_TIM, TIM_ForcedAction_InActive);
    TIM_Cmd(IR_SEND_TIM, DISABLE);
}

// 3.5 发送一个字节（LSB 优先，NEC 协议）
static void IR_Send_Byte(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        PwmEnable();
        systick_delay_us(560);          // 载波脉冲 560us
        PwmDisable();

        if (data & 0x01) {
            systick_delay_us(1680);     // 逻辑 1：空闲 1680us
        } else {
            systick_delay_us(560);      // 逻辑 0：空闲 560us
        }
        data >>= 1;
    }
}

// 3.6 发送完整的 NEC 帧（地址 + 键码）
static void IR_Send(uint8_t addr, uint8_t code)
{
    // 引导码：9ms 载波 + 4.5ms 空闲
    PwmEnable();
    systick_delay_us(9000);
    PwmDisable();
    systick_delay_us(4500);

    // 4 字节数据：地址、地址反码、键码、键码反码
    IR_Send_Byte(addr);
    IR_Send_Byte(~addr);
    IR_Send_Byte(code);
    IR_Send_Byte(~code);

    // 停止位：560us 载波
    PwmEnable();
    systick_delay_us(560);
    PwmDisable();
}

// 3.7 红外发送模块总初始化
static void IR_Send_Init(void)
{
    IR_Send_GPIO_Init();
    IR_Send_PWM_Init();
}

// ----------------------------------------------------------------------------
// 4. 红外接收占位（本实验仅发送，接收只留空函数）
// ----------------------------------------------------------------------------
static void IR_Recv_Init(void)
{
    // 若需要接收功能，在此补充初始化代码
}

static void IR_Recv(void)
{
    // 若需要接收功能，在此补充接收处理代码
}

// ----------------------------------------------------------------------------
// 5. 主函数
// ----------------------------------------------------------------------------
int main(void)
{
    uint32_t i = 0;
    uint32_t msCounter = 0;

    // 初始化 SysTick，用于精确延时（系统时钟需提前配置）
    SysTick_Config(SystemCoreClock / 1000);   // 1ms 中断

    // 初始化串口调试
    USART2_Init(115200);
    printf("IR Send Test Start\r\n");

    // 初始化红外发送
    IR_Send_Init();
    printf("IR Send Init OK\r\n");

    // 初始化红外接收（占位）
    IR_Recv_Init();
    printf("IR Recv Init OK\r\n");

    while (1) {
        // 如果接收功能真实存在，可调用 IR_Recv();

        // 每 10 秒发送一次
        if (msCounter >= 10000) {
            printf("Send: addr=0x01, code=0x7B (123)\r\n");
            IR_Send(0x01, 0x7B);
            msCounter = 0;
        }

        // 大约 1ms 计数
        if (i >= 4000) {
            msCounter++;
            i = 0;
        } else {
            i++;
        }
    }
}
