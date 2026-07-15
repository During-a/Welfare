#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

/* ========== 定义结构体变量 ========== */
GPIO_InitTypeDef  GPIO_InitStructure;
NVIC_InitTypeDef  NVIC_InitStructure;
EXTI_InitTypeDef  EXTI_InitStructure;

/* ========== 全局时间变量（中断会修改，加 volatile 防止被优化） ========== */
volatile u16 LedTime = 500;   // 流水灯速度，单位 ms，默认为 500ms

/* ========== 延时函数声明 ========== */
void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);

/* ========== 延时函数定义 ========== */
void delay_us(uint16_t nus)
{
    uint16_t i;
    while(nus--)
    {
        i = 31;
        while(i--);
    }
}

void delay_ms(uint16_t nms)
{
    uint16_t i;
    while(nms--)
    {
        i = 33800;
        while(i--);
    }
}

/* ========== 主函数 ========== */
int main(void)
{
    /* ---- 1. 初始化三个 LED（PB0, PB1, PB5） ---- */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 初始状态：三个 LED 全部熄灭 */
    GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5);

    /* ---- 2. 初始化 KEY1（PA0）为外部中断 ---- */
    /* 2.1 开启 GPIOA 和 SYSCFG 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);   // 注意是 APB2，不是 AHB1

    /* 2.2 配置 PA0 为上拉输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 2.3 将 PA0 连接到 EXTI Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* 2.4 配置 EXTI Line0：下降沿触发中断 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;   // 按键按下时 PA0 从高→低
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 2.5 配置 NVIC 中断优先级 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* ---- 3. 流水灯主循环 ---- */
    while(1)
    {
        // LED1（PB0）亮，其余灭
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
        GPIO_SetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_5);
        delay_ms(LedTime);   // 使用全局变量 LedTime，中断会修改它

        // LED2（PB1）亮，其余灭
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_5);
        delay_ms(LedTime);

        // LED3（PB5）亮，其余灭
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);
        GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
        delay_ms(LedTime);
    }
}

/* ========== 外部中断0服务函数 ========== */
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);   // 清除中断标志
        LedTime = 100;                        // 加快速度：500ms → 100ms（5倍速）
    }
}