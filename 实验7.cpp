#include "stm32f4xx.h"

/***** 简单延时函数, ms, us *****/
void delay_us(uint16_t nus)
{
    uint16_t i;
    while (nus--)
    {
        i = 31;          // 粗略延时，实际需根据主频校准
        while (i--);
    }
}

void delay_ms(uint16_t nms)
{
    uint16_t i;
    while (nms--)
    {
        i = 33800;       // 粗略延时，实际需根据主频校准
        while (i--);
    }
}

/* 定义一帧数据，\r\n 代表回车换行，共 17 个字节 */
u8 SendBuf[] = "Hello Everyone!\r\n";

/* 串口2 初始化函数 */
void USART2_Init(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* 使能 GPIOA 和 USART2 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* USART2 引脚复用配置 (PA2-TX, PA3-RX) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 引脚复用映射到 USART2 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* USART2 参数配置 */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    /* 使能 USART2 */
    USART_Cmd(USART2, ENABLE);
}

int main(void)
{
    DMA_InitTypeDef DMA_InitStruct;

    /* 初始化 LED（用于指示程序运行） */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;   // 使用 PB5 控制 LED1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_5);   // 初始熄灭

    /* 初始化串口2 */
    USART2_Init(115200);

    /* 使能串口2的 DMA 发送和接收（本实验只用到发送） */
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    // USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  // 如不用接收可注释

    /* 开启 DMA1 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* 配置 DMA1 Stream6 Channel4 用于 USART2_TX */
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (u32)SendBuf;
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize = 17;          // 数据长度
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;   // 单次传输，非循环
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6, &DMA_InitStruct);

    /* 主循环：循环启动 DMA 发送，并让 LED 闪烁指示运行 */
    while (1)
    {
        /* 先关闭 DMA，重置传输计数器，再重新开启 */
        DMA_Cmd(DMA1_Stream6, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Stream6, 17);   // 重置计数器
        DMA_Cmd(DMA1_Stream6, ENABLE);

        /* LED 闪烁指示程序运行 */
        GPIO_ResetBits(GPIOB, GPIO_Pin_5);   // 点亮 LED1
        delay_ms(200);
        GPIO_SetBits(GPIOB, GPIO_Pin_5);     // 熄灭 LED1
        delay_ms(200);

        /* 等待一段时间后再发送下一帧，避免连续发送过快 */
        delay_ms(2000);
    }
}
