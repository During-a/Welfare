
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

/* ************** 宏定义 ************** */
#define DEBUG_USART   USART2

/* ************** 函数声明 ************** */
void UART2_Init(uint32_t bound);
void UART_SendByte(USART_TypeDef *pUSARTx, uint8_t ch);
void UART_SendString(USART_TypeDef *pUSARTx, uint8_t *str);
void UART_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t ch);
void delay_ms(uint32_t ms);
void CAN1_Init(void);
void CAN1_FilterConfig(void);
void CAN1_SendMsg(uint32_t uID, uint8_t *pData, uint32_t uLen);

/* ************** USART2 驱动实现 ************** */

/**
  * @brief  初始化USART2 (PA2-TX, PA3-RX) 115200 8N1
  * @param  bound: 波特率
  */
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

/**
  * @brief  发送单字节
  */
void UART_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
    USART_SendData(pUSARTx, ch);
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  发送字符串
  */
void UART_SendString(USART_TypeDef *pUSARTx, uint8_t *str)
{
    while (*str)
        UART_SendByte(pUSARTx, *str++);
}

/**
  * @brief  发送16位数据（高字节先发）
  */
void UART_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t ch)
{
    uint8_t temp_h = (ch & 0xFF00) >> 8;
    uint8_t temp_l = ch & 0xFF;
    USART_SendData(pUSARTx, temp_h);
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
    USART_SendData(pUSARTx, temp_l);
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  printf重定向到USART2
  */
int fputc(int ch, FILE *f)
{
    USART_SendByte(DEBUG_USART, (uint8_t)ch);
    return ch;
}

/**
  * @brief  scanf/getchar重定向到USART2
  */
int fgetc(FILE *f)
{
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);
    return (int)USART_ReceiveData(DEBUG_USART);
}

/**
  * @brief  USART2中断服务函数（回显）
  */
void USART2_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
    {
        uint8_t sbuf = USART_ReceiveData(USART2);
        USART_SendData(USART2, sbuf);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
}

/* ************** 简单延时（非精确） ************** */
void delay_ms(uint32_t ms)
{
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 10000; j++);
}

/* ************** CAN 驱动实现 ************** */

/**
  * @brief  CAN1初始化（自发自收模式，波特率500K）
  */
void CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    CAN_InitTypeDef CAN_InitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* PA11(CAN1_RX), PA12(CAN1_TX) 复用 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    /* CAN参数：500Kbps = 42M / (Prescaler * (1+BS1+BS2)) => Prescaler=6, BS1=6, BS2=7 */
    CAN_InitStruct.CAN_Prescaler = 6;
    CAN_InitStruct.CAN_Mode = CAN_Mode_LoopBack;      // 自发自收
    CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStruct.CAN_BS1 = CAN_BS1_6tq;             // 修正为6
    CAN_InitStruct.CAN_BS2 = CAN_BS2_7tq;             // 修正为7
    CAN_InitStruct.CAN_TTCM = DISABLE;
    CAN_InitStruct.CAN_ABOM = DISABLE;
    CAN_InitStruct.CAN_AWUM = DISABLE;
    CAN_InitStruct.CAN_NART = DISABLE;
    CAN_InitStruct.CAN_RFLM = DISABLE;
    CAN_InitStruct.CAN_TXFP = DISABLE;
    CAN_Init(&CAN1, &CAN_InitStruct);

    /* 配置筛选器（接收所有帧） */
    CAN1_FilterConfig();

    /* 使能CAN接收中断（FIFO0） */
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    CAN_Cmd(CAN1, ENABLE);
}

/**
  * @brief  CAN筛选器配置（32位掩码，掩码为0，接收所有）
  */
void CAN1_FilterConfig(void)
{
    CAN_FilterInitTypeDef CAN_Filter;

    CAN_Filter.CAN_FilterNumber = 0;
    CAN_Filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_Filter.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_Filter.CAN_FilterIdHigh = 0x0000;
    CAN_Filter.CAN_FilterIdLow = 0x0000;
    CAN_Filter.CAN_FilterMaskIdHigh = 0x0000;
    CAN_Filter.CAN_FilterMaskIdLow = 0x0000;
    CAN_Filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_Filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_Filter);
}

/**
  * @brief  CAN发送数据
  * @param  uID: 扩展ID（本实验使用0x1234）
  * @param  pData: 数据指针
  * @param  uLen: 数据长度（1~8）
  */
void CAN1_SendMsg(uint32_t uID, uint8_t *pData, uint32_t uLen)
{
    uint32_t i;
    uint8_t uMailBox;
    CanTxMsg TxMessage;

    TxMessage.StdId = 0;
    TxMessage.ExtId = uID;
    TxMessage.IDE = CAN_Id_Extended;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = uLen;

    printf("Send CAN Data:[0x%04X; ", uID);
    for (i = 0; i < uLen; i++)
    {
        printf("%02X ", pData[i]);
        TxMessage.Data[i] = pData[i];
    }
    printf("|\\r\\n");

    uMailBox = CAN_Transmit(CAN1, &TxMessage);
    i = 0;
    while (CAN_TransmitStatus(CAN1, uMailBox) == CAN_TxStatus_Failed)
    {
        i++;
        if (i >= 0XFFFF) break;
    }
    if (i >= 0XFFFF)
    {
        printf("Send Failed!\\r\\n");
        return;
    }
    // printf("Send Success! Tx_Mail:%u\\r\\n", uMailBox);
}

/**
  * @brief  CAN接收中断服务函数（FIFO0）
  */
void CAN1_RX0_IRQHandler(void)
{
    int i;
    uint32_t uCanID;
    CanRxMsg RxMessage;

    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

    uCanID = (RxMessage.IDE == CAN_Id_Standard) ? RxMessage.StdId : RxMessage.ExtId;
    printf("\\nRecv CANID:0x%08X Data:[", uCanID);
    for (i = 0; i < 8; i++)
    {
        printf("%02X ", RxMessage.Data[i]);
    }
    printf("]\\r\\n");
}

/* ************** 主程序 ************** */
int main(void)
{
    uint8_t pTxData[8] = {0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11};

    UART2_Init(115200);
    CAN1_Init();

    printf("\\r\\nCAN LoopBack Test Start...\\r\\n");

    while (1)
    {
        CAN1_SendMsg(0x1234, pTxData, 8);
        delay_ms(5000);
    }
}