/*****************************************************************************
 * 文件名：main.c
 * 描述：W25Q64 SPI FLASH 读写实验，基于 STM32F407 标准外设库
 * 说明：使用 SPI1（PA5-SCK, PA6-MISO, PA7-MOSI, PC0-CS）与 W25Q64 通信
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
 * 3. W25Q64 SPI FLASH 驱动（SPI1, PA5-SCK, PA6-MISO, PA7-MOSI, PC0-CS）
 * ===================================================== */

/* ---------- 硬件引脚定义 ---------- */
#define W25QXX_SPI          SPI1
#define W25QXX_CS_LOW()     GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define W25QXX_CS_HIGH()    GPIO_SetBits(GPIOC, GPIO_Pin_0)

/* ---------- W25Q64 命令码 ---------- */
#define CMD_WRITE_ENABLE        0x06
#define CMD_WRITE_DISABLE       0x04
#define CMD_READ_STATUS_REG     0x05
#define CMD_WRITE_STATUS_REG    0x01
#define CMD_READ_DATA           0x03
#define CMD_FAST_READ_DATA      0x0B
#define CMD_PAGE_PROGRAM        0x02
#define CMD_SECTOR_ERASE        0x20
#define CMD_BLOCK_ERASE         0xD8
#define CMD_CHIP_ERASE          0xC7
#define CMD_POWER_DOWN          0xB9
#define CMD_RELEASE_POWER_DOWN  0xAB
#define CMD_DEVICE_ID           0xAB
#define CMD_MANU_DEVICE_ID      0x90
#define CMD_JEDEC_DEVICE_ID     0x9F

#define WIP_FLAG                0x01        // 状态寄存器忙标志位
#define DUMMY_BYTE              0xFF        // 哑字节
#define W25QXX_PAGE_SIZE        256         // 每页字节数
#define W25QXX_SECTOR_SIZE      4096        // 每扇区字节数

/* 3.1 等待 SPI 标志位（带超时） */
static int8_t W25QXX_WaitSpiFlag(uint32_t SPI_FLAG, FlagStatus Status)
{
    uint32_t timeout = 0x100000;

    while (SPI_I2S_GetFlagStatus(W25QXX_SPI, SPI_FLAG) != Status) {
        if ((timeout--) == 0) {
            return -1;
        }
    }
    return 0;
}

/* 3.2 SPI 发送一个字节并同时接收一个字节 */
static uint8_t W25QXX_SendByte(uint8_t byte)
{
    if (W25QXX_WaitSpiFlag(SPI_I2S_FLAG_TXE, SET) != 0) {
        return 0;
    }
    SPI_I2S_SendData(W25QXX_SPI, byte);

    if (W25QXX_WaitSpiFlag(SPI_I2S_FLAG_RXNE, SET) != 0) {
        return 0;
    }
    return SPI_I2S_ReceiveData(W25QXX_SPI);
}

/* 3.3 SPI 读取一个字节（发送哑字节） */
static uint8_t W25QXX_ReadByte(void)
{
    return W25QXX_SendByte(DUMMY_BYTE);
}

/* 3.4 发送写使能命令 */
static void W25QXX_WriteEnable(void)
{
    W25QXX_CS_LOW();
    W25QXX_SendByte(CMD_WRITE_ENABLE);
    W25QXX_CS_HIGH();
}

/* 3.5 等待 FLASH 内部写入完成（BUSY 位清除） */
static void W25QXX_WaitForWriteEnd(void)
{
    uint8_t status;
    uint32_t timeout = 0x100000;

    W25QXX_CS_LOW();
    W25QXX_SendByte(CMD_READ_STATUS_REG);

    do {
        status = W25QXX_SendByte(DUMMY_BYTE);
        if ((timeout--) == 0) {
            break;
        }
    } while ((status & WIP_FLAG) == SET);

    W25QXX_CS_HIGH();
}

/* 3.6 SPI 和 GPIO 初始化 */
static void W25QXX_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

    /* 使能 GPIO 和 SPI 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* 配置片选 PC0 为推挽输出，初始高电平 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    W25QXX_CS_HIGH();

    /* 配置 SPI1 引脚：PA5-SCK, PA6-MISO, PA7-MOSI */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    /* SPI1 工作模式配置（模式3：CPOL=High, CPHA=2Edge） */
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(W25QXX_SPI, &SPI_InitStruct);

    SPI_Cmd(W25QXX_SPI, ENABLE);
}

/* 3.7 读取 FLASH ID（JEDEC 标准） */
static uint32_t W25QXX_ReadID(void)
{
    uint32_t temp0, temp1, temp2;

    W25QXX_CS_LOW();
    W25QXX_SendByte(CMD_JEDEC_DEVICE_ID);
    temp0 = W25QXX_SendByte(DUMMY_BYTE);
    temp1 = W25QXX_SendByte(DUMMY_BYTE);
    temp2 = W25QXX_SendByte(DUMMY_BYTE);
    W25QXX_CS_HIGH();

    return (temp0 << 16) | (temp1 << 8) | temp2;
}

/* 3.8 扇区擦除（4KB） */
static void W25QXX_SectorErase(uint32_t SectorAddr)
{
    W25QXX_WriteEnable();

    W25QXX_CS_LOW();
    W25QXX_SendByte(CMD_SECTOR_ERASE);
    W25QXX_SendByte((SectorAddr & 0xFF0000) >> 16);
    W25QXX_SendByte((SectorAddr & 0xFF00) >> 8);
    W25QXX_SendByte(SectorAddr & 0xFF);
    W25QXX_CS_HIGH();

    W25QXX_WaitForWriteEnd();
}

/* 3.9 页写入（最多 256 字节，不能跨页） */
static void W25QXX_PageWrite(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    if (NumByteToWrite > W25QXX_PAGE_SIZE) {
        NumByteToWrite = W25QXX_PAGE_SIZE;
        printf("W25QXX_PageWrite: too large, truncated to page size\r\n");
    }

    W25QXX_WriteEnable();

    W25QXX_CS_LOW();
    W25QXX_SendByte(CMD_PAGE_PROGRAM);
    W25QXX_SendByte((WriteAddr & 0xFF0000) >> 16);
    W25QXX_SendByte((WriteAddr & 0xFF00) >> 8);
    W25QXX_SendByte(WriteAddr & 0xFF);

    while (NumByteToWrite--) {
        W25QXX_SendByte(*pBuffer++);
    }

    W25QXX_CS_HIGH();
    W25QXX_WaitForWriteEnd();
}

/* 3.10 任意长度批量写入（自动分页） */
static void W25QXX_BufferWrite(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t FirstCount, PageNum, ByteNum;
    uint16_t PageIndex;

    PageIndex = WriteAddr % W25QXX_PAGE_SIZE;
    FirstCount = W25QXX_PAGE_SIZE - PageIndex;
    if (FirstCount > NumByteToWrite) {
        FirstCount = NumByteToWrite;
    }

    /* 写入首页（非对齐部分） */
    if (PageIndex != 0) {
        W25QXX_PageWrite(pBuffer, WriteAddr, FirstCount);
        WriteAddr += FirstCount;
        pBuffer += FirstCount;
        NumByteToWrite -= FirstCount;
    }

    /* 写入中间完整页 */
    PageNum = NumByteToWrite / W25QXX_PAGE_SIZE;
    ByteNum = NumByteToWrite % W25QXX_PAGE_SIZE;

    while (PageNum > 0) {
        W25QXX_PageWrite(pBuffer, WriteAddr, W25QXX_PAGE_SIZE);
        WriteAddr += W25QXX_PAGE_SIZE;
        pBuffer += W25QXX_PAGE_SIZE;
        PageNum--;
    }

    /* 写入最后一页（剩余部分） */
    if (ByteNum > 0) {
        W25QXX_PageWrite(pBuffer, WriteAddr, ByteNum);
    }
}

/* 3.11 任意长度批量读取 */
static void W25QXX_BufferRead(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    W25QXX_CS_LOW();

    W25QXX_SendByte(CMD_READ_DATA);
    W25QXX_SendByte((ReadAddr & 0xFF0000) >> 16);
    W25QXX_SendByte((ReadAddr & 0xFF00) >> 8);
    W25QXX_SendByte(ReadAddr & 0xFF);

    while (NumByteToRead--) {
        *pBuffer++ = W25QXX_SendByte(DUMMY_BYTE);
    }

    W25QXX_CS_HIGH();
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
    uint32_t id;

    UART2_Init(115200);
    printf("W25Q64 FLASH Test Start\r\n");

    W25QXX_Init();
    printf("W25QXX Init OK\r\n");

    /* 读取 FLASH ID */
    id = W25QXX_ReadID();
    printf("ReadID: [0x%08X]\r\n", id);
    if (id != 0xEF4017) {
        printf("WARNING: Unexpected FLASH ID (expected 0xEF4017 for W25Q64)\r\n");
    }

    /* 擦除起始扇区 */
    printf("Erasing Sector 0 ...\r\n");
    W25QXX_SectorErase(0);
    printf("SectorErase OK\r\n");

    /* 生成测试数据 */
    printf("Test Data:\r\n");
    MakeTestData();

    /* 写入 FLASH */
    printf("Writing to FLASH ...\r\n");
    W25QXX_BufferWrite(BufWrite, 0, TEST_SIZE);
    printf("Write OK\r\n");

    /* 从 FLASH 读出 */
    printf("Reading from FLASH ...\r\n");
    W25QXX_BufferRead(BufRead, 0, TEST_SIZE);

    /* 校验数据 */
    printf("Check Data:\r\n");
    CheckTestData();

    printf("FLASH Test Complete\r\n");

    while (1) {
        /* 保持运行 */
    }
}