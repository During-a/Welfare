#include <stddef.h>
#include <stdio.h>
#include "stm32f4xx.h"

/* ========================== 延时函数（SysTick 实现） ========================= */
static void delay_init(void)
{
    // SysTick 时钟源 = HCLK/8 (168MHz/8=21MHz)，重装载为 21M-1，产生1ms中断
    // 但这里我们仅使用轮询计数，不开启中断
    SysTick_Config(SystemCoreClock / 1000);  // 1ms 中断，但不用中断服务函数
    // 实际上我们使用 SysTick->VAL 和 LOAD 来微秒延时，简单实现
}

// 微秒延时（粗略）
static void delay_us(uint32_t us)
{
    uint32_t ticks = (SystemCoreClock / 8000) * us; // 21MHz 时钟，每个 tick 约 47.6ns，这里粗略
    uint32_t start = SysTick->VAL;
    uint32_t elapsed;
    do {
        elapsed = (start - SysTick->VAL) & 0x00FFFFFF;
    } while (elapsed < ticks);
}

// 毫秒延时
static void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

// 为兼容原代码中的宏
#define XPT2046_DelayUS(x)  delay_us(x)
#define ILI9341_DELAY_MS(x) delay_ms(x)   // 未使用，但保留

/* ========================== USART2 调试串口 ================================ */
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

// 重定向 printf 到串口
int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    return ch;
}

// 中断处理（简单回显）
void USART2_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) {
        uint8_t ch = USART_ReceiveData(USART2);
        USART_SendData(USART2, ch);
    }
}

/* ========================== XPT2046 驱动 ================================== */

// 引脚定义（根据手册）
#define XPT2046_CS_ENABLE()   GPIO_ResetBits(GPIOI, GPIO_Pin_13)
#define XPT2046_CS_DISABLE()  GPIO_SetBits(GPIOI, GPIO_Pin_13)

#define XPT2046_CLK_HIGH()    GPIO_SetBits(GPIOE, GPIO_Pin_0)
#define XPT2046_CLK_LOW()     GPIO_ResetBits(GPIOE, GPIO_Pin_0)

#define XPT2046_MOSI_1()      GPIO_SetBits(GPIOE, GPIO_Pin_2)
#define XPT2046_MOSI_0()      GPIO_ResetBits(GPIOE, GPIO_Pin_2)

#define XPT2046_MISO()        GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)
#define XPT2046_ReadIRQ()     GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)

// 通道命令
#define XPT2046_CHANNEL_X     0x90   // X+ (手册中写0x90，但有些是0xd0，以手册为准)
#define XPT2046_CHANNEL_Y     0xD0   // Y+

#define TOUCH_PRESSED         1
#define TOUCH_NOT_PRESSED     0
#define XPT2046_PENIRQ_Active 0       // 低电平有效
#define DURATION_TIME         2       // 消抖周期数

// 坐标结构体
typedef struct {
    int16_t x;
    int16_t y;
    int16_t pre_x;
    int16_t pre_y;
} XPT2046_Coord;

// 校准因子结构体
typedef struct {
    long double A;
    long double B;
    long double C;
    long double D;
    long double E;
    long double F;
} XPT2046_Factor;

// 触摸状态枚举
typedef enum {
    XPT2046_STATE_RELEASE = 0,
    XPT2046_STATE_WAITING,
    XPT2046_STATE_PRESSED,
} enumTouchState;

// 全局变量
// 当前LCD扫描模式（默认为2，与提供的校准因子对应）
int LCD_SCAN_MODE = 2;

// 预设校准因子（8种扫描模式，取自手册）
XPT2046_Factor TouchFactor[] = {
    {-0.006464, -0.073259, 280.358032,  0.074878,  0.002052, -6.545977},  // 0
    { 0.086314,  0.001891, -12.836658, -0.003722, -0.065799,254.715714},  // 1
    { 0.002782,  0.061522, -11.595689,  0.083393,  0.005159,-15.650089},  // 2
    { 0.089743, -0.000289, -20.612209, -0.001374,  0.064451,-16.054003},  // 3
    { 0.000767, -0.068258, 250.891769, -0.085559, -0.000195,334.747650},  // 4
    {-0.084744,  0.000047, 323.163147, -0.002109, -0.066371,260.985809},  // 5
    {-0.001848,  0.066984, -12.807136, -0.084858, -0.000805,333.395386},  // 6
    {-0.085470, -0.000876, 334.023163, -0.003390,  0.064725, -6.211169}   // 7
};

// 内部函数声明
static void XPT2046_WriteCMD(uint8_t ucCmd);
static uint16_t XPT2046_ReadCMD(void);
static uint16_t XPT2046_ReadAdc(uint8_t ucChannel);
static void XPT2046_ReadAdcXY(uint16_t *sX_Ad, uint16_t *sY_Ad);
static int8_t XPT2046_FilterXY(XPT2046_Coord *pCoord);

// 对外函数
void XPT2046_Init(void);
int8_t XPT2046_GetTouchPoint(XPT2046_Coord *pLCD);
uint8_t XPT2046_DetectTouch(void);
void XPT2046_TouchDown(XPT2046_Coord *touch);
void XPT2046_TouchUp(XPT2046_Coord *touch);

/* ======================== 驱动函数实现 ==================================== */

// GPIO 初始化
void XPT2046_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI | RCC_AHB1Periph_GPIOE, ENABLE);

    // CLK (PE0)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // MOSI (PE2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // CS (PI13)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOI, &GPIO_InitStructure);

    // MISO (PE3) 输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // IRQ (PE4) 输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  // 开漏，但输入模式不受影响
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // 拉低片选（使能芯片，低电平有效）
    XPT2046_CS_ENABLE();
}

// 写入命令（8位）
static void XPT2046_WriteCMD(uint8_t ucCmd)
{
    uint8_t i;
    XPT2046_MOSI_0();
    XPT2046_CLK_LOW();

    for (i = 0; i < 8; i++) {
        if (ucCmd & (0x80 >> i))
            XPT2046_MOSI_1();
        else
            XPT2046_MOSI_0();
        XPT2046_DelayUS(5);
        XPT2046_CLK_HIGH();
        XPT2046_DelayUS(5);
        XPT2046_CLK_LOW();
    }
}

// 读取数据（12位）
static uint16_t XPT2046_ReadCMD(void)
{
    uint8_t i;
    uint16_t usBuf = 0;

    XPT2046_MOSI_0();
    XPT2046_CLK_HIGH();

    for (i = 0; i < 12; i++) {
        XPT2046_CLK_LOW();
        if (XPT2046_MISO())
            usBuf |= (1 << (11 - i));
        XPT2046_CLK_HIGH();
    }
    return usBuf;
}

// 读取指定通道ADC值
static uint16_t XPT2046_ReadAdc(uint8_t ucChannel)
{
    XPT2046_WriteCMD(ucChannel);
    return XPT2046_ReadCMD();
}

// 读取XY通道ADC值
static void XPT2046_ReadAdcXY(uint16_t *sX_Ad, uint16_t *sY_Ad)
{
    *sX_Ad = XPT2046_ReadAdc(XPT2046_CHANNEL_X);
    XPT2046_DelayUS(1);
    *sY_Ad = XPT2046_ReadAdc(XPT2046_CHANNEL_Y);
}

// 滤波函数：采样10次，去掉最大最小，平均
static int8_t XPT2046_FilterXY(XPT2046_Coord *pCoord)
{
    uint8_t ucCount = 0, i;
    uint16_t tempAD_X, tempAD_Y;
    uint16_t sBufferArray[2][10] = { {0}, {0} };
    uint32_t lX_Min, lX_Max, lY_Min, lY_Max;
    uint32_t sumX = 0, sumY = 0;

    do {
        XPT2046_ReadAdcXY(&tempAD_X, &tempAD_Y);
        sBufferArray[0][ucCount] = tempAD_X;
        sBufferArray[1][ucCount] = tempAD_Y;
        ucCount++;
    } while ((XPT2046_ReadIRQ() == XPT2046_PENIRQ_Active) && (ucCount < 10));

    if (ucCount < 10)
        return -1;   // 采样不足

    lX_Max = lX_Min = sBufferArray[0][0];
    lY_Max = lY_Min = sBufferArray[1][0];
    sumX = sBufferArray[0][0];
    sumY = sBufferArray[1][0];

    for (i = 1; i < 10; i++) {
        // X
        if (sBufferArray[0][i] < lX_Min) lX_Min = sBufferArray[0][i];
        if (sBufferArray[0][i] > lX_Max) lX_Max = sBufferArray[0][i];
        sumX += sBufferArray[0][i];
        // Y
        if (sBufferArray[1][i] < lY_Min) lY_Min = sBufferArray[1][i];
        if (sBufferArray[1][i] > lY_Max) lY_Max = sBufferArray[1][i];
        sumY += sBufferArray[1][i];
    }

    // 去除最大最小，取平均（右移3位等效除以8，因为10个数去掉2个剩8个）
    pCoord->x = (int16_t)((sumX - lX_Min - lX_Max) >> 3);
    pCoord->y = (int16_t)((sumY - lY_Min - lY_Max) >> 3);
    return 0;
}

// 获取触摸点（校准后）坐标
int8_t XPT2046_GetTouchPoint(XPT2046_Coord *pLCD)
{
    XPT2046_Coord adcCoord;

    if (XPT2046_FilterXY(&adcCoord) < 0)
        return -1;

    // 根据当前扫描模式选择校准因子
    XPT2046_Factor *pF = &TouchFactor[LCD_SCAN_MODE];

    pLCD->x = (int16_t)(pF->A * adcCoord.x + pF->B * adcCoord.y + pF->C);
    pLCD->y = (int16_t)(pF->D * adcCoord.x + pF->E * adcCoord.y + pF->F);

    return 0;
}

// 弱函数：触摸按下回调（默认打印）
__weak void XPT2046_TouchDown(XPT2046_Coord *touch)
{
    printf("TouchDown, x:%d, y:%d\r\n", touch->x, touch->y);
}

// 弱函数：触摸释放回调
__weak void XPT2046_TouchUp(XPT2046_Coord *touch)
{
    printf("TouchUp, x:%d, y:%d\r\n", touch->x, touch->y);
}

// 触摸检测状态机
uint8_t XPT2046_DetectTouch(void)
{
    uint8_t detectResult = TOUCH_NOT_PRESSED;
    uint16_t value = 0;
    static uint32_t i = 0;
    static enumTouchState touch_state = XPT2046_STATE_RELEASE;
    static XPT2046_Coord cinfo = {-1, -1, -1, -1};

    value = XPT2046_ReadIRQ();

    switch (touch_state) {
        case XPT2046_STATE_RELEASE:
            if (value == XPT2046_PENIRQ_Active) {
                touch_state = XPT2046_STATE_WAITING;
                detectResult = TOUCH_NOT_PRESSED;
            } else {
                touch_state = XPT2046_STATE_RELEASE;
                detectResult = TOUCH_NOT_PRESSED;
            }
            break;

        case XPT2046_STATE_WAITING:
            if (value == XPT2046_PENIRQ_Active) {
                i++;
                if (i > DURATION_TIME) {
                    i = 0;
                    touch_state = XPT2046_STATE_PRESSED;
                    detectResult = TOUCH_PRESSED;
                } else {
                    touch_state = XPT2046_STATE_WAITING;
                    detectResult = TOUCH_NOT_PRESSED;
                }
            } else {
                i = 0;
                touch_state = XPT2046_STATE_RELEASE;
                detectResult = TOUCH_NOT_PRESSED;
            }
            break;

        case XPT2046_STATE_PRESSED:
            if (value == XPT2046_PENIRQ_Active) {
                touch_state = XPT2046_STATE_PRESSED;
                detectResult = TOUCH_PRESSED;
            } else {
                touch_state = XPT2046_STATE_RELEASE;
                detectResult = TOUCH_NOT_PRESSED;
            }
            break;

        default:
            touch_state = XPT2046_STATE_RELEASE;
            detectResult = TOUCH_NOT_PRESSED;
            break;
    }

    if (detectResult == TOUCH_PRESSED) {
        if (XPT2046_GetTouchPoint(&cinfo) == 0) {
            // 更新前次坐标
            cinfo.pre_x = cinfo.x;
            cinfo.pre_y = cinfo.y;
            XPT2046_TouchDown(&cinfo);
        }
    } else {
        // 释放处理
        if (cinfo.pre_x != -1 && cinfo.pre_y != -1) {
            XPT2046_TouchUp(&cinfo);
            cinfo.x = -1;
            cinfo.y = -1;
            cinfo.pre_x = -1;
            cinfo.pre_y = -1;
        }
    }
    return detectResult;
}

/* ========================== 主程序 ======================================== */

int main(void)
{
    delay_init();
    UART2_Init(115200);
    printf("XPT2046 Touch Test Start...\r\n");

    XPT2046_Init();

    while (1) {
        XPT2046_DetectTouch();
        delay_ms(50);   // 约50ms轮询一次
    }
}
