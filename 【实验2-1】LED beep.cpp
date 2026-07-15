#include "stm32f4xx.h"


GPIO_InitTypeDef GPIO_InitStructure;

int main(void)
{
    // 1. 开启 GPIOB 时钟（F4必须用 AHB1）
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // 2. 配置 PB0、PB1、PB5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // 3. 初始化
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 4. 点亮所有 LED（低电平有效）
    GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5);
	     /*开启 GPIOA 的时钟，才能使用 GPIOA*/
   RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA,ENABLE); //AHB1上开启GPIOA时钟
  
   /*配置蜂鸣器所在的GPIO（PA8）*/
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //要初始化的引脚号
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //GPIO设置为输出模式
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //GPIO设置为推挽模式
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;  //速度设置为25MHz
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  //没有上拉下拉电阻
   GPIO_SetBits(GPIOA, GPIO_Pin_8);//预先设置为高电平，蜂鸣器是不响的
   GPIO_Init( GPIOA,  &GPIO_InitStructure );  //使用上述的数据，初始化GPIOB

   /*PA8输出低电平，蜂鸣器响*/
   GPIO_ResetBits(GPIOA, GPIO_Pin_8);
  
   /*如果PA8输出高电平，蜂鸣器停止鸣响*/
//  GPIO_SetBits(GPIOA, GPIO_Pin_8);

    while(1)
    {
        // 保持常亮
    }
}
