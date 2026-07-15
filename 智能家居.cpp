#include "stm32f4xx.h"

//系统基础延时
void Delay_ms(uint32_t ms)
{
	uint32_t i,j;
	for(i = 0; i < ms; i++)
		for(j = 0; j < 7200; j++);
}

//外设统一初始化：红外输入、双LED、蜂鸣器
void GPIO_All_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//PA0 人体红外输入
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//PB0绿 PB1红 LED输出
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);

	//PA8 有源蜂鸣器输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

//红外传感器10ms软件防抖采集
uint8_t Read_Human_Sensor(void)
{
	uint8_t val1 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
	Delay_ms(10);
	uint8_t val2 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
	if(val1 == val2)
		return val1;
	return 0;
}

//全部外设关闭：绿灯、红灯、蜂鸣器
void Close_All_Device(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

//灯光状态控制函数
void Led_Ctrl(uint8_t mode)
{
	switch(mode)
	{
		case 0: //全灭
			Close_All_Device();
			break;
		case 1: //仅绿灯亮
			GPIO_ResetBits(GPIOB, GPIO_Pin_0);
			GPIO_SetBits(GPIOB, GPIO_Pin_1);
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			break;
		case 2: //红绿同亮 + 间歇长短蜂鸣
			GPIO_ResetBits(GPIOB, GPIO_Pin_0);
			GPIO_ResetBits(GPIOB, GPIO_Pin_1);
			//短滴
			GPIO_ResetBits(GPIOA, GPIO_Pin_8);
			Delay_ms(150);
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			Delay_ms(200);
			//长滴
			GPIO_ResetBits(GPIOA, GPIO_Pin_8);
			Delay_ms(300);
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			Delay_ms(200);
			break;
		default:
			Close_All_Device();
			break;
	}
}

//开机自检灯光闪烁
void Self_Check(void)
{
	uint8_t i;
	for(i=0;i<3;i++)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
		Delay_ms(200);
		GPIO_SetBits(GPIOB, GPIO_Pin_0);
		Delay_ms(200);
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
		Delay_ms(200);
		GPIO_SetBits(GPIOB, GPIO_Pin_1);
		Delay_ms(200);
	}
	Close_All_Device();
}

//全局状态结构体，统一管理计时与标志
typedef struct
{
	uint8_t human_stable;    //稳定人体信号标志
	uint8_t has_people;      //区域内有人自锁标志
	uint32_t stay_cnt;       //连续停留计时器
	uint32_t leave_cnt;      //离开倒计时计时器
	uint8_t over_one_minute; //是否满一分钟标志
}Sys_State;

int main(void)
{
	Sys_State sys;
	//初始化结构体全部变量清零
	sys.human_stable = 0;
	sys.has_people = 0;
	sys.stay_cnt = 0;
	sys.leave_cnt = 0;
	sys.over_one_minute = 0;

	GPIO_All_Init();
	Close_All_Device();
	Self_Check(); //上电自检

	while(1)
	{
		//1、采集防抖后的人体信号
		sys.human_stable = Read_Human_Sensor();

		if(sys.human_stable == 1)
		{
			sys.has_people = 1;
			sys.leave_cnt = 0; //有人则清空离开倒计时
			//计数器上限保护，防止溢出
			if(sys.stay_cnt < 6000)
				sys.stay_cnt++;

			//判断是否连续停留满60秒 6000*10ms=60000ms=1min
			if(sys.stay_cnt >= 6000)
			{
				sys.over_one_minute = 1;
				Led_Ctrl(2); //红绿同亮+蜂鸣
			}
			else
			{
				sys.over_one_minute = 0;
				Led_Ctrl(1); //仅绿灯照明
			}
		}
		else
		{
			//人体信号消失，清空停留计时与一分钟标志
			sys.stay_cnt = 0;
			sys.over_one_minute = 0;

			if(sys.has_people == 1)
			{
				//人刚离开，启动3秒延时关灯
				sys.leave_cnt++;
				if(sys.leave_cnt >= 300) //300*10ms=3s
				{
					sys.has_people = 0;
					sys.leave_cnt = 0;
					Led_Ctrl(0);
				}
			}
			else
			{
				//长期无人，保持全部关闭
				Led_Ctrl(0);
			}
		}
		Delay_ms(10); //系统基础扫描节拍10ms
	}
}
