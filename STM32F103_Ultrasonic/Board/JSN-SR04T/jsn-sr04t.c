#include "jsn-sr04t.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"

float Distance;



void jsn_sr04t_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 	
	RCC_APB2PeriphClockCmd(UL1_TRIG_CLK, ENABLE);	 //使能PC端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断，需要使能AFIO时钟

	GPIO_InitStructure.GPIO_Pin = UL1_TRIG_PIN | UL2_TRIG_PIN;				 // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(UL1_TRIG_PORT, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC.4
  GPIO_Init(UL2_TRIG_PORT, &GPIO_InitStructure);
	//GPIO_ResetBits(JSN_SR04T_TRIG_PORT,JSN_SR04T_TRIG_PIN);						 //PC4初始化为低电平
	
	GPIO_InitStructure.GPIO_Pin = UL1_ECHO_PIN | UL2_ECHO_PIN;				 // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //下拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(UL1_ECHO_PORT, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC.5
  GPIO_Init(UL2_ECHO_PORT, &GPIO_InitStructure);
	
	
	//GPIOC.5 中断线以及中断初始化配置
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5 | GPIO_PinSource7);

  EXTI_InitStructure.EXTI_Line=EXTI_Line5 | EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);

  TIM3_Int_Init(0xffff,719);  //初始化TIM3定时器，计数一次为1/100000S（10us）
}
	
//发送20us的脉冲触发信号
void jsn_sr04t_start(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	delay_us(15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  delay_ms(25);
  GPIO_SetBits(GPIOA,GPIO_Pin_6);
  delay_us(15);
  GPIO_ResetBits(GPIOA,GPIO_Pin_6);
}



void EXTI9_5_IRQHandler(void)
{			
	//delay_us(10);
	uint16_t tmp_cnt;
  
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		TIM_SetCounter(TIM3,0);
		TIM_Cmd(TIM3,ENABLE);
		
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5))  //
    {
      if(TIM_GetCounter(TIM3) > 1500)
      {
        tmp_cnt = 1500;
        break;
      }
    }
		
		TIM_Cmd(TIM3,DISABLE);
		tmp_cnt = TIM_GetCounter(TIM3);
		Distance = tmp_cnt*340/2000.0;  //cnt * 1/100000 * 340 / 2 *100(单位：cm)
		
		//printf("UL1 Counter:%d\n",TIM_GetCounter(TIM3));
		//if(tmp_cnt >)
		//if(Distance>0)
		//{
			//printf("UL1 Distance:%f cm\r\n",Distance);
		//}
		//Distance = 0;
			
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		TIM_SetCounter(TIM3,0);
		//TIM_Cmd(TIM3,ENABLE);
		
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7));  //
		
		//TIM_Cmd(TIM3,DISABLE);
		tmp_cnt = TIM_GetCounter(TIM3);
    if(tmp_cnt > 117 && tmp_cnt<23529)
    {
		  Distance = tmp_cnt*340/20000.0;  //cnt * 1/10000 * 340 / 2(单位：m)
		}
    else
    {
      Distance = 400;
    }
		printf("UL2 Counter:%d\n",tmp_cnt);
		
		if(Distance>0)
		{
			printf("UL2 Distance:%f cm\r\n",Distance);
		}
		Distance = 0;
			
		EXTI_ClearITPendingBit(EXTI_Line7);
	}  
}

