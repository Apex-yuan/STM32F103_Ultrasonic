#include "ultrasonic.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"

UltrasonicDef ultrasonic[2];

//float distance_ultrasonic[ULTRASONIC_NUMBER];
//uint8_t flag_ultrasonic[ULTRASONIC_NUMBER];

//uint32_t start_time[2];
//uint32_t end_time[2];

void ULTRASONIC_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 	
	RCC_APB2PeriphClockCmd(UL1_TRIG_CLK, ENABLE);	             //使能PC端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);        //外部中断，需要使能AFIO时钟

	GPIO_InitStructure.GPIO_Pin = UL1_TRIG_PIN | UL2_TRIG_PIN;	// 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		        //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		        //IO口速度为50MHz
	GPIO_Init(UL1_TRIG_PORT, &GPIO_InitStructure);					    //根据设定参数初始化GPIO端口
  GPIO_Init(UL2_TRIG_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = UL1_ECHO_PIN | UL2_ECHO_PIN;  // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		          //下拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		        //IO口速度为50MHz
	GPIO_Init(UL1_ECHO_PORT, &GPIO_InitStructure);					    //根据设定参数初始化GPIO端口
  GPIO_Init(UL2_ECHO_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(UL1_ECHO_PORT, UL1_ECHO_PIN);                //设置ECHO脚的初始状态
	
	
	//GPIOC.5 中断线以及中断初始化配置
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5 | GPIO_PinSource7);

  EXTI_InitStructure.EXTI_Line=EXTI_Line5 | EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;        //上升沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);                               //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			      //使能按键所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);

  TIM3_Int_Init(9,71);                                    //初始化TIM3定时器，计数一次为1/100000S（10us）：1/(72M/(719+1))s
}
	
//发送d大于10us的脉冲触发信号
void ULTRASONIC_Measure(void)
{
  /* UL1 */
  //distance_ultrasonic[0] = 2;  //设定初值（可以探测的最小距离），如果数据一直为2说明超声波模块没有连接
//  GPIO_ResetBits(UL1_ECHO_PORT, UL1_ECHO_PIN); //复位ECHO引脚，可以防止在带电拔掉超声波模块时程序死掉。
  
	GPIO_SetBits(UL1_TRIG_PORT, UL1_TRIG_PIN);
	delay_us(15);
	GPIO_ResetBits(UL1_TRIG_PORT, UL1_TRIG_PIN);
  //delay_ms(ULTRASONIC_TIM_MAX_TIME);  //当前时间可以确保一次测量已经完成               //分时测量，设定时间为为回响最长时间
  
  /* UL2 */
  //distance_ultrasonic[1] = 2;
//  GPIO_ResetBits(UL2_ECHO_PORT,UL2_ECHO_PIN);
  
  GPIO_SetBits(UL2_TRIG_PORT, UL2_TRIG_PIN);
  delay_us(15);
  GPIO_ResetBits(UL2_TRIG_PORT, UL2_TRIG_PIN);
 // delay_ms(ULTRASONIC_TIM_MAX_TIME);
}



void EXTI9_5_IRQHandler(void)
{		
  /*Ultrasonic 1*/	
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line5);	 
		if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)))  //read rising 
    {
      ultrasonic[0].flag = 1;
      ultrasonic[0].start_time = millis();
    }
    else   //read falling
    {
      if(1 == ultrasonic[0].flag)
      {
        ultrasonic[0].flag = 0;
        ultrasonic[0].end_time = millis();
        if(ultrasonic[0].end_time - ultrasonic[0].start_time > 1500)  //cnt overflow
        {
          ultrasonic[0].distance = MAX_MEASURE_DISTANCE;
        }
        else
        {
          ultrasonic[0].distance = (ultrasonic[0].end_time - ultrasonic[0].start_time) * 340 / 2000.0;
        }
      }
    }
	}
  /*Ultrasonic 1*/	
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line7);	 
		if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)))  //read rising 
    {
      ultrasonic[1].flag = 1;
      ultrasonic[1].start_time = millis();
    }
    else   //read falling
    {
      if(1 == ultrasonic[1].flag)
      {
        ultrasonic[1].flag = 0;
        ultrasonic[1].end_time = millis();
        if(ultrasonic[1].end_time - ultrasonic[1].start_time > 1500)  //cnt overflow
        {
          ultrasonic[1].distance = MAX_MEASURE_DISTANCE;
        }
        else
        {
          ultrasonic[1].distance = (ultrasonic[1].end_time - ultrasonic[1].start_time) * 340 / 2000.0;
        }
      }
    }
	}
//  /* Ultrasonic 2 */
//  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
//	{
//    EXTI_ClearITPendingBit(EXTI_Line7);	 
//		if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)))  //read rising 
//    {
//      TIM_SetCounter(TIM3,0);
//      TIM_Cmd(TIM3,ENABLE);
//      flag_ultrasonic[1] = 1;
//    }
//    else   //read falling
//    {
//      if(flag_ultrasonic[1])
//      {
//        TIM_Cmd(TIM3, DISABLE);    //close timer
//        flag_ultrasonic[1] = 0;       //clear flag
//        if(TIM_GetCounter(TIM3) >= ULTRASONIC_TIM_MAX_COUNT)  //cnt overflow
//        {
//          distance_ultrasonic[1] = MAX_MEASURE_DISTANCE;
//        }
//        else
//        {
//          distance_ultrasonic[1] = TIM_GetCounter(TIM3) * 340 / 2000.0;
//        }
//      }
//    }
//	}
}

