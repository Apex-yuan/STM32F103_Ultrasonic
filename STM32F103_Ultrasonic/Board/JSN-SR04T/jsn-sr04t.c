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
 	
	RCC_APB2PeriphClockCmd(UL1_TRIG_CLK, ENABLE);	 //ʹ��PC�˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//�ⲿ�жϣ���Ҫʹ��AFIOʱ��

	GPIO_InitStructure.GPIO_Pin = UL1_TRIG_PIN | UL2_TRIG_PIN;				 // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(UL1_TRIG_PORT, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOC.4
  GPIO_Init(UL2_TRIG_PORT, &GPIO_InitStructure);
	//GPIO_ResetBits(JSN_SR04T_TRIG_PORT,JSN_SR04T_TRIG_PIN);						 //PC4��ʼ��Ϊ�͵�ƽ
	
	GPIO_InitStructure.GPIO_Pin = UL1_ECHO_PIN | UL2_ECHO_PIN;				 // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(UL1_ECHO_PORT, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOC.5
  GPIO_Init(UL2_ECHO_PORT, &GPIO_InitStructure);
	
	
	//GPIOC.5 �ж����Լ��жϳ�ʼ������
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5 | GPIO_PinSource7);

  EXTI_InitStructure.EXTI_Line=EXTI_Line5 | EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//�����ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);

  TIM3_Int_Init(0xffff,719);  //��ʼ��TIM3��ʱ��������һ��Ϊ1/100000S��10us��
}
	
//����20us�����崥���ź�
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
		Distance = tmp_cnt*340/2000.0;  //cnt * 1/100000 * 340 / 2 *100(��λ��cm)
		
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
		  Distance = tmp_cnt*340/20000.0;  //cnt * 1/10000 * 340 / 2(��λ��m)
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

