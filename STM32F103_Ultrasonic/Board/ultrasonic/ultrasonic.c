#include "ultrasonic.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"

float distance_ultrasonic[ULTRASONIC_NUMBER];
uint8_t flag_ultrasonic[ULTRASONIC_NUMBER];

void ULTRASONIC_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 	
	RCC_APB2PeriphClockCmd(UL1_TRIG_CLK, ENABLE);	             //ʹ��PC�˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);        //�ⲿ�жϣ���Ҫʹ��AFIOʱ��

	GPIO_InitStructure.GPIO_Pin = UL1_TRIG_PIN | UL2_TRIG_PIN;	// �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		        //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		        //IO���ٶ�Ϊ50MHz
	GPIO_Init(UL1_TRIG_PORT, &GPIO_InitStructure);					    //�����趨������ʼ��GPIO�˿�
  GPIO_Init(UL2_TRIG_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = UL1_ECHO_PIN | UL2_ECHO_PIN;  // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		          //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		        //IO���ٶ�Ϊ50MHz
	GPIO_Init(UL1_ECHO_PORT, &GPIO_InitStructure);					    //�����趨������ʼ��GPIO�˿�
  GPIO_Init(UL2_ECHO_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(UL1_ECHO_PORT, UL1_ECHO_PIN);
  GPIO_ResetBits(UL2_ECHO_PORT, UL2_ECHO_PIN);
	
	
	//GPIOC.5 �ж����Լ��жϳ�ʼ������
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5 | GPIO_PinSource7);

  EXTI_InitStructure.EXTI_Line=EXTI_Line5 | EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;        //�����ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);                               //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			      //ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);

  TIM3_Int_Init(ULTRASONIC_TIM_ARR,ULTRASONIC_TIM_PSC);                                    //��ʼ��TIM3��ʱ��������һ��Ϊ1/100000S��10us����1/(72M/(719+1))s
}
	
//����d����10us�����崥���ź�
void ULTRASONIC_Measure(void)
{
  /* UL1 */
  distance_ultrasonic[0] = 2;  //�趨��ֵ������̽�����С���룩���������һֱΪ2˵��������ģ��û������
  GPIO_ResetBits(UL1_ECHO_PORT, UL1_ECHO_PIN); //��λECHO���ţ����Է�ֹ�ڴ���ε�������ģ��ʱ����������
  
	GPIO_SetBits(UL1_TRIG_PORT, UL1_TRIG_PIN);
	delay_us(15);
	GPIO_ResetBits(UL1_TRIG_PORT, UL1_TRIG_PIN);
  delay_ms(ULTRASONIC_TIM_MAX_TIME);  //��ǰʱ�����ȷ��һ�β����Ѿ����               //��ʱ�������趨ʱ��ΪΪ�����ʱ��
  
  /* UL2 */
  distance_ultrasonic[1] = 2;
  GPIO_ResetBits(UL2_ECHO_PORT,UL2_ECHO_PIN);
  
  GPIO_SetBits(UL2_TRIG_PORT, UL2_TRIG_PIN);
  delay_us(15);
  GPIO_ResetBits(UL2_TRIG_PORT, UL2_TRIG_PIN);
  delay_ms(ULTRASONIC_TIM_MAX_TIME);
}



void EXTI9_5_IRQHandler(void)
{		
  /*Ultrasonic 1*/	
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line5);
    
		TIM_SetCounter(TIM3,0);
		TIM_Cmd(TIM3,ENABLE);
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5))  
    {
      TIM_SetCounter(TIM3,0);
      TIM_Cmd(TIM3,ENABLE);
      
      if(TIM_GetCounter(TIM3) >= ULTRASONIC_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;
      }
    }
		TIM_Cmd(TIM3,DISABLE);
		distance_ultrasonic[0] = TIM_GetCounter(TIM3) * 340 / 2000.0;  //cnt * 1/100000 * 340 / 2 *100(��λ��cm)
	}
  
  /* Ultrosonic 2 */
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
    EXTI_ClearITPendingBit(EXTI_Line7);
    
		TIM_SetCounter(TIM3,0);
		TIM_Cmd(TIM3,ENABLE);
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7))  
    {
      if(TIM_GetCounter(TIM3) >= ULTRASONIC_TIM_MAX_COUNT) // cnt = 255cm * 2 /(340 * 100) * 100000
      {
        break;
      }
    }
		TIM_Cmd(TIM3,DISABLE);
		distance_ultrasonic[1] = TIM_GetCounter(TIM3) * 340 / 2000.0;  //cnt * 1/100000 * 340 / 2 *100(��λ��cm)
	} 
}

