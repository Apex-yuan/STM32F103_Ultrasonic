#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"
//�˿ں궨��
#define UL1_TRIG_CLK   RCC_APB2Periph_GPIOA
#define UL1_TRIG_PORT  GPIOA
#define UL1_TRIG_PIN   GPIO_Pin_4

#define UL1_ECHO_CLK   RCC_APB2Periph_GPIOA
#define UL1_ECHO_PORT  GPIOA
#define UL1_ECHO_PIN   GPIO_Pin_5

#define UL2_TRIG_CLK   RCC_APB2Periph_GPIOA
#define UL2_TRIG_PORT  GPIOA
#define UL2_TRIG_PIN   GPIO_Pin_6

#define UL2_ECHO_CLK   RCC_APB2Periph_GPIOA
#define UL2_ECHO_PORT  GPIOA
#define UL2_ECHO_PIN   GPIO_Pin_7


//
#define ULTRASONIC_NUMBER 2
#define ULTRASONIC_TIM_ARR 0XFFFF
#define ULTRASONIC_TIM_PSC 719  //����Ƶ�ʣ�72M/(719+1) = 100000������һ������ʱ��Ϊ��1S/100000=10us
#define MAX_MEASURE_DISTANCE 255  //��λ��cm
#define ULTRASONIC_TIM_MAX_TIME (15+5) //���������������ʱ��(��λ��ms)��MAX_MEASURE_DISTANCE * 2 /100 / 340 * 1000
#define ULTRASONIC_TIM_MAX_COUNT 1500

extern float distance_ultrasonic[2];


void ULTRASONIC_Init(void);
void ULTRASONIC_Measure(void);



#endif /* __ULTRASONIC_H*/





