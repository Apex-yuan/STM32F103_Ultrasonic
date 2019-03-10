#ifndef __JSN_SR04_H
#define __JSN_SR04_H

#include "stm32f10x.h"

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


extern float Distance;


void jsn_sr04t_init(void);
void jsn_sr04t_start(void);



#endif /* __JSN_SR04_H*/





