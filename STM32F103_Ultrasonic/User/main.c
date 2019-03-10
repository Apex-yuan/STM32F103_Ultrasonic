#include "stm32f10x.h"
#include "usart.h"
#include "delay.h"
#include "ultrasonic.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  usart1_init(115200);
  delay_init();
	ULTRASONIC_Init();
  
  printf("**³¬Éù²¨Ä£¿é²âÊÔ**\r\n");
  
  while(1)
  {
	  ULTRASONIC_Measure();
    printf("UL1 Distance:%.2fcm\r\n",distance_ultrasonic[0]);
    printf("UL2 Distance:%.2fcm\r\n",distance_ultrasonic[1]);
	  delay_ms(500);
  }
}
