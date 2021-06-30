#include "Timer.h"

void Timer_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;


  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 48000;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
   
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

/****************************************************************************
* 名    称：delay_ms(u16 nms)
* 功    能：毫秒延时函数
* 入口参数：u16 nms
* 出口参数：无
* 说    明：输入范围
* 调用方法：无 
****************************************************************************/ 
void delay_ms(uint16_t nms)
{

	uint16_t DelayNum;
	uint16_t DelayNum1;
	for(DelayNum = nms;DelayNum >0;DelayNum--)
	    {
		  for(DelayNum1 = 0; DelayNum1<4800;DelayNum1++)
	        {;}
				
			}
}