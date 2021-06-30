#include "Led.h"

void usrGPIO_Init(void)
{
/***********************************************************
	        推挽输出
	  FEED_DOG PREDSG PRECHG UDSG BLEED HEAT AFE_REF(上拉) EN_5V EN_POW EN_CAN BUZ LED1~7
**********************************************************/
   	GPIO_InitTypeDef GPIO_InitStr; 
	  GPIO_DeInit(GPIOA);
	  GPIO_DeInit(GPIOB);
	  GPIO_DeInit(GPIOC);
	  GPIO_DeInit(GPIOD);
	  GPIO_DeInit(GPIOE);
	  GPIO_DeInit(GPIOF);
	  GPIO_InitStr.GPIO_Mode = GPIO_Mode_OUT;//输出
	  GPIO_InitStr.GPIO_OType = GPIO_OType_PP;//推挽
	  GPIO_InitStr.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;
	  GPIO_InitStr.GPIO_PuPd = GPIO_PuPd_NOPULL;//浮空
	  GPIO_InitStr.GPIO_Speed = GPIO_Speed_Level_3;
	  GPIO_Init(GPIOA,&GPIO_InitStr); 
	
	GPIO_InitStr.GPIO_Pin = LED_W1|LED_W2|LED_W3|LED_W4|btdate;           	  /*选择要控制的GPIOE引脚*/				
	GPIO_InitStr.GPIO_Mode=GPIO_Mode_OUT;       	/*设置要控制的GPIOE引脚为输出模式*/	
	GPIO_InitStr.GPIO_Speed = GPIO_Speed_50MHz;  	/*设置引脚速率为50MHz */ 
	GPIO_InitStr.GPIO_OType = GPIO_OType_OD; 	    /*设置引脚模式为通用推挽输出*/
	GPIO_InitStr.GPIO_PuPd   = GPIO_PuPd_UP;      /*设置引脚模式为上拉*/
	GPIO_Init(GPIOx_W1_4, &GPIO_InitStr);	         /*调用库函数，初始化GPIOB*/
	GPIO_ResetBits(GPIOx_W1_4,LED_W1|LED_W2|LED_W3|LED_W4);
	GPIO_ResetBits(GPIOB,btdate);
	
	GPIO_InitStr.GPIO_Pin = LED_W5|LED_W6;           	  /*选择要控制的GPIOA引脚*/				
	GPIO_Init(GPIOx_W5_6, &GPIO_InitStr);	         /*调用库函数，初始化GPIOA*/	
	GPIO_SetBits(GPIOx_W5_6,LED_W5|LED_W6);
	
	GPIO_InitStr.GPIO_Pin = LED_W7;           	  	/*选择要控制的GPIOA引脚*/				
	GPIO_Init(GPIOx_W7, &GPIO_InitStr);	         	/*调用库函数，初始化GPIOA*/
	GPIO_SetBits(GPIOx_W7,LED_W7);
	
	 GPIO_InitStr.GPIO_Pin = GPIO_Pin_4;            //总电源控制
	 GPIO_InitStr.GPIO_OType = GPIO_OType_PP;
	 GPIO_Init(GPIOB,&GPIO_InitStr);	
	 GPIO_SetBits(GPIOB, GPIO_Pin_4);   //打开总电源
	  
	  GPIO_ResetBits(GPIOA,GPIO_Pin_2); //空脚
	  GPIO_ResetBits(GPIOA,GPIO_Pin_3);  
	  GPIO_ResetBits(GPIOA,GPIO_Pin_4);  
	  GPIO_ResetBits(GPIOA,GPIO_Pin_6);	//BQ_WK 先关闭BQ76940 
	  GPIO_ResetBits(GPIOA,GPIO_Pin_7);  //PDHG
	  

	  GPIO_ResetBits(GPIOA,GPIO_Pin_15);  //空脚



	  GPIO_InitStr.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	  GPIO_Init(GPIOB,&GPIO_InitStr);
		

		
	    GPIO_SetBits(GPIOB,GPIO_Pin_0);  //PCHG
		  GPIO_SetBits(GPIOB,GPIO_Pin_1);  //UVP_DSG
		GPIO_ResetBits(GPIOB,GPIO_Pin_2);  //BQ_I2C_PULL    //BQ34Z100 先下拉关闭输出使能
		GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //LED3
		//GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //LED4
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);  //AFE_I2C_PULL //BP76940 先下拉关闭输出使能
	

		GPIO_InitStr.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOC,&GPIO_InitStr);
		GPIO_ResetBits(GPIOC,GPIO_Pin_13); //空脚

		GPIO_InitStr.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
		GPIO_Init(GPIOF,&GPIO_InitStr);
		GPIO_ResetBits(GPIOF,GPIO_Pin_0|GPIO_Pin_1); //空脚




	 
/************************************************************
					输入配置
************************************************************/
//     GPIO_InitStr.GPIO_Mode = GPIO_Mode_IN;
//	 GPIO_InitStr.GPIO_Pin = GPIO_Pin_9;  //RS232_AWK //按键 KEY0_BOTTOM
//	 GPIO_InitStr.GPIO_PuPd = GPIO_PuPd_UP;//上拉输入
//	 GPIO_InitStr.GPIO_Speed = GPIO_Speed_Level_3;
//    GPIO_Init(GPIOA,&GPIO_InitStr);


		 
//	 GPIO_InitStr.GPIO_PuPd = GPIO_PuPd_NOPULL;//浮空输入
//	 GPIO_InitStr.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_11; //充电检测CHG_WK
//	 GPIO_Init(GPIOB,&GPIO_InitStr); 
//	 GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_11); 


  //   GPIO_InitStr.GPIO_Pin = GPIO_Pin_12; 
  //   GPIO_Init(GPIOA,&GPIO_InitStr);//ALERT  //BQ76940异常检测管脚 

 
}
//======================================================
void Led_Cpl(void)
{
		if(bLedDisSign == 1)
		{
					if(bLedDisSign_Cpl==1)
						{
							bLedDisSign_Cpl=0;
								All_LED_H;
						}
						else
						{
								bLedDisSign_Cpl=1;		
								All_LED_L;
						}					
			}
			else
			{
					All_LED_H;
			}

}			 
volatile uint8_t LedDisNowNum;
volatile uint8_t LedDisVal;
volatile uint8_t bLedDisSign; 
volatile uint8_t bLedDisSign_Cpl; 

volatile uint16_t led_period_cnt ;
volatile uint16_t CAN_send_period_cnt ;
//volatile uint16_t BOOT_TIME_OUT_cnt;
volatile uint8_t b10msOnSign;
volatile uint16_t DelayNum;







