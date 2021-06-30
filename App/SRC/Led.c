#include "Led.h"

void usrGPIO_Init(void)
{
/***********************************************************
	        �������
	  FEED_DOG PREDSG PRECHG UDSG BLEED HEAT AFE_REF(����) EN_5V EN_POW EN_CAN BUZ LED1~7
**********************************************************/
   	GPIO_InitTypeDef GPIO_InitStr; 
	  GPIO_DeInit(GPIOA);
	  GPIO_DeInit(GPIOB);
	  GPIO_DeInit(GPIOC);
	  GPIO_DeInit(GPIOD);
	  GPIO_DeInit(GPIOE);
	  GPIO_DeInit(GPIOF);
	  GPIO_InitStr.GPIO_Mode = GPIO_Mode_OUT;//���
	  GPIO_InitStr.GPIO_OType = GPIO_OType_PP;//����
	  GPIO_InitStr.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;
	  GPIO_InitStr.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
	  GPIO_InitStr.GPIO_Speed = GPIO_Speed_Level_3;
	  GPIO_Init(GPIOA,&GPIO_InitStr); 
	
	GPIO_InitStr.GPIO_Pin = LED_W1|LED_W2|LED_W3|LED_W4|btdate;           	  /*ѡ��Ҫ���Ƶ�GPIOE����*/				
	GPIO_InitStr.GPIO_Mode=GPIO_Mode_OUT;       	/*����Ҫ���Ƶ�GPIOE����Ϊ���ģʽ*/	
	GPIO_InitStr.GPIO_Speed = GPIO_Speed_50MHz;  	/*������������Ϊ50MHz */ 
	GPIO_InitStr.GPIO_OType = GPIO_OType_OD; 	    /*��������ģʽΪͨ���������*/
	GPIO_InitStr.GPIO_PuPd   = GPIO_PuPd_UP;      /*��������ģʽΪ����*/
	GPIO_Init(GPIOx_W1_4, &GPIO_InitStr);	         /*���ÿ⺯������ʼ��GPIOB*/
	GPIO_ResetBits(GPIOx_W1_4,LED_W1|LED_W2|LED_W3|LED_W4);
	GPIO_ResetBits(GPIOB,btdate);
	
	GPIO_InitStr.GPIO_Pin = LED_W5|LED_W6;           	  /*ѡ��Ҫ���Ƶ�GPIOA����*/				
	GPIO_Init(GPIOx_W5_6, &GPIO_InitStr);	         /*���ÿ⺯������ʼ��GPIOA*/	
	GPIO_SetBits(GPIOx_W5_6,LED_W5|LED_W6);
	
	GPIO_InitStr.GPIO_Pin = LED_W7;           	  	/*ѡ��Ҫ���Ƶ�GPIOA����*/				
	GPIO_Init(GPIOx_W7, &GPIO_InitStr);	         	/*���ÿ⺯������ʼ��GPIOA*/
	GPIO_SetBits(GPIOx_W7,LED_W7);
	
	 GPIO_InitStr.GPIO_Pin = GPIO_Pin_4;            //�ܵ�Դ����
	 GPIO_InitStr.GPIO_OType = GPIO_OType_PP;
	 GPIO_Init(GPIOB,&GPIO_InitStr);	
	 GPIO_SetBits(GPIOB, GPIO_Pin_4);   //���ܵ�Դ
	  
	  GPIO_ResetBits(GPIOA,GPIO_Pin_2); //�ս�
	  GPIO_ResetBits(GPIOA,GPIO_Pin_3);  
	  GPIO_ResetBits(GPIOA,GPIO_Pin_4);  
	  GPIO_ResetBits(GPIOA,GPIO_Pin_6);	//BQ_WK �ȹر�BQ76940 
	  GPIO_ResetBits(GPIOA,GPIO_Pin_7);  //PDHG
	  

	  GPIO_ResetBits(GPIOA,GPIO_Pin_15);  //�ս�



	  GPIO_InitStr.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	  GPIO_Init(GPIOB,&GPIO_InitStr);
		

		
	    GPIO_SetBits(GPIOB,GPIO_Pin_0);  //PCHG
		  GPIO_SetBits(GPIOB,GPIO_Pin_1);  //UVP_DSG
		GPIO_ResetBits(GPIOB,GPIO_Pin_2);  //BQ_I2C_PULL    //BQ34Z100 �������ر����ʹ��
		GPIO_ResetBits(GPIOB,GPIO_Pin_3);  //LED3
		//GPIO_ResetBits(GPIOB,GPIO_Pin_4);  //LED4
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);  //AFE_I2C_PULL //BP76940 �������ر����ʹ��
	

		GPIO_InitStr.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOC,&GPIO_InitStr);
		GPIO_ResetBits(GPIOC,GPIO_Pin_13); //�ս�

		GPIO_InitStr.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
		GPIO_Init(GPIOF,&GPIO_InitStr);
		GPIO_ResetBits(GPIOF,GPIO_Pin_0|GPIO_Pin_1); //�ս�




	 
/************************************************************
					��������
************************************************************/
//     GPIO_InitStr.GPIO_Mode = GPIO_Mode_IN;
//	 GPIO_InitStr.GPIO_Pin = GPIO_Pin_9;  //RS232_AWK //���� KEY0_BOTTOM
//	 GPIO_InitStr.GPIO_PuPd = GPIO_PuPd_UP;//��������
//	 GPIO_InitStr.GPIO_Speed = GPIO_Speed_Level_3;
//    GPIO_Init(GPIOA,&GPIO_InitStr);


		 
//	 GPIO_InitStr.GPIO_PuPd = GPIO_PuPd_NOPULL;//��������
//	 GPIO_InitStr.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_11; //�����CHG_WK
//	 GPIO_Init(GPIOB,&GPIO_InitStr); 
//	 GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_11); 


  //   GPIO_InitStr.GPIO_Pin = GPIO_Pin_12; 
  //   GPIO_Init(GPIOA,&GPIO_InitStr);//ALERT  //BQ76940�쳣���ܽ� 

 
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







