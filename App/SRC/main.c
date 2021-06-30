#include "Main.h"


typedef  void (*pFunction)(void);
unsigned int APP_FLG_ADDR=0x0801FC00;//APP标志位存放位置
unsigned short FLASH_APP_FLG;
unsigned char BaudRate_115200[]="TTM:BPS-4";
unsigned char BT_RESET[]="TTM:RST-SYS";
uint32_t JumpAddress;
pFunction Jump_To_Application;

void usrRCC_Configuration(void)
{
	/* Configure the System clock frequency, AHB/APBx prescalers and Flash settings */
      SystemInit();

	  RCC_PLLCmd(DISABLE);
	 // RCC_PLLConfig(RCC_PLLSource_HSE,RCC_PLLMul_5);  //睿频到40MHZ的系统时钟
	  RCC_PLLConfig(RCC_PLLSource_HSI,RCC_PLLMul_6);  //睿频到48MHZ的系统时钟

	  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //SYS时钟选择为PLL
	  RCC_HCLKConfig(RCC_SYSCLK_Div1);  //AHB时钟为SYSCLK/ 48MHZ
	  RCC_PCLKConfig(RCC_SYSCLK_Div1);  //APB时钟为HCLK/  48MHZ
//	  RCC_ADCCLKConfig(RCC_ADCCLK_HSI14); //14MHZ的ADC时钟
	  RCC_PLLCmd(ENABLE);  //等待100ms

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  
//      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);	  //CAN RCC
}

static void NVIC_Configuration(void)//中断优先级初始化函数
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;   //TIME
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//CAN
	NVIC_InitStructure.NVIC_IRQChannel = CEC_CAN_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


int main(void)
{
	static uint8_t falg=0;
	usrRCC_Configuration();	  							// 系统时钟
	FLASH_SetLatency(FLASH_Latency_1); 					//1个Flash访问延时，由STM公司推荐
	usrGPIO_Init();  									//GPIO初始化
	Uart_Init(9600);     								//串口初始化9600
	Timer_Init();    									//定时器初始化
	CAN_Configation();									//CAN初始化
	NVIC_Configuration();								//中断初始化
    GPIO_SetBits(GPIOx_W1_4,LED_W2|LED_W3|LED_W4);
    GPIO_SetBits(GPIOx_W5_6,LED_W5|LED_W6);
    GPIO_SetBits(GPIOx_W7,LED_W7);
	bLedDisSign =1;
//////////////////////////////////////////////////////////连续使用9600波特率发送修改波特率指令，后复位，不做校对//////////////////////////////////////////
	delay_ms(500);
	USART_Send_string(BaudRate_115200,9);
	delay_ms(100);
	USART_Send_string(BaudRate_115200,9);
	delay_ms(100);
	USART_Send_string(BT_RESET,11);
	delay_ms(100);
	USART_Send_string(BT_RESET,11);
	delay_ms(100);
////////////////////////////////////////////////////////
    Uart_Init(115200);     //串口初始化115200
    //WWDG_Config();   //看门狗初始化
	/*在这里读取程序更新允许标志，没有应用则进入for循环，有应用程序直接跳转到APP*/
	/*bootloader更新完成，跳转到应用程序入口*/
boolkka:
	TIM_Cmd(TIM3, DISABLE); 
	FLASH_APP_FLG = STMFLASH_ReadHalfWord(APP_FLG_ADDR);
//	FLASH_test=(*(uint32_t *)APP_ADDR);
//	FLASH_test1=(*(uint32_t *)(APP_ADDR+4));
	TIM_Cmd(TIM3, ENABLE); 
	//if(((*(uint32_t *)APP_ADDR)&0x2FFE0000)==0X20000000)//app应用已经存在
	if((FLASH_APP_FLG==0x55AA)&&(((*(uint32_t *)APP_ADDR)&0x2FFE0000)==0X20000000))//app应用已经存在
	{
		CAN_DeInit(CAN);
		USART_DeInit(USART1);
	    TIM_DeInit(TIM3);
		GPIO_DeInit(GPIOA);                                                
		GPIO_DeInit(GPIOB);
		GPIO_DeInit(GPIOC);
		GPIO_DeInit(GPIOD);
		GPIO_DeInit(GPIOE);
		GPIO_DeInit(GPIOF);

		JumpAddress = *(__IO uint32_t*) (APP_ADDR + 4);
		Jump_To_Application = (pFunction) JumpAddress;

		/* Initialize user application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*) APP_ADDR);
  
		/* Jump to application */
		Jump_To_Application();
	}
	while(1)
	{
		 
		if(falg== 0)
		{
			GPIO_ResetBits(GPIOx_W1_4,LED_W1);
			GPIO_SetBits(GPIOx_W1_4,LED_W2|LED_W3|LED_W4);
			GPIO_SetBits(GPIOx_W5_6,LED_W5|LED_W6);
			GPIO_SetBits(GPIOx_W7,LED_W7);
			if(led_period_cnt >= 6000)
			{
				 falg =1;
				 led_period_cnt = 0;
			}
		}
			 
		//WWDG_SetCounter(WDG_ReloadValue);  //清看门狗
		USART_FRAME_PROCESS();  //处理串口收到的每一帧数据
		//UsartToCAN_Process();    //串口接收到的数据发送到CAN
		if(CAN_send_period_cnt >=1)// 1ms发送1包
		{
			//BOOT_TIME_OUT_cnt++;
			CAN_send_period_cnt = 0;
			UAV_CAN_send();	  //定时发送CAN数据
		}	

		if((led_period_cnt >= 300)&&(falg))
		{
			led_period_cnt=0;
			Led_Cpl();	   //灯处理
			//USART_Send_status(USART_Operate_ERR);//返回操作失败 
			// USART_Send_status(USART_SUCCESS);   
		}							
				
		CAN_FRAME_PROCESS();  //处理接收到的每一个UAVCAN数据帧
		//CANToUsart_Process();    //CAN接收到的数据发送到串口
		/*bootloader更新完成，跳转到应用程序入口*/
		if(FLASH_Program_finish_flg==1)//程序更新完成
		{  
			TIM_Cmd(TIM3, DISABLE); 
			FLASH_Unlock();//解锁
			FLASH_ErasePage(APP_FLG_ADDR);
			FLASH_Lock();//重新上锁
			FLASH_Unlock();//解锁
			FLASH_ProgramWord(APP_FLG_ADDR,0x55AA);//写入应用程序已存在标志
			FLASH_Lock();//重新上锁
			TIM_Cmd(TIM3, ENABLE); 
			FLASH_Program_finish_flg = 0;
			goto boolkka;
		}	           		
	}
}




