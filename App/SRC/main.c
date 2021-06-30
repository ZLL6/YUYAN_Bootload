#include "Main.h"


typedef  void (*pFunction)(void);
unsigned int APP_FLG_ADDR=0x0801FC00;//APP��־λ���λ��
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
	 // RCC_PLLConfig(RCC_PLLSource_HSE,RCC_PLLMul_5);  //�Ƶ��40MHZ��ϵͳʱ��
	  RCC_PLLConfig(RCC_PLLSource_HSI,RCC_PLLMul_6);  //�Ƶ��48MHZ��ϵͳʱ��

	  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //SYSʱ��ѡ��ΪPLL
	  RCC_HCLKConfig(RCC_SYSCLK_Div1);  //AHBʱ��ΪSYSCLK/ 48MHZ
	  RCC_PCLKConfig(RCC_SYSCLK_Div1);  //APBʱ��ΪHCLK/  48MHZ
//	  RCC_ADCCLKConfig(RCC_ADCCLK_HSI14); //14MHZ��ADCʱ��
	  RCC_PLLCmd(ENABLE);  //�ȴ�100ms

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

static void NVIC_Configuration(void)//�ж����ȼ���ʼ������
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
	usrRCC_Configuration();	  							// ϵͳʱ��
	FLASH_SetLatency(FLASH_Latency_1); 					//1��Flash������ʱ����STM��˾�Ƽ�
	usrGPIO_Init();  									//GPIO��ʼ��
	Uart_Init(9600);     								//���ڳ�ʼ��9600
	Timer_Init();    									//��ʱ����ʼ��
	CAN_Configation();									//CAN��ʼ��
	NVIC_Configuration();								//�жϳ�ʼ��
    GPIO_SetBits(GPIOx_W1_4,LED_W2|LED_W3|LED_W4);
    GPIO_SetBits(GPIOx_W5_6,LED_W5|LED_W6);
    GPIO_SetBits(GPIOx_W7,LED_W7);
	bLedDisSign =1;
//////////////////////////////////////////////////////////����ʹ��9600�����ʷ����޸Ĳ�����ָ���λ������У��//////////////////////////////////////////
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
    Uart_Init(115200);     //���ڳ�ʼ��115200
    //WWDG_Config();   //���Ź���ʼ��
	/*�������ȡ������������־��û��Ӧ�������forѭ������Ӧ�ó���ֱ����ת��APP*/
	/*bootloader������ɣ���ת��Ӧ�ó������*/
boolkka:
	TIM_Cmd(TIM3, DISABLE); 
	FLASH_APP_FLG = STMFLASH_ReadHalfWord(APP_FLG_ADDR);
//	FLASH_test=(*(uint32_t *)APP_ADDR);
//	FLASH_test1=(*(uint32_t *)(APP_ADDR+4));
	TIM_Cmd(TIM3, ENABLE); 
	//if(((*(uint32_t *)APP_ADDR)&0x2FFE0000)==0X20000000)//appӦ���Ѿ�����
	if((FLASH_APP_FLG==0x55AA)&&(((*(uint32_t *)APP_ADDR)&0x2FFE0000)==0X20000000))//appӦ���Ѿ�����
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
			 
		//WWDG_SetCounter(WDG_ReloadValue);  //�忴�Ź�
		USART_FRAME_PROCESS();  //�������յ���ÿһ֡����
		//UsartToCAN_Process();    //���ڽ��յ������ݷ��͵�CAN
		if(CAN_send_period_cnt >=1)// 1ms����1��
		{
			//BOOT_TIME_OUT_cnt++;
			CAN_send_period_cnt = 0;
			UAV_CAN_send();	  //��ʱ����CAN����
		}	

		if((led_period_cnt >= 300)&&(falg))
		{
			led_period_cnt=0;
			Led_Cpl();	   //�ƴ���
			//USART_Send_status(USART_Operate_ERR);//���ز���ʧ�� 
			// USART_Send_status(USART_SUCCESS);   
		}							
				
		CAN_FRAME_PROCESS();  //������յ���ÿһ��UAVCAN����֡
		//CANToUsart_Process();    //CAN���յ������ݷ��͵�����
		/*bootloader������ɣ���ת��Ӧ�ó������*/
		if(FLASH_Program_finish_flg==1)//����������
		{  
			TIM_Cmd(TIM3, DISABLE); 
			FLASH_Unlock();//����
			FLASH_ErasePage(APP_FLG_ADDR);
			FLASH_Lock();//��������
			FLASH_Unlock();//����
			FLASH_ProgramWord(APP_FLG_ADDR,0x55AA);//д��Ӧ�ó����Ѵ��ڱ�־
			FLASH_Lock();//��������
			TIM_Cmd(TIM3, ENABLE); 
			FLASH_Program_finish_flg = 0;
			goto boolkka;
		}	           		
	}
}




