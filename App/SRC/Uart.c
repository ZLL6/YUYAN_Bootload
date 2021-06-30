//unsigned char
#include "Uart.h"
#include "Can.h"
#include <string.h>
#include "stm32f0xx_can.h"
#include "Flash.h"
// USART1
unsigned	char 	UART_frame_flg=0; 				//收到完整帧待处理标志
unsigned 	char 	UART_head_flg=0;				//识别到有效帧头标志
unsigned 	char 	UART_index=0;					//接收数据数组脚标
unsigned 	char 	UART_Wate_byte=0x55;			//等待接收的字符，每帧首个字符是0x55；

unsigned  	int 	flame_cmd;						//帧命令
unsigned  	int 	UART_Data_addr=0;				//帧有效数据起始地址
unsigned  	int 	Devide_addr=0;					//?帧有效数据起始地址
unsigned  	char 	UART_Data_RW=0;					//帧有效数据起始地址

//unsigned  char RE_from_seir[250];					//?????????????;

unsigned  char USART_SEND_data[224];				/// UART有效数据帧缓存

unsigned  int UART_CRC_RE;							//收到的校验码
unsigned  int UART_CRC_CA;							//根据收到的数据计算出来的校验码
unsigned  char USART_RV_Data_long=0;				//帧有效数据长度；
volatile unsigned  char UART_sending_flg;
unsigned  char USART_SEND_LONG;
unsigned  char  USART_send_cnt;

static unsigned char ReadUARTData;					//读取串口数据寄存器


unsigned  char Usart_Frame_Buffer[224];				//UART有效数据帧缓存
unsigned char  UsartReturn_Buffer[224];  			//串口转接CAN 口数据缓存

 //================================================================
extern unsigned char    UsartCAN_frame_flg; 		//收到完整帧待处理标志 
extern unsigned  char   UartSendWait;	
extern unsigned  char	CanReturn_Buffer[224];
extern unsigned  char   CAN_Send_long;
//extern uint32_t FLASH_WR_ADDR;
unsigned int  FLASH_WR_ADDR;
unsigned char FLASH_Program_finish_flg=0;
unsigned char FLASH_Erase_finish_flg=0;

/////////////////???????????
//int i;
//int len= strlen(RE_from_seir);??????????

//======================================================

/*union
{
	unsigned  char tmpBytes[2U];
	unsigned   int tmpWord;
}un_ByteWord;
*/

//static unsigned char fg_Usart1Busying;
//static unsigned  char Usart1BusyTime;



//=CRC ??==================================================================
#define CRC_CCITT_INIT 0xFFFF
#define CRC_CCITT_POLY 0x1021U
uint16_t CCITT_CRC16;
void CCITT_CRCStep(uint8_t byte)
{
	uint32_t j;
	CCITT_CRC16 ^= ((uint16_t)byte << 8); 
	for ( j = 0; j < 8; j++ )
	{
		CCITT_CRC16 = (CCITT_CRC16 & 0x8000U)?((CCITT_CRC16 << 1) ^ CRC_CCITT_POLY):(CCITT_CRC16 << 1);
	}
}
void CCITT_CRC_ARRAY(uint8_t const * bytes, uint16_t len)
{
	while (len--) CCITT_CRCStep(*bytes++);
}

void CCITT_CRC16Init(uint8_t const * bytes, uint16_t len) 
{
	CCITT_CRC16 = CRC_CCITT_INIT; 
	CCITT_CRC_ARRAY(bytes,len);
}

//==============================================================
//CAN缓存发送到串口
//==============================================================
/*void CANToUsart_Process(void)
{
			if(UartSendWait==1)    
			{
				USART_Send_string(CanReturn_Buffer,CAN_Send_long);//数据缓存，发送长度
				UartSendWait=0;
			}

}
*/
//===================================================================
void Uart_Init(uint32_t BaudRate)  
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure; 
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);   //初始化UART1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	USART_InitStructure.USART_BaudRate = BaudRate;   //串口参数初始化
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_ClearFlag(USART1,0XFFFFFFFF);
	USART_Init(USART1, &USART_InitStructure); //配置串口中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


	USART_OverrunDetectionConfig(USART1,USART_OVRDetection_Disable); //溢出检测关闭
	USART_ClearFlag(USART1,0XFFFFFFFF);

	USART_Cmd(USART1, ENABLE);   //开启串口使能

	UART_index=0;//?????0
	UART_Wate_byte =0x55;//??????????		
	UART_frame_flg=0;
	UART_head_flg=0;
}
	
//========================================================================================
 //把串口接收到的完整帧加载到CAN发送中间缓存	
//========================================================================================	
void USART_FRAME_PROCESS(void)	
{
	unsigned char temp_cmd;
	unsigned short temp_addr;  
	unsigned char temp_Data_long;
	FLASH_Status flash_Operate_St;
	unsigned char flash_Operate_cnt=0;
	unsigned char temp_i;
	unsigned int temp_app_addr;
	unsigned int temp_data;
	unsigned char DDword_num;

	if(UART_frame_flg==1)  //串口缓存有帧接收完成标志，程序处理
	{

		CCITT_CRC16Init( (uint8_t const *)Usart_Frame_Buffer, (4+USART_RV_Data_long) );								
		UART_CRC_CA= CCITT_CRC16;				
		//根据收到的数据自己计算校验码UART_CRC_CA
		if(UART_CRC_RE==UART_CRC_CA)//校验码正确，置帧接收完成标志，通知程序处理一帧数据,处理完数据后程序要让flame_flg=0;UART_Wate_byte=0x55;//head_flg=0//index = 0;//脚标归0
		{
			temp_cmd = Usart_Frame_Buffer[0]; 	 
			temp_addr = Usart_Frame_Buffer[2];
			temp_addr = (temp_addr<<8)+Usart_Frame_Buffer[1];
			temp_Data_long=Usart_Frame_Buffer[3];
			/*校验正确*/
			/*根据不同功能码对flash进行操作*/
			/*USART_Send_status(USART_SUCCESS)OR USART_Send_status(USART_Operate_ERR);*/
			//返回操作成功或者失败帧
			if(temp_addr==0x8000)//上位进请求进入bootloader
			{

				USART_Send_status(USART_SUCCESS);//enter bootloader success
			}
			else if(temp_addr==0x8001) // 上位机请求刷除flash
			{ 

				TIM_Cmd(TIM3, DISABLE); 
				temp_app_addr=APP_ADDR;//程序入口地址

				for(temp_i=0;temp_i<64;temp_i++)//刷除64K
				{
					FLASH_Unlock();//解锁
					flash_Operate_St=FLASH_ErasePage(temp_app_addr);
					FLASH_Lock();//重新上锁
					if(flash_Operate_St==FLASH_COMPLETE)
					{
						flash_Operate_cnt++;
					}
					temp_app_addr=temp_app_addr+1024;
				}
				TIM_Cmd(TIM3, ENABLE); 
				if(flash_Operate_cnt==64)
				{

					USART_Send_status(USART_SUCCESS);//返回操作成功
					FLASH_WR_ADDR = APP_ADDR;//flash等待写入的地址
					FLASH_Erase_finish_flg = 1;
				}
				else
				{
					FLASH_WR_ADDR = APP_ADDR;//flash等待写入的地址 
					USART_Send_status(USART_Operate_ERR);//返回操作失败
				}

			}
			else if(temp_addr==0x8002) //校验FLASH指令
			{

				 USART_Send_status(USART_SUCCESS);//返回操作成功，暂时不做总体校验
				 FLASH_Program_finish_flg = 1;
				 FLASH_Erase_finish_flg = 0;	
				 delay_ms(10);//保证最后的状态回复结束
			}
	//		else if((temp_addr>=0x9001)&&(0x9fff>=temp_addr)&&(FLASH_Erase_finish_flg == 1)) //程序代码帧数据存放CAN_Rev_Buffer[6]，字节数temp_Data_long
			else if((temp_addr>=0x9001)&&(0x9fff>=temp_addr))
			{ 
				TIM_Cmd(TIM3, DISABLE); 
				DDword_num = temp_Data_long/4;//求需要写入的字数
				flash_Operate_cnt =0;
				temp_app_addr = FLASH_WR_ADDR;//准备要写入的地址
				for(temp_i=0;temp_i<DDword_num;temp_i++)
				{
					temp_data = Usart_Frame_Buffer[4*temp_i+7];
					temp_data =(temp_data<<8)+ Usart_Frame_Buffer[4*temp_i+6];
					temp_data =(temp_data<<8)+ Usart_Frame_Buffer[4*temp_i+5];
					temp_data =(temp_data<<8)+ Usart_Frame_Buffer[4*temp_i+4];
					FLASH_Unlock();//解锁
					flash_Operate_St= FLASH_ProgramWord(temp_app_addr,temp_data);
					FLASH_Lock();//重新上锁
					temp_app_addr=temp_app_addr+4;
					flash_Operate_cnt = flash_Operate_cnt+4;
					if(flash_Operate_St !=FLASH_COMPLETE)
					{
						break;
					  
					}
				}
				if(flash_Operate_St !=FLASH_COMPLETE)
				{
					USART_Send_status(USART_Operate_ERR);//错误的功能码
				}
				else
				{
					USART_Send_status(USART_SUCCESS);//返回操作成功，暂时不做总体校验
					FLASH_WR_ADDR = FLASH_WR_ADDR + flash_Operate_cnt;
				}

				TIM_Cmd(TIM3, ENABLE); 
			}
			/*else if(temp_addr==0x9fff) //最后一帧数据，刷完就OK
			{;}*/
			else
			{
				USART_Send_status(USART_Operate_ERR);//错误的功能码
			}			

		}
		else /*校验失败*/
		{			
			/*返回校验错误帧*/
			USART_Send_status(USART_CRC_ERR);
		}
		UART_index=0;//?????0
		UART_Wate_byte =0x55;//??????????		
		UART_frame_flg=0;
		UART_head_flg=0;
						
		/*CANSendBuffer_long = 4+USART_RV_Data_long+2;
		UsartReturn_Buffer[0]=Usart_Frame_Buffer[4+USART_RV_Data_long];	
		UsartReturn_Buffer[1]=Usart_Frame_Buffer[4+USART_RV_Data_long+1];	

		for(i=0;i<CANSendBuffer_long-2;i++)//把串口接收到的数据放入CAN发送中间缓存
		{	
			UsartReturn_Buffer[i+2]=Usart_Frame_Buffer[i];													

		}			
		UsartCAN_frame_flg =1; //通知CAN可以发送数据
		UART_index=0;//?????0
		UART_Wate_byte =0x55;//??????????		
		UART_head_flg=0;		
		UART_frame_flg = 0; // 重新打开接收数据
			  */
	}									

}
/*串口收到命令后的状态答复函数*/
void USART_Send_status(uint8_t status)
{
	char temp_i=0;
	UsartReturn_Buffer[temp_i++]=0x55;
	UsartReturn_Buffer[temp_i++]=0xAA;	
	UsartReturn_Buffer[temp_i++]=0X00;
	UsartReturn_Buffer[temp_i++]=0X00;
	UsartReturn_Buffer[temp_i++]=0XAA;
	UsartReturn_Buffer[temp_i++]=0X55;
	UsartReturn_Buffer[temp_i++]=UART_Data_RW;
	UsartReturn_Buffer[temp_i++]=(char)UART_Data_addr;
	UsartReturn_Buffer[temp_i++]=(char)(UART_Data_addr>>8);
	UsartReturn_Buffer[temp_i++]=1;//数据长度固定1
	UsartReturn_Buffer[temp_i++]=status;
	CCITT_CRC16Init(&UsartReturn_Buffer[6], 5); 
	UsartReturn_Buffer[temp_i++]=(char)CCITT_CRC16;
	UsartReturn_Buffer[temp_i++]=(char)(CCITT_CRC16>>8);
	USART_Send_string(UsartReturn_Buffer,13);
 
}

////////////////////////////////
/*串口发送字符串*/
///////////////////////////////////////////////////////
void USART_Send_string(uint8_t *addr,uint8_t USART_RV_Data_long)
{
	uint8_t tem_i;
	while(UART_sending_flg);
	USART_SEND_LONG=USART_RV_Data_long;
	for(tem_i=0;tem_i<USART_SEND_LONG;tem_i++)//把需要发送的数据装载到发送缓存
	{
		USART_SEND_data[tem_i] = addr[tem_i];
	}
	USART_ClearFlag(USART1, USART_IT_TC);
	USART_SendData(USART1,addr[USART_send_cnt++]);//发送第一个字符
	UART_sending_flg =1;
	USART_ITConfig(USART1, USART_IT_TC, ENABLE);//使能发送完成中断，后续的字节发送在中断中完成
}	

//=====================================================================================
void USART1_IRQHandler(void)//串口中断函数
{
	if(UART_sending_flg)  //UART正在发送数据中
	{ 
		if(USART_GetITStatus(USART1,USART_IT_TC) != RESET)//发送完成中断
		{
			USART_ClearFlag(USART1, USART_IT_TC);//清中断标志位
			if(USART_send_cnt==USART_SEND_LONG)//数据发送完成
			{
				UART_sending_flg=0;
				USART_send_cnt=0;
				USART_ITConfig(USART1, USART_IT_TC, DISABLE);//关闭串口发送完成中断
			}
			else//还有数据没发完
			{
				USART_SendData(USART1,USART_SEND_data[USART_send_cnt++]);//继续装入下一数据
			}
		}
	}
//===============读取数据================================================================================	
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)//字符接收完成中断
	{
		ReadUARTData =USART_ReceiveData(USART1);	//缓冲器3收串口接收缓存内获得每一个字符

		if(UART_frame_flg==1)//已经有接收完成的帧未处理
		{
			UART_Wate_byte =0x55;//??????????		
		}
		else
		{
			if(UART_index>224)  //  最大允许224个字节， 串口接收数据长度过负载，重新清0
			{
				UART_index=0;//数组脚标归0
				UART_Wate_byte =0x55;//重新等待帧头首个字符
				UART_frame_flg=0;
				UART_head_flg=0;
			} 
			if(UART_head_flg==0)//尚未捕捉到完整帧头
			{
				if(UART_Wate_byte !=ReadUARTData)//非等待的帧头字符
				{ 
					UART_index=0;//数组脚标归0
					UART_Wate_byte =0x55;//重新等待帧头首个字符
					if(ReadUARTData==0x55)//非等待字符，但是帧开始字符0x55
					{
						Usart_Frame_Buffer[UART_index++]=ReadUARTData;
						UART_Wate_byte = 0xAA;						
					}				 
				}
				else  //是等待的帧头字符
				{

					Usart_Frame_Buffer[UART_index++]=ReadUARTData;
					switch(UART_index)//根据当前脚标位置确定下一个等待接收的字符是什么
					{
						 case 1:UART_Wate_byte = 0xAA;break;
						 case 2:UART_Wate_byte = 0x00;break;
						 case 3:UART_Wate_byte = 0x00;break;
						 case 4:UART_Wate_byte = 0xAA;break;
						 case 5:UART_Wate_byte = 0x55;break;
						 case 6:UART_Wate_byte = 0x55;break;
						 default: UART_Wate_byte =0x55; UART_index=0;break;          
					}
					if(UART_index==6)
					{
						UART_head_flg=1;
						UART_index=0;
						//fg_Usart1Busying=1;			
						//Usart1BusyTime=0;											
					}//收到完整的帧头，6个字符
				}
			}
			else  //已经收到有完整的帧头 UART_head_flg==1
			{
				Usart_Frame_Buffer[UART_index++]=ReadUARTData;//直接把数据放入帧缓存
				if(UART_index==4)// ?收到字节数等于11，开始计算设备地址，数据起始地址，数据长度，帧命令
				{						 								 
					UART_Data_addr = Usart_Frame_Buffer[2U];         
					UART_Data_addr = (UART_Data_addr<<8)+Usart_Frame_Buffer[1U];
						 
					UART_Data_RW= Usart_Frame_Buffer[0U];          // igh byte
					if(UART_Data_RW==0x01)  //控制数据方向，0代表读，1代表写入；转接板传给上位机时该字节为1
					{ 
						USART_RV_Data_long = Usart_Frame_Buffer[3U];		
					}
					else
					{
						USART_RV_Data_long=0;
					}
				}
									
				if(UART_index==(4+USART_RV_Data_long+2))   //收完所有数据和2个字节的CRC校验码
				{
					UART_CRC_RE = Usart_Frame_Buffer[USART_RV_Data_long+5U];         
					UART_CRC_RE= (UART_CRC_RE<<8)+Usart_Frame_Buffer[USART_RV_Data_long+4U];         
					UART_frame_flg=1;
								  
					//CCITT_CRC16Init( (uint8_t const *)Usart_Frame_Buffer, (4+USART_RV_Data_long) );								
					//UART_CRC_CA= CCITT_CRC16;				
					//								
													
					//根据收到的数据自己计算校验码UART_CRC_CA
					////if(UART_CRC_RE==UART_CRC_CA)//校验码正确，置帧接收完成标志，通知程序处理一帧数据,处理完数据后程序要让flame_flg=0;UART_Wate_byte=0x55;//head_flg=0//index = 0;//脚标归0
					//{
					//	UART_frame_flg=1;

					//}
					//else
					//{ 
					//	UART_index=0;//?????0
					//	UART_Wate_byte =0x55;//??????????		
					//	UART_frame_flg=0;
					//	UART_head_flg=0;
					//}
				}
			}
		}
	}
}
