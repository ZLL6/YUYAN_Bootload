#include "Can.h"
#include "Uart.h"
#include <string.h>
#include "Flash.h"
//////////////////////////////////////
//UAVCAN需要定义的全局变量和标志位
///////////////////////////
#define CAN_ID0_INT   					((u32)0x0E416E4)
#define CAN_ID1_OUT   					((u32)0x016E416)
#define Start_of_transfer 0x80
#define End_of_transfer   0x40

//unsigned  char UartSendWait;
unsigned char  CAN_transfer_toggle;
unsigned char  CAN_transfer_start_flg;
unsigned char  CAN_transfer_end_flg;
unsigned char  can_send_cnt;//从缓存内取数据计数
unsigned char  CAN_transfer_id;
unsigned char  CAN_SEND_LONG;//需要发送的缓存长度，

volatile unsigned char  CAN_transfering_flg;//can正在发送标志位

//unsigned char UsartCAN_frame_flg; //收到完整帧待处理标志
unsigned char CAN_frame_flg=0; //收到完整帧待处理标志
unsigned char CAN_head_flg=0;//识别到有效帧头标志
//unsigned char CAN_number=0;//接收数据数组脚标
//unsigned char CAN_number1=0;//接收数据数组脚标
//unsigned char CAN_number_Dec=0;//接收数据数组脚标

unsigned  char CAN_Data_RW=0;//数据方向，读写；
uint16_t CAN_RE_DDR=0;//can接收到的操作地址

unsigned  int CAN_CRC_RE;//收到的校验码
unsigned  int CAN_CRC_CA;//根据收到的数据计算出来的校验码
unsigned  char CAN_RV_Data_long=0;//帧有效数据长度；

//unsigned  char CANSendBuffer_long=0;//帧有效数据长度

unsigned char CAN_Rev_Length = 0;//CAN接收数据长度



unsigned  char	CAN_Send_long ;
	//==============================================================
unsigned  char	CanReturn_Buffer[224];
unsigned char  can_send_data[224];//uavcan发送数据缓存，把要发送的数据按顺序填进来				
unsigned char  CAN_Rev_Buffer[224];
	//==============================================================
union
{
	unsigned  char tmpBytes_CAN[2U];
	unsigned   int tmpWord_CAN;
}un_ByteWord_CAN;

 //=========================================================
static enum
                     {
                      CAN_RT_IDLE        = 0x00U,
                      CAN_RT_TRANSMIT    = 0x02U,
                      CAN_RT_RECEIVE     = 0x04U,
                      CAN_RT_FAULT       = 0x08U
                     }CAN_RT_State;

//static unsigned char fg_CANBusying;
//static unsigned char		fg_CANBusyingTime=0;



 //=========================================================





										 
CanRxMsg RxMessage;
CanTxMsg TxMessage;

//=============================================================
void CAN_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个GPIO_InitTypeDef类型的结构体
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);	//PB端口时钟使能                        											 
	
	/* CAN1 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);//CAN端口时钟使能
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_4); //定义PB8复用功能
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_4); //定义PB9复用功能
	
	/* Configure CAN pin: RX */									               // PB8
	
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP;//GPIO_PuPd_UP = 0x01,
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure CAN pin: TX */									               // PB9

	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF; //复用模式
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL; //GPIO_PuPd_NOPULL = 0x00
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_OUT;//复用模式
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL; //GPIO_PuPd_NOPULL = 0x00
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);
		
}

//=======================================================================
//=======================================================================
void CAN_Configation(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	

	
	CAN_GPIO();
	
	CAN_DeInit(CAN);
	CAN_StructInit(&CAN_InitStructure);
	
	
	CAN_InitStructure.CAN_TTCM=DISABLE;         // 时间触发通信禁止
	CAN_InitStructure.CAN_ABOM=ENABLE;	        // 离线退出是在中断置位清0后退出
	CAN_InitStructure.CAN_AWUM=ENABLE;	        // 自动唤醒模式：清零sleep
	CAN_InitStructure.CAN_NART=ENABLE;	        // 自动重新传送报文，知道发送成功
	CAN_InitStructure.CAN_RFLM=DISABLE;	        // FIFO没有锁定，新报文覆盖旧报文
	CAN_InitStructure.CAN_TXFP=DISABLE;         // 发送报文优先级确定：标志符
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; // 正常模式

	//1M
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
	CAN_InitStructure.CAN_Prescaler = 3;

	CAN_Init(CAN, &CAN_InitStructure);	// 初始化CAN1
	
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//CAN_FilterMode_IdList;// 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	

	/* CAN FIFO0 message pending interrupt enable */ 
	CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE); //中断使能
}

//===============================================================
//=========================================================
void CAN_FRAME_PROCESS(void)
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
	if(CAN_frame_flg==1)//can收到完整数据帧
		{
                
			    /*for(temp_i=0;temp_i<(CAN_Rev_Length-2);temp_i++)
			      {
							 CAN_Rev_Buffer[temp_i]=CAN_Rev_Buffer[temp_i+2];
						}*/
			      //CAN_Rev_Length = CAN_Rev_Length-2;
				    CCITT_CRC16Init(&(CAN_Rev_Buffer[2]), CAN_Rev_Length-2);								
			      CAN_CRC_CA= CCITT_CRC16;
             if(CAN_CRC_RE==CAN_CRC_CA)//校验码正确
								{
									temp_cmd = CAN_Rev_Buffer[2]; //rend or write	 
									temp_addr = CAN_Rev_Buffer[4];//address h
									temp_addr = (temp_addr<<8)+CAN_Rev_Buffer[3];
									temp_Data_long=CAN_Rev_Buffer[5];	
                                        /*校验正确*/
										/*根据不同功能码对flash进行操作*/
										/*返回操作成功或者失败帧*/
									if(temp_addr==0x8000)//上位进请求进入bootloader
									{
									  CAN_Send_status(CAN_SUCCESS);//enter bootloader success
									}
                  else if(temp_addr==0x8001) // 上位机请求刷除flash
                          {
                   temp_app_addr=APP_ADDR;//程序入口地址
                   TIM_Cmd(TIM3, DISABLE);                   
									 FLASH_Unlock();//解锁 
									for(temp_i=0;temp_i<64;temp_i++)//刷除64K
									  {
                      
										  flash_Operate_St=FLASH_ErasePage(temp_app_addr);
                    
											
										  if(flash_Operate_St==FLASH_COMPLETE)
                                         {
                                          flash_Operate_cnt++;
                                         }
										 temp_app_addr=temp_app_addr+1024;
									  }
									 FLASH_Lock();//重新上锁
									 TIM_Cmd(TIM3, ENABLE);	
									  if(flash_Operate_cnt==64)
									  	{
									  	 CAN_Send_status(CAN_SUCCESS);//返回操作成功
									  	 FLASH_WR_ADDR = APP_ADDR;//flash等待写入的地址
											 FLASH_Erase_finish_flg = 1;
									  	}
									    else
										{
										 CAN_Send_status(CAN_Operate_ERR);//返回操作失败
										 FLASH_WR_ADDR = APP_ADDR;//flash等待写入的地址 
										}

									}
									else if(temp_addr==0x8002) //校验FLASH指令
									{
									 CAN_Send_status(CAN_SUCCESS);//返回操作成功，暂时不做总体校验
									 FLASH_Program_finish_flg = 1;
									 FLASH_Erase_finish_flg = 0;	
									 delay_ms(100);//保证最后的状态回复结束
									}
									else if((temp_addr>=0x9001)&&(0x9fff>=temp_addr)&&(FLASH_Erase_finish_flg==1)) //程序代码帧数据存放CAN_Rev_Buffer[6]，字节数temp_Data_long
									{ 
										TIM_Cmd(TIM3, DISABLE); 
									  DDword_num = temp_Data_long/4;//求需要写入的字数
										flash_Operate_cnt =0;
                    temp_app_addr = FLASH_WR_ADDR;//准备要写入的地址
									  for(temp_i=0;temp_i<DDword_num;temp_i++)
									  	{

                     temp_data = CAN_Rev_Buffer[4*temp_i+9];
										 temp_data =(temp_data<<8)+ CAN_Rev_Buffer[4*temp_i+8];
										 temp_data =(temp_data<<8)+ CAN_Rev_Buffer[4*temp_i+7];
										 temp_data =(temp_data<<8)+ CAN_Rev_Buffer[4*temp_i+6];
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
									  	  CAN_Send_status(CAN_Operate_ERR);//operate fail
												FLASH_WR_ADDR = APP_ADDR;//flash等待写入的地址
											  temp_app_addr = FLASH_WR_ADDR;//准备要写入的地址
									  	}
									    else
                                        {
                                          CAN_Send_status(CAN_SUCCESS);//返回操作成功，暂时不做总体校验
                                          FLASH_WR_ADDR = FLASH_WR_ADDR + flash_Operate_cnt;
                                        }
                      TIM_Cmd(TIM3, ENABLE); 
									  
									}
									/*else if(temp_addr==0x9fff) //最后一帧数据，刷完就OK
									{;}*/
									else
									{
									  CAN_Send_status(CAN_Operate_ERR);//错误的功能码
									}	

								}
								else /*校验失败*/
								{ 

                                  /*返回校验错误帧*/
								  CAN_Send_status(CAN_CRC_ERR);
								}
						CAN_frame_flg=0;
						CAN_head_flg=0;		
						CAN_Rev_Length=0;		
		/*
						if(UartSendWait==0)
			      {
						unsigned char i;
						CAN_Send_long =CAN_Rev_Length+6 ;
						CanReturn_Buffer[0]=0x55;   //加入串口协议贞头，6个
						CanReturn_Buffer[1]=0xAA;
						CanReturn_Buffer[2]=0x00;
						CanReturn_Buffer[3]=0x00;
						CanReturn_Buffer[4]=0xAA;			
						CanReturn_Buffer[5]=0x55;	
			
			  		for(i=2;i<CAN_Rev_Length;i++)
						{	
								CanReturn_Buffer[i+4]=CAN_Rev_Buffer[i];													
	
						}				
						CanReturn_Buffer[(i++)+4]=CAN_Rev_Buffer[0];
				        CanReturn_Buffer[(i++)+4]=CAN_Rev_Buffer[1];
						UartSendWait=1;
					  CAN_frame_flg=0;
						CAN_Rev_Length=0;
						
			}*/
			
		}	
}
/*UAVcan返回指令状态*/
void CAN_Send_status(unsigned char STATUS)
{
  char temp_i=0;
  CanReturn_Buffer[2+(temp_i++)]= CAN_Data_RW;//读写命令
  CanReturn_Buffer[2+(temp_i++)]= (char)CAN_RE_DDR;//读写地諰L
  CanReturn_Buffer[2+(temp_i++)]= (char)(CAN_RE_DDR>>8);//读写地址H
  CanReturn_Buffer[2+(temp_i++)]= 1;//一个字节有效数据
  CanReturn_Buffer[2+(temp_i++)]= STATUS;//返回的状态
  CCITT_CRC16Init(&(CanReturn_Buffer[2]),5);	
  CAN_CRC_CA = CCITT_CRC16;
  CanReturn_Buffer[0]=(char)CAN_CRC_CA;//crc L
  CanReturn_Buffer[1]=(char)(CAN_CRC_CA>>8);//crc h
  UAV_CAN_start(CanReturn_Buffer,7);
  UAV_CAN_send();   //定时发送CAN数据
  CAN_send_period_cnt=0;
  
}

///////////////////////////////////
//UACCAN发送启动函数
/////////////////////////////////
void UAV_CAN_start(unsigned char *addr,unsigned char datalong)
{
	unsigned char temp_i;
	if(datalong>224) //uac协议最大发送224字节
	{
		return;
	}
	while(CAN_transfering_flg);//有数据正在发送，等待上一帧数据发送完成
	CAN_SEND_LONG = datalong;//获得需要发送的数据长度
	for(temp_i=0;temp_i<CAN_SEND_LONG;temp_i++) ///把数据放入发送缓存
	{
		can_send_data[temp_i]= addr[temp_i];
	}
	CAN_transfer_id=0;
	CAN_transfer_toggle=0;
	CAN_transfer_start_flg=1;
	CAN_transfer_end_flg=0;
	can_send_cnt = 0;
	CAN_transfering_flg=1;//启动CAN发送，该位置1后进入,便会自动打包数据发送
}

//////////////////////////////////
//根据需要的CAN发送数据包周期调用以下函数即可
//如果使用定时器，则调用蚒AV_CAN_start函数后立马调用该函数，然后启动定时器
/////////////////////////////////////
void UAV_CAN_send(void)
{
	unsigned char temp_i;
	unsigned char CAN_transfe_tailByte;
	if(CAN_transfering_flg==1)//把数据装载到发送缓存后把该标志置1，再跟据需求的CAN发送周期调用
	{
		if(CAN_transfer_id==0)//第一个数据包
		{
			CAN_transfer_start_flg=1;//传输开始标志位置1
		}else
		{
			CAN_transfer_start_flg=0;
		}
		for(temp_i=0;temp_i<7;)
		{
			TxMessage.Data[temp_i]=can_send_data[can_send_cnt++];
			temp_i++;
			if(can_send_cnt>=CAN_SEND_LONG)//发送到了最后一个 字节
			{
				CAN_transfer_end_flg =1;
				CAN_transfering_flg=0;			
				break;
			}			
		}
		CAN_transfe_tailByte=((((CAN_transfer_start_flg<<1)+CAN_transfer_end_flg)<<1)+CAN_transfer_toggle);//合成尾字节
		CAN_transfe_tailByte=(CAN_transfe_tailByte<<5)+CAN_transfer_id;
		TxMessage.Data[temp_i]=CAN_transfe_tailByte;//装载尾字节
		TxMessage.DLC=temp_i+1;
		TxMessage.IDE=CAN_ID_EXT;//RxMessage.IDE;
		TxMessage.ExtId=0x16E416;//装入数据包29位ID
		CAN_Transmit(CAN,&TxMessage);//CAN 发送数据,掉用CAN发送函数进行发送一帧数据

		CAN_transfer_id++;//为下一个数据包做准备
		CAN_transfer_toggle=(~CAN_transfer_toggle)&(0x01);
	}
}
		//===============================================================
/*void UsartToCAN_Process(void)
{
		if(UsartCAN_frame_flg==1)
		{
				UsartCAN_frame_flg=0;
				UAV_CAN_start(UsartReturn_Buffer,CANSendBuffer_long);//发送CAN数据缓存，数据长度
				UAV_CAN_send();   //定时发送CAN数据
   			       CAN_send_period_cnt=0;
		}
}
*/
	

//*********************************************
//===================================================================
//*********************************************/

void CAN_Recv_Control(void)  //CAN????recvoˉêy·μ???μ?μ?÷
{
	uint8_t  k; //
	uint8_t  i;
	CanRxMsg RxMessage;
	//CAN?óê?
	CAN_Receive(CAN,CAN_FIFO0,&RxMessage);
	if((RxMessage.ExtId==CAN_ID0_INT)&&(CAN_frame_flg==0))
	//	if((RxMessage.ExtId == CAN_ID1_OUT)&&(CAN_frame_flg == 0))
	{
		if(CAN_head_flg==0)		
		{
			if((RxMessage.Data[RxMessage.DLC-1]&Start_of_transfer)==0x80)//
			{
				CAN_head_flg=1;						//接收开始标志
				CAN_Rev_Length=0;
				for(i=0;i<RxMessage.DLC;i++)
				{CAN_Rev_Buffer[i]=RxMessage.Data[i];}
				//memcpy(&CAN_Rev_Buffer[0],&RxMessage.Data[0],RxMessage.DLC);
				CAN_CRC_RE=CAN_Rev_Buffer[1];
				CAN_CRC_RE=(CAN_CRC_RE<<8)+CAN_Rev_Buffer[0];
				CAN_Data_RW = CAN_Rev_Buffer[2];  
				CAN_RE_DDR = CAN_Rev_Buffer[4];
				CAN_RE_DDR=(CAN_RE_DDR<<8)+CAN_Rev_Buffer[3]; 
				CAN_RV_Data_long = CAN_Rev_Buffer[5];   //  数据个数
				CAN_Rev_Length  = RxMessage.DLC-1;	//接收总数		

			}													 				
		}
		else          //数据帧
		{					
			for(i=0;i<RxMessage.DLC;i++)
			{CAN_Rev_Buffer[CAN_Rev_Length+i]=RxMessage.Data[i];}

			// memcpy(&CAN_Rev_Buffer[CAN_Rev_Length&0XFF],&RxMessage.Data[0],RxMessage.DLC);
			CAN_Rev_Length = CAN_Rev_Length+RxMessage.DLC-1;		//接收总数	
		 
		}
		if(CAN_head_flg==1)		
		{				 
			 k = CAN_Rev_Buffer[CAN_Rev_Length]&End_of_transfer;
			 if(k==End_of_transfer)  //
			 {
						CAN_frame_flg = 1; 	
						///CAN_head_flg=0;						////结束帧					 
			 }
		}
	}			 		 
}

/*********************************************
        CAN 接收发送中断
***********************************************/

void CEC_CAN_IRQHandler(void)//CAN中断接收函数
{
	if(CAN_GetITStatus(CAN,CAN_IT_FMP0) != RESET)
  {		
			CAN_Recv_Control();  //CAN接收数据

	}
}
//===============================================================
