//unsigned char
#include "Uart.h"
#include "Can.h"
#include <string.h>
#include "stm32f0xx_can.h"
#include "Flash.h"
// USART1
unsigned	char 	UART_frame_flg=0; 				//�յ�����֡�������־
unsigned 	char 	UART_head_flg=0;				//ʶ����Ч֡ͷ��־
unsigned 	char 	UART_index=0;					//������������ű�
unsigned 	char 	UART_Wate_byte=0x55;			//�ȴ����յ��ַ���ÿ֡�׸��ַ���0x55��

unsigned  	int 	flame_cmd;						//֡����
unsigned  	int 	UART_Data_addr=0;				//֡��Ч������ʼ��ַ
unsigned  	int 	Devide_addr=0;					//?֡��Ч������ʼ��ַ
unsigned  	char 	UART_Data_RW=0;					//֡��Ч������ʼ��ַ

//unsigned  char RE_from_seir[250];					//?????????????;

unsigned  char USART_SEND_data[224];				/// UART��Ч����֡����

unsigned  int UART_CRC_RE;							//�յ���У����
unsigned  int UART_CRC_CA;							//�����յ������ݼ��������У����
unsigned  char USART_RV_Data_long=0;				//֡��Ч���ݳ��ȣ�
volatile unsigned  char UART_sending_flg;
unsigned  char USART_SEND_LONG;
unsigned  char  USART_send_cnt;

static unsigned char ReadUARTData;					//��ȡ�������ݼĴ���


unsigned  char Usart_Frame_Buffer[224];				//UART��Ч����֡����
unsigned char  UsartReturn_Buffer[224];  			//����ת��CAN �����ݻ���

 //================================================================
extern unsigned char    UsartCAN_frame_flg; 		//�յ�����֡�������־ 
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
//CAN���淢�͵�����
//==============================================================
/*void CANToUsart_Process(void)
{
			if(UartSendWait==1)    
			{
				USART_Send_string(CanReturn_Buffer,CAN_Send_long);//���ݻ��棬���ͳ���
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
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);   //��ʼ��UART1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	USART_InitStructure.USART_BaudRate = BaudRate;   //���ڲ�����ʼ��
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_ClearFlag(USART1,0XFFFFFFFF);
	USART_Init(USART1, &USART_InitStructure); //���ô����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


	USART_OverrunDetectionConfig(USART1,USART_OVRDetection_Disable); //������ر�
	USART_ClearFlag(USART1,0XFFFFFFFF);

	USART_Cmd(USART1, ENABLE);   //��������ʹ��

	UART_index=0;//?????0
	UART_Wate_byte =0x55;//??????????		
	UART_frame_flg=0;
	UART_head_flg=0;
}
	
//========================================================================================
 //�Ѵ��ڽ��յ�������֡���ص�CAN�����м仺��	
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

	if(UART_frame_flg==1)  //���ڻ�����֡������ɱ�־��������
	{

		CCITT_CRC16Init( (uint8_t const *)Usart_Frame_Buffer, (4+USART_RV_Data_long) );								
		UART_CRC_CA= CCITT_CRC16;				
		//�����յ��������Լ�����У����UART_CRC_CA
		if(UART_CRC_RE==UART_CRC_CA)//У������ȷ����֡������ɱ�־��֪ͨ������һ֡����,���������ݺ����Ҫ��flame_flg=0;UART_Wate_byte=0x55;//head_flg=0//index = 0;//�ű��0
		{
			temp_cmd = Usart_Frame_Buffer[0]; 	 
			temp_addr = Usart_Frame_Buffer[2];
			temp_addr = (temp_addr<<8)+Usart_Frame_Buffer[1];
			temp_Data_long=Usart_Frame_Buffer[3];
			/*У����ȷ*/
			/*���ݲ�ͬ�������flash���в���*/
			/*USART_Send_status(USART_SUCCESS)OR USART_Send_status(USART_Operate_ERR);*/
			//���ز����ɹ�����ʧ��֡
			if(temp_addr==0x8000)//��λ���������bootloader
			{

				USART_Send_status(USART_SUCCESS);//enter bootloader success
			}
			else if(temp_addr==0x8001) // ��λ������ˢ��flash
			{ 

				TIM_Cmd(TIM3, DISABLE); 
				temp_app_addr=APP_ADDR;//������ڵ�ַ

				for(temp_i=0;temp_i<64;temp_i++)//ˢ��64K
				{
					FLASH_Unlock();//����
					flash_Operate_St=FLASH_ErasePage(temp_app_addr);
					FLASH_Lock();//��������
					if(flash_Operate_St==FLASH_COMPLETE)
					{
						flash_Operate_cnt++;
					}
					temp_app_addr=temp_app_addr+1024;
				}
				TIM_Cmd(TIM3, ENABLE); 
				if(flash_Operate_cnt==64)
				{

					USART_Send_status(USART_SUCCESS);//���ز����ɹ�
					FLASH_WR_ADDR = APP_ADDR;//flash�ȴ�д��ĵ�ַ
					FLASH_Erase_finish_flg = 1;
				}
				else
				{
					FLASH_WR_ADDR = APP_ADDR;//flash�ȴ�д��ĵ�ַ 
					USART_Send_status(USART_Operate_ERR);//���ز���ʧ��
				}

			}
			else if(temp_addr==0x8002) //У��FLASHָ��
			{

				 USART_Send_status(USART_SUCCESS);//���ز����ɹ�����ʱ��������У��
				 FLASH_Program_finish_flg = 1;
				 FLASH_Erase_finish_flg = 0;	
				 delay_ms(10);//��֤����״̬�ظ�����
			}
	//		else if((temp_addr>=0x9001)&&(0x9fff>=temp_addr)&&(FLASH_Erase_finish_flg == 1)) //�������֡���ݴ��CAN_Rev_Buffer[6]���ֽ���temp_Data_long
			else if((temp_addr>=0x9001)&&(0x9fff>=temp_addr))
			{ 
				TIM_Cmd(TIM3, DISABLE); 
				DDword_num = temp_Data_long/4;//����Ҫд�������
				flash_Operate_cnt =0;
				temp_app_addr = FLASH_WR_ADDR;//׼��Ҫд��ĵ�ַ
				for(temp_i=0;temp_i<DDword_num;temp_i++)
				{
					temp_data = Usart_Frame_Buffer[4*temp_i+7];
					temp_data =(temp_data<<8)+ Usart_Frame_Buffer[4*temp_i+6];
					temp_data =(temp_data<<8)+ Usart_Frame_Buffer[4*temp_i+5];
					temp_data =(temp_data<<8)+ Usart_Frame_Buffer[4*temp_i+4];
					FLASH_Unlock();//����
					flash_Operate_St= FLASH_ProgramWord(temp_app_addr,temp_data);
					FLASH_Lock();//��������
					temp_app_addr=temp_app_addr+4;
					flash_Operate_cnt = flash_Operate_cnt+4;
					if(flash_Operate_St !=FLASH_COMPLETE)
					{
						break;
					  
					}
				}
				if(flash_Operate_St !=FLASH_COMPLETE)
				{
					USART_Send_status(USART_Operate_ERR);//����Ĺ�����
				}
				else
				{
					USART_Send_status(USART_SUCCESS);//���ز����ɹ�����ʱ��������У��
					FLASH_WR_ADDR = FLASH_WR_ADDR + flash_Operate_cnt;
				}

				TIM_Cmd(TIM3, ENABLE); 
			}
			/*else if(temp_addr==0x9fff) //���һ֡���ݣ�ˢ���OK
			{;}*/
			else
			{
				USART_Send_status(USART_Operate_ERR);//����Ĺ�����
			}			

		}
		else /*У��ʧ��*/
		{			
			/*����У�����֡*/
			USART_Send_status(USART_CRC_ERR);
		}
		UART_index=0;//?????0
		UART_Wate_byte =0x55;//??????????		
		UART_frame_flg=0;
		UART_head_flg=0;
						
		/*CANSendBuffer_long = 4+USART_RV_Data_long+2;
		UsartReturn_Buffer[0]=Usart_Frame_Buffer[4+USART_RV_Data_long];	
		UsartReturn_Buffer[1]=Usart_Frame_Buffer[4+USART_RV_Data_long+1];	

		for(i=0;i<CANSendBuffer_long-2;i++)//�Ѵ��ڽ��յ������ݷ���CAN�����м仺��
		{	
			UsartReturn_Buffer[i+2]=Usart_Frame_Buffer[i];													

		}			
		UsartCAN_frame_flg =1; //֪ͨCAN���Է�������
		UART_index=0;//?????0
		UART_Wate_byte =0x55;//??????????		
		UART_head_flg=0;		
		UART_frame_flg = 0; // ���´򿪽�������
			  */
	}									

}
/*�����յ�������״̬�𸴺���*/
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
	UsartReturn_Buffer[temp_i++]=1;//���ݳ��ȹ̶�1
	UsartReturn_Buffer[temp_i++]=status;
	CCITT_CRC16Init(&UsartReturn_Buffer[6], 5); 
	UsartReturn_Buffer[temp_i++]=(char)CCITT_CRC16;
	UsartReturn_Buffer[temp_i++]=(char)(CCITT_CRC16>>8);
	USART_Send_string(UsartReturn_Buffer,13);
 
}

////////////////////////////////
/*���ڷ����ַ���*/
///////////////////////////////////////////////////////
void USART_Send_string(uint8_t *addr,uint8_t USART_RV_Data_long)
{
	uint8_t tem_i;
	while(UART_sending_flg);
	USART_SEND_LONG=USART_RV_Data_long;
	for(tem_i=0;tem_i<USART_SEND_LONG;tem_i++)//����Ҫ���͵�����װ�ص����ͻ���
	{
		USART_SEND_data[tem_i] = addr[tem_i];
	}
	USART_ClearFlag(USART1, USART_IT_TC);
	USART_SendData(USART1,addr[USART_send_cnt++]);//���͵�һ���ַ�
	UART_sending_flg =1;
	USART_ITConfig(USART1, USART_IT_TC, ENABLE);//ʹ�ܷ�������жϣ��������ֽڷ������ж������
}	

//=====================================================================================
void USART1_IRQHandler(void)//�����жϺ���
{
	if(UART_sending_flg)  //UART���ڷ���������
	{ 
		if(USART_GetITStatus(USART1,USART_IT_TC) != RESET)//��������ж�
		{
			USART_ClearFlag(USART1, USART_IT_TC);//���жϱ�־λ
			if(USART_send_cnt==USART_SEND_LONG)//���ݷ������
			{
				UART_sending_flg=0;
				USART_send_cnt=0;
				USART_ITConfig(USART1, USART_IT_TC, DISABLE);//�رմ��ڷ�������ж�
			}
			else//��������û����
			{
				USART_SendData(USART1,USART_SEND_data[USART_send_cnt++]);//����װ����һ����
			}
		}
	}
//===============��ȡ����================================================================================	
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)//�ַ���������ж�
	{
		ReadUARTData =USART_ReceiveData(USART1);	//������3�մ��ڽ��ջ����ڻ��ÿһ���ַ�

		if(UART_frame_flg==1)//�Ѿ��н�����ɵ�֡δ����
		{
			UART_Wate_byte =0x55;//??????????		
		}
		else
		{
			if(UART_index>224)  //  �������224���ֽڣ� ���ڽ������ݳ��ȹ����أ�������0
			{
				UART_index=0;//����ű��0
				UART_Wate_byte =0x55;//���µȴ�֡ͷ�׸��ַ�
				UART_frame_flg=0;
				UART_head_flg=0;
			} 
			if(UART_head_flg==0)//��δ��׽������֡ͷ
			{
				if(UART_Wate_byte !=ReadUARTData)//�ǵȴ���֡ͷ�ַ�
				{ 
					UART_index=0;//����ű��0
					UART_Wate_byte =0x55;//���µȴ�֡ͷ�׸��ַ�
					if(ReadUARTData==0x55)//�ǵȴ��ַ�������֡��ʼ�ַ�0x55
					{
						Usart_Frame_Buffer[UART_index++]=ReadUARTData;
						UART_Wate_byte = 0xAA;						
					}				 
				}
				else  //�ǵȴ���֡ͷ�ַ�
				{

					Usart_Frame_Buffer[UART_index++]=ReadUARTData;
					switch(UART_index)//���ݵ�ǰ�ű�λ��ȷ����һ���ȴ����յ��ַ���ʲô
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
					}//�յ�������֡ͷ��6���ַ�
				}
			}
			else  //�Ѿ��յ���������֡ͷ UART_head_flg==1
			{
				Usart_Frame_Buffer[UART_index++]=ReadUARTData;//ֱ�Ӱ����ݷ���֡����
				if(UART_index==4)// ?�յ��ֽ�������11����ʼ�����豸��ַ��������ʼ��ַ�����ݳ��ȣ�֡����
				{						 								 
					UART_Data_addr = Usart_Frame_Buffer[2U];         
					UART_Data_addr = (UART_Data_addr<<8)+Usart_Frame_Buffer[1U];
						 
					UART_Data_RW= Usart_Frame_Buffer[0U];          // igh byte
					if(UART_Data_RW==0x01)  //�������ݷ���0�������1����д�룻ת�Ӱ崫����λ��ʱ���ֽ�Ϊ1
					{ 
						USART_RV_Data_long = Usart_Frame_Buffer[3U];		
					}
					else
					{
						USART_RV_Data_long=0;
					}
				}
									
				if(UART_index==(4+USART_RV_Data_long+2))   //�����������ݺ�2���ֽڵ�CRCУ����
				{
					UART_CRC_RE = Usart_Frame_Buffer[USART_RV_Data_long+5U];         
					UART_CRC_RE= (UART_CRC_RE<<8)+Usart_Frame_Buffer[USART_RV_Data_long+4U];         
					UART_frame_flg=1;
								  
					//CCITT_CRC16Init( (uint8_t const *)Usart_Frame_Buffer, (4+USART_RV_Data_long) );								
					//UART_CRC_CA= CCITT_CRC16;				
					//								
													
					//�����յ��������Լ�����У����UART_CRC_CA
					////if(UART_CRC_RE==UART_CRC_CA)//У������ȷ����֡������ɱ�־��֪ͨ������һ֡����,���������ݺ����Ҫ��flame_flg=0;UART_Wate_byte=0x55;//head_flg=0//index = 0;//�ű��0
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
