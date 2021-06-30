#include "Can.h"
#include "Uart.h"
#include <string.h>
#include "Flash.h"
//////////////////////////////////////
//UAVCAN��Ҫ�����ȫ�ֱ����ͱ�־λ
///////////////////////////
#define CAN_ID0_INT   					((u32)0x0E416E4)
#define CAN_ID1_OUT   					((u32)0x016E416)
#define Start_of_transfer 0x80
#define End_of_transfer   0x40

//unsigned  char UartSendWait;
unsigned char  CAN_transfer_toggle;
unsigned char  CAN_transfer_start_flg;
unsigned char  CAN_transfer_end_flg;
unsigned char  can_send_cnt;//�ӻ�����ȡ���ݼ���
unsigned char  CAN_transfer_id;
unsigned char  CAN_SEND_LONG;//��Ҫ���͵Ļ��泤�ȣ�

volatile unsigned char  CAN_transfering_flg;//can���ڷ��ͱ�־λ

//unsigned char UsartCAN_frame_flg; //�յ�����֡�������־
unsigned char CAN_frame_flg=0; //�յ�����֡�������־
unsigned char CAN_head_flg=0;//ʶ����Ч֡ͷ��־
//unsigned char CAN_number=0;//������������ű�
//unsigned char CAN_number1=0;//������������ű�
//unsigned char CAN_number_Dec=0;//������������ű�

unsigned  char CAN_Data_RW=0;//���ݷ��򣬶�д��
uint16_t CAN_RE_DDR=0;//can���յ��Ĳ�����ַ

unsigned  int CAN_CRC_RE;//�յ���У����
unsigned  int CAN_CRC_CA;//�����յ������ݼ��������У����
unsigned  char CAN_RV_Data_long=0;//֡��Ч���ݳ��ȣ�

//unsigned  char CANSendBuffer_long=0;//֡��Ч���ݳ���

unsigned char CAN_Rev_Length = 0;//CAN�������ݳ���



unsigned  char	CAN_Send_long ;
	//==============================================================
unsigned  char	CanReturn_Buffer[224];
unsigned char  can_send_data[224];//uavcan�������ݻ��棬��Ҫ���͵����ݰ�˳�������				
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
	GPIO_InitTypeDef GPIO_InitStructure; //����һ��GPIO_InitTypeDef���͵Ľṹ��
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);	//PB�˿�ʱ��ʹ��                        											 
	
	/* CAN1 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);//CAN�˿�ʱ��ʹ��
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_4); //����PB8���ù���
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_4); //����PB9���ù���
	
	/* Configure CAN pin: RX */									               // PB8
	
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;//����ģʽ
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP;//GPIO_PuPd_UP = 0x01,
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure CAN pin: TX */									               // PB9

	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF; //����ģʽ
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL; //GPIO_PuPd_NOPULL = 0x00
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_OUT;//����ģʽ
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;//�������
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
	
	
	CAN_InitStructure.CAN_TTCM=DISABLE;         // ʱ�䴥��ͨ�Ž�ֹ
	CAN_InitStructure.CAN_ABOM=ENABLE;	        // �����˳������ж���λ��0���˳�
	CAN_InitStructure.CAN_AWUM=ENABLE;	        // �Զ�����ģʽ������sleep
	CAN_InitStructure.CAN_NART=ENABLE;	        // �Զ����´��ͱ��ģ�֪�����ͳɹ�
	CAN_InitStructure.CAN_RFLM=DISABLE;	        // FIFOû���������±��ĸ��Ǿɱ���
	CAN_InitStructure.CAN_TXFP=DISABLE;         // ���ͱ������ȼ�ȷ������־��
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal; // ����ģʽ

	//1M
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
	CAN_InitStructure.CAN_Prescaler = 3;

	CAN_Init(CAN, &CAN_InitStructure);	// ��ʼ��CAN1
	
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
	CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE); //�ж�ʹ��
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
	if(CAN_frame_flg==1)//can�յ���������֡
		{
                
			    /*for(temp_i=0;temp_i<(CAN_Rev_Length-2);temp_i++)
			      {
							 CAN_Rev_Buffer[temp_i]=CAN_Rev_Buffer[temp_i+2];
						}*/
			      //CAN_Rev_Length = CAN_Rev_Length-2;
				    CCITT_CRC16Init(&(CAN_Rev_Buffer[2]), CAN_Rev_Length-2);								
			      CAN_CRC_CA= CCITT_CRC16;
             if(CAN_CRC_RE==CAN_CRC_CA)//У������ȷ
								{
									temp_cmd = CAN_Rev_Buffer[2]; //rend or write	 
									temp_addr = CAN_Rev_Buffer[4];//address h
									temp_addr = (temp_addr<<8)+CAN_Rev_Buffer[3];
									temp_Data_long=CAN_Rev_Buffer[5];	
                                        /*У����ȷ*/
										/*���ݲ�ͬ�������flash���в���*/
										/*���ز����ɹ�����ʧ��֡*/
									if(temp_addr==0x8000)//��λ���������bootloader
									{
									  CAN_Send_status(CAN_SUCCESS);//enter bootloader success
									}
                  else if(temp_addr==0x8001) // ��λ������ˢ��flash
                          {
                   temp_app_addr=APP_ADDR;//������ڵ�ַ
                   TIM_Cmd(TIM3, DISABLE);                   
									 FLASH_Unlock();//���� 
									for(temp_i=0;temp_i<64;temp_i++)//ˢ��64K
									  {
                      
										  flash_Operate_St=FLASH_ErasePage(temp_app_addr);
                    
											
										  if(flash_Operate_St==FLASH_COMPLETE)
                                         {
                                          flash_Operate_cnt++;
                                         }
										 temp_app_addr=temp_app_addr+1024;
									  }
									 FLASH_Lock();//��������
									 TIM_Cmd(TIM3, ENABLE);	
									  if(flash_Operate_cnt==64)
									  	{
									  	 CAN_Send_status(CAN_SUCCESS);//���ز����ɹ�
									  	 FLASH_WR_ADDR = APP_ADDR;//flash�ȴ�д��ĵ�ַ
											 FLASH_Erase_finish_flg = 1;
									  	}
									    else
										{
										 CAN_Send_status(CAN_Operate_ERR);//���ز���ʧ��
										 FLASH_WR_ADDR = APP_ADDR;//flash�ȴ�д��ĵ�ַ 
										}

									}
									else if(temp_addr==0x8002) //У��FLASHָ��
									{
									 CAN_Send_status(CAN_SUCCESS);//���ز����ɹ�����ʱ��������У��
									 FLASH_Program_finish_flg = 1;
									 FLASH_Erase_finish_flg = 0;	
									 delay_ms(100);//��֤����״̬�ظ�����
									}
									else if((temp_addr>=0x9001)&&(0x9fff>=temp_addr)&&(FLASH_Erase_finish_flg==1)) //�������֡���ݴ��CAN_Rev_Buffer[6]���ֽ���temp_Data_long
									{ 
										TIM_Cmd(TIM3, DISABLE); 
									  DDword_num = temp_Data_long/4;//����Ҫд�������
										flash_Operate_cnt =0;
                    temp_app_addr = FLASH_WR_ADDR;//׼��Ҫд��ĵ�ַ
									  for(temp_i=0;temp_i<DDword_num;temp_i++)
									  	{

                     temp_data = CAN_Rev_Buffer[4*temp_i+9];
										 temp_data =(temp_data<<8)+ CAN_Rev_Buffer[4*temp_i+8];
										 temp_data =(temp_data<<8)+ CAN_Rev_Buffer[4*temp_i+7];
										 temp_data =(temp_data<<8)+ CAN_Rev_Buffer[4*temp_i+6];
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
									  	  CAN_Send_status(CAN_Operate_ERR);//operate fail
												FLASH_WR_ADDR = APP_ADDR;//flash�ȴ�д��ĵ�ַ
											  temp_app_addr = FLASH_WR_ADDR;//׼��Ҫд��ĵ�ַ
									  	}
									    else
                                        {
                                          CAN_Send_status(CAN_SUCCESS);//���ز����ɹ�����ʱ��������У��
                                          FLASH_WR_ADDR = FLASH_WR_ADDR + flash_Operate_cnt;
                                        }
                      TIM_Cmd(TIM3, ENABLE); 
									  
									}
									/*else if(temp_addr==0x9fff) //���һ֡���ݣ�ˢ���OK
									{;}*/
									else
									{
									  CAN_Send_status(CAN_Operate_ERR);//����Ĺ�����
									}	

								}
								else /*У��ʧ��*/
								{ 

                                  /*����У�����֡*/
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
						CanReturn_Buffer[0]=0x55;   //���봮��Э����ͷ��6��
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
/*UAVcan����ָ��״̬*/
void CAN_Send_status(unsigned char STATUS)
{
  char temp_i=0;
  CanReturn_Buffer[2+(temp_i++)]= CAN_Data_RW;//��д����
  CanReturn_Buffer[2+(temp_i++)]= (char)CAN_RE_DDR;//��д���LL
  CanReturn_Buffer[2+(temp_i++)]= (char)(CAN_RE_DDR>>8);//��д��ַH
  CanReturn_Buffer[2+(temp_i++)]= 1;//һ���ֽ���Ч����
  CanReturn_Buffer[2+(temp_i++)]= STATUS;//���ص�״̬
  CCITT_CRC16Init(&(CanReturn_Buffer[2]),5);	
  CAN_CRC_CA = CCITT_CRC16;
  CanReturn_Buffer[0]=(char)CAN_CRC_CA;//crc L
  CanReturn_Buffer[1]=(char)(CAN_CRC_CA>>8);//crc h
  UAV_CAN_start(CanReturn_Buffer,7);
  UAV_CAN_send();   //��ʱ����CAN����
  CAN_send_period_cnt=0;
  
}

///////////////////////////////////
//UACCAN������������
/////////////////////////////////
void UAV_CAN_start(unsigned char *addr,unsigned char datalong)
{
	unsigned char temp_i;
	if(datalong>224) //uacЭ�������224�ֽ�
	{
		return;
	}
	while(CAN_transfering_flg);//���������ڷ��ͣ��ȴ���һ֡���ݷ������
	CAN_SEND_LONG = datalong;//�����Ҫ���͵����ݳ���
	for(temp_i=0;temp_i<CAN_SEND_LONG;temp_i++) ///�����ݷ��뷢�ͻ���
	{
		can_send_data[temp_i]= addr[temp_i];
	}
	CAN_transfer_id=0;
	CAN_transfer_toggle=0;
	CAN_transfer_start_flg=1;
	CAN_transfer_end_flg=0;
	can_send_cnt = 0;
	CAN_transfering_flg=1;//����CAN���ͣ���λ��1�����,����Զ�������ݷ���
}

//////////////////////////////////
//������Ҫ��CAN�������ݰ����ڵ������º�������
//���ʹ�ö�ʱ����������UAV_CAN_start������������øú�����Ȼ��������ʱ��
/////////////////////////////////////
void UAV_CAN_send(void)
{
	unsigned char temp_i;
	unsigned char CAN_transfe_tailByte;
	if(CAN_transfering_flg==1)//������װ�ص����ͻ����Ѹñ�־��1���ٸ��������CAN�������ڵ���
	{
		if(CAN_transfer_id==0)//��һ�����ݰ�
		{
			CAN_transfer_start_flg=1;//���俪ʼ��־λ��1
		}else
		{
			CAN_transfer_start_flg=0;
		}
		for(temp_i=0;temp_i<7;)
		{
			TxMessage.Data[temp_i]=can_send_data[can_send_cnt++];
			temp_i++;
			if(can_send_cnt>=CAN_SEND_LONG)//���͵������һ�� �ֽ�
			{
				CAN_transfer_end_flg =1;
				CAN_transfering_flg=0;			
				break;
			}			
		}
		CAN_transfe_tailByte=((((CAN_transfer_start_flg<<1)+CAN_transfer_end_flg)<<1)+CAN_transfer_toggle);//�ϳ�β�ֽ�
		CAN_transfe_tailByte=(CAN_transfe_tailByte<<5)+CAN_transfer_id;
		TxMessage.Data[temp_i]=CAN_transfe_tailByte;//װ��β�ֽ�
		TxMessage.DLC=temp_i+1;
		TxMessage.IDE=CAN_ID_EXT;//RxMessage.IDE;
		TxMessage.ExtId=0x16E416;//װ�����ݰ�29λID
		CAN_Transmit(CAN,&TxMessage);//CAN ��������,����CAN���ͺ������з���һ֡����

		CAN_transfer_id++;//Ϊ��һ�����ݰ���׼��
		CAN_transfer_toggle=(~CAN_transfer_toggle)&(0x01);
	}
}
		//===============================================================
/*void UsartToCAN_Process(void)
{
		if(UsartCAN_frame_flg==1)
		{
				UsartCAN_frame_flg=0;
				UAV_CAN_start(UsartReturn_Buffer,CANSendBuffer_long);//����CAN���ݻ��棬���ݳ���
				UAV_CAN_send();   //��ʱ����CAN����
   			       CAN_send_period_cnt=0;
		}
}
*/
	

//*********************************************
//===================================================================
//*********************************************/

void CAN_Recv_Control(void)  //CAN????recvo����y����???��?��?��
{
	uint8_t  k; //
	uint8_t  i;
	CanRxMsg RxMessage;
	//CAN?����?
	CAN_Receive(CAN,CAN_FIFO0,&RxMessage);
	if((RxMessage.ExtId==CAN_ID0_INT)&&(CAN_frame_flg==0))
	//	if((RxMessage.ExtId == CAN_ID1_OUT)&&(CAN_frame_flg == 0))
	{
		if(CAN_head_flg==0)		
		{
			if((RxMessage.Data[RxMessage.DLC-1]&Start_of_transfer)==0x80)//
			{
				CAN_head_flg=1;						//���տ�ʼ��־
				CAN_Rev_Length=0;
				for(i=0;i<RxMessage.DLC;i++)
				{CAN_Rev_Buffer[i]=RxMessage.Data[i];}
				//memcpy(&CAN_Rev_Buffer[0],&RxMessage.Data[0],RxMessage.DLC);
				CAN_CRC_RE=CAN_Rev_Buffer[1];
				CAN_CRC_RE=(CAN_CRC_RE<<8)+CAN_Rev_Buffer[0];
				CAN_Data_RW = CAN_Rev_Buffer[2];  
				CAN_RE_DDR = CAN_Rev_Buffer[4];
				CAN_RE_DDR=(CAN_RE_DDR<<8)+CAN_Rev_Buffer[3]; 
				CAN_RV_Data_long = CAN_Rev_Buffer[5];   //  ���ݸ���
				CAN_Rev_Length  = RxMessage.DLC-1;	//��������		

			}													 				
		}
		else          //����֡
		{					
			for(i=0;i<RxMessage.DLC;i++)
			{CAN_Rev_Buffer[CAN_Rev_Length+i]=RxMessage.Data[i];}

			// memcpy(&CAN_Rev_Buffer[CAN_Rev_Length&0XFF],&RxMessage.Data[0],RxMessage.DLC);
			CAN_Rev_Length = CAN_Rev_Length+RxMessage.DLC-1;		//��������	
		 
		}
		if(CAN_head_flg==1)		
		{				 
			 k = CAN_Rev_Buffer[CAN_Rev_Length]&End_of_transfer;
			 if(k==End_of_transfer)  //
			 {
						CAN_frame_flg = 1; 	
						///CAN_head_flg=0;						////����֡					 
			 }
		}
	}			 		 
}

/*********************************************
        CAN ���շ����ж�
***********************************************/

void CEC_CAN_IRQHandler(void)//CAN�жϽ��պ���
{
	if(CAN_GetITStatus(CAN,CAN_IT_FMP0) != RESET)
  {		
			CAN_Recv_Control();  //CAN��������

	}
}
//===============================================================
