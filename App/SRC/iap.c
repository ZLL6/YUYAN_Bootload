#include "../INC/iap.h"

#define FLASH_APP1_ADDR		0x08002000  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)  8k
#define VECTOR_SIZE 192
											//�����Ŀռ�ΪIAPʹ��

u16 iapbuf[68] = {0}; //���ڻ������ݵ�����
u16 receiveDataCur = 0;	//��ǰiapbuffer���Ѿ��������ݳ���,һ���������֮��д��flash������
u32 addrCur = FLASH_APP1_ADDR;			//��ǰϵͳд���ַ,ÿ��д��֮���ַ����2048
//u16 FrameIndex=0;

OperationStep OperateStep;

void iap_down_s(void);
void iap_jump_app_s(void);
void StartUpdate_Func(void);
void ReadyToUpdate_Func(void);


void MainDelay(uint16_t x)    /*104us * x */
{
	uint16_t DelayNum;
	DelayNum = x;
	DelayNum = DelayNum<<8;
	for(;DelayNum >0;DelayNum--)
	    ;
}
/***************************************************** 
     CRC8У����
******************************************************/  
unsigned char FindCRC(uint8_t *PData,uint8_t Len)
{
        uint8_t Crc;
        uint8_t ch[8], ch1;
        uint8_t i, j, k;
 
        Crc = 0x00;
        for (i=0;i<Len;i++)
        {
        ch1 = PData[i];
                for (j=0;j<8;j++)
                {
                ch[j] = ch1 & 0x01;
                ch1 >>= 1;
                }
                        for (k=0;k<8;k++)
                        {
                ch[7-k] <<=7;
                if ((Crc ^ ch[7-k]) & 0x80)
                Crc = (Crc << 1)^0x07 ;
                else
                Crc <<= 1;
                        }
         }
          Crc &= 0xff;
        return  Crc;
}
/******************************************************************
       CRC16-CITT-FALSE 
       x16+12+x5+1
       InitVal = 0xFFFF;
			 XorOut :0x0000
****************************************************************/
uint16_t CRCCCITT(unsigned char* pDataIn, int iLenIn,uint16_t wCRC1)     
{     
    uint16_t wTemp = 0;      
    uint16_t wCRC;
    uint8_t i,j;	
    
	  wCRC = wCRC1;
    for(i = 0;i < iLenIn;i++)      
    {             
        for(j = 0; j < 8; j++)      
        {      
            wTemp = ((pDataIn[i] << j) & 0x80 ) ^ ((wCRC & 0x8000) >> 8);      
            wCRC <<= 1;      
            if(wTemp != 0)       
            {     
                wCRC ^= 0x1021;      
            }     
        }      
    }      
    
    return wCRC;     
}  

//��ʼ����
void iap_down_s(void)
{
	u16 i = 0;
	u16 temp = 0;
	u16 receiveCount;
	uint8_t CrcNum;

			receiveCount = (u8)(Uart1Buffer.Uart1Index&0x00ff);
			if(receiveCount == 135)// 0XAA 0X55 DLC Cmd 2BytesDatas  128bytes Crc = 135
			{
				CrcNum = FindCRC(Uart1Buffer.Uart1Buffer,134);
				if((CrcNum != Uart1Buffer.Uart1Buffer[134])||(OperateStep != DownLoad))
				{
					 Uart1Data[0]= 0xAA;
					 Uart1Data[1]= 0x55;
					 Uart1Data[2]= 0x03;
					 Uart1Data[3]= Uart1Buffer.Uart1Command;
					 Uart1Data[4]= 0x01;
					 Uart1Data[5]= FindCRC(Uart1Data, 5);
				     UART1_SendData(Uart1Data, 6);
		
				return;
				}
				for(receiveDataCur = 0,i = 4; i < 134; i+=2)
				{
					//���ݰ�λ�ں�Ϊ16λ
					temp = (((uint16_t)Uart1Buffer.Uart1Buffer[i+1])<<8) + ((uint16_t)Uart1Buffer.Uart1Buffer[i]);
					iapbuf[receiveDataCur] = temp;
					receiveDataCur++;
					
				}
			
				if((FrameIndex != iapbuf[0])||(FrameIndex == 0))   //�Ƿ�Ϊ���·�������,���ǵ�һ֡��һֱдFlash
				{
					FLASH_Unlock();
					STMFLASH_Write_NoCheck(addrCur,&iapbuf[1],64);
					FLASH_Lock();
					addrCur += 128;//��ַ+128
					FrameIndex = iapbuf[0];
				}
				
		
					 Uart1Data[0]= 0xAA;
					 Uart1Data[1]= 0x55;
					 Uart1Data[2]= 0x03;
					 Uart1Data[3]= Uart1Buffer.Uart1Command;
					 Uart1Data[4]= 0x00;
					 Uart1Data[5]= FindCRC(Uart1Data, 5);
				     UART1_SendData(Uart1Data, 6);
					
		     }
			
	
}

//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(uint32_t addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
  // MOV PC r14
	
}

//��ת��Ӧ�ó����
//appxaddr:�û�������ʼ��ַ.
void iap_load_app(u32 appxaddr)
{
	 uint8_t i;
	if(((*(volatile uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)	//���ջ����ַ�Ƿ�Ϸ�.0x20000000��sram����ʼ��ַ,Ҳ�ǳ����ջ����ַ
	{ 
		        Uart1Data[0]= 0xAA;
				Uart1Data[1]= 0x55;
				Uart1Data[2]= 0x03;
				Uart1Data[3]= Uart1Buffer.Uart1Command;
				Uart1Data[4]= 0x00;
				Uart1Data[5]= FindCRC(Uart1Data, 5);
				UART1_SendData(Uart1Data, 6);
				
	//	bLedDisSign =0;
	//	All_LED_L;
	   RCC_APB1PeriphResetCmd(RCC_APB1Periph_WWDG, DISABLE);
	    MainDelay(400);
		 
	     TIM_Cmd(TIM3,DISABLE);
		 USART_Cmd(USART1,DISABLE);  //��������ʹ��
		 USART_Cmd(USART2,DISABLE);
		

	
		i = __get_PRIMASK();         //��ȡPRIMASK�Ĵ�����ǰ״̬
        __set_PRIMASK(1);            //���������ж�
			/*�������ض���*/
		 memcpy((void*)0x20000000, (void*)FLASH_APP1_ADDR, VECTOR_SIZE);//����������RAM��
		SYSCFG->CFGR1 |= 0x03;  //����STM32F072���ж�������λ��RAM��
		__set_PRIMASK(i);       //�ָ��ж�	
				
		MSR_MSP(*(volatile uint32_t*)appxaddr);					//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
	   (*(void (*)())(*(volatile uint32_t*)(appxaddr+4)))();   //appxaddr+4�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)//��ת��APP.	

	
	}
}
//��ת��app��������
void iap_jump_app_s(void)
{ 
	uint16_t flag = 0,i;
	uint16_t CRC16[3]={0};

    STMFLASH_Read(APP_CODE_CRC_ADDR,CRC16,2);//CRC16[0]ΪCRC��CRC16[1]ΪCODE���ȣ�
    
	for(addrCur = FLASH_APP1_ADDR,i=0;i<CRC16[1];i++)  //У��CODE ��CRC16
	{
		     STMFLASH_Read(addrCur,&iapbuf[1],64);
		     addrCur +=128;
		     if(i == 0)
				 CRC16[2] = CRCCCITT((uint8_t*)&iapbuf[1],128,0xFFFF);  //����ǿ��ת���󣬻�ӵ�λ�ڴ濪ʼָ��Ѱַ
				 else
				 { 
                       //iapbuf[0]=CRC16[2];
					    CRC16[2] = CRCCCITT((uint8_t*)&iapbuf[1],128,CRC16[2]);
				 }	    
	}
	if(CRC16[2] != CRC16[0])
		flag = 1;
	if(flag == 1)
	{     

	      Uart1Data[0]= 0xAA;
		 Uart1Data[1]= 0x55;
		 Uart1Data[2]= 0x03;
		 Uart1Data[3]= Uart1Buffer.Uart1Command;
		  Uart1Data[4]= 0x01;
		 Uart1Data[5]= FindCRC(Uart1Data, 5);
	    UART1_SendData(Uart1Data, 6);
			   return;
	}
	OperateStep = UpdateOk;
	
	Test_Write(APP_CONFIG_ADDR,APP_CONFIG_SET_VALUE);
	iap_load_app(FLASH_APP1_ADDR);//��ת��app�ĸ�λ������ַ
}
void StartUpdate_Func(void)
{
    if((Uart1Buffer.Uart1Buffer[4]==0xc3)&&(Uart1Buffer.Uart1Buffer[5]==0x3c))
	 {
		 Uart1Data[4]= 0x00;
		OperateStep = ReadyToUpdate;
     }
	else
	{	
		   Uart1Data[4]= 0x01;
		   OperateStep = StartUpdate;
	}
		 Uart1Data[0]= 0xAA;
		 Uart1Data[1]= 0x55;
		 Uart1Data[2]= 0x03;
		 Uart1Data[3]= Uart1Buffer.Uart1Command;
		 Uart1Data[5]= FindCRC(Uart1Data, 5);
	UART1_SendData(Uart1Data, 6);

}

void ReadyToUpdate_Func(void)
{
	  uint32_t PageAddress;
	  uint16_t temp;
	  PageAddress = FLASH_APP1_ADDR;

	  FLASH_Unlock();						//����
	  for(;PageAddress < 0x08010000;PageAddress+=2048)
			{	FLASH_ErasePage(PageAddress);//�����������  //2K����
				WWDG_SetCounter(WDG_ReloadValue);
	     	}
	  FLASH_Lock();	
	
		temp = (((uint16_t)Uart1Buffer.Uart1Buffer[5])<<8) + ((uint16_t)Uart1Buffer.Uart1Buffer[4]);  //CRC16
		iapbuf[0] = temp;
		temp = (((uint16_t)Uart1Buffer.Uart1Buffer[7])<<8) + ((uint16_t)Uart1Buffer.Uart1Buffer[6]);  //����
		iapbuf[1] = temp;
		temp = (((uint16_t)Uart1Buffer.Uart1Buffer[9])<<8) + ((uint16_t)Uart1Buffer.Uart1Buffer[8]);  //�汾��
		iapbuf[2] = temp;
		FLASH_Unlock();
		STMFLASH_Write_NoCheck(APP_CODE_CRC_ADDR,&iapbuf[0],3);
		FLASH_Lock();
		
			 Uart1Data[0]= 0xAA;
			 Uart1Data[1]= 0x55;
			 Uart1Data[2]= 0x03;
			 Uart1Data[3]= Uart1Buffer.Uart1Command;
			 Uart1Data[4]= 0x00;
			 Uart1Data[5]= FindCRC(Uart1Data, 5);
		UART1_SendData(Uart1Data, 6);

		addrCur = FLASH_APP1_ADDR;   //��¼��ַ����ֵ
		OperateStep = DownLoad;
}















