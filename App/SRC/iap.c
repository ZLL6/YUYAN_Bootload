#include "../INC/iap.h"

#define FLASH_APP1_ADDR		0x08002000  	//第一个应用程序起始地址(存放在FLASH)  8k
#define VECTOR_SIZE 192
											//保留的空间为IAP使用

u16 iapbuf[68] = {0}; //用于缓存数据的数组
u16 receiveDataCur = 0;	//当前iapbuffer中已经填充的数据长度,一次填充满了之后写入flash并清零
u32 addrCur = FLASH_APP1_ADDR;			//当前系统写入地址,每次写入之后地址增加2048
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
     CRC8校验查表
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

//开始下载
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
					//数据八位融合为16位
					temp = (((uint16_t)Uart1Buffer.Uart1Buffer[i+1])<<8) + ((uint16_t)Uart1Buffer.Uart1Buffer[i]);
					iapbuf[receiveDataCur] = temp;
					receiveDataCur++;
					
				}
			
				if((FrameIndex != iapbuf[0])||(FrameIndex == 0))   //是否为重新发的数据,但是第一帧会一直写Flash
				{
					FLASH_Unlock();
					STMFLASH_Write_NoCheck(addrCur,&iapbuf[1],64);
					FLASH_Lock();
					addrCur += 128;//地址+128
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

//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(uint32_t addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
  // MOV PC r14
	
}

//跳转到应用程序段
//appxaddr:用户代码起始地址.
void iap_load_app(u32 appxaddr)
{
	 uint8_t i;
	if(((*(volatile uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法.0x20000000是sram的起始地址,也是程序的栈顶地址
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
		 USART_Cmd(USART1,DISABLE);  //开启串口使能
		 USART_Cmd(USART2,DISABLE);
		

	
		i = __get_PRIMASK();         //获取PRIMASK寄存器当前状态
        __set_PRIMASK(1);            //屏蔽所有中断
			/*向量表重定向*/
		 memcpy((void*)0x20000000, (void*)FLASH_APP1_ADDR, VECTOR_SIZE);//拷贝向量表到RAM中
		SYSCFG->CFGR1 |= 0x03;  //设置STM32F072的中断向量表位于RAM中
		__set_PRIMASK(i);       //恢复中断	
				
		MSR_MSP(*(volatile uint32_t*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
	   (*(void (*)())(*(volatile uint32_t*)(appxaddr+4)))();   //appxaddr+4用户代码区第二个字为程序开始地址(复位地址)//跳转到APP.	

	
	}
}
//跳转到app区域运行
void iap_jump_app_s(void)
{ 
	uint16_t flag = 0,i;
	uint16_t CRC16[3]={0};

    STMFLASH_Read(APP_CODE_CRC_ADDR,CRC16,2);//CRC16[0]为CRC，CRC16[1]为CODE长度，
    
	for(addrCur = FLASH_APP1_ADDR,i=0;i<CRC16[1];i++)  //校验CODE 的CRC16
	{
		     STMFLASH_Read(addrCur,&iapbuf[1],64);
		     addrCur +=128;
		     if(i == 0)
				 CRC16[2] = CRCCCITT((uint8_t*)&iapbuf[1],128,0xFFFF);  //这里强制转换后，会从低位内存开始指针寻址
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
	iap_load_app(FLASH_APP1_ADDR);//跳转到app的复位向量地址
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

	  FLASH_Unlock();						//解锁
	  for(;PageAddress < 0x08010000;PageAddress+=2048)
			{	FLASH_ErasePage(PageAddress);//擦除这个扇区  //2K擦除
				WWDG_SetCounter(WDG_ReloadValue);
	     	}
	  FLASH_Lock();	
	
		temp = (((uint16_t)Uart1Buffer.Uart1Buffer[5])<<8) + ((uint16_t)Uart1Buffer.Uart1Buffer[4]);  //CRC16
		iapbuf[0] = temp;
		temp = (((uint16_t)Uart1Buffer.Uart1Buffer[7])<<8) + ((uint16_t)Uart1Buffer.Uart1Buffer[6]);  //长度
		iapbuf[1] = temp;
		temp = (((uint16_t)Uart1Buffer.Uart1Buffer[9])<<8) + ((uint16_t)Uart1Buffer.Uart1Buffer[8]);  //版本号
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

		addrCur = FLASH_APP1_ADDR;   //烧录地址辅初值
		OperateStep = DownLoad;
}















