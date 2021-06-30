#ifndef __CAN_H__
#define __CAN_H__
#include "stm32f0xx.h"
//#include "Flash.h"



extern uint16_t FrameIndex;
extern uint8_t DataFrameSign;
extern uint16_t CAN_SendID;		
extern unsigned char serial_Buffer[224];
extern unsigned short int serial_Buffer_Length;
extern  unsigned  char CanSendBuffer[224];///CAN Send ÓÐÐ§Êý¾ÝÖ¡»º´
extern unsigned  char CANSendBuffer_long;//Ö¡ÓÐÐ§Êý¾Ý³¤¶È

#define  CAN_SUCCESS 0x00
#define  CAN_CRC_ERR 0x5A
#define  CAN_Operate_ERR 0xFF


extern void usrCAN_Init(void);
extern CanTxMsg TxMessage;

extern void CAN_Configation(void);
extern void UAV_CAN_send(void);
//extern void   UsartToCAN_Process(void);
extern void   CAN_Configation(void);
extern void   CAN_FRAME_PROCESS(void);
void UAV_CAN_start(unsigned char *addr,unsigned char datalong);
void CAN_Send_status(unsigned char STATUS);


#endif


/************************ ÂÛÌ³µØÖ· www.zxkjmcu.com ******************************/
