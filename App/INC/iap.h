#ifndef __IAP_H_
#define __IAP_H_
#include "Main.h"
//0x08000000 flash��ʼ��ַ
//0x2000 iap���볤��
//0x1ffc-0x1fff��Ϊ�̻����ô���

#define APP_CONFIG_ADDR 	0X0800FFFC	//���õ�ַ  ���2K���ڴ������
#define APP_CODE_CRC_ADDR   0X0800FFF6  //CRC16��ַ
#define APP_CODE_LEN_ADDR   0X0800FFF8  //APP���볤��
#define APP_Version_ADDR    0X0800FFFA  //APP�汾�ŵ�ַ

#define APP_CONFIG_SET_VALUE	0xA5A5	//����ֵ
#define APP_CONFIG_CLEAR_VALUE	0XFFFF	//����ֵ

typedef enum{StartUpdate=0,ReadyToUpdate,DownLoad,UpdateOk}OperationStep;

extern OperationStep OperateStep;
extern void iap_down_s(void);
extern void iap_jump_app_s(void);
extern void StartUpdate_Func(void);
extern void ReadyToUpdate_Func(void);
extern unsigned char FindCRC(uint8_t *PData,uint8_t Len);
uint16_t CRCCCITT(unsigned char* pDataIn, int iLenIn,uint16_t wCRC1);
extern void MainDelay(uint16_t x);
#endif


















