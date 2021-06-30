#ifndef __IAP_H_
#define __IAP_H_
#include "Main.h"
//0x08000000 flash起始地址
//0x2000 iap代码长度
//0x1ffc-0x1fff作为固化配置存在

#define APP_CONFIG_ADDR 	0X0800FFFC	//配置地址  最后2K用于存放数据
#define APP_CODE_CRC_ADDR   0X0800FFF6  //CRC16地址
#define APP_CODE_LEN_ADDR   0X0800FFF8  //APP代码长度
#define APP_Version_ADDR    0X0800FFFA  //APP版本号地址

#define APP_CONFIG_SET_VALUE	0xA5A5	//设置值
#define APP_CONFIG_CLEAR_VALUE	0XFFFF	//清零值

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


















