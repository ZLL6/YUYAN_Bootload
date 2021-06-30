#ifndef __FLASH_H__
#define __FLASH_H__
#include "Main.h"
typedef unsigned int u32;
typedef unsigned short int u16;
typedef unsigned char u8;
//////////////////////////////////////////////////////////////////////////////////////////////////////
//用户根据自己的需要设置
#define STM32_FLASH_SIZE 128 	 		//所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 1                      //使能FLASH写入(0，不是能;1，使能)
//////////////////////////////////////////////////////////////////////////////////////////////////////

#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
//FLASH解锁键值
//#define RDP_Key                  ((uint16_t)0x00A5)
//#define FLASH_KEY1               ((uint32_t)0x45670123)
//#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

#define APP_ADDR     (uint32_t)0x08004000

 

u16  STMFLASH_ReadHalfWord(u32 faddr);		  //读出半字  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//指定地址开始写入指定长度的数据
u32  STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);				//指定地址开始读取指定长度数据
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//从指定地址开始读出指定长度的数据
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);
//测试写入
void Test_Write(u32 WriteAddr,u16 WriteData);								   
#endif

/*#define CRC_CCITT_INIT 0xFFFF
#define CRC_CCITT_POLY 0x1021U
uint16_t CCITT_CRC16;
void CCITT_CRCStep(uint8_t byte);

void CCITT_CRC_ARRAY(uint8_t const * bytes, uint16_t len);


void CCITT_CRC16Init(uint8_t const * bytes, uint16_t len) ;
*/


















