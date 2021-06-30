#ifndef __LED_H__
#define __LED_H__

#include "Main.h"


#define LED2_PORT 					GPIOB
#define LED1_PORT           GPIOB



#define LED1_PIN            GPIO_Pin_1
#define LED2_PIN 						GPIO_Pin_0

#define  btdate  GPIO_Pin_11 
#define  LED_W1  GPIO_Pin_10 
#define  LED_W2  GPIO_Pin_2 
#define  LED_W3  GPIO_Pin_1
#define  LED_W4  GPIO_Pin_0
#define  GPIOx_W1_4   GPIOB
#define  LED_W5  GPIO_Pin_3
#define  LED_W6  GPIO_Pin_2
#define  GPIOx_W5_6   GPIOA
#define  LED_W7  GPIO_Pin_1
#define  GPIOx_W7   GPIOF


//#define LED1_H      GPIO_SetBits(LED1_PORT,LED1_PIN)
//#define LED2_H      GPIO_SetBits(LED2_PORT,LED2_PIN)


#define All_LED_L		 do{   \
								     GPIO_SetBits(GPIOx_W1_4,LED_W1|LED_W2|LED_W3|LED_W4);\
						         GPIO_SetBits(GPIOx_W5_6,LED_W5|LED_W6);\
	                   GPIO_SetBits(GPIOx_W7,LED_W7);\
											}while(0) 

#define All_LED_H		 do{   \
								     GPIO_ResetBits(GPIOx_W1_4,LED_W1|LED_W2|LED_W3|LED_W4);\
						         GPIO_ResetBits(GPIOx_W5_6,LED_W5|LED_W6);\
	                   GPIO_ResetBits(GPIOx_W7,LED_W7);\
											}while(0) 

extern void usrGPIO_Init(void);

extern volatile uint8_t LedDisNowNum;
extern volatile uint8_t LedDisVal;
extern volatile uint8_t bLedDisSign; 
extern volatile uint8_t  bLedDisSign_Cpl;
extern volatile uint16_t led_period_cnt ;
extern volatile uint16_t CAN_send_period_cnt ;

extern volatile uint8_t b10msOnSign ;
extern volatile uint8_t bSwitchSign;

extern volatile uint16_t DelayNum;

extern void Led_Cpl(void);
#endif
