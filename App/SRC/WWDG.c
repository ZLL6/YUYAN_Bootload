#include "WWDG.h"
/********************************************************
			���ڿ��Ź�ԭ��
			WWDG_CR  ��WDG_ReloadValue �� 0x40 ��ȫ�ڣ�0x40 -0x3F  ���¸�λ
			WWDG_CFR ��0x40 �� WWDG_CFR���ڼ䣬��ȫι�����������¸�λ
			��������������
*******************************************************/
void WWDG_Config(void)
{
	 /* Enable WWDG clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

  /* WWDG clock counter = (PCLK1(10MHz)/4096)/8 = 305.18Hz (~3.28ms)  */
  WWDG_SetPrescaler(WWDG_Prescaler_8);

  /* Set Window value to 80; WWDG counter should be refreshed only when the counter
    is below 80 (and greater than 64) otherwise a reset will be generated */
  WWDG_SetWindowValue(WDG_ReloadValue);  //��������ֵ

  /* Enable WWDG and set counter value to 127, WWDG timeout = ~683 us * 64 = 43.7 ms 
     In this case the refresh window is: ~3.28 * (100-100)= 0ms < refresh window < ~3.28 * 36 = 118.08ms
     */
  WWDG_Enable(WDG_ReloadValue);   //���ڿ��Ź�����ֵ
}

