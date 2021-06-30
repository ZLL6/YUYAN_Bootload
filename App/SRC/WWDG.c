#include "WWDG.h"
/********************************************************
			窗口看门狗原理：
			WWDG_CR  从WDG_ReloadValue 到 0x40 安全期，0x40 -0x3F  导致复位
			WWDG_CFR 从0x40 到 WWDG_CFR的期间，安全喂狗器，否则导致复位
			上述条件与起来
*******************************************************/
void WWDG_Config(void)
{
	 /* Enable WWDG clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

  /* WWDG clock counter = (PCLK1(10MHz)/4096)/8 = 305.18Hz (~3.28ms)  */
  WWDG_SetPrescaler(WWDG_Prescaler_8);

  /* Set Window value to 80; WWDG counter should be refreshed only when the counter
    is below 80 (and greater than 64) otherwise a reset will be generated */
  WWDG_SetWindowValue(WDG_ReloadValue);  //窗口上限值

  /* Enable WWDG and set counter value to 127, WWDG timeout = ~683 us * 64 = 43.7 ms 
     In this case the refresh window is: ~3.28 * (100-100)= 0ms < refresh window < ~3.28 * 36 = 118.08ms
     */
  WWDG_Enable(WDG_ReloadValue);   //窗口看门狗重载值
}

