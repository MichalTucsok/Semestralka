/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 6.0.0   2016-11-22

The MIT License (MIT)
Copyright (c) 2009-2016 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32l1xx.h"
#include <stdio.h>
#include "konfiguracia.h"
#include <pwm.h>
/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

extern void (*TIM_Tvz_IRQ_Callback)(void);
extern Struct_Of_Values sensors;


volatile uint32_t msTicks;  //counts 1ms timeTicks
//void SysTick_Handler(void){
//	msTicks++;
//}

//Delays number of Systicks (happens every 1ms)
static void Delaypwm(__IO uint32_t dlyTicks){
	uint32_t curTicks = msTicks;
	while ((msTicks-curTicks)<dlyTicks);
}
void setSysTick(void){
	if(SysTick_Config(SystemCoreClock / 1000)){
		while(1){}
	}
}

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	TIM_Tvz_IRQ_Callback=Timer_Tvz_Callback;



sysClockSetup();
Init_GPIO();
Init_USART();
Init_ADC();
Init_Timer_Tvz();
	setSysTick();
	InitializeGPIO();
	InitializeTimer();
	InitializePWMChannel();
	InitializePWMChannel2();


char buffer[50];
long motor_pravy = 0;
long motor_lavy = 0;
long naj_lavy = 250;
long naj_pravy = 250;
long najv_lavy = 3500;
long najv_pravy = 3500;
int rozdiel = 0;
int sucet =0;
int r=0;
int rychlost1=1495;
int rychlost3=1490;


		while (1)
			{



			motor_pravy = ((sensors.Right_Sensor - naj_pravy)*(1000-300))/(najv_pravy-naj_pravy);
			motor_lavy = ((sensors.Left_Sensor - naj_lavy)*(1000-300))/(najv_lavy-naj_lavy);

			rozdiel = motor_pravy - motor_lavy;

			sucet = motor_pravy + motor_lavy;



			 if(sucet>300 && sucet<=600){
					 rychlost1=10+1495;
					 rychlost3=-10+1490;
			 }
			 else if (sucet>600){
				 rychlost1=20+1495;
				 rychlost3=-20+1490;
			 }
			 else if (sucet<=300){
				 rychlost1=1495;
				 rychlost3=1490;
			 }

			if( rozdiel <= 70 && rozdiel >= -70)
			{
				TIM4->CCR1 = rychlost1; //1495 stred
				TIM4->CCR3 = rychlost3; //1490 stred

			}

			if(rozdiel > 70 )
			{
				TIM4->CCR1 = rychlost1+(rozdiel/10);
				TIM4->CCR3 = rychlost3; //1200


			}

			if (rozdiel < -70)
			{
				TIM4->CCR3 = rychlost3+(rozdiel/10); //1800
				TIM4->CCR1 = rychlost1;


			}



			//sprintf(buffer,"Right->%d\n\rLeft->%d\n\r",sensors.Right_Sensor,sensors.Left_Sensor);
			//Send_Buffer(buffer);
			//Delay(800000);
			//sprintf(buffer,"pravy motor->%d\n\rlavy motor->%d\n\r",motor_pravy, motor_lavy);
			//Send_Buffer(buffer);
			//Delay(800000);
			sprintf(buffer,"sucet->%d\n\r",sucet);
			Send_Buffer(buffer);
						Delay(800000);



			}
			return 0;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}
