#include "stm32l1xx_tim.h"
#include "stm32l1xx.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;


uint16_t PrescalerValue = 0, timerPeriodValue;

static uint16_t gIC2Value = 0;
static uint16_t gDutyCycleIn = 0;
static uint16_t gDutyCycleOut = 0;
static uint16_t gFrequency = 0;

void pwm_initOutput()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	/* GPIOA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/*--------------------------------- GPIO Configuration -------------------------*/
	/* GPIOA Configuration: Pin 2 */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM9);

	/* -----------------------------------------------------------------------
	TIM9 Configuration: generate 4 PWM signals with 4 different duty cycles:
	The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
	clock at 8 MHz the Prescaler is computed as following:
	 - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	SystemCoreClock is set to 32 MHz for  Medium-density devices.

	The TIM3 is running at 12 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
												  = 8 MHz / 666 = 12 KHz
	TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
	TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
	TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
	TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
	----------------------------------------------------------------------- */
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1;
	timerPeriodValue = (1000000/50) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = timerPeriodValue;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;//1000 - 1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM9, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM9, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM9, ENABLE);
	//nastavenie defaultnej hodnoty
  TIM9->ARR = 1000;
}

void pwm_initInput()
{
	/* --------------------------- System Clocks Configuration ---------------------*/
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* GPIOA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/*--------------------------------- GPIO Configuration -------------------------*/
	/* GPIOA Configuration: Pin 7 */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

	PrescalerValue = (uint16_t) (SystemCoreClock / 1000000) - 1;
	timerPeriodValue = 65535;//(1000000/50) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = timerPeriodValue;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/*-------------------------------- NVIC Configuration --------------------------*/
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM3 configuration: PWM Input mode ------------------------
	The external signal is connected to TIM3 CH2 pin (PA.07),
	The Rising edge is used as active edge,
	The TIM3 CCR2 is used to compute the frequency value
	The TIM3 CCR1 is used to compute the duty cycle value
	------------------------------------------------------------ */

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

	/* Input Trigger selection */
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

	/* TIM3 enable counter */
	TIM_Cmd(TIM2, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	gDutyCycleIn = 0;
}

uint8_t pwm_setDutyCycle(uint16_t value)
{
	TIM9->CCR1 = gDutyCycleOut = value;
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET)
	{
		/* Clear TIM3 Capture compare interrupt pending bit */
		/* Get the Input Capture value */
		gIC2Value = TIM2->CCR2;

		/* Duty cycle computation */
		gDutyCycleIn = TIM2->CCR1;

		/* Frequency computation */
		gFrequency = 1000000 / gIC2Value;

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		gDutyCycleIn = gDutyCycleOut = 0;
		gFrequency = 0;

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

