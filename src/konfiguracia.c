/*
 * konfiguracia.c
 *
 *  Created on: 20. 12. 2016
 *      Author: Michal
 */

#include <stddef.h>
#include "stm32l1xx.h"
#include "konfiguracia.h"

void (*TIM_Tvz_IRQ_Callback)(void);
Struct_Of_Values sensors;

void Init_GPIO()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	GPIO_InitTypeDef init_gpio;

	//USART2
	GPIO_StructInit(&init_gpio);
	init_gpio.GPIO_Mode=GPIO_Mode_AF;
	init_gpio.GPIO_Pin=GPIO_Pin_3 | GPIO_Pin_2;
	init_gpio.GPIO_Speed=GPIO_Speed_40MHz;
	GPIO_Init(GPIOA,&init_gpio);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

	//ADC1 1.sensor
	GPIO_StructInit(&init_gpio);
	init_gpio.GPIO_Mode=GPIO_Mode_AN;
	init_gpio.GPIO_Pin=GPIO_Pin_0;
	GPIO_Init(GPIOA,&init_gpio);

	//ADC1 2.sensor
	GPIO_StructInit(&init_gpio);
	init_gpio.GPIO_Mode=GPIO_Mode_AN;
	init_gpio.GPIO_Pin=GPIO_Pin_1;
	GPIO_Init(GPIOA,&init_gpio);
}

void Init_USART()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_InitTypeDef init_usart;
	NVIC_InitTypeDef init_nvic;

	USART_StructInit(&init_usart);
	init_usart.USART_BaudRate=9600;
	init_usart.USART_Mode=USART_Mode_Tx;
	USART_Init(USART2,&init_usart);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	init_nvic.NVIC_IRQChannel=USART2_IRQn;
	init_nvic.NVIC_IRQChannelCmd=ENABLE;
	init_nvic.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&init_nvic);

	USART_Cmd(USART2,ENABLE);
}

void Init_ADC()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC_InitTypeDef init_adc;

	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY)==RESET);

	ADC_StructInit(&init_adc);
	ADC_Init(ADC1,&init_adc);
	//ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_384Cycles);

	ADC_Cmd(ADC1,ENABLE);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADONS)==RESET);
}

void Init_Timer_Tvz()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseInitTypeDef init_tim;
	NVIC_InitTypeDef init_nvic;

	TIM_TimeBaseStructInit(&init_tim);
	init_tim.TIM_Period=1000-1;
	init_tim.TIM_Prescaler=2000-1;
	TIM_TimeBaseInit(TIM3,&init_tim);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

	init_nvic.NVIC_IRQChannel=TIM3_IRQn;
	init_nvic.NVIC_IRQChannelCmd=ENABLE;
	init_nvic.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&init_nvic);

	TIM_Cmd(TIM3,ENABLE);
}

void TIM3_IRQHandler()
{
	if(TIM_GetFlagStatus(TIM3,TIM_FLAG_Update))
	{
		TIM_Tvz_IRQ_Callback();
		TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	}
}



void USART2_IRQHandler()
{
	char character;
	if(USART_GetFlagStatus(USART2,USART_FLAG_TXE))
	{
		character=*sensors.TX_Buffer++;
		if(character) USART_SendData(USART2,character);
		else USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
	}
}

void Send_Buffer(char *Buffer)
{
	sensors.TX_Buffer=Buffer;
	USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
}


uint16_t ADC_Right_Sensor()
{
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_384Cycles);
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	return ADC_GetConversionValue(ADC1);
}


uint16_t ADC_Left_Sensor()
{
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_384Cycles);
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	return ADC_GetConversionValue(ADC1);
}

void Delay(uint32_t time)
{
	uint32_t i;
	for(i=0;i<time;i++)
	{
	}
}

void Timer_Tvz_Callback()
{
	uint16_t AD_value;
	uint16_t AD_value2;

	AD_value=ADC_Right_Sensor();
	sensors.Right_Sensor=AD_value;

	AD_value2=ADC_Left_Sensor();
    sensors.Left_Sensor=AD_value2;
}
