/*
 * konfiguracia.h
 *
 *  Created on: 20. 12. 2016
 *      Author: Michal
 */

#ifndef KONFIGURACIA_H_
#define KONFIGURACIA_H_


void Init_GPIO();
void Init_USART();
void Init_ADC();
void Init_Timer_Tvz();
void Delay(uint32_t time);
void Send_Buffer(char *Buffer);
uint16_t ADC_Right_Sensor(); //Hodnota praveho senzora
uint16_t ADC_Left_Sensor(); //Hodnota laveho senzora
void Timer_Tvz_Callback();

typedef struct
{
	uint16_t Right_Sensor;
	uint16_t Left_Sensor;
	char *TX_Buffer;
}Struct_Of_Values;

#endif /* KONFIGURACIA_H_ */
