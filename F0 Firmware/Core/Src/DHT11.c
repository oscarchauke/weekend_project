/*
 * DHT11.c
 *
 *  Created on: Jun 22, 2024
 *      Author: Oscar
 */
#include "DHT11.h"

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

TIM_HandleTypeDef *h_tim;

void DHT11_init(TIM_HandleTypeDef *htim){
	h_tim = htim;
}

void delay_us(uint16_t time){
	__HAL_TIM_SET_COUNTER(h_tim, 0);
	while(__HAL_TIM_GET_COUNTER(h_tim) < time);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void){
	Set_Pin_Output(DHT11_PORT, DHT11_PIN);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
	delay_us(18000);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
	delay_us(30);
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t DHT11_Check_Response(void){
	uint8_t response = 0;
	delay_us(40);
	if(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)){
		delay_us(80);
		if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)){
			response = 1;
		}else{
			response = 0;
		}
	}
	while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
	return response;
}

uint8_t DHT11_Read_Byte(){
	uint8_t i, j = 0;
	for(j=0; j<8; j++){
		while(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
		delay_us(40);
		if(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)){
			i&= ~(1<<(7-j));
		}else{
			i|= (1<<(7-j));
		}
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
	}
	return i;
}

void DHT11_Read(SensorDataPoint *temp, SensorDataPoint *real_hum){
	uint8_t presense = 0;
	uint8_t RH_Byte1, Temp_Byte1 = 0;

	DHT11_Start();
	presense = DHT11_Check_Response();

	if(presense == 1){
		RH_Byte1 = DHT11_Read_Byte();		// Read RHum byte 1
		DHT11_Read_Byte();					// Read RHum byte 2
		Temp_Byte1 = DHT11_Read_Byte(); 	// Read Temperature byte 1
		DHT11_Read_Byte();					// Read Temperature byte 2
		DHT11_Read_Byte();					// Read checksum

		temp->value = (float)Temp_Byte1;
		real_hum->value = (float)RH_Byte1;
	}else {
		console_log("DHT11 not connected\r\n");
	}
}
