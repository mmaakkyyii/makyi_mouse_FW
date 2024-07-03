#include "ui.hpp"
#include "main.h"
#include "gpio.h"

void UI::SetLED(int led_data)
{
	if((led_data>>0)&0b1) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	if((led_data>>1)&0b1)HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	if((led_data>>2)&0b1)HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

	if((led_data>>3)&0b1)HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

}

void UI::SetRBLED(int value){//0~100
	if(value<0)value=0;
	if(value>100)value=100;
	RBLED_value=value;
}

void UI::Init(){
	SetLED(0b1111);
}
void UI::Update(){
}
int UI::GetSW1(){
	return HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin);
}
int UI::GetSW2(){
	return 0;
	//return HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin);
}
int UI::GetSW3(){
	return 0;//HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin);
}
