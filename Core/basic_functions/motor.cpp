#include "motor.hpp"
#include "tim.h"

Motors::Motors(){
	Vin=3.0;
}
void Motors::Init(){
	InitMotorL();
	InitMotorR();

	SetDutyPWMR(0);
	SetDutyPWML(0);

}
void Motors::InitMotorR(){
	//HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_3);

}
void Motors::InitMotorL(){
	//HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_1);

}


void Motors::SetDutyPWML(unsigned short duty){//max 1000
	htim->Instance->CCR2=duty;

//	TIM_OC_InitTypeDef sConfigOC;
//
//	sConfigOC.OCMode = TIM_OCMODE_PWM1;
//	sConfigOC.Pulse = duty;
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//
//	HAL_TIM_PWM_ConfigChannel(&htim1,&sConfigOC,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2);

}
void Motors::SetDutyPWMR(unsigned short duty){
	htim->Instance->CCR4=duty;
//	TIM_OC_InitTypeDef sConfigOC;
//
//	sConfigOC.OCMode = TIM_OCMODE_PWM1;
//	sConfigOC.Pulse = duty;
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//
//	HAL_TIM_PWM_ConfigChannel(&htim1,&sConfigOC,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_4);
}


void Motors::SetVoltageL(float v){
	if(v>0){
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		v=-v;
	}
	SetDutyPWML(v/Vin*1000);

}
void Motors::SetVoltageR(float v){
	if(v>0){
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
	}else{
		v=-v;
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	}
	SetDutyPWMR(v/Vin*1000);
}
