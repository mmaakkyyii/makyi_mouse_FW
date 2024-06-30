#include "battery_check.hpp"
#include "adc.h"

BatteryCheck::BatteryCheck(){
	adc_val[0]=0;
}
void BatteryCheck::Init(){
	//HAL_ADC_Init(&hadc2);
	HAL_ADC_Start_DMA(&hadc2,  (uint32_t *)adc_val, 1);

}

void BatteryCheck::Update(){
	HAL_ADC_Start_DMA(&hadc2,  (uint32_t *)adc_val, 1);
}
float BatteryCheck::GetBatteryVoltage_V(){
	return adc_val[0]/4096.0*3.3 *(7.8+15.0)/(15.0);
}
