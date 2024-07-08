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
//	HAL_ADC_Start_DMA(&hadc2,  (uint32_t *)adc_val, 1);
//    HAL_ADC_Start(&hadc2);
//
//    if( HAL_ADC_PollForConversion(&hadc2, 1000) == HAL_OK )
//    {
//	adc_val[0]=HAL_ADC_GetValue(&hadc2);
//    }
//    HAL_ADC_Stop(&hadc2);
}

float BatteryCheck::GetBatteryVoltage_V(){
	return (float)adc_val[0]/4096.0*3.3 *(R1+R2)/(R2);
}
