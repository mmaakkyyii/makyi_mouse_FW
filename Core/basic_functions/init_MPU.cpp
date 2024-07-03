#include "init_mpu.hpp"
#include "debug.hpp"
#include "tim.h"
void init_sys(){
}

void init_delay_clk(){
}

void timer_start(){
	HAL_TIM_Base_Start_IT(&htim16);
	//HAL_TIM_Base_Start_IT(&htim17);
	//HAL_TIM_Base_Start_IT(&htim7);
}

void init_mpu(){
	init_sys();
	
}
