#ifndef _BUZZER_H_
#define _BUZZER_H_

#include "stdint.h"
#include "tim.h"
class Buzzer{
public:
	Buzzer(int period_ms);
	void Init();
	int Update();
	void SetFrequency(int f);
	void On();
	void On_ms(int f,int _time_ms);
	void Off();
private:
	TIM_HandleTypeDef *htim = &htim1;
	uint32_t channel =TIM_CHANNEL_4;
	uint32_t period;
	int set_time_ms;
	int time_ms;
	int update_period_ms;
};


#endif //_BUZZER_H_


