#ifndef _BATTERY_CHECK_HPP_
#define _BATTERY_CHECK_HPP_

#include "stdint.h"

class BatteryCheck{
public:
	BatteryCheck();
	void Init();
	void Update();
	float GetBatteryVoltage_V();
private:
	float R1=6.8;
	float R2=15;
	uint16_t adc_val[1];
};

#endif //_BATTERY_CHECK_HPP_
