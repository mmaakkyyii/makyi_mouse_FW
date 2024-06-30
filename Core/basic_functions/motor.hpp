#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "tim.h"

class Motors{
private:
	float Vin;
	TIM_HandleTypeDef * htim=&htim3;

public:
	Motors();
	void Init();
	void InitMotorL();
	void InitMotorR();

	void SetSupplayVoltage(float v){Vin=v;}

	void SetDutyPWMR(unsigned short duty);
	void SetDutyPWML(unsigned short duty);
	void SetVoltageR(float v);
	void SetVoltageL(float v);

};

#endif //_MOTOT_H_
