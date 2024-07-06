#ifndef MODE_SENSORCHECKMODE_HPP_
#define MODE_SENSORCHECKMODE_HPP_
#include "MachineMode.hpp"

class SensorCheck:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	SensorCheck(Mouse* _mouse);
	virtual ~SensorCheck(){};
	float acc[3];
	float gyro[3];

	float theta_gyro=0;
};




#endif /* MODE_SENSORCHECKMODE_HPP_ */
