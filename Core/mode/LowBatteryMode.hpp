#ifndef MODE_LOWBATTERYMODE_HPP_
#define MODE_LOWBATTERYMODE_HPP_
#include "MachineMode.hpp"

class LowBattery:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	LowBattery(Mouse* _mouse);
	virtual ~LowBattery(){};
};

#endif /* MODE_LOWBATTERYMODE_HPP_ */
