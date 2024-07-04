#ifndef MODE_MODESELECTMODE_HPP_
#define MODE_MODESELECTMODE_HPP_

#include "MachineMode.hpp"

class ModeSelect:public MachineMode{
public:

	void Loop();
	void Init();
	void Interrupt_1ms();
	ModeSelect(Mouse* _mouse);
	virtual ~ModeSelect(){};
private:
	int mode_val;
	int sw1,sw2,sw3,pre_sw1,pre_sw2,pre_sw3;
	float encL_deg;
	float encR_deg;


};



#endif /* MODE_MODESELECTMODE_HPP_ */
