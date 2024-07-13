#ifndef MODE_LOGOUTPUTMODE_HPP_
#define MODE_LOGOUTPUTMODE_HPP_

#include "MachineMode.hpp"

class LogOutput:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	LogOutput(Mouse* _mouse);
	virtual ~LogOutput(){};
	int index;
};

#endif /* MODE_LOGOUTPUTMODE_HPP_ */
