/*
 * DoNotRotateMode.hpp
 *
 *  Created on: Jul 7, 2024
 *      Author: kyoro
 */

#ifndef MODE_DONOTROTATEMODE_HPP_
#define MODE_DONOTROTATEMODE_HPP_
#include "MachineMode.hpp"

class DoNotRotate:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	DoNotRotate(Mouse* _mouse);
	virtual ~DoNotRotate(){};

	float gyro[3];

	float gyro_theta;
	float omega_z;

	bool idle;
	int gesture_flag;
	int no_hand_flag;


};

#endif /* MODE_DONOTROTATEMODE_HPP_ */
