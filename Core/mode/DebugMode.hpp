#ifndef MODE_DEBUGMODE_HPP_
#define MODE_DEBUGMODE_HPP_

#include "MachineMode.hpp"

class Debug:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	Debug(Mouse* _mouse);
	virtual ~Debug(){};
private:
	std::unique_ptr<Trajectory> trajectory;

	float velocity_l,velocity_r;
	float target_velocity_l,target_velocity_r;

	float V_r;
	float V_l;

	float target_x,target_y,target_theta;
	float current_x,current_y,current_theta;
	float target_vx,target_vy,target_omega;
	float current_vx,current_vy,current_omega;

	int gesture_flag;
	int no_hand_flag;
	int timer;
	int wait_ms;
	bool idle;
	int t_ms;


};



#endif /* MODE_DEBUGMODE_HPP_ */
