#ifndef MODE_FASTRUNMODE_HPP_
#define MODE_FASTRUNMODE_HPP_
#include "MachineMode.hpp"

class FastRun:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	FastRun(Mouse* _mouse);
	virtual ~FastRun(){};
private:
	std::unique_ptr<Trajectory> trajectory;
	clothoid_params clothoid;
	char setting_mode;
	char vel_mode;
	char sla_mode;
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

//	bool goal;

	bool end_run_flag;
	bool idle;
	int path_length;
	int path_index;
	bool crash_en;

	float v_max_mm_s;
	float turn_v_max_mm_s;
	float a_mm_ss;
	float a_omega_max_rad_ss;

	const float crash_acc=45.0*2;
};




#endif /* MODE_FASTRUNMODE_HPP_ */
