#ifndef _MACHINE_MODE_HPP_
#define _MACHINE_MODE_HPP_

#include "mouse.hpp"
#include <memory>
#include "flash_util.hpp"

typedef enum {
	none_mode=-1,
	idle_mode=0,
	lowBattery_mode=1,
	modeSelect_mode=2,
	serchRun_mode=3,
	fastRun_mode=4,
	parameterSetting_mode=5,
	sensorCheck_mode=6,
	debug_mode=7,
	doNotRotate_mode=8,
	logOutput_mode=9,
	reset_map=15
}ModeType;

Trajectory* trajectryUpdate(Mouse* mouse,clothoid_params clothoid);


class MachineMode{
public:
	MachineMode(Mouse* _mouse):mouse(_mouse),current_mode(modeSelect_mode),next_mode(modeSelect_mode),low_batt_count(0){
		goal_x_setting=FlashGetGoalX();
		goal_y_setting=FlashGetGoalY();
	}
	virtual ~MachineMode(){};
	Mouse* mouse;
	virtual void Loop(){};
	virtual void Init(){};
	virtual void Interrupt_1ms(){};
	int IsOtherMode(){return 0;};
	void CheckBattery(){
		float V=mouse->battery_check->GetBatteryVoltage_V();
		if(V < 0){
			low_batt_count++;
		}else{
			low_batt_count=0;
		}
		if(low_batt_count>2000){
			next_mode=lowBattery_mode;
		}
	}
	ModeType GetCurrentMode(){return current_mode;}
	ModeType GetNextMode(){return next_mode;}
	
protected:
	ModeType current_mode;
	ModeType next_mode;
	int low_batt_count;
	int goal_x_setting;
	int goal_y_setting;
};

class Idle:public MachineMode{
public:
	void Loop(){};
	void Init(){};
	void Interrupt_1ms(){};
	Idle(Mouse* _mouse):MachineMode(_mouse){
	current_mode=idle_mode;
	next_mode=idle_mode;
	};
	virtual ~Idle(){};
};
class LowBattery:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	LowBattery(Mouse* _mouse);
	virtual ~LowBattery(){};
};
class SerchRun:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	SerchRun(Mouse* _mouse);
	virtual ~SerchRun(){};
private:
	std::unique_ptr<Trajectory> trajectory;
	clothoid_params clothoid;

	float velocity_l,velocity_r;
	float target_velocity_l,target_velocity_r;

	float V_r;
	float V_l;
	
	float target_x,target_y,target_theta;
	float current_x,current_y,current_theta;
	float target_vx,target_vy,target_omega;
	float current_vx,current_vy,current_omega;

	char sla_mode;

	int gesture_flag;
	int no_hand_flag;
	int timer;

	bool idle;
	
	
	//WallMask wall_mask;
};
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
	
	bool goal;

	bool idle;
	int path_length;
	int path_index;
	bool crash_en;

};
class ParameterSetting:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	ParameterSetting(Mouse* _mouse);
	virtual ~ParameterSetting(){};
	int mode=0;
	int time_ms=0;
	int led=0;
	int x=0;
	int y=0;

};

class DoNotRotate:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	DoNotRotate(Mouse* _mouse);
	virtual ~DoNotRotate(){};
	
	float gyro_theta;
	float omega_z;

	bool idle;
	int gesture_flag;
	int no_hand_flag;


};
class LogOutput:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	LogOutput(Mouse* _mouse);
	virtual ~LogOutput(){};
	int index;
};
class ResetMap:public MachineMode{
public:
	void Loop();
	void Init();
	void Interrupt_1ms();
	ResetMap(Mouse* _mouse):MachineMode(_mouse){};
	virtual ~ResetMap(){};
private:
};
#endif //_MACHINE_MODE_HPP_
