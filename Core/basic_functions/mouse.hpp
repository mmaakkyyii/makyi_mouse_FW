#ifndef _MOUSE_HPP_
#define _MOUSE_HPP_

#include "buzzer.hpp"

#include "PID.hpp"
#include "motor.hpp"
#include "encorder.hpp"
#include "ui.hpp"
#include "IMU.hpp"
#include "wall_sensor.hpp"
#include "battery_check.hpp"
#include "localization.hpp"
#include "trajectory.hpp"
#include "MazeSolver.hpp"

class Mouse{
public:
	Mouse(PID_Controler* motorR, PID_Controler* motorL, Motors* _motors, Localization* _localization, Encorders* _encorders, IMU* _imu, WallSensor* _wall_sensor, BatteryCheck* _battery_check,Buzzer* _buzzer,UI* _ui, MazeSolver* _maze_solver);
	void Init();
	void Loop();
	void Interrupt_10ms();
	void Interrupt_1ms();
	void Interrupt_125us();
	
	void SetGoal(int x,int y, WallMask _mask);
	int GetWallInfo();

	PID_Controler* motorR_PID;
	PID_Controler* motorL_PID;
	Motors* motors;
	Localization* localization;
	Encorders* encorders;
	IMU* imu;
	WallSensor* wall_sensor;
	BatteryCheck* battery_check;
	Buzzer* buzzer;
	UI* ui;
	MazeSolver* maze_solver;

	int mouse_pos_x;
	int mouse_pos_y;
	Dirction mouse_dir;

	int goal_pos_x;
	int goal_pos_y;
	int goal_time;
	WallMask wall_mask;
	bool runing_buzzer;

	
private:	

	float period_ms;
	
	float v_max;
	float turn_v_max;
	

};

#endif //_MOUSE_HPP_
