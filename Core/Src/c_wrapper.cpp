/*
 * c_wrapper.cpp
 *
 *  Created on: Jun 23, 2024
 *      Author: kyoro
 */

#include "c_wrapper.h"
#include "interrupt_func.h"

#include "stdio.h"
#include "IMU.hpp"
#include "motor.hpp"
#include "encorder.hpp"
#include "buzzer.hpp"
#include "PID.hpp"
#include "gpio.h"
#include "ui.hpp"
#include "localization.hpp"
#include "wall_sensor.hpp"
#include "battery_check.hpp"
#include "MazeSolver.hpp"
#include "mouse.hpp"
#include "MachineMode.hpp"
#include "ModeSelectMode.hpp"

//PID_Controler motor_r_pid(0.0005,0.0,0,CONTROL_PERIOD_ms);
//PID_Controler motor_l_pid(0.0005,0.0,0,CONTROL_PERIOD_ms);
//PID_Controler angle_pid(6,0.0,0,CONTROL_PERIOD_ms);

PID_Controler PID_motorL(Kp_motorL, Ki_motorL, Kd_motorL, CONTROL_PERIOD_ms);
PID_Controler PID_motorR(Kp_motorR, Ki_motorR, Kd_motorR, CONTROL_PERIOD_ms);
Motors motors;
Encorders encorders(CONTROL_PERIOD_ms);
IMU imu;
Localization localization(0,0,0,CONTROL_PERIOD_ms,&encorders);
WallSensor wall_sensor;
BatteryCheck battery_check;
Buzzer buzzer(CONTROL_PERIOD_ms);
UI ui;
MazeSolver maze_solver;

Mouse mouse(&PID_motorR,&PID_motorL, &motors,&localization,&encorders,&imu,&wall_sensor,&battery_check,&buzzer,&ui,&maze_solver);
MachineMode* mode;

void Init(){
	mode=new ModeSelect(&mouse);
	mouse.Init();
}

void Loop(){
	mouse.Loop();
	mode->Loop();
	HAL_Delay(10);

}

void Interrupt1ms(){
	int_1ms();
}
void Interrupt125us(){
	int_125ums();
}
