/*
 * c_wrapper.cpp
 *
 *  Created on: Jun 23, 2024
 *      Author: kyoro
 */

#include "c_wrapper.h"

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


PID_Controler motor_r_pid(0.0005,0.00005,0,CONTROL_PERIOD_ms);
PID_Controler motor_l_pid(0.0005,0.00005,0,CONTROL_PERIOD_ms);
PID_Controler angle_pid(6,0.0052,0,CONTROL_PERIOD_ms);

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
	HAL_Delay(1000);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	buzzer.Init();
	imu.Init();
	motors.Init();
	encorders.Init();
	//wall_sensor.Init();
	printf("aiueo!!\r\n");
	  buzzer.SetFrequency(450);
	  buzzer.On();
	  HAL_Delay(100);

	  buzzer.Off();

	HAL_TIM_Base_Start_IT(&htim16);
}

float v=0;
int dir=1;
void Loop(){

}

float theta_deg=0;
bool control_flag=true;
void Interrupt1ms(){

	imu.Update();
	//wall_sensor.Update();
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	encorders.Update();
	float gyro[3];
	imu.GetGyro(gyro);
	float acc[3];
	imu.GetAcc(acc);
	theta_deg+=gyro[2]*(CONTROL_PERIOD_ms/1000.0);
	angle_pid.SetTarget(0);
	float target_vel=angle_pid.Update(theta_deg);

	  //target_vel=30;

	  if(!ui.GetSW1()){
		  v=0.3;
		  control_flag=false;
	  }else{
		  v=0;
	  }
	  if(!control_flag){
		  target_vel=0;
	  }

	  motor_l_pid.SetTarget(-target_vel);
	  motor_r_pid.SetTarget(target_vel);
	  float vel_l=encorders.GetVelociryL_mm_s();
	  float v_l=motor_l_pid.Update(vel_l);
	  float vel_r=encorders.GetVelociryR_mm_s();
	  float v_r=motor_r_pid.Update(vel_r);
//	  if(control_flag){
//		motors.SetVoltageL(0);
//		motors.SetVoltageR(0);
//	  }else{
//		motors.SetVoltageL(0);
//		motors.SetVoltageR(0);
//	  }
		motors.SetVoltageL(v);
		motors.SetVoltageR(v);


	printf("%5d,%5d,%5d,\r\n"
			,(int)theta_deg
			,(int)vel_l
			,(int)vel_r

	);
}
