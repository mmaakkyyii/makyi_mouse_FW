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
//#include "wall_sensor.hpp"
#include "gpio.h"

//WallSensor wall_sensor;
IMU imu;
Motors motors;
Encorders encorders(CONTROL_PERIOD_ms);
Buzzer buzzer(CONTROL_PERIOD_ms);

PID_Controler motor_r_pid(0.0005,0.00005,0,CONTROL_PERIOD_ms);
PID_Controler motor_l_pid(0.0005,0.00005,0,CONTROL_PERIOD_ms);
PID_Controler angle_pid(6,0.0052,0,CONTROL_PERIOD_ms);

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

	  if(!HAL_GPIO_ReadPin(USER_SW_GPIO_Port, USER_SW_Pin)){
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
