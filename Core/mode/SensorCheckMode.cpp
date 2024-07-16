#include "SensorCheckMode.hpp"

SensorCheck::SensorCheck(Mouse* _mouse):MachineMode(_mouse){
	current_mode=sensorCheck_mode;
	next_mode=sensorCheck_mode;
}

void SensorCheck::Loop(){
//	printf("%4d,%4d,%4d\r\n",mouse->imu->GetGzOffset(),(int)( gyro_raw[2]),(int)(1000*gyro[2]));
	/*
		printf("%4d,%4d,%4d,%4d\r\n",
				(int)(mouse->encorders->GetAngleL()),
				(int)(mouse->encorders->GetAngleR()),
				(int)(mouse->encorders->GetVelociryL_mm_s()),
				(int)(mouse->encorders->GetVelociryR_mm_s())
			);
	//*/
///*
			printf("%4d|%4d,%4d,%4d,%4d, %5d,%5d,%4d,%4d\r\n",
				(int)(1000*mouse->battery_check->GetBatteryVoltage_V()),
				mouse->wall_sensor->GetLeft(),
				mouse->wall_sensor->GetFrontL(),
				mouse->wall_sensor->GetFrontR(),
				mouse->wall_sensor->GetRight(),
				(int)(mouse->encorders->GetVelociryL_mm_s()),
				(int)(mouse->encorders->GetVelociryR_mm_s()),
				(int)(theta_gyro),
				(int)acc[2]


				);
		//*/
//	printf("%4d,%4d,%4d,%4d\r\n",
//			(int)(mouse->encorders->GetAngleL()),
//			(int)(mouse->encorders->GetAngleR()),
//			(int)(mouse->encorders->GetVelociryL_mm_s()),
//			(int)(mouse->encorders->GetVelociryR_mm_s())
//		);

}

void SensorCheck::Init(){
	mouse->motorR_PID->SetTarget(0);
	mouse->motorL_PID->SetTarget(0);
	printf("Start Sensor Check mode!\n\r");
}
void SensorCheck::Interrupt_1ms(){
	mouse->imu->GetGyro(gyro);
	theta_gyro+=(gyro[2]*0.001);
	mouse->imu->GetAcc(acc);
	float acc_th=1000;
	if(acc[0]*acc[0]+acc[1]*acc[1]>acc_th){
		mouse->buzzer->On_ms(3000,10);
		next_mode=modeSelect_mode;

	}

	if(mouse->wall_sensor->GetWallFR() || mouse->wall_sensor->GetWallFL()){
//			mouse->motorR_PID->SetTarget(0);
//			mouse->motorL_PID->SetTarget(0);
	}else if(mouse->wall_sensor->GetWallL()){
//		mouse->motorR_PID->SetTarget(500);
//		mouse->motorL_PID->SetTarget(500);
	}
	mouse->motorR_PID->SetTarget(500);
	mouse->motorL_PID->SetTarget(500);

	float velocity_r=mouse->encorders->GetVelociryR_mm_s();
	float velocity_l=mouse->encorders->GetVelociryL_mm_s();
	float V_r=mouse->motorR_PID->Update(velocity_r);
	float V_l=mouse->motorL_PID->Update(velocity_l);
	float v_max=1.5;
	if(V_r>v_max)V_r=v_max;
	if(V_r<-v_max)V_r=-v_max;
	if(V_l>v_max)V_l=v_max;
	if(V_l<-v_max)V_l=-v_max;
//	mouse->motors->SetVoltageR(V_r);
//	mouse->motors->SetVoltageL(V_l);
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);

	static int led=1;
	led=1-led;
	mouse->ui->SetLED( mouse->wall_sensor->GetWallR() <<0 |
			mouse->wall_sensor->GetWallFR()<<1 |
			mouse->wall_sensor->GetWallFL()<<2 |
			mouse->wall_sensor->GetWallL()<<3       );

	static int pre_sw1=mouse->ui->GetSW1();

	if(pre_sw1==1 && mouse->ui->GetSW1()==0){
		mouse->buzzer->On_ms(3000,10);
		next_mode=modeSelect_mode;
	}

};

