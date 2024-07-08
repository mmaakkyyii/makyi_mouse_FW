#include "DoNotRotateMode.hpp"

DoNotRotate::DoNotRotate(Mouse* _mouse)
:MachineMode(_mouse),
gyro_theta(0),
omega_z(0),
idle(true),
gesture_flag(false),
no_hand_flag(false)
{
	current_mode=doNotRotate_mode;
	next_mode=doNotRotate_mode;
}
void DoNotRotate::Loop(){
	printf("%4d,%4d\r\n",(int)( omega_z*1000),(int)(gyro_theta));

}
void DoNotRotate::Init(){
	gyro_theta=0;
}
void DoNotRotate::Interrupt_1ms(){
	mouse->imu->GetGyro(gyro);
	//mouse->imu->GetGyroRaw(gyro_raw);

	if(idle){
		static int gesture_sensor_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensor_th || mouse->wall_sensor->GetFrontL() > gesture_sensor_th )){
			gesture_flag=true;
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensor_th && mouse->wall_sensor->GetFrontL()< gesture_sensor_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(400,40);
		}
		bool cal=false;
		if(no_hand_flag)cal=mouse->imu->Calibration();
		if(cal){
			idle=false;
		}
	}else{


		mouse->log_index=0;
		mouse->imu->GetGyro(gyro);

		omega_z=gyro[2];
		gyro_theta+=(omega_z)*0.001;
		static float e_sum=0;
		float Kp=0.15;//2;
		float Ki=0.02;//0.02;
		float e=(0-gyro_theta);
		e_sum+=e;
		float output_omega=Kp*e+Ki*0.001*e_sum;
		float target_velocity_l,target_velocity_r;
		Jacobian(0,output_omega,&target_velocity_r,&target_velocity_l);

		mouse->motorR_PID->SetTarget(target_velocity_r);
		mouse->motorL_PID->SetTarget(target_velocity_l);

		float velocity_r=mouse->encorders->GetVelociryR_mm_s();
		float velocity_l=mouse->encorders->GetVelociryL_mm_s();
		float V_r=mouse->motorR_PID->Update(velocity_r);
		float V_l=mouse->motorL_PID->Update(velocity_l);

		mouse->motors->SetVoltageR(V_r);
		mouse->motors->SetVoltageL(V_l);

		if(gyro_theta<-100 || gyro_theta>100 )
		{
			mouse->motors->SetVoltageR(0);
			mouse->motors->SetVoltageL(0);
			next_mode=modeSelect_mode;

		}
		if(mouse->wall_sensor->GetFrontR() > 300 && mouse->wall_sensor->GetFrontL() > 300 ){
			next_mode=modeSelect_mode;
			mouse->buzzer->On_ms(300,300);

		}

	}

}


