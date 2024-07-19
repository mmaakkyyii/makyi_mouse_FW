#include "DebugMode.hpp"

Debug::Debug(Mouse* _mouse)
:MachineMode(_mouse),
velocity_l(0),
velocity_r(0),
target_velocity_l(0),
target_velocity_r(0),
V_r(0),
V_l(0),
target_x(0),
target_y(0),
target_theta(0),
current_x(0),
current_y(0),
current_theta(0),
target_vx(0),
target_vy(0),
target_omega(0),
current_vx(0),
current_vy(0),
current_omega(0),
gesture_flag(false),
no_hand_flag(false),
timer(0),
wait_ms(0),
idle(true),
t_ms(0)
{
};
void Debug::Loop(){
	printf("%d,%d,%d,%d,%d,%d\r\n",(int)target_velocity_r,(int)target_velocity_l,(int)velocity_r,(int)velocity_l,(int)(1000*V_r),(int)(1000*V_l));

}
void Debug::Init(){
	current_mode=debug_mode;
	next_mode=debug_mode;

	printf("Start debug mode!\n\r");

	idle=true;
	timer=0;
	gesture_flag=false;
	no_hand_flag=false;
	wait_ms=0;
	t_ms=0;

}

void Debug::Interrupt_1ms(){
	if(idle){
		static int gesture_sensor_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensor_th || mouse->wall_sensor->GetFrontL() > gesture_sensor_th )){
			gesture_flag=true;
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensor_th && mouse->wall_sensor->GetFrontL()< gesture_sensor_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(500,50);
		}
		bool cal=false;
		if(no_hand_flag)cal=mouse->imu->Calibration();
		if(cal){
			idle=false;
			float v_max_mm_s=200;
			float a_mm_ss= 2000;
			clothoid=clothoid_200mm_90deg;

			int stright_num=3;
			float turn_v_max=300;
			float turn_omega_max=2*200/50;
			float a_omega=80;
//			trajectory=std::unique_ptr<Rotate>(new Rotate(180,turn_omega_max,a_omega));

//			trajectory=std::unique_ptr<ConstantVoltage>(new ConstantVoltage(0.3, 0.3, 500));
			trajectory =std::unique_ptr<DoubleTrajectory>(new DoubleTrajectory(
					new Line(0.0, 1*SECTION_WIDTH, 0.0, 0, 300, 200,a_mm_ss, 0.0),
					new Line(0.0, 1*SECTION_WIDTH, 0.0, 200, 200, 0,a_mm_ss, 0.0)
				));


/*
			trajectory =std::unique_ptr<DoubleTrajectory>(new DoubleTrajectory(
					new MultTrajectory(
						new Line(0.0, 7*SECTION_WIDTH+SECTION_WIDTH/2+clothoid.in_mm, 0.0, 0, v_max_mm_s, clothoid.v, 2000, 0.0),
						new Clothoid(clothoid,-1),
						new Line(0.0, clothoid.out_mm +SECTION_WIDTH/2, 0.0, clothoid.v, clothoid.v, 0, 2000, 0.0)
					),
					new Stay(500)
				));
//*/

//			trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
//							new Line(0.0, 2*SECTION_WIDTH+SECTION_WIDTH/2+clothoid.in_mm, 0.0, 0, clothoid.v, clothoid.v, 2000, 0.0),
//							new Clothoid(clothoid,1),
//							new Line(0.0, clothoid.out_mm +SECTION_WIDTH/2, 0.0, clothoid.v, clothoid.v, 0, 2000, 0.0)
//					));

/* 45deg 45deg
			trajectory =std::unique_ptr<DoubleTrajectory>(new DoubleTrajectory(
					new MultTrajectory(
							new Line(0.0, SECTION_WIDTH/2+clothoid.in_mm, 0.0, 0, clothoid.v, clothoid.v, 2000, 0.0),
							new Clothoid(clothoid,1),
							new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0)
					),
					new MultTrajectory(
							new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0),
							new Clothoid(clothoid,-1),
							new Line(0.0, SECTION_WIDTH/2+clothoid.in_mm, 0.0, clothoid.v, clothoid.v, 0, 2000, 0.0)
					)
					));
//*/
		}
	}else{
		if(trajectory->Update()){
			next_mode=modeSelect_mode;
		}else{

			trajectory->GetTargetPosition(&target_x, &target_y, &target_theta);
			trajectory->GetTargetVelocity(&target_vx,&target_vy,&target_omega);

			static float pre_error_wall;
			float period_s=0.001;
			float wall_control=Kp_wall*mouse->wall_sensor->GetError() + Kd_wall *(mouse->wall_sensor->GetError()-pre_error_wall)/period_s;
			if(trajectory->GetTragType()==line)target_omega+=wall_control;

			pre_error_wall=mouse->wall_sensor->GetError();


			Jacobian(target_vy,target_omega,&target_velocity_r,&target_velocity_l);

			mouse->motorR_PID->SetTarget(target_velocity_r);
			mouse->motorL_PID->SetTarget(target_velocity_l);

			velocity_r=mouse->encorders->GetVelociryR_mm_s();
			velocity_l=mouse->encorders->GetVelociryL_mm_s();
			V_r=mouse->motorR_PID->Update(velocity_r);
			V_l=mouse->motorL_PID->Update(velocity_l);

			mouse->motors->SetVoltageR(V_r);
			mouse->motors->SetVoltageL(V_l);

			mouse->log_data[mouse->log_index][0]=(int)(target_velocity_r );
			mouse->log_data[mouse->log_index][1]=(int)(V_r*1000 );
			mouse->log_data[mouse->log_index][2]=(int)(velocity_r );
			mouse->log_data[mouse->log_index][3]=(int)(target_velocity_l );
			mouse->log_data[mouse->log_index][4]=(int)(V_l*1000 );
			mouse->log_data[mouse->log_index][5]=(int)(velocity_l );
			mouse->log_data[mouse->log_index][6]=mouse->wall_sensor->GetErrorR();
			mouse->log_data[mouse->log_index][7]=mouse->wall_sensor->GetErrorL();
			mouse->log_index++;

		}
	}

}
