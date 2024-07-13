#include "DigoRun.hpp"

DigoRun::DigoRun(Mouse* _mouse):
MachineMode(_mouse),
setting_mode(0),
vel_mode(0),
sla_mode(0),
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
gesture_flag(0),
no_hand_flag(0),
timer(0),
//goal(false),
end_run_flag(false),
idle(true),
path_length(0),
path_index(0),
crash_en(false)
{
	clothoid=clothoid_350mm_90deg_short;

};
void DigoRun::Loop(){
printf("%d\r\n",trajectory->GetTragType());
}
void DigoRun::Init(){
	mouse->ui->SetLED(0);
	//if(goal_flag==false && FlashGetGoalFlag()==0){
	if(FlashGetGoalFlag()==0){
		current_mode=digoRun_mode;
		next_mode=modeSelect_mode;
		mouse->buzzer->On_ms(250,400);
		return;
	}
	current_mode=digoRun_mode;
	next_mode=digoRun_mode;

	trajectory=std::unique_ptr<Stop>(new Stop());

	mouse->motorR_PID->Reset();
	mouse->motorL_PID->Reset();

	v_max_mm_s=600;
	turn_v_max_mm_s=250;
	//turn_omega_max=2*100/50;
	a_omega_max_rad_ss=80;
	a_mm_ss=3500;
	clothoid=clothoid_200mm_90deg_1;

	mouse->mouse_pos_x=0;
	mouse->mouse_pos_y=0;
	mouse->goal_pos_x=goal_x_setting;
	mouse->goal_pos_y=goal_y_setting;
	mouse->mouse_dir=North;
	mouse->goal_time=0;
	mouse->wall_mask=UNUSE_UNKOWN_WALL_MASK;

	idle=true;
//	goal=false;
	end_run_flag=false;
//	return_start_flag=false;
	printf("MakePathPlan\r\n");
	path_length = mouse->maze_solver->adachi.MakePathPlan(mouse->mouse_pos_x,mouse->mouse_pos_y,mouse->mouse_dir,mouse->goal_pos_x,mouse->goal_pos_y);
	printf("MakeRunPlan %d\r\n",path_length);
	mouse->maze_solver->adachi.MakeRunPlan(path_length,mouse->mouse_dir);
	printf("OK\r\n");
	path_index=0;
	vel_mode=0;
	sla_mode=0;
	setting_mode=0;

}
void DigoRun::Interrupt_1ms(){
	float acc_data[3];
	mouse->imu->GetAcc(acc_data);
	if(idle){
		static int sw1,sw2,pre_sw1,pre_sw2;

		pre_sw1=sw1;
		pre_sw2=sw2;
		sw1=mouse->ui->GetSW1();
		sw2=mouse->ui->GetSW2();

		if(acc_data[2]<-9.8*2){
			crash_en=true;
			mouse->buzzer->On_ms(500,100);

		}

		if(setting_mode!=1 && mouse->encorders->GetVelociryL_mm_s()>100){
			setting_mode=1;
			mouse->buzzer->On_ms(400,100);
		}
		if(setting_mode!=0 && mouse->encorders->GetVelociryR_mm_s()>100){
			setting_mode=0;
			mouse->buzzer->On_ms(400,100);
		}
		if(pre_sw1>sw1){
			if(setting_mode==0){
				vel_mode++;
				mouse->buzzer->On_ms(300,40);
				if(vel_mode>8){
					vel_mode=0;
					mouse->buzzer->On_ms(500,40);
				}
			}else if(setting_mode==1){
				sla_mode++;
				mouse->buzzer->On_ms(300,40);
				if(sla_mode>8){
					sla_mode=0;
					mouse->buzzer->On_ms(500,40);
				}

			}
		}
		static int led=0;
		static int led_timer=0;
		led_timer++;
		if(led_timer>50+50*setting_mode){
			led_timer=0;
			led=8-led;
		}

		if(setting_mode==0)mouse->ui->SetLED(vel_mode|led);
		if(setting_mode==1)mouse->ui->SetLED(sla_mode|led);

		switch(sla_mode){
		case 0:
			break;
		case 1:
			break;
		case 2:
			break;
		default:
			break;
		}

		switch(vel_mode){
		case 0:
			v_max_mm_s=400;
			a_mm_ss=4000;
			break;
		case 1:
			v_max_mm_s=600;
			a_mm_ss=4000;
			break;
		case 2:
			break;
		case 3:
			v_max_mm_s=700;
			a_mm_ss=4000;
			break;
		case 4:
			v_max_mm_s=800;
			a_mm_ss=4000;
			break;
		case 5:
			v_max_mm_s=900;
			a_mm_ss=4000;
			break;
		case 6:
			v_max_mm_s=1000;
			a_mm_ss=4000;
			break;
		case 7:
			v_max_mm_s=1500;
			a_mm_ss=4000;
			break;
		case 8:
			v_max_mm_s=2000;
			a_mm_ss=5000;
			break;
		default:
			v_max_mm_s=200;
			break;
		}

		static int gesture_sensorR_th=250;
		static int gesture_sensorL_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensorR_th || mouse->wall_sensor->GetFrontL() > gesture_sensorL_th )){
			gesture_flag=true;
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensorR_th && mouse->wall_sensor->GetFrontL()< gesture_sensorL_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(400,40);
		}
		bool cal=false;
		if(no_hand_flag)cal=mouse->imu->Calibration();
		if(cal){
			idle=false;

			int stright_num=0;
			while(mouse->maze_solver->adachi.run_plan[path_index] == Forward){
				path_index++;
				stright_num++;
			}


			trajectory=std::unique_ptr<Line>(new Line(0.0, SECTION_WIDTH/2.0+(stright_num-1)*SECTION_WIDTH, 0.0, 0.0, v_max_mm_s, clothoid.v, a_mm_ss, 0.0));
			//mouse->mouse_pos_y++;
		}
	}else{
		if(acc_data[0]*acc_data[0]+acc_data[1]*acc_data[1]>crash_acc*crash_acc){
			next_mode=modeSelect_mode;
		}

		if(trajectory->Update()){

			if(path_index>=path_length){
				mouse->buzzer->On_ms(400,40);
//					path_index++;
				end_run_flag=true;

			}else if(path_index>=path_length+1){
				end_run_flag=true;
			}else{
				int stright_num=0;
				if(mouse->maze_solver->adachi.run_plan[path_index] == Forward){
					while(mouse->maze_solver->adachi.run_plan[path_index] == Forward){
						if(path_index<path_length){
							path_index++;
							stright_num++;
						}else{
							break;
						}
					}
					if( mouse->maze_solver->adachi.run_plan[path_index    ] == TurnRight &&
							mouse->maze_solver->adachi.run_plan[path_index + 1] == TurnLeft &&
							mouse->maze_solver->adachi.run_plan[path_index + 2] == Forward
						){
						path_index+=3;
						clothoid=clothoid_200mm_45deg;

						if(path_index<path_length){
							trajectory =std::unique_ptr<DoubleTrajectory>(new DoubleTrajectory(
									new MultTrajectory(
											new Line(0.0, (stright_num-1)*SECTION_WIDTH+SECTION_WIDTH/2+clothoid.in_mm, 0.0, clothoid.v, v_max_mm_s, clothoid.v, 2000, 0.0),
											new Clothoid(clothoid,-1),
											new LineWoWall(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0)
									),
									new MultTrajectory(
											new LineWoWall(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0),
											new Clothoid(clothoid,1),
											new Line(0.0, SECTION_WIDTH/2+clothoid.in_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0)
									)
									));
						}else{
							trajectory =std::unique_ptr<DoubleTrajectory>(new DoubleTrajectory(
									new MultTrajectory(
											new Line(0.0, (stright_num-1)*SECTION_WIDTH+SECTION_WIDTH/2+clothoid.in_mm, 0.0, clothoid.v, v_max_mm_s, clothoid.v, 2000, 0.0),
											new Clothoid(clothoid,-1),
											new LineWoWall(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0)
									),
									new MultTrajectory(
											new LineWoWall(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0),
											new Clothoid(clothoid,1),
											new Line(0.0, SECTION_WIDTH/2+clothoid.in_mm +SECTION_WIDTH/2, 0.0, clothoid.v, clothoid.v, 0, 2000, 0.0)
									)
									));

						}

					}else if( mouse->maze_solver->adachi.run_plan[path_index    ] == TurnLeft &&
							mouse->maze_solver->adachi.run_plan[path_index + 1] == TurnRight &&
							mouse->maze_solver->adachi.run_plan[path_index + 2] == Forward
						){
						path_index+=3;
						clothoid=clothoid_200mm_45deg;

						if(path_index<path_length){
							trajectory =std::unique_ptr<DoubleTrajectory>(new DoubleTrajectory(
									new MultTrajectory(
											new Line(0.0, (stright_num-1)*SECTION_WIDTH+SECTION_WIDTH/2+clothoid.in_mm, 0.0, clothoid.v, v_max_mm_s, clothoid.v, 2000, 0.0),
											new Clothoid(clothoid,1),
											new LineWoWall(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0)
									),
									new MultTrajectory(
											new LineWoWall(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0),
											new Clothoid(clothoid,-1),
											new Line(0.0, SECTION_WIDTH/2+clothoid.in_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0)
									)
									));
						}else{
							trajectory =std::unique_ptr<DoubleTrajectory>(new DoubleTrajectory(
									new MultTrajectory(
											new Line(0.0, (stright_num-1)*SECTION_WIDTH+SECTION_WIDTH/2+clothoid.in_mm, 0.0, clothoid.v, v_max_mm_s, clothoid.v, 2000, 0.0),
											new Clothoid(clothoid,1),
											new LineWoWall(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0)
									),
									new MultTrajectory(
											new LineWoWall(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, 2000, 0.0),
											new Clothoid(clothoid,-1),
											new Line(0.0, SECTION_WIDTH/2+clothoid.in_mm +SECTION_WIDTH/2, 0.0, clothoid.v, clothoid.v, 0, 2000, 0.0)
									)
									));

						}

					}else if(path_index>=path_length){
						path_index++;
						trajectory=std::unique_ptr<Line>(new Line(0.0, (stright_num)*SECTION_WIDTH+SECTION_WIDTH/2, 0.0, clothoid.v, v_max_mm_s, 0,          a_mm_ss, 0.0));
					}else{
						trajectory=std::unique_ptr<Line>(new Line(0.0, (stright_num)*SECTION_WIDTH        , 0.0, clothoid.v, v_max_mm_s, clothoid.v, a_mm_ss, 0.0));
					}
				}else if(mouse->maze_solver->adachi.run_plan[path_index] == TurnRight){
					clothoid=clothoid_200mm_90deg_1;

					path_index++;
					if(path_index>=path_length){
						trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
							new Line(0.0, clothoid.in_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, a_mm_ss, 0.0),
							new Clothoid(clothoid,-1),
							new Line(0.0, clothoid.out_mm+SECTION_WIDTH/2, 0.0, clothoid.v, clothoid.v, 0, a_mm_ss, 0.0)
							));
					}else{
						trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
							new Line(0.0, clothoid.in_mm, 0.0, clothoid.v, clothoid.v+1, clothoid.v, a_mm_ss, 0.0),
							new Clothoid(clothoid,-1),
							new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v+1, clothoid.v, a_mm_ss, 0.0)
						));

					}

//					printf("R\r\n");
				}else if(mouse->maze_solver->adachi.run_plan[path_index] == TurnLeft){
					clothoid=clothoid_200mm_90deg_1;
					path_index++;
					if(path_index>=path_length){
						trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
							new Line(0.0, clothoid.in_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, a_mm_ss, 0.0),
							new Clothoid(clothoid,1),
							new Line(0.0, clothoid.out_mm+SECTION_WIDTH/2, 0.0, clothoid.v, clothoid.v+1, 0, a_mm_ss, 0.0)
							));
					}else{
						trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
							new Line(0.0, clothoid.in_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, a_mm_ss, 0.0),
							new Clothoid(clothoid,1),
							new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v+1, clothoid.v, a_mm_ss, 0.0)
						));
					}
//					printf("L\r\n");
				}
			}

			mouse->ui->SetLED(path_index);

		}else{

			trajectory->GetTargetPosition(&target_x, &target_y, &target_theta);
			trajectory->GetTargetVelocity(&target_vx,&target_vy,&target_omega);

			float Kp_wall_correction = 0.004;//0.004;
			if(target_vy>500){
				Kp_wall_correction=0.00006*(target_vy-500);//v=0~2000; 0~1300
			}
			static float pre_error_wall;
			float period_s=0.001;
			float wall_control=Kp_wall*mouse->wall_sensor->GetError() + Kd_wall *(mouse->wall_sensor->GetError()-pre_error_wall)/period_s;
			if(trajectory->GetTragType()==line)target_omega+=wall_control;

			pre_error_wall=mouse->wall_sensor->GetError();

			Jacobian(target_vy,target_omega,&target_velocity_r,&target_velocity_l);

			mouse->motorR_PID->SetTarget(target_velocity_r);
			mouse->motorL_PID->SetTarget(target_velocity_l);
	//		mouse->motorR_PID->SetTarget(200);
	//		mouse->motorL_PID->SetTarget(200);

			velocity_r=mouse->encorders->GetVelociryR_mm_s();
			velocity_l=mouse->encorders->GetVelociryL_mm_s();
			V_r=mouse->motorR_PID->Update(velocity_r);
			V_l=mouse->motorL_PID->Update(velocity_l);

			mouse->motors->SetVoltageR(V_r);
			mouse->motors->SetVoltageL(V_l);
		}

	}
	if(end_run_flag){
		next_mode=modeSelect_mode;
	}
}
