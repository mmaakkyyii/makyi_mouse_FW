#include "MachineMode.hpp"
#include  <memory>
#include "debug.hpp"
#include "machine_paramater.h"
#include "Jacobian.hpp"
#include "clothoid_param.hpp"

#include "MazeDef.hpp"
#include "Adachi.hpp"

#include "flash_util.hpp"

bool end_serch_flag;
bool return_start_flag;
bool flash_flag=false;
float v_max;
float turn_v_max;
float turn_omega_max;
float a_omega;

float acc;

bool goal_flag=false;

const int log_data_num=3000;
int log_data[log_data_num][4];
int log_index=0;
float gyro[3];
int gyro_raw[3];

float Kp_wall=0.004;//0.004
float Kd_wall=0.0001;//0.0003
const float Kp_omega=0.0005;//0.001
const float crash_acc=45.0*2;



ModeSelect::ModeSelect(Mouse* _mouse):MachineMode(_mouse){
	current_mode=modeSelect_mode;
	next_mode=modeSelect_mode;
	mode_val=0;
	sw1=mouse->ui->GetSW1();
	pre_sw1=sw1;
	sw2=mouse->ui->GetSW2();
	pre_sw2=sw2;
	sw3=mouse->ui->GetSW3();
	pre_sw3=sw3;
}

void ModeSelect::Init(){
	sw1=mouse->ui->GetSW1();
	pre_sw1=sw1;
	sw2=mouse->ui->GetSW2();
	pre_sw2=sw2;
	sw3=mouse->ui->GetSW3();
	pre_sw3=sw3;
	printf("ModeSelect%d\n\r",mode_val);
	
}
void ModeSelect::Loop(){
	//printf("%d\n\r",mode_val);
}
void ModeSelect::Interrupt_1ms(){
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);

	pre_sw1=sw1;
	pre_sw2=sw2;
	pre_sw3=sw3;
	sw1=mouse->ui->GetSW1();
	sw2=mouse->ui->GetSW2();
	sw3=mouse->ui->GetSW3();
	
	if(sw1<pre_sw1){
		if(mode_val<15){
			mouse->buzzer->On_ms(400,20);
			mode_val++;
			printf("mode:%d\n\r",mode_val);
		}else{
			mouse->buzzer->On_ms(600,20);
			mode_val=0;
			printf("mode:%d\n\r",mode_val);

		}
	}
	if(sw2<pre_sw2){

		switch(mode_val){
		case 0:
			next_mode=serchRun_mode;
			mouse->buzzer->On_ms(800,50);
			break;
		case 1:
			next_mode=fastRun_mode;
			mouse->buzzer->On_ms(800,50);
			break;
		case 2:
			next_mode=debug_mode;
			mouse->buzzer->On_ms(800,50);
			break;
		case 3:
			next_mode=parameterSetting_mode;
			mouse->buzzer->On_ms(800,50);
			break;
		case 4:
			next_mode=sensorCheck_mode;
			mouse->buzzer->On_ms(800,50);
			break;
		case 5:
			next_mode=doNotRotate_mode;
			mouse->buzzer->On_ms(800,50);
			break;
		case 6:
			next_mode=logOutput_mode;
			mouse->buzzer->On_ms(800,50);
			break;
		case 15:
			next_mode=reset_map;
			mouse->buzzer->On_ms(800,50);
			break;
		}
	}
	mouse->ui->SetLED(mode_val);

}
/////////////////////////////
/*
Dirction GetRotetaLeft(Dirction dir){
	switch(dir){
	case North:
		return West;
		break;
	case West:
		return South;
		break;
	case South:
		return East;
		break;
	case East:
		return North;
		break;
	}
	return North;
	//return -1;
}
Dirction GetRotetaRight(Dirction dir){
	switch(dir){
	case North:
		return East;
		break;
	case West:
		return North;
		break;
	case South:
		return West;
		break;
	case East:
		return South;
		break;
	}
	return North;
	//return -1;
}
//*/


Trajectory* trajectryUpdate(Mouse* mouse,clothoid_params clothoid){
		Trajectory* traj;			
			if(return_start_flag==true){
				//next_mode=modeSelect_mode;
				end_serch_flag=true;
				return new Stop();
			}

			mouse->ui->SetLED( mouse->wall_sensor->GetWallL() <<3 |  
					   mouse->wall_sensor->GetWallFL()<<2 |
					   mouse->wall_sensor->GetWallFR()<<1 |  
					   mouse->wall_sensor->GetWallR()       );
			mouse->maze_solver->adachi.MakeStepMap(mouse->goal_pos_x,mouse->goal_pos_y,mouse->wall_mask);
	 		
			if(mouse->mouse_pos_x==mouse->goal_pos_x && mouse->mouse_pos_y==mouse->goal_pos_y ){
				flash_flag=true;
				mouse->goal_time++;
				if(mouse->goal_time%2==1){
					mouse->goal_pos_x = 0;
					mouse->goal_pos_y = 0;
					goal_flag=true;
				}else if(mouse->goal_time%2==0){
					return_start_flag=true;
				}
				switch(mouse->mouse_dir){
				case North:
					mouse->mouse_dir=South;
					mouse->mouse_pos_y--;
					break;
				case West:
					mouse->mouse_dir=East;
					mouse->mouse_pos_x++;
					break;
				case South:
					mouse->mouse_dir=North;
					mouse->mouse_pos_y++;
					break;
				case East:
					mouse->mouse_dir=West;
					mouse->mouse_pos_x--;
					break;
				}
				if(return_start_flag!=true){
					//mouse->buzzer->On_ms(240,500);
					traj=new DoubleTrajectory(
						new MultTrajectory(
							new Line(0.0, SECTION_WIDTH/2, 0.0 , v_max, v_max, 0.0, acc, 0.0),
							new Rotate(180,turn_omega_max,a_omega),
							new Stay(100)
						),
						new DoubleTrajectory(
							new Stay(1500),
							new Line(0.0, SECTION_WIDTH/2.0, 0.0, 0, v_max, v_max, acc, 0.0)
						)
						);
				}else{
					//mouse->buzzer->On_ms(240,500);
					traj=new MultTrajectory(
							new Line(0.0, SECTION_WIDTH/2, 0.0 , v_max, v_max, 0.0, acc, 0.0),
//							new Rotate(180, 0, turn_v_max, 0, 1),
							new Rotate(180,turn_omega_max,a_omega),
							new Stay(500)
						);
				}
			}else{
				if(mouse->runing_buzzer)mouse->buzzer->On_ms(200,50);
				int wall = mouse->GetWallInfo();
				mouse->maze_solver->adachi.SetMap(mouse->mouse_pos_x,mouse->mouse_pos_y,wall,mouse->mouse_dir);

				Dirction pre_mouse_dir = mouse->mouse_dir;
				
				Dirction next_dir = mouse->maze_solver->adachi.GetNextDirection(mouse->mouse_pos_x,mouse->mouse_pos_y,mouse->mouse_dir);
				switch(next_dir){
				case North:
					mouse->mouse_dir=North;
					mouse->mouse_pos_y++;
					break;
				case West:
					mouse->mouse_dir=West;
					mouse->mouse_pos_x--;
					break;
				case South:
					mouse->mouse_dir=South;
					mouse->mouse_pos_y--;
					break;
				case East:
					mouse->mouse_dir=East;
					mouse->mouse_pos_x++;
					break;
				default:
					mouse->buzzer->On_ms(1500,100);
					break;
				}

				float a_clothoid_line=1000.0;
				switch( (int)next_dir - (int)pre_mouse_dir ){
				case -3:
					traj=new MultTrajectory(
						new Line(0.0, clothoid.in_mm, 0.0, v_max, v_max, clothoid.v, a_clothoid_line, 0.0),
						new Clothoid(clothoid,1),
						new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, v_max, v_max, a_clothoid_line, 0.0)
						);
					break;
				case -2:
					traj=new MultTrajectory(
						new Line(0.0, SECTION_WIDTH/2.0, 0.0, v_max, v_max, v_max, acc, 0.0),
						new MultTrajectory(
							new Rotate(180,turn_omega_max,a_omega),
							new ConstantVoltage(-MACHINE_BACK_VOLTAGE_R,-MACHINE_BACK_VOLTAGE_L,MACHINE_BACK_TIME),
							new Stay(100)
							//new Stay(100)	
							//new Line(0.0, -180.0, 0.0, 0, 400, 0, 400.0, 0.0)
						),
						new Line(0.0, SECTION_WIDTH/2.0+MACHINE_BACK_LENGTH, 0.0, 0, v_max, v_max, acc, 0.0)
//						new Line(0.0, 180/2.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
						);
					break;
				case -1:
					traj=new MultTrajectory(
						new Line(0.0, clothoid.in_mm, 0.0, v_max, v_max, clothoid.v, a_clothoid_line, 0.0),
						new Clothoid(clothoid,-1),
						new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, v_max, v_max, a_clothoid_line, 0.0)
						);
					break;
				case 0:
					traj=new Line(0.0, SECTION_WIDTH, 0.0, v_max, v_max, v_max, 10000.0, 0.0);
					break;
				case 1:
					traj=new MultTrajectory(
						new Line(0.0, clothoid.in_mm, 0.0, v_max, v_max, clothoid.v, a_clothoid_line, 0.0),
						new Clothoid(clothoid,1),
						new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, v_max, v_max, a_clothoid_line, 0.0)
						);
					break;
				case 2:
					traj=new MultTrajectory(
						new Line(0.0, SECTION_WIDTH/2.0, 0.0, v_max, v_max, v_max, acc, 0.0),
						new MultTrajectory(
							new Rotate(180,turn_omega_max,a_omega),
							//new Stay(100)	
							new ConstantVoltage(-MACHINE_BACK_VOLTAGE_R,-MACHINE_BACK_VOLTAGE_L,MACHINE_BACK_TIME),
							new Stay(100)

							//new Line(0.0, -180.0, 0.0, 0, 400, 0, 400.0, 0.0)
						),
						new Line(0.0, SECTION_WIDTH/2.0+MACHINE_BACK_LENGTH, 0.0, 0, v_max, v_max, acc, 0.0)
//						new Line(0.0, 180/2.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
						);
					break;
				case 3:
					traj=new MultTrajectory(
						new Line(0.0, clothoid.in_mm, 0.0, v_max, v_max, clothoid.v, a_clothoid_line, 0.0),
						new Clothoid(clothoid,-1),
						new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, v_max, v_max, a_clothoid_line, 0.0)
						);
					break;
				default:
					traj=new Stop();
					mouse->buzzer->On_ms(150,100);
					break;
				}
			}
	return traj;
}

/////////////////////////////
SerchRun::SerchRun(Mouse* _mouse)
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
sla_mode(0),
gesture_flag(false),
no_hand_flag(false),
timer(0),
idle(true)
{
clothoid=clothoid_200mm_90deg_1;
flash_flag=false;
};
void SerchRun::Loop(){
	if(idle){
		printf("sens:%d,%d,%d,%d\r\n",mouse->wall_sensor->GetLeft(),mouse->wall_sensor->GetFrontL(),mouse->wall_sensor->GetFrontR(),mouse->wall_sensor->GetRight());
		//printf("%d,%d,%d,%d\r\n",(int)clothoid.v,(int)clothoid.in_mm,(int)clothoid.out_mm);
	}else{
		//mouse->imu->GetGyro(gyro);

		printf("%d,(%d,%d)%d,%d|%d,%d|%d,%d\r\n",trajectory->GetTragType(),(int)mouse->mouse_pos_x,mouse->mouse_pos_y,(int)target_velocity_r,(int)target_velocity_l,(int )(V_r*1000),(int )(V_l*1000),(int)velocity_r,(int)velocity_l);
	}
	float acc_data[3];
	mouse->imu->GetAcc(acc_data);
	static int reverse_time=0;
	if(acc_data[2]<-5.0){
		reverse_time++;
		if(reverse_time>50){
			next_mode=modeSelect_mode;
			mouse->buzzer->On_ms(240,40);
		}
	}else{
		reverse_time=0;
	}
}

void SerchRun::Init(){
	current_mode=serchRun_mode;
	next_mode=serchRun_mode;
	
	trajectory=std::unique_ptr<Stop>(new Stop());

	mouse->motorR_PID->Reset();
	mouse->motorL_PID->Reset();
	
	v_max=200;
	acc=3500;

	turn_v_max=200;
	turn_omega_max=2*200/50;
	a_omega=80;

	clothoid=clothoid_200mm_90deg_1;

	mouse->mouse_pos_x=0;
	mouse->mouse_pos_y=0;
	mouse->mouse_dir=North;
	mouse->goal_time=0;
	mouse->wall_mask=USE_UNKOWN_WALL_MASK;	
	
	idle=true;
	end_serch_flag=false;
	return_start_flag=false;
	mouse->runing_buzzer=false;
}


void SerchRun::Interrupt_1ms(){
	static float thetaa=0;
	static float sum_theta=0;
	static int log_index=0;


	if(idle){
		static int sw1,sw2,pre_sw1,pre_sw2;

		pre_sw1=sw1;
		pre_sw2=sw2;
		sw1=mouse->ui->GetSW1();
		sw2=mouse->ui->GetSW2();

		if(pre_sw1>sw1){
			sla_mode++;
			mouse->buzzer->On_ms(300,40);
			if(sla_mode>8){
				sla_mode=0;
				mouse->buzzer->On_ms(500,40);
			}
		}
		static int led=0;
		static int led_timer=0;
		led_timer++;
		if(led_timer>200){
			led_timer=0;
			led=8-led;
		}
		mouse->ui->SetLED(sla_mode | led);

		
		switch(sla_mode){
			case 0:
				clothoid=clothoid_150mm_90deg_1;
				v_max=180;
				acc=2000;
				break;
			case 1:
				clothoid=clothoid_150mm_90deg_1;
				v_max=150;
				acc=2000;
				break;
			case 2:
				clothoid=clothoid_150mm_90deg_1;
				v_max=200;
				acc=2000;
				break;
			case 3:
				clothoid=clothoid_150mm_90deg_1;
				v_max=250;
				acc=2000;
				break;
			case 4:
				clothoid=clothoid_150mm_90deg_1;
				v_max=250;
				acc=3500;
				break;
			default: 
				clothoid=clothoid_200mm_90deg_1;
				v_max=200;
				acc=3500;
				break;
		}

		static int gesture_sensorR_th=250;
		static int gesture_sensorL_th=250;

//		static int gesture_sensor_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensorR_th || mouse->wall_sensor->GetFrontL() > gesture_sensorL_th )){
			gesture_flag=true;
			mouse->buzzer->On_ms(300,40);
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensorR_th && mouse->wall_sensor->GetFrontL()< gesture_sensorL_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(400,40);
		}
		bool cal=false;
		if(no_hand_flag)cal=mouse->imu->Calibration();
		if(cal){
			idle=false;
			mouse->ui->SetLED(15);
			trajectory=std::unique_ptr<Line>(new Line(0.0, SECTION_WIDTH/2.0, 0.0, 0.0, v_max, v_max, acc, 0.0));
			mouse->mouse_pos_y++;
			//trajectory=new Rotate(90, 0, 100.0, 0, 1);
			trajectory->Update();

		}
	}else{
		if(trajectory->Update()){
			trajectory = std::unique_ptr<Trajectory>(trajectryUpdate(mouse,clothoid));
			//trajectory =std::unique_ptr<Line>(new Line(0.0, SECTION_WIDTH, 0.0, v_max, v_max, v_max, 10000.0, 0.0));

			thetaa=0;
			sum_theta=0;
			log_index=0;
			mouse->motors->SetVoltageR(V_r);
			mouse->motors->SetVoltageL(V_l);

		}else{

			trajectory->GetTargetPosition(&target_x, &target_y, &target_theta);
			trajectory->GetTargetVelocity(&target_vx,&target_vy,&target_omega);
			static float pre_error_wall;
			float period_s=0.001;
			float wall_control=Kp_wall*mouse->wall_sensor->GetError() + Kd_wall *(mouse->wall_sensor->GetError()-pre_error_wall)/period_s;
			if(trajectory->GetTragType()==line)target_omega+=wall_control;

			pre_error_wall=mouse->wall_sensor->GetError();

			static float Kp_theta=10;//10
			static float Ki_theta=0.0;
			
			mouse->imu->GetGyro(gyro);
			thetaa+=gyro[2]*3.14/180.0*0.001;

			float e_theta=target_theta-thetaa;
			sum_theta+=e_theta;
			if(trajectory->GetTragType()==rotate){

				target_omega+= Kp_theta*e_theta + Ki_theta*sum_theta;
				if(log_index<log_data_num-1){
					log_data[log_index][0]=(int)(target_theta*1000);
					log_data[log_index][1]=(int)(thetaa*1000);
					log_index++;
				}
			}else if(flash_flag==true && trajectory->GetTragType()==stay){
				flash_flag=false;
				mouse->buzzer->On_ms(500,500);
				int param_data[param_data_num]={1,FlashGetGoalX(),FlashGetGoalY(),0,0,0,0,0};
				FlashSetData(mouse->maze_solver->adachi.map,param_data);

			}else{
				sum_theta=0;
				thetaa=0;
			}
			
			Jacobian(target_vy,target_omega,&target_velocity_r,&target_velocity_l);
			
			mouse->motorR_PID->SetTarget(target_velocity_r);
			mouse->motorL_PID->SetTarget(target_velocity_l);
	//		mouse->motorR_PID->SetTarget(200);
	//		mouse->motorL_PID->SetTarget(200);

			velocity_r=mouse->encorders->GetVelociryR_mm_s();
			velocity_l=mouse->encorders->GetVelociryL_mm_s();
			V_r=mouse->motorR_PID->Update(velocity_r);
			V_l=mouse->motorL_PID->Update(velocity_l);
			
			if(trajectory->GetTragType()!=constant_voltage){
				mouse->motors->SetVoltageR(V_r);
				mouse->motors->SetVoltageL(V_l);
			}else{
				mouse->motors->SetVoltageR(target_vx);
				mouse->motors->SetVoltageL(target_vy);
				mouse->motorR_PID->Reset();
				mouse->motorL_PID->Reset();
			}
		}

	}
	if(end_serch_flag){
		next_mode=modeSelect_mode;
	}
}

/////////////////////////////
FastRun::FastRun(Mouse* _mouse):
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
goal(false),
idle(true),
path_length(0),
path_index(0),
crash_en(false)
{
	clothoid=clothoid_350mm_90deg_short;

};
void FastRun::Loop(){
	if(mouse->ui->GetSW2()==0 && mouse->ui->GetSW1()==0){
		mouse->buzzer->On_ms(3000,100);
		next_mode=modeSelect_mode;
	}

	if(idle){
	//	printf("%d,%d,%d,%d\r\n",(int)clothoid.v,(int)clothoid.in_mm,(int)clothoid.out_mm);

	//	printf("sens:%d,%d,%d,%d\r\n",mouse->wall_sensor->GetLeft(),mouse->wall_sensor->GetFrontL(),mouse->wall_sensor->GetFrontR(),mouse->wall_sensor->GetRight());
	}else{
		printf("%d,%d,%d,%d\r\n",(int)target_velocity_r,(int)target_velocity_l,(int)velocity_r,(int)velocity_l);
	}
}
void FastRun::Init(){
	mouse->ui->SetLED(0);
	if(goal_flag==false && FlashGetGoalFlag()==0){
		current_mode=fastRun_mode;
		next_mode=modeSelect_mode;
		mouse->buzzer->On_ms(2500,400);
		return;	
	}
	current_mode=fastRun_mode;
	next_mode=fastRun_mode;

	trajectory=std::unique_ptr<Stop>(new Stop());

	mouse->motorR_PID->Reset();
	mouse->motorL_PID->Reset();
	
	v_max=600;
	turn_v_max=250;
	turn_omega_max=2*100/50;
	a_omega=80;
	acc=3500;
	clothoid=clothoid_200mm_90deg_1;

	mouse->mouse_pos_x=0;
	mouse->mouse_pos_y=0;
	mouse->goal_pos_x=goal_x_setting;
	mouse->goal_pos_y=goal_y_setting;
	mouse->mouse_dir=North;
	mouse->goal_time=0;
	mouse->wall_mask=UNUSE_UNKOWN_WALL_MASK;	
	
	idle=true;
	goal=false;
	end_serch_flag=false;
	return_start_flag=false;
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

void FastRun::Interrupt_1ms(){
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
			clothoid=clothoid_200mm_90deg_1;
			break;
		case 1:
			clothoid=clothoid_250mm_90deg_1;
			break;
		case 2:
			clothoid=clothoid_300mm_90deg_1;
			break;
		default:
			clothoid=clothoid_200mm_90deg_1;
			break;
		}

		switch(vel_mode){
		case 0:
			v_max=200;
			acc=2000;
			break;
		case 1:
			v_max=400;
			acc=2000;
			break;
		case 2:
			v_max=500;
			acc=2000;
			break;
		case 3:
			v_max=550;
			acc=2000;
			break;
		case 4:
			v_max=600;
			acc=2000;
			break;
		case 5:
			v_max=650;
			acc=2000;
			break;
		case 6:
			v_max=700;
			acc=2000;
			break;
		case 7:
			v_max=750;
			acc=2000;
			break;
		case 8:
			v_max=800;
			acc=2000;
			break;
		default:
			v_max=200;
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
			

			trajectory=std::unique_ptr<Line>(new Line(0.0, SECTION_WIDTH/2.0+(stright_num-1)*SECTION_WIDTH, 0.0, 0.0, v_max, clothoid.v, acc, 0.0));
			//mouse->mouse_pos_y++;
		}
	}else{
		if(acc_data[0]*acc_data[0]+acc_data[1]*acc_data[1]>crash_acc*crash_acc){
			next_mode=modeSelect_mode;
		}

		if(trajectory->Update()){

			//printf("path index:%d run:%d\r\n",path_index ,mouse->maze_solver->adachi.run_plan[path_index]);
			if(path_index>=path_length){
				mouse->buzzer->On_ms(4000,40);
//					path_index++;
				end_serch_flag=true;
				
/*
				if(goal){
					mouse->goal_pos_x=0;
					mouse->goal_pos_y=0;
					mouse->mouse_dir=South;
					path_length = mouse->maze_solver->adachi.MakePathPlan(mouse->mouse_pos_x,mouse->mouse_pos_y,mouse->mouse_dir,mouse->goal_pos_x,mouse->goal_pos_y);
					//printf("MakeRunPlan %d\r\n",path_length);
					mouse->maze_solver->adachi.MakeRunPlan(path_length,mouse->mouse_dir);
					path_index=0;

					int stright_num=0;
					while(mouse->maze_solver->adachi.run_plan[path_index] == Forward){
						path_index++;
						stright_num++;
					}
					
					delete trajectory;
					trajectory=new Line(0.0, 180.0/2.0+(stright_num-1)*180.0, 0.0, 0.0, v_max, clothoid.v, acc, 0.0);
					//printf("traj straight%d\r\n",stright_num);


				}else{
					mouse->mouse_pos_x=GOAL_X;
					mouse->mouse_pos_y=GOAL_Y;
					mouse->goal_pos_x = 0;
					mouse->goal_pos_y = 0;
					
		//			mouse->mouse_dir=GetRotetaRight(mouse->mouse_dir);
		//			mouse->mouse_dir=GetRotetaRight(mouse->mouse_dir);

					goal=true;
					delete trajectory;
					trajectory= new MultTrajectory(
						new Line(0.0, 180.0/2.0, 0.0, clothoid.v, clothoid.v , 0, acc, 0.0),
						new Rotate(180,turn_omega_max,a_omega),
						new Stay(100)
					);
				}
*/
			}else if(path_index>=path_length+1){
				end_serch_flag=true;
			}else{
				int stright_num=0;
				if(mouse->maze_solver->adachi.run_plan[path_index] == TurnRight){
					path_index++;
		//			mouse->mouse_dir=GetRotetaRight(mouse->mouse_dir);
//				printf("%d,%d,%d\r\n",(int)clothoid.v,(int)clothoid.in_mm,(int)clothoid.out_mm);

					if(path_index>=path_length){
						trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
							new Line(0.0, clothoid.in_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, acc, 0.0),
							new Clothoid(clothoid,-1),
							new Line(0.0, clothoid.out_mm+SECTION_WIDTH/2, 0.0, clothoid.v, clothoid.v, 0, acc, 0.0)
							));
					}else{
						trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
							new Line(0.0, clothoid.in_mm, 0.0, clothoid.v, clothoid.v+1, clothoid.v, acc, 0.0),
							new Clothoid(clothoid,-1),
							new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v+1, clothoid.v, acc, 0.0)
						));

					}

//					printf("R\r\n");
				}else if(mouse->maze_solver->adachi.run_plan[path_index] == TurnLeft){
					path_index++;
		//			mouse->mouse_dir=GetRotetaLeft(mouse->mouse_dir);

					if(path_index>=path_length){
						trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
							new Line(0.0, clothoid.in_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, acc, 0.0),
							new Clothoid(clothoid,1),
							new Line(0.0, clothoid.out_mm+SECTION_WIDTH/2, 0.0, clothoid.v, clothoid.v+1, 0, acc, 0.0)
							));
					}else{
						trajectory =std::unique_ptr<MultTrajectory>(new MultTrajectory(
							new Line(0.0, clothoid.in_mm, 0.0, clothoid.v, clothoid.v, clothoid.v, acc, 0.0),
							new Clothoid(clothoid,1),
							new Line(0.0, clothoid.out_mm, 0.0, clothoid.v, clothoid.v+1, clothoid.v, acc, 0.0)
						));
					}
//					printf("L\r\n");
				}else if(mouse->maze_solver->adachi.run_plan[path_index] == Forward){
					while(mouse->maze_solver->adachi.run_plan[path_index] == Forward){
						if(path_index<path_length){
							path_index++;
							stright_num++;
						}else{
							break;
						}
					}

					if(path_index>=path_length){
						path_index++;
						trajectory=std::unique_ptr<Line>(new Line(0.0, (stright_num)*SECTION_WIDTH+SECTION_WIDTH/2, 0.0, clothoid.v, v_max, 0,          acc, 0.0));
					}else{
						trajectory=std::unique_ptr<Line>(new Line(0.0, (stright_num)*SECTION_WIDTH        , 0.0, clothoid.v, v_max, clothoid.v, acc, 0.0));
					}
//					printf("S%d\r\n",stright_num);
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
	if(end_serch_flag){
		next_mode=modeSelect_mode;
	}
}

/////////////////////////////
LowBattery::LowBattery(Mouse* _mouse):MachineMode(_mouse){
	current_mode=lowBattery_mode;
	next_mode=lowBattery_mode;
};
void LowBattery::Loop(){
	printf("Battery Voltage is Low! %d\n\r",(int)(1000*mouse->battery_check->GetBatteryVoltage_V()));
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);
	if(mouse->ui->GetSW2()==0 && mouse->ui->GetSW3()==0){
		mouse->buzzer->On_ms(300,100);
		next_mode=modeSelect_mode;
	}


}
void LowBattery::Init(){
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);
}
void LowBattery::Interrupt_1ms(){
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);
	static int timer=0;
	static int led=1;
	if(timer>100){
		timer=0;
		led=9-led;
	}
	timer++;
	mouse->ui->SetLED(led);

//	mouse->buzzer->On_ms(400,1000);
}

/////////////////////////////
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
	mouse->imu->GetGyroRaw(gyro_raw);

	if(idle){
		static int gesture_sensor_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensor_th || mouse->wall_sensor->GetFrontL() > gesture_sensor_th )){
			gesture_flag=true;
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensor_th && mouse->wall_sensor->GetFrontL()< gesture_sensor_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(4000,40);
		}
		bool cal=false;
		if(no_hand_flag)cal=mouse->imu->Calibration();
		if(cal){
			idle=false;
		}
	}else{


		log_index=0;
		mouse->imu->GetGyro(gyro);
		mouse->imu->GetGyroRaw(gyro_raw);
		if(log_index<log_data_num){
			log_data[log_index][0]=gyro_raw[2];
			log_index++;
		}else{
			mouse->motors->SetVoltageR(0);
			mouse->motors->SetVoltageL(0);
			next_mode=modeSelect_mode;
			mouse->buzzer->On_ms(3000,100);

		}
		omega_z=gyro[2];
		gyro_theta+=(omega_z)*0.001;
		static float e_sum=0;
		float Kp=0.15;//2;
		float Ki=0.01;//0.02;
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

	}	

}
/////////////////////////////
SensorCheck::SensorCheck(Mouse* _mouse):MachineMode(_mouse){
	current_mode=sensorCheck_mode;
	next_mode=sensorCheck_mode;
}

float acc_data[3];

void SensorCheck::Loop(){
//	printf("\x1b[2J");		//�N���A�X�N���[��[CLS]
//	printf("\x1b[0;0H");	//�J�[�\����0,0�Ɉړ�
	float _x,_y,_theta;
	mouse->localization->GetPosition(&_x, &_y, &_theta);

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
			printf("%4d,%4d,%4d,%4d,%4d,%d,%d,%d,%d,%5d,%5d\r\n",
				(int)(acc_data[0]*acc_data[0]+acc_data[1]*acc_data[1]),
				(int)(acc_data[0]*1000),
				(int)(acc_data[1]*1000),
				(int)(theta_gyro),
				(int)(1000*mouse->battery_check->GetBatteryVoltage_V()),
				mouse->wall_sensor->GetLeft(),
				mouse->wall_sensor->GetFrontL(),
				mouse->wall_sensor->GetFrontR(),
				mouse->wall_sensor->GetRight(),
				(int)(mouse->encorders->GetVelociryL_mm_s()),
				(int)(mouse->encorders->GetVelociryR_mm_s())
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
	mouse->imu->GetAcc(acc_data);
	float acc_th=1000;
	if(acc_data[0]*acc_data[0]+acc_data[1]*acc_data[1]>acc_th){
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
	mouse->ui->SetLED( mouse->wall_sensor->GetWallR() <<3 |
			mouse->wall_sensor->GetWallFR()<<2 |
			mouse->wall_sensor->GetWallFL()<<1 |
			mouse->wall_sensor->GetWallL()       );

	if(mouse->ui->GetSW1()==0){
		mouse->buzzer->On_ms(3000,10);
		next_mode=modeSelect_mode;
	}

};

/////////////////////////////
void ParameterSetting::Loop(){
	printf("%d (%d,%d)\r\n",mode,x,y);
};
void ParameterSetting::Init(){
	x=goal_x_setting;
	y=goal_y_setting;
};
void ParameterSetting::Interrupt_1ms(){
	float acc_data[3];
	mouse->imu->GetAcc(acc_data);

	static int sw1,sw2,pre_sw1,pre_sw2;

	pre_sw1=sw1;
	pre_sw2=sw2;
	sw1=mouse->ui->GetSW1();
	sw2=mouse->ui->GetSW2();

	static int blink_time_ms=200;

	switch(mode){
	case 0:
		if(mouse->encorders->GetVelociryR_mm_s()>100){
			mouse->buzzer->On_ms(500,100);
			mode=1;
			blink_time_ms=100;
		}

		if(pre_sw1>sw1){
			x++;
			if(x>MAZESIZE_X)x=0;
			led=x;
		}

		if(time_ms>blink_time_ms){
			led=x-led;
			time_ms=0;
		}
		mouse->ui->SetLED(led);


		break;
	case 1:
		if(mouse->encorders->GetVelociryL_mm_s()>100){
			mouse->buzzer->On_ms(300,100);
			mode=0;
			blink_time_ms=200;
		}

		if(pre_sw1>sw1){
			y++;
			if(y>MAZESIZE_Y)y=0;
			led=y;
		}

		if(time_ms>blink_time_ms){
			led=y-led;
			time_ms=0;
		}
		mouse->ui->SetLED(led);


		break;
	}


	time_ms++;

	if(acc_data[2]<-7.0){
		mouse->buzzer->On_ms(400,500);
		mouse->goal_pos_x=x;
		goal_x_setting=x;
		mouse->goal_pos_y=y;
		goal_y_setting=y;
		int param_data[param_data_num]={0,goal_x_setting,goal_y_setting,0,0,0,0,0};

		FlashSetData(mouse->maze_solver->adachi.map,param_data);

		next_mode=modeSelect_mode;
	}


};
ParameterSetting::ParameterSetting(Mouse* _mouse):MachineMode(_mouse){
current_mode=parameterSetting_mode;
next_mode=parameterSetting_mode;
};

/////////////////////////////
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
idle(true)
{
};
void Debug::Loop(){
	printf("%d,%d,%d,%d\r\n",(int)target_velocity_r,(int)target_velocity_l,(int)velocity_r,(int)velocity_l);

}
void Debug::Init(){
	current_mode=debug_mode;
	next_mode=debug_mode;

	printf("Start debug mode!\n\r");
	v_max=400;
	turn_v_max=300;
	
	idle=true;
	timer=0;
	end_serch_flag=false;
	return_start_flag=false;
	gesture_flag=false;
	no_hand_flag=false;
	wait_ms=0;


	log_index=0;


}

void Debug::Interrupt_1ms(){
	mouse->imu->GetGyro(gyro);
	mouse->imu->GetGyroRaw(gyro_raw);

	if(idle){
		static int gesture_sensor_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensor_th || mouse->wall_sensor->GetFrontL() > gesture_sensor_th )){
			gesture_flag=true;
			mouse->buzzer->On_ms(200,40);
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensor_th && mouse->wall_sensor->GetFrontL()< gesture_sensor_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(400,40);
		}
		bool cal=false;
		if(no_hand_flag){
			wait_ms+=1;
			if(wait_ms>1000){
				cal=true;
				mouse->buzzer->On_ms(500,100);

			}
		}
		if(cal){
			idle=false;

/*

			acc=2000;
			clothoid_params clothoid=clothoid_200mm_90deg_1;

			trajectory= std::unique_ptr<MultTrajectory>(new MultTrajectory(
					new Line(0.0, SECTION_WIDTH*3/2.0+clothoid.in_mm, 0.0, 0, 600, clothoid.v, 2000.0, 0.0),
					new Clothoid(clothoid,-1),
					new Line(0.0, clothoid.out_mm+SECTION_WIDTH*3/2.0, 0.0, clothoid.v, clothoid.v, 0, 2000.0, 0.0)
				));
//*/

			turn_v_max=200;
			turn_omega_max=2*100/50;
			a_omega=80;


			trajectory= std::unique_ptr<Line>(new Line(0.0, SECTION_WIDTH * 3, 0.0, 0, 400, 0, 2000.0, 0.0));
			//trajectory= std::unique_ptr<Rotate>(new Rotate(360*1,turn_omega_max,a_omega));
			mouse->mouse_pos_y++;
		}
	}else{
		if(trajectory->Update()){
			trajectory = std::unique_ptr<Stop>(new Stop());
			end_serch_flag=true;

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
	//		mouse->motorR_PID->SetTarget(200);
	//		mouse->motorL_PID->SetTarget(200);

			velocity_r=mouse->encorders->GetVelociryR_mm_s();
			velocity_l=mouse->encorders->GetVelociryL_mm_s();
			V_r=mouse->motorR_PID->Update(velocity_r);
			V_l=mouse->motorL_PID->Update(velocity_l);


			mouse->motors->SetVoltageR(V_r);
			mouse->motors->SetVoltageL(V_l);

///*
			if(log_index<log_data_num){
				log_data[log_index][0]=(int)(target_velocity_r);
				log_data[log_index][1]=(int)(velocity_r);
				log_data[log_index][2]=(int)(target_velocity_l);
				log_data[log_index][3]=(int)(velocity_l);
				log_index++;
			}else{
//				mouse->motors->SetVoltageR(0);
//				mouse->motors->SetVoltageL(0);
//				next_mode=modeSelect_mode;
//				mouse->buzzer->On_ms(300,100);
			}
//*/

		}

	}
	if(end_serch_flag){
		printf("deleat\r\n");
		next_mode=modeSelect_mode;
	}

}
///////////////////////////
void LogOutput::Loop(){

	printf("%d,%d,%d,%d\r\n",log_data[index][0],log_data[index][1],log_data[index][2],log_data[index][3]);
	index++;
	if(index>log_data_num)next_mode=modeSelect_mode;
}
void LogOutput::Init(){
	current_mode=logOutput_mode;
	next_mode=logOutput_mode;
	index=0;

}
void LogOutput::Interrupt_1ms(){
}
LogOutput::LogOutput(Mouse* _mouse)
:MachineMode(_mouse),
index(0)
{
}

void ResetMap::Loop(){};
void ResetMap::Init(){
	current_mode=reset_map;
	next_mode=reset_map;
	printf("ResetMap\r\n");
	int map_data[MAZESIZE_X][MAZESIZE_Y]={0};
	int i=0;
	for(int x=0;x<MAZESIZE_X;x++){
		for(int y=0;y<MAZESIZE_Y;y++){
			map_data[x][y]=0;
			i++;
		}
	}
	mouse->maze_solver->adachi.InitMaze(UNKNOWN, map_data);
		int param_data[param_data_num]={0,goal_x_setting,goal_y_setting,0,0,0,0,0};

		FlashSetData(mouse->maze_solver->adachi.map,param_data);
	//FlashPrintMazeData(mouse->maze_solver->adachi.map);

};
void ResetMap::Interrupt_1ms(){

	next_mode=modeSelect_mode;
};


