#include "MachineMode.hpp"
#include  <memory>
#include "debug.hpp"
#include "machine_paramater.h"
//#include "clothoid_param.hpp"

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

float gyro[3];
int gyro_raw[3];

float Kp_wall=0.002;//0.004
float Kd_wall=0.00001;//0.0003
const float Kp_omega=0.0005;//0.001
const float crash_acc=45.0*2;



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
	static float theta_rad=0;
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

			theta_rad=0;
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
			theta_rad+=gyro[2]*3.14/180.0*0.001;
			float e_theta=target_theta-theta_rad;
			sum_theta+=e_theta;
			if(trajectory->GetTragType()==rotate){

				target_omega+= Kp_theta*e_theta + Ki_theta*sum_theta;
				if(mouse->log_index < mouse->log_data_num-1){
					mouse->log_data[mouse->log_index][0]=(int)(target_theta*1000);
					mouse->log_data[mouse->log_index][1]=(int)(theta_rad*1000);
					mouse->log_index++;
				}
			}else if(flash_flag==true && trajectory->GetTragType()==stay){
				flash_flag=false;
				mouse->buzzer->On_ms(500,500);
				int param_data[param_data_num]={1,FlashGetGoalX(),FlashGetGoalY(),0,0,0,0,0};
				FlashSetData(mouse->maze_solver->adachi.map,param_data);
				FlashUpdateData();

			}else{
				sum_theta=0;
				theta_rad=0;
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
	if(gyro[2]>1000 || gyro[2]<-1000){
		next_mode=modeSelect_mode;

		mouse->motors->SetVoltageR(V_r);
		mouse->motors->SetVoltageL(V_l);

	}

	if(end_serch_flag){
		next_mode=modeSelect_mode;
	}
}

/////////////////////////////
/////////////////////////////

/////////////////////////////

/////////////////////////////
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

///////////////////////////
void LogOutput::Loop(){

	printf("%d,%d,%d,%d\r\n",mouse->log_data[index][0],mouse->log_data[index][1],mouse->log_data[index][2],mouse->log_data[index][3]);
	index++;
	if(index > mouse->log_data_num)next_mode=modeSelect_mode;
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
	//	FlashPrintMazeData(mouse->maze_solver->adachi.map);

};
void ResetMap::Interrupt_1ms(){

	next_mode=modeSelect_mode;
};


