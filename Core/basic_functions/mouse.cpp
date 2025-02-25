#include "mouse.hpp"
#include "debug.hpp"
#include "init_mpu.hpp"
#include "Jacobian.hpp"
#include "delay.h"

#include "usart.h"

#include "flash_util.hpp"


Mouse::Mouse(PID_Controler* motorR, PID_Controler* motorL, Motors* _motors, Localization* _localization, Encorders* _encorders,IMU* _imu, WallSensor* _wall_sensor, BatteryCheck* _battery_check, Buzzer* _buzzer,UI* _ui,MazeSolver* _maze_solver)
:motorR_PID(motorR),
 motorL_PID(motorL),
 motors(_motors),
 localization(_localization),
 encorders(_encorders),
 imu(_imu),
 wall_sensor(_wall_sensor),
 battery_check(_battery_check),
 buzzer(_buzzer),
 ui(_ui),
 maze_solver(_maze_solver),
 runing_buzzer(true)
 {
	//mode=new ModeSelect();
	period_ms=CONTROL_PERIOD_ms;
	
	v_max=350;
	turn_v_max=200;
	goal_time=0;
	mouse_pos_x=0;
	mouse_pos_y=0;
	mouse_dir=North;
	goal_pos_x=GOAL_X;
	goal_pos_y=GOAL_Y;
	wall_mask=USE_UNKOWN_WALL_MASK;

}

void Mouse::Init(){
	init_mpu();
	
	ui->Init();
	wall_sensor->Init();
	battery_check->Init();
	buzzer->Init();
	motors->Init();
	encorders->Init();
	imu->Init();
	
//	encorders->SetDataL(encorders->BCT_ADDR, 0);
//	encorders->SetDataL(4, 0b11000000);
//	encorders->SetDataL(6, 0b00011100);
//	encorders->SetDataR(4, 0b11000000);
//	encorders->SetDataR(6, 0b00011100);

//	encorders->SetDataL(0x3, 0b0);
	battery_check->Update();

	printf("READ L\r\n");
	HAL_Delay(50);
	printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
			encorders->ReadDataL(0),encorders->ReadDataL(1),encorders->ReadDataL(2),encorders->ReadDataL(3),encorders->ReadDataL(4),encorders->ReadDataL(5),encorders->ReadDataL(6),encorders->ReadDataL(9),encorders->ReadDataL(0x1b));
	HAL_Delay(200);

	printf("READ R\r\n");
	HAL_Delay(50);
	printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
			encorders->ReadDataR(0),encorders->ReadDataR(1),encorders->ReadDataR(2),encorders->ReadDataR(3),encorders->ReadDataR(4),encorders->ReadDataR(5),encorders->ReadDataR(6),encorders->ReadDataR(9),encorders->ReadDataR(0x1b));
	HAL_Delay(200);

	printf("Start\r\n");


	int map_data[MAZESIZE_X][MAZESIZE_Y]={0};
	int param_data[param_data_num]={0};
	FlashGetData(map_data,param_data);

	goal_pos_x=FlashGetGoalX();
	goal_pos_y=FlashGetGoalY();
//	goal_pos_x=3;
//	goal_pos_y=3;

	maze_solver->Init();
	maze_solver->adachi.InitMaze(UNKNOWN, map_data);


	FlashPrintMazeData(map_data);
	mouse_pos_x=0;
	mouse_pos_y=0;
	mouse_dir=North;

	maze_solver->adachi.SetMap(mouse_pos_x,mouse_pos_y,map_data[mouse_pos_x][mouse_pos_y],mouse_dir);


	buzzer->SetFrequency(400);	//�l�������������̎��g����ݒ�
	buzzer->On();		//�u�U�[�𔭐U������


	for(int i=1;i<16;i=i*2){
		ui->SetLED(i);
		delay_ms(25);
	}
	buzzer->Off();		//�u�U�[�̔��U���~������	

	printf("Battery %d mV: goal(%d,%d)|\r\n",(int)(battery_check->GetBatteryVoltage_V()*1000),goal_pos_x,goal_pos_y);


	mouse_pos_y++;

	timer_start();

}
void Mouse::Interrupt_10ms(){
	//printf("%4d,%4d,%4d,%4d\r\n",wall_sensor->GetLeft(),wall_sensor->GetFrontL(),wall_sensor->GetFrontR(),wall_sensor->GetRight());
	//int data[3];
	//imu->GetGyroRaw(data);
//	printf("batt:%d\r\n",(int)(battery_check->GetBatteryVoltage_V()*1000));
//	printf("%5d,%5d,%5d\r\n",data[0],data[1],data[2]);
//	printf("%d,%d\r\n",(int)encorders->GetVelociryL_mm_s(),(int)encorders->GetVelociryR_mm_s());

}

void SetGoal(int x, int y){
	
}
int Mouse::GetWallInfo(){
	int FR=wall_sensor->GetWallFR();
	int FL=wall_sensor->GetWallFL();
	int R=wall_sensor->GetWallR();
	int L=wall_sensor->GetWallL();

	int wall_info=0;
	switch(mouse_dir){
		//NWSE
		//3210
		case North:
			wall_info=(L<<2) | (FR|FL)<<3 | R << 0;
			break;
		case West:
			wall_info=(L<<1) | (FR|FL)<<2 | R << 3;
			break;
		case South:
			wall_info=(L<<0) | (FR|FL)<<1 | R << 2;
			break;
		case East:
			wall_info=(L<<3) | (FR|FL)<<0 | R << 1;
			break;
	}
	
	return wall_info;

}


int count=0;
void Mouse::Interrupt_1ms(){
	encorders->Update();
	buzzer->Update();
	if(count>1){
		count=0;	
		imu->Update();
	}else{
		count++;
	}
	//localization->Update();
	//localization->GetPosition(&current_x,&current_y,&current_theta);

}

void Mouse::Interrupt_125us(){
	wall_sensor->Update();
	battery_check->Update();
	motors->SetSupplayVoltage(battery_check->GetBatteryVoltage_V());
	ui->Update();

}

void Mouse::Loop(){
//printf("%d\r\n",(int)(battery_check->GetBatteryVoltage_V()*1000));
}
