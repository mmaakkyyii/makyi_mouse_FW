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
	printf("%d,%d,%d,%d\r\n",(int)target_velocity_r,(int)target_velocity_l,(int)velocity_r,(int)velocity_l);

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
	t_ms++;
	printf("t=%d\r\n",t_ms);

	mouse->motors->SetVoltageR(0.25);
	mouse->motors->SetVoltageL(0.25);

	mouse->log_data[mouse->log_index][0]=(int)(mouse->encorders->GetVelociryL_mm_s() );
	mouse->log_data[mouse->log_index][1]=(int)(mouse->encorders->GetVelociryR_mm_s() );
	mouse->log_index++;

	if(t_ms>1000){
		printf("deleat\r\n");
		next_mode=modeSelect_mode;
	}

}
