#include "LowBatteryMode.hpp"

LowBattery::LowBattery(Mouse* _mouse):MachineMode(_mouse){
	current_mode=lowBattery_mode;
	next_mode=lowBattery_mode;
};
void LowBattery::Loop(){
	printf("Battery Voltage is Low! %d\n\r",(int)(1000*mouse->battery_check->GetBatteryVoltage_V()));
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);
//	if(mouse->ui->GetSW2()==0 && mouse->ui->GetSW3()==0){
//		mouse->buzzer->On_ms(300,100);
//		next_mode=modeSelect_mode;
//	}


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
