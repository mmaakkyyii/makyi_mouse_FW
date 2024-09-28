#include "ModeSelectMode.hpp"


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
	encL_deg=0;
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

	encL_deg += mouse->encorders->GetPulseL()/16384.0*360;

	if(encL_deg>90){
		encL_deg=0;
		mode_val++;

		if(mode_val<=15){
			mouse->buzzer->On_ms(400,20);
		}else{
			mode_val=0;
				mouse->buzzer->On_ms(300,20);
		}
		printf("ModeSelect%d\n\r",mode_val);
	}else if(encL_deg<-90){
		encL_deg=0;
		mode_val--;
		if(mode_val>=0){
			mouse->buzzer->On_ms(500,20);
		}else{
			mode_val=15;
			mouse->buzzer->On_ms(300,20);
		}
		printf("ModeSelect%d\n\r",mode_val);
	}

	if(sw1<pre_sw1){

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
			next_mode=digoRun_mode;
			mouse->buzzer->On_ms(800,50);
			break;
		case 3:
			next_mode=debug_mode;
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
		case 14:
			next_mode=parameterSetting_mode;
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


