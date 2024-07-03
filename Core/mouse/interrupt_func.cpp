#include "interrupt_func.h"
#include "mouse.hpp"
#include "MachineMode.hpp"

extern Mouse mouse;
extern MachineMode* mode; 
void int_1ms(){
	mouse.Interrupt_1ms();
	mode->CheckBattery();	
	mode->Interrupt_1ms();

	if(mode->GetCurrentMode()!=mode->GetNextMode()){
		switch(mode->GetNextMode()){
		case idle_mode:
			delete mode;
			mode = new Idle(&mouse);
			mode->Init();
			break;
		case lowBattery_mode:
			delete mode;
			mode = new LowBattery(&mouse);
			mode->Init();
			break;
		case modeSelect_mode:
			delete mode;
			mode = new ModeSelect(&mouse);
			mode->Init();
			break;
		case serchRun_mode:
			delete mode;
			mode = new SerchRun(&mouse);
			mode->Init();
			break;
		case fastRun_mode:
			delete mode;
			mode = new FastRun(&mouse);
			mode->Init();
			break;
		case parameterSetting_mode:
			delete mode;
			mode = new ParameterSetting(&mouse);
			mode->Init();
			break;
		case sensorCheck_mode:
			delete mode;
			mode = new SensorCheck(&mouse);
			mode->Init();
			break;
		case debug_mode:
			delete mode;
			mode = new Debug(&mouse);
			mode->Init();
			break;
		case doNotRotate_mode:
			delete mode;
			mode = new DoNotRotate(&mouse);
			mode->Init();
			break;
		case logOutput_mode:
			delete mode;
			mode = new LogOutput(&mouse);
			mode->Init();
			break;
		case reset_map:
			delete mode;
			mode = new ResetMap(&mouse);
			mode->Init();
			break;
		default:
			break;
		}
		
	}

}

void int_125ums(){
	mouse.Interrupt_125us();
}

void int_10ms(){
	mouse.Interrupt_10ms();
	mode->Loop();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi==&hspi1){
		int_spi1();
	}else if(hspi==&hspi2){
		int_spi2();
	}
}

void int_spi1(){
	mouse.encorders->InterruptR();
}
void int_spi2(){
	mouse.encorders->InterruptL();
}
