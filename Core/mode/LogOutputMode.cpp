#include "LogOutputMode.hpp"

void LogOutput::Loop(){

	printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
			mouse->log_data[index][0],
			mouse->log_data[index][1],
			mouse->log_data[index][2],
			mouse->log_data[index][3],
			mouse->log_data[index][4],
			mouse->log_data[index][5],
			mouse->log_data[index][6],
			mouse->log_data[index][7]);
	index++;
	if(index >= mouse->log_data_num)next_mode=modeSelect_mode;
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
