#include "encorder.hpp"

Encorders::Encorders(int _period_ms)
:
radius_mm(TIYA_R),
period_ms(_period_ms),
pulseR(0),
pulseL(0),
velocityR(0),
velocityL(0),
angle_dataL(0),
angle_dataR(0),
pre_angle_dataL(0),
pre_angle_dataR(0)
{
	pluse2mm =  1/(PPR)*3.14*gear_ratio*radius_mm;

}


float fc=30;
float tau=1/(2*3.14*fc);
float alpha=CONTROL_PERIOD_ms/1000.0/(tau+CONTROL_PERIOD_ms/1000.0);


void Encorders::Init(){
	InitEncorderL();
	InitEncorderR();
	pulseR=0;
	pulseL=0;
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_dataL, (uint8_t *)&angle_dataL, 1,100);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)tx_dataR, (uint8_t *)&angle_dataR, 1,100);
	angle_dataL=angle_dataL>>2;
	angle_dataR=angle_dataR>>2;
	pre_angle_dataL=angle_dataL;
	pre_angle_dataR=angle_dataR;

}
void Encorders::InitEncorderL(){

}
void Encorders::InitEncorderR(){
}

void Encorders::InterruptL(){
	angle_dataL=angle_dataL>>2;
	pulseL=angle_dataL-pre_angle_dataL;
	if(pulseL>8192)pulseL-=16384;
	if(pulseL<-8192)pulseL+=16384;
	pulseL*=dirL;

	static float pre_velL;
	float input=(float)pulseL/PPR*2*3.14*gear_ratio*radius_mm/(period_ms*0.001);
	velocityL=alpha*input+(1-alpha)*pre_velL;
	pre_velL=velocityL;

}
void Encorders::InterruptR(){
	angle_dataR=angle_dataR>>2;
	pulseR=angle_dataR-pre_angle_dataR;
	if(pulseR>8192)pulseR-=16384;
	if(pulseR<-8192)pulseR+=16384;
	pulseR*=dirR;

	static float pre_velR;
	float input=(float)pulseR/PPR*2*3.14*gear_ratio*radius_mm/(period_ms*0.001);
	velocityR=alpha*input+(1-alpha)*pre_velR;
	pre_velR=velocityR;
}

void Encorders::Update(){
	pre_angle_dataL=angle_dataL;
	pre_angle_dataR=angle_dataR;

	tx_dataL[0]=0;
	tx_dataR[0]=0;
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_dataL, (uint8_t *)&angle_dataL, 1,100);
	InterruptL();
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)tx_dataR, (uint8_t *)&angle_dataR, 1,100);
	InterruptR();
}

int Encorders::GetAngleL(){
	return angle_dataL;
}
int Encorders::GetAngleR(){
	return angle_dataR;
}


int Encorders::GetPulseL(){
	return pulseL;
}
int Encorders::GetPulseR(){
	return pulseR;
}

float Encorders::GetRPSL(){
	return pulseL/period_ms;
}
float Encorders::GetRPSR(){
	return pulseR/period_ms;
}


float Encorders::GetVelociryL_mm_s(){
	return velocityL;
}
float Encorders::GetVelociryR_mm_s(){
	return velocityR;
}
