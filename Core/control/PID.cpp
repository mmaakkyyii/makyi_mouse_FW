#include "PID.hpp"

PID_Controler::PID_Controler(float _kp,float _ki,float _kd,float _period_ms):kp(_kp),ki(_ki),kd(_kd),period_ms(_period_ms),sum(0),old_error(0),r(0)
{
/*
	kp=_kp;
	ki=_kp;
	kd=_kd;
//*/
}

void PID_Controler::SetTarget(float _r){
	r=_r;
}
void PID_Controler::Reset(){
	old_error=0;
	sum=0;
	r=0;
}
float PID_Controler::Update(float y){
	float e=r-y;
	sum+=e;
	old_error=e;
	//return e;
	return kp*e + ki*sum + kd*(e-old_error);//period_ms;
}

void PID_Controler::SetKp(float _kp){
	kp=_kp;
}
void PID_Controler::SetKi(float _ki){
	ki=_ki;
}
void PID_Controler::SetKd(float _kd){
	kd=_kd;
}
void PID_Controler::SetPID(float _kp,float _ki,float _kd){
	SetKp(_kp);
	SetKi(_ki);
	SetKd(_kd);
}
