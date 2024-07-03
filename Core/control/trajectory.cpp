#include "trajectory.hpp"
#include "math.h"


#include "machine_paramater.h"

Trajectory::Trajectory():period_s(CONTROL_PERIOD_ms),x(0),y(0),theta(0),vx(0),vy(0),omega(0),v0(0),vmax(0),vf(0),ax(0),ay(0),atheta(0),traj_type(none)
{
}

void Trajectory::GetTargetPosition(float* _x,float* _y, float* _theta){
	*_x=x;
	*_y=y;
	*_theta=theta;
}
void Trajectory::GetTargetVelocity(float* _vx,float* _vy, float* _omega){
	*_vx=vx;
	*_vy=vy;
	*_omega=omega;
}
void Trajectory::GetTargetAcceleration(float* _ax,float* _ay, float* _atheta){
	*_ax=ax;
	*_ay=ay;
	*_atheta=atheta;
}

Stop::Stop():Trajectory(){
	traj_type=stop;
}
int Stop::Update(){
	x=0;
	y=0;
	theta=0;
	vx=0;
	vy=0;
	omega=0;

	return 1;
}

Stay::Stay(int t_ms):Trajectory(){
	traj_type=stay;
	t1=t_ms*0.001;
	period_s=CONTROL_PERIOD_ms/1000.0;
	t_s=0;
}
int Stay::Update(){
	int is_finish=0;
	x=0;
	y=0;
	theta=0;
	vx=0;
	vy=0;
	omega=0;

	t_s+=period_s;

	if(t_s>t1)is_finish=1;
	return is_finish;
}


DoubleTrajectory::DoubleTrajectory(Trajectory* _traj1, Trajectory* _traj2):Trajectory(){
	traj1=_traj1;
	traj2=_traj2;
	phase=1;
	traj_type=traj1->GetTragType();

}

int DoubleTrajectory::Update(){
	int is_finish=0;
	switch(phase){
	case 1:
		if(traj1->Update()){
			delete traj1;
			traj_type=traj2->GetTragType();
			phase++;
		}
		traj_type=traj1->GetTragType();
		traj1->GetTargetPosition(&x,&y,&theta);
		traj1->GetTargetVelocity(&vx,&vy,&omega);
		traj1->GetTargetAcceleration(&ax,&ay,&atheta);
		break;
	case 2:
		if(traj2->Update()){
			delete traj2;
			is_finish=1;
		}
		traj_type=traj2->GetTragType();
		traj2->GetTargetPosition(&x,&y,&theta);
		traj2->GetTargetVelocity(&vx,&vy,&omega);
		traj2->GetTargetAcceleration(&ax,&ay,&atheta);
		break;
	}
	return is_finish;
}


MultTrajectory::MultTrajectory(Trajectory* _traj1, Trajectory* _traj2, Trajectory* _traj3):Trajectory(){
	traj1=_traj1;
	traj2=_traj2;
	traj3=_traj3;
	phase=1;
	traj_type=traj1->GetTragType();
}


int MultTrajectory::Update(){
	int is_finish=0;
	switch(phase){
	case 1:
		if(traj1->Update()){
			delete traj1;
			traj_type=traj2->GetTragType();
			phase++;
		}
		traj_type=traj1->GetTragType();
		traj1->GetTargetPosition(&x,&y,&theta);
		traj1->GetTargetVelocity(&vx,&vy,&omega);
		traj1->GetTargetAcceleration(&ax,&ay,&atheta);
		break;
	case 2:
		if(traj2->Update()){
			delete traj2;
			traj_type=traj3->GetTragType();
			phase++;
		}
		traj_type=traj2->GetTragType();
		traj2->GetTargetPosition(&x,&y,&theta);
		traj2->GetTargetVelocity(&vx,&vy,&omega);
		traj2->GetTargetAcceleration(&ax,&ay,&atheta);
		break;
	case 3:
		if(traj3->Update()){
			delete traj3;
			is_finish=1;
		}
		traj_type=traj3->GetTragType();
		traj3->GetTargetPosition(&x,&y,&theta);
		traj3->GetTargetVelocity(&vx,&vy,&omega);
		traj3->GetTargetAcceleration(&ax,&ay,&atheta);
		break;
	}
	return is_finish;
}

Slalom::Slalom(float _theta_deg,float radius,float _v0, float _vmax, float _vf, float _a):Trajectory(),t_s(0)
{
	traj_type=slalom;
	
	period_s=CONTROL_PERIOD_ms/1000.0; //[s]
	r=radius;
	vmax=_vmax;
	if(_theta_deg<0){
		dir=-1;
		t1=- r*3.14* _theta_deg/180.0 /vmax;
	}else{
		dir=1;
		t1=  r*3.14* _theta_deg/180.0 /vmax;
	}
}
//t1=TREAD_WIDTH/2.0 * 3.14* _theta_deg/180.0 /vmax;
int Slalom::Update(){
	int is_finish=0;
	x=0;
	y=0;
	theta=0;
	vx=0;
	vy=vmax;
	omega=	dir*vmax/r;
	if(t_s>t1)is_finish=1;
	t_s+=period_s;

	return is_finish;
	
}



Rotate::Rotate(float _theta_deg,float _omega_max, float _a_omega):Trajectory(),t_s(0)
{
	traj_type=rotate;
	
	period_s=CONTROL_PERIOD_ms/1000.0; //[s]
	
	if(_theta_deg<0){
		dir=-1;
	}else{
		dir=1;
	}
	theta=0;
	a_omega=_a_omega;
	omega_max=_omega_max;
	t1=omega_max/a_omega;
	t2=dir*((_theta_deg)*3.14/180.0)/omega_max;
	t3=t1+t2;
}

int Rotate::Update(){
	int is_finish=0;
	x=0;
	y=0;
	vx=0;
	vy=0;
	if(t_s<t1){
		omega= dir*(a_omega*t_s);
	}else if(t_s<t2){
		omega=dir* omega_max;//dir* (a_omega*t1);
	}else if(t_s<t3){
		omega=dir* (a_omega*t1- a_omega*(t_s-t2) );
	}else{
		omega=0;
		is_finish=1;
	}
	theta+=omega*period_s;

	t_s+=period_s;

	return is_finish;
	
}

int ConstantVoltage::Update(){
	int is_finish=0;
	if(t_s>t1_s){
		is_finish=1;
	}
	
	t_s+=period_s;
	
	return is_finish;
}

ConstantVoltage::ConstantVoltage(float Vr, float Vl, float time_ms){
	traj_type=constant_voltage;
	x=0;
	y=0;
	theta=0;
	vx=Vr;
	vy=Vl;
	omega=0;
	ax=0;
	ay=0;
	atheta=0;
	period_s=CONTROL_PERIOD_ms/1000.0;
	t1_s=time_ms*0.001;
	t_s=0;
}

Line::Line(float _x, float _y, float _theta, float _v0, float _vmax, float _vf, float _a, float _j)
:Trajectory(),
target_x(_x),
target_y(_y),
target_theta(_theta),
a(_a),
j(_j),
v(_v0),
pos(0)
{
	traj_type=line;

	v0=_v0;
	vmax=_vmax;
	vf=_vf;
	period_s=CONTROL_PERIOD_ms/1000.0; //[s]
	t_s=0;

	l=sqrtf(target_x*target_x+target_y*target_y);

	if(l-(vmax*vmax-v0*v0)/(2*a)-(vmax*vmax-vf*vf)/(2*a) > 0){
		t1=(vmax-v0)/a; //[s]
		t2=l/vmax-(vmax*vmax-v0*v0)/(2*a*vmax)-(vmax*vmax-vf*vf)/(2*a*vmax)+t1;
		t3=(vmax-vf)/a+t2;
	}else{
		t1=-(2*v0 - sqrt(2*v0*v0 + 2*vf*vf + 4*l*a))/(2*a);
		//t1=(-v0+sqrt(v0*v0 - (v0*v0 -3*v0*vf+2*vf*vf)/2 + a*l ))/(a);
		t2=t1;
		t3=(v0+a*t1-vf)/a+t1;
	}
	/*
	if(l-(vmax*vmax+v0*v0)/(2*a)-(vmax*vmax+vf*vf)/(2*a)>0){
		t1=(vmax-v0)/a; //[s]
		t2=l/vmax-(vmax*vmax-v0*v0)/(2*a*vmax)-(vmax*vmax-vf*vf)/(2*a*vmax)+t1;
		t3=(vmax-vf)/a+t2;
	}else{
		//v0+a*t1-a*(t3-t1)=vf  //t3=-(vf-v0-a*t1)/a+t1
		//(v0+v0+a*t1)*t1/2+(v0+a*t1+vf)*(t3-t1)/2=l
		t1=(-2*a*v0+sqrtf(4*a*a*v0*v0 - 2*a*a*(v0*v0-vf*vf-2*a*l)))/(2*a*a);
		t3=(v0+a*t1-vf)/a+t1;		
		t2=t1;
	}
	*/
}

int Line::Update(){
	int is_finish=0;
	if(t_s<t1){
		v=a*t_s+v0;
		pos=1/2.0*a*t_s*t_s+v0*t_s;
	}else if(t_s<t2){
		v=a*t1+v0;
		pos=(a*t1+v0)*(t_s-t1)+1/2.0*a*t1*t1+v0*t1;
	}else if(t_s<t3){
		v=-a*(t_s-t2)+(a*t1+v0);
		pos=-1/2.0*a*(t_s-t2)*(t_s-t2)+(a*t1+v0)*(t_s-t2)+ (a*t1+v0)*(t2-t1)+1/2.0*a*t1*t1+v0*t1;
	}else{
		v=vf;
		pos=l;
		is_finish=1;
	}
	theta=0;
	omega=0;
	x=target_x/l*pos;
	y=target_y/l*pos;
	vx=target_x/l*v;
	vy=target_y/l*v;
	t_s+=period_s;
	return is_finish;
}

Clothoid::Clothoid(clothoid_params params,int _cw_ccw){
	traj_type=clothoid;
	
	period_s=CONTROL_PERIOD_ms/1000.0; //[s]
	cw_ccw=_cw_ccw;
	t1=params.t1;
	t2=params.t2;
	t3=params.t3;
	v=params.v;
	omega_max=params.omega;
	t_s=0;
}

int Clothoid::Update(){
	int is_finish=0;
	if(t_s<t1){
		omega=cw_ccw*omega_max/t1*t_s;

	}else if(t_s<t2){
		omega=cw_ccw*omega_max;

	}else if(t_s<t3){
		omega=cw_ccw * ( omega_max-omega_max/(t3-t2)*(t_s-t2) );
	}else{
		omega=0;
		is_finish=1;
	}
	theta+=omega*period_s;
	vx=0;
	vy=v;
	t_s+=period_s;
	return is_finish;
}
