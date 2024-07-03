#ifndef _TRAJECTORY_HPP_
#define _TRAJECTORY_HPP_

typedef enum {
	none=-1,
	stop=0,
	line=1,
	rotate=2,
	stay=3,
	slalom=4,
	clothoid=5,
	constant_voltage=6
	
}TrajType;

struct clothoid_params{
	float t1;
	float t2;
	float t3;
	float omega;
	float v;
	float in_mm;
	float out_mm;
};

class Trajectory{
public:
	Trajectory();
	virtual ~Trajectory(){}
	virtual int Update(){return 0;};
	void GetTargetPosition(float* _x,float* _y, float* _theta);
	void GetTargetVelocity(float* _vx,float* _vy, float* _omega);
	void GetTargetAcceleration(float* _ax,float* _ay, float* _atheta);
	TrajType GetTragType(){return traj_type;}
protected:
	float period_s;
	float x,y,theta;
	float vx,vy,omega;
	float v0,vmax,vf;
	float ax,ay,atheta;
	TrajType traj_type;
};

class DoubleTrajectory:public Trajectory{
public:
	int Update();
	DoubleTrajectory(Trajectory* _traj1, Trajectory* _traj2);
	virtual ~DoubleTrajectory(){}
private:
	Trajectory* traj1;
	Trajectory* traj2;
	
	int phase;
};

class MultTrajectory:public Trajectory{
public:
	int Update();
	MultTrajectory(Trajectory* _traj1, Trajectory* _traj2, Trajectory* _traj3);
	virtual ~MultTrajectory(){}
private:
	Trajectory* traj1;
	Trajectory* traj2;
	Trajectory* traj3;
	
	int phase;
};

class Slalom:public Trajectory{
public:
	int Update();
	Slalom(float _theta_deg,float radius,float _v0, float _vmax, float _vf, float _a);
	virtual ~Slalom(){}
private:
	float t_s; //[s]
	float t1;
	int dir;
	float r;
};


class Rotate:public Trajectory{
public:
	int Update();
	//Rotate(float _theta_deg,float _v0, float _vmax, float _vf, float _a);
	Rotate(float _theta_deg, float _omega_max, float _a_omega);
	virtual ~Rotate(){}
private:
	float t_s; //[s]
	float t1;
	float t2;
	float t3;
	float a_omega;
	float omega_max;
	int dir;
};
class Line:public Trajectory{
public:
	int Update();
	Line(float _x, float _y, float _theta, float _v0, float _vmax, float _vf, float _a, float _j);
	virtual ~Line(){}
private:
	float target_x,target_y,target_theta;//[mm]
	float l; //[mm]
	//float vmax_x,vmax_y,omega_max; //[mm/s]
	float a,j; //[mm/s/s]
	float t_s; //[s]
	float t1,t2,t3; //[ms]
	
	float v; //[mm/s]
	float pos; //[mm]

};
class ConstantVoltage:public Trajectory{
public:
	int Update();
	ConstantVoltage(float Vr, float Vl, float time_ms);
	virtual ~ConstantVoltage(){}
private:
	float t_s; //[s]
	float t1_s; //[s]
};

class Stop:public Trajectory{
public:
	int Update();
	Stop();
	virtual ~Stop(){}
private:
};

class Stay:public Trajectory{
public:
	int Update();
	Stay(int t_ms);
	virtual ~Stay(){}
private:
	float t_s; //[s]
	float t1;

};

class Clothoid:public Trajectory{
public:
	int Update();
	Clothoid(clothoid_params params,int _cw_ccw);
	virtual ~Clothoid(){}
private:
	float t_s; //[s]
	int cw_ccw;
	float t1,t2,t3;
	float v;
	float omega_max;

};


#endif //_TRAJECTORY_HPP_
