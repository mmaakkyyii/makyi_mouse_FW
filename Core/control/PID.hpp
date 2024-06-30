#ifndef _PID_HPP_
#define _PID_HPP_

class PID_Controler{
public:
	float kp;
	float ki;
	float kd;
	float period_ms;
	float old_error;	
	float sum;
	float r;

	PID_Controler(float _kp,float _ki,float _kd,float _period_ms);
	void SetTarget(float _r);
	float Update(float y);
	void Reset();
	void SetKp(float _kp);
	void SetKi(float _ki);
	void SetKd(float _kd);
	void SetPID(float _kp,float _ki,float _kd);


private:

};


#endif //_PID_HPP_