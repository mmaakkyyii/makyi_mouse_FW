#ifndef _LOCALIZATION_HPP_
#define _LOCALIZATION_HPP_
#include "encorder.hpp"

class Localization{
public:
	Localization(float _x, float _y, float _theta, float _period_ms, Encorders* _encorders);
	void Update();
	void GetPosition(float* _x, float* _y, float* _theta);
	float GetTheta();
private:
	Encorders* encorders;
	float period_ms;
	float x;
	float y;
	float theta;

};
#endif //_LOCALIZATION_HPP_
