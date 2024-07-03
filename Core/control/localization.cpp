#include "localization.hpp"
#include "math.h"
#include "machine_paramater.h"

Localization::Localization(float _x, float _y, float _theta, float _period_ms, Encorders* _encorders)
:x(_x),y(_y),theta(_theta),period_ms(_period_ms),encorders(_encorders){
	
}
void Localization::Update(){

	float dr=encorders->GetVelociryR_mm_s()*period_ms/1000.0;
	float dl=encorders->GetVelociryL_mm_s()*period_ms/1000.0;
	theta+=(dr-dl)/TREAD_WIDTH;
	x-=((dr+dl)/2.0)*sinf(theta);
	y+=((dr+dl)/2.0)*cosf(theta);
	
}
void Localization::GetPosition(float* _x, float* _y, float* _theta){
	*_x=x;
	*_y=y;
	*_theta=theta;
}
float Localization::GetTheta(){
	return theta;
}
