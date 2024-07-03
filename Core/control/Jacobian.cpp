#include "Jacobian.hpp"
#include "machine_paramater.h"

void Jacobian(float v, float omega, float* vr, float* vl){
	*vr=v+TREAD_WIDTH/2.0*omega;
	*vl=v-TREAD_WIDTH/2.0*omega;
}
