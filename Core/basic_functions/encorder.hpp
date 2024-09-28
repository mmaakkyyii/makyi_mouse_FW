#ifndef _ENCORDER_H_
#define _ENCORDER_H_
#include "machine_paramater.h"
#include "spi.h"

class Encorders{
private:
	const float PPR=16384;//14 bit;
	const float gear_ratio=1;//35.0/9.0;
	const float radius_mm;
	float pluse2mm;

	float period_ms;

	int pulseR;
	int pulseL;
	float velocityR;
	float velocityL;
	const int dirR=1;
	const int dirL=-1;

	uint16_t angle_dataL;
	uint16_t angle_dataR;
	uint16_t pre_angle_dataL;
	uint16_t pre_angle_dataR;

	uint16_t tx_dataL[2]={0,0};
	uint16_t tx_dataR[2]={0,0};


	void InitEncorderL();//SPI2
	void InitEncorderR();//SPI1


	float GetRPSL();
	float GetRPSR();

public:
	const uint8_t Z_ADDR_L=0x0;
	const uint8_t Z_ADDR_H=0x1;
	const uint8_t BCT_ADDR=0x2;
	Encorders(int _period_ms);
	void Init();
	void Update();
	float GetVelociryL_mm_s();
	float GetVelociryR_mm_s();

	int GetAngleL();
	int GetAngleR();

	int GetPulseL();
	int GetPulseR();

	void InterruptL();
	void InterruptR();

	void SetDataL(uint8_t addr,uint8_t data);
	void SetDataR(uint8_t addr,uint8_t data);
	uint8_t ReadDataL(uint8_t addr);
	uint8_t ReadDataR(uint8_t addr);

};

#endif //_ENCORDER_H_
