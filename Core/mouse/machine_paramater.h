#ifndef _MACHINE_PARAMATER_H_
#define _MACHINE_PARAMATER_H_

const float SECTION_WIDTH=90.0;
const float TREAD_WIDTH=35.54;//TODO
const float TIYA_R=13.2/2.0;//TODO 13.05/2.0
const float MACHINE_BACK_LENGTH=90/2.0-29-5;//-29
const float MACHINE_BACK_VOLTAGE_R=0.55;//TODO
const float MACHINE_BACK_VOLTAGE_L=0.55;//TODO
const float MACHINE_BACK_TIME=200;//TODO



const float CONTROL_PERIOD_ms=1.0;

const float Kp_motorR=0.006;//0.001
const float Ki_motorR=0.0001;//8;//0.00005
const float Kd_motorR=0.000;

const float Kp_motorL=0.006;
const float Ki_motorL=0.0001;//8;
const float Kd_motorL=0.000;

const float Kp_wall=0.001;//0.002
const float Kd_wall=0.0000;//0.00003

#endif //_MACHINE_PARAMATER_H_
