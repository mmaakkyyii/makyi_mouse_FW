/*
 * c_wrapper.hpp
 *
 *  Created on: Jun 23, 2024
 *      Author: kyoro
 */

#ifndef INC_C_WRAPPER_H_
#define INC_C_WRAPPER_H_

#if __cplusplus
extern "C"{
#endif
void BuzzerOn();
void BuzzerOff();
void SetFrequency(int f);
void Init();

void Loop();

void Interrupt125us();
void Interrupt1ms();
void Interrupt10ms();

#if __cplusplus
};
#endif



#endif /* INC_C_WRAPPER_H_ */
