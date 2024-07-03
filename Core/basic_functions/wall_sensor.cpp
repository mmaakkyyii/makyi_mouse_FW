#include "wall_sensor.hpp"
#include "gpio.h"
#include "adc.h"

WallSensor::WallSensor():
right(0),
left(0),
frontR(0),
frontL(0),
pre_right(0),
pre_left(0),
pre_frontR(0),
pre_frontL(0),
error_R(0),
error_L(0),
is_wallR(false),
is_wallL(false),
is_wallFR(false),
is_wallFL(false),
is_controlR(false),
is_controlL(false)
{
	
}
void WallSensor::Init(){
}

int WallSensor::GetError(){
	int error=0;
	if(is_controlR && is_controlL)error=GetErrorR()-GetErrorL();
	if(!is_controlR && is_controlL)error=2*(-GetErrorL());
	if(is_controlR && !is_controlL)error=2* (GetErrorR());
	if(!is_controlR && !is_controlL)error=0;
	return error;
}

uint16_t ReadADC(int ch){

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Rank = ADC_REGULAR_RANK_1;

	switch(ch){
	case 1:
		sConfig.Channel = ADC_CHANNEL_0;
		//sConfig.Rank = ADC_REGULAR_RANK_1;
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_11;
		//sConfig.Rank = ADC_REGULAR_RANK_2;
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_13;
		//sConfig.Rank = ADC_REGULAR_RANK_3;
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_14;
		//sConfig.Rank = ADC_REGULAR_RANK_4;
		break;
	}
	sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) !=HAL_OK){
		Error_Handler();
	}
	if(HAL_ADC_Start(&hadc1) == HAL_OK){
		if (HAL_ADC_PollForEvent(&hadc1,ADC_EOSMP_EVENT, 200) == HAL_OK){
			//while(HAL_ADC_GetState(&hadc1)==HAL_ADC_STATE_BUSY){
			//}
			uint16_t data=HAL_ADC_GetValue(&hadc1);
			//HAL_ADC_Stop(&hadc1);
			return data;
		}

	}

	return -1;
}

void WallSensor::Update(){
	static int state = 0;		//�ǂݍ��ރZ���T�̃��[�e�[�V�����Ǘ��p�ϐ�
	//int i;
	int loop_num=50;
	switch(state)
	{
		case 1:
			for(int i=0;i<loop_num;i++)HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
			right= ReadADC(2);
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
			pre_right = right;			//�ߋ��̒l��ۑ�
			if(right > TH_SEN_R)is_wallR = 1;
			else				is_wallR = 0;
			
			if(right > TH_CTRL_R){
				error_R = right - REF_SEN_R;	//�����������ꍇ�͕΍����v�Z
				is_controlR = 1;			//�E�Z���T�𐧌�Ɏg��
			}else{
				error_R = 0;					//����Ɏg��Ȃ��ꍇ�͕΍���0�ɂ��Ă���
				is_controlR = 0;			//�E�Z���T�𐧌�Ɏg��Ȃ�
			}			
			break;

		case 3:
			for(int i=0;i<loop_num;i++)HAL_GPIO_WritePin(LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_SET);
			frontL= ReadADC(4);//4
			HAL_GPIO_WritePin(LED_FL_GPIO_Port, LED_FL_Pin, GPIO_PIN_RESET);
			pre_frontL = frontL;			//�ߋ��̒l��ۑ�
			if(frontL > TH_SEN_FL)is_wallFL = 1;
			else				  is_wallFL = 0;
			break;

		case 0:
			for(int i=0;i<loop_num;i++)HAL_GPIO_WritePin(LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_SET);
			frontR= ReadADC(1);//
			HAL_GPIO_WritePin(LED_FR_GPIO_Port, LED_FR_Pin, GPIO_PIN_RESET);
			pre_frontR = frontR;
			if(frontR > TH_SEN_FR)is_wallFR = 1;
			else				  is_wallFR = 0;
			break;

		case 2:
			for(int i=0;i<loop_num;i++)HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_SET);
			left= ReadADC(3);
			HAL_GPIO_WritePin(LED_L_GPIO_Port, LED_L_Pin, GPIO_PIN_RESET);
			pre_left = left;			//�ߋ��̒l��ۑ�
			if(left > TH_SEN_L)is_wallL = 1;
			else			   is_wallL = 0;
			
			if(left > TH_CTRL_L){
				error_L = left - REF_SEN_L;	//�����������ꍇ�͕΍����v�Z����
				is_controlL = 1;			//���Z���T�𐧌�Ɏg��
			}else{
				error_L = 0;					//����Ɏg��Ȃ��ꍇ�͕΍���0�ɂ��Ă���
				is_controlL = 0;			//���Z���T�𐧌�Ɏg��Ȃ�
			}
			break;
	}
	
	state++;		//�S�񂲂ƂɌJ��Ԃ�
	if(state > 3)
	{
		state = 0;
	}
		

}
int WallSensor::GetRight(){
	return right;
}
int WallSensor::GetLeft(){
	return left;
}
int WallSensor::GetFrontR(){
	return frontR;
}
int WallSensor::GetFrontL(){
	return frontL;
}
