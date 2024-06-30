/*
 * IMU.cpp
 *
 *  Created on: Jun 23, 2024
 *      Author: kyoro
 */

#include "IMU.hpp"
//#include "delay.h"

//MISO PB14
//MOSI PB15
//CLK  PB13
//CS   PB12

IMU::IMU(){
	calibration_flag=false;
	calibration_finish_flag=false;
	cal_num=0;
}

void IMU::Init(){
	uint16_t who_am_i=Read(WHO_AM_I_ADDR);
	Write(0x4E,0x0f);//Gyro mode LN ACCEL mode LN
	HAL_Delay(10);
	uint8_t gyro_config0=Read(0x4f);

	Write(0x4F,(gyro_config0 & 0b000111) | 0b000000);//Gyro FS +-2000d@s
	HAL_Delay(10);

	//Write(0x16, 0b01000000);
}
bool IMU::Calibration(){
	if(calibration_flag==false){
		calibration_flag=true;
		calibration_finish_flag=false;
		gyro_data_offset[2]=0;
		cal_num=0;
	}
	const int offset_num=512;

//	unsigned long _imu=Read16bit(GYRO_XOUT_H_ADDR+2*2);
//	gyro_data_offset[2]+=(_imu>>8) & 0xffff;
	if(cal_num>offset_num){
		gyro_data_offset[2]=gyro_data_offset[2]/cal_num;
		calibration_finish_flag=true;
		calibration_flag=false;
	}
	return calibration_finish_flag;
}

unsigned short IMU::Read16bit(unsigned char  addr){
	uint8_t tx_data[2]={addr|0b10000000,0};
	uint8_t rx_data[2]={0,0};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi, tx_data, rx_data,2,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//	HAL_SPI_Transmit(&hspi3, tx_data, 1,100);
//	HAL_SPI_Receive(&hspi3, rx_data, 1,100);
	return rx_data[1];
}

uint8_t IMU::Read(uint8_t addr){
	uint8_t tx_data[2]={addr|0b10000000,0};
	uint8_t rx_data[2]={0,0};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi, tx_data, rx_data,2,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//	HAL_SPI_Transmit(&hspi3, tx_data, 1,100);
//	HAL_SPI_Receive(&hspi3, rx_data, 1,100);
	return rx_data[1];
}

void IMU::Read(uint8_t addr, uint8_t* data){
	uint8_t a=(addr|0b10000000);
	uint8_t tx_data[7]={};
	tx_data[0]=a;
//	uint8_t tx_data[7]={0b10000000,0,0,0,0,0,0};
//	uint8_t tx_data[7]={1                ,2,3,4,5,6,7};
	//uint8_t rx_data[2]={0,0};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, tx_data,1,100);
	HAL_SPI_Receive(spi, data,6,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

}

void IMU::Write(uint8_t addr, uint8_t data){
	uint8_t tx_data[2]={addr,data};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, tx_data,2,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);}

void IMU::Update(){
	uint16_t who_am_i=Read(WHO_AM_I_ADDR);

	uint16_t _imu_h=0;
	uint16_t _imu_l=0;
	uint16_t _imu=0;
	//_imu=Read16bit(GYRO_XOUT_H_ADDR+2*2);
	//_imu_h=Read(GYRO_XOUT_H_ADDR+4|0b10000000);
	//_imu_l=Read(GYRO_XOUT_H_ADDR+5|0b10000000);
	//_imu=_imu_h<<8|_imu_l;
	//uint8_t ddata[3];
	Read((uint8_t)GYRO_XOUT_H_ADDR,gyro_data_8bit);
	gyro_data[0]=(int16_t) ((uint16_t)gyro_data_8bit[0]<<8 | (uint16_t)gyro_data_8bit[1]);
	gyro_data[1]=(int16_t) ((uint16_t)gyro_data_8bit[2]<<8 | (uint16_t)gyro_data_8bit[3]);
	gyro_data[2]=(int16_t) ((uint16_t)gyro_data_8bit[4]<<8 | (uint16_t)gyro_data_8bit[5]);

	Read(ACCEL_XOUT_H_ADDR,acc_data_8bit);
	acc_data[0]=(int16_t) ((uint16_t)acc_data_8bit[0]<<8 | (uint16_t)acc_data_8bit[1]);
	acc_data[1]=(int16_t) ((uint16_t)acc_data_8bit[2]<<8 | (uint16_t)acc_data_8bit[3]);
	acc_data[2]=(int16_t) ((uint16_t)acc_data_8bit[4]<<8 | (uint16_t)acc_data_8bit[5]);



	if(calibration_flag){
		gyro_data_offset[2]+=gyro_data[2];
		gyro_data[2]=-1;
		cal_num++;
	}else{
		//gyro_data[2]=(signed short)(_imu);
		gyro_data[2]-=gyro_data_offset[2];
	}

}
void IMU::GetGyroRaw(int * gyro){
	gyro[0]=gyro_data[0];
	gyro[1]=gyro_data[1];
	gyro[2]=gyro_data[2];
}
void IMU::GetAccRaw(short * acc){
	acc[0]=acc_data[0];
	acc[1]=acc_data[1];
	acc[2]=acc_data[2];
}
void IMU::GetGyro(float * gyro){
	gyro[0]=(float)gyro_data[0]/32768.0*GYRO_FS;
	gyro[1]=(float)gyro_data[1]/32768.0*GYRO_FS;
	gyro[2]=(float)gyro_data[2]/32768.0*GYRO_FS;
}
void IMU::GetAcc(float * acc){
	acc[0]=(float)acc_data[0]/acc_sensitivity;
	acc[1]=(float)acc_data[1]/acc_sensitivity;
	acc[2]=(float)acc_data[2]/acc_sensitivity;
}
