/*
 * tempSensor.c
 *
 *  Created on: Oct 28, 2023
 *      Author: Meow
 */


#include "main.h"
#include "tempSensor.h"
#include "i2cm.h"
#include <stdio.h>


static int32_t (* functionFrameRead)(uint8_t deAdd, uint8_t whoReg, uint8_t size) = 0;
static int32_t (* functionFrameWrite)(uint8_t slave,uint8_t Reg, uint8_t pBuff, uint16_t nBuffSize) = 0;

uint16_t T0DegC8 = 0;
uint16_t T1DegC8 = 0;
uint16_t T1T0msb = 0;
uint16_t T0OUT = 0;
uint16_t T1OUT = 0;
uint8_t lowByte = 0;
uint8_t highByte = 0;
uint16_t H0rhx2 = 0;
uint16_t H1rhx2 = 0;
uint16_t H0T0OUT = 0;
uint16_t H1T0OUT = 0;

void callBackFunction(int32_t *callbackRead,int32_t *callbackWrite)
{
	if(callbackRead != 0 && callbackWrite != 0)
	{
		functionFrameRead = callbackRead;
		functionFrameWrite = callbackWrite;
	}
}


void htsInit(){
	if(functionFrameRead == 0 && functionFrameWrite == 0)
		return;

	htsWHO();

	uint8_t resolution = AV_CONF;
	uint8_t CTRLREG1 = CTRL_REG1;
	uint8_t CTRLREG2 = CTRL_REG2;

	resolution |= (1<<1);
	resolution |= (1<<4);
	//1B
	functionFrameWrite(HTSadd,AV_CONF,0x36U,1);
	LL_mDelay(10);

	CTRLREG1  |= (1 << 7);
	CTRLREG1  |= (1 << 1);
	CTRLREG1  &= ~(1 << 2);
	//0x81U 1 Hz
	//0x82 7Hz
	functionFrameWrite(HTSadd,CTRL_REG1,0x83U,1);
	LL_mDelay(10);



	CTRLREG2  &= ~(1 << 7);
	CTRLREG2  &= ~(1 << 0);
	functionFrameWrite(HTSadd,CTRL_REG2,0x00U,1);
	LL_mDelay(10);



	T0DegC8 = functionFrameRead(HTSadd,T0_degC_x8,1);
	LL_mDelay(10);
	T1DegC8 = functionFrameRead(HTSadd,T1_degC_x8,1);
	LL_mDelay(10);
	T1T0msb = functionFrameRead(HTSadd,T1_T0msb,1);
	LL_mDelay(10);
	lowByte = functionFrameRead(HTSadd,T0_OUT,1);
	LL_mDelay(10);
	highByte += functionFrameRead(HTSadd,T0_OUT_,1);
	LL_mDelay(10);
	T0OUT = ((uint16_t)highByte << 8) | lowByte;
	LL_mDelay(10);
	lowByte =  functionFrameRead(HTSadd,T1_OUT,1);
	LL_mDelay(10);
	highByte =  functionFrameRead(HTSadd,T1_OUT_,1);
	LL_mDelay(10);
	T1OUT = ((uint16_t)highByte << 8) | lowByte;
	LL_mDelay(10);
	H0rhx2 = functionFrameRead(HTSadd,H0_rH_x2,1);
	LL_mDelay(10);
	H1rhx2 = functionFrameRead(HTSadd,H1_rH_x2,1);
	LL_mDelay(10);
	lowByte = functionFrameRead(HTSadd,H0_T0_OUT,1);
	LL_mDelay(10);
	highByte = functionFrameRead(HTSadd,H0_T0_OUT_,1);
	LL_mDelay(10);
	H0T0OUT = ((uint16_t)highByte << 8) | lowByte;
	LL_mDelay(10);
	lowByte = functionFrameRead(HTSadd,H1_T0_OUT,1);
	LL_mDelay(10);
	highByte = functionFrameRead(HTSadd,H1_T0_OUT_,1);
	LL_mDelay(10);
	H1T0OUT = ((uint16_t)highByte << 8) | lowByte;
	LL_mDelay(10);







}
uint8_t htsWHO(){

	if(functionFrameRead == 0 && functionFrameWrite == 0)
		return 0;

	uint8_t WHO =  functionFrameRead(HTSadd, HTSwhoAdd,1);
	if (WHO == HTSwhoVal)
		return 1;
	else
		return 0;

}


double readTemp(){

	if(functionFrameRead == 0 && functionFrameWrite == 0)
			return 0;

	//sendData(HTSadd,CTRL_REG2,0x01U,1);
	uint16_t output = 0;
	highByte = functionFrameRead(HTSadd, Tout_,1);
	//output = highByte <<8;
	lowByte = functionFrameRead(HTSadd, Tout,1);
	output = ((uint16_t)highByte << 8) | lowByte;

	double tempInCFinallyMeowIwasGoingInsaneMeooowMrooow = interpolTemp(output);
	return tempInCFinallyMeowIwasGoingInsaneMeooowMrooow;



}

double readHum(){

	if(functionFrameRead == 0 && functionFrameWrite == 0)
			return 0;

	uint16_t output = 0;
	highByte = functionFrameRead(HTSadd, Hout_,1);
	lowByte = functionFrameRead(HTSadd, Hout,1);
	output = ((uint16_t)highByte << 8) | lowByte;

	double tempInCFinallyMeowIwasGoingInsaneMeooowMrooow = interpolHum(output);
	return tempInCFinallyMeowIwasGoingInsaneMeooowMrooow;



}

double interpolTemp(uint16_t output){
	double T_DegC;
	double T0_degC = (T0DegC8 + (1 << 8) * (T1T0msb & 0x03)) / 8.0;
	double T1_degC = (T1DegC8 + (1 << 6) * (T1T0msb & 0x0C)) / 8.0; // Value is in 3rd and fourth bit, so we only need to shift this value 6 more bits.
	T_DegC =  (T0_degC + (output - T0OUT) * (T1_degC - T0_degC) / (T1OUT - T0OUT));
	return T_DegC;
}
double interpolHum(uint16_t output){

	double H0_rH = (float)H0rhx2/2.0;
	double H1_rH = (float)H1rhx2/2.0;
	//Calculate the relative Humidity using linear interpolation
	double Humidity_rH = ( float )(((( output - H0T0OUT ) * ( H1_rH - H0_rH )) / ( H1T0OUT - H0T0OUT )) + H0_rH );
	return Humidity_rH;
}

