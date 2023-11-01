/*
 * tempSensor.h
 *
 *  Created on: Oct 28, 2023
 *      Author: Meow
 */

#ifndef TEMPSENSOR_H_
#define TEMPSENSOR_H_




#define 	HTSadd						0xBEU
#define 	HTSwhoVal					0xBCU
#define 	HTSwhoAdd					0x0FU
#define 	T0_degC_x8					0x32U
#define 	T1_degC_x8					0x33U
#define 	T1_T0msb					0x35U
#define 	T0_OUT						0x3CU
#define 	T0_OUT_						0x3DU
#define 	T1_OUT						0x3EU
#define 	T1_OUT_						0x3FU
#define		AV_CONF						0x10U
#define		CTRL_REG1					0x20U
#define		CTRL_REG2 					0x21U
#define		CTRL_REG3 					0x22U
#define 	H0_rH_x2					0x30U
#define 	H1_rH_x2					0x31U
#define 	H0_T0_OUT					0x36U
#define 	H0_T0_OUT_					0x37U
#define 	H1_T0_OUT					0x3AU
#define 	H1_T0_OUT_					0x3BU
//Output
#define		Tout 						0x2AU
#define		Tout_ 						0x2BU
#define 	Hout 						0x28U
#define 	Hout_ 						0x29U

void callBackFunction(int32_t *callbackRead,int32_t *callbackWrite);
void htsInit();
uint8_t htsWHO();
double readTemp();
double interpolTemp(uint16_t output);
double interpolHum(uint16_t output);
double readHum();

#endif /* TEMPSENSOR_H_ */
