/*
 * i2cc.h
 *
 *  Created on: Oct 27, 2023
 *      Author: Meow
 */

#ifndef INC_I2CC_H_
#define INC_I2CC_H_


uint16_t sendAndRead(uint8_t deAdd, uint8_t whoReg, uint8_t size);
void sendData(uint8_t deAdd, uint8_t whoReg, uint8_t data, uint8_t size);
void I2C_SendData(uint8_t slave_address, uint8_t register_address, uint8_t data, uint8_t dataSize);
uint32_t readRegisterMeow(uint32_t* buffer, uint8_t slave, uint8_t Reg, uint16_t nBuffSize);
uint32_t writeRegisterMroow(uint8_t slave,uint8_t Reg, uint8_t pBuff, uint16_t nBuffSize);
void Activate_I2C1_IT(void);
void I2C1_SoftwareReset(void);

#endif /* INC_I2CC_H_ */
