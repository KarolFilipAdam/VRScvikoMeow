/*
 * i2cm.c
 *
 *  Created on: Oct 27, 2023
 *      Author: Meow
 */
#include "main.h"
#include "stm32f3xx_it.h"
#include "tempSensor.h"

void Activate_I2C1_IT(void);
void I2C1_SoftwareReset(void);
uint16_t sendAndRead(uint8_t deAdd, uint8_t whoReg,uint8_t size);
uint32_t writeRegisterMroow(uint8_t slave, uint8_t Reg, uint8_t pBuff, uint16_t nBuffSize);
uint32_t readRegisterMeow(uint32_t* buffer, uint8_t slave, uint8_t Reg, uint16_t nBuffSize);

uint16_t sendAndRead(uint8_t deAdd, uint8_t whoReg,uint8_t size)
{
	uint16_t receivedData;
    LL_I2C_EnableIT_RX(I2C1);

    // Initialize I2C communication to read WHO_AM_I register
    LL_I2C_HandleTransfer(I2C1, deAdd, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    // Send the WHO_AM_I register address
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
        if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {
            LL_I2C_TransmitData8(I2C1, whoReg);
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    // Receive WHO_AM_I value
    LL_I2C_HandleTransfer(I2C1, deAdd, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
    	receivedData = LL_I2C_ReceiveData8(I2C1);
    	LL_mDelay(10);
		//LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
	    //LL_I2C_ClearFlag_ADDR(I2C1);
    	//LL_mDelay(100);
    }

	LL_I2C_DisableIT_RX(I2C1);
	LL_I2C_ClearFlag_STOP(I2C1);
	LL_I2C_ClearFlag_NACK(I2C1);
	//receivedData = getData();
	//clearData();
	return receivedData;
}


void sendData(uint8_t deAdd, uint8_t whoReg, uint8_t data, uint8_t size) {


    // Send the register address and data to the sensor
    LL_I2C_HandleTransfer(I2C1, deAdd, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    LL_I2C_TransmitData8(I2C1, whoReg);
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
        if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {
            LL_I2C_TransmitData8(I2C1, data);

        }
    }

	LL_I2C_ClearFlag_STOP(I2C1);
	//LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
	//LL_I2C_ClearFlag_NACK(I2C1);
}


void I2C_WriteRegister(uint8_t slave_address, uint8_t register_address, uint8_t data) {



	  /* Initialize the handle transfer */
	  LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	  /* Wait for the TX Empty flag */
	  while(LL_I2C_IsActiveFlag_TXE(I2C1) == 0);

	  /* Get the received byte from the RX FIFO */
	  LL_I2C_TransmitData8(I2C1, register_address); // |0x80 auto-increment

	  /* Wait for the TX Empty flag */
	  while(LL_I2C_IsActiveFlag_TXE(I2C1) == 0);

	    /* Fill the TX FIFO with data */
	  LL_I2C_TransmitData8(I2C1, data);

	  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
	  LL_I2C_ClearFlag_ADDR(I2C1);
	  LL_I2C_ClearFlag_STOP(I2C1);
	  LL_I2C_ClearFlag_TXE(I2C1);
	  LL_I2C_ClearFlag_NACK(I2C1);

	  /* Wait for the Transfer Complete flag */
	  //  while(LL_I2C_IsActiveFlag_TC(BSP_I2C) == 0);


}





/**
  * @brief  I2C read function used for the ASM330LHH IMU sensor.
  * @param  handle: handle.
  * @param  Reg: Reg.
  * @param  pBuff: pBuff.
  * @param  nBuffSize: nBuffSize.
  * @retval None
  */
uint32_t readRegisterMeow(uint32_t* buffer, uint8_t slave, uint8_t Reg, uint16_t nBuffSize)
{

  //I2C1_SoftwareReset();
  //LL_mDelay(10);


  uint32_t receive = 0;
  uint8_t Regval = Reg;
  for(uint16_t i = 0; i < nBuffSize; i++) {
  I2C1_SoftwareReset();

  /* Initialize the handle transfer */
  LL_I2C_HandleTransfer(I2C1, slave, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  /* Wait for the TX Empty flag */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) == 0);

  /* Get the received byte from the RX FIFO */
  LL_I2C_TransmitData8(I2C1, Regval); // |0x80 auto-increment

  /* Wait for the TX Empty flag */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) == 0);


  /* Wait for the Transfer Complete flag */
    //while(LL_I2C_IsActiveFlag_TC(I2C1) == 0);

  /* Initialize the handle transfer */
  LL_I2C_HandleTransfer(I2C1, slave, LL_I2C_ADDRSLAVE_7BIT, nBuffSize,
  LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);



    /* Wait for the RX Not Empty flag */
    while(LL_I2C_IsActiveFlag_RXNE(I2C1) == 0);

    /* Get the received byte from the RX FIFO */
   // pBuff[i] = LL_I2C_ReceiveData8(I2C1);
    buffer[i] = LL_I2C_ReceiveData8(I2C1);
    receive++;
    LL_mDelay(10);
    Regval++;

  }

  /* Wait for the Transfer Complete flag */
  //    while(LL_I2C_IsActiveFlag_TC(BSP_I2C) == 0);
  return receive;
}


uint32_t writeRegisterMroow(uint8_t slave, uint8_t Reg, uint8_t pBuff, uint16_t nBuffSize)
{
  I2C1_SoftwareReset();

  /* Initialize the handle transfer */
  LL_I2C_HandleTransfer(I2C1, slave, LL_I2C_ADDRSLAVE_7BIT, nBuffSize+1, LL_I2C_MODE_AUTOEND , LL_I2C_GENERATE_START_WRITE);

  /* Wait for the TX Empty flag */

  while(LL_I2C_IsActiveFlag_TXE(I2C1) == 0);

  /* Get the received byte from the RX FIFO */
  LL_I2C_TransmitData8(I2C1, Reg); // |0x80 auto-increment

  /* Wait for the TX Empty flag */
  while(LL_I2C_IsActiveFlag_TXE(I2C1) == 0);

  //while(!LL_I2C_IsActiveFlag_TC(I2C1));

  for(uint16_t i = 0; i < nBuffSize; i++) {
    /* Wait for the TX flag */
    while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
    /* Fill the TX FIFO with data */
    LL_I2C_TransmitData8(I2C1, pBuff);
    LL_mDelay(10);

  }


  /* Wait for the Transfer Complete flag */
  //  while(LL_I2C_IsActiveFlag_TC(BSP_I2C) == 0);
return 0;

}

void Activate_I2C1_IT(void)
{
    LL_I2C_EnableIT_TX(I2C1);
    LL_I2C_EnableIT_RX(I2C1);
    LL_I2C_EnableIT_ADDR(I2C1);
    LL_I2C_EnableIT_NACK(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    LL_I2C_EnableIT_STOP(I2C1);
    LL_I2C_Enable(I2C1);
}
void I2C1_SoftwareReset(void)
{
    /* Disable peripheral */
    LL_I2C_Disable(I2C1);
    LL_I2C_ClearFlag_STOP(I2C1);
    /* Perform a dummy read to delay the disable of peripheral for minimum
    3 APB clock cycles to perform the software reset functionality */
    *(__IO uint32_t *)(uint32_t)I2C1;

    /* Enable peripheral */
    LL_I2C_Enable(I2C1);
}

