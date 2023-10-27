/*
 * i2cm.c
 *
 *  Created on: Oct 27, 2023
 *      Author: Meow
 */
#include "main.h"
#include "stm32f3xx_it.h"



uint8_t ReadWhoAmI(uint8_t deAdd, uint8_t whoReg, uint8_t whoVal) {
    uint8_t receivedData;
    LL_I2C_EnableIT_RX(I2C1);

    // Initialize I2C communication to read WHO_AM_I register
    LL_I2C_HandleTransfer(I2C1, deAdd, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    // Send the WHO_AM_I register address
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
        if (LL_I2C_IsActiveFlag_TXIS(I2C1)) {
            LL_I2C_TransmitData8(I2C1, whoReg);
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    // Receive WHO_AM_I value
    LL_I2C_HandleTransfer(I2C1, deAdd, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {}

	LL_I2C_DisableIT_RX(I2C1);
	LL_I2C_ClearFlag_STOP(I2C1);
	LL_I2C_ClearFlag_NACK(I2C1);
	receivedData = getData();
    // Check if the received WHO_AM_I value matches the expected value
    if (receivedData == whoVal) {
        return 1; // WHO_AM_I value is correct
    } else {
        return 0; // WHO_AM_I value is incorrect
    }
}
