#ifndef __IG_ACC_H__
#define __IG_ACC_H__

#include <I2C.h>
#include <stdint.h>
#include "faceup.h"

/**
 * @brief Reads the WHO_AM_I register from an I2C device.
 *
 * This function communicates with an I2C device to read the WHO_AM_I register,
 * which typically contains a device identification value.
 *
 * @param i2cHandle The handle to the I2C interface.
 * @param device_addr The I2C address of the device to communicate with.
 * @return The value of the WHO_AM_I register.
 */
uint8_t readWhoAmI(I2C_Handle i2cHandle, uint8_t device_addr)
{
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[1];
    uint8_t rxBuffer[1];

    txBuffer[0] = 0x0F; // Adres rejestru WHO_AM_I

    i2cTransaction.slaveAddress = device_addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;

    if (I2C_transfer(i2cHandle, &i2cTransaction))
    {
        printf("WHO_AM_I: 0x%02X\n", rxBuffer[0]);
        return rxBuffer[0];
    }
    else
    {
        printf("I2C transfer failed.\n");
        return 0xFF;
    }
}

/**
 * @brief Configures the LIS2DTW12 sensor with the specified settings.
 *
 * This function sets up the LIS2DTW12 sensor by configuring its data rate, mode, and low-power mode.
 *
 * @param i2cHandle The handle to the I2C interface.
 * @param device_addr The I2C address of the LIS2DTW12 sensor.
 * @param dataRate The desired data rate configuration for the sensor.
 * @param mode The operating mode of the sensor.
 * @param lpMode The low-power mode configuration for the sensor.
 */
bool configureLIS2DTW12(struct accelerometer_data *accelerometer)
{
    // to do: parametry powymieniać
    bool result = false;
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[2];

    txBuffer[0] = 0x20; // Adres rejestru CTRL1
    txBuffer[1] = (accelerometer->dataRate << 4) | (accelerometer->mode << 2) | accelerometer->lpMode;
    i2cTransaction.slaveAddress = device_addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = 0;

    if (I2C_transfer(i2cHandle, &i2cTransaction))
    {
        printf("LIS2DTW12 configured.\n");
    }
    else
    {
        return false;
    }
    result = readWhoAmI(accelerometer->i2cHandle, accelerometer->device_addr) == 0x44;
    accelerometer->callback = &accStateChanged;
    return result;
}

/**
 * @brief Reads the accelerometer data from the LIS2DTW12 sensor.
 *
 * This function reads the X, Y, and Z-axis accelerometer data from the LIS2DTW12 sensor.
 *
 * @param i2cHandle The handle to the I2C interface.
 * @param x A pointer to store the X-axis accelerometer data.
 * @param y A pointer to store the Y-axis accelerometer data.
 * @param z A pointer to store the Z-axis accelerometer data.
 */
void i2c_communicate(struct accelerometer_data *accelerometer)
{
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[1];
    uint8_t rxBuffer[6];

    txBuffer[0] = 0x28 | 0x80; // Adres OUTX_L z ustawieniem bitu inkrementacji

    i2cTransaction.slaveAddress = accelerometer->device_addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 6;

    if (I2C_transfer(accelerometer->i2cHandle, &i2cTransaction))
    {
        // Składanie wartości 16-bitowych BigEndian
        int16_t raw_x = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);
        int16_t raw_y = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
        int16_t raw_z = (int16_t)((rxBuffer[5] << 8) | rxBuffer[4]);

        // Przeliczanie na jednostki g
        accelerometer->x = convert_to_g(raw_x, 2, 14); // Zakres 2g, rozdzielczość 14-bitowa
        accelerometer->y = convert_to_g(raw_y, 2, 14);
        accelerometer->z = convert_to_g(raw_z, 2, 14);

        printf("X: %f g, Y: %f g, Z: %f g\n", x, y, z);
    }
    else
    {
        printf("Failed to read accelerometer data.\n");
    }
}

/**
 * @brief Converts raw accelerometer data to g units.
 *
 * This function converts the raw accelerometer data to g units based on the
 * configuration of the LIS2DTW12 sensor.
 *
 * @param raw_data The raw accelerometer data.
 * @param range The full-scale range setting of the sensor.
 * @param resolution The resolution setting of the sensor.
 * @return The converted accelerometer data in g units.
 */
float convert_to_g(int16_t raw_data, uint8_t range, uint8_t resolution)
{
    float sensitivity = 0.0;

    // Determine sensitivity based on range and resolution
    switch (range)
    {
    case 2:
        sensitivity = (resolution == 14) ? 0.000244 : 0.000061;
        break;
    case 4:
        sensitivity = (resolution == 14) ? 0.000488 : 0.000122;
        break;
    case 8:
        sensitivity = (resolution == 14) ? 0.000976 : 0.000244;
        break;
    case 16:
        sensitivity = (resolution == 14) ? 0.001953 : 0.000488;
        break;
    default:
        sensitivity = 0.0;
        break;
    }

    return raw_data * sensitivity;
}

#endif // __IG_ACC_H__