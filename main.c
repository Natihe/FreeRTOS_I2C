/**
 * @file main.c
 * @brief Main file for the FreeRTOS I2C project.
 *
 * This file contains the main function and initialization code for the
 * FreeRTOS I2C project. It sets up the necessary hardware and software
 * components to enable I2C communication using the FreeRTOS operating system.
 *
 * @note Ensure that the correct hardware configuration is set up before
 * running this code.
 */

// #include <stdio.h>
#include "faceup.h"
#include <FreeRTOS.h>
#include <I2C.h>

#define SCALE_2G 16384.0f
#define SCALE_4G 8192.0f
#define SCALE_8G 4096.0f
#define SCALE_16G 2048.0f

#define FACE_UP_THRESHOLD 1.0    // Przykładowy próg dla stanu face-up
#define FACE_DOWN_THRESHOLD -1.0 // Przykładowy próg dla stanu face-down

static struct accelerometer_data accelerometer01;
static struct accelerometer_data accelerometer02;

/**
 * @brief 
 * 
 */ @brief Initializes the I2C master interface.
 *
 * This function sets up the I2C master interface with the necessary
 * configurations such as clock speed, GPIO pins, and other relevant
 * parameters. It prepares the I2C master for communication with I2C
 * slave devices.
 *
 * @note This function must be called before any I2C communication
 *       can take place.
 */
void i2c_master_init()
{
    Board_init(); // Tutaj jest wybór pinów do I2C
    I2C_init();
    // Ustawienia parametrów dla obu magistrali I2C
    I2C_Params i2cParams.bitRate = I2C_400kHz;
    I2C_Params_init(&i2cParams);

    // Inicjalizacja magistrali pierwszej I2C
    I2C_Handle i2cHandle0 = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2cHandle0 == NULL)
    {
        printf("Failed to initialize CONFIG_I2C_0\n");
        while (1)
            ;
    }

    // Inicjalizacja magistrali drugiej I2C
    I2C_Handle i2cHandle1 = I2C_open(CONFIG_I2C_1, &i2cParams);
    if (i2cHandle1 == NULL)
    {
        printf("Failed to initialize CONFIG_I2C_1\n");
        while (1)
            ;
    }
}

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
void configureLIS2DTW12(I2C_Handle i2cHandle, uint8_t device_addr, CTRL_DATA_RATE_CONFIGURATION dataRate, CTRL_MODE mode, LP_MODE lpMode)
{
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[2];

    txBuffer[0] = 0x20; // Adres rejestru CTRL1
    txBuffer[1] = (dataRate << 4) | (mode << 2) | lpMode;
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
        printf("Failed to configure LIS2DTW12.\n");
    }
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
        // Składanie wartości 16-bitowych
        int16_t raw_x = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);
        int16_t raw_y = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
        int16_t raw_z = (int16_t)((rxBuffer[5] << 8) | rxBuffer[4]);

        // Przeliczanie na jednostki g
        float x = convert_to_g(raw_x, 2, 14); // Zakres 2g, rozdzielczość 14-bitowa
        float y = convert_to_g(raw_y, 2, 14);
        float z = convert_to_g(raw_z, 2, 14);

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
// Funkcja wykrywająca stan face-up/face-down
void detect_face_status(float accel_data, face_status_t *status)
{
    if (accel_data > FACE_UP_THRESHOLD)
    {
        *status = FACE_UP;
    }
    else if (accel_data < FACE_DOWN_THRESHOLD)
    {
        *status = FACE_DOWN;
    }
}

// Zadanie do obsługi czujnika
void StartFaceUp(struct accelerometer_data *accelerometer)
{

    if (accelerometer == NULL)
    {
        EroorHalnder();
        return;
    }
    int16_t buffer[];
    // Sprawdzenie identyfikatora urządzenia
    if (readWhoAmI(accelerometer.i2cHandle, accelerometer.device_addr) == 0x44)
    {
        configureLIS2DTW12(accelerometer.i2cHandle, accelerometer.device_addr, accelerometer.CTRLDataRateConfiguration, accelerometer.ctrlMode, accelerometer.lpMode);

        while (1)
        {
            i2c_communicate(accelerometer);
            accelerometer.up = buffer > 0;
            // convertRawData
            vTaskDelay(pdMS_TO_TICKS(1000)); // Odczekaj 1 sekundę
        }
    }
}

int main()
{
    // Inicjalizacja obu magistrali I2C
    i2c_master_init();
    // Konfiguracja czujników
    accelerometer01.i2cHandle = i2cHandle0;
    accelerometer01.device_addr = 0x18;
    accelerometer01.ctrlMode = HIGH_PERFORMANCE;
    accelerometer01.lpMode = LP_MODE_4;
    accelerometer01.CTRLDataRateConfiguration = HP_LP_50_HZ;

    accelerometer02.i2cHandle = i2cHandle1;
    accelerometer02.device_addr = 0x19;
    accelerometer02.ctrlMode = HIGH_PERFORMANCE;
    accelerometer02.lpMode = LP_MODE_4;
    accelerometer02.CTRLDataRateConfiguration = HP_LP_50_HZ;

    xTaskCreate(StartFaceUp(&accelerometer01), "Sensor Task 1", 2048, NULL, 5, NULL);
    xTaskCreate(StartFaceUp(&accelerometer02), "Sensor Task 2", 2048, NULL, 5, NULL);

    return 0;
}
