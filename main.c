// #include <stdio.h>
#include "faceup.h"
#include <FreeRTOS.h>
#include <I2C.h>

#define FACE_UP_THRESHOLD 1.0    // Przykładowy próg dla stanu face-up
#define FACE_DOWN_THRESHOLD -1.0 // Przykładowy próg dla stanu face-down

static struct accelerometer_data accelerometer01;
static struct accelerometer_data accelerometer02;

// funkcja do inicjalizacji I2C
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
    printf("I2C initialized successfully.\n");
}

// Funckaj do sprawdzania identyfikatora urządzenia (WHO_AM_I)
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

// Funkcja do konfiguracji LIS2DTW12
// Domyślne ustawienia: ODR = 100 Hz, FS = ±2g TO DO
void configureLIS2DTW12(I2C_Handle i2cHandle, uint8_t device_addr)
{
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[2];

    // Ustawienia w CTRL1: ODR = 100 Hz, FS = ±2g
    txBuffer[0] = 0x20; // Adres rejestru CTRL1
    txBuffer[1] = 0x50; // ODR = 100 Hz, FS = ±2g
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

// Funkcja odczytu danych z LIS2DTW12
void i2c_communicate(I2C_Handle i2cHandle, uint8_t device_addr, const char *task_name)
{
    I2C_Transaction i2cTransaction;
    uint8_t txBuffer[1];
    uint8_t rxBuffer[6];

    txBuffer[0] = 0x2C | 0x80; // Adres OUTZ_L z ustawieniem bitu inkrementacji

    i2cTransaction.slaveAddress = device_addr;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 6;

    if (I2C_transfer(i2cHandle, &i2cTransaction))
    {
        // Składanie wartości 16-bitowych
        *x = (int16_t)((rxBuffer[1] << 8) | rxBuffer[0]);
        *y = (int16_t)((rxBuffer[3] << 8) | rxBuffer[2]);
        *z = (int16_t)((rxBuffer[5] << 8) | rxBuffer[4]);
        printf("X: %d, Y: %d, Z: %d\n", *x, *y, *z);
    }
    else
    {
        printf("Failed to read accelerometer data.\n");
    }
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
    // TUTAJ URUCHAMIAM WĄTEK
    if (readWhoAmI(accelerometer.i2cHandle, accelerometer.device_addr) == 0x44)
    {
        configureLIS2DTW12();

        while (1)
        {
            readAccelerometerData(accelerometer.x, accelerometer.y, accelerometer.z);
            accelerometer.up = buffer > 0;
            vTaskDelay(pdMS_TO_TICKS(1000)); // Odczekaj 1 sekundę
        }
    }
}

int main()
{

    // Inicjalizacja obu magistrali I2C
    i2c_master_init();

    accelerometer01.i2cHandle = i2cHandle0;
    accelerometer01.device_addr = 0x18;
    accelerometer02.i2cHandle = i2cHandle1;
    accelerometer02.device_addr = 0x19;

    xTaskCreate(StartFaceUp(&accelerometer01), "Sensor Task 1", 2048, NULL, 5, NULL);
    xTaskCreate(StartFaceUp(&accelerometer02), "Sensor Task 2", 2048, NULL, 5, NULL);

    return 0;
}
