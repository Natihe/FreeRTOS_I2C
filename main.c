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
#include <stdint.h>
#include <FreeRTOS.h>
#include <I2C.h>
#include <stdbool.h>
#include <math.h>
#include "semphr.h"
#include "faceup.h"
#include "acc.h"

#define SCALE_2G 16384.0f
#define SCALE_4G 8192.0f
#define SCALE_8G 4096.0f
#define SCALE_16G 2048.0f

#define FACE_UP_THRESHOLD 0.8f              // Przykładowy próg dla stanu face-up
#define FACE_DOWN_THRESHOLD -0.8f           // Przykładowy próg dla stanu face-down
#define STABLE_DURATION pdMS_TO_TICKS(1000) // 1 second in ticks
#define MOVEMENT_THRESHOLD 0.1f

static struct accelerometer_data accelerometer01;
static struct accelerometer_data accelerometer02;

// Mutex handle.
SemaphoreHandle_t xMutex;


/**
 * @brief Callback function to handle changes in accelerometer state.
 *
 * This function is called whenever there is a change in the state of the accelerometer.
 * It processes the new accelerometer data and performs necessary actions based on the
 * updated state.
 *
 * @param accelerometer Pointer to a structure containing the accelerometer data.
 */
void accStateChanged(struct accelerometer_data *accelerometer)
{
    printf("Accelerometer at address 0x%02X changed state from %d to %d\n", accelerometer->device_addr, accelerometer->last_stable_status, accelerometer->current_status);
}

/**
 * @brief Initializes the I2C master interface.
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
 * @brief Detects the face status based on accelerometer data.
 *
 * This function analyzes the data from the accelerometer to determine the face status.
 *
 * @param accelerometer A pointer to a structure containing accelerometer data.
 */
void detect_face_status(struct accelerometer_data *accelerometer)
{
    TickType_t current_time = xTaskGetTickCount();

    // Check for minimal movement in X/Y direction
    if (fabs(accelerometer->x) < MOVEMENT_THRESHOLD && fabs(accelerometer->y) < MOVEMENT_THRESHOLD)
    {
        if (accelerometer->z > FACE_UP_THRESHOLD)
        {
            if (accelerometer->current_status != FACE_UP)
            {
                accelerometer->current_status = FACE_UP;
                accelerometer->stable_start_time = current_time;
            }
            else if ((current_time - accelerometer->stable_start_time) >= STABLE_DURATION)
            {
                accelerometer->last_stable_status = FACE_UP;
            }
        }
        else if (accelerometer->z < FACE_DOWN_THRESHOLD)
        {
            if (accelerometer->current_status != FACE_DOWN)
            {
                accelerometer->current_status = FACE_DOWN;
                accelerometer->stable_start_time = current_time;
            }
            else if ((current_time - accelerometer->stable_start_time) >= STABLE_DURATION)
            {
                accelerometer->last_stable_status = FACE_DOWN;
            }
        }
    }
    else
    {
        // Reset if there is significant movement in X/Y direction
        accelerometer->current_status = FACE_UNKNOWN;
        accelerometer->stable_start_time = current_time;
    }
    if (accelerometer->current_status != FACE_UNKNOWN && accelerometer->current_status != accelerometer->last_stable_status && accelerometer->callback != NULL)
    {
        accelerometer->callback(accelerometer);
    }
}


/**
 * @brief Starts the FaceUp detection process using accelerometer data.
 *
 * This function initializes and starts the process of detecting if the device
 * is facing up based on the provided accelerometer data.
 *
 * @param accelerometer Pointer to a structure containing accelerometer data.
 */
void StartFaceUp(struct accelerometer_data *accelerometer)
{
    if (accelerometer == NULL)
    {
        EroorHalnder();
        return;
    }
    int16_t buffer[];
    // Sprawdzenie identyfikatora urządzenia
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE && configureLIS2DTW12(accelerometer))
    {
        xSemaphoreGive(xMutex);
        while (1)
        {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE){
            i2c_communicate(accelerometer); // Można wymienić na inną komunikację
            }
            xSemaphoreGive(xMutex);
            detect_face_status(accelerometer);
            // convertRawData
            vTaskDelay(pdMS_TO_TICKS(1000)); // Odczekaj 1 sekundę
        }
    }
    else
    {
        xSemaphoreGive(xMutex);
        printf("Failed to configure LIS2DTW12.\n");
    }
}

int main()
{
    // Inicjalizacja obu magistrali I2C
    i2c_master_init();

    // Utworzenie mutexu
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL)
    {
        printf("Failed to create mutex.\n");
        return 1; // Exit if mutex creation fails.
    }

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
    
    // Stworzenie zadań dla obu czujników
    xTaskCreate(StartFaceUp, "Sensor Task 1", 2048, &accelerometer01, 5, NULL);
    xTaskCreate(StartFaceUp, "Sensor Task 2", 2048, &accelerometer02, 5, NULL);
    
    // Rozpoczęcie planisty
    vTaskStartScheduler();
    return 0;
}
