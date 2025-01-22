#ifndef __IG_FACEUP_H__
#define __IG_FACEUP_H__

// Zmienna do przechowywania informacji o stanie (face-up/face-down)
typedef enum
{
    FACE_UP,
    FACE_DOWN
} face_status_t;

typedef enum SensorStatus
{
    SENSOR_OK,
    SENSOR_ERROR
} SENSOR_STATUS;

typedef void (*callback_t)(struct accelerometer_data *accelerometer);

struct accelerometer_data
{
    uint8_t device_addr;
    I2C_Handle i2cHandle;
    callback_t callback;
    bool up;
    int16_t x, y, z;
    SENSOR_STATUS status;
};

typedef enum CTRLDataRateConfiguration
{
    POWER_DOWN = 0x00,        // 0000 Power-down
    HP_LP_12_5_1_6_HZ = 0x01, // 0001 High-performance / low-power mode 12.5 / 1.6 Hz
    HP_LP_12_5_HZ = 0x02,     // 0010 High-performance / low-power mode 12.5 Hz
    HP_LP_25_HZ = 0x03,       // 0011 High-performance / low-power mode 25 Hz
    HP_LP_50_HZ = 0x04,       // 0100 High-performance / low-power mode 50 Hz
    HP_LP_100_HZ = 0x05,      // 0101 High-performance / low-power mode 100 Hz
    HP_LP_200_HZ = 0x06,      // 0110 High-performance / low-power mode 200 Hz
    HP_LP_400_200_HZ = 0x07,  // 0111 High-performance / low-power mode 400 / 200 Hz
    HP_LP_800_200_HZ = 0x08,  // 1000 High-performance / low-power mode 800 / 200 Hz
    HP_LP_1600_200_HZ = 0x09  // 1001 High-performance / low-power mode 1600 / 200 Hz
} CTRL_DATA_RATE_CONFIGURATION;

typedef enum CTRL_MODE
{
    LOW_POWER = 0x0,        // 00 Low-power mode (12/14-bit resolution)
    HIGH_PERFORMANCE = 0x1, // 01 High-performance mode (14-bit resolution)
    SINGLE_DATA_CONV = 0x2, // 10 Single data conversion on-demand mode (12/14-bit resolution)
} CTRL_MODE;

typedef enum LP_Mode
{
    LP_MODE_1 = 0x0, // 00 Low-power mode 1 (12bit resolution)
    LP_MODE_2 = 0x1, // 01 Low-power mode 2 (14bit resolution)
    LP_MODE_3 = 0x2, // 10 Low-power mode 3 (14bit resolution)
    LP_MODE_4 = 0x3  // 11 Low-power mode 4 (14bit resolution)
} LP_MODE;

#endif // __IG_FACEUP_H__