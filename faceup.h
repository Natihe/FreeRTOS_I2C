
// Zmienna do przechowywania informacji o stanie (face-up/face-down)
typedef enum
{
    FACE_UP,
    FACE_DOWN
} face_status_t;

typedef void (*callback_t)(struct accelerometer_data *accelerometer);

struct accelerometer_data
{
    uint8_t device_addr;
    I2C_Handle i2cHandle;
    callback_t callback;
    bool up;
    int16_t x, y, z;
};