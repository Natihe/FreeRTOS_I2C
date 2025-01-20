// Import I2C Driver definitions
#include <ti/drivers/I2C.h>
// Define name for an index of an I2C bus
#define SENSORS 0
// Define the slave address of device on the SENSORS bus
#define OPT_ADDR 0x18
// One-time init of I2C driver
I2C_init();
// initialize optional I2C bus parameters
I2C_Params params;
I2C_Params_init(&params);
params.bitRate = I2C_400kHz;
// Open I2C bus for usage
I2C_Handle i2cHandle = I2C_open(SENSORS, &params);
// Initialize slave address of transaction
I2C_Transaction transaction = {0};
transaction.slaveAddress = OPT_ADDR;
// Read from I2C slave device
transaction.readBuf = data;
transaction.readCount = sizeof(data);
transaction.writeCount = 0;
I2C_transfer(i2cHandle, &transaction);
// Write to I2C slave device
transaction.writeBuf = command;
transaction.writeCount = sizeof(command);
transaction.readCount = 0;
I2C_transferTimeout(i2cHandle, &transaction, 5000);
// Close I2C
I2C_close(i2cHandle);