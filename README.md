# FreeRTOS_I2C
Program to run two LIS2DTW12 sensors in redundancy with usage of FreeRTOS


# Design model
## Design assumptions
The system uses two separate I2C buses to communicate with the accelerometers. This approach enhances redundancy and ensures that a failure in one bus does not affect the other sensor's operation.

## Functional Layer

The functional layer of the FreeRTOS_I2C project is responsible for managing the communication and data processing of the LIS2DTW12 sensors

### Sensor Interface
The sensor interface handles the initialization and configuration of the LIS2DTW12 sensors. It sets up the I2C communication and configures the sensors to operate in the desired mode.

### Data Acquisition
The data acquisition component is responsible for reading data from the sensors at regular intervals. It uses FreeRTOS tasks to periodically poll the sensors and retrieve acceleration data.

### Data Processing
The data processing component processes the raw data obtained from the sensors. It includes filtering, calibration, and redundancy checks to ensure the accuracy and reliability of the sensor data.

### Communication
The communication component manages the transmission of processed data to other parts of the system or external devices. It ensures that the data is transmitted reliably and efficiently.

### Error Handling
The error handling component monitors the system for any errors or anomalies. It includes mechanisms for detecting sensor failures, communication errors, and other issues, and takes appropriate actions to mitigate them.

### Task Management
The task management component uses FreeRTOS to manage the various tasks in the system. It ensures that tasks are scheduled and executed in a timely manner, and handles task synchronization and inter-task communication.

By organizing the project into these functional components, the FreeRTOS_I2C project ensures a modular and maintainable design that can be easily extended and adapted to different requirements.