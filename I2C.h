/*  * Copyright (c) 2015-2020, Texas Instruments Incorporated
    * All rights reserved.
    *
    * Redistribution and use in source and binary forms, with or without
    * modification, are permitted provided that the following conditions
    * are met:
    *
    * *  Redistributions of source code must retain the above copyright
    *    notice, this list of conditions and the following disclaimer.
    *
    * *  Redistributions in binary form must reproduce the above copyright
    *    notice, this list of conditions and the following disclaimer in the
    *    documentation and/or other materials provided with the distribution.
    *
    * *  Neither the name of Texas Instruments Incorporated nor the names of
    *    its contributors may be used to endorse or promote products derived
    *    from this software without specific prior written permission.
    *
    * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
    * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
    * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
    * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
    * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    */
   /*!****************************************************************************
    *  @file       I2C.h
    *  @brief      Inter-Integrated Circuit (I2C) Driver
    *
    *  @anchor ti_drivers_I2C_Overview
    *  # Overview
    *
    *  The I2C driver is designed to operate as an I2C master and will not
    *  function as an I2C slave. Multi-master arbitration is not supported;
    *  therefore, this driver assumes it is the only I2C master on the bus.
    *  This I2C driver's API set provides the ability to transmit and receive
    *  data over an I2C bus between the I2C master and I2C slave(s). The
    *  application is responsible for manipulating and interpreting the data.
    *
    *
    *  <hr>
    *  @anchor ti_drivers_I2C_Usage
    *  # Usage
    *
    *  This section provides a basic @ref ti_drivers_I2C_Synopsis
    *  "usage summary" and a set of @ref ti_drivers_I2C_Examples "examples"
    *  in the form of commented code fragments. Detailed descriptions of the
    *  I2C APIs and their effect are provided in subsequent sections.
    *
    *  @anchor ti_drivers_I2C_Synopsis
    *  ## Synopsis #
    *  @anchor ti_drivers_I2C_Synopsis_Code
    *  @code
    *  // Import I2C Driver definitions
    *  #include <ti/drivers/I2C.h>
    *
    *  // Define name for an index of an I2C bus
    *  #define SENSORS 0
    *
    *  // Define the slave address of device on the SENSORS bus
    *  #define OPT_ADDR 0x47
    *
    *  // One-time init of I2C driver
    *  I2C_init();
    *
    *  // initialize optional I2C bus parameters
    *  I2C_Params params;
    *  I2C_Params_init(&params);
    *  params.bitRate = I2C_400kHz;
    *
    *  // Open I2C bus for usage
    *  I2C_Handle i2cHandle = I2C_open(SENSORS, &params);
    *
    *  // Initialize slave address of transaction
    *  I2C_Transaction transaction = {0};
    *  transaction.slaveAddress = OPT_ADDR;
    *
    *  // Read from I2C slave device
    *  transaction.readBuf = data;
    *  transaction.readCount = sizeof(data);
    *  transaction.writeCount = 0;
    *  I2C_transfer(i2cHandle, &transaction);
    *
    *  // Write to I2C slave device
    *  transaction.writeBuf = command;
    *  transaction.writeCount = sizeof(command);
    *  transaction.readCount = 0;
    *  I2C_transferTimeout(i2cHandle, &transaction, 5000);
    *
    *  // Close I2C
    *  I2C_close(i2cHandle);
    *  @endcode
    *
    *  @anchor ti_drivers_I2C_Examples
    *  ## Examples
    *
    *  @li @ref ti_drivers_I2C_Example_open "Getting an I2C bus handle"
    *  @li @ref ti_drivers_I2C_Example_write3bytes "Sending 3 bytes"
    *  @li @ref ti_drivers_I2C_Example_read5bytes "Reading 5 bytes"
    *  @li @ref ti_drivers_I2C_Example_writeread "Writing then reading in a single transaction"
    *  @li @ref ti_drivers_I2C_Example_callback "Using Callback mode"
    *
    *  @anchor ti_drivers_I2C_Example_open
    *  ## Opening the I2C Driver
    *
    *  After calling I2C_init(), the application can open an I2C instance by
    *  calling I2C_open().The following code example opens an I2C instance with
    *  default parameters by passing @p NULL for the #I2C_Params argument.
    *
    *  @code
    *  I2C_Handle i2cHandle;
    *
    *  i2cHandle = I2C_open(0, NULL);
    *
    *  if (i2cHandle == NULL) {
    *      // Error opening I2C
    *      while (1) {}
    *  }
    *  @endcode
    *
    *  @anchor ti_drivers_I2C_Example_write3bytes
    *  ## Sending three bytes of data.
    *
    *  @code
    *  I2C_Transaction i2cTransaction = {0};
    *  uint8_t writeBuffer[3];
    *
    *  writeBuffer[0] = 0xAB;
    *  writeBuffer[1] = 0xCD;
    *  writeBuffer[2] = 0xEF;
    *
    *  i2cTransaction.slaveAddress = 0x50;
    *  i2cTransaction.writeBuf = writeBuffer;
    *  i2cTransaction.writeCount = 3;
    *  i2cTransaction.readBuf = NULL;
    *  i2cTransaction.readCount = 0;
    *
    *  status = I2C_transfer(i2cHandle, &i2cTransaction);
    *
    *  if (status == false) {
    *      // Unsuccessful I2C transfer
    *      if (i2cTransaction.status == I2C_STATUS_ADDR_NACK) {
    *          // I2C slave address not acknowledged
    *      }
    *  }
    *  @endcode
    *
    *  @anchor ti_drivers_I2C_Example_read5bytes
    *  ## Reading five bytes of data.
    *
    *  @code
    *  I2C_Transaction i2cTransaction = {0};
    *  uint8_t readBuffer[5];
    *
    *  i2cTransaction.slaveAddress = 0x50;
    *  i2cTransaction.writeBuf = NULL;
    *  i2cTransaction.writeCount = 0;
    *  i2cTransaction.readBuf = readBuffer;
    *  i2cTransaction.readCount = 5;
    *
    *  status = I2C_transfer(i2cHandle, &i2cTransaction);
    *
    *  if (status == false) {
    *      if (i2cTransaction.status == I2C_STATUS_ADDR_NACK) {
    *          // I2C slave address not acknowledged
    *      }
    *  }
    *  @endcode
    *
    *  @anchor ti_drivers_I2C_Example_writeread
    *  ## Writing two bytes and reading four bytes in a single transaction.
    *
    *  @code
    *  I2C_Transaction i2cTransaction = {0};
    *  uint8_t readBuffer[4];
    *  uint8_t writeBuffer[2];
    *
    *  writeBuffer[0] = 0xAB;
    *  writeBuffer[1] = 0xCD;
    *
    *  i2cTransaction.slaveAddress = 0x50;
    *  i2cTransaction.writeBuf = writeBuffer;
    *  i2cTransaction.writeCount = 2;
    *  i2cTransaction.readBuf = readBuffer;
    *  i2cTransaction.readCount = 4;
    *
    *  status = I2C_transfer(i2cHandle, &i2cTransaction);
    *
    *  if (status == false) {
    *       if (i2cTransaction->status == I2C_STATUS_ADDR_NACK) {
    *           // slave address not acknowledged
    *       }
    *  }
    *  @endcode
    *
    *  @anchor ti_drivers_I2C_Example_callback
    *  ## Using callback mode
    *  This final example shows usage of #I2C_MODE_CALLBACK, with queuing
    *  of multiple transactions. Because multiple transactions are simultaneously
    *  queued, separate #I2C_Transaction structures must be used. Each
    *  #I2C_Transaction will contain a custom application argument of a
    *  semaphore handle. The #I2C_Transaction.arg will point to the semaphore
    *  handle. When the callback function is called, the #I2C_Transaction.arg is
    *  checked for @p NULL. If this value is not @p NULL, then it can be assumed
    *  the @p arg is pointing to a valid semaphore handle. The semaphore handle
    *  is then used to call @p sem_post(). Hypothetically, this can be used to
    *  signal transaction completion to the task(s) that queued the
    *  transaction(s).
    *
    *  @code
    *  void callbackFxn(I2C_Handle handle, I2C_Transaction *msg, bool status)
    *  {
    *      // if transaction failed
    *      if (status == false) {
    *          if (msg->status == I2C_STATUS_ADDR_NACK) {
    *              // slave address not acknowledged
    *          }
    *          else if (msg->status == I2C_STATUS_CANCEL) {
    *              // transaction canceled by I2C_cancel()
    *          }
    *      }
    *
    *      // Check for a custom argument
    *      if (msg->arg != NULL) {
    *
    *          // In this example, the custom argument is a semaphore handle
    *          // Perform a semaphore post
    *          sem_post((sem_t *) (msg->arg));
    *      }
    *  }
    *  @endcode
    *
    *  Snippets of the thread code that initiates the transactions are shown below.
    *  Note the use of multiple #I2C_Transaction structures. The handle of the
    *  semaphore to be posted is specified via @p i2cTransaction2.arg.
    *  I2C_transfer() is called three times to initiate each transaction.
    *  Since callback mode is used, these functions return immediately. After
    *  the transactions have been queued, other work can be done. Eventually,
    *  @p sem_wait() is called causing the thread to block until the transaction
    *  completes. When the transaction completes, the application's callback
    *  function, @p callbackFxn will be called. Once #I2C_CallbackFxn posts the
    *  semaphore, the thread will be unblocked and can resume execution.
    *
    *  @code
    *  void thread(arg0, arg1)
    *  {
    *
    *      I2C_Transaction i2cTransaction0 = {0};
    *      I2C_Transaction i2cTransaction1 = {0};
    *      I2C_Transaction i2cTransaction2 = {0};
    *
    *      // ...
    *
    *      i2cTransaction0.arg = NULL;
    *      i2cTransaction1.arg = NULL;
    *      i2cTransaction2.arg = semaphoreHandle;
    *
    *      // ...
    *
    *      I2C_transfer(i2c, &i2cTransaction0);
    *      I2C_transfer(i2c, &i2cTransaction1);
    *      I2C_transfer(i2c, &i2cTransaction2);
    *
    *      // ...
    *
    *      sem_wait(semaphoreHandle);
    *  }
    *  @endcode
    *
    *  <hr>
    *  @anchor ti_drivers_I2C_Configuration
    *  # Configuration
    *
    *  Refer to the @ref driver_configuration "Driver's Configuration" section
    *  for driver configuration information.
    *  <hr>
    ******************************************************************************
    */
   
   #ifndef ti_drivers_I2C__include
   #define ti_drivers_I2C__include
   
   #include <stdbool.h>
   #include <stddef.h>
   #include <stdint.h>
   
   #include <ti/drivers/dpl/HwiP.h>
   #include <ti/drivers/dpl/SemaphoreP.h>
   #ifdef __cplusplus
   extern "C" {
   #endif
   
   #define I2C_STATUS_RESERVED        (-32)
   
   #define I2C_STATUS_QUEUED          (1)
   
   #define I2C_STATUS_SUCCESS         (0)
   
   #define I2C_STATUS_ERROR           (-1)
   
   #define I2C_STATUS_UNDEFINEDCMD    (-2)
   
   #define I2C_STATUS_TIMEOUT         (-3)
   
   #define I2C_STATUS_CLOCK_TIMEOUT   (-4)
   
   #define I2C_STATUS_ADDR_NACK       (-5)
   
   #define I2C_STATUS_DATA_NACK       (-6)
   
   #define I2C_STATUS_ARB_LOST        (-7)
   
   #define I2C_STATUS_INCOMPLETE      (-8)
   
   #define I2C_STATUS_BUS_BUSY        (-9)
   
   #define I2C_STATUS_CANCEL          (-10)
   
   #define I2C_STATUS_INVALID_TRANS   (-11)
   
   #define I2C_WAIT_FOREVER           (~(0U))
   
   typedef struct I2C_Config_ *I2C_Handle;
   
   typedef struct {
       void         *writeBuf;
   
       size_t        writeCount;
   
       void         *readBuf;
   
       size_t        readCount;
   
       void         *arg;
   
       volatile int_fast16_t status;
   
       uint_least8_t slaveAddress;
   
       void         *nextPtr;
   } I2C_Transaction;
   
   typedef enum {
       I2C_MODE_BLOCKING,
   
       I2C_MODE_CALLBACK
   } I2C_TransferMode;
   
   typedef void (*I2C_CallbackFxn)(I2C_Handle handle, I2C_Transaction *transaction,
       bool transferStatus);
   
   typedef enum {
       I2C_100kHz     = 0,    
       I2C_400kHz     = 1,    
       I2C_1000kHz    = 2,    
       I2C_3330kHz    = 3,    
       I2C_3400kHz    = 3,    
   } I2C_BitRate;
   
   typedef struct {
       I2C_TransferMode transferMode;
   
       I2C_CallbackFxn transferCallbackFxn;
   
       I2C_BitRate bitRate;
   
       void *custom;
   } I2C_Params;
   
   #define I2C_BASE_OBJECT \
       /* I2C control variables */ \
       I2C_TransferMode           transferMode;        /* Blocking or Callback mode */ \
       I2C_CallbackFxn            transferCallbackFxn; /* Callback function pointer */ \
       I2C_Transaction           *currentTransaction;  /* Ptr to current I2C transaction */ \
       \
       /* I2C transaction pointers for I2C_MODE_CALLBACK */ \
       I2C_Transaction * volatile headPtr;            /* Head ptr for queued transactions */ \
       I2C_Transaction           *tailPtr;            /* Tail ptr for queued transactions */ \
       \
       /* I2C RTOS objects */ \
       HwiP_Struct                hwi;                /* Hwi object handle */ \
       SemaphoreP_Struct          mutex;              /* Grants exclusive access to I2C */ \
       SemaphoreP_Struct          transferComplete;   /* Signal I2C transfer complete */ \
       \
       /* Read and write variables */ \
       const uint8_t             *writeBuf;           /* Internal inc. writeBuf index */ \
       size_t                     writeCount;         /* Internal dec. writeCounter */ \
       uint8_t                   *readBuf;            /* Internal inc. readBuf index */ \
       size_t                     readCount;          /* Internal dec. readCounter */ \
       \
       bool                       isOpen;             /* Flag to show module is open */ \
   
   typedef struct {
       I2C_BASE_OBJECT
   } I2C_Object;
   #define I2C_BASE_HWATTRS \
    \
       uint32_t baseAddr; \
    \
       uint32_t intNum; \
    \
       uint32_t intPriority;
   
   typedef struct {
       I2C_BASE_HWATTRS
   } I2C_HWAttrs;
   typedef struct I2C_Config_ {
       void               *object;
   
       void         const *hwAttrs;
   } I2C_Config;
   
   extern const I2C_Config I2C_config[];
   extern const uint_least8_t I2C_count;
   
   extern void I2C_cancel(I2C_Handle handle);
   
   extern void I2C_close(I2C_Handle handle);
   
   extern int_fast16_t I2C_control(I2C_Handle handle, uint_fast16_t cmd,
       void *controlArg);
   
   extern void I2C_init(void);
   
   extern I2C_Handle I2C_open(uint_least8_t index, I2C_Params *params);
   
   extern void I2C_Params_init(I2C_Params *params);
   
   extern int_fast16_t I2C_setClockTimeout(I2C_Handle handle, uint32_t timeout);
   
   extern bool I2C_transfer(I2C_Handle handle, I2C_Transaction *transaction);
   
   extern int_fast16_t I2C_transferTimeout(I2C_Handle handle,
       I2C_Transaction *transaction, uint32_t timeout);
   
   #ifdef __cplusplus
   }
   #endif
   
   #endif /* ti_drivers_I2C__include */