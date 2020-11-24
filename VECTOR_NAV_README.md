# VectorNav
This class enables initializing communication with the VectorNav and writing and reading register structs, defined in *registers.h*, to the device.

## Methods

**VectorNav(SPIClass &ast;bus, uint8_t cs)** Constructs a *VectorNav* object given a pointer to the SPI bus object that is is communicating over and the chip select pin number.

```C++
sensors::VectorNav vn(&SPI, 2);
```

**ErrorCode<a name="error_code"></a>** Most *VectorNav* methods return an error code indicating success or failure of the operation. Below is a table of the error code values:

| ENUM | Value |
| :-: | :-: |
| ERROR_SUCCESS | 0 |
| ERROR_HARD_FAULT | 1 |
| ERROR_SERIAL_BUFFER_OVERFLOW | 2 |
| ERROR_INVALID_CHECKSUM | 3 |
| ERROR_INVALID_COMMAND | 4 |
| ERROR_NOT_ENOUGH_PARAMETERS | 5 |
| ERROR_TOO_MANY_PARAMETERS | 6 |
| ERROR_INVALID_PARAMETER | 7 |
| ERROR_INVALID_REGISTER | 8 |
| ERROR_UNAUTHORIZED_ACCESS | 9 |
| ERROR_WATCHDOG_RESET | 10 |
| ERROR_OUTPUT_BUFFER_OVERFLOW | 11 |
| ERROR_INSUFFICIENT_BAUD_RATE | 12 |
| ERROR_NULL_PTR | 13 |
| ERROR_ERROR_BUFFER_OVERFLOW | 255 |

**void Init()** Initializes communication with the sensor. **Note:** this method simply initializes the communication bus and chip select pin. It does not test whether communication with the VectorNav sensor is successful. It is recommend to read a register to test for successful communication.

```C++
vn.Init();
```

**ErrorCode ReadRegister(REG &ast;ptr)** Reads a register given a pointer to the register struct. Returns an *ErrorCode* indicating success or failure of the operation.

```C++
sensors::vector_nav::common::SerialNumber sn;
ErrorCode err = vn.ReadRegister(&sn);
if (err == ERROR_SUCCESS) {
  Serial.println(sn.payload.serial_num);
}
```

**ErrorCode WriteRegister(const REG &ref)** Writes a register given a reference to the register struct. Returns an *ErrorCode* indicating success or failure of the operation.

```C++
sensors::vector_nav::vn200::GnssAntennaOffset ant;
ant.payload.position_x = 2;
ant.payload.position_y = 0;
ant.payload.position_z = 0;
ErrorCode err = vn.WriteRegister(ant);
```

**ErrorCode WriteSettings()** Writes the current settings to the VectorNav non-volatile memory. Returns an *ErrorCode* indicating success or failure of the operation.

**void RestoreFactorySettings()** Restores the VectorNav to factory default settings.

**void Reset()** Resets the VectorNav.

**ErrorCode KnownMagneticDisturbance(bool present)** Notifies the VectorNav that a magnetic disturbance is present. The sensor will tune out the magnetometer and pause the current hard / soft iron calibration, if enabled. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**ErrorCode KnownAccelerationDisturbance(bool present)** Notifies the VectorNav that an acceleration disturbance is present. The sensor will tune out the accelerometer. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**ErrorCode SetGyroBias()** Commands the VectorNav to copy the current gyro bias estimates into volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.

**ErrorCode Tare()** Commands the VectorNav to zero out its current orientation. **Note:** This command is only available on the VN-100.

**ErrorCode SetFilterBias()** Commands the VectorNav to copy the current filter bias estimates ino volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command. **Note:** This command is only available on the VN-200 and VN-300.
