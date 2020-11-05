# vector_nav
Driver for VectorNav Inertial Measurement Unit (IMU) and Inertial Navigation System (INS) sensors.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

## Description
VectorNav produces a line of high accuracy IMU and INS sensors. The MEMS sensors in these units are temperature calibrated and an integrated microcontroller provides real-time Extended Kalman Filtering (EKF). The VN-100 is an IMU and Attitude and Heading Reference System (AHRS) providing IMU data and an estimate of the vehicle's attitude and heading. The VN-200 and VN-300 include an integrated GNSS receiver, providing GNSS data and extending the sensor to an Inertial Navigation System (INS), providing high rate estimates of the vehicle's velocity and position in addition to attitude. The VN-300 utilizes two GNSS receivers to enhance heading accuracy. This library communicates with the VN-100, VN-200, and VN-300 sensors using SPI communication.

## Installation
CMake is used to build this library, which is exported as a library target called *vector_nav*. The header is added as:

```
#include "vector_nav/vector_nav.h"
```
Note that you'll need CMake version 3.13 or above; it is recommended to build and install CMake from source, directions are located in the [CMake GitLab repository](https://github.com/Kitware/CMake).

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and an example executables called *spi_example*. The example executable source files are located at *examples/spi_example.cc*. This code is built and tested on AARCH64 and AMD64 systems running Linux and AMD64 systems running the Windows Subsystem for Linux (WSL). The [arm-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) toolchain must be installed in your Linux environment.

Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64

These are known to work with the same packages used in Teensy products. Also switching the MK66FX1M0 or MK64FX512 from BGA to LQFP packages is known to work well. Swapping packages of other chips is probably fine, as long as it's only a package change.

The *spi_example* target creates an executable for communicating with the sensor using SPI communication. The target also has a *_hex* for creating the hex file and a *_upload* to upload the software to the microcontroller. 

## Namespace
This library is within the namespace *sensors*.

## Registers
*registers.h*, within the namespace *vector_nav* defines all of the VectorNav configuration and data registers. Registers that are common across VectorNav's product line are within the namespace *common*, registers for the VN-100 are within the namespace *vn100*, registers for the VN-200 are within the namespace *vn200*, and registers for the VN-300 are within the namespace *vn300*. Each register is defined as a struct, whose name matches the register name within the VectorNav User Manuals. The structs define the register ID, the register size, whether it is read only, and the register payload. The payload contains the register fields that are written to or read from.

The concept is the *VectorNav* class, described below, initializes communication with the sensor and provides methods for writing and reading these register structs to/from the sensor. The *VectorNav* class, including structs defined in *registers.h*, enables using any of the sensor's available functionality. The *Vn100*, *Vn200*, and *Vn300* classes wrap around the *VectorNav* class to provide convenience methods for the most common configuration and data collection functions. Whereas these classes provide limited functionality, they provide a more intuitive for the majority of use cases.

## VectorNav
This class enables initializing communication with the VectorNav and writing and reading register structs, defined in *registers.h*, to the device.

### Methods

**VectorNav(SPIClass &ast;bus, uint8_t cs)** Constructs a *VectorNav* object given a pointer to the SPI bus object that is is communicating over and the chip select pin number.

```C++
VectorNav vn(&SPI, 2);
```

**ErrorCode** Most *VectorNav* methods return an error code indicating success or failure of the operation. Below is a table of the error code values:

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

**void Init()** Initializes communication with the sensor. **Note:** this method simply initializes the communication bus and chi select pin. It does not test whether communication with the VectorNav sensor is successful. It is recommend to read a register to test for successful communication.

```C++
vn.Init();
```

**ErrorCode ReadRegister(REG &ast;ptr)** Reads a register given a pointer to the register struct. Returns an *ErrorCode* indicating success or failure of the operation.

```C++
vector_nav::common::SerialNumber sn;
ErrorCode err = vn.ReadRegister(&sn);
if (err == ERROR_SUCCESS) {
   Serial.println(sn.payload.serial_num);
}
```

**ErrorCode WriteRegister(const REG &ref)** Writes a register given a reference to the register struct. Returns an *ErrorCode* indicating success or failure of the operation.

```C++
vector_nav::vn200::GnssAntennaOffset ant;
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

## Vn100
This class wraps around the *VectorNav* class to provide convenience methods for the most comman functionality using the VN-100 sensor.

### Methods

**Vn100(SPIClass &ast;bus, const uint8_t cs)** Constructs a *Vn100* object given a pointer to the SPI bus object that is is communicating over and the chip select pin number.

```C++
Vn100 vn(&SPI, 2);
```

**VectorNav::ErrorCode error_code()** Most methods within the *Vn100* class return a boolean indicating success or failure of the operation. The error code from the last operation is returned by this method, which can be useful in debugging if an operation fails.

```C++
if (!vn.Begin()) {
   Serial.println(vn.error_code());
}
```

**bool Begin()** Initializes communication with the VN-100 sensor. Returns true on successfully establishing communication and false on failure.

```C++
bool status = vn.Begin();
```

**bool EnableDrdyInt(const DrdyMode mode, const uint16_t srd)** Enables the data ready interrupt, which is a positive pulse with a width of 500 us. The mode enables setting what triggers the interrupt:

| Enum | Description |
| :-: | :-: |
| IMU_START | Interrupt triggered at the start of IMU sampling |
| IMU_READY | Interrupt triggered when IMU data is ready |
| AHRS | Interrupt triggered when attitude solution is ready |

The sample rate divider (SRD) sets how many data ready events are skipped before an interrupt is triggered.

```C++
/* Interrupt on attitude solution, 50 Hz rate */
bool status = vn.EnableDrdyInt(Vn100::AHRS, 7);
```

**bool DisableDrdyInt()** Disables the data ready interrupt.

**bool ApplyRotation(const Eigen::Matrix3f &c)** Applies a rotation. This is useful if the sensor is mounted in a vehicle such that the sensor axes no longer align with the vehicle axes. Rotates the sensor and filter outputs. Outputs are defined as:

```math
x_{u} = c * x_{b}
```

Where $`x_{u}`$ is the output, $`c`$ is the rotation matrix, and $`x_{b}`$ are the measurements in the sensor reference frame.

```C++
/*
* c =   0 1 0
*       1 0 0
*       0 0 1
*/
Eigen::Matrix3f c = Eigen::Matrix3f::Zero();
c(0, 1) = 1.0f;
c(1, 0) = 1.0f;
c(2, 2) = 1.0f;
bool status = vn.ApplyRotation(c);
```

**Note:** The VectorNav needs to write this rotation to non-volatile memory and perform a reset in order for it to be applied. These operations are performed within this method, so it takes a few seconds to complete.

**bool GetRotation(Eigen::Matrix3f &ast;c)** Retrieves the current rotation matrix from the VN-100.

```C++
Eigen::Matrix3f c;
bool status = vn.GetRotation(&c);
```

**Get / Set Filter** The following methods enable setting and getting digital low pass filters for the VectorNav sensors. Each filter can be set to filter the uncompensated, compensated, or both sets of data. The filter is tuned by setting a window length for the first order FIR, boxcar filter.

**bool SetMagFilter(const FilterMode mode, const uint16_t window)** Sets the magnetometer filter.

**bool GetMagFilter(FilterMode &ast;mode, uint16_t &ast;window)** Gets the magnetometer filter current settings.

**bool SetAccelFilter(const FilterMode mode, const uint16_t window)** Sets the accelerometer filter.

**bool GetAccelFilter(FilterMode &ast;mode, uint16_t &ast;window)** Gets the accelerometer filter current settings.

**bool SetGyroFilter(const FilterMode mode, const uint16_t window)** Sets the gyro filter.

**bool GetGyroFilter(FilterMode &ast;mode, uint16_t &ast;window)** Gets the gyro filter current settings.

**bool SetTemperatureFilter(const FilterMode mode, const uint16_t window)** Sets the temperature filter.

**bool GetTemperatureFilter(FilterMode &ast;mode, uint16_t &ast;window)** Gets the temperature filter current settings.

**bool SetPressureFilter(const FilterMode mode, const uint16_t window)** Sets the pressure filter.

**bool GetPressureFilter(FilterMode &ast;mode, uint16_t &ast;window)** Gets the pressure filter current settings.

**bool DrdyCallback(const uint8_t int_pin, void (&ast;function)())** Enables setting a callback function to be executed on the data ready interrupt, given the pin number of the microcontroller connected to the data ready interrupt pin.

**bool VelocityCompensation(const float speed_mps)** Compensates the VN-100 accelerometer for turning flight by passing the sensor the current airspeed in m/s.

**bool Read()** Retrieves the current data from the VN-100 sensor and, on success, stores the updated values in the *Vn100* object.

```C++
if (vn.Read()) {
   Serial.print(vn.yaw_rad());
   Serial.print("\t");
   Serial.print(vn.pitch_rad());
   Serial.print("\t");
   Serial.print(vn.roll_rad());
   Serial.print("\n");
}
```

**float yaw_rad()** Returns the yaw angle, rad.

**float pitch_rad()** Returns the pitch angle, rad.

**float roll_rad()** Returns the roll angle, rad.

**float accel_x_mps2()** Returns the compensated x accelerometer, m/s/s.

**float accel_y_mps2()** Returns the compensated y accelerometer, m/s/s.

**float accel_z_mps2()** Returns the compensated z accelerometer, m/s/s.

**Eigen::Vector3f accel_mps2()** Returns the compensated accelerometer as a vector, m/s/s.

**float gyro_x_radps()** Returns the compensated x gyro, rad/s.

**float gyro_y_radps()** Returns the compensated y gyro, rad/s.

**float gyro_z_radps()** Returns the compensated z gyro, rad/s.

**Eigen::Vector3f gyro_radps()** Returns the compensated gyro as a vector, rad/s.

**float mag_x_ut()** Returns the compensated x magnetometer, uT.

**float mag_y_ut()** Returns the compensated y magnetometer, uT.

**float mag_z_ut()** Returns the compensated z magnetometer, uT.

**Eigen::Vector3f mag_ut()** Returns the compensated magnetometer as a vector, uT.

**float uncomp_accel_x_mps2()** Returns the uncompensated x accelerometer, m/s/s.

**float uncomp_accel_y_mps2()** Returns the uncompensated y accelerometer, m/s/s.

**float uncomp_accel_z_mps2()** Returns the uncompensated z accelerometer, m/s/s.

**Eigen::Vector3f uncomp_accel_mps2()** Returns the uncompensated accelerometer as a vector, m/s/s.

**float uncomp_gyro_x_radps()** Returns the uncompensated x gyro, rad/s.

**float uncomp_gyro_y_radps()** Returns the uncompensated y gyro, rad/s.

**float uncomp_gyro_z_radps()** Returns the uncompensated z gyro, rad/s.

**Eigen::Vector3f uncomp_gyro_radps()** Returns the uncompensated gyro as a vector, rad/s.

**float uncomp_mag_x_ut()** Returns the uncompensated x magnetometer, uT.

**float uncomp_mag_y_ut()** Returns the uncompensated y magnetometer, uT.

**float uncomp_mag_z_ut()** Returns the uncompensated z magnetometer, uT.

**Eigen::Vector3f uncomp_mag_ut()** Returns the uncompensated magnetometer as a vector, uT.

**float die_temperature_c()** Returns the sensor die temperature, C.

**float pressure_pa()** Returns the measured static pressure, Pa.

**void RestoreFactorySettings()** Restores the VectorNav to factory default settings.

**void Reset()** Resets the VectorNav.

**bool KnownMagneticDisturbance(bool present)** Notifies the VectorNav that a magnetic disturbance is present. The sensor will tune out the magnetometer and pause the current hard / soft iron calibration, if enabled. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool KnownAccelerationDisturbance(bool present)** Notifies the VectorNav that an acceleration disturbance is present. The sensor will tune out the accelerometer. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool SetGyroBias()** Commands the VectorNav to copy the current gyro bias estimates into volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.

**bool Tare()** Commands the VectorNav to zero out its current orientation.

## Vn200

## Vn300

