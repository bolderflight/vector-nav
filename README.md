[![Pipeline](https://gitlab.com/bolderflight/software/vector_nav/badges/main/pipeline.svg)](https://gitlab.com/bolderflight/software/vector_nav/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# VectorNav
Driver for VectorNav Inertial Measurement Unit (IMU) and Inertial Navigation System (INS) sensors. This library is compatible with Arduino ARM and with CMake build systems.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
VectorNav produces a line of high accuracy IMU and INS sensors. The MEMS sensors in these units are temperature calibrated and an integrated microcontroller provides real-time Extended Kalman Filtering (EKF). The VN-100 is an IMU and Attitude and Heading Reference System (AHRS) providing IMU data and an estimate of the vehicle's attitude and heading. The VN-200 and VN-300 include an integrated GNSS receiver, providing GNSS data and extending the sensor to an Inertial Navigation System (INS), providing high rate estimates of the vehicle's velocity and position in addition to attitude. VN-200 and VN-300 attitude accuracy is higher than the VN-100, especially in flight dynamics and manuevers, such as prolonged turns. The VN-300 utilizes two GNSS receivers to enhance heading accuracy at low speeds. This library communicates with the VN-100, VN-200, and VN-300 sensors using SPI communication.

# Installation

## Arduino
Simply clone or download and extract the zipped library into your Arduino/libraries folder. In addition to this library, the [Bolder Flight Systems Units library](https://github.com/bolderflight/units) and [Bolder Flight Systems Eigen library](https://github.com/bolderflight/eigen) must be installed. The library is added as:

```C++
#include "vector_nav.h"
```

An example is located in *examples/arduino/spi_example/spi_example.ino*. This library is tested with Teensy 3.x, 4.x, and LC devices and is expected to work with other Arduino ARM devices. It is **not** expected to work with AVR devices.

## CMake
CMake is used to build this library, which is exported as a library target called *vector_nav*. The header is added as:

```C++
#include "vector_nav.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and an example executables called *spi_example*. The example executable source files are located at *examples/cmake/spi_example.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *spi_example* target creates an executable for communicating with the sensor using SPI communication. This target also has a *_hex* for creating the hex file and an *_upload* for using the [Teensy CLI Uploader](https://www.pjrc.com/teensy/loader_cli.html) to flash the Teensy. Please note that the CMake build tooling is expected to be run under Linux or WSL, instructions for setting up your build environment can be found in our [build-tools repo](https://github.com/bolderflight/build-tools). 

# Namespace
This library is within the namespace *bfs*.

# Registers
*registers.h* defines all of the VectorNav configuration and data registers. Each register is defined as a struct, whose name matches the register name within the VectorNav User Manuals. The structs define the register ID, the register size, whether it is read only, and the register payload. The payload contains the register fields that are written to or read from.

The concept is the *VectorNav* class initializes communication with the sensor and provides methods for writing and reading these register structs to/from the sensor. The *VectorNav* class, including structs defined in *registers.h*, enables using any of the sensor's available functionality. The *Vn100*, *Vn200*, and *Vn300* classes wrap around the *VectorNav* class to provide convenience methods for the most common configuration and data collection functions. Whereas these classes provide limited functionality, they provide a more intuitive interface for the majority of use cases.

# Classes
   * [VectorNav](#vector_nav): enables initializing communication and writing / reading register structs.
   * [Vn100](#vn100): provides convenience methods for the VN-100 IMU / AHRS.
   * [Vn200](#vn200): provides convenience methods for the VN-200 GNSS-aided INS.
   * [Vn300](#vn300): provides convenience methods for the VN-300 GNSS-aided INS.

# VectorNav<a name="vector_nav"></a>
This class enables initializing communication with the VectorNav and writing and reading register structs, defined in *registers.h*, to the device.

## Methods

**VectorNav(SPIClass &ast;bus, const uint8_t cs)** Constructs a *VectorNav* object given a pointer to the SPI bus object that it is communicating over and the chip select pin number.

```C++
bfs::VectorNav vn(&SPI, 2);
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
| ERROR_NO_COMM | 14 |
| ERROR_WRONG_MODEL | 15 |
| ERROR_ERROR_BUFFER_OVERFLOW | 255 |

**void Init()** Initializes communication with the sensor. **Note:** this method simply initializes the communication bus and chip select pin. It does not test whether communication with the VectorNav sensor is successful. It is recommend to read a register to test for successful communication. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

```C++
SPI.begin();
vn.Init();
```

**ErrorCode ReadRegister(REG &ast;ptr)** Reads a register given a pointer to the register struct. Returns an *ErrorCode* indicating success or failure of the operation.

```C++
VnSerialNumber sn;
ErrorCode err = vn.ReadRegister(&sn);
if (err == ERROR_SUCCESS) {
  Serial.println(sn.payload.serial_num);
}
```

**ErrorCode WriteRegister(const REG &ref)** Writes a register given a reference to the register struct. Returns an *ErrorCode* indicating success or failure of the operation.

```C++
VnGnssAntennaOffset ant;
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

# Vn100<a name="vn100"></a>
This class wraps around the *VectorNav* class to provide convenience methods for the most common functionality using the VN-100 sensor.

## Methods

**Vn100(SPIClass &ast;bus, const uint8_t cs)** Constructs a *Vn100* object given a pointer to the SPI bus object that it is communicating over and the chip select pin number.

```C++
bfs::Vn100 vn(&SPI, 2);
```

**VectorNav::ErrorCode error_code()** Most methods within the *Vn100* class return a boolean indicating success or failure of the operation. The [error code](#error_code) from the last operation is returned by this method, which can be useful in debugging if an operation fails.

```C++
if (!vn.Begin()) {
   Serial.println(vn.error_code());
}
```

**bool Begin()** Initializes communication with the VN-100 sensor. Returns true on successfully establishing communication and false on failure. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

```C++
SPI.begin();
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

```
Xu = c * Xb
```

Where *Xu* is the output, *c* is the rotation matrix, and *Xb* are the measurements in the sensor reference frame.

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

**Get / Set Filter** The following methods enable setting and getting digital low pass filters for the VectorNav sensors. Each filter can be set to filter the uncompensated, compensated, or both sets of data using the *FilterMode* enum.

| Enum | Description |
| :-: | :-: |
| FILTER_NONE | No filtering is applied |
| FILTER_UNCOMP_ONLY | Filtering is applied to the uncompensated data only |
| FILTER_COMP_ONLY | Filtering is applied to the compensated data only |
| FILTER_BOTH | Filtering is applied to both the uncompensated and compensated data |

The filter is tuned by setting a window length for the first order FIR, boxcar filter.

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

**float die_temp_c()** Returns the sensor die temperature, C.

**float pres_pa()** Returns the measured static pressure, Pa.

**bool WriteSettings()** Writes the current settings to the VectorNav non-volatile memory.

**void RestoreFactorySettings()** Restores the VectorNav to factory default settings.

**void Reset()** Resets the VectorNav.

**bool KnownMagneticDisturbance(bool present)** Notifies the VectorNav that a magnetic disturbance is present. The sensor will tune out the magnetometer and pause the current hard / soft iron calibration, if enabled. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool KnownAccelerationDisturbance(bool present)** Notifies the VectorNav that an acceleration disturbance is present. The sensor will tune out the accelerometer. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool SetGyroBias()** Commands the VectorNav to copy the current gyro bias estimates into volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.

**bool Tare()** Commands the VectorNav to zero out its current orientation.

# Vn200<a name="vn200"></a>
This class wraps around the *VectorNav* class to provide convenience methods for the most common functionality using the VN-200 sensor.

## Methods

**Vn200(SPIClass &ast;bus, const uint8_t cs)** Constructs a *Vn200* object given a pointer to the SPI bus object that is is communicating over and the chip select pin number.

```C++
bfs::Vn200 vn(&SPI, 2);
```

**VectorNav::ErrorCode error_code()** Most methods within the *Vn200* class return a boolean indicating success or failure of the operation. The [error code](#error_code) from the last operation is returned by this method, which can be useful in debugging if an operation fails.

```C++
if (!vn.Begin()) {
   Serial.println(vn.error_code());
}
```

**bool Begin()** Initializes communication with the VN-200 sensor. Returns true on successfully establishing communication and false on failure. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

```C++
SPI.begin();
bool status = vn.Begin();
```

**bool EnableDrdyInt(const DrdyMode mode, const uint16_t srd)** Enables the data ready interrupt, which is a positive pulse with a width of 500 us. The mode enables setting what triggers the interrupt:

| Enum | Description |
| :-: | :-: |
| IMU_START | Interrupt triggered at the start of IMU sampling |
| IMU_READY | Interrupt triggered when IMU data is ready |
| INS | Interrupt triggered when the INS solution is ready |
| GNSS_PPS | Interrupt triggered by the GNSS Pulse-Per-Second (PPS) |

The sample rate divider (SRD) sets how many data ready events are skipped before an interrupt is triggered.

```C++
/* Interrupt on INS solution, 50 Hz rate */
bool status = vn.EnableDrdyInt(Vn200::INS, 7);
```

**bool DisableDrdyInt()** Disables the data ready interrupt.

**bool EnableExternalGnss(const PpsSource pps)** Enables using an external GNSS receiver, rather than the VN-200 internal receiver. The external receiver must send GNSS to the VN-200 at a rate of 5 Hz using the *SendExternalGnssData* method. Additionally, a Pulse-Per-Second (PPS) output from the external receiver must be connected to the VN-200. This method takes an enum parameter specifying the PPS input pin and polarity.

| Enum | Description |
| :-: | :-: |
| PPS_RISING | External PPS is connected to the VN-200 PPS pin, triggered on a rising edge |
| PPS_FALLING | External PPS is connected to the VN-200 PPS pin, triggered on a falling edge |
| SYNC_IN_RISING | External PPS is connected to the VN-200 sync in pin, triggered on a rising edge |
| SYNC_IN_FALLING | External PPS is connected to the VN-200 sync in pin, triggered on a falling edge |

**bool DisableExternalGnss()** Disables the external GNSS receiver.

**bool ApplyRotation(const Eigen::Matrix3f &c)** Applies a rotation. This is useful if the sensor is mounted in a vehicle such that the sensor axes no longer align with the vehicle axes. Rotates the sensor and filter outputs. Outputs are defined as:

```
Xu = c * Xb
```

Where *Xu* is the output, *c* is the rotation matrix, and *Xb* are the measurements in the sensor reference frame.

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

**bool GetRotation(Eigen::Matrix3f &ast;c)** Retrieves the current rotation matrix from the VN-200.

```C++
Eigen::Matrix3f c;
bool status = vn.GetRotation(&c);
```

**bool SetAntennaOffset(const Eigen::Vector3f &b)** Sets the offset from the VN-200 to the antenna. Units are in meters and given in the VN-200 reference frame.

```C++
/* Antenna 2 meters in front on and above the VN-200 */
Eigen::Vector3f b = {2.0, 0, -2.0};
bool status = vn.SetAntennaOffset(b);
```

**bool GetAntennaOffset(Eigen::Vector3f &ast;b)** Retrieves the current antenna offset from the VN-200.

```C++
Eigen::Vector3f b;
bool status = vn.GetAntennaOffset(&b);
```

**Get / Set Filter** The following methods enable setting and getting digital low pass filters for the VectorNav sensors. Each filter can be set to filter the uncompensated, compensated, or both sets of data using the *FilterMode* enum.

| Enum | Description |
| :-: | :-: |
| FILTER_NONE | No filtering is applied |
| FILTER_UNCOMP_ONLY | Filtering is applied to the uncompensated data only |
| FILTER_COMP_ONLY | Filtering is applied to the compensated data only |
| FILTER_BOTH | Filtering is applied to both the uncompensated and compensated data |

The filter is tuned by setting a window length for the first order FIR, boxcar filter.

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

**bool SendExternalGnssData(const VnGnssSolutionLla &ref)** Sends external GNSS data to the VN-200 given a reference to the LLA solution register. The VN-200 must first be configured to use an external GNSS receiver using the *EnableExternalGnss* method.

**bool SendExternalGnssData(const VnGnssSolutionEcef &ref)** Sends external GNSS data to the VN-200 given a reference to the ECEF solution register. The VN-200 must first be configured to use an external GNSS receiver using the *EnableExternalGnss* method.

**bool Read()** Retrieves the current data from the VN-200 sensor and, on success, stores the updated values in the *Vn200* object.

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

**InsMode ins_mode()** Returns the current INS mode, which is an enum.

| Enum | Description |
| :-: | :-: |
| NOT_TRACKING | The INS filter is awaiting initialization and not currently tracking. |
| DEGRADED | The INS filter is tracking, but the performance is below specifications. This can occur during initial alignment or if the attitude uncertainty rises above 2 degrees. |
| HEALTHY | The INS filter is tracking within specification. |
| GNSS_LOSS | Communication has been lost from the GNSS for more than 45 seconds. The INS will no longer update position or velocity estimates, but the attitude solution remains valid |

**bool ins_error()** Returns true if an INS error has occurred.

**bool ins_time_error()** Returns true if a time error has occurred.

**bool ins_imu_error()** Returns true if an IMU read error has occurred.

**bool ins_mag_pres_error()** Returns true if a magnetometer or pressure read error has occurred.

**bool ins_gnss_error()** Returns true if a GNSS receiver read error has occurred.

**double ins_time_s()** Returns the INS time of week in seconds. This is equivalent to the GNSS time of week, but updated at the INS rate.

**uint16_t ins_week()** Returns the INS week number. This is equivalent to the GNSS week number, but updated at the INS rate.

**double ins_lat_rad()** Returns the latitude from the INS, rad.

**double ins_lon_rad()** Returns the longitude from the INS, rad.

**double ins_alt_m()** Returns the altitude above the WGS-84 ellipsoid from the INS, m.

**Eigen::Vector3d ins_lla_rad_m()** Returns the INS latitude, longitude, and altitude as a vector.

**float ins_north_vel_mps()** Returns the INS velocity in the north direction, m/s.

**float ins_east_vel_mps()** Returns the INS velocity in the east direction, m/s.

**float ins_down_vel_mps()** Returns the INS velocity in the down direction, m/s.

**Eigen::Vector3f ins_ned_vel_mps()** Returns the INS north, east, and down velocity as a vector, m/s.

**float ins_att_uncertainty_rad()** Returns the INS attitude estimation uncertainty, rad.

**float ins_pos_uncertainty()** Returns the INS position estimation uncertainty, m.

**float ins_vel_uncertainty()** Returns the INS velocity estimation uncertainty, m/s.

**double gnss_time_s()** Returns the GNSS time of week, seconds.

**uint16_t gnss_week()** Returns the GNSS week number.

**GnssFix gnss_fix()** Returns the GNSS fix status.

| Enum | Description |
| :-: | :-: |
| FIX_NONE | No fix |
| FIX_TIME_ONLY | Time only fix |
| FIX_2D | 2D fix |
| FIX_3D | 3D fix |
| FIX_SBAS | 3D fix with corrections from SBAS |

**uint8_t gnss_num_satellites()** Returns the number of satellites used in the GNSS solution.

**double gnss_lat_rad()** Returns the GNSS latitude, rad.

**double gnss_lon_rad()** Returns the GNSS longitude, rad.

**double gnss_alt_m()** Returns the GNSS altitude above the WGS-84 ellipsoid, m.

**Eigen::Vector3d gnss_lla_rad_m()** Returns the latitude, longitude, and altitude as a vector.

**float gnss_north_vel_mps()** Returns the GNSS velocity in the north direction, m/s.

**float gnss_east_vel_mps()** Returns the GNSS velocity in the east direction, m/s.

**float gnss_down_vel_mps()** Returns the GNSS velocity in the down direction, m/s.

**Eigen::Vector3f gnss_ned_vel_mps()** Returns the GNSS north, east, and down velocity as a vector, m/s.

**float gnss_north_acc_m()** Returns the estimated GNSS position accuracy in the north direction, m.

**float gnss_east_acc_m()** Returns the estimated GNSS position accuracy in the east direction, m.

**float gnss_down_acc_m()** Returns the estimated GNSS position accuracy in the down direction, m.

**float gnss_speed_acc_mps()** Returns the estimated GNSS speed accuracy, m/s.

**float gnss_time_acc_s()** Returns the estimated GNSS time accuracy, s.

**float yaw_rad()** Returns the yaw angle relative to true north, rad.

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

**float die_temp_c()** Returns the sensor die temperature, C.

**float pres_pa()** Returns the measured static pressure, Pa.

**bool WriteSettings()** Writes the current settings to the VectorNav non-volatile memory.

**void RestoreFactorySettings()** Restores the VectorNav to factory default settings.

**void Reset()** Resets the VectorNav.

**bool KnownMagneticDisturbance(bool present)** Notifies the VectorNav that a magnetic disturbance is present. The sensor will tune out the magnetometer and pause the current hard / soft iron calibration, if enabled. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool KnownAccelerationDisturbance(bool present)** Notifies the VectorNav that an acceleration disturbance is present. The sensor will tune out the accelerometer. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool SetGyroBias()** Commands the VectorNav to copy the current gyro bias estimates into volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.

**bool SetFilterBias()** Commands the VectorNav to copy the current filter bias estimates ino volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.

# Vn300<a name="vn300"></a>
This class wraps around the *VectorNav* class to provide convenience methods for the most common functionality using the VN-300 sensor.

## Methods

**Vn300(SPIClass &ast;bus, const uint8_t cs)** Constructs a *Vn300* object given a pointer to the SPI bus object that it is communicating over and the chip select pin number.

```C++
bfs::Vn300 vn(&SPI, 2);
```

**VectorNav::ErrorCode error_code()** Most methods within the *Vn300* class return a boolean indicating success or failure of the operation. The [error code](#error_code) from the last operation is returned by this method, which can be useful in debugging if an operation fails.

```C++
if (!vn.Begin()) {
   Serial.println(vn.error_code());
}
```

**bool Begin()** Initializes communication with the VN-300 sensor. Returns true on successfully establishing communication and false on failure. The communication bus is not initialized within this library and must be initialized seperately; this enhances compatibility with other sensors that may on the same bus.

```C++
SPI.begin();
bool status = vn.Begin();
```

**bool EnableDrdyInt(const DrdyMode mode, const uint16_t srd)** Enables the data ready interrupt, which is a positive pulse with a width of 500 us. The mode enables setting what triggers the interrupt:

| Enum | Description |
| :-: | :-: |
| IMU_START | Interrupt triggered at the start of IMU sampling |
| IMU_READY | Interrupt triggered when IMU data is ready |
| INS | Interrupt triggered when the INS solution is ready |
| GNSS_PPS | Interrupt triggered by the GNSS Pulse-Per-Second (PPS) |

The sample rate divider (SRD) sets how many data ready events are skipped before an interrupt is triggered.

```C++
/* Interrupt on INS solution, 50 Hz rate */
bool status = vn.EnableDrdyInt(Vn300::INS, 7);
```

**bool DisableDrdyInt()** Disables the data ready interrupt.

**bool ApplyRotation(const Eigen::Matrix3f &c)** Applies a rotation. This is useful if the sensor is mounted in a vehicle such that the sensor axes no longer align with the vehicle axes. Rotates the sensor and filter outputs. Outputs are defined as:

```
Xu = c * Xb
```

Where *Xu* is the output, *c* is the rotation matrix, and *Xb* are the measurements in the sensor reference frame.

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

**bool GetRotation(Eigen::Matrix3f &ast;c)** Retrieves the current rotation matrix from the VN-300.

```C++
Eigen::Matrix3f c;
bool status = vn.GetRotation(&c);
```

**bool SetAntennaOffset(const Eigen::Vector3f &b)** Sets the offset from the VN-300 to the antenna. Units are in meters and given in the VN-300 reference frame.

```C++
/* Antenna 2 meters in front on and above the VN-300 */
Eigen::Vector3f b = {2.0, 0, -2.0};
bool status = vn.SetAntennaOffset(b);
```

**bool GetAntennaOffset(Eigen::Vector3f &ast;b)** Retrieves the current antenna offset from the VN-300.

```C++
Eigen::Vector3f b;
bool status = vn.GetAntennaOffset(&b);
```

**bool SetCompassBaseline(const Eigen::Vector3f &pos, const Eigen::Vector3f &uncert)** Sets the position of the second GNSS antenna relative to the first GNSS antenna in the VN-300 reference frame, meters. Accuracy of the GNSS heading is related to the baseline distance. In addition to the baseline position, uncertainty in each direction must also be provided, meters.

```C++
/* Second antenna is 1 meter in front of first antenna with an uncertainty of 1 cm */
Eigen::Vector3f b = {1.0, 0, 0.0};
Eigen::Vector3f c = {0.01, 0, 0.0};
bool status = vn.SetCompassBaseline(b, c);
```

**bool GetCompassBaseline(Eigen::Vector3f &ast;pos, Eigen::Vector3f &ast;uncert)** Retrieves the current compass baseline from the VN-300.

```C++
Eigen::Vector3f b, c;
bool status = vn.GetCompassBaseline(&b, &c);
```

**Get / Set Filter** The following methods enable setting and getting digital low pass filters for the VectorNav sensors. Each filter can be set to filter the uncompensated, compensated, or both sets of data using the *FilterMode* enum.

| Enum | Description |
| :-: | :-: |
| FILTER_NONE | No filtering is applied |
| FILTER_UNCOMP_ONLY | Filtering is applied to the uncompensated data only |
| FILTER_COMP_ONLY | Filtering is applied to the compensated data only |
| FILTER_BOTH | Filtering is applied to both the uncompensated and compensated data |

The filter is tuned by setting a window length for the first order FIR, boxcar filter.

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

**bool Read()** Retrieves the current data from the VN-300 sensor and, on success, stores the updated values in the *Vn300* object.

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

**InsMode ins_mode()** Returns the current INS mode, which is an enum.

| Enum | Description |
| :-: | :-: |
| NOT_TRACKING | The INS filter is awaiting initialization and not currently tracking. |
| DEGRADED | The INS filter is tracking, but the performance is below specifications. This can occur during initial alignment or if the attitude uncertainty rises above 2 degrees. |
| HEALTHY | The INS filter is tracking within specification. |
| GNSS_LOSS | Communication has been lost from the GNSS for more than 45 seconds. The INS will no longer update position or velocity estimates, but the attitude solution remains valid |

**bool ins_error()** Returns true if an INS error has occurred.

**bool ins_time_error()** Returns true if a time error has occurred.

**bool ins_imu_error()** Returns true if an IMU read error has occurred.

**bool ins_mag_pres_error()** Returns true if a magnetometer or pressure read error has occurred.

**bool ins_gnss_error()** Returns true if a GNSS receiver read error has occurred.

**bool ins_gnss_heading()** Returns true if the GNSS compass is currently aiding the INS filter heading solution.

**bool ins_gnss_compass()** Returns true if the GNSS compass is currently operational and reporting a heading solution.

**double ins_time_s()** Returns the INS time of week in seconds. This is equivalent to the GNSS time of week, but updated at the INS rate.

**uint16_t ins_week()** Returns the INS week number. This is equivalent to the GNSS week number, but updated at the INS rate.

**double ins_lat_rad()** Returns the latitude from the INS, rad.

**double ins_lon_rad()** Returns the longitude from the INS, rad.

**double ins_alt_m()** Returns the altitude above the WGS-84 ellipsoid from the INS, m.

**Eigen::Vector3d ins_lla_rad_m()** Returns the INS latitude, longitude, and altitude as a vector.

**float ins_north_vel_mps()** Returns the INS velocity in the north direction, m/s.

**float ins_east_vel_mps()** Returns the INS velocity in the east direction, m/s.

**float ins_down_vel_mps()** Returns the INS velocity in the down direction, m/s.

**Eigen::Vector3f ins_ned_vel_mps()** Returns the INS north, east, and down velocity as a vector, m/s.

**float ins_att_uncertainty_rad()** Returns the INS attitude estimation uncertainty, rad.

**float ins_pos_uncertainty()** Returns the INS position estimation uncertainty, m.

**float ins_vel_uncertainty()** Returns the INS velocity estimation uncertainty, m/s.

**double gnss_time_s()** Returns the GNSS time of week, seconds.

**uint16_t gnss_week()** Returns the GNSS week number.

**GnssFix gnss_fix()** Returns the GNSS fix status.

| Enum | Description |
| :-: | :-: |
| FIX_NONE | No fix |
| FIX_TIME_ONLY | Time only fix |
| FIX_2D | 2D fix |
| FIX_3D | 3D fix |
| FIX_SBAS | 3D fix with corrections from SBAS |

**uint8_t gnss_num_satellites()** Returns the number of satellites used in the GNSS solution.

**double gnss_lat_rad()** Returns the GNSS latitude, rad.

**double gnss_lon_rad()** Returns the GNSS longitude, rad.

**double gnss_alt_m()** Returns the GNSS altitude above the WGS-84 ellipsoid, m.

**Eigen::Vector3d gnss_lla_rad_m()** Returns the latitude, longitude, and altitude as a vector.

**float gnss_north_vel_mps()** Returns the GNSS velocity in the north direction, m/s.

**float gnss_east_vel_mps()** Returns the GNSS velocity in the east direction, m/s.

**float gnss_down_vel_mps()** Returns the GNSS velocity in the down direction, m/s.

**Eigen::Vector3f gnss_ned_vel_mps()** Returns the GNSS north, east, and down velocity as a vector, m/s.

**float gnss_north_acc_m()** Returns the estimated GNSS position accuracy in the north direction, m.

**float gnss_east_acc_m()** Returns the estimated GNSS position accuracy in the east direction, m.

**float gnss_down_acc_m()** Returns the estimated GNSS position accuracy in the down direction, m.

**float gnss_speed_acc_mps()** Returns the estimated GNSS speed accuracy, m/s.

**float gnss_time_acc_s()** Returns the estimated GNSS time accuracy, s.

**float yaw_rad()** Returns the yaw angle relative to true north, rad.

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

**float die_temp_c()** Returns the sensor die temperature, C.

**float pres_pa()** Returns the measured static pressure, Pa.

**bool WriteSettings()** Writes the current settings to the VectorNav non-volatile memory.

**void RestoreFactorySettings()** Restores the VectorNav to factory default settings.

**void Reset()** Resets the VectorNav.

**bool KnownMagneticDisturbance(bool present)** Notifies the VectorNav that a magnetic disturbance is present. The sensor will tune out the magnetometer and pause the current hard / soft iron calibration, if enabled. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool KnownAccelerationDisturbance(bool present)** Notifies the VectorNav that an acceleration disturbance is present. The sensor will tune out the accelerometer. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool SetGyroBias()** Commands the VectorNav to copy the current gyro bias estimates into volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.

**bool SetFilterBias()** Commands the VectorNav to copy the current filter bias estimates ino volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.
