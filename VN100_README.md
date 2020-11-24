# Vn100
This class wraps around the *VectorNav* class to provide convenience methods for the most common functionality using the VN-100 sensor.

## Methods

**Vn100(SPIClass &ast;bus, const uint8_t cs)** Constructs a *Vn100* object given a pointer to the SPI bus object that it is communicating over and the chip select pin number.

```C++
sensors::Vn100 vn(&SPI, 2);
```

**VectorNav::ErrorCode error_code()** Most methods within the *Vn100* class return a boolean indicating success or failure of the operation. The [error code](VECTOR_NAV_README.md#error_code) from the last operation is returned by this method, which can be useful in debugging if an operation fails.

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

**float die_temperature_c()** Returns the sensor die temperature, C.

**float pressure_pa()** Returns the measured static pressure, Pa.

**bool WriteSettings()** Writes the current settings to the VectorNav non-volatile memory.

**void RestoreFactorySettings()** Restores the VectorNav to factory default settings.

**void Reset()** Resets the VectorNav.

**bool KnownMagneticDisturbance(bool present)** Notifies the VectorNav that a magnetic disturbance is present. The sensor will tune out the magnetometer and pause the current hard / soft iron calibration, if enabled. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool KnownAccelerationDisturbance(bool present)** Notifies the VectorNav that an acceleration disturbance is present. The sensor will tune out the accelerometer. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool SetGyroBias()** Commands the VectorNav to copy the current gyro bias estimates into volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.

**bool Tare()** Commands the VectorNav to zero out its current orientation.
