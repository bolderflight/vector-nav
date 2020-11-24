# Vn300
This class wraps around the *VectorNav* class to provide convenience methods for the most common functionality using the VN-300 sensor.

## Methods

**Vn300(SPIClass &ast;bus, const uint8_t cs)** Constructs a *Vn300* object given a pointer to the SPI bus object that is is communicating over and the chip select pin number.

```C++
sensors::Vn300 vn(&SPI, 2);
```

**VectorNav::ErrorCode error_code()** Most methods within the *Vn300* class return a boolean indicating success or failure of the operation. The [error code](VECTOR_NAV_README.md#error_code) from the last operation is returned by this method, which can be useful in debugging if an operation fails.

```C++
if (!vn.Begin()) {
   Serial.println(vn.error_code());
}
```

**bool Begin()** Initializes communication with the VN-300 sensor. Returns true on successfully establishing communication and false on failure.

```C++
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

**float die_temperature_c()** Returns the sensor die temperature, C.

**float pressure_pa()** Returns the measured static pressure, Pa.

**bool WriteSettings()** Writes the current settings to the VectorNav non-volatile memory.

**void RestoreFactorySettings()** Restores the VectorNav to factory default settings.

**void Reset()** Resets the VectorNav.

**bool KnownMagneticDisturbance(bool present)** Notifies the VectorNav that a magnetic disturbance is present. The sensor will tune out the magnetometer and pause the current hard / soft iron calibration, if enabled. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool KnownAccelerationDisturbance(bool present)** Notifies the VectorNav that an acceleration disturbance is present. The sensor will tune out the accelerometer. A *true* should be passed if a disturbance is present and a *false* passed if a disturbance is no longer present.

**bool SetGyroBias()** Commands the VectorNav to copy the current gyro bias estimates into volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.

**bool SetFilterBias()** Commands the VectorNav to copy the current filter bias estimates ino volatile memory. These can then be saved in non-volatile memory using the *WriteSettings* command.
