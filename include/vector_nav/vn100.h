/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef INCLUDE_VECTOR_NAV_VN100_H_
#define INCLUDE_VECTOR_NAV_VN100_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "vector_nav/vn.h"
#include "vector_nav/registers.h"
#include "units/units.h"

namespace bfs {

class Vn100 {
 public:
  enum DrdyMode : uint8_t {
    IMU_START = 1,
    IMU_READY = 2,
    AHRS = 3
  };
  enum FilterMode : uint8_t {
    FILTER_NONE = 0,
    FILTER_UNCOMP_ONLY = 1,
    FILTER_COMP_ONLY = 2,
    FILTER_BOTH = 3
  };
  Vn100(SPIClass *bus, const uint8_t cs) : vn_(bus, cs) {}
  bool Begin();
  bool EnableDrdyInt(const DrdyMode mode, const uint16_t srd);
  bool DisableDrdyInt();
  bool ApplyRotation(const Eigen::Matrix3f &c);
  bool GetRotation(Eigen::Matrix3f *c);
  bool SetMagFilter(const FilterMode mode, const uint16_t window);
  bool GetMagFilter(FilterMode *mode, uint16_t *window);
  bool SetAccelFilter(const FilterMode mode, const uint16_t window);
  bool GetAccelFilter(FilterMode *mode, uint16_t *window);
  bool SetGyroFilter(const FilterMode mode, const uint16_t window);
  bool GetGyroFilter(FilterMode *mode, uint16_t *window);
  bool SetTemperatureFilter(const FilterMode mode, const uint16_t window);
  bool GetTemperatureFilter(FilterMode *mode, uint16_t *window);
  bool SetPressureFilter(const FilterMode mode, const uint16_t window);
  bool GetPressureFilter(FilterMode *mode, uint16_t *window);
  bool DrdyCallback(const uint8_t int_pin, void (*function)());
  bool VelocityCompensation(const float speed_mps);
  bool Read();
  inline VectorNav::ErrorCode error_code() {return error_code_;}

  /* Commands */
  bool WriteSettings() {
    return (vn_.WriteSettings() == VectorNav::ERROR_SUCCESS);
  }
  void RestoreFactorySettings() {
    vn_.RestoreFactorySettings();
  }
  void Reset() {
    vn_.Reset();
  }
  bool Tare() {
    return (vn_.Tare() == VectorNav::ERROR_SUCCESS);
  }
  bool KnownMagneticDisturbance(const bool present) {
    return (vn_.KnownMagneticDisturbance(present) == VectorNav::ERROR_SUCCESS);
  }
  bool KnownAccelerationDisturbance(const bool present) {
    return (vn_.KnownAccelerationDisturbance(present) ==
           VectorNav::ERROR_SUCCESS);
  }
  bool SetGyroBias() {
    return (vn_.SetGyroBias() == VectorNav::ERROR_SUCCESS);
  }

  /* Data */
  inline float yaw_rad() const {
    return deg2rad(attitude_.payload.yaw);
  }
  inline float pitch_rad() const {
    return deg2rad(attitude_.payload.pitch);
  }
  inline float roll_rad() const {
    return deg2rad(attitude_.payload.roll);
  }
  inline float accel_x_mps2() const {
    return attitude_.payload.accel_x;
  }
  inline float accel_y_mps2() const {
    return attitude_.payload.accel_y;
  }
  inline float accel_z_mps2() const {
    return attitude_.payload.accel_z;
  }
  inline Eigen::Vector3f accel_mps2() const {
    Eigen::Vector3f accel;
    accel(0) = attitude_.payload.accel_x;
    accel(1) = attitude_.payload.accel_y;
    accel(2) = attitude_.payload.accel_z;
    return accel;
  }
  inline float gyro_x_radps() const {
    return attitude_.payload.gyro_x;
  }
  inline float gyro_y_radps() const {
    return attitude_.payload.gyro_y;
  }
  inline float gyro_z_radps() const {
    return attitude_.payload.gyro_z;
  }
  inline Eigen::Vector3f gyro_radps() const {
    Eigen::Vector3f gyro;
    gyro(0) = attitude_.payload.gyro_x;
    gyro(1) = attitude_.payload.gyro_y;
    gyro(2) = attitude_.payload.gyro_z;
    return gyro;
  }
  inline float mag_x_ut() const {
    return attitude_.payload.mag_x * 100.0f;
  }
  inline float mag_y_ut() const {
    return attitude_.payload.mag_y * 100.0f;
  }
  inline float mag_z_ut() const {
    return attitude_.payload.mag_z * 100.0f;
  }
  inline Eigen::Vector3f mag_ut() const {
    Eigen::Vector3f mag;
    mag(0) = attitude_.payload.mag_x * 100.0f;
    mag(1) = attitude_.payload.mag_y * 100.0f;
    mag(2) = attitude_.payload.mag_z * 100.0f;
    return mag;
  }
  inline float uncomp_accel_x_mps2() const {
    return imu_.payload.accel_x;
  }
  inline float uncomp_accel_y_mps2() const {
    return imu_.payload.accel_y;
  }
  inline float uncomp_accel_z_mps2() const {
    return imu_.payload.accel_z;
  }
  inline Eigen::Vector3f uncomp_accel_mps2() const {
    Eigen::Vector3f accel;
    accel(0) = imu_.payload.accel_x;
    accel(1) = imu_.payload.accel_y;
    accel(2) = imu_.payload.accel_z;
    return accel;
  }
  inline float uncomp_gyro_x_radps() const {
    return imu_.payload.gyro_x;
  }
  inline float uncomp_gyro_y_radps() const {
    return imu_.payload.gyro_y;
  }
  inline float uncomp_gyro_z_radps() const {
    return imu_.payload.gyro_z;
  }
  inline Eigen::Vector3f uncomp_gyro_radps() const {
    Eigen::Vector3f gyro;
    gyro(0) = imu_.payload.gyro_x;
    gyro(1) = imu_.payload.gyro_y;
    gyro(2) = imu_.payload.gyro_z;
    return gyro;
  }
  inline float uncomp_mag_x_ut() const {
    return imu_.payload.mag_x * 100.0f;
  }
  inline float uncomp_mag_y_ut() const {
    return imu_.payload.mag_y * 100.0f;
  }
  inline float uncomp_mag_z_ut() const {
    return imu_.payload.mag_z * 100.0f;
  }
  inline Eigen::Vector3f uncomp_mag_ut() const {
    Eigen::Vector3f mag;
    mag(0) = imu_.payload.mag_x * 100.0f;
    mag(1) = imu_.payload.mag_y * 100.0f;
    mag(2) = imu_.payload.mag_z * 100.0f;
    return mag;
  }
  inline float die_temperature_c() const {
    return imu_.payload.temp;
  }
  inline float pressure_pa() const {
    return imu_.payload.pressure * 1000.0f;
  }

 private:
  /* Register reading and writing */
  VectorNav vn_;
  /* Registers */
  VectorNav::ErrorCode error_code_;
  VnSerialNumber serial_num_;
  VnSynchronizationControl sync_cntrl_;
  VnReferenceFrameRotation rotation_;
  VnImuFilteringConfiguration filter_;
  VnVelocityCompensationMeasurement vel_comp_;
  VnYawPitchRollMagneticAccelerationAngularRates attitude_;
  VnImuMeasurements imu_;
};

}  // namespace bfs

#endif  // INCLUDE_VECTOR_NAV_VN100_H_
