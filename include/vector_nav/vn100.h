/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems
*/

#ifndef INCLUDE_VECTOR_NAV_VN100_H_
#define INCLUDE_VECTOR_NAV_VN100_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "vector_nav/vn.h"
#include "vector_nav/registers.h"
#include "units/units.h"

namespace sensors {

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
  Vn100(SPIClass *bus, const uint8_t cs) : vector_nav_(bus, cs) {}
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
  bool WriteSettings() {return (vector_nav_.WriteSettings() == VectorNav::ERROR_SUCCESS);}
  void RestoreFactorySettings() {vector_nav_.RestoreFactorySettings();}
  void Reset() {vector_nav_.Reset();}
  bool Tare() {return (vector_nav_.Tare() == VectorNav::ERROR_SUCCESS);}
  bool KnownMagneticDisturbance(bool present) {return (vector_nav_.KnownMagneticDisturbance(present) == VectorNav::ERROR_SUCCESS);}
  bool KnownAccelerationDisturbance(bool present) {return (vector_nav_.KnownAccelerationDisturbance(present) == VectorNav::ERROR_SUCCESS);}
  bool SetGyroBias() {return (vector_nav_.SetGyroBias() == VectorNav::ERROR_SUCCESS);}

  /* Data */
  inline float yaw_rad() {return conversions::Deg_to_Rad(attitude_.payload.yaw);}
  inline float pitch_rad() {return conversions::Deg_to_Rad(attitude_.payload.pitch);}
  inline float roll_rad() {return conversions::Deg_to_Rad(attitude_.payload.roll);}
  inline float accel_x_mps2() {return attitude_.payload.accel_x;}
  inline float accel_y_mps2() {return attitude_.payload.accel_y;}
  inline float accel_z_mps2() {return attitude_.payload.accel_z;}
  Eigen::Vector3f accel_mps2();
  inline float gyro_x_radps() {return attitude_.payload.gyro_x;}
  inline float gyro_y_radps() {return attitude_.payload.gyro_y;}
  inline float gyro_z_radps() {return attitude_.payload.gyro_z;}
  Eigen::Vector3f gyro_radps();
  inline float mag_x_ut() {return conversions::Gauss_to_uT(attitude_.payload.mag_x);}
  inline float mag_y_ut() {return conversions::Gauss_to_uT(attitude_.payload.mag_y);}
  inline float mag_z_ut() {return conversions::Gauss_to_uT(attitude_.payload.mag_z);}
  Eigen::Vector3f mag_ut();
  inline float uncomp_accel_x_mps2() {return imu_.payload.accel_x;}
  inline float uncomp_accel_y_mps2() {return imu_.payload.accel_y;}
  inline float uncomp_accel_z_mps2() {return imu_.payload.accel_z;}
  Eigen::Vector3f uncomp_accel_mps2();
  inline float uncomp_gyro_x_radps() {return imu_.payload.gyro_x;}
  inline float uncomp_gyro_y_radps() {return imu_.payload.gyro_y;}
  inline float uncomp_gyro_z_radps() {return imu_.payload.gyro_z;}
  Eigen::Vector3f uncomp_gyro_radps();
  inline float uncomp_mag_x_ut() {return conversions::Gauss_to_uT(imu_.payload.mag_x);}
  inline float uncomp_mag_y_ut() {return conversions::Gauss_to_uT(imu_.payload.mag_y);}
  inline float uncomp_mag_z_ut() {return conversions::Gauss_to_uT(imu_.payload.mag_z);}
  Eigen::Vector3f uncomp_mag_ut();
  inline float die_temperature_c() {return imu_.payload.temp;}
  inline float pressure_pa() {return imu_.payload.pressure * 1000.0f;}  // kPa to Pa

 private:
  /* Register reading and writing */
  VectorNav vector_nav_;
  /* Registers */
  VectorNav::ErrorCode error_code_;
  vector_nav::common::SerialNumber serial_num_;
  vector_nav::common::SynchronizationControl sync_cntrl_;
  vector_nav::common::ReferenceFrameRotation rotation_;
  vector_nav::common::ImuFilteringConfiguration filter_;
  vector_nav::vn100::VelocityCompensationMeasurement vel_comp_;
  vector_nav::common::YawPitchRollMagneticAccelerationAngularRates attitude_;
  vector_nav::common::ImuMeasurements imu_;
};

}  // namespace sensors

#endif  // INCLUDE_VECTOR_NAV_VN100_H_
