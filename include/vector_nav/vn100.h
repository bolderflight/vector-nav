/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_VECTOR_NAV_VN100_H_
#define INCLUDE_VECTOR_NAV_VN100_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "vector_nav/vector_nav.h"
#include "vector_nav/registers.h"
#include "global_defs/global_defs.h"

namespace sensors {

class Vn100 {
 public:
  enum DrdyMode : uint8_t {
    NONE = 0,
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
  Vn100(SPIClass *bus, uint8_t cs) : vector_nav_(bus, cs) {}
  bool Begin();
  bool EnableDrdyInt(DrdyMode mode, uint16_t srd);
  bool DisableDrdyInt();
  bool ApplyRotation(Eigen::Matrix3f c);
  bool GetRotation(Eigen::Matrix3f *c);
  bool SetMagFilter(FilterMode mode, uint16_t window);
  bool GetMagFilter(FilterMode *mode, uint16_t *window);
  bool SetAccelFilter(FilterMode mode, uint16_t window);
  bool GetAccelFilter(FilterMode *mode, uint16_t *window);
  bool SetGyroFilter(FilterMode mode, uint16_t window);
  bool GetGyroFilter(FilterMode *mode, uint16_t *window);
  bool SetTemperatureFilter(FilterMode mode, uint16_t window);
  bool GetTemperatureFilter(FilterMode *mode, uint16_t *window);
  bool SetPressureFilter(FilterMode mode, uint16_t window);
  bool GetPressureFilter(FilterMode *mode, uint16_t *window);
  void DrdyCallback(uint8_t int_pin, void (*function)());
  bool VelocityCompensation(float speed_mps);
  bool Read();

  /* Commands */
  VectorNav::ErrorCode WriteSettings() {return vector_nav_.WriteSettings();}
  void RestoreFactorySettings() {vector_nav_.RestoreFactorySettings();}
  void Reset() {vector_nav_.Reset();}
  VectorNav::ErrorCode Tare() {return vector_nav_.Tare();}
  VectorNav::ErrorCode KnownMagneticDisturbance(bool present) {return vector_nav_.KnownMagneticDisturbance(present);}
  VectorNav::ErrorCode KnownAccelerationDisturbance(bool present) {return vector_nav_.KnownAccelerationDisturbance(present);}
  VectorNav::ErrorCode SetGyroBias() {return vector_nav_.SetGyroBias();}

  /* Data */
  inline float yaw_rad() {return global::conversions::Deg_to_Rad(attitude_.payload.yaw);}
  inline float pitch_rad() {return global::conversions::Deg_to_Rad(attitude_.payload.pitch);}
  inline float roll_rad() {return global::conversions::Deg_to_Rad(attitude_.payload.roll);}
  inline float accel_x_mps2() {return attitude_.payload.accel_x;}
  inline float accel_y_mps2() {return attitude_.payload.accel_y;}
  inline float accel_z_mps2() {return attitude_.payload.accel_z;}
  Eigen::Vector3f accel_mps2();
  inline float gyro_x_radps() {return attitude_.payload.gyro_x;}
  inline float gyro_y_radps() {return attitude_.payload.gyro_y;}
  inline float gyro_z_radps() {return attitude_.payload.gyro_z;}
  Eigen::Vector3f gyro_radps();
  inline float mag_x_ut() {return global::conversions::Gauss_to_uT(attitude_.payload.mag_x);}
  inline float mag_y_ut() {return global::conversions::Gauss_to_uT(attitude_.payload.mag_y);}
  inline float mag_z_ut() {return global::conversions::Gauss_to_uT(attitude_.payload.mag_z);}
  Eigen::Vector3f mag_ut();
  inline float uncomp_accel_x_mps2() {return imu_.payload.accel_x;}
  inline float uncomp_accel_y_mps2() {return imu_.payload.accel_y;}
  inline float uncomp_accel_z_mps2() {return imu_.payload.accel_z;}
  Eigen::Vector3f uncomp_accel_mps2();
  inline float uncomp_gyro_x_radps() {return imu_.payload.gyro_x;}
  inline float uncomp_gyro_y_radps() {return imu_.payload.gyro_y;}
  inline float uncomp_gyro_z_radps() {return imu_.payload.gyro_z;}
  Eigen::Vector3f uncomp_gyro_radps();
  inline float uncomp_mag_x_ut() {return global::conversions::Gauss_to_uT(imu_.payload.mag_x);}
  inline float uncomp_mag_y_ut() {return global::conversions::Gauss_to_uT(imu_.payload.mag_y);}
  inline float uncomp_mag_z_ut() {return global::conversions::Gauss_to_uT(imu_.payload.mag_z);}
  Eigen::Vector3f uncomp_mag_ut();
  inline float die_temperature_c() {return imu_.payload.temp;}
  inline float pressure_pa() {return imu_.payload.pressure / 1000.0f;}  // kPa to Pa

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

}

#endif  // INCLUDE_VECTOR_NAV_VN100_H_
