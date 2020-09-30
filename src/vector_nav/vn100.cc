/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "vector_nav/vn100.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "vector_nav/vector_nav.h"
#include "vector_nav/registers.h"
#include "global_defs/global_defs.h"

namespace sensors {

bool Vn100::Begin() {
  vector_nav_.Init();
  vector_nav_.RestoreFactorySettings();
  vector_nav::common::SerialNumber serial_num;
  error_code_ = vector_nav_.ReadRegister(&serial_num);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::EnableDrdyInt(uint16_t srd) {
  vector_nav::common::SynchronizationControl sync_cntrl;
  error_code_ = vector_nav_.ReadRegister(&sync_cntrl);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  enum SyncOutMode : uint8_t {
    NONE = 0,
    IMU_START = 1,
    IMU_READY = 2,
    AHRS = 3
  };
  enum SyncOutPolarity : uint8_t {
    NEG_PULSE = 0,
    POS_PULSE = 1
  };
  sync_cntrl.payload.sync_out_mode = AHRS;
  sync_cntrl.payload.sync_out_polarity = POS_PULSE;
  sync_cntrl.payload.sync_out_pulse_width = 500000;
  sync_cntrl.payload.sync_out_skip_factor = srd;
  error_code_ = vector_nav_.WriteRegister(sync_cntrl);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::DisableDrdyInt() {
  vector_nav::common::SynchronizationControl sync_cntrl;
  error_code_ = vector_nav_.ReadRegister(&sync_cntrl);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  enum SyncOutMode : uint8_t {
    NONE = 0,
    IMU_START = 1,
    IMU_READY = 2,
    AHRS = 3
  };
  sync_cntrl.payload.sync_out_mode = NONE;
  error_code_ = vector_nav_.WriteRegister(sync_cntrl);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::ApplyRotation(Eigen::Matrix3f c) {
  vector_nav::common::ReferenceFrameRotation rot;
  for (std::size_t m = 0; m < 3; m++) {
    for (std::size_t n = 0; n < 3; n++) {
      rot.payload.c[m][n] = c(m, n);
    }
  }
  error_code_ = vector_nav_.WriteRegister(rot);
  vector_nav_.WriteSettings();
  vector_nav_.Reset();
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::GetRotation(Eigen::Matrix3f *c) {
  if (!c) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  vector_nav::common::ReferenceFrameRotation rot;
  error_code_ = vector_nav_.ReadRegister(&rot);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  for (std::size_t m = 0; m < 3; m++) {
    for (std::size_t n = 0; n < 3; n++) {
      (*c)(m, n) = rot.payload.c[m][n];
    }
  }
  return true;
}

// bool Vn100::SetDlpfBandwidth(float hz) {
//   enum ImuFilteringModes : uint8_t {
//     NONE = 0,
//     UNCOMP_ONLY = 1,
//     COMP_ONLY = 2,
//     BOTH = 3
//   };
//   static constexpr float fs_hz = 800.0f;
//   float f = hz / fs_hz;
//   uint16_t window = ceil(static_cast<uint16_t>(sqrtf(0.196202f + f * f) / f));
//   vector_nav::common::ImuFilteringConfiguration filter;
//   filter.payload.mag_window_size = window;
//   filter.payload.accel_window_size = window;
//   filter.payload.gyro_window_size = window;
//   filter.payload.pres_window_size = window;
//   filter.payload.temp_window_size = window;
//   filter.payload.mag_filter_mode = BOTH;
//   filter.payload.accel_filter_mode = BOTH;
//   filter.payload.gyro_filter_mode = BOTH;
//   filter.payload.pres_filter_mode = UNCOMP_ONLY;
//   filter.payload.temp_filter_mode = UNCOMP_ONLY;
//   error_code_ = vector_nav_.WriteRegister(filter);
//   return (error_code_ == VectorNav::ERROR_SUCCESS);
// }

// bool Vn100::GetDlpfBandwidth(float *hz) {
//   if (!hz) {
//     error_code_ = VectorNav::ERROR_NULL_PTR;
//     return false;
//   }
//   vector_nav::common::ImuFilteringConfiguration filter;
//   error_code_ = vector_nav_.ReadRegister(&filter);
//   if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
//   float window = static_cast<float>(filter.payload.mag_window_size);
//   float f = 0.442947f / sqrtf(window * window - 1.0f);

// }

void Vn100::DrdyCallback(uint8_t int_pin, void (*function)()) {
  pinMode(int_pin, INPUT);
  attachInterrupt(int_pin, function, RISING);
}

bool Vn100::Read() {
  vector_nav::common::YawPitchRollMagneticAccelerationAngularRates attitude;
  vector_nav::common::ImuMeasurements imu;
  error_code_ = vector_nav_.ReadRegister(&attitude);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vector_nav_.ReadRegister(&imu);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  /* Store data */
  ypr_rad_(0) = global::conversions::Deg_to_Rad(attitude.payload.yaw);
  ypr_rad_(1) = global::conversions::Deg_to_Rad(attitude.payload.pitch);
  ypr_rad_(2) = global::conversions::Deg_to_Rad(attitude.payload.roll);
  mag_ut_(0) = attitude.payload.mag_x / 100.0f;  // gauss to uT
  mag_ut_(1) = attitude.payload.mag_y / 100.0f;
  mag_ut_(2) = attitude.payload.mag_z / 100.0f;
  accel_mps2_(0) = attitude.payload.accel_x;
  accel_mps2_(1) = attitude.payload.accel_y;
  accel_mps2_(2) = attitude.payload.accel_z;
  gyro_radps_(0) = attitude.payload.gyro_x;
  gyro_radps_(1) = attitude.payload.gyro_y;
  gyro_radps_(2) = attitude.payload.gyro_z;
  uncomp_mag_ut_(0) = imu.payload.mag_x / 100.0f;  // gauss to uT
  uncomp_mag_ut_(1) = imu.payload.mag_y / 100.0f;
  uncomp_mag_ut_(2) = imu.payload.mag_z / 100.0f;
  uncomp_accel_mps2_(0) = imu.payload.accel_x;
  uncomp_accel_mps2_(1) = imu.payload.accel_y;
  uncomp_accel_mps2_(2) = imu.payload.accel_z;
  uncomp_gyro_radps_(0) = imu.payload.gyro_x;
  uncomp_gyro_radps_(1) = imu.payload.gyro_y;
  uncomp_gyro_radps_(2) = imu.payload.gyro_z;
  die_temp_c_ = imu.payload.temp;
  pressure_pa_ = imu.payload.pressure * 1000.0f;  // kPa to Pa
  return true;
}

}  // namespace sensors
