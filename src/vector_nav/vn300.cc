/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "vector_nav/vn300.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "vector_nav/vector_nav.h"
#include "vector_nav/registers.h"
#include "global_defs/global_defs.h"

namespace sensors {

bool Vn300::Begin() {
  vector_nav_.Init();
  error_code_ = vector_nav_.ReadRegister(&serial_num_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::EnableDrdyInt(const DrdyMode mode, const uint16_t srd) {
  error_code_ = vector_nav_.ReadRegister(&sync_cntrl_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  enum SyncOutPolarity : uint8_t {
    NEG_PULSE = 0,
    POS_PULSE = 1
  };
  sync_cntrl_.payload.sync_out_mode = static_cast<uint8_t>(mode);
  sync_cntrl_.payload.sync_out_polarity = POS_PULSE;
  sync_cntrl_.payload.sync_out_pulse_width = 500000;
  sync_cntrl_.payload.sync_out_skip_factor = srd;
  error_code_ = vector_nav_.WriteRegister(sync_cntrl_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::DisableDrdyInt() {
  error_code_ = vector_nav_.ReadRegister(&sync_cntrl_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  sync_cntrl_.payload.sync_out_mode = 0;
  error_code_ = vector_nav_.WriteRegister(sync_cntrl_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::ApplyRotation(const Eigen::Matrix3f &c) {
  for (std::size_t m = 0; m < 3; m++) {
    for (std::size_t n = 0; n < 3; n++) {
      rotation_.payload.c[m][n] = c(m, n);
    }
  }
  error_code_ = vector_nav_.WriteRegister(rotation_);
  vector_nav_.WriteSettings();
  vector_nav_.Reset();
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetRotation(Eigen::Matrix3f *c) {
  if (!c) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vector_nav_.ReadRegister(&rotation_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  for (std::size_t m = 0; m < 3; m++) {
    for (std::size_t n = 0; n < 3; n++) {
      (*c)(m, n) = rotation_.payload.c[m][n];
    }
  }
  return true;
}

bool Vn300::SetAntennaOffset(const Eigen::Vector3f &b) {
  antenna_.payload.position_x = b(0);
  antenna_.payload.position_y = b(1);
  antenna_.payload.position_z = b(2);
  error_code_ = vector_nav_.WriteRegister(antenna_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetAntennaOffset(Eigen::Vector3f *b) {
  if (!b) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;    
  }
  error_code_ = vector_nav_.ReadRegister(&antenna_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  (*b)(0) = antenna_.payload.position_x;
  (*b)(1) = antenna_.payload.position_y;
  (*b)(2) = antenna_.payload.position_z;
  return true;
}

bool Vn300::SetCompassBaseline(const Eigen::Vector3f &pos, const Eigen::Vector3f &uncert) {
  baseline_.payload.position_x = pos(0);
  baseline_.payload.position_y = pos(1);
  baseline_.payload.position_z = pos(2);
  baseline_.payload.uncertainty_x = uncert(0);
  baseline_.payload.uncertainty_y = uncert(1);
  baseline_.payload.uncertainty_z = uncert(2);
  error_code_ = vector_nav_.WriteRegister(baseline_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetCompassBaseline(Eigen::Vector3f *pos, Eigen::Vector3f *uncert) {
  if ((!pos) || (!uncert)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;    
  }
  error_code_ = vector_nav_.ReadRegister(&baseline_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  (*pos)(0) = baseline_.payload.position_x;
  (*pos)(1) = baseline_.payload.position_y;
  (*pos)(2) = baseline_.payload.position_z;
  (*uncert)(0) = baseline_.payload.uncertainty_x;
  (*uncert)(1) = baseline_.payload.uncertainty_y;
  (*uncert)(2) = baseline_.payload.uncertainty_z;
  return true;
}

bool Vn300::SetMagFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.mag_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.mag_window_size = window;
  error_code_ = vector_nav_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetMagFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.mag_filter_mode);
  *window = filter_.payload.mag_window_size;
  return true;
}

bool Vn300::SetAccelFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.accel_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.accel_window_size = window;
  error_code_ = vector_nav_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetAccelFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.accel_filter_mode);
  *window = filter_.payload.accel_window_size;
  return true;
}

bool Vn300::SetGyroFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.gyro_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.gyro_window_size = window;
  error_code_ = vector_nav_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetGyroFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.gyro_filter_mode);
  *window = filter_.payload.gyro_window_size;
  return true;
}

bool Vn300::SetTemperatureFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.temp_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.temp_window_size = window;
  error_code_ = vector_nav_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetTemperatureFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.temp_filter_mode);
  *window = filter_.payload.temp_window_size;
  return true;
}

bool Vn300::SetPressureFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.pres_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.pres_window_size = window;
  error_code_ = vector_nav_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn300::GetPressureFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vector_nav_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.pres_filter_mode);
  *window = filter_.payload.pres_window_size;
  return true;
}

bool Vn300::DrdyCallback(const uint8_t int_pin, void (*function)()) {
  if (!function) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;    
  }
  pinMode(int_pin, INPUT);
  attachInterrupt(int_pin, function, RISING);
  error_code_ == VectorNav::ERROR_SUCCESS;
  return true;
}

bool Vn300::Read() {
  error_code_ = vector_nav_.ReadRegister(&ins_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vector_nav_.ReadRegister(&gnss_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vector_nav_.ReadRegister(&comp_imu_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vector_nav_.ReadRegister(&uncomp_imu_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  /* INS status parsing */
  ins_status_buff_[0] = ins_.payload.status & 0xFF;
  ins_status_buff_[1] = ins_.payload.status >> 8 & 0xFF;
  ins_mode_ = static_cast<InsMode>(ins_status_buff_[0] & 0x03);
  ins_gnss_fix_ = ins_status_buff_[0] & 0x04;
  ins_time_error_ = ins_status_buff_[0] & 0x08;
  ins_imu_error_ = ins_status_buff_[0] & 0x10;
  ins_mag_press_error_ = ins_status_buff_[0] & 0x20;
  ins_gnss_error_ = ins_status_buff_[0] & 0x40;
  ins_error_ = ins_time_error_ || ins_imu_error_ || ins_mag_press_error_ || ins_gnss_error_;
  ins_gnss_heading_ = ins_status_buff_[1] & 0x01;
  ins_gnss_compass_ = ins_status_buff_[1] & 0x02;
  return true;
}

Eigen::Vector3d Vn300::ins_lla_rad_m() {
  Eigen::Vector3d lla;
  lla(0) = global::conversions::Deg_to_Rad(ins_.payload.latitude);
  lla(1) = global::conversions::Deg_to_Rad(ins_.payload.longitude);
  lla(2) = ins_.payload.altitude;
  return lla;
}

Eigen::Vector3f Vn300::ins_ned_vel_mps() {
  Eigen::Vector3f ned_vel;
  ned_vel(0) = ins_.payload.ned_vel_x;
  ned_vel(1) = ins_.payload.ned_vel_y;
  ned_vel(2) = ins_.payload.ned_vel_z;
  return ned_vel;
}

Eigen::Vector3d Vn300::gnss_lla_rad_m() {
  Eigen::Vector3d lla;
  lla(0) = global::conversions::Deg_to_Rad(gnss_.payload.latitude);
  lla(1) = global::conversions::Deg_to_Rad(gnss_.payload.longitude);
  lla(2) = gnss_.payload.altitude;
  return lla;
}

Eigen::Vector3f Vn300::gnss_ned_vel_mps() {
  Eigen::Vector3f ned_vel;
  ned_vel(0) = gnss_.payload.ned_vel_x;
  ned_vel(1) = gnss_.payload.ned_vel_y;
  ned_vel(2) = gnss_.payload.ned_vel_z;
  return ned_vel;
}

Eigen::Vector3f Vn300::accel_mps2() {
  Eigen::Vector3f accel;
  accel(0) = comp_imu_.payload.accel_x;
  accel(1) = comp_imu_.payload.accel_y;
  accel(2) = comp_imu_.payload.accel_z;
  return accel;
}

Eigen::Vector3f Vn300::gyro_radps() {
  Eigen::Vector3f gyro;
  gyro(0) = comp_imu_.payload.gyro_x;
  gyro(1) = comp_imu_.payload.gyro_y;
  gyro(2) = comp_imu_.payload.gyro_z;
  return gyro;
}

Eigen::Vector3f Vn300::mag_ut() {
  Eigen::Vector3f mag;
  mag(0) = global::conversions::Gauss_to_uT(comp_imu_.payload.mag_x);
  mag(1) = global::conversions::Gauss_to_uT(comp_imu_.payload.mag_y);
  mag(2) = global::conversions::Gauss_to_uT(comp_imu_.payload.mag_z);
  return mag;
}

Eigen::Vector3f Vn300::uncomp_accel_mps2() {
  Eigen::Vector3f accel;
  accel(0) = uncomp_imu_.payload.accel_x;
  accel(1) = uncomp_imu_.payload.accel_y;
  accel(2) = uncomp_imu_.payload.accel_z;
  return accel;
}

Eigen::Vector3f Vn300::uncomp_gyro_radps() {
  Eigen::Vector3f gyro;
  gyro(0) = uncomp_imu_.payload.gyro_x;
  gyro(1) = uncomp_imu_.payload.gyro_y;
  gyro(2) = uncomp_imu_.payload.gyro_z;
  return gyro;
}

Eigen::Vector3f Vn300::uncomp_mag_ut() {
  Eigen::Vector3f mag;
  mag(0) = global::conversions::Gauss_to_uT(uncomp_imu_.payload.mag_x);
  mag(1) = global::conversions::Gauss_to_uT(uncomp_imu_.payload.mag_y);
  mag(2) = global::conversions::Gauss_to_uT(uncomp_imu_.payload.mag_z);
  return mag;
}

}  // namespace sensors
