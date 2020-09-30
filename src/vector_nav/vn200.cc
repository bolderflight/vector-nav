/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "vector_nav/vn200.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "vector_nav/vector_nav.h"
#include "vector_nav/registers.h"
#include "global_defs/global_defs.h"

namespace sensors {

bool Vn200::Begin() {
  vector_nav_.Init();
  vector_nav_.RestoreFactorySettings();
  vector_nav::common::SerialNumber serial_num;
  error_code_ = vector_nav_.ReadRegister(&serial_num);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn200::EnableDrdyInt(uint16_t srd) {
  vector_nav::common::SynchronizationControl sync_cntrl;
  error_code_ = vector_nav_.ReadRegister(&sync_cntrl);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  enum SyncOutMode : uint8_t {
    NONE = 0,
    IMU_START = 1,
    IMU_READY = 2,
    INS = 3,
    GPS_PPS = 6
  };
  enum SyncOutPolarity : uint8_t {
    NEG_PULSE = 0,
    POS_PULSE = 1
  };
  sync_cntrl.payload.sync_out_mode = INS;
  sync_cntrl.payload.sync_out_polarity = POS_PULSE;
  sync_cntrl.payload.sync_out_pulse_width = 500000;
  sync_cntrl.payload.sync_out_skip_factor = srd;
  error_code_ = vector_nav_.WriteRegister(sync_cntrl);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn200::DisableDrdyInt() {
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

bool Vn200::ApplyRotation(Eigen::Matrix3f c) {
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

bool Vn200::GetRotation(Eigen::Matrix3f *c) {
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

bool Vn200::SetAntennaOffset(Eigen::Vector3f b) {
  vector_nav::vn200::GpsAntennaOffset ant_offset;
  ant_offset.payload.position_x = b(0);
  ant_offset.payload.position_y = b(1);
  ant_offset.payload.position_z = b(2);
  error_code_ = vector_nav_.WriteRegister(ant_offset);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn200::GetAntennaOffset(Eigen::Vector3f *b) {
  if (!b) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;    
  }
  vector_nav::vn200::GpsAntennaOffset ant_offset;
  error_code_ = vector_nav_.ReadRegister(&ant_offset);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  (*b)(0) = ant_offset.payload.position_x;
  (*b)(1) = ant_offset.payload.position_y;
  (*b)(2) = ant_offset.payload.position_z;
  return true;
}

void Vn200::DrdyCallback(uint8_t int_pin, void (*function)()) {
  pinMode(int_pin, INPUT);
  attachInterrupt(int_pin, function, RISING);
}

bool Vn200::Read() {
  vector_nav::vn200::InsSolutionLla ins;
  vector_nav::vn200::GpsSolutionLla gnss;
  vector_nav::common::MagneticAccelerationAngularRates comp_imu;
  vector_nav::common::ImuMeasurements uncomp_imu;
  error_code_ = vector_nav_.ReadRegister(&ins);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vector_nav_.ReadRegister(&gnss);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vector_nav_.ReadRegister(&comp_imu);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vector_nav_.ReadRegister(&uncomp_imu);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  /* Store data */
  ins_time_s_ = ins.payload.time;
  ins_week_ = ins.payload.week;
  ins_mode_ = static_cast<InsMode>(ins.payload.status & 0x03);
  ins_gnss_fix_ = ins.payload.status & 0x04;
  bool ins_time_error = ins.payload.status & 0x08;
  bool ins_imu_error = ins.payload.status & 0x10;
  bool ins_mag_press_error = ins.payload.status & 0x20;
  bool ins_gnss_error = ins.payload.status & 0x40;
  ins_error_ = ins_time_error || ins_imu_error || ins_mag_press_error || ins_gnss_error;
  ins_lla_rad_m_(0) = global::conversions::Deg_to_Rad(ins.payload.latitude);
  ins_lla_rad_m_(1) = global::conversions::Deg_to_Rad(ins.payload.longitude);
  ins_lla_rad_m_(2) = ins.payload.altitude;
  ins_ned_vel_mps_(0) = ins.payload.ned_vel_x;
  ins_ned_vel_mps_(1) = ins.payload.ned_vel_y;
  ins_ned_vel_mps_(2) = ins.payload.ned_vel_z;
  ins_att_uncertainty_rad_ = global::conversions::Deg_to_Rad(ins.payload.att_uncertainty);
  ins_pos_uncertainty_m_ = ins.payload.pos_uncertainty;
  ins_vel_uncertainty_mps_ = ins.payload.vel_uncertainty;
  gnss_time_s_ = gnss.payload.time;
  gnss_week_ = gnss.payload.week;
  gnss_fix_ = static_cast<GnssFix>(gnss.payload.gps_fix);
  gnss_num_sv_ = gnss.payload.num_sats;
  gnss_lla_rad_m_(0) = global::conversions::Deg_to_Rad(gnss.payload.latitude);
  gnss_lla_rad_m_(1) = global::conversions::Deg_to_Rad(gnss.payload.longitude);
  gnss_lla_rad_m_(2) = gnss.payload.altitude;
  gnss_ned_vel_mps_(0) = gnss.payload.ned_vel_x;
  gnss_ned_vel_mps_(1) = gnss.payload.ned_vel_y;
  gnss_ned_vel_mps_(2) = gnss.payload.ned_vel_z;
  gnss_ned_acc_m_(0) = gnss.payload.north_acc;
  gnss_ned_acc_m_(1) = gnss.payload.east_acc;
  gnss_ned_acc_m_(2) = gnss.payload.vert_acc;
  gnss_speed_acc_mps_ = gnss.payload.speed_acc;
  gnss_time_acc_s_ = gnss.payload.time_acc;
  ypr_rad_(0) = global::conversions::Deg_to_Rad(ins.payload.yaw);
  ypr_rad_(1) = global::conversions::Deg_to_Rad(ins.payload.pitch);
  ypr_rad_(2) = global::conversions::Deg_to_Rad(ins.payload.roll);
  mag_ut_(0) = comp_imu.payload.mag_x / 100.0f;  // gauss to uT
  mag_ut_(1) = comp_imu.payload.mag_y / 100.0f;
  mag_ut_(2) = comp_imu.payload.mag_z / 100.0f;
  accel_mps2_(0) = comp_imu.payload.accel_x;
  accel_mps2_(1) = comp_imu.payload.accel_y;
  accel_mps2_(2) = comp_imu.payload.accel_z;
  gyro_radps_(0) = comp_imu.payload.gyro_x;
  gyro_radps_(1) = comp_imu.payload.gyro_y;
  gyro_radps_(2) = comp_imu.payload.gyro_z;
  uncomp_mag_ut_(0) = uncomp_imu.payload.mag_x / 100.0f;  // gauss to uT
  uncomp_mag_ut_(1) = uncomp_imu.payload.mag_y / 100.0f;
  uncomp_mag_ut_(2) = uncomp_imu.payload.mag_z / 100.0f;
  uncomp_accel_mps2_(0) = uncomp_imu.payload.accel_x;
  uncomp_accel_mps2_(1) = uncomp_imu.payload.accel_y;
  uncomp_accel_mps2_(2) = uncomp_imu.payload.accel_z;
  uncomp_gyro_radps_(0) = uncomp_imu.payload.gyro_x;
  uncomp_gyro_radps_(1) = uncomp_imu.payload.gyro_y;
  uncomp_gyro_radps_(2) = uncomp_imu.payload.gyro_z;
  die_temp_c_ = uncomp_imu.payload.temp;
  pressure_pa_ = uncomp_imu.payload.pressure * 1000.0f;  // kPa to Pa
  return true;
}

}  // namespace sensors
