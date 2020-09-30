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

namespace sensors {

class Vn100 {
 public:
  Vn100(SPIClass *bus, uint8_t cs) : vector_nav_(bus, cs) {}
  bool Begin();
  bool EnableDrdyInt(uint16_t srd);
  bool DisableDrdyInt();
  bool ApplyRotation(Eigen::Matrix3f c);
  bool GetRotation(Eigen::Matrix3f *c);
  // bool SetDlpfBandwidth(float hz);
  // bool GetDlpfBandwidth(float *hz);
  void DrdyCallback(uint8_t int_pin, void (*function)());
  bool Read();
  inline VectorNav::ErrorCode error_code() {return error_code_;}
  inline float yaw_rad() {return ypr_rad_(0);}
  inline float pitch_rad() {return ypr_rad_(1);}
  inline float roll_rad() {return ypr_rad_(2);}
  inline Eigen::Vector3f mag_ut() {return mag_ut_;}
  inline float mag_x_ut() {return mag_ut_(0);}
  inline float mag_y_ut() {return mag_ut_(1);}
  inline float mag_z_ut() {return mag_ut_(2);}
  inline Eigen::Vector3f accel_mps2() {return accel_mps2_;}
  inline float accel_x_mps2() {return accel_mps2_(0);}
  inline float accel_y_mps2() {return accel_mps2_(1);}
  inline float accel_z_mps2() {return accel_mps2_(2);}
  inline Eigen::Vector3f gyro_radps() {return gyro_radps_;}
  inline float gyro_x_radps() {return gyro_radps_(0);}
  inline float gyro_y_radps() {return gyro_radps_(1);}
  inline float gyro_z_radps() {return gyro_radps_(2);}
  inline Eigen::Vector3f uncomp_mag_ut() {return uncomp_mag_ut_;}
  inline float uncomp_mag_x_ut() {return uncomp_mag_ut_(0);}
  inline float uncomp_mag_y_ut() {return uncomp_mag_ut_(1);}
  inline float uncomp_mag_z_ut() {return uncomp_mag_ut_(2);}
  inline Eigen::Vector3f uncomp_accel_mps2() {return uncomp_accel_mps2_;}
  inline float uncomp_accel_x_mps2() {return uncomp_accel_mps2_(0);}
  inline float uncomp_accel_y_mps2() {return uncomp_accel_mps2_(1);}
  inline float uncomp_accel_z_mps2() {return uncomp_accel_mps2_(2);}
  inline Eigen::Vector3f uncomp_gyro_radps() {return uncomp_gyro_radps_;}
  inline float uncomp_gyro_x_radps() {return uncomp_gyro_radps_(0);}
  inline float uncomp_gyro_y_radps() {return uncomp_gyro_radps_(1);}
  inline float uncomp_gyro_z_radps() {return uncomp_gyro_radps_(2);}
  inline float die_temperature_c() {return die_temp_c_;}
  inline float pressure_pa() {return pressure_pa_;}

 private:
  /* Register reading and writing */
  VectorNav vector_nav_;
  /* Data */
  VectorNav::ErrorCode error_code_;
  Eigen::Vector3f ypr_rad_;
  Eigen::Vector3f mag_ut_;
  Eigen::Vector3f accel_mps2_;
  Eigen::Vector3f gyro_radps_;
  Eigen::Vector3f uncomp_mag_ut_;
  Eigen::Vector3f uncomp_accel_mps2_;
  Eigen::Vector3f uncomp_gyro_radps_;
  float die_temp_c_;
  float pressure_pa_;
};

}

#endif  // INCLUDE_VECTOR_NAV_VN100_H_
