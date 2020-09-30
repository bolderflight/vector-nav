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

bool Vn100::SetDlpfBandwidth(float hz) {

}

bool Vn100::GetDlpfBandwidth(float *hz) {

}

void Vn100::DrdyCallback(uint8_t int_pin, void (*function)()) {

}

bool Vn100::Read() {

}

}  // namespace sensors
