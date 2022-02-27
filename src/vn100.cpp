/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#else
#include "core/core.h"
#endif
#include "vn100.h"  // NOLINT
#include "eigen.h"  // NOLINT
#include "Eigen/Dense"
#include "vector_nav.h"  // NOLINT
#include "registers.h"  // NOLINT
#include "units.h"  // NOLINT

namespace bfs {

constexpr char Vn100::PROD_NAME_[];

bool Vn100::Begin() {
  vn_.Init();
  error_code_ = vn_.ReadRegister(&serial_num_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  if (serial_num_.payload.serial_num == 0) {
    error_code_ = VectorNav::ERROR_NO_COMM;
    return false;
  }
  error_code_ = vn_.ReadRegister(&model_num_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  for (std::size_t i = 0; i < sizeof(PROD_NAME_) - 1; i++) {
    if (model_num_.payload.product_name[i] != PROD_NAME_[i]) {
      error_code_ = VectorNav::ERROR_WRONG_MODEL;
      return false;
    }
  }
  return true;
}

bool Vn100::EnableDrdyInt(const DrdyMode mode, const uint16_t srd) {
  error_code_ = vn_.ReadRegister(&sync_cntrl_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  enum SyncOutPolarity : uint8_t {
    NEG_PULSE = 0,
    POS_PULSE = 1
  };
  sync_cntrl_.payload.sync_out_mode = static_cast<uint8_t>(mode);
  sync_cntrl_.payload.sync_out_polarity = POS_PULSE;
  sync_cntrl_.payload.sync_out_pulse_width = 500000;
  sync_cntrl_.payload.sync_out_skip_factor = srd;
  error_code_ = vn_.WriteRegister(sync_cntrl_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::DisableDrdyInt() {
  error_code_ = vn_.ReadRegister(&sync_cntrl_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  sync_cntrl_.payload.sync_out_mode = 0;
  error_code_ = vn_.WriteRegister(sync_cntrl_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::ApplyRotation(const Eigen::Matrix3f &c) {
  for (std::size_t m = 0; m < 3; m++) {
    for (std::size_t n = 0; n < 3; n++) {
      rotation_.payload.c[m][n] = c(m, n);
    }
  }
  error_code_ = vn_.WriteRegister(rotation_);
  vn_.WriteSettings();
  vn_.Reset();
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::GetRotation(Eigen::Matrix3f *c) {
  if (!c) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&rotation_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  for (std::size_t m = 0; m < 3; m++) {
    for (std::size_t n = 0; n < 3; n++) {
      (*c)(m, n) = rotation_.payload.c[m][n];
    }
  }
  return true;
}

bool Vn100::SetMagFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.mag_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.mag_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::GetMagFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.mag_filter_mode);
  *window = filter_.payload.mag_window_size;
  return true;
}

bool Vn100::SetAccelFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.accel_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.accel_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::GetAccelFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.accel_filter_mode);
  *window = filter_.payload.accel_window_size;
  return true;
}

bool Vn100::SetGyroFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.gyro_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.gyro_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::GetGyroFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.gyro_filter_mode);
  *window = filter_.payload.gyro_window_size;
  return true;
}

bool Vn100::SetTemperatureFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.temp_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.temp_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::GetTemperatureFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.temp_filter_mode);
  *window = filter_.payload.temp_window_size;
  return true;
}

bool Vn100::SetPressureFilter(const FilterMode mode, const uint16_t window) {
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  filter_.payload.pres_filter_mode = static_cast<uint8_t>(mode);
  filter_.payload.pres_window_size = window;
  error_code_ = vn_.WriteRegister(filter_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::GetPressureFilter(FilterMode *mode, uint16_t *window) {
  if ((!mode) || (!window)) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  error_code_ = vn_.ReadRegister(&filter_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  *mode = static_cast<FilterMode>(filter_.payload.pres_filter_mode);
  *window = filter_.payload.pres_window_size;
  return true;
}


bool Vn100::DrdyCallback(const uint8_t int_pin, void (*function)()) {
  if (!function) {
    error_code_ = VectorNav::ERROR_NULL_PTR;
    return false;
  }
  pinMode(int_pin, INPUT);
  attachInterrupt(int_pin, function, RISING);
  error_code_ = VectorNav::ERROR_SUCCESS;
  return true;
}

bool Vn100::VelocityCompensation(float speed_mps) {
  vel_comp_.payload.velocity_x = speed_mps;
  vel_comp_.payload.velocity_y = 0.0f;
  vel_comp_.payload.velocity_z = 0.0f;
  error_code_ = vn_.WriteRegister(vel_comp_);
  return (error_code_ == VectorNav::ERROR_SUCCESS);
}

bool Vn100::Read() {
  error_code_ = vn_.ReadRegister(&attitude_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  error_code_ = vn_.ReadRegister(&imu_);
  if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
  return true;
}

}  // namespace bfs
