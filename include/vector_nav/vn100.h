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
  bool SetDlpfBandwidth(float hz);
  bool GetDlpfBandwidth(float *hz);
  void DrdyCallback(uint8_t int_pin, void (*function)());
  bool Read();
  inline VectorNav::ErrorCode error_code() {return error_code_;}

 private:
  VectorNav vector_nav_;
  VectorNav::ErrorCode error_code_;
};

}

#endif  // INCLUDE_VECTOR_NAV_VN100_H_
