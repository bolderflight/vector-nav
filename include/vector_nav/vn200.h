/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_VECTOR_NAV_VN200_H_
#define INCLUDE_VECTOR_NAV_VN200_H_

namespace sensors {

class Vn200 {
 public:
  Vn200();
  bool Begin();
  bool EnableDrdyInt(uint16_t srd);
  bool DisableDrdyInt();
  void rotation(Eigen::Matrix3f c);
  Eigen::Matrix3f rotation();
  void antenna_offset(Eigen::Vector3f b);
  Eigen::Vector3f antenna_offset();
  void dlpf_bandwidth(float hz);
  float dlpf_bandwidth();
  void DrdyCallback(uint8_t int_pin, void (*function)());
  bool Read();

 private:

};

}

#endif  // INCLUDE_VECTOR_NAV_VN200_H_
