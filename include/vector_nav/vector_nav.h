/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_VECTOR_NAV_VECTOR_NAV_H_
#define INCLUDE_VECTOR_NAV_VECTOR_NAV_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"

namespace sensors {

class VectorNav {
 public:
  VectorNav(SPIClass *bus, uint8_t cs);
  bool Begin();

 private:

};

}  // namespace sensors

#endif  // INCLUDE_VECTOR_NAV_VECTOR_NAV_H_
