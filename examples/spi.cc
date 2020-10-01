/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "vector_nav/vn100.h"

sensors::Vn100 vn(&SPI, 25);

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  bool status = vn.Begin();
  Serial.println(status);
  status = vn.SetAccelFilter(sensors::Vn100::FILTER_BOTH, 20);
  Serial.println(status);
  sensors::Vn100::FilterMode mode;
  uint16_t window;
  status = vn.GetAccelFilter(&mode, &window);
  Serial.println(status);
  Serial.println(mode);
  Serial.println(window);
}

