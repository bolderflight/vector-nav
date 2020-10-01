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
  vn.KnownAccelerationDisturbance(true);
}

