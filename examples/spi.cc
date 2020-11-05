/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "vector_nav/vector_nav.h"

sensors::VectorNav vn(&SPI, 10);

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  vn.Init();
  sensors::vector_nav::common::SerialNumber sn;
  sensors::VectorNav::ErrorCode err = vn.ReadRegister(&sn);
  // if (err == sensors::VectorNav::ERROR_SUCCESS) {
  //   Serial.println(sn.payload.serial_num);
  // }
  // err = vn.WriteSettings();
  // Serial.println(err);
  // err = vn.Tare();
  // Serial.println(err);
  // err = vn.KnownAccelerationDisturbance(true);
  // Serial.println(err);
  // err = vn.KnownMagneticDisturbance(true);
  // Serial.println(err);
  // err = vn.KnownAccelerationDisturbance(false);
  // Serial.println(err);
  // err = vn.KnownMagneticDisturbance(false);
  // Serial.println(err);
  // err = vn.SetGyroBias();
  // Serial.println(err);
  // err = vn.SetFilterBias();
  // Serial.println(err);
  // vn.Reset();
  // vn.RestoreFactorySettings();
}

