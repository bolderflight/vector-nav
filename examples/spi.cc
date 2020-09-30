/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "vector_nav/vector_nav.h"
#include "vector_nav/registers.h"

#include <string>

sensors::VectorNav vn(&SPI, 25);

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  vn.Init();
  sensors::vector_nav::common::UserTag tag_write, tag_read;
  std::string tag = "my vector nav";


  strncpy(tag_write.payload.tag, tag.c_str(), sizeof(tag_write.payload.tag) - 1);


  bool status = vn.WriteRegister(tag_write);
  Serial.println(status);
  status = vn.ReadRegister(&tag_read);
  std::string ret(tag_read.payload.tag);
  
  Serial.println(status);
  Serial.println(ret);

  tag = "hola";


  strncpy(tag_write.payload.tag, tag.c_str(), sizeof(tag_write.payload.tag) - 1);


  status = vn.WriteRegister(tag_write);
  Serial.println(status);
  status = vn.ReadRegister(&tag_read);
  std::string ret2(tag_read.payload.tag);
  
  Serial.println(status);
  Serial.println(ret2);




  // status = vn.SetSyncInConfig(sensors::VectorNav::SyncInMode::COUNT, true, 9);
  // Serial.println(status);
  // Serial.println(vn.error_code());

  // sensors::VectorNav::SyncInMode mode;
  // bool rising_edge;
  // uint16_t srd;

  // status = vn.GetSyncInConfig(&mode, &rising_edge, &srd);
  // Serial.println(status);
  // Serial.println(mode);
  // Serial.println(rising_edge);
  // Serial.println(srd);

  // Serial.println();
  // Serial.println();
  // status = vn.SetSyncOutConfig(sensors::VectorNav::SyncOutMode::AHRS, true, 9, 100000);
  // Serial.println(status);
  // Serial.println(vn.error_code());

  // sensors::VectorNav::SyncOutMode out_mode;
  // bool pos_pulse;
  // uint16_t out_srd;
  // uint32_t pulse_width;

  // status = vn.GetSyncOutConfig(&out_mode, &pos_pulse, &out_srd, &pulse_width);
  // Serial.println(status);
  // Serial.println(out_mode);
  // Serial.println(pos_pulse);
  // Serial.println(out_srd);
  // Serial.println(pulse_width);

  // Serial.println();
  // status = vn.SetUserTag("Main IMU");
  // Serial.println(status);
  // std::string ret = "SOME VERY LONG STRING 123456789";
  
  // Serial.println();
  // status = vn.GetUserTag(&ret);
  // Serial.println(status);
  // Serial.println(ret);
  
  // Serial.println();
  // status = vn.GetModelNum(&ret);
  // Serial.println(status);
  // Serial.println(ret);

  // Serial.println();
  // uint32_t rev;
  // status = vn.GetHardwareRev(&rev);
  // Serial.println(status);
  // Serial.println(rev);

  // Serial.println();
  // uint32_t serial;
  // status = vn.GetSerialNum(&serial);
  // Serial.println(status);
  // Serial.println(serial);

  // Serial.println();
  // sensors::VectorNav::FirmwareVersion firmware;
  // status = vn.GetFirmwareVer(&firmware);
  // Serial.println(status);
  // Serial.println(firmware.major);
  // Serial.println(firmware.minor);
  // Serial.println(firmware.feature);
  // Serial.println(firmware.hotfix);


  // Serial.println(vn.error_code());
  // Serial.println(vn.hardware_rev());
  // Serial.println(vn.serial_number());
  // sensors::VectorNav::FirmwareVersion firmware = vn.firmware_version();
  // Serial.println(firmware.major);
  // Serial.println(firmware.minor);
  // Serial.println(firmware.feature);
  // Serial.println(firmware.hotfix);
}

