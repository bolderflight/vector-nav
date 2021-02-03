/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "vector_nav/vector_nav.h"

sensors::Vn200 vn(&SPI, 10);

int main() {
  Serial.begin(115200);
  while(!Serial) {}
  /* Initialize communication */
  if (!vn.Begin()) {
    Serial.println(vn.error_code());
  }
  while (1) {
    /* Read sensor and print values */
    if (vn.Read()) {
      Serial.print(global::conversions::Rad_to_Deg(vn.yaw_rad()));
      Serial.print("\t");
      Serial.print(global::conversions::Rad_to_Deg(vn.pitch_rad()));
      Serial.print("\t");
      Serial.println(global::conversions::Rad_to_Deg(vn.roll_rad()));
    }
    delay(50);
  }
}

