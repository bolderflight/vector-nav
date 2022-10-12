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

#ifndef VECTOR_NAV_SRC_VN300_H_  // NOLINT
#define VECTOR_NAV_SRC_VN300_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "vn.h"  // NOLINT
#include "registers.h"  // NOLINT

namespace bfs {

class Vn300 {
 public:
  enum DrdyMode : uint8_t {
    IMU_START = 1,
    IMU_READY = 2,
    INS = 3,
    GNSS_PPS = 6
  };
  enum FilterMode : uint8_t {
    FILTER_NONE = 0,
    FILTER_UNCOMP_ONLY = 1,
    FILTER_COMP_ONLY = 2,
    FILTER_BOTH = 3
  };
  enum InsMode : uint8_t {
    NOT_TRACKING = 0,
    DEGRADED = 1,
    HEALTHY = 2,
    GNSS_LOSS = 3
  };
  enum GnssFix : uint8_t {
    FIX_NONE = 0,
    FIX_TIME_ONLY = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_SBAS = 4
  };
  Vn300() {}
  Vn300(SPIClass *bus, const uint8_t cs) : vn_(bus, cs) {}
  void Config(SPIClass *bus, const uint8_t cs) {vn_.Config(bus, cs);}
  bool Begin();
  bool EnableDrdyInt(const DrdyMode mode, const uint16_t srd);
  bool DisableDrdyInt();
  template<size_t M, size_t N>
  bool ApplyRotation(const float (&c)[M][N]) {
    static_assert(M == 3, "Expecting 3 x 3 matrix");
    static_assert(N == 3, "Expecting 3 x 3 matrix");
    for (int8_t m = 0; m < M; m++) {
      for (int8_t n = 0; n < N; n++) {
        rotation_.payload.c[m][n] = c[m][n];
      }
    }
    error_code_ = vn_.WriteRegister(rotation_);
    vn_.WriteSettings();
    vn_.Reset();
    return (error_code_ == VectorNav::ERROR_SUCCESS);
  }
  template<size_t M, size_t N>
  bool GetRotation(float (&c)[M][N]) {
    static_assert(M == 3, "Expecting 3 x 3 matrix");
    static_assert(N == 3, "Expecting 3 x 3 matrix");
    error_code_ = vn_.ReadRegister(&rotation_);
    if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
    for (int8_t m = 0; m < M; m++) {
      for (int8_t n = 0; n < N; n++) {
        c[m][n] = rotation_.payload.c[m][n];
      }
    }
    return true;
  }
  template<size_t M>
  bool SetAntennaOffset(const float (&b)[M]) {
    static_assert(M == 3, "Expecting 3 x 1 vector");
    antenna_.payload.position_x = b[0];
    antenna_.payload.position_y = b[1];
    antenna_.payload.position_z = b[2];
    error_code_ = vn_.WriteRegister(antenna_);
    return (error_code_ == VectorNav::ERROR_SUCCESS);
  }
  template<size_t M>
  bool GetAntennaOffset(float (&b)[M]) {
    static_assert(M == 3, "Expecting 3 x 1 vector");
    error_code_ = vn_.ReadRegister(&antenna_);
    if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
    b[0] = antenna_.payload.position_x;
    b[1] = antenna_.payload.position_y;
    b[2] = antenna_.payload.position_z;
    return true;
  }
  template<size_t M>
  bool SetCompassBaseline(const float (&pos)[M],
                          const float (&uncert)[M]) {
    static_assert(M == 3, "Expecting 3 x 1 vector");
    baseline_.payload.position_x = pos[0];
    baseline_.payload.position_y = pos[1];
    baseline_.payload.position_z = pos[2];
    baseline_.payload.uncertainty_x = uncert[0];
    baseline_.payload.uncertainty_y = uncert[1];
    baseline_.payload.uncertainty_z = uncert[2];
    error_code_ = vn_.WriteRegister(baseline_);
    return (error_code_ == VectorNav::ERROR_SUCCESS);
  }
  template<size_t M>
  bool GetCompassBaseline(float (&pos)[M], float (&uncert)[M]) {
    static_assert(M == 3, "Expecting 3 x 1 vector");
    error_code_ = vn_.ReadRegister(&baseline_);
    if (error_code_ != VectorNav::ERROR_SUCCESS) {return false;}
    pos[0] = baseline_.payload.position_x;
    pos[1] = baseline_.payload.position_y;
    pos[2] = baseline_.payload.position_z;
    uncert[0] = baseline_.payload.uncertainty_x;
    uncert[1] = baseline_.payload.uncertainty_y;
    uncert[2] = baseline_.payload.uncertainty_z;
    return true;
  }
  bool SetMagFilter(const FilterMode mode, const uint16_t window);
  bool GetMagFilter(FilterMode *mode, uint16_t *window);
  bool SetAccelFilter(const FilterMode mode, const uint16_t window);
  bool GetAccelFilter(FilterMode *mode, uint16_t *window);
  bool SetGyroFilter(const FilterMode mode, const uint16_t window);
  bool GetGyroFilter(FilterMode *mode, uint16_t *window);
  bool SetTemperatureFilter(const FilterMode mode, const uint16_t window);
  bool GetTemperatureFilter(FilterMode *mode, uint16_t *window);
  bool SetPressureFilter(const FilterMode mode, const uint16_t window);
  bool GetPressureFilter(FilterMode *mode, uint16_t *window);
  bool DrdyCallback(const uint8_t int_pin, void (*function)());
  bool Read();
  inline VectorNav::ErrorCode error_code() {return error_code_;}

  /* Commands */
  bool WriteSettings() {
    return vn_.WriteSettings();
  }
  void RestoreFactorySettings() {
    vn_.RestoreFactorySettings();
  }
  void Reset() {
    vn_.Reset();
  }
  bool SetFilterBias() {
    return vn_.SetFilterBias();
  }
  bool KnownMagneticDisturbance(const bool present) {
    return vn_.KnownMagneticDisturbance(present);
  }
  bool KnownAccelerationDisturbance(const bool present) {
    return vn_.KnownAccelerationDisturbance(present);
  }
  bool SetGyroBias() {
    return vn_.SetGyroBias();
  }

  /* Data */
  inline InsMode ins_mode() const {
    return ins_mode_;
  }
  inline bool ins_error() const {
    return ins_error_;
  }
  inline bool ins_time_error() const {
    return ins_time_error_;
  }
  inline bool ins_imu_error() const {
    return ins_imu_error_;
  }
  inline bool ins_mag_pres_error() const {
    return ins_mag_press_error_;
  }
  inline bool ins_gnss_error() const {
    return ins_gnss_error_;
  }
  inline bool ins_gnss_heading() const {
    return ins_gnss_heading_;
  }
  inline bool ins_gnss_compass() const {
    return ins_gnss_compass_;
  }
  inline double ins_time_s() const {
    return ins_.payload.time;
  }
  inline uint16_t ins_week() const {
    return ins_.payload.week;
  }
  inline float yaw_rad() const {
    return ins_.payload.yaw * DEG2RADf_;
  }
  inline float pitch_rad() const {
    return ins_.payload.pitch * DEG2RADf_;
  }
  inline float roll_rad() const {
    return ins_.payload.roll * DEG2RADf_;
  }
  inline double ins_lat_rad() const {
    return ins_.payload.latitude * DEG2RADl_;
  }
  inline double ins_lon_rad() const {
    return ins_.payload.longitude * DEG2RADl_;
  }
  inline double ins_alt_m() const {
    return ins_.payload.altitude;
  }
  inline float ins_north_vel_mps() const {
    return ins_.payload.ned_vel_x;
  }
  inline float ins_east_vel_mps() const {
    return ins_.payload.ned_vel_y;
  }
  inline float ins_down_vel_mps() const {
    return ins_.payload.ned_vel_z;
  }
  inline float ins_att_uncertainty_rad() const {
    return ins_.payload.att_uncertainty * DEG2RADf_;
  }
  inline float ins_pos_uncertainty_m() const {
    return ins_.payload.pos_uncertainty;
  }
  inline float ins_vel_uncertainty_mps() const {
    return ins_.payload.vel_uncertainty;
  }
  inline double gnss_time_s() const {
    return gnss_.payload.time;
  }
  inline uint16_t gnss_week() const {
    return gnss_.payload.week;
  }
  inline GnssFix gnss_fix() const {
    return static_cast<GnssFix>(gnss_.payload.gps_fix);
  }
  inline uint8_t gnss_num_sats() const {
    return gnss_.payload.num_sats;
  }
  inline double gnss_lat_rad() const {
    return gnss_.payload.latitude * DEG2RADl_;
  }
  inline double gnss_lon_rad() const {
    return gnss_.payload.longitude * DEG2RADl_;
  }
  inline double gnss_alt_m() const {
    return gnss_.payload.altitude;
  }
  inline float gnss_north_vel_mps() const {
    return gnss_.payload.ned_vel_x;
  }
  inline float gnss_east_vel_mps() const {
    return gnss_.payload.ned_vel_y;
  }
  inline float gnss_down_vel_mps() const {
    return gnss_.payload.ned_vel_z;
  }
  inline float gnss_north_acc_m() const {
    return gnss_.payload.north_acc;
  }
  inline float gnss_east_acc_m() const {
    return gnss_.payload.east_acc;
  }
  inline float gnss_down_acc_m() const {
    return gnss_.payload.vert_acc;
  }
  inline float gnss_speed_acc_mps() const {
    return gnss_.payload.speed_acc;
  }
  inline float gnss_time_acc_s() const {
    return gnss_.payload.time_acc;
  }
  inline float accel_x_mps2() const {
    return comp_imu_.payload.accel_x;
  }
  inline float accel_y_mps2() const {
    return comp_imu_.payload.accel_y;
  }
  inline float accel_z_mps2() const {
    return comp_imu_.payload.accel_z;
  }
  inline float gyro_x_radps() const {
    return comp_imu_.payload.gyro_x;
  }
  inline float gyro_y_radps() const {
    return comp_imu_.payload.gyro_y;
  }
  inline float gyro_z_radps() const {
    return comp_imu_.payload.gyro_z;
  }
  inline float mag_x_ut() const {
    return comp_imu_.payload.mag_x * 100.0f;
  }
  inline float mag_y_ut() const {
    return comp_imu_.payload.mag_y * 100.0f;
  }
  inline float mag_z_ut() const {
    return comp_imu_.payload.mag_z * 100.0f;
  }
  inline float uncomp_accel_x_mps2() const {
    return uncomp_imu_.payload.accel_x;
  }
  inline float uncomp_accel_y_mps2() const {
    return uncomp_imu_.payload.accel_y;
  }
  inline float uncomp_accel_z_mps2() const {
    return uncomp_imu_.payload.accel_z;
  }
  inline float uncomp_gyro_x_radps() const {
    return uncomp_imu_.payload.gyro_x;
  }
  inline float uncomp_gyro_y_radps() const {
    return uncomp_imu_.payload.gyro_y;
  }
  inline float uncomp_gyro_z_radps() const {
    return uncomp_imu_.payload.gyro_z;
  }
  inline float uncomp_mag_x_ut() const {
    return uncomp_imu_.payload.mag_x * 100.0f;
  }
  inline float uncomp_mag_y_ut() const {
    return uncomp_imu_.payload.mag_y * 100.0f;
  }
  inline float uncomp_mag_z_ut() const {
    return uncomp_imu_.payload.mag_z * 100.0f;
  }
  inline float die_temp_c() const {
    return uncomp_imu_.payload.temp;
  }
  inline float pres_pa() const {
    return uncomp_imu_.payload.pressure * 1000.0f;
  }

 private:
  /* Register reading and writing */
  VectorNav vn_;
  /* Expected product name */
  static constexpr char PROD_NAME_[] = {"VN-300"};
  /* Data */
  uint8_t ins_status_buff_[2];
  InsMode ins_mode_;
  bool ins_gnss_fix_;
  bool ins_error_;
  bool ins_time_error_;
  bool ins_imu_error_;
  bool ins_mag_press_error_;
  bool ins_gnss_error_;
  bool ins_gnss_heading_;
  bool ins_gnss_compass_;
  static constexpr float DEG2RADf_ = 3.14159265358979323846264338327950288f /
                                     180.0f;
  static constexpr double DEG2RADl_ = 3.14159265358979323846264338327950288 /
                                      180.0;
  /* Registers */
  VectorNav::ErrorCode error_code_;
  VnModelNumber model_num_;
  VnSerialNumber serial_num_;
  VnSynchronizationControl sync_cntrl_;
  VnReferenceFrameRotation rotation_;
  VnGnssAntennaOffset antenna_;
  VnGpsCompassBaseline baseline_;
  VnImuFilteringConfiguration filter_;
  VnInsSolutionLla ins_;
  VnGnssSolutionLla gnss_;
  VnMagneticAccelerationAngularRates comp_imu_;
  VnImuMeasurements uncomp_imu_;
};

}  // namespace bfs

#endif  // VECTOR_NAV_SRC_VN300_H_ NOLINT
