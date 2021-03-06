/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
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

#ifndef INCLUDE_VECTOR_NAV_VN300_H_
#define INCLUDE_VECTOR_NAV_VN300_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "vector_nav/vn.h"
#include "vector_nav/registers.h"
#include "units/units.h"

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
  Vn300(SPIClass *bus, const uint8_t cs) : vn_(bus, cs) {}
  bool Begin();
  bool EnableDrdyInt(const DrdyMode mode, const uint16_t srd);
  bool DisableDrdyInt();
  bool ApplyRotation(const Eigen::Matrix3f &c);
  bool GetRotation(Eigen::Matrix3f *c);
  bool SetAntennaOffset(const Eigen::Vector3f &b);
  bool GetAntennaOffset(Eigen::Vector3f *b);
  bool SetCompassBaseline(const Eigen::Vector3f &pos,
                          const Eigen::Vector3f &uncert);
  bool GetCompassBaseline(Eigen::Vector3f *pos, Eigen::Vector3f *uncert);
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
    return deg2rad(ins_.payload.yaw);
  }
  inline float pitch_rad() const {
    return deg2rad(ins_.payload.pitch);
  }
  inline float roll_rad() const {
    return deg2rad(ins_.payload.roll);
  }
  inline double ins_lat_rad() const {
    return deg2rad(ins_.payload.latitude);
  }
  inline double ins_lon_rad() const {
    return deg2rad(ins_.payload.longitude);
  }
  inline double ins_alt_m() const {
    return ins_.payload.altitude;
  }
  inline Eigen::Vector3d ins_lla_rad_m() const {
    Eigen::Vector3d lla;
    lla(0) = deg2rad(ins_.payload.latitude);
    lla(1) = deg2rad(ins_.payload.longitude);
    lla(2) = ins_.payload.altitude;
    return lla;
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
  inline Eigen::Vector3f ins_ned_vel_mps() const {
    Eigen::Vector3f ned_vel;
    ned_vel(0) = ins_.payload.ned_vel_x;
    ned_vel(1) = ins_.payload.ned_vel_y;
    ned_vel(2) = ins_.payload.ned_vel_z;
    return ned_vel;
  }
  inline float ins_att_uncertainty_rad() const {
    return deg2rad(ins_.payload.att_uncertainty);
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
  inline uint8_t gnss_num_satellites() const {
    return gnss_.payload.num_sats;
  }
  inline double gnss_lat_rad() const {
    return deg2rad(gnss_.payload.latitude);
  }
  inline double gnss_lon_rad() const {
    return deg2rad(gnss_.payload.longitude);
  }
  inline double gnss_alt_m() const {
    return gnss_.payload.altitude;
  }
  inline Eigen::Vector3d gnss_lla_rad_m() const {
    Eigen::Vector3d lla;
    lla(0) = deg2rad(gnss_.payload.latitude);
    lla(1) = deg2rad(gnss_.payload.longitude);
    lla(2) = gnss_.payload.altitude;
    return lla;
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
  inline Eigen::Vector3f gnss_ned_vel_mps() const {
    Eigen::Vector3f ned_vel;
    ned_vel(0) = gnss_.payload.ned_vel_x;
    ned_vel(1) = gnss_.payload.ned_vel_y;
    ned_vel(2) = gnss_.payload.ned_vel_z;
    return ned_vel;
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
  inline Eigen::Vector3f accel_mps2() const {
    Eigen::Vector3f accel;
    accel(0) = comp_imu_.payload.accel_x;
    accel(1) = comp_imu_.payload.accel_y;
    accel(2) = comp_imu_.payload.accel_z;
    return accel;
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
  inline Eigen::Vector3f gyro_radps() const {
    Eigen::Vector3f gyro;
    gyro(0) = comp_imu_.payload.gyro_x;
    gyro(1) = comp_imu_.payload.gyro_y;
    gyro(2) = comp_imu_.payload.gyro_z;
    return gyro;
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
  inline Eigen::Vector3f mag_ut() const {
    Eigen::Vector3f mag;
    mag(0) = comp_imu_.payload.mag_x * 100.0f;
    mag(1) = comp_imu_.payload.mag_y * 100.0f;
    mag(2) = comp_imu_.payload.mag_z * 100.0f;
    return mag;
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
  inline Eigen::Vector3f uncomp_accel_mps2() const {
    Eigen::Vector3f accel;
    accel(0) = uncomp_imu_.payload.accel_x;
    accel(1) = uncomp_imu_.payload.accel_y;
    accel(2) = uncomp_imu_.payload.accel_z;
    return accel;
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
  inline Eigen::Vector3f uncomp_gyro_radps() const {
    Eigen::Vector3f gyro;
    gyro(0) = uncomp_imu_.payload.gyro_x;
    gyro(1) = uncomp_imu_.payload.gyro_y;
    gyro(2) = uncomp_imu_.payload.gyro_z;
    return gyro;
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
  inline Eigen::Vector3f uncomp_mag_ut() const {
    Eigen::Vector3f mag;
    mag(0) = uncomp_imu_.payload.mag_x * 100.0f;
    mag(1) = uncomp_imu_.payload.mag_y * 100.0f;
    mag(2) = uncomp_imu_.payload.mag_z * 100.0f;
    return mag;
  }
  inline float die_temperature_c() const {
    return uncomp_imu_.payload.temp;
  }
  inline float pressure_pa() const {
    return uncomp_imu_.payload.pressure * 1000.0f;
  }

 private:
  /* Register reading and writing */
  VectorNav vn_;
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
  /* Registers */
  VectorNav::ErrorCode error_code_;
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

#endif  // INCLUDE_VECTOR_NAV_VN300_H_
