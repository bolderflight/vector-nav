/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_VECTOR_NAV_VN300_H_
#define INCLUDE_VECTOR_NAV_VN300_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "vector_nav/vn.h"
#include "vector_nav/registers.h"
#include "global_defs/global_defs.h"

namespace sensors {

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
  Vn300(SPIClass *bus, const uint8_t cs) : vector_nav_(bus, cs) {}
  bool Begin();
  bool EnableDrdyInt(const DrdyMode mode, const uint16_t srd);
  bool DisableDrdyInt();
  bool ApplyRotation(const Eigen::Matrix3f &c);
  bool GetRotation(Eigen::Matrix3f *c);
  bool SetAntennaOffset(const Eigen::Vector3f &b);
  bool GetAntennaOffset(Eigen::Vector3f *b);
  bool SetCompassBaseline(const Eigen::Vector3f &pos, const Eigen::Vector3f &uncert);
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
  bool DrdyCallback(const uint8_t int_pin, void (*function)());//   bool Read();
  bool Read();
  inline VectorNav::ErrorCode error_code() {return error_code_;}

  /* Commands */
  bool WriteSettings() {return vector_nav_.WriteSettings();}
  void RestoreFactorySettings() {vector_nav_.RestoreFactorySettings();}
  void Reset() {vector_nav_.Reset();}
  bool SetFilterBias() {return vector_nav_.SetFilterBias();}
  bool KnownMagneticDisturbance(bool present) {return vector_nav_.KnownMagneticDisturbance(present);}
  bool KnownAccelerationDisturbance(bool present) {return vector_nav_.KnownAccelerationDisturbance(present);}
  bool SetGyroBias() {return vector_nav_.SetGyroBias();}

  /* Data */
  inline InsMode ins_mode() {return ins_mode_;}
  inline bool ins_error() {return ins_error_;}
  inline bool ins_time_error() {return ins_time_error_;}
  inline bool ins_imu_error() {return ins_imu_error_;}
  inline bool ins_mag_pres_error() {return ins_mag_press_error_;}
  inline bool ins_gnss_error() {return ins_gnss_error_;}
  inline bool ins_gnss_heading() {return ins_gnss_heading_;}
  inline bool ins_gnss_compass() {return ins_gnss_compass_;}
  inline double ins_time_s() {return ins_.payload.time;}
  inline uint16_t ins_week() {return ins_.payload.week;}
  inline float yaw_rad() {return global::conversions::Deg_to_Rad(ins_.payload.yaw);}
  inline float pitch_rad() {return global::conversions::Deg_to_Rad(ins_.payload.pitch);}
  inline float roll_rad() {return global::conversions::Deg_to_Rad(ins_.payload.roll);}
  inline double ins_lat_rad() {return global::conversions::Deg_to_Rad(ins_.payload.latitude);}
  inline double ins_lon_rad() {return global::conversions::Deg_to_Rad(ins_.payload.longitude);}
  inline double ins_alt_m() {return ins_.payload.altitude;}
  Eigen::Vector3d ins_lla_rad_m();
  inline float ins_north_vel_mps() {return ins_.payload.ned_vel_x;}
  inline float ins_east_vel_mps() {return ins_.payload.ned_vel_y;}
  inline float ins_down_vel_mps() {return ins_.payload.ned_vel_z;}
  Eigen::Vector3f ins_ned_vel_mps();
  inline float ins_att_uncertainty_rad() {return global::conversions::Deg_to_Rad(ins_.payload.att_uncertainty);}
  inline float ins_pos_uncertainty_m() {return ins_.payload.pos_uncertainty;}
  inline float ins_vel_uncertainty_mps() {return ins_.payload.vel_uncertainty;}
  inline double gnss_time_s() {return gnss_.payload.time;}
  inline uint16_t gnss_week() {return gnss_.payload.week;}
  inline GnssFix gnss_fix() {return static_cast<GnssFix>(gnss_.payload.gps_fix);}
  inline uint8_t gnss_num_satellites() {return gnss_.payload.num_sats;}
  inline double gnss_lat_rad() {return global::conversions::Deg_to_Rad(gnss_.payload.latitude);}
  inline double gnss_lon_rad() {return global::conversions::Deg_to_Rad(gnss_.payload.longitude);}
  inline double gnss_alt_m() {return gnss_.payload.altitude;}
  Eigen::Vector3d gnss_lla_rad_m();
  inline float gnss_north_vel_mps() {return gnss_.payload.ned_vel_x;}
  inline float gnss_east_vel_mps() {return gnss_.payload.ned_vel_y;}
  inline float gnss_down_vel_mps() {return gnss_.payload.ned_vel_z;}
  Eigen::Vector3f gnss_ned_vel_mps();
  inline float gnss_north_acc_m() {return gnss_.payload.north_acc;}
  inline float gnss_east_acc_m() {return gnss_.payload.east_acc;}
  inline float gnss_down_acc_m() {return gnss_.payload.vert_acc;}
  inline float gnss_speed_acc_mps() {return gnss_.payload.speed_acc;}
  inline float gnss_time_acc_s() {return gnss_.payload.time_acc;}
  inline float accel_x_mps2() {return comp_imu_.payload.accel_x;}
  inline float accel_y_mps2() {return comp_imu_.payload.accel_y;}
  inline float accel_z_mps2() {return comp_imu_.payload.accel_z;}
  Eigen::Vector3f accel_mps2();
  inline float gyro_x_radps() {return comp_imu_.payload.gyro_x;}
  inline float gyro_y_radps() {return comp_imu_.payload.gyro_y;}
  inline float gyro_z_radps() {return comp_imu_.payload.gyro_z;}
  Eigen::Vector3f gyro_radps();
  inline float mag_x_ut() {return global::conversions::Gauss_to_uT(comp_imu_.payload.mag_x);}
  inline float mag_y_ut() {return global::conversions::Gauss_to_uT(comp_imu_.payload.mag_y);}
  inline float mag_z_ut() {return global::conversions::Gauss_to_uT(comp_imu_.payload.mag_z);}
  Eigen::Vector3f mag_ut();
  inline float uncomp_accel_x_mps2() {return uncomp_imu_.payload.accel_x;}
  inline float uncomp_accel_y_mps2() {return uncomp_imu_.payload.accel_y;}
  inline float uncomp_accel_z_mps2() {return uncomp_imu_.payload.accel_z;}
  Eigen::Vector3f uncomp_accel_mps2();
  inline float uncomp_gyro_x_radps() {return uncomp_imu_.payload.gyro_x;}
  inline float uncomp_gyro_y_radps() {return uncomp_imu_.payload.gyro_y;}
  inline float uncomp_gyro_z_radps() {return uncomp_imu_.payload.gyro_z;}
  Eigen::Vector3f uncomp_gyro_radps();
  inline float uncomp_mag_x_ut() {return global::conversions::Gauss_to_uT(uncomp_imu_.payload.mag_x);}
  inline float uncomp_mag_y_ut() {return global::conversions::Gauss_to_uT(uncomp_imu_.payload.mag_y);}
  inline float uncomp_mag_z_ut() {return global::conversions::Gauss_to_uT(uncomp_imu_.payload.mag_z);}
  Eigen::Vector3f uncomp_mag_ut();
  inline float die_temperature_c() {return uncomp_imu_.payload.temp;}
  inline float pressure_pa() {return uncomp_imu_.payload.pressure * 1000.0f;}  // kPa to Pa

 private:
  /* Register reading and writing */
  VectorNav vector_nav_;
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
  vector_nav::common::SerialNumber serial_num_;
  vector_nav::common::SynchronizationControl sync_cntrl_;
  vector_nav::common::ReferenceFrameRotation rotation_;
  vector_nav::vn200::GnssAntennaOffset antenna_;
  vector_nav::vn300::GpsCompassBaseline baseline_;
  vector_nav::common::ImuFilteringConfiguration filter_;
  vector_nav::vn200::InsSolutionLla ins_;
  vector_nav::vn200::GnssSolutionLla gnss_;
  vector_nav::common::MagneticAccelerationAngularRates comp_imu_;
  vector_nav::common::ImuMeasurements uncomp_imu_;
};

}  // namespace sensors

#endif  // INCLUDE_VECTOR_NAV_VN300_H_
