// /*
// * Brian R Taylor
// * brian.taylor@bolderflight.com
// * 
// * Copyright (c) 2020 Bolder Flight Systems
// */

// #ifndef INCLUDE_VECTOR_NAV_VN300_H_
// #define INCLUDE_VECTOR_NAV_VN300_H_

// #include "Eigen/Core"
// #include "Eigen/Dense"
// #include "core/core.h"
// #include "vector_nav/vector_nav.h"

// namespace sensors {

// class Vn300 {
//  public:
//   enum InsMode : uint8_t{
//     NOT_TRACKING = 0,
//     DEGRADED = 1,
//     HEALTHY = 2
//   };
//   enum GnssFix : uint8_t {
//     FIX_NONE = 0,
//     FIX_TIME_ONLY = 1,
//     FIX_2D = 2,
//     FIX_3D = 3
//   };
//   Vn300(SPIClass *bus, uint8_t cs) : vector_nav_(bus, cs) {}
//   bool Begin();
//   bool EnableDrdyInt(uint16_t srd);
//   bool DisableDrdyInt();
//   bool ApplyRotation(Eigen::Matrix3f c);
//   bool GetRotation(Eigen::Matrix3f *c);
//   bool SetAntennaOffset(Eigen::Vector3f b);
//   bool GetAntennaOffset(Eigen::Vector3f *b);
//   bool SetAntennaBaseline(Eigen::Vector3f b, Eigen::Vector3f uncertainty);
//   bool GetAntennaBaseline(Eigen::Vector3f *b, Eigen::Vector3f *uncertainty);
//   void DrdyCallback(uint8_t int_pin, void (*function)());
//   bool Read();
//   inline VectorNav::ErrorCode error_code() {return error_code_;}
//   inline double ins_time_s() {return ins_time_s_;}
//   inline uint16_t ins_week() {return ins_week_;}
//   inline InsMode ins_mode() {return ins_mode_;}
//   inline bool ins_error() {return ins_error_;}
//   inline Eigen::Vector3d ins_lla_rad_m() {return ins_lla_rad_m_;}
//   inline double ins_latitude_rad() {return ins_lla_rad_m_(0);}
//   inline double ins_longitude_rad() {return ins_lla_rad_m_(1);}
//   inline double ins_altitude_m() {return ins_lla_rad_m_(2);}
//   inline Eigen::Vector3f ins_ned_vel_mps() {return ins_ned_vel_mps_;}
//   inline double ins_north_vel_mps() {return ins_ned_vel_mps_(0);}
//   inline double ins_east_vel_mps() {return ins_ned_vel_mps_(1);}
//   inline double ins_down_vel_mps() {return ins_ned_vel_mps_(2);}
//   inline float ins_att_uncertainty_rad() {return ins_att_uncertainty_rad_;}
//   inline float ins_pos_uncertainty_m() {return ins_pos_uncertainty_m_;}
//   inline float ins_vel_uncertainty_mps() {return ins_vel_uncertainty_mps_;}
//   inline bool ins_gnss_heading() {}
//   inline bool ins_gnss_compass() {}
//   inline double gnss_time_s() {return gnss_time_s_;}
//   inline uint16_t gnss_week() {return gnss_week_;}
//   inline GnssFix gnss_fix() {return gnss_fix_;}
//   inline uint8_t gnss_num_satellites() {return gnss_num_sv_;}
//   inline Eigen::Vector3d gnss_lla_rad_m() {return gnss_lla_rad_m_;}
//   inline double gnss_latitude_rad() {return gnss_lla_rad_m_(0);}
//   inline double gnss_longitude_rad() {return gnss_lla_rad_m_(1);}
//   inline double gnss_altitude_m() {return gnss_lla_rad_m_(2);}
//   inline Eigen::Vector3f gnss_ned_vel_mps() {return gnss_ned_vel_mps_;}
//   inline double gnss_north_vel_mps() {return gnss_ned_vel_mps_(0);}
//   inline double gnss_east_vel_mps() {return gnss_ned_vel_mps_(1);}
//   inline double gnss_down_vel_mps() {return gnss_ned_vel_mps_(2);}
//   inline double gnss_north_acc_m() {return gnss_ned_acc_m_(0);}
//   inline double gnss_east_acc_m() {return gnss_ned_acc_m_(1);}
//   inline double gnss_down_acc_m() {return gnss_ned_acc_m_(2);}
//   inline double gnss_speed_acc_mps() {return gnss_speed_acc_mps_;}
//   inline double gnss_time_acc_s() {return gnss_time_acc_s_;}
//   inline float yaw_rad() {return ypr_rad_(0);}
//   inline float pitch_rad() {return ypr_rad_(1);}
//   inline float roll_rad() {return ypr_rad_(2);}
//   inline Eigen::Vector3f mag_ut() {return mag_ut_;}
//   inline float mag_x_ut() {return mag_ut_(0);}
//   inline float mag_y_ut() {return mag_ut_(1);}
//   inline float mag_z_ut() {return mag_ut_(2);}
//   inline Eigen::Vector3f accel_mps2() {return accel_mps2_;}
//   inline float accel_x_mps2() {return accel_mps2_(0);}
//   inline float accel_y_mps2() {return accel_mps2_(1);}
//   inline float accel_z_mps2() {return accel_mps2_(2);}
//   inline Eigen::Vector3f gyro_radps() {return gyro_radps_;}
//   inline float gyro_x_radps() {return gyro_radps_(0);}
//   inline float gyro_y_radps() {return gyro_radps_(1);}
//   inline float gyro_z_radps() {return gyro_radps_(2);}
//   inline Eigen::Vector3f uncomp_mag_ut() {return uncomp_mag_ut_;}
//   inline float uncomp_mag_x_ut() {return uncomp_mag_ut_(0);}
//   inline float uncomp_mag_y_ut() {return uncomp_mag_ut_(1);}
//   inline float uncomp_mag_z_ut() {return uncomp_mag_ut_(2);}
//   inline Eigen::Vector3f uncomp_accel_mps2() {return uncomp_accel_mps2_;}
//   inline float uncomp_accel_x_mps2() {return uncomp_accel_mps2_(0);}
//   inline float uncomp_accel_y_mps2() {return uncomp_accel_mps2_(1);}
//   inline float uncomp_accel_z_mps2() {return uncomp_accel_mps2_(2);}
//   inline Eigen::Vector3f uncomp_gyro_radps() {return uncomp_gyro_radps_;}
//   inline float uncomp_gyro_x_radps() {return uncomp_gyro_radps_(0);}
//   inline float uncomp_gyro_y_radps() {return uncomp_gyro_radps_(1);}
//   inline float uncomp_gyro_z_radps() {return uncomp_gyro_radps_(2);}
//   inline float die_temperature_c() {return die_temp_c_;}
//   inline float pressure_pa() {return pressure_pa_;}
//   /* Commands */
//   void WriteSettings() {vector_nav_.WriteSettings();}
//   void RestoreFactorySettings() {vector_nav_.RestoreFactorySettings(();}
//   void Reset() {vector_nav_.Reset();}
//   void SetFilterBias();

//  private:
//   /* Register reading and writing */
//   VectorNav vector_nav_;
//   /* Data */
//   VectorNav::ErrorCode error_code_;
//   double ins_time_s_;
//   uint16_t ins_week_;
//   InsMode ins_mode_;
//   bool ins_gnss_fix_;
//   bool ins_error_;
//   Eigen::Vector3d ins_lla_rad_m_;
//   Eigen::Vector3f ins_ned_vel_mps_;
//   float ins_att_uncertainty_rad_;
//   float ins_pos_uncertainty_m_;
//   float ins_vel_uncertainty_mps_;
//   double gnss_time_s_;
//   uint16_t gnss_week_;
//   GnssFix gnss_fix_;
//   uint8_t gnss_num_sv_;
//   Eigen::Vector3d gnss_lla_rad_m_;
//   Eigen::Vector3f gnss_ned_vel_mps_;
//   Eigen::Vector3f gnss_ned_acc_m_;
//   float gnss_speed_acc_mps_;
//   float gnss_time_acc_s_;
//   Eigen::Vector3f ypr_rad_;
//   Eigen::Vector3f mag_ut_;
//   Eigen::Vector3f accel_mps2_;
//   Eigen::Vector3f gyro_radps_;
//   Eigen::Vector3f uncomp_mag_ut_;
//   Eigen::Vector3f uncomp_accel_mps2_;
//   Eigen::Vector3f uncomp_gyro_radps_;
//   float die_temp_c_;
//   float pressure_pa_;
// };

// }  // namespace sensors

// #endif  // INCLUDE_VECTOR_NAV_VN300_H_
