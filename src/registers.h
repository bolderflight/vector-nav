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

#ifndef VECTOR_NAV_SRC_REGISTERS_H_  // NOLINT
#define VECTOR_NAV_SRC_REGISTERS_H_

#if defined(ARDUINO)
#include <Arduino.h>
#pragma pack(4)
#else
#include <cstdint>
#endif

namespace bfs {

/*
* Defines all the available VectorNav registers with their id and data.
* Namespaces are used to separate common registers from unit
* specific registers (i.e. registers only available on VN-300).
*/

/* Registers common accross VectorNav products */

struct VnUserTag {
  static const uint8_t id = 0;
  static const uint8_t size = 20;
  static const bool read_only = false;
  struct {
    char tag[20];
  } payload;
};

struct VnModelNumber {
  static const uint8_t id = 1;
  static const uint8_t size = 24;
  static const bool read_only = true;
  struct {
    char product_name[24];
  } payload;
};

struct VnHardwareRevision {
  static const uint8_t id = 2;
  static const uint8_t size = 4;
  static const bool read_only = true;
  struct {
    uint32_t revision;
  } payload;
};

struct VnSerialNumber {
  static const uint8_t id = 3;
  static const uint8_t size = 4;
  static const bool read_only = true;
  struct {
    uint32_t serial_num;
  } payload;
};

struct VnFirmwareVersion {
  static const uint8_t id = 4;
  static const uint8_t size = 4;
  static const bool read_only = true;
  struct {
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t feature_version;
    uint8_t hotfix;
  } payload;
};

struct VnSynchronizationControl {
  static const uint8_t id = 32;
  static const uint8_t size = 20;
  static const bool read_only = false;
  struct {
    uint8_t sync_in_mode;
    uint8_t sync_in_edge;
    uint16_t sync_in_skip_factor;
    uint32_t resv1 = 0;
    uint8_t sync_out_mode;
    uint8_t sync_out_polarity;
    uint16_t sync_out_skip_factor;
    uint32_t sync_out_pulse_width;
    uint32_t resv2 = 0;
  } payload;
};

struct VnSynchronizationStatus {
  static const uint8_t id = 33;
  static const uint8_t size = 12;
  static const bool read_only = false;
  struct {
    uint32_t sync_in_count;
    uint32_t sync_in_time;
    uint32_t sync_out_count;
  } payload;
};

struct VnImuMeasurements {
  static const uint8_t id = 54;
  static const uint8_t size = 44;
  static const bool read_only = true;
  struct {
    float mag_x;
    float mag_y;
    float mag_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
    float pressure;
  } payload;
};

struct VnDeltaThetaDeltaVelocity {
  static const uint8_t id = 80;
  static const uint8_t size = 28;
  static const bool read_only = true;
  struct {
    float delta_time;
    float delta_theta_x;
    float delta_theta_y;
    float delta_theta_z;
    float delta_velocity_x;
    float delta_velocity_y;
    float delta_velocity_z;
  } payload;
};

struct VnMagnetometerCompensation {
  static const uint8_t id = 23;
  static const uint8_t size = 48;
  static const bool read_only = false;
  struct {
    float c[3][3];
    float b[3];
  } payload;
};

struct VnAccelerationCompensation {
  static const uint8_t id = 25;
  static const uint8_t size = 48;
  static const bool read_only = false;
  struct {
    float c[3][3];
    float b[3];
  } payload;
};

struct VnGyroCompensation {
  static const uint8_t id = 84;
  static const uint8_t size = 48;
  static const bool read_only = false;
  struct {
    float c[3][3];
    float b[3];
  } payload;
};

struct VnReferenceFrameRotation {
  static const uint8_t id = 26;
  static const uint8_t size = 36;
  static const bool read_only = false;
  struct {
    float c[3][3];
  } payload;
};

struct VnImuFilteringConfiguration {
  static const uint8_t id = 85;
  static const uint8_t size = 16;  // tested to work with 16
  static const bool read_only = false;
  struct {
    uint16_t mag_window_size;
    uint16_t accel_window_size;
    uint16_t gyro_window_size;
    uint16_t temp_window_size;
    uint16_t pres_window_size;
    uint8_t mag_filter_mode;
    uint8_t accel_filter_mode;
    uint8_t gyro_filter_mode;
    uint8_t temp_filter_mode;
    uint8_t pres_filter_mode;
    uint8_t padding;
  } payload;
};

struct VnDeltaThetaDeltaVelocityConfiguration {
  static const uint8_t id = 82;
  static const uint8_t size = 6;
  static const bool read_only = false;
  struct {
    uint8_t integration_frame;
    uint8_t gyro_compensation;
    uint8_t accel_compensation;
    uint8_t resv1 = 0;
    uint16_t resv2 = 0;
  } payload;
};

struct VnYawPitchRoll {
  static const uint8_t id = 8;
  static const uint8_t size = 12;
  static const bool read_only = true;
  struct {
    float yaw;
    float pitch;
    float roll;
  } payload;
};

struct VnAttitudeQuaternion {
  static const uint8_t id = 9;
  static const uint8_t size = 16;
  static const bool read_only = true;
  struct {
    float quat[4];
  } payload;
};

struct VnYawPitchRollMagneticAccelerationAngularRates {
  static const uint8_t id = 27;
  static const uint8_t size = 48;
  static const bool read_only = true;
  struct {
    float yaw;
    float pitch;
    float roll;
    float mag_x;
    float mag_y;
    float mag_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } payload;
};

struct VnQuaternionMagneticAccelerationAngularRates {
  static const uint8_t id = 15;
  static const uint8_t size = 52;
  static const bool read_only = true;
  struct {
    float quat[4];
    float mag_x;
    float mag_y;
    float mag_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } payload;
};

struct VnMagneticMeasurements {
  static const uint8_t id = 17;
  static const uint8_t size = 12;
  static const bool read_only = true;
  struct {
    float mag_x;
    float mag_y;
    float mag_z;
  } payload;
};

struct VnAccelerationMeasurements {
  static const uint8_t id = 18;
  static const uint8_t size = 12;
  static const bool read_only = true;
  struct {
    float accel_x;
    float accel_y;
    float accel_z;
  } payload;
};

struct VnAngularRateMeasurements {
  static const uint8_t id = 19;
  static const uint8_t size = 12;
  static const bool read_only = true;
  struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } payload;
};

struct VnMagneticAccelerationAngularRates {
  static const uint8_t id = 20;
  static const uint8_t size = 36;
  static const bool read_only = true;
  struct {
    float mag_x;
    float mag_y;
    float mag_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } payload;
};

struct VnYawPitchRollTrueBodyAccelerationAngularRates {
  static const uint8_t id = 239;
  static const uint8_t size = 36;
  static const bool read_only = true;
  struct {
    float yaw;
    float pitch;
    float roll;
    float body_accel_x;
    float body_accel_y;
    float body_accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } payload;
};

struct VnYawPitchRollTrueInertialAccelerationAngularRates {
  static const uint8_t id = 240;
  static const uint8_t size = 36;
  static const bool read_only = true;
  struct {
    float yaw;
    float pitch;
    float roll;
    float inertial_accel_x;
    float inertial_accel_y;
    float inertial_accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } payload;
};

struct VnHeave {
  static const uint8_t id = 115;
  static const uint8_t size = 12;
  static const bool read_only = true;
  struct {
    float heave;
    float heave_rate;
    float delayed_heave;
  } payload;
};

struct VnHeaveConfiguration {
  static const uint8_t id = 116;
  static const uint8_t size = 28;
  static const bool read_only = false;
  struct {
    float initial_wave_period;
    float initial_wave_amplitude;
    float max_wave_period;
    float min_wave_amplitude;
    float delayed_heave_cutoff_freq;
    float heave_cutoff_freq;
    float heave_rate_cutoff_freq;
  } payload;
};

struct VnVpeBasicControl {
  static const uint8_t id = 35;
  static const uint8_t size = 4;
  static const bool read_only = false;
  struct {
    uint8_t enable;
    uint8_t heading_mode;
    uint8_t filtering_mode;
    uint8_t tuning_mode;
  } payload;
};

struct VnMagnetometerCalibrationControl {
  static const uint8_t id = 44;
  static const uint8_t size = 4;
  static const bool read_only = false;
  struct {
    uint8_t hsi_mode;
    uint8_t hsi_output;
    uint8_t converge_rate;
  } payload;
};

struct VnCalculatedMagnetometerCalibration {
  static const uint8_t id = 47;
  static const uint8_t size = 48;
  static const bool read_only = true;
  struct {
    float c[3][3];
    float b[3];
  } payload;
};

struct VnMagneticGravityReferenceVectors {
  static const uint8_t id = 21;
  static const uint8_t size = 24;
  static const bool read_only = false;
  struct {
    float mag_ref_x;
    float mag_ref_y;
    float mag_ref_z;
    float acc_ref_x;
    float acc_ref_y;
    float acc_ref_z;
  } payload;
};

struct VnReferenceVectorConfiguration {
  static const uint8_t id = 83;
  /* differs from manual, but confirmed 40 bytes with VN support */
  static const uint8_t size = 40;
  static const bool read_only = false;
  struct {
    uint8_t use_mag_model;
    uint8_t use_gravity_model;
    uint8_t resv1 = 0;
    uint8_t resv2 = 0;
    uint32_t recalc_threshold;
    float year;
    uint8_t padding[4];
    double latitude;
    double longitude;
    double altitude;
  } payload;
};

/* VN-100 specific registers */

struct VnVpeMagnetometerBasicTuning {
  static const uint8_t id = 36;
  static const uint8_t size = 36;
  static const bool read_only = false;
  struct {
    float base_tuning_x;
    float base_tuning_y;
    float base_tuning_z;
    float adaptive_tuning_x;
    float adaptive_tuning_y;
    float adaptive_tuning_z;
    float adaptive_filtering_x;
    float adaptive_filtering_y;
    float adaptive_filtering_z;
  } payload;
};

struct VnVpeAccelerometerBasicTuning {
  static const uint8_t id = 38;
  static const uint8_t size = 36;
  static const bool read_only = false;
  struct {
    float base_tuning_x;
    float base_tuning_y;
    float base_tuning_z;
    float adaptive_tuning_x;
    float adaptive_tuning_y;
    float adaptive_tuning_z;
    float adaptive_filtering_x;
    float adaptive_filtering_y;
    float adaptive_filtering_z;
  } payload;
};

struct VnFilterStartupGyroBias {
  static const uint8_t id = 43;
  static const uint8_t size = 12;
  static const bool read_only = false;
  struct {
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
  } payload;
};

struct VnVpeGyroBasicTuning {
  static const uint8_t id = 40;
  static const uint8_t size = 36;
  static const bool read_only = false;
  struct {
    float base_tuning_x;
    float base_tuning_y;
    float base_tuning_z;
    float adaptive_tuning_x;
    float adaptive_tuning_y;
    float adaptive_tuning_z;
    float adaptive_filtering_x;
    float adaptive_filtering_y;
    float adaptive_filtering_z;
  } payload;
};

struct VnVelocityCompensationControl {
  static const uint8_t id = 51;
  static const uint8_t size = 12;
  static const bool read_only = false;
  struct {
    uint8_t mode;
    float velocity_tuning;
    float rate_tuning;
  } payload;
};

struct VnVelocityCompensationMeasurement {
  static const uint8_t id = 50;
  static const uint8_t size = 12;
  static const bool read_only = false;
  struct {
    float velocity_x;
    float velocity_y;
    float velocity_z;
  } payload;
};

/* VN-200 specific registers */

struct VnGnssSolutionLla {
  static const uint8_t id = 58;
  static const uint8_t size = 72;
  static const bool read_only = false;
  struct {
    double time;
    uint16_t week;
    uint8_t gps_fix;
    uint8_t num_sats;
    uint8_t padding[4];
    double latitude;
    double longitude;
    double altitude;
    float ned_vel_x;
    float ned_vel_y;
    float ned_vel_z;
    float north_acc;
    float east_acc;
    float vert_acc;
    float speed_acc;
    float time_acc;
  } payload;
};

struct VnGnssSolutionEcef {
  static const uint8_t id = 59;
  static const uint8_t size = 72;
  static const bool read_only = false;
  struct {
    double time;
    uint16_t week;
    uint8_t gps_fix;
    uint8_t num_sats;
    uint8_t padding[4];
    double position_x;
    double position_y;
    double position_z;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float pos_acc_x;
    float pos_acc_y;
    float pos_acc_z;
    float speed_acc;
    float time_acc;
  } payload;
};

struct VnGnssConfiguration {
  static const uint8_t id = 55;
  static const uint8_t size = 5;
  static const bool read_only = false;
  struct {
    uint8_t mode;
    uint8_t pps_source;
    uint8_t rate = 5;
    uint8_t time_sync_delta = 0;
    uint8_t ant_power = 1;
  } payload;
};

struct VnGnssAntennaOffset {
  static const uint8_t id = 57;
  static const uint8_t size = 12;
  static const bool read_only = false;
  struct {
    float position_x;
    float position_y;
    float position_z;
  } payload;
};

struct VnInsSolutionLla {
  static const uint8_t id = 63;
  static const uint8_t size = 72;
  static const bool read_only = true;
  struct {
    double time;
    uint16_t week;
    uint16_t status;
    float yaw;
    float pitch;
    float roll;
    double latitude;
    double longitude;
    double altitude;
    float ned_vel_x;
    float ned_vel_y;
    float ned_vel_z;
    float att_uncertainty;
    float pos_uncertainty;
    float vel_uncertainty;
  } payload;
};

struct VnInsSolutionEcef {
  static const uint8_t id = 64;
  static const uint8_t size = 72;
  static const bool read_only = true;
  struct {
    double time;
    uint16_t week;
    uint16_t status;
    float yaw;
    float pitch;
    float roll;
    double position_x;
    double position_y;
    double position_z;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float att_uncertainty;
    float pos_uncertainty;
    float vel_uncertainty;
  } payload;
};

struct VnInsStateLla {
  static const uint8_t id = 72;
  static const uint8_t size = 80;
  static const bool read_only = true;
  struct {
    float yaw;
    float pitch;
    float roll;
    uint8_t padding[4];
    double latitude;
    double longitude;
    double altitude;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
  } payload;
};

struct VnInsStateEcef {
  static const uint8_t id = 73;
  static const uint8_t size = 80;
  static const bool read_only = true;
  struct {
    float yaw;
    float pitch;
    float roll;
    uint8_t padding[4];
    double position_x;
    double position_y;
    double position_z;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
  } payload;
};

struct VnStartupFilterBiasEstimate {
  static const uint8_t id = 74;
  static const uint8_t size = 28;
  static const bool read_only = false;
  struct {
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    float accel_bias_x;
    float accel_bias_y;
    float accel_bias_z;
    float pressure_bias;
  } payload;
};

/* VN-300 specific registers */

struct VnGpsCompassBaseline {
  static const uint8_t id = 93;
  static const uint8_t size = 24;
  static const bool read_only = false;
  struct {
    float position_x;
    float position_y;
    float position_z;
    float uncertainty_x;
    float uncertainty_y;
    float uncertainty_z;
  } payload;
};

struct VnGpsCompassEstimatedBaseline {
  static const uint8_t id = 97;
  static const uint8_t size = 28;
  static const bool read_only = true;
  struct {
    uint8_t estimated_baseline_used;
    uint8_t resv1 = 0;
    uint16_t num_meas;
    float position_x;
    float position_y;
    float position_z;
    float uncertainty_x;
    float uncertainty_y;
    float uncertainty_z;
  } payload;
};

struct VnInsBasicConfiguration {
  static const uint8_t id = 67;
  static const uint8_t size = 4;
  static const bool read_only = false;
  struct {
    uint8_t scenario;
    uint8_t ahrs_aiding;
    uint8_t est_baseline;
    uint8_t resv2 = 0;
  } payload;
};

}  // namespace bfs

#endif  // VECTOR_NAV_SRC_REGISTERS_H_ NOLINT
