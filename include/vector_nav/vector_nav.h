/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_VECTOR_NAV_VECTOR_NAV_H_
#define INCLUDE_VECTOR_NAV_VECTOR_NAV_H_

#include "core/core.h"

namespace sensors {

class VectorNav {
 public:
  enum ErrorCode {
    ERROR_SUCCESS = 0,
    ERROR_HARD_FAULT = 1,
    ERROR_SERIAL_BUFFER_OVERFLOW = 2,
    ERROR_INVALID_CHECKSUM = 3,
    ERROR_INVALID_COMMAND = 4,
    ERROR_NOT_ENOUGH_PARAMETERS = 5,
    ERROR_TOO_MANY_PARAMETERS = 6,
    ERROR_INVALID_PARAMETER = 7,
    ERROR_INVALID_REGISTER = 8,
    ERROR_UNAUTHORIZED_ACCESS = 9,
    ERROR_WATCHDOG_RESET = 10,
    ERROR_OUTPUT_BUFFER_OVERFLOW = 11,
    ERROR_INSUFFICIENT_BAUD_RATE = 12,
    ERROR_ERROR_BUFFER_OVERFLOW = 255
  };
  VectorNav(SPIClass *bus, uint8_t cs) : bus_(bus), cs_(cs) {}
  /* Initialize communication */
  void Init() {
    pinMode(cs_, OUTPUT);
    digitalWriteFast(cs_, HIGH);
    bus_->begin(); 
  }
  /* Read register */
  template<class REG>
  ErrorCode ReadRegister(REG *ptr) {
    static_assert(ptr->size == sizeof(ptr->payload), "VectorNav register payload size incorrect");
    /* Delay if necessary */
    if (time_since_comm_us_ < WAIT_TIME_US_) {
      delayMicroseconds(WAIT_TIME_US_ - time_since_comm_us_);
    }
    /* Read request */
    bus_->beginTransaction(SPISettings(SPI_CLOCK_, MSBFIRST, SPI_MODE3));
    digitalWriteFast(cs_, LOW);
    bus_->transfer(CMD_READ_);
    bus_->transfer(ptr->id);
    bus_->transfer(0x00);
    bus_->transfer(0x00);
    digitalWriteFast(cs_, HIGH);
    /* Wait for VectorNav to fill response buffer */
    delayMicroseconds(WAIT_TIME_US_);
    /* Read the response buffer header */
    digitalWriteFast(cs_, LOW);
    empty_ = bus_->transfer(0x00);
    cmd_ = bus_->transfer(0x00);
    arg_ = bus_->transfer(0x00);
    err_ = bus_->transfer(0x00);
    /* Check for errors */
    if (err_ != ERROR_SUCCESS) {
      digitalWriteFast(cs_, HIGH);
      bus_->endTransaction();
      time_since_comm_us_ = 0;
      return static_cast<ErrorCode>(err_);
    }
    /* Read the response data payload */
    for (std::size_t i = 0; i < sizeof(ptr->payload); i++) {
      reinterpret_cast<uint8_t *>(&ptr->payload)[i] = bus_->transfer(0x00);
    }
    digitalWriteFast(cs_, HIGH);
    bus_->endTransaction();
    time_since_comm_us_ = 0;
    return ERROR_SUCCESS;
  }
  /* Write register */
  template<class REG>
  ErrorCode WriteRegister(const REG &ref) {
    static_assert(ref.size == sizeof(ref.payload), "VectorNav register payload size incorrect");
    static_assert(ref.read_only == false, "VectorNav read-only register");
    /* Delay if necessary */
    if (time_since_comm_us_ < WAIT_TIME_US_) {
      delayMicroseconds(WAIT_TIME_US_ - time_since_comm_us_);
    }
    /* Write register */
    bus_->beginTransaction(SPISettings(SPI_CLOCK_, MSBFIRST, SPI_MODE3));
    digitalWriteFast(cs_, LOW);
    bus_->transfer(CMD_WRITE_);
    bus_->transfer(ref.id);
    bus_->transfer(0x00);
    bus_->transfer(0x00);
    for (std::size_t i = 0; i < sizeof(ref.payload); i++) {
      bus_->transfer(reinterpret_cast<const uint8_t *>(&ref.payload)[i]);
    }
    digitalWriteFast(cs_, HIGH);  
    /* Wait for VectorNav to fill response buffer */
    delayMicroseconds(WAIT_TIME_US_);
    /* Read the response buffer header */
    digitalWriteFast(cs_, LOW);
    empty_ = bus_->transfer(0x00);
    cmd_ = bus_->transfer(0x00);
    arg_ = bus_->transfer(0x00);
    err_ = bus_->transfer(0x00);
    digitalWriteFast(cs_, HIGH);
    bus_->endTransaction();
    time_since_comm_us_ = 0;
    return static_cast<ErrorCode>(err_);
  }
  /* Write command */
  void WriteSettings() {

  }
  void RestoreFactorySettings() {

  }
  void Reset() {

  }

  /* VN-100 */
  void Tare() {

  }
  void KnownMagneticDisturbance(bool present) {

  }
  void KnownAccelerationDisturbance(bool present) {

  }

  /* VN-200 and VN-300 */
  void SetFilterBias() {

  }

  // void SetInitialHeading(float heading) {
    // checking with VectorNav on this
  // }

 private:
  /* SPI */
  SPIClass *bus_;
  uint8_t cs_;
  static constexpr uint32_t SPI_CLOCK_ = 16000000;
  static constexpr uint8_t WAIT_TIME_US_ = 50;
  elapsedMicros time_since_comm_us_ = WAIT_TIME_US_;
  /* Response header */
  uint8_t cmd_, arg_, empty_, err_;
  /* Commands */
  static constexpr uint8_t CMD_READ_  = 0x01;
  static constexpr uint8_t CMD_WRITE_ = 0x02;
  static constexpr uint8_t CMD_WRITE_SETTINGS_ = 0x03;
  static constexpr uint8_t CMD_RESTORE_FACTORY_SETTINGS_ = 0x04;
  static constexpr uint8_t CMD_RESET_ = 0x06;
};

}  // namespace sensors

#endif  // INCLUDE_VECTOR_NAV_VECTOR_NAV_H_
