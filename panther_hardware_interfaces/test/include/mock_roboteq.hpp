#ifndef PANTHER_HARDWARE_INTERFACES__MOCK_ROBOTEQ_HPP_
#define PANTHER_HARDWARE_INTERFACES__MOCK_ROBOTEQ_HPP_

#include <atomic>
#include <filesystem>
#include <iostream>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/slave.hpp>

enum class DriverFaultFlags {
  OVERHEAT = 0,
  OVERVOLTAGE,
  UNDERVOLTAGE,
  SHORT_CIRCUIT,
  EMERGENCY_STOP,
  MOTOR_OR_SENSOR_SETUP_FAULT,
  MOSFET_FAILURE,
  DEFAULT_CONFIG_LOADED_AT_STARTUP,
};

enum class DriverRuntimeErrors {
  AMPS_LIMIT_ACTIVE = 0,
  MOTOR_STALL,
  LOOP_ERROR,
  SAFETY_STOP_ACTIVE,
  FORWARD_LIMIT_TRIGGERED,
  REVERSE_LIMIT_TRIGGERED,
  AMPS_TRIGGER_ACTIVATED,
};

enum class DriverScriptFlags {
  LOOP_ERROR = 0,
  ENCODER_DISCONNECTED,
  AMP_LIMITER,
};

class RoboteqSlave : public lely::canopen::BasicSlave
{
public:
  using BasicSlave::BasicSlave;

  void SetPosition(uint8_t channel, int32_t value);
  void SetVelocity(uint8_t channel, int32_t value);
  void SetCurrent(uint8_t channel, int32_t value);
  void SetDriverFaultFlag(DriverFaultFlags flag);
  void SetDriverScriptFlag(DriverScriptFlags flag);
  void SetDriverRuntimeErrors(uint8_t channel, DriverRuntimeErrors flag);
  void SetTemperature(int16_t value) { (*this)[0x210F][1] = value; }
  void SetVoltage(uint16_t value) { (*this)[0x210D][2] = value; }
  void SetBatAmps1(int16_t value) { (*this)[0x210C][1] = value; }
  void SetBatAmps2(int16_t value) { (*this)[0x210C][2] = value; }

  void ClearErrorFlags();

  void InitializeValues();

  void StartPublishing();
  void StopPublishing();

  void TriggerPDOPublish();

private:
  std::thread pdo_publishing_thread_;
  std::atomic_bool stop_publishing_ = false;
};

class RoboteqMock
{
public:
  RoboteqMock() {}
  ~RoboteqMock() {}

  void Start();
  void Stop();

private:
  std::unique_ptr<RoboteqSlave> front_driver_;
  std::unique_ptr<RoboteqSlave> rear_driver_;

  std::unique_ptr<lely::io::IoGuard> io_guard_;
  std::unique_ptr<lely::io::Context> ctx_;
  std::shared_ptr<lely::ev::Loop> loop_;
  std::unique_ptr<lely::io::Poll> poll_;
  std::unique_ptr<lely::ev::Executor> exec_;

  std::unique_ptr<lely::io::CanController> ctrl_;

  std::unique_ptr<lely::io::Timer> timer1_;
  std::unique_ptr<lely::io::CanChannel> chan1_;

  std::unique_ptr<lely::io::Timer> timer2_;
  std::unique_ptr<lely::io::CanChannel> chan2_;

  // TODO: change name
  std::thread executor_thread_;
};

#endif