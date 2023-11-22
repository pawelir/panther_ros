// Copyright 2023 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <cmath>
#include <future>

#include <panther_hardware_interfaces/utils.hpp>

// All ids and sub ids were read directly from the eds file. Lely CANopen doesn't have the option to
// parse them based on the ParameterName. Additionally between version v60 and v80 ParameterName
// changed, for example: Cmd_ESTOP (old), Cmd_ESTOP Emergency Shutdown (new)
// As parameter names changed, but ids stayed the same, it will be better to just use ids directly

namespace panther_hardware_interfaces
{

RoboteqDriver::RoboteqDriver(
  ev_exec_t * exec, lely::canopen::AsyncMaster & master, uint8_t id,
  std::chrono::milliseconds sdo_operation_timeout)
: lely::canopen::FiberDriver(exec, master, id),
  sdo_operation_timeout_(sdo_operation_timeout),
  // Wait timeout has to be longer - first we want to give a chance for lely to cancel
  // operation
  sdo_operation_wait_timeout_(sdo_operation_timeout + std::chrono::milliseconds(1))
{
}

bool RoboteqDriver::Boot()
{
  booted_.store(false);
  return FiberDriver::Boot();
}

bool RoboteqDriver::WaitForBoot()
{
  if (booted_.load()) {
    return true;
  }
  std::unique_lock<std::mutex> lck(boot_mtx_);

  // TODO: test timeout
  if (boot_cond_var_.wait_for(lck, std::chrono::seconds(5)) == std::cv_status::timeout) {
    throw std::runtime_error("Timeout while waiting for boot");
  }

  if (booted_.load()) {
    return true;
  } else {
    throw std::runtime_error(boot_error_str_);
  }
}

int16_t RoboteqDriver::ReadTemperature()
{
  try {
    return SyncSdoRead<int8_t>(0x210F, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to read temperature: " + std::string(e.what()));
  }
}

uint16_t RoboteqDriver::ReadVoltage()
{
  try {
    return SyncSdoRead<uint16_t>(0x210D, 2);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to read voltage: " + std::string(e.what()));
  }
}

int16_t RoboteqDriver::ReadBatAmps1()
{
  try {
    return SyncSdoRead<int16_t>(0x210C, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to read bat amps 1: " + std::string(e.what()));
  }
}

int16_t RoboteqDriver::ReadBatAmps2()
{
  try {
    return SyncSdoRead<int16_t>(0x210C, 2);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to read bat amps 2: " + std::string(e.what()));
  }
}

RoboteqDriverFeedback RoboteqDriver::ReadRoboteqDriverFeedback()
{
  RoboteqDriverFeedback fb;

  // uint32_t
  // already does locking when accessing rpdo
  fb.motor_1.pos = rpdo_mapped[0x2106][1];
  fb.motor_2.pos = rpdo_mapped[0x2106][2];

  fb.motor_1.vel = rpdo_mapped[0x2106][3];
  fb.motor_2.vel = rpdo_mapped[0x2106][4];

  fb.motor_1.current = rpdo_mapped[0x2106][5];
  fb.motor_2.current = rpdo_mapped[0x2106][6];

  fb.fault_flags = GetByte(rpdo_mapped[0x2106][7], 0);
  fb.script_flags = GetByte(rpdo_mapped[0x2106][7], 2);

  fb.runtime_stat_flag_motor_1 = GetByte(rpdo_mapped[0x2106][8], 0);
  fb.runtime_stat_flag_motor_2 = GetByte(rpdo_mapped[0x2106][8], 1);

  std::unique_lock<std::mutex> lck(rpdo_timestamp_mtx_);
  fb.timestamp = last_rpdo_write_timestamp_;

  return fb;
}

// TODO check what happens when publishing is stopped
void RoboteqDriver::SendRoboteqCmdChannel1(int32_t cmd)
{
  try {
    SyncSdoWrite<int32_t>(0x2000, 1, cmd);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to send channel 1 Roboteq command: " + std::string(e.what()));
  }
}

void RoboteqDriver::SendRoboteqCmdChannel2(int32_t cmd)
{
  try {
    SyncSdoWrite<int32_t>(0x2000, 2, cmd);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to send channel 2 Roboteq command: " + std::string(e.what()));
  }
}

void RoboteqDriver::ResetRoboteqScript()
{
  try {
    SyncSdoWrite<uint8_t>(0x2018, 0, 2);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to reset Roboteq script: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnEstop()
{
  // Cmd_ESTOP
  try {
    SyncSdoWrite<uint8_t>(0x200C, 0, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn on estop: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOffEstop()
{
  // Cmd_MGO
  try {
    SyncSdoWrite<uint8_t>(0x200D, 0, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn off estop: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnSafetyStopChannel1()
{
  // Cmd_SFT Safety Stop
  try {
    SyncSdoWrite<uint8_t>(0x202C, 0, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to turn on safety stop on channel 1: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnSafetyStopChannel2()
{
  // Cmd_SFT Safety Stop
  try {
    SyncSdoWrite<uint8_t>(0x202C, 0, 2);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to turn on safety stop on channel 2: " + std::string(e.what()));
  }
}

template <typename type>
type RoboteqDriver::SyncSdoRead(uint16_t index, uint8_t subindex)
{
  // TODO: describe edge case
  std::unique_lock<std::mutex> sdo_read_lk(sdo_read_mtx_, std::defer_lock);
  if (!sdo_read_lk.try_lock()) {
    throw std::runtime_error(
      "Can't submit new SDO read operation - the previous one is still being processed");
  }

  std::mutex mtx;
  std::condition_variable cv;
  type data;
  std::error_code err_code;

  if (sdo_read_timed_out_) {
    throw std::runtime_error(
      "Can't submit new SDO read operation - previous one that timed out is still in queue");
  }

  try {
    this->SubmitRead<type>(
      index, subindex,
      [&sdo_read_timed_out_ = sdo_read_timed_out_, &mtx, &cv, &err_code, &data](
        uint8_t, uint16_t, uint8_t, std::error_code ec, type value) mutable {
        // In this case function has already finished, and other variables don't exist
        // and we have to end

        // TODO in timeout it won't be reached
        if (sdo_read_timed_out_) {
          sdo_read_timed_out_.store(false);
          return;
        }
        {
          std::lock_guard lck(mtx);
          if (ec) {
            err_code = ec;
          } else {
            data = value;
          }
        }
        cv.notify_one();
      },
      sdo_operation_timeout_);
  } catch (const lely::canopen::SdoError & e) {
    throw std::runtime_error("SDO read error, message: " + std::string(e.what()));
  }

  // TODO: lk vs lck
  std::unique_lock lk(mtx);
  if (cv.wait_for(lk, sdo_operation_wait_timeout_) == std::cv_status::timeout) {
    sdo_read_timed_out_.store(true);
    throw std::runtime_error("Timeout while waiting for finish of SDO read operation");
  }

  if (err_code) {
    throw std::runtime_error("Error msg: " + err_code.message());
  }

  return data;
}

template <typename type>
void RoboteqDriver::SyncSdoWrite(uint16_t index, uint8_t subindex, type data)
{
  std::unique_lock<std::mutex> sdo_write_lk(sdo_write_mtx_, std::defer_lock);
  if (!sdo_write_lk.try_lock()) {
    throw std::runtime_error(
      "Can't submit new SDO write operation - the previous one is still being processed");
  }

  std::mutex mtx;
  std::condition_variable cv;
  std::error_code err_code;

  if (sdo_write_timed_out_) {
    throw std::runtime_error(
      "Can't submit new SDO write operation - previous one that timed out is still in queue");
  }

  try {
    this->SubmitWrite(
      index, subindex, data,
      [&sdo_write_timed_out_ = sdo_write_timed_out_, &mtx, &cv, &err_code](
        uint8_t, uint16_t, uint8_t, std::error_code ec) mutable {
        // In this case function has already finished, and other variables don't exist
        // and we have to end
        if (sdo_write_timed_out_) {
          sdo_write_timed_out_.store(false);
          return;
        }
        {
          std::lock_guard lck(mtx);
          if (ec) {
            err_code = ec;
          }
        }
        cv.notify_one();
      },
      sdo_operation_timeout_);
  } catch (const lely::canopen::SdoError & e) {
    throw std::runtime_error("SDO write error, message: " + std::string(e.what()));
  }

  std::unique_lock lk(mtx);

  if (cv.wait_for(lk, sdo_operation_wait_timeout_) == std::cv_status::timeout) {
    sdo_write_timed_out_.store(true);
    throw std::runtime_error("Timeout while waiting for finish of SDO write operation");
  }

  if (err_code) {
    throw std::runtime_error("Error msg: " + err_code.message());
  }
}

void RoboteqDriver::OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept
{
  FiberDriver::OnBoot(st, es, what);

  if (!es || es == 'L') {
    booted_.store(true);
  }

  std::unique_lock<std::mutex> lck(boot_mtx_);
  this->boot_error_str_ = what;
  boot_cond_var_.notify_all();
}

void RoboteqDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
  if (idx == 0x2106 && subidx == 1) {
    std::unique_lock<std::mutex> lck(rpdo_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &last_rpdo_write_timestamp_);
  }
}

}  // namespace panther_hardware_interfaces
