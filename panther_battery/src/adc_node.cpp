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

#include <panther_battery/adc_node.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <panther_battery/adc_battery.hpp>
#include <panther_battery/adc_data_reader.hpp>
#include <panther_battery/battery.hpp>
#include <panther_battery/battery_publisher.hpp>
#include <panther_battery/dual_battery_publisher.hpp>
#include <panther_battery/single_battery_publisher.hpp>

namespace panther_battery
{
using std::placeholders::_1;

ADCNode::ADCNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<std::string>("adc0_device", "/sys/bus/iio/devices/iio:device0");
  this->declare_parameter<std::string>("adc1_device", "/sys/bus/iio/devices/iio:device1");
  this->declare_parameter<int>("ma_window_len/voltage", 10);
  this->declare_parameter<int>("ma_window_len/current", 10);
  this->declare_parameter<int>("ma_window_len/temp", 10);
  this->declare_parameter<int>("ma_window_len/charge", 10);

  // running at 10 Hz
  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ADCNode::BatteryPubTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void ADCNode::Initialize()
{
  const std::string adc0_device = this->get_parameter("adc0_device").as_string();
  const std::string adc1_device = this->get_parameter("adc1_device").as_string();

  adc0_reader_ = std::make_shared<ADCDataReader>(adc0_device);
  adc1_reader_ = std::make_shared<ADCDataReader>(adc1_device);

  const ADCBatteryParams battery_params = {
    static_cast<std::size_t>(this->get_parameter("ma_window_len/voltage").as_int()),
    static_cast<std::size_t>(this->get_parameter("ma_window_len/current").as_int()),
    static_cast<std::size_t>(this->get_parameter("ma_window_len/temp").as_int()),
    static_cast<std::size_t>(this->get_parameter("ma_window_len/charge").as_int()),
  };

  battery_2_ = std::make_shared<ADCBattery>(
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 3, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 1, kADCCurrentOffest),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 0, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 2, 0), battery_params);

  if (battery_2_->Present()) {
    RCLCPP_INFO(this->get_logger(), "Second battery detected");
    battery_1_ = std::make_shared<ADCBattery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 2, kADCCurrentOffest),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), battery_params);
    battery_publisher_ = std::make_shared<DualBatteryPublisher>(
      this->shared_from_this(), battery_1_, battery_2_);
  } else {
    battery_2_.reset();
    battery_1_ = std::make_shared<ADCBattery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      [&]() {
        return adc1_reader_->GetADCMeasurement(2, kADCCurrentOffest) +
               adc1_reader_->GetADCMeasurement(1, kADCCurrentOffest);
      },
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), battery_params);
    battery_publisher_ = std::make_shared<SingleBatteryPublisher>(
      this->shared_from_this(), battery_1_);
  }

  RCLCPP_INFO(this->get_logger(), "Battery publisher initialized");
}

void ADCNode::BatteryPubTimerCB()
{
  if (!battery_publisher_) {
    Initialize();
  }
  battery_publisher_->Publish();
}

}  // namespace panther_battery