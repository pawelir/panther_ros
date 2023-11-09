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

#ifndef PANTHER_BATTERY_TEST_ADC_NODE_

#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/adc_node.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;

class ADCNodeWrapper : public panther_battery::ADCNode
{
public:
  ADCNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ADCNode(node_name, options)
  {
  }

  BatteryStateMsg MergeBatteryMsgs(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2)
  {
    return ADCNode::MergeBatteryMsgs(battery_msg_1, battery_msg_2);
  }
};

class TestADCNode : public testing::Test
{
public:
  TestADCNode(const bool dual_battery = false);
  ~TestADCNode();

protected:
  template <typename T>
  void WriteNumberToFile(const T number, const std::string file_path);

  std::filesystem::path device0_path_;
  std::filesystem::path device1_path_;
  BatteryStateMsg::SharedPtr battery_state_;
  BatteryStateMsg::SharedPtr battery_1_state_;
  BatteryStateMsg::SharedPtr battery_2_state_;
  std::shared_ptr<ADCNodeWrapper> adc_node_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_1_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_2_sub_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
};

TestADCNode::TestADCNode(const bool dual_battery)
{
  device0_path_ = std::filesystem::path(testing::TempDir()) / "device0";
  device1_path_ = std::filesystem::path(testing::TempDir()) / "device1";

  // Create the device0 and device1 directories if they do not exist
  std::filesystem::create_directory(device0_path_);
  std::filesystem::create_directory(device1_path_);

  // create only files that are required for adc_node to start
  int dual_bat_volt = dual_battery ? 800 : 1600;
  WriteNumberToFile<int>(dual_bat_volt, std::filesystem::path(device0_path_ / "in_voltage0_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(device0_path_ / "in_voltage1_raw"));
  WriteNumberToFile<int>(2, std::filesystem::path(device0_path_ / "in_voltage2_raw"));
  WriteNumberToFile<int>(2, std::filesystem::path(device0_path_ / "in_voltage3_raw"));
  WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage0_scale"));
  WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage1_scale"));
  WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage2_scale"));
  WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage3_scale"));

  WriteNumberToFile<int>(1400, std::filesystem::path(device1_path_ / "in_voltage0_raw"));
  WriteNumberToFile<int>(600, std::filesystem::path(device1_path_ / "in_voltage1_raw"));
  WriteNumberToFile<int>(600, std::filesystem::path(device1_path_ / "in_voltage2_raw"));
  WriteNumberToFile<int>(1400, std::filesystem::path(device1_path_ / "in_voltage3_raw"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(device1_path_ / "in_voltage0_scale"));
  WriteNumberToFile<float>(2.0, std::filesystem::path(device1_path_ / "in_voltage1_scale"));
  WriteNumberToFile<float>(2.0, std::filesystem::path(device1_path_ / "in_voltage2_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(device1_path_ / "in_voltage3_scale"));

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("adc0_device", device0_path_));
  params.push_back(rclcpp::Parameter("adc1_device", device1_path_));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  adc_node_ = std::make_shared<ADCNodeWrapper>("adc_node", options);

  battery_sub_ = adc_node_->create_subscription<BatteryStateMsg>(
    "battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
  battery_1_sub_ = adc_node_->create_subscription<BatteryStateMsg>(
    "battery_1_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_1_state_ = msg; });
  battery_2_sub_ = adc_node_->create_subscription<BatteryStateMsg>(
    "battery_2_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_2_state_ = msg; });

  io_state_pub_ = adc_node_->create_publisher<IOStateMsg>("hardware/io_state", 10);
}

TestADCNode::~TestADCNode()
{
  // Delete the devices directories
  if (std::filesystem::exists(device0_path_)) {
    std::filesystem::remove_all(device0_path_);
  }
  if (std::filesystem::exists(device1_path_)) {
    std::filesystem::remove_all(device1_path_);
  }
}

template <typename T>
void TestADCNode::WriteNumberToFile(const T number, const std::string file_path)
{
  std::ofstream file(file_path);
  if (file.is_open()) {
    file << number;
    file.close();
  } else {
    throw std::runtime_error("Failed to create file: " + file_path);
  }
}

#endif  // PANTHER_BATTERY_TEST_ADC_NODE_