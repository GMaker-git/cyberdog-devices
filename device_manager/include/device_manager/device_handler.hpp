// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef DEVICE_MANAGER__DEVICE_HANDLER_HPP_
#define DEVICE_MANAGER__DEVICE_HANDLER_HPP_

#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <map>
#include <functional>
#include <condition_variable>

#include "std_msgs/msg/int32.hpp"
#include "device_manager/device_config.hpp"

namespace cyberdog
{
namespace device
{
class DeviceHandler
{
public:
  DeviceHandler() {}
  ~DeviceHandler() {}

  void Config();
  bool Init(rclcpp::Node::SharedPtr node_ptr);
  bool SelfCheck();

public:
  void ExecuteLed(
    const protocol::srv::LedExecute_Request::SharedPtr request,
    protocol::srv::LedExecute_Response::SharedPtr response);
  void PublishTouch(protocol::msg::TouchStatus msg);
  void PublishBmsMessage(protocol::msg::Bms msg);

  // void HandleTestBMSCaseCallback(const std_msgs::msg::Int32 & msg);
  void HandleTestBMSCaseCallback(const std_msgs::msg::Int32::SharedPtr msg);

private:
  std::vector<std::string> device_vec_;
  std::map<std::string, std::string> device_map_;

private:
  std::shared_ptr<LedBase> led_ptr {nullptr};
  std::shared_ptr<TouchBase> touch_ptr {nullptr};
  std::shared_ptr<BMSBase> bms_ptr_ {nullptr};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bms_test_subscription_;

  rclcpp::Publisher<protocol::msg::TouchStatus>::SharedPtr touch_pub_ {nullptr};
  rclcpp::Publisher<protocol::msg::Bms>::SharedPtr bms_pub_ {nullptr};
};  // class DeviceHandler

}  // namespace device
}  // namespace cyberdog

#endif  // DEVICE_MANAGER__DEVICE_HANDLER_HPP_
