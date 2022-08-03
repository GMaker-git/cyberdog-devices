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

#ifndef CYBERDOG_UWB__UWB_PLUGIN_HPP_
#define CYBERDOG_UWB__UWB_PLUGIN_HPP_

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <random>
#include <mutex>

#include "protocol/msg/uwb_raw.hpp"
#include "protocol/msg/uwb_array.hpp"
#include "cyberdog_uwb/uwb_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "embed_protocol/embed_protocol.hpp"
namespace cyberdog
{
namespace device
{

struct UWBHeadData
{
  uint8_t head_data_array[16];
  uint8_t head_tof_data_array[16];
  uint8_t version[2];

  // can0
  uint8_t head_enable_initial_ack;
  uint8_t head_enable_on_ack;
  uint8_t head_enable_off_ack;
  uint8_t head_tof_enable_initial_ack;
  uint8_t head_tof_enable_on_ack;
  uint8_t head_tof_enable_off_ack;
};

struct UWBRearData
{
  uint8_t rear_data_array[16];
  uint8_t rear_tof_data_array[16];
  uint8_t version[2];

  // can1
  uint8_t rear_enable_initial_ack;
  uint8_t rear_enable_on_ack;
  uint8_t rear_enable_off_ack;
  uint8_t rear_tof_enable_initial_ack;
  uint8_t rear_tof_enable_on_ack;
  uint8_t rear_tof_enable_off_ack;
};


class UWBCarpo : public cyberdog::device::UWBBase
{
public:
  UWBCarpo();
  bool Config() override;
  bool Init(
    std::function<void(UwbRawStatusMsg)>
    function_callback, bool simulation = false) override;
  bool SelfCheck() override;
  bool RegisterTopic(std::function<void(UwbRawStatusMsg)> function_callback) override;

  bool Open();
  bool Close();
  bool Initialize();
  bool GetVersion();

private:
  void HandleCan0Messages(std::string & name, std::shared_ptr<cyberdog::device::UWBRearData> data);
  void HandleCan1Messages(std::string & name, std::shared_ptr<cyberdog::device::UWBHeadData> data);

  void RunTask();

  // CAN
  bool InitializeCanCommunication();

  // Dimulation Data for debug
  void RunSimulation();

  // Generate random number
  int GenerateRandomNumber(int start, int end);

  // Convert struct dat to ROS format
  UwbRawStatusMsg ToROS();

  // uwb raw data conver readable data
  int ConvertAngle(const int & angle);
  int ConvertRange(const int & rangle);

  // AOA
  inline float format_9_7(short data)  // NOLINT
  {
    return data * 1.0 / 128;
  }

  // RSSI
  inline float format_8_8(short data)  // NOLINT
  {
    return data * 1.0 / 256;
  }


  std::vector<float> FrontPose(const float & dist, const float & angle);
  std::vector<float> BackPose(const float & dist, const float & angle);
  std::vector<float> RightPose(const float & dist, const float & angle);
  std::vector<float> LeftPose(const float & dist, const float & angle);  // NOLINT

  enum class Type
  {
    HeadTOF,
    HeadUWB,
    RearTOF,
    RearUWB
  };

  void SetData(const Type & type, const UwbSignleStatusMsg & data);
  void Debug2String(const Type & type);


  std::shared_ptr<cyberdog::embed::Protocol<UWBHeadData>> head_can_ptr_ {nullptr};
  std::shared_ptr<cyberdog::embed::Protocol<UWBRearData>> rear_can_ptr_ {nullptr};
  std::shared_ptr<std::thread> uwb_thread_ {nullptr};

  std::mutex mutex_;
  UwbRawStatusMsg ros_uwb_status_;
  std::function<void(UwbRawStatusMsg)> status_function_;

  // turn on initial flag
  bool head_enable_initial_ {false};
  bool head_tof_enable_initial_ {false};
  bool rear_enable_initial_ {false};
  bool rear_tof_enable_initial_ {false};

  // turn on flag
  bool head_turn_on_ {false};
  bool rear_turn_on_ {false};
  bool head_tof_turn_on_ {false};
  bool rear_tof_turn_on_ {false};

  // version flag
  bool head_version_ {false};
  bool head_tof_version_ {false};
  bool rear_version_ {false};
  bool rear_tof_version_ {false};

  bool simulation_ {false};
  bool initialized_finished_ {false};
  bool enable_initialized_finished_ {false};
};  //  class UWBCarpo
}   //  namespace device
}   //  namespace cyberdog

#endif  // CYBERDOG_UWB__UWB_PLUGIN_HPP_
