// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "message_diagnostics.hpp"
#include "greenwave_monitor_interfaces/srv/manage_topic.hpp"
#include "greenwave_monitor_interfaces/srv/set_expected_frequency.hpp"

class GreenwaveMonitor : public rclcpp::Node
{
public:
  explicit GreenwaveMonitor(const rclcpp::NodeOptions & options);

private:
  std::optional<std::string> find_topic_type_with_retry(
    const std::string & topic, const int max_retries, const int retry_period_s);

  void topic_callback(
    const std::shared_ptr<rclcpp::SerializedMessage> msg,
    const std::string & topic, const std::string & type);

  void timer_callback();

  void handle_manage_topic(
    const std::shared_ptr<greenwave_monitor_interfaces::srv::ManageTopic::Request> request,
    std::shared_ptr<greenwave_monitor_interfaces::srv::ManageTopic::Response> response);

  void handle_set_expected_frequency(
    const std::shared_ptr<greenwave_monitor_interfaces::srv::SetExpectedFrequency::Request> request,
    std::shared_ptr<greenwave_monitor_interfaces::srv::SetExpectedFrequency::Response> response);

  bool add_topic(const std::string & topic, std::string & message);

  bool remove_topic(const std::string & topic, std::string & message);

  bool has_header_from_type(const std::string & type_name);

  std::chrono::time_point<std::chrono::system_clock>
  GetTimestampFromSerializedMessage(
    std::shared_ptr<rclcpp::SerializedMessage> serialized_message_ptr,
    const std::string & type);

  std::map<std::string,
    std::unique_ptr<message_diagnostics::MessageDiagnostics>> message_diagnostics_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<greenwave_monitor_interfaces::srv::ManageTopic>::SharedPtr
    manage_topic_service_;
  rclcpp::Service<greenwave_monitor_interfaces::srv::SetExpectedFrequency>::SharedPtr
    set_expected_frequency_service_;
};
