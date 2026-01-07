// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "greenwave_diagnostics.hpp"
#include "greenwave_monitor_interfaces/srv/manage_topic.hpp"

class GreenwaveMonitor : public rclcpp::Node
{
public:
  explicit GreenwaveMonitor(const rclcpp::NodeOptions & options);

private:
  void topic_callback(
    const std::shared_ptr<rclcpp::SerializedMessage> msg,
    const std::string & topic, const std::string & type);

  void timer_callback();

  void handle_manage_topic(
    const std::shared_ptr<greenwave_monitor_interfaces::srv::ManageTopic::Request> request,
    std::shared_ptr<greenwave_monitor_interfaces::srv::ManageTopic::Response> response);

  bool add_topic(
    const std::string & topic, std::string & message,
    int max_retries = 5, double retry_period_s = 1.0);

  bool remove_topic(const std::string & topic, std::string & message);

  bool try_set_external_enabled_param(
    const std::string & topic, bool enabled, std::string & message);

  bool has_header_from_type(const std::string & type_name);

  std::set<std::string> get_topics_from_parameters();

  std::chrono::time_point<std::chrono::system_clock>
  GetTimestampFromSerializedMessage(
    std::shared_ptr<rclcpp::SerializedMessage> serialized_message_ptr,
    const std::string & type);

  std::map<std::string,
    std::unique_ptr<greenwave_diagnostics::GreenwaveDiagnostics>> greenwave_diagnostics_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<greenwave_monitor_interfaces::srv::ManageTopic>::SharedPtr
    manage_topic_service_;
};
