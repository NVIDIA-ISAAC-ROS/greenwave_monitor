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
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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
  struct TopicConfig
  {
    std::optional<double> expected_frequency;
    std::optional<double> tolerance;
  };

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

  bool set_topic_expected_frequency(
    const std::string & topic_name,
    double expected_hz,
    double tolerance_percent,
    bool add_topic_if_missing,
    std::string & message,
    bool update_parameters = true);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);

  void apply_topic_config_if_complete(const std::string & topic_name);

  void load_topic_parameters_from_overrides();

  std::optional<double> get_numeric_parameter(const std::string & param_name);

  void try_undeclare_parameter(const std::string & param_name);

  void declare_or_set_parameter(const std::string & param_name, double value);

  void undeclare_topic_parameters(const std::string & topic_name);

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

  std::map<std::string, TopicConfig> pending_topic_configs_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Mutex protecting message_diagnostics_, subscriptions_, and pending_topic_configs_
  mutable std::mutex state_mutex_;
  // Flag to skip parameter callback when updating params internally (avoids redundant work)
  bool updating_params_internally_ = false;
};
