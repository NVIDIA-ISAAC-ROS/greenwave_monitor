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
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "greenwave_diagnostics.hpp"

struct TopicValidationResult
{
  bool valid = false;
  std::string topic;
  std::string message_type;
  std::string error_message;
};

class GreenwaveMonitor : public rclcpp::Node
{
public:
  explicit GreenwaveMonitor(const rclcpp::NodeOptions & options);
  ~GreenwaveMonitor()
  {
    // Cancel timers first to stop callbacks from firing
    if (timer_) {
      timer_->cancel();
    }
    if (init_timer_) {
      init_timer_->cancel();
    }
    // Clear diagnostics before base Node destructor runs to avoid accessing invalid node state
    greenwave_diagnostics_.clear();
    subscriptions_.clear();
  }

private:
  void topic_callback(
    const std::shared_ptr<rclcpp::SerializedMessage> msg,
    const std::string & topic, const std::string & type);

  void timer_callback();

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);

  void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);

  void internal_on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);

  void external_on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);

  TopicValidationResult validate_add_topic(
    const std::string & topic, int max_retries = 5, double retry_period_s = 1.0);

  bool execute_add_topic(const TopicValidationResult & validated, std::string & message);

  void deferred_init();

  void fetch_external_topic_map();

  bool add_topic(
    const std::string & topic, std::string & message,
    int max_retries = 5, double retry_period_s = 1.0);

  bool remove_topic(const std::string & topic, std::string & message);

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
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::unordered_map<std::string, TopicValidationResult> pending_validations_;
  std::unordered_map<std::string, std::string> external_topic_to_node_;
};
