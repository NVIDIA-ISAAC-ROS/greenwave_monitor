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

#include "example_greenwave_publisher_node.hpp"

#include <cstdlib>

namespace
{
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
}

ExampleGreenwavePublisherNode::ExampleGreenwavePublisherNode(const rclcpp::NodeOptions & options)
: Node("example_greenwave_publisher_node", options)
{
  this->declare_parameter<std::string>("topic", "/example_imu");
  this->declare_parameter<double>("frequency_hz", 30.0);

  const auto topic = this->get_parameter("topic").as_string();
  const auto frequency_hz = this->get_parameter("frequency_hz").as_double();
  if (frequency_hz <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Parameter frequency_hz must be > 0.0");
    std::exit(EXIT_FAILURE);
  }

  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);

  greenwave_diagnostics::GreenwaveDiagnosticsConfig diagnostics_config;
  diagnostics_config.enable_all_topic_diagnostics = true;
  greenwave_diagnostics_ = std::make_unique<greenwave_diagnostics::GreenwaveDiagnostics>(
    *this, topic, diagnostics_config);

  publish_timer_ = this->create_wall_timer(
    std::chrono::nanoseconds(
      static_cast<int64_t>(kNanosecondsPerSecond / frequency_hz)),
    std::bind(&ExampleGreenwavePublisherNode::publish_message, this));

  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ExampleGreenwavePublisherNode::publish_diagnostics, this));
}

void ExampleGreenwavePublisherNode::publish_message()
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = this->now();
  message.header.frame_id = "example_frame_" + std::to_string(count_++);
  message.orientation.w = 1.0;

  publisher_->publish(message);
  const uint64_t stamp_ns = static_cast<uint64_t>(message.header.stamp.sec) *
    static_cast<uint64_t>(kNanosecondsPerSecond) + message.header.stamp.nanosec;
  greenwave_diagnostics_->updateDiagnostics(stamp_ns);
}

void ExampleGreenwavePublisherNode::publish_diagnostics()
{
  greenwave_diagnostics_->publishDiagnostics();
}
