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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "message_diagnostics.hpp"
#include "rclcpp/subscription_options.hpp"

using std::chrono_literals::operator""ms;

class MinimalPublisher : public rclcpp::Node
{
public:
  explicit MinimalPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PublisherBase::SharedPtr publisher_;
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  std::unique_ptr<message_diagnostics::MessageDiagnostics> message_diagnostics_;
  size_t count_;
  std::string message_type_;

  // Image variables
  static constexpr int width_ = 100;
  static constexpr int height_ = 100;
  inline static const std::vector<uint8_t> image_data_ =
    std::vector<uint8_t>(width_ * height_ * 3, 128);
};
