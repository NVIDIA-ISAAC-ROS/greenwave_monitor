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

#include <memory>
#include <string>

#include "greenwave_diagnostics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ExampleGreenwavePublisherNode : public rclcpp::Node
{
public:
  explicit ExampleGreenwavePublisherNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ExampleGreenwavePublisherNode()
  {
    if (publish_timer_) {
      publish_timer_->cancel();
    }
    if (diagnostics_timer_) {
      diagnostics_timer_->cancel();
    }
    greenwave_diagnostics_.reset();
  }

private:
  void publish_message();
  void publish_diagnostics();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<greenwave_diagnostics::GreenwaveDiagnostics> greenwave_diagnostics_;
  uint64_t count_ = 0;
};
