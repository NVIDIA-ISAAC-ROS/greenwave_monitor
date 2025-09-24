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

#include "minimal_publisher_node.hpp"

MinimalPublisher::MinimalPublisher(const rclcpp::NodeOptions & options)
: Node("minimal_publisher", options), count_(0)
{
  // Declare and get the topic and frequency parameters
  this->declare_parameter<std::string>("topic", "topic1");
  this->declare_parameter<double>("frequency_hz", 1.0);
  this->declare_parameter<std::string>("message_type", "imu");
  this->declare_parameter<bool>("create_subscriber", false);

  const auto topic = this->get_parameter("topic").as_string();
  const auto frequency_hz = this->get_parameter("frequency_hz").as_double();
  const auto period_ns = static_cast<int64_t>(
    ::message_diagnostics::constants::kSecondsToNanoseconds / frequency_hz);
  const auto create_subscriber = this->get_parameter("create_subscriber").as_bool();

  message_type_ = this->get_parameter("message_type").as_string();
  // Validate message type
  if (message_type_ != "image" && message_type_ != "imu" && message_type_ != "string") {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid message type: '%s'. Supported types are 'image', 'imu', and 'string'",
      message_type_.c_str());
    std::exit(EXIT_FAILURE);
  }

  // Setup subscription options for topic statistics
  auto statistics_options = rclcpp::SubscriptionOptions();
  statistics_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  statistics_options.topic_stats_options.publish_period = std::chrono::seconds(10);

  if (message_type_ == "image") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic, 10);
    if (create_subscriber) {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic, 10,
        [this](sensor_msgs::msg::Image::SharedPtr) {
          // Empty callback - we only care about statistics
        },
        statistics_options);
    }
  } else if (message_type_ == "imu") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
    if (create_subscriber) {
      subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        topic, 10,
        [this](sensor_msgs::msg::Imu::SharedPtr) {
          // Empty callback - we only care about statistics
        },
        statistics_options);
    }
  } else if (message_type_ == "string") {
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic, 10);
    if (create_subscriber) {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        topic, 10,
        [this](std_msgs::msg::String::SharedPtr) {
          // Empty callback - we only care about statistics
        },
        statistics_options);
    }
  }

  timer_ = this->create_wall_timer(
    std::chrono::nanoseconds(period_ns), std::bind(&MinimalPublisher::timer_callback, this));

  message_diagnostics::MessageDiagnosticsConfig diagnostics_config;
  diagnostics_config.enable_all_topic_diagnostics = true;
  message_diagnostics_ = std::make_unique<message_diagnostics::MessageDiagnostics>(
    *this, topic, diagnostics_config);
}

void MinimalPublisher::timer_callback()
{
  if (message_type_ == "image") {
    auto message = sensor_msgs::msg::Image();
    message.header.stamp = this->now();
    message.header.frame_id = "frame_" + std::to_string(count_++);
    message.height = height_;
    message.width = width_;
    message.encoding = "rgb8";
    message.is_bigendian = false;
    message.step = width_ * 3;
    message.data = image_data_;

    RCLCPP_INFO_ONCE(
      this->get_logger(), "Publishing image: frame_id='%s', stamp='%d'",
      message.header.frame_id.c_str(), message.header.stamp.nanosec);

    std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Image>>(publisher_)->publish(
      message);
  } else if (message_type_ == "imu") {
    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->now();
    message.header.frame_id = "frame_" + std::to_string(count_++);
    message.orientation.x = 0.0;
    message.orientation.y = 0.0;
    message.orientation.z = 0.0;
    message.orientation.w = 1.0;
    message.angular_velocity.x = 0.0;
    message.angular_velocity.y = 0.0;
    message.angular_velocity.z = 0.0;
    message.linear_acceleration.x = 0.0;
    message.linear_acceleration.y = 0.0;
    message.linear_acceleration.z = 0.0;

    RCLCPP_INFO_ONCE(
      this->get_logger(), "Publishing IMU: frame_id='%s', stamp='%d'",
      message.header.frame_id.c_str(), message.header.stamp.nanosec);

    std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Imu>>(publisher_)->publish(
      message);
  } else if (message_type_ == "string") {
    auto message = std_msgs::msg::String();
    message.data = "Test string! " + std::to_string(count_++);
    RCLCPP_INFO_ONCE(
      this->get_logger(), "Publishing string: '%s'", message.data.c_str());
    std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(publisher_)->publish(
      message);
  }

  const auto msg_timestamp = this->now();
  message_diagnostics_->updateDiagnostics(msg_timestamp.nanoseconds());
  // message_diagnostics_->publishDiagnostics();
}
