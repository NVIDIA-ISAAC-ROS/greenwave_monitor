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

#include <gtest/gtest.h>

#include "example_greenwave_publisher_node.hpp"

class ExampleGreenwavePublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(ExampleGreenwavePublisherTest, TestDefaultParameters) {
  const ExampleGreenwavePublisherNode node;
  EXPECT_EQ(node.get_parameter("topic").as_string(), "/example_imu");
  EXPECT_EQ(node.get_parameter("frequency_hz").as_double(), 30.0);
}

TEST_F(ExampleGreenwavePublisherTest, TestPublishesImuMessage) {
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides(
  {
    {"topic", "/test_example_imu"},
    {"frequency_hz", 20.0}
  });

  const auto publisher = std::make_shared<ExampleGreenwavePublisherNode>(options);
  const auto subscriber = std::make_shared<rclcpp::Node>("example_test_subscriber");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(subscriber);

  sensor_msgs::msg::Imu::SharedPtr received_message;
  auto subscription = subscriber->create_subscription<sensor_msgs::msg::Imu>(
    "/test_example_imu", 10,
    [&received_message](const sensor_msgs::msg::Imu::SharedPtr msg) {
      received_message = msg;
    });
  (void)subscription;

  const auto start_time = std::chrono::steady_clock::now();
  while (!received_message &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))
  {
    executor.spin_some(std::chrono::milliseconds(100));
  }

  ASSERT_NE(received_message, nullptr);
  EXPECT_GT(received_message->header.stamp.nanosec, 0U);
  EXPECT_EQ(received_message->orientation.w, 1.0);
}
