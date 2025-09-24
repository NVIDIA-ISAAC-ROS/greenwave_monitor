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

/**
Unit tests for minimal publisher node.
**/

#include <gtest/gtest.h>

#include "minimal_publisher_node.hpp"

class MinimalPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(MinimalPublisherTest, TestDefaultParameters) {
  // Create the publisher with default parameters
  const MinimalPublisher node;

  EXPECT_EQ(node.get_parameter("topic").as_string(), "topic1");
  EXPECT_EQ(node.get_parameter("frequency_hz").as_double(), 1.0);
  EXPECT_EQ(node.get_parameter("message_type").as_string(), "imu");
  EXPECT_EQ(node.get_parameter("create_subscriber").as_bool(), false);
}

TEST_F(MinimalPublisherTest, TestCustomParameters) {
  // Create the publisher with custom parameters
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides(
  {
    {"topic", "custom_topic"},
    {"frequency_hz", 5.0},
    {"message_type", "image"},
    {"create_subscriber", false}
  });
  const MinimalPublisher node(options);

  EXPECT_EQ(node.get_parameter("topic").as_string(), "custom_topic");
  EXPECT_EQ(node.get_parameter("frequency_hz").as_double(), 5.0);
  EXPECT_EQ(node.get_parameter("message_type").as_string(), "image");
  EXPECT_EQ(node.get_parameter("create_subscriber").as_bool(), false);
}

TEST_F(MinimalPublisherTest, TestInvalidMessageType) {
  // Test that constructor exits with error for invalid message type

  // Set death test style to threadsafe to avoid forking issues
  // Use legacy API for compatibility across all ROS 2 distros
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides(
  {
    {"message_type", "invalid"}
  });

  EXPECT_DEATH({MinimalPublisher node(options);}, "Invalid message type: 'invalid'");
}

TEST_F(MinimalPublisherTest, TestImuTypePublishesImu) {
  // Test that when message_type=imu, the node publishes imu messages
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides(
  {
    {"message_type", "imu"},
    {"topic", "test_imu_topic"}
  });

  // Create the publisher and subscriber nodes
  const auto publisher = std::make_shared<MinimalPublisher>(options);
  const auto subscriber = std::make_shared<rclcpp::Node>("test_subscriber");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(subscriber);
  executor->add_node(publisher);

  // Subscribe to the topic and verify we receive imu messages
  sensor_msgs::msg::Imu::SharedPtr received_imu;
  const auto subscription = subscriber->create_subscription<sensor_msgs::msg::Imu>(
    "test_imu_topic", 10,
    [&received_imu](const sensor_msgs::msg::Imu::SharedPtr msg) {
      received_imu = msg;
    });

  // Allow messages to be published and received
  const auto start_time = std::chrono::steady_clock::now();
  while (!received_imu &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))
  {
    executor->spin_some(std::chrono::milliseconds(100));
  }

  // Verify an imu message was received
  ASSERT_NE(received_imu, nullptr);

  // Verify the message has the correct properties
  EXPECT_GT(received_imu->header.stamp.nanosec, 0);
  EXPECT_EQ(received_imu->orientation.x, 0.0);
  EXPECT_EQ(received_imu->orientation.y, 0.0);
  EXPECT_EQ(received_imu->orientation.z, 0.0);
  EXPECT_EQ(received_imu->orientation.w, 1.0);
  EXPECT_EQ(received_imu->angular_velocity.x, 0.0);
  EXPECT_EQ(received_imu->angular_velocity.y, 0.0);
  EXPECT_EQ(received_imu->angular_velocity.z, 0.0);
  EXPECT_EQ(received_imu->linear_acceleration.x, 0.0);
  EXPECT_EQ(received_imu->linear_acceleration.y, 0.0);
  EXPECT_EQ(received_imu->linear_acceleration.z, 0.0);
}

TEST_F(MinimalPublisherTest, TestImageTypePublishesImage) {
  // Test that when message_type=image the node publishes image messages
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides(
  {
    {"message_type", "image"},
    {"topic", "test_image_topic"}
  });

  // Create the publisher and subscriber nodes
  const auto publisher = std::make_shared<MinimalPublisher>(options);
  const auto subscriber = std::make_shared<rclcpp::Node>("test_subscriber");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(publisher);
  executor->add_node(subscriber);

  // Subscribe to the topic and verify we receive image messages
  sensor_msgs::msg::Image::SharedPtr received_image;
  const auto subscription = subscriber->create_subscription<sensor_msgs::msg::Image>(
    "test_image_topic", 10,
    [&received_image](const sensor_msgs::msg::Image::SharedPtr msg) {
      received_image = msg;
    });

  // Allow messages to be published and received
  const auto start_time = std::chrono::steady_clock::now();
  while (!received_image &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))
  {
    executor->spin_some(std::chrono::milliseconds(100));
  }

  // Verify an image message was received
  ASSERT_NE(received_image, nullptr);

  // Verify the message has the correct properties
  EXPECT_GT(received_image->header.stamp.nanosec, 0);
  EXPECT_EQ(received_image->height, 100);
  EXPECT_EQ(received_image->width, 100);
  EXPECT_EQ(received_image->encoding, "rgb8");
  EXPECT_EQ(received_image->data.size(), 100 * 100 * 3);
}

TEST_F(MinimalPublisherTest, TestStringTypePublishesString) {
  // Test that when message_type=string the node publishes string messages
  const rclcpp::NodeOptions options = rclcpp::NodeOptions().parameter_overrides(
  {
    {"message_type", "string"},
    {"topic", "test_string_topic"}
  });

  // Create the publisher and subscriber nodes
  const auto publisher = std::make_shared<MinimalPublisher>(options);
  const auto subscriber = std::make_shared<rclcpp::Node>("test_subscriber");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(publisher);
  executor->add_node(subscriber);

  // Subscribe to the topic and verify we receive image messages
  std_msgs::msg::String::SharedPtr received_string;
  const auto subscription = subscriber->create_subscription<std_msgs::msg::String>(
    "test_string_topic", 10,
    [&received_string](const std_msgs::msg::String::SharedPtr msg) {
      received_string = msg;
    });

  // Allow messages to be published and received
  const auto start_time = std::chrono::steady_clock::now();
  while (!received_string &&
    std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2))
  {
    executor->spin_some(std::chrono::milliseconds(100));
  }

  // Verify an string message was received
  ASSERT_NE(received_string, nullptr);

  // Verify the message has the correct properties
  const bool is_test_string = received_string->data.find("Test string! ") == 0;
  EXPECT_TRUE(is_test_string);
}
