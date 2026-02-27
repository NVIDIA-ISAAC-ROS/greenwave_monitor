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

/**
Unit tests for functionality in greenwave_diagnostics.hpp,
such as frame rate and latency calculation accuracy.
**/

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <string>

#include "greenwave_diagnostics.hpp"

namespace test_constants
{
inline constexpr uint64_t kMillisecondsToSeconds = 1000ULL;
inline constexpr uint64_t kStartTimestampNs = 10000000ULL;
}  // namespace test_constants

class GreenwaveDiagnosticsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(GreenwaveDiagnosticsTest, FrameRateHdrTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.has_hdr_timestamp = true;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  uint64_t timestamp = test_constants::kStartTimestampNs;  // in nanoseconds
  for (int i = 0; i < 1000; i++) {
    greenwave_diagnostics.updateDiagnostics(timestamp);
    timestamp += 10000000;  // 10 ms in nanoseconds
  }
  EXPECT_EQ(greenwave_diagnostics.getFrameRateHdr(), 100);  // 100 Hz
}

TEST_F(GreenwaveDiagnosticsTest, FrameRateNodeTest)
{
  // Initialize GreenwaveDiagnostics
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", greenwave_diagnostics::GreenwaveDiagnosticsConfig());

  // dummy timestamp, not used for node time calculation
  constexpr auto timestamp = test_constants::kStartTimestampNs;
  const auto start_time = std::chrono::high_resolution_clock::now();

  constexpr int num_messages = 1000;
  constexpr int interarrival_time_ms = 10;  // 100 hz

  for (int i = 0; i < num_messages; i++) {
    greenwave_diagnostics.updateDiagnostics(timestamp);
    std::this_thread::sleep_for(std::chrono::milliseconds(interarrival_time_ms));
  }

  const auto end_time = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> total_duration = end_time - start_time;

  const double expected_frame_rate = static_cast<double>(num_messages) / total_duration.count();

  // allow 2.0 Hz error
  EXPECT_NEAR(greenwave_diagnostics.getFrameRateNode(), expected_frame_rate, 2.0);
}

TEST_F(GreenwaveDiagnosticsTest, MessageLatencyTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::HeaderOnly;
  config.has_hdr_timestamp = true;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  const rclcpp::Time current_time = node_->get_clock()->now();
  // Make header timestamp a certain amount of time earlier than current time
  constexpr double expected_latency_ms = 10.0;
  const rclcpp::Time hdr_timestamp =
    current_time - rclcpp::Duration::from_seconds(
    expected_latency_ms / static_cast<double>(test_constants::kMillisecondsToSeconds));

  greenwave_diagnostics.updateDiagnostics(hdr_timestamp.nanoseconds());

  // allow 1 ms tolerance
  EXPECT_NEAR(greenwave_diagnostics.getLatency(), expected_latency_ms, 1.0);
}

TEST_F(GreenwaveDiagnosticsTest, DiagnosticPublishSubscribeTest)
{
  constexpr int input_frequency = 50;  // 50 Hz
  // 20 ms in nanoseconds
  const int64_t interarrival_time_ns = static_cast<int64_t>(
    ::greenwave_diagnostics::constants::kSecondsToNanoseconds / input_frequency);

  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.enable_hdr_time_diagnostics = true;
  config.enable_node_time_diagnostics = true;
  config.enable_fps_jitter_hdr_time_diagnostics = true;
  config.enable_increasing_hdr_time_diagnostics = true;
  config.has_hdr_timestamp = true;
  config.expected_dt_us = interarrival_time_ns /
    ::greenwave_diagnostics::constants::kMicrosecondsToNanoseconds;

  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(*node_, "test_topic", config);

  // Create a subscriber to receive diagnostic messages
  std::vector<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> received_diagnostics;
  const auto diagnostic_subscription =
    node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&received_diagnostics](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      received_diagnostics.push_back(msg);
    });

  // 50 ms delay
  constexpr int64_t delay_time_ns = 50 *
    static_cast<int64_t>(::greenwave_diagnostics::constants::kMillisecondsToMicroseconds) *
    static_cast<int64_t>(::greenwave_diagnostics::constants::kMicrosecondsToNanoseconds);
  // Starting header timestamp in nanoseconds
  auto hdr_timestamp = test_constants::kStartTimestampNs;

  int sent_count = 0;
  const auto start_time = std::chrono::high_resolution_clock::now();

  // Send 100 messages
  constexpr int num_messages = 100;
  while (sent_count < num_messages) {
    if (sent_count != 0) {
      hdr_timestamp += interarrival_time_ns;
    }

    sent_count++;

    greenwave_diagnostics.updateDiagnostics(hdr_timestamp);
    greenwave_diagnostics.publishDiagnostics();

    // Add a non-increasing timestamp at count 5
    if (sent_count == 5) {
      hdr_timestamp -= interarrival_time_ns;
    }
    // Add a jitter by delaying at count 10
    if (sent_count == 10) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(delay_time_ns));  // 50 ms delay
      hdr_timestamp += delay_time_ns;
    }

    rclcpp::spin_some(node_);

    std::this_thread::sleep_for(std::chrono::nanoseconds(interarrival_time_ns));
  }

  ASSERT_EQ(received_diagnostics.size(), num_messages);

  const int interarrival_time_count = sent_count - 1;
  // Calculate expected node and message frame rates
  const auto actual_end_time = std::chrono::high_resolution_clock::now();
  const std::chrono::duration<double> total_duration = actual_end_time - start_time;
  const double expected_frame_rate_node = static_cast<double>(interarrival_time_count) /
    total_duration.count();

  const auto sum_interarrival_time_hdr_sec = static_cast<double>(
    hdr_timestamp - test_constants::kStartTimestampNs) /
    static_cast<double>(::greenwave_diagnostics::constants::kSecondsToNanoseconds);
  const double expected_frame_rate_hdr =
    static_cast<double>(interarrival_time_count) / sum_interarrival_time_hdr_sec;

  // Verify that we received diagnostic messages
  ASSERT_FALSE(received_diagnostics.empty());

  // Use the last diagnostic message
  const auto & last_diagnostic = received_diagnostics.back();
  ASSERT_FALSE(last_diagnostic->status.empty());

  // Verify the diagnostic status information
  const auto & diagnostic_status = last_diagnostic->status[0];
  EXPECT_TRUE(diagnostic_status.name.find("test_topic") != std::string::npos);
  EXPECT_EQ(diagnostic_status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostic_status.message, "FRAME DROP DETECTED, NONINCREASING TIMESTAMP");

  // Parse diagnostic values
  std::map<std::string, double> diagnostics_values = {
    {"frame_rate_node", 0.0},
    {"num_non_increasing_hdr", 0.0},
    {"num_jitter_outliers_hdr", 0.0},
    {"num_jitter_outliers_node", 0.0},
    {"max_abs_jitter_hdr", 0.0},
    {"max_abs_jitter_node", 0.0},
    {"mean_abs_jitter_hdr", 0.0},
    {"mean_abs_jitter_node", 0.0},
    {"frame_rate_hdr", 0.0},
    {"total_dropped_frames", 0.0}
  };
  for (const auto & value : diagnostic_status.values) {
    if (diagnostics_values.find(value.key) != diagnostics_values.end()) {
      diagnostics_values[value.key] = std::stod(value.value);
    }
  }

  // Sometimes diagnostics may arrive out of order, so we use getter methods instead of values from
  //  the last diagnostic message to prevent flakiness
  EXPECT_NEAR(greenwave_diagnostics.getFrameRateNode(), expected_frame_rate_node, 1.0);
  // Allow small floating point differences for frame rate hdr
  constexpr double frame_rate_hdr_tolerance = 0.001;
  EXPECT_NEAR(
    greenwave_diagnostics.getFrameRateHdr(), expected_frame_rate_hdr, frame_rate_hdr_tolerance);

  // Sometimes diagnostics may arrive out of order, so we need to check all received diagnostics
  //  to see if the expected hdr frame rate is somewhere in there
  double smallest_hdr_frame_rate_diff = std::numeric_limits<double>::infinity();
  for (const auto & diag_msg : received_diagnostics) {
    if (diag_msg->status.empty()) {
      continue;
    }
    const auto & status = diag_msg->status[0];
    double frame_rate_hdr = 0.0;
    for (const auto & value : status.values) {
      if (value.key == "frame_rate_hdr") {
        frame_rate_hdr = std::stod(value.value);
        break;
      }
    }
    if (std::abs(frame_rate_hdr - expected_frame_rate_hdr) < smallest_hdr_frame_rate_diff) {
      smallest_hdr_frame_rate_diff = std::abs(frame_rate_hdr - expected_frame_rate_hdr);
    }
  }

  EXPECT_LT(smallest_hdr_frame_rate_diff, frame_rate_hdr_tolerance);

  // Diagnostics should have at least one jitter due to the intentional delay
  //  possibly more if the system was very busy
  EXPECT_GE(diagnostics_values["num_jitter_outliers_node"], 1.0);
  EXPECT_GE(diagnostics_values["max_abs_jitter_node"], 0.0);
  EXPECT_GE(diagnostics_values["mean_abs_jitter_node"], 0.0);

  EXPECT_GE(diagnostics_values["num_jitter_outliers_hdr"], 1.0);
  EXPECT_GE(diagnostics_values["max_abs_jitter_hdr"], 0.0);
  EXPECT_GE(diagnostics_values["mean_abs_jitter_hdr"], 0.0);

  EXPECT_GE(diagnostics_values["total_dropped_frames"], 1.0);
  EXPECT_GE(diagnostics_values["num_non_increasing_hdr"], 1.0);
}

static diagnostic_msgs::msg::DiagnosticStatus run_fps_sequence(
  const std::shared_ptr<rclcpp::Node> & node,
  const greenwave_diagnostics::GreenwaveDiagnosticsConfig & config,
  double expected_hz,
  double tolerance_percent,
  double actual_hz)
{
  greenwave_diagnostics::GreenwaveDiagnostics diagnostics(*node, "test_topic", config);
  diagnostics.setExpectedDt(expected_hz, tolerance_percent);
  std::vector<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> received_diagnostics;
  const auto diagnostic_subscription =
    node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [&received_diagnostics](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      received_diagnostics.push_back(msg);
    });
  (void)diagnostic_subscription;

  constexpr int samples = 15;
  const int64_t input_dt_ns = static_cast<int64_t>(
    1e9 / expected_hz);
  const auto sleep_duration = std::chrono::duration<double>(1.0 / actual_hz);
  uint64_t hdr_timestamp = test_constants::kStartTimestampNs;
  for (int i = 0; i < samples; ++i) {
    diagnostics.updateDiagnostics(hdr_timestamp);
    diagnostics.publishDiagnostics();
    rclcpp::spin_some(node);
    hdr_timestamp += static_cast<uint64_t>(input_dt_ns);
    std::this_thread::sleep_for(sleep_duration);
  }

  if (received_diagnostics.empty() || received_diagnostics.back()->status.empty()) {
    diagnostic_msgs::msg::DiagnosticStatus empty_status;
    empty_status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    return empty_status;
  }

  return received_diagnostics.back()->status[0];
}

TEST_F(GreenwaveDiagnosticsTest, HeaderlessFallbackRaisesOutOfRangeFpsError)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::HeaderWithFallback;
  config.has_hdr_timestamp = false;

  const auto status = run_fps_sequence(node_, config, 100.0, 10.0, 40.0);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(status.message.find("FPS OUT OF RANGE (NODE TIME)"), std::string::npos);
}

TEST_F(GreenwaveDiagnosticsTest, HeaderOnlyDoesNotUseFallbackForHeaderlessTopics)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::HeaderOnly;
  config.has_hdr_timestamp = false;

  const auto status = run_fps_sequence(node_, config, 100.0, 10.0, 40.0);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "OK");
}

TEST_F(GreenwaveDiagnosticsTest, FallbackModeDoesNotUseFpsWindowForHeaderedTopics)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::HeaderWithFallback;
  config.has_hdr_timestamp = true;

  const auto status = run_fps_sequence(node_, config, 100.0, 10.0, 100.0);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "OK");
}

TEST_F(GreenwaveDiagnosticsTest, NodetimeOnlyUsesFpsWindowForHeaderedTopics)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::NodetimeOnly;
  config.has_hdr_timestamp = true;

  const auto status = run_fps_sequence(node_, config, 100.0, 10.0, 40.0);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(status.message.find("FPS OUT OF RANGE (NODE TIME)"), std::string::npos);
}

TEST_F(GreenwaveDiagnosticsTest, TimeCheckPresetNone)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::None;
  config.applyTimeCheckPreset();
  EXPECT_FALSE(config.enable_node_time_diagnostics);
  EXPECT_FALSE(config.enable_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_jitter_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_jitter_node_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_window_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_window_node_time_diagnostics);
  EXPECT_FALSE(config.enable_increasing_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_increasing_node_time_diagnostics);
  EXPECT_FALSE(config.fallback_to_nodetime);
}

TEST_F(GreenwaveDiagnosticsTest, TimeCheckPresetHeaderOnly)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::HeaderOnly;
  config.applyTimeCheckPreset();
  EXPECT_FALSE(config.enable_node_time_diagnostics);
  EXPECT_TRUE(config.enable_hdr_time_diagnostics);
  EXPECT_TRUE(config.enable_fps_jitter_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_jitter_node_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_window_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_window_node_time_diagnostics);
  EXPECT_TRUE(config.enable_increasing_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_increasing_node_time_diagnostics);
  EXPECT_FALSE(config.fallback_to_nodetime);
}

TEST_F(GreenwaveDiagnosticsTest, TimeCheckPresetNodetimeOnly)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::NodetimeOnly;
  config.applyTimeCheckPreset();
  EXPECT_TRUE(config.enable_node_time_diagnostics);
  EXPECT_FALSE(config.enable_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_jitter_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_jitter_node_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_window_hdr_time_diagnostics);
  EXPECT_TRUE(config.enable_fps_window_node_time_diagnostics);
  EXPECT_FALSE(config.enable_increasing_hdr_time_diagnostics);
  EXPECT_TRUE(config.enable_increasing_node_time_diagnostics);
  EXPECT_FALSE(config.fallback_to_nodetime);
}

TEST_F(GreenwaveDiagnosticsTest, TimeCheckPresetHeaderWithFallbackNoHdrTimestamp)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::HeaderWithFallback;
  config.has_hdr_timestamp = false;
  config.applyTimeCheckPreset();
  EXPECT_TRUE(config.fallback_to_nodetime);
  EXPECT_TRUE(config.enable_node_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_jitter_node_time_diagnostics);
  EXPECT_TRUE(config.enable_fps_window_node_time_diagnostics);
  EXPECT_TRUE(config.enable_increasing_node_time_diagnostics);
  EXPECT_FALSE(config.enable_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_jitter_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_window_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_increasing_hdr_time_diagnostics);
}

TEST_F(GreenwaveDiagnosticsTest, TimeCheckPresetHeaderWithFallbackWithHdrTimestamp)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.time_check_preset = greenwave_diagnostics::TimeCheckPreset::HeaderWithFallback;
  config.has_hdr_timestamp = true;
  config.applyTimeCheckPreset();
  EXPECT_TRUE(config.fallback_to_nodetime);
  EXPECT_TRUE(config.enable_node_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_jitter_node_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_window_node_time_diagnostics);
  EXPECT_TRUE(config.enable_increasing_node_time_diagnostics);
  EXPECT_TRUE(config.enable_hdr_time_diagnostics);
  EXPECT_TRUE(config.enable_fps_jitter_hdr_time_diagnostics);
  EXPECT_FALSE(config.enable_fps_window_hdr_time_diagnostics);
  EXPECT_TRUE(config.enable_increasing_hdr_time_diagnostics);
}
