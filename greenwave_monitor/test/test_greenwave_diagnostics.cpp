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
#include <functional>

#include "greenwave_diagnostics.hpp"

namespace test_constants
{
inline constexpr uint64_t kMillisecondsToSeconds = 1000ULL;
inline constexpr uint64_t kStartTimestampNs = 10000000ULL;

// Short aliases for parameter suffixes
constexpr auto kFreq = greenwave_diagnostics::constants::kFreqSuffix;
constexpr auto kTol = greenwave_diagnostics::constants::kTolSuffix;
constexpr auto kEnabled = greenwave_diagnostics::constants::kEnabledSuffix;

inline std::string makeParam(const std::string & topic, const char * suffix)
{
  return std::string(greenwave_diagnostics::constants::kTopicParamPrefix) + topic + suffix;
}
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
    received_diagnostics_.clear();
    diagnostic_subscription_.reset();
    node_.reset();
  }

  void recreateNode()
  {
    received_diagnostics_.clear();
    diagnostic_subscription_.reset();
    node_.reset();
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void setupDiagnosticSubscription()
  {
    received_diagnostics_.clear();
    diagnostic_subscription_ =
      node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        received_diagnostics_.push_back(msg);
      });
  }

  void spinAndWait(int iterations = 5, int sleep_ms = 10)
  {
    for (int i = 0; i < iterations; i++) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
  }

  double getDiagnosticValue(const std::string & key) const
  {
    if (received_diagnostics_.empty() ||
      received_diagnostics_.back()->status.empty())
    {
      return 0.0;
    }
    for (const auto & val : received_diagnostics_.back()->status[0].values) {
      if (val.key == key) {
        return std::stod(val.value);
      }
    }
    return 0.0;
  }

  bool hasDiagnosticMessage(const std::string & message_substring) const
  {
    for (const auto & diag : received_diagnostics_) {
      if (!diag->status.empty() &&
        diag->status[0].message.find(message_substring) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  bool hasErrorStatusForTopic(const std::string & topic_name) const
  {
    for (const auto & diag : received_diagnostics_) {
      for (const auto & status : diag->status) {
        if (status.name.find(topic_name) != std::string::npos &&
          status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
        {
          return true;
        }
      }
    }
    return false;
  }

  template<typename T>
  void setParam(const std::string & param_name, const T & value)
  {
    auto result = node_->set_parameter(rclcpp::Parameter(param_name, value));
    EXPECT_TRUE(result.successful) << "Failed to set " << param_name << ": " << result.reason;
  }

  template<typename T>
  void setParamFail(
    const std::string & param_name,
    const T & value,
    const std::string & expected_error)
  {
    auto result = node_->set_parameter(rclcpp::Parameter(param_name, value));
    EXPECT_FALSE(result.successful) << "Expected " << param_name << " to fail";
    EXPECT_TRUE(result.reason.find(expected_error) != std::string::npos)
      << "Expected error containing '" << expected_error << "', got: " << result.reason;
  }

  void testBooleanParameterAcceptance(const std::string & param_name)
  {
    setParam(param_name, true);
    setParam(param_name, false);
  }

  // Send messages, optionally with publishing, jitter, sleep, and interval override callback
  // Callback signature: int64_t(int iteration, int64_t default_interval) -> interval to use
  void sendMessages(
    greenwave_diagnostics::GreenwaveDiagnostics & diag,
    uint64_t & timestamp,
    int64_t interval_ns,
    int count,
    bool publish = false,
    int64_t jitter_ns = 0,
    int sleep_ms = 0,
    const std::function<int64_t(int, int64_t)> & interval_override = nullptr)
  {
    for (int i = 0; i < count; i++) {
      diag.updateDiagnostics(timestamp);
      if (publish) {
        diag.publishDiagnostics();
        rclcpp::spin_some(node_);
      }
      int64_t jitter = (jitter_ns != 0) ? ((i % 2 == 0) ? jitter_ns : -jitter_ns) : 0;
      int64_t interval = interval_override ? interval_override(i, interval_ns) : interval_ns;
      timestamp += interval + jitter;
      if (sleep_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      }
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::vector<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> received_diagnostics_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscription_;
};

TEST_F(GreenwaveDiagnosticsTest, FrameRateMsgTest)
{
  greenwave_diagnostics::GreenwaveDiagnostics diag(
    *node_, "test_topic", greenwave_diagnostics::GreenwaveDiagnosticsConfig());

  uint64_t timestamp = test_constants::kStartTimestampNs;
  sendMessages(diag, timestamp, 10000000, 1000);  // 10 ms = 100 Hz
  EXPECT_EQ(diag.getFrameRateMsg(), 100);  // 100 Hz
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
  // Initialize GreenwaveDiagnostics
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", greenwave_diagnostics::GreenwaveDiagnosticsConfig());

  const rclcpp::Time current_time = node_->get_clock()->now();
  // Make message timestamp a certain amount of time earlier than current time
  constexpr double expected_latency_ms = 10.0;
  const rclcpp::Time msg_timestamp =
    current_time - rclcpp::Duration::from_seconds(
    expected_latency_ms / static_cast<double>(test_constants::kMillisecondsToSeconds));

  greenwave_diagnostics.updateDiagnostics(msg_timestamp.nanoseconds());

  // allow 1 ms tolerance
  EXPECT_NEAR(greenwave_diagnostics.getLatency(), expected_latency_ms, 1.0);
}

TEST_F(GreenwaveDiagnosticsTest, DiagnosticPublishSubscribeTest)
{
  constexpr int input_frequency = 50;  // 50 Hz
  // 20 ms in nanoseconds
  const int64_t interarrival_time_ns = static_cast<int64_t>(
    ::greenwave_diagnostics::constants::kSecondsToNanoseconds / input_frequency);

  // Initialize GreenwaveDiagnostics with diagnostics enabled
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.enable_msg_time_diagnostics = true;
  config.enable_node_time_diagnostics = true;
  config.enable_increasing_msg_time_diagnostics = true;
  // in us
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
  // Starting message timestamp in nanoseconds
  auto msg_timestamp = test_constants::kStartTimestampNs;

  int sent_count = 0;
  const auto start_time = std::chrono::high_resolution_clock::now();

  // Send 100 messages
  constexpr int num_messages = 100;
  while (sent_count < num_messages) {
    if (sent_count != 0) {
      msg_timestamp += interarrival_time_ns;
    }

    sent_count++;

    greenwave_diagnostics.updateDiagnostics(msg_timestamp);
    greenwave_diagnostics.publishDiagnostics();

    // Add a non-increasing timestamp at count 5
    if (sent_count == 5) {
      msg_timestamp -= interarrival_time_ns;
    }
    // Add a jitter by delaying at count 10
    if (sent_count == 10) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(delay_time_ns));  // 50 ms delay
      msg_timestamp += delay_time_ns;
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

  const auto sum_interarrival_time_msg_sec = static_cast<double>(
    msg_timestamp - test_constants::kStartTimestampNs) /
    static_cast<double>(::greenwave_diagnostics::constants::kSecondsToNanoseconds);
  const double expected_frame_rate_msg =
    static_cast<double>(interarrival_time_count) / sum_interarrival_time_msg_sec;

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
    {"num_non_increasing_msg", 0.0},
    {"num_jitter_outliers_msg", 0.0},
    {"num_jitter_outliers_node", 0.0},
    {"max_abs_jitter_msg", 0.0},
    {"max_abs_jitter_node", 0.0},
    {"mean_abs_jitter_msg", 0.0},
    {"mean_abs_jitter_node", 0.0},
    {"frame_rate_msg", 0.0},
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
  // Allow small floating point differences for frame rate msg
  constexpr double frame_rate_msg_tolerance = 0.001;
  EXPECT_NEAR(
    greenwave_diagnostics.getFrameRateMsg(), expected_frame_rate_msg, frame_rate_msg_tolerance);

  // Sometimes diagnostics may arrive out of order, so we need to check all received diagnostics
  //  to see if the expected msg frame rate is somewhere in there
  double smallest_msg_frame_rate_diff = std::numeric_limits<double>::infinity();
  for (const auto & diag_msg : received_diagnostics) {
    if (diag_msg->status.empty()) {
      continue;
    }
    const auto & status = diag_msg->status[0];
    double frame_rate_msg = 0.0;
    for (const auto & value : status.values) {
      if (value.key == "frame_rate_msg") {
        frame_rate_msg = std::stod(value.value);
        break;
      }
    }
    if (std::abs(frame_rate_msg - expected_frame_rate_msg) < smallest_msg_frame_rate_diff) {
      smallest_msg_frame_rate_diff = std::abs(frame_rate_msg - expected_frame_rate_msg);
    }
  }

  EXPECT_LT(smallest_msg_frame_rate_diff, frame_rate_msg_tolerance);

  // Diagnostics should have at least one jitter due to the intentional delay
  //  possibly more if the system was very busy
  EXPECT_GE(diagnostics_values["num_jitter_outliers_node"], 1.0);
  EXPECT_GE(diagnostics_values["max_abs_jitter_node"], 0.0);
  EXPECT_GE(diagnostics_values["mean_abs_jitter_node"], 0.0);

  EXPECT_GE(diagnostics_values["num_jitter_outliers_msg"], 1.0);
  EXPECT_GE(diagnostics_values["max_abs_jitter_msg"], 0.0);
  EXPECT_GE(diagnostics_values["mean_abs_jitter_msg"], 0.0);

  EXPECT_GE(diagnostics_values["total_dropped_frames"], 1.0);
  EXPECT_GE(diagnostics_values["num_non_increasing_msg"], 1.0);
}

TEST_F(GreenwaveDiagnosticsTest, InvalidFrequencyParameterTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  const std::string freq_param = test_constants::makeParam("test_topic", test_constants::kFreq);

  // Test negative frequency is rejected
  setParamFail(freq_param, -10.0, "must be positive");

  // Test zero frequency is rejected
  setParamFail(freq_param, 0.0, "must be positive");

  // Test NaN is accepted (clears the expected frequency)
  setParam(freq_param, std::numeric_limits<double>::quiet_NaN());

  // Test positive frequency is accepted
  setParam(freq_param, 30.0);
}

TEST_F(GreenwaveDiagnosticsTest, InvalidToleranceParameterTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  const std::string tol_param = test_constants::makeParam("test_topic", test_constants::kTol);

  // Test negative tolerance is rejected
  setParamFail(tol_param, -5.0, "must be non-negative");

  // Test zero tolerance is accepted
  setParam(tol_param, 0.0);

  // Test positive tolerance is accepted
  setParam(tol_param, 10.0);
}

TEST_F(GreenwaveDiagnosticsTest, EnabledParameterImpactTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic", config);

  const std::string enabled_param =
    test_constants::makeParam("test_topic", test_constants::kEnabled);
  constexpr int64_t interval_ns = 10000000;  // 10 ms

  // Send messages while enabled (default)
  uint64_t timestamp = test_constants::kStartTimestampNs;
  sendMessages(diag, timestamp, interval_ns, 100);
  EXPECT_GT(diag.getFrameRateMsg(), 0.0);

  // Disable diagnostics
  setParam(enabled_param, false);
  spinAndWait(10);

  // Frame rate should be cleared after disabling (windows are cleared)
  EXPECT_EQ(diag.getFrameRateMsg(), 0.0);

  // Send more messages while disabled (they should be ignored)
  sendMessages(diag, timestamp, interval_ns, 100);
  EXPECT_EQ(diag.getFrameRateMsg(), 0.0);

  // Re-enable diagnostics
  setParam(enabled_param, true);
  spinAndWait(10);

  // Send messages while re-enabled
  sendMessages(diag, timestamp, interval_ns, 100);
  EXPECT_GT(diag.getFrameRateMsg(), 0.0);
}

TEST_F(GreenwaveDiagnosticsTest, EnableMsgTimeConfigImpactTest)
{
  // Test that enable_msg_time_diagnostics config controls frame drop detection
  constexpr int input_frequency = 50;
  const int64_t interarrival_time_ns = static_cast<int64_t>(
    ::greenwave_diagnostics::constants::kSecondsToNanoseconds / input_frequency);

  // Large delay to trigger frame drop (100ms when expecting ~20ms)
  constexpr int64_t large_delay_ns = 100 *
    static_cast<int64_t>(::greenwave_diagnostics::constants::kMillisecondsToMicroseconds) *
    static_cast<int64_t>(::greenwave_diagnostics::constants::kMicrosecondsToNanoseconds);

  // Test with msg_time diagnostics ENABLED - should detect frame drop
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.enable_msg_time_diagnostics = true;
  config.expected_dt_us = interarrival_time_ns /
    ::greenwave_diagnostics::constants::kMicrosecondsToNanoseconds;
  config.jitter_tolerance_us = 1000;

  greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic", config);

  setupDiagnosticSubscription();

  // Lambda to inject large delay at iteration 10
  auto delay_at_10 = [large_delay_ns](int i, int64_t default_interval) {
      return (i == 10) ? large_delay_ns : default_interval;
    };

  auto msg_timestamp = test_constants::kStartTimestampNs;
  sendMessages(diag, msg_timestamp, interarrival_time_ns, 20, true, 0, 0, delay_at_10);

  ASSERT_FALSE(received_diagnostics_.empty());
  EXPECT_TRUE(hasErrorStatusForTopic("test_topic"))
    << "ERROR status should occur with msg_time enabled when frame drop happens";
}

TEST_F(GreenwaveDiagnosticsTest, EnableIncreasingMsgTimeConfigImpactTest)
{
  // Test that enable_increasing_msg_time_diagnostics config controls detection
  constexpr int64_t normal_interval_ns = 10000000ULL;
  constexpr int64_t decrement_ns = -500000000LL;

  // Lambda to decrement timestamp at iteration 10
  auto decrement_at_10 = [decrement_ns](int i, int64_t default_interval) {
      return (i == 10) ? decrement_ns : default_interval;
    };

  // Test with increasing msg time diagnostics DISABLED
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    config.enable_increasing_msg_time_diagnostics = false;

    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic_disabled", config);

    setupDiagnosticSubscription();

    uint64_t msg_timestamp = test_constants::kStartTimestampNs + 1000000000ULL;
    sendMessages(diag, msg_timestamp, normal_interval_ns, 20, true, 0, 0, decrement_at_10);

    ASSERT_FALSE(received_diagnostics_.empty());
    EXPECT_FALSE(hasDiagnosticMessage("NONINCREASING"))
      << "NONINCREASING should NOT be detected with increasing_msg_time disabled";
  }

  recreateNode();

  // Test with increasing msg time diagnostics ENABLED
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    config.enable_increasing_msg_time_diagnostics = true;

    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic_enabled", config);

    setupDiagnosticSubscription();

    uint64_t msg_timestamp = test_constants::kStartTimestampNs + 1000000000ULL;
    sendMessages(diag, msg_timestamp, normal_interval_ns, 20, true, 0, 0, decrement_at_10);

    ASSERT_FALSE(received_diagnostics_.empty());
    EXPECT_TRUE(hasDiagnosticMessage("NONINCREASING"))
      << "NONINCREASING should be detected with increasing_msg_time enabled";
  }
}

TEST_F(GreenwaveDiagnosticsTest, FrequencyParameterAffectsJitterCalculationTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.enable_msg_time_diagnostics = true;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  const std::string freq_param = test_constants::makeParam("test_topic", test_constants::kFreq);
  const std::string tol_param = test_constants::makeParam("test_topic", test_constants::kTol);

  setupDiagnosticSubscription();

  // Set expected frequency to 100 Hz (10ms between messages) with tight tolerance
  setParam(freq_param, 100.0);
  setParam(tol_param, 1.0);  // 1% tolerance

  // Send messages at exactly 100 Hz (should be within tolerance)
  uint64_t msg_timestamp = test_constants::kStartTimestampNs;
  constexpr int64_t interarrival_100hz_ns = 10000000;  // 10ms

  sendMessages(greenwave_diagnostics, msg_timestamp, interarrival_100hz_ns, 50, true, 0, 10);

  // Should not detect frame drops with correct frequency
  EXPECT_FALSE(hasDiagnosticMessage("FRAME DROP"));

  received_diagnostics_.clear();

  // Now change to expect 50 Hz, but keep sending at 100 Hz
  // This should trigger jitter detection since messages arrive too fast
  setParam(freq_param, 50.0);

  sendMessages(greenwave_diagnostics, msg_timestamp, interarrival_100hz_ns, 50, true, 0, 10);

  // Should detect frame drops now due to frequency mismatch
  EXPECT_TRUE(hasDiagnosticMessage("FRAME DROP"));
}

TEST_F(GreenwaveDiagnosticsTest, ToleranceParameterAffectsJitterDetectionTest)
{
  // Test that tolerance parameter controls how sensitive jitter detection is
  const std::string freq_param = test_constants::makeParam("test_topic", test_constants::kFreq);
  const std::string tol_param = test_constants::makeParam("test_topic", test_constants::kTol);

  constexpr int64_t base_interval_ns = 10000000;  // 10ms = 100 Hz
  constexpr int64_t jitter_ns = 500000;  // 0.5ms jitter (5% of interval)

  // Test with tight tolerance (1%) - jitter should exceed tolerance
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic", config);

    setParam(freq_param, 100.0);
    setParam(tol_param, 1.0);  // 1% tolerance = 100us
    rclcpp::spin_some(node_);

    setupDiagnosticSubscription();
    uint64_t msg_timestamp = test_constants::kStartTimestampNs;
    sendMessages(diag, msg_timestamp, base_interval_ns, 30, true, jitter_ns);

    ASSERT_FALSE(received_diagnostics_.empty());
    EXPECT_GT(getDiagnosticValue("num_jitter_outliers_msg"), 0.0)
      << "Expected jitter outliers with 1% tolerance and 5% jitter";
  }

  recreateNode();

  // Test with loose tolerance (50%) - same jitter should not exceed tolerance
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic", config);

    setParam(freq_param, 100.0);
    setParam(tol_param, 50.0);  // 50% tolerance = 5000us
    rclcpp::spin_some(node_);

    setupDiagnosticSubscription();
    uint64_t msg_timestamp = test_constants::kStartTimestampNs;
    sendMessages(diag, msg_timestamp, base_interval_ns, 30, true, jitter_ns);

    ASSERT_FALSE(received_diagnostics_.empty());
    EXPECT_EQ(getDiagnosticValue("num_jitter_outliers_msg"), 0.0)
      << "Expected no jitter outliers with 50% tolerance and 5% jitter";
  }
}

TEST_F(GreenwaveDiagnosticsTest, BooleanParameterAcceptanceTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  // Test enabled boolean parameter accepts valid values
  testBooleanParameterAcceptance(
    test_constants::makeParam("test_topic", test_constants::kEnabled));
}

TEST_F(GreenwaveDiagnosticsTest, FrequencyNaNClearsExpectedDtTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.enable_msg_time_diagnostics = true;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  const std::string freq_param = test_constants::makeParam("test_topic", test_constants::kFreq);
  const std::string tol_param = test_constants::makeParam("test_topic", test_constants::kTol);

  setupDiagnosticSubscription();

  // Set frequency first
  setParam(freq_param, 100.0);
  setParam(tol_param, 1.0);
  spinAndWait();

  // Send messages at wrong frequency to trigger frame drop detection
  uint64_t msg_timestamp = test_constants::kStartTimestampNs;
  constexpr int64_t wrong_interval_ns = 50000000;  // 50ms = 20 Hz instead of 100 Hz

  sendMessages(greenwave_diagnostics, msg_timestamp, wrong_interval_ns, 20, true);

  EXPECT_TRUE(hasDiagnosticMessage("FRAME DROP"));

  // Clear diagnostics and set frequency to NaN
  received_diagnostics_.clear();
  setParam(freq_param, std::numeric_limits<double>::quiet_NaN());
  spinAndWait();

  // Send more messages at wrong frequency
  sendMessages(greenwave_diagnostics, msg_timestamp, wrong_interval_ns, 20, true);

  // Should NOT detect frame drops after clearing with NaN
  EXPECT_FALSE(hasDiagnosticMessage("FRAME DROP"));
}

TEST_F(GreenwaveDiagnosticsTest, ParameterBoundaryValuesTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  const std::string freq_param = test_constants::makeParam("test_topic", test_constants::kFreq);
  const std::string tol_param = test_constants::makeParam("test_topic", test_constants::kTol);

  // Tolerance boundary values
  setParam(tol_param, 0.0);
  setParam(tol_param, 1000.0);
  setParam(tol_param, 0.001);

  // Frequency boundary values
  setParam(freq_param, 0.001);
  setParam(freq_param, 10000.0);
  setParam(freq_param, std::numeric_limits<double>::infinity());
}

TEST_F(GreenwaveDiagnosticsTest, FilterWindowSizeImpactTest)
{
  constexpr int64_t interval_100hz_ns = 10000000;  // 10 ms = 100 Hz
  constexpr int64_t interval_50hz_ns = 20000000;  // 20 ms = 50 Hz

  // Test with small window size - frame rate should stabilize quickly
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    config.filter_window_size = 10;
    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic_small", config);

    uint64_t timestamp = test_constants::kStartTimestampNs;
    sendMessages(diag, timestamp, interval_100hz_ns, 15);

    EXPECT_NEAR(diag.getFrameRateMsg(), 100.0, 1.0);
  }

  // Test with large window size - frame rate calculation includes more history
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    config.filter_window_size = 100;
    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic_large", config);

    uint64_t timestamp = test_constants::kStartTimestampNs;
    sendMessages(diag, timestamp, interval_100hz_ns, 50);  // 100 Hz
    sendMessages(diag, timestamp, interval_50hz_ns, 50);   // 50 Hz

    // With large window, frame rate should be between 50 and 100 Hz
    double frame_rate = diag.getFrameRateMsg();
    EXPECT_GT(frame_rate, 50.0);
    EXPECT_LT(frame_rate, 100.0);
  }
}

TEST_F(GreenwaveDiagnosticsTest, FilterWindowSizeSmallVsLargeStabilityTest)
{
  constexpr int64_t interarrival_time_ns = 10000000;  // 10ms = 100 Hz
  constexpr int64_t jitter_ns = 2000000;  // 2ms jitter

  // Small window: more sensitive to recent jitter
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    config.filter_window_size = 5;
    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic_jitter_small", config);

    uint64_t timestamp = test_constants::kStartTimestampNs;
    sendMessages(diag, timestamp, interarrival_time_ns, 20);
    double stable_rate = diag.getFrameRateMsg();

    // Send jittery messages
    sendMessages(diag, timestamp, interarrival_time_ns, 10, false, jitter_ns);
    double jittery_rate = diag.getFrameRateMsg();

    // Small window should show more variation
    EXPECT_GT(std::abs(stable_rate - jittery_rate), 0.0);
  }

  // Large window: more stable despite recent jitter
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    config.filter_window_size = 100;
    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic_jitter_large", config);

    uint64_t timestamp = test_constants::kStartTimestampNs;
    sendMessages(diag, timestamp, interarrival_time_ns, 50);
    double stable_rate = diag.getFrameRateMsg();

    // Send jittery messages
    sendMessages(diag, timestamp, interarrival_time_ns, 10, false, jitter_ns);
    double jittery_rate = diag.getFrameRateMsg();

    // Large window should show less variation (more stable)
    EXPECT_LT(std::abs(stable_rate - jittery_rate), 5.0);
  }
}

TEST_F(GreenwaveDiagnosticsTest, ToleranceZeroDetectsAllJitterTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.enable_msg_time_diagnostics = true;
  greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic", config);

  const std::string freq_param = test_constants::makeParam("test_topic", test_constants::kFreq);
  const std::string tol_param = test_constants::makeParam("test_topic", test_constants::kTol);

  setParam(freq_param, 100.0);
  setParam(tol_param, 0.0);
  spinAndWait();

  setupDiagnosticSubscription();

  uint64_t msg_timestamp = test_constants::kStartTimestampNs;
  constexpr int64_t interarrival_100hz_ns = 10000000;  // 10ms
  constexpr int64_t tiny_jitter_ns = 1000;  // 1 microsecond
  sendMessages(diag, msg_timestamp, interarrival_100hz_ns, 30, true, tiny_jitter_ns);

  ASSERT_FALSE(received_diagnostics_.empty());
  EXPECT_GT(getDiagnosticValue("num_jitter_outliers_msg"), 0.0)
    << "Zero tolerance should detect even tiny jitter";
}

TEST_F(GreenwaveDiagnosticsTest, FrequencyIntegerParameterTest)
{
  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  const std::string freq_param = test_constants::makeParam("test_topic", test_constants::kFreq);
  const std::string tol_param = test_constants::makeParam("test_topic", test_constants::kTol);

  // Test integer frequency is accepted and converted to double
  setParam(freq_param, 100);

  // Test integer tolerance is accepted and converted to double
  setParam(tol_param, 5);
  spinAndWait();

  setupDiagnosticSubscription();

  uint64_t msg_timestamp = test_constants::kStartTimestampNs;
  constexpr int64_t interarrival_100hz_ns = 10000000;

  sendMessages(greenwave_diagnostics, msg_timestamp, interarrival_100hz_ns, 20, true);

  // Should not have errors with matching frequency
  EXPECT_FALSE(hasErrorStatusForTopic("test_topic"));
}

TEST_F(GreenwaveDiagnosticsTest, ExpectedDtUsConfigImpactTest)
{
  // Test that expected_dt_us in config affects jitter calculations
  constexpr int64_t expected_dt_us = 10000;  // 10ms = 100 Hz
  constexpr int64_t jitter_tolerance_us = 500;  // 0.5ms

  greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
  config.enable_msg_time_diagnostics = true;
  config.expected_dt_us = expected_dt_us;
  config.jitter_tolerance_us = jitter_tolerance_us;
  greenwave_diagnostics::GreenwaveDiagnostics greenwave_diagnostics(
    *node_, "test_topic", config);

  setupDiagnosticSubscription();

  // Send messages at wrong frequency (50 Hz instead of 100 Hz)
  uint64_t msg_timestamp = test_constants::kStartTimestampNs;
  constexpr int64_t wrong_interval_ns = 20000000;  // 20ms = 50 Hz

  sendMessages(greenwave_diagnostics, msg_timestamp, wrong_interval_ns, 30, true);

  // Should detect frame drops due to config expected_dt_us mismatch
  EXPECT_TRUE(hasDiagnosticMessage("FRAME DROP"));
}

TEST_F(GreenwaveDiagnosticsTest, JitterToleranceUsConfigImpactTest)
{
  // Test that jitter_tolerance_us in config affects outlier detection
  constexpr int64_t expected_dt_us = 10000;  // 10ms = 100 Hz
  constexpr int64_t base_interval_ns = 10000000;
  constexpr int64_t jitter_ns = 500000;  // 500us

  // Test with tight tolerance - should detect outliers
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    config.enable_msg_time_diagnostics = true;
    config.expected_dt_us = expected_dt_us;
    config.jitter_tolerance_us = 100;  // 100us tolerance
    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic_tight", config);

    setupDiagnosticSubscription();
    uint64_t msg_timestamp = test_constants::kStartTimestampNs;
    sendMessages(diag, msg_timestamp, base_interval_ns, 30, true, jitter_ns);

    ASSERT_FALSE(received_diagnostics_.empty());
    EXPECT_GT(getDiagnosticValue("num_jitter_outliers_msg"), 0.0)
      << "Tight tolerance should detect 500us jitter as outliers";
  }

  recreateNode();

  // Test with loose tolerance - should not detect outliers
  {
    greenwave_diagnostics::GreenwaveDiagnosticsConfig config;
    config.enable_msg_time_diagnostics = true;
    config.expected_dt_us = expected_dt_us;
    config.jitter_tolerance_us = 1000000;  // 1000ms tolerance (very loose)
    greenwave_diagnostics::GreenwaveDiagnostics diag(*node_, "test_topic_loose", config);

    setupDiagnosticSubscription();
    uint64_t msg_timestamp = test_constants::kStartTimestampNs;
    sendMessages(diag, msg_timestamp, base_interval_ns, 30, true, jitter_ns);

    ASSERT_FALSE(received_diagnostics_.empty());
    EXPECT_EQ(getDiagnosticValue("num_jitter_outliers_msg"), 0.0)
      << "Loose tolerance should not detect 500us jitter as outliers";
  }
}
