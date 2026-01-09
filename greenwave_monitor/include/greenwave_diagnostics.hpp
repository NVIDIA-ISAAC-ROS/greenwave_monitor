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

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rcpputils/join.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace greenwave_diagnostics
{
namespace constants
{
inline constexpr uint64_t kSecondsToNanoseconds = 1000000000ULL;
inline constexpr uint64_t kSecondsToMicroseconds = 1000000ULL;
inline constexpr uint64_t kMicrosecondsToNanoseconds = 1000ULL;
inline constexpr uint64_t kMillisecondsToMicroseconds = 1000ULL;
// Convenience constant for converting milliseconds to seconds in floating-point math
inline constexpr uint64_t kMillisecondsToSeconds = 1000ULL;
inline constexpr int64_t kDropWarnTimeoutSeconds = 5LL;
// Cutoff where we consider latency to be nonsense
inline constexpr int64_t kNonsenseLatencyMs = 365LL * 24LL * 60LL * 60LL * 1000LL;
// Parameter constants
inline constexpr const char * kTopicParamPrefix = "greenwave_diagnostics.";
inline constexpr const char * kFreqSuffix = ".expected_frequency";
inline constexpr const char * kTolSuffix = ".tolerance";
inline constexpr const char * kEnabledSuffix = ".enabled";
inline constexpr double kDefaultTolerancePercent = 5.0;
inline constexpr double kDefaultFrequencyHz = std::numeric_limits<double>::quiet_NaN();
inline constexpr bool kDefaultEnabled = true;
}  // namespace constants

// Configurations for a message diagnostics
struct GreenwaveDiagnosticsConfig
{
  // diagnostics toggle
  bool enable_diagnostics{false};

  // corresponds to launch arguments
  bool enable_all_diagnostics{false};
  bool enable_node_time_diagnostics{false};
  bool enable_msg_time_diagnostics{false};
  bool enable_increasing_msg_time_diagnostics{false};

  // enable basic diagnostics for all topics, triggered by an environment variable
  bool enable_all_topic_diagnostics{false};

  // Window size of the mean filter in terms of number of messages received
  int filter_window_size{300};

  // Expected time difference between messages in microseconds for this topic
  int64_t expected_dt_us{0LL};

  // Tolerance for jitter from expected frame rate in microseconds
  int64_t jitter_tolerance_us{0LL};
};

class GreenwaveDiagnostics
{
public:
  GreenwaveDiagnostics(
    rclcpp::Node & node,
    const std::string & topic_name,
    const GreenwaveDiagnosticsConfig & diagnostics_config)
  : node_(node), topic_name_(topic_name), diagnostics_config_(diagnostics_config)
  {
    clock_ = node_.get_clock();
    node_window_.window_size = diagnostics_config_.filter_window_size;
    msg_window_.window_size = diagnostics_config_.filter_window_size;
    diagnostic_msgs::msg::DiagnosticStatus topic_status;
    topic_status.name = topic_name;
    topic_status.hardware_id = "nvidia";
    topic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    topic_status.message = "UNDEFINED STATE";
    status_vec_.push_back(topic_status);
    deadlines_missed_since_last_pub_ = 0;

    t_start_ = clock_->now();

    prev_drop_ts_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    prev_noninc_msg_ts_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    prev_timestamp_node_us_ = std::numeric_limits<uint64_t>::min();
    prev_timestamp_msg_us_ = std::numeric_limits<uint64_t>::min();
    num_non_increasing_msg_ = 0;
    message_latency_msg_ms_ = 0;
    outdated_msg_ = true;

    diagnostic_publisher_ =
      node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    // Build parameter names for this topic
    freq_param_name_ = std::string(constants::kTopicParamPrefix) + topic_name_ +
      constants::kFreqSuffix;
    tol_param_name_ = std::string(constants::kTopicParamPrefix) + topic_name_ +
      constants::kTolSuffix;
    enabled_param_name_ = std::string(constants::kTopicParamPrefix) + topic_name_ +
      constants::kEnabledSuffix;

    // Register parameter callback for this topic's parameters
    param_callback_handle_ = node_.add_on_set_parameters_callback(
      std::bind(&GreenwaveDiagnostics::onParameterChange, this, std::placeholders::_1));

    // Subscribe to parameter events
    param_event_subscription_ = node_.create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      std::bind(&GreenwaveDiagnostics::onParameterEvent, this, std::placeholders::_1));

    // Convert config's expected_dt_us to frequency if set
    double default_freq = constants::kDefaultFrequencyHz;
    if (diagnostics_config_.expected_dt_us > 0) {
      default_freq = static_cast<double>(constants::kSecondsToMicroseconds) /
        static_cast<double>(diagnostics_config_.expected_dt_us);
    }

    // Declare frequency, tolerance, and enabled parameters if they won't automatically be declared
    // from overrides. Declared parameters cannot be deleted. Use passed DiagnosticsConfig values.
    bool auto_declare = node_.get_node_options().automatically_declare_parameters_from_overrides();
    if (!auto_declare) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing = true;
      node_.declare_parameter(enabled_param_name_, constants::kDefaultEnabled);
      node_.declare_parameter(tol_param_name_, constants::kDefaultTolerancePercent, descriptor);
      node_.declare_parameter(freq_param_name_, default_freq, descriptor);
    } else {
      // Parameters declared via launch/YAML/constructor are ignored by onParameterChange()
      // Re-set parameters to their current value to trigger callbacks. If
      // the parameter fails to set, use defaults.
      setParameterOrDefault(enabled_param_name_, constants::kDefaultEnabled);
      setParameterOrDefault(freq_param_name_, default_freq);
      setParameterOrDefault(tol_param_name_, constants::kDefaultTolerancePercent);
    }
  }

  ~GreenwaveDiagnostics()
  {
    if (param_callback_handle_) {
      node_.remove_on_set_parameters_callback(param_callback_handle_.get());
      param_callback_handle_.reset();
    }
    param_event_subscription_.reset();
    diagnostic_publisher_.reset();
  }

  // Update diagnostics numbers. To be called in Subscriber and Publisher
  void updateDiagnostics(uint64_t msg_timestamp_ns)
  {
    // If the topic is not enabled, skip updating diagnostics
    if (!enabled_) {
      RCLCPP_DEBUG_THROTTLE(
        node_.get_logger(), *clock_, 1000,
        "Topic %s is not enabled, skipping update diagnostics", topic_name_.c_str());
      return;
    }
    // Mutex lock to prevent simultaneous access of common parameters
    // used by updateDiagnostics() and publishDiagnostics()
    const std::lock_guard<std::mutex> lock(greenwave_diagnostics_mutex_);
    // Message diagnostics checks message intervals both using the node clock
    // and the message timestamp.
    // All variables name _node refers to the node timestamp checks.
    // All variables name _msg refers to the message timestamp checks.
    status_vec_[0].message = "";
    bool error_found = false;

    // Get the current timestamps in microseconds
    uint64_t current_timestamp_msg_us =
      msg_timestamp_ns / greenwave_diagnostics::constants::kMicrosecondsToNanoseconds;
    uint64_t current_timestamp_node_us = static_cast<uint64_t>(clock_->now().nanoseconds() /
      greenwave_diagnostics::constants::kMicrosecondsToNanoseconds);

    // we can only calculate frame rate after 2 messages have been received
    if (prev_timestamp_node_us_ != std::numeric_limits<uint64_t>::min()) {
      const int64_t timestamp_diff_node_us = current_timestamp_node_us - prev_timestamp_node_us_;
      node_window_.addInterarrival(timestamp_diff_node_us);
      if (diagnostics_config_.enable_node_time_diagnostics) {
        error_found |= updateNodeTimeDiagnostics(timestamp_diff_node_us);
      }
    }

    const rclcpp::Time time_from_node = node_.get_clock()->now();
    uint64_t ros_node_system_time_us = time_from_node.nanoseconds() /
      greenwave_diagnostics::constants::kMicrosecondsToNanoseconds;

    const double latency_wrt_current_timestamp_node_ms =
      static_cast<double>(ros_node_system_time_us - current_timestamp_msg_us) /
      greenwave_diagnostics::constants::kMillisecondsToMicroseconds;

    if (prev_timestamp_msg_us_ != std::numeric_limits<uint64_t>::min()) {
      const int64_t timestamp_diff_msg_us = current_timestamp_msg_us - prev_timestamp_msg_us_;
      msg_window_.addInterarrival(timestamp_diff_msg_us);
      // Do the same checks as above, but for message timestamp
      if (diagnostics_config_.enable_msg_time_diagnostics) {
        error_found |= updateMsgTimeDiagnostics(timestamp_diff_msg_us);
      }
      if (diagnostics_config_.enable_increasing_msg_time_diagnostics) {
        error_found |= updateIncreasingMsgTimeDiagnostics(current_timestamp_msg_us);
      }
    }

    prev_timestamp_node_us_ = current_timestamp_node_us;
    prev_timestamp_msg_us_ = current_timestamp_msg_us;

    // calculate key values for diagnostics status (computed on publish/getters)
    message_latency_msg_ms_ = latency_wrt_current_timestamp_node_ms;
    if (message_latency_msg_ms_ > greenwave_diagnostics::constants::kNonsenseLatencyMs) {
      message_latency_msg_ms_ = std::numeric_limits<double>::quiet_NaN();
    }

    // frame dropping warnings
    if (!error_found) {
      status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status_vec_[0].message = "OK";
    }
    outdated_msg_ = false;
  }

  // Function to publish diagnostics to a ROS topic
  void publishDiagnostics()
  {
    // If the topic is not enabled, skip publishing diagnostics
    if (!enabled_) {
      RCLCPP_DEBUG_THROTTLE(
        node_.get_logger(), *clock_, 1000,
        "Topic %s is not enabled, skipping publish diagnostics", topic_name_.c_str());
      return;
    }
    // Mutex lock to prevent simultaneous access of common parameters
    // used by updateDiagnostics() and publishDiagnostics()
    const std::lock_guard<std::mutex> lock(greenwave_diagnostics_mutex_);

    std::vector<diagnostic_msgs::msg::KeyValue> values;
    // publish diagnostics stale if message has not been updated since the last call
    if (outdated_msg_) {
      status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status_vec_[0].message = "DIAGNOSTICS STALE";
    } else {
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("num_non_increasing_msg")
        .value(std::to_string(num_non_increasing_msg_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("num_jitter_outliers_msg")
        .value(std::to_string(msg_window_.outlier_count)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("num_jitter_outliers_node")
        .value(std::to_string(node_window_.outlier_count)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("max_abs_jitter_msg")
        .value(std::to_string(msg_window_.max_abs_jitter_us)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("max_abs_jitter_node")
        .value(std::to_string(node_window_.max_abs_jitter_us)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("mean_abs_jitter_msg")
        .value(std::to_string(msg_window_.meanAbsJitterUs())));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("mean_abs_jitter_node")
        .value(std::to_string(node_window_.meanAbsJitterUs())));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("frame_rate_msg")
        .value(std::to_string(msg_window_.frameRateHz())));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("current_delay_from_realtime_ms")
        .value(
          std::isnan(message_latency_msg_ms_) ?
          "N/A" : std::to_string(message_latency_msg_ms_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("frame_rate_node")
        .value(std::to_string(node_window_.frameRateHz())));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("total_dropped_frames")
        .value(std::to_string(msg_window_.outlier_count)));
    }
    status_vec_[0].values = values;

    // timestamp from std::chrono (steady clock); split into sec/nanosec correctly
    const uint64_t elapsed_ns = static_cast<uint64_t>((clock_->now() - t_start_).nanoseconds());
    const uint32_t time_seconds = static_cast<uint32_t>(
      elapsed_ns / static_cast<uint64_t>(greenwave_diagnostics::constants::kSecondsToNanoseconds));
    const uint32_t time_ns = static_cast<uint32_t>(
      elapsed_ns % static_cast<uint64_t>(greenwave_diagnostics::constants::kSecondsToNanoseconds));

    diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
    std_msgs::msg::Header header;
    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = time_seconds;
    timestamp.nanosec = time_ns;
    header.stamp = timestamp;
    diagnostic_msg.header = header;
    diagnostic_msg.status = status_vec_;

    diagnostic_publisher_->publish(diagnostic_msg);
    outdated_msg_ = true;
  }

  double getFrameRateNode() const
  {
    return node_window_.frameRateHz();
  }

  double getFrameRateMsg() const
  {
    return msg_window_.frameRateHz();
  }

  double getLatency() const
  {
    return message_latency_msg_ms_;
  }

  void setExpectedDt(double expected_hz, double tolerance_percent)
  {
    const std::lock_guard<std::mutex> lock(greenwave_diagnostics_mutex_);

    // This prevents accidental 0 division in the calculations in case of
    // a direct function call (not from service in greenwave_monitor.cpp)
    if (expected_hz == 0.0) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "expected_hz is 0.0. It should be set to a value greater than 0."
        " Keeping previous values: expected_dt_us = %" PRId64 ","
        " jitter_tolerance_us = %" PRId64 ".",
        static_cast<int64_t>(diagnostics_config_.expected_dt_us),
        diagnostics_config_.jitter_tolerance_us);
      return;
    }

    const int64_t expected_dt_us =
      static_cast<int64_t>(greenwave_diagnostics::constants::kSecondsToMicroseconds / expected_hz);
    diagnostics_config_.expected_dt_us = expected_dt_us;

    const int tolerance_us =
      static_cast<int>((greenwave_diagnostics::constants::kSecondsToMicroseconds / expected_hz) *
      (tolerance_percent / 100.0));
    diagnostics_config_.jitter_tolerance_us = tolerance_us;

    // Enable node and msg time diagnostics when expected frequency is set
    diagnostics_config_.enable_node_time_diagnostics = true;
    diagnostics_config_.enable_msg_time_diagnostics = true;
  }

  void clearExpectedDt()
  {
    const std::lock_guard<std::mutex> lock(greenwave_diagnostics_mutex_);
    diagnostics_config_.expected_dt_us = 0;
    diagnostics_config_.jitter_tolerance_us = 0;

    // Disable node and msg time diagnostics when expected frequency is cleared
    diagnostics_config_.enable_node_time_diagnostics = false;
    diagnostics_config_.enable_msg_time_diagnostics = false;
  }

private:
  struct RollingWindow
  {
    int window_size{0};
    std::queue<int64_t> interarrival_us;
    int64_t sum_interarrival_us{0};
    std::queue<int64_t> jitter_abs_us;
    int64_t sum_jitter_abs_us{0};
    int64_t max_abs_jitter_us{0};
    uint64_t outlier_count{0};

    void addInterarrival(int64_t delta_us)
    {
      interarrival_us.push(delta_us);
      sum_interarrival_us += delta_us;
      if (static_cast<int>(interarrival_us.size()) > window_size) {
        sum_interarrival_us -= interarrival_us.front();
        interarrival_us.pop();
      }
    }

    // Returns true if abs_jitter_us exceeded tolerance (i.e., counts as a missed deadline)
    bool addJitter(int64_t abs_jitter_us, int64_t jitter_tolerance_us)
    {
      max_abs_jitter_us = std::max(max_abs_jitter_us, abs_jitter_us);
      jitter_abs_us.push(abs_jitter_us);
      sum_jitter_abs_us += abs_jitter_us;
      if (static_cast<int>(jitter_abs_us.size()) > window_size) {
        sum_jitter_abs_us -= jitter_abs_us.front();
        jitter_abs_us.pop();
      }
      if (abs_jitter_us > jitter_tolerance_us) {
        ++outlier_count;
        return true;
      }
      return false;
    }

    double frameRateHz() const
    {
      if (sum_interarrival_us == 0 || interarrival_us.empty()) {
        return 0.0;
      }
      return static_cast<double>(greenwave_diagnostics::constants::kSecondsToMicroseconds) /
             (static_cast<double>(sum_interarrival_us) /
             static_cast<double>(interarrival_us.size()));
    }

    int64_t meanAbsJitterUs() const
    {
      if (jitter_abs_us.empty()) {
        return 0;
      }
      return sum_jitter_abs_us / static_cast<int64_t>(jitter_abs_us.size());
    }

    void clear()
    {
      interarrival_us = std::queue<int64_t>();
      sum_interarrival_us = 0;
      jitter_abs_us = std::queue<int64_t>();
      sum_jitter_abs_us = 0;
      max_abs_jitter_us = 0;
      outlier_count = 0;
    }
  };
  // Mutex lock to prevent simultaneous access of common parameters
  // used by updateDiagnostics() and publishDiagnostics()
  std::mutex greenwave_diagnostics_mutex_;

  rclcpp::Node & node_;
  std::string topic_name_;
  GreenwaveDiagnosticsConfig diagnostics_config_;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time t_start_;
  rclcpp::Time prev_drop_ts_, prev_noninc_msg_ts_;
  uint64_t prev_timestamp_node_us_, prev_timestamp_msg_us_;

  RollingWindow node_window_;
  RollingWindow msg_window_;
  uint64_t num_non_increasing_msg_;
  uint64_t deadlines_missed_since_last_pub_;
  double message_latency_msg_ms_;
  bool outdated_msg_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;

  void update_status_message(diagnostic_msgs::msg::DiagnosticStatus & status, std::string update)
  {
    if (status.message.empty()) {
      status.message = update;
    } else if (status.message.find(update) == std::string::npos) {
      // Only append if not already present
      status.message.append(", ").append(update);
    }
  }

  // Update node (pub/sub) time diagnostics
  bool updateNodeTimeDiagnostics(const int64_t timestamp_diff_node_us)
  {
    bool error_found = false;
    if (diagnostics_config_.expected_dt_us <= 0) {
      return error_found;
    }
    // calculate difference between time between msgs(using node clock)
    // and expected time between msgs
    const int64_t expected_dt_us_node = diagnostics_config_.expected_dt_us;
    const int64_t diff_node_us = timestamp_diff_node_us - expected_dt_us_node;
    const int64_t abs_jitter_node = std::abs(diff_node_us);
    const bool missed_deadline_node =
      node_window_.addJitter(abs_jitter_node, diagnostics_config_.jitter_tolerance_us);
    if (missed_deadline_node) {
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[GreenwaveDiagnostics Node Time]"
        " Difference of time between messages(%" PRId64 ") and expected time between"
        " messages(%" PRId64 ") is out of tolerance(%" PRId64 ") by %" PRId64 " for topic %s."
        " Units are microseconds.",
        static_cast<int64_t>(timestamp_diff_node_us),
        expected_dt_us_node,
        static_cast<int64_t>(diagnostics_config_.jitter_tolerance_us),
        abs_jitter_node, topic_name_.c_str());
    }
    return error_found;
  }

  bool updateMsgTimeDiagnostics(const int64_t timestamp_diff_msg_us)
  {
    bool error_found = false;
    if (diagnostics_config_.expected_dt_us <= 0) {
      return error_found;
    }
    // calculate difference between time between msgs(using msg clock)
    // and expected time between msgs
    const int64_t expected_dt_us_msg = diagnostics_config_.expected_dt_us;
    const int64_t diff_msg_us = timestamp_diff_msg_us - expected_dt_us_msg;
    const int64_t abs_jitter_msg = std::abs(diff_msg_us);
    const bool missed_deadline_msg =
      msg_window_.addJitter(abs_jitter_msg, diagnostics_config_.jitter_tolerance_us);
    if (missed_deadline_msg) {
      deadlines_missed_since_last_pub_++;
      prev_drop_ts_ = clock_->now();
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[GreenwaveDiagnostics Message Timestamp]"
        " Difference of time between messages(%" PRId64 ") and expected "
        "time between"
        " messages(%" PRId64 ") is out of tolerance(%" PRId64 ") by %" PRId64 " for topic %s. "
        "Units are microseconds.",
        static_cast<int64_t>(timestamp_diff_msg_us),
        expected_dt_us_msg,
        static_cast<int64_t>(diagnostics_config_.jitter_tolerance_us),
        abs_jitter_msg, topic_name_.c_str());
    }

    if (prev_drop_ts_.nanoseconds() != 0) {
      auto time_since_drop = (clock_->now() - prev_drop_ts_).seconds();
      if (time_since_drop < greenwave_diagnostics::constants::kDropWarnTimeoutSeconds) {
        error_found = true;
        status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        deadlines_missed_since_last_pub_ = 0;
        update_status_message(status_vec_[0], "FRAME DROP DETECTED");
      }
    }
    return error_found;
  }

  bool updateIncreasingMsgTimeDiagnostics(const uint64_t current_timestamp_msg_us)
  {
    bool error_found = false;
    // Check if message timestamp is increasing
    if (current_timestamp_msg_us <= prev_timestamp_msg_us_) {
      // Increment non increasing message count
      num_non_increasing_msg_++;
      prev_noninc_msg_ts_ = clock_->now();
      RCLCPP_WARN(
        node_.get_logger(),
        "[GreenwaveDiagnostics Message Timestamp Non Increasing]"
        " Message timestamp is not increasing. Current timestamp: %" PRIu64 ", "
        " Previous timestamp: %" PRIu64 " for topic %s. Units are microseconds.",
        current_timestamp_msg_us, prev_timestamp_msg_us_, topic_name_.c_str());
    }
    if (prev_noninc_msg_ts_.nanoseconds() != 0) {
      auto time_since_noninc = (clock_->now() - prev_noninc_msg_ts_).seconds();
      if (time_since_noninc < greenwave_diagnostics::constants::kDropWarnTimeoutSeconds) {
        error_found = true;
        status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        deadlines_missed_since_last_pub_ = 0;
        update_status_message(status_vec_[0], "NONINCREASING TIMESTAMP");
      }
    }
    return error_found;
  }

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::optional<double> new_freq;
    std::optional<double> new_tol;
    std::optional<bool> new_enabled;
    std::vector<std::string> error_reasons;

    //////////////////////////////////////////////////////////////////////////////
    // Validate parameters
    //////////////////////////////////////////////////////////////////////////////
    for (const auto & param : parameters) {
      // Only handle parameters for this topic
      if (param.get_name() != freq_param_name_ && param.get_name() != tol_param_name_ &&
        param.get_name() != enabled_param_name_)
      {
        continue;
      }

      // Reject parameter deletion attempts
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        result.successful = false;
        error_reasons.push_back(param.get_name() + ": parameter deletion not supported");
        continue;
      }

      // Handle numeric types together
      if (param.get_name() == freq_param_name_ || param.get_name() == tol_param_name_) {
        auto value_opt = paramToDouble(param);
        if (!value_opt.has_value()) {
          result.successful = false;
          error_reasons.push_back(
            param.get_name() +
            ": must be a numeric type (int or double or NaN)");
          continue;
        }

        double value = value_opt.value();
        if (param.get_name() == freq_param_name_) {
          if (value <= 0.0 && !std::isnan(value)) {
            result.successful = false;
            error_reasons.push_back(param.get_name() + ": must be positive or NaN");
          }
          new_freq = value;
        } else if (param.get_name() == tol_param_name_) {
          if (value < 0.0) {
            result.successful = false;
            error_reasons.push_back(param.get_name() + ": must be non-negative");
          }
          new_tol = value;
        }
      } else if (param.get_name() == enabled_param_name_) {
        new_enabled = param.as_bool();
      }
    }

    // Exit early if any parameters are invalid. No half changes.
    if (!error_reasons.empty()) {
      result.successful = false;
      result.reason = rcpputils::join(error_reasons, "; ");
    }

    // Execution of changes happens in onParameterEvent after parameters are committed
    return result;
  }

  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
  {
    // Only process events from this node
    if (msg->node != node_.get_fully_qualified_name()) {
      return;
    }

    // Process changed and new parameters
    bool freq_changed = false;
    bool tol_changed = false;

    auto process_params = [&](const auto & params) {
        for (const auto & param : params) {
          applyParameterChange(param);
          if (param.name == freq_param_name_) {
            freq_changed = true;
          } else if (param.name == tol_param_name_) {
            tol_changed = true;
          }
        }
      };
    process_params(msg->changed_parameters);
    process_params(msg->new_parameters);

    // Update expected dt only if frequency or tolerance was explicitly changed
    if (freq_changed || tol_changed) {
      auto freq_opt = getNumericParameter(freq_param_name_);
      auto tol_opt = getNumericParameter(tol_param_name_);
      double freq = freq_opt.value_or(std::numeric_limits<double>::quiet_NaN());
      double tol = tol_opt.value_or(constants::kDefaultTolerancePercent);

      if (std::isnan(freq) || freq <= 0.0) {
        clearExpectedDt();
      } else {
        setExpectedDt(freq, tol);
      }
    }
  }

  void applyParameterChange(const rcl_interfaces::msg::Parameter & param)
  {
    if (param.name == enabled_param_name_) {
      bool new_enabled = param.value.bool_value;
      if (!new_enabled) {
        // Clear windows when disabling diagnostics
        const std::lock_guard<std::mutex> lock(greenwave_diagnostics_mutex_);
        node_window_.clear();
        msg_window_.clear();
        prev_timestamp_node_us_ = std::numeric_limits<uint64_t>::min();
        prev_timestamp_msg_us_ = std::numeric_limits<uint64_t>::min();
      }
      enabled_ = new_enabled;
    }
  }

  std::optional<double> getNumericParameter(const std::string & param_name)
  {
    if (!node_.has_parameter(param_name)) {
      return std::nullopt;
    }
    return paramToDouble(node_.get_parameter(param_name));
  }

  static std::optional<double> paramToDouble(const rclcpp::Parameter & param)
  {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return param.as_double();
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(param.as_int());
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      std::string str = param.as_string();
      if (str == "nan" || str == "NaN" || str == "NAN") {
        return std::numeric_limits<double>::quiet_NaN();
      }
    }
    return std::nullopt;
  }

  template<typename T>
  void setParameterOrDefault(const std::string & param_name, const T & default_value)
  {
    if (node_.has_parameter(param_name)) {
      auto current_param = node_.get_parameter(param_name);
      if (current_param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
        auto result = node_.set_parameter(current_param);
        if (result.successful) {
          return;
        } else {
          RCLCPP_WARN(
            node_.get_logger(),
            "Initial parameter %s failed to set for topic %s: %s",
            param_name.c_str(), topic_name_.c_str(), result.reason.c_str());
        }
      }
    }
    auto result = node_.set_parameter(rclcpp::Parameter(param_name, default_value));
    if (!result.successful) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "Failed to set default value for parameter %s for topic %s",
        param_name.c_str(), topic_name_.c_str());
    }
  }

  // Parameter names for this topic
  std::string freq_param_name_;
  std::string tol_param_name_;
  std::string enabled_param_name_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_subscription_;

  // Flag for indicating if message diagnostics are enabled for this topic
  bool enabled_{true};
};

}  // namespace greenwave_diagnostics
