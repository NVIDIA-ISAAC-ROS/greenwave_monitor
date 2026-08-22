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

#include <algorithm>
#include <limits>
#include <cinttypes>
#include <cmath>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "std_msgs/msg/header.hpp"
#include "builtin_interfaces/msg/time.hpp"
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
}  // namespace constants

enum class TimeCheckPreset
{
  HeaderWithFallback,
  HeaderOnly,
  NodetimeOnly,
  None
};

// Configurations for a greenwave diagnostics
struct GreenwaveDiagnosticsConfig
{
  // preset configuration for the diagnostic flags
  TimeCheckPreset time_check_preset{TimeCheckPreset::None};

  // flags for enabling/disabling specific diagnostics
  bool enable_node_time_diagnostics{false};
  bool enable_hdr_time_diagnostics{false};
  bool enable_fps_jitter_hdr_time_diagnostics{false};
  bool enable_fps_jitter_node_time_diagnostics{false};
  bool enable_fps_window_hdr_time_diagnostics{false};
  bool enable_fps_window_node_time_diagnostics{false};
  bool enable_increasing_hdr_time_diagnostics{false};
  bool enable_increasing_node_time_diagnostics{false};

  // fallback to node time when header time is not available
  bool fallback_to_nodetime{false};

  // flag indicating if it's expected for the message to have a header timestamp
  bool has_hdr_timestamp{false};

  // enable basic diagnostics for all topics, triggered by an environment variable
  bool enable_all_topic_diagnostics{false};

  // Window size of the mean filter in terms of number of messages received
  int filter_window_size{300};

  // Expected time difference between messages in microseconds for this topic
  int64_t expected_dt_us{0LL};

  // Tolerance for jitter from expected frame rate in microseconds
  int64_t jitter_tolerance_us{0LL};

  void applyTimeCheckPreset()
  {
    switch (time_check_preset) {
      case TimeCheckPreset::None:
        break;
      case TimeCheckPreset::HeaderOnly:
        enable_node_time_diagnostics = false;
        enable_fps_jitter_node_time_diagnostics = false;
        enable_fps_window_node_time_diagnostics = false;
        enable_increasing_node_time_diagnostics = false;
        enable_hdr_time_diagnostics = true;
        enable_fps_jitter_hdr_time_diagnostics = true;
        enable_fps_window_hdr_time_diagnostics = false;
        enable_increasing_hdr_time_diagnostics = true;
        break;
      case TimeCheckPreset::NodetimeOnly:
        enable_node_time_diagnostics = true;
        enable_fps_jitter_node_time_diagnostics = false;
        enable_fps_window_node_time_diagnostics = true;
        enable_increasing_node_time_diagnostics = true;
        enable_hdr_time_diagnostics = false;
        enable_fps_jitter_hdr_time_diagnostics = false;
        enable_fps_window_hdr_time_diagnostics = false;
        enable_increasing_hdr_time_diagnostics = false;
        break;
      case TimeCheckPreset::HeaderWithFallback:
        // Enable hdr time diagnostics only when it is known there is a hdr timestamp, otherwise
        // disable them and use node time only. fps jitter checks for node time are too restrictive,
        // disable them.
        fallback_to_nodetime = true;
        enable_node_time_diagnostics = true;
        enable_fps_jitter_node_time_diagnostics = false;
        enable_fps_window_node_time_diagnostics = !has_hdr_timestamp;
        enable_increasing_node_time_diagnostics = true;
        enable_hdr_time_diagnostics = has_hdr_timestamp;
        enable_fps_jitter_hdr_time_diagnostics = has_hdr_timestamp;
        enable_fps_window_hdr_time_diagnostics = false;
        enable_increasing_hdr_time_diagnostics = has_hdr_timestamp;
        break;
    }
  }
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
    diagnostics_config_.applyTimeCheckPreset();

    node_ts_series_.label = "Node Time";
    node_ts_series_.enabled = diagnostics_config_.enable_node_time_diagnostics;
    node_ts_series_.check_fps_jitter = diagnostics_config_.enable_fps_jitter_node_time_diagnostics;
    node_ts_series_.check_fps_window = diagnostics_config_.enable_fps_window_node_time_diagnostics;
    node_ts_series_.check_increasing = diagnostics_config_.enable_increasing_node_time_diagnostics;
    node_ts_series_.window.window_size = diagnostics_config_.filter_window_size;
    node_ts_series_.prev_drop_ts = rclcpp::Time(0, 0, clock_->get_clock_type());
    node_ts_series_.prev_noninc_ts = rclcpp::Time(0, 0, clock_->get_clock_type());
    node_ts_series_.prev_fps_out_of_range_ts = rclcpp::Time(0, 0, clock_->get_clock_type());
    node_ts_series_.drop_error_message = "FRAME DROP DETECTED (NODE TIME)";
    node_ts_series_.increasing_error_message = "NONINCREASING TIMESTAMP (NODE TIME)";
    node_ts_series_.fps_window_error_message = "FPS OUT OF RANGE (NODE TIME)";

    hdr_ts_series_.label = "Header Time";
    hdr_ts_series_.enabled = diagnostics_config_.enable_hdr_time_diagnostics;
    hdr_ts_series_.check_fps_jitter = diagnostics_config_.enable_fps_jitter_hdr_time_diagnostics;
    hdr_ts_series_.check_fps_window = diagnostics_config_.enable_fps_window_hdr_time_diagnostics;
    hdr_ts_series_.check_increasing = diagnostics_config_.enable_increasing_hdr_time_diagnostics;
    hdr_ts_series_.window.window_size = diagnostics_config_.filter_window_size;
    hdr_ts_series_.prev_drop_ts = rclcpp::Time(0, 0, clock_->get_clock_type());
    hdr_ts_series_.prev_noninc_ts = rclcpp::Time(0, 0, clock_->get_clock_type());
    hdr_ts_series_.prev_fps_out_of_range_ts = rclcpp::Time(0, 0, clock_->get_clock_type());
    hdr_ts_series_.drop_error_message = "FRAME DROP DETECTED";
    hdr_ts_series_.increasing_error_message = "NONINCREASING TIMESTAMP";
    hdr_ts_series_.fps_window_error_message = "FPS OUT OF RANGE";

    diagnostic_msgs::msg::DiagnosticStatus topic_status;
    topic_status.name = topic_name;
    topic_status.hardware_id = "nvidia";
    topic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    topic_status.message = "UNDEFINED STATE";
    status_vec_.push_back(topic_status);

    t_start_ = clock_->now();

    diagnostic_publisher_ =
      node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
  }

  // Update diagnostics numbers. To be called in Subscriber and Publisher
  void updateDiagnostics(uint64_t hdr_timestamp_ns)
  {
    const std::lock_guard<std::mutex> lock(greenwave_diagnostics_mutex_);
    status_vec_[0].message = "";
    bool error_found = false;

    const uint64_t current_timestamp_hdr_us = diagnostics_config_.has_hdr_timestamp ?
      hdr_timestamp_ns / constants::kMicrosecondsToNanoseconds : 0;
    const uint64_t current_timestamp_node_us = static_cast<uint64_t>(
      clock_->now().nanoseconds() / constants::kMicrosecondsToNanoseconds);

    // Node time source
    error_found |= updateTimeSource(node_ts_series_, current_timestamp_node_us);

    // Header time source (header-bearing messages only)
    if (diagnostics_config_.has_hdr_timestamp) {
      error_found |= updateTimeSource(hdr_ts_series_, current_timestamp_hdr_us);
    }

    // Latency (only valid if header time source is available)
    if (diagnostics_config_.has_hdr_timestamp) {
      const uint64_t ros_node_system_time_us = static_cast<uint64_t>(
        clock_->now().nanoseconds() / constants::kMicrosecondsToNanoseconds);
      message_latency_hdr_ms_ =
        static_cast<double>(ros_node_system_time_us - current_timestamp_hdr_us) /
        constants::kMillisecondsToMicroseconds;
      if (message_latency_hdr_ms_ > constants::kNonsenseLatencyMs) {
        message_latency_hdr_ms_ = std::numeric_limits<double>::quiet_NaN();
      }
    }

    if (!error_found) {
      status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status_vec_[0].message = "OK";
    }
    outdated_msg_ = false;
  }

  // Function to publish diagnostics to a ROS topic
  void publishDiagnostics()
  {
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
        .key("num_non_increasing_hdr")
        .value(std::to_string(hdr_ts_series_.num_non_increasing)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("num_jitter_outliers_hdr")
        .value(std::to_string(hdr_ts_series_.window.outlier_count)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("num_jitter_outliers_node")
        .value(std::to_string(node_ts_series_.window.outlier_count)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("max_abs_jitter_hdr")
        .value(std::to_string(hdr_ts_series_.window.max_abs_jitter_us)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("max_abs_jitter_node")
        .value(std::to_string(node_ts_series_.window.max_abs_jitter_us)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("mean_abs_jitter_hdr")
        .value(std::to_string(hdr_ts_series_.window.meanAbsJitterUs())));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("mean_abs_jitter_node")
        .value(std::to_string(node_ts_series_.window.meanAbsJitterUs())));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("frame_rate_hdr")
        .value(std::to_string(hdr_ts_series_.window.frameRateHz())));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("current_delay_from_realtime_ms")
        .value(
          std::isnan(message_latency_hdr_ms_) ?
          "N/A" : std::to_string(message_latency_hdr_ms_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("frame_rate_node")
        .value(std::to_string(node_ts_series_.window.frameRateHz())));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("total_dropped_frames")
        .value(std::to_string(hdr_ts_series_.window.outlier_count)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("expected_frequency")
        .value(std::to_string(expected_frequency_)));
      values.push_back(
        diagnostic_msgs::build<diagnostic_msgs::msg::KeyValue>()
        .key("tolerance")
        .value(std::to_string(tolerance_)));
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
    return node_ts_series_.window.frameRateHz();
  }

  double getFrameRateHdr() const
  {
    return hdr_ts_series_.window.frameRateHz();
  }

  double getLatency() const
  {
    return message_latency_hdr_ms_;
  }

  void setExpectedDt(double expected_hz, double tolerance_percent)
  {
    const std::lock_guard<std::mutex> lock(greenwave_diagnostics_mutex_);
    node_ts_series_.enabled = true;
    hdr_ts_series_.enabled = true;
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
    expected_frequency_ = expected_hz;
    diagnostics_config_.expected_dt_us = expected_dt_us;

    const int tolerance_us =
      static_cast<int>((greenwave_diagnostics::constants::kSecondsToMicroseconds / expected_hz) *
      (tolerance_percent / 100.0));
    tolerance_ = tolerance_percent;
    diagnostics_config_.jitter_tolerance_us = tolerance_us;
  }

  void clearExpectedDt()
  {
    const std::lock_guard<std::mutex> lock(greenwave_diagnostics_mutex_);
    node_ts_series_.enabled = false;
    hdr_ts_series_.enabled = false;

    diagnostics_config_.expected_dt_us = 0;
    diagnostics_config_.jitter_tolerance_us = 0;

    expected_frequency_ = 0.0;
    tolerance_ = 0.0;
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
  };

  struct TimeSeriesState
  {
    // Per-source identity and behavior (set at construction)
    std::string label;
    bool enabled{false};
    bool check_fps_jitter{false};
    bool check_fps_window{false};
    bool check_increasing{false};
    std::string drop_error_message{"FRAME DROP DETECTED"};
    std::string increasing_error_message{"NONINCREASING TIMESTAMP"};
    std::string fps_window_error_message{"FPS OUT OF RANGE"};

    // Rolling statistics
    RollingWindow window;

    // Tracking state
    uint64_t prev_timestamp_us{std::numeric_limits<uint64_t>::min()};
    rclcpp::Time prev_drop_ts;           // last deadline miss
    rclcpp::Time prev_noninc_ts;         // last increasing timestamp violation
    rclcpp::Time prev_fps_out_of_range_ts;  // last FPS window (min/max) violation
    uint64_t num_non_increasing{0};
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

  TimeSeriesState node_ts_series_;
  TimeSeriesState hdr_ts_series_;

  double message_latency_hdr_ms_{std::numeric_limits<double>::quiet_NaN()};
  bool outdated_msg_{true};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;

  void update_status_message(diagnostic_msgs::msg::DiagnosticStatus & status, std::string update)
  {
    if (status.message.empty()) {
      status.message = update;
    } else {
      status.message.append(", ").append(update);
    }
  }

  bool checkFpsJitter(TimeSeriesState & source, int64_t timestamp_diff_us)
  {
    if (diagnostics_config_.expected_dt_us <= 0) {
      return false;
    }
    const int64_t diff_us = timestamp_diff_us - diagnostics_config_.expected_dt_us;
    const int64_t abs_jitter_us = std::abs(diff_us);
    const bool missed_deadline =
      source.window.addJitter(abs_jitter_us, diagnostics_config_.jitter_tolerance_us);
    if (!source.check_fps_jitter) {
      return false;
    }
    if (missed_deadline) {
      source.prev_drop_ts = clock_->now();
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[GreenwaveDiagnostics %s] dt(%" PRId64 ") expected(%" PRId64
        ") tolerance(%" PRId64 ") excess(%" PRId64 ") topic %s. Units: microseconds.",
        source.label.c_str(), timestamp_diff_us, diagnostics_config_.expected_dt_us,
        diagnostics_config_.jitter_tolerance_us, abs_jitter_us, topic_name_.c_str());
    }
    if (source.prev_drop_ts.nanoseconds() != 0 &&
      (clock_->now() - source.prev_drop_ts).seconds() < constants::kDropWarnTimeoutSeconds)
    {
      status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      update_status_message(status_vec_[0], source.drop_error_message);
      return true;
    }
    return false;
  }

  bool checkIncreasing(TimeSeriesState & source, uint64_t current_timestamp_us)
  {
    if (!source.check_increasing) {
      return false;
    }
    if (current_timestamp_us <= source.prev_timestamp_us) {
      source.num_non_increasing++;
      source.prev_noninc_ts = clock_->now();
      RCLCPP_WARN(
        node_.get_logger(),
        "[GreenwaveDiagnostics %s Increasing] current(%" PRIu64
        ") <= previous(%" PRIu64 ") topic %s. Units: microseconds.",
        source.label.c_str(), current_timestamp_us,
        source.prev_timestamp_us, topic_name_.c_str());
    }
    if (source.prev_noninc_ts.nanoseconds() != 0 &&
      (clock_->now() - source.prev_noninc_ts).seconds() < constants::kDropWarnTimeoutSeconds)
    {
      status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      update_status_message(status_vec_[0], source.increasing_error_message);
      return true;
    }
    return false;
  }

  bool checkFpsWindow(TimeSeriesState & source)
  {
    if (expected_frequency_ <= 0.0 ||
      source.window.interarrival_us.empty() || !source.check_fps_window)
    {
      return false;
    }
    const double clamped_tolerance_percent = std::max(0.0, tolerance_);
    const double tolerance_ratio = clamped_tolerance_percent / 100.0;
    const double min_allowed_fps =
      expected_frequency_ * std::max(0.0, 1.0 - tolerance_ratio);
    const double max_allowed_fps = expected_frequency_ * (1.0 + tolerance_ratio);
    const double current_fps = source.window.frameRateHz();

    if (current_fps < min_allowed_fps) {
      source.prev_fps_out_of_range_ts = clock_->now();
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[GreenwaveDiagnostics %s FPS] Current FPS (%.3f) is below minimum allowed (%.3f)"
        " for topic %s.",
        source.label.c_str(), current_fps, min_allowed_fps, topic_name_.c_str());
    }
    if (current_fps > max_allowed_fps) {
      source.prev_fps_out_of_range_ts = clock_->now();
      RCLCPP_DEBUG(
        node_.get_logger(),
        "[GreenwaveDiagnostics %s FPS] Current FPS (%.3f) is above maximum allowed (%.3f)"
        " for topic %s.",
        source.label.c_str(), current_fps, max_allowed_fps, topic_name_.c_str());
    }

    if (source.prev_fps_out_of_range_ts.nanoseconds() != 0 &&
      (clock_->now() - source.prev_fps_out_of_range_ts).seconds() <
      constants::kDropWarnTimeoutSeconds)
    {
      status_vec_[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      update_status_message(status_vec_[0], source.fps_window_error_message);
      return true;
    }
    return false;
  }

  bool updateTimeSource(
    TimeSeriesState & source,
    uint64_t current_timestamp_us)
  {
    if (source.prev_timestamp_us == std::numeric_limits<uint64_t>::min()) {
      source.prev_timestamp_us = current_timestamp_us;
      return false;
    }
    const int64_t timestamp_diff_us =
      static_cast<int64_t>(current_timestamp_us - source.prev_timestamp_us);
    source.window.addInterarrival(timestamp_diff_us);

    if (!source.enabled) {
      source.prev_timestamp_us = current_timestamp_us;
      return false;
    }

    bool error_found = false;
    error_found |= checkFpsJitter(source, timestamp_diff_us);
    error_found |= checkFpsWindow(source);
    error_found |= checkIncreasing(source, current_timestamp_us);
    source.prev_timestamp_us = current_timestamp_us;
    return error_found;
  }

  double expected_frequency_{0.0};
  double tolerance_{0.0};
};

}  // namespace greenwave_diagnostics
