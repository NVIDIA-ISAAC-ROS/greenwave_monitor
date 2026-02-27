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

#include "greenwave_monitor.hpp"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <set>
#include <unordered_map>

#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr const char * kTimestampModeHeaderWithFallback = "header_with_nodetime_fallback";
constexpr const char * kTimestampModeHeaderOnly = "header_only";
constexpr const char * kTimestampModeNodetimeOnly = "nodetime_only";
constexpr const char * kTimestampModeNone = "none";
constexpr const char * kTimeCheckPresetsParam = "gw_time_check_preset";

greenwave_diagnostics::TimeCheckPreset timeCheckPresetFromString(const std::string & s)
{
  if (s == kTimestampModeHeaderOnly) {
    return greenwave_diagnostics::TimeCheckPreset::HeaderOnly;
  }
  if (s == kTimestampModeNodetimeOnly) {
    return greenwave_diagnostics::TimeCheckPreset::NodetimeOnly;
  }
  if (s == kTimestampModeHeaderWithFallback) {
    return greenwave_diagnostics::TimeCheckPreset::HeaderWithFallback;
  }
  return greenwave_diagnostics::TimeCheckPreset::None;
}

}  // namespace

namespace greenwave_monitor
{
namespace constants
{
inline constexpr const char * kTopicParamPrefix = "gw_frequency_monitored_topics.";
inline constexpr const char * kExpectedFrequencySuffix = ".expected_frequency";
inline constexpr const char * kToleranceSuffix = ".tolerance";
}  // namespace constants
}  // namespace greenwave_monitor

GreenwaveMonitor::GreenwaveMonitor(const rclcpp::NodeOptions & options)
: Node("greenwave_monitor",
    rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  RCLCPP_INFO(this->get_logger(), "Starting GreenwaveMonitorNode");

  if (!this->has_parameter("gw_monitored_topics")) {
    this->declare_parameter<std::vector<std::string>>("gw_monitored_topics", {""});
  }
  if (!this->has_parameter(kTimeCheckPresetsParam)) {
    this->declare_parameter<std::string>(
      kTimeCheckPresetsParam, kTimestampModeHeaderWithFallback);
  }
  std::string time_check_preset_str = this->get_parameter(kTimeCheckPresetsParam).as_string();
  time_check_preset_ = timeCheckPresetFromString(time_check_preset_str);
  // Give a warning if the time check preset has an invalid string and use the default
  if (time_check_preset_ == greenwave_diagnostics::TimeCheckPreset::None &&
    time_check_preset_str != kTimestampModeNone)
  {
    RCLCPP_WARN(
      this->get_logger(), "Invalid time check preset '%s', using default '%s'. Valid presets are: "
      "%s, %s, %s, %s",
      time_check_preset_str.c_str(), kTimestampModeHeaderWithFallback,
      kTimestampModeHeaderWithFallback, kTimestampModeHeaderOnly, kTimestampModeNodetimeOnly,
      kTimestampModeNone);
    time_check_preset_ = greenwave_diagnostics::TimeCheckPreset::HeaderWithFallback;
  } else if (time_check_preset_ != greenwave_diagnostics::TimeCheckPreset::None) {
    RCLCPP_INFO(
      this->get_logger(), "Using time check preset '%s'",
      time_check_preset_str.c_str());
  }

  timer_ = this->create_wall_timer(
    1s, std::bind(&GreenwaveMonitor::timer_callback, this));

  // Defer topic discovery to allow the ROS graph to settle before querying other nodes
  init_timer_ = this->create_wall_timer(
    100ms, [this]() {
      init_timer_->cancel();
      deferred_init();
    });
}

void GreenwaveMonitor::deferred_init()
{
  // Get all topics from YAML and parameters
  add_topics_from_parameters();

  // Add service servers after all topics are added to prevent race conditions
  manage_topic_service_ =
    this->create_service<greenwave_monitor_interfaces::srv::ManageTopic>(
    "~/manage_topic",
    std::bind(
      &GreenwaveMonitor::handle_manage_topic, this,
      std::placeholders::_1, std::placeholders::_2));

  set_expected_frequency_service_ =
    this->create_service<greenwave_monitor_interfaces::srv::SetExpectedFrequency>(
    "~/set_expected_frequency",
    std::bind(
      &GreenwaveMonitor::handle_set_expected_frequency, this,
      std::placeholders::_1, std::placeholders::_2));
}

std::optional<std::string> GreenwaveMonitor::find_topic_type(
  const std::string & topic, int max_retries, double retry_wait_s)
{
  for (int attempt = 0; attempt <= max_retries; ++attempt) {
    auto publishers = this->get_publishers_info_by_topic(topic);
    if (!publishers.empty()) {
      return publishers[0].topic_type();
    }
    if (attempt < max_retries && retry_wait_s > 0.0) {
      std::this_thread::sleep_for(
        std::chrono::duration<double>(retry_wait_s));
    }
  }
  return std::nullopt;
}

void GreenwaveMonitor::topic_callback(
  const std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & topic, const std::string & type)
{
  auto hdr_timestamp = GetTimestampFromSerializedMessage(msg, type);
  greenwave_diagnostics_[topic]->updateDiagnostics(hdr_timestamp.time_since_epoch().count());
}

void GreenwaveMonitor::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "====================================================");
  if (greenwave_diagnostics_.empty()) {
    RCLCPP_INFO(this->get_logger(), "No topics to monitor");
  }
  for (auto & [topic, diagnostics] : greenwave_diagnostics_) {
    diagnostics->publishDiagnostics();
    RCLCPP_INFO(
      this->get_logger(), "Frame rate for topic %s: %.2f hz",
      topic.c_str(), diagnostics->getFrameRateNode());
    RCLCPP_INFO(
      this->get_logger(), "Latency for topic %s: %.2f ms",
      topic.c_str(), diagnostics->getLatency());
  }
  RCLCPP_INFO(this->get_logger(), "====================================================");
}

void GreenwaveMonitor::handle_manage_topic(
  const std::shared_ptr<greenwave_monitor_interfaces::srv::ManageTopic::Request> request,
  std::shared_ptr<greenwave_monitor_interfaces::srv::ManageTopic::Response> response)
{
  if (request->add_topic) {
    response->success = add_topic(request->topic_name, response->message);
  } else {
    response->success = remove_topic(request->topic_name, response->message);
  }
}

void GreenwaveMonitor::handle_set_expected_frequency(
  const std::shared_ptr<greenwave_monitor_interfaces::srv::SetExpectedFrequency::Request> request,
  std::shared_ptr<greenwave_monitor_interfaces::srv::SetExpectedFrequency::Response> response)
{
  auto it = greenwave_diagnostics_.find(request->topic_name);

  if (it == greenwave_diagnostics_.end()) {
    if (!request->add_topic_if_missing) {
      response->success = false;
      response->message = "Failed to find topic";
      return;
    }

    if (!add_topic(request->topic_name, response->message)) {
      response->success = false;
      return;
    }
    it = greenwave_diagnostics_.find(request->topic_name);
  }

  greenwave_diagnostics::GreenwaveDiagnostics & msg_diagnostics_obj = *(it->second);

  if (request->clear_expected) {
    msg_diagnostics_obj.clearExpectedDt();
    response->success = true;
    response->message = "Successfully cleared expected frequency for topic '" +
      request->topic_name + "'";
    return;
  }

  if (request->expected_hz <= 0.0) {
    response->success = false;
    response->message = "Invalid expected frequency, must be set to a positive value";
    return;
  }
  if (request->tolerance_percent < 0.0) {
    response->success = false;
    response->message =
      "Invalid tolerance, must be a non-negative percentage";
    return;
  }

  msg_diagnostics_obj.setExpectedDt(request->expected_hz, request->tolerance_percent);

  response->success = true;
  response->message = "Successfully set expected frequency for topic '" +
    request->topic_name + "' to " + std::to_string(request->expected_hz) +
    " hz with tolerance " + std::to_string(request->tolerance_percent) + "%";
}

bool GreenwaveMonitor::has_header_from_type(const std::string & type_name)
{
  // We use a cache to avoid repeated lookups for the same message type.
  // ex. {sensor_msgs/msg/Image : true, std_msgs/msg/String : false}
  static std::unordered_map<std::string, bool> type_has_header_cache;

  static std::mutex has_header_cache_mutex;
  std::lock_guard<std::mutex> lock(has_header_cache_mutex);

  if (type_has_header_cache.find(type_name) != type_has_header_cache.end()) {
    return type_has_header_cache[type_name];
  }

  // rosidl typesupport API is unstable across ROS distributions, so we use this
  // map as a more robust way to determine if a message type has a header
  static const std::unordered_map<std::string, bool> known_header_types = {
    // sensor_msgs
    {"sensor_msgs/msg/Image", true},
    {"sensor_msgs/msg/CompressedImage", true},
    {"sensor_msgs/msg/CameraInfo", true},
    {"sensor_msgs/msg/PointCloud2", true},
    {"sensor_msgs/msg/LaserScan", true},
    {"sensor_msgs/msg/Imu", true},
    {"sensor_msgs/msg/NavSatFix", true},
    {"sensor_msgs/msg/MagneticField", true},
    {"sensor_msgs/msg/FluidPressure", true},
    {"sensor_msgs/msg/Illuminance", true},
    {"sensor_msgs/msg/RelativeHumidity", true},
    {"sensor_msgs/msg/Temperature", true},
    {"sensor_msgs/msg/Range", true},
    {"sensor_msgs/msg/PointCloud", true},

    // geometry_msgs
    {"geometry_msgs/msg/PoseStamped", true},
    {"geometry_msgs/msg/TwistStamped", true},
    {"geometry_msgs/msg/AccelStamped", true},
    {"geometry_msgs/msg/Vector3Stamped", true},
    {"geometry_msgs/msg/PointStamped", true},
    {"geometry_msgs/msg/QuaternionStamped", true},
    {"geometry_msgs/msg/TransformStamped", true},
    {"geometry_msgs/msg/WrenchStamped", true},

    // nav_msgs
    {"nav_msgs/msg/OccupancyGrid", true},
    {"nav_msgs/msg/GridCells", true},
    {"nav_msgs/msg/Path", true},
    {"nav_msgs/msg/Odometry", true},

    // visualization_msgs
    {"visualization_msgs/msg/Marker", true},
    {"visualization_msgs/msg/MarkerArray", true},
    {"visualization_msgs/msg/InteractiveMarker", true},

    // std_msgs (no headers)
    {"std_msgs/msg/String", false},
    {"std_msgs/msg/Int32", false},
    {"std_msgs/msg/Float64", false},
    {"std_msgs/msg/Bool", false},
    {"std_msgs/msg/Empty", false},
    {"std_msgs/msg/Header", false},  // Header itself doesn't have a header

    // Common message types without headers
    {"geometry_msgs/msg/Twist", false},
    {"geometry_msgs/msg/Pose", false},
    {"geometry_msgs/msg/Point", false},
    {"geometry_msgs/msg/Vector3", false},
    {"geometry_msgs/msg/Quaternion", false}
  };

  auto it = known_header_types.find(type_name);
  bool has_header = (it != known_header_types.end()) ? it->second : false;

  type_has_header_cache[type_name] = has_header;

  // Fallback of no header in case of unknown type, log for reference
  if (it == known_header_types.end()) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "Unknown message type '%s' - assuming no header. Consider adding to registry.",
      type_name.c_str());
  }

  return has_header;
}

bool GreenwaveMonitor::add_topic(
  const std::string & topic, std::string & message, int max_retries, double retry_wait_s)
{
  // Check if topic already exists
  if (greenwave_diagnostics_.find(topic) != greenwave_diagnostics_.end()) {
    message = "Topic already being monitored";
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Adding subscription for topic '%s'", topic.c_str());

  auto maybe_type = find_topic_type(topic, max_retries, retry_wait_s);
  if (!maybe_type.has_value()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find type for topic '%s'", topic.c_str());
    message = "Failed to find type for topic";
    return false;
  }

  const std::string type = maybe_type.value();
  auto sub = this->create_generic_subscription(
    topic,
    type,
    rclcpp::QoS(
      rclcpp::KeepLast(10), rmw_qos_profile_sensor_data),
    [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      this->topic_callback(msg, topic, type);
    });

  greenwave_diagnostics::GreenwaveDiagnosticsConfig diagnostics_config;
  diagnostics_config.enable_all_topic_diagnostics = true;
  diagnostics_config.time_check_preset = time_check_preset_;
  diagnostics_config.has_hdr_timestamp = has_header_from_type(type);

  subscriptions_.push_back(sub);
  greenwave_diagnostics_.emplace(
    topic,
    std::make_unique<greenwave_diagnostics::GreenwaveDiagnostics>(
      *this, topic, diagnostics_config));

  message = "Successfully added topic";
  return true;
}

bool GreenwaveMonitor::remove_topic(const std::string & topic, std::string & message)
{
  auto diag_it = greenwave_diagnostics_.find(topic);
  if (diag_it == greenwave_diagnostics_.end()) {
    message = "Topic not found";
    return false;
  }

  // Find and remove the subscription
  auto sub_it = std::find_if(
    subscriptions_.begin(), subscriptions_.end(),
    [&topic](const auto & sub) {
      return sub->get_topic_name() == topic;
    });

  if (sub_it != subscriptions_.end()) {
    subscriptions_.erase(sub_it);
  }

  greenwave_diagnostics_.erase(diag_it);
  message = "Successfully removed topic";
  return true;
}

// From ros2_benchmark monitor_node.cpp
// This assumes the message has a std_msgs header as the first
std::chrono::time_point<std::chrono::system_clock>
GreenwaveMonitor::GetTimestampFromSerializedMessage(
  std::shared_ptr<rclcpp::SerializedMessage> serialized_message_ptr,
  const std::string & type)
{
  if (!has_header_from_type(type)) {
    return std::chrono::time_point<std::chrono::system_clock>();  // timestamp 0 as fallback
  }

  int32_t timestamp_sec;
  uint8_t * sec_byte_ptr = reinterpret_cast<uint8_t *>(&timestamp_sec);
  *(sec_byte_ptr + 0) = serialized_message_ptr->get_rcl_serialized_message().buffer[4];
  *(sec_byte_ptr + 1) = serialized_message_ptr->get_rcl_serialized_message().buffer[5];
  *(sec_byte_ptr + 2) = serialized_message_ptr->get_rcl_serialized_message().buffer[6];
  *(sec_byte_ptr + 3) = serialized_message_ptr->get_rcl_serialized_message().buffer[7];

  uint32_t timestamp_nanosec;
  uint8_t * ns_byte_ptr = reinterpret_cast<uint8_t *>(&timestamp_nanosec);
  *(ns_byte_ptr + 0) = serialized_message_ptr->get_rcl_serialized_message().buffer[8];
  *(ns_byte_ptr + 1) = serialized_message_ptr->get_rcl_serialized_message().buffer[9];
  *(ns_byte_ptr + 2) = serialized_message_ptr->get_rcl_serialized_message().buffer[10];
  *(ns_byte_ptr + 3) = serialized_message_ptr->get_rcl_serialized_message().buffer[11];

  std::chrono::time_point<std::chrono::system_clock> timestamp(
    std::chrono::seconds(timestamp_sec) + std::chrono::nanoseconds(timestamp_nanosec));
  return timestamp;
}

void GreenwaveMonitor::add_topics_from_parameters()
{
  using greenwave_monitor::constants::kTopicParamPrefix;
  using greenwave_monitor::constants::kExpectedFrequencySuffix;
  using greenwave_monitor::constants::kToleranceSuffix;

  std::set<std::string> topics;

  // List all parameters with "gw_frequency_monitored_topics." prefix
  auto list_result = this->list_parameters({"gw_frequency_monitored_topics"}, 10);

  // Loop over and find all unique topics with "gw_frequency_monitored_topics." prefix
  for (const auto & param_name : list_result.names) {
    // Parameter names are like "gw_frequency_monitored_topics./my_topic.tolerance"
    // We need to extract the topic name (e.g., "/my_topic")
    if (param_name.find(kTopicParamPrefix) != 0) {
      continue;
    }

    // Remove the "gw_frequency_monitored_topics." prefix
    std::string remainder = param_name.substr(std::strlen(kTopicParamPrefix));

    // Find the last '.' to separate topic name from parameter suffix
    size_t last_dot = remainder.rfind('.');
    if (last_dot == std::string::npos || last_dot == 0) {
      continue;
    }

    std::string topic_name = remainder.substr(0, last_dot);
    if (!topic_name.empty() && topic_name[0] == '/') {
      topics.insert(topic_name);
    }
  }

  // Add topics from "gw_monitored_topics" parameter
  auto topics_param = this->get_parameter("gw_monitored_topics").as_string_array();
  topics.insert(topics_param.begin(), topics_param.end());

  // Helper function to get double parameters from the node
  auto get_double_param = [this](const std::string & name) -> double {
      auto param = this->get_parameter(name);
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return param.as_double();
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<double>(param.as_int());
      }
      return 0.0;
    };

  // For each topic, read parameters and add topic with expected frequency settings
  for (const auto & topic : topics) {
    if (topic.empty()) {
      continue;
    }
    std::string freq_param = std::string(kTopicParamPrefix) + topic + kExpectedFrequencySuffix;
    std::string tol_param = std::string(kTopicParamPrefix) + topic + kToleranceSuffix;

    double expected_frequency = 0.0;
    double tolerance = 0.0;

    if (this->has_parameter(freq_param)) {
      expected_frequency = get_double_param(freq_param);
    }
    if (this->has_parameter(tol_param)) {
      tolerance = get_double_param(tol_param);
      // Default to 0 if tolerance is negative
      if (tolerance < 0.0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid tolerance for topic '%s', clamping to 0.0",
          topic.c_str());
        tolerance = 0.0;
      }
    }

    std::string message;
    static const int max_retries = 5;
    static const double retry_wait_s = 0.5;
    if (add_topic(topic, message, max_retries, retry_wait_s)) {
      if (expected_frequency > 0.0) {
        greenwave_diagnostics_[topic]->setExpectedDt(expected_frequency, tolerance);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Expected frequency is 0 or negative for topic '%s', skipping parameter settings",
          topic.c_str());
      }
    }
  }
}
