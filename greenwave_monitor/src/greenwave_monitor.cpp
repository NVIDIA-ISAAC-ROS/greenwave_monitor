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

#include "greenwave_monitor.hpp"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <unordered_map>

#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

using namespace std::chrono_literals;

GreenwaveMonitor::GreenwaveMonitor(const rclcpp::NodeOptions & options)
: Node("greenwave_monitor", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting GreenwaveMonitorNode");

  // Declare and get the topics parameter
  this->declare_parameter<std::vector<std::string>>("topics", {""});
  auto topics = this->get_parameter("topics").as_string_array();

  message_diagnostics::MessageDiagnosticsConfig diagnostics_config;
  diagnostics_config.enable_all_topic_diagnostics = true;

  auto topic_names_and_types = this->get_topic_names_and_types();

  for (const auto & topic : topics) {
    if (!topic.empty()) {
      std::string message;
      add_topic(topic, message);
    }
  }

  timer_ = this->create_wall_timer(
    1s, std::bind(&GreenwaveMonitor::timer_callback, this));

  // Add service server
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

std::optional<std::string> GreenwaveMonitor::find_topic_type_with_retry(
  const std::string & topic, const int max_retries, const int retry_period_s)
{
  for (int i = 0; i < max_retries; ++i) {
    auto topic_names_and_types = this->get_topic_names_and_types();
    auto it = topic_names_and_types.find(topic);
    if (it == topic_names_and_types.end() || it->second.empty()) {
      std::this_thread::sleep_for(std::chrono::seconds(retry_period_s));
      continue;
    } else {
      return it->second[0];
    }
  }
  return std::nullopt;
}

void GreenwaveMonitor::topic_callback(
  const std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & topic, const std::string & type)
{
  auto msg_timestamp = GetTimestampFromSerializedMessage(msg, type);
  message_diagnostics_[topic]->updateDiagnostics(msg_timestamp.time_since_epoch().count());
}

void GreenwaveMonitor::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "====================================================");
  if (message_diagnostics_.empty()) {
    RCLCPP_INFO(this->get_logger(), "No topics to monitor");
  }
  for (auto & [topic, diagnostics] : message_diagnostics_) {
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
  auto it = message_diagnostics_.find(request->topic_name);

  if (it == message_diagnostics_.end()) {
    if (!request->add_topic_if_missing) {
      response->success = false;
      response->message = "Failed to find topic";
      return;
    }

    if (!add_topic(request->topic_name, response->message)) {
      response->success = false;
      return;
    }
    it = message_diagnostics_.find(request->topic_name);
  }

  message_diagnostics::MessageDiagnostics & msg_diagnostics_obj = *(it->second);

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

bool GreenwaveMonitor::add_topic(const std::string & topic, std::string & message)
{
  // Check if topic already exists
  if (message_diagnostics_.find(topic) != message_diagnostics_.end()) {
    message = "Topic already being monitored";
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Adding subscription for topic '%s'", topic.c_str());

  const int max_retries = 5;
  const int retry_period_s = 1;
  auto maybe_type = find_topic_type_with_retry(topic, max_retries, retry_period_s);
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

  message_diagnostics::MessageDiagnosticsConfig diagnostics_config;
  diagnostics_config.enable_all_topic_diagnostics = true;

  subscriptions_.push_back(sub);
  message_diagnostics_.emplace(
    topic,
    std::make_unique<message_diagnostics::MessageDiagnostics>(*this, topic, diagnostics_config));

  message = "Successfully added topic";
  return true;
}

bool GreenwaveMonitor::remove_topic(const std::string & topic, std::string & message)
{
  auto diag_it = message_diagnostics_.find(topic);
  if (diag_it == message_diagnostics_.end()) {
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

  message_diagnostics_.erase(diag_it);
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
