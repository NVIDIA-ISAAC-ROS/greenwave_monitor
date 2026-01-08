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
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

using namespace std::chrono_literals;

GreenwaveMonitor::GreenwaveMonitor(const rclcpp::NodeOptions & options)
: Node("greenwave_monitor",
    rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  RCLCPP_INFO(this->get_logger(), "Starting GreenwaveMonitorNode");

  // Get the topics parameter (declare only if not already declared from overrides)
  if (!this->has_parameter("topics")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    this->declare_parameter<std::vector<std::string>>("topics", {""}, descriptor);
  }

  auto topics = this->get_parameter("topics").as_string_array();
  for (const auto & topic : topics) {
    if (!topic.empty()) {
      std::string message;
      add_topic(topic, message);
    }
  }

  // Also check for topics defined via greenwave_diagnostics.<topic_name>.* parameters
  auto topics_from_params = get_topics_from_parameters();
  for (const auto & topic : topics_from_params) {
    if (greenwave_diagnostics_.find(topic) == greenwave_diagnostics_.end()) {
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
}

void GreenwaveMonitor::topic_callback(
  const std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & topic, const std::string & type)
{
  auto msg_timestamp = GetTimestampFromSerializedMessage(msg, type);
  greenwave_diagnostics_[topic]->updateDiagnostics(msg_timestamp.time_since_epoch().count());
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
    // No retries for service calls - caller can retry if needed
    response->success = add_topic(request->topic_name, response->message, 0);
  } else {
    response->success = remove_topic(request->topic_name, response->message);
  }
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
  const std::string & topic, std::string & message,
  int max_retries, double retry_period_s)
{
  // Check if topic already exists
  if (greenwave_diagnostics_.find(topic) != greenwave_diagnostics_.end()) {
    message = "Topic already being monitored";
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Adding subscription for topic '%s'", topic.c_str());

  // Get publishers for this topic with retry logic
  std::vector<rclcpp::TopicEndpointInfo> publishers;
  for (int attempt = 0; attempt <= max_retries; ++attempt) {
    publishers = this->get_publishers_info_by_topic(topic);
    if (!publishers.empty()) {
      break;
    }
    if (attempt < max_retries) {
      RCLCPP_INFO(
        this->get_logger(),
        "No publishers found for topic '%s', retrying in %.1fs (%d/%d)",
        topic.c_str(), retry_period_s, attempt + 1, max_retries);
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(retry_period_s * 1000)));
    }
  }

  if (publishers.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find publishers for topic '%s'", topic.c_str());
    message = "Failed to find publishers for topic";
    return false;
  }

  // Get the message type from the first publisher
  const std::string type = publishers[0].topic_type();

  // Try to enable monitoring on an existing node with the parameter
  try {
    if (try_set_external_enabled_param(topic, true, message)) {
      return true;
    }
    // If already monitored externally, return false
    if (message.find("already being monitored") != std::string::npos) {
      return false;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      this->get_logger(),
      "Exception while checking for existing monitoring on topic '%s': %s",
      topic.c_str(), e.what());
  }

  // No existing node with the parameter found, create local GreenwaveDiagnostics
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

  subscriptions_.push_back(sub);
  greenwave_diagnostics_.emplace(
    topic,
    std::make_unique<greenwave_diagnostics::GreenwaveDiagnostics>(*this, topic,
    diagnostics_config));

  message = "Successfully added topic";
  return true;
}

bool GreenwaveMonitor::remove_topic(const std::string & topic, std::string & message)
{
  auto diag_it = greenwave_diagnostics_.find(topic);
  if (diag_it == greenwave_diagnostics_.end()) {
    // Topic not monitored locally, try to find a publisher node with the enabled parameter
    try {
      return try_set_external_enabled_param(topic, false, message);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Exception while checking for existing monitoring on topic '%s': %s",
        topic.c_str(), e.what());
      message = "Exception while checking external nodes";
      return false;
    }
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

  // NOTE: the parameters are not removed when the diagnostics object is destroyed. This allows
  // for settings to persist even when a topic is not available.
  greenwave_diagnostics_.erase(diag_it);

  message = "Successfully removed topic";
  return true;
}

bool GreenwaveMonitor::try_set_external_enabled_param(
  const std::string & topic, bool enabled, std::string & message)
{
  std::string enabled_param_name =
    std::string(greenwave_diagnostics::constants::kTopicParamPrefix) +
    topic + greenwave_diagnostics::constants::kEnabledSuffix;

  auto publishers = this->get_publishers_info_by_topic(topic);
  if (publishers.empty()) {
    message = "No publishers found for topic";
    return false;
  }

  std::string our_ns = this->get_namespace();
  std::string our_name = this->get_name();
  std::string our_full_name = (our_ns == "/") ?
    ("/" + our_name) : (our_ns + "/" + our_name);

  static std::atomic<uint64_t> temp_node_counter{0};
  rclcpp::NodeOptions temp_options;
  temp_options.start_parameter_services(false);
  temp_options.start_parameter_event_publisher(false);
  auto temp_node = std::make_shared<rclcpp::Node>(
    "_gw_param_" + std::to_string(temp_node_counter++),
    "/_greenwave_internal",
    temp_options);

  for (const auto & pub_info : publishers) {
    std::string node_name = pub_info.node_name();
    std::string node_namespace = pub_info.node_namespace();
    std::string full_node_name = (node_namespace == "/") ?
      ("/" + node_name) : (node_namespace + "/" + node_name);

    if (full_node_name == our_full_name) {
      continue;
    }

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
      temp_node, full_node_name);
    if (!param_client->wait_for_service(std::chrono::milliseconds(500))) {
      continue;
    }

    if (!param_client->has_parameter(enabled_param_name)) {
      continue;
    }

    auto current_params = param_client->get_parameters({enabled_param_name});
    if (!current_params.empty() &&
      current_params[0].get_type() == rclcpp::ParameterType::PARAMETER_BOOL &&
      current_params[0].as_bool() == enabled)
    {
      message = enabled ?
        "Topic already being monitored on node: " + full_node_name :
        "Topic already disabled on node: " + full_node_name;
      return false;
    }

    auto results = param_client->set_parameters({
      rclcpp::Parameter(enabled_param_name, enabled)
    });
    if (!results.empty() && results[0].successful) {
      RCLCPP_INFO(
        this->get_logger(),
        "%s monitoring via parameter on node '%s' for topic '%s'",
        enabled ? "Enabled" : "Disabled",
        full_node_name.c_str(), topic.c_str());
      message = (enabled ? std::string("Enabled") : std::string("Disabled")) +
        " monitoring on existing node: " + full_node_name;
      return true;
    }
  }

  message = "No external node with parameter " + enabled_param_name + " found";
  return false;
}

std::set<std::string> GreenwaveMonitor::get_topics_from_parameters()
{
  std::set<std::string> topics;

  // List all parameters with "greenwave_diagnostics." prefix
  auto list_result = this->list_parameters({"greenwave_diagnostics"}, 10);

  for (const auto & param_name : list_result.names) {
    // Parameter names are like "greenwave_diagnostics./my_topic.enabled"
    // We need to extract the topic name (e.g., "/my_topic")
    if (param_name.find(greenwave_diagnostics::constants::kTopicParamPrefix) != 0) {
      continue;
    }

    // Remove the "greenwave_diagnostics." prefix
    std::string remainder = param_name.substr(
      std::strlen(greenwave_diagnostics::constants::kTopicParamPrefix));

    // Find the last '.' to separate topic name from parameter suffix
    // Topic names can contain '/' but parameter suffixes are like ".enabled", ".tolerance", etc.
    size_t last_dot = remainder.rfind('.');
    if (last_dot == std::string::npos || last_dot == 0) {
      continue;
    }

    std::string topic_name = remainder.substr(0, last_dot);
    if (!topic_name.empty() && topic_name[0] == '/') {
      topics.insert(topic_name);
    }
  }

  // Filter out topics with enabled=false
  std::set<std::string> filtered_topics;
  for (const auto & topic : topics) {
    std::string enabled_param =
      std::string(greenwave_diagnostics::constants::kTopicParamPrefix) +
      topic + greenwave_diagnostics::constants::kEnabledSuffix;

    if (this->has_parameter(enabled_param)) {
      auto param = this->get_parameter(enabled_param);
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL &&
        !param.as_bool())
      {
        continue;
      }
    }
    filtered_topics.insert(topic);
  }

  return filtered_topics;
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
