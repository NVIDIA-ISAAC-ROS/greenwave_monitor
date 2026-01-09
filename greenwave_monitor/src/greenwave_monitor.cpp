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

#include <optional>

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

  // Timer callback to publish diagnostics and print feedback
  timer_ = this->create_wall_timer(
    1s, std::bind(&GreenwaveMonitor::timer_callback, this));

  // Defer topic discovery to allow the ROS graph to settle before querying other nodes
  init_timer_ = this->create_wall_timer(0ms, [this]() {
    init_timer_->cancel();
    deferred_init();
  });
}

void GreenwaveMonitor::deferred_init()
{
  fetch_external_topic_map();

  std::set<std::string> all_topics = get_topics_from_parameters();
  auto topics_param = this->get_parameter("topics").as_string_array();
  for (const auto & topic : topics_param) {
    if (!topic.empty()) {
      all_topics.insert(topic);
    }
  }

  for (const auto & topic : all_topics) {
    std::string message;
    add_topic(topic, message);
  }

  // Register parameter callbacks after initialization is complete
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&GreenwaveMonitor::on_parameter_change, this, std::placeholders::_1));

  // Subscribe to parameter events to execute pending topic additions and track external monitoring
  param_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", 10,
    std::bind(&GreenwaveMonitor::on_parameter_event, this, std::placeholders::_1));
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

std::optional<std::string> parse_enabled_topic_param(const std::string & name)
{
  const std::string prefix = greenwave_diagnostics::constants::kTopicParamPrefix;
  const std::string suffix = greenwave_diagnostics::constants::kEnabledSuffix;

  if (name.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  if (name.size() <= prefix.size() + suffix.size()) {
    return std::nullopt;
  }
  if (name.substr(name.size() - suffix.size()) != suffix) {
    return std::nullopt;
  }

  std::string topic = name.substr(
    prefix.size(), name.size() - prefix.size() - suffix.size());
  if (topic.empty() || topic[0] != '/') {
    return std::nullopt;
  }

  return topic;
}

rcl_interfaces::msg::SetParametersResult GreenwaveMonitor::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    auto topic_opt = parse_enabled_topic_param(param.get_name());
    if (!topic_opt.has_value()) {
      continue;
    }

    if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      continue;
    }

    const std::string & topic = topic_opt.value();
    bool enabled = param.as_bool();

    if (enabled) {
      // Skip if already monitored or already pending (prevents recursive additions)
      if (greenwave_diagnostics_.find(topic) != greenwave_diagnostics_.end() ||
        pending_validations_.find(topic) != pending_validations_.end())
      {
        continue;
      }
      // Allow 1 retry with 0.5s wait for publisher discovery
      auto validation = validate_add_topic(topic, 1, 0.5);
      if (!validation.valid) {
        result.successful = false;
        result.reason = validation.error_message;
        return result;
      }
      pending_validations_[topic] = validation;
    } else {
      if (external_topic_to_node_.find(topic) != external_topic_to_node_.end()) {
        result.successful = false;
        result.reason = "Topic being monitored by external node: " + external_topic_to_node_[topic];
        return result;
      }
      if (greenwave_diagnostics_.find(topic) == greenwave_diagnostics_.end()) {
        result.successful = false;
        result.reason = "Topic not being monitored: " + topic;
        return result;
      }
    }
  }

  return result;
}

void GreenwaveMonitor::on_parameter_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
{
  if (msg->node == this->get_fully_qualified_name()) {
    internal_on_parameter_event(msg);
  } else {
    external_on_parameter_event(msg);
  }
}

void GreenwaveMonitor::internal_on_parameter_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
{
  // Process new and changed parameters - execute pending topic additions and removals
  auto process_params = [this](const auto & params) {
      for (const auto & param : params) {
        auto topic_opt = parse_enabled_topic_param(param.name);
        if (!topic_opt.has_value()) {
          continue;
        }

        const std::string & topic = topic_opt.value();
        bool enabled = param.value.bool_value;
        std::string message;

        if (enabled) {
          auto it = pending_validations_.find(topic);
          if (it != pending_validations_.end()) {
            auto validation = std::move(it->second);
            pending_validations_.erase(it);
            execute_add_topic(validation, message);
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
          }
        } else {
          remove_topic(topic, message);
          RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        }
      }
    };

  process_params(msg->new_parameters);
  process_params(msg->changed_parameters);
}

void GreenwaveMonitor::external_on_parameter_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
{
  // Process new parameters - track external nodes with enabled parameters
  for (const auto & param : msg->new_parameters) {
    auto topic_opt = parse_enabled_topic_param(param.name);
    if (!topic_opt.has_value()) {
      continue;
    }
    if (param.value.bool_value) {
      external_topic_to_node_[topic_opt.value()] = msg->node;
    }
  }

  // Process deleted parameters - remove from external map
  for (const auto & param : msg->deleted_parameters) {
    auto topic_opt = parse_enabled_topic_param(param.name);
    if (!topic_opt.has_value()) {
      continue;
    }
    external_topic_to_node_.erase(topic_opt.value());
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

TopicValidationResult GreenwaveMonitor::validate_add_topic(
  const std::string & topic, int max_retries, double retry_period_s)
{
  TopicValidationResult result;
  result.topic = topic;

  auto it = external_topic_to_node_.find(topic);
  if (it != external_topic_to_node_.end()) {
    result.error_message = "Topic already monitored by external node: " + it->second;
    return result;
  }

  if (greenwave_diagnostics_.find(topic) != greenwave_diagnostics_.end()) {
    result.error_message = "Topic already monitored: " + topic;
    return result;
  }

  std::vector<rclcpp::TopicEndpointInfo> publishers;
  for (int attempt = 0; attempt <= max_retries; ++attempt) {
    try {
      publishers = this->get_publishers_info_by_topic(topic);
    } catch (const rclcpp::exceptions::RCLError & e) {
      // Context may be invalid during shutdown
      result.error_message = "Node context invalid (shutting down)";
      return result;
    }
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
    result.error_message = "No publishers found for topic: " + topic;
    return result;
  }

  result.valid = true;
  result.message_type = publishers[0].topic_type();
  return result;
}

bool GreenwaveMonitor::execute_add_topic(
  const TopicValidationResult & validated, std::string & message)
{
  if (!validated.valid) {
    message = validated.error_message;
    return false;
  }

  const std::string & topic = validated.topic;
  const std::string & type = validated.message_type;

  // Guard against duplicate subscriptions from parameter re-set in GreenwaveDiagnostics
  if (greenwave_diagnostics_.find(topic) != greenwave_diagnostics_.end()) {
    message = "Topic already monitored: " + topic;
    return true;
  }

  RCLCPP_INFO(this->get_logger(), "Adding subscription for topic '%s'", topic.c_str());

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
    std::make_unique<greenwave_diagnostics::GreenwaveDiagnostics>(
      *this, topic, diagnostics_config));

  message = "Successfully added topic: " + topic;
  return true;
}

bool GreenwaveMonitor::add_topic(
  const std::string & topic, std::string & message,
  int max_retries, double retry_period_s)
{
  auto validation = validate_add_topic(topic, max_retries, retry_period_s);
  if (!validation.valid) {
    RCLCPP_ERROR(this->get_logger(), "%s", validation.error_message.c_str());
    message = validation.error_message;
    return false;
  }
  return execute_add_topic(validation, message);
}

bool GreenwaveMonitor::remove_topic(const std::string & topic, std::string & message)
{
  auto diag_it = greenwave_diagnostics_.find(topic);
  if (diag_it == greenwave_diagnostics_.end()) {
    message = "Nothing to remove, topic not being monitored";
    return true;
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

void GreenwaveMonitor::fetch_external_topic_map()
{
  const std::string our_node = this->get_fully_qualified_name();
  const std::string prefix = greenwave_diagnostics::constants::kTopicParamPrefix;
  const std::string suffix = greenwave_diagnostics::constants::kEnabledSuffix;

  static uint64_t temp_node_counter = 0;
  rclcpp::NodeOptions temp_options;
  temp_options.start_parameter_services(false);
  temp_options.start_parameter_event_publisher(false);
  auto temp_node = std::make_shared<rclcpp::Node>(
    "_gw_init_" + std::to_string(temp_node_counter++),
    "/_greenwave_internal",
    temp_options);

  auto node_names = this->get_node_names();
  for (const auto & full_name : node_names) {
    if (full_name == our_node) {
      continue;
    }

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
      temp_node, full_name);
    if (!param_client->wait_for_service(std::chrono::milliseconds(100))) {
      continue;
    }

    std::vector<std::string> param_names;
    std::vector<rclcpp::Parameter> params;
    try {
      param_names = param_client->list_parameters({"greenwave_diagnostics"}, 10).names;
      if (param_names.empty()) {
        continue;
      }
      params = param_client->get_parameters(param_names);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Failed to query parameters from node '%s': %s",
        full_name.c_str(), e.what());
      continue;
    }

    for (size_t i = 0; i < param_names.size(); ++i) {
      const auto & name = param_names[i];
      const auto & param = params[i];

      if (name.rfind(prefix, 0) != 0) {
        continue;
      }
      if (name.size() <= prefix.size() + suffix.size()) {
        continue;
      }
      if (name.substr(name.size() - suffix.size()) != suffix) {
        continue;
      }

      std::string topic = name.substr(
        prefix.size(), name.size() - prefix.size() - suffix.size());
      if (topic.empty() || topic[0] != '/') {
        continue;
      }

      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL && param.as_bool()) {
        external_topic_to_node_[topic] = full_name;
        RCLCPP_DEBUG(
          this->get_logger(),
          "Found external monitoring for topic '%s' on node '%s'",
          topic.c_str(), full_name.c_str());
      }
    }
  }
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
